/* ivp_ledgetree_collision.c — Hierarchical poly-poly collision via ledgetree BVH */

#include <ivp/collision/ivp_ledgetree_collision.h>
#include <ivp/collision/ivp_mindist.h>
#include <ivp/collision/ivp_oo_watcher.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/base/ivp_allocator.h>
#include <string.h>
#include <math.h>

/* ── BV overlap test ────────────────────────────────────────────────────── */

bool ivp_ledgetree_bv_overlap(const ivp_compact_ledgetree_node_t *node_a,
                               const ivp_compact_ledgetree_node_t *node_b,
                               const ivp_transform_r_t *b_in_a,
                               ivp_real_t margin)
{
    /* Transform node_b's center from B-space to A-space */
    ivp_vec3r_t center_b_local = ivp_vec3r((ivp_real_t)node_b->center[0],
                                            (ivp_real_t)node_b->center[1],
                                            (ivp_real_t)node_b->center[2]);
    ivp_vec3r_t center_b_in_a;
    ivp_transform_point(&center_b_in_a, b_in_a, &center_b_local);

    /* Node A's center in A-space */
    ivp_vec3r_t center_a = ivp_vec3r((ivp_real_t)node_a->center[0],
                                      (ivp_real_t)node_a->center[1],
                                      (ivp_real_t)node_a->center[2]);

    /* Squared distance between centers */
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff, &center_b_in_a, &center_a);
    ivp_real_t qdist = ivp_vec3r_quad_length(&diff);

    /* Combined radius threshold */
    ivp_real_t ra = (ivp_real_t)node_a->radius;
    ivp_real_t rb = (ivp_real_t)node_b->radius;
    ivp_real_t threshold = ra + rb + margin;

    return qdist <= threshold * threshold;
}

/* ── Pair traversal (explicit stack, descend-the-larger) ────────────────── */

typedef struct traverse_stack_entry {
    const ivp_compact_ledgetree_node_t *node_a;
    const ivp_compact_ledgetree_node_t *node_b;
} traverse_stack_entry_t;

int ivp_ledgetree_traverse_pairs(const ivp_compact_ledgetree_node_t *root_a,
                                  const ivp_compact_ledgetree_node_t *root_b,
                                  const ivp_transform_r_t *b_in_a,
                                  ivp_real_t margin,
                                  ivp_leaf_pair_fn callback,
                                  void *user_data)
{
    if (!root_a || !root_b) return 0;

    traverse_stack_entry_t stack[IVP_LEDGETREE_MAX_STACK];
    int sp = 0;
    int leaf_count = 0;

    stack[sp].node_a = root_a;
    stack[sp].node_b = root_b;
    sp++;

    while (sp > 0 && leaf_count < IVP_LEDGETREE_MAX_LEAF_PAIRS) {
        sp--;
        const ivp_compact_ledgetree_node_t *na = stack[sp].node_a;
        const ivp_compact_ledgetree_node_t *nb = stack[sp].node_b;

        if (!ivp_ledgetree_bv_overlap(na, nb, b_in_a, margin)) {
            continue;
        }

        bool term_a = ivp_ledgetree_is_terminal(na);
        bool term_b = ivp_ledgetree_is_terminal(nb);

        if (term_a && term_b) {
            /* Leaf pair found */
            const ivp_compact_ledge_t *ledge_a = ivp_ledgetree_get_compact_ledge(na);
            const ivp_compact_ledge_t *ledge_b = ivp_ledgetree_get_compact_ledge(nb);
            leaf_count++;
            if (!callback(ledge_a, ledge_b, b_in_a, user_data)) {
                break;
            }
            continue;
        }

        /* Descend the larger node (by radius) */
        bool descend_a;
        if (term_a) {
            descend_a = false;
        } else if (term_b) {
            descend_a = true;
        } else {
            descend_a = (na->radius >= nb->radius);
        }

        if (descend_a) {
            /* Push (left_a, nb) and (right_a, nb) */
            if (sp + 2 <= IVP_LEDGETREE_MAX_STACK) {
                stack[sp].node_a = ivp_ledgetree_right_son(na);
                stack[sp].node_b = nb;
                sp++;
                stack[sp].node_a = ivp_ledgetree_left_son(na);
                stack[sp].node_b = nb;
                sp++;
            }
        } else {
            /* Push (na, left_b) and (na, right_b) */
            if (sp + 2 <= IVP_LEDGETREE_MAX_STACK) {
                stack[sp].node_a = na;
                stack[sp].node_b = ivp_ledgetree_right_son(nb);
                sp++;
                stack[sp].node_a = na;
                stack[sp].node_b = ivp_ledgetree_left_son(nb);
                sp++;
            }
        }
    }

    return leaf_count;
}

/* ── Contact deduplication (local, matching ivp_contact.c's contact_is_duplicate) */

static bool contact_is_duplicate_local(const ivp_contact_point_t *a,
                                        const ivp_contact_point_t *b)
{
    const ivp_real_t point_eps = IVP_REAL(1e-3);
    const ivp_real_t gap_eps = IVP_REAL(1e-3);
    const ivp_real_t point_eps_q = point_eps * point_eps;

    if (ivp_vec3r_quad_distance(&a->point_a, &b->point_a) > point_eps_q) {
        return false;
    }
    if (ivp_vec3r_quad_distance(&a->point_b, &b->point_b) > point_eps_q) {
        return false;
    }
    if (ivp_fabs(a->gap_length - b->gap_length) > gap_eps) {
        return false;
    }
    return ivp_vec3r_dot(&a->normal, &b->normal) > IVP_REAL(0.995);
}

/* ── Leaf pair callback context ─────────────────────────────────────────── */

typedef struct leaf_pair_ctx {
    ivp_object_t        *obj_a;
    ivp_object_t        *obj_b;
    ivp_contact_point_t *contacts_out;
    int                  max_contacts;
    int                  count;
    const ivp_allocator_t *alloc;
} leaf_pair_ctx_t;

/* Compute a single contact from a leaf pair of ledges.
 * ledge_a is in A's object space, ledge_b is in B's object space.
 * b_in_a transforms B→A. We copy+transform ledge_b into A's space,
 * then run ivp_find_closest_pair. */
static bool leaf_pair_callback(const ivp_compact_ledge_t *ledge_a,
                                const ivp_compact_ledge_t *ledge_b,
                                const ivp_transform_r_t *b_in_a,
                                void *user_data)
{
    leaf_pair_ctx_t *ctx = (leaf_pair_ctx_t *)user_data;
    if (ctx->count >= ctx->max_contacts) return false;

    const ivp_core_t *ca = &ctx->obj_a->core;
    const ivp_core_t *cb = &ctx->obj_b->core;

    /* Copy and transform ledge_b into A's object space */
    int ledge_b_size = ivp_ledge_get_size(ledge_b);
    int ledge_b_points = ivp_ledge_get_n_points(ledge_b);
    int ledge_b_triangles = ivp_ledge_get_n_triangles(ledge_b);

    if (ledge_b_size < (int)sizeof(ivp_compact_ledge_t) ||
        (ledge_b_size & 0xF) != 0 ||
        ledge_b_triangles <= 0 ||
        ledge_b_points <= 0) {
        return true; /* skip bad ledge, continue traversal */
    }

    _Alignas(16) uint8_t stack_buf[4096];
    uint8_t *buf = (ledge_b_size <= (int)sizeof(stack_buf))
                 ? stack_buf
                 : (uint8_t *)ivp_alloc(ctx->alloc, (size_t)ledge_b_size);
    if (!buf) return true;

    memcpy(buf, ledge_b, (size_t)ledge_b_size);
    ivp_compact_ledge_t *ledge_b_tx = (ivp_compact_ledge_t *)buf;

    int n_pts = ivp_ledge_get_n_points(ledge_b_tx);
    ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array_mut(ledge_b_tx);
    for (int i = 0; i < n_pts; i++) {
        ivp_vec3r_t local = ivp_vec3r((ivp_real_t)pts[i].k[0],
                                       (ivp_real_t)pts[i].k[1],
                                       (ivp_real_t)pts[i].k[2]);
        ivp_vec3r_t transformed;
        ivp_transform_point(&transformed, b_in_a, &local);
        pts[i].k[0] = (float)transformed.k[0];
        pts[i].k[1] = (float)transformed.k[1];
        pts[i].k[2] = (float)transformed.k[2];
    }

    /* Find closest pair in A's object space */
    ivp_closest_pair_t pair;
    ivp_result_t res = ivp_find_closest_pair(ledge_a, ledge_b_tx, &pair);

    if (res == IVP_OK && pair.status == IVP_MINDIST_STATUS_OK) {
        /* Sanity check: surface distance should not exceed center-to-center */
        ivp_real_t center_qdist = ivp_vec3r_quad_length(&b_in_a->pos);
        if (pair.qdistance > center_qdist) {
            res = IVP_ERR_DEGENERATE_GEOMETRY;
        }
    }

    if (buf != stack_buf) ivp_free(ctx->alloc, buf, (size_t)ledge_b_size);

    if (res != IVP_OK || pair.status != IVP_MINDIST_STATUS_OK) {
        return true; /* skip this leaf pair, continue traversal */
    }

    /* Build a contact point from the result */
    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = ctx->obj_a;
    cp.obj_b = ctx->obj_b;

    /* Transform closest points from A's object space to world space */
    ivp_transform_point(&cp.point_a, &ca->m_world_f_core_last_psi, &pair.point_a);
    ivp_transform_point(&cp.point_b, &ca->m_world_f_core_last_psi, &pair.point_b);

    /* Compute contact normal (surface diff direction) */
    ivp_vec3r_t surf_diff;
    ivp_vec3r_sub(&surf_diff, &cp.point_b, &cp.point_a);

    ivp_vec3r_t center_diff;
    ivp_vec3r_sub(&center_diff,
                   &cb->pos_world_f_core_last_psi,
                   &ca->pos_world_f_core_last_psi);

    ivp_real_t agreement = ivp_vec3r_dot(&surf_diff, &center_diff);
    ivp_real_t surf_dist = ivp_vec3r_length(&surf_diff);

    if (surf_dist > IVP_REAL_EPS) {
        ivp_real_t inv_sd = IVP_REAL(1.0) / surf_dist;
        cp.normal.k[0] = surf_diff.k[0] * inv_sd;
        cp.normal.k[1] = surf_diff.k[1] * inv_sd;
        cp.normal.k[2] = surf_diff.k[2] * inv_sd;
        if (agreement < IVP_REAL(0.0)) {
            cp.normal.k[0] = -cp.normal.k[0];
            cp.normal.k[1] = -cp.normal.k[1];
            cp.normal.k[2] = -cp.normal.k[2];
        }
    } else {
        ivp_real_t center_dist = ivp_vec3r_length(&center_diff);
        if (center_dist > IVP_REAL_EPS) {
            ivp_real_t inv_cd = IVP_REAL(1.0) / center_dist;
            cp.normal.k[0] = center_diff.k[0] * inv_cd;
            cp.normal.k[1] = center_diff.k[1] * inv_cd;
            cp.normal.k[2] = center_diff.k[2] * inv_cd;
        } else {
            cp.normal.k[0] = IVP_REAL(1.0);
            cp.normal.k[1] = IVP_REAL(0.0);
            cp.normal.k[2] = IVP_REAL(0.0);
        }
    }

    /* Signed gap */
    if (surf_dist > IVP_REAL_EPS) {
        cp.gap_length = (agreement >= IVP_REAL(0.0)) ? surf_dist : -surf_dist;
    } else {
        cp.gap_length = IVP_REAL(0.0);
    }

    /* Project contact points onto normal axis (same as polygon-polygon path) */
    {
        ivp_vec3r_t ra_raw, rb_raw;
        ivp_vec3r_sub(&ra_raw, &cp.point_a, &ca->pos_world_f_core_last_psi);
        ivp_vec3r_sub(&rb_raw, &cp.point_b, &cb->pos_world_f_core_last_psi);

        ivp_real_t proj_a = ivp_vec3r_dot(&ra_raw, &cp.normal);
        ivp_real_t proj_b = ivp_vec3r_dot(&rb_raw, &cp.normal);

        for (int i = 0; i < 3; i++) {
            cp.point_a.k[i] = ca->pos_world_f_core_last_psi.k[i]
                              + proj_a * cp.normal.k[i];
            cp.point_b.k[i] = cb->pos_world_f_core_last_psi.k[i]
                              + proj_b * cp.normal.k[i];
        }
    }

    /* Finalize: r vectors, cross products, tangent basis */
    {
        const ivp_vec3r_t *pos_a = &ca->pos_world_f_core_last_psi;
        const ivp_vec3r_t *pos_b = &cb->pos_world_f_core_last_psi;
        ivp_vec3r_sub(&cp.r_a, &cp.point_a, pos_a);
        ivp_vec3r_sub(&cp.r_b, &cp.point_b, pos_b);

        /* Cross products in core space */
        {
            ivp_vec3r_t r_a_cs, n_cs;
            ivp_mat3r_vimult(&r_a_cs, &ca->m_world_f_core_last_psi.rot, &cp.r_a);
            ivp_mat3r_vimult(&n_cs, &ca->m_world_f_core_last_psi.rot, &cp.normal);
            ivp_vec3r_cross(&cp.cross_normal_cs_a, &r_a_cs, &n_cs);
        }
        {
            ivp_vec3r_t r_b_cs, n_cs;
            ivp_mat3r_vimult(&r_b_cs, &cb->m_world_f_core_last_psi.rot, &cp.r_b);
            ivp_mat3r_vimult(&n_cs, &cb->m_world_f_core_last_psi.rot, &cp.normal);
            ivp_vec3r_cross(&cp.cross_normal_cs_b, &r_b_cs, &n_cs);
        }

        /* Tangent basis */
        ivp_vec3r_t ref;
        if (ivp_fabs(cp.normal.k[0]) < IVP_REAL(0.9)) {
            ref.k[0] = IVP_REAL(1.0);
            ref.k[1] = IVP_REAL(0.0);
            ref.k[2] = IVP_REAL(0.0);
        } else {
            ref.k[0] = IVP_REAL(0.0);
            ref.k[1] = IVP_REAL(1.0);
            ref.k[2] = IVP_REAL(0.0);
        }
        ivp_vec3r_cross(&cp.span_dirs[0], &ref, &cp.normal);
        ivp_real_t len = ivp_vec3r_length(&cp.span_dirs[0]);
        if (len > IVP_REAL_EPS) {
            ivp_real_t inv_len = IVP_REAL(1.0) / len;
            cp.span_dirs[0].k[0] *= inv_len;
            cp.span_dirs[0].k[1] *= inv_len;
            cp.span_dirs[0].k[2] *= inv_len;
        }
        ivp_vec3r_cross(&cp.span_dirs[1], &cp.normal, &cp.span_dirs[0]);
    }

    /* Material combination and virtual mass */
    ivp_contact_combine_materials(&cp);
    ivp_contact_compute_virtual_mass(&cp);

    /* Deduplicate against existing contacts */
    for (int i = 0; i < ctx->count; i++) {
        if (contact_is_duplicate_local(&ctx->contacts_out[i], &cp)) {
            return true; /* duplicate, skip but continue traversal */
        }
    }

    ctx->contacts_out[ctx->count++] = cp;
    return (ctx->count < ctx->max_contacts);
}

/* ── Hierarchical multi-contact computation ─────────────────────────────── */

int ivp_contact_compute_hierarchical_multi(ivp_object_t *obj_a,
                                            ivp_object_t *obj_b,
                                            ivp_contact_point_t *contacts_out,
                                            int max_contacts)
{
    if (!obj_a || !obj_b || !contacts_out || max_contacts <= 0) {
        return 0;
    }

    const ivp_core_t *ca = &obj_a->core;
    const ivp_core_t *cb = &obj_b->core;

    /* Get ledgetree roots. For simple objects, use the ledgetree root
     * from the compact surface (which IS terminal for single-convex). */
    const ivp_compact_ledgetree_node_t *root_a = NULL;
    const ivp_compact_ledgetree_node_t *root_b = NULL;

    if (obj_a->surman) {
        /* Try non-terminal root first (compound) */
        root_a = ivp_surman_get_ledgetree_root(obj_a->surman);
        if (!root_a) {
            /* Single-convex: get the terminal root node from the surface */
            if (ivp_surman_get_type(obj_a->surman) == IVP_SURMAN_POLYGON) {
                const ivp_surman_polygon_t *poly =
                    (const ivp_surman_polygon_t *)obj_a->surman;
                root_a = ivp_surface_get_ledgetree_root(poly->compact_surface);
            }
        }
    }

    if (obj_b->surman) {
        root_b = ivp_surman_get_ledgetree_root(obj_b->surman);
        if (!root_b) {
            if (ivp_surman_get_type(obj_b->surman) == IVP_SURMAN_POLYGON) {
                const ivp_surman_polygon_t *poly =
                    (const ivp_surman_polygon_t *)obj_b->surman;
                root_b = ivp_surface_get_ledgetree_root(poly->compact_surface);
            }
        }
    }

    if (!root_a || !root_b) {
        return 0; /* Cannot do hierarchical collision without tree roots */
    }

    /* Compute relative transform: B's object space → A's object space */
    ivp_transform_r_t b_in_a;
    ivp_transform_inv_compose(&b_in_a,
                               &ca->m_world_f_core_last_psi,
                               &cb->m_world_f_core_last_psi);

    /* Set up callback context */
    leaf_pair_ctx_t ctx;
    ctx.obj_a = obj_a;
    ctx.obj_b = obj_b;
    ctx.contacts_out = contacts_out;
    ctx.max_contacts = max_contacts;
    ctx.count = 0;
    ctx.alloc = ivp_default_allocator();

    /* Use a margin based on the sum of surface deviations */
    ivp_real_t margin = ca->max_surface_deviation + cb->max_surface_deviation;
    if (margin < IVP_REAL(0.01)) {
        margin = IVP_REAL(0.01);
    }

    ivp_ledgetree_traverse_pairs(root_a, root_b, &b_in_a, margin,
                                  leaf_pair_callback, &ctx);

    /* Delegate recursive wakeups through OO watcher semantics so compound
     * hierarchies can invalidate child mindists via a single gate. */
    if (!ivp_ledgetree_is_terminal(root_a) || !ivp_ledgetree_is_terminal(root_b)) {
        ivp_oo_watcher_t watcher;
        ivp_oo_watcher_init(&watcher, obj_a, obj_b, ctx.alloc);
        ivp_oo_watcher_hull_wakeup(&watcher);
        ivp_oo_watcher_destroy(&watcher);
    }

    return ctx.count;
}
