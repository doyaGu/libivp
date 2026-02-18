/* ivp_contact.c — Contact detection and physics
 *
 * Range-manager-aware broadphase, sphere-sphere contact geometry,
 * virtual mass computation, material combination, impulse application.
 */
#include <ivp/solver/ivp_contact.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/collision/ivp_mindist.h>
#include <ivp/collision/ivp_compact_ledge_solver.h>
#include <ivp/collision/ivp_range_manager.h>
#include <ivp/collision/ivp_ledgetree_collision.h>
#include <ivp/geometry/ivp_surman.h>
#include <string.h>
#include <math.h>
#include <limits.h>

/* ── Material combination ───────────────────────────────────────────────── */

void ivp_contact_combine_materials(ivp_contact_point_t *cp)
{
    /* Check if the environment has a custom material manager */
    ivp_environment_t *env = cp->obj_a ? cp->obj_a->owner : NULL;
    if (!env && cp->obj_b) {
        env = cp->obj_b->owner;
    }
    if (env && env->material_manager) {
        ivp_contact_situation_t sit;
        sit.obj_a = cp->obj_a;
        sit.obj_b = cp->obj_b;
        /* Legacy contact situation uses the primary (object A) contact point. */
        sit.contact_point_ws = cp->point_a;
        sit.contact_normal_ws = cp->normal;

        env->material_manager(env->material_manager_ctx, &sit,
                              &cp->friction, &cp->elasticity, &cp->adhesion);
        return;
    }

    /* Default IVP material combination rules:
     *   friction:   multiply
     *   elasticity: multiply
     *   adhesion:   add */
    const ivp_material_t *ma = &cp->obj_a->material;
    const ivp_material_t *mb = &cp->obj_b->material;
    cp->friction   = ma->friction   * mb->friction;
    cp->elasticity = ma->elasticity * mb->elasticity;
    cp->adhesion   = ma->adhesion   + mb->adhesion;
}

/* ── Tangent basis from normal ──────────────────────────────────────────── */

static void compute_tangent_basis(const ivp_vec3r_t *normal,
                                   ivp_vec3r_t *span0,
                                   ivp_vec3r_t *span1)
{
    /* Pick the axis least aligned with normal to avoid degeneracy */
    ivp_vec3r_t ref;
    if (ivp_fabs(normal->k[0]) < IVP_REAL(0.9)) {
        ref.k[0] = IVP_REAL(1.0);
        ref.k[1] = IVP_REAL(0.0);
        ref.k[2] = IVP_REAL(0.0);
    } else {
        ref.k[0] = IVP_REAL(0.0);
        ref.k[1] = IVP_REAL(1.0);
        ref.k[2] = IVP_REAL(0.0);
    }

    /* span0 = normalize(ref × normal) */
    ivp_vec3r_cross(span0, &ref, normal);
    ivp_real_t len = ivp_vec3r_length(span0);
    if (len > IVP_REAL_EPS) {
        ivp_real_t inv_len = IVP_REAL(1.0) / len;
        span0->k[0] *= inv_len;
        span0->k[1] *= inv_len;
        span0->k[2] *= inv_len;
    }

    /* span1 = normal × span0 (already unit) */
    ivp_vec3r_cross(span1, normal, span0);
}

static void invalidate_contact_geometry(ivp_contact_point_t *cp)
{
    if (!cp) return;
    ivp_vec3r_set(&cp->normal, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    cp->point_a = cp->obj_a->core.pos_world_f_core_last_psi;
    cp->point_b = cp->obj_b->core.pos_world_f_core_last_psi;
    cp->gap_length = IVP_REAL_MAX;
    ivp_vec3r_set_zero(&cp->r_a);
    ivp_vec3r_set_zero(&cp->r_b);
    ivp_vec3r_set_zero(&cp->cross_normal_cs_a);
    ivp_vec3r_set_zero(&cp->cross_normal_cs_b);
    ivp_vec3r_set_zero(&cp->span_dirs[0]);
    ivp_vec3r_set_zero(&cp->span_dirs[1]);
}

/* ── Finalize contact geometry (shared) ──────────────────────────────────── */

/* After normal, point_a, point_b, gap_length are set, this computes
 * r_a, r_b, cross products, and tangent spans. */
static void finalize_contact_geometry(ivp_contact_point_t *cp)
{
    const ivp_core_t *ca = &cp->obj_a->core;
    const ivp_core_t *cb = &cp->obj_b->core;

    const ivp_vec3r_t *pos_a = &ca->pos_world_f_core_last_psi;
    const ivp_vec3r_t *pos_b = &cb->pos_world_f_core_last_psi;

    /* Offset from center of mass */
    ivp_vec3r_sub(&cp->r_a, &cp->point_a, pos_a);
    ivp_vec3r_sub(&cp->r_b, &cp->point_b, pos_b);

    /* Cross products in core space */
    {
        ivp_vec3r_t r_a_cs, n_cs;
        ivp_mat3r_vimult(&r_a_cs, &ca->m_world_f_core_last_psi.rot, &cp->r_a);
        ivp_mat3r_vimult(&n_cs, &ca->m_world_f_core_last_psi.rot, &cp->normal);
        ivp_vec3r_cross(&cp->cross_normal_cs_a, &r_a_cs, &n_cs);
    }
    {
        ivp_vec3r_t r_b_cs, n_cs;
        ivp_mat3r_vimult(&r_b_cs, &cb->m_world_f_core_last_psi.rot, &cp->r_b);
        ivp_mat3r_vimult(&n_cs, &cb->m_world_f_core_last_psi.rot, &cp->normal);
        ivp_vec3r_cross(&cp->cross_normal_cs_b, &r_b_cs, &n_cs);
    }

    /* Tangent basis */
    compute_tangent_basis(&cp->normal, &cp->span_dirs[0], &cp->span_dirs[1]);
}

/* ── Sphere-sphere contact geometry ─────────────────────────────────────── */

void ivp_contact_compute_sphere_sphere(ivp_contact_point_t *cp)
{
    const ivp_core_t *ca = &cp->obj_a->core;
    const ivp_core_t *cb = &cp->obj_b->core;

    /* Positions */
    const ivp_vec3r_t *pos_a = &ca->pos_world_f_core_last_psi;
    const ivp_vec3r_t *pos_b = &cb->pos_world_f_core_last_psi;

    /* Normal = normalized (B - A) */
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff, pos_b, pos_a);

    ivp_real_t dist = ivp_vec3r_length(&diff);
    if (dist < IVP_REAL_EPS) {
        cp->normal.k[0] = IVP_REAL(1.0);
        cp->normal.k[1] = IVP_REAL(0.0);
        cp->normal.k[2] = IVP_REAL(0.0);
        dist = IVP_REAL_EPS;
    } else {
        ivp_real_t inv_dist = IVP_REAL(1.0) / dist;
        cp->normal.k[0] = diff.k[0] * inv_dist;
        cp->normal.k[1] = diff.k[1] * inv_dist;
        cp->normal.k[2] = diff.k[2] * inv_dist;
    }

    ivp_real_t ra = ca->upper_limit_radius;
    ivp_real_t rb = cb->upper_limit_radius;

    cp->gap_length = dist - (ra + rb);

    cp->point_a.k[0] = pos_a->k[0] + cp->normal.k[0] * ra;
    cp->point_a.k[1] = pos_a->k[1] + cp->normal.k[1] * ra;
    cp->point_a.k[2] = pos_a->k[2] + cp->normal.k[2] * ra;

    cp->point_b.k[0] = pos_b->k[0] - cp->normal.k[0] * rb;
    cp->point_b.k[1] = pos_b->k[1] - cp->normal.k[1] * rb;
    cp->point_b.k[2] = pos_b->k[2] - cp->normal.k[2] * rb;

    finalize_contact_geometry(cp);
}

/* ── Sphere vs polygon contact ──────────────────────────────────────────── */

void ivp_contact_compute_sphere_polygon(ivp_contact_point_t *cp)
{
    const ivp_core_t *ca = &cp->obj_a->core; /* sphere */
    const ivp_core_t *cb = &cp->obj_b->core; /* polygon */

    /* Get the convex ledge from the polygon's surman */
    const ivp_compact_ledge_t *ledge =
        ivp_surman_get_single_convex(cp->obj_b->surman);

    if (!ledge) {
        /* Hard-switch parity: do not fabricate fallback contacts. */
        invalidate_contact_geometry(cp);
        return;
    }

    /* Transform sphere center from world space to polygon's object space */
    ivp_vec3r_t center_os;
    ivp_transform_inv_point(&center_os,
                             &cb->m_world_f_core_last_psi,
                             &ca->pos_world_f_core_last_psi);

    /* Find closest point on ledge to the sphere center (in object space) */
    ivp_real_t qdist_min = IVP_REAL_MAX;
    ivp_vec3r_t closest_os = center_os; /* fallback */

    int n_tri = ivp_ledge_get_n_triangles(ledge);
    const ivp_compact_triangle_t *tri0 = ivp_ledge_get_first_triangle(ledge);

    for (int t = 0; t < n_tri; t++) {
        const ivp_compact_triangle_t *tri = &tri0[t];

        /* Get triangle vertices in object space */
        ivp_vec3r_t v[3];
        for (int e = 0; e < 3; e++) {
            const ivp_compact_edge_t *edge = ivp_tri_get_edge(tri, e);
            const ivp_compact_poly_point_t *pp =
                ivp_edge_get_start_point(edge, ledge);
            v[e] = ivp_vec3r((ivp_real_t)pp->k[0],
                             (ivp_real_t)pp->k[1],
                             (ivp_real_t)pp->k[2]);
        }

        /* Find closest point on this triangle to center_os */
        /* Use barycentric projection */
        ivp_vec3r_t ab, ac, ap;
        ivp_vec3r_sub(&ab, &v[1], &v[0]);
        ivp_vec3r_sub(&ac, &v[2], &v[0]);
        ivp_vec3r_sub(&ap, &center_os, &v[0]);

        ivp_real_t d00 = ivp_vec3r_dot(&ab, &ab);
        ivp_real_t d01 = ivp_vec3r_dot(&ab, &ac);
        ivp_real_t d11 = ivp_vec3r_dot(&ac, &ac);
        ivp_real_t d20 = ivp_vec3r_dot(&ap, &ab);
        ivp_real_t d21 = ivp_vec3r_dot(&ap, &ac);

        ivp_real_t denom = d00 * d11 - d01 * d01;
        if (ivp_fabs(denom) < IVP_REAL_EPS) continue;

        ivp_real_t inv_denom = IVP_REAL(1.0) / denom;
        ivp_real_t u = (d11 * d20 - d01 * d21) * inv_denom;
        ivp_real_t w = (d00 * d21 - d01 * d20) * inv_denom;

        /* Clamp to triangle */
        ivp_vec3r_t cp_os;
        if (u >= IVP_REAL(0.0) && w >= IVP_REAL(0.0) &&
            u + w <= IVP_REAL(1.0)) {
            /* Inside triangle */
            cp_os.k[0] = v[0].k[0] + ab.k[0] * u + ac.k[0] * w;
            cp_os.k[1] = v[0].k[1] + ab.k[1] * u + ac.k[1] * w;
            cp_os.k[2] = v[0].k[2] + ab.k[2] * u + ac.k[2] * w;
        } else {
            /* Project onto edges and pick closest */
            ivp_real_t best_q = IVP_REAL_MAX;
            cp_os = v[0];

            for (int i = 0; i < 3; i++) {
                int j = (i + 1) % 3;
                ivp_vec3r_t edge_vec, to_point;
                ivp_vec3r_sub(&edge_vec, &v[j], &v[i]);
                ivp_vec3r_sub(&to_point, &center_os, &v[i]);

                ivp_real_t el = ivp_vec3r_dot(&edge_vec, &edge_vec);
                if (el < IVP_REAL_EPS) continue;

                ivp_real_t param = ivp_vec3r_dot(&to_point, &edge_vec) / el;
                if (param < IVP_REAL(0.0)) param = IVP_REAL(0.0);
                if (param > IVP_REAL(1.0)) param = IVP_REAL(1.0);

                ivp_vec3r_t candidate;
                candidate.k[0] = v[i].k[0] + edge_vec.k[0] * param;
                candidate.k[1] = v[i].k[1] + edge_vec.k[1] * param;
                candidate.k[2] = v[i].k[2] + edge_vec.k[2] * param;

                ivp_real_t q = ivp_vec3r_quad_distance(&center_os, &candidate);
                if (q < best_q) {
                    best_q = q;
                    cp_os = candidate;
                }
            }
        }

        ivp_real_t q = ivp_vec3r_quad_distance(&center_os, &cp_os);
        if (q < qdist_min) {
            qdist_min = q;
            closest_os = cp_os;
        }
    }

    /* Transform closest point back to world space */
    ivp_vec3r_t closest_ws;
    ivp_transform_point(&closest_ws,
                         &cb->m_world_f_core_last_psi,
                         &closest_os);

    /* Normal = normalized (closest_ws - sphere_center) */
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff, &closest_ws, &ca->pos_world_f_core_last_psi);

    ivp_real_t dist = ivp_vec3r_length(&diff);
    if (dist < IVP_REAL_EPS) {
        cp->normal.k[0] = IVP_REAL(1.0);
        cp->normal.k[1] = IVP_REAL(0.0);
        cp->normal.k[2] = IVP_REAL(0.0);
        dist = IVP_REAL_EPS;
    } else {
        ivp_real_t inv_dist = IVP_REAL(1.0) / dist;
        cp->normal.k[0] = diff.k[0] * inv_dist;
        cp->normal.k[1] = diff.k[1] * inv_dist;
        cp->normal.k[2] = diff.k[2] * inv_dist;
    }

    ivp_real_t ra = ca->upper_limit_radius;

    /* Contact point on sphere surface (toward polygon) */
    cp->point_a.k[0] = ca->pos_world_f_core_last_psi.k[0] + cp->normal.k[0] * ra;
    cp->point_a.k[1] = ca->pos_world_f_core_last_psi.k[1] + cp->normal.k[1] * ra;
    cp->point_a.k[2] = ca->pos_world_f_core_last_psi.k[2] + cp->normal.k[2] * ra;

    /* Contact point on polygon surface */
    cp->point_b = closest_ws;

    /* Gap = distance from sphere center to polygon surface - sphere radius */
    cp->gap_length = dist - ra;

    finalize_contact_geometry(cp);
}

static bool compute_triangle_normal_world(const ivp_core_t *core,
                                          const ivp_compact_ledge_t *ledge,
                                          const ivp_compact_edge_t *edge,
                                          ivp_vec3r_t *normal_out)
{
    if (!core || !ledge || !edge || !normal_out) return false;

    const ivp_compact_triangle_t *tri = ivp_edge_get_triangle(edge);
    const ivp_compact_edge_t *e0 = &tri->edges[0];
    const ivp_compact_edge_t *e1 = &tri->edges[1];
    const ivp_compact_edge_t *e2 = &tri->edges[2];

    const ivp_compact_poly_point_t *pp0 = ivp_edge_get_start_point(e0, ledge);
    const ivp_compact_poly_point_t *pp1 = ivp_edge_get_start_point(e1, ledge);
    const ivp_compact_poly_point_t *pp2 = ivp_edge_get_start_point(e2, ledge);

    ivp_vec3r_t p0 = ivp_vec3r((ivp_real_t)pp0->k[0], (ivp_real_t)pp0->k[1], (ivp_real_t)pp0->k[2]);
    ivp_vec3r_t p1 = ivp_vec3r((ivp_real_t)pp1->k[0], (ivp_real_t)pp1->k[1], (ivp_real_t)pp1->k[2]);
    ivp_vec3r_t p2 = ivp_vec3r((ivp_real_t)pp2->k[0], (ivp_real_t)pp2->k[1], (ivp_real_t)pp2->k[2]);

    ivp_vec3r_t a, b, n_os;
    ivp_vec3r_sub(&a, &p1, &p0);
    ivp_vec3r_sub(&b, &p2, &p0);
    ivp_vec3r_cross(&n_os, &a, &b);

    ivp_real_t n_len = ivp_vec3r_length(&n_os);
    if (n_len <= IVP_REAL_EPS) return false;

    ivp_real_t inv_n_len = IVP_REAL(1.0) / n_len;
    n_os.k[0] *= inv_n_len;
    n_os.k[1] *= inv_n_len;
    n_os.k[2] *= inv_n_len;

    ivp_mat3r_vmult(normal_out, &core->m_world_f_core_last_psi.rot, &n_os);

    ivp_real_t world_len = ivp_vec3r_length(normal_out);
    if (world_len <= IVP_REAL_EPS) return false;

    ivp_real_t inv_world_len = IVP_REAL(1.0) / world_len;
    normal_out->k[0] *= inv_world_len;
    normal_out->k[1] *= inv_world_len;
    normal_out->k[2] *= inv_world_len;

    return true;
}

/* ── Polygon vs polygon contact ─────────────────────────────────────────── */

void ivp_contact_compute_polygon_polygon(ivp_contact_point_t *cp)
{
    const ivp_core_t *ca = &cp->obj_a->core;
    const ivp_core_t *cb = &cp->obj_b->core;

    const ivp_compact_ledge_t *ledge_a =
        cp->obj_a->surman ? ivp_surman_get_single_convex(cp->obj_a->surman) : NULL;
    const ivp_compact_ledge_t *ledge_b =
        cp->obj_b->surman ? ivp_surman_get_single_convex(cp->obj_b->surman) : NULL;

    if (!ledge_a || !ledge_b) {
        invalidate_contact_geometry(cp);
        return;
    }

    /* Transform B's vertices into A's object space.
     * inv(A) * B gives the transform from B's object space to A's. */
    ivp_transform_r_t b_in_a;
    ivp_transform_inv_compose(&b_in_a,
                               &ca->m_world_f_core_last_psi,
                               &cb->m_world_f_core_last_psi);

    /* Create a temporary copy of ledge_b with vertices transformed into
     * A's object space. The compact ledge structure is self-contained
     * with relative offsets, so a byte copy preserves all edge/triangle
     * references — we only need to transform the point coordinates. */
    int ledge_b_size = ivp_ledge_get_size(ledge_b);
    int ledge_b_triangles = ivp_ledge_get_n_triangles(ledge_b);
    int ledge_b_points = ivp_ledge_get_n_points(ledge_b);

    if (ledge_b_size < (int)sizeof(ivp_compact_ledge_t) ||
        (ledge_b_size & 0xF) != 0 ||
        ledge_b_triangles <= 0 ||
        ledge_b_points <= 0 ||
        ledge_b_points > INT16_MAX) {
        invalidate_contact_geometry(cp);
        return;
    }

    /* Use stack for small ledges, allocator for large ones */
    const ivp_allocator_t *alloc = cp->obj_a->owner
        ? (const ivp_allocator_t *)cp->obj_a->owner->allocator
        : ivp_default_allocator();
    _Alignas(16) uint8_t stack_buf[4096];
    uint8_t *buf = (ledge_b_size <= (int)sizeof(stack_buf))
                 ? stack_buf : (uint8_t *)ivp_alloc(alloc, (size_t)ledge_b_size);
    if (!buf) {
        invalidate_contact_geometry(cp);
        return;
    }

    memcpy(buf, ledge_b, (size_t)ledge_b_size);

    ivp_compact_ledge_t *ledge_b_tx = (ivp_compact_ledge_t *)buf;
    int n_pts = ivp_ledge_get_n_points(ledge_b_tx);
    if (n_pts != ledge_b_points || n_pts <= 0) {
        if (buf != stack_buf) ivp_free(alloc, buf, (size_t)ledge_b_size);
        invalidate_contact_geometry(cp);
        return;
    }
    ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array_mut(ledge_b_tx);
    for (int i = 0; i < n_pts; i++) {
        ivp_vec3r_t local = ivp_vec3r((ivp_real_t)pts[i].k[0],
                                       (ivp_real_t)pts[i].k[1],
                                       (ivp_real_t)pts[i].k[2]);
        ivp_vec3r_t transformed;
        ivp_transform_point(&transformed, &b_in_a, &local);
        pts[i].k[0] = (float)transformed.k[0];
        pts[i].k[1] = (float)transformed.k[1];
        pts[i].k[2] = (float)transformed.k[2];
    }

    /* Run hill-climbing closest pair in A's object space */
    ivp_closest_pair_t pair;
    ivp_result_t res = ivp_find_closest_pair(ledge_a, ledge_b_tx, &pair);

    /* Sanity check: for two convex shapes, the closest surface distance
     * must be <= center-to-center distance (centers are inside the shapes).
     * The center of ledge_b in A's space is at b_in_a.pos.
     * If the hill-climbing result exceeds this bound, it converged to a
     * local minimum — fall back to brute-force. */
    if (res == IVP_OK) {
        if (pair.status != IVP_MINDIST_STATUS_OK) {
            res = IVP_ERR_DEGENERATE_GEOMETRY;
        }
        ivp_real_t center_qdist = ivp_vec3r_quad_length(&b_in_a.pos);
        if (pair.qdistance > center_qdist) {
            res = IVP_ERR_DEGENERATE_GEOMETRY; /* force brute-force fallback */
        }
    }

    if (res != IVP_OK) {
        /* Hill-climbing failed — use brute-force vertex-to-face minimum distance
         * as a robust fallback instead of sphere-sphere approximation.
         * Check every vertex of B against all triangles of A and vice versa. */
        ivp_real_t best_dist2 = IVP_REAL_MAX;
        ivp_vec3r_t best_pa = {{IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0)}};
        ivp_vec3r_t best_pb = best_pa;

        /* B vertices → A faces */
        int n_pts_b = ivp_ledge_get_n_points(ledge_b_tx);
        const ivp_compact_poly_point_t *pts_b = ivp_ledge_get_point_array(ledge_b_tx);
        int n_tri_a = ivp_ledge_get_n_triangles(ledge_a);
        const ivp_compact_triangle_t *tri0_a = ivp_ledge_get_first_triangle(ledge_a);

        for (int pi = 0; pi < n_pts_b; pi++) {
            ivp_vec3r_t pb = ivp_vec3r((ivp_real_t)pts_b[pi].k[0],
                                        (ivp_real_t)pts_b[pi].k[1],
                                        (ivp_real_t)pts_b[pi].k[2]);
            for (int t = 0; t < n_tri_a; t++) {
                const ivp_compact_triangle_t *tri = &tri0_a[t];
                ivp_vec3r_t v[3];
                for (int e = 0; e < 3; e++) {
                    const ivp_compact_edge_t *edge = ivp_tri_get_edge(tri, e);
                    const ivp_compact_poly_point_t *pp = ivp_edge_get_start_point(edge, ledge_a);
                    v[e] = ivp_vec3r((ivp_real_t)pp->k[0], (ivp_real_t)pp->k[1], (ivp_real_t)pp->k[2]);
                }
                /* Project pb onto triangle plane */
                ivp_vec3r_t ab, ac, ap;
                ivp_vec3r_sub(&ab, &v[1], &v[0]);
                ivp_vec3r_sub(&ac, &v[2], &v[0]);
                ivp_vec3r_sub(&ap, &pb, &v[0]);
                ivp_real_t d00 = ivp_vec3r_dot(&ab, &ab);
                ivp_real_t d01 = ivp_vec3r_dot(&ab, &ac);
                ivp_real_t d11 = ivp_vec3r_dot(&ac, &ac);
                ivp_real_t d20 = ivp_vec3r_dot(&ap, &ab);
                ivp_real_t d21 = ivp_vec3r_dot(&ap, &ac);
                ivp_real_t denom = d00 * d11 - d01 * d01;
                if (ivp_fabs(denom) < IVP_REAL_EPS) continue;
                ivp_real_t inv_d = IVP_REAL(1.0) / denom;
                ivp_real_t u = (d11 * d20 - d01 * d21) * inv_d;
                ivp_real_t w = (d00 * d21 - d01 * d20) * inv_d;

                ivp_vec3r_t closest;
                if (u >= IVP_REAL(0.0) && w >= IVP_REAL(0.0) && u + w <= IVP_REAL(1.0)) {
                    closest.k[0] = v[0].k[0] + ab.k[0] * u + ac.k[0] * w;
                    closest.k[1] = v[0].k[1] + ab.k[1] * u + ac.k[1] * w;
                    closest.k[2] = v[0].k[2] + ab.k[2] * u + ac.k[2] * w;
                } else {
                    /* Clamp to nearest edge */
                    ivp_real_t bq = IVP_REAL_MAX;
                    closest = v[0];
                    for (int i = 0; i < 3; i++) {
                        int j = (i + 1) % 3;
                        ivp_vec3r_t ev, tp;
                        ivp_vec3r_sub(&ev, &v[j], &v[i]);
                        ivp_vec3r_sub(&tp, &pb, &v[i]);
                        ivp_real_t el = ivp_vec3r_dot(&ev, &ev);
                        if (el < IVP_REAL_EPS) continue;
                        ivp_real_t s = ivp_vec3r_dot(&tp, &ev) / el;
                        if (s < IVP_REAL(0.0)) s = IVP_REAL(0.0);
                        if (s > IVP_REAL(1.0)) s = IVP_REAL(1.0);
                        ivp_vec3r_t cand;
                        cand.k[0] = v[i].k[0] + ev.k[0] * s;
                        cand.k[1] = v[i].k[1] + ev.k[1] * s;
                        cand.k[2] = v[i].k[2] + ev.k[2] * s;
                        ivp_real_t q = ivp_vec3r_quad_distance(&pb, &cand);
                        if (q < bq) { bq = q; closest = cand; }
                    }
                }
                ivp_real_t dist2 = ivp_vec3r_quad_distance(&pb, &closest);
                if (dist2 < best_dist2) {
                    best_dist2 = dist2;
                    best_pa = closest;
                    best_pb = pb;
                }
            }
        }

        /* A vertices → B faces */
        int n_pts_a = ivp_ledge_get_n_points(ledge_a);
        const ivp_compact_poly_point_t *pts_a = ivp_ledge_get_point_array(ledge_a);
        int n_tri_b = ivp_ledge_get_n_triangles(ledge_b_tx);
        const ivp_compact_triangle_t *tri0_b = ivp_ledge_get_first_triangle(ledge_b_tx);

        for (int pi = 0; pi < n_pts_a; pi++) {
            ivp_vec3r_t pa = ivp_vec3r((ivp_real_t)pts_a[pi].k[0],
                                        (ivp_real_t)pts_a[pi].k[1],
                                        (ivp_real_t)pts_a[pi].k[2]);
            for (int t = 0; t < n_tri_b; t++) {
                const ivp_compact_triangle_t *tri = &tri0_b[t];
                ivp_vec3r_t v[3];
                for (int e = 0; e < 3; e++) {
                    const ivp_compact_edge_t *edge = ivp_tri_get_edge(tri, e);
                    const ivp_compact_poly_point_t *pp = ivp_edge_get_start_point(edge, ledge_b_tx);
                    v[e] = ivp_vec3r((ivp_real_t)pp->k[0], (ivp_real_t)pp->k[1], (ivp_real_t)pp->k[2]);
                }
                ivp_vec3r_t ab, ac, ap;
                ivp_vec3r_sub(&ab, &v[1], &v[0]);
                ivp_vec3r_sub(&ac, &v[2], &v[0]);
                ivp_vec3r_sub(&ap, &pa, &v[0]);
                ivp_real_t d00 = ivp_vec3r_dot(&ab, &ab);
                ivp_real_t d01 = ivp_vec3r_dot(&ab, &ac);
                ivp_real_t d11 = ivp_vec3r_dot(&ac, &ac);
                ivp_real_t d20 = ivp_vec3r_dot(&ap, &ab);
                ivp_real_t d21 = ivp_vec3r_dot(&ap, &ac);
                ivp_real_t denom = d00 * d11 - d01 * d01;
                if (ivp_fabs(denom) < IVP_REAL_EPS) continue;
                ivp_real_t inv_d = IVP_REAL(1.0) / denom;
                ivp_real_t u = (d11 * d20 - d01 * d21) * inv_d;
                ivp_real_t w = (d00 * d21 - d01 * d20) * inv_d;

                ivp_vec3r_t closest;
                if (u >= IVP_REAL(0.0) && w >= IVP_REAL(0.0) && u + w <= IVP_REAL(1.0)) {
                    closest.k[0] = v[0].k[0] + ab.k[0] * u + ac.k[0] * w;
                    closest.k[1] = v[0].k[1] + ab.k[1] * u + ac.k[1] * w;
                    closest.k[2] = v[0].k[2] + ab.k[2] * u + ac.k[2] * w;
                } else {
                    ivp_real_t bq = IVP_REAL_MAX;
                    closest = v[0];
                    for (int i = 0; i < 3; i++) {
                        int j = (i + 1) % 3;
                        ivp_vec3r_t ev, tp;
                        ivp_vec3r_sub(&ev, &v[j], &v[i]);
                        ivp_vec3r_sub(&tp, &pa, &v[i]);
                        ivp_real_t el = ivp_vec3r_dot(&ev, &ev);
                        if (el < IVP_REAL_EPS) continue;
                        ivp_real_t s = ivp_vec3r_dot(&tp, &ev) / el;
                        if (s < IVP_REAL(0.0)) s = IVP_REAL(0.0);
                        if (s > IVP_REAL(1.0)) s = IVP_REAL(1.0);
                        ivp_vec3r_t cand;
                        cand.k[0] = v[i].k[0] + ev.k[0] * s;
                        cand.k[1] = v[i].k[1] + ev.k[1] * s;
                        cand.k[2] = v[i].k[2] + ev.k[2] * s;
                        ivp_real_t q = ivp_vec3r_quad_distance(&pa, &cand);
                        if (q < bq) { bq = q; closest = cand; }
                    }
                }
                ivp_real_t dist2 = ivp_vec3r_quad_distance(&pa, &closest);
                if (dist2 < best_dist2) {
                    best_dist2 = dist2;
                    best_pa = pa;
                    best_pb = closest;
                }
            }
        }

        if (best_dist2 < IVP_REAL_MAX) {
            pair.point_a = best_pa;
            pair.point_b = best_pb;
            pair.status = IVP_MINDIST_STATUS_OK;
            res = IVP_OK;
        }
    }

    if (buf != stack_buf) ivp_free(alloc, buf, (size_t)ledge_b_size);

    if (res != IVP_OK) {
        invalidate_contact_geometry(cp);
        return;
    }

    /* Transform results from A's object space to world space */
    ivp_transform_point(&cp->point_a,
                         &ca->m_world_f_core_last_psi, &pair.point_a);
    ivp_transform_point(&cp->point_b,
                         &ca->m_world_f_core_last_psi, &pair.point_b);

    /* Compute contact normal from the closest surface pair.
     *
     * For separated shapes, surf_diff = (B_point - A_point) points from A's
     * surface toward B's surface — this is the correct separation direction.
     *
     * For overlapping shapes, B_point has crossed past A_point, so surf_diff
     * points in the PENETRATION direction. Detect this by comparing with the
     * center-to-center direction: if they disagree (negative dot), flip the
     * normal to get the separation direction. */
    ivp_vec3r_t surf_diff;
    ivp_vec3r_sub(&surf_diff, &cp->point_b, &cp->point_a);

    ivp_vec3r_t center_diff;
    ivp_vec3r_sub(&center_diff,
                   &cb->pos_world_f_core_last_psi,
                   &ca->pos_world_f_core_last_psi);

    /* Check if surface direction agrees with center-to-center */
    ivp_real_t agreement = ivp_vec3r_dot(&surf_diff, &center_diff);
    ivp_real_t surf_dist = ivp_vec3r_length(&surf_diff);
    if (surf_dist > IVP_REAL_EPS) {
        ivp_real_t inv_sd = IVP_REAL(1.0) / surf_dist;
        cp->normal.k[0] = surf_diff.k[0] * inv_sd;
        cp->normal.k[1] = surf_diff.k[1] * inv_sd;
        cp->normal.k[2] = surf_diff.k[2] * inv_sd;
        if (agreement < IVP_REAL(0.0)) {
            cp->normal.k[0] = -cp->normal.k[0];
            cp->normal.k[1] = -cp->normal.k[1];
            cp->normal.k[2] = -cp->normal.k[2];
        }
    } else {
        bool normal_ok = false;

        if (pair.type_a == IVP_FEATURE_TRIANGLE && pair.edge_a) {
            normal_ok = compute_triangle_normal_world(ca, ledge_a, pair.edge_a, &cp->normal);
        }
        if (!normal_ok && pair.type_b == IVP_FEATURE_TRIANGLE && pair.edge_b) {
            normal_ok = compute_triangle_normal_world(cb, ledge_b, pair.edge_b, &cp->normal);
        }

        if (normal_ok) {
            if (ivp_vec3r_dot(&cp->normal, &center_diff) < IVP_REAL(0.0)) {
                cp->normal.k[0] = -cp->normal.k[0];
                cp->normal.k[1] = -cp->normal.k[1];
                cp->normal.k[2] = -cp->normal.k[2];
            }
        } else {
            /* Surface points coincide — final fallback to center-to-center */
            ivp_real_t center_dist = ivp_vec3r_length(&center_diff);
            if (center_dist > IVP_REAL_EPS) {
                ivp_real_t inv_cd = IVP_REAL(1.0) / center_dist;
                cp->normal.k[0] = center_diff.k[0] * inv_cd;
                cp->normal.k[1] = center_diff.k[1] * inv_cd;
                cp->normal.k[2] = center_diff.k[2] * inv_cd;
            } else {
                cp->normal.k[0] = IVP_REAL(1.0);
                cp->normal.k[1] = IVP_REAL(0.0);
                cp->normal.k[2] = IVP_REAL(0.0);
            }
        }
    }

    /* Signed gap: positive when separated, negative when overlapping */
    if (surf_dist > IVP_REAL_EPS) {
        cp->gap_length = (agreement >= IVP_REAL(0.0)) ? surf_dist : -surf_dist;
    } else {
        cp->gap_length = IVP_REAL(0.0);
    }

    /* Project contact points onto the normal axis through each center.
     *
     * The closest-pair algorithm returns the closest POINT pair, which for
     * overlapping boxes is often a corner (e.g., r_b = (0.5, 0.5, 0.5) for a
     * unit box).  The large tangential component of r causes cross(r, n) ≠ 0,
     * inflating the angular term in virtual mass so that the impulse is far too
     * weak for face-to-face contacts.
     *
     * By projecting r onto the normal, the tangential component is removed:
     *   r' = (r · n) * n   →   cross(r', n) = 0   →   angular_term = 0
     *   →   vm = mass (correct for face contacts).
     *
     * The gap_length is already computed from the original surface pair and is
     * unaffected. */
    {
        ivp_vec3r_t ra_raw, rb_raw;
        ivp_vec3r_sub(&ra_raw, &cp->point_a, &ca->pos_world_f_core_last_psi);
        ivp_vec3r_sub(&rb_raw, &cp->point_b, &cb->pos_world_f_core_last_psi);

        ivp_real_t proj_a = ivp_vec3r_dot(&ra_raw, &cp->normal);
        ivp_real_t proj_b = ivp_vec3r_dot(&rb_raw, &cp->normal);

        for (int i = 0; i < 3; i++) {
            cp->point_a.k[i] = ca->pos_world_f_core_last_psi.k[i]
                              + proj_a * cp->normal.k[i];
            cp->point_b.k[i] = cb->pos_world_f_core_last_psi.k[i]
                              + proj_b * cp->normal.k[i];
        }
    }

    finalize_contact_geometry(cp);
}

/* ── Generic contact computation ────────────────────────────────────────── */

void ivp_contact_compute_generic(ivp_contact_point_t *cp)
{
    ivp_object_type_t ta = cp->obj_a->type;
    ivp_object_type_t tb = cp->obj_b->type;

    if (ta == IVP_OBJECT_BALL && tb == IVP_OBJECT_BALL) {
        ivp_contact_compute_sphere_sphere(cp);
    } else if (ta == IVP_OBJECT_BALL && tb == IVP_OBJECT_POLYGON) {
        ivp_contact_compute_sphere_polygon(cp);
    } else if (ta == IVP_OBJECT_POLYGON && tb == IVP_OBJECT_BALL) {
        /* Swap so sphere is always obj_a */
        ivp_object_t *tmp = cp->obj_a;
        cp->obj_a = cp->obj_b;
        cp->obj_b = tmp;
        ivp_contact_compute_sphere_polygon(cp);

        /* Restore original ordering (polygon=obj_a, ball=obj_b). */
        tmp = cp->obj_a;
        cp->obj_a = cp->obj_b;
        cp->obj_b = tmp;

        /* Map points back to restored ordering and recompute derived geometry
         * so r/cross/span stay consistent with the flipped normal. */
        ivp_vec3r_t sphere_point = cp->point_a;
        ivp_vec3r_t poly_point = cp->point_b;
        cp->point_a = poly_point;
        cp->point_b = sphere_point;
        cp->normal.k[0] = -cp->normal.k[0];
        cp->normal.k[1] = -cp->normal.k[1];
        cp->normal.k[2] = -cp->normal.k[2];
        finalize_contact_geometry(cp);
    } else {
        /* polygon-polygon */
        ivp_contact_compute_polygon_polygon(cp);
    }

    ivp_contact_combine_materials(cp);
    ivp_contact_compute_virtual_mass(cp);
}

void ivp_contact_refresh_geometry(ivp_contact_point_t *cp)
{
    if (!cp || !cp->obj_a || !cp->obj_b) return;
    ivp_object_type_t ta = cp->obj_a->type;
    ivp_object_type_t tb = cp->obj_b->type;

    if (ta == IVP_OBJECT_BALL && tb == IVP_OBJECT_BALL) {
        ivp_contact_compute_sphere_sphere(cp);
    } else if (ta == IVP_OBJECT_BALL && tb == IVP_OBJECT_POLYGON) {
        ivp_contact_compute_sphere_polygon(cp);
    } else if (ta == IVP_OBJECT_POLYGON && tb == IVP_OBJECT_BALL) {
        ivp_contact_compute_generic(cp);
        return;
    } else {
        ivp_contact_compute_polygon_polygon(cp);
    }

    ivp_contact_combine_materials(cp);
    ivp_contact_compute_virtual_mass(cp);
}

static bool contact_is_duplicate(const ivp_contact_point_t *a,
                                 const ivp_contact_point_t *b)
{
    const ivp_real_t point_eps = IVP_REAL(1e-3);
    const ivp_real_t gap_eps = IVP_REAL(1e-3);

    ivp_real_t pa_qd = ivp_vec3r_quad_distance(&a->point_a, &b->point_a);
    ivp_real_t pb_qd = ivp_vec3r_quad_distance(&a->point_b, &b->point_b);
    ivp_real_t point_eps_q = point_eps * point_eps;

    if (pa_qd > point_eps_q || pb_qd > point_eps_q) {
        return false;
    }

    if (ivp_fabs(a->gap_length - b->gap_length) > gap_eps) {
        return false;
    }

    ivp_real_t n_dot = ivp_vec3r_dot(&a->normal, &b->normal);
    return n_dot > IVP_REAL(0.995);
}

typedef struct ivp_manifold_candidate {
    ivp_vec3r_t point_a;
    ivp_vec3r_t point_b;
    ivp_real_t gap;
} ivp_manifold_candidate_t;

static bool manifold_candidate_duplicate(const ivp_manifold_candidate_t *a,
                                         const ivp_manifold_candidate_t *b)
{
    const ivp_real_t point_eps = IVP_REAL(2e-3);
    const ivp_real_t gap_eps = IVP_REAL(2e-3);
    const ivp_real_t point_eps_q = point_eps * point_eps;

    if (ivp_vec3r_quad_distance(&a->point_a, &b->point_a) > point_eps_q) {
        return false;
    }
    if (ivp_vec3r_quad_distance(&a->point_b, &b->point_b) > point_eps_q) {
        return false;
    }
    return ivp_fabs(a->gap - b->gap) <= gap_eps;
}

static void init_contact_from_points(ivp_contact_point_t *cp,
                                     ivp_object_t *obj_a,
                                     ivp_object_t *obj_b,
                                     const ivp_vec3r_t *normal,
                                     const ivp_vec3r_t *point_a,
                                     const ivp_vec3r_t *point_b,
                                     ivp_real_t gap)
{
    memset(cp, 0, sizeof(*cp));
    cp->obj_a = obj_a;
    cp->obj_b = obj_b;
    cp->point_a = *point_a;
    cp->point_b = *point_b;
    cp->normal = *normal;

    ivp_real_t n_len = ivp_vec3r_length(&cp->normal);
    if (n_len > IVP_REAL_EPS) {
        ivp_real_t inv_n = IVP_REAL(1.0) / n_len;
        cp->normal.k[0] *= inv_n;
        cp->normal.k[1] *= inv_n;
        cp->normal.k[2] *= inv_n;
    } else {
        cp->normal.k[0] = IVP_REAL(1.0);
        cp->normal.k[1] = IVP_REAL(0.0);
        cp->normal.k[2] = IVP_REAL(0.0);
    }

    cp->gap_length = gap;

    finalize_contact_geometry(cp);
    ivp_contact_combine_materials(cp);
    ivp_contact_compute_virtual_mass(cp);
}

int build_poly_poly_manifold_contacts(ivp_object_t *obj_a,
                                      ivp_object_t *obj_b,
                                      const ivp_contact_point_t *base,
                                      ivp_contact_point_t *contacts_out,
                                      int max_contacts)
{
    if (!obj_a || !obj_b || !base || !contacts_out || max_contacts <= 0) {
        return 0;
    }

    contacts_out[0] = *base;
    int out_count = 1;

    const int manifold_limit = (max_contacts < 4) ? max_contacts : 4;
    if (manifold_limit <= 1 || base->gap_length > IVP_REAL(0.05)) {
        return out_count;
    }

    const int extras_limit = manifold_limit - 1;
    if (extras_limit <= 0) {
        return out_count;
    }

    const ivp_compact_ledge_t *ledge_a =
        obj_a->surman ? ivp_surman_get_single_convex(obj_a->surman) : NULL;
    const ivp_compact_ledge_t *ledge_b =
        obj_b->surman ? ivp_surman_get_single_convex(obj_b->surman) : NULL;
    if (!ledge_a || !ledge_b) {
        /* Compound objects: skip manifold expansion — the hierarchical path
         * already produces multiple contacts from different leaf pairs. */
        return out_count;
    }

    const ivp_core_t *ca = &obj_a->core;
    const ivp_core_t *cb = &obj_b->core;
    const ivp_vec3r_t n = base->normal;

    const ivp_compact_poly_point_t *pts_a = ivp_ledge_get_point_array(ledge_a);
    const ivp_compact_poly_point_t *pts_b = ivp_ledge_get_point_array(ledge_b);
    int n_pts_a = ivp_ledge_get_n_points(ledge_a);
    int n_pts_b = ivp_ledge_get_n_points(ledge_b);
    if (n_pts_a <= 0 || n_pts_b <= 0) {
        return out_count;
    }

    ivp_real_t max_a = -IVP_REAL_MAX;
    ivp_real_t min_b = IVP_REAL_MAX;

    for (int i = 0; i < n_pts_a; i++) {
        ivp_vec3r_t local = ivp_vec3r((ivp_real_t)pts_a[i].k[0],
                                      (ivp_real_t)pts_a[i].k[1],
                                      (ivp_real_t)pts_a[i].k[2]);
        ivp_vec3r_t ws;
        ivp_transform_point(&ws, &ca->m_world_f_core_last_psi, &local);
        ivp_real_t d = ivp_vec3r_dot(&ws, &n);
        if (d > max_a) max_a = d;
    }

    for (int i = 0; i < n_pts_b; i++) {
        ivp_vec3r_t local = ivp_vec3r((ivp_real_t)pts_b[i].k[0],
                                      (ivp_real_t)pts_b[i].k[1],
                                      (ivp_real_t)pts_b[i].k[2]);
        ivp_vec3r_t ws;
        ivp_transform_point(&ws, &cb->m_world_f_core_last_psi, &local);
        ivp_real_t d = ivp_vec3r_dot(&ws, &n);
        if (d < min_b) min_b = d;
    }

    ivp_real_t support_band = (ca->upper_limit_radius + cb->upper_limit_radius) * IVP_REAL(0.01);
    if (support_band < IVP_REAL(1e-4)) {
        support_band = IVP_REAL(1e-4);
    }

    enum { MANIFOLD_CANDIDATE_CAP = 64 };
    ivp_manifold_candidate_t candidates[MANIFOLD_CANDIDATE_CAP];
    int candidate_count = 0;

    for (int i = 0; i < n_pts_a && candidate_count < MANIFOLD_CANDIDATE_CAP; i++) {
        ivp_vec3r_t local = ivp_vec3r((ivp_real_t)pts_a[i].k[0],
                                      (ivp_real_t)pts_a[i].k[1],
                                      (ivp_real_t)pts_a[i].k[2]);
        ivp_vec3r_t pa;
        ivp_transform_point(&pa, &ca->m_world_f_core_last_psi, &local);

        ivp_real_t da = ivp_vec3r_dot(&pa, &n);
        if (max_a - da > support_band) {
            continue;
        }

        ivp_manifold_candidate_t cand;
        cand.point_a = pa;
        cand.gap = min_b - da;
        cand.point_b.k[0] = pa.k[0] + n.k[0] * cand.gap;
        cand.point_b.k[1] = pa.k[1] + n.k[1] * cand.gap;
        cand.point_b.k[2] = pa.k[2] + n.k[2] * cand.gap;

        bool dupe = false;
        for (int c = 0; c < candidate_count; c++) {
            if (manifold_candidate_duplicate(&candidates[c], &cand)) {
                dupe = true;
                break;
            }
        }
        if (!dupe) {
            candidates[candidate_count++] = cand;
        }
    }

    for (int i = 0; i < n_pts_b && candidate_count < MANIFOLD_CANDIDATE_CAP; i++) {
        ivp_vec3r_t local = ivp_vec3r((ivp_real_t)pts_b[i].k[0],
                                      (ivp_real_t)pts_b[i].k[1],
                                      (ivp_real_t)pts_b[i].k[2]);
        ivp_vec3r_t pb;
        ivp_transform_point(&pb, &cb->m_world_f_core_last_psi, &local);

        ivp_real_t db = ivp_vec3r_dot(&pb, &n);
        if (db - min_b > support_band) {
            continue;
        }

        ivp_manifold_candidate_t cand;
        cand.point_b = pb;
        cand.gap = db - max_a;
        cand.point_a.k[0] = pb.k[0] - n.k[0] * cand.gap;
        cand.point_a.k[1] = pb.k[1] - n.k[1] * cand.gap;
        cand.point_a.k[2] = pb.k[2] - n.k[2] * cand.gap;

        bool dupe = false;
        for (int c = 0; c < candidate_count; c++) {
            if (manifold_candidate_duplicate(&candidates[c], &cand)) {
                dupe = true;
                break;
            }
        }
        if (!dupe) {
            candidates[candidate_count++] = cand;
        }
    }

    if (candidate_count == 0) {
        return out_count;
    }

    ivp_vec3r_t t0, t1;
    compute_tangent_basis(&n, &t0, &t1);

    int selected[4] = {-1, -1, -1, -1};
    bool used[MANIFOLD_CANDIDATE_CAP];
    memset(used, 0, sizeof(used));

    int seed = 0;
    for (int i = 1; i < candidate_count; i++) {
        if (candidates[i].gap < candidates[seed].gap) {
            seed = i;
        }
    }

    selected[0] = seed;
    used[seed] = true;
    int selected_count = 1;

    while (selected_count < extras_limit) {
        int best_i = -1;
        ivp_real_t best_score = -IVP_REAL_MAX;

        for (int i = 0; i < candidate_count; i++) {
            if (used[i]) continue;

            ivp_real_t ta_i = ivp_vec3r_dot(&candidates[i].point_a, &t0);
            ivp_real_t tb_i = ivp_vec3r_dot(&candidates[i].point_a, &t1);

            ivp_real_t min_dist2 = IVP_REAL_MAX;
            for (int s = 0; s < selected_count; s++) {
                int si = selected[s];
                ivp_real_t ta_s = ivp_vec3r_dot(&candidates[si].point_a, &t0);
                ivp_real_t tb_s = ivp_vec3r_dot(&candidates[si].point_a, &t1);

                ivp_real_t dta = ta_i - ta_s;
                ivp_real_t dtb = tb_i - tb_s;
                ivp_real_t d2 = dta * dta + dtb * dtb;
                if (d2 < min_dist2) {
                    min_dist2 = d2;
                }
            }

            ivp_real_t depth_bias = -candidates[i].gap * IVP_REAL(0.05);
            ivp_real_t score = min_dist2 + depth_bias;
            if (score > best_score) {
                best_score = score;
                best_i = i;
            }
        }

        if (best_i < 0 || best_score <= IVP_REAL(1e-8)) {
            break;
        }

        selected[selected_count++] = best_i;
        used[best_i] = true;
    }

    out_count = 1;
    for (int i = 0; i < selected_count && out_count < manifold_limit; i++) {
        ivp_manifold_candidate_t *cand = &candidates[selected[i]];

        ivp_contact_point_t cp;
        init_contact_from_points(&cp,
                                 obj_a,
                                 obj_b,
                                 &n,
                                 &cand->point_a,
                                 &cand->point_b,
                                 cand->gap);

        if (cp.gap_length > IVP_REAL(0.02)) {
            continue;
        }

        bool dupe = false;
        for (int j = 0; j < out_count; j++) {
            if (contact_is_duplicate(&contacts_out[j], &cp)) {
                dupe = true;
                break;
            }
        }

        if (!dupe) {
            contacts_out[out_count++] = cp;
        }
    }

    return out_count;
}

int ivp_contact_compute_generic_multi(ivp_object_t *obj_a,
                                      ivp_object_t *obj_b,
                                      ivp_contact_point_t *contacts_out,
                                      int max_contacts)
{
    if (!obj_a || !obj_b || !contacts_out || max_contacts <= 0) {
        return 0;
    }

    /* Check if either polygon object has a compound (non-terminal) ledgetree.
     * If so, use hierarchical traversal instead of single-convex path. */
    if (obj_a->type == IVP_OBJECT_POLYGON &&
        obj_b->type == IVP_OBJECT_POLYGON) {
        bool has_compound_a = (obj_a->surman &&
                               ivp_surman_get_ledgetree_root(obj_a->surman) != NULL);
        bool has_compound_b = (obj_b->surman &&
                               ivp_surman_get_ledgetree_root(obj_b->surman) != NULL);
        if (has_compound_a || has_compound_b) {
            return ivp_contact_compute_hierarchical_multi(obj_a, obj_b,
                                                          contacts_out,
                                                          max_contacts);
        }
    }

    memset(&contacts_out[0], 0, sizeof(contacts_out[0]));
    contacts_out[0].obj_a = obj_a;
    contacts_out[0].obj_b = obj_b;
    ivp_contact_compute_generic(&contacts_out[0]);

    if (obj_a->type == IVP_OBJECT_POLYGON &&
        obj_b->type == IVP_OBJECT_POLYGON &&
        max_contacts > 1) {
        return 1;
    }

    return 1;
}

/* ── Virtual mass ───────────────────────────────────────────────────────── */

ivp_real_t ivp_calc_virtual_mass(const ivp_core_t *core,
                                  const ivp_vec3r_t *r_world,
                                  const ivp_vec3r_t *normal)
{
    if (core->physical_unmoveable) {
        return IVP_REAL(1e30); /* effectively infinite */
    }

    /* Transform r and normal to core space */
    ivp_vec3r_t r_cs, n_cs;
    ivp_mat3r_vimult(&r_cs, &core->m_world_f_core_last_psi.rot, r_world);
    ivp_mat3r_vimult(&n_cs, &core->m_world_f_core_last_psi.rot, normal);

    /* cross = r_cs × n_cs */
    ivp_vec3r_t cross_rn;
    ivp_vec3r_cross(&cross_rn, &r_cs, &n_cs);

    /* angular_term = (inv_I componentwise * cross) · cross */
    ivp_real_t angular_term =
        cross_rn.k[0] * cross_rn.k[0] * core->inv_rot_inertia.k[0] +
        cross_rn.k[1] * cross_rn.k[1] * core->inv_rot_inertia.k[1] +
        cross_rn.k[2] * cross_rn.k[2] * core->inv_rot_inertia.k[2];

    /* virt_mass = 1 / (inv_mass + angular_term) */
    ivp_real_t inv_virt_mass = core->inv_mass + angular_term;
    if (inv_virt_mass < IVP_REAL_EPS) {
        return IVP_REAL(1e30);
    }
    return IVP_REAL(1.0) / inv_virt_mass;
}

void ivp_contact_compute_virtual_mass(ivp_contact_point_t *cp)
{
    const ivp_core_t *ca = &cp->obj_a->core;
    const ivp_core_t *cb = &cp->obj_b->core;

    ivp_real_t vm_a = ivp_calc_virtual_mass(ca, &cp->r_a, &cp->normal);
    ivp_real_t vm_b = ivp_calc_virtual_mass(cb, &cp->r_b, &cp->normal);

    if (ca->physical_unmoveable) {
        cp->virtual_mass = vm_b;
    } else if (cb->physical_unmoveable) {
        cp->virtual_mass = vm_a;
    } else {
        /* Reduced mass: 1/vm = 1/vm_a + 1/vm_b → vm = (vm_a * vm_b) / (vm_a + vm_b) */
        cp->virtual_mass = (vm_a * vm_b) / (vm_a + vm_b);
    }

    if (cp->virtual_mass > IVP_REAL_EPS) {
        cp->inv_virtual_mass = IVP_REAL(1.0) / cp->virtual_mass;
    } else {
        cp->inv_virtual_mass = IVP_REAL(0.0);
    }
}

/* ── Velocities at contact ──────────────────────────────────────────────── */

void ivp_contact_relative_velocity(const ivp_contact_point_t *cp,
                                    ivp_vec3r_t *v_rel_out)
{
    const ivp_core_t *ca = &cp->obj_a->core;
    const ivp_core_t *cb = &cp->obj_b->core;

    /* v_at_contact_A = v_a + ω_a_world × r_a */
    ivp_vec3r_t omega_a_rs;
    omega_a_rs.k[0] = (ivp_real_t)ca->rot_speed.k[0];
    omega_a_rs.k[1] = (ivp_real_t)ca->rot_speed.k[1];
    omega_a_rs.k[2] = (ivp_real_t)ca->rot_speed.k[2];
    ivp_vec3r_t omega_a_world, wa_cross_ra;
    ivp_mat3r_vmult(&omega_a_world, &ca->m_world_f_core_last_psi.rot, &omega_a_rs);
    ivp_vec3r_cross(&wa_cross_ra, &omega_a_world, &cp->r_a);

    ivp_vec3r_t v_contact_a;
    ivp_vec3r_add(&v_contact_a, &ca->speed, &wa_cross_ra);

    /* v_at_contact_B = v_b + ω_b_world × r_b */
    ivp_vec3r_t omega_b_rs;
    omega_b_rs.k[0] = (ivp_real_t)cb->rot_speed.k[0];
    omega_b_rs.k[1] = (ivp_real_t)cb->rot_speed.k[1];
    omega_b_rs.k[2] = (ivp_real_t)cb->rot_speed.k[2];
    ivp_vec3r_t omega_b_world, wb_cross_rb;
    ivp_mat3r_vmult(&omega_b_world, &cb->m_world_f_core_last_psi.rot, &omega_b_rs);
    ivp_vec3r_cross(&wb_cross_rb, &omega_b_world, &cp->r_b);

    ivp_vec3r_t v_contact_b;
    ivp_vec3r_add(&v_contact_b, &cb->speed, &wb_cross_rb);

    /* v_rel = v_b - v_a (relative velocity of B w.r.t. A at contact) */
    ivp_vec3r_sub(v_rel_out, &v_contact_b, &v_contact_a);
}

ivp_real_t ivp_contact_closing_speed(const ivp_contact_point_t *cp)
{
    ivp_vec3r_t v_rel;
    ivp_contact_relative_velocity(cp, &v_rel);
    /* Closing speed = -v_rel · normal
     * (positive when approaching: v_rel points from B to A along normal) */
    return -ivp_vec3r_dot(&v_rel, &cp->normal);
}

/* ── Impulse application ────────────────────────────────────────────────── */

void ivp_contact_apply_impulse(ivp_contact_point_t *cp, ivp_real_t impulse)
{
    ivp_contact_apply_impulse_dir(cp, &cp->normal, impulse);
}

void ivp_contact_apply_impulse_dir(ivp_contact_point_t *cp,
                                    const ivp_vec3r_t *direction,
                                    ivp_real_t impulse)
{
    ivp_core_t *ca = &cp->obj_a->core;
    ivp_core_t *cb = &cp->obj_b->core;

    /* Apply to object A: push AWAY from B (negative direction) */
    if (!ca->physical_unmoveable) {
        /* Linear: v_a -= impulse * inv_mass * direction */
        ivp_real_t scale_a = impulse * ca->inv_mass;
        ca->speed.k[0] -= direction->k[0] * scale_a;
        ca->speed.k[1] -= direction->k[1] * scale_a;
        ca->speed.k[2] -= direction->k[2] * scale_a;

        /* Angular: ω_a -= inv_I * (r_a × (impulse * direction)) [core space] */
        /* Transform direction and r_a to core space */
        ivp_vec3r_t dir_cs, r_a_cs, cross_r_dir;
        ivp_mat3r_vimult(&dir_cs, &ca->m_world_f_core_last_psi.rot, direction);
        ivp_mat3r_vimult(&r_a_cs, &ca->m_world_f_core_last_psi.rot, &cp->r_a);
        ivp_vec3r_cross(&cross_r_dir, &r_a_cs, &dir_cs);

        ca->rot_speed.k[0] -= (float)(cross_r_dir.k[0] * ca->inv_rot_inertia.k[0] * impulse);
        ca->rot_speed.k[1] -= (float)(cross_r_dir.k[1] * ca->inv_rot_inertia.k[1] * impulse);
        ca->rot_speed.k[2] -= (float)(cross_r_dir.k[2] * ca->inv_rot_inertia.k[2] * impulse);
    }

    /* Apply to object B: push TOWARD A (positive direction) */
    if (!cb->physical_unmoveable) {
        /* Linear: v_b += impulse * inv_mass * direction */
        ivp_real_t scale_b = impulse * cb->inv_mass;
        cb->speed.k[0] += direction->k[0] * scale_b;
        cb->speed.k[1] += direction->k[1] * scale_b;
        cb->speed.k[2] += direction->k[2] * scale_b;

        /* Angular: ω_b += inv_I * (r_b × (impulse * direction)) [core space] */
        ivp_vec3r_t dir_cs, r_b_cs, cross_r_dir;
        ivp_mat3r_vimult(&dir_cs, &cb->m_world_f_core_last_psi.rot, direction);
        ivp_mat3r_vimult(&r_b_cs, &cb->m_world_f_core_last_psi.rot, &cp->r_b);
        ivp_vec3r_cross(&cross_r_dir, &r_b_cs, &dir_cs);

        cb->rot_speed.k[0] += (float)(cross_r_dir.k[0] * cb->inv_rot_inertia.k[0] * impulse);
        cb->rot_speed.k[1] += (float)(cross_r_dir.k[1] * cb->inv_rot_inertia.k[1] * impulse);
        cb->rot_speed.k[2] += (float)(cross_r_dir.k[2] * cb->inv_rot_inertia.k[2] * impulse);
    }
}
