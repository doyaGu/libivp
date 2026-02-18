/* ivp_ccd_narrowphase.c — Continuous collision detection narrowphase */

#include <ivp/collision/ivp_ccd_narrowphase.h>
#include <ivp/collision/ivp_mindist.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/solver/ivp_impact.h>
#include <ivp/base/ivp_allocator.h>
#include <string.h>
#include <math.h>

/* ── Distance function ──────────────────────────────────────────────────── */

ivp_real_t ivp_3d_dist_convex_convex(const ivp_transform_r_t *m_a,
                                       const ivp_transform_r_t *m_b,
                                       void *user_data)
{
    ivp_ccd_convex_pair_t *pair = (ivp_ccd_convex_pair_t *)user_data;
    ivp_object_t *obj_a = pair->obj_a;
    ivp_object_t *obj_b = pair->obj_b;

    bool is_ball_a = (obj_a->type == IVP_OBJECT_BALL);
    bool is_ball_b = (obj_b->type == IVP_OBJECT_BALL);

    /* Sphere-sphere: ||pos_a - pos_b|| - r_a - r_b */
    if (is_ball_a && is_ball_b) {
        ivp_vec3r_t diff;
        ivp_vec3r_sub(&diff, &m_b->pos, &m_a->pos);
        ivp_real_t dist = ivp_vec3r_length(&diff);
        return dist - obj_a->core.upper_limit_radius
                     - obj_b->core.upper_limit_radius;
    }

    /* Sphere-poly: closest point on poly to sphere center minus radius */
    if (is_ball_a && !is_ball_b) {
        const ivp_compact_ledge_t *ledge_b =
            obj_b->surman ? ivp_surman_get_single_convex(obj_b->surman) : NULL;
        if (!ledge_b) {
            /* Fallback: sphere-sphere approximation */
            ivp_vec3r_t diff;
            ivp_vec3r_sub(&diff, &m_b->pos, &m_a->pos);
            return ivp_vec3r_length(&diff)
                   - obj_a->core.upper_limit_radius
                   - obj_b->core.upper_limit_radius;
        }

        /* Transform sphere center to B's object space using m_b inverse */
        ivp_vec3r_t center_in_b;
        ivp_transform_inv_point(&center_in_b, m_b, &m_a->pos);

        /* Walk triangles to find closest point */
        ivp_real_t best_qdist = IVP_REAL_MAX;
        int n_tri = ivp_ledge_get_n_triangles(ledge_b);
        const ivp_compact_triangle_t *tri0 = ivp_ledge_get_first_triangle(ledge_b);

        for (int t = 0; t < n_tri; t++) {
            const ivp_compact_triangle_t *tri = &tri0[t];
            ivp_vec3r_t v[3];
            for (int e = 0; e < 3; e++) {
                const ivp_compact_edge_t *edge = ivp_tri_get_edge(tri, e);
                const ivp_compact_poly_point_t *pp =
                    ivp_edge_get_start_point(edge, ledge_b);
                v[e] = ivp_vec3r((ivp_real_t)pp->k[0],
                                  (ivp_real_t)pp->k[1],
                                  (ivp_real_t)pp->k[2]);
            }

            /* Barycentric closest point on triangle */
            ivp_vec3r_t ab, ac, ap;
            ivp_vec3r_sub(&ab, &v[1], &v[0]);
            ivp_vec3r_sub(&ac, &v[2], &v[0]);
            ivp_vec3r_sub(&ap, &center_in_b, &v[0]);

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
            if (u >= IVP_REAL(0.0) && w >= IVP_REAL(0.0) &&
                u + w <= IVP_REAL(1.0)) {
                closest.k[0] = v[0].k[0] + ab.k[0] * u + ac.k[0] * w;
                closest.k[1] = v[0].k[1] + ab.k[1] * u + ac.k[1] * w;
                closest.k[2] = v[0].k[2] + ab.k[2] * u + ac.k[2] * w;
            } else {
                /* Clamp to edges */
                ivp_real_t bq = IVP_REAL_MAX;
                closest = v[0];
                for (int i = 0; i < 3; i++) {
                    int j = (i + 1) % 3;
                    ivp_vec3r_t ev, tp;
                    ivp_vec3r_sub(&ev, &v[j], &v[i]);
                    ivp_vec3r_sub(&tp, &center_in_b, &v[i]);
                    ivp_real_t el = ivp_vec3r_dot(&ev, &ev);
                    if (el < IVP_REAL_EPS) continue;
                    ivp_real_t s = ivp_vec3r_dot(&tp, &ev) / el;
                    if (s < IVP_REAL(0.0)) s = IVP_REAL(0.0);
                    if (s > IVP_REAL(1.0)) s = IVP_REAL(1.0);
                    ivp_vec3r_t cand;
                    cand.k[0] = v[i].k[0] + ev.k[0] * s;
                    cand.k[1] = v[i].k[1] + ev.k[1] * s;
                    cand.k[2] = v[i].k[2] + ev.k[2] * s;
                    ivp_real_t q = ivp_vec3r_quad_distance(&center_in_b, &cand);
                    if (q < bq) { bq = q; closest = cand; }
                }
            }

            ivp_real_t q = ivp_vec3r_quad_distance(&center_in_b, &closest);
            if (q < best_qdist) best_qdist = q;
        }

        if (best_qdist < IVP_REAL_MAX) {
            return ivp_sqrt(best_qdist) - obj_a->core.upper_limit_radius;
        }
        /* Fallback */
        ivp_vec3r_t diff;
        ivp_vec3r_sub(&diff, &m_b->pos, &m_a->pos);
        return ivp_vec3r_length(&diff)
               - obj_a->core.upper_limit_radius
               - obj_b->core.upper_limit_radius;
    }

    /* Poly-sphere: swap and use sphere-poly logic */
    if (!is_ball_a && is_ball_b) {
        /* Swap objects temporarily in pair */
        ivp_ccd_convex_pair_t swapped;
        swapped.obj_a = obj_b;
        swapped.obj_b = obj_a;
        return ivp_3d_dist_convex_convex(m_b, m_a, &swapped);
    }

    /* Poly-poly: transform B into A's space, run closest pair */
    {
        const ivp_compact_ledge_t *ledge_a =
            obj_a->surman ? ivp_surman_get_single_convex(obj_a->surman) : NULL;
        const ivp_compact_ledge_t *ledge_b =
            obj_b->surman ? ivp_surman_get_single_convex(obj_b->surman) : NULL;

        if (!ledge_a || !ledge_b) {
            ivp_vec3r_t diff;
            ivp_vec3r_sub(&diff, &m_b->pos, &m_a->pos);
            return ivp_vec3r_length(&diff)
                   - obj_a->core.upper_limit_radius
                   - obj_b->core.upper_limit_radius;
        }

        /* Compute B-in-A transform */
        ivp_transform_r_t b_in_a;
        ivp_transform_inv_compose(&b_in_a, m_a, m_b);

        /* Copy and transform B ledge into A's space (stack buffer) */
        int ledge_b_size = ivp_ledge_get_size(ledge_b);
        int n_pts = ivp_ledge_get_n_points(ledge_b);
        int n_tri = ivp_ledge_get_n_triangles(ledge_b);

        if (ledge_b_size < (int)sizeof(ivp_compact_ledge_t) ||
            (ledge_b_size & 0xF) != 0 || n_tri <= 0 || n_pts <= 0) {
            ivp_vec3r_t diff;
            ivp_vec3r_sub(&diff, &m_b->pos, &m_a->pos);
            return ivp_vec3r_length(&diff)
                   - obj_a->core.upper_limit_radius
                   - obj_b->core.upper_limit_radius;
        }

        /* Stack buffer for small ledges */
        _Alignas(16) uint8_t stack_buf[4096];
        uint8_t *buf = (ledge_b_size <= (int)sizeof(stack_buf))
                     ? stack_buf : NULL;
        if (!buf) {
            /* Too large for stack, fall back to center distance */
            ivp_vec3r_t diff;
            ivp_vec3r_sub(&diff, &m_b->pos, &m_a->pos);
            return ivp_vec3r_length(&diff)
                   - obj_a->core.upper_limit_radius
                   - obj_b->core.upper_limit_radius;
        }

        memcpy(buf, ledge_b, (size_t)ledge_b_size);
        ivp_compact_ledge_t *ledge_b_tx = (ivp_compact_ledge_t *)buf;

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

        ivp_closest_pair_t result;
        ivp_result_t res = ivp_find_closest_pair(ledge_a, ledge_b_tx, &result);

        if (res == IVP_OK && result.status == IVP_MINDIST_STATUS_OK) {
            return ivp_sqrt(result.qdistance);
        }

        /* Fallback */
        ivp_vec3r_t diff;
        ivp_vec3r_sub(&diff, &m_b->pos, &m_a->pos);
        return ivp_vec3r_length(&diff)
               - obj_a->core.upper_limit_radius
               - obj_b->core.upper_limit_radius;
    }
}

/* ── Sweep pair query ───────────────────────────────────────────────────── */

bool ivp_ccd_pair_needs_sweep(const ivp_object_t *obj_a,
                               const ivp_object_t *obj_b,
                               ivp_real_t dt)
{
    if (!obj_a || !obj_b) return false;
    if (dt <= IVP_REAL_EPS) return false;

    /* At least one object must have CCD enabled */
    if (!obj_a->ccd_enabled && !obj_b->ccd_enabled) return false;

    /* Check speed threshold for each CCD-enabled object */
    for (int which = 0; which < 2; which++) {
        const ivp_object_t *obj = (which == 0) ? obj_a : obj_b;
        if (!obj->ccd_enabled) continue;

        ivp_real_t speed = ivp_vec3r_length(&obj->core.speed);
        ivp_real_t threshold = obj->ccd_speed_threshold;

        if (threshold <= IVP_REAL(0.0)) {
            /* Auto threshold: 50% of radius / dt */
            threshold = IVP_REAL(0.5) * obj->core.upper_limit_radius / dt;
        }

        if (speed * dt > IVP_REAL(0.5) * obj->core.upper_limit_radius) {
            return true;
        }
    }

    return false;
}

/* ── CCD sweep pair ─────────────────────────────────────────────────────── */

bool ivp_ccd_sweep_pair(ivp_object_t *obj_a,
                          ivp_object_t *obj_b,
                          ivp_time_t base_time,
                          ivp_real_t dt,
                          ivp_ccd_hit_t *hit_out)
{
    if (!obj_a || !obj_b || !hit_out || dt <= IVP_REAL_EPS) {
        return false;
    }

    hit_out->valid = false;

    /* Initialize matrix caches for both objects */
    ivp_matrix_cache_t cache_a;
    ivp_matrix_cache_t cache_b;
    ivp_matrix_cache_init(&cache_a, obj_a, base_time, (ivp_time_t)dt);
    ivp_matrix_cache_init(&cache_b, obj_b, base_time, (ivp_time_t)dt);

    /* Set up distance function context */
    ivp_ccd_convex_pair_t pair_ctx;
    pair_ctx.obj_a = obj_a;
    pair_ctx.obj_b = obj_b;

    /* Combined surface deviation for threshold */
    ivp_real_t max_dev = obj_a->core.max_surface_deviation
                       + obj_b->core.max_surface_deviation;
    if (max_dev < IVP_REAL(0.001)) max_dev = IVP_REAL(0.001);

    /* Initialize 3D solver */
    ivp_3d_solver_t solver;
    ivp_3d_solver_init(&solver, ivp_3d_dist_convex_convex, &pair_ctx, max_dev);

    /* Find first time where distance crosses zero (contact threshold) */
    ivp_time_t toi;
    ivp_real_t threshold = IVP_REAL(0.0);
    bool found = ivp_3d_solver_find_first_t(&solver, threshold,
                                              base_time,
                                              base_time + (ivp_time_t)dt,
                                              &cache_a, &cache_b,
                                              &toi);

    if (!found) return false;

    /* Compute contact at TOI using transforms from matrix cache */
    const ivp_transform_r_t *m_a_toi = ivp_matrix_cache_get_at_time(&cache_a, toi);
    const ivp_transform_r_t *m_b_toi = ivp_matrix_cache_get_at_time(&cache_b, toi);

    /* Build a contact point using the TOI transforms as if they were
     * the current PSI transforms. We use the "speculative contacts" approach:
     * compute geometry at TOI but apply impulses to current velocities. */
    ivp_contact_point_t *cp = &hit_out->contact;
    memset(cp, 0, sizeof(*cp));
    cp->obj_a = obj_a;
    cp->obj_b = obj_b;

    /* Use the same dispatch as the discrete narrowphase but with TOI positions.
     * For simplicity, compute a sphere-sphere-like contact using TOI positions
     * and the actual surface radii. */
    const ivp_vec3r_t *pos_a = &m_a_toi->pos;
    const ivp_vec3r_t *pos_b = &m_b_toi->pos;

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

    ivp_real_t ra = obj_a->core.upper_limit_radius;
    ivp_real_t rb = obj_b->core.upper_limit_radius;

    /* Contact points on surfaces along normal */
    cp->point_a.k[0] = pos_a->k[0] + cp->normal.k[0] * ra;
    cp->point_a.k[1] = pos_a->k[1] + cp->normal.k[1] * ra;
    cp->point_a.k[2] = pos_a->k[2] + cp->normal.k[2] * ra;

    cp->point_b.k[0] = pos_b->k[0] - cp->normal.k[0] * rb;
    cp->point_b.k[1] = pos_b->k[1] - cp->normal.k[1] * rb;
    cp->point_b.k[2] = pos_b->k[2] - cp->normal.k[2] * rb;

    cp->gap_length = dist - (ra + rb);

    /* Compute r vectors using CURRENT positions (for impulse application) */
    const ivp_core_t *ca = &obj_a->core;
    const ivp_core_t *cb = &obj_b->core;

    ivp_vec3r_sub(&cp->r_a, &cp->point_a, &ca->pos_world_f_core_last_psi);
    ivp_vec3r_sub(&cp->r_b, &cp->point_b, &cb->pos_world_f_core_last_psi);

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
    {
        ivp_vec3r_t ref;
        if (ivp_fabs(cp->normal.k[0]) < IVP_REAL(0.9)) {
            ref.k[0] = IVP_REAL(1.0);
            ref.k[1] = IVP_REAL(0.0);
            ref.k[2] = IVP_REAL(0.0);
        } else {
            ref.k[0] = IVP_REAL(0.0);
            ref.k[1] = IVP_REAL(1.0);
            ref.k[2] = IVP_REAL(0.0);
        }
        ivp_vec3r_cross(&cp->span_dirs[0], &ref, &cp->normal);
        ivp_real_t len = ivp_vec3r_length(&cp->span_dirs[0]);
        if (len > IVP_REAL_EPS) {
            ivp_real_t inv_len = IVP_REAL(1.0) / len;
            cp->span_dirs[0].k[0] *= inv_len;
            cp->span_dirs[0].k[1] *= inv_len;
            cp->span_dirs[0].k[2] *= inv_len;
        }
        ivp_vec3r_cross(&cp->span_dirs[1], &cp->normal, &cp->span_dirs[0]);
    }

    ivp_contact_combine_materials(cp);
    ivp_contact_compute_virtual_mass(cp);

    hit_out->toi = toi;
    hit_out->valid = true;
    return true;
}
