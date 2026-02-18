/* ivp_compact_ledge_solver.c — Compact ledge solver utilities
 *
 * Higher-level collision functions operating on compact ledges:
 *   - Triangle hesse plane computation (normalized)
 *   - Bounding radius and deviation to given center
 *   - Point-to-ledge minimum squared distance
 *
 * Ported from ivp_compact_ledge_solver.cxx.
 */
#include <ivp/collision/ivp_compact_ledge_solver.h>
#include <ivp/collision/ivp_narrowphase.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/math/ivp_fast_math.h>

/* ── Helper: get vertex position as ivp_vec3r_t ────────────────────────── */

static inline ivp_vec3r_t cls_point_to_vec3r(const ivp_compact_poly_point_t *p)
{
    return ivp_vec3r((ivp_real_t)p->k[0],
                     (ivp_real_t)p->k[1],
                     (ivp_real_t)p->k[2]);
}

/* ── Hesse plane (normalized) ──────────────────────────────────────────── */

void ivp_cls_calc_hesse_normalized(const ivp_compact_ledge_t *ledge,
                                    const ivp_compact_edge_t *tri_edge,
                                    ivp_vec3r_t *normal_out,
                                    ivp_real_t *dist_out)
{
    IVP_ASSERT(ledge != NULL);
    IVP_ASSERT(tri_edge != NULL);
    IVP_ASSERT(normal_out != NULL);
    IVP_ASSERT(dist_out != NULL);

    /* Get unnormalized hesse vector (cross product of two edge vectors). */
    ivp_vec3r_t hesse;
    ivp_calc_hesse_vec(ledge, tri_edge, &hesse);

    /* Normalize. */
    ivp_real_t len_sq = ivp_vec3r_quad_length(&hesse);
    if (len_sq < IVP_REAL_RES * IVP_REAL_RES) {
        /* Degenerate triangle — return zero normal and zero distance. */
        ivp_vec3r_set_zero(normal_out);
        *dist_out = IVP_REAL(0.0);
        return;
    }

    ivp_real_t inv_len = IVP_REAL(1.0) / ivp_sqrt(len_sq);
    ivp_vec3r_scale(normal_out, &hesse, inv_len);

    /* Signed distance from origin to the triangle plane:
     * dist = dot(normal, vertex) */
    const ivp_compact_triangle_t *tri = ivp_edge_get_triangle(tri_edge);
    ivp_vec3r_t p0 = cls_point_to_vec3r(
        ivp_edge_get_start_point(&tri->edges[0], ledge));
    *dist_out = ivp_vec3r_dot(normal_out, &p0);
}

/* ── Radius and deviation to center ────────────────────────────────────── */

void ivp_cls_calc_radius_to_center(const ivp_compact_ledge_t *ledge,
                                    const ivp_vec3r_t *center,
                                    ivp_real_t *radius_inout,
                                    ivp_real_t *deviation_out)
{
    IVP_ASSERT(ledge != NULL);
    IVP_ASSERT(center != NULL);
    IVP_ASSERT(radius_inout != NULL);

    ivp_real_t q_max = *radius_inout * *radius_inout;
    ivp_real_t dev_max_sq = (deviation_out != NULL)
                          ? (*deviation_out * *deviation_out)
                          : IVP_REAL(0.0);

    int n_tri = ivp_ledge_get_n_triangles(ledge);
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);

    for (int t = 0; t < n_tri; t++) {
        /* Unnormalized face normal via cross product of two edges. */
        const ivp_compact_edge_t *e = &tri->edges[0];
        ivp_vec3r_t hesse;
        ivp_calc_hesse_vec(ledge, e, &hesse);
        ivp_real_t hesse_len_sq = ivp_vec3r_quad_length(&hesse);
        bool has_valid_hesse = hesse_len_sq > (IVP_REAL_RES * IVP_REAL_RES);
        ivp_real_t i_hv_len_sq = has_valid_hesse
            ? (IVP_REAL(1.0) / hesse_len_sq)
            : IVP_REAL(0.0);

        /* Check all three vertices of this triangle. */
        for (int j = 0; j < 3; j++) {
            ivp_vec3r_t p = cls_point_to_vec3r(
                ivp_edge_get_start_point(&tri->edges[j], ledge));
            ivp_vec3r_t diff;
            ivp_vec3r_sub(&diff, &p, center);

            /* Max radius: squared distance from center to vertex. */
            ivp_real_t q = ivp_vec3r_quad_length(&diff);
            if (q > q_max) q_max = q;

            /* Max deviation: distance from vertex to nearest point on
             * the edge formed by (center, normal).
             * deviation = |diff × hesse| / |hesse|  */
            if (deviation_out != NULL && has_valid_hesse) {
                ivp_vec3r_t cp;
                ivp_vec3r_cross(&cp, &diff, &hesse);
                ivp_real_t dev = ivp_vec3r_quad_length(&cp) * i_hv_len_sq;
                if (dev > dev_max_sq) dev_max_sq = dev;
            }
        }

        /* Advance to next triangle (triangles are contiguous 16-byte structs). */
        tri++;
    }

    /* Output: take square roots and max-accumulate. */
    ivp_real_t r = ivp_sqrt(q_max);
    if (r > *radius_inout) *radius_inout = r;

    if (deviation_out != NULL) {
        ivp_real_t d = ivp_sqrt(dev_max_sq);
        if (d > *deviation_out) *deviation_out = d;
    }
}

/* ── Radius for full surface ───────────────────────────────────────────── */

/* Visitor callback for traversing all ledges. */
typedef struct {
    const ivp_vec3r_t *center;
    ivp_real_t *radius;
    ivp_real_t *deviation;
} radius_visitor_ctx_t;

static bool radius_visitor(const ivp_compact_ledge_t *ledge, void *user_data)
{
    radius_visitor_ctx_t *ctx = (radius_visitor_ctx_t *)user_data;
    ivp_cls_calc_radius_to_center(ledge, ctx->center,
                                   ctx->radius, ctx->deviation);
    return true; /* continue traversal */
}

void ivp_cls_calc_radius_for_surface(const ivp_compact_surface_t *surface,
                                      const ivp_vec3r_t *center,
                                      ivp_real_t *radius_out,
                                      ivp_real_t *deviation_out)
{
    IVP_ASSERT(surface != NULL);
    IVP_ASSERT(center != NULL);
    IVP_ASSERT(radius_out != NULL);

    *radius_out = IVP_REAL(0.0);
    if (deviation_out != NULL) *deviation_out = IVP_REAL(0.0);

    const ivp_compact_ledgetree_node_t *root =
        ivp_surface_get_ledgetree_root(surface);

    radius_visitor_ctx_t ctx;
    ctx.center = center;
    ctx.radius = radius_out;
    ctx.deviation = deviation_out;

    ivp_ledgetree_visit_all(root, radius_visitor, &ctx);
}

/* ── Point-to-ledge squared distance ───────────────────────────────────── */

ivp_real_t ivp_cls_qdist_point_to_ledge(const ivp_compact_ledge_t *ledge,
                                         const ivp_vec3r_t *point_os)
{
    IVP_ASSERT(ledge != NULL);
    IVP_ASSERT(point_os != NULL);

    ivp_real_t min_dist = IVP_REAL_MAX;

    int n_tri = ivp_ledge_get_n_triangles(ledge);
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);

    for (int t = 0; t < n_tri; t++) {
        /* Use ivp_qdist_pf from narrowphase to get point-to-triangle distance. */
        ivp_real_t d = ivp_qdist_pf(ledge, &tri->edges[0], point_os);
        if (d < min_dist) min_dist = d;
        tri++;
    }

    return min_dist;
}
