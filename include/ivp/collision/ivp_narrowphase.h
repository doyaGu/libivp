/* ivp_narrowphase.h — Narrowphase distance primitives
 *
 * Pure geometric distance queries between compact ledge features:
 *   - Point-point (PP)
 *   - Point-edge (PK)
 *   - Point-triangle (PF)
 *   - Edge-edge (KK)
 *   - Sphere-sphere (BB)
 *
 * Naming convention follows original IVP German abbreviations:
 *   P = Punkt (point/vertex)
 *   K = Kante (edge)
 *   F = Fläche (face/triangle)
 *   B = Ball (sphere)
 *
 * All functions work in object space with compact ledge data structures.
 */
#ifndef IVP_NARROWPHASE_H
#define IVP_NARROWPHASE_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/geometry/ivp_compact_ledge.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Barycentric result structures ──────────────────────────────────────── */

/* Point-in-triangle barycentric result (unscaled QR values).
 * checks[j] >= 0 for all j means point projects inside the triangle.
 * Uses sign-bit OR trick for fast is_outside() test. */
typedef struct ivp_qr_result {
    ivp_real_t checks[3];   /* barycentric checks per edge */
    ivp_real_t scale;       /* determinant for normalization */
} ivp_qr_result_t;

/* Point-on-edge parametric result (unscaled S values).
 * Both checks[0] >= 0 && checks[1] >= 0 means point projects onto edge. */
typedef struct ivp_s_result {
    ivp_real_t checks[2];   /* parametric checks for start/end */
} ivp_s_result_t;

/* Edge-edge closest-point result (unscaled KK values).
 * checks_k[0..1] for edge K, checks_l[0..1] for edge L. */
typedef struct ivp_kk_result {
    ivp_real_t checks_k[2]; /* parametric position on edge K */
    ivp_real_t checks_l[2]; /* parametric position on edge L */
} ivp_kk_result_t;

/* ── Fast outside test ──────────────────────────────────────────────────── */

/* Check if point lies outside triangle via sign-bit OR trick. */
static inline bool ivp_qr_is_outside(const ivp_qr_result_t *qr) {
    /* Union for float/double sign bit extraction */
    union {
        ivp_real_t f;
        uint32_t   u32;  /* float mode */
#ifdef IVP_REAL_DOUBLE
        uint64_t   u64;  /* double mode */
#endif
    } a, b, c;
    a.f = qr->checks[0];
    b.f = qr->checks[1];
    c.f = qr->checks[2];
#ifdef IVP_REAL_DOUBLE
    return (a.u64 | b.u64 | c.u64) >> 63;
#else
    return (a.u32 | b.u32 | c.u32) >> 31;
#endif
}

/* Check if point projects onto edge interior. */
static inline bool ivp_s_is_inside(const ivp_s_result_t *s) {
    return s->checks[0] >= IVP_REAL(0.0) && s->checks[1] >= IVP_REAL(0.0);
}

/* ── Distance query result ──────────────────────────────────────────────── */

typedef struct ivp_distance_result {
    ivp_real_t  distance;       /* signed distance (positive = separated) */
    ivp_real_t  quad_distance;  /* squared distance */
    ivp_vec3r_t closest_point_a; /* closest point on object A (object coords) */
    ivp_vec3r_t closest_point_b; /* closest point on object B (object coords) */
    ivp_vec3r_t normal;          /* contact normal (A→B, unit) */
} ivp_distance_result_t;

/* ── Barycentric / parametric computations ──────────────────────────────── */

/* Compute unscaled barycentric coordinates (QR) of a point projected onto
 * a triangle. The triangle is defined by edge (first edge of triangle).
 *
 * point_os: query point in object space
 * ledge:    owning ledge (for vertex lookup)
 * edge:     first edge (edge 0) of the triangle
 * result:   output QR values
 */
void ivp_calc_qr_vals(const ivp_compact_ledge_t    *ledge,
                      const ivp_compact_edge_t     *edge,
                      const ivp_vec3r_t            *point_os,
                      ivp_qr_result_t              *result);

/* Compute unscaled parametric position (S) of a point projected onto an edge.
 *
 * point_os: query point in object space
 * ledge:    owning ledge (for vertex lookup)
 * edge:     the edge to project onto
 * result:   output S values
 */
void ivp_calc_s_val(const ivp_compact_ledge_t    *ledge,
                    const ivp_compact_edge_t     *edge,
                    const ivp_vec3r_t            *point_os,
                    ivp_s_result_t               *result);

/* Compute edge-edge closest points (unscaled KK).
 *
 * ledge_k, edge_k: first edge
 * ledge_l, edge_l: second edge
 * result:          output KK values
 */
void ivp_calc_kk_vals(const ivp_compact_ledge_t    *ledge_k,
                      const ivp_compact_edge_t     *edge_k,
                      const ivp_compact_ledge_t    *ledge_l,
                      const ivp_compact_edge_t     *edge_l,
                      ivp_kk_result_t              *result);

/* ── Squared distance functions ─────────────────────────────────────────── */

/* Point-point squared distance. */
ivp_real_t ivp_qdist_pp(const ivp_vec3r_t *a, const ivp_vec3r_t *b);

/* Point-edge squared distance (clamps to endpoints if outside). */
ivp_real_t ivp_qdist_pk(const ivp_compact_ledge_t *ledge,
                        const ivp_compact_edge_t  *edge,
                        const ivp_vec3r_t         *point_os);

/* Point-triangle squared distance.
 * Returns squared distance from point to nearest feature of triangle. */
ivp_real_t ivp_qdist_pf(const ivp_compact_ledge_t *ledge,
                        const ivp_compact_edge_t  *tri_edge,
                        const ivp_vec3r_t         *point_os);

/* Edge-edge squared distance (handles all Voronoi regions). */
ivp_real_t ivp_qdist_kk(const ivp_compact_ledge_t *ledge_k,
                        const ivp_compact_edge_t  *edge_k,
                        const ivp_compact_ledge_t *ledge_l,
                        const ivp_compact_edge_t  *edge_l);

/* Perpendicular squared distance from point to infinite line through edge. */
ivp_real_t ivp_qdist_point_to_line(const ivp_compact_ledge_t *ledge,
                                    const ivp_compact_edge_t  *edge,
                                    const ivp_vec3r_t         *point_os);

/* ── Higher-level queries ───────────────────────────────────────────────── */

/* Sphere-sphere distance (positive = separated, negative = overlapping).
 * center_a, center_b: sphere centers
 * radius_a, radius_b: sphere radii */
ivp_real_t ivp_dist_sphere_sphere(const ivp_vec3r_t *center_a, ivp_real_t radius_a,
                                   const ivp_vec3r_t *center_b, ivp_real_t radius_b);

/* Compute Hesse normal vector of a triangle (unnormalized).
 * Returns cross product of two edges → outward face normal × 2×area. */
void ivp_calc_hesse_vec(const ivp_compact_ledge_t *ledge,
                        const ivp_compact_edge_t  *tri_edge,
                        ivp_vec3r_t               *hesse_out);

/* Compute signed distance from point to triangle plane.
 * Positive = on front side (same side as normal). */
ivp_real_t ivp_signed_dist_to_plane(const ivp_compact_ledge_t *ledge,
                                     const ivp_compact_edge_t  *tri_edge,
                                     const ivp_vec3r_t         *point_os);

#ifdef __cplusplus
}
#endif

#endif /* IVP_NARROWPHASE_H */
