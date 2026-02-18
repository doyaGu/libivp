/* ivp_ray_solver.h — Ray casting against compact surfaces
 *
 * Finds closest ray-surface intersection by traversing the ledgetree
 * hierarchy and testing individual triangles.
 *
 * For single-triangle ledges: plane intersection + barycentric inside test.
 * For multi-triangle convex ledges: convex walk to find the exit triangle.
 */
#ifndef IVP_RAY_SOLVER_H
#define IVP_RAY_SOLVER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/geometry/ivp_compact_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ──────────────────────────────────────────────────────────── */

#define IVP_MAX_RAY_HITS 256

/* ── Ray hit result ─────────────────────────────────────────────────────── */

typedef struct ivp_ray_hit {
    ivp_real_t                       hit_distance;        /* parametric t along ray */
    ivp_vec3r_t                      hit_point;           /* world/object space hit position */
    ivp_vec3r_t                      hit_normal;          /* surface normal at hit (unnormalized) */
    const ivp_compact_ledge_t       *hit_ledge;           /* which ledge was hit */
    const ivp_compact_triangle_t    *hit_triangle;        /* which triangle was hit */
    int                              hit_material_index;  /* material index from triangle */
} ivp_ray_hit_t;

/* ── Ray solver input ───────────────────────────────────────────────────── */

typedef struct ivp_ray_input {
    ivp_vec3r_t origin;       /* ray start point (object space) */
    ivp_vec3r_t direction;    /* normalized ray direction */
    ivp_real_t  max_distance; /* maximum ray length */
} ivp_ray_input_t;

/* ── Ray solver API ─────────────────────────────────────────────────────── */

/* Cast a ray against a single compact surface (object space).
 * Returns IVP_OK if a hit was found, IVP_ERR_DEGENERATE_GEOMETRY if no hit.
 *
 * Parameters:
 *   surface   — the compact surface to test against
 *   ray       — ray origin, direction (must be normalized), max distance
 *   hit_out   — [out] closest hit result (only valid if returns IVP_OK)
 *
 * The ray is assumed to be in the same coordinate system as the surface
 * (object space). For world-space casting, transform the ray first.
 */
ivp_result_t ivp_ray_cast_surface(const ivp_compact_surface_t *surface,
                                   const ivp_ray_input_t      *ray,
                                   ivp_ray_hit_t              *hit_out);

/* Cast a ray against a single convex ledge (object space).
 * Lower-level function for testing against individual convex pieces.
 */
ivp_result_t ivp_ray_cast_ledge(const ivp_compact_ledge_t *ledge,
                                 const ivp_ray_input_t     *ray,
                                 ivp_ray_hit_t             *hit_out);

/* Cast a ray against a single triangle.
 * Returns IVP_OK if the ray hits the triangle.
 *
 * Parameters:
 *   ledge     — owning ledge
 *   tri       — the triangle to test
 *   ray       — ray input
 *   hit_dist  — [out] parametric hit distance (only if returns IVP_OK)
 */
ivp_result_t ivp_ray_test_triangle(const ivp_compact_ledge_t    *ledge,
                                    const ivp_compact_triangle_t *tri,
                                    const ivp_ray_input_t        *ray,
                                    ivp_real_t                   *hit_dist);

/* ── Ray-sphere intersection utility ────────────────────────────────────── */

/* Test if a ray intersects a bounding sphere.
 * Returns true if the ray passes within 'radius' of 'center'.
 * If hit, *t_out receives parametric distance to closest approach. */
bool ivp_ray_test_sphere(const ivp_vec3r_t *origin,
                         const ivp_vec3r_t *direction,
                         const ivp_vec3r_t *center,
                         ivp_real_t         radius,
                         ivp_real_t        *t_out);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RAY_SOLVER_H */
