/* ivp_sphere_solver.h -- Sphere overlap queries against compact geometry
 *
 * Static spatial queries: does a query sphere overlap with the bounding
 * hierarchy of a compact surface?
 *
 * This is the object-level check. Environment-level broadphase queries
 * (checking against all objects) require the broadphase module.
 *
 * Ported from ivp_sphere_solver.hxx / ivp_sphere_solver.cxx.
 */
#ifndef IVP_SPHERE_SOLVER_H
#define IVP_SPHERE_SOLVER_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/geometry/ivp_compact_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Sphere query template ─────────────────────────────────────────────── */

typedef struct ivp_sphere_query {
    ivp_vec3r_t center;           /* sphere center (object space for object
                                     queries, world space for env queries) */
    ivp_real_t  radius;           /* sphere radius */
    int         max_depth;        /* max BVH traversal depth (0 = root only) */
} ivp_sphere_query_t;

/* ── Sphere-vs-ledgetree overlap ───────────────────────────────────────── */

/* Test if a query sphere overlaps with any terminal ledge's bounding sphere
 * in the ledgetree hierarchy.
 *
 * Traverses the BVH up to max_depth levels. At each node, checks if the
 * query sphere overlaps the node's bounding sphere. If a terminal node
 * overlaps, returns true.
 *
 * Parameters:
 *   root      -- ledgetree root node
 *   center_os -- query sphere center in object space
 *   radius    -- query sphere radius
 *   max_depth -- maximum BVH traversal depth (0 = root only)
 *
 * Returns true if any terminal ledge's bounding sphere overlaps. */
bool ivp_sphere_overlaps_ledgetree(
    const ivp_compact_ledgetree_node_t *root,
    const ivp_vec3r_t *center_os,
    ivp_real_t radius,
    int max_depth);

/* Test if a query sphere overlaps with a compact surface's bounding hierarchy.
 *
 * This is a convenience wrapper that gets the ledgetree root from the surface.
 *
 * Parameters:
 *   surface   -- compact surface
 *   center_os -- query sphere center in object space
 *   radius    -- query sphere radius
 *   max_depth -- maximum BVH traversal depth (0 = root only)
 *
 * Returns true if any terminal ledge overlaps. */
bool ivp_sphere_overlaps_surface(
    const ivp_compact_surface_t *surface,
    const ivp_vec3r_t *center_os,
    ivp_real_t radius,
    int max_depth);

/* ── Sphere-sphere overlap ─────────────────────────────────────────────── */

/* Test if two spheres overlap (centers + radii).
 * Returns true if the spheres intersect or touch. */
static inline bool ivp_spheres_overlap(
    const ivp_vec3r_t *center_a, ivp_real_t radius_a,
    const ivp_vec3r_t *center_b, ivp_real_t radius_b)
{
    ivp_real_t sum_r = radius_a + radius_b;
    return ivp_vec3r_quad_distance(center_a, center_b) <= sum_r * sum_r;
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_SPHERE_SOLVER_H */
