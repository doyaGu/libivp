/* ivp_sphere_solver.h -- Sphere overlap queries against compact geometry
 *
 * Static spatial queries: does a query sphere overlap with the bounding
 * hierarchy of a compact surface?
 *
 * Object-level ledgetree checks plus the environment query
 * (check_sphere_against_environment, an OV tree walk).
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

/* ── Environment / object queries (IVP_Sphere_Solver) ──────────────────── */

struct ivp_object;
struct ivp_environment;

/* IVP_Sphere_Solver_Template */
typedef struct ivp_sphere_solver_template {
    ivp_vec3r_t center;              /* world space */
    ivp_real_t  radius;
    int         max_traversal_depth; /* ledgetree depth used for culling */
} ivp_sphere_solver_template_t;

/* IVP_Sphere_Solver::check_sphere_against_object: query sphere (world
 * space) against one object.  Polygons: the sphere is transformed into
 * object space (object transform at the environment's current time) and
 * tested against the ledgetree down to max_traversal_depth.  Balls: sphere
 * vs. ball (core upper_limit_radius).  Note: like the reference
 * (ivp_sphere_solver.cxx:93) the object space query center is compared
 * with the ball's WORLD core position, so balls away from the world origin
 * are usually not reported; kept for parity. */
bool ivp_sphere_solver_check_object(const ivp_sphere_solver_template_t *tmpl,
                                    const struct ivp_object *object);

/* IVP_Sphere_Solver::check_sphere_against_environment: walks the
 * environment's OV tree (recursively_collect_intruding_objects).  A node is
 * entered when its cube overlaps the query sphere's bounding box; its
 * objects (first to last) are reported when their core sphere (position at
 * the last PSI, upper_limit_radius) overlaps the query sphere and they pass
 * ivp_sphere_solver_check_object(); then the children are visited (first
 * to last).  Objects without collision detection are not in the OV tree
 * and are never reported; phantoms are reported like any other object.
 * The output is in this traversal order.  Writes at most `capacity`
 * objects and returns the total number of intruding objects. */
int ivp_sphere_solver_check_environment(const ivp_sphere_solver_template_t *tmpl,
                                        const struct ivp_environment *env,
                                        struct ivp_object **intruding_objects,
                                        int capacity);

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
