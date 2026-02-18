/* ivp_build_surface.h — Minimal surface builder API
 *
 * M2 builders:
 *   ivp_build_box_surface()      — hardcoded box topology
 *   ivp_build_convex_surface()   — convex hull from points (QHull)
 *
 * All returned surfaces must be freed with ivp_free_aligned().
 */
#ifndef IVP_BUILD_SURFACE_H
#define IVP_BUILD_SURFACE_H

#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/math/ivp_vec3.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Build a compact surface for an axis-aligned box centered at the origin.
 *
 * Parameters:
 *   alloc  — allocator (NULL for default)
 *   half_x — half-extent along X axis
 *   half_y — half-extent along Y axis
 *   half_z — half-extent along Z axis
 *
 * Returns:
 *   Pointer to a newly allocated compact surface, or NULL on failure.
 *   The caller must free with ivp_free_aligned(alloc, result).
 *
 * The box has:
 *   - 8 vertices
 *   - 12 triangles (6 faces × 2 triangles each)
 *   - 1 terminal ledgetree node
 *   - mass_center at origin
 *   - rotation_inertia for unit mass
 */
ivp_compact_surface_t *ivp_build_box_surface(const ivp_allocator_t *alloc,
                                               float half_x,
                                               float half_y,
                                               float half_z);

/* Build a compact convex surface from an input point cloud (3D).
 *
 * Parameters:
 *   alloc    — allocator (NULL for default)
 *   points   — input point array
 *   n_points — number of points (must be >= 4)
 *
 * Returns:
 *   Pointer to a newly allocated compact surface, or NULL on failure/degeneracy.
 *   The caller must free with ivp_free_aligned(alloc, result).
 */
ivp_compact_surface_t *ivp_build_convex_surface(const ivp_allocator_t *alloc,
                                                 const ivp_vec3f_t *points,
                                                 int n_points);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_SURFACE_H */
