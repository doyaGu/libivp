/* ivp_build_compact_recursive.h — Root convex hull builder
 *
 * Computes a single convex hull that envelops all vertices from a set
 * of compact ledges.  The resulting hull can serve as a fast broadphase
 * bounding shape for compound (concave) surfaces.
 *
 * Ported from ivp_compact_recursive.hxx.
 */
#ifndef IVP_BUILD_COMPACT_RECURSIVE_H
#define IVP_BUILD_COMPACT_RECURSIVE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_compact_ledge.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Build a convex hull surface that envelops all vertices from the
 * given set of compact ledges.
 *
 * Parameters:
 *   alloc     -- allocator (NULL for default)
 *   ledges    -- array of compact ledge pointers
 *   n_ledges  -- number of ledges (must be >= 1)
 *
 * Returns:
 *   A newly allocated compact surface containing a single convex ledge,
 *   or NULL on failure.  Caller must free with ivp_compact_surface_free().
 *
 * The input ledges are not modified or freed. */
ivp_compact_surface_t *ivp_build_root_convex_hull(
    const ivp_allocator_t *alloc,
    const ivp_compact_ledge_t **ledges,
    int n_ledges);

/* Extract all unique vertices from a set of compact ledges into a flat
 * float array suitable for point-soup hull building.
 *
 * Parameters:
 *   ledges      -- array of compact ledge pointers
 *   n_ledges    -- number of ledges
 *   out_coords  -- output: flat float array [x0,y0,z0, x1,...] (caller frees)
 *   out_count   -- output: number of 3D points written
 *   alloc       -- allocator for out_coords (NULL for default)
 *
 * Returns 0 on success, -1 on failure. */
int ivp_extract_ledge_vertices(
    const ivp_compact_ledge_t **ledges,
    int n_ledges,
    float **out_coords,
    int *out_count,
    const ivp_allocator_t *alloc);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_COMPACT_RECURSIVE_H */
