/* ivp_compact_modify.h — Surface modification utilities
 *
 * Post-creation operations on compact surfaces:
 *   - Chop: slice off part of a surface using a halfspace plane
 *   - Shrink: contract surfaces inward by a uniform distance
 *
 * Ported from ivp_compact_modify.hxx.
 */
#ifndef IVP_COMPACT_MODIFY_H
#define IVP_COMPACT_MODIFY_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_compact_ledge.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Chop a compact surface along a halfspace plane.
 *
 * Removes all geometry on the positive side of the plane defined by:
 *   chop_normal · point >= chop_depth
 *
 * Parameters:
 *   alloc       -- allocator (NULL for default)
 *   surface_in  -- input surface (single convex ledge only)
 *   chop_normal -- plane normal direction (need not be normalized)
 *   chop_depth  -- plane offset along normal
 *
 * Returns a newly allocated chopped surface, or NULL on failure.
 * The input surface is NOT modified or freed. */
ivp_compact_surface_t *ivp_compact_modify_chop(
    const ivp_allocator_t *alloc,
    const ivp_compact_surface_t *surface_in,
    const float chop_normal[3],
    float chop_depth);

/* Shrink a single compact ledge by contracting all faces inward.
 *
 * Each face plane is shifted toward the centroid by shrink_value.
 * Points closer than merge_threshold are merged.
 *
 * Parameters:
 *   alloc            -- allocator (NULL for default)
 *   ledge_in         -- input convex ledge
 *   shrink_value     -- distance to shrink inward [m]
 *   merge_threshold  -- vertex merge distance (0 for default 1e-4)
 *
 * Returns a newly allocated compact surface, or NULL on failure. */
ivp_compact_surface_t *ivp_compact_modify_shrink_ledge(
    const ivp_allocator_t *alloc,
    const ivp_compact_ledge_t *ledge_in,
    float shrink_value,
    float merge_threshold);

/* Shrink a complete compact surface by contracting all faces inward.
 *
 * For compound surfaces, each component ledge is shrunk individually
 * and then recompiled into a new compound surface.
 *
 * Parameters:
 *   alloc            -- allocator (NULL for default)
 *   surface_in       -- input surface (single or compound)
 *   shrink_value     -- distance to shrink inward [m]
 *   merge_threshold  -- vertex merge distance (0 for default 1e-4)
 *
 * Returns a newly allocated shrunk surface, or NULL on failure.
 * The input surface is NOT modified or freed. */
ivp_compact_surface_t *ivp_compact_modify_shrink(
    const ivp_allocator_t *alloc,
    const ivp_compact_surface_t *surface_in,
    float shrink_value,
    float merge_threshold);

#ifdef __cplusplus
}
#endif

#endif /* IVP_COMPACT_MODIFY_H */
