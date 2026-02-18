/* ivp_build_halfspacesoup.h — Half-space intersection surface builder
 *
 * Builds a convex compact surface from the intersection of half-spaces.
 * Each half-space is defined by a plane (normal + distance).
 *
 * Algorithm:
 *   1. Find all triple-plane intersection points.
 *   2. Test each point for containment within all other half-spaces.
 *   3. Merge nearby points within a tolerance.
 *   4. Build convex hull from the surviving vertices.
 *
 * Ported from ivp_surbuild_halfspacesoup.hxx.
 */
#ifndef IVP_BUILD_HALFSPACESOUP_H
#define IVP_BUILD_HALFSPACESOUP_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Half-space (plane) definition ─────────────────────────────────────── */

typedef struct ivp_halfspace {
    float normal[3];   /* inward-pointing unit normal */
    float dist;        /* signed distance: normal · point = dist on the plane */
} ivp_halfspace_t;

/* ── Build functions ───────────────────────────────────────────────────── */

/* Build a convex compact surface from half-space intersection.
 *
 * Parameters:
 *   alloc               -- allocator (NULL for default)
 *   halfspaces          -- array of half-space plane definitions
 *   n_halfspaces        -- number of half-spaces (>= 4)
 *   merge_threshold     -- distance below which vertices are merged (default: 0.01)
 *
 * Returns a newly allocated compact surface, or NULL on failure.
 * Fails if the intersection is empty, unbounded, or degenerate.
 * The caller must free with ivp_compact_surface_free(). */
ivp_compact_surface_t *ivp_build_halfspacesoup_surface(
    const ivp_allocator_t *alloc,
    const ivp_halfspace_t *halfspaces,
    int n_halfspaces,
    float merge_threshold);

/* Convert a half-space set to a point cloud (intermediate step).
 *
 * Parameters:
 *   halfspaces          -- array of half-space plane definitions
 *   n_halfspaces        -- number of half-spaces (>= 4)
 *   merge_threshold     -- vertex merge distance
 *   out_points          -- output: allocated float array [n*3]
 *   out_n_points        -- output: number of points
 *   alloc               -- allocator (NULL for default)
 *
 * Returns IVP_OK on success. Caller must free out_points. */
ivp_result_t ivp_halfspacesoup_to_points(
    const ivp_halfspace_t *halfspaces,
    int n_halfspaces,
    float merge_threshold,
    float **out_points,
    int *out_n_points,
    const ivp_allocator_t *alloc);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_HALFSPACESOUP_H */
