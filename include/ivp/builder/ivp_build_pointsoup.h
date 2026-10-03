/* ivp_build_pointsoup.h -- Build compact surface from a point cloud
 *
 * Takes a raw array of 3D points and builds a convex hull surface
 * (IVP_SurfaceBuilder_Pointsoup).  Thin wrapper over
 * ivp_build_convex_surface that accepts raw float arrays, plus optional
 * pre-processing (welding nearby points, centering).
 *
 * Defaults follow the reference: only bitwise identical points are merged
 * (IVP_I_Point_VHash) and the points are never re-centered, so the hull
 * geometry and surface->mass_center live in the caller's coordinate system.
 *
 * Ported from ivp_surbuild_pointsoup.hxx.
 */
#ifndef IVP_BUILD_POINTSOUP_H
#define IVP_BUILD_POINTSOUP_H

#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/base/ivp_allocator.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Point soup options ────────────────────────────────────────────────── */

typedef struct ivp_pointsoup_options {
    /* Merge points closer than this before building the hull.
     * Default 0: only exact duplicates are merged (reference behaviour). */
    float weld_epsilon;
    /* libivp extension: translate the points so that their centroid is at
     * the origin before building the hull.  The geometry AND mass_center of
     * the returned surface are then expressed in that shifted frame
     * (mass_center is NOT shifted back).  Default false (reference never
     * re-centers). */
    bool  center_points;
} ivp_pointsoup_options_t;

static inline ivp_pointsoup_options_t ivp_pointsoup_options_default(void) {
    ivp_pointsoup_options_t o;
    o.weld_epsilon = 0.0f;
    o.center_points = false;
    return o;
}

/* ── Builder ───────────────────────────────────────────────────────────── */

/* Build a convex hull compact surface from a raw float array of points.
 *
 * Parameters:
 *   alloc      -- allocator (NULL for default)
 *   coords     -- flat array of [x0,y0,z0, x1,y1,z1, ...] (3 * n_points floats)
 *   n_points   -- number of 3D points (>= 3; 3 points give a two sided
 *                 triangle like the reference)
 *   options    -- optional config (NULL for defaults)
 *
 * Returns:
 *   Newly allocated compact surface, or NULL on failure.
 *   Caller must free with ivp_free_aligned(). */
ivp_compact_surface_t *ivp_build_pointsoup_surface(
    const ivp_allocator_t *alloc,
    const float *coords,
    int n_points,
    const ivp_pointsoup_options_t *options);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_POINTSOUP_H */
