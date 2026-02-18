/* ivp_build_pointsoup.h -- Build compact surface from a point cloud
 *
 * Takes a raw array of 3D points and builds a convex hull surface.
 * Thin wrapper over ivp_build_convex_surface that accepts raw float
 * arrays instead of ivp_vec3f_t, plus optional pre-processing
 * (welding duplicate points, centering).
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
    float weld_epsilon;     /* merge points closer than this (default: 1e-4) */
    bool  center_points;    /* subtract centroid before hull (default: true) */
} ivp_pointsoup_options_t;

static inline ivp_pointsoup_options_t ivp_pointsoup_options_default(void) {
    ivp_pointsoup_options_t o;
    o.weld_epsilon = 1e-4f;
    o.center_points = true;
    return o;
}

/* ── Builder ───────────────────────────────────────────────────────────── */

/* Build a convex hull compact surface from a raw float array of points.
 *
 * Parameters:
 *   alloc      -- allocator (NULL for default)
 *   coords     -- flat array of [x0,y0,z0, x1,y1,z1, ...] (3 * n_points floats)
 *   n_points   -- number of 3D points (must be >= 4)
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
