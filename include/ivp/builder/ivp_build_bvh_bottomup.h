/* ivp_build_bvh_bottomup.h -- Bottom-up BVH builder
 *
 * Alternative BVH construction via greedy agglomeration.
 * Iteratively merges closest sphere pairs within a radius tolerance
 * using a spatial hash, producing tighter bounding volumes for
 * certain geometry distributions.
 *
 * Ported from the legacy bottom-up BVH strategy.
 */
#ifndef IVP_BUILD_BVH_BOTTOMUP_H
#define IVP_BUILD_BVH_BOTTOMUP_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/builder/ivp_build_ledge_soup.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Configuration ─────────────────────────────────────────────────────── */

typedef struct ivp_bvh_bottomup_config {
    ivp_real_t radius_tolerance;     /* max growth factor per merge (default: 1.1) */
    const ivp_allocator_t *alloc;
} ivp_bvh_bottomup_config_t;

/* Initialize config with defaults. */
void ivp_bvh_bottomup_config_init(ivp_bvh_bottomup_config_t *cfg);

/* ── Compile ───────────────────────────────────────────────────────────── */

/* Compile a ledge soup using bottom-up BVH construction.
 * Returns a newly allocated compact surface, or NULL on failure.
 * The caller must free with ivp_free_aligned(). */
ivp_compact_surface_t *ivp_ledge_soup_compile_bottomup(
    ivp_ledge_soup_t *soup,
    const ivp_bvh_bottomup_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_BVH_BOTTOMUP_H */
