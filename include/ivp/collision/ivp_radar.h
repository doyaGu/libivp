/* ivp_radar.h — Proximity query (radar) interface
 *
 * Given a source object and max range, iterates all objects in the
 * environment, applies collision filter, computes distance estimates,
 * and calls a user callback for each match within range.
 *
 * Ported from ivp_physics/ivp_radar.hxx.
 */
#ifndef IVP_RADAR_H
#define IVP_RADAR_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_forward_decls.h>
#include <ivp/math/ivp_vec3.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Radar hit callback ────────────────────────────────────────────────── */

/* Called for each object within range.
 * distance: estimated center-to-center distance minus bounding radii.
 * direction: unit vector from source to target (world space). */
typedef void (*ivp_radar_hit_fn_t)(void *ctx,
                                    ivp_object_t *target,
                                    ivp_real_t distance,
                                    const ivp_vec3r_t *direction);

/* ── Radar configuration ───────────────────────────────────────────────── */

typedef struct ivp_radar_config {
    ivp_real_t        max_range;          /* maximum detection distance, default: 100 */
    ivp_real_t        max_relative_error; /* reference parity: allowed error fraction (unused, reserved) */
    bool              use_filter;         /* apply collision filter, default: true */
    bool              skip_unmoveable;    /* skip static objects, default: false */
    ivp_radar_hit_fn_t hit_fn;            /* callback for each match (required) */
    void              *ctx;               /* user context for callback */
} ivp_radar_config_t;

/* Fill with defaults. */
void ivp_radar_config_init(ivp_radar_config_t *cfg);

/* ── Radar scan ────────────────────────────────────────────────────────── */

/* Perform a proximity scan from the given source object.
 * Calls hit_fn for each object within max_range. */
void ivp_environment_do_radar(ivp_environment_t *env,
                              ivp_object_t *source,
                              const ivp_radar_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RADAR_H */
