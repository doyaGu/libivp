/* ivp_liquid_surface.h — Liquid surface descriptor interface
 *
 * Callback-based water surface interface for dynamic (wave, current)
 * surfaces. The buoyancy controller can optionally accept an
 * ivp_liquid_surface_t for non-flat water instead of a static plane.
 *
 * Also provides ivp_liquid_surface_simple_t as a convenience for
 * flat static water.
 *
 * Ported from ivp_physics/ivp_liquid_surface_descript.hxx.
 */
#ifndef IVP_LIQUID_SURFACE_H
#define IVP_LIQUID_SURFACE_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Callback type ─────────────────────────────────────────────────────── */

/* Evaluate the liquid surface at a world-space position.
 *
 * Given a query point (world space), returns:
 *   out_surface_normal: unit normal pointing OUT of the liquid
 *   out_height: signed height of the surface at (x,z) along the normal
 *   out_velocity: flow velocity of the liquid at this point
 *
 * Return true if the point is within the liquid domain, false if outside. */
typedef bool (*ivp_liquid_surface_eval_fn_t)(void *ctx,
                                             const ivp_vec3r_t *query_pos_ws,
                                             ivp_vec3r_t *out_surface_normal,
                                             ivp_real_t  *out_height,
                                             ivp_vec3r_t *out_velocity);

/* ── Liquid surface descriptor ─────────────────────────────────────────── */

typedef struct ivp_liquid_surface {
    ivp_liquid_surface_eval_fn_t eval_fn;
    void                        *ctx;
} ivp_liquid_surface_t;

/* Initialize with a custom evaluation callback. */
static inline void ivp_liquid_surface_init(ivp_liquid_surface_t *ls,
                                           ivp_liquid_surface_eval_fn_t fn,
                                           void *ctx)
{
    ls->eval_fn = fn;
    ls->ctx     = ctx;
}

/* ── Simple flat-plane convenience ─────────────────────────────────────── */

typedef struct ivp_liquid_surface_simple {
    ivp_liquid_surface_t base;

    ivp_vec3r_t surface_normal;    /* unit normal pointing out of liquid */
    ivp_real_t  surface_height;    /* plane offset along normal */
    ivp_vec3r_t flow_velocity;     /* constant flow velocity */
    ivp_real_t  fluid_density;     /* kg/m^3 */
} ivp_liquid_surface_simple_t;

/* Initialize a flat-plane liquid surface (Y-down gravity system).
 * surface_y: Y coordinate of the water surface.
 * density: fluid density in kg/m^3. */
void ivp_liquid_surface_simple_init(ivp_liquid_surface_simple_t *ls,
                                    ivp_real_t surface_y,
                                    ivp_real_t density);

/* Initialize a flat-plane liquid surface with a custom normal. */
void ivp_liquid_surface_simple_init_plane(ivp_liquid_surface_simple_t *ls,
                                          const ivp_vec3r_t *normal,
                                          ivp_real_t height,
                                          ivp_real_t density);

#ifdef __cplusplus
}
#endif

#endif /* IVP_LIQUID_SURFACE_H */
