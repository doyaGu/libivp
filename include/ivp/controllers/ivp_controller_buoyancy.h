/* ivp_controller_buoyancy.h -- Buoyancy / floating controller
 *
 * Simulates buoyancy using Archimedes' principle: objects below the
 * fluid surface receive an upward force proportional to their submerged
 * volume (approximated as fraction of bounding sphere submerged).
 *
 * Also applies fluid drag (linear + angular damping scaled by immersion).
 *
 * Ported from ivp_controller_buoyancy.hxx / ivp_buoyancy_solver.hxx.
 */
#ifndef IVP_CONTROLLER_BUOYANCY_H
#define IVP_CONTROLLER_BUOYANCY_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Buoyancy state ────────────────────────────────────────────────────── */

typedef struct ivp_buoyancy_state {
    /* Fluid surface: defined as a plane (normal · p = height) */
    ivp_vec3r_t surface_normal;    /* unit normal pointing OUT of fluid (default: 0,-1,0 for Y-up) */
    ivp_real_t  surface_height;    /* signed distance from origin along normal */

    /* Fluid properties */
    ivp_real_t  fluid_density;     /* kg/m^3 (water: 1000, air: 1.2) */
    ivp_real_t  linear_drag;       /* linear velocity damping in fluid (default: 0.5) */
    ivp_real_t  angular_drag;      /* angular velocity damping in fluid (default: 0.3) */

    /* Flow (optional current) */
    ivp_vec3r_t flow_velocity;     /* fluid flow velocity in world space */

    bool active;
} ivp_buoyancy_state_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Initialize buoyancy for a Y-down gravity system.
 * surface_y: Y coordinate of the fluid surface.
 * fluid_density: density in kg/m^3. */
void ivp_buoyancy_init(ivp_buoyancy_state_t *state,
                         ivp_real_t surface_y,
                         ivp_real_t fluid_density);

/* Initialize buoyancy with custom plane normal. */
void ivp_buoyancy_init_plane(ivp_buoyancy_state_t *state,
                               const ivp_vec3r_t *normal,
                               ivp_real_t height,
                               ivp_real_t fluid_density);

/* ── Controller callback ───────────────────────────────────────────────── */

/* Pass ivp_buoyancy_state_t* as ctx. */
void ivp_controller_buoyancy(const ivp_event_sim_t *event_sim,
                               ivp_object_t *obj,
                               void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_BUOYANCY_H */
