/* ivp_controller_floating.h — Floating/levitation controller
 *
 * Maintains an object at a target distance from a surface by applying
 * a 1D impulse along a configurable ray direction. The user provides
 * a ray-cast callback to determine the current distance to the surface.
 *
 * The controller clamps the applied force between max_repulsive_force
 * (pushing away from surface) and max_adhesive_force (pulling toward
 * surface).
 *
 * Ported from ivp_controller_floating.hxx.
 */
#ifndef IVP_CONTROLLER_FLOATING_H
#define IVP_CONTROLLER_FLOATING_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Ray-cast callback ─────────────────────────────────────────────────── */

/* User-provided callback to measure the distance from an object to a
 * surface along the ray direction.
 *
 * Parameters:
 *   ctx             -- user context pointer
 *   obj             -- the floating object
 *   ray_start_ws    -- ray origin in world space (anchor position)
 *   ray_dir_ws      -- normalized ray direction in world space
 *   out_distance    -- write the measured distance here
 *
 * Returns true if a surface was hit, false if no surface found (object
 * is too far from any surface). */
typedef bool (*ivp_floating_raycast_fn_t)(void *ctx,
                                          const ivp_object_t *obj,
                                          const ivp_vec3r_t *ray_start_ws,
                                          const ivp_vec3r_t *ray_dir_ws,
                                          ivp_real_t *out_distance);

/* ── Floating state ────────────────────────────────────────────────────── */

typedef struct ivp_floating_state {
    /* Attachment point in object space */
    ivp_vec3r_t  position_os;

    /* Ray direction in world space (unit vector) */
    ivp_vec3r_t  ray_direction_ws;

    /* Distance targets */
    ivp_real_t   target_distance;
    ivp_real_t   current_distance;

    /* Force limits */
    ivp_real_t   max_repulsive_force;   /* max push-away force (positive) */
    ivp_real_t   max_adhesive_force;    /* max pull-toward force (positive) */

    /* Damping */
    ivp_real_t   damping;               /* velocity damping along ray (0-1) */

    /* Ray-cast callback */
    ivp_floating_raycast_fn_t raycast_fn;
    void                     *raycast_ctx;

    bool active;
} ivp_floating_state_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Initialize a floating controller for Y-down gravity (levitate above ground).
 *
 * target_dist:   desired hover distance
 * max_repulsive: maximum upward force
 * max_adhesive:  maximum downward force (to pull object toward surface) */
void ivp_floating_init(ivp_floating_state_t *state,
                       ivp_real_t target_dist,
                       ivp_real_t max_repulsive,
                       ivp_real_t max_adhesive);

/* Set the ray-cast callback. Must be called before the controller runs. */
void ivp_floating_set_raycast(ivp_floating_state_t *state,
                              ivp_floating_raycast_fn_t fn,
                              void *ctx);

/* Set anchor position in object space. */
void ivp_floating_set_position_os(ivp_floating_state_t *state,
                                  const ivp_vec3r_t *pos_os);

/* Set ray direction in world space (will be normalized). */
void ivp_floating_set_ray_direction_ws(ivp_floating_state_t *state,
                                       const ivp_vec3r_t *dir_ws);

/* ── Controller callback ───────────────────────────────────────────────── */

/* Pass ivp_floating_state_t* as ctx. */
void ivp_controller_floating(const ivp_event_sim_t *event_sim,
                             ivp_object_t *obj,
                             void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_FLOATING_H */
