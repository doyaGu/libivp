/* ivp_controller_motion.h -- Motion controller for direct velocity/force
 *
 * Applies direct velocity targets, forces, or torques to objects.
 * This is the primary controller for user-driven object movement.
 *
 * Usage:
 *   1. Create an ivp_motion_state_t
 *   2. Register ivp_controller_motion as a controller callback
 *   3. Update the motion state each frame
 *
 * Ported from ivp_controller_motion.hxx.
 */
#ifndef IVP_CONTROLLER_MOTION_H
#define IVP_CONTROLLER_MOTION_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Motion mode ────────────────────────────────────────────────────────── */

typedef enum ivp_motion_mode {
    IVP_MOTION_NONE       = 0,  /* no motion applied */
    IVP_MOTION_VELOCITY   = 1,  /* set target velocity directly */
    IVP_MOTION_FORCE      = 2,  /* apply force (velocity += F/m * dt) */
    IVP_MOTION_IMPULSE    = 3   /* apply instantaneous impulse (one-shot) */
} ivp_motion_mode_t;

/* ── Motion state ───────────────────────────────────────────────────────── */

typedef struct ivp_motion_state {
    /* Linear motion */
    ivp_motion_mode_t linear_mode;
    ivp_vec3r_t       linear_value;    /* velocity (m/s), force (N), or impulse (N·s) */

    /* Angular motion */
    ivp_motion_mode_t angular_mode;
    ivp_vec3r_t       angular_value;   /* angular velocity (rad/s), torque (N·m),
                                          or angular impulse (N·m·s) */

    /* Speed limits */
    ivp_real_t        max_speed;       /* max linear speed (0 = unlimited) */
    ivp_real_t        max_angular_speed; /* max angular speed (0 = unlimited) */
} ivp_motion_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

/* Initialize a motion state to no motion. */
void ivp_motion_state_init(ivp_motion_state_t *state);

/* Set target velocity (will apply force to reach it). */
void ivp_motion_state_set_velocity(ivp_motion_state_t *state,
                                    const ivp_vec3r_t *velocity);

/* Set force to apply each step. */
void ivp_motion_state_set_force(ivp_motion_state_t *state,
                                 const ivp_vec3r_t *force);

/* Set one-shot impulse (cleared after application). */
void ivp_motion_state_set_impulse(ivp_motion_state_t *state,
                                   const ivp_vec3r_t *impulse);

/* Set angular velocity target. */
void ivp_motion_state_set_angular_velocity(ivp_motion_state_t *state,
                                             const ivp_vec3r_t *angular_velocity);

/* Set torque to apply each step. */
void ivp_motion_state_set_torque(ivp_motion_state_t *state,
                                  const ivp_vec3r_t *torque);

/* ── Controller callback ────────────────────────────────────────────────── */

/* The controller callback. Pass an ivp_motion_state_t* as ctx.
 * Register with ivp_environment_add_controller(env, priority, fn, state). */
void ivp_controller_motion(const ivp_event_sim_t *event_sim,
                            ivp_object_t *obj,
                            void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_MOTION_H */
