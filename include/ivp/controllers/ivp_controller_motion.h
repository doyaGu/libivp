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
#include <ivp/math/ivp_quat.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Motion mode ────────────────────────────────────────────────────────── */

/* Each channel (linear / angular) runs in one mode.  A new state is in
 * IVP_MOTION_TARGET mode on both channels with the reference constructor's
 * target (origin, identity orientation): like IVP_Controller_Motion the
 * controller pulls its object toward that target until set_target_* moves
 * it.  The other modes are libivp extras that replace a channel's target
 * drive (IVP_MOTION_NONE disables a channel). */
typedef enum ivp_motion_mode {
    IVP_MOTION_NONE       = 0,  /* no motion applied */
    IVP_MOTION_VELOCITY   = 1,  /* set target velocity directly */
    IVP_MOTION_FORCE      = 2,  /* apply force (velocity += F/m * dt) */
    IVP_MOTION_IMPULSE    = 3,  /* apply instantaneous impulse (one-shot) */
    IVP_MOTION_TARGET     = 4   /* reference IVP_Controller_Motion: drive toward a
                                   target position / orientation with
                                   force_factor, damp_factor and force clipping */
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

    /* IVP_MOTION_TARGET parameters (reference IVP_Template_Controller_Motion).
     * Each PSI the controller computes the position error scaled by
     * force_factor, subtracts speed * dt * damp_factor, converts that to a
     * per-axis force and clips it to max_translation_force before applying it
     * as a synchronous speed change.  The angular path does the same with the
     * quaternion error, torque_factor, angular_damp_factor and max_torque. */
    ivp_vec3r_t       target_pos_ws;
    ivp_quatr_t       target_q_world_f_core;
    ivp_real_t        force_factor;          /* default 0.8 */
    ivp_real_t        damp_factor;           /* default 1.0 */
    ivp_real_t        torque_factor;         /* default 0.8 */
    ivp_real_t        angular_damp_factor;   /* default 1.0 */
    ivp_vec3r_t       max_translation_force; /* per axis, default 1e6 */
    ivp_vec3r_t       max_torque;            /* per axis, default 1e6 */

    /* Reference IVP_Controller_Motion::real_object: the controlled object
     * (ivp_motion_state_setup).  The target setters wake it
     * (ensure_core_in_simulation) as the reference does.  libivp extra:
     * NULL acts on every core the controller is dispatched for. */
    ivp_object_t     *object;
} ivp_motion_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

/* Initialize a motion state: reference IVP_Template_Controller_Motion
 * defaults and the IVP_Controller_Motion constructor's target (origin,
 * identity orientation) in IVP_MOTION_TARGET mode on both channels, i.e. the
 * controller drives toward the origin until a target is set. */
void ivp_motion_state_init(ivp_motion_state_t *state);

/* Bind the state to one object (reference IVP_Controller_Motion
 * constructor; the targets keep their values). */
void ivp_motion_state_setup(ivp_motion_state_t *state, ivp_object_t *object);

/* Drive toward a world-space target position of the mass center (reference
 * IVP_Controller_Motion::set_target_position_ws): returns early when the
 * channel already targets this position (e.g. the initial origin), else
 * wakes the bound object. Switches linear_mode to IVP_MOTION_TARGET. */
void ivp_motion_state_set_target_position(ivp_motion_state_t *state,
                                          const ivp_vec3r_t *target_ws);

/* Reference set_target_object_position_ws: target given for the object
 * space origin with the desired orientation (the core/object shift is
 * removed when it is not zero). */
void ivp_motion_state_set_target_object_position(ivp_motion_state_t *state,
                                                 const ivp_object_t *obj,
                                                 const ivp_quatr_t *desired_orientation,
                                                 const ivp_vec3r_t *position_of_object_space_center_ws);

/* Drive toward a world-space target orientation (reference
 * set_target_q_world_f_core): invalid quaternions (NaN, |q|^2 <= eps or
 * > max) are ignored, the quaternion is normalized, an unchanged target
 * returns early, else the bound object is woken. Switches angular_mode to
 * IVP_MOTION_TARGET. */
void ivp_motion_state_set_target_orientation(ivp_motion_state_t *state,
                                             const ivp_quatr_t *q_world_f_core);

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
 * Register with ivp_environment_add_controller(env, any_priority, fn, state).
 * Built-in priorities are type-driven; the priority argument is ignored. */
void ivp_controller_motion(const ivp_event_sim_t *event_sim,
                          ivp_object_t *obj,
                          void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_MOTION_H */
