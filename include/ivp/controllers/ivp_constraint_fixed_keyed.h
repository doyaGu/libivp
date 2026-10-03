/* ivp_constraint_fixed_keyed.h — Keyframed fixed constraint
 *
 * Port of IVP_Constraint_Fixed_Keyframed (ivp_constraint_fixed_keyed.cxx):
 * drives the attached object towards a prime position/orientation given in
 * the reference object's space.  The targets are extrapolated in time
 * (position: prime + velocity * (t - t_prime + dt); orientation: smooth
 * interpolation of two prime orientations over their time span) and reached
 * through two 3x3 core-reaction solves per PSI (rotation, then translation)
 * acting on BOTH cores (the reference object receives the reaction), with
 * per-axis impulse caps max_torque * dt and max_translation_force * dt.
 *
 * Defaults are those of IVP_Template_Controller_Motion: force 0.8, damp
 * 1.0, torque 0.8, angular damp 1.0, limits 1e6.
 */
#ifndef IVP_CONSTRAINT_FIXED_KEYED_H
#define IVP_CONSTRAINT_FIXED_KEYED_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Keyframed constraint state ────────────────────────────────────────── */

typedef struct ivp_fixed_keyed_state {
    /* Objects */
    ivp_object_t *reference_obj;    /* kinematic or dynamic reference */
    ivp_object_t *attached_obj;     /* dynamic object being driven */

    /* Prime position (reference object space) */
    ivp_vec3r_t   target_pos_ros;   /* prime_position_Ros */
    ivp_vec3r_t   target_vel_ros;   /* velocity_Ros */
    ivp_time_t    time_of_prime_position;

    /* Prime orientation (q_Ros_f_Aos), keyframed by time */
    ivp_quatr_t   target_orient_0;  /* prime_orientation_0 */
    ivp_quatr_t   target_orient_1;  /* prime_orientation_1 */
    ivp_time_t    time_of_prime_orientation_0;
    ivp_real_t    i_delta_prime_orientation_time;
    bool          orient_keyed;     /* angular_velocity_set */
    ivp_real_t    orient_blend;     /* deprecated (unused) */

    /* Stiffness and damping */
    ivp_real_t    force_factor;     /* default 0.8 */
    ivp_real_t    damp_factor;      /* default 1.0 */
    ivp_real_t    torque_factor;    /* default 0.8 */
    ivp_real_t    angular_damp_factor; /* default 1.0 */

    /* Force limits (impulse caps per world axis: limit * dt) */
    ivp_vec3r_t   max_translation_force; /* default 1e6 */
    ivp_vec3r_t   max_torque;            /* default 1e6 */

    bool active;
} ivp_fixed_keyed_state_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Reference constructor: the prime position/orientation is the current
 * relative pose of attached_obj in reference_obj space.  offset_ros (may be
 * NULL) overrides the prime position. */
void ivp_fixed_keyed_init(ivp_fixed_keyed_state_t *state,
                          ivp_object_t *reference_obj,
                          ivp_object_t *attached_obj,
                          const ivp_vec3r_t *offset_ros);

/* ── Target control ────────────────────────────────────────────────────── */

/* Reference set_prime_position_Ros. */
void ivp_fixed_keyed_set_prime_position_Ros(ivp_fixed_keyed_state_t *state,
                                            const ivp_vec3r_t *position_ros,
                                            const ivp_vec3r_t *velocity_ros,
                                            ivp_time_t time);

/* Reference set_prime_orientation_Ros (orientation1 may be NULL; dt is the
 * time between the two orientations). */
void ivp_fixed_keyed_set_prime_orientation_Ros(ivp_fixed_keyed_state_t *state,
                                               const ivp_quatr_t *orientation0,
                                               ivp_time_t time0,
                                               const ivp_quatr_t *orientation1,
                                               ivp_real_t dt);

/* Convenience: prime position at the current time, keeping the velocity. */
void ivp_fixed_keyed_set_target_pos(ivp_fixed_keyed_state_t *state, const ivp_vec3r_t *pos_ros);

/* Convenience: prime velocity, re-priming the current target position now. */
void ivp_fixed_keyed_set_target_vel(ivp_fixed_keyed_state_t *state, const ivp_vec3r_t *vel_ros);

/* Static target orientation interpolated between q0 and q1 by blend. */
void ivp_fixed_keyed_set_orientation(ivp_fixed_keyed_state_t *state,
                                     const ivp_quatr_t *q0,
                                     const ivp_quatr_t *q1,
                                     ivp_real_t blend);

/* Single static target orientation. */
void ivp_fixed_keyed_set_orientation_single(ivp_fixed_keyed_state_t *state,
                                            const ivp_quatr_t *target_q);

/* ── Force limits ──────────────────────────────────────────────────────── */

static inline void ivp_fixed_keyed_set_max_force(
    ivp_fixed_keyed_state_t *state, const ivp_vec3r_t *max_force) {
    state->max_translation_force = *max_force;
}

static inline void ivp_fixed_keyed_set_max_torque(
    ivp_fixed_keyed_state_t *state, const ivp_vec3r_t *max_torque) {
    state->max_torque = *max_torque;
}

/* Discovery function for ivp_environment_announce_dependent_controller(). */
int ivp_fixed_keyed_discovery(void *ctx, ivp_environment_t *env,
                              ivp_object_handle_t *out_handles, int max_handles);

/* ── Controller callback ───────────────────────────────────────────────── */

/* Pass ivp_fixed_keyed_state_t* as ctx. */
void ivp_controller_fixed_keyed(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONSTRAINT_FIXED_KEYED_H */
