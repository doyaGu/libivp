/* ivp_constraint_fixed_keyed.h — Keyframed fixed constraint
 *
 * A constraint that drives an attached object toward a target position
 * and orientation defined in the reference object's local frame.
 * The target can be smoothly keyframed (interpolated) over time.
 *
 * Used for: ragdoll bones, animation-driven physics, grab/carry mechanics,
 * and any scenario where a physics object tracks a kinematic target.
 *
 * The constraint applies spring-damper impulses clamped by configurable
 * max force/torque limits. When limits are exceeded, the constraint
 * yields gracefully (no breakage).
 *
 * Ported from ivp_constraint_fixed_keyed.hxx.
 */
#ifndef IVP_CONSTRAINT_FIXED_KEYED_H
#define IVP_CONSTRAINT_FIXED_KEYED_H

#include <ivp/base/ivp_types.h>
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

    /* Target position in reference object space */
    ivp_vec3r_t   target_pos_ros;   /* target offset from reference origin */
    ivp_vec3r_t   target_vel_ros;   /* target velocity in reference space */

    /* Target orientation (keyframed between two quaternions) */
    ivp_quatr_t   target_orient_0;  /* start orientation */
    ivp_quatr_t   target_orient_1;  /* end orientation (for interpolation) */
    ivp_real_t    orient_blend;     /* 0..1 interpolation factor */
    bool          orient_keyed;     /* true if using interpolated orientation */

    /* Stiffness and damping */
    ivp_real_t    force_factor;     /* translational stiffness [0..1] (default: 1) */
    ivp_real_t    damp_factor;      /* translational damping [0..1] (default: 0.9) */
    ivp_real_t    torque_factor;    /* angular stiffness [0..1] (default: 1) */
    ivp_real_t    angular_damp_factor; /* angular damping [0..1] (default: 0.9) */

    /* Force limits */
    ivp_vec3r_t   max_translation_force; /* per-axis max force [N] */
    ivp_vec3r_t   max_torque;            /* per-axis max torque [N*m] */

    bool active;
} ivp_fixed_keyed_state_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Initialize with default parameters.
 * The attached object will track the reference object at the given offset. */
void ivp_fixed_keyed_init(ivp_fixed_keyed_state_t *state,
                          ivp_object_t *reference_obj,
                          ivp_object_t *attached_obj,
                          const ivp_vec3r_t *offset_ros);

/* ── Target control ────────────────────────────────────────────────────── */

/* Set target position in reference object space. */
static inline void ivp_fixed_keyed_set_target_pos(
    ivp_fixed_keyed_state_t *state, const ivp_vec3r_t *pos_ros) {
    state->target_pos_ros = *pos_ros;
}

/* Set target velocity in reference object space. */
static inline void ivp_fixed_keyed_set_target_vel(
    ivp_fixed_keyed_state_t *state, const ivp_vec3r_t *vel_ros) {
    state->target_vel_ros = *vel_ros;
}

/* Set keyframed target orientation (interpolated between two quats). */
void ivp_fixed_keyed_set_orientation(ivp_fixed_keyed_state_t *state,
                                     const ivp_quatr_t *q0,
                                     const ivp_quatr_t *q1,
                                     ivp_real_t blend);

/* Set single target orientation (no interpolation). */
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

/* ── Controller callback ───────────────────────────────────────────────── */

/* Pass ivp_fixed_keyed_state_t* as ctx. */
void ivp_controller_fixed_keyed(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONSTRAINT_FIXED_KEYED_H */
