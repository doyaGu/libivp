/* ivp_actuator_rot_mot.h -- Rotation motor actuator
 *
 * A rotation motor applies torque to spin an object around a defined axis.
 * Like a real motor, it has adjustable power, max torque, and max speed.
 * Torque decreases linearly as rotation speed approaches max_rotation_speed
 * (automatic gear behaviour).
 *
 * Ported from IVP_Actuator_Rot_Mot (ivp_actuator.hxx).
 */
#ifndef IVP_ACTUATOR_ROT_MOT_H
#define IVP_ACTUATOR_ROT_MOT_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Rotation motor state ───────────────────────────────────────────────── */

typedef struct ivp_rot_mot_state {
    /* Objects: the motor spins obj_b relative to obj_a (or world if NULL) */
    ivp_object_t *obj_a;            /* reference object (NULL = world) */
    ivp_object_t *obj_b;            /* driven object */

    /* Axis definition via two anchor points (world-relative direction) */
    ivp_vec3r_t anchor_a_os;        /* first anchor point in A/world space */
    ivp_vec3r_t anchor_b_os;        /* second anchor point in A/world space */
    /* The rotation axis = normalize(anchor_b - anchor_a).
     * The object rotates around its own centre of mass on this axis. */

    /* Motor parameters */
    ivp_real_t power;               /* motor power (watts) -- drives torque */
    ivp_real_t max_torque;          /* maximum torque (N·m) */
    ivp_real_t max_rotation_speed;  /* speed limit (rad/s) */

    /* Runtime state */
    ivp_real_t current_speed;       /* current angular speed on axis */
    ivp_real_t current_torque;      /* current applied torque */
    bool       active;
} ivp_rot_mot_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_rot_mot_state_init(ivp_rot_mot_state_t *state);

void ivp_rot_mot_state_setup(ivp_rot_mot_state_t *state,
                              ivp_object_t *obj_a,
                              const ivp_vec3r_t *anchor_a_os,
                              ivp_object_t *obj_b,
                              const ivp_vec3r_t *anchor_b_os,
                              ivp_real_t power,
                              ivp_real_t max_torque,
                              ivp_real_t max_rotation_speed);

/* Runtime adjustment */
void ivp_rot_mot_set_power(ivp_rot_mot_state_t *state, ivp_real_t power);
void ivp_rot_mot_set_max_torque(ivp_rot_mot_state_t *state, ivp_real_t max_torque);
void ivp_rot_mot_set_max_rotation_speed(ivp_rot_mot_state_t *state, ivp_real_t max_speed);

/* ── Controller callback ────────────────────────────────────────────────── */

/* Rotation motor controller. Pass ivp_rot_mot_state_t* as ctx. */
void ivp_controller_rot_mot(const ivp_event_sim_t *event_sim,
                              ivp_object_t *obj,
                              void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_ROT_MOT_H */
