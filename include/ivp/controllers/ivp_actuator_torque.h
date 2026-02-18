/* ivp_actuator_torque.h -- Pure torque actuator
 *
 * Applies a constant torque around a specified axis in object space.
 * The torque spins the object up to an optional speed limit.
 *
 * Distinct from the rotation motor (ivp_actuator_rot_mot), which derives
 * torque from a power curve. This actuator applies torque directly.
 *
 * Ported from IVP_Actuator_Torque (legacy SDK).
 */
#ifndef IVP_ACTUATOR_TORQUE_H
#define IVP_ACTUATOR_TORQUE_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Torque actuator state ─────────────────────────────────────────────── */

typedef struct ivp_torque_actuator_state {
    ivp_object_t *obj;              /* object to torque */
    ivp_vec3r_t   axis_os;          /* rotation axis in object space (unit) */
    ivp_real_t    torque;           /* constant torque (N*m) */
    ivp_real_t    max_rotation_speed; /* speed limit (rad/s), 0 = unlimited */

    /* Runtime state */
    ivp_real_t    current_speed;    /* current omega projected on axis */
    bool          active;
} ivp_torque_actuator_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_torque_actuator_state_init(ivp_torque_actuator_state_t *state);

void ivp_torque_actuator_state_setup(ivp_torque_actuator_state_t *state,
                                      ivp_object_t *obj,
                                      const ivp_vec3r_t *axis_os,
                                      ivp_real_t torque,
                                      ivp_real_t max_rotation_speed);

/* Runtime adjustment */
void ivp_torque_actuator_set_torque(ivp_torque_actuator_state_t *state,
                                     ivp_real_t torque);

void ivp_torque_actuator_set_max_rotation_speed(ivp_torque_actuator_state_t *state,
                                                 ivp_real_t max_speed);

/* ── Controller callback ────────────────────────────────────────────────── */

/* Pure torque controller. Pass ivp_torque_actuator_state_t* as ctx. */
void ivp_controller_torque_actuator(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_TORQUE_H */
