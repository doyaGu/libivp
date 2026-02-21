/* ivp_actuator_torque.c -- Pure torque actuator implementation
 *
 * Port of IVP_Actuator_Torque from the legacy SDK.
 *
 * Applies a constant torque along a specified axis in object space.
 * Respects an optional angular speed limit: once the object's angular
 * velocity on the axis reaches max_rotation_speed, no more torque is applied.
 *
 * Algorithm:
 *   1. Transform axis from object space to world space via rotation matrix
 *   2. Project current angular velocity onto axis
 *   3. If |omega| >= max_rotation_speed (and positive torque direction), skip
 *   4. Apply angular impulse: d_omega = I^-1 * (R^T * axis_ws * torque) * dt
 */
#include <ivp/controllers/ivp_actuator_torque.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_mat3.h>

#include <string.h>

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_torque_actuator_state_init(ivp_torque_actuator_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->active = true;
}

void ivp_torque_actuator_state_setup(ivp_torque_actuator_state_t *state,
                                      ivp_object_t *obj,
                                      const ivp_vec3r_t *axis_os,
                                      ivp_real_t torque,
                                      ivp_real_t max_rotation_speed)
{
    memset(state, 0, sizeof(*state));
    state->obj = obj;
    state->axis_os = *axis_os;
    state->torque = torque;
    state->max_rotation_speed = max_rotation_speed;
    state->active = true;
}

void ivp_torque_actuator_set_torque(ivp_torque_actuator_state_t *state,
                                     ivp_real_t torque)
{
    state->torque = torque;
}

void ivp_torque_actuator_set_max_rotation_speed(ivp_torque_actuator_state_t *state,
                                                 ivp_real_t max_speed)
{
    state->max_rotation_speed = max_speed;
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_torque_actuator(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;

    ivp_torque_actuator_state_t *state = (ivp_torque_actuator_state_t *)ctx;
    if (!state->active || !state->obj) return;
    if (dt < IVP_REAL_EPS) return;
    if (ivp_fabs(state->torque) < IVP_REAL_EPS) return;

    /* Only execute when called for the target object */
    if (obj != state->obj) return;

    ivp_core_t *core = &state->obj->core;
    if (core->physical_unmoveable) return;

    /* 1. Transform axis from object space to world space */
    ivp_vec3r_t axis_ws;
    ivp_transform_dir(&axis_ws, &core->m_world_f_core_last_psi, &state->axis_os);

    ivp_real_t axis_len = ivp_vec3r_length(&axis_ws);
    if (axis_len < IVP_REAL_EPS) return;
    ivp_vec3r_scale(&axis_ws, &axis_ws, IVP_REAL(1.0) / axis_len);

    /* 2. Project current angular velocity onto axis */
    ivp_vec3r_t omega_cs;
    omega_cs.k[0] = (ivp_real_t)core->rot_speed.k[0];
    omega_cs.k[1] = (ivp_real_t)core->rot_speed.k[1];
    omega_cs.k[2] = (ivp_real_t)core->rot_speed.k[2];

    ivp_vec3r_t omega_ws;
    ivp_mat3r_vmult(&omega_ws, &core->m_world_f_core_last_psi.rot, &omega_cs);

    ivp_real_t omega_on_axis = ivp_vec3r_dot(&omega_ws, &axis_ws);
    state->current_speed = omega_on_axis;

    /* 3. Speed limiter: if already at or beyond max speed in the torque
     *    direction, don't apply more torque. */
    if (state->max_rotation_speed > IVP_REAL_EPS) {
        if (state->torque > IVP_REAL(0.0) && omega_on_axis >= state->max_rotation_speed) return;
        if (state->torque < IVP_REAL(0.0) && omega_on_axis <= -state->max_rotation_speed) return;
    }

    /* 4. Apply angular impulse: d_omega = I^-1 * (R^T * axis_ws * torque) * dt */
    ivp_vec3r_t torque_ws;
    ivp_vec3r_scale(&torque_ws, &axis_ws, state->torque);

    ivp_vec3r_t torque_cs;
    ivp_mat3r_vimult(&torque_cs, &core->m_world_f_core_last_psi.rot, &torque_ws);

    ivp_vec3r_t d_omega_cs;
    d_omega_cs.k[0] = torque_cs.k[0] * core->inv_rot_inertia.k[0] * dt;
    d_omega_cs.k[1] = torque_cs.k[1] * core->inv_rot_inertia.k[1] * dt;
    d_omega_cs.k[2] = torque_cs.k[2] * core->inv_rot_inertia.k[2] * dt;

    /* Async push — priority 1500 (above gravity barrier at 1000) */
    core->rot_speed_change.k[0] += (float)d_omega_cs.k[0];
    core->rot_speed_change.k[1] += (float)d_omega_cs.k[1];
    core->rot_speed_change.k[2] += (float)d_omega_cs.k[2];
}
