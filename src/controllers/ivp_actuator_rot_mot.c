/* ivp_actuator_rot_mot.c -- Rotation motor actuator implementation
 *
 * Port of IVP_Actuator_Rot_Mot from the legacy SDK.
 *
 * The motor applies a torque along a defined axis.  Torque decreases
 * linearly as angular speed approaches max_rotation_speed, simulating
 * an automatic-gear motor.
 *
 *   effective_torque = min(power / (|omega| + eps), max_torque)
 *                    * (1.0 - |omega| / max_rotation_speed)
 *
 * Clamped so torque is zero once max_rotation_speed is reached and
 * negative (braking) if exceeded.
 */
#include <ivp/controllers/ivp_actuator_rot_mot.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_mat3.h>

#include <string.h>

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_rot_mot_state_init(ivp_rot_mot_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->active = true;
}

void ivp_rot_mot_state_setup(ivp_rot_mot_state_t *state,
                              ivp_object_t *obj_a,
                              const ivp_vec3r_t *anchor_a_os,
                              ivp_object_t *obj_b,
                              const ivp_vec3r_t *anchor_b_os,
                              ivp_real_t power,
                              ivp_real_t max_torque,
                              ivp_real_t max_rotation_speed)
{
    memset(state, 0, sizeof(*state));
    state->obj_a = obj_a;
    state->obj_b = obj_b;
    state->anchor_a_os = *anchor_a_os;
    state->anchor_b_os = *anchor_b_os;
    state->power = power;
    state->max_torque = max_torque;
    state->max_rotation_speed = max_rotation_speed;
    state->active = true;
}

void ivp_rot_mot_set_power(ivp_rot_mot_state_t *state, ivp_real_t power)
{
    state->power = power;
}

void ivp_rot_mot_set_max_torque(ivp_rot_mot_state_t *state, ivp_real_t max_torque)
{
    state->max_torque = max_torque;
}

void ivp_rot_mot_set_max_rotation_speed(ivp_rot_mot_state_t *state, ivp_real_t max_speed)
{
    state->max_rotation_speed = max_speed;
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_rot_mot(const ivp_event_sim_t *event_sim,
                               ivp_object_t *obj,
                               void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;

    ivp_rot_mot_state_t *state = (ivp_rot_mot_state_t *)ctx;
    if (!state->active || !state->obj_b) return;
    if (dt < IVP_REAL_EPS) return;

    /* Environment dispatches controllers once per active object.
     * Run only when called for the driven object. */
    if (obj != state->obj_b) return;

    ivp_core_t *cb = &state->obj_b->core;
    if (cb->physical_unmoveable) return;

    /* Compute rotation axis in world space from the two anchor points.
     * The anchors define the axis orientation, not position.
     * If obj_a is non-NULL, transform anchors from A's body space. */
    ivp_vec3r_t pa_ws, pb_ws;
    ivp_controller_get_anchor_ws(state->obj_a, &state->anchor_a_os, &pa_ws);
    ivp_controller_get_anchor_ws(state->obj_a, &state->anchor_b_os, &pb_ws);

    ivp_vec3r_t axis_ws;
    ivp_vec3r_sub(&axis_ws, &pb_ws, &pa_ws);
    ivp_real_t axis_len = ivp_vec3r_length(&axis_ws);
    if (axis_len < IVP_REAL_EPS) return;

    ivp_vec3r_scale(&axis_ws, &axis_ws, IVP_REAL(1.0) / axis_len);

    /* Current angular velocity of B projected onto the rotation axis.
     * rot_speed is in core space — transform to world space. */
    ivp_vec3r_t omega_ws;
    ivp_vec3r_t rot_speed_r;
    rot_speed_r.k[0] = (ivp_real_t)cb->rot_speed.k[0];
    rot_speed_r.k[1] = (ivp_real_t)cb->rot_speed.k[1];
    rot_speed_r.k[2] = (ivp_real_t)cb->rot_speed.k[2];
    ivp_transform_dir(&omega_ws, &cb->m_world_f_core_last_psi, &rot_speed_r);

    ivp_real_t omega_on_axis = ivp_vec3r_dot(&omega_ws, &axis_ws);
    state->current_speed = omega_on_axis;

    /* Motor torque: power-based with speed limit.
     * torque = min(power / (|omega| + eps), max_torque)
     * Then linearly reduce as omega approaches max_rotation_speed. */
    ivp_real_t abs_omega = ivp_fabs(omega_on_axis);
    ivp_real_t torque_from_power = state->power / (abs_omega + IVP_REAL(0.01));
    ivp_real_t torque = torque_from_power;

    if (torque > state->max_torque) {
        torque = state->max_torque;
    }

    /* Linear ramp-down: at max speed, torque → 0; beyond, braking */
    if (state->max_rotation_speed > IVP_REAL_EPS) {
        ivp_real_t speed_ratio = omega_on_axis / state->max_rotation_speed;
        torque *= (IVP_REAL(1.0) - speed_ratio);
    }

    state->current_torque = torque;

    if (ivp_fabs(torque) < IVP_REAL_EPS) return;

    /* Convert torque to angular velocity change: dω = (τ / I_axis) * dt
     * I_axis = axis^T * I * axis  (moment of inertia about rotation axis)
     * But we have inv_rot_inertia in core space, so:
     * dω_core = inv_I_core * (R^T * axis_ws * torque) * dt */
    ivp_vec3r_t torque_ws;
    ivp_vec3r_scale(&torque_ws, &axis_ws, torque);

    /* Transform torque to core space */
    ivp_vec3r_t torque_cs;
    ivp_mat3r_vimult(&torque_cs, &cb->m_world_f_core_last_psi.rot, &torque_ws);

    /* Angular acceleration in core space: alpha_cs = inv_I * torque_cs */
    ivp_vec3r_t d_omega_cs;
    d_omega_cs.k[0] = torque_cs.k[0] * cb->inv_rot_inertia.k[0] * dt;
    d_omega_cs.k[1] = torque_cs.k[1] * cb->inv_rot_inertia.k[1] * dt;
    d_omega_cs.k[2] = torque_cs.k[2] * cb->inv_rot_inertia.k[2] * dt;

    /* Apply */
    cb->rot_speed.k[0] += (float)d_omega_cs.k[0];
    cb->rot_speed.k[1] += (float)d_omega_cs.k[1];
    cb->rot_speed.k[2] += (float)d_omega_cs.k[2];
}
