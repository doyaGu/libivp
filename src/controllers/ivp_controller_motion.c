/* ivp_controller_motion.c -- Motion controller implementation */

#include <ivp/controllers/ivp_controller_motion.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>
#include <math.h>
#include <string.h>
#include "controllers/ivp_controller_ref.h"

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_motion_state_init(ivp_motion_state_t *state)
{
    memset(state, 0, sizeof(*state));
    /* Reference IVP_Controller_Motion constructor: target_pos_ws.set_to_zero(),
     * target_q_world_f_core.init(); the controller drives toward that target
     * from the start (do_simulation_controller has no "no target" state). */
    state->linear_mode  = IVP_MOTION_TARGET;
    state->angular_mode = IVP_MOTION_TARGET;
    ivp_vec3r_set_zero(&state->target_pos_ws);

    /* Reference IVP_Template_Controller_Motion defaults */
    state->target_q_world_f_core = ivp_quatr_identity();
    state->force_factor        = IVP_REAL(0.8);
    state->damp_factor         = IVP_REAL(1.0);
    state->torque_factor       = IVP_REAL(0.8);
    state->angular_damp_factor = IVP_REAL(1.0);
    ivp_vec3r_set(&state->max_translation_force, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    ivp_vec3r_set(&state->max_torque, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
}

void ivp_motion_state_setup(ivp_motion_state_t *state, ivp_object_t *object)
{
    if (!state) return;
    state->object = object;
}

void ivp_motion_state_set_target_position(ivp_motion_state_t *state,
                                          const ivp_vec3r_t *target_ws)
{
    if (!state || !target_ws) return;
    /* Reference: if (position->quad_distance_to(&target_pos_ws) == 0) return; */
    if (state->linear_mode == IVP_MOTION_TARGET &&
        ivp_vec3r_quad_distance(target_ws, &state->target_pos_ws) == IVP_REAL(0.0)) return;
    state->target_pos_ws = *target_ws;
    state->linear_mode = IVP_MOTION_TARGET;
    ivp_controller_wake_object(state->object);
}

void ivp_motion_state_set_target_object_position(ivp_motion_state_t *state,
                                                 const ivp_object_t *obj,
                                                 const ivp_quatr_t *desired_orientation,
                                                 const ivp_vec3r_t *position_of_object_space_center_ws)
{
    if (!state || !obj || !desired_orientation || !position_of_object_space_center_ws) return;
    if (!obj->shift_core_f_object_is_zero) {
        /* m = desired_orientation; pos = center - m * shift_core_f_object */
        ivp_mat3r_t m;
        ivp_quat_to_mat3(desired_orientation, &m);
        ivp_vec3r_t shift_world_f_object, pos_ws;
        ivp_mat3r_vmult(&shift_world_f_object, &m, &obj->shift_core_f_object);
        ivp_vec3r_sub(&pos_ws, position_of_object_space_center_ws, &shift_world_f_object);
        ivp_motion_state_set_target_position(state, &pos_ws);
    } else {
        ivp_motion_state_set_target_position(state, position_of_object_space_center_ws);
    }
}

void ivp_motion_state_set_target_orientation(ivp_motion_state_t *state,
                                             const ivp_quatr_t *q_world_f_core)
{
    if (!state || !q_world_f_core) return;
    /* Reference set_target_q_world_f_core (ivp_controller_motion.cxx:180-200) */
    ivp_quatr_t q = *q_world_f_core;
    const double square = (double)q.x * (double)q.x + (double)q.y * (double)q.y +
                          (double)q.z * (double)q.z + (double)q.w * (double)q.w;
    if (square != square || !(square > (double)IVP_DOUBLE_EPS && square <= (double)IVP_DOUBLE_MAX)) {
        IVP_ASSERT(0);
        return;
    }
    /* IVP_U_Quat::normize_quat: dist = 1.0f / sqrtd(square) */
    const double dist = 1.0 / sqrt(square);
    q.x = (ivp_real_t)((double)q.x * dist);
    q.y = (ivp_real_t)((double)q.y * dist);
    q.z = (ivp_real_t)((double)q.z * dist);
    q.w = (ivp_real_t)((double)q.w * dist);
    if (state->angular_mode == IVP_MOTION_TARGET) {
        const double acos_quat = (double)q.x * (double)state->target_q_world_f_core.x +
                                 (double)q.y * (double)state->target_q_world_f_core.y +
                                 (double)q.z * (double)state->target_q_world_f_core.z +
                                 (double)q.w * (double)state->target_q_world_f_core.w;
        if (fabs(acos_quat - 1.0) < (double)IVP_DOUBLE_EPS) return;
    }
    state->target_q_world_f_core = q;
    state->angular_mode = IVP_MOTION_TARGET;
    ivp_controller_wake_object(state->object);
}

void ivp_motion_state_set_velocity(ivp_motion_state_t *state,
                                    const ivp_vec3r_t *velocity)
{
    state->linear_mode  = IVP_MOTION_VELOCITY;
    state->linear_value = *velocity;
}

void ivp_motion_state_set_force(ivp_motion_state_t *state,
                                 const ivp_vec3r_t *force)
{
    state->linear_mode  = IVP_MOTION_FORCE;
    state->linear_value = *force;
}

void ivp_motion_state_set_impulse(ivp_motion_state_t *state,
                                   const ivp_vec3r_t *impulse)
{
    state->linear_mode  = IVP_MOTION_IMPULSE;
    state->linear_value = *impulse;
}

void ivp_motion_state_set_angular_velocity(ivp_motion_state_t *state,
                                             const ivp_vec3r_t *angular_velocity)
{
    state->angular_mode  = IVP_MOTION_VELOCITY;
    state->angular_value = *angular_velocity;
}

void ivp_motion_state_set_torque(ivp_motion_state_t *state,
                                  const ivp_vec3r_t *torque)
{
    state->angular_mode  = IVP_MOTION_FORCE;
    state->angular_value = *torque;
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_motion(const ivp_event_sim_t *event_sim,
                            ivp_object_t *obj,
                            void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    ivp_motion_state_t *state = (ivp_motion_state_t *)ctx;
    if (!state) return;
    if (state->object && obj != state->object) return;

    ivp_core_t *core = obj->physical_core;
    if (core->physical_unmoveable) return;

    /* ── Linear motion ──────────────────────────────────────────────── */
    /* Reference: IVP_Controller_Motion writes DIRECTLY to core->speed
     * (sync push), not speed_change (async). See ivp_controller_motion.cxx. */
    switch (state->linear_mode) {
    case IVP_MOTION_VELOCITY: {
        /* Set target velocity directly (sync push) */
        ivp_vec3r_t delta;
        ivp_vec3r_sub(&delta, &state->linear_value, &core->speed);
        ivp_vec3r_add(&core->speed, &core->speed, &delta);
        ivp_ctl_round_speed(core);
        break;
    }
    case IVP_MOTION_FORCE: {
        /* F = ma → dv = F/m * dt */
        ivp_vec3r_t dv;
        ivp_vec3r_scale(&dv, &state->linear_value, core->inv_mass * dt);
        ivp_vec3r_add(&core->speed, &core->speed, &dv);
        ivp_ctl_round_speed(core);
        break;
    }
    case IVP_MOTION_IMPULSE: {
        /* Apply impulse then clear */
        ivp_vec3r_t dv;
        ivp_vec3r_scale(&dv, &state->linear_value, core->inv_mass);
        ivp_vec3r_add(&core->speed, &core->speed, &dv);
        ivp_ctl_round_speed(core);
        state->linear_mode = IVP_MOTION_NONE;
        ivp_vec3r_set_zero(&state->linear_value);
        break;
    }
    case IVP_MOTION_TARGET: {
        /* Reference IVP_Controller_Motion::do_simulation_controller
         * (ivp_controller_motion.cxx:53-100): IVP_U_Float_Point delta,
         * IVP_FLOAT factors / masses, IVP_DOUBLE time factors.
         *   delta = (target - pos) * force_factor - speed * dt * damp_factor
         *   per axis: force = delta * mass / dt^2, clipped to max_translation_force
         *   speed += delta / dt */
        const double delta_time = (double)dt;
        const double inv_d_time = (double)event_sim->inverse_delta_time;
        const ivp_vec3r_t *cur_pos_ws = &core->m_world_f_core_last_psi.pos;
        ivp_vec3f_t delta_position;
        const double force_factor = (double)(float)state->force_factor;
        for (int k = 0; k < 3; k++) { /* inline_subtract_and_mult (IVP_U_Point) */
            double a = (double)state->target_pos_ws.k[k] - (double)cur_pos_ws->k[k];
            a *= force_factor;
            delta_position.k[k] = (float)a;
        }
        ivp_vec3f_t speed = ivp_core_speed_f(core);
        lvf_add_multiple(&delta_position, &speed, (D)(-delta_time * (double)(float)state->damp_factor));

        const double qidt = inv_d_time * inv_d_time;
        const float mass = ivp_core_get_mass_f(core), inv_mass = ivp_core_get_inv_mass_f(core);
        for (int i = 2; i >= 0; i--) {
            const float max_tf = (float)state->max_translation_force.k[i];
            const float axis_force = (float)((double)(delta_position.k[i] * mass) * qidt);
            if (fabs((double)axis_force) < (double)max_tf) continue;
            /* clip force */
            if (delta_position.k[i] < 0) {
                delta_position.k[i] = (float)((double)(-max_tf * inv_mass) * delta_time * delta_time);
            } else {
                delta_position.k[i] = (float)((double)(max_tf * inv_mass) * delta_time * delta_time);
            }
        }
        lvf_add_multiple(&speed, &delta_position, (D)inv_d_time);
        ivp_core_set_speed_f(core, &speed);
        break;
    }
    case IVP_MOTION_NONE:
    default:
        break;
    }

    /* ── Angular motion ─────────────────────────────────────────────── */
    /* Reference: core->rot_speed.add() — direct sync push. */
    switch (state->angular_mode) {
    case IVP_MOTION_VELOCITY: {
        /* Set target angular velocity.
         * angular_value is in world space; convert to body space. */
        ivp_vec3r_t target_cs;
        ivp_mat3r_vimult(&target_cs, &core->m_world_f_core_last_psi.rot,
                         &state->angular_value);
        ivp_vec3r_t current_cs;
        current_cs.k[0] = (ivp_real_t)core->rot_speed.k[0];
        current_cs.k[1] = (ivp_real_t)core->rot_speed.k[1];
        current_cs.k[2] = (ivp_real_t)core->rot_speed.k[2];
        ivp_vec3r_t delta;
        ivp_vec3r_sub(&delta, &target_cs, &current_cs);
        core->rot_speed.k[0] += (float)delta.k[0];
        core->rot_speed.k[1] += (float)delta.k[1];
        core->rot_speed.k[2] += (float)delta.k[2];
        break;
    }
    case IVP_MOTION_FORCE: {
        /* Torque: dω = I⁻¹ * τ * dt (in body space) */
        ivp_vec3r_t torque_cs;
        ivp_mat3r_vimult(&torque_cs, &core->m_world_f_core_last_psi.rot,
                         &state->angular_value);
        ivp_vec3r_t dw;
        dw.k[0] = torque_cs.k[0] * core->inv_rot_inertia.k[0] * dt;
        dw.k[1] = torque_cs.k[1] * core->inv_rot_inertia.k[1] * dt;
        dw.k[2] = torque_cs.k[2] * core->inv_rot_inertia.k[2] * dt;
        core->rot_speed.k[0] += (float)dw.k[0];
        core->rot_speed.k[1] += (float)dw.k[1];
        core->rot_speed.k[2] += (float)dw.k[2];
        break;
    }
    case IVP_MOTION_IMPULSE: {
        ivp_vec3r_t imp_cs;
        ivp_mat3r_vimult(&imp_cs, &core->m_world_f_core_last_psi.rot,
                         &state->angular_value);
        ivp_vec3r_t dw;
        dw.k[0] = imp_cs.k[0] * core->inv_rot_inertia.k[0];
        dw.k[1] = imp_cs.k[1] * core->inv_rot_inertia.k[1];
        dw.k[2] = imp_cs.k[2] * core->inv_rot_inertia.k[2];
        core->rot_speed.k[0] += (float)dw.k[0];
        core->rot_speed.k[1] += (float)dw.k[1];
        core->rot_speed.k[2] += (float)dw.k[2];
        state->angular_mode = IVP_MOTION_NONE;
        ivp_vec3r_set_zero(&state->angular_value);
        break;
    }
    case IVP_MOTION_TARGET: {
        /* Reference (ivp_controller_motion.cxx:102-137):
         *   delta_quat = inv(target) * q_next; angles = 2*asin(xyz)  (float)
         *   sign chosen by the quaternion dot product; scaled by
         *   torque_factor/dt, damped by angular_damp_factor * rot_speed,
         *   clipped to max_torque, rot_speed += change (float). */
        const double delta_time = (double)dt;
        const double inv_d_time = (double)event_sim->inverse_delta_time;
        ivp_quatr_t target_q_core_f_world = ivp_quat_conjugate(&state->target_q_world_f_core);
        ivp_quatr_t dq;
        ivp_quat_mult(&dq, &target_q_core_f_world, &core->q_world_f_core_next_psi);
        const double comps[3] = { (double)dq.x, (double)dq.y, (double)dq.z };
        ivp_vec3f_t delta_angles_now_cs;
        for (int i = 0; i < 3; i++) { /* IVP_U_Quat::get_angles */
            double c = comps[i];
            if (c > 1.0) c = 1.0;
            if (c < -1.0) c = -1.0;
            delta_angles_now_cs.k[i] = (float)(2.0 * asin(c));
        }
        const double torque_factor = (double)(float)state->torque_factor;
        const double acos_quat = (double)state->target_q_world_f_core.x * (double)core->q_world_f_core_next_psi.x +
                                 (double)state->target_q_world_f_core.y * (double)core->q_world_f_core_next_psi.y +
                                 (double)state->target_q_world_f_core.z * (double)core->q_world_f_core_next_psi.z +
                                 (double)state->target_q_world_f_core.w * (double)core->q_world_f_core_next_psi.w;
        if (acos_quat < 0) {
            lvf_mult(&delta_angles_now_cs, (D)(torque_factor * inv_d_time));
        } else {
            lvf_mult(&delta_angles_now_cs, (D)(-torque_factor * inv_d_time));
        }

        ivp_vec3f_t rot_speed_change_cs;
        lvf_add_multiple2(&rot_speed_change_cs, &delta_angles_now_cs, &core->rot_speed,
                          (D)(-(double)(float)state->angular_damp_factor));
        const ivp_vec3f_t ri = ivp_core_rot_inertia_f(core), iri = ivp_core_inv_rot_inertia_f(core);
        for (int i = 2; i >= 0; --i) {
            const float max_t = (float)state->max_torque.k[i];
            const float rot_force = (float)((double)rot_speed_change_cs.k[i] * inv_d_time * (double)ri.k[i]);
            if (fabs((double)rot_force) < (double)max_t) continue;
            /* clip rot change so that resulting rot force doesn't exceed max rot force */
            if (rot_force < 0) {
                rot_speed_change_cs.k[i] = (float)((double)(-max_t * iri.k[i]) * delta_time);
            } else {
                rot_speed_change_cs.k[i] = (float)((double)(max_t * iri.k[i]) * delta_time);
            }
        }
        lvf_add(&core->rot_speed, &rot_speed_change_cs);
        break;
    }
    case IVP_MOTION_NONE:
    default:
        break;
    }

    /* ── Speed limiting ─────────────────────────────────────────────── */
    if (state->max_speed > IVP_REAL(0.0)) {
        ivp_real_t spd = ivp_vec3r_length(&core->speed);
        if (spd > state->max_speed) {
            ivp_real_t scale = state->max_speed / spd;
            ivp_vec3r_scale(&core->speed, &core->speed, scale);
            ivp_ctl_round_speed(core);
        }
    }

    if (state->max_angular_speed > IVP_REAL(0.0)) {
        ivp_real_t aspd = ivp_sqrt(
            (ivp_real_t)core->rot_speed.k[0] * (ivp_real_t)core->rot_speed.k[0] +
            (ivp_real_t)core->rot_speed.k[1] * (ivp_real_t)core->rot_speed.k[1] +
            (ivp_real_t)core->rot_speed.k[2] * (ivp_real_t)core->rot_speed.k[2]);
        if (aspd > state->max_angular_speed) {
            float scale = (float)(state->max_angular_speed / aspd);
            core->rot_speed.k[0] *= scale;
            core->rot_speed.k[1] *= scale;
            core->rot_speed.k[2] *= scale;
        }
    }
}
