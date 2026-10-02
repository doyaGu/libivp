/* ivp_controller_motion.c -- Motion controller implementation */

#include <ivp/controllers/ivp_controller_motion.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <string.h>

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_motion_state_init(ivp_motion_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->linear_mode  = IVP_MOTION_NONE;
    state->angular_mode = IVP_MOTION_NONE;

    /* Reference IVP_Template_Controller_Motion defaults */
    state->target_q_world_f_core = ivp_quatr_identity();
    state->force_factor        = IVP_REAL(0.8);
    state->damp_factor         = IVP_REAL(1.0);
    state->torque_factor       = IVP_REAL(0.8);
    state->angular_damp_factor = IVP_REAL(1.0);
    ivp_vec3r_set(&state->max_translation_force, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    ivp_vec3r_set(&state->max_torque, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
}

void ivp_motion_state_set_target_position(ivp_motion_state_t *state,
                                          const ivp_vec3r_t *target_ws)
{
    if (!state || !target_ws) return;
    state->target_pos_ws = *target_ws;
    state->linear_mode = IVP_MOTION_TARGET;
}

void ivp_motion_state_set_target_orientation(ivp_motion_state_t *state,
                                             const ivp_quatr_t *q_world_f_core)
{
    if (!state || !q_world_f_core) return;
    state->target_q_world_f_core = *q_world_f_core;
    state->angular_mode = IVP_MOTION_TARGET;
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

    ivp_core_t *core = &obj->core;
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
        break;
    }
    case IVP_MOTION_FORCE: {
        /* F = ma → dv = F/m * dt */
        ivp_vec3r_t dv;
        ivp_vec3r_scale(&dv, &state->linear_value, core->inv_mass * dt);
        ivp_vec3r_add(&core->speed, &core->speed, &dv);
        break;
    }
    case IVP_MOTION_IMPULSE: {
        /* Apply impulse then clear */
        ivp_vec3r_t dv;
        ivp_vec3r_scale(&dv, &state->linear_value, core->inv_mass);
        ivp_vec3r_add(&core->speed, &core->speed, &dv);
        state->linear_mode = IVP_MOTION_NONE;
        ivp_vec3r_set_zero(&state->linear_value);
        break;
    }
    case IVP_MOTION_TARGET: {
        /* Reference IVP_Controller_Motion::do_simulation_controller:
         *   delta = (target - pos) * force_factor - speed * dt * damp_factor
         *   per axis: force = delta * mass / dt^2, clipped to max_translation_force
         *   speed += delta / dt */
        const ivp_vec3r_t *cur_pos = &core->m_world_f_core_last_psi.pos;
        ivp_real_t inv_dt = IVP_REAL(1.0) / dt;
        ivp_real_t qidt = inv_dt * inv_dt;
        ivp_vec3r_t delta;
        ivp_vec3r_sub(&delta, &state->target_pos_ws, cur_pos);
        ivp_vec3r_scale(&delta, &delta, state->force_factor);
        ivp_vec3r_add_scaled(&delta, &core->speed, -dt * state->damp_factor);
        for (int i = 0; i < 3; i++) {
            ivp_real_t axis_force = delta.k[i] * core->mass * qidt;
            ivp_real_t max_f = state->max_translation_force.k[i];
            if (ivp_fabs(axis_force) < max_f) continue;
            ivp_real_t clipped = max_f * core->inv_mass * dt * dt;
            delta.k[i] = (delta.k[i] < IVP_REAL(0.0)) ? -clipped : clipped;
        }
        ivp_vec3r_add_scaled(&core->speed, &delta, inv_dt);
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
        /* Reference: delta_quat = inv(target) * q_next; angles = 2*asin(xyz);
         * sign chosen by the quaternion dot product; scaled by torque_factor/dt,
         * damped by angular_damp_factor * rot_speed, clipped to max_torque. */
        ivp_real_t inv_dt = IVP_REAL(1.0) / dt;
        ivp_quatr_t target_inv = ivp_quat_conjugate(&state->target_q_world_f_core);
        ivp_quatr_t dq;
        ivp_quat_mult(&dq, &target_inv, &core->q_world_f_core_next_psi);
        ivp_real_t comps[3] = { dq.x, dq.y, dq.z };
        ivp_real_t angles[3];
        for (int i = 0; i < 3; i++) {
            ivp_real_t c = comps[i];
            if (c > IVP_REAL(1.0)) c = IVP_REAL(1.0);
            if (c < IVP_REAL(-1.0)) c = IVP_REAL(-1.0);
            angles[i] = IVP_REAL(2.0) * ivp_asin(c);
        }
        ivp_real_t sign = (ivp_quat_dot(&state->target_q_world_f_core,
                                        &core->q_world_f_core_next_psi) < IVP_REAL(0.0))
                        ? IVP_REAL(1.0) : IVP_REAL(-1.0);
        ivp_real_t change[3];
        for (int i = 0; i < 3; i++) {
            change[i] = angles[i] * sign * state->torque_factor * inv_dt
                      - (ivp_real_t)core->rot_speed.k[i] * state->angular_damp_factor;
            ivp_real_t rot_force = change[i] * inv_dt * core->rot_inertia.k[i];
            ivp_real_t max_t = state->max_torque.k[i];
            if (ivp_fabs(rot_force) >= max_t) {
                ivp_real_t clipped = max_t * core->inv_rot_inertia.k[i] * dt;
                change[i] = (rot_force < IVP_REAL(0.0)) ? -clipped : clipped;
            }
            core->rot_speed.k[i] += (float)change[i];
        }
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
