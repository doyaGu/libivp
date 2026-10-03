/* ivp_constraint_fixed_keyed.c — Keyframed fixed constraint
 *
 * Port of IVP_Constraint_Fixed_Keyframed
 * (ivp_controller/ivp_constraint_fixed_keyed.cxx:51-232).
 */
#include <ivp/controllers/ivp_constraint_fixed_keyed.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/solver/ivp_solver_core_reaction.h>
#include "solver/ivp_scr_internal.h"
#include "controllers/ivp_controller_ref.h"
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_mat3.h>
#include "ivp_controller_ref_math.h"
#include <ivp/base/ivp_message.h>
#include <string.h>

static ivp_time_t fk_now(const ivp_fixed_keyed_state_t *state)
{
    const ivp_object_t *o = state->reference_obj ? state->reference_obj : state->attached_obj;
    return (o && o->owner) ? ivp_environment_get_time(o->owner) : (ivp_time_t)0;
}

static void fk_wake(ivp_fixed_keyed_state_t *state)
{
    ivp_controller_wake_object(state->reference_obj);
    ivp_controller_wake_object(state->attached_obj);
}

/* IVP_Real_Object::get_m_world_f_object_AT (identity for NULL) */
static void fk_world(const ivp_object_t *o, ivp_rm_m3_t rot, double pos[3])
{
    if (!o) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) rot[i][j] = (i == j) ? 1.0 : 0.0;
            pos[i] = 0.0;
        }
        return;
    }
    ivp_transform_r_t m;
    ivp_ctl_m_world_f_object_AT(o, &m);
    ivp_rm_m3_from_transform(&m, rot);
    for (int i = 0; i < 3; i++) pos[i] = (double)m.pos.k[i];
}

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_fixed_keyed_init(ivp_fixed_keyed_state_t *state,
                          ivp_object_t *reference_obj,
                          ivp_object_t *attached_obj,
                          const ivp_vec3r_t *offset_ros)
{
    memset(state, 0, sizeof(*state));

    state->reference_obj = reference_obj;
    state->attached_obj  = attached_obj;

    /* m_Ros_f_Aos = m_world_f_Ros^-1 * m_world_f_Aos */
    ivp_rm_m3_t rr, ra, rel;
    double pr[3], pa[3];
    fk_world(reference_obj, rr, pr);
    fk_world(attached_obj, ra, pa);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            rel[i][j] = rr[0][i] * ra[0][j] + rr[1][i] * ra[1][j] + rr[2][i] * ra[2][j];
    double d[3] = { pa[0] - pr[0], pa[1] - pr[1], pa[2] - pr[2] };
    double p_ros[3];
    for (int i = 0; i < 3; i++) p_ros[i] = rr[0][i] * d[0] + rr[1][i] * d[1] + rr[2][i] * d[2];

    if (offset_ros) {
        state->target_pos_ros = *offset_ros;
    } else {
        ivp_vec3r_set(&state->target_pos_ros, (ivp_real_t)p_ros[0], (ivp_real_t)p_ros[1],
                      (ivp_real_t)p_ros[2]);
    }
    ivp_vec3r_set_zero(&state->target_vel_ros);

    double q[4];
    ivp_rm_m3_to_quat(rel, q);
    ivp_rm_quatr_from_quat(q, &state->target_orient_0);
    state->target_orient_1 = state->target_orient_0;
    state->orient_keyed = false;
    state->time_of_prime_orientation_0 = 0.0;
    state->time_of_prime_position = 0.0;

    /* IVP_Template_Controller_Motion defaults */
    state->force_factor        = IVP_REAL(0.8);
    state->damp_factor         = IVP_REAL(1.0);
    state->torque_factor       = IVP_REAL(0.8);
    state->angular_damp_factor = IVP_REAL(1.0);
    ivp_vec3r_set(&state->max_translation_force, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    ivp_vec3r_set(&state->max_torque, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));

    state->active = true;
}

/* ── Target control ────────────────────────────────────────────────────── */

void ivp_fixed_keyed_set_prime_position_Ros(ivp_fixed_keyed_state_t *state,
                                            const ivp_vec3r_t *position_ros,
                                            const ivp_vec3r_t *velocity_ros,
                                            ivp_time_t time)
{
    state->time_of_prime_position = time;
    state->target_pos_ros = *position_ros;
    if (velocity_ros) {
        /* velocity_Ros is an IVP_U_Float_Point */
        ivp_vec3f_t v = ivp_ctl_f(velocity_ros);
        state->target_vel_ros = ivp_ctl_r(&v);
    } else {
        ivp_vec3r_set_zero(&state->target_vel_ros);
    }
    fk_wake(state);
}

void ivp_fixed_keyed_set_prime_orientation_Ros(ivp_fixed_keyed_state_t *state,
                                               const ivp_quatr_t *orientation0,
                                               ivp_time_t time0,
                                               const ivp_quatr_t *orientation1,
                                               ivp_real_t dt)
{
    state->target_orient_0 = *orientation0;
    if (orientation1) {
        state->target_orient_1 = *orientation1;
        if ((float)dt > P_FLOAT_EPS) {
            /* IVP_FLOAT i_delta_prime_orientation_time = 1.0f / dt (IVP_FLOAT dt) */
            state->i_delta_prime_orientation_time = (ivp_real_t)(1.0f / (float)dt);
            state->orient_keyed = true;
        } else {
            state->i_delta_prime_orientation_time = IVP_REAL(0.0);
            state->orient_keyed = false;
        }
    } else {
        state->orient_keyed = false;
    }
    state->time_of_prime_orientation_0 = time0;
    fk_wake(state);
}

void ivp_fixed_keyed_set_target_pos(ivp_fixed_keyed_state_t *state, const ivp_vec3r_t *pos_ros)
{
    ivp_vec3r_t vel = state->target_vel_ros;
    ivp_fixed_keyed_set_prime_position_Ros(state, pos_ros, &vel, fk_now(state));
}

void ivp_fixed_keyed_set_target_vel(ivp_fixed_keyed_state_t *state, const ivp_vec3r_t *vel_ros)
{
    /* re-prime the currently extrapolated position at "now" */
    ivp_time_t now = fk_now(state);
    ivp_vec3r_t pos = state->target_pos_ros;
    ivp_vec3r_add_scaled(&pos, &state->target_vel_ros,
                         (ivp_real_t)(now - state->time_of_prime_position));
    ivp_fixed_keyed_set_prime_position_Ros(state, &pos, vel_ros, now);
}

void ivp_fixed_keyed_set_orientation(ivp_fixed_keyed_state_t *state,
                                     const ivp_quatr_t *q0,
                                     const ivp_quatr_t *q1,
                                     ivp_real_t blend)
{
    double a[4], b[4], r[4];
    ivp_rm_quat_from_quatr(q0, a);
    ivp_rm_quat_from_quatr(q1, b);
    ivp_rm_interpolate_smoothly(a, b, (double)blend, r);
    ivp_quatr_t q;
    ivp_rm_quatr_from_quat(r, &q);
    state->orient_blend = blend;
    ivp_fixed_keyed_set_prime_orientation_Ros(state, &q, fk_now(state), NULL, IVP_REAL(1.0));
}

void ivp_fixed_keyed_set_orientation_single(ivp_fixed_keyed_state_t *state,
                                            const ivp_quatr_t *target_q)
{
    ivp_fixed_keyed_set_prime_orientation_Ros(state, target_q, fk_now(state), NULL, IVP_REAL(1.0));
}

int ivp_fixed_keyed_discovery(void *ctx, ivp_environment_t *env,
                              ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_fixed_keyed_state_t *s = (const ivp_fixed_keyed_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[2] = { s->reference_obj, s->attached_obj };
    return ivp_controller_discover_objects(objs, 2, out_handles, max_handles);
}

/* ── Controller callback ───────────────────────────────────────────────── */

static ivp_core_t *fk_core(ivp_object_t *o)
{
    return (!o || o->physical_core->physical_unmoveable) ? NULL : o->physical_core;
}

/* Reference IVP_Constraint_Fixed_Keyframed::do_simulation_controller
 * (ivp_constraint_fixed_keyed.cxx:100-232): AT matrices of both objects,
 * IVP_FLOAT factors, IVP_U_Float_Point angles / deltas / impulses and the
 * float IVP_Solver_Core_Reaction. */
void ivp_controller_fixed_keyed(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx)
{
    ivp_fixed_keyed_state_t *state = (ivp_fixed_keyed_state_t *)ctx;
    if (!state || !state->active || !state->attached_obj) return;

    ivp_object_t *ends[2] = { state->reference_obj, state->attached_obj };
    if (obj != ivp_controller_pick_exec_object(ends, 2)) return;

    const double d_time = (double)event_sim->delta_time;
    const double i_dt = (double)event_sim->inverse_delta_time;
    const ivp_time_t current_time = event_sim->environment
        ? ivp_environment_get_time(event_sim->environment) : (ivp_time_t)0;

    ivp_rm_m3_t m_Rws_f_Ros, m_Aws_f_Aos;
    double pos_R[3], pos_A[3];
    fk_world(state->reference_obj, m_Rws_f_Ros, pos_R);
    fk_world(state->attached_obj, m_Aws_f_Aos, pos_A);

    ivp_vec3f_t x_axis, y_axis, z_axis;
    lvf_set3(&x_axis, 1.0f, 0.0f, 0.0f);
    lvf_set3(&y_axis, 0.0f, 1.0f, 0.0f);
    lvf_set3(&z_axis, 0.0f, 0.0f, 1.0f);

    ivp_core_t *core_A = fk_core(state->reference_obj); /* reference */
    ivp_core_t *core_B = fk_core(state->attached_obj);  /* attached */

    /* orientation */
    {
        const float dt = (float)(current_time - state->time_of_prime_orientation_0);
        double q0[4], q1[4], qt[4];
        ivp_rm_quat_from_quatr(&state->target_orient_0, q0);
        if (state->orient_keyed) {
            ivp_rm_quat_from_quatr(&state->target_orient_1, q1);
            ivp_rm_interpolate_smoothly(q0, q1, ((double)dt + d_time) *
                                        (double)(float)state->i_delta_prime_orientation_time, qt);
        } else {
            for (int i = 0; i < 4; i++) qt[i] = q0[i];
        }
        ivp_rm_m3_t m_Ros_f_Rrs, m_Rws_f_Rrs, m;
        ivp_rm_quat_to_m3(qt, m_Ros_f_Rrs);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m_Rws_f_Rrs[i][j] = m_Rws_f_Ros[i][0] * m_Ros_f_Rrs[0][j] +
                                    m_Rws_f_Ros[i][1] * m_Ros_f_Rrs[1][j] +
                                    m_Rws_f_Ros[i][2] * m_Ros_f_Rrs[2][j];
        /* reference: m_Aws_f_Aos->mi2mult3(&m_Rws_f_Rrs) = Aws * Rws_f_Rrs^T */
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m[i][j] = m_Aws_f_Aos[i][0] * m_Rws_f_Rrs[j][0] +
                          m_Aws_f_Aos[i][1] * m_Rws_f_Rrs[j][1] +
                          m_Aws_f_Aos[i][2] * m_Rws_f_Rrs[j][2];
        double q[4];
        ivp_rm_m3_to_quat(m, q);
        ivp_vec3f_t drRA_rs;
        drRA_rs.k[0] = 2.0f * (float)ivp_rm_fast_asin(q[0]);
        drRA_rs.k[1] = 2.0f * (float)ivp_rm_fast_asin(q[1]);
        drRA_rs.k[2] = 2.0f * (float)ivp_rm_fast_asin(q[2]);
        if (q[3] > 0.0f) lvf_mult(&drRA_rs, (D)-1.0f);

        ivp_scr_f_t tcb;
        ivp_scr_f_init_rotation_ws(&tcb, core_B, core_A, &x_axis, &y_axis, &z_axis);
        if (!ivp_scr_f_invert_3x3(&tcb)) {
            ivp_debug_printf("do_constraint_system: Couldn't invert rot matrix!\n");
            return;
        }

        ivp_vec3f_t delta_angles, rot_impulse_ws;
        lvf_set_multiple(&delta_angles, &drRA_rs, (D)(i_dt * (double)(float)state->torque_factor));
        lvf_add_multiple(&delta_angles, &tcb.delta_velocity_ds, (D)(-(double)(float)state->angular_damp_factor));
        ivp_scr_f_vmult3(&tcb, &delta_angles, &rot_impulse_ws);

        /* limit angular impulse so per-axis torque does not exceed the configured cap */
        for (int i = 2; i >= 0; i--) {
            const float max_torque_axis = (float)fabs((double)(float)state->max_torque.k[i]);
            const float max_impulse_axis = (float)((double)max_torque_axis * d_time);
            if (rot_impulse_ws.k[i] > max_impulse_axis) {
                rot_impulse_ws.k[i] = max_impulse_axis;
            } else if (rot_impulse_ws.k[i] < -max_impulse_axis) {
                rot_impulse_ws.k[i] = -max_impulse_axis;
            }
        }
        ivp_scr_f_exert_angular_impulse_dim3(&tcb, core_B, core_A, &rot_impulse_ws);
    }

    /* position */
    {
        const float dt = (float)(current_time - state->time_of_prime_position);
        /* new_target_position_Ros.add_multiple(&prime_position_Ros, &velocity_Ros, dt + delta_time) */
        const double f = (double)dt + d_time;
        double tp_ros[3];
        for (int k = 0; k < 3; k++) {
            double a = (double)(float)state->target_vel_ros.k[k] * f;
            a += (double)state->target_pos_ros.k[k];
            tp_ros[k] = a;
        }
        /* m_Rws_f_Ros->vmult4(&new_target_position_Ros, &target_pos_Ws) */
        ivp_transform_r_t m_R;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) m_R.rot.rows[i].k[j] = (ivp_real_t)m_Rws_f_Ros[i][j];
            m_R.pos.k[i] = (ivp_real_t)pos_R[i];
        }
        ivp_vec3r_t tp_ros_r, target_pos_ws;
        lvp_set3(&tp_ros_r, (D)tp_ros[0], (D)tp_ros[1], (D)tp_ros[2]);
        lvm_vmult4(&m_R, &tp_ros_r, &target_pos_ws);

        /* delta_position_ws.inline_subtract_and_mult(&target_pos_Ws, A pos, force_factor * i_dt) */
        const double factor = (double)(float)state->force_factor * i_dt;
        ivp_vec3f_t delta_position_ws;
        for (int k = 0; k < 3; k++) {
            double a = (double)target_pos_ws.k[k] - pos_A[k];
            a *= factor;
            delta_position_ws.k[k] = (float)a;
        }
        const double damp_factor = (double)(float)state->damp_factor;
        if (state->reference_obj) {
            ivp_vec3f_t vR = ivp_core_speed_f(state->reference_obj->physical_core);
            lvf_add_multiple(&delta_position_ws, &vR, (D)damp_factor);
        }
        {
            ivp_vec3f_t vA = ivp_core_speed_f(state->attached_obj->physical_core);
            lvf_add_multiple(&delta_position_ws, &vA, (D)-damp_factor);
        }

        ivp_scr_f_t tcb;
        ivp_scr_f_init_translation_ws(&tcb, core_B, core_A, &target_pos_ws, &x_axis, &y_axis, &z_axis);
        if (!ivp_scr_f_invert_3x3(&tcb)) {
            ivp_debug_printf("do_constraint_system: Couldn't invert rot matrix!\n");
            return;
        }

        ivp_vec3f_t impulse_ws;
        ivp_scr_f_vmult3(&tcb, &delta_position_ws, &impulse_ws);

        /* limit linear impulse so per-axis force does not exceed the configured cap */
        for (int i = 2; i >= 0; i--) {
            const float max_force_axis = (float)fabs((double)(float)state->max_translation_force.k[i]);
            const float max_impulse_axis = (float)((double)max_force_axis * d_time);
            if (impulse_ws.k[i] > max_impulse_axis) {
                impulse_ws.k[i] = max_impulse_axis;
            } else if (impulse_ws.k[i] < -max_impulse_axis) {
                impulse_ws.k[i] = -max_impulse_axis;
            }
        }
        ivp_scr_f_exert_impulse_dim3(&tcb, core_B, core_A, &impulse_ws);
    }
}
