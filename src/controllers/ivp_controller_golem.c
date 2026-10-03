/* ivp_controller_golem.c — Animation-driven physics controller
 *
 * Port of IVP_Controller_Golem (ivp_controller/ivp_controller_motion.cxx:206-399).
 */

#include <ivp/controllers/ivp_controller_golem.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>
#include <math.h>
#include <string.h>
#include "controllers/ivp_controller_ref.h"

void ivp_golem_state_init(ivp_golem_state_t *state)
{
    memset(state, 0, sizeof(*state));
    /* IVP_Template_Controller_Motion (ivp_controller_motion.cxx:13-22) */
    state->force_factor        = IVP_REAL(0.8);
    state->damp_factor         = IVP_REAL(1.0);
    state->torque_factor       = IVP_REAL(0.8);
    state->angular_damp_factor = IVP_REAL(1.0);
    ivp_vec3r_set(&state->max_translation_force, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    ivp_vec3r_set(&state->max_torque, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    /* IVP_Template_Controller_Golem (ivp_controller_motion.cxx:206-213) */
    state->max_delta_position            = IVP_REAL(10.0);
    state->max_delta_orientation         = IVP_REAL(0.5) * IVP_PI;
    state->max_integrated_delta_position = IVP_REAL(10.0);
    state->max_golem_force               = (ivp_real_t)IVP_FLOAT_MAX;
    state->filter_dtime                  = IVP_REAL(5.0);

    state->target_position     = ivp_vec3r_zero();
    state->velocity_ws         = ivp_vec3r_zero();
    state->target_orientation  = ivp_quatr_identity();
    state->prime_orientation_1 = ivp_quatr_identity();
    state->target_q_world_f_core = ivp_quatr_identity();
    state->last_problem = IVP_GP_NONE;
}

static ivp_time_t golem_now(const ivp_object_t *obj)
{
    return (obj && obj->owner) ? ivp_environment_get_time(obj->owner) : (ivp_time_t)0;
}

static void golem_prime_from_object(ivp_golem_state_t *state, ivp_object_t *obj)
{
    ivp_time_t now = golem_now(obj);
    if (!state->target_position_set) {
        state->target_position = obj->physical_core->m_world_f_core_last_psi.pos;
        state->velocity_ws = ivp_vec3r_zero();
        state->time_of_prime_position = now;
        state->target_position_set = true;
    }
    if (!state->target_orientation_set) {
        state->target_orientation = obj->physical_core->q_world_f_core_next_psi;
        state->prime_orientation_1 = state->target_orientation;
        state->angular_velocity_set = false;
        state->i_delta_prime_orientation_time = IVP_REAL(0.0);
        state->time_of_prime_orientation_0 = now;
        state->target_orientation_set = true;
    }
}

void ivp_golem_state_setup(ivp_golem_state_t *state, ivp_object_t *object)
{
    state->object = object;
    state->integrated_delta_position = IVP_REAL(0.0);
    state->target_position_set = false;
    state->target_orientation_set = false;
    if (object) golem_prime_from_object(state, object);
}

void ivp_golem_set_prime_position(ivp_golem_state_t *state,
                                  const ivp_vec3r_t *position,
                                  const ivp_vec3r_t *velocity,
                                  ivp_time_t time)
{
    state->time_of_prime_position = time;
    state->target_position = *position;
    state->velocity_ws = velocity ? *velocity : ivp_vec3r_zero();
    state->target_position_set = true;
    ivp_controller_wake_object(state->object);
}

void ivp_golem_set_prime_orientation(ivp_golem_state_t *state,
                                     const ivp_quatr_t *orientation0,
                                     ivp_time_t time0,
                                     const ivp_quatr_t *orientation1,
                                     ivp_real_t dt)
{
    state->target_orientation = *orientation0;
    if (orientation1 && dt > (ivp_real_t)IVP_FLOAT_EPS) {
        state->prime_orientation_1 = *orientation1;
        state->angular_velocity_set = true;
        /* IVP_FLOAT i_delta_prime_orientation_time = 1.0f / dt (IVP_FLOAT dt) */
        state->i_delta_prime_orientation_time = (ivp_real_t)(1.0f / (float)dt);
    } else {
        if (orientation1) {
            IVP_ASSERT(0);
        }
        state->prime_orientation_1 = state->target_orientation;
        state->angular_velocity_set = false;
        state->i_delta_prime_orientation_time = IVP_REAL(0.0);
    }
    state->time_of_prime_orientation_0 = time0;
    state->target_orientation_set = true;
    ivp_controller_wake_object(state->object);
}

void ivp_golem_reset_time(ivp_golem_state_t *state, ivp_time_t offset)
{
    /* Reference IVP_Controller_Golem::reset_time: only the prime position
     * time is shifted. */
    if (!state) return;
    state->time_of_prime_position -= offset;
}

void ivp_golem_set_target_position(ivp_golem_state_t *state,
                                   const ivp_vec3r_t *pos)
{
    ivp_golem_set_prime_position(state, pos, NULL, golem_now(state->object));
}

void ivp_golem_set_target_orientation(ivp_golem_state_t *state,
                                      const ivp_quatr_t *orientation)
{
    ivp_golem_set_prime_orientation(state, orientation, golem_now(state->object), NULL,
                                    IVP_REAL(0.0));
}

/* IVP_Controller_Golem::beam_object_to_target_position:
 * real_object->beam_object_to_new_position(&target_q_world_f_core,
 * &target_pos_ws, IVP_TRUE) -- the targets of the current PSI are the
 * object frame (core shift and q_core_f_object applied by the beam). */
void ivp_golem_beam_object_to_target_position(ivp_golem_state_t *state,
                                              ivp_object_t *object)
{
    ivp_object_t *o = object ? object : state->object;
    if (!o || !o->owner) return;
    ivp_object_beam_to_new_position(o, &state->target_q_world_f_core, &state->target_pos_ws, true);
}

/* Reference IVP_U_Quat::set_interpolate_smoothly (ivu_quat.cxx:293-345). */
static void golem_interpolate_smoothly(ivp_quatr_t *res, const ivp_quatr_t *from,
                                       const ivp_quatr_t *to, double t)
{
    double cosom = (double)from->x * (double)to->x + (double)from->y * (double)to->y +
                   (double)from->z * (double)to->z + (double)from->w * (double)to->w;
    double sign;
    if (cosom > 0.0) {
        sign = 1.0;
    } else {
        cosom = -cosom;
        sign = -1.0;
    }
    if (cosom < (double)(1.0f - 0.001f)) { /* IVP_QUAT_DELTA, float expression */
        double omega = acos(cosom);
        double i_sinom = 1.0 / sqrt(1.0 - cosom * cosom);
        double scale0 = sin((1.0 - t) * omega) * i_sinom;
        double scale1 = sin(t * omega) * i_sinom * sign;
        res->x = (ivp_real_t)(scale0 * (double)from->x + scale1 * (double)to->x);
        res->y = (ivp_real_t)(scale0 * (double)from->y + scale1 * (double)to->y);
        res->z = (ivp_real_t)(scale0 * (double)from->z + scale1 * (double)to->z);
        res->w = (ivp_real_t)(scale0 * (double)from->w + scale1 * (double)to->w);
    } else {
        double x = (double)from->x + t * (sign * (double)to->x - (double)from->x);
        double y = (double)from->y + t * (sign * (double)to->y - (double)from->y);
        double z = (double)from->z + t * (sign * (double)to->z - (double)from->z);
        double w = (double)from->w + t * (sign * (double)to->w - (double)from->w);
        /* normize_correct_step(3) */
        double square = (x * x + y * y + z * z + w * w) * 0.5;
        double factor = 1.5 - square;
        factor += 0.5 - (factor * factor * square);
        factor += 0.5 - (factor * factor * square);
        res->x = (ivp_real_t)(x * factor);
        res->y = (ivp_real_t)(y * factor);
        res->z = (ivp_real_t)(z * factor);
        res->w = (ivp_real_t)(w * factor);
    }
}

static void golem_problem(ivp_golem_state_t *state, ivp_object_t *obj,
                          ivp_golem_problem_t problem,
                          ivp_real_t pos_error, ivp_real_t rot_error)
{
    state->last_problem = problem;
    if (state->problem_fn) state->problem_fn(state->problem_ctx, obj, pos_error, rot_error);
}

void ivp_controller_golem(const ivp_event_sim_t *event_sim,
                          ivp_object_t *obj,
                          void *ctx)
{
    ivp_golem_state_t *state = (ivp_golem_state_t *)ctx;
    if (!state || !obj) return;
    if (state->object && obj != state->object) return;

    ivp_core_t *core = ivp_object_get_core(obj);
    if (!core || core->physical_unmoveable) return;

    const ivp_real_t dt = event_sim->delta_time;
    const ivp_real_t inv_d_time = event_sim->inverse_delta_time;
    if (dt <= IVP_REAL(0.0)) return;

    if (!state->target_position_set || !state->target_orientation_set) {
        golem_prime_from_object(state, obj);
    }
    state->last_problem = IVP_GP_NONE;

    const ivp_time_t current_time = event_sim->environment
        ? ivp_environment_get_time(event_sim->environment) : (ivp_time_t)0;

    /* Reference IVP_Controller_Golem::do_simulation_controller
     * (ivp_controller_motion.cxx:252-370): IVP_U_Float_Point delta /
     * velocity / angles, IVP_FLOAT factors, IVP_DOUBLE time factors. */
    const double delta_time = (double)dt;
    const double inv_dt = (double)inv_d_time;

    /* position: target_pos_ws.add_multiple(&prime_position_ws, &velocity_ws, dt) */
    {
        const float pdt = (float)(current_time - state->time_of_prime_position);
        for (int k = 0; k < 3; k++) {
            double a = (double)(float)state->velocity_ws.k[k] * (double)pdt;
            a += (double)state->target_position.k[k];
            state->target_pos_ws.k[k] = (ivp_real_t)a;
        }
    }

    /* orientation */
    if (!state->angular_velocity_set) {
        state->target_q_world_f_core = state->target_orientation;
    } else {
        /* IVP_FLOAT dt; set_interpolate_smoothly(.., dt * i_delta_prime_orientation_time) */
        const float odt = (float)(current_time - state->time_of_prime_orientation_0);
        golem_interpolate_smoothly(&state->target_q_world_f_core, &state->target_orientation,
                                   &state->prime_orientation_1,
                                   (double)(odt * (float)state->i_delta_prime_orientation_time));
    }

    /* ── Translation ─────────────────────────────────────────────────── */
    const ivp_transform_r_t *m_world_f_core = &core->m_world_f_core_last_psi;
    ivp_vec3f_t delta_position;
    lvf_subtract2_p(&delta_position, &state->target_pos_ws, &m_world_f_core->pos);

    /* the target is the object origin: remove the core/object shift
     * (ivp_controller_motion.cxx:278-283) */
    if (!obj->shift_core_f_object_is_zero) {
        ivp_vec3f_t shift = ivp_ctl_f(&obj->shift_core_f_object), shift_cs_os_ws;
        lvm_vmult3_f(m_world_f_core, &shift, &shift_cs_os_ws);
        lvf_subtract(&delta_position, &shift_cs_os_ws);
    }

    const double qdist = (double)lvf_quad_length(&delta_position);
    state->integrated_delta_position =
        (ivp_real_t)(float)((double)((float)state->integrated_delta_position * 0.9f) + qdist);

    const float max_delta_position = (float)state->max_delta_position;
    if (qdist > (double)(max_delta_position * max_delta_position)) {
        golem_problem(state, obj, IVP_GP_FAR_DISTANCE, (ivp_real_t)sqrt(qdist), IVP_REAL(0.0));
        return; /* reference: resolve_for_problem(); return; */
    }

    /* extrapolate position (with velocity feed-forward) */
    const double damp_factor = (double)(float)state->damp_factor;
    lvf_mult(&delta_position, (D)(double)(float)state->force_factor);
    {
        ivp_vec3f_t velocity_ws = ivp_ctl_f(&state->velocity_ws);
        lvf_add_multiple(&delta_position, &velocity_ws, (D)(delta_time * damp_factor));
    }
    ivp_vec3f_t speed = ivp_core_speed_f(core);
    lvf_add_multiple(&delta_position, &speed, (D)(-delta_time * damp_factor));

    {
        const double qidt = inv_dt * inv_dt;
        const float mass = ivp_core_get_mass_f(core), inv_mass = ivp_core_get_inv_mass_f(core);
        for (int i = 2; i >= 0; i--) {
            const float max_tf = (float)state->max_translation_force.k[i];
            const float axis_force = (float)((double)(delta_position.k[i] * mass) * qidt);
            if (fabs((double)axis_force) < (double)max_tf) continue;
            if (delta_position.k[i] < 0) {
                delta_position.k[i] = (float)((double)(-max_tf * inv_mass) * delta_time * delta_time);
            } else {
                delta_position.k[i] = (float)((double)(max_tf * inv_mass) * delta_time * delta_time);
            }
        }
        lvf_add_multiple(&speed, &delta_position, (D)inv_dt);
        ivp_core_set_speed_f(core, &speed);
    }

    /* ── Rotation ────────────────────────────────────────────────────── */
    {
        ivp_quatr_t target_q_core_f_world = ivp_quat_conjugate(&state->target_q_world_f_core);
        ivp_quatr_t dq;
        ivp_quat_mult(&dq, &target_q_core_f_world, &core->q_world_f_core_next_psi);

        /* get_angles: (float)(2 * asin(x/y/z)) */
        const double comps[3] = { (double)dq.x, (double)dq.y, (double)dq.z };
        ivp_vec3f_t delta_angles_now_cs;
        for (int i = 0; i < 3; i++) {
            double c = comps[i];
            if (c > 1.0) c = 1.0;
            if (c < -1.0) c = -1.0;
            delta_angles_now_cs.k[i] = (float)(2.0 * asin(c));
        }
        const double qangle = (double)lvf_quad_length(&delta_angles_now_cs);
        const float max_delta_orientation = (float)state->max_delta_orientation;
        if (qangle > (double)(max_delta_orientation * max_delta_orientation)) {
            golem_problem(state, obj, IVP_GP_BIG_ANGLE, IVP_REAL(0.0), (ivp_real_t)sqrt(qangle));
            return;
        }

        const double torque_factor = (double)(float)state->torque_factor;
        const double acos_quat = (double)state->target_q_world_f_core.x * (double)core->q_world_f_core_next_psi.x +
                                 (double)state->target_q_world_f_core.y * (double)core->q_world_f_core_next_psi.y +
                                 (double)state->target_q_world_f_core.z * (double)core->q_world_f_core_next_psi.z +
                                 (double)state->target_q_world_f_core.w * (double)core->q_world_f_core_next_psi.w;
        if (acos_quat < 0) {
            lvf_mult(&delta_angles_now_cs, (D)(torque_factor * inv_dt));
        } else {
            lvf_mult(&delta_angles_now_cs, (D)(-torque_factor * inv_dt));
        }

        /* reference golem damps rotation with damp_factor */
        ivp_vec3f_t rot_speed_change_cs;
        lvf_add_multiple2(&rot_speed_change_cs, &delta_angles_now_cs, &core->rot_speed, (D)(-damp_factor));
        const ivp_vec3f_t ri = ivp_core_rot_inertia_f(core), iri = ivp_core_inv_rot_inertia_f(core);
        for (int i = 2; i >= 0; --i) {
            const float max_t = (float)state->max_torque.k[i];
            const float rot_force = (float)((double)rot_speed_change_cs.k[i] * inv_dt * (double)ri.k[i]);
            if (fabs((double)rot_force) < (double)max_t) continue;
            if (rot_force < 0) {
                rot_speed_change_cs.k[i] = (float)((double)(-max_t * iri.k[i]) * delta_time);
            } else {
                rot_speed_change_cs.k[i] = (float)((double)(max_t * iri.k[i]) * delta_time);
            }
        }
        lvf_add(&core->rot_speed, &rot_speed_change_cs);
    }
}
