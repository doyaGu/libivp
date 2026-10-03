/* ivp_controller_world_friction.c -- World friction controller */

#include <ivp/controllers/ivp_controller_world_friction.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include "controllers/ivp_controller_ref.h"
#include <string.h>

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_world_friction_init_ground(ivp_world_friction_state_t *state,
                                      const ivp_vec3r_t *normal,
                                      ivp_real_t height,
                                      ivp_real_t friction_coeff)
{
    memset(state, 0, sizeof(*state));
    state->mode             = IVP_WORLD_FRICTION_GROUND;
    if (normal) {
        state->ground_normal = *normal;
    } else {
        ivp_vec3r_set(&state->ground_normal, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    }
    if (ivp_vec3r_normalize(&state->ground_normal) != IVP_OK) {
        ivp_vec3r_set(&state->ground_normal, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    }
    state->ground_dist      = height;
    state->ground_tolerance = IVP_REAL(0.5);
    state->friction_coeff   = friction_coeff;
    state->angular_friction = IVP_REAL(0.1);
    state->active           = true;
}

void ivp_world_friction_init_global(ivp_world_friction_state_t *state,
                                      ivp_real_t friction_coeff)
{
    memset(state, 0, sizeof(*state));
    state->mode             = IVP_WORLD_FRICTION_GLOBAL;
    state->friction_coeff   = friction_coeff;
    state->angular_friction = IVP_REAL(0.1);
    state->active           = true;
}

static void wf_clamp_non_negative(ivp_vec3r_t *v)
{
    for (int i = 0; i < 3; i++) {
        if (v->k[i] < IVP_REAL(0.0)) v->k[i] = IVP_REAL(0.0);
    }
}

void ivp_world_friction_init(ivp_world_friction_state_t *state, ivp_object_t *object)
{
    /* Reference IVP_Template_Controller_World_Friction
     * (ivp_controller_world_frict.cxx:39-46). */
    memset(state, 0, sizeof(*state));
    state->mode = IVP_WORLD_FRICTION_REFERENCE;
    state->object = object;
    const float friction_val = 0.4f * 9.81f; /* m/s, computed in float as the reference */
    ivp_vec3r_set(&state->friction_value_translation,
                  (ivp_real_t)friction_val, (ivp_real_t)friction_val, (ivp_real_t)friction_val);
    const ivp_real_t rot = (ivp_real_t)((double)friction_val * 0.3);
    ivp_vec3r_set(&state->friction_value_rotation, rot, rot, rot);
    ivp_vec3r_set(&state->ground_normal, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    state->active = true;
}

void ivp_world_friction_set_desired_speed_ws(ivp_world_friction_state_t *state,
                                             const ivp_vec3r_t *speed_ws)
{
    state->desired_speed_ws = *speed_ws;
    ivp_controller_wake_object(state->object);
}

void ivp_world_friction_set_desired_rot_speed_cs(ivp_world_friction_state_t *state,
                                                 const ivp_vec3r_t *rot_speed_cs)
{
    state->desired_rot_speed_cs = *rot_speed_cs;
    ivp_controller_wake_object(state->object);
}

void ivp_world_friction_set_friction_value_translation(ivp_world_friction_state_t *state,
                                                       const ivp_vec3r_t *value)
{
    state->friction_value_translation = *value;
    wf_clamp_non_negative(&state->friction_value_translation);
    ivp_controller_wake_object(state->object);
}

void ivp_world_friction_set_friction_value_rotation(ivp_world_friction_state_t *state,
                                                    const ivp_vec3r_t *value)
{
    state->friction_value_rotation = *value;
    wf_clamp_non_negative(&state->friction_value_rotation);
    ivp_controller_wake_object(state->object);
}

/* Port of IVP_Controller_World_Friction::do_simulation_controller. */
/* Reference IVP_Controller_World_Friction::do_simulation_controller
 * (ivp_controller_world_frict.cxx): IVP_U_Float_Point members and
 * corrections, IVP_U_Matrix3 float transforms. */
static void world_friction_reference(const ivp_world_friction_state_t *state,
                                     ivp_core_t *core, ivp_real_t dt)
{
    const ivp_transform_r_t *transform_mat = &core->m_world_f_core_last_psi;
    /* translation speed */
    {
        ivp_vec3f_t current_ws_speed = ivp_core_speed_f(core);
        ivp_vec3f_t desired = ivp_ctl_f(&state->desired_speed_ws), correction_speed_ws, correction_speed_cs;
        lvf_subtract2(&correction_speed_ws, &desired, &current_ws_speed);
        lvm_vimult3_f(transform_mat, &correction_speed_ws, &correction_speed_cs);
        ivp_vec3f_t fv = ivp_ctl_f(&state->friction_value_translation), max_friction_correction;
        lvf_set_multiple(&max_friction_correction, &fv, (D)dt);
        for (int i = 0; i < 3; i++) {
            if (fabs((double)correction_speed_cs.k[i]) > (double)max_friction_correction.k[i]) {
                int sign_val = 1;
                if (correction_speed_cs.k[i] < 0.0) sign_val = -1;
                correction_speed_cs.k[i] = max_friction_correction.k[i] * (float)sign_val;
            }
        }
        lvm_vmult3_f(transform_mat, &correction_speed_cs, &correction_speed_ws);
        /* center_push_core_multiple_ws(&corr, get_mass()):
         * speed.add_multiple(corr, factor * get_inv_mass()) */
        ivp_vec3f_t speed = ivp_core_speed_f(core);
        lvf_add_multiple(&speed, &correction_speed_ws,
                         (D)((double)ivp_core_get_mass_f(core) * (double)ivp_core_get_inv_mass_f(core)));
        ivp_core_set_speed_f(core, &speed);
    }
    /* rotation speed */
    {
        ivp_vec3f_t desired = ivp_ctl_f(&state->desired_rot_speed_cs), corr;
        lvf_subtract2(&corr, &desired, &core->rot_speed);
        ivp_vec3f_t fv = ivp_ctl_f(&state->friction_value_rotation), max_friction_correction;
        lvf_set_multiple(&max_friction_correction, &fv, (D)dt);
        for (int i = 0; i < 3; i++) {
            if (fabs((double)corr.k[i]) > (double)max_friction_correction.k[i]) {
                int sign_val = 1;
                if (corr.k[i] < 0.0) sign_val = -1;
                corr.k[i] = max_friction_correction.k[i] * (float)sign_val;
            }
        }
        /* impulses = corr * I; rot_push_core_cs: rot_speed += impulses * inv_I */
        const ivp_vec3f_t ri = ivp_core_rot_inertia_f(core), iri = ivp_core_inv_rot_inertia_f(core);
        ivp_vec3f_t imp, h;
        lvf_set_pairwise_mult(&imp, &corr, &ri);
        lvf_set_pairwise_mult(&h, &imp, &iri);
        lvf_add(&core->rot_speed, &h);
    }
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_world_friction(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx)
{
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    ivp_world_friction_state_t *state = (ivp_world_friction_state_t *)ctx;
    if (!state || !state->active) return;

    ivp_core_t *core = obj->physical_core;
    if (state->mode == IVP_WORLD_FRICTION_REFERENCE) {
        if (state->object && obj != state->object) return;
        if (core->physical_unmoveable) return;
        world_friction_reference(state, core, dt);
        return;
    }
    if (core->physical_unmoveable || core->pinned) return;

    bool apply = false;

    if (state->mode == IVP_WORLD_FRICTION_GLOBAL) {
        apply = true;
    } else {
        /* Ground mode: check distance from ground plane */
        ivp_real_t dist = ivp_vec3r_dot(&core->pos_world_f_core_last_psi,
                                         &state->ground_normal)
                        - state->ground_dist;

        /* Apply friction when within tolerance (considering object radius) */
        if (dist - core->upper_limit_radius <= state->ground_tolerance) {
            apply = true;
        }
    }

    if (!apply) return;

    ivp_vec3r_t friction_vel = core->speed;
    if (state->mode == IVP_WORLD_FRICTION_GROUND) {
        /* Ground mode only damps tangential velocity, not normal approach speed. */
        ivp_real_t vn = ivp_vec3r_dot(&friction_vel, &state->ground_normal);
        friction_vel.k[0] -= state->ground_normal.k[0] * vn;
        friction_vel.k[1] -= state->ground_normal.k[1] * vn;
        friction_vel.k[2] -= state->ground_normal.k[2] * vn;
    }

    ivp_real_t speed_sq = ivp_vec3r_dot(&friction_vel, &friction_vel);
    if (speed_sq >= IVP_REAL_EPS) {
        ivp_real_t speed = ivp_sqrt(speed_sq);

        /* Coulomb friction force: F = mu * m * g
         * dv = mu * g * dt, clamped to not exceed current speed.
         * Use environment gravity if set, otherwise assume standard Earth gravity. */
        ivp_real_t g = ivp_vec3r_length(&env->gravity);
        if (g < IVP_REAL_EPS) g = IVP_REAL(9.81);
        ivp_real_t max_dv = state->friction_coeff * g * dt;

        if (max_dv > speed) max_dv = speed;

        /* Apply deceleration along friction velocity direction (sync push). */
        ivp_real_t factor = -max_dv / speed;
        core->speed.k[0] += friction_vel.k[0] * factor;
        core->speed.k[1] += friction_vel.k[1] * factor;
        core->speed.k[2] += friction_vel.k[2] * factor;
        ivp_ctl_round_speed(core);
    }

    /* Angular friction: simple damping */
    if (state->angular_friction > IVP_REAL(0.0)) {
        ivp_real_t ang_factor = IVP_REAL(1.0) - state->angular_friction * dt;
        if (ang_factor < IVP_REAL(0.0)) ang_factor = IVP_REAL(0.0);

        core->rot_speed.k[0] *= (float)ang_factor;
        core->rot_speed.k[1] *= (float)ang_factor;
        core->rot_speed.k[2] *= (float)ang_factor;
    }
}
