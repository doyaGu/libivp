/* ivp_controller_floating.c — Floating/levitation controller
 *
 * Port of IVP_Controller_Floating (ivp_controller/ivp_controller_floating.cxx).
 */

#include <ivp/controllers/ivp_controller_floating.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/solver/ivp_solver_core_reaction.h>
#include "solver/ivp_scr_internal.h"
#include "controllers/ivp_controller_ref.h"
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/base/ivp_assert.h>
#include <string.h>

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_floating_init_defaults(ivp_floating_state_t *state)
{
    /* Reference IVP_Template_Controller_Floating (ivp_controller_floating.cxx:10-16) */
    memset(state, 0, sizeof(*state));
    state->max_repulsive_force = (ivp_real_t)IVP_FLOAT_MAX;
    state->max_adhesive_force  = (ivp_real_t)IVP_FLOAT_MAX;
    state->ray_direction_ws.k[1] = IVP_REAL(1.0);
    state->active = true;
}

void ivp_floating_init(ivp_floating_state_t *state,
                       ivp_real_t target_dist,
                       ivp_real_t max_repulsive,
                       ivp_real_t max_adhesive)
{
    ivp_floating_init_defaults(state);
    state->target_distance     = target_dist;
    state->current_distance    = target_dist;
    state->max_repulsive_force = (ivp_real_t)(float)max_repulsive; /* IVP_FLOAT */
    state->max_adhesive_force  = (ivp_real_t)(float)max_adhesive;  /* IVP_FLOAT */
}

void ivp_floating_set_object(ivp_floating_state_t *state, ivp_object_t *object)
{
    state->object = object;
}

void ivp_floating_set_current_distance(ivp_floating_state_t *state, ivp_real_t d)
{
    state->current_distance = d;
}

void ivp_floating_set_target_distance(ivp_floating_state_t *state, ivp_real_t d)
{
    state->target_distance = d;
}

void ivp_floating_set_raycast(ivp_floating_state_t *state,
                              ivp_floating_raycast_fn_t fn,
                              void *ctx)
{
    state->raycast_fn  = fn;
    state->raycast_ctx = ctx;
}

void ivp_floating_set_position_os(ivp_floating_state_t *state,
                                  const ivp_vec3r_t *pos_os)
{
    /* position_os is an IVP_U_Float_Point */
    ivp_vec3f_t p = ivp_ctl_f(pos_os);
    state->position_os = ivp_ctl_r(&p);
}

void ivp_floating_set_ray_direction_ws(ivp_floating_state_t *state,
                                       const ivp_vec3r_t *dir_ws)
{
    /* Reference: IVP_U_Float_Point new_dir; normalize, reject (keep old)
     * if |dir| < P_FLOAT_EPS. */
    ivp_vec3f_t d = ivp_ctl_f(dir_ws);
    if (lvf_real_length_plus_normize(&d) < P_FLOAT_EPS) {
        IVP_ASSERT(0);
        return;
    }
    state->ray_direction_ws = ivp_ctl_r(&d);
}

/* ── Controller callback ───────────────────────────────────────────────── */

/* IVP_Real_Object::transform_position_to_world_coords: cache object
 * (AT) matrix times the IVP_U_Float_Point position_os */
static void floating_position_ws(ivp_object_t *obj, const ivp_vec3r_t *p_os, ivp_vec3r_t *out)
{
    ivp_vec3f_t p = ivp_ctl_f(p_os);
    ivp_cache_obj_t *co = ivp_object_get_cache_object_no_lock(obj);
    lvm_vmult4_fp(&co->m_world_f_object, &p, out);
}

void ivp_controller_floating(const ivp_event_sim_t *event_sim,
                             ivp_object_t *obj,
                             void *ctx)
{
    ivp_floating_state_t *state = (ivp_floating_state_t *)ctx;
    if (!state || !state->active) return;
    if (!obj || obj->physical_core->physical_unmoveable) return;
    if (state->object && obj != state->object) return;

    ivp_core_t *core = obj->physical_core;

    /* do_ray_casting(): IVP_FAULT -> return */
    if (state->raycast_fn) {
        ivp_vec3r_t position_ws;
        floating_position_ws(obj, &state->position_os, &position_ws);
        ivp_real_t dist;
        if (!state->raycast_fn(state->raycast_ctx, obj, &position_ws,
                               &state->ray_direction_ws, &dist)) {
            return;
        }
        state->current_distance = dist;
    }

    ivp_vec3r_t position_ws;
    floating_position_ws(obj, &state->position_os, &position_ws);

    const ivp_vec3f_t ray_direction_ws = ivp_ctl_f(&state->ray_direction_ws);
    ivp_scr_f_t tcb;
    ivp_scr_f_init_translation_ws(&tcb, core, NULL, &position_ws, &ray_direction_ws, NULL, NULL);
    const double i_dt = (double)event_sim->inverse_delta_time;
    const double d_t = (double)event_sim->delta_time;
    double a = ((double)state->current_distance - (double)state->target_distance) * i_dt
             - (double)tcb.delta_velocity_ds.k[0];
    const double tpm00 = tcb.m[0][0];
    if (tpm00 < P_FLOAT_EPS && tpm00 > -P_FLOAT_EPS) return;

    double impulse = a / tpm00;
    const double max_adhesive_force = (double)(float)state->max_adhesive_force;
    const double max_repulsive_force = (double)(float)state->max_repulsive_force;
    if (impulse * i_dt > max_adhesive_force) {
        impulse = max_adhesive_force * d_t;
    }
    if (-impulse * i_dt > max_repulsive_force) {
        impulse = -max_repulsive_force * d_t;
    }
    ivp_vec3f_t impulse_ds;
    lvf_set3(&impulse_ds, (float)impulse, 0.0f, 0.0f);
    ivp_scr_f_exert_impulse_dim1(&tcb, core, NULL, &impulse_ds);
}
