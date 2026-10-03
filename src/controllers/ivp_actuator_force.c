/* ivp_actuator_force.c -- Anchor-based force actuator implementation
 *
 * Port of IVP_Actuator_Force from the legacy SDK.
 *
 * Applies a constant force along the line connecting two anchor points.
 * The force direction is from anchor_a toward anchor_b (attraction).
 * Negative force values push objects apart (repulsion).
 *
 * Force is applied at the anchor points, producing both linear and angular
 * effects when anchors are offset from the center of mass.
 */
#include <ivp/controllers/ivp_actuator_force.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>

#include <string.h>
#include "controllers/ivp_controller_ref.h"

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_force_actuator_state_init(ivp_force_actuator_state_t *state)
{
    /* Reference IVP_Template_Force (ivp_actuator.cxx:46-52):
     * force 0, push_first_object TRUE, push_second_object FALSE. */
    memset(state, 0, sizeof(*state));
    state->active = true;
    state->push_first_object = true;
    state->push_second_object = false;
}

void ivp_force_actuator_state_setup(ivp_force_actuator_state_t *state,
                                     ivp_object_t *obj_a,
                                     const ivp_vec3r_t *anchor_a_os,
                                     ivp_object_t *obj_b,
                                     const ivp_vec3r_t *anchor_b_os,
                                     ivp_real_t force,
                                     bool push_first,
                                     bool push_second)
{
    memset(state, 0, sizeof(*state));
    state->obj_a = obj_a;
    state->obj_b = obj_b;
    /* IVP_Template_Anchor::set_anchor_position_os + IVP_Anchor::init_anchor */
    ivp_environment_t *env = ivp_ctl_env_of2(obj_a, obj_b);
    ivp_ctl_anchor_setup_os(obj_a, env, anchor_a_os, &state->anchor_a_os);
    ivp_ctl_anchor_setup_os(obj_b, env, anchor_b_os, &state->anchor_b_os);
    state->force = (ivp_real_t)(float)force; /* IVP_FLOAT */
    state->push_first_object = push_first;
    state->push_second_object = push_second;
    state->active = true;
}

void ivp_force_actuator_set_force(ivp_force_actuator_state_t *state,
                                    ivp_real_t force)
{
    /* Reference IVP_Actuator_Force::set_force(IVP_DOUBLE) (ivp_actuator.cxx:403-409):
     * IVP_FLOAT member. */
    if ((double)(float)state->force == (double)force) return;
    state->force = (ivp_real_t)(float)force;
    ivp_controller_wake_actuator(state->obj_a, state->obj_b);
}

/* ── IVP_Actuator_Force_Active ──────────────────────────────────────────── */

/* Reference IVP_Actuator_Force_Active::active_float_changed. */
static void force_active_float_changed(void *ctx, ivp_active_float_t *af,
                                       double old_value, double new_value)
{
    (void)old_value;
    (void)new_value;
    ivp_force_actuator_state_t *state = (ivp_force_actuator_state_t *)ctx;
    if (af == state->active_float_force) {
        ivp_force_actuator_set_force(state, (ivp_real_t)ivp_active_float_get_float(af));
    }
}

void ivp_force_actuator_unbind_active(ivp_force_actuator_state_t *state)
{
    if (!state || !state->active_float_force) return;
    ivp_active_float_t *af = state->active_float_force;
    state->active_float_force = NULL;
    (void)ivp_active_float_remove_listener(af, force_active_float_changed, state);
}

void ivp_force_actuator_bind_active(ivp_force_actuator_state_t *state,
                                    ivp_active_float_t *active_float_force)
{
    if (!state) return;
    ivp_force_actuator_unbind_active(state);
    if (!active_float_force) return;
    /* constructor: add_dependency(this); active_float_changed(af) */
    if (!ivp_active_float_add_listener(active_float_force, force_active_float_changed, state)) return;
    state->active_float_force = active_float_force;
    force_active_float_changed(state, active_float_force, active_float_force->value,
                               active_float_force->value);
}

/* ── Controller callback ────────────────────────────────────────────────── */

/* Port of IVP_Actuator_Force::do_simulation_controller
 * (ivp_actuator.cxx:829-879). */
void ivp_controller_force_actuator(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx)
{
    ivp_force_actuator_state_t *state = (ivp_force_actuator_state_t *)ctx;
    if (!state->active) return;
    const double force_val = (double)(float)state->force;
    if (force_val == 0.0f) return; /* nothing to push */

    ivp_object_t *ends[2] = { state->obj_a, state->obj_b };
    if (obj != ivp_controller_pick_exec_object(ends, 2)) return;

    ivp_environment_t *env = event_sim->environment;
    ivp_core_t *pc0 = ivp_ctl_anchor_core(state->obj_a, env);
    ivp_core_t *pc1 = ivp_ctl_anchor_core(state->obj_b, env);
    const ivp_vec3f_t core_pos0 = ivp_ctl_f(&state->anchor_a_os);
    const ivp_vec3f_t core_pos1 = ivp_ctl_f(&state->anchor_b_os);

    ivp_vec3r_t pos0_ws, pos1_ws, dir_ws;
    ivp_ctl_anchor_pos_ws(pc0, &core_pos0, &pos0_ws);
    ivp_ctl_anchor_pos_ws(pc1, &core_pos1, &pos1_ws);
    /* dir_ws = pos0 - pos1 (IVP_U_Point): a positive force pushes the first
     * object away from the second anchor. */
    lvp_subtract2(&dir_ws, &pos0_ws, &pos1_ws);
    state->current_distance = (ivp_real_t)lvp_real_length(&dir_ws);

    if (lvp_quad_length(&dir_ws) < P_DOUBLE_EPS) return;
    (void)lvp_fast_normize(&dir_ws);

    ivp_vec3f_t force_dir;
    lvf_set_p(&force_dir, &dir_ws);
    lvf_mult(&force_dir, (D)(force_val * (double)event_sim->delta_time));

    if (state->push_first_object && ivp_controller_object_pushable(state->obj_a)) {
        ivp_ctl_async_push_core_ws(pc0, &pos0_ws, &force_dir);
    }
    if (state->push_second_object && ivp_controller_object_pushable(state->obj_b)) {
        /* also push at other end (reversed) */
        ivp_vec3f_t force_dir_rev;
        lvf_set_multiple(&force_dir_rev, &force_dir, (D)-1.0);
        ivp_ctl_async_push_core_ws(pc1, &pos1_ws, &force_dir_rev);
    }
}

int ivp_force_actuator_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_force_actuator_state_t *s = (const ivp_force_actuator_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[2] = { s->obj_a, s->obj_b };
    return ivp_controller_discover_objects(objs, 2, out_handles, max_handles);
}
