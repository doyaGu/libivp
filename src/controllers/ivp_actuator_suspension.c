/* ivp_actuator_suspension.c -- Suspension spring implementation
 *
 * Port of IVP_Actuator_Suspension::do_simulation_controller
 * (ivp_controller/ivp_actuator_spring.cxx:320-379).
 */

#include <ivp/controllers/ivp_actuator_suspension.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>
#include "controllers/ivp_controller_ref.h"

/* reference IVP_FLOAT members */
static inline ivp_real_t susp_fl(double v) { return (ivp_real_t)(float)v; }

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_suspension_state_init(ivp_suspension_state_t *state)
{
    /* Reference IVP_Template_Suspension (ivp_actuator_spring.cxx:31-35) on
     * top of IVP_Template_Spring: every constant 0, max_body_force 0. */
    memset(state, 0, sizeof(*state));
    state->spring_values_factor = IVP_REAL(1.0);
}

void ivp_suspension_state_setup(ivp_suspension_state_t *state,
                                 ivp_object_t *body,
                                 ivp_object_t *wheel,
                                 const ivp_vec3r_t *anchor_body_os,
                                 const ivp_vec3r_t *anchor_wheel_os,
                                 ivp_real_t stiffness,
                                 ivp_real_t damping,
                                 ivp_real_t damping_compression,
                                 ivp_real_t rest_length)
{
    if (state->spring_values_factor == IVP_REAL(0.0)) {
        state->spring_values_factor = IVP_REAL(1.0);
    }
    state->obj_a              = body;
    state->obj_b              = wheel;
    /* IVP_Template_Anchor::set_anchor_position_os + IVP_Anchor::init_anchor */
    ivp_environment_t *env = ivp_ctl_env_of2(body, wheel);
    ivp_ctl_anchor_setup_os(body, env, anchor_body_os, &state->anchor_a_os, &state->anchors[0]);
    ivp_ctl_anchor_setup_os(wheel, env, anchor_wheel_os, &state->anchor_b_os, &state->anchors[1]);
    /* IVP_Actuator_Suspension constructor: IVP_FLOAT members (the template
     * values are taken as given, i.e. spring_values_are_relative == FALSE) */
    state->stiffness          = susp_fl(stiffness);
    state->damping            = susp_fl(damping);
    state->damping_compression = susp_fl(damping_compression);
    state->rest_length        = susp_fl(rest_length);
}

/* ── Runtime adjustment ────────────────────────────────────────────────── */

static void suspension_wake(ivp_suspension_state_t *state)
{
    ivp_controller_wake_actuator(state->obj_a, state->obj_b);
}

static ivp_real_t suspension_factor(const ivp_suspension_state_t *state)
{
    return state->spring_values_factor > IVP_REAL(0.0) ? state->spring_values_factor : IVP_REAL(1.0);
}

void ivp_suspension_set_damping_compression(ivp_suspension_state_t *state,
                                             ivp_real_t value)
{
    /* Reference set_spring_damp_compression(IVP_FLOAT comp)
     * (ivp_actuator_spring.cxx:304-310):
     *   if (spring_dampening_compression == comp) return;
     *   spring_dampening_compression = comp * spring_values_factor; */
    const float comp = (float)value;
    if ((float)state->damping_compression == comp) return;
    state->damping_compression = (ivp_real_t)(comp * (float)suspension_factor(state));
    suspension_wake(state);
}

/* IVP_Actuator_Spring::set_constant / set_damp / set_len (IVP_DOUBLE argument) */
void ivp_suspension_set_constant(ivp_suspension_state_t *state, ivp_real_t value)
{
    state->stiffness = susp_fl((double)value * (double)(float)suspension_factor(state));
    suspension_wake(state);
}

void ivp_suspension_set_damp(ivp_suspension_state_t *state, ivp_real_t value)
{
    state->damping = susp_fl((double)value * (double)(float)suspension_factor(state));
    suspension_wake(state);
}

void ivp_suspension_set_len(ivp_suspension_state_t *state, ivp_real_t value)
{
    if ((double)(float)state->rest_length == (double)value) return;
    state->rest_length = susp_fl(value);
    suspension_wake(state);
}

void ivp_suspension_set_max_body_force(ivp_suspension_state_t *state,
                                        ivp_real_t value)
{
    /* set_max_body_force(IVP_FLOAT) */
    const float max_force = (float)value;
    if ((float)state->max_body_force == max_force) return;
    state->max_body_force = (ivp_real_t)max_force;
    suspension_wake(state);
}

/* ── Controller callback ───────────────────────────────────────────────── */

void ivp_controller_suspension(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx)
{
    ivp_suspension_state_t *state = (ivp_suspension_state_t *)ctx;
    if (!state) return;

    ivp_object_t *ends[2] = { state->obj_a, state->obj_b };
    if (obj != ivp_controller_pick_exec_object(ends, 2)) return;

    /* Reference IVP_Actuator_Suspension::do_simulation_controller
     * (ivp_actuator_spring.cxx:312-379). */
    ivp_environment_t *env = event_sim->environment;
    ivp_core_t *pc0 = ivp_ctl_anchor_core(state->obj_a, env);
    ivp_core_t *pc1 = ivp_ctl_anchor_core(state->obj_b, env);
    const ivp_vec3f_t core_pos0 = ivp_ctl_f(&state->anchor_a_os);
    const ivp_vec3f_t core_pos1 = ivp_ctl_f(&state->anchor_b_os);

    ivp_vec3r_t pos0_ws, pos1_ws;
    ivp_ctl_anchor_pos_ws(pc0, &core_pos0, &pos0_ws);
    ivp_ctl_anchor_pos_ws(pc1, &core_pos1, &pos1_ws);
    ivp_vec3f_t dir_ws;
    lvf_subtract2_p(&dir_ws, &pos0_ws, &pos1_ws); /* force direction world coords */

    const float dlen = (float)lvf_real_length_plus_normize(&dir_ws);
    state->current_length = (ivp_real_t)dlen;
    if (dlen < P_FLOAT_EPS) {
        state->current_force = IVP_REAL(0.0);
        state->current_body_force = IVP_REAL(0.0);
        return;
    }

    const double delta_len = (double)(dlen - (float)state->rest_length);
    /* linear (standard) spring */
    double force = delta_len * (double)(float)state->stiffness;

    /* spring damping */
    ivp_vec3f_t relative_world_speed;
    ivp_ctl_diff_surface_speed(pc1, pc0, &core_pos1, &core_pos0, &relative_world_speed);
    const double damp_speed = (double)lvf_dot(&relative_world_speed, &dir_ws); /* positive: closing */

    /* SPECIAL suspension treatment: another factor when the spring is compressing */
    double damp_s;
    if (damp_speed < 0.0f) {
        damp_s = (double)(float)state->damping;
    } else {
        damp_s = (double)(float)state->damping_compression;
    }

    force = (force - damp_s * damp_speed);
    double force_clipped = -force;

    /* maximum force for body */
    const double max_body_force = (double)(float)state->max_body_force;
    if (force_clipped < -max_body_force) {
        force_clipped = -max_body_force;
    } else if (force_clipped > max_body_force) {
        force_clipped = max_body_force;
    }
    state->current_force = (ivp_real_t)force;
    state->current_body_force = (ivp_real_t)force_clipped;

    const double delta_time = (double)event_sim->delta_time;
    const double impulse = force * delta_time;
    const double impulse_clipped = force_clipped * delta_time;

    ivp_vec3f_t impulse_vector_ws;
    lvf_set_multiple(&impulse_vector_ws, &dir_ws, (D)impulse);
    if (pc1 && !pc1->physical_unmoveable) ivp_ctl_async_push_core_ws(pc1, &pos1_ws, &impulse_vector_ws);

    lvf_set_multiple(&impulse_vector_ws, &dir_ws, (D)impulse_clipped);
    if (pc0 && !pc0->physical_unmoveable) ivp_ctl_async_push_core_ws(pc0, &pos0_ws, &impulse_vector_ws);
}

int ivp_suspension_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_suspension_state_t *s = (const ivp_suspension_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[2] = { s->obj_a, s->obj_b };
    return ivp_controller_discover_objects(objs, 2, out_handles, max_handles);
}
