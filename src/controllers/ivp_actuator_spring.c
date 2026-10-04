/* ivp_actuator_spring.c -- Spring force actuator implementation
 *
 * Port of IVP_Actuator_Spring::do_simulation_controller
 * (ivp_controller/ivp_actuator_spring.cxx:233-294).
 */

#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/base/ivp_assert.h>
#include <string.h>
#include "controllers/ivp_controller_ref.h"

/* reference IVP_FLOAT members */
static inline ivp_real_t spring_fl(double v) { return (ivp_real_t)(float)v; }

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_spring_state_init(ivp_spring_state_t *state)
{
    /* Reference IVP_Template_Spring::IVP_Template_Spring
     * (ivp_actuator_spring.cxx:15-27): every constant defaults to zero. */
    memset(state, 0, sizeof(*state));
    state->spring_values_factor = IVP_REAL(1.0);
    state->break_max_len = (ivp_real_t)IVP_FLOAT_MAX;
    state->max_len_exceed_type = IVP_SFE_NONE;
}

void ivp_spring_state_setup(ivp_spring_state_t *state,
                              ivp_object_t *obj_a,
                              const ivp_vec3r_t *anchor_a_os,
                              ivp_object_t *obj_b,
                              const ivp_vec3r_t *anchor_b_os,
                              ivp_real_t stiffness,
                              ivp_real_t damping,
                              ivp_real_t rest_length)
{
    ivp_spring_state_init(state);
    state->obj_a       = obj_a;
    state->obj_b       = obj_b;
    /* IVP_Template_Anchor::set_anchor_position_os + IVP_Anchor::init_anchor */
    ivp_environment_t *env = ivp_ctl_env_of2(obj_a, obj_b);
    ivp_ctl_anchor_setup_os(obj_a, env, anchor_a_os, &state->anchor_a_os, &state->anchors[0]);
    ivp_ctl_anchor_setup_os(obj_b, env, anchor_b_os, &state->anchor_b_os, &state->anchors[1]);
    /* IVP_Template_Spring / IVP_Actuator_Spring members are IVP_FLOAT */
    state->stiffness   = spring_fl(stiffness);
    state->damping     = spring_fl(damping);
    state->rest_length = spring_fl(rest_length);
}

void ivp_spring_state_make_relative(ivp_spring_state_t *state)
{
    if (!state) return;
    /* Reference constructor (ivp_actuator_spring.cxx:66-77):
     *   min_mass_i = core_i->calc_virt_mass(&anchor_i->core_pos, NULL)
     *   spring_values_factor = (m0 * m1) / (m0 + m1)          (IVP_FLOAT)
     *   spring_constant = templ->spring_constant * factor      (float * float)
     * A static / world endpoint counts as infinitely heavy. */
    const ivp_core_t *c0 = state->obj_a ? state->obj_a->physical_core : NULL;
    const ivp_core_t *c1 = state->obj_b ? state->obj_b->physical_core : NULL;
    const bool inf0 = !c0 || c0->physical_unmoveable;
    const bool inf1 = !c1 || c1->physical_unmoveable;
    ivp_vec3f_t p0 = ivp_ctl_f(&state->anchor_a_os), p1 = ivp_ctl_f(&state->anchor_b_os);
    double factor = 1.0;
    if (inf0 && inf1) {
        factor = 1.0;
    } else if (inf0) {
        factor = ivp_ctl_calc_virt_mass_min(c1, &p1);
    } else if (inf1) {
        factor = ivp_ctl_calc_virt_mass_min(c0, &p0);
    } else {
        double m0 = ivp_ctl_calc_virt_mass_min(c0, &p0);
        double m1 = ivp_ctl_calc_virt_mass_min(c1, &p1);
        factor = (m0 * m1) / (m0 + m1);
    }
    /* Undo a previous factor so repeated calls are idempotent. */
    float old = (float)state->spring_values_factor;
    if (old > 0.0f && old != 1.0f) {
        state->stiffness = spring_fl((float)state->stiffness / old);
        state->damping = spring_fl((float)state->damping / old);
        state->rel_pos_damp = spring_fl((float)state->rel_pos_damp / old);
    }
    float f = (float)factor;
    state->spring_values_factor = (ivp_real_t)f;
    state->stiffness = (ivp_real_t)((float)state->stiffness * f);
    state->damping = (ivp_real_t)((float)state->damping * f);
    state->rel_pos_damp = (ivp_real_t)((float)state->rel_pos_damp * f);
}

void ivp_spring_ensure_in_simulation(ivp_spring_state_t *state)
{
    if (!state) return;
    ivp_controller_wake_actuator(state->obj_a, state->obj_b);
}

/* Reference setters (ivp_actuator_spring.cxx:80-114): IVP_DOUBLE argument,
 * IVP_FLOAT members. */
void ivp_spring_set_constant(ivp_spring_state_t *state, ivp_real_t value)
{
    state->stiffness = spring_fl((double)value * (double)(float)state->spring_values_factor);
    ivp_spring_ensure_in_simulation(state);
}

void ivp_spring_set_damp(ivp_spring_state_t *state, ivp_real_t value)
{
    state->damping = spring_fl((double)value * (double)(float)state->spring_values_factor);
    ivp_spring_ensure_in_simulation(state);
}

void ivp_spring_set_rel_pos_damp(ivp_spring_state_t *state, ivp_real_t value)
{
    state->rel_pos_damp = spring_fl((double)value * (double)(float)state->spring_values_factor);
    ivp_spring_ensure_in_simulation(state);
}

void ivp_spring_set_len(ivp_spring_state_t *state, ivp_real_t value)
{
    if ((double)(float)state->rest_length == (double)value) return;
    state->rest_length = spring_fl(value);
    ivp_spring_ensure_in_simulation(state);
}

void ivp_spring_set_break_max_len(ivp_spring_state_t *state, ivp_real_t value)
{
    if ((double)(float)state->break_max_len == (double)value) return;
    state->break_max_len = spring_fl(value);
    ivp_spring_ensure_in_simulation(state);
}

void ivp_spring_set_broken_listener(ivp_spring_state_t *state,
                                    ivp_spring_broken_fn_t fn, void *ctx)
{
    state->broken_fn = fn;
    state->broken_ctx = ctx;
}

/* ── IVP_Actuator_Spring_Active ─────────────────────────────────────────── */

/* Reference IVP_Actuator_Spring_Active::active_float_changed
 * (ivp_actuator_spring.cxx:130-153). */
static void spring_active_float_changed(void *ctx, ivp_active_float_t *af,
                                        double old_value, double new_value)
{
    (void)old_value;
    (void)new_value;
    ivp_spring_state_t *state = (ivp_spring_state_t *)ctx;
    if (af == state->active_float_spring_len) {
        ivp_spring_set_len(state, (ivp_real_t)ivp_active_float_get_float(af));
        return;
    }
    if (af == state->active_float_spring_constant) {
        ivp_spring_set_constant(state, (ivp_real_t)ivp_active_float_get_float(af));
        return;
    }
    if (af == state->active_float_spring_damp) {
        ivp_spring_set_damp(state, (ivp_real_t)ivp_active_float_get_float(af));
        return;
    }
    if (af == state->active_float_spring_rel_pos_damp) {
        ivp_spring_set_rel_pos_damp(state, (ivp_real_t)ivp_active_float_get_float(af));
        return;
    }
    IVP_CORE(); /* ivp_actuator_spring.cxx:151 */
}

static void spring_release(ivp_spring_state_t *state, ivp_active_float_t **slot)
{
    ivp_active_float_t *af = *slot;
    if (!af) return;
    *slot = NULL;
    (void)ivp_active_float_remove_listener(af, spring_active_float_changed, state);
}

static bool spring_add(ivp_spring_state_t *state, ivp_active_float_t **slot, ivp_active_float_t *af)
{
    if (!af) return false;
    if (!ivp_active_float_add_listener(af, spring_active_float_changed, state)) return false;
    *slot = af;
    return true;
}

void ivp_spring_unbind_active(ivp_spring_state_t *state)
{
    if (!state) return;
    spring_release(state, &state->active_float_spring_len);
    spring_release(state, &state->active_float_spring_constant);
    spring_release(state, &state->active_float_spring_damp);
    spring_release(state, &state->active_float_spring_rel_pos_damp);
}

void ivp_spring_bind_active(ivp_spring_state_t *state,
                            ivp_active_float_t *active_float_spring_len,
                            ivp_active_float_t *active_float_spring_constant,
                            ivp_active_float_t *active_float_spring_damp,
                            ivp_active_float_t *active_float_spring_rel_pos_damp)
{
    if (!state) return;
    ivp_spring_unbind_active(state);
    /* IVP_Actuator_Spring_Active constructor (ivp_actuator_spring.cxx:155-183) */
    if (spring_add(state, &state->active_float_spring_len, active_float_spring_len)) {
        ivp_spring_set_len(state, (ivp_real_t)ivp_active_float_get_float(active_float_spring_len));
    }
    if (spring_add(state, &state->active_float_spring_constant, active_float_spring_constant)) {
        ivp_spring_set_constant(state, (ivp_real_t)ivp_active_float_get_float(active_float_spring_constant));
    }
    if (spring_add(state, &state->active_float_spring_damp, active_float_spring_damp)) {
        ivp_spring_set_damp(state, (ivp_real_t)ivp_active_float_get_float(active_float_spring_damp));
    }
    if (spring_add(state, &state->active_float_spring_rel_pos_damp, active_float_spring_rel_pos_damp)) {
        ivp_spring_set_rel_pos_damp(state,
            (ivp_real_t)ivp_active_float_get_float(active_float_spring_rel_pos_damp));
    }
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_spring(const ivp_event_sim_t *event_sim,
                             ivp_object_t *obj,
                             void *ctx)
{
    ivp_spring_state_t *state = (ivp_spring_state_t *)ctx;
    if (!state || state->broken) return;

    /* Execute once per PSI when any movable endpoint is simulated
     * (reference IVP_Controller_Dependent semantics). */
    ivp_object_t *ends[2] = { state->obj_a, state->obj_b };
    if (obj != ivp_controller_pick_exec_object(ends, 2)) return;

    /* Reference IVP_Actuator_Spring::do_simulation_controller
     * (ivp_actuator_spring.cxx:204-270). */
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
        return;
    }

    if (dlen > (float)state->break_max_len && state->max_len_exceed_type == IVP_SFE_BREAK) {
        /* Reference: fire_event_spring_broken(); P_DELETE_THIS(this). */
        state->broken = true;
        state->current_force = IVP_REAL(0.0);
        if (state->broken_fn) state->broken_fn(state->broken_ctx, state);
        return;
    }

    const float spring_len = (float)state->rest_length;
    if (state->spring_force_only_on_stretch && !(dlen > spring_len)) {
        state->current_force = IVP_REAL(0.0);
        return;
    }

    /* IVP_FLOAT arithmetic: (dlen - spring_len) * spring_constant */
    double force = (double)((dlen - spring_len) * (float)state->stiffness);

    /* spring damping */
    ivp_vec3f_t relative_world_speed;
    ivp_ctl_diff_surface_speed(pc1, pc0, &core_pos1, &core_pos0, &relative_world_speed);
    const double damp_speed = (double)lvf_dot(&dir_ws, &relative_world_speed); /* positive: closing */
    const double damp_s = (double)(float)state->damping;
    const double damp_m = (double)(float)state->rel_pos_damp;
    force = force - damp_s * damp_speed;

    if (state->max_force > IVP_REAL(0.0)) { /* libivp extra */
        if (force > (double)state->max_force) force = (double)state->max_force;
        if (force < -(double)state->max_force) force = -(double)state->max_force;
    }
    state->current_force = (ivp_real_t)force;

    const double delta_time = (double)event_sim->delta_time;
    ivp_vec3f_t force_vector_ws;
    lvf_set_multiple(&force_vector_ws, &dir_ws, (D)(force * delta_time));
    { /* manhattan damping */
        const double manhattan_factor = -delta_time * damp_m;
        lvf_add_multiple(&force_vector_ws, &relative_world_speed, (D)manhattan_factor);
    }

    if (ivp_controller_object_pushable(state->obj_b)) {
        ivp_ctl_async_push_core_ws(pc1, &pos1_ws, &force_vector_ws);
    }
    if (ivp_controller_object_pushable(state->obj_a)) {
        lvf_mult(&force_vector_ws, (D)-1.0);
        ivp_ctl_async_push_core_ws(pc0, &pos0_ws, &force_vector_ws);
    }
}

int ivp_spring_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_spring_state_t *s = (const ivp_spring_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[2] = { s->obj_a, s->obj_b };
    return ivp_controller_discover_objects(objs, 2, out_handles, max_handles);
}
