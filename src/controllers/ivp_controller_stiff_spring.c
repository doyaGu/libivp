/* ivp_controller_stiff_spring.c -- Stiff spring controller implementation
 *
 * Port of IVP_Controller_Stiff_Spring::do_simulation_controller
 * (ivp_controller/ivp_controller_stiff_spring.cxx:166-226).
 */
#include <ivp/controllers/ivp_controller_stiff_spring.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/solver/ivp_solver_core_reaction.h>
#include "solver/ivp_scr_internal.h"
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>

#include <ivp/base/ivp_assert.h>

#include <string.h>
#include "controllers/ivp_controller_ref.h"

/* reference IVP_FLOAT members */
static inline ivp_real_t stiff_fl(double v) { return (ivp_real_t)(float)v; }

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_stiff_spring_state_init(ivp_stiff_spring_state_t *state)
{
    /* Reference IVP_Template_Stiff_Spring (ivp_controller_stiff_spring.cxx:16-23):
     * spring_len = spring_constant = spring_damp = 0, break_max_len = P_FLOAT_MAX. */
    memset(state, 0, sizeof(*state));
    state->break_max_len = (ivp_real_t)IVP_FLOAT_MAX;
    state->max_len_exceed_type = IVP_SFE_NONE;
    state->active = true;
}

void ivp_stiff_spring_state_setup(ivp_stiff_spring_state_t *state,
                                    ivp_object_t *obj_a,
                                    const ivp_vec3r_t *anchor_a_os,
                                    ivp_object_t *obj_b,
                                    const ivp_vec3r_t *anchor_b_os,
                                    ivp_real_t stiffness,
                                    ivp_real_t damping,
                                    ivp_real_t rest_length)
{
    ivp_stiff_spring_state_init(state);
    state->obj_a = obj_a;
    state->obj_b = obj_b;
    /* IVP_Template_Anchor::set_anchor_position_os + IVP_Anchor::init_anchor */
    ivp_environment_t *env = ivp_ctl_env_of2(obj_a, obj_b);
    ivp_ctl_anchor_setup_os(obj_a, env, anchor_a_os, &state->anchor_a_os);
    ivp_ctl_anchor_setup_os(obj_b, env, anchor_b_os, &state->anchor_b_os);
    /* IVP_FLOAT members */
    state->stiffness = stiff_fl(stiffness);
    state->damping = stiff_fl(damping);
    state->rest_length = stiff_fl(rest_length);
}

static void stiff_spring_wake(ivp_stiff_spring_state_t *state)
{
    ivp_controller_wake_actuator(state->obj_a, state->obj_b);
}

/* Reference setters: IVP_DOUBLE argument, IVP_FLOAT members. */
void ivp_stiff_spring_set_constant(ivp_stiff_spring_state_t *state, ivp_real_t value)
{
    state->stiffness = stiff_fl(value);
    stiff_spring_wake(state);
}

void ivp_stiff_spring_set_damp(ivp_stiff_spring_state_t *state, ivp_real_t value)
{
    state->damping = stiff_fl(value);
    stiff_spring_wake(state);
}

void ivp_stiff_spring_set_len(ivp_stiff_spring_state_t *state, ivp_real_t value)
{
    if ((double)(float)state->rest_length == (double)value) return;
    state->rest_length = stiff_fl(value);
    stiff_spring_wake(state);
}

void ivp_stiff_spring_set_break_max_len(ivp_stiff_spring_state_t *state, ivp_real_t value)
{
    if ((double)(float)state->break_max_len == (double)value) return;
    state->break_max_len = stiff_fl(value);
    stiff_spring_wake(state);
}

void ivp_stiff_spring_set_broken_listener(ivp_stiff_spring_state_t *state,
                                          ivp_stiff_spring_broken_fn_t fn, void *ctx)
{
    state->broken_fn = fn;
    state->broken_ctx = ctx;
}

/* ── IVP_Controller_Stiff_Spring_Active ─────────────────────────────────── */

/* Reference IVP_Controller_Stiff_Spring_Active::active_float_changed
 * (ivp_controller_stiff_spring.cxx:102-120). */
static void stiff_spring_active_float_changed(void *ctx, ivp_active_float_t *af,
                                              double old_value, double new_value)
{
    (void)old_value;
    (void)new_value;
    ivp_stiff_spring_state_t *state = (ivp_stiff_spring_state_t *)ctx;
    if (af == state->active_float_spring_len) {
        ivp_stiff_spring_set_len(state, (ivp_real_t)ivp_active_float_get_float(af));
        return;
    }
    if (af == state->active_float_spring_constant) {
        ivp_stiff_spring_set_constant(state, (ivp_real_t)ivp_active_float_get_float(af));
        return;
    }
    if (af == state->active_float_spring_damp) {
        ivp_stiff_spring_set_damp(state, (ivp_real_t)ivp_active_float_get_float(af));
        return;
    }
    IVP_CORE(); /* ivp_controller_stiff_spring.cxx:118 */
}

static void stiff_spring_release(ivp_stiff_spring_state_t *state, ivp_active_float_t **slot)
{
    ivp_active_float_t *af = *slot;
    if (!af) return;
    *slot = NULL;
    (void)ivp_active_float_remove_listener(af, stiff_spring_active_float_changed, state);
}

static bool stiff_spring_add(ivp_stiff_spring_state_t *state, ivp_active_float_t **slot,
                             ivp_active_float_t *af)
{
    if (!af) return false;
    if (!ivp_active_float_add_listener(af, stiff_spring_active_float_changed, state)) return false;
    *slot = af;
    return true;
}

void ivp_stiff_spring_unbind_active(ivp_stiff_spring_state_t *state)
{
    if (!state) return;
    stiff_spring_release(state, &state->active_float_spring_len);
    stiff_spring_release(state, &state->active_float_spring_constant);
    stiff_spring_release(state, &state->active_float_spring_damp);
}

void ivp_stiff_spring_bind_active(ivp_stiff_spring_state_t *state,
                                  ivp_active_float_t *active_float_spring_len,
                                  ivp_active_float_t *active_float_spring_constant,
                                  ivp_active_float_t *active_float_spring_damp)
{
    if (!state) return;
    ivp_stiff_spring_unbind_active(state);
    /* constructor (ivp_controller_stiff_spring.cxx:122-145): values copied
     * directly, no ensure_actuator_in_simulation */
    if (stiff_spring_add(state, &state->active_float_spring_len, active_float_spring_len)) {
        state->rest_length = stiff_fl(ivp_active_float_get_float(active_float_spring_len));
    }
    if (stiff_spring_add(state, &state->active_float_spring_constant, active_float_spring_constant)) {
        state->stiffness = stiff_fl(ivp_active_float_get_float(active_float_spring_constant));
    }
    if (stiff_spring_add(state, &state->active_float_spring_damp, active_float_spring_damp)) {
        state->damping = stiff_fl(ivp_active_float_get_float(active_float_spring_damp));
    }
}

/* ── Controller callback ────────────────────────────────────────────────── */

/* Reference (ivp_controller_stiff_spring.cxx:200-203):
 *   if (pc->movement_state >= IVP_MT_NOT_SIM || pc->pinned) pc = NULL;
 * Static cores (IVP_MT_STATIC, also the static object of a world anchor),
 * frozen and pinned cores receive no impulse.  The spring is a dependent
 * controller, so a movable partner shares the simulation unit and is
 * simulated whenever the spring runs. */
static ivp_core_t *stiff_spring_core(ivp_core_t *c)
{
    if (!c) return NULL;
    if (c->movement_state >= IVP_MT_NOT_SIM || c->pinned) return NULL;
    return c;
}

void ivp_controller_stiff_spring(const ivp_event_sim_t *event_sim,
                                   ivp_object_t *obj,
                                   void *ctx)
{
    ivp_stiff_spring_state_t *state = (ivp_stiff_spring_state_t *)ctx;
    if (!state || !state->active || state->broken) return;

    ivp_object_t *ends[2] = { state->obj_a, state->obj_b };
    if (obj != ivp_controller_pick_exec_object(ends, 2)) return;

    /* Reference IVP_Controller_Stiff_Spring::do_simulation_controller
     * (ivp_controller_stiff_spring.cxx:166-226). */
    /* first step: collect all data */
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
    const double dlen = (double)lvf_real_length_plus_normize(&dir_ws);
    state->current_length = (ivp_real_t)dlen;
    state->last_impulse = IVP_REAL(0.0);
    if (dlen < P_FLOAT_EPS) return;

    if (dlen > (double)(float)state->break_max_len && state->max_len_exceed_type == IVP_SFE_BREAK) {
        state->broken = true;
        if (state->broken_fn) state->broken_fn(state->broken_ctx, state);
        return;
    }

    pc0 = stiff_spring_core(pc0);
    pc1 = stiff_spring_core(pc1);
    if (!pc0 && !pc1) return;

    /* second step: calc and apply forces */
    ivp_scr_f_t scr;
    ivp_scr_f_init_translation_ws(&scr, pc0, pc1, &pos0_ws, &dir_ws, NULL, NULL);

    const float spring_constant = (float)state->stiffness;
    const float spring_damp = (float)state->damping;
    /* IVP_FLOAT arithmetic for the damping factor and its product with the
     * IVP_U_Float_Point delta_velocity_ds */
    const float damp_factor = spring_constant + spring_damp * (1.0f - spring_constant);
    const float dv_term = scr.delta_velocity_ds.k[0] * damp_factor;
    double a = ((double)(float)state->rest_length - dlen) * (double)event_sim->inverse_delta_time *
                   (double)spring_constant -
               (double)dv_term;
    const double tpm00 = scr.m[0][0];
    if (tpm00 < P_FLOAT_EPS && tpm00 > -P_FLOAT_EPS) return;

    if (state->max_dv > IVP_REAL(0.0)) {
        /* libivp extra: clamp the resulting relative velocity change. */
        const double lim = (double)state->max_dv;
        if (a > lim) a = lim;
        if (a < -lim) a = -lim;
    }
    ivp_vec3f_t impulses; /* IVP_U_Float_Point */
    lvf_zero(&impulses);
    impulses.k[0] = (float)(a / tpm00);
    state->last_impulse = (ivp_real_t)impulses.k[0];

    ivp_scr_f_exert_impulse_dim1(&scr, pc0, pc1, &impulses);
}

int ivp_stiff_spring_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_stiff_spring_state_t *s = (const ivp_stiff_spring_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[2] = { s->obj_a, s->obj_b };
    return ivp_controller_discover_objects(objs, 2, out_handles, max_handles);
}
