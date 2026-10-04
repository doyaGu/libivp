/* ivp_actuator_stabilizer.c -- Anti-roll bar stabilizer implementation
 *
 * 4 anchor points as 2 pairs. Applies the reference force proportional to
 * (dist[1] - dist[0]) * stabi_constant.  Its sign increases an existing
 * distance difference; this counter-intuitive legacy behavior is intentional.
 */

#include <ivp/controllers/ivp_actuator_stabilizer.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>
#include "controllers/ivp_controller_ref.h"

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_stabilizer_state_init(ivp_stabilizer_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->active         = true;
    state->stabi_constant = IVP_REAL(0.0);
}

void ivp_stabilizer_state_setup(ivp_stabilizer_state_t *state,
                                 ivp_object_t *obj0,
                                 ivp_object_t *obj1,
                                 ivp_object_t *obj2,
                                 ivp_object_t *obj3,
                                 const ivp_vec3r_t *anchor0,
                                 const ivp_vec3r_t *anchor1,
                                 const ivp_vec3r_t *anchor2,
                                 const ivp_vec3r_t *anchor3,
                                 ivp_real_t stabi_constant)
{
    state->obj[0] = obj0;
    state->obj[1] = obj1;
    state->obj[2] = obj2;
    state->obj[3] = obj3;
    /* IVP_Actuator_Four_Point: IVP_Anchor::init_anchor for every anchor
     * (object space -> world -> core_pos) */
    const ivp_vec3r_t *anchors[4] = { anchor0, anchor1, anchor2, anchor3 };
    ivp_environment_t *env = ivp_ctl_env_of(state->obj, 4);
    for (int i = 0; i < 4; i++) {
        ivp_ctl_anchor_setup_os(state->obj[i], env, anchors[i], &state->anchor_os[i], &state->anchors[i]);
    }
    state->stabi_constant = (ivp_real_t)(float)stabi_constant; /* IVP_FLOAT */
}

/* ── Runtime adjustment ────────────────────────────────────────────────── */

void ivp_stabilizer_set_constant(ivp_stabilizer_state_t *state,
                                  ivp_real_t value)
{
    /* Reference set_stabi_constant(IVP_DOUBLE): IVP_FLOAT member, anchor 0
     * object ensure_in_simulation(). */
    if ((double)(float)state->stabi_constant == (double)value) return;
    state->stabi_constant = (ivp_real_t)(float)value;
    ivp_controller_wake_object(state->obj[0]);
}

/* ── Controller callback ───────────────────────────────────────────────── */

/* Port of IVP_Actuator_Stabilizer::do_simulation_controller
 * (ivp_controller/ivp_actuator.cxx:1052-1099):
 *   dir_i  = pos0_i - pos1_i, dist_i = |dir_i|
 *   force  = (dist_1 - dist_0) * stabi_constant * dt
 *   pair i: second anchor pushed by +dir_i * force, first by -dir_i * force,
 *           force negated for the second pair.
 * Note: the reference IVP_Actuator_Four_Point constructor never announces the
 * controller to the environment, so a reference stabilizer created through
 * the factory is inert; libivp keeps the explicit registration. */
void ivp_controller_stabilizer(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx)
{
    ivp_stabilizer_state_t *state = (ivp_stabilizer_state_t *)ctx;
    if (!state || !state->active) return;

    if (obj != ivp_controller_pick_exec_object(state->obj, 4)) return;

    ivp_environment_t *env = event_sim->environment;
    ivp_core_t *core[4];
    ivp_vec3f_t core_pos[4];
    for (int i = 0; i < 4; i++) {
        core[i] = ivp_ctl_anchor_friction_core(state->obj[i], env);   /* l_anchor_object->friction_core */
        core_pos[i] = ivp_ctl_f(&state->anchor_os[i]);
    }

    ivp_vec3f_t dir[2];
    ivp_vec3r_t pos0_ws[2], pos1_ws[2];
    double dist[2];
    for (int i = 0; i < 2; i++) {
        ivp_ctl_anchor_pos_ws(core[i * 2], &core_pos[i * 2], &pos0_ws[i]);
        ivp_ctl_anchor_pos_ws(core[i * 2 + 1], &core_pos[i * 2 + 1], &pos1_ws[i]);
        lvf_subtract2_p(&dir[i], &pos0_ws[i], &pos1_ws[i]); /* force direction world coords */
        dist[i] = (double)lvf_real_length_plus_normize(&dir[i]);
    }

    const double diff_dist = dist[1] - dist[0];
    state->current_diff_dist = (ivp_real_t)diff_dist;

    double force = diff_dist * (double)(float)state->stabi_constant * (double)event_sim->delta_time;

    for (int i = 0; i < 2; i++) {
        ivp_vec3f_t force_vector;
        lvf_set_multiple(&force_vector, &dir[i], (D)force);

        ivp_object_t *o0 = state->obj[i * 2];
        ivp_object_t *o1 = state->obj[i * 2 + 1];
        if (ivp_controller_object_pushable(o1)) {
            ivp_ctl_async_push_core_ws(core[i * 2 + 1], &pos1_ws[i], &force_vector);
        }
        if (ivp_controller_object_pushable(o0)) {
            lvf_mult(&force_vector, (D)-1.0);
            ivp_ctl_async_push_core_ws(core[i * 2], &pos0_ws[i], &force_vector);
        }

        force *= -1; /* reverted force direction for other anchor pair */
    }
}

int ivp_stabilizer_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_stabilizer_state_t *s = (const ivp_stabilizer_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[4] = { s->obj[0], s->obj[1], s->obj[2], s->obj[3] };
    return ivp_controller_discover_objects(objs, 4, out_handles, max_handles);
}
