/* ivp_constraint.c -- Constraint system implementation
 *
 * All constraint types are thin wrappers around the reference
 * IVP_Constraint_Local solver (ivp_constraint_local_core.c), configured the
 * way IVP_Template_Constraint::set_* configures it
 * (ivp_controller/ivp_template_constraint.{hxx,cxx}).
 */

#include <ivp/controllers/ivp_constraint.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/events/ivp_event.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_mat3.h>

#include "dynamics/ivp_environment_events.h"

#include <string.h>

/* Reference IVP_Template_Constraint defaults (ivp_template_constraint.cxx:6-28) */
#define IVP_CONSTRAINT_DEFAULT_TAU          IVP_REAL(1.0)
#define IVP_CONSTRAINT_DEFAULT_DAMPING      IVP_REAL(1.0)

/* ── Initialization ─────────────────────────────────────────────────────── */

static void init_common(ivp_constraint_state_t *state, ivp_constraint_type_t type,
                        ivp_object_t *obj_r, ivp_object_t *obj_a)
{
    memset(state, 0, sizeof(*state));
    state->type = type;
    state->obj_a = obj_r;
    state->obj_b = obj_a;
    state->tau = IVP_CONSTRAINT_DEFAULT_TAU;
    state->damping = IVP_CONSTRAINT_DEFAULT_DAMPING;
    state->solver_iterations = 1;
    state->active = true;
    state->force_exceed_action = IVP_CFE_NONE;
    state->limits.limit_stiffness = IVP_REAL(0.3);
    state->ref_q_a = obj_r ? obj_r->physical_core->q_world_f_core_last_psi : ivp_quatr_identity();
    state->ref_q_b = obj_a ? obj_a->physical_core->q_world_f_core_last_psi : ivp_quatr_identity();
}

static void finish_init(ivp_constraint_state_t *state, ivp_constraint_template_t *t)
{
    t->force_factor = state->tau;
    t->damp_factor = state->damping;
    ivp_constraint_local_core_init(&state->core, t);
    state->core_ready = true;
}

void ivp_constraint_init_ball_socket(ivp_constraint_state_t *state,
                                      ivp_object_t *obj_a,
                                      const ivp_vec3r_t *anchor_a_os,
                                      ivp_object_t *obj_b,
                                      const ivp_vec3r_t *anchor_b_os)
{
    init_common(state, IVP_CONSTRAINT_BALL_SOCKET, obj_a, obj_b);
    state->anchor_a_os = *anchor_a_os;
    state->anchor_b_os = *anchor_b_os;
    ivp_constraint_template_t t;
    ivp_constraint_template_init(&t);
    ivp_constraint_template_set_constraint_Ros(&t, obj_a, anchor_a_os, NULL, 3, 0, obj_b, NULL);
    ivp_constraint_template_set_attached_Aos(&t, anchor_b_os, NULL, -1);
    finish_init(state, &t);
}

void ivp_constraint_init_ball_socket_ws(ivp_constraint_state_t *state,
                                        ivp_object_t *obj_r,
                                        const ivp_vec3r_t *anchor_ws,
                                        ivp_object_t *obj_a)
{
    init_common(state, IVP_CONSTRAINT_BALL_SOCKET, obj_r, obj_a);
    if (obj_r) ivp_transform_inv_point(&state->anchor_a_os, &obj_r->physical_core->m_world_f_core_last_psi, anchor_ws);
    else state->anchor_a_os = *anchor_ws;
    if (obj_a) ivp_transform_inv_point(&state->anchor_b_os, &obj_a->physical_core->m_world_f_core_last_psi, anchor_ws);
    else state->anchor_b_os = *anchor_ws;
    ivp_constraint_template_t t;
    ivp_constraint_template_init(&t);
    ivp_constraint_template_set_ballsocket_ws(&t, obj_r, anchor_ws, obj_a);
    finish_init(state, &t);
}

void ivp_constraint_init_ball_socket_tense_Ros(ivp_constraint_state_t *state,
                                               ivp_object_t *obj_r,
                                               const ivp_vec3r_t *anchor_Ros,
                                               ivp_object_t *obj_a,
                                               const ivp_vec3r_t *distanceAR_Ros)
{
    init_common(state, IVP_CONSTRAINT_BALL_SOCKET_TENSE, obj_r, obj_a);
    state->anchor_a_os = *anchor_Ros;
    ivp_constraint_template_t t;
    ivp_constraint_template_init(&t);
    ivp_constraint_template_set_ballsocket_tense_Ros(&t, obj_r, anchor_Ros, obj_a, distanceAR_Ros);
    finish_init(state, &t);
}

void ivp_constraint_init_hinge(ivp_constraint_state_t *state,
                                 ivp_object_t *obj_a,
                                 const ivp_vec3r_t *anchor_a_os,
                                 const ivp_vec3r_t *axis_a_os,
                                 ivp_object_t *obj_b,
                                 const ivp_vec3r_t *anchor_b_os,
                                 const ivp_vec3r_t *axis_b_os)
{
    init_common(state, IVP_CONSTRAINT_HINGE, obj_a, obj_b);
    state->anchor_a_os = *anchor_a_os;
    state->anchor_b_os = *anchor_b_os;
    state->axis_a_os = *axis_a_os;
    state->axis_b_os = *axis_b_os;
    ivp_constraint_template_t t;
    ivp_constraint_template_init(&t);
    /* hinge: transdim 3, rotdim 2, free axis = Rfs Z */
    ivp_constraint_template_set_constraint_Ros(&t, obj_a, anchor_a_os, axis_a_os, 3, 2, obj_b, NULL);
    ivp_constraint_template_set_attached_Aos(&t, anchor_b_os, axis_b_os, 2);
    finish_init(state, &t);
}

void ivp_constraint_init_hinge_ws(ivp_constraint_state_t *state,
                                  ivp_object_t *obj_r,
                                  const ivp_vec3r_t *anchor_ws,
                                  const ivp_vec3r_t *free_axis_ws,
                                  ivp_object_t *obj_a)
{
    init_common(state, IVP_CONSTRAINT_HINGE, obj_r, obj_a);
    if (obj_r) {
        ivp_transform_inv_point(&state->anchor_a_os, &obj_r->physical_core->m_world_f_core_last_psi, anchor_ws);
        ivp_transform_inv_dir(&state->axis_a_os, &obj_r->physical_core->m_world_f_core_last_psi, free_axis_ws);
    } else {
        state->anchor_a_os = *anchor_ws;
        state->axis_a_os = *free_axis_ws;
    }
    if (obj_a) {
        ivp_transform_inv_point(&state->anchor_b_os, &obj_a->physical_core->m_world_f_core_last_psi, anchor_ws);
        ivp_transform_inv_dir(&state->axis_b_os, &obj_a->physical_core->m_world_f_core_last_psi, free_axis_ws);
    } else {
        state->anchor_b_os = *anchor_ws;
        state->axis_b_os = *free_axis_ws;
    }
    ivp_constraint_template_t t;
    ivp_constraint_template_init(&t);
    ivp_constraint_template_set_hinge_ws(&t, obj_r, anchor_ws, free_axis_ws, obj_a);
    finish_init(state, &t);
}

void ivp_constraint_init_cardan_ws(ivp_constraint_state_t *state,
                                   ivp_object_t *obj_r,
                                   const ivp_vec3r_t *anchor_ws,
                                   const ivp_vec3r_t *fixed_axis_ws,
                                   ivp_object_t *obj_a)
{
    init_common(state, IVP_CONSTRAINT_CARDAN, obj_r, obj_a);
    if (obj_r) ivp_transform_inv_point(&state->anchor_a_os, &obj_r->physical_core->m_world_f_core_last_psi, anchor_ws);
    else state->anchor_a_os = *anchor_ws;
    ivp_constraint_template_t t;
    ivp_constraint_template_init(&t);
    ivp_constraint_template_set_cardanjoint_ws(&t, obj_r, anchor_ws, fixed_axis_ws, obj_a);
    finish_init(state, &t);
}

void ivp_constraint_init_fixed(ivp_constraint_state_t *state,
                                 ivp_object_t *obj_a,
                                 const ivp_vec3r_t *anchor_a_os,
                                 ivp_object_t *obj_b,
                                 const ivp_vec3r_t *anchor_b_os)
{
    init_common(state, IVP_CONSTRAINT_FIXED, obj_a, obj_b);
    state->anchor_a_os = *anchor_a_os;
    state->anchor_b_os = *anchor_b_os;
    ivp_constraint_template_t t;
    ivp_constraint_template_init(&t);
    ivp_constraint_template_set_constraint_Ros(&t, obj_a, anchor_a_os, NULL, 3, 3, obj_b, NULL);
    ivp_constraint_template_set_attached_Aos(&t, anchor_b_os, NULL, -1);
    finish_init(state, &t);
}

void ivp_constraint_init_fixed_ws(ivp_constraint_state_t *state,
                                  ivp_object_t *obj_r,
                                  ivp_object_t *obj_a)
{
    init_common(state, IVP_CONSTRAINT_FIXED, obj_r, obj_a);
    ivp_constraint_template_t t;
    ivp_constraint_template_init(&t);
    ivp_constraint_template_set_fixed(&t, obj_r, obj_a);
    finish_init(state, &t);
}

void ivp_constraint_init_general_ws(ivp_constraint_state_t *state,
                                    ivp_object_t *obj_r,
                                    const ivp_vec3r_t *anchor_ws,
                                    const ivp_vec3r_t *known_axis_ws,
                                    unsigned fixed_trans_dim,
                                    unsigned fixed_rot_dim,
                                    ivp_object_t *obj_a)
{
    init_common(state, IVP_CONSTRAINT_GENERAL, obj_r, obj_a);
    ivp_constraint_template_t t;
    ivp_constraint_template_init(&t);
    ivp_constraint_template_set_constraint_ws(&t, obj_r, anchor_ws, known_axis_ws,
                                              fixed_trans_dim > 3 ? 3u : fixed_trans_dim,
                                              fixed_rot_dim > 3 ? 3u : fixed_rot_dim,
                                              obj_a, NULL);
    finish_init(state, &t);
}

void ivp_constraint_init_from_template(ivp_constraint_state_t *state,
                                       const ivp_constraint_template_t *tmpl)
{
    init_common(state, IVP_CONSTRAINT_GENERAL, tmpl->obj_r, tmpl->obj_a);
    state->tau = tmpl->force_factor;
    state->damping = tmpl->damp_factor;
    state->limits.limit_stiffness = tmpl->limited_axis_stiffness;
    ivp_constraint_local_core_init(&state->core, tmpl);
    state->core_ready = true;
}

void ivp_constraint_set_limits(ivp_constraint_state_t *state,
                                 ivp_real_t min_angle,
                                 ivp_real_t max_angle,
                                 ivp_real_t stiffness,
                                 ivp_real_t damping)
{
    state->limits.limited         = true;
    state->limits.min_angle       = min_angle;
    state->limits.max_angle       = max_angle;
    state->limits.limit_stiffness = stiffness;
    state->limits.limit_damping   = damping;
    if (state->core_ready) ivp_constraint_local_core_wake(&state->core);
}

int ivp_constraint_discovery(void *ctx, ivp_environment_t *env,
                             ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_constraint_state_t *s = (const ivp_constraint_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[2] = { s->obj_a, s->obj_b };
    return ivp_controller_discover_objects(objs, 2, out_handles, max_handles);
}

/* Push the user-visible parameters into the reference solver core. */
static void sync_core(ivp_constraint_state_t *state)
{
    ivp_constraint_local_core_t *c = &state->core;
    ivp_constraint_local_core_set_factors(c, state->tau, state->damping);

    if (state->type == IVP_CONSTRAINT_HINGE) {
        /* free hinge axis = rotation Z of the reference frame */
        if (state->limits.limited) {
            /* the FIXED bit of a limited axis is recomputed every PSI */
            c->fixed[5] = IVP_CONSTRAINT_AXIS_LIMITED;
            c->borderleft_Rfs[5] = state->limits.min_angle;
            c->borderright_Rfs[5] = state->limits.max_angle;
            c->limited_axis_stiffness = state->limits.limit_stiffness;
        } else {
            c->fixed[5] = IVP_CONSTRAINT_AXIS_FREE;
        }
        ivp_constraint_local_core_sort_mappings(c);
    }

    if (state->force_exceed_action != IVP_CFE_NONE &&
        (state->break_force_threshold > IVP_REAL(0.0) || state->break_torque_threshold > IVP_REAL(0.0))) {
        c->use_maxforce = true;
        for (int i = 0; i < 3; i++) {
            if (state->break_force_threshold > IVP_REAL(0.0)) {
                c->halfimpulse[i] = state->break_force_threshold;
                c->maxforce_type[i] = state->force_exceed_action;
            } else {
                c->halfimpulse[i] = (ivp_real_t)IVP_FLOAT_MAX;
                c->maxforce_type[i] = IVP_CFE_NONE;
            }
            if (state->break_torque_threshold > IVP_REAL(0.0)) {
                c->halfimpulse[i + 3] = state->break_torque_threshold;
                c->maxforce_type[i + 3] = state->force_exceed_action;
            } else {
                c->halfimpulse[i + 3] = (ivp_real_t)IVP_FLOAT_MAX;
                c->maxforce_type[i + 3] = IVP_CFE_NONE;
            }
        }
    } else if (state->type != IVP_CONSTRAINT_GENERAL) {
        c->use_maxforce = false;
    }
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_constraint(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx)
{
    ivp_constraint_state_t *state = (ivp_constraint_state_t *)ctx;
    if (!state || !state->active || !state->core_ready) return;

    ivp_object_t *ends[2] = { state->obj_a, state->obj_b };
    ivp_object_t *exec_obj = ivp_controller_pick_exec_object(ends, 2);
    if (!exec_obj || obj != exec_obj) return;

    sync_core(state);
    state->core.enabled = true;
    bool broke = ivp_constraint_local_core_simulate(&state->core, event_sim);

    const ivp_real_t *li = state->core.last_impulse;
    state->accumulated_impulse =
        ivp_sqrt(li[0] * li[0] + li[1] * li[1] + li[2] * li[2]) +
        ivp_sqrt(li[3] * li[3] + li[4] * li[4] + li[5] * li[5]);

    if (broke) {
        /* Reference: fire_event_constraint_broken(this); deactivate(); */
        state->active = false;
        state->is_broken = true;
        if (event_sim->environment) {
            ivp_environment_emit_event_internal(event_sim->environment,
                                                IVP_EVENT_CONSTRAINT_BROKEN,
                                                event_sim->delta_time,
                                                exec_obj->dense_index, 0);
        }
    }
}
