/* ivp_constraint_local.c — General 6-DOF local constraint
 *
 * Wrapper of the reference IVP_Constraint_Local solver
 * (ivp_constraint_local_core.c, port of ivp_constraint_local.cxx).
 * The per-axis configuration of the wrapper is mapped onto the reference
 * template (axis types, borders, maximpulse + type) every PSI, which is how
 * the reference runtime-change API (fix/free/limit_*_axis,
 * change_max_*_impulse) updates the constraint.
 */
#include <ivp/controllers/ivp_constraint_local.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_mat3.h>

#include "dynamics/ivp_environment_events.h"

#include <string.h>

/* Reference IVP_Template_Constraint defaults (ivp_template_constraint.cxx:6-28) */
#define IVP_CL_DEFAULT_TAU          IVP_REAL(1.0)
#define IVP_CL_DEFAULT_DAMPING      IVP_REAL(1.0)

static void rebuild_core(ivp_constraint_local_state_t *state)
{
    ivp_constraint_template_t *t = &state->tmpl;
    ivp_constraint_template_init(t);
    /* reference frame at anchor_a (axes = R object axes) */
    ivp_constraint_template_set_constraint_Ros(t, state->obj_a, &state->anchor_a_os, NULL,
                                               3, 3, state->obj_b, NULL);
    /* attached frame at anchor_b, axes following the current relative pose */
    ivp_constraint_template_set_attached_Aos(t, &state->anchor_b_os, NULL, -1);
    t->force_factor = state->tau;
    t->damp_factor = state->damping;
    ivp_constraint_local_core_init(&state->core, t);
}

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_constraint_local_init(ivp_constraint_local_state_t *state,
                               ivp_object_t *obj_a,
                               const ivp_vec3r_t *anchor_a_os,
                               ivp_object_t *obj_b,
                               const ivp_vec3r_t *anchor_b_os)
{
    memset(state, 0, sizeof(*state));

    state->obj_a       = obj_a;
    state->obj_b       = obj_b;
    state->anchor_a_os = *anchor_a_os;
    state->anchor_b_os = *anchor_b_os;

    state->ref_q_a = obj_a ? obj_a->physical_core->q_world_f_core_last_psi : ivp_quatr_identity();
    state->ref_q_b = obj_b ? obj_b->physical_core->q_world_f_core_last_psi : ivp_quatr_identity();

    /* Default: all axes FIXED, no impulse limit */
    for (int i = 0; i < 6; i++) {
        state->axes[i].mode        = IVP_CAXIS_FIXED;
        state->axes[i].border_min  = IVP_REAL(0.0);
        state->axes[i].border_max  = IVP_REAL(0.0);
        state->axes[i].max_impulse = IVP_REAL(1e10);
        state->axes[i].exceed_mode = IVP_CFORCE_NONE;
    }

    state->tau              = IVP_CL_DEFAULT_TAU;
    state->damping          = IVP_CL_DEFAULT_DAMPING;
    state->limited_axis_stiffness = IVP_REAL(0.3);
    state->solver_iterations = 1;
    state->active           = true;
    state->broken           = false;

    rebuild_core(state);
}

/* ── Per-axis configuration (reference setters wake the constraint) ───── */

static void cl_wake(ivp_constraint_local_state_t *state)
{
    ivp_constraint_local_core_wake(&state->core);
}

static void set_axis(ivp_constraint_local_state_t *state, int idx,
                     ivp_constraint_axis_mode_t mode, ivp_real_t mn, ivp_real_t mx)
{
    if (idx < 0 || idx >= 6) return;
    state->axes[idx].mode = mode;
    if (mode == IVP_CAXIS_LIMITED) {
        state->axes[idx].border_min = mn;
        state->axes[idx].border_max = mx;
    }
    cl_wake(state);
}

void ivp_constraint_local_fix_translation(ivp_constraint_local_state_t *state, int axis)
{
    if (axis < 0 || axis > 2) return;
    set_axis(state, axis, IVP_CAXIS_FIXED, IVP_REAL(0.0), IVP_REAL(0.0));
}

void ivp_constraint_local_free_translation(ivp_constraint_local_state_t *state, int axis)
{
    if (axis < 0 || axis > 2) return;
    set_axis(state, axis, IVP_CAXIS_FREE, IVP_REAL(0.0), IVP_REAL(0.0));
}

void ivp_constraint_local_limit_translation(ivp_constraint_local_state_t *state, int axis,
                                            ivp_real_t border_min, ivp_real_t border_max)
{
    if (axis < 0 || axis > 2) return;
    set_axis(state, axis, IVP_CAXIS_LIMITED, border_min, border_max);
}

void ivp_constraint_local_fix_rotation(ivp_constraint_local_state_t *state, int axis)
{
    if (axis < 0 || axis > 2) return;
    set_axis(state, 3 + axis, IVP_CAXIS_FIXED, IVP_REAL(0.0), IVP_REAL(0.0));
}

void ivp_constraint_local_free_rotation(ivp_constraint_local_state_t *state, int axis)
{
    if (axis < 0 || axis > 2) return;
    set_axis(state, 3 + axis, IVP_CAXIS_FREE, IVP_REAL(0.0), IVP_REAL(0.0));
}

void ivp_constraint_local_limit_rotation(ivp_constraint_local_state_t *state, int axis,
                                         ivp_real_t min_rad, ivp_real_t max_rad)
{
    if (axis < 0 || axis > 2) return;
    set_axis(state, 3 + axis, IVP_CAXIS_LIMITED, min_rad, max_rad);
}

void ivp_constraint_local_set_max_impulse(ivp_constraint_local_state_t *state,
                                          int axis_index,
                                          ivp_real_t max_impulse,
                                          ivp_constraint_force_exceed_t mode)
{
    if (axis_index < 0 || axis_index >= 6) return;
    state->axes[axis_index].max_impulse = max_impulse;
    state->axes[axis_index].exceed_mode = mode;
    cl_wake(state);
}

/* ── Enable / Disable / Broken ─────────────────────────────────────────── */

void ivp_constraint_local_set_active(ivp_constraint_local_state_t *state, bool active)
{
    if (active && !state->active) cl_wake(state);  /* reference activate() */
    state->active = active;
}

bool ivp_constraint_local_is_active(const ivp_constraint_local_state_t *state)
{
    return state->active;
}

bool ivp_constraint_local_is_broken(const ivp_constraint_local_state_t *state)
{
    return state->broken;
}

void ivp_constraint_local_reset_broken(ivp_constraint_local_state_t *state, bool reactivate)
{
    state->broken = false;
    state->core.broken = false;
    if (reactivate) {
        state->active = true;
        cl_wake(state);
    }
}

/* ── Runtime rebinding ─────────────────────────────────────────────────── */

void ivp_constraint_local_rebind_anchors(ivp_constraint_local_state_t *state,
                                          const ivp_vec3r_t *anchor_a_os,
                                          const ivp_vec3r_t *anchor_b_os)
{
    state->anchor_a_os = *anchor_a_os;
    state->anchor_b_os = *anchor_b_os;
    /* reference change_fixing_point_Ros + attached anchor: the frame axes
     * keep following the pose recorded in the core */
    ivp_cmatrix_t afs = state->core.m_Afs_f_Acs;
    rebuild_core(state);
    /* keep the attached orientation of the previous binding */
    double v[3] = { (double)anchor_b_os->k[0], (double)anchor_b_os->k[1], (double)anchor_b_os->k[2] };
    for (int i = 0; i < 3; i++) {
        state->core.m_Afs_f_Acs.rows[i][0] = afs.rows[i][0];
        state->core.m_Afs_f_Acs.rows[i][1] = afs.rows[i][1];
        state->core.m_Afs_f_Acs.rows[i][2] = afs.rows[i][2];
    }
    for (int i = 0; i < 3; i++) {
        state->core.m_Afs_f_Acs.vv[i] = -(afs.rows[i][0] * v[0] + afs.rows[i][1] * v[1] +
                                          afs.rows[i][2] * v[2]);
    }
    cl_wake(state);
}

void ivp_constraint_local_rebind_reference(ivp_constraint_local_state_t *state)
{
    state->ref_q_a = state->obj_a ? state->obj_a->physical_core->q_world_f_core_last_psi : ivp_quatr_identity();
    state->ref_q_b = state->obj_b ? state->obj_b->physical_core->q_world_f_core_last_psi : ivp_quatr_identity();
    rebuild_core(state);
    cl_wake(state);
}

void ivp_constraint_local_relax_attached(ivp_constraint_local_state_t *state)
{
    ivp_constraint_local_core_relax_attached(&state->core);
}

int ivp_constraint_local_discovery(void *ctx, ivp_environment_t *env,
                                   ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_constraint_local_state_t *s = (const ivp_constraint_local_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[2] = { s->obj_a, s->obj_b };
    return ivp_controller_discover_objects(objs, 2, out_handles, max_handles);
}

/* ── Controller callback ───────────────────────────────────────────────── */

static void sync_core(ivp_constraint_local_state_t *state)
{
    ivp_constraint_local_core_t *c = &state->core;
    ivp_constraint_local_core_set_factors(c, state->tau, state->damping);
    c->limited_axis_stiffness = state->limited_axis_stiffness;
    c->use_maxforce = false;
    for (int i = 0; i < 6; i++) {
        const ivp_constraint_axis_config_t *cfg = &state->axes[i];
        c->fixed[i] = (int)cfg->mode; /* FIXED bit of limited axes is recomputed per PSI */
        c->borderleft_Rfs[i] = cfg->border_min;
        c->borderright_Rfs[i] = cfg->border_max;
        c->halfimpulse[i] = cfg->max_impulse;
        c->maxforce_type[i] = cfg->exceed_mode;
        if (cfg->exceed_mode != IVP_CFE_NONE) c->use_maxforce = true;
    }
    ivp_constraint_local_core_sort_mappings(c);
}

void ivp_controller_constraint_local(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx)
{
    ivp_constraint_local_state_t *state = (ivp_constraint_local_state_t *)ctx;
    if (!state || !state->active || state->broken) return;

    ivp_object_t *ends[2] = { state->obj_a, state->obj_b };
    if (obj != ivp_controller_pick_exec_object(ends, 2)) return;

    sync_core(state);
    state->core.enabled = true;
    bool broke = ivp_constraint_local_core_simulate(&state->core, event_sim);
    for (int i = 0; i < 6; i++) {
        state->accumulated_impulse[i] = ivp_fabs(state->core.last_impulse[i]);
    }
    if (broke) {
        state->broken = true;
        state->active = false;
        if (event_sim->environment) {
            ivp_environment_emit_event_internal(event_sim->environment,
                                                IVP_EVENT_CONSTRAINT_BROKEN,
                                                event_sim->delta_time,
                                                obj->dense_index, 0);
        }
    }
}
