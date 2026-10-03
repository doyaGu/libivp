/* ivp_actuator_check_dist.c -- IVP_Actuator_Check_Dist / IVP_Anchor_Check_Dist
 *
 * Port of ivp_controller/ivp_actuator.cxx:245-383.  Attached states
 * (ivp_check_dist_attach) are driven by lazy hull synapses on both anchor
 * objects, as in the reference; an unattached state falls back to a
 * per-PSI evaluation through the controller callback.
 */
#include <ivp/controllers/ivp_actuator_check_dist.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>

#include <ivp/dynamics/ivp_cache_object.h>
#include <ivp/utility/ivp_active_value.h>

#include <math.h>
#include <string.h>
#include "controllers/ivp_controller_ref.h"

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_check_dist_state_init(ivp_check_dist_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->active = true;
    state->is_outside = 0;      /* reference: is_outside = IVP_FALSE */
    state->prev_is_outside = 0;
    state->use_object_centers = false;
}

/* hull_limit_exceeded_event: m_world_f_object_AT * object_pos (IVP_U_Float_Point) */
static void check_dist_anchor_ws(const ivp_object_t *obj,
                                 const ivp_vec3r_t *anchor_os,
                                 const ivp_vec3r_t *fallback_ws,
                                 bool use_center,
                                 ivp_vec3r_t *out)
{
    if (!obj) {
        *out = *fallback_ws;
    } else if (use_center) {
        *out = obj->physical_core->pos_world_f_core_last_psi;
    } else {
        ivp_transform_r_t m_world_f_object;
        ivp_ctl_m_world_f_object_AT(obj, &m_world_f_object);
        lvm_vmult4(&m_world_f_object, anchor_os, out);
    }
}

/* Reference hull_limit_exceeded_event(): evaluate and fire on change. */
static void check_dist_evaluate(ivp_check_dist_state_t *state, bool notify)
{
    ivp_vec3r_t pa, pb;
    check_dist_anchor_ws(state->obj_a, &state->anchor_a_os, &state->position_a_ws,
                         state->use_object_centers, &pa);
    check_dist_anchor_ws(state->obj_b, &state->anchor_b_os, &state->position_b_ws,
                         state->use_object_centers, &pb);

    double qlen = (double)lvp_quad_distance(&pa, &pb);
    ivp_real_t len = (ivp_real_t)sqrt(qlen); /* IVP_Inline_Math::ivp_sqrtf is sqrt(double) */
    state->current_distance = len;

    state->prev_is_outside = state->is_outside;
    if ((double)len < (double)(float)state->range) {
        if (state->is_outside) {
            state->is_outside = 0;
            if (notify && state->event_fn) {
                state->event_fn(state->event_ctx, state->obj_a, state->obj_b, len, true);
            }
        }
    } else {
        if (!state->is_outside) {
            state->is_outside = 1;
            if (notify && state->event_fn) {
                state->event_fn(state->event_ctx, state->obj_a, state->obj_b, len, false);
            }
        }
    }
}

void ivp_check_dist_state_setup(ivp_check_dist_state_t *state,
                                  ivp_object_t *obj_a,
                                  const ivp_vec3r_t *pos_a_ws,
                                  ivp_object_t *obj_b,
                                  const ivp_vec3r_t *pos_b_ws,
                                  ivp_real_t range)
{
    ivp_check_dist_state_init(state);
    state->obj_a = obj_a;
    state->obj_b = obj_b;
    state->position_a_ws = *pos_a_ws;
    state->position_b_ws = *pos_b_ws;
    state->range = (ivp_real_t)(float)range; /* IVP_FLOAT */
    /* init_anchor_check_dist: object_pos (IVP_U_Float_Point) =
     * m_world_f_object_AT^-1 * pos_ws */
    ivp_object_t *objs[2] = { obj_a, obj_b };
    const ivp_vec3r_t *pws[2] = { pos_a_ws, pos_b_ws };
    ivp_vec3r_t *aos[2] = { &state->anchor_a_os, &state->anchor_b_os };
    for (int i = 0; i < 2; i++) {
        if (!objs[i]) continue;
        ivp_transform_r_t m_world_f_object;
        ivp_ctl_m_world_f_object_AT(objs[i], &m_world_f_object);
        ivp_vec3f_t object_pos;
        lvm_vimult4_pf(&m_world_f_object, pws[i], &object_pos);
        *aos[i] = ivp_ctl_r(&object_pos);
    }
    /* Constructor: hull_limit_exceeded_event() before any listener exists. */
    check_dist_evaluate(state, false);
}

/* ── reference event driven mode ─────────────────────────────────────── */

static ivp_object_t *cd_anchor_object(ivp_check_dist_state_t *state, int i)
{
    ivp_object_t *o = i == 0 ? state->obj_a : state->obj_b;
    return o ? o : ivp_environment_get_static_object(state->env);
}

/* IVP_Actuator_Check_Dist::hull_limit_exceeded_event */
static void cd_hull_limit_exceeded(ivp_check_dist_state_t *state, bool notify)
{
    ivp_time_t t_now = ivp_environment_get_time(state->env);
    double pos_ws[2][3];
    for (int i = 0; i < 2; i++) {
        ivp_transform_r_t m_world_f_object;
        ivp_object_get_at_matrix(state->anchors[i].real_object, t_now, &m_world_f_object);
        const ivp_vec3f_t *op = &state->anchors[i].object_pos;
        for (int r = 0; r < 3; r++) {
            pos_ws[i][r] = (double)m_world_f_object.rot.rows[r].k[0] * (double)op->k[0] +
                           (double)m_world_f_object.rot.rows[r].k[1] * (double)op->k[1] +
                           (double)m_world_f_object.rot.rows[r].k[2] * (double)op->k[2] +
                           (double)m_world_f_object.pos.k[r];
        }
    }
    double dx = pos_ws[0][0] - pos_ws[1][0];
    double dy = pos_ws[0][1] - pos_ws[1][1];
    double dz = pos_ws[0][2] - pos_ws[1][2];
    dx *= dx;
    dy *= dy;
    dz *= dz;
    double qlen = dx + dy + dz;
    double len = sqrt(qlen); /* IVP_Inline_Math::ivp_sqrtf is sqrt(double) */
    state->current_distance = (ivp_real_t)len;

    state->prev_is_outside = state->is_outside;
    if (len < (double)(float)state->range) {
        if (state->is_outside) {
            state->is_outside = 0;
            if (notify && state->event_fn) state->event_fn(state->event_ctx, state->obj_a, state->obj_b, (ivp_real_t)len, true);
        }
    } else {
        if (!state->is_outside) {
            state->is_outside = 1;
            if (notify && state->event_fn) state->event_fn(state->event_ctx, state->obj_a, state->obj_b, (ivp_real_t)len, false);
        }
    }
    if (state->mod_is_outside) ivp_active_int_set_delayed(state->mod_is_outside, state->is_outside, true);

    /* re-insert in hull managers */
    double new_delta_hull = fabs(len - (double)(float)state->range) * (double)0.5f;
    for (int i = 0; i < 2; i++) {
        ivp_object_t *o = state->anchors[i].real_object;
        if (o->hull_manager) ivp_hull_mgr_update_lazy_synapse(o->hull_manager, &state->anchors[i].lh, t_now, new_delta_hull);
    }
}

static void cd_anchor_hull_limit_exceeded(ivp_listener_hull_t *lh, struct ivp_hull_mgr *hm, float v)
{
    (void)hm;
    (void)v;
    cd_hull_limit_exceeded(((ivp_check_dist_anchor_t *)lh)->state, true);
}

/* IVP_Anchor_Check_Dist::hull_manager_is_going_to_be_deleted_event:
 * delete l_actuator_check_dist */
static void cd_anchor_hm_deleted(ivp_listener_hull_t *lh, struct ivp_hull_mgr *hm)
{
    (void)hm;
    ivp_check_dist_detach(((ivp_check_dist_anchor_t *)lh)->state);
}

static const ivp_listener_hull_vt_t cd_anchor_vt = {
    IVP_HULL_ELEM_ANCHOR, cd_anchor_hull_limit_exceeded, cd_anchor_hm_deleted, NULL
};

bool ivp_check_dist_attach(ivp_check_dist_state_t *state, struct ivp_environment *env)
{
    if (!state || !env || state->attached) return false;
    state->env = env;
    ivp_time_t t_now = ivp_environment_get_time(env);
    for (int i = 0; i < 2; i++) {
        ivp_check_dist_anchor_t *a = &state->anchors[i];
        ivp_listener_hull_init(&a->lh, &cd_anchor_vt);
        a->state = state;
        a->real_object = cd_anchor_object(state, i);
        if (!a->real_object || !a->real_object->hull_manager) return false;
        /* init_anchor_check_dist: object_pos = m_world_f_object_AT^-1 * position_ws */
        ivp_transform_r_t m_world_f_object;
        ivp_object_get_at_matrix(a->real_object, t_now, &m_world_f_object);
        const ivp_vec3r_t *pws = i == 0 ? &state->position_a_ws : &state->position_b_ws;
        ivp_transform_r_t m_world_f_core;
        if (state->use_object_centers) {
            /* libivp extra: anchor at the mass centre (the core origin), so the
             * reference anchor path measures the centre distance */
            ivp_core_calc_at_matrix(a->real_object->physical_core, t_now, &m_world_f_core);
            pws = &m_world_f_core.pos;
        }
        lvm_vimult4_pf(&m_world_f_object, pws, &a->object_pos);
    }
    state->is_outside = 0;
    for (int i = 0; i < 2; i++) {
        (void)ivp_hull_mgr_insert_lazy_synapse(state->anchors[i].real_object->hull_manager, &state->anchors[i].lh, 0.0, 0.0);
    }
    state->attached = true;
    cd_hull_limit_exceeded(state, false); /* "does all we want" */
    return true;
}

void ivp_check_dist_detach(ivp_check_dist_state_t *state)
{
    if (!state || !state->attached) return;
    state->attached = false;
    if (state->deleted_fn) state->deleted_fn(state->event_ctx, state);
    for (int i = 0; i < 2; i++) {
        ivp_object_t *o = state->anchors[i].real_object;
        if (o && o->hull_manager) ivp_hull_mgr_remove_synapse(o->hull_manager, &state->anchors[i].lh);
    }
    state->active = false;
}

void ivp_check_dist_set_range(ivp_check_dist_state_t *state, ivp_real_t range)
{
    state->range = (ivp_real_t)(float)range; /* IVP_FLOAT member */
    if (state->attached) {
        cd_hull_limit_exceeded(state, true);
    } else {
        check_dist_evaluate(state, true);
    }
}

void ivp_check_dist_set_callback(ivp_check_dist_state_t *state,
                                   ivp_check_dist_event_fn_t fn,
                                   void *ctx)
{
    state->event_fn = fn;
    state->event_ctx = ctx;
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_check_dist(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx)
{
    (void)event_sim;
    ivp_check_dist_state_t *state = (ivp_check_dist_state_t *)ctx;
    if (!state->active || state->attached) return; /* attached: hull events */

    /* Evaluate once per PSI while any endpoint moves (frozen/static anchors
     * cannot change the distance; the reference only re-evaluates on hull
     * events, i.e. when an anchor moved). */
    ivp_object_t *ends[2] = { state->obj_a, state->obj_b };
    if (obj != ivp_controller_pick_exec_object(ends, 2)) return;

    check_dist_evaluate(state, true);
}
