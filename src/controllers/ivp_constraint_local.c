/* ivp_constraint_local.c — General 6-DOF local constraint
 *
 * Solves a bilateral constraint with per-axis free/fixed/limited modes.
 * Uses sequential impulse with Baumgarte stabilization, similar to the
 * existing constraint solver but with configurable DOFs and force limits.
 */
#include <ivp/controllers/ivp_constraint_local.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_mat3.h>
#include <string.h>

/* ── Default parameters ────────────────────────────────────────────────── */

#define IVP_CL_DEFAULT_TAU          IVP_REAL(0.2)
#define IVP_CL_DEFAULT_DAMPING      IVP_REAL(0.9)
#define IVP_CL_DEFAULT_ITERATIONS   4

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

    /* Record reference orientations */
    if (obj_a) state->ref_q_a = obj_a->core.q_world_f_core_last_psi;
    else       state->ref_q_a = ivp_quatr_identity();
    if (obj_b) state->ref_q_b = obj_b->core.q_world_f_core_last_psi;
    else       state->ref_q_b = ivp_quatr_identity();

    /* Default: all axes FIXED */
    for (int i = 0; i < 6; i++) {
        state->axes[i].mode        = IVP_CAXIS_FIXED;
        state->axes[i].border_min  = IVP_REAL(0.0);
        state->axes[i].border_max  = IVP_REAL(0.0);
        state->axes[i].max_impulse = IVP_REAL(1e10);
        state->axes[i].exceed_mode = IVP_CFORCE_NONE;
    }

    state->tau              = IVP_CL_DEFAULT_TAU;
    state->damping          = IVP_CL_DEFAULT_DAMPING;
    state->solver_iterations = IVP_CL_DEFAULT_ITERATIONS;
    state->active           = true;
    state->broken           = false;
}

/* ── Per-axis configuration ────────────────────────────────────────────── */

void ivp_constraint_local_fix_translation(ivp_constraint_local_state_t *state,
                                          int axis)
{
    if (axis >= 0 && axis < 3)
        state->axes[axis].mode = IVP_CAXIS_FIXED;
}

void ivp_constraint_local_free_translation(ivp_constraint_local_state_t *state,
                                           int axis)
{
    if (axis >= 0 && axis < 3)
        state->axes[axis].mode = IVP_CAXIS_FREE;
}

void ivp_constraint_local_limit_translation(ivp_constraint_local_state_t *state,
                                            int axis,
                                            ivp_real_t border_min,
                                            ivp_real_t border_max)
{
    if (axis < 0 || axis >= 3) return;
    state->axes[axis].mode       = IVP_CAXIS_LIMITED;
    state->axes[axis].border_min = border_min;
    state->axes[axis].border_max = border_max;
}

void ivp_constraint_local_fix_rotation(ivp_constraint_local_state_t *state,
                                       int axis)
{
    if (axis >= 0 && axis < 3)
        state->axes[3 + axis].mode = IVP_CAXIS_FIXED;
}

void ivp_constraint_local_free_rotation(ivp_constraint_local_state_t *state,
                                        int axis)
{
    if (axis >= 0 && axis < 3)
        state->axes[3 + axis].mode = IVP_CAXIS_FREE;
}

void ivp_constraint_local_limit_rotation(ivp_constraint_local_state_t *state,
                                         int axis,
                                         ivp_real_t min_rad,
                                         ivp_real_t max_rad)
{
    if (axis < 0 || axis >= 3) return;
    state->axes[3 + axis].mode       = IVP_CAXIS_LIMITED;
    state->axes[3 + axis].border_min = min_rad;
    state->axes[3 + axis].border_max = max_rad;
}

void ivp_constraint_local_set_max_impulse(ivp_constraint_local_state_t *state,
                                          int axis_index,
                                          ivp_real_t max_impulse,
                                          ivp_constraint_force_exceed_t mode)
{
    if (axis_index < 0 || axis_index >= 6) return;
    state->axes[axis_index].max_impulse = max_impulse;
    state->axes[axis_index].exceed_mode = mode;
}

/* ── Helpers ───────────────────────────────────────────────────────────── */

static void get_anchor_ws(const ivp_object_t *obj,
                          const ivp_vec3r_t *anchor_os,
                          ivp_vec3r_t *out)
{
    if (obj) {
        ivp_transform_point(out, &obj->core.m_world_f_core_last_psi, anchor_os);
    } else {
        *out = *anchor_os;
    }
}

/* Get a reference-frame axis direction in world space.
 * axis_idx: 0=X, 1=Y, 2=Z of the reference object's frame. */
static void get_ref_axis_ws(const ivp_constraint_local_state_t *state,
                            int axis_idx,
                            ivp_vec3r_t *out)
{
    ivp_vec3r_t local = {.k = {0, 0, 0}};
    local.k[axis_idx] = IVP_REAL(1.0);

    if (state->obj_a) {
        ivp_mat3r_vmult(out, &state->obj_a->core.m_world_f_core_last_psi.rot, &local);
    } else {
        *out = local;
    }
}

/* Compute relative orientation error as a 3-vector (axis-angle approx).
 * Uses the same approach as the fixed constraint: q_err = inv(q_ref) * q_rel */
static void compute_orientation_error(const ivp_constraint_local_state_t *state,
                                      ivp_vec3r_t *error_out)
{
    ivp_quatr_t q_a, q_b;
    if (state->obj_a)
        q_a = state->obj_a->core.q_world_f_core_last_psi;
    else
        q_a = ivp_quatr_identity();
    q_b = state->obj_b->core.q_world_f_core_last_psi;

    /* q_rel = inv(q_a) * q_b */
    ivp_quatr_t q_a_inv = {-q_a.x, -q_a.y, -q_a.z, q_a.w};
    ivp_quatr_t q_rel;
    ivp_quat_mult(&q_rel, &q_a_inv, &q_b);

    /* q_ref = inv(ref_q_a) * ref_q_b */
    ivp_quatr_t ref_a_inv = {-state->ref_q_a.x, -state->ref_q_a.y,
                              -state->ref_q_a.z,  state->ref_q_a.w};
    ivp_quatr_t q_ref;
    ivp_quat_mult(&q_ref, &ref_a_inv, &state->ref_q_b);

    /* q_err = inv(q_ref) * q_rel */
    ivp_quatr_t q_ref_inv = {-q_ref.x, -q_ref.y, -q_ref.z, q_ref.w};
    ivp_quatr_t q_err;
    ivp_quat_mult(&q_err, &q_ref_inv, &q_rel);

    /* axis-angle ≈ 2 * vec(q_err), ensure shortest path */
    ivp_real_t sign = (q_err.w < IVP_REAL(0.0)) ? IVP_REAL(-2.0) : IVP_REAL(2.0);
    error_out->k[0] = q_err.x * sign;
    error_out->k[1] = q_err.y * sign;
    error_out->k[2] = q_err.z * sign;
}

/* Compute angular inverse virtual mass on a world-space axis */
static ivp_real_t angular_inv_vm(const ivp_core_t *ca, const ivp_core_t *cb,
                                 const ivp_vec3r_t *axis_ws)
{
    ivp_real_t inv_vm = IVP_REAL(0.0);
    if (ca && !ca->physical_unmoveable) {
        ivp_vec3r_t ax_cs;
        ivp_mat3r_vimult(&ax_cs, &ca->m_world_f_core_last_psi.rot, axis_ws);
        inv_vm += ax_cs.k[0] * ax_cs.k[0] * ca->inv_rot_inertia.k[0] +
                  ax_cs.k[1] * ax_cs.k[1] * ca->inv_rot_inertia.k[1] +
                  ax_cs.k[2] * ax_cs.k[2] * ca->inv_rot_inertia.k[2];
    }
    if (cb && !cb->physical_unmoveable) {
        ivp_vec3r_t ax_cs;
        ivp_mat3r_vimult(&ax_cs, &cb->m_world_f_core_last_psi.rot, axis_ws);
        inv_vm += ax_cs.k[0] * ax_cs.k[0] * cb->inv_rot_inertia.k[0] +
                  ax_cs.k[1] * ax_cs.k[1] * cb->inv_rot_inertia.k[1] +
                  ax_cs.k[2] * ax_cs.k[2] * cb->inv_rot_inertia.k[2];
    }
    return inv_vm;
}

/* Compute positional inverse virtual mass along a direction */
static ivp_real_t positional_inv_vm(const ivp_core_t *ca,
                                    const ivp_vec3r_t *r_a_ws,
                                    const ivp_core_t *cb,
                                    const ivp_vec3r_t *r_b_ws,
                                    const ivp_vec3r_t *dir)
{
    ivp_real_t inv_vm = IVP_REAL(0.0);

    if (ca && !ca->physical_unmoveable) {
        ivp_vec3r_t r_cs, n_cs, cross;
        ivp_mat3r_vimult(&r_cs, &ca->m_world_f_core_last_psi.rot, r_a_ws);
        ivp_mat3r_vimult(&n_cs, &ca->m_world_f_core_last_psi.rot, dir);
        ivp_vec3r_cross(&cross, &r_cs, &n_cs);
        inv_vm += ca->inv_mass;
        inv_vm += cross.k[0] * cross.k[0] * ca->inv_rot_inertia.k[0] +
                  cross.k[1] * cross.k[1] * ca->inv_rot_inertia.k[1] +
                  cross.k[2] * cross.k[2] * ca->inv_rot_inertia.k[2];
    }

    if (cb && !cb->physical_unmoveable) {
        ivp_vec3r_t r_cs, n_cs, cross;
        ivp_mat3r_vimult(&r_cs, &cb->m_world_f_core_last_psi.rot, r_b_ws);
        ivp_mat3r_vimult(&n_cs, &cb->m_world_f_core_last_psi.rot, dir);
        ivp_vec3r_cross(&cross, &r_cs, &n_cs);
        inv_vm += cb->inv_mass;
        inv_vm += cross.k[0] * cross.k[0] * cb->inv_rot_inertia.k[0] +
                  cross.k[1] * cross.k[1] * cb->inv_rot_inertia.k[1] +
                  cross.k[2] * cross.k[2] * cb->inv_rot_inertia.k[2];
    }

    return inv_vm;
}

/* Apply positional impulse along a direction */
static void apply_pos_impulse(ivp_core_t *ca, const ivp_vec3r_t *r_a_ws,
                              ivp_core_t *cb, const ivp_vec3r_t *r_b_ws,
                              const ivp_vec3r_t *dir, ivp_real_t impulse)
{
    if (ca && !ca->physical_unmoveable) {
        ivp_real_t s = impulse * ca->inv_mass;
        ca->speed.k[0] -= dir->k[0] * s;
        ca->speed.k[1] -= dir->k[1] * s;
        ca->speed.k[2] -= dir->k[2] * s;

        ivp_vec3r_t dir_cs, r_cs, cross;
        ivp_mat3r_vimult(&dir_cs, &ca->m_world_f_core_last_psi.rot, dir);
        ivp_mat3r_vimult(&r_cs, &ca->m_world_f_core_last_psi.rot, r_a_ws);
        ivp_vec3r_cross(&cross, &r_cs, &dir_cs);

        ca->rot_speed.k[0] -= (float)(cross.k[0] * ca->inv_rot_inertia.k[0] * impulse);
        ca->rot_speed.k[1] -= (float)(cross.k[1] * ca->inv_rot_inertia.k[1] * impulse);
        ca->rot_speed.k[2] -= (float)(cross.k[2] * ca->inv_rot_inertia.k[2] * impulse);
    }

    if (cb && !cb->physical_unmoveable) {
        ivp_real_t s = impulse * cb->inv_mass;
        cb->speed.k[0] += dir->k[0] * s;
        cb->speed.k[1] += dir->k[1] * s;
        cb->speed.k[2] += dir->k[2] * s;

        ivp_vec3r_t dir_cs, r_cs, cross;
        ivp_mat3r_vimult(&dir_cs, &cb->m_world_f_core_last_psi.rot, dir);
        ivp_mat3r_vimult(&r_cs, &cb->m_world_f_core_last_psi.rot, r_b_ws);
        ivp_vec3r_cross(&cross, &r_cs, &dir_cs);

        cb->rot_speed.k[0] += (float)(cross.k[0] * cb->inv_rot_inertia.k[0] * impulse);
        cb->rot_speed.k[1] += (float)(cross.k[1] * cb->inv_rot_inertia.k[1] * impulse);
        cb->rot_speed.k[2] += (float)(cross.k[2] * cb->inv_rot_inertia.k[2] * impulse);
    }
}

/* Apply angular impulse along a world-space axis */
static void apply_angular_impulse(ivp_core_t *ca, ivp_core_t *cb,
                                  const ivp_vec3r_t *axis_ws,
                                  ivp_real_t impulse)
{
    if (ca && !ca->physical_unmoveable) {
        ivp_vec3r_t ax_cs;
        ivp_mat3r_vimult(&ax_cs, &ca->m_world_f_core_last_psi.rot, axis_ws);
        ca->rot_speed.k[0] -= (float)(ax_cs.k[0] * ca->inv_rot_inertia.k[0] * impulse);
        ca->rot_speed.k[1] -= (float)(ax_cs.k[1] * ca->inv_rot_inertia.k[1] * impulse);
        ca->rot_speed.k[2] -= (float)(ax_cs.k[2] * ca->inv_rot_inertia.k[2] * impulse);
    }
    if (cb && !cb->physical_unmoveable) {
        ivp_vec3r_t ax_cs;
        ivp_mat3r_vimult(&ax_cs, &cb->m_world_f_core_last_psi.rot, axis_ws);
        cb->rot_speed.k[0] += (float)(ax_cs.k[0] * cb->inv_rot_inertia.k[0] * impulse);
        cb->rot_speed.k[1] += (float)(ax_cs.k[1] * cb->inv_rot_inertia.k[1] * impulse);
        cb->rot_speed.k[2] += (float)(ax_cs.k[2] * cb->inv_rot_inertia.k[2] * impulse);
    }
}

/* Clamp impulse per axis config. Returns false if constraint should break. */
static bool clamp_impulse(const ivp_constraint_axis_config_t *cfg,
                          ivp_real_t *impulse)
{
    if (cfg->exceed_mode == IVP_CFORCE_NONE) return true;

    ivp_real_t mag = ivp_fabs(*impulse);
    if (mag <= cfg->max_impulse) return true;

    switch (cfg->exceed_mode) {
    case IVP_CFORCE_CLIP:
        *impulse = (*impulse > IVP_REAL(0.0))
                 ? cfg->max_impulse
                 : -cfg->max_impulse;
        return true;
    case IVP_CFORCE_BREAK:
        return false;
    case IVP_CFORCE_BEND: {
        /* Squared ratio: softer than CLIP.
         * At 2x overshoot, CLIP gives 50% but BEND gives 25%. */
        ivp_real_t ratio = cfg->max_impulse / mag;
        ivp_real_t scale = ratio * ratio;
        *impulse *= scale;
        return true;
    }
    default:
        return true;
    }
}

/* ── Solve one iteration ───────────────────────────────────────────────── */

static bool solve_one_iteration(ivp_constraint_local_state_t *state,
                                ivp_real_t dt)
{
    if (dt <= IVP_REAL_EPS) return true;

    ivp_core_t *ca = state->obj_a ? &state->obj_a->core : NULL;
    ivp_core_t *cb = &state->obj_b->core;

    /* ── Translation axes ──────────────────────────────────────────────── */
    {
        ivp_vec3r_t pos_a_ws, pos_b_ws;
        get_anchor_ws(state->obj_a, &state->anchor_a_os, &pos_a_ws);
        get_anchor_ws(state->obj_b, &state->anchor_b_os, &pos_b_ws);

        /* Position error in world space */
        ivp_vec3r_t error_ws;
        ivp_vec3r_sub(&error_ws, &pos_b_ws, &pos_a_ws);

        /* Offset from CoM */
        ivp_vec3r_t r_a_ws, r_b_ws;
        if (state->obj_a)
            ivp_vec3r_sub(&r_a_ws, &pos_a_ws, &state->obj_a->core.pos_world_f_core_last_psi);
        else
            ivp_vec3r_set_zero(&r_a_ws);
        ivp_vec3r_sub(&r_b_ws, &pos_b_ws, &state->obj_b->core.pos_world_f_core_last_psi);

        /* Project error onto each reference axis */
        for (int a = 0; a < 3; a++) {
            const ivp_constraint_axis_config_t *cfg = &state->axes[a];
            if (cfg->mode == IVP_CAXIS_FREE) continue;

            ivp_vec3r_t axis_ws;
            get_ref_axis_ws(state, a, &axis_ws);

            ivp_real_t err = ivp_vec3r_dot(&error_ws, &axis_ws);

            /* For LIMITED mode, only constrain when outside borders */
            if (cfg->mode == IVP_CAXIS_LIMITED) {
                if (err >= cfg->border_min && err <= cfg->border_max) {
                    continue;  /* within range, no constraint */
                }
                if (err < cfg->border_min) err = err - cfg->border_min;
                else                       err = err - cfg->border_max;
            }

            if (ivp_fabs(err) < IVP_REAL_EPS) continue;

            ivp_real_t inv_vm = positional_inv_vm(ca, &r_a_ws, cb, &r_b_ws, &axis_ws);
            if (inv_vm < IVP_REAL_EPS) continue;

            /* Baumgarte bias */
            ivp_real_t bias = state->tau / dt * err;

            /* Relative velocity on this axis */
            ivp_real_t rel_vel = IVP_REAL(0.0);
            if (ca) rel_vel -= ivp_vec3r_dot(&ca->speed, &axis_ws);
            rel_vel += ivp_vec3r_dot(&cb->speed, &axis_ws);

            ivp_real_t impulse = (-rel_vel * state->damping - bias) / inv_vm;

            if (!clamp_impulse(cfg, &impulse)) return false; /* BREAK */

            apply_pos_impulse(ca, &r_a_ws, cb, &r_b_ws, &axis_ws, impulse);
            state->accumulated_impulse[a] += ivp_fabs(impulse);
        }
    }

    /* ── Rotation axes ─────────────────────────────────────────────────── */
    {
        ivp_vec3r_t rot_error;
        compute_orientation_error(state, &rot_error);

        for (int a = 0; a < 3; a++) {
            const ivp_constraint_axis_config_t *cfg = &state->axes[3 + a];
            if (cfg->mode == IVP_CAXIS_FREE) continue;

            ivp_vec3r_t axis_ws;
            get_ref_axis_ws(state, a, &axis_ws);

            /* Project rotation error onto this axis */
            ivp_real_t err = ivp_vec3r_dot(&rot_error, &axis_ws);

            /* For LIMITED mode, only constrain outside borders */
            if (cfg->mode == IVP_CAXIS_LIMITED) {
                if (err >= cfg->border_min && err <= cfg->border_max) {
                    continue;
                }
                if (err < cfg->border_min) err = err - cfg->border_min;
                else                       err = err - cfg->border_max;
            }

            if (ivp_fabs(err) < IVP_REAL_EPS) continue;

            ivp_real_t inv_vm = angular_inv_vm(ca, cb, &axis_ws);
            if (inv_vm < IVP_REAL_EPS) continue;

            ivp_real_t impulse = err * state->tau / inv_vm;

            if (!clamp_impulse(cfg, &impulse)) return false; /* BREAK */

            apply_angular_impulse(ca, cb, &axis_ws, impulse);
            state->accumulated_impulse[3 + a] += ivp_fabs(impulse);
        }
    }

    return true;
}

/* ── Enable / Disable / Broken ─────────────────────────────────────────── */

void ivp_constraint_local_set_active(ivp_constraint_local_state_t *state,
                                      bool active)
{
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

void ivp_constraint_local_reset_broken(ivp_constraint_local_state_t *state,
                                        bool reactivate)
{
    state->broken = false;
    if (reactivate) {
        state->active = true;
    }
}

/* ── Runtime rebinding ─────────────────────────────────────────────────── */

void ivp_constraint_local_rebind_anchors(ivp_constraint_local_state_t *state,
                                          const ivp_vec3r_t *anchor_a_os,
                                          const ivp_vec3r_t *anchor_b_os)
{
    state->anchor_a_os = *anchor_a_os;
    state->anchor_b_os = *anchor_b_os;
}

void ivp_constraint_local_rebind_reference(ivp_constraint_local_state_t *state)
{
    if (state->obj_a)
        state->ref_q_a = state->obj_a->core.q_world_f_core_last_psi;
    else
        state->ref_q_a = ivp_quatr_identity();

    if (state->obj_b)
        state->ref_q_b = state->obj_b->core.q_world_f_core_last_psi;
    else
        state->ref_q_b = ivp_quatr_identity();
}

/* ── Controller callback ───────────────────────────────────────────────── */

void ivp_controller_constraint_local(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    ivp_constraint_local_state_t *state = (ivp_constraint_local_state_t *)ctx;
    if (!state || !state->active || state->broken) return;

    /* Only execute on the first dynamic body */
    ivp_object_t *exec_obj = NULL;
    if (state->obj_a && !state->obj_a->core.physical_unmoveable) {
        exec_obj = state->obj_a;
    } else if (state->obj_b && !state->obj_b->core.physical_unmoveable) {
        exec_obj = state->obj_b;
    }
    if (!exec_obj || obj != exec_obj) return;

    /* Clear accumulated impulses */
    for (int i = 0; i < 6; i++)
        state->accumulated_impulse[i] = IVP_REAL(0.0);

    for (int iter = 0; iter < state->solver_iterations; iter++) {
        if (!solve_one_iteration(state, dt)) {
            state->broken = true;
            state->active = false;
            return;
        }
    }
}
