/* ivp_constraint.c -- Constraint system implementation
 *
 * Uses sequential impulse solver with Baumgarte stabilization.
 * Each constraint type applies corrective impulses to enforce
 * the constraint condition.
 */

#include <ivp/controllers/ivp_constraint.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_mat3.h>
#include <string.h>

/* ── Default parameters ─────────────────────────────────────────────────── */

#define IVP_CONSTRAINT_DEFAULT_TAU          IVP_REAL(0.2)
#define IVP_CONSTRAINT_DEFAULT_DAMPING      IVP_REAL(0.9)
#define IVP_CONSTRAINT_DEFAULT_ITERATIONS   4

/* ── Initialization ─────────────────────────────────────────────────────── */

static void init_common(ivp_constraint_state_t *state)
{
    state->tau              = IVP_CONSTRAINT_DEFAULT_TAU;
    state->damping          = IVP_CONSTRAINT_DEFAULT_DAMPING;
    state->solver_iterations = IVP_CONSTRAINT_DEFAULT_ITERATIONS;
    state->active           = true;
    state->accumulated_impulse = IVP_REAL(0.0);
    memset(&state->limits, 0, sizeof(state->limits));
}

void ivp_constraint_init_ball_socket(ivp_constraint_state_t *state,
                                      ivp_object_t *obj_a,
                                      const ivp_vec3r_t *anchor_a_os,
                                      ivp_object_t *obj_b,
                                      const ivp_vec3r_t *anchor_b_os)
{
    memset(state, 0, sizeof(*state));
    state->type        = IVP_CONSTRAINT_BALL_SOCKET;
    state->obj_a       = obj_a;
    state->obj_b       = obj_b;
    state->anchor_a_os = *anchor_a_os;
    state->anchor_b_os = *anchor_b_os;
    init_common(state);
}

void ivp_constraint_init_hinge(ivp_constraint_state_t *state,
                                 ivp_object_t *obj_a,
                                 const ivp_vec3r_t *anchor_a_os,
                                 const ivp_vec3r_t *axis_a_os,
                                 ivp_object_t *obj_b,
                                 const ivp_vec3r_t *anchor_b_os,
                                 const ivp_vec3r_t *axis_b_os)
{
    memset(state, 0, sizeof(*state));
    state->type        = IVP_CONSTRAINT_HINGE;
    state->obj_a       = obj_a;
    state->obj_b       = obj_b;
    state->anchor_a_os = *anchor_a_os;
    state->anchor_b_os = *anchor_b_os;
    state->axis_a_os   = *axis_a_os;
    state->axis_b_os   = *axis_b_os;

    /* Store reference orientations for hinge-angle limits */
    if (obj_a) state->ref_q_a = obj_a->core.q_world_f_core_last_psi;
    else       state->ref_q_a = ivp_quatr_identity();
    if (obj_b) state->ref_q_b = obj_b->core.q_world_f_core_last_psi;
    else       state->ref_q_b = ivp_quatr_identity();

    init_common(state);
}

void ivp_constraint_init_fixed(ivp_constraint_state_t *state,
                                 ivp_object_t *obj_a,
                                 const ivp_vec3r_t *anchor_a_os,
                                 ivp_object_t *obj_b,
                                 const ivp_vec3r_t *anchor_b_os)
{
    memset(state, 0, sizeof(*state));
    state->type        = IVP_CONSTRAINT_FIXED;
    state->obj_a       = obj_a;
    state->obj_b       = obj_b;
    state->anchor_a_os = *anchor_a_os;
    state->anchor_b_os = *anchor_b_os;

    /* Record current relative orientation as reference */
    if (obj_a) state->ref_q_a = obj_a->core.q_world_f_core_last_psi;
    else       state->ref_q_a = ivp_quatr_identity();
    if (obj_b) state->ref_q_b = obj_b->core.q_world_f_core_last_psi;
    else       state->ref_q_b = ivp_quatr_identity();

    init_common(state);
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
}

/* ── Helper: get anchor world position ──────────────────────────────────── */

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

/* ── Helper: compute virtual mass for positional constraint ─────────────── */

static ivp_real_t compute_positional_virtual_mass(
    const ivp_core_t *ca, const ivp_vec3r_t *r_a_ws,
    const ivp_core_t *cb, const ivp_vec3r_t *r_b_ws,
    const ivp_vec3r_t *direction)
{
    ivp_real_t inv_vm = IVP_REAL(0.0);

    if (ca && !ca->physical_unmoveable) {
        ivp_vec3r_t r_cs, n_cs, cross;
        ivp_mat3r_vimult(&r_cs, &ca->m_world_f_core_last_psi.rot, r_a_ws);
        ivp_mat3r_vimult(&n_cs, &ca->m_world_f_core_last_psi.rot, direction);
        ivp_vec3r_cross(&cross, &r_cs, &n_cs);

        inv_vm += ca->inv_mass;
        inv_vm += cross.k[0] * cross.k[0] * ca->inv_rot_inertia.k[0] +
                  cross.k[1] * cross.k[1] * ca->inv_rot_inertia.k[1] +
                  cross.k[2] * cross.k[2] * ca->inv_rot_inertia.k[2];
    }

    if (cb && !cb->physical_unmoveable) {
        ivp_vec3r_t r_cs, n_cs, cross;
        ivp_mat3r_vimult(&r_cs, &cb->m_world_f_core_last_psi.rot, r_b_ws);
        ivp_mat3r_vimult(&n_cs, &cb->m_world_f_core_last_psi.rot, direction);
        ivp_vec3r_cross(&cross, &r_cs, &n_cs);

        inv_vm += cb->inv_mass;
        inv_vm += cross.k[0] * cross.k[0] * cb->inv_rot_inertia.k[0] +
                  cross.k[1] * cross.k[1] * cb->inv_rot_inertia.k[1] +
                  cross.k[2] * cross.k[2] * cb->inv_rot_inertia.k[2];
    }

    return (inv_vm > IVP_REAL_EPS) ? (IVP_REAL(1.0) / inv_vm) : IVP_REAL(0.0);
}

/* ── Helper: apply positional impulse to both objects ───────────────────── */

static void apply_positional_impulse(ivp_core_t *ca, const ivp_vec3r_t *r_a_ws,
                                      ivp_core_t *cb, const ivp_vec3r_t *r_b_ws,
                                      const ivp_vec3r_t *direction,
                                      ivp_real_t impulse)
{
    /* Object A: push in -direction */
    if (ca && !ca->physical_unmoveable) {
        ivp_real_t s = impulse * ca->inv_mass;
        ca->speed.k[0] -= direction->k[0] * s;
        ca->speed.k[1] -= direction->k[1] * s;
        ca->speed.k[2] -= direction->k[2] * s;

        ivp_vec3r_t dir_cs, r_cs, cross;
        ivp_mat3r_vimult(&dir_cs, &ca->m_world_f_core_last_psi.rot, direction);
        ivp_mat3r_vimult(&r_cs, &ca->m_world_f_core_last_psi.rot, r_a_ws);
        ivp_vec3r_cross(&cross, &r_cs, &dir_cs);

        ca->rot_speed.k[0] -= (float)(cross.k[0] * ca->inv_rot_inertia.k[0] * impulse);
        ca->rot_speed.k[1] -= (float)(cross.k[1] * ca->inv_rot_inertia.k[1] * impulse);
        ca->rot_speed.k[2] -= (float)(cross.k[2] * ca->inv_rot_inertia.k[2] * impulse);
    }

    /* Object B: push in +direction */
    if (cb && !cb->physical_unmoveable) {
        ivp_real_t s = impulse * cb->inv_mass;
        cb->speed.k[0] += direction->k[0] * s;
        cb->speed.k[1] += direction->k[1] * s;
        cb->speed.k[2] += direction->k[2] * s;

        ivp_vec3r_t dir_cs, r_cs, cross;
        ivp_mat3r_vimult(&dir_cs, &cb->m_world_f_core_last_psi.rot, direction);
        ivp_mat3r_vimult(&r_cs, &cb->m_world_f_core_last_psi.rot, r_b_ws);
        ivp_vec3r_cross(&cross, &r_cs, &dir_cs);

        cb->rot_speed.k[0] += (float)(cross.k[0] * cb->inv_rot_inertia.k[0] * impulse);
        cb->rot_speed.k[1] += (float)(cross.k[1] * cb->inv_rot_inertia.k[1] * impulse);
        cb->rot_speed.k[2] += (float)(cross.k[2] * cb->inv_rot_inertia.k[2] * impulse);
    }
}

/* ── Solve positional constraint ────────────────────────────────────────── */

static void solve_position(ivp_constraint_state_t *state, ivp_real_t dt)
{
    if (dt <= IVP_REAL_EPS) return;

    ivp_vec3r_t pos_a_ws, pos_b_ws;
    get_anchor_ws(state->obj_a, &state->anchor_a_os, &pos_a_ws);
    get_anchor_ws(state->obj_b, &state->anchor_b_os, &pos_b_ws);

    /* Position error: B_anchor - A_anchor (should be zero) */
    ivp_vec3r_t error;
    ivp_vec3r_sub(&error, &pos_b_ws, &pos_a_ws);

    ivp_real_t error_len = ivp_vec3r_length(&error);
    if (error_len < IVP_REAL_EPS) return;

    /* Direction of constraint force */
    ivp_vec3r_t dir;
    ivp_real_t inv_error = IVP_REAL(1.0) / error_len;
    ivp_vec3r_scale(&dir, &error, inv_error);

    /* Offset from CoM */
    ivp_vec3r_t r_a_ws, r_b_ws;
    if (state->obj_a) {
        ivp_vec3r_sub(&r_a_ws, &pos_a_ws, &state->obj_a->core.pos_world_f_core_last_psi);
    } else {
        ivp_vec3r_set_zero(&r_a_ws);
    }
    ivp_vec3r_sub(&r_b_ws, &pos_b_ws, &state->obj_b->core.pos_world_f_core_last_psi);

    ivp_core_t *ca = state->obj_a ? &state->obj_a->core : NULL;
    ivp_core_t *cb = &state->obj_b->core;

    ivp_real_t vm = compute_positional_virtual_mass(ca, &r_a_ws, cb, &r_b_ws, &dir);
    if (vm < IVP_REAL_EPS) return;

    /* Baumgarte position correction: bias = tau/dt * error */
    ivp_real_t bias = state->tau / dt * error_len;

    /* Relative velocity along constraint direction */
    ivp_real_t rel_vel = IVP_REAL(0.0);
    if (ca) {
        rel_vel -= ivp_vec3r_dot(&ca->speed, &dir);
    }
    rel_vel += ivp_vec3r_dot(&cb->speed, &dir);

    /* Impulse = vm * (-rel_vel * damping - bias)
     * so positive position error produces a restoring impulse. */
    ivp_real_t impulse = vm * (-rel_vel * state->damping - bias);

    apply_positional_impulse(ca, &r_a_ws, cb, &r_b_ws, &dir, impulse);
    state->accumulated_impulse += ivp_fabs(impulse);

    /* Positional projection (XPBD-style stabilization):
     * Directly project anchor error to avoid long-term drift after strong impacts.
     * This complements velocity-level Baumgarte impulses and keeps joints tight. */
    {
        ivp_real_t inv_ma = (ca && !ca->physical_unmoveable) ? ca->inv_mass : IVP_REAL(0.0);
        ivp_real_t inv_mb = (cb && !cb->physical_unmoveable) ? cb->inv_mass : IVP_REAL(0.0);
        ivp_real_t inv_m_sum = inv_ma + inv_mb;

        if (inv_m_sum > IVP_REAL_EPS) {
            const ivp_real_t proj_factor = IVP_REAL(0.35);
            ivp_real_t corr_mag = error_len * proj_factor;

            if (ca && !ca->physical_unmoveable) {
                ivp_real_t wa = inv_ma / inv_m_sum;
                ivp_vec3r_t pos = ca->pos_world_f_core_last_psi;
                pos.k[0] += dir.k[0] * corr_mag * wa;
                pos.k[1] += dir.k[1] * corr_mag * wa;
                pos.k[2] += dir.k[2] * corr_mag * wa;
                ca->pos_world_f_core_last_psi = pos;
                ca->m_world_f_core_last_psi.pos = pos;
            }

            if (cb && !cb->physical_unmoveable) {
                ivp_real_t wb = inv_mb / inv_m_sum;
                ivp_vec3r_t pos = cb->pos_world_f_core_last_psi;
                pos.k[0] -= dir.k[0] * corr_mag * wb;
                pos.k[1] -= dir.k[1] * corr_mag * wb;
                pos.k[2] -= dir.k[2] * corr_mag * wb;
                cb->pos_world_f_core_last_psi = pos;
                cb->m_world_f_core_last_psi.pos = pos;
            }
        }
    }
}

/* ── Helper: apply angular impulse along world-space axis ───────────────── */

static void apply_angular_impulse_on_axis(ivp_core_t *ca, ivp_core_t *cb,
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

/* ── Helper: angular inverse virtual mass on an axis ────────────────────── */

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

/* ── Solve angular constraint (hinge axis alignment + limits) ───────────── */

static void solve_hinge_axis(ivp_constraint_state_t *state, ivp_real_t dt)
{
    /* Get hinge axes in world space */
    ivp_vec3r_t axis_a_ws, axis_b_ws;
    if (state->obj_a) {
        ivp_transform_dir(&axis_a_ws,
                           &state->obj_a->core.m_world_f_core_last_psi,
                           &state->axis_a_os);
    } else {
        axis_a_ws = state->axis_a_os;
    }
    ivp_transform_dir(&axis_b_ws,
                       &state->obj_b->core.m_world_f_core_last_psi,
                       &state->axis_b_os);

    ivp_core_t *ca = state->obj_a ? &state->obj_a->core : NULL;
    ivp_core_t *cb = &state->obj_b->core;

    /* ── Part 1: Align axes (remove 2 angular DOF) ─────────────────────── */
    {
        ivp_vec3r_t cross;
        ivp_vec3r_cross(&cross, &axis_b_ws, &axis_a_ws);

        ivp_real_t error_len = ivp_vec3r_length(&cross);
        if (error_len > IVP_REAL_EPS) {
            ivp_vec3r_t correction_axis;
            ivp_vec3r_scale(&correction_axis, &cross, IVP_REAL(1.0) / error_len);

            ivp_real_t inv_vm = angular_inv_vm(ca, cb, &correction_axis);
            if (inv_vm > IVP_REAL_EPS) {
                ivp_real_t angular_impulse = error_len * state->tau / inv_vm;
                apply_angular_impulse_on_axis(ca, cb, &correction_axis, angular_impulse);
            }
        }
    }

    /* ── Part 2: Angular limits on the hinge rotation axis ─────────────── */
    if (state->limits.limited) {
        /* Compute current hinge angle:
         * Use relative orientation projected onto the hinge axis.
         * angle = decompose relative rotation around axis_a_ws. */
        ivp_quatr_t q_a, q_b;
        if (state->obj_a) {
            q_a = state->obj_a->core.q_world_f_core_last_psi;
        } else {
            q_a = ivp_quatr_identity();
        }
        q_b = state->obj_b->core.q_world_f_core_last_psi;

        /* q_rel = conj(q_a) * q_b */
        ivp_quatr_t q_a_inv = {-q_a.x, -q_a.y, -q_a.z, q_a.w};
        ivp_quatr_t q_rel;
        ivp_quat_mult(&q_rel, &q_a_inv, &q_b);

        /* Reference relative: conj(ref_q_a) * ref_q_b */
        ivp_quatr_t ref_a_inv = {-state->ref_q_a.x, -state->ref_q_a.y,
                                  -state->ref_q_a.z,  state->ref_q_a.w};
        ivp_quatr_t q_ref;
        ivp_quat_mult(&q_ref, &ref_a_inv, &state->ref_q_b);

        /* q_err = conj(q_ref) * q_rel  — relative rotation from reference */
        ivp_quatr_t q_ref_inv = {-q_ref.x, -q_ref.y, -q_ref.z, q_ref.w};
        ivp_quatr_t q_err;
        ivp_quat_mult(&q_err, &q_ref_inv, &q_rel);

        /* Project onto hinge axis (in A's body space → use axis_a_os)
         * angle = 2 * atan2(dot(quat_vec, axis), quat_w) */
        ivp_vec3r_t q_vec = {{q_err.x, q_err.y, q_err.z}};
        ivp_real_t proj = ivp_vec3r_dot(&q_vec, &state->axis_a_os);
        ivp_real_t angle = IVP_REAL(2.0) * ivp_atan2(proj, q_err.w);

        /* Wrap angle to [-pi, pi] */
        while (angle >  IVP_PI) angle -= IVP_REAL(2.0) * IVP_PI;
        while (angle < -IVP_PI) angle += IVP_REAL(2.0) * IVP_PI;

        /* Check limits */
        ivp_real_t violation = IVP_REAL(0.0);
        if (angle < state->limits.min_angle) {
            violation = angle - state->limits.min_angle;
        } else if (angle > state->limits.max_angle) {
            violation = angle - state->limits.max_angle;
        }

        if (ivp_fabs(violation) > IVP_REAL_EPS) {
            ivp_real_t inv_vm = angular_inv_vm(ca, cb, &axis_a_ws);
            if (inv_vm > IVP_REAL_EPS) {
                /* Spring-damper limit: impulse = (stiffness * violation + damping * omega_on_axis) / inv_vm */
                ivp_vec3r_t omega_b_ws;
                ivp_vec3r_t rot_speed_r;
                rot_speed_r.k[0] = (ivp_real_t)cb->rot_speed.k[0];
                rot_speed_r.k[1] = (ivp_real_t)cb->rot_speed.k[1];
                rot_speed_r.k[2] = (ivp_real_t)cb->rot_speed.k[2];
                ivp_transform_dir(&omega_b_ws, &cb->m_world_f_core_last_psi, &rot_speed_r);
                ivp_real_t omega_on_axis = ivp_vec3r_dot(&omega_b_ws, &axis_a_ws);

                ivp_real_t limit_impulse =
                    (state->limits.limit_stiffness * violation +
                     state->limits.limit_damping * omega_on_axis) * dt / inv_vm;

                apply_angular_impulse_on_axis(ca, cb, &axis_a_ws, limit_impulse);
            }
        }
    }
}

/* ── Solve fixed orientation constraint ─────────────────────────────────── */

static void solve_fixed_orientation(ivp_constraint_state_t *state, ivp_real_t dt)
{

    /* Compute relative orientation error.
     * Current relative: q_rel = inv(q_a) * q_b
     * Reference relative: q_ref = inv(ref_q_a) * ref_q_b
     * Error: q_err = inv(q_ref) * q_rel */
    ivp_quatr_t q_a, q_b;
    if (state->obj_a) {
        q_a = state->obj_a->core.q_world_f_core_last_psi;
    } else {
        q_a = ivp_quatr_identity();
    }
    q_b = state->obj_b->core.q_world_f_core_last_psi;

    /* q_rel_current = conjugate(q_a) * q_b */
    ivp_quatr_t q_a_inv = {-q_a.x, -q_a.y, -q_a.z, q_a.w};
    ivp_quatr_t q_rel;
    ivp_quat_mult(&q_rel, &q_a_inv, &q_b);

    /* q_rel_ref = conjugate(ref_q_a) * ref_q_b */
    ivp_quatr_t ref_a_inv = {-state->ref_q_a.x, -state->ref_q_a.y,
                              -state->ref_q_a.z,  state->ref_q_a.w};
    ivp_quatr_t q_ref;
    ivp_quat_mult(&q_ref, &ref_a_inv, &state->ref_q_b);

    /* q_err = conjugate(q_ref) * q_rel */
    ivp_quatr_t q_ref_inv = {-q_ref.x, -q_ref.y, -q_ref.z, q_ref.w};
    ivp_quatr_t q_err;
    ivp_quat_mult(&q_err, &q_ref_inv, &q_rel);

    /* Error axis = normalized vector part of q_err, angle = 2*asin(|v|)
     * For small angles: error ≈ 2 * (x, y, z) */
    ivp_vec3r_t error_axis;
    error_axis.k[0] = q_err.x * IVP_REAL(2.0);
    error_axis.k[1] = q_err.y * IVP_REAL(2.0);
    error_axis.k[2] = q_err.z * IVP_REAL(2.0);

    /* Ensure shortest path */
    if (q_err.w < IVP_REAL(0.0)) {
        error_axis.k[0] = -error_axis.k[0];
        error_axis.k[1] = -error_axis.k[1];
        error_axis.k[2] = -error_axis.k[2];
    }

    ivp_real_t err_mag = ivp_vec3r_length(&error_axis);
    if (err_mag < IVP_REAL_EPS) return;

    /* Apply angular correction on each axis */
    ivp_core_t *ca = state->obj_a ? &state->obj_a->core : NULL;
    ivp_core_t *cb = &state->obj_b->core;

    for (int axis = 0; axis < 3; axis++) {
        if (ivp_fabs(error_axis.k[axis]) < IVP_REAL_EPS) continue;

        ivp_vec3r_t dir = {.k = {0, 0, 0}};
        dir.k[axis] = IVP_REAL(1.0);

        /* Transform to world space via A's orientation */
        ivp_vec3r_t dir_ws;
        if (state->obj_a) {
            ivp_mat3r_vmult(&dir_ws, &state->obj_a->core.m_world_f_core_last_psi.rot, &dir);
        } else {
            dir_ws = dir;
        }

        ivp_real_t inv_vm = IVP_REAL(0.0);
        if (ca && !ca->physical_unmoveable) {
            ivp_vec3r_t ax_cs;
            ivp_mat3r_vimult(&ax_cs, &ca->m_world_f_core_last_psi.rot, &dir_ws);
            inv_vm += ax_cs.k[0] * ax_cs.k[0] * ca->inv_rot_inertia.k[0] +
                      ax_cs.k[1] * ax_cs.k[1] * ca->inv_rot_inertia.k[1] +
                      ax_cs.k[2] * ax_cs.k[2] * ca->inv_rot_inertia.k[2];
        }
        if (!cb->physical_unmoveable) {
            ivp_vec3r_t ax_cs;
            ivp_mat3r_vimult(&ax_cs, &cb->m_world_f_core_last_psi.rot, &dir_ws);
            inv_vm += ax_cs.k[0] * ax_cs.k[0] * cb->inv_rot_inertia.k[0] +
                      ax_cs.k[1] * ax_cs.k[1] * cb->inv_rot_inertia.k[1] +
                      ax_cs.k[2] * ax_cs.k[2] * cb->inv_rot_inertia.k[2];
        }

        if (inv_vm < IVP_REAL_EPS) continue;

        ivp_real_t angular_impulse = error_axis.k[axis] * state->tau / inv_vm;

        if (ca && !ca->physical_unmoveable) {
            ivp_vec3r_t ax_cs;
            ivp_mat3r_vimult(&ax_cs, &ca->m_world_f_core_last_psi.rot, &dir_ws);
            ca->rot_speed.k[0] -= (float)(ax_cs.k[0] * ca->inv_rot_inertia.k[0] * angular_impulse);
            ca->rot_speed.k[1] -= (float)(ax_cs.k[1] * ca->inv_rot_inertia.k[1] * angular_impulse);
            ca->rot_speed.k[2] -= (float)(ax_cs.k[2] * ca->inv_rot_inertia.k[2] * angular_impulse);
        }
        if (!cb->physical_unmoveable) {
            ivp_vec3r_t ax_cs;
            ivp_mat3r_vimult(&ax_cs, &cb->m_world_f_core_last_psi.rot, &dir_ws);
            cb->rot_speed.k[0] += (float)(ax_cs.k[0] * cb->inv_rot_inertia.k[0] * angular_impulse);
            cb->rot_speed.k[1] += (float)(ax_cs.k[1] * cb->inv_rot_inertia.k[1] * angular_impulse);
            cb->rot_speed.k[2] += (float)(ax_cs.k[2] * cb->inv_rot_inertia.k[2] * angular_impulse);
        }
    }
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_constraint(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    ivp_constraint_state_t *state = (ivp_constraint_state_t *)ctx;
    if (!state || !state->active) return;

    /* Only execute once per step on the first dynamic body.
     * This also covers constraints where obj_a is static. */
    ivp_object_t *exec_obj = NULL;
    if (state->obj_a && !state->obj_a->core.physical_unmoveable) {
        exec_obj = state->obj_a;
    } else if (state->obj_b && !state->obj_b->core.physical_unmoveable) {
        exec_obj = state->obj_b;
    }
    if (!exec_obj || obj != exec_obj) return;

    state->accumulated_impulse = IVP_REAL(0.0);

    for (int iter = 0; iter < state->solver_iterations; iter++) {
        /* Positional constraint (all types) */
        solve_position(state, dt);

        /* Angular constraints (type-dependent) */
        switch (state->type) {
        case IVP_CONSTRAINT_HINGE:
            solve_hinge_axis(state, dt);
            break;
        case IVP_CONSTRAINT_FIXED:
            solve_fixed_orientation(state, dt);
            break;
        case IVP_CONSTRAINT_BALL_SOCKET:
        default:
            break; /* no angular constraint for ball socket */
        }
    }
}
