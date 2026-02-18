/* ivp_constraint_fixed_keyed.c — Keyframed fixed constraint
 *
 * Drives an attached object toward a target position and orientation
 * using spring-damper impulses with configurable force limits.
 */
#include <ivp/controllers/ivp_constraint_fixed_keyed.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_mat3.h>
#include <string.h>

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_fixed_keyed_init(ivp_fixed_keyed_state_t *state,
                          ivp_object_t *reference_obj,
                          ivp_object_t *attached_obj,
                          const ivp_vec3r_t *offset_ros)
{
    memset(state, 0, sizeof(*state));

    state->reference_obj = reference_obj;
    state->attached_obj  = attached_obj;
    state->target_pos_ros = *offset_ros;

    /* Default: match current relative orientation */
    ivp_quatr_t q_ref = reference_obj
        ? reference_obj->core.q_world_f_core_last_psi
        : ivp_quatr_identity();
    ivp_quatr_t q_att = attached_obj->core.q_world_f_core_last_psi;

    /* Store target as current relative: inv(q_ref) * q_att */
    ivp_quatr_t q_ref_inv = {-q_ref.x, -q_ref.y, -q_ref.z, q_ref.w};
    ivp_quat_mult(&state->target_orient_0, &q_ref_inv, &q_att);
    state->target_orient_1 = state->target_orient_0;
    state->orient_blend = IVP_REAL(0.0);
    state->orient_keyed = false;

    state->force_factor       = IVP_REAL(1.0);
    state->damp_factor        = IVP_REAL(0.9);
    state->torque_factor      = IVP_REAL(1.0);
    state->angular_damp_factor = IVP_REAL(0.9);

    /* Large default limits (effectively unlimited) */
    ivp_vec3r_set(&state->max_translation_force,
                  IVP_REAL(1e8), IVP_REAL(1e8), IVP_REAL(1e8));
    ivp_vec3r_set(&state->max_torque,
                  IVP_REAL(1e8), IVP_REAL(1e8), IVP_REAL(1e8));

    state->active = true;
}

/* ── Orientation control ───────────────────────────────────────────────── */

void ivp_fixed_keyed_set_orientation(ivp_fixed_keyed_state_t *state,
                                     const ivp_quatr_t *q0,
                                     const ivp_quatr_t *q1,
                                     ivp_real_t blend)
{
    state->target_orient_0 = *q0;
    state->target_orient_1 = *q1;
    state->orient_blend = blend;
    state->orient_keyed = true;
}

void ivp_fixed_keyed_set_orientation_single(ivp_fixed_keyed_state_t *state,
                                            const ivp_quatr_t *target_q)
{
    state->target_orient_0 = *target_q;
    state->target_orient_1 = *target_q;
    state->orient_blend = IVP_REAL(0.0);
    state->orient_keyed = true;
}

/* ── Helpers ───────────────────────────────────────────────────────────── */

static ivp_real_t clamp_val(ivp_real_t v, ivp_real_t lo, ivp_real_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Simple quaternion slerp for blend ∈ [0,1] */
static ivp_quatr_t quat_slerp(const ivp_quatr_t *a, const ivp_quatr_t *b,
                               ivp_real_t t)
{
    if (t <= IVP_REAL(0.0)) return *a;
    if (t >= IVP_REAL(1.0)) return *b;

    ivp_real_t dot = a->x * b->x + a->y * b->y + a->z * b->z + a->w * b->w;

    /* Ensure shortest path */
    ivp_quatr_t b2 = *b;
    if (dot < IVP_REAL(0.0)) {
        b2.x = -b2.x; b2.y = -b2.y; b2.z = -b2.z; b2.w = -b2.w;
        dot = -dot;
    }

    if (dot > IVP_REAL(0.9999)) {
        /* Nearly identical: use linear interpolation */
        ivp_quatr_t r;
        r.x = a->x + t * (b2.x - a->x);
        r.y = a->y + t * (b2.y - a->y);
        r.z = a->z + t * (b2.z - a->z);
        r.w = a->w + t * (b2.w - a->w);
        /* Normalize */
        ivp_real_t len = ivp_sqrt(r.x*r.x + r.y*r.y + r.z*r.z + r.w*r.w);
        if (len > IVP_REAL_EPS) {
            ivp_real_t inv = IVP_REAL(1.0) / len;
            r.x *= inv; r.y *= inv; r.z *= inv; r.w *= inv;
        }
        return r;
    }

    ivp_real_t theta = ivp_acos(dot);
    ivp_real_t sin_theta = ivp_sin(theta);
    ivp_real_t wa = ivp_sin((IVP_REAL(1.0) - t) * theta) / sin_theta;
    ivp_real_t wb = ivp_sin(t * theta) / sin_theta;

    ivp_quatr_t r;
    r.x = wa * a->x + wb * b2.x;
    r.y = wa * a->y + wb * b2.y;
    r.z = wa * a->z + wb * b2.z;
    r.w = wa * a->w + wb * b2.w;
    return r;
}

/* ── Controller callback ───────────────────────────────────────────────── */

void ivp_controller_fixed_keyed(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    ivp_fixed_keyed_state_t *state = (ivp_fixed_keyed_state_t *)ctx;
    if (!state || !state->active) return;
    if (!state->attached_obj || state->attached_obj->core.physical_unmoveable) return;
    if (dt <= IVP_REAL_EPS) return;

    /* Only execute on the attached object */
    if (obj != state->attached_obj) return;

    ivp_core_t *att = &state->attached_obj->core;
    ivp_core_t *ref = state->reference_obj ? &state->reference_obj->core : NULL;

    ivp_real_t inv_dt = IVP_REAL(1.0) / dt;

    /* ── Translation ───────────────────────────────────────────────────── */
    {
        /* Compute target world position:
         * target_ws = ref_pos + ref_rot * (target_pos_ros + target_vel_ros * dt) */
        ivp_vec3r_t target_ros = state->target_pos_ros;
        target_ros.k[0] += state->target_vel_ros.k[0] * dt;
        target_ros.k[1] += state->target_vel_ros.k[1] * dt;
        target_ros.k[2] += state->target_vel_ros.k[2] * dt;

        ivp_vec3r_t target_ws;
        if (ref) {
            ivp_transform_point(&target_ws, &ref->m_world_f_core_last_psi, &target_ros);
        } else {
            target_ws = target_ros;
        }

        /* Position error */
        ivp_vec3r_t pos_error;
        ivp_vec3r_sub(&pos_error, &target_ws, &att->pos_world_f_core_last_psi);

        /* Desired velocity = (error / dt) * force_factor */
        /* Current velocity of attached object */
        /* Impulse = mass * (desired_vel - current_vel * damp_factor) */
        ivp_real_t mass = (att->inv_mass > IVP_REAL_EPS)
                        ? (IVP_REAL(1.0) / att->inv_mass)
                        : IVP_REAL(0.0);
        if (mass < IVP_REAL_EPS) return;

        for (int a = 0; a < 3; a++) {
            ivp_real_t desired_vel = pos_error.k[a] * inv_dt * state->force_factor;
            ivp_real_t current_vel = att->speed.k[a];
            ivp_real_t impulse = mass * (desired_vel - current_vel * state->damp_factor);

            /* Clamp by force limit: impulse = force * dt */
            ivp_real_t max_imp = state->max_translation_force.k[a] * dt;
            impulse = clamp_val(impulse, -max_imp, max_imp);

            att->speed.k[a] += impulse * att->inv_mass;
        }
    }

    /* ── Rotation ──────────────────────────────────────────────────────── */
    {
        /* Compute target world orientation:
         * target_q_ws = q_ref * target_orient (interpolated if keyed) */
        ivp_quatr_t target_local;
        if (state->orient_keyed) {
            target_local = quat_slerp(&state->target_orient_0,
                                       &state->target_orient_1,
                                       state->orient_blend);
        } else {
            target_local = state->target_orient_0;
        }

        ivp_quatr_t target_ws;
        if (ref) {
            ivp_quat_mult(&target_ws, &ref->q_world_f_core_last_psi, &target_local);
        } else {
            target_ws = target_local;
        }

        /* Orientation error: q_err = inv(q_att) * target_ws
         * This gives the rotation from current to target. */
        ivp_quatr_t q_att_inv = {-att->q_world_f_core_last_psi.x,
                                  -att->q_world_f_core_last_psi.y,
                                  -att->q_world_f_core_last_psi.z,
                                   att->q_world_f_core_last_psi.w};
        ivp_quatr_t q_err;
        ivp_quat_mult(&q_err, &q_att_inv, &target_ws);

        /* Ensure shortest path */
        if (q_err.w < IVP_REAL(0.0)) {
            q_err.x = -q_err.x; q_err.y = -q_err.y;
            q_err.z = -q_err.z; q_err.w = -q_err.w;
        }

        /* Axis-angle approximation: error_axis ≈ 2 * vec(q_err) */
        ivp_vec3r_t error_ws;
        error_ws.k[0] = q_err.x * IVP_REAL(2.0);
        error_ws.k[1] = q_err.y * IVP_REAL(2.0);
        error_ws.k[2] = q_err.z * IVP_REAL(2.0);

        /* Transform error to core space for applying angular impulse */
        ivp_vec3r_t error_cs;
        ivp_mat3r_vimult(&error_cs, &att->m_world_f_core_last_psi.rot, &error_ws);

        for (int a = 0; a < 3; a++) {
            if (att->inv_rot_inertia.k[a] < IVP_REAL_EPS) continue;

            ivp_real_t inertia = IVP_REAL(1.0) / att->inv_rot_inertia.k[a];
            ivp_real_t desired_omega = error_cs.k[a] * inv_dt * state->torque_factor;
            ivp_real_t current_omega = (ivp_real_t)att->rot_speed.k[a];
            ivp_real_t impulse = inertia * (desired_omega - current_omega * state->angular_damp_factor);

            /* Clamp by torque limit */
            ivp_real_t max_imp = state->max_torque.k[a] * dt;
            impulse = clamp_val(impulse, -max_imp, max_imp);

            att->rot_speed.k[a] += (float)(impulse * att->inv_rot_inertia.k[a]);
        }
    }
}
