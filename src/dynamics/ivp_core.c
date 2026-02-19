/* ivp_core.c — Rigid body core implementation */
#include <ivp/dynamics/ivp_core.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>
#include <math.h>
#include <string.h>

/* ── Core initialization ───────────────────────────────────────────────── */

void ivp_core_init(ivp_core_t *core, ivp_real_t mass,
                   const ivp_vec3r_t *rot_inertia) {
    IVP_ASSERT(mass >= IVP_REAL(0.0));

    memset(core, 0, sizeof(*core));

    core->mass     = mass;
    core->inv_mass = (mass > IVP_REAL(0.0)) ? IVP_REAL(1.0) / mass : IVP_REAL(0.0);

    core->rot_inertia     = *rot_inertia;
    core->inv_rot_inertia = ivp_vec3r(
        (rot_inertia->k[0] > IVP_REAL(0.0)) ? IVP_REAL(1.0) / rot_inertia->k[0] : IVP_REAL(0.0),
        (rot_inertia->k[1] > IVP_REAL(0.0)) ? IVP_REAL(1.0) / rot_inertia->k[1] : IVP_REAL(0.0),
        (rot_inertia->k[2] > IVP_REAL(0.0)) ? IVP_REAL(1.0) / rot_inertia->k[2] : IVP_REAL(0.0)
    );

    /* Check if all three inertias are equal */
    ivp_real_t eps = IVP_REAL(1e-6);
    core->rot_inertias_are_equal =
        (ivp_fabs(rot_inertia->k[0] - rot_inertia->k[1]) < eps) &&
        (ivp_fabs(rot_inertia->k[1] - rot_inertia->k[2]) < eps);

    /* Default damping */
    core->speed_damp_factor      = IVP_REAL(0.01);
    core->rot_speed_damp_factor  = ivp_vec3r(IVP_REAL(0.01), IVP_REAL(0.01), IVP_REAL(0.01));

    /* Default bounding radius */
    core->upper_limit_radius     = IVP_REAL(1.0);
    core->max_surface_deviation  = IVP_REAL(0.0);

    /* Identity pose */
    core->q_world_f_core_last_psi = ivp_quatr_identity();
    core->q_world_f_core_next_psi = ivp_quatr_identity();
    ivp_quat_to_mat3(&core->q_world_f_core_last_psi, &core->m_world_f_core_last_psi.rot);
    ivp_vec3r_set_zero(&core->m_world_f_core_last_psi.pos);

    /* Zero velocities */
    core->rot_speed = ivp_vec3f(0.0f, 0.0f, 0.0f);
    ivp_vec3r_set_zero(&core->speed);
    ivp_vec3r_set_zero(&core->rot_speed_change);
    ivp_vec3r_set_zero(&core->speed_change);
    ivp_vec3r_set_zero(&core->pos_world_f_core_last_psi);
    ivp_vec3r_set_zero(&core->delta_world_f_core_psis);

    /* State */
    core->movement_state = IVP_MT_MOVING;
    core->time_of_last_psi = 0.0;
    core->i_delta_time = (ivp_real_t)IVP_DEFAULT_PSI_RATE;
    core->current_speed = IVP_REAL(0.0);
    core->max_surface_rot_speed = IVP_REAL(0.0);
    core->movement_check_counter = 0;
    core->collisions_this_psi = 0;
    core->collision_count_epoch = 0;
    core->fast_piling_allowed = false;

    /* Initialize freeze checkpoints */
    core->checkpoint_short.pos  = core->pos_world_f_core_last_psi;
    core->checkpoint_short.quat = core->q_world_f_core_last_psi;
    core->checkpoint_short.time = 0.0;
    core->checkpoint_long.pos   = core->pos_world_f_core_last_psi;
    core->checkpoint_long.quat  = core->q_world_f_core_last_psi;
    core->checkpoint_long.time  = 0.0;

    core->physical_unmoveable = false;
    core->pinned              = false;
    core->spin_clipping_enabled = false;
    core->spin_clipping = ivp_vec3f(0.0f, 0.0f, 0.0f);

    /* Per-core friction info (Phase 3 alignment) */
    core->core_friction_info.for_unmoveables.l_friction_info_hash = NULL;

    /* Old sync rot — NULL means "not pushed in current cascade" */
    core->old_sync_rot = NULL;

    /* Multi-object core list — initialized one-to-one in object creation */
    core->object_list = NULL;
    core->object_count = 0;
}

/* ── Set pose ───────────────────────────────────────────────────────────── */

void ivp_core_set_pose(ivp_core_t *core, const ivp_vec3r_t *pos,
                       const ivp_quatr_t *quat) {
    core->pos_world_f_core_last_psi = *pos;
    core->q_world_f_core_last_psi   = *quat;
    core->q_world_f_core_next_psi   = *quat;
    ivp_quat_to_mat3(quat, &core->m_world_f_core_last_psi.rot);
    core->m_world_f_core_last_psi.pos = *pos;
    core->delta_world_f_core_psis = core->speed;

    /* Reset checkpoints */
    core->checkpoint_short.pos  = *pos;
    core->checkpoint_short.quat = *quat;
    core->checkpoint_short.time = core->time_of_last_psi;
    core->checkpoint_long.pos   = *pos;
    core->checkpoint_long.quat  = *quat;
    core->checkpoint_long.time  = core->time_of_last_psi;
}

/* ── Beam to ────────────────────────────────────────────────────────────── */

void ivp_core_beam_to(ivp_core_t *core, const ivp_vec3r_t *pos,
                      const ivp_quatr_t *quat) {
    /* Reset velocity */
    ivp_vec3r_set_zero(&core->speed);
    core->rot_speed = ivp_vec3f(0.0f, 0.0f, 0.0f);
    ivp_vec3r_set_zero(&core->speed_change);
    ivp_vec3r_set_zero(&core->rot_speed_change);
    core->current_speed = IVP_REAL(0.0);
    core->max_surface_rot_speed = IVP_REAL(0.0);
    ivp_vec3r_set_zero(&core->delta_world_f_core_psis);

    ivp_core_set_pose(core, pos, quat);
    core->movement_state = IVP_MT_MOVING;
}

/* ── Spin clipping ──────────────────────────────────────────────────────── */

void ivp_core_set_spin_clipping(ivp_core_t *core, const ivp_vec3f_t *max_spin) {
    core->spin_clipping = *max_spin;
    core->spin_clipping_enabled = true;
}

void ivp_core_clear_spin_clipping(ivp_core_t *core) {
    core->spin_clipping_enabled = false;
    core->spin_clipping = ivp_vec3f(0.0f, 0.0f, 0.0f);
}

static void apply_spin_clipping_to_vec(const ivp_vec3f_t *clipping, ivp_vec3f_t *v) {
    /* Find minimum of non-zero spin_clipping components as max magnitude */
    float max_rs = 0.0f;
    bool found = false;
    for (int i = 0; i < 3; i++) {
        if (clipping->k[i] > 0.0f) {
            if (!found || clipping->k[i] < max_rs) {
                max_rs = clipping->k[i];
                found = true;
            }
        }
    }
    if (!found) return;

    float rs2 = v->k[0] * v->k[0] + v->k[1] * v->k[1] + v->k[2] * v->k[2];
    if (rs2 > max_rs * max_rs) {
        float scale = max_rs / sqrtf(rs2);
        v->k[0] *= scale;
        v->k[1] *= scale;
        v->k[2] *= scale;
    }
}

/* ── Damping ────────────────────────────────────────────────────────────── */

void ivp_core_damp(ivp_core_t *core, ivp_real_t delta_time) {
    if (core->physical_unmoveable || core->movement_state == IVP_MT_NOT_SIM)
        return;

    /* Extra damping for slow/calm objects */
    ivp_real_t speed_damp = core->speed_damp_factor;
    ivp_vec3r_t rot_damp = core->rot_speed_damp_factor;
    if (core->movement_state >= IVP_MT_SLOW) {
        speed_damp += IVP_SLOW_ADDON_DAMPING;
        rot_damp.k[0] += IVP_SLOW_ADDON_DAMPING;
        rot_damp.k[1] += IVP_SLOW_ADDON_DAMPING;
        rot_damp.k[2] += IVP_SLOW_ADDON_DAMPING;
    }

    /* Angular damping (per-axis).
     * Matches original IVP_Core::damp_object():
     *   rot = rotation_factor * delta_time   (per-axis product)
     *   if (rot.quad_length() < 0.5)  →  linear: factor_i = 1 - rot_i
     *   else                          →  exact:  factor_i = exp(-rot_i)
     */
    ivp_real_t r0 = rot_damp.k[0] * delta_time;
    ivp_real_t r1 = rot_damp.k[1] * delta_time;
    ivp_real_t r2 = rot_damp.k[2] * delta_time;
    ivp_real_t rot_qlength = r0 * r0 + r1 * r1 + r2 * r2;

    if (rot_qlength < IVP_REAL(0.5)) {
        /* Linear approximation: factor_i = 1 - damp_i * dt */
        core->rot_speed.k[0] *= (float)(IVP_REAL(1.0) - r0);
        core->rot_speed.k[1] *= (float)(IVP_REAL(1.0) - r1);
        core->rot_speed.k[2] *= (float)(IVP_REAL(1.0) - r2);
    } else {
        /* Exact: factor_i = exp(-damp_i * dt) */
        core->rot_speed.k[0] *= (float)ivp_exp(-r0);
        core->rot_speed.k[1] *= (float)ivp_exp(-r1);
        core->rot_speed.k[2] *= (float)ivp_exp(-r2);
    }

    /* Linear damping (scalar) */
    ivp_real_t sf = speed_damp * delta_time;
    ivp_real_t factor;
    if (sf < IVP_REAL(0.25)) {
        factor = IVP_REAL(1.0) - sf;
    } else {
        factor = ivp_exp(-sf);
    }
    ivp_vec3r_scale(&core->speed, &core->speed, factor);
}

/* ── Commit async pushes ────────────────────────────────────────────────── */

void ivp_core_commit_async_pushes(ivp_core_t *core) {
    /* Merge accumulated speed_change into speed */
    core->rot_speed.k[0] += (float)core->rot_speed_change.k[0];
    core->rot_speed.k[1] += (float)core->rot_speed_change.k[1];
    core->rot_speed.k[2] += (float)core->rot_speed_change.k[2];

    ivp_vec3r_add(&core->speed, &core->speed, &core->speed_change);

    /* Clear changes */
    ivp_vec3r_set_zero(&core->rot_speed_change);
    ivp_vec3r_set_zero(&core->speed_change);
}

/* ── Init for PSI ───────────────────────────────────────────────────────── */

void ivp_core_init_for_psi_ex(ivp_core_t *core, ivp_real_t delta_time,
                               bool forward_project) {
    if (core->physical_unmoveable) return;

    /* Bake q_next_psi → matrix rotation. */
    ivp_quat_to_mat3(&core->q_world_f_core_next_psi,
                     &core->m_world_f_core_last_psi.rot);

    if (forward_project) {
        /* Legacy init_moving_core_for_psi forward-projects:
         *   m_world_f_core.pos = pos + delta_world_f_core_psis * dt
         * This makes the matrix position lead the base position by one
         * PSI step, matching the semi-implicit Euler scheme.  The collision
         * tolerance margin handles the resulting overlap. */
        core->m_world_f_core_last_psi.pos.k[0] =
            core->pos_world_f_core_last_psi.k[0] +
            core->delta_world_f_core_psis.k[0] * delta_time;
        core->m_world_f_core_last_psi.pos.k[1] =
            core->pos_world_f_core_last_psi.k[1] +
            core->delta_world_f_core_psis.k[1] * delta_time;
        core->m_world_f_core_last_psi.pos.k[2] =
            core->pos_world_f_core_last_psi.k[2] +
            core->delta_world_f_core_psis.k[2] * delta_time;
    } else {
        core->m_world_f_core_last_psi.pos = core->pos_world_f_core_last_psi;
    }
}

void ivp_core_init_for_psi(ivp_core_t *core, ivp_real_t delta_time) {
    /* Default: forward-project to match legacy behavior */
    ivp_core_init_for_psi_ex(core, delta_time, true);
}

/* ── Rotation integration (gyroscopic correction) ───────────────────────── */

static void calc_rotation_matrix(ivp_core_t *core, ivp_real_t delta_time,
                                  ivp_quatr_t *q_core_f_core) {
    if (core->rot_inertias_are_equal) {
        /* CASE 1: Equal inertias — simple rotation, no gyroscopic torque */
        ivp_quat_set_fast_multiple_with_clip(q_core_f_core, &core->rot_speed,
                                              delta_time);
        return;
    }

    /* CASE 2: Asymmetric inertia → gyroscopic correction with substeps */
    ivp_real_t ql = (ivp_real_t)core->rot_speed.k[0] * (ivp_real_t)core->rot_speed.k[0]
                  + (ivp_real_t)core->rot_speed.k[1] * (ivp_real_t)core->rot_speed.k[1]
                  + (ivp_real_t)core->rot_speed.k[2] * (ivp_real_t)core->rot_speed.k[2];
    ivp_real_t qangle = ql * delta_time * delta_time;
    ivp_real_t max_angle = (ivp_real_t)IVP_GYRO_SUBSTEP_MAX_ANGLE;

    int steps = 1;
    ivp_real_t sub_dt = delta_time;
    if (qangle > IVP_REAL(4.0) * max_angle * max_angle) {
        steps = 1 + (int)ivp_sqrt(qangle / (max_angle * max_angle));
        sub_dt = delta_time / (ivp_real_t)steps;
    }

    /* Euler equation coefficients: diff[i] = (I[j]-I[k]) / I[i] */
    ivp_real_t Ix = core->rot_inertia.k[0];
    ivp_real_t Iy = core->rot_inertia.k[1];
    ivp_real_t Iz = core->rot_inertia.k[2];
    ivp_real_t diff0 = (Iy - Iz) * core->inv_rot_inertia.k[0];
    ivp_real_t diff1 = (Iz - Ix) * core->inv_rot_inertia.k[1];
    ivp_real_t diff2 = (Ix - Iy) * core->inv_rot_inertia.k[2];

    /* First substep: build initial rotation quaternion */
    ivp_quat_set_very_fast_multiple(q_core_f_core, &core->rot_speed, sub_dt);

    for (int s = 1; s <= steps; s++) {
        /* Gyroscopic torque: dω/dt = I⁻¹ × (ω × Iω) */
        ivp_real_t wx = (ivp_real_t)core->rot_speed.k[0];
        ivp_real_t wy = (ivp_real_t)core->rot_speed.k[1];
        ivp_real_t wz = (ivp_real_t)core->rot_speed.k[2];

        ivp_real_t hp0 = wy * wz * diff0;
        ivp_real_t hp1 = wz * wx * diff1;
        ivp_real_t hp2 = wx * wy * diff2;

        core->rot_speed.k[0] += (float)(hp0 * sub_dt);
        core->rot_speed.k[1] += (float)(hp1 * sub_dt);
        core->rot_speed.k[2] += (float)(hp2 * sub_dt);

        if (s < steps) {
            /* Intermediate substep: chain rotation */
            ivp_quatr_t q_step;
            ivp_quat_set_very_fast_multiple(&q_step, &core->rot_speed, sub_dt);
            ivp_quatr_t q_combined;
            ivp_quat_mult(&q_combined, &q_step, q_core_f_core);
            *q_core_f_core = q_combined;
        }
    }
}

/* ── Calc next PSI matrix ───────────────────────────────────────────────── */

void ivp_core_calc_next_psi_matrix(ivp_core_t *core, ivp_real_t delta_time) {
    if (core->physical_unmoveable) return;
    IVP_ASSERT(delta_time > IVP_REAL(0.0));

    /* Cache speed magnitude */
    core->current_speed = ivp_vec3r_length(&core->speed);

    /* Surface rotation speed = |ω| * bounding_radius */
    ivp_real_t omega_sq =
        (ivp_real_t)core->rot_speed.k[0] * (ivp_real_t)core->rot_speed.k[0] +
        (ivp_real_t)core->rot_speed.k[1] * (ivp_real_t)core->rot_speed.k[1] +
        (ivp_real_t)core->rot_speed.k[2] * (ivp_real_t)core->rot_speed.k[2];
    core->max_surface_rot_speed = ivp_sqrt(omega_sq) * core->upper_limit_radius;

    /* Compute rotation quaternion for this step */
    ivp_quatr_t q_core_f_core;
    calc_rotation_matrix(core, delta_time, &q_core_f_core);

    /* Advance position using the PREVIOUS PSI's speed (delta_world_f_core_psis),
     * matching the legacy IVP formula:
     *   pos += delta * dt;  delta = speed;
     * This one-step lag is the standard semi-implicit Euler scheme used by the
     * original engine. After advancing, delta is updated to current speed for
     * the next PSI's position advance. */
    ivp_vec3r_add_scaled(&core->pos_world_f_core_last_psi,
                         &core->delta_world_f_core_psis, delta_time);
    core->m_world_f_core_last_psi.pos = core->pos_world_f_core_last_psi;

    core->delta_world_f_core_psis = core->speed;
    core->q_world_f_core_last_psi = core->q_world_f_core_next_psi;

    ivp_quat_mult(&core->q_world_f_core_next_psi,
                  &core->q_world_f_core_last_psi,
                  &q_core_f_core);
    ivp_quat_fast_normalize(&core->q_world_f_core_next_psi);

    /* Spin clipping: scale rot_speed if it exceeds per-object limit */
    if (core->spin_clipping_enabled) {
        apply_spin_clipping_to_vec(&core->spin_clipping, &core->rot_speed);
        /* Also clamp rot_speed_change */
        ivp_vec3f_t rsc = {{
            (float)core->rot_speed_change.k[0],
            (float)core->rot_speed_change.k[1],
            (float)core->rot_speed_change.k[2]
        }};
        apply_spin_clipping_to_vec(&core->spin_clipping, &rsc);
        core->rot_speed_change.k[0] = (ivp_real_t)rsc.k[0];
        core->rot_speed_change.k[1] = (ivp_real_t)rsc.k[1];
        core->rot_speed_change.k[2] = (ivp_real_t)rsc.k[2];
    }

    /* Store i_delta_time for AT interpolation */
    core->i_delta_time = IVP_REAL(1.0) / delta_time;
}

/* ── Movement state calculation ─────────────────────────────────────────── */

void ivp_core_calc_movement_state(ivp_core_t *core,
                                   const ivp_freeze_params_t *freeze_params) {
    if (core->physical_unmoveable || core->movement_state == IVP_MT_STATIC ||
        core->movement_state == IVP_MT_NOT_SIM) {
        return;
    }

    ivp_freeze_params_t fp;
    if (freeze_params) {
        fp = *freeze_params;
    } else {
        fp = ivp_freeze_params_default();
    }

    ivp_time_t now = core->time_of_last_psi;

    /* Short-term check */
    {
        ivp_real_t dpos2 = ivp_vec3r_quad_distance(&core->pos_world_f_core_last_psi,
                                                    &core->checkpoint_short.pos);
        ivp_real_t dquat2 = ivp_quat_estimate_qdiff_r(&core->q_world_f_core_last_psi,
                                                       &core->checkpoint_short.quat);
        /* Scale quat diff by radius for meaningful comparison */
        ivp_real_t r2 = core->upper_limit_radius * core->upper_limit_radius;
        ivp_real_t trans_thresh2 = (ivp_real_t)(fp.trans_threshold_short * fp.trans_threshold_short);
        ivp_real_t rot_thresh2   = (ivp_real_t)(fp.rot_threshold_short * fp.rot_threshold_short);

        if (dpos2 > trans_thresh2 || dquat2 * r2 > rot_thresh2) {
            /* Object has moved significantly — reset short checkpoint */
            core->checkpoint_short.pos  = core->pos_world_f_core_last_psi;
            core->checkpoint_short.quat = core->q_world_f_core_last_psi;
            core->checkpoint_short.time = now;
            core->movement_state = IVP_MT_MOVING;
        } else {
            /* Object hasn't moved much */
            ivp_time_t dt = now - core->checkpoint_short.time;
            if (dt > (ivp_time_t)fp.freeze_check_dtime) {
                core->movement_state = IVP_MT_CALM;
            } else {
                core->movement_state = IVP_MT_SLOW;
            }
        }
    }

    /* Long-term check (can force CALM even if short-term says MOVING) */
    if (core->movement_state == IVP_MT_MOVING) {
        ivp_real_t dpos2 = ivp_vec3r_quad_distance(&core->pos_world_f_core_last_psi,
                                                    &core->checkpoint_long.pos);
        ivp_real_t dquat2 = ivp_quat_estimate_qdiff_r(&core->q_world_f_core_last_psi,
                                                       &core->checkpoint_long.quat);
        ivp_real_t r2 = core->upper_limit_radius * core->upper_limit_radius;
        ivp_real_t trans_thresh2 = (ivp_real_t)(fp.trans_threshold_long * fp.trans_threshold_long);
        ivp_real_t rot_thresh2   = (ivp_real_t)(fp.rot_threshold_long * fp.rot_threshold_long);

        if (dpos2 > trans_thresh2 || dquat2 * r2 > rot_thresh2) {
            /* Significant long-term movement — reset long checkpoint */
            core->checkpoint_long.pos  = core->pos_world_f_core_last_psi;
            core->checkpoint_long.quat = core->q_world_f_core_last_psi;
            core->checkpoint_long.time = now;
        } else {
            ivp_time_t dt = now - core->checkpoint_long.time;
            if (dt > (ivp_time_t)fp.has_moved_long_time) {
                core->movement_state = IVP_MT_CALM;
            }
        }
    }
}

/* ── Push / Impulse API ────────────────────────────────────────────────── */

/* Internal: decompose impulse at world-space point into linear + angular. */
static void apply_impulse_at_point(ivp_core_t *core,
                                     const ivp_vec3r_t *pos_ws,
                                     const ivp_vec3r_t *impulse_ws,
                                     ivp_vec3r_t *out_dv,
                                     ivp_vec3r_t *out_dw)
{
    /* Linear: dv = impulse / mass */
    out_dv->k[0] = impulse_ws->k[0] * core->inv_mass;
    out_dv->k[1] = impulse_ws->k[1] * core->inv_mass;
    out_dv->k[2] = impulse_ws->k[2] * core->inv_mass;

    /* Torque: τ = r × impulse (world space) */
    ivp_vec3r_t r_ws;
    ivp_vec3r_sub(&r_ws, pos_ws, &core->pos_world_f_core_last_psi);

    ivp_vec3r_t torque_ws;
    ivp_vec3r_cross(&torque_ws, &r_ws, impulse_ws);

    /* Transform to core space */
    ivp_vec3r_t torque_cs;
    ivp_mat3r_vimult(&torque_cs, &core->m_world_f_core_last_psi.rot, &torque_ws);

    /* Angular: dω = I⁻¹ · τ (core space) */
    out_dw->k[0] = torque_cs.k[0] * core->inv_rot_inertia.k[0];
    out_dw->k[1] = torque_cs.k[1] * core->inv_rot_inertia.k[1];
    out_dw->k[2] = torque_cs.k[2] * core->inv_rot_inertia.k[2];
}

void ivp_core_push_ws(ivp_core_t *core,
                       const ivp_vec3r_t *pos_ws,
                       const ivp_vec3r_t *impulse_ws)
{
    if (core->physical_unmoveable) return;

    ivp_vec3r_t dv, dw;
    apply_impulse_at_point(core, pos_ws, impulse_ws, &dv, &dw);

    ivp_vec3r_add(&core->speed, &core->speed, &dv);
    core->rot_speed.k[0] += (float)dw.k[0];
    core->rot_speed.k[1] += (float)dw.k[1];
    core->rot_speed.k[2] += (float)dw.k[2];
}

void ivp_core_async_push_ws(ivp_core_t *core,
                              const ivp_vec3r_t *pos_ws,
                              const ivp_vec3r_t *impulse_ws)
{
    if (core->physical_unmoveable) return;

    ivp_vec3r_t dv, dw;
    apply_impulse_at_point(core, pos_ws, impulse_ws, &dv, &dw);

    ivp_vec3r_add(&core->speed_change, &core->speed_change, &dv);
    ivp_vec3r_add(&core->rot_speed_change, &core->rot_speed_change, &dw);
}

void ivp_core_center_push(ivp_core_t *core,
                            const ivp_vec3r_t *impulse_ws)
{
    if (core->physical_unmoveable) return;
    if (core->pinned) return;

    core->speed.k[0] += impulse_ws->k[0] * core->inv_mass;
    core->speed.k[1] += impulse_ws->k[1] * core->inv_mass;
    core->speed.k[2] += impulse_ws->k[2] * core->inv_mass;
}

void ivp_core_async_center_push(ivp_core_t *core,
                                  const ivp_vec3r_t *impulse_ws)
{
    if (core->physical_unmoveable) return;
    if (core->pinned) return;

    core->speed_change.k[0] += impulse_ws->k[0] * core->inv_mass;
    core->speed_change.k[1] += impulse_ws->k[1] * core->inv_mass;
    core->speed_change.k[2] += impulse_ws->k[2] * core->inv_mass;
}

void ivp_core_rot_push_cs(ivp_core_t *core,
                            const ivp_vec3r_t *torque_impulse_cs)
{
    if (core->physical_unmoveable) return;

    core->rot_speed.k[0] += (float)(torque_impulse_cs->k[0] * core->inv_rot_inertia.k[0]);
    core->rot_speed.k[1] += (float)(torque_impulse_cs->k[1] * core->inv_rot_inertia.k[1]);
    core->rot_speed.k[2] += (float)(torque_impulse_cs->k[2] * core->inv_rot_inertia.k[2]);
}

void ivp_core_async_rot_push_cs(ivp_core_t *core,
                                  const ivp_vec3r_t *torque_impulse_cs)
{
    if (core->physical_unmoveable) return;

    core->rot_speed_change.k[0] += torque_impulse_cs->k[0] * core->inv_rot_inertia.k[0];
    core->rot_speed_change.k[1] += torque_impulse_cs->k[1] * core->inv_rot_inertia.k[1];
    core->rot_speed_change.k[2] += torque_impulse_cs->k[2] * core->inv_rot_inertia.k[2];
}

/* ── Property mutation ─────────────────────────────────────────────────── */

void ivp_core_set_mass(ivp_core_t *core, ivp_real_t mass)
{
    if (mass <= IVP_REAL(0.0)) return;
    core->mass = mass;
    core->inv_mass = IVP_REAL(1.0) / mass;
}

void ivp_core_set_rotation_inertia(ivp_core_t *core,
                                     const ivp_vec3r_t *rot_inertia)
{
    core->rot_inertia = *rot_inertia;
    core->inv_rot_inertia.k[0] = (rot_inertia->k[0] > IVP_REAL_EPS)
        ? IVP_REAL(1.0) / rot_inertia->k[0] : IVP_REAL(0.0);
    core->inv_rot_inertia.k[1] = (rot_inertia->k[1] > IVP_REAL_EPS)
        ? IVP_REAL(1.0) / rot_inertia->k[1] : IVP_REAL(0.0);
    core->inv_rot_inertia.k[2] = (rot_inertia->k[2] > IVP_REAL_EPS)
        ? IVP_REAL(1.0) / rot_inertia->k[2] : IVP_REAL(0.0);

    core->rot_inertias_are_equal =
        (ivp_fabs(rot_inertia->k[0] - rot_inertia->k[1]) < IVP_REAL_EPS &&
         ivp_fabs(rot_inertia->k[1] - rot_inertia->k[2]) < IVP_REAL_EPS);
}

void ivp_core_set_pinned(ivp_core_t *core, bool pinned)
{
    core->pinned = pinned;
}

/* ── Extended queries ──────────────────────────────────────────────────── */

void ivp_core_get_surface_speed_ws(const ivp_core_t *core,
                                     const ivp_vec3r_t *point_ws,
                                     ivp_vec3r_t *out_speed_ws)
{
    /* Start with center-of-mass velocity */
    *out_speed_ws = core->speed;

    /* Add rotational contribution: v_rot = ω_ws × r */
    ivp_vec3r_t omega_cs;
    omega_cs.k[0] = (ivp_real_t)core->rot_speed.k[0];
    omega_cs.k[1] = (ivp_real_t)core->rot_speed.k[1];
    omega_cs.k[2] = (ivp_real_t)core->rot_speed.k[2];

    ivp_vec3r_t omega_ws;
    ivp_mat3r_vmult(&omega_ws, &core->m_world_f_core_last_psi.rot, &omega_cs);

    ivp_vec3r_t r_ws;
    ivp_vec3r_sub(&r_ws, point_ws, &core->pos_world_f_core_last_psi);

    ivp_vec3r_t v_rot;
    ivp_vec3r_cross(&v_rot, &omega_ws, &r_ws);
    ivp_vec3r_add(out_speed_ws, out_speed_ws, &v_rot);
}

void ivp_core_calc_at_matrix(const ivp_core_t *core,
                              ivp_time_t t,
                              ivp_transform_r_t *out)
{
    /* AT position */
    ivp_core_get_at_position(core, t, &out->pos);

    /* AT orientation → rotation matrix */
    ivp_quatr_t q;
    ivp_core_get_at_quat(core, t, &q);
    ivp_quat_to_mat3(&q, &out->rot);
}

ivp_real_t ivp_core_get_energy(const ivp_core_t *core)
{
    if (core->physical_unmoveable) return IVP_REAL(0.0);

    /* Translational: 0.5 * m * v² */
    ivp_real_t v2 = ivp_vec3r_quad_length(&core->speed);
    ivp_real_t ke_trans = IVP_REAL(0.5) * core->mass * v2;

    /* Rotational: 0.5 * ω · (I * ω) [core space, diagonal I] */
    ivp_real_t ke_rot = IVP_REAL(0.5) * (
        core->rot_inertia.k[0] * (ivp_real_t)core->rot_speed.k[0] * (ivp_real_t)core->rot_speed.k[0] +
        core->rot_inertia.k[1] * (ivp_real_t)core->rot_speed.k[1] * (ivp_real_t)core->rot_speed.k[1] +
        core->rot_inertia.k[2] * (ivp_real_t)core->rot_speed.k[2] * (ivp_real_t)core->rot_speed.k[2]
    );

    return ke_trans + ke_rot;
}
