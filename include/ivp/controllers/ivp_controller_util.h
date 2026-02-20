/* ivp_controller_util.h -- Shared controller utility functions
 *
 * Inline helpers used by multiple controller implementations.
 */
#ifndef IVP_CONTROLLER_UTIL_H
#define IVP_CONTROLLER_UTIL_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/dynamics/ivp_object.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Transform an anchor from object space to world space.
 * If obj is NULL, the anchor is assumed to already be in world space. */
static inline void ivp_controller_get_anchor_ws(
    const ivp_object_t *obj,
    const ivp_vec3r_t *anchor_os,
    ivp_vec3r_t *out_ws)
{
    if (obj) {
        ivp_transform_point(out_ws,
                             &obj->core.m_world_f_core_last_psi,
                             anchor_os);
    } else {
        *out_ws = *anchor_os;
    }
}

/* Compute the world-space velocity of a point on an object.
 * Combines linear velocity with the rotational contribution: v = v_cm + omega x r.
 * If obj is NULL, outputs zero velocity. */
static inline void ivp_controller_get_point_velocity_ws(
    const ivp_object_t *obj,
    const ivp_vec3r_t *point_ws,
    ivp_vec3r_t *out_vel_ws)
{
    if (!obj) {
        ivp_vec3r_set_zero(out_vel_ws);
        return;
    }

    *out_vel_ws = obj->core.speed;

    ivp_vec3r_t omega_cs;
    omega_cs.k[0] = (ivp_real_t)obj->core.rot_speed.k[0];
    omega_cs.k[1] = (ivp_real_t)obj->core.rot_speed.k[1];
    omega_cs.k[2] = (ivp_real_t)obj->core.rot_speed.k[2];

    ivp_vec3r_t omega_ws;
    ivp_mat3r_vmult(&omega_ws, &obj->core.m_world_f_core_last_psi.rot, &omega_cs);

    ivp_vec3r_t r_ws;
    ivp_vec3r_sub(&r_ws, point_ws, &obj->core.pos_world_f_core_last_psi);

    ivp_vec3r_t v_rot;
    ivp_vec3r_cross(&v_rot, &omega_ws, &r_ws);
    ivp_vec3r_add(out_vel_ws, out_vel_ws, &v_rot);
}

/* Apply a world-space force at a world-space anchor point on an object.
 * Produces both a linear impulse (dv = F/m * dt) and an angular impulse
 * from the lever arm (tau = r x F, d_omega = I^-1 * tau * dt).
 * Accumulates into speed_change / rot_speed_change for merging before integration. */
static inline void ivp_controller_apply_force_at_anchor(
    ivp_object_t *obj,
    const ivp_vec3r_t *anchor_ws,
    const ivp_vec3r_t *force_ws,
    ivp_real_t dt)
{
    if (!obj || obj->core.physical_unmoveable) return;

    ivp_core_t *core = &obj->core;

    /* Linear impulse: dv = F * (1/m) * dt */
    ivp_vec3r_t dv;
    ivp_vec3r_scale(&dv, force_ws, core->inv_mass * dt);
    ivp_vec3r_add(&core->speed_change, &core->speed_change, &dv);

    /* Torque from lever arm: tau = r x F */
    ivp_vec3r_t r_ws;
    ivp_vec3r_sub(&r_ws, anchor_ws, &core->pos_world_f_core_last_psi);

    ivp_vec3r_t torque_ws;
    ivp_vec3r_cross(&torque_ws, &r_ws, force_ws);

    /* Transform torque to core space */
    ivp_vec3r_t torque_cs;
    ivp_mat3r_vimult(&torque_cs, &core->m_world_f_core_last_psi.rot, &torque_ws);

    /* Angular impulse: d_omega = I^-1 * torque * dt */
    ivp_vec3r_t dw;
    dw.k[0] = torque_cs.k[0] * core->inv_rot_inertia.k[0] * dt;
    dw.k[1] = torque_cs.k[1] * core->inv_rot_inertia.k[1] * dt;
    dw.k[2] = torque_cs.k[2] * core->inv_rot_inertia.k[2] * dt;
    ivp_vec3r_add(&core->rot_speed_change, &core->rot_speed_change, &dw);
}

/* Constraint solver helper: inverse virtual mass for positional correction. */
static inline ivp_real_t ivp_controller_compute_positional_inv_vm(
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

    return inv_vm;
}

/* Constraint solver helper: apply positional impulse along a world direction. */
static inline void ivp_controller_apply_positional_impulse(ivp_core_t *ca,
                                                           const ivp_vec3r_t *r_a_ws,
                                                           ivp_core_t *cb,
                                                           const ivp_vec3r_t *r_b_ws,
                                                           const ivp_vec3r_t *direction,
                                                           ivp_real_t impulse)
{
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

/* Constraint solver helper: inverse virtual mass for angular correction. */
static inline ivp_real_t ivp_controller_compute_angular_inv_vm(const ivp_core_t *ca,
                                                               const ivp_core_t *cb,
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

/* Constraint solver helper: apply angular impulse along a world axis. */
static inline void ivp_controller_apply_angular_impulse(ivp_core_t *ca,
                                                        ivp_core_t *cb,
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

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_UTIL_H */
