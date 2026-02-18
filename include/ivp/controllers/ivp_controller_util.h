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

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_UTIL_H */
