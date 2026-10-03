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
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/dynamics/ivp_core.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Transform an anchor given in the core space of obj (an IVP_Anchor
 * core_pos) to world space with the PSI matrix:
 * core->get_m_world_f_core_PSI()->vmult4(core_pos).
 * If obj is NULL, the anchor is assumed to already be in world space. */
static inline void ivp_controller_get_anchor_ws(
    const ivp_object_t *obj,
    const ivp_vec3r_t *anchor_os,
    ivp_vec3r_t *out_ws)
{
    if (obj) {
        ivp_transform_point(out_ws,
                             &obj->physical_core->m_world_f_core_last_psi,
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

    *out_vel_ws = obj->physical_core->speed;

    ivp_vec3r_t omega_cs;
    omega_cs.k[0] = (ivp_real_t)obj->physical_core->rot_speed.k[0];
    omega_cs.k[1] = (ivp_real_t)obj->physical_core->rot_speed.k[1];
    omega_cs.k[2] = (ivp_real_t)obj->physical_core->rot_speed.k[2];

    ivp_vec3r_t omega_ws;
    ivp_mat3r_vmult(&omega_ws, &obj->physical_core->m_world_f_core_last_psi.rot, &omega_cs);

    ivp_vec3r_t r_ws;
    ivp_vec3r_sub(&r_ws, point_ws, &obj->physical_core->pos_world_f_core_last_psi);

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
    if (!obj || obj->physical_core->physical_unmoveable) return;

    ivp_core_t *core = obj->physical_core;

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

/* ── Reference-parity helpers ───────────────────────────────────────────
 *
 * These mirror small IVP_Core helpers used by the reference controllers so
 * that the ported formulas keep the reference operation order. */

/* Reference IVP_U_(Float_)Point::real_length_plus_normize: returns 0 and
 * leaves v untouched when |v|^2 < P_DOUBLE_EPS, else normalizes in place
 * (isqrt with 3 resolution steps) and returns the length. */
static inline ivp_real_t ivp_controller_length_plus_normize(ivp_vec3r_t *v)
{
    double q = (double)v->k[0] * (double)v->k[0] +
               (double)v->k[1] * (double)v->k[1] +
               (double)v->k[2] * (double)v->k[2];
    if (q < (double)IVP_DOUBLE_EPS) return IVP_REAL(0.0);
    double f = ivp_isqrt_double(q, 3);
    v->k[0] = (ivp_real_t)((double)v->k[0] * f);
    v->k[1] = (ivp_real_t)((double)v->k[1] * f);
    v->k[2] = (ivp_real_t)((double)v->k[2] * f);
    return (ivp_real_t)(f * q);
}

/* Reference IVP_U_Point::fast_normize / normize (isqrt, 2 resolution steps).
 * Returns false (vector untouched) when |v|^2 < P_DOUBLE_EPS. */
static inline bool ivp_controller_normize(ivp_vec3r_t *v)
{
    double q = (double)v->k[0] * (double)v->k[0] +
               (double)v->k[1] * (double)v->k[1] +
               (double)v->k[2] * (double)v->k[2];
    if (q < (double)IVP_DOUBLE_EPS) return false;
    double f = ivp_isqrt_double(q, 2);
    v->k[0] = (ivp_real_t)((double)v->k[0] * f);
    v->k[1] = (ivp_real_t)((double)v->k[1] * f);
    v->k[2] = (ivp_real_t)((double)v->k[2] * f);
    return true;
}

/* Reference IVP_MTIS_SIMULATED(core->movement_state). */
static inline bool ivp_controller_core_is_simulated(const ivp_core_t *core)
{
    return core && IVP_MTIS_SIMULATED(core->movement_state);
}

/* True when the object may receive pushes from an actuator
 * (reference: movement_state < IVP_MT_NOT_SIM && !pinned; libivp revives a
 * frozen partner that received a push at the start of the next PSI, which
 * stands in for the reference's shared simulation unit). */
static inline bool ivp_controller_object_pushable(const ivp_object_t *obj)
{
    return obj && !obj->physical_core->physical_unmoveable && !obj->physical_core->pinned;
}

/* Pick the endpoint on whose controller callback a multi-object actuator
 * executes.  Reference IVP_Controller_Dependent runs once per simulation
 * unit whenever ANY of its associated (movable) cores is simulated; libivp
 * dispatches independent controllers per active core, so the first movable
 * endpoint that is currently simulated is chosen.  Returns NULL when no
 * endpoint is simulated (controller is dormant, as in the reference). */
static inline ivp_object_t *ivp_controller_pick_exec_object(ivp_object_t *const *objs, int n)
{
    for (int i = 0; i < n; i++) {
        ivp_object_t *o = objs[i];
        if (!o || o->physical_core->physical_unmoveable) continue;
        if (!ivp_controller_core_is_simulated(o->physical_core)) continue;
        return o;
    }
    return NULL;
}

/* True while the environment looks for the controllers anchored on an
 * object that is being deleted: discovery then also reports the unmoveable
 * anchor objects (the reference deletes an actuator together with any of
 * its anchor objects, static ones included). */
bool ivp_environment_discovery_reports_all_anchors(const ivp_environment_t *env);

/* Discovery helper for dependent controllers (reference
 * IVP_Controller_Dependent::get_associated_controlled_cores): writes the
 * handles of the movable objects among objs[0..n). */
static inline int ivp_controller_discover_objects(ivp_object_t *const *objs, int n,
                                                  ivp_object_handle_t *out, int max)
{
    int count = 0;
    for (int i = 0; i < n && count < max; i++) {
        const ivp_object_t *o = objs[i];
        if (!o) continue;
        if (o->physical_core->physical_unmoveable &&
            !(o->owner && ivp_environment_discovery_reports_all_anchors(o->owner))) continue;
        out[count++] = ivp_environment_get_object_handle(o);
    }
    return count;
}

/* Reference ensure_*_in_simulation for one controlled object. */
static inline void ivp_controller_wake_object(ivp_object_t *obj)
{
    if (!obj || obj->physical_core->physical_unmoveable || !obj->owner) return;
    (void)ivp_environment_ensure_core_in_simulation(obj->owner, obj);
}

/* IVP_Actuator_Two_Point::ensure_actuator_in_simulation of an actuator whose
 * anchors are attached to obj_a / obj_b (controller manager semantics: the
 * sim unit is revived at once). */
static inline void ivp_controller_wake_actuator(ivp_object_t *obj_a, ivp_object_t *obj_b)
{
    ivp_object_t *objs[2] = { obj_a, obj_b };
    ivp_environment_t *env = obj_a ? obj_a->owner : (obj_b ? obj_b->owner : NULL);
    if (env) ivp_environment_ensure_actuator_in_simulation(env, objs, 2);
}

/* Reference IVP_Core::(async_)push_core_ws (ivp_core.cxx:603-630): the
 * impulse is an IVP_U_Float_Point, the lever arm is measured from the PSI
 * matrix position (get_position_PSI()) and speed_change / rot_speed_change
 * accumulate in float.  These delegate to the core implementations
 * (ivp_core_async_push_ws / ivp_core_push_ws); static cores are skipped. */
static inline void ivp_controller_async_push_ws(ivp_core_t *core,
                                                const ivp_vec3r_t *pos_ws,
                                                const ivp_vec3r_t *imp_ws)
{
    if (!core || core->physical_unmoveable) return;
    ivp_core_async_push_ws(core, pos_ws, imp_ws);
}

static inline void ivp_controller_push_ws(ivp_core_t *core,
                                          const ivp_vec3r_t *pos_ws,
                                          const ivp_vec3r_t *imp_ws)
{
    if (!core || core->physical_unmoveable) return;
    ivp_core_push_ws(core, pos_ws, imp_ws);
}

/* Lever-arm impulse response (libivp helper): dv = imp / m,
 * dw = I^-1 (.) R^T ((pos - pos_PSI) x imp), computed like push_core_ws
 * (IVP_U_Float_Point arithmetic). */
static inline void ivp_controller_impulse_at_point(const ivp_core_t *core,
                                                   const ivp_vec3r_t *pos_ws,
                                                   const ivp_vec3r_t *imp_ws,
                                                   ivp_vec3r_t *dv, ivp_vec3r_t *dw)
{
    ivp_core_t tmp = *core;
    ivp_vec3r_set_zero(&tmp.speed_change);
    ivp_vec3r_set_zero(&tmp.rot_speed_change);
    ivp_core_async_push_ws(&tmp, pos_ws, imp_ws);
    *dv = tmp.speed_change;
    *dw = tmp.rot_speed_change;
}

/* IVP_U_Float_Point::inline_calc_cross_product */
static inline void ivp_controller_cross_f(ivp_vec3f_t *r, const ivp_vec3f_t *v1, const ivp_vec3f_t *v2)
{
    double a = (double)(v1->k[1] * v2->k[2]);
    double a2 = (double)(v2->k[1] * v1->k[2]);
    double b = (double)(v1->k[2] * v2->k[0]);
    double b2 = (double)(v2->k[2] * v1->k[0]);
    a -= a2;
    double c = (double)(v1->k[0] * v2->k[1]);
    double c2 = (double)(v2->k[0] * v1->k[1]);
    b -= b2;
    c -= c2;
    r->k[0] = (float)a;
    r->k[1] = (float)b;
    r->k[2] = (float)c;
}

/* Reference IVP_Core::get_surface_speed_on_test(point_cs, &speed, &rot_speed):
 * speed_cs = rot_speed x p_cs (float), v_ws = R * speed_cs (float) + speed.
 * Static / NULL cores yield zero.  The result is float-exact. */
static inline void ivp_controller_surface_speed_cs(const ivp_core_t *core,
                                                   const ivp_vec3r_t *point_cs,
                                                   ivp_vec3r_t *out_ws)
{
    if (!core || core->physical_unmoveable) {
        ivp_vec3r_set_zero(out_ws);
        return;
    }
    ivp_vec3f_t p, s_cs;
    p.k[0] = (float)point_cs->k[0];
    p.k[1] = (float)point_cs->k[1];
    p.k[2] = (float)point_cs->k[2];
    ivp_controller_cross_f(&s_cs, &core->rot_speed, &p);
    const ivp_mat3r_t *m = &core->m_world_f_core_last_psi.rot;
    for (int i = 0; i < 3; i++) {
        double a = (double)m->rows[i].k[0] * (double)s_cs.k[0] + (double)m->rows[i].k[1] * (double)s_cs.k[1] +
                   (double)m->rows[i].k[2] * (double)s_cs.k[2];
        float v = (float)a;
        v = v + (float)core->speed.k[i];
        out_ws->k[i] = (ivp_real_t)v;
    }
}

/* Reference IVP_Core::get_diff_surface_speed_of_two_cores(this, other, ...):
 * out = surface_speed(this @ p_this) - surface_speed(other @ p_other)
 * (IVP_U_Float_Point subtraction). */
static inline void ivp_controller_diff_surface_speed(const ivp_core_t *core_this,
                                                     const ivp_core_t *core_other,
                                                     const ivp_vec3r_t *p_this_cs,
                                                     const ivp_vec3r_t *p_other_cs,
                                                     ivp_vec3r_t *out_ws)
{
    ivp_vec3r_t a, b;
    ivp_controller_surface_speed_cs(core_this, p_this_cs, &a);
    ivp_controller_surface_speed_cs(core_other, p_other_cs, &b);
    for (int i = 0; i < 3; i++) out_ws->k[i] = (ivp_real_t)((float)a.k[i] - (float)b.k[i]);
}

/* Reference IVP_Core::calc_virt_mass(core_point, NULL): minimal virtual mass
 * of a point (IVP_U_Float_Point arithmetic).  Static / NULL cores are
 * treated as infinitely heavy. */
static inline ivp_real_t ivp_controller_calc_virt_mass_min(const ivp_core_t *core,
                                                           const ivp_vec3r_t *point_cs)
{
    if (!core || core->physical_unmoveable) return IVP_REAL_MAX;
    float d[3];
    for (int i = 0; i < 3; i++) {
        float p = (float)point_cs->k[i];
        d[i] = (p * p) * (float)core->inv_rot_inertia.k[i];
    }
    double dv = (double)(float)core->inv_mass + sqrt((double)(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
    return (ivp_real_t)(1.0 / dv);
}

/* Reference IVP_Core::get_rot_inertia_cs: |I (.) axis_cs| (float). */
static inline ivp_real_t ivp_controller_rot_inertia_cs(const ivp_core_t *core,
                                                       const ivp_vec3r_t *axis_cs)
{
    float h[3];
    for (int i = 0; i < 3; i++) h[i] = (float)axis_cs->k[i] * (float)core->rot_inertia.k[i];
    return (ivp_real_t)sqrt((double)(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]));
}

/* Reference IVP_Core::async_rot_push_core_multiple_cs:
 * rot_speed_change += (axis_cs (.) inv_I) * rot_impulse (float). */
static inline void ivp_controller_async_rot_push_axis_cs(ivp_core_t *core,
                                                         const ivp_vec3r_t *axis_cs,
                                                         ivp_real_t rot_impulse)
{
    if (!core || core->physical_unmoveable) return;
    for (int i = 0; i < 3; i++) {
        float h = (float)axis_cs->k[i] * (float)core->inv_rot_inertia.k[i];
        double a = (double)h * (double)rot_impulse + (double)(float)core->rot_speed_change.k[i];
        core->rot_speed_change.k[i] = (ivp_real_t)(float)a;
    }
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
