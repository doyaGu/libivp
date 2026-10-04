/* ivp_controller_ref.h -- private reference-precision helpers for the
 * actuator / controller ports.
 *
 *  - IVP_Anchor (ivp_controller/ivp_actuator.cxx): IVP_Template_Anchor::
 *    set_anchor_position_os/ws/cs + IVP_Anchor::init_anchor.  The anchor is
 *    taken from object space to world space with the object's cache matrix
 *    (m_world_f_object at the current environment time, i.e. including the
 *    object<->core shift), back to object space (stored as
 *    IVP_U_Float_Point object_pos) and to core space with
 *    calc_m_core_f_object (stored as IVP_U_Float_Point core_pos).
 *    Anchors not fixed to an object use the environment's static object
 *    (its frame is the world frame).
 *  - The IVP_Core float routines the reference controllers call
 *    (get_surface_speed_on_test, get_diff_surface_speed_of_two_cores,
 *    async_push_core_ws, calc_virt_mass, get_rot_speed_cs, ...), with the
 *    reference IVP_U_Float_Point / IVP_DOUBLE operation order.
 *
 * libivp actuator states keep the anchors' core_pos in ivp_vec3r_t fields
 * (anchor_*_os); the values are float-exact, so converting them back to
 * ivp_vec3f_t is lossless.
 */
#ifndef IVP_CONTROLLER_REF_H
#define IVP_CONTROLLER_REF_H

#include <string.h>
#include <math.h>

#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_anchor.h>
#include <ivp/math/ivp_quat.h>

/* Like the reference pipeline sources (CMake IVP_REFERENCE_PIPELINE_SOURCES),
 * code including this header mixes IVP_FLOAT storage with IVP_DOUBLE
 * arithmetic on purpose. */
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#endif

#include "collision/ivp_lvec.h"
#include "dynamics/ivp_sim_internal.h"

/* ── IVP_Real_Object helpers ─────────────────────────────────────────────── */

/* IVP_Real_Object::calc_m_core_f_object (shift_core_f_object is an
 * IVP_U_Float_Point in the reference). */
static inline void ivp_ctl_calc_m_core_f_object(const ivp_object_t *obj, ivp_transform_r_t *m)
{
    memset(m, 0, sizeof(*m));
    m->rot.rows[0].k[0] = 1.0f;
    m->rot.rows[1].k[1] = 1.0f;
    m->rot.rows[2].k[2] = 1.0f;
    if (!obj) return;
    if (obj->q_core_f_object) ivp_quat_to_mat3(obj->q_core_f_object, &m->rot);
    m->pos.k[0] = (D)(float)obj->shift_core_f_object.k[0];
    m->pos.k[1] = (D)(float)obj->shift_core_f_object.k[1];
    m->pos.k[2] = (D)(float)obj->shift_core_f_object.k[2];
}

/* IVP_Real_Object::get_m_world_f_object_AT (identity for NULL = world). */
static inline void ivp_ctl_m_world_f_object_AT(const ivp_object_t *obj, ivp_transform_r_t *m)
{
    if (!obj || !obj->owner) {
        ivp_ctl_calc_m_core_f_object(NULL, m);
        if (obj) {
            /* no environment: fall back to the last PSI frame */
            *m = obj->physical_core->m_world_f_core_last_psi;
        }
        return;
    }
    ivp_object_get_m_world_f_object_AT(obj, m);
}

/* Anchor object: a NULL (world) anchor belongs to the environment's static
 * object in the reference.  May return NULL when no environment is known. */
static inline ivp_object_t *ivp_ctl_anchor_object(ivp_object_t *obj, const ivp_environment_t *env)
{
    if (obj) return obj;
    return env ? ivp_environment_get_static_object(env) : NULL;
}

/* IVP_Real_Object::get_cache_object_no_lock()->m_world_f_object (object
 * frame at the current environment time); objects outside an environment
 * fall back to the AT helper. */
static inline const ivp_transform_r_t *ivp_ctl_cache_m_world_f_object(ivp_object_t *obj, ivp_transform_r_t *tmp)
{
    if (obj->owner) return &ivp_object_get_cache_object_no_lock(obj)->m_world_f_object;
    ivp_ctl_m_world_f_object_AT(obj, tmp);
    return tmp;
}

/* ── IVP_Anchor ──────────────────────────────────────────────────────────── */

/* IVP_Anchor::init_anchor with IVP_Template_Anchor::coords_world: the
 * anchor's object_pos (IVP_U_Float_Point, may be NULL) and core_pos. */
static inline void ivp_ctl_anchor_core_pos_ws(ivp_object_t *obj, const ivp_vec3r_t *coords_world,
                                              ivp_vec3f_t *core_pos, ivp_vec3f_t *object_pos_out)
{
    if (!obj) {
        /* static object frame == world frame */
        lvf_set_p(core_pos, coords_world);
        if (object_pos_out) *object_pos_out = *core_pos;
        return;
    }
    ivp_transform_r_t m_core_f_object, tmp;
    ivp_ctl_calc_m_core_f_object(obj, &m_core_f_object);
    const ivp_transform_r_t *m_world_f_object = ivp_ctl_cache_m_world_f_object(obj, &tmp);
    ivp_vec3r_t obj_pos;
    lvm_vimult4(m_world_f_object, coords_world, &obj_pos);
    ivp_vec3f_t object_pos;
    lvf_set_p(&object_pos, &obj_pos);
    lvm_vmult4_ff(&m_core_f_object, &object_pos, core_pos);
    if (object_pos_out) *object_pos_out = object_pos;
}

/* IVP_Template_Anchor::set_anchor_position_os + IVP_Anchor::init_anchor. */
static inline void ivp_ctl_anchor_core_pos_os(ivp_object_t *obj, const ivp_vec3f_t *coords_os,
                                              ivp_vec3f_t *core_pos, ivp_vec3f_t *object_pos_out)
{
    if (!obj) {
        *core_pos = *coords_os;
        if (object_pos_out) *object_pos_out = *coords_os;
        return;
    }
    ivp_vec3r_t coords_world;
    ivp_transform_r_t tmp;
    lvm_vmult4_fp(ivp_ctl_cache_m_world_f_object(obj, &tmp), coords_os, &coords_world);
    ivp_ctl_anchor_core_pos_ws(obj, &coords_world, core_pos, object_pos_out);
}

/* IVP_Template_Anchor::set_anchor_position_cs + IVP_Anchor::init_anchor. */
static inline void ivp_ctl_anchor_core_pos_cs(ivp_object_t *obj, const ivp_vec3f_t *coords_cs,
                                              ivp_vec3f_t *core_pos, ivp_vec3f_t *object_pos_out)
{
    ivp_transform_r_t mat;
    ivp_ctl_calc_m_core_f_object(obj, &mat);
    ivp_vec3f_t coords_os;
    lvm_vimult4_ff(&mat, coords_cs, &coords_os);
    ivp_ctl_anchor_core_pos_os(obj, &coords_os, core_pos, object_pos_out);
}

/* The object and object_pos of an IVP_Anchor (linked when the actuator is
 * registered, see ivp_anchor.h).  A world anchor (obj NULL) belongs to the
 * static object, which never moves: it is not linked. */
static inline void ivp_ctl_anchor_set(ivp_anchor_t *anchor, ivp_object_t *obj, const ivp_vec3f_t *object_pos)
{
    if (!anchor) return;
    anchor->l_anchor_object = obj;
    anchor->object_pos = *object_pos;
}

/* Actuator setup helper: anchor given in the object space of obj
 * (set_anchor_position_os(obj, x, y, z): the IVP_DOUBLE coordinates become
 * an IVP_U_Float_Point), or in world space when obj is NULL (anchor on the
 * static object of env, whose object space is world space).  Writes the
 * anchor's core_pos (float-exact) into core_pos_out. */
static inline void ivp_ctl_anchor_setup_os(ivp_object_t *obj, const ivp_environment_t *env,
                                           const ivp_vec3r_t *coords, ivp_vec3r_t *core_pos_out,
                                           ivp_anchor_t *anchor)
{
    ivp_vec3f_t c, core_pos, object_pos;
    lvf_set_p(&c, coords);
    ivp_ctl_anchor_core_pos_os(ivp_ctl_anchor_object(obj, env), &c, &core_pos, &object_pos);
    lvp_set_f(core_pos_out, &core_pos);
    ivp_ctl_anchor_set(anchor, obj, &object_pos);
}

/* Environment of an actuator from its anchor objects. */
static inline ivp_environment_t *ivp_ctl_env_of(ivp_object_t *const *objs, int n)
{
    for (int i = 0; i < n; i++) {
        if (objs[i] && objs[i]->owner) return objs[i]->owner;
    }
    return NULL;
}

static inline ivp_environment_t *ivp_ctl_env_of2(ivp_object_t *a, ivp_object_t *b)
{
    ivp_object_t *objs[2] = { a, b };
    return ivp_ctl_env_of(objs, 2);
}

/* Core of an anchor object at simulation time (static object for NULL):
 * anchor->l_anchor_object->get_core(). */
static inline ivp_core_t *ivp_ctl_anchor_core(ivp_object_t *obj, const ivp_environment_t *env)
{
    ivp_object_t *o = ivp_ctl_anchor_object(obj, env);
    return o ? o->physical_core : NULL;
}

/* anchor->l_anchor_object->friction_core */
static inline ivp_core_t *ivp_ctl_anchor_friction_core(ivp_object_t *obj, const ivp_environment_t *env)
{
    ivp_object_t *o = ivp_ctl_anchor_object(obj, env);
    return o ? o->friction_core : NULL;
}

/* anchor->l_anchor_object->get_original_core() */
static inline ivp_core_t *ivp_ctl_anchor_original_core(ivp_object_t *obj, const ivp_environment_t *env)
{
    ivp_object_t *o = ivp_ctl_anchor_object(obj, env);
    return o ? o->original_core : NULL;
}

/* core->get_m_world_f_core_PSI()->vmult4(&anchor->core_pos, &pos_ws) */
static inline void ivp_ctl_anchor_pos_ws(const ivp_core_t *core, const ivp_vec3f_t *core_pos, ivp_vec3r_t *pos_ws)
{
    if (!core) {
        lvp_set_f(pos_ws, core_pos);
        return;
    }
    lvm_vmult4_fp(&core->m_world_f_core_last_psi, core_pos, pos_ws);
}

static inline ivp_vec3f_t ivp_ctl_f(const ivp_vec3r_t *v)
{
    ivp_vec3f_t r;
    lvf_set_p(&r, v);
    return r;
}

static inline ivp_vec3r_t ivp_ctl_r(const ivp_vec3f_t *v)
{
    ivp_vec3r_t r;
    lvp_set_f(&r, v);
    return r;
}

/* ── IVP_Core float routines ─────────────────────────────────────────────── */

/* IVP_Core::async_push_core_ws / push_core_ws with an IVP_U_Float_Point impulse. */
static inline void ivp_ctl_async_push_core_ws(ivp_core_t *core, const ivp_vec3r_t *pos_ws, const ivp_vec3f_t *imp_ws)
{
    ivp_vec3r_t imp = ivp_ctl_r(imp_ws);
    ivp_core_async_push_ws(core, pos_ws, &imp);
}

static inline void ivp_ctl_push_core_ws(ivp_core_t *core, const ivp_vec3r_t *pos_ws, const ivp_vec3f_t *imp_ws)
{
    ivp_vec3r_t imp = ivp_ctl_r(imp_ws);
    ivp_core_push_ws(core, pos_ws, &imp);
}

/* IVP_Core::get_surface_speed_on_test(point_cs, &speed, &rot_speed, out) */
static inline void ivp_ctl_surface_speed(const ivp_core_t *core, const ivp_vec3f_t *point_cs, ivp_vec3f_t *out_ws)
{
    ivp_core_get_surface_speed(core, point_cs, out_ws);
}

/* IVP_Core::get_diff_surface_speed_of_two_cores(this, other, p_this, p_other, out):
 * surface speed of this (zero if NULL / unmoveable) minus the surface speed
 * of other (skipped if NULL / unmoveable). */
static inline void ivp_ctl_diff_surface_speed(const ivp_core_t *this_core, const ivp_core_t *other_core,
                                              const ivp_vec3f_t *p_this, const ivp_vec3f_t *p_other,
                                              ivp_vec3f_t *delta_velocity_ws)
{
    if (this_core && !this_core->physical_unmoveable) {
        ivp_core_get_surface_speed(this_core, p_this, delta_velocity_ws);
    } else {
        lvf_zero(delta_velocity_ws);
    }
    if (other_core && !other_core->physical_unmoveable) {
        ivp_vec3f_t velocity_ws_2;
        ivp_core_get_surface_speed(other_core, p_other, &velocity_ws_2);
        lvf_subtract(delta_velocity_ws, &velocity_ws_2);
    }
}

/* IVP_Core::calc_virt_mass(core_point, NULL): minimal virtual mass. */
static inline double ivp_ctl_calc_virt_mass_min(const ivp_core_t *core, const ivp_vec3f_t *core_point)
{
    ivp_vec3f_t d = *core_point, iri = ivp_core_inv_rot_inertia_f(core);
    lvf_set_pairwise_mult(&d, &d, &d);
    lvf_set_pairwise_mult(&d, &d, &iri);
    double dv = (double)ivp_core_get_inv_mass_f(core) + (double)sqrt((double)lvf_quad_length(&d));
    return 1.0f / dv;
}

/* IVP_Core::get_rot_speed_cs */
static inline double ivp_ctl_get_rot_speed_cs(const ivp_core_t *core, const ivp_vec3f_t *axis_cs)
{
    return (double)lvf_dot(&core->rot_speed, axis_cs);
}

/* IVP_Core::get_rot_inertia_cs */
static inline double ivp_ctl_get_rot_inertia_cs(const ivp_core_t *core, const ivp_vec3f_t *axis_cs)
{
    ivp_vec3f_t h, ri = ivp_core_rot_inertia_f(core);
    lvf_set_pairwise_mult(&h, axis_cs, &ri);
    return sqrt((double)lvf_quad_length(&h));
}

/* IVP_Core::async_rot_push_core_multiple_cs(axis_cs, rot_impulse) */
static inline void ivp_ctl_async_rot_push_core_multiple_cs(ivp_core_t *core, const ivp_vec3f_t *axis_cs,
                                                           double rot_impulse)
{
    ivp_vec3f_t h, iri = ivp_core_inv_rot_inertia_f(core);
    lvf_set_pairwise_mult(&h, axis_cs, &iri);
    ivp_vec3f_t rsc;
    lvf_set_p(&rsc, &core->rot_speed_change);
    lvf_add_multiple(&rsc, &h, (D)rot_impulse);
    lvp_set_f(&core->rot_speed_change, &rsc);
}

/* core->speed is an IVP_U_Float_Point in the reference: keep it float-exact
 * after libivp-only controller modes changed it in double precision. */
static inline void ivp_ctl_round_speed(ivp_core_t *core)
{
    ivp_vec3f_t sp = ivp_core_speed_f(core);
    ivp_core_set_speed_f(core, &sp);
}

/* reference: movement_state < IVP_MT_NOT_SIM && !pinned */
static inline bool ivp_ctl_core_pushable(const ivp_core_t *core)
{
    return core && IVP_MTIS_SIMULATED(core->movement_state) && !core->pinned;
}

#endif /* IVP_CONTROLLER_REF_H */
