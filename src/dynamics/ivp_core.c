/* ivp_core.c — port of IVP_Core (ivp_physics/ivp_core.cxx, ivp_core_macros.hxx),
 * IVP_Calc_Next_PSI_Solver (ivp_intern/ivp_calc_next_psi_solver.cxx) and the core
 * revive/freeze parts of ivp_intern/ivp_physic_private.cxx.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/diagnostics/ivp_debug_manager.h>
#include <ivp/utility/ivp_string.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/collision/ivp_hull_mgr.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include "collision/ivp_lvec.h"
#include "dynamics/ivp_sim_internal.h"
#include "dynamics/ivp_env_hot.h"
#include "solver/ivp_friction_internal.h"
#include "controllers/ivp_car_wheel_physics.h"

void ivp_core_free_friction_info_hash(ivp_core_t *core);   /* ivp_contact_point.c */

/* ── helpers ───────────────────────────────────────────────────────────── */

static inline ivp_vec3f_t vr_to_f(const ivp_vec3r_t *v)
{
    ivp_vec3f_t r;
    r.k[0] = (float)v->k[0];
    r.k[1] = (float)v->k[1];
    r.k[2] = (float)v->k[2];
    return r;
}

/* ── lifecycle / public API ────────────────────────────────────────────── */

void ivp_core_init(ivp_core_t *core, ivp_real_t mass, const ivp_vec3r_t *rot_inertia)
{
    memset(core, 0, sizeof(*core));
    ivp_pvec_init(&core->controllers_of_core);
    core->movement_state = IVP_MT_NOT_SIM;
    core->q_world_f_core_last_psi = ivp_quatr_identity();
    core->q_world_f_core_next_psi = core->q_world_f_core_last_psi;
    ivp_quat_to_mat3(&core->q_world_f_core_last_psi, &core->m_world_f_core_last_psi.rot);
    core->upper_limit_radius = IVP_REAL(1000.0);      /* IVP_Core ctor: set_radius(1000, 1000) */
    core->max_surface_deviation = IVP_REAL(1000.0);
    /* rot_inertia / mass are IVP_U_Float_Hesse (float) in the reference */
    core->rot_inertia = rot_inertia ? *rot_inertia : ivp_vec3r(IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    for (int i = 0; i < 3; i++) core->rot_inertia.k[i] = (ivp_real_t)(float)core->rot_inertia.k[i];
    core->mass = (ivp_real_t)(float)((mass >= (ivp_real_t)P_RES_EPS) ? mass : (ivp_real_t)P_RES_EPS);
    core->speed_damp_factor = IVP_REAL(0.01);
    core->rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.01), IVP_REAL(0.01), IVP_REAL(0.01));
    ivp_core_calc_calc(core);
}

void ivp_core_release(ivp_core_t *core)
{
    ivp_core_free_friction_info_hash(core);
    ivp_pvec_clear(&core->controllers_of_core);
    ivp_pvec_clear(&core->objects);
}

/* ── cores of objects (IVP_Core ctors / dtor, ivp_core.cxx) ────────────── */

/* IVP_Core::init: every value zero (quaternions and matrix included), then
 * link the object, a new simulation unit and the standard gravity controller */
static bool core_init_ref(ivp_core_t *core, ivp_object_t *io)
{
    memset(core, 0, sizeof(*core));
    core->environment = io->owner;
    core->heap_allocation_size = sizeof(*core);
    const ivp_allocator_t *allocator = ivp_env_get_allocator(core->environment);
    ivp_pvec_init_with_allocator(&core->controllers_of_core, allocator);
    ivp_pvec_init_with_allocator(&core->objects, allocator);
    if (ivp_pvec_add(&core->objects, io) < 0) return false;
    core->sim_unit = ivp_sim_unit_new(allocator);
    if (!core->sim_unit) goto fail;
    if (!ivp_sim_unit_add_sim_unit_core(core->sim_unit, core)) goto fail;
    core->sim_unit->sim_unit_movement_type = IVP_MT_NOT_SIM;
    if (!ivp_core_add_core_controller(core, ivp_env_get_standard_gravity_controller(core->environment))) goto fail;
    core->movement_state = IVP_MT_NOT_SIM;
    return true;

fail:
    ivp_sim_unit_delete(core->sim_unit);
    core->sim_unit = NULL;
    ivp_pvec_clear(&core->controllers_of_core);
    ivp_pvec_clear(&core->objects);
    return false;
}

/* IVP_Core::IVP_Core(IVP_Real_Object *io): init + register the sim unit */
bool ivp_core_init_from_object(ivp_core_t *core, ivp_object_t *io)
{
    if (!core_init_ref(core, io)) return false;
    ivp_sim_units_mgr_add_sim_unit(ivp_env_get_sim_units_manager(core->environment), core->sim_unit);
    return true;
}

/* IVP_Core::IVP_Core(object, q_world_f_object_init, position, physical_unmoveable, enable_piling_optimization) */
ivp_core_t *ivp_core_create(ivp_object_t *io, const ivp_quatr_t *q_world_f_object_init, const ivp_vec3r_t *position,
                            bool physical_unmoveable, bool enable_piling_optimization)
{
    const ivp_allocator_t *allocator = ivp_env_get_allocator(io->owner);
    ivp_core_t *core = (ivp_core_t *)ivp_alloc(allocator, sizeof(ivp_core_t));
    if (!core) return NULL;
    if (!core_init_ref(core, io)) {
        ivp_free(allocator, core, sizeof(*core));
        return NULL;
    }
    core->physical_unmoveable = physical_unmoveable;
    ivp_sim_units_mgr_add_sim_unit(ivp_env_get_sim_units_manager(core->environment), core->sim_unit);
    core->q_world_f_core_last_psi = *q_world_f_object_init;
    (void)ivp_quat_normalize(&core->q_world_f_core_last_psi);   /* normize_quat */
    core->q_world_f_core_next_psi = core->q_world_f_core_last_psi;
    core->pos_world_f_core_last_psi = *position;
    ivp_vec3r_set_zero(&core->delta_world_f_core_psis);
    ivp_quat_to_mat3(&core->q_world_f_core_last_psi, &core->m_world_f_core_last_psi.rot);
    core->m_world_f_core_last_psi.pos = *position;
    core->upper_limit_radius = IVP_REAL(1000.0);   /* set_radius(1000.0f, 1000.0f) */
    core->max_surface_deviation = IVP_REAL(1000.0);
    core->fast_piling_allowed = enable_piling_optimization;
    return core;
}

/* IVP_Core::~IVP_Core (+ delete) */
void ivp_core_delete(ivp_core_t *core)
{
    struct ivp_environment *env = ivp_core_env(core);
    const size_t allocation_size = core->heap_allocation_size;
    ivp_env_remove_revive_core(env, core);
    if (!core->physical_unmoveable) {
        for (int i = ivp_pvec_len(&core->controllers_of_core) - 1; i >= 0; i--) {
            if (i >= ivp_pvec_len(&core->controllers_of_core)) continue;
            ivp_sim_ctrl_t *c = (ivp_sim_ctrl_t *)ivp_pvec_at(&core->controllers_of_core, i);
            c->vt->core_is_going_to_be_deleted_event(c, core);
        }
    }
    if (core->sim_unit) {
        ivp_sim_unit_sim_unit_remove_core(core->sim_unit, core);
        core->sim_unit = NULL;
    }
    ivp_core_release(core);   /* friction info hash of unmoveable cores, vectors */
    IVP_ASSERT(allocation_size >= sizeof(*core));
    ivp_free(ivp_env_get_allocator(env), core, allocation_size);
}

/* IVP_Core::core_add_link_to_obj */
bool ivp_core_core_add_link_to_obj(ivp_core_t *core, ivp_object_t *add_obj)
{
    IVP_IF(1)
    {
        for (int j = ivp_core_n_objects(core) - 1; j >= 0; j--) {
            IVP_ASSERT(ivp_core_object_at(core, j) != add_obj);
        }
    }
    return ivp_pvec_add(&core->objects, add_obj) >= 0;
}

/* IVP_Core::unlink_obj_from_core_and_maybe_destroy */
void ivp_core_unlink_obj_from_core_and_maybe_destroy(ivp_core_t *core, ivp_object_t *remove_obj)
{
    ivp_pvec_remove(&core->objects, remove_obj);
    if (ivp_pvec_len(&core->objects) == 0) {
        if (IVP_MTIS_SIMULATED(core->movement_state)) ivp_core_stop_physical_movement(core);
        ivp_core_delete(core);
    }
}

void ivp_core_set_pose(ivp_core_t *core, const ivp_vec3r_t *pos, const ivp_quatr_t *quat)
{
    core->q_world_f_core_last_psi = *quat;
    (void)ivp_quat_normalize(&core->q_world_f_core_last_psi);
    core->q_world_f_core_next_psi = core->q_world_f_core_last_psi;
    core->pos_world_f_core_last_psi = *pos;
    ivp_vec3r_set_zero(&core->delta_world_f_core_psis);
    ivp_quat_to_mat3(&core->q_world_f_core_last_psi, &core->m_world_f_core_last_psi.rot);
    core->m_world_f_core_last_psi.pos = *pos;
}

void ivp_core_beam_to(ivp_core_t *core, const ivp_vec3r_t *pos, const ivp_quatr_t *quat)
{
    ivp_vec3r_set_zero(&core->speed);
    lvf_zero(&core->rot_speed);
    ivp_vec3r_set_zero(&core->speed_change);
    ivp_vec3r_set_zero(&core->rot_speed_change);
    core->current_speed = 0.0f;
    core->max_surface_rot_speed = 0.0f;
    ivp_core_set_pose(core, pos, quat);
}

void ivp_core_damp(ivp_core_t *core, ivp_real_t delta_time) { ivp_core_global_damp_core(core, (double)delta_time); }

void ivp_core_commit_async_pushes(ivp_core_t *core) { ivp_core_commit_all_async_pushes_ref(core); }

void ivp_core_set_spin_clipping(ivp_core_t *core, const ivp_vec3f_t *max_spin)
{
    core->spin_clipping = *max_spin;
    core->spin_clipping_enabled = true;
}

void ivp_core_clear_spin_clipping(ivp_core_t *core) { core->spin_clipping_enabled = false; }

/* IVP_Core::inline_calc_at_quaternion */
void ivp_core_inline_calc_at_quaternion(const ivp_core_t *core, ivp_time_t t, ivp_quatr_t *out)
{
    double d_time = ivp_time_diff(t, core->time_of_last_psi);
    ivp_quat_set_interpolate_smoothly(out, &core->q_world_f_core_last_psi, &core->q_world_f_core_next_psi,
                                      (ivp_real_t)(d_time * (double)core->i_delta_time));
}

/* IVP_Core::inline_calc_at_position */
void ivp_core_inline_calc_at_position(const ivp_core_t *core, ivp_time_t t, ivp_vec3r_t *out)
{
    double d_time = ivp_time_diff(t, core->time_of_last_psi);
    lvp_add_multiple2(out, &core->pos_world_f_core_last_psi, &core->delta_world_f_core_psis, (D)d_time);
}

void ivp_core_get_at_position(const ivp_core_t *core, ivp_time_t t, ivp_vec3r_t *out)
{
    ivp_core_inline_calc_at_position(core, t, out);
}

void ivp_core_get_at_quat(const ivp_core_t *core, ivp_time_t t, ivp_quatr_t *out)
{
    ivp_core_inline_calc_at_quaternion(core, t, out);
}

/* IVP_Core::calc_at_matrix */
void ivp_core_calc_at_matrix_ref(const ivp_core_t *core, ivp_time_t current_time, ivp_transform_r_t *out)
{
    while (IVP_MTIS_SIMULATED(core->movement_state)) {
        double d_time = ivp_time_diff(current_time, core->time_of_last_psi);
        if (d_time == 0.0) break;
        ivp_quatr_t res;
        ivp_quat_set_interpolate_smoothly(&res, &core->q_world_f_core_last_psi, &core->q_world_f_core_next_psi,
                                          (ivp_real_t)(d_time * (double)core->i_delta_time));
        ivp_quat_to_mat3(&res, &out->rot);
        lvp_add_multiple2(&out->pos, &core->pos_world_f_core_last_psi, &core->delta_world_f_core_psis, (D)d_time);
        return;
    }
    *out = core->m_world_f_core_last_psi;
}

void ivp_core_calc_at_matrix(const ivp_core_t *core, ivp_time_t t, ivp_transform_r_t *out)
{
    ivp_core_calc_at_matrix_ref(core, t, out);
}

/* ── pushes (ivp_core.cxx) ─────────────────────────────────────────────── */

/* IVP_Core::test_push_core */
void ivp_core_test_push_core(const ivp_core_t *core, const ivp_vec3f_t *point_cs, const ivp_vec3f_t *impulse_cs,
                             const ivp_vec3f_t *impulse_ws, ivp_vec3f_t *speed_out, ivp_vec3f_t *rot_out)
{
    ivp_vec3f_t distances_2d, iri = ivp_core_inv_rot_inertia_f(core);
    lvf_cross(&distances_2d, point_cs, impulse_cs);
    lvf_set_pairwise_mult(rot_out, &distances_2d, &iri);
    lvf_set_multiple(speed_out, impulse_ws, (D)ivp_core_get_inv_mass_f(core));
}

/* IVP_Core::async_push_core */
void ivp_core_async_push_core(ivp_core_t *core, const ivp_vec3f_t *point_cs, const ivp_vec3f_t *impulse_cs,
                              const ivp_vec3f_t *impulse_ws)
{
    ivp_vec3f_t rot_change, center_change;
    ivp_core_test_push_core(core, point_cs, impulse_cs, impulse_ws, &center_change, &rot_change);
    ivp_vec3f_t rsc = vr_to_f(&core->rot_speed_change), sc = vr_to_f(&core->speed_change);
    lvf_add(&rsc, &rot_change);
    lvf_add(&sc, &center_change);
    lvp_set_f(&core->rot_speed_change, &rsc);
    lvp_set_f(&core->speed_change, &sc);
}

/* IVP_Core::push_core_ws / async_push_core_ws */
static void core_push_ws_impl(ivp_core_t *core, const ivp_vec3r_t *world_point, const ivp_vec3r_t *impulse_r, bool async)
{
    const ivp_transform_r_t *m_world_f_core = &core->m_world_f_core_last_psi;
    ivp_vec3f_t impulse_in_world = vr_to_f(impulse_r);
    ivp_vec3f_t point_d_ws;
    lvf_subtract2_p(&point_d_ws, world_point, &m_world_f_core->pos);
    ivp_vec3f_t cross_point_dir;
    lvf_cross(&cross_point_dir, &point_d_ws, &impulse_in_world);
    ivp_vec3f_t tmp;
    lvm_vimult3_f(m_world_f_core, &cross_point_dir, &tmp);
    ivp_vec3f_t iri = ivp_core_inv_rot_inertia_f(core);
    lvf_set_pairwise_mult(&cross_point_dir, &tmp, &iri);
    if (async) {
        ivp_vec3f_t rsc = vr_to_f(&core->rot_speed_change), sc = vr_to_f(&core->speed_change);
        lvf_add(&rsc, &cross_point_dir);
        lvf_add_multiple(&sc, &impulse_in_world, (D)ivp_core_get_inv_mass_f(core));
        lvp_set_f(&core->rot_speed_change, &rsc);
        lvp_set_f(&core->speed_change, &sc);
    } else {
        lvf_add(&core->rot_speed, &cross_point_dir);
        ivp_vec3f_t sp = ivp_core_speed_f(core);
        lvf_add_multiple(&sp, &impulse_in_world, (D)ivp_core_get_inv_mass_f(core));
        ivp_core_set_speed_f(core, &sp);
    }
}

void ivp_core_push_ws(ivp_core_t *core, const ivp_vec3r_t *pos_ws, const ivp_vec3r_t *impulse_ws)
{
    core_push_ws_impl(core, pos_ws, impulse_ws, false);
}

void ivp_core_async_push_ws(ivp_core_t *core, const ivp_vec3r_t *pos_ws, const ivp_vec3r_t *impulse_ws)
{
    core_push_ws_impl(core, pos_ws, impulse_ws, true);
}

/* center_push_core_multiple_ws(impulse, 1.0) */
void ivp_core_center_push(ivp_core_t *core, const ivp_vec3r_t *impulse_ws)
{
    ivp_vec3f_t imp = vr_to_f(impulse_ws), sp = ivp_core_speed_f(core);
    lvf_add_multiple(&sp, &imp, (D)(1.0f * ivp_core_get_inv_mass_f(core)));
    ivp_core_set_speed_f(core, &sp);
}

void ivp_core_async_center_push(ivp_core_t *core, const ivp_vec3r_t *impulse_ws)
{
    ivp_vec3f_t imp = vr_to_f(impulse_ws), sc = vr_to_f(&core->speed_change);
    lvf_add_multiple(&sc, &imp, (D)(1.0f * ivp_core_get_inv_mass_f(core)));
    lvp_set_f(&core->speed_change, &sc);
}

/* rot_push_core_cs */
void ivp_core_rot_push_cs(ivp_core_t *core, const ivp_vec3r_t *torque_impulse_cs)
{
    ivp_vec3f_t ri = vr_to_f(torque_impulse_cs), iri = ivp_core_inv_rot_inertia_f(core), h;
    lvf_set_pairwise_mult(&h, &ri, &iri);
    lvf_add(&core->rot_speed, &h);
}

/* async_rot_push_core_multiple_cs(impulse, 1.0) */
void ivp_core_async_rot_push_cs(ivp_core_t *core, const ivp_vec3r_t *torque_impulse_cs)
{
    ivp_vec3f_t ri = vr_to_f(torque_impulse_cs), iri = ivp_core_inv_rot_inertia_f(core), h;
    lvf_set_pairwise_mult(&h, &ri, &iri);
    ivp_vec3f_t rsc = vr_to_f(&core->rot_speed_change);
    lvf_add_multiple(&rsc, &h, 1.0f);
    lvp_set_f(&core->rot_speed_change, &rsc);
}

/* IVP_U_Float_Point::real_length = sqrtd(float quad_length). */
static double core_float_point_real_length(const ivp_vec3r_t *v)
{
    const float x = (float)v->k[0], y = (float)v->k[1], z = (float)v->k[2];
    return sqrt((double)(x * x + y * y + z * z));
}

/* IVP_Core::calc_calc */
void ivp_core_calc_calc(ivp_core_t *core)
{
    /* clamp inertia and mass to a finite, positive range first, then check
     * the length (reference 1587518): zero inertia input is valid */
    const float min_inertia = P_FLOAT_EPS;
    const float max_inertia = P_FLOAT_MAX;
    for (int i = 0; i < 3; i++) {
        float k = (float)core->rot_inertia.k[i];
        if (!(k >= min_inertia && k <= max_inertia)) {
            core->rot_inertia.k[i] = (ivp_real_t)((k < min_inertia) ? min_inertia : max_inertia);
        }
    }
    float m = (float)core->mass;
    if (!((double)m >= P_RES_EPS && (double)m <= P_DOUBLE_MAX)) core->mass = (ivp_real_t)(float)P_RES_EPS;
    IVP_ASSERT(core_float_point_real_length(&core->rot_inertia) > (double)P_DOUBLE_EPS);
    core->inv_rot_inertia.k[0] = (ivp_real_t)(1.0f / (float)core->rot_inertia.k[0]);
    core->inv_rot_inertia.k[1] = (ivp_real_t)(1.0f / (float)core->rot_inertia.k[1]);
    core->inv_rot_inertia.k[2] = (ivp_real_t)(1.0f / (float)core->rot_inertia.k[2]);
    core->inv_mass = (ivp_real_t)(1.0f / (float)core->mass);
    ivp_vec3f_t iri = ivp_core_inv_rot_inertia_f(core);
    ivp_vec3f_t diff_vec;
    lvf_set3(&diff_vec, iri.k[1] - iri.k[2], iri.k[2] - iri.k[0], iri.k[0] - iri.k[1]);
    double qdiff = (double)lvf_quad_length(&diff_vec);
    double qrlen = (double)lvf_quad_length(&iri);
    const double eps = 0.1f;
    core->rot_inertias_are_equal = (qdiff < qrlen * (eps * eps));
    core->inv_object_diameter = 0.5f / (float)core->upper_limit_radius;
}

/* IVP_Core::set_mass */
void ivp_core_set_mass(ivp_core_t *core, ivp_real_t new_mass_in)
{
    float new_mass = (float)new_mass_in;
    if (!((double)new_mass >= P_RES_EPS && (double)new_mass <= P_DOUBLE_MAX)) new_mass = (float)P_RES_EPS;
    double old_mass = (double)(float)core->mass;
    if (old_mass >= P_RES_EPS && old_mass <= P_DOUBLE_MAX) {
        double factor = (double)new_mass / old_mass;
        for (int i = 0; i < 3; i++) core->rot_inertia.k[i] = (ivp_real_t)(float)((double)(float)core->rot_inertia.k[i] * factor);
    } else {
        core->rot_inertia.k[0] = core->rot_inertia.k[1] = core->rot_inertia.k[2] = (ivp_real_t)P_FLOAT_EPS;
    }
    core->mass = (ivp_real_t)new_mass;
    ivp_core_calc_calc(core);
}

/* IVP_Core::set_rotation_inertia */
void ivp_core_set_rotation_inertia(ivp_core_t *core, const ivp_vec3r_t *r)
{
    const float min_inertia = P_FLOAT_EPS;
    const float max_inertia = P_FLOAT_MAX;
    for (int i = 0; i < 3; i++) {
        float k = (float)r->k[i];
        if (!(k >= min_inertia && k <= max_inertia)) k = (k < min_inertia) ? min_inertia : max_inertia;
        core->rot_inertia.k[i] = (ivp_real_t)k;
    }
    ivp_core_calc_calc(core);
}

void ivp_core_set_pinned(ivp_core_t *core, bool pinned) { core->pinned = pinned; }

/* IVP_Core::get_surface_speed_ws */
void ivp_core_get_surface_speed_ws(const ivp_core_t *core, const ivp_vec3r_t *position_ws_in, ivp_vec3r_t *speed_ws_out)
{
    ivp_vec3f_t av_ws;
    lvm_vmult3_f(&core->m_world_f_core_last_psi, &core->rot_speed, &av_ws);
    ivp_vec3f_t pos_rel;
    lvf_subtract2_p(&pos_rel, position_ws_in, &core->m_world_f_core_last_psi.pos);
    ivp_vec3f_t cross;
    lvf_cross(&cross, &av_ws, &pos_rel);
    ivp_vec3f_t sp = ivp_core_speed_f(core), out;
    lvf_add2(&out, &sp, &cross);
    lvp_set_f(speed_ws_out, &out);
}

ivp_real_t ivp_core_get_energy(const ivp_core_t *core)
{
    ivp_vec3f_t sp = ivp_core_speed_f(core);
    return (ivp_real_t)ivp_core_get_energy_on_test(core, &sp, &core->rot_speed);
}

/* ── internal reference functions ──────────────────────────────────────── */

void ivp_core_get_surface_speed_on_test(const ivp_core_t *core, const ivp_vec3f_t *point_cs,
                                        const ivp_vec3f_t *center_speed_ws, const ivp_vec3f_t *rot_speed_cs,
                                        ivp_vec3f_t *speed_out_ws)
{
    ivp_vec3f_t speed_cs;
    lvf_cross(&speed_cs, rot_speed_cs, point_cs);
    lvm_vmult3_f(&core->m_world_f_core_last_psi, &speed_cs, speed_out_ws);
    lvf_add(speed_out_ws, center_speed_ws);
}

void ivp_core_get_surface_speed(const ivp_core_t *core, const ivp_vec3f_t *point_cs, ivp_vec3f_t *out_ws)
{
    ivp_vec3f_t sp = ivp_core_speed_f(core);
    ivp_core_get_surface_speed_on_test(core, point_cs, &sp, &core->rot_speed, out_ws);
}

void ivp_core_get_diff_surface_speed_of_two_cores_on_test(const ivp_core_t *this_core, const ivp_core_t *other_core,
                                                          const ivp_vec3f_t *p_this, const ivp_vec3f_t *p_other,
                                                          const ivp_vec3f_t *ts0, const ivp_vec3f_t *rs0,
                                                          const ivp_vec3f_t *ts1, const ivp_vec3f_t *rs1, ivp_vec3f_t *out)
{
    ivp_vec3f_t world_this, world_other;
    if (!this_core->physical_unmoveable) {
        ivp_core_get_surface_speed_on_test(this_core, p_this, ts0, rs0, &world_this);
    } else {
        lvf_zero(&world_this);
    }
    if (!other_core->physical_unmoveable) {
        ivp_core_get_surface_speed_on_test(other_core, p_other, ts1, rs1, &world_other);
    } else {
        lvf_zero(&world_other);
    }
    lvf_subtract2(out, &world_this, &world_other);
}

double ivp_core_calc_virt_mass_worst_case(const ivp_core_t *core, const ivp_vec3f_t *core_point)
{
    ivp_vec3f_t worst_direction, iri = ivp_core_inv_rot_inertia_f(core);
    worst_direction.k[0] = core_point->k[1] * core_point->k[1] + core_point->k[2] * core_point->k[2];
    worst_direction.k[1] = core_point->k[0] * core_point->k[0] + core_point->k[2] * core_point->k[2];
    worst_direction.k[2] = core_point->k[0] * core_point->k[0] + core_point->k[1] * core_point->k[1];
    lvf_set_pairwise_mult(&worst_direction, &worst_direction, &iri);
    double max_of_coords = (double)worst_direction.k[0];
    if ((double)worst_direction.k[1] > max_of_coords) max_of_coords = (double)worst_direction.k[1];
    if ((double)worst_direction.k[2] > max_of_coords) max_of_coords = (double)worst_direction.k[2];
    double dv;
    if (!core->pinned) {
        dv = (double)ivp_core_get_inv_mass_f(core) + max_of_coords;
    } else {
        dv = 1.0 + max_of_coords;
    }
    double ret = 1.0f / dv;
    IVP_ASSERT(ret < 1e31f); /* OG test for NaN */
    return ret;
}

double ivp_core_calc_correct_virt_mass(const ivp_core_t *core, const ivp_vec3f_t *core_point,
                                       const ivp_vec3f_t *direction_core, const ivp_vec3f_t *direction_world)
{
    if (!core->pinned) {
        ivp_vec3f_t ret_center_speed, ret_rot_speed;
        ivp_core_test_push_core(core, core_point, direction_core, direction_world, &ret_center_speed, &ret_rot_speed);
        ivp_vec3f_t ret_world_speed;
        ivp_core_get_surface_speed_on_test(core, core_point, &ret_center_speed, &ret_rot_speed, &ret_world_speed);
        double speed_after_test_push = (double)lvf_real_length(&ret_world_speed);
        return 1.0f / speed_after_test_push;
    }
    return 1.0;
}

double ivp_core_get_energy_on_test(const ivp_core_t *core, const ivp_vec3f_t *speed_vec, const ivp_vec3f_t *rot_speed_vec)
{
    double energy = (double)lvf_quad_length(speed_vec) * (double)ivp_core_get_mass_f(core);
    ivp_vec3f_t hp, ri = ivp_core_rot_inertia_f(core);
    lvf_set_pairwise_mult(&hp, rot_speed_vec, rot_speed_vec);
    energy += (double)lvf_dot(&hp, &ri);
    return 0.5f * energy;
}

void ivp_core_commit_all_async_pushes_ref(ivp_core_t *core)
{
    ivp_vec3f_t rsc = vr_to_f(&core->rot_speed_change), sc = vr_to_f(&core->speed_change);
    lvf_add(&core->rot_speed, &rsc);
    ivp_vec3f_t sp = ivp_core_speed_f(core);
    lvf_add(&sp, &sc);
    ivp_core_set_speed_f(core, &sp);
    ivp_vec3r_set_zero(&core->speed_change);
    ivp_vec3r_set_zero(&core->rot_speed_change);
    IVP_IF(1)
    {
        ivp_core_core_plausible_check(core);
    }
}

/* IVP_Core::core_plausible_check (ivp_core.cxx:1393): asserts only. */
#define IVP_CORE_MAX_PLAUSIBLE_LEN 100000.0f /* MAX_PLAUSIBLE_LEN */

void ivp_core_core_plausible_check(ivp_core_t *core)
{
    double rot_change_len, trans_change_len;
    rot_change_len = core_float_point_real_length(&core->rot_speed_change);
    trans_change_len = core_float_point_real_length(&core->speed_change);
    IVP_ASSERT(rot_change_len < (double)IVP_CORE_MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(trans_change_len < (double)IVP_CORE_MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(rot_change_len > (double)-IVP_CORE_MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(trans_change_len > (double)-IVP_CORE_MAX_PLAUSIBLE_LEN);

    double rot_len, trans_len;
    rot_len = (double)lvf_real_length(&core->rot_speed);
    trans_len = core_float_point_real_length(&core->speed);
    IVP_ASSERT(rot_len < (double)IVP_CORE_MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(trans_len < (double)IVP_CORE_MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(rot_len > (double)-IVP_CORE_MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(trans_len > (double)-IVP_CORE_MAX_PLAUSIBLE_LEN);
    IVP_USE(rot_change_len);
    IVP_USE(trans_change_len);
    IVP_USE(rot_len);
    IVP_USE(trans_len);
}

void ivp_core_abort_all_async_pushes(ivp_core_t *core)
{
    ivp_vec3r_set_zero(&core->rot_speed_change);
    ivp_vec3r_set_zero(&core->speed_change);
}

/* IVP_Core::damp_object */
static void core_damp_object(ivp_core_t *core, double delta_time_, const ivp_vec3f_t *rotation_factor, double speed_factor)
{
    ivp_vec3f_t rot;
    lvf_set_multiple(&rot, rotation_factor, (D)delta_time_);
    double qlen = (double)lvf_quad_length(&rot);
    ivp_vec3f_t r_factor;
    if (qlen < 0.5) {
        lvf_set3(&r_factor, 1.0f - rot.k[0], 1.0f - rot.k[1], 1.0f - rot.k[2]);
    } else {
        lvf_setd3(&r_factor, (D)exp(-(double)rot.k[0]), (D)exp(-(double)rot.k[1]), (D)exp(-(double)rot.k[2]));
    }
    double sf = speed_factor * delta_time_;
    if (sf < 0.25) {
        speed_factor = 1.0 - sf;
    } else {
        speed_factor = exp(-sf);
    }
    lvf_set_pairwise_mult(&core->rot_speed, &core->rot_speed, &r_factor);
    ivp_vec3f_t sp = ivp_core_speed_f(core);
    lvf_mult(&sp, (D)speed_factor);
    ivp_core_set_speed_f(core, &sp);
}

/* IVP_Core::global_damp_core */
void ivp_core_global_damp_core(ivp_core_t *core, double d_time)
{
    ivp_vec3f_t rdf = vr_to_f(&core->rot_speed_damp_factor);
    if (core->movement_state >= IVP_MT_SLOW) {
        float s_add_on = 0.1f;
        ivp_vec3f_t add_on, rot_damp_factor;
        lvf_set3(&add_on, s_add_on, s_add_on, s_add_on);
        lvf_add2(&rot_damp_factor, &rdf, &add_on);
        core_damp_object(core, d_time, &rot_damp_factor, (double)((float)core->speed_damp_factor + s_add_on));
    } else {
        core_damp_object(core, d_time, &rdf, (double)(float)core->speed_damp_factor);
    }
}

void ivp_core_reset_freeze_check_values(ivp_core_t *core)
{
    ivp_time_t t = ivp_env_get_current_time(ivp_core_env(core));
    core->time_of_calm_reference[0] = t;
    core->time_of_calm_reference[1] = t;
}

/* IVP_Core::clip_velocity (ivp_core_macros.hxx) */
void ivp_core_clip_velocity(ivp_core_t *core, ivp_vec3f_t *velocity, ivp_vec3f_t *rotation_vec)
{
    struct ivp_environment *env = ivp_core_env(core);
    if (core->spin_clipping_enabled) {
        const ivp_vec3f_t *sc = &core->spin_clipping;
        for (int i = 0; i < 3; i++) {
            if (rotation_vec->k[i] > sc->k[i]) {
                rotation_vec->k[i] = sc->k[i];
            } else if (rotation_vec->k[i] < -sc->k[i]) {
                rotation_vec->k[i] = -sc->k[i];
            }
        }
    } else {
        if ((!core->car_wheel) || (core->max_surface_deviation != IVP_REAL(0.0))) {
            double square_rot_speed = (double)lvf_quad_length(rotation_vec);
            /* IVP_FLOAT * IVP_FLOAT */
            double max_rot_speed = (double)(ivp_env_anomaly_get_max_angular_velocity_per_psi(env) *
                                            ivp_env_get_inv_delta_psi_time(env));
            double quad_max_rot_speed = max_rot_speed * max_rot_speed;
            if (square_rot_speed > quad_max_rot_speed) {
                ivp_env_anomaly_max_angular_velocity_exceeded(env, core, rotation_vec);
            }
        }
    }
    double max_speed = (double)ivp_env_anomaly_get_max_velocity(env);
    double quad_max_speed = max_speed * max_speed;
    double square_speed = (double)lvf_quad_length(velocity);
    if (square_speed > quad_max_speed) {
        ivp_env_anomaly_max_velocity_exceeded(env, core, velocity);
    }
}

/* IVP_Core::synchronize_with_rot_z */
void ivp_core_synchronize_with_rot_z(ivp_core_t *core)
{
    struct ivp_environment *env = ivp_core_env(core);
    IVP_ASSERT(core->old_sync_rot == NULL);
    ivp_old_sync_rot_t *info = (ivp_old_sync_rot_t *)ivp_env_sim_mem_alloc(env, sizeof(ivp_old_sync_rot_t));
    core->old_sync_rot = info;
    info->was_pushed_during_i_s = false;
    ivp_time_t current_time = ivp_env_get_current_time(env);
    IVP_IF(1)
    {
        ivp_debug_manager_t *dm = ivp_env_get_debug_manager(env);
        if (dm->file_out_impacts) {
            fprintf(dm->out_deb_file, "doing_synchronize %x at %f\n", (unsigned int)(0x0000ffffu & (uintptr_t)core),
                    current_time);
        }
    }
    info->old_sync_rot_speed = core->rot_speed;
    info->old_sync_q_world_f_core_next_psi = core->q_world_f_core_next_psi;
    ivp_quatr_t q_core_f_core;
    ivp_quat_set_invert_mult(&q_core_f_core, &core->q_world_f_core_last_psi, &core->q_world_f_core_next_psi);
    ivp_core_inline_calc_at_quaternion(core, current_time, &core->q_world_f_core_next_psi);
    ivp_quat_to_mat3(&core->q_world_f_core_next_psi, &core->m_world_f_core_last_psi.rot);
    ivp_core_inline_calc_at_position(core, current_time, &core->m_world_f_core_last_psi.pos);
    if (!core->rot_inertias_are_equal) {   /* IVP_FAST_WHEELS_ENABLED */
        double f = (double)(2.0f * core->i_delta_time);
        lvf_setd3(&core->rot_speed, (D)(f * asin((double)q_core_f_core.x)), (D)(f * asin((double)q_core_f_core.y)),
                  (D)(f * asin((double)q_core_f_core.z)));
    }
}

/* IVP_Core::undo_synchronize_rot_z */
void ivp_core_undo_synchronize_rot_z(ivp_core_t *core)
{
    IVP_IF(1)
    {
        struct ivp_environment *env = ivp_core_env(core);
        ivp_debug_manager_t *dm = ivp_env_get_debug_manager(env);
        if (dm->file_out_impacts) {
            fprintf(dm->out_deb_file, "undoing_synchro %x at %f\n", (unsigned int)(0x0000ffffu & (uintptr_t)core),
                    ivp_env_get_current_time(env));
        }
    }
    core->rot_speed = core->old_sync_rot->old_sync_rot_speed;
    core->q_world_f_core_next_psi = core->old_sync_rot->old_sync_q_world_f_core_next_psi;
    core->old_sync_rot = NULL;
}

/* IVP_Core::update_exact_mindist_events_of_core */
void ivp_core_update_exact_mindist_events_of_core(ivp_core_t *core)
{
    struct ivp_environment *env = ivp_core_env(core);
    core->mindist_event_already_done = ivp_env_get_mindist_event_timestamp_reference(env);
    for (int c = ivp_core_n_objects(core) - 1; c >= 0; c--) {
        ivp_object_update_exact_mindist_events_of_object(ivp_core_object_at(core, c));
    }
}

/* IVP_Core::calc_movement_state */
ivp_movement_type_t ivp_core_calc_movement_state(ivp_core_t *core, ivp_time_t psi_time)
{
    struct ivp_environment *env = ivp_core_env(core);
    /* IVP_MINIMAL_*_SPEED_PER_PSI_* are float literals: their squares are float products */
    const double ts2 = (double)(0.01f * 0.01f), rs2 = (double)(0.005f * 0.005f);
    const double tl2 = (double)(0.1f * 0.1f), rl2 = (double)(0.2f * 0.2f), long_time = 4.0f;
    for (;;) {
        ivp_vec3f_t ref0 = core->position_world_f_core_calm_reference[0];
        double qdist = (double)lvp_quad_distance_f(&core->pos_world_f_core_last_psi, &ref0);
        if (qdist > ts2) break;
        double qrot_sum = (double)ivp_quat_estimate_qdiff(&core->q_world_f_core_next_psi, &core->q_world_f_core_calm_reference[0]);
        double ulr = (double)(float)core->upper_limit_radius;
        if (qrot_sum * ulr * ulr > rs2) break;
        if (ivp_time_diff(psi_time, core->time_of_calm_reference[0]) > ivp_env_get_freeze_check_dtime(env)) {
            return IVP_MT_CALM;
        }
        return IVP_MT_SLOW;
    }
    ivp_quatf_set(&core->q_world_f_core_calm_reference[0], &core->q_world_f_core_next_psi);
    lvf_set_p(&core->position_world_f_core_calm_reference[0], &core->pos_world_f_core_last_psi);
    core->time_of_calm_reference[0] = psi_time;
    for (;;) {
        ivp_vec3f_t ref1 = core->position_world_f_core_calm_reference[1];
        double q_dist = (double)lvp_quad_distance_f(&core->pos_world_f_core_last_psi, &ref1);
        if (q_dist > tl2) break;
        double qrot_sum = (double)ivp_quat_estimate_qdiff(&core->q_world_f_core_last_psi, &core->q_world_f_core_calm_reference[1]);
        double ulr = (double)(float)core->upper_limit_radius;
        if (qrot_sum * ulr * ulr > rl2) break;
        if (ivp_time_diff(psi_time, core->time_of_calm_reference[1]) > long_time) {
            return IVP_MT_CALM;
        }
        return IVP_MT_MOVING;
    }
    ivp_quatf_set(&core->q_world_f_core_calm_reference[1], &core->q_world_f_core_last_psi);
    lvf_set_p(&core->position_world_f_core_calm_reference[1], &core->pos_world_f_core_last_psi);
    core->time_of_calm_reference[1] = psi_time;
    return IVP_MT_MOVING;
}

/* IVP_Core::calc_next_PSI_matrix_zero_speed */
void ivp_core_calc_next_psi_matrix_zero_speed(ivp_core_t *core, ivp_sim_es_t *es)
{
    core->i_delta_time = (float)es->i_delta_time;
    core->q_world_f_core_next_psi = core->q_world_f_core_last_psi;
    core->abs_omega = 0.0f;
    ivp_vec3r_set_zero(&core->speed);
    ivp_vec3r_set_zero(&core->delta_world_f_core_psis);
    core->current_speed = 0.0f;
    core->max_surface_rot_speed = 0.0f;
    lvf_set3(&core->rotation_axis_world_space, 1.0f, 0.0f, 0.0f);
}

/* IVP_Core::stop_movement_without_collision_recheck */
void ivp_core_stop_movement_without_collision_recheck(ivp_core_t *core)
{
    core->movement_state = IVP_MT_NOT_SIM;
    ivp_vec3r_set_zero(&core->speed);
    lvf_zero(&core->rot_speed);
    ivp_vec3r_set_zero(&core->speed_change);
    ivp_vec3r_set_zero(&core->rot_speed_change);
    ivp_vec3r_set_zero(&core->delta_world_f_core_psis);
    ivp_sim_es_t es;
    ivp_sim_es_init(&es, ivp_core_env(core));
    ivp_core_calc_next_psi_matrix_zero_speed(core, &es);
}

/* IVP_Core::stop_physical_movement */
void ivp_core_stop_physical_movement(ivp_core_t *core)
{
    ivp_core_stop_movement_without_collision_recheck(core);
    struct ivp_environment *env = ivp_core_env(core);
    ivp_time_t time = ivp_env_get_current_time(env);
    for (int c = ivp_core_n_objects(core) - 1; c >= 0; c--) {
        ivp_object_t *r_obj = ivp_core_object_at(core, c);
        IVP_ASSERT(r_obj->object_movement_state != IVP_MT_STATIC);
        ivp_object_set_movement_state(r_obj, IVP_MT_NOT_SIM);
        ivp_mindist_mgr_recheck_ov_element(ivp_env_get_mindist_mgr(env), r_obj);
        if (r_obj->hull_manager) ivp_hull_mgr_exit_hull_for_simulation(r_obj->hull_manager, time);
        if (r_obj->cache_object) ivp_object_invalid_cache_object(r_obj);
    }
}

/* IVP_Core::init_core_for_simulation */
void ivp_core_init_core_for_simulation(ivp_core_t *core)
{
    IVP_ASSERT(core->movement_state == IVP_MT_NOT_SIM);
    core->movement_state = IVP_MT_MOVING;
    struct ivp_environment *env = ivp_core_env(core);
    ivp_time_t time = ivp_env_get_current_time(env);
    core->time_of_last_psi = time - (double)ivp_env_get_delta_psi_time(env);
    ivp_core_reset_freeze_check_values(core);
    core->time_of_last_psi = time;
    for (int c = ivp_core_n_objects(core) - 1; c >= 0; c--) {
        ivp_object_t *r_obj = ivp_core_object_at(core, c);
        IVP_ASSERT(r_obj->object_movement_state != IVP_MT_STATIC);
        ivp_object_set_movement_state(r_obj, IVP_MT_MOVING);
        /* IVP_Hull_Manager::init_hull_for_simulation is empty in the reference */
        ivp_object_get_all_near_mindists(r_obj);
    }
}

/* IVP_Core::revive_simulation_core (ivp_physic_private.cxx) */
bool ivp_core_revive_simulation_core(ivp_core_t *core)
{
    IVP_ASSERT(!core->physical_unmoveable);
    IVP_ASSERT(core->movement_state == IVP_MT_NOT_SIM);
    struct ivp_environment *env = ivp_core_env(core);
    for (int c = ivp_core_n_objects(core) - 1; c >= 0; c--) {
        ivp_object_t *r_obj = ivp_core_object_at(core, c);
        IVP_IF(1)
        {
            IVP_ASSERT(r_obj->object_movement_state >= IVP_MT_NOT_SIM);
        }
        ivp_object_set_movement_state(r_obj, IVP_MT_MOVING);
    }
    ivp_core_init_core_for_simulation(core);
    ivp_sim_es_t es;
    ivp_sim_es_init_dt(&es, env, ivp_time_diff(ivp_env_get_next_psi_time(env), ivp_env_get_current_time(env)));
    ivp_core_calc_next_psi_matrix_zero_speed(core, &es);

    /* @@@ hack, go back to last PSI at wakeup at PSI ( satisfy init_PSI )
     * (debug builds only: changes the simulation, ivp_physic_private.cxx:494) */
    IVP_IF(ivp_env_get_state(env) == IVP_ES_PSI)
    {
        double d_time = (double)ivp_env_get_delta_psi_time(env); /* IVP_FLOAT get_delta_PSI_time() */
        core->time_of_last_psi += -d_time;
    }

    IVP_IF(IVP_DEBUG_OBJECT0)
    {
        const char *search0 = IVP_DEBUG_OBJECT0;
        const char *name0 = ivp_core_object_at(core, 0)->name;
        if (!ivp_string_cmp(name0, search0, false)) {
            if (ivp_env_get_current_time(env) >= IVP_DEBUG_TIME) {
                ivp_debug_printf("revive object %s time:%f\n", name0, ivp_env_get_current_time(env));
            }
        }
    }
    IVP_IF(IVP_DEBUG_OBJECT1)
    {
        const char *search0 = IVP_DEBUG_OBJECT1;
        const char *name0 = ivp_core_object_at(core, 0)->name;
        if (!ivp_string_cmp(name0, search0, false)) {
            if (ivp_env_get_current_time(env) >= IVP_DEBUG_TIME) {
                ivp_debug_printf("revive object %s time:%f\n", name0, ivp_env_get_current_time(env));
            }
        }
    }

    bool fs_was_grown = ivp_core_grow_friction_system(core);
    for (int d = ivp_core_n_objects(core) - 1; d >= 0; d--) {
        ivp_env_fire_object_revived(env, ivp_core_object_at(core, d));
    }
    return fs_was_grown;
}

/* IVP_Core::fire_event_object_frozen */
void ivp_core_fire_event_object_frozen(ivp_core_t *core)
{
    for (int i = ivp_core_n_objects(core) - 1; i >= 0; i--) {
        ivp_env_fire_object_frozen(ivp_core_env(core), ivp_core_object_at(core, i));
    }
}

/* IVP_Core::freeze_simulation_core */
void ivp_core_freeze_simulation_core(ivp_core_t *core)
{
    ivp_core_stop_physical_movement(core); /* because of Interpolations */

    /* deactivate all mindists */
    IVP_IF(IVP_DEBUG_OBJECT0)
    {
        const char *search0 = IVP_DEBUG_OBJECT0;
        const char *name0 = ivp_core_object_at(core, 0)->name;
        if (!ivp_string_cmp(name0, search0, false)) {
            if (ivp_env_get_current_time(ivp_core_env(core)) >= IVP_DEBUG_TIME) {
                ivp_debug_printf("freeze object %s time:%f\n", name0, ivp_env_get_current_time(ivp_core_env(core)));
            }
        }
    }
    IVP_IF(IVP_DEBUG_OBJECT1)
    {
        const char *search0 = IVP_DEBUG_OBJECT1;
        const char *name0 = ivp_core_object_at(core, 0)->name;
        if (!ivp_string_cmp(name0, search0, false)) {
            if (ivp_env_get_current_time(ivp_core_env(core)) >= IVP_DEBUG_TIME) {
                ivp_debug_printf("freeze object %s time:%f\n", name0, ivp_env_get_current_time(ivp_core_env(core)));
            }
        }
    }

    ivp_core_fire_event_object_frozen(core);
}

/* IVP_Core::debug_vec_movement_state (ivp_physic_private.cxx:613) */
void ivp_core_debug_vec_movement_state(ivp_core_t *core)
{
    ivp_core_t *my_core = core;
    ivp_core_t *one_object = core;
    IVP_IF(0)
    {
        char *out_text;
        float ivp_pointer[3];
        double ivp_start[3];
        int v_color;
        /* ivp_start.set(my_core->get_position_PSI()) */
        ivp_start[0] = (double)my_core->m_world_f_core_last_psi.pos.k[0];
        ivp_start[1] = (double)my_core->m_world_f_core_last_psi.pos.k[1];
        ivp_start[2] = (double)my_core->m_world_f_core_last_psi.pos.k[2];
        ivp_pointer[0] = 0.0f;
        ivp_pointer[1] = -7.0f;
        ivp_pointer[2] = 0.0f;
        out_text = ivp_string_make(NULL, "oob%u_sp%.3f", (unsigned int)((uintptr_t)one_object & 0x0000ffffU),
                                   core_float_point_real_length(&one_object->speed));
        if (my_core->movement_state == IVP_MT_CALM) {
            v_color = 3;
        } else {
            if (my_core->movement_state == IVP_MT_SLOW) {
                v_color = 6;
            } else {
                v_color = 1;
            }
        }
        ivp_env_add_draw_vector(ivp_core_env(core), ivp_start, ivp_pointer, out_text, v_color);
        ivp_string_free(NULL, out_text);
    }
}

/* IVP_Core::ensure_core_to_be_in_simulation */
void ivp_core_ensure_core_to_be_in_simulation(ivp_core_t *core)
{
    if (core->physical_unmoveable) return;
    if (core->movement_state == IVP_MT_NOT_SIM) {
        ivp_sim_unit_revive_for_simulation(core->sim_unit, ivp_core_env(core));
    } else {
        ivp_sim_unit_ensure_cores_movement(core->sim_unit);
    }
}

void ivp_core_ensure_core_in_simulation_delayed(ivp_core_t *core)
{
    ivp_env_add_revive_core(ivp_core_env(core), core);
}

void ivp_core_ensure_all_core_objs_in_simulation(ivp_core_t *core)
{
    IVP_ASSERT(!core->physical_unmoveable);
    for (int c = ivp_core_n_objects(core) - 1; c >= 0; c--) {
        ivp_object_ensure_in_simulation(ivp_core_object_at(core, c));
    }
}

/* IVP_Core::ensure_all_core_objs_in_simulation_now */
void ivp_core_ensure_all_core_objs_in_simulation_now(ivp_core_t *core)
{
    IVP_ASSERT(!core->physical_unmoveable);
    for (int c = ivp_core_n_objects(core) - 1; c >= 0; c--) {
        ivp_object_ensure_in_simulation_now(ivp_core_object_at(core, c));
    }
}

/* IVP_Core::revive_adjacent_to_unmoveable */
void ivp_core_revive_adjacent_to_unmoveable(ivp_core_t *core)
{
    IVP_ASSERT(core->physical_unmoveable);
    struct ivp_environment *env = ivp_core_env(core);
    ivp_env_sim_mem_begin(env);
    for (int d = ivp_core_n_objects(core) - 1; d >= 0; d--) {
        ivp_object_t *r_obj = ivp_core_object_at(core, d);
        ivp_fr_synapse_t *fr_synapse = r_obj->fr_synapses;
        while (fr_synapse) {
            ivp_fr_synapse_t *next_syn = fr_synapse->next;
            ivp_contact_pt_t *fr_mindist = fr_synapse->contact_point;
            ivp_fr_system_t *fs = fr_mindist->l_friction_system;
            fs->union_find_necessary = true;
            ivp_core_t *other_core = fr_mindist->synapse[0].l_obj->physical_core;   /* get_core() */
            if (other_core == core) other_core = fr_mindist->synapse[1].l_obj->physical_core;
            if (other_core->physical_unmoveable) {
                ivp_contact_pt_destroy(fr_mindist);
            } else {
                ivp_core_ensure_all_core_objs_in_simulation(other_core);
                ivp_contact_pt_recalc_friction_s_vals(fr_mindist);
                ivp_impact_solver_lt_t info;
                memset(&info, 0, sizeof(info));
                ivp_contact_pt_read_materials_for_contact_situation(fr_mindist, &info);
                ivp_contact_pt_calc_virtual_mass_of_mindist(fr_mindist);
            }
            fr_synapse = next_syn;
        }
    }
    ivp_env_sim_mem_end(env);
}

/* IVP_Core::values_changed_recalc_redundants */
void ivp_core_values_changed_recalc_redundants(ivp_core_t *core)
{
    struct ivp_environment *env = ivp_core_env(core);
    ivp_env_sim_mem_begin(env);
    ivp_core_calc_calc(core);
    ivp_env_sim_mem_end(env);
    ivp_movement_type_t mt_for_objs;
    if (core->physical_unmoveable) {
        mt_for_objs = IVP_MT_STATIC;
        core->movement_state = IVP_MT_NOT_SIM;
    } else {
        if (core->movement_state > IVP_MT_NOT_SIM) core->movement_state = IVP_MT_NOT_SIM;
        mt_for_objs = core->movement_state;
    }
    for (int d = ivp_core_n_objects(core) - 1; d >= 0; d--) {
        ivp_object_set_movement_state(ivp_core_object_at(core, d), mt_for_objs);
    }
    if (!core->physical_unmoveable) {
        ivp_core_ensure_core_to_be_in_simulation(core);
        ivp_env_sim_mem_begin(env);
        for (int d = ivp_core_n_objects(core) - 1; d >= 0; d--) {
            ivp_object_t *r_obj = ivp_core_object_at(core, d);
            ivp_fr_synapse_t *fr_synapse = r_obj->fr_synapses;
            ivp_fr_system_t *fs = NULL;
            if (fr_synapse) fs = fr_synapse->contact_point->l_friction_system;
            for (; fr_synapse; fr_synapse = fr_synapse->next) {
                ivp_contact_pt_t *fr_mindist = fr_synapse->contact_point;
                ivp_fr_system_t *next_fs = fr_mindist->l_friction_system;
                if (next_fs != fs) ivp_fr_system_fusion_friction_systems(fs, next_fs);
                ivp_impact_solver_lt_t info;
                memset(&info, 0, sizeof(info));
                ivp_contact_pt_recalc_friction_s_vals(fr_mindist);
                ivp_contact_pt_read_materials_for_contact_situation(fr_mindist, &info);
                ivp_contact_pt_calc_virtual_mass_of_mindist(fr_mindist);
            }
        }
        ivp_env_sim_mem_end(env);
    } else {
        ivp_core_stop_movement_without_collision_recheck(core);
        ivp_core_revive_adjacent_to_unmoveable(core);
    }
}

void ivp_core_reset_time(ivp_core_t *core, double offset)
{
    core->time_of_last_psi -= offset;
    for (int i = ivp_core_n_objects(core) - 1; i >= 0; i--) {
        ivp_object_reset_time(ivp_core_object_at(core, i), offset);
    }
}

/* ── controllers of core (ivp_sim_unit.cxx) ────────────────────────────── */

bool ivp_core_add_core_controller(ivp_core_t *core, ivp_sim_ctrl_t *c)
{
    if (!core || !core->sim_unit || !c) {
        IVP_ASSERT(0);
        return false;
    }
    int index = ivp_pvec_add(&core->controllers_of_core, c);
    if (index < 0) return false;
    if (!ivp_sim_unit_add_controller_of_core(core->sim_unit, core, c)) {
        ivp_pvec_remove_at(&core->controllers_of_core, index);
        return false;
    }
    return true;
}

void ivp_core_rem_core_controller(ivp_core_t *core, ivp_sim_ctrl_t *c)
{
    ivp_pvec_remove(&core->controllers_of_core, c);
    if (core->sim_unit) {
        ivp_sim_unit_remove_controller_of_core(core->sim_unit, core, c);
    } else {
        IVP_ASSERT(0);
    }
}

/* ── IVP_Calc_Next_PSI_Solver ──────────────────────────────────────────── */

static void cnps_calc_rotation_matrix(ivp_core_t *pc, float delta_sim_time, ivp_quatr_t *q_core_f_core)
{
    struct ivp_environment *env = ivp_core_env(pc);
    if (pc->rot_inertias_are_equal || ivp_env_get_state(env) == IVP_ES_AT) {
        if (pc->car_wheel && pc->max_surface_deviation == IVP_REAL(0.0)) {   /* IVP_FAST_WHEELS_ENABLED */
            ivp_car_wheel_calc_fast_wheel_rotation(pc, delta_sim_time, q_core_f_core);
        } else {
            ivp_quat_set_fast_multiple_with_clip(q_core_f_core, &pc->rot_speed, (ivp_real_t)delta_sim_time);
        }
        return;
    }
    const double max_angle = (double)(5.0f * 6.0f / 360.0f);
    const double qmax_angle = max_angle * max_angle;
    double square_rot_speed = (double)lvf_quad_length(&pc->rot_speed);
    double dt = (double)delta_sim_time;
    double qangle = square_rot_speed * dt * dt;
    int steps = 1;
    if (qangle > 4.0 * qmax_angle) {
        steps = 1 + (int)sqrt(qangle / qmax_angle);
        dt /= (double)(float)steps;
    }
    ivp_quat_set_very_fast_multiple(q_core_f_core, &pc->rot_speed, (ivp_real_t)dt);
    ivp_vec3f_t ri = ivp_core_rot_inertia_f(pc), iri = ivp_core_inv_rot_inertia_f(pc);
    /* IVP_U_Point diff_other_rot_inertia_div_this((ri1 - ri2) * iri0, ...): float products */
    double d0 = (double)((ri.k[1] - ri.k[2]) * iri.k[0]);
    double d1 = (double)((ri.k[2] - ri.k[0]) * iri.k[1]);
    double d2 = (double)((ri.k[0] - ri.k[1]) * iri.k[2]);
    for (int i = 1;; i++) {
        ivp_vec3f_t hp;
        lvf_setd3(&hp, (D)((double)(pc->rot_speed.k[1] * pc->rot_speed.k[2]) * d0),
                  (D)((double)(pc->rot_speed.k[2] * pc->rot_speed.k[0]) * d1),
                  (D)((double)(pc->rot_speed.k[0] * pc->rot_speed.k[1]) * d2));
        lvf_add_multiple(&pc->rot_speed, &hp, (D)dt);
        if (i >= steps) break;
        ivp_quatr_t rot_matrix;
        ivp_quat_set_very_fast_multiple(&rot_matrix, &pc->rot_speed, (ivp_real_t)dt);
        ivp_quat_mult(q_core_f_core, &rot_matrix, q_core_f_core);
    }
}

static void cnps_calc_psi_rotation_axis(ivp_core_t *core, const ivp_quatr_t *q_core_f_core)
{
    double qlen = (double)q_core_f_core->x * (double)q_core_f_core->x + (double)q_core_f_core->y * (double)q_core_f_core->y +
                  (double)q_core_f_core->z * (double)q_core_f_core->z;
    if (qlen > P_DOUBLE_EPS) {
        double ilen = ivp_isqrt_double(qlen, 3);
        float abs_angle = lv_upper_limit_asin((float)(qlen * ilen));
        IVP_IF(1)
        {
            /* reference: "#if defined(DEBUG) && 0" (never compiled):
             * ref_angle = asin(sqrt(qlen)) - 1E9f; IVP_ASSERT(abs_angle >= ref_angle); */
        }
        core->abs_omega = 2.0f * abs_angle * core->i_delta_time;
        ivp_vec3r_t rot_axis_ws, core_f_core_ws;
        lvp_set3(&core_f_core_ws, q_core_f_core->x, q_core_f_core->y, q_core_f_core->z);
        lvm_vmult3(&core->m_world_f_core_last_psi, &core_f_core_ws, &rot_axis_ws);
        lvf_set_multiple_p(&core->rotation_axis_world_space, &rot_axis_ws, (D)ilen);
    } else {
        core->abs_omega = 0.0f;
        lvf_set3(&core->rotation_axis_world_space, 1.0f, 0.0f, 0.0f);
    }
    core->max_surface_rot_speed = (float)core->max_surface_deviation * core->abs_omega;
}

static bool cnps_has_identity_rotation(const ivp_core_t *core)
{
    return core->rot_speed.k[0] == 0.0f && !signbit(core->rot_speed.k[0]) &&
           core->rot_speed.k[1] == 0.0f && !signbit(core->rot_speed.k[1]) &&
           core->rot_speed.k[2] == 0.0f && !signbit(core->rot_speed.k[2]) &&
           core->q_world_f_core_next_psi.x == IVP_REAL(0.0) &&
           !signbit(core->q_world_f_core_next_psi.x) &&
           core->q_world_f_core_next_psi.y == IVP_REAL(0.0) &&
           !signbit(core->q_world_f_core_next_psi.y) &&
           core->q_world_f_core_next_psi.z == IVP_REAL(0.0) &&
           !signbit(core->q_world_f_core_next_psi.z) &&
           core->q_world_f_core_next_psi.w == IVP_REAL(1.0);
}

void ivp_cnps_calc_next_psi_matrix(ivp_core_t *core, ivp_sim_es_t *event_sim, ivp_pvec_t *active_hull_managers_out)
{
    struct ivp_environment *env = ivp_core_env(core);
    IVP_IF(1)
    {
        ivp_debug_manager_t *dm = ivp_env_get_debug_manager(event_sim->environment);
        if (dm->file_out_impacts) {
            /* reference prints the IVP_Calc_Next_PSI_Solver (this); libivp has no solver object: the core */
            fprintf(dm->out_deb_file, "making_calc_next_psi %x at %f\n", (unsigned int)(0x0000ffffu & (uintptr_t)core),
                    ivp_env_get_current_time(env));
        }
    }
    if (!(core->car_wheel && core->max_surface_deviation == IVP_REAL(0.0))) {
        ivp_vec3f_t sp = ivp_core_speed_f(core);
        ivp_core_clip_velocity(core, &sp, &core->rot_speed);
        ivp_core_set_speed_f(core, &sp);
    }
    {
        ivp_vec3f_t sp = ivp_core_speed_f(core);
        core->current_speed = (float)lvf_real_length(&sp);
    }
    float simulation_delta_time = (float)event_sim->delta_time;
    core->i_delta_time = (float)event_sim->i_delta_time;
    ivp_quatr_t q_core_f_core;
    const bool identity_rotation = cnps_has_identity_rotation(core);
    if (identity_rotation) {
        q_core_f_core = ivp_quatr_identity();
    } else {
        cnps_calc_rotation_matrix(core, simulation_delta_time, &q_core_f_core);
    }
    {
        ivp_time_t now = ivp_env_get_current_time(env);
        double d_last_time = ivp_time_diff(now, core->time_of_last_psi);
        core->time_of_last_psi = now;
        lvp_add_multiple(&core->pos_world_f_core_last_psi, &core->delta_world_f_core_psis, (D)d_last_time);
        ivp_vec3f_t sp = ivp_core_speed_f(core);
        lvp_set_f(&core->delta_world_f_core_psis, &sp);
        core->q_world_f_core_last_psi = core->q_world_f_core_next_psi;
        if (!identity_rotation) {
            ivp_quat_mult(&core->q_world_f_core_next_psi,
                          &core->q_world_f_core_next_psi, &q_core_f_core);
            ivp_quat_fast_normalize(&core->q_world_f_core_next_psi);
        }
    }
    if (identity_rotation) {
        core->abs_omega = 0.0f;
        lvf_set3(&core->rotation_axis_world_space, 1.0f, 0.0f, 0.0f);
        core->max_surface_rot_speed = 0.0f;
    } else {
        cnps_calc_psi_rotation_axis(core, &q_core_f_core);
    }
    double new_speed = (double)core->max_surface_rot_speed + (double)core->current_speed;
    for (int c = ivp_core_n_objects(core) - 1; c >= 0; c--) {
        ivp_object_t *r_obj = ivp_core_object_at(core, c);
        ivp_hull_mgr_t *h_manager = r_obj->hull_manager;
        if (!h_manager) continue;
        ivp_hull_mgr_increase_hull_by_x(h_manager, ivp_env_get_current_time(env), (float)event_sim->delta_time,
                                        (float)new_speed, core->current_speed);
        if (ivp_hull_mgr_are_events_in_hull(h_manager)) {
            /* the reference adds unconditionally (IVP_Core_Merged passes NULL and never has events) */
            if (active_hull_managers_out) ivp_pvec_add(active_hull_managers_out, h_manager);
        }
    }
}

void ivp_cnps_set_transformation(ivp_core_t *core, const ivp_quatr_t *rotation, const ivp_vec3r_t *position,
                                 bool optimize_for_repeated_calls)
{
    struct ivp_environment *env = ivp_core_env(core);
    ivp_time_t current_time = ivp_env_get_current_time(env);
    ivp_environment_set_current_time_internal(env, current_time);
    float simulation_delta_time = (float)ivp_time_diff(current_time, core->time_of_last_psi);
    if (simulation_delta_time >= ivp_env_get_delta_psi_time(env)) {
        core->i_delta_time = 1.0f / simulation_delta_time;
    } else {
        simulation_delta_time = ivp_env_get_delta_psi_time(env);
        core->i_delta_time = ivp_env_get_inv_delta_psi_time(env);
    }
    core->time_of_last_psi = current_time;
    core->q_world_f_core_next_psi = *rotation;
    ivp_quatr_t q_core_f_core;
    ivp_quat_set_invert_mult(&q_core_f_core, &core->q_world_f_core_last_psi, &core->q_world_f_core_next_psi);
    core->q_world_f_core_last_psi = *rotation;
    core->time_of_last_psi = ivp_env_get_old_time_of_last_psi(env);
    ivp_vec3r_t moved_distance;
    lvp_subtract2(&moved_distance, position, &core->pos_world_f_core_last_psi);
    float moved_distance_val = (float)lvp_real_length(&moved_distance);
    core->pos_world_f_core_last_psi = *position;
    ivp_vec3r_set_zero(&core->delta_world_f_core_psis);
    ivp_vec3r_set_zero(&core->speed);
    core->m_world_f_core_last_psi.pos = *position;
    ivp_quat_to_mat3(rotation, &core->m_world_f_core_last_psi.rot);
    if (optimize_for_repeated_calls) {
        core->current_speed = moved_distance_val * core->i_delta_time;
        cnps_calc_psi_rotation_axis(core, &q_core_f_core);
    } else {
        core->current_speed = 0.0f;
        q_core_f_core = ivp_quatr_identity();
        cnps_calc_psi_rotation_axis(core, &q_core_f_core);
    }
    /* IVP_FLOAT expression */
    float diff = core->max_surface_rot_speed * simulation_delta_time + moved_distance_val;
    for (int c = ivp_core_n_objects(core) - 1; c >= 0; c--) {
        ivp_object_t *r_obj = ivp_core_object_at(core, c);
        ivp_hull_mgr_t *h_manager = r_obj->hull_manager;
        if (h_manager) ivp_hull_mgr_jump_add_hull(h_manager, diff, moved_distance_val);
        if (!IVP_MTIS_SIMULATED(r_obj->object_movement_state)) {
            ivp_object_invalid_cache_object(r_obj);
            ivp_object_recalc_exact_mindists_of_object(r_obj);
            ivp_object_recalc_invalid_mindists_of_object(r_obj);
        }
        if (!optimize_for_repeated_calls) {
            ivp_mindist_mgr_recheck_ov_element(ivp_env_get_mindist_mgr(env), r_obj);
        }
        if (h_manager) {
            ivp_hull_mgr_check_hull_synapses(h_manager, env);
            ivp_hull_mgr_check_for_reset(h_manager);
        }
    }
    core->current_speed = 0.0f;
    core->abs_omega = 0.0f;
    core->max_surface_rot_speed = 0.0f;
}

void ivp_cnps_commit_all_calc_next_psi_matrix(struct ivp_environment *env, ivp_pvec_t *cores, ivp_pvec_t *active_hulls_out)
{
    ivp_sim_es_t es;
    ivp_sim_es_init_dt(&es, env, (double)ivp_env_get_delta_psi_time(env));
    for (int i = ivp_pvec_len(cores) - 1; i >= 0; i--) {
        ivp_cnps_calc_next_psi_matrix((ivp_core_t *)ivp_pvec_at(cores, i), &es, active_hulls_out);
    }
}

void ivp_cnps_commit_all_hull_managers(struct ivp_environment *env, ivp_pvec_t *active_hull_managers)
{
    for (int i = ivp_pvec_len(active_hull_managers) - 1; i >= 0; i--) {
        ivp_hull_mgr_t *hm = (ivp_hull_mgr_t *)ivp_pvec_at(active_hull_managers, i);
        ivp_hull_mgr_check_hull_synapses(hm, env);
        ivp_hull_mgr_check_for_reset(hm);
    }
}

/* IVP_Core::apply_velocity_limit (public helper; the simulation clips in
 * clip_velocity) */
void ivp_core_apply_velocity_limit(ivp_core_t *core)
{
    float unk;
    struct ivp_environment *env = ivp_core_env(core);
    double max_speed = (double)ivp_env_anomaly_get_max_velocity(env);
    if (max_speed > 0.0) {
        ivp_vec3f_t speed = ivp_core_speed_f(core);
        double trans_len = (double)lvf_real_length(&speed);
        if (trans_len > max_speed) {
            unk = (float)(max_speed / trans_len);
            lvf_mult(&speed, (D)unk);
            ivp_core_set_speed_f(core, &speed);
        }
        ivp_vec3f_t speed_change;
        lvf_set_p(&speed_change, &core->speed_change);
        double rot_change_len = (double)lvf_real_length(&speed_change);
        if (rot_change_len > max_speed) {
            unk = (float)(max_speed / rot_change_len);
            lvf_mult(&speed_change, (D)unk);
            lvp_set_f(&core->speed_change, &speed_change);
        }
    }
    float max_angular_velocity_per_psi = ivp_env_anomaly_get_max_angular_velocity_per_psi(env);
    if (max_angular_velocity_per_psi > 0.0f) {
        double max_rot_speed = (double)(ivp_env_get_inv_delta_psi_time(env) * max_angular_velocity_per_psi); /* float product */
        double rot_len = (double)lvf_real_length(&core->rot_speed);
        if (rot_len > max_rot_speed) {
            unk = (float)(max_rot_speed / rot_len);
            lvf_mult(&core->rot_speed, (D)unk);
        }
        ivp_vec3f_t rot_speed_change;
        lvf_set_p(&rot_speed_change, &core->rot_speed_change);
        double rot_change_len = (double)lvf_real_length(&rot_speed_change);
        if (rot_change_len > max_rot_speed) {
            unk = (float)(max_rot_speed / rot_change_len);
            lvf_mult(&rot_speed_change, (D)unk);
            lvp_set_f(&core->rot_speed_change, &rot_speed_change);
        }
    }
}
