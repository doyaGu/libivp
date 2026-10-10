/* ivp_sim_internal.h — internal simulation pipeline (port of the reference
 * IVP_Event_Sim / IVP_Controller / IVP_Simulation_Unit / IVP_Core helpers)
 *
 * Reference sources: ivp_physics/ivp_controller.{hxx,cxx}, ivp_intern/ivp_sim_unit.{hxx,cxx},
 * ivp_physics/ivp_core.{hxx,cxx}, ivp_intern/ivp_calc_next_psi_solver.{hxx,cxx},
 * ivp_intern/ivp_physic_private.cxx.
 *
 * Precision notes (reference types):
 *   IVP_Core::speed / rot_speed / speed_change / rot_speed_change : IVP_U_Float_Point (float)
 *   IVP_Core::pos_world_f_core_last_psi : IVP_U_Point (double)
 *   IVP_Core::q_world_f_core_* / m_world_f_core_last_psi : double
 * libivp keeps its public field types; arithmetic follows the reference expression
 * order with double intermediates.
 */
#ifndef IVP_SIM_INTERNAL_H
#define IVP_SIM_INTERNAL_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_time.h>
#include <ivp/base/ivp_legacy.h>
#include <ivp/containers/ivp_pvec.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include "base/ivp_simd_internal.h"
#include "dynamics/ivp_env_internal.h"

struct ivp_environment;
struct ivp_sim_unit;
struct ivp_fr_system;
struct ivp_fr_info_for_core;
struct ivp_hull_mgr;

/* ── IVP_Event_Sim ──────────────────────────────────────────────────────── */

typedef struct ivp_sim_es {
    double                  delta_time;
    double                  i_delta_time;
    struct ivp_environment *environment;
    struct ivp_sim_unit    *sim_unit;
} ivp_sim_es_t;

static inline void ivp_sim_es_init(ivp_sim_es_t *es, struct ivp_environment *env) {
    es->environment = env;
    es->delta_time = (double)ivp_env_get_delta_psi_time(env);
    es->i_delta_time = (double)ivp_env_get_inv_delta_psi_time(env);
    es->sim_unit = NULL;
}

static inline void ivp_sim_es_init_dt(ivp_sim_es_t *es, struct ivp_environment *env, double dtime) {
    es->environment = env;
    es->delta_time = dtime;
    if (dtime > (double)P_FLOAT_EPS) {
        es->i_delta_time = 1.0 / dtime;
    } else {
        es->i_delta_time = (double)(1.0f / P_FLOAT_EPS);
    }
    es->sim_unit = NULL;
}

/* ── IVP_Controller (internal dispatch object) ──────────────────────────── */

typedef struct ivp_sim_ctrl ivp_sim_ctrl_t;

typedef struct ivp_sim_ctrl_vt {
    int  (*get_priority)(ivp_sim_ctrl_t *c);
    void (*do_simulation)(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *core_list);
    /* IVP_Controller::get_associated_controlled_cores (NULL = empty list) */
    ivp_pvec_t *(*get_associated_controlled_cores)(ivp_sim_ctrl_t *c);
    void (*core_is_going_to_be_deleted_event)(ivp_sim_ctrl_t *c, ivp_core_t *core);
    void (*reset_time)(ivp_sim_ctrl_t *c, double offset); /* nullable */
} ivp_sim_ctrl_vt_t;

struct ivp_sim_ctrl {
    const ivp_sim_ctrl_vt_t *vt;
    uint64_t registration_sequence;
};

static inline int ivp_sim_ctrl_priority(ivp_sim_ctrl_t *c) { return c->vt->get_priority(c); }

/* True only for built-in controllers whose island-local writes and lack of
 * shared callbacks make a worker-thread dispatch safe. */
bool ivp_env_sim_ctrl_is_parallel_safe(ivp_sim_ctrl_t *c);

/* ── IVP_Simulation_Unit ────────────────────────────────────────────────── */

typedef struct ivp_su_ctrl_list {
    ivp_sim_ctrl_t *l_controller;
    ivp_pvec_t      cores_controlled_by;
} ivp_su_ctrl_list_t;

typedef struct ivp_sim_unit {
    uint8_t  sim_unit_movement_type;        /* ivp_movement_type_t */
    bool     union_find_needed_for_sim_unit;
    bool     sim_unit_has_fast_objects;
    bool     sim_unit_just_slowed_down;
    bool     parallel_movement_check;
    bool     parallel_should_freeze;
    int      parallel_touched_offset;
    struct ivp_sim_unit *prev_sim_unit;
    struct ivp_sim_unit *next_sim_unit;
    const ivp_allocator_t *allocator;
    struct ivp_sim_units_manager *manager; /* NULL while detached */
    ivp_pvec_t sim_unit_cores;              /* ivp_core_t*           */
    ivp_pvec_t controller_cores;            /* ivp_su_ctrl_list_t*   */
} ivp_sim_unit_t;

typedef struct ivp_sim_units_manager {
    struct ivp_environment *l_environment;
    ivp_sim_unit_t *sim_units_slot0;        /* sim_units_slots[0] (others unused) */
    ivp_sim_unit_t *still_slot;
    /* Exact count of linked units with union_find_needed_for_sim_unit set.
     * Topology changes run on the world owner, never in safe island kernels. */
    size_t pending_union_find_count;
} ivp_sim_units_manager_t;

ivp_sim_unit_t *ivp_sim_unit_new(const ivp_allocator_t *allocator);
void ivp_sim_unit_delete(ivp_sim_unit_t *su);           /* ~IVP_Simulation_Unit (+free) */

void ivp_sim_units_mgr_init(ivp_sim_units_manager_t *m, struct ivp_environment *env);
void ivp_sim_units_mgr_add_sim_unit(ivp_sim_units_manager_t *m, ivp_sim_unit_t *su);
void ivp_sim_units_mgr_rem_sim_unit(ivp_sim_units_manager_t *m, ivp_sim_unit_t *su);
void ivp_sim_units_mgr_simulate_sim_units_psi(ivp_sim_units_manager_t *m, ivp_pvec_t *touched_cores);
void ivp_sim_units_mgr_reset_time(ivp_sim_units_manager_t *m, double offset);
uint64_t ivp_core_stable_order_key(const ivp_core_t *core);
uint64_t ivp_sim_unit_stable_order_key(const ivp_sim_unit_t *su);

bool ivp_sim_unit_add_sim_unit_core(ivp_sim_unit_t *su, ivp_core_t *core);
void ivp_sim_unit_rem_sim_unit_core(ivp_sim_unit_t *su, ivp_core_t *core);
void ivp_sim_unit_sim_unit_remove_core(ivp_sim_unit_t *su, ivp_core_t *core); /* may delete su */
bool ivp_sim_unit_add_controller_of_core(ivp_sim_unit_t *su, ivp_core_t *core, ivp_sim_ctrl_t *c);
void ivp_sim_unit_remove_controller_of_core(ivp_sim_unit_t *su, ivp_core_t *core, ivp_sim_ctrl_t *c);
void ivp_sim_unit_clean_sim_unit(ivp_sim_unit_t *su);
bool ivp_sim_unit_sim_unit_calc_redundants(ivp_sim_unit_t *su);
bool ivp_sim_unit_throw_cores_into_my_sim_unit(ivp_sim_unit_t *su, ivp_sim_unit_t *second);
bool ivp_sim_unit_fusion_simulation_unities(ivp_sim_unit_t *su, ivp_sim_unit_t *second);
void ivp_sim_unit_revive_for_simulation(ivp_sim_unit_t *su, struct ivp_environment *env);
void ivp_sim_unit_ensure_cores_movement(ivp_sim_unit_t *su);
void ivp_sim_unit_ensure_in_simulation(ivp_sim_unit_t *su);
void ivp_sim_unit_do_sim_unit_union_find(ivp_sim_unit_t *su);
/* All dirty transitions must go through this helper, including OOM retries. */
void ivp_sim_unit_request_union_find(ivp_sim_unit_t *su);
bool ivp_sim_unit_sim_unit_calc_movement_state(ivp_sim_unit_t *su, struct ivp_environment *env);
bool ivp_sim_unit_sim_unit_core_exists(ivp_sim_unit_t *su, ivp_core_t *core);
#ifdef IVP_DEBUG_BUILD
/* reference ivp_sim_unit.hxx "#ifdef DEBUG" methods */
void ivp_sim_unit_sim_unit_debug_consistency(ivp_sim_unit_t *su);
void ivp_sim_unit_sim_unit_debug_out(ivp_sim_unit_t *su);
#endif

/* IVP_Controller_Manager */
bool ivp_ctrl_mgr_announce_controller_to_environment(struct ivp_environment *env, ivp_sim_ctrl_t *c);
void ivp_ctrl_mgr_remove_controller_from_environment(struct ivp_environment *env, ivp_sim_ctrl_t *c, bool silently);
void ivp_ctrl_mgr_ensure_controller_in_simulation(struct ivp_environment *env, ivp_sim_ctrl_t *c);
bool ivp_ctrl_mgr_add_controller_to_core(ivp_sim_ctrl_t *c, ivp_core_t *core);
void ivp_ctrl_mgr_remove_controller_from_core(ivp_sim_ctrl_t *c, ivp_core_t *core);
bool ivp_ctrl_mgr_core_has_controller(ivp_core_t *core, ivp_sim_ctrl_t *c);

/* ── Core helpers (IVP_Core) ─────────────────────────────────────────────── */

static inline struct ivp_environment *ivp_core_env(const ivp_core_t *core) { return core->environment; }

/* IVP_Core::objects: number of objects / object i */
static inline int ivp_core_n_objects(const ivp_core_t *core) { return ivp_pvec_len(&core->objects); }
static inline struct ivp_object *ivp_core_object_at(const ivp_core_t *core, int i) {
    return (struct ivp_object *)ivp_pvec_at(&core->objects, i);
}

/* IVP_Core ctors / dtor and object links (cores are heap allocated; an
 * object's cores are physical_core / friction_core / original_core) */
bool ivp_core_init_from_object(ivp_core_t *core, struct ivp_object *io);   /* IVP_Core(io) */
ivp_core_t *ivp_core_create(struct ivp_object *io, const ivp_quatr_t *q_world_f_object_init, const ivp_vec3r_t *position,
                            bool physical_unmoveable, bool enable_piling_optimization);
void ivp_core_delete(ivp_core_t *core);                                     /* ~IVP_Core + free */
bool ivp_core_core_add_link_to_obj(ivp_core_t *core, struct ivp_object *add_obj);
void ivp_core_unlink_obj_from_core_and_maybe_destroy(ivp_core_t *core, struct ivp_object *remove_obj);
void ivp_core_fire_event_object_frozen(ivp_core_t *core);
/* IVP_Core_Merged / IVP_Core_Collision helpers (ivp_merge_core.c) */
void ivp_core_merged_set_by_merge(ivp_core_merged_t *mcore, ivp_core_t *c0, ivp_core_t *c1);
ivp_core_merged_t *ivp_core_merged_create_from_cores(ivp_core_t *core0, ivp_core_t *core1);
void ivp_core_create_collision_merged_core_with(ivp_core_t *core, ivp_core_t *other_core);
void ivp_core_split_collision_merged_core_next_psi(ivp_core_merged_t *ccore);
void ivp_core_ensure_all_core_objs_in_simulation_now(ivp_core_t *core);

bool ivp_core_add_core_controller(ivp_core_t *core, ivp_sim_ctrl_t *c);
void ivp_core_rem_core_controller(ivp_core_t *core, ivp_sim_ctrl_t *c);

struct ivp_fr_info_for_core *ivp_core_get_friction_info(ivp_core_t *core, struct ivp_fr_system *fs);
struct ivp_fr_info_for_core *ivp_core_moveable_core_has_friction_info(ivp_core_t *core);
bool ivp_core_add_friction_info(ivp_core_t *core, struct ivp_fr_info_for_core *info);
void ivp_core_unlink_friction_info(ivp_core_t *core, struct ivp_fr_info_for_core *info);
void ivp_core_delete_friction_info(ivp_core_t *core, struct ivp_fr_info_for_core *info);
bool ivp_core_grow_friction_system(ivp_core_t *core);

ivp_core_t *ivp_core_union_find_get_father(ivp_core_t *core);

void   ivp_core_get_surface_speed_on_test(const ivp_core_t *core, const ivp_vec3f_t *point_cs,
                                          const ivp_vec3f_t *center_speed_ws, const ivp_vec3f_t *rot_speed_cs,
                                          ivp_vec3f_t *speed_out_ws);
void   ivp_core_get_surface_speed(const ivp_core_t *core, const ivp_vec3f_t *point_cs, ivp_vec3f_t *out_ws);
void   ivp_core_get_diff_surface_speed_of_two_cores_on_test(
           const ivp_core_t *c0, const ivp_core_t *c1, const ivp_vec3f_t *p0, const ivp_vec3f_t *p1,
           const ivp_vec3f_t *ts0, const ivp_vec3f_t *rs0, const ivp_vec3f_t *ts1, const ivp_vec3f_t *rs1,
           ivp_vec3f_t *out);
void   ivp_core_test_push_core(const ivp_core_t *core, const ivp_vec3f_t *point_cs,
                               const ivp_vec3f_t *impulse_cs, const ivp_vec3f_t *impulse_ws,
                               ivp_vec3f_t *speed_out, ivp_vec3f_t *rot_out);
double ivp_core_calc_virt_mass_worst_case(const ivp_core_t *core, const ivp_vec3f_t *core_point);
double ivp_core_calc_correct_virt_mass(const ivp_core_t *core, const ivp_vec3f_t *core_point,
                                       const ivp_vec3f_t *direction_core, const ivp_vec3f_t *direction_world);
double ivp_core_get_energy_on_test(const ivp_core_t *core, const ivp_vec3f_t *speed_vec, const ivp_vec3f_t *rot_vec);
void   ivp_core_commit_all_async_pushes_ref(ivp_core_t *core);
/* IVP_Core::core_plausible_check (debug check, asserts only) */
void   ivp_core_core_plausible_check(ivp_core_t *core);
/* IVP_Core::debug_vec_movement_state (debug draw vector; its body is IVP_IF(0)) */
void   ivp_core_debug_vec_movement_state(ivp_core_t *core);
void   ivp_core_abort_all_async_pushes(ivp_core_t *core);
void   ivp_core_global_damp_core(ivp_core_t *core, double d_time);
void   ivp_core_reset_freeze_check_values(ivp_core_t *core);
void   ivp_core_clip_velocity(ivp_core_t *core, ivp_vec3f_t *velocity, ivp_vec3f_t *rotation_vec);
void   ivp_core_synchronize_with_rot_z(ivp_core_t *core);
void   ivp_core_undo_synchronize_rot_z(ivp_core_t *core);
void   ivp_core_update_exact_mindist_events_of_core(ivp_core_t *core);
void   ivp_core_calc_at_matrix_ref(const ivp_core_t *core, ivp_time_t t, ivp_transform_r_t *out);
void   ivp_core_inline_calc_at_quaternion(const ivp_core_t *core, ivp_time_t t, ivp_quatr_t *out);
void   ivp_core_inline_calc_at_position(const ivp_core_t *core, ivp_time_t t, ivp_vec3r_t *out);
ivp_movement_type_t ivp_core_calc_movement_state(ivp_core_t *core, ivp_time_t psi_time);
void   ivp_core_calc_next_psi_matrix_zero_speed(ivp_core_t *core, ivp_sim_es_t *es);
void   ivp_core_stop_movement_without_collision_recheck(ivp_core_t *core);
void   ivp_core_stop_physical_movement(ivp_core_t *core);
void   ivp_core_init_core_for_simulation(ivp_core_t *core);
void   ivp_core_freeze_simulation_core(ivp_core_t *core);
bool   ivp_core_revive_simulation_core(ivp_core_t *core);
void   ivp_core_ensure_core_to_be_in_simulation(ivp_core_t *core);
void   ivp_core_ensure_core_in_simulation_delayed(ivp_core_t *core);
void   ivp_core_ensure_all_core_objs_in_simulation(ivp_core_t *core);
void   ivp_core_revive_adjacent_to_unmoveable(ivp_core_t *core);
void   ivp_core_calc_calc(ivp_core_t *core);
void   ivp_core_reset_time(ivp_core_t *core, double offset);
void   ivp_core_values_changed_recalc_redundants(ivp_core_t *core);

/* get_inv_rot_inertia() / get_rot_inertia() as float vectors */
static inline ivp_vec3f_t ivp_core_inv_rot_inertia_f(const ivp_core_t *c) {
    ivp_vec3f_t r;
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    vst1_f32(r.k, vcvt_f32_f64(vld1q_f64(c->inv_rot_inertia.k)));
    r.k[2] = (float)c->inv_rot_inertia.k[2];
#else
    r.k[0] = (float)c->inv_rot_inertia.k[0];
    r.k[1] = (float)c->inv_rot_inertia.k[1];
    r.k[2] = (float)c->inv_rot_inertia.k[2];
#endif
    return r;
}
static inline ivp_vec3f_t ivp_core_rot_inertia_f(const ivp_core_t *c) {
    ivp_vec3f_t r;
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    vst1_f32(r.k, vcvt_f32_f64(vld1q_f64(c->rot_inertia.k)));
    r.k[2] = (float)c->rot_inertia.k[2];
#else
    r.k[0] = (float)c->rot_inertia.k[0];
    r.k[1] = (float)c->rot_inertia.k[1];
    r.k[2] = (float)c->rot_inertia.k[2];
#endif
    return r;
}
static inline float ivp_core_get_mass_f(const ivp_core_t *c) { return (float)c->mass; }
static inline float ivp_core_get_inv_mass_f(const ivp_core_t *c) { return (float)c->inv_mass; }

/* speed (world) and speed_change as float vectors (IVP_U_Float_Point in the reference) */
static inline ivp_vec3f_t ivp_core_speed_f(const ivp_core_t *c) {
    ivp_vec3f_t r;
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    vst1_f32(r.k, vcvt_f32_f64(vld1q_f64(c->speed.k)));
    r.k[2] = (float)c->speed.k[2];
#else
    r.k[0] = (float)c->speed.k[0];
    r.k[1] = (float)c->speed.k[1];
    r.k[2] = (float)c->speed.k[2];
#endif
    return r;
}
static inline void ivp_core_set_speed_f(ivp_core_t *c, const ivp_vec3f_t *v) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    vst1q_f64(c->speed.k, vcvt_f64_f32(vld1_f32(v->k)));
    c->speed.k[2] = (ivp_real_t)v->k[2];
#else
    c->speed.k[0] = (ivp_real_t)v->k[0];
    c->speed.k[1] = (ivp_real_t)v->k[1];
    c->speed.k[2] = (ivp_real_t)v->k[2];
#endif
}

/* ── IVP_Calc_Next_PSI_Solver ───────────────────────────────────────────── */

void ivp_cnps_calc_next_psi_matrix(ivp_core_t *core, ivp_sim_es_t *es, ivp_pvec_t *active_hull_managers_out);
void ivp_cnps_set_transformation(ivp_core_t *core, const ivp_quatr_t *rotation, const ivp_vec3r_t *position,
                                 bool optimize_for_repeated_calls);
void ivp_cnps_commit_all_calc_next_psi_matrix(struct ivp_environment *env, ivp_pvec_t *cores,
                                              ivp_pvec_t *active_hulls_out);
void ivp_cnps_commit_all_hull_managers(struct ivp_environment *env, ivp_pvec_t *active_hull_managers);

/* ── Object helpers (IVP_Real_Object) ───────────────────────────────────── */

/* IVP_Real_Object::set_new_m_object_f_core / init_object_core and
 * IVP_Core::transform_PSI_matrizes_core (ivp_object_pipeline.c) */
void ivp_object_set_new_m_object_f_core(struct ivp_object *obj, const ivp_transform_r_t *new_m_object_f_core);
void ivp_object_init_object_core(struct ivp_object *obj, const ivp_object_template_t *templ);
void ivp_core_transform_psi_matrizes_core(ivp_core_t *core, const ivp_transform_r_t *m_core_f_core);
/* IVP_Real_Object::calc_m_core_f_object */
void ivp_object_calc_m_core_f_object(const struct ivp_object *obj, ivp_transform_r_t *m_core_f_object);
/* IVP_Real_Object::unlink_contact_points / get_quat_world_f_object_AT */
void ivp_object_unlink_contact_points(struct ivp_object *obj, bool silent);
void ivp_object_get_quat_world_f_object_AT(const struct ivp_object *obj, ivp_quatr_t *q_world_f_object, ivp_vec3r_t *position);

void ivp_object_revive_object_for_simulation(struct ivp_object *obj);
void ivp_object_ensure_in_simulation(struct ivp_object *obj);
void ivp_object_set_movement_state(struct ivp_object *obj, ivp_movement_type_t mt);
void ivp_object_recalc_invalid_mindists_of_object(struct ivp_object *obj);
void ivp_object_recalc_exact_mindists_of_object(struct ivp_object *obj);
void ivp_object_update_exact_mindist_events_of_object(struct ivp_object *obj);
void ivp_object_get_all_near_mindists(struct ivp_object *obj);
void ivp_object_reset_time(struct ivp_object *obj, double offset);
void ivp_object_calc_at_matrix(const struct ivp_object *obj, ivp_time_t t, ivp_transform_r_t *out);
void ivp_object_async_push_object_ws(struct ivp_object *obj, const ivp_vec3r_t *position_ws, const ivp_vec3f_t *impulse_ws);
void ivp_object_async_add_rot_speed_object_cs(struct ivp_object *obj, const ivp_vec3f_t *rotation_vec);
void ivp_object_async_add_speed_object_ws(struct ivp_object *obj, const ivp_vec3f_t *speed_vec);
void ivp_core_async_push_core(ivp_core_t *core, const ivp_vec3f_t *point_cs, const ivp_vec3f_t *impulse_cs,
                              const ivp_vec3f_t *impulse_ws);

/* ── Material / events glue ─────────────────────────────────────────────── */

struct ivp_impact_solver_lt;
struct ivp_contact_pt;
float ivp_env_material_get_elasticity(struct ivp_environment *env, struct ivp_impact_solver_lt *info);
float ivp_env_material_get_friction_factor(struct ivp_environment *env, struct ivp_impact_solver_lt *info);

void ivp_env_fire_friction_created(struct ivp_environment *env, struct ivp_contact_pt *cp);
void ivp_env_fire_friction_deleted(struct ivp_environment *env, struct ivp_contact_pt *cp);
struct ivp_fr_core_pair;
void ivp_env_fire_friction_pair_created(struct ivp_environment *env, struct ivp_fr_core_pair *pair);
void ivp_env_fire_friction_pair_deleted(struct ivp_environment *env, struct ivp_fr_core_pair *pair);
void ivp_env_fire_pre_collision(struct ivp_environment *env, struct ivp_impact_solver_lt *info, double d_time_since_last);
void ivp_env_fire_post_collision(struct ivp_environment *env, struct ivp_impact_solver_lt *info, double d_time_since_last);

void ivp_env_fire_object_revived(struct ivp_environment *env, struct ivp_object *obj);
void ivp_env_fire_object_frozen(struct ivp_environment *env, struct ivp_object *obj);
/* IVP_Environment::set_current_time (increments the time code) */
void ivp_environment_set_current_time_internal(struct ivp_environment *env, ivp_time_t t);

/* deferred object deletion around impacts (IVP_Real_Object::begin/end_deferred_deletion) */
void ivp_env_begin_deferred_deletion(struct ivp_environment *env);
void ivp_env_end_deferred_deletion(struct ivp_environment *env);
bool ivp_env_object_deletion_is_deferred(struct ivp_environment *env, const struct ivp_object *obj);

/* anomaly manager hooks */
void ivp_env_anomaly_max_velocity_exceeded(struct ivp_environment *env, ivp_core_t *core, ivp_vec3f_t *velocity);
void ivp_env_anomaly_max_angular_velocity_exceeded(struct ivp_environment *env, ivp_core_t *core, ivp_vec3f_t *rot);
bool ivp_env_anomaly_max_collisions_exceeded_check_freezing(struct ivp_environment *env, ivp_core_t *core);
float ivp_env_anomaly_get_max_velocity(struct ivp_environment *env);
float ivp_env_anomaly_get_max_angular_velocity_per_psi(struct ivp_environment *env);
int   ivp_env_anomaly_get_max_collisions_per_psi(struct ivp_environment *env);

/* freeze manager */
double ivp_env_get_freeze_check_dtime(struct ivp_environment *env);
bool   ivp_env_must_perform_movement_check(struct ivp_environment *env);

/* the environment's standard gravity controller (attached to every core) */
ivp_sim_ctrl_t *ivp_env_get_standard_gravity_controller(struct ivp_environment *env);
ivp_sim_units_manager_t *ivp_env_get_sim_units_manager(struct ivp_environment *env);

#endif /* IVP_SIM_INTERNAL_H */
