/* ivp_environment.h — Physics environment for IVP C17
 *
 * The environment owns objects, manages time, schedules controllers,
 * and runs a multi-phase PSI loop aligned with original IVP ordering.
 */
#ifndef IVP_ENVIRONMENT_H
#define IVP_ENVIRONMENT_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/utility/ivp_active_value.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_anomaly_manager.h>
#include <ivp/dynamics/ivp_cluster.h>
#include <ivp/dynamics/ivp_sim_unit.h>
#include <ivp/collision/ivp_collision_filter.h>
#include <ivp/events/ivp_event.h>
#include <ivp/events/ivp_listener_collision.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_environment;

/* ── Environment configuration ──────────────────────────────────────────── */

#define IVP_OBJECTS_INIT_CAPACITY      64
#define IVP_CONTROLLERS_INIT_CAPACITY  16
#define IVP_EVENT_LISTENERS_INIT_CAPACITY 8
#define IVP_SIM_UNIT_COUNT_DEFAULT      1

typedef struct ivp_environment_config {
    ivp_vec3r_t         gravity;           /* default: (0, 9.83, 0) Y-down     */
    ivp_real_t          delta_psi_time;    /* default: 1/66 s                   */
    ivp_freeze_params_t freeze_params;     /* calm/freeze detection thresholds  */
    ivp_allocator_t    *allocator;         /* if NULL, uses default allocator   */
    int                 initial_object_capacity;
    int                 initial_controller_capacity;
    int                 initial_event_listener_capacity;
    int                 sim_unit_count;
    ivp_anomaly_config_t    anomaly_config;
    ivp_collision_filter_t  collision_filter;
    bool                    ccd_enabled;          /* unused (the reference pipeline is continuous) */
    int                     n_cache_objects;      /* IVP_Application_Environment::n_cache_object (256) */
    /* IVP_Application_Environment::env_active_float_manager: NULL (default)
     * creates a manager owned by the environment; otherwise the caller's
     * manager is used and must outlive the environment. */
    struct ivp_active_value_manager *active_value_manager;
    /* IVP_Application_Environment::range_manager: NULL (default) uses an
     * environment owned IVP_Range_Manager with the reference parameters;
     * otherwise the caller's manager (must outlive the environment). */
    struct ivp_range_manager *range_manager;
    /* IVP_Application_Environment::performancecounter: NULL (default) makes
     * the environment create an ivp_performance_counter_simple_t; otherwise
     * the caller's counter is used.  Either way the environment calls its
     * environment_is_going_to_be_deleted on destruction
     * (ivp/diagnostics/ivp_performance_counter.h). */
    struct ivp_performance_counter *performance_counter;
} ivp_environment_config_t;

/* Fill with sensible defaults */
void ivp_environment_config_init(ivp_environment_config_t *cfg);

/* ── PSI phases and hooks ──────────────────────────────────────────────── */

typedef enum ivp_psi_phase {
    IVP_PSI_PHASE_PRE_PSI = 0,
    IVP_PSI_PHASE_SIM_UNIT_BEGIN,
    IVP_PSI_PHASE_REVIVE,
    IVP_PSI_PHASE_MANAGER_ACTIVE_VALUES,
    IVP_PSI_PHASE_MANAGER_CONSTRAINTS,
    IVP_PSI_PHASE_PRE_CONTROLLERS,
    IVP_PSI_PHASE_CONTROLLERS,
    IVP_PSI_PHASE_INTEGRATE,
    IVP_PSI_PHASE_COLLISION_BROADPHASE,
    IVP_PSI_PHASE_COLLISION_NARROWPHASE,
    IVP_PSI_PHASE_COLLISION_SOLVE,
    IVP_PSI_PHASE_MANAGER_EVENTS,
    IVP_PSI_PHASE_MOVEMENT_STATE,
    IVP_PSI_PHASE_SIM_UNIT_END,
    IVP_PSI_PHASE_POST_PSI,
    IVP_PSI_PHASE_COUNT
} ivp_psi_phase_t;

typedef void (*ivp_phase_hook_fn_t)(void *ctx,
                                         struct ivp_environment *env,
                                         ivp_psi_phase_t phase,
                                         ivp_time_t time,
                                         ivp_real_t dt);

/* ── Material manager ───────────────────────────────────────────────────── */

/* Contact situation passed to material manager callbacks.
 * Provides context about the collision for material lookup. */
typedef struct ivp_contact_situation {
    const struct ivp_object *obj_a;
    const struct ivp_object *obj_b;
    ivp_vec3r_t              contact_point_ws;  /* world space contact point */
    ivp_vec3r_t              contact_normal_ws;  /* world space contact normal */
} ivp_contact_situation_t;

/* Material manager callback: computes combined material properties
 * for a contact between two objects. Returns combined friction, elasticity,
 * and adhesion via out parameters.
 *
 * If NULL, the default combination rules are used:
 *   friction:   obj_a.friction * obj_b.friction
 *   elasticity: obj_a.elasticity * obj_b.elasticity
 *   adhesion:   obj_a.adhesion + obj_b.adhesion */
typedef void (*ivp_material_manager_fn_t)(void *ctx,
                                           const ivp_contact_situation_t *situation,
                                           ivp_real_t *out_friction,
                                           ivp_real_t *out_elasticity,
                                           ivp_real_t *out_adhesion);

/* ── Environment ────────────────────────────────────────────────────────── */

#define IVP_MAX_GLOBAL_COLLISION_LISTENERS 8 /* deprecated: no limit any more */

struct ivp_env_pipeline;
struct ivp_collision_delegator_root;

#define IVP_MAX_COLLISION_DELEGATOR_ROOTS 8

typedef struct ivp_environment {
    /* Configuration */
    ivp_vec3r_t         gravity;            /* use ivp_environment_set_gravity() to change */
    ivp_real_t          delta_psi_time;
    ivp_freeze_params_t freeze_params;

    /* Time (current_time mirrors IVP_Environment::current_time) */
    ivp_time_t          current_time;
    ivp_time_t          residual_time;      /* unused, kept for API compatibility */
    int                 sim_unit_count;     /* informational (reference has one PSI loop) */
    int                 current_sim_unit_index;

    /* Subsystems (opaque; implementation in dedicated modules) */
    struct ivp_object_system     *object_system;
    struct ivp_controller_system *controller_system;
    struct ivp_event_system      *event_system;

    /* Allocator */
    ivp_allocator_t    *allocator;
    bool                owns_allocator;

    /* Extended phase hook (optional) */
    ivp_phase_hook_fn_t phase_hook;
    void               *phase_hook_ctx;

    /* Material manager (optional) */
    ivp_material_manager_fn_t material_manager;
    void                     *material_manager_ctx;

    /* Anomaly manager */
    ivp_anomaly_manager_t    anomaly_manager;

    /* Active value time source (IVP_U_Active_Value_Manager current time) */
    ivp_active_float_t      *mod_current_time;

    /* Collision filter (in addition to the reference nocoll group ident check) */
    ivp_collision_filter_t   collision_filter;

    bool                     ccd_enabled;   /* unused */

    /* Universe manager (optional LOD/streaming callbacks) */
    struct ivp_universe_manager *universe_manager;

    /* Collision delegator roots (IVP_Environment::collision_delegator_roots).
     * Root 0 is the built-in mindist root. */
    struct ivp_collision_delegator_root *collision_delegator_roots[IVP_MAX_COLLISION_DELEGATOR_ROOTS];
    int                             collision_delegator_root_count;

    /* Reference simulation pipeline state (internal) */
    struct ivp_env_pipeline  *pl;

    /* Diagnostics (ivp/diagnostics/): IVP_Environment::performancecounter,
     * better_statisticsmanager, debug_information.  Use the getters. */
    struct ivp_performance_counter       *performance_counter;
    struct ivp_better_statistics_manager *better_statistics_manager;
    struct ivp_debug_manager             *debug_manager;
} ivp_environment_t;

/* ── Environment lifecycle ──────────────────────────────────────────────── */

/* Create a new environment. config may be NULL for all defaults. */
ivp_environment_t *ivp_environment_create(const ivp_environment_config_t *config);

/* Destroy environment and all objects in it (~IVP_Environment): the static
 * object first, then the objects in cluster order (the root cluster deletes
 * its first object -- the newest one -- until it is empty; sub-clusters
 * delete their objects first and are freed with the environment), firing
 * the object / collision listener events of every deleted object. */
void ivp_environment_destroy(ivp_environment_t *env);

/* IVP_Environment::get_static_object: an environment global static ball
 * (radius 1 at the origin, no collision detection) used as the world end of
 * anchors / actuators.  It is not part of the object list
 * (ivp_environment_get_object_count / get_object). */
ivp_object_t *ivp_environment_get_static_object(const ivp_environment_t *env);

/* IVP_Environment::get_active_value_manager: refreshed at the start of every
 * PSI (current_time terminal, then delayed active values). */
struct ivp_active_value_manager *ivp_environment_get_active_value_manager(const ivp_environment_t *env);

/* IVP_Environment::get_time_manager: user ivp_time_event_t events can be
 * queued with ivp_time_manager_insert_event(tm, ev, absolute_time); they are
 * fired between PSIs in time order (ivp_time_manager.h).  Events may be
 * due after the next PSI (libivp fixes the reference rebase, which would
 * shift such events). */
struct ivp_time_manager *ivp_environment_get_time_manager(ivp_environment_t *env);

/* IVP_Environment::get_range_manager (parameters may be changed at any time). */
struct ivp_range_manager *ivp_environment_get_range_manager(const ivp_environment_t *env);

/* IVP_Environment::get_performancecounter (ivp/diagnostics/ivp_performance_counter.h). */
struct ivp_performance_counter *ivp_environment_get_performance_counter(const ivp_environment_t *env);

/* IVP_Environment::get_betterstatisticsmanager: its simulation time follows
 * the simulate calls (ivp/diagnostics/ivp_better_statistics_manager.h). */
struct ivp_better_statistics_manager *ivp_environment_get_better_statistics_manager(const ivp_environment_t *env);

/* IVP_Environment::get_debug_manager: psi_counter counts the PSIs in debug
 * builds (ivp/diagnostics/ivp_debug_manager.h).  The reference creates the
 * manager only in debug builds (NULL in release); libivp always provides it,
 * its switches are only read by debug-build code. */
struct ivp_debug_manager *ivp_environment_get_debug_manager(const ivp_environment_t *env);

/* IVP_Draw_Vector_Debug (ivp_physics/ivp_debug.hxx): debug vectors the
 * engine records in debug builds (IVP_IF: friction, buoyancy, core movement
 * state, ...) for the application to draw.  The list is cleared at the start
 * of every PSI (newest first). */
typedef struct ivp_draw_vector_debug {
    struct ivp_draw_vector_debug *next;
    ivp_vec3d_t first_point;   /* world space */
    ivp_vec3d_t direction_vec;
    int         color;
    char       *debug_text;    /* may be NULL */
} ivp_draw_vector_debug_t;

/* IVP_Environment::draw_vectors (NULL in release builds) */
const ivp_draw_vector_debug_t *ivp_environment_get_draw_vectors(const ivp_environment_t *env);

/* IVP_Environment::get_statistic_manager: the reference statistic counters
 * (impacts, mindists, range events, ...), maintained in every build
 * (ivp/diagnostics/ivp_statistic_manager.h).  Debug builds print and clear
 * them once per simulated second. */
struct ivp_statistic_manager *ivp_environment_get_statistic_manager(const ivp_environment_t *env);

/* ── Object management ──────────────────────────────────────────────────── */

/* Create a dynamic object from template. Surface manager is borrowed.
 * Returns NULL on failure. */
ivp_object_t *ivp_environment_create_object(ivp_environment_t *env,
                                     const ivp_object_template_t *tmpl,
                                     const ivp_surman_t *surman);

/* Create object and also return a stable generational handle.
 * Returns NULL on failure. */
ivp_object_t *ivp_environment_create_object_with_handle(ivp_environment_t *env,
                                                const ivp_object_template_t *tmpl,
                                                const ivp_surman_t *surman,
                                                ivp_object_handle_t *out_handle);

/* IVP_Environment::get_root_cluster: the cluster holding the static object
 * and every created object (newest first) unless moved into a sub-cluster
 * with ivp_cluster_add_child.  The universe manager visits the objects in
 * this tree's order. */
ivp_cluster_t *ivp_environment_get_root_cluster(ivp_environment_t *env);

/* Create a cluster (object container hierarchy) below parent, or below the
 * root cluster when parent is NULL.  tmpl may be NULL for defaults. */
ivp_cluster_t *ivp_environment_create_cluster(ivp_environment_t *env,
                                              ivp_cluster_t *parent,
                                              const ivp_cluster_template_t *tmpl);

/* Destroy a cluster.  Its children are NOT destroyed: they move to the
 * cluster's parent. */
void ivp_environment_destroy_cluster(ivp_environment_t *env, ivp_cluster_t *cluster);

/* Destroy an object and remove it from the environment. */
void ivp_environment_destroy_object(ivp_environment_t *env, ivp_object_t *obj);

/* Destroy object by stable handle. Returns true if found and removed. */
bool ivp_environment_destroy_object_by_handle(ivp_environment_t *env, ivp_object_handle_t handle);

/* Lookup object by stable handle. Returns NULL for stale/invalid handles. */
ivp_object_t *ivp_environment_get_object_by_handle(const ivp_environment_t *env, ivp_object_handle_t handle);

/* Returns object's current stable handle. Invalid handle if obj is NULL. */
ivp_object_handle_t ivp_environment_get_object_handle(const ivp_object_t *obj);

/* ── Simulation ─────────────────────────────────────────────────────────── */

/* IVP_Environment::simulate_dtime: simulate_until(current_time + elapsed_time)
 * (PSIs and time events up to that time; AT queries interpolate). */
void ivp_environment_simulate_dtime(ivp_environment_t *env, ivp_real_t elapsed_time);

/* Run exactly one PSI step. Exposed for fine-grained control and testing. */
void ivp_environment_simulate_one_psi(ivp_environment_t *env);

/* ── Controller API ────────────────────────────────────────────────────── */

/* Add a controller (IVP_Controller_Manager::announce_controller_to_environment).
 * Built-in callbacks (ivp_controller_spring, ...) use the priority of their
 * reference class (the argument is ignored); those acting on a fixed set of
 * objects (actuators, constraints, springs, cars) become dependent
 * controllers that merge the sim units of their objects, and are removed
 * when one of their objects is deleted.  Any other callback is a user
 * controller (IVP_Controller subclass) with the given priority (IVP_CP_*),
 * called for every simulated core.  Returns an invalid handle on failure. */
ivp_controller_handle_t ivp_environment_add_controller(ivp_environment_t *env,
                                               int priority,
                                               ivp_controller_fn_t fn,
                                               void *ctx);

/* Announce a dependent controller (IVP_Controller_Dependent) whose cores are
 * returned by discovery_fn (re-evaluated every PSI; their sim units are
 * merged).  discovery_fn must be non-NULL.  Built-in callbacks use their
 * reference priority, user callbacks the given one. */
ivp_controller_handle_t ivp_environment_announce_dependent_controller(
    ivp_environment_t *env,
    int priority,
    ivp_controller_fn_t fn,
    void *ctx,
    ivp_controller_discovery_fn_t discovery_fn);

/* Remove a controller by id. Returns true if removed. */
bool ivp_environment_remove_controller(ivp_environment_t *env, ivp_controller_handle_t controller_handle);

/* Manager-equivalent dependent removal. silently=true suppresses event emission. */
bool ivp_environment_remove_dependent_controller(ivp_environment_t *env,
                                                 ivp_controller_handle_t controller_handle,
                                                 bool silently);

/* Set or replace dynamic associated-core discovery callback for a controller. */
bool ivp_environment_set_controller_discovery(ivp_environment_t *env,
                                              ivp_controller_handle_t controller_handle,
                                              ivp_controller_discovery_fn_t discovery_fn);

/* Manager-equivalent independent attach/remove operations. */
bool ivp_environment_add_controller_to_core(ivp_environment_t *env,
                                            ivp_controller_handle_t controller_handle,
                                            ivp_object_t *object);
bool ivp_environment_remove_controller_from_core(ivp_environment_t *env,
                                                 ivp_controller_handle_t controller_handle,
                                                 ivp_object_t *object);

/* IVP_Controller_Independent attached to no core: add cores with
 * ivp_environment_add_controller_to_core (IVP_Controller_Manager::
 * add_controller_to_core).  Built-in callbacks use their reference priority. */
ivp_controller_handle_t ivp_environment_create_core_controller(ivp_environment_t *env, int priority,
                                                               ivp_controller_fn_t fn, void *ctx);

/* Wake-up parity helpers. */
bool ivp_environment_ensure_controller_in_simulation(ivp_environment_t *env,
                                                     ivp_controller_handle_t controller_handle);
bool ivp_environment_ensure_core_in_simulation(ivp_environment_t *env,
                                               ivp_object_t *object);
/* IVP_Actuator_Two_Point::ensure_actuator_in_simulation for an actuator whose
 * anchors are attached to objects[0..count) (anchor order): revives the sim
 * unit of the controlled cores at once, like the reference controller
 * manager (ivp_environment_ensure_core_in_simulation only queues a revive). */
void ivp_environment_ensure_actuator_in_simulation(ivp_environment_t *env,
                                                   ivp_object_t *const *objects, int count);

/* Remove all controllers. */
void ivp_environment_clear_controllers(ivp_environment_t *env);

/* ── Event API ─────────────────────────────────────────────────────────── */

/* Add an event listener. Returns listener id (>0) on success, -1 on failure. */
ivp_event_listener_handle_t ivp_environment_add_event_listener(ivp_environment_t *env,
                                                       ivp_event_listener_fn_t fn,
                                                       void *ctx);

/* Remove an event listener by id. Returns true if removed. */
bool ivp_environment_remove_event_listener(ivp_environment_t *env,
                                   ivp_event_listener_handle_t listener_handle);

/* Remove all event listeners. */
void ivp_environment_clear_event_listeners(ivp_environment_t *env);

/* ── Sim unit API ──────────────────────────────────────────────────────── */

/* Sets number of simulation units per PSI. Values <= 0 are clamped to 1. */
void ivp_environment_set_sim_unit_count(ivp_environment_t *env, int sim_unit_count);

/* Current configured sim unit count. */
int ivp_environment_get_sim_unit_count(const ivp_environment_t *env);

/* Active sim unit index during callback execution, else -1. */
int ivp_environment_get_current_sim_unit_index(const ivp_environment_t *env);

/* ── Getters / Setters ──────────────────────────────────────────────────── */

static inline ivp_time_t ivp_environment_get_time(const ivp_environment_t *env) {
    return env->current_time;
}

static inline ivp_real_t ivp_environment_get_delta_psi_time(const ivp_environment_t *env) {
    return env->delta_psi_time;
}

/* IVP_Environment::set_gravity (also updates the standard gravity controller). */
void ivp_environment_set_gravity(ivp_environment_t *env, const ivp_vec3r_t *g);

/* IVP_Environment::set_delta_PSI_time (range IVP_MIN_DELTA_PSI_TIME..IVP_MAX_DELTA_PSI_TIME). */
void ivp_environment_set_delta_psi_time(ivp_environment_t *env, ivp_real_t dt);

/* IVP_Environment::simulate_until / simulate_variable_time_step / reset_time. */
void ivp_environment_simulate_until(ivp_environment_t *env, ivp_time_t until_time);
void ivp_environment_simulate_variable_time_step(ivp_environment_t *env, ivp_real_t delta_time);
void ivp_environment_reset_time(ivp_environment_t *env);

/* Collision tolerance (IVP_Environment::set_global_collision_tolerance). */
void ivp_environment_set_global_collision_tolerance(ivp_environment_t *env, ivp_real_t tolerance);

static inline void ivp_environment_get_gravity(const ivp_environment_t *env, ivp_vec3r_t *g) {
    *g = env->gravity;
}

int ivp_environment_get_object_count(const ivp_environment_t *env);
ivp_object_t *ivp_environment_get_object(const ivp_environment_t *env, int index);

/* Set extended phase hook (called once per phase per PSI step). */
static inline void ivp_environment_set_phase_hook(ivp_environment_t *env,
                                           ivp_phase_hook_fn_t fn,
                                           void *ctx) {
    env->phase_hook = fn;
    env->phase_hook_ctx = ctx;
}

/* Set material manager callback. If NULL, default combination rules are used. */
static inline void ivp_environment_set_material_manager(ivp_environment_t *env,
                                                  ivp_material_manager_fn_t fn,
                                                  void *ctx) {
    env->material_manager = fn;
    env->material_manager_ctx = ctx;
}

/* ── Collision Delegator API (reference: IVP_Collision_Delegator_Root) ── */

/* Register a collision delegator root on the environment.
 * During broadphase, each root's delegate_collisions() is called for
 * overlapping pairs. Returns true on success, false if list full. */
bool ivp_environment_add_collision_delegator(ivp_environment_t *env,
                                             struct ivp_collision_delegator_root *root);

/* Remove a collision delegator root. Returns true if found & removed. */
bool ivp_environment_remove_collision_delegator(ivp_environment_t *env,
                                                struct ivp_collision_delegator_root *root);

/* ── Global Collision Listener API (reference: IVP_Environment::collision_listeners) ── */

/* Register a global collision listener that fires for ALL collisions.
 * Global listeners run before per-object listeners.
 * Returns 0 on success, -1 if max listeners reached. */
int ivp_environment_add_collision_listener_global(ivp_environment_t *env,
                                                  ivp_collision_listener_t *listener);

/* Remove a global collision listener. Returns 0 on success, -1 if not found. */
int ivp_environment_remove_collision_listener_global(ivp_environment_t *env,
                                                     ivp_collision_listener_t *listener);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ENVIRONMENT_H */
