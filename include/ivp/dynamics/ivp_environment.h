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
#include <ivp/controllers/ivp_controller.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_anomaly_manager.h>
#include <ivp/dynamics/ivp_cluster.h>
#include <ivp/collision/ivp_collision_filter.h>
#include <ivp/collision/ivp_hull_manager.h>
#include <ivp/collision/ivp_mindist_persistent.h>
#include <ivp/collision/ivp_time_event_queue.h>
#include <ivp/solver/ivp_friction_system.h>
#include <ivp/solver/ivp_solver_config.h>
#include <ivp/collision/ivp_broadphase.h>
#include <ivp/dynamics/ivp_island_manager.h>
#include <ivp/events/ivp_event.h>
#include <ivp/collision/ivp_collision_delegator.h>
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
    ivp_vec3r_t         gravity;           /* default: (0, 9.81, 0) Y-down     */
    ivp_real_t          delta_psi_time;    /* default: 1/66 s                   */
    ivp_freeze_params_t freeze_params;     /* calm/freeze detection thresholds  */
    ivp_allocator_t    *allocator;         /* if NULL, uses default allocator   */
    int                 initial_object_capacity;
    int                 initial_controller_capacity;
    int                 initial_event_listener_capacity;
    int                 sim_unit_count;
    ivp_anomaly_config_t    anomaly_config;
    ivp_collision_filter_t  collision_filter;
    bool                    ccd_enabled;          /* continuous collision detection (default: false) */
    ivp_solver_config_t     solver_config;        /* solver tuning parameters */
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

typedef struct ivp_environment {
    /* Configuration */
    ivp_vec3r_t         gravity;
    ivp_real_t          delta_psi_time;
    ivp_freeze_params_t freeze_params;

    /* Time */
    ivp_time_t          current_time;       /* accumulated simulation time */
    ivp_time_t          residual_time;      /* leftover from last simulate */
    int                 sim_unit_count;     /* number of sub-units per PSI */
    int                 current_sim_unit_index;

    /* Subsystems (opaque; implementation in dedicated modules) */
    struct ivp_object_system     *object_system;
    struct ivp_controller_system *controller_system;
    struct ivp_event_system      *event_system;

    /* Allocator */
    ivp_allocator_t    *allocator;
    bool                owns_allocator;     /* true if we created default */

    /* Extended phase hook (optional) */
    ivp_phase_hook_fn_t phase_hook;
    void                   *phase_hook_ctx;

    /* Material manager (optional) */
    ivp_material_manager_fn_t material_manager;
    void                     *material_manager_ctx;

    /* Anomaly manager */
    ivp_anomaly_manager_t    anomaly_manager;

    /* Collision filter */
    ivp_collision_filter_t   collision_filter;

    /* Friction persistence */
    ivp_friction_manager_t   friction_manager;

    /* Legacy-style persistent mindist + hull + time-event orchestration. */
    ivp_mindist_manager_t    mindist_manager;
    ivp_hull_manager_t       hull_manager;
    ivp_time_event_queue_t   time_event_queue;

    /* CCD enabled */
    bool                     ccd_enabled;

    /* Legacy forward-projection: init_for_psi advances position by
     * delta_world_f_core_psis * dt before controllers run.  Legacy's
     * incremental mindist handles the resulting overlap.  Default true
     * to match legacy behavior. */
    bool                     use_forward_projection;

    /* Sweep-and-prune broadphase (used when object count >= IVP_SAP_THRESHOLD) */
    ivp_broadphase_sap_t     broadphase_sap;

    /* Simulation island manager */
    ivp_island_manager_t     island_manager;

    /* Solver configuration (runtime-tunable) */
    ivp_solver_config_t      solver_config;

    /* Universe manager (optional LOD/streaming callbacks) */
    struct ivp_universe_manager *universe_manager;

    /* Collision delegator roots (reference: IVP_Environment::collision_delegator_roots).
     * During broadphase overlap, each root is queried to create a collision
     * checker for the pair.  Fixed-size array avoids dynamic allocation. */
    ivp_collision_delegator_root_t *collision_delegator_roots[IVP_MAX_COLLISION_DELEGATOR_ROOTS];
    int                             collision_delegator_root_count;

    /* Global collision listeners (reference: IVP_Environment::collision_listeners).
     * Fired for ALL collision events regardless of which objects participate,
     * before per-object listeners. */
#define IVP_MAX_GLOBAL_COLLISION_LISTENERS 8
    ivp_collision_listener_t *global_collision_listeners[IVP_MAX_GLOBAL_COLLISION_LISTENERS];
    int                       global_collision_listener_count;
} ivp_environment_t;

/* ── Environment lifecycle ──────────────────────────────────────────────── */

/* Create a new environment. config may be NULL for all defaults. */
ivp_environment_t *ivp_environment_create(const ivp_environment_config_t *config);

/* Destroy environment and all objects in it. */
void ivp_environment_destroy(ivp_environment_t *env);

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

/* Create a cluster (object container hierarchy). parent may be NULL.
 * tmpl may be NULL for defaults. */
ivp_cluster_t *ivp_environment_create_cluster(ivp_environment_t *env,
                                              ivp_cluster_t *parent,
                                              const ivp_cluster_template_t *tmpl);

/* Destroy a cluster. Children are detached but NOT destroyed. */
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

/* Advance simulation by elapsed_time. Internally runs N PSI steps
 * (fixed timestep accumulation). */
void ivp_environment_simulate_dtime(ivp_environment_t *env, ivp_real_t elapsed_time);

/* Run exactly one PSI step. Exposed for fine-grained control and testing. */
void ivp_environment_simulate_one_psi(ivp_environment_t *env);

/* ── Controller API ────────────────────────────────────────────────────── */

/* Add a controller. Returns controller id (>0) on success, -1 on failure.
 * Priority follows original IVP semantics:
 *   - priority < IVP_CP_GRAVITY: synchronous push stage
 *   - priority > IVP_CP_GRAVITY: asynchronous push stage
 *   - priority == IVP_CP_GRAVITY: explicit gravity-divider stage */
ivp_controller_handle_t ivp_environment_add_controller(ivp_environment_t *env,
                                               int priority,
                                               ivp_controller_fn_t fn,
                                               void *ctx);

/* Announce a dependent controller with dynamic associated-core discovery. */
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

/* Wake-up parity helpers. */
bool ivp_environment_ensure_controller_in_simulation(ivp_environment_t *env,
                                                     ivp_controller_handle_t controller_handle);
bool ivp_environment_ensure_core_in_simulation(ivp_environment_t *env,
                                               ivp_object_t *object);

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

static inline void ivp_environment_set_gravity(ivp_environment_t *env, const ivp_vec3r_t *g) {
    env->gravity = *g;
}

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
                                             ivp_collision_delegator_root_t *root);

/* Remove a collision delegator root. Returns true if found & removed. */
bool ivp_environment_remove_collision_delegator(ivp_environment_t *env,
                                                ivp_collision_delegator_root_t *root);

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
