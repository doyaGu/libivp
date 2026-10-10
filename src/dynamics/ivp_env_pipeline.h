/* ivp_env_pipeline.h — internal state of the reference simulation pipeline
 * (the IVP_Environment members that the libivp public struct does not expose).
 */
#ifndef IVP_ENV_PIPELINE_H
#define IVP_ENV_PIPELINE_H

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_time_manager.h>
#include <ivp/collision/ivp_collision_core.h>
#include "dynamics/ivp_sim_internal.h"
#include <ivp/collision/ivp_range_manager.h>
#include "base/ivp_arena_internal.h"
#include <ivp/diagnostics/ivp_statistic_manager.h>
#include <stdatomic.h>

typedef struct ivp_cache_obj_mgr {
    ivp_cache_obj_t *cache_objects;
    int              n_cache_objects;
    int              reuse_loop_index;
    const ivp_allocator_t *allocator;
} ivp_cache_obj_mgr_t;

typedef struct ivp_env_pipeline {
    ivp_time_manager_t       time_manager;
    ivp_mindist_mgr_t        mindist_mgr;
    ivp_ov_tree_t            ov_tree;
    ivp_sim_units_manager_t  sim_units_manager;
    ivp_cache_obj_mgr_t      cache_mgr;
    /* Allocations survive nested callers and are released by the outermost
     * transaction. Environment initialization normally enables clearing; a
     * reference-internals build leaves reused spans intact. */
    ivp_arena_t              sim_unit_mem;
    /* IVP_Environment::short_term_mem: the persistent arena of the friction
     * solver and its linear constraint solver.  It preserves the reference
     * IVP_U_Memory layout; initialization normally clears reused spans. */
    ivp_arena_t              short_term_mem;

    ivp_time_t               time_of_last_psi;
    ivp_time_t               time_of_next_psi;
    ivp_time_code_t          current_time_code;
    ivp_time_code_t          mindist_event_timestamp_reference;
    short                    next_movement_check;
    double                   delta_psi_time;        /* IVP_DOUBLE delta_PSI_time    */
    double                   inv_delta_psi_time;
    double                   integrated_energy_damp;
    double                   gravity_scalar;
    ivp_env_state_t          state;
    int                      rand_seed;

    /* Runtime state that the reference keeps in process-wide statics.  It is
     * environment-owned here so independent worlds are reentrant. */
    ivp_mindist_settings_t   mindist_settings;
    ivp_mindist_t           *current_mindist;
    bool                     defer_delete_mindist;
    int                      friction_debug_counter;
    int                      global_friction_debug_counter;

    /* Protects the single-owner simulation pipeline of one environment.
     * Its private workers run only while that owner waits at a kernel barrier.
     * Different environments may hold this guard at the same time. */
    atomic_bool              simulation_running;

    /* Optional persistent pool for deterministic intra-world kernels. */
    struct ivp_task_pool    *task_pool;
    int                      parallel_worker_count;
    uint64_t                 next_object_sequence;
    uint64_t                 next_controller_sequence;
    uint64_t                 next_contact_sequence;
#if !IVP_REFERENCE_INTERNAL_STATE
    int                      live_cluster_count;
#endif
    ivp_parallel_stats_t     parallel_stats;

    ivp_pvec_t               core_revive_list;       /* ivp_core_t* */
    /* High-water scratch vectors reused by this environment's ordered PSI
     * pipeline; the per-environment simulation guard prevents overlap. */
    ivp_pvec_t               touched_cores;          /* ivp_core_t* */
    ivp_pvec_t               active_hulls;           /* ivp_hull_mgr_t* */
    ivp_pvec_t               parallel_sim_units;     /* ivp_sim_unit_t* scratch */

    /* IVP_Standard_Gravity_Controller */
    ivp_sim_ctrl_t           gravity_ctrl;
    ivp_vec3f_t              grav_vec;

    /* built-in IVP_Collision_Delegator_Root_Mindist */
    ivp_collision_delegator_root_t *root_mindist;

    /* deferred object deletion (IVP_Real_Object::begin/end_deferred_deletion) */
    int                      deferred_deletion_depth;
    bool                     deferred_deletion_draining;   /* end_deferred_deletion is deleting the queue */
    struct ivp_object       *object_being_deleted;   /* its deletion is running */
    ivp_pvec_t               deferred_deletions;     /* ivp_object_t* */

    /* libivp controller slots attached to every core (run_for_all_cores) */
    ivp_pvec_t               all_core_ctrls;         /* ivp_sim_ctrl_t* */
    /* all slot adapters (for association refresh) */
    ivp_pvec_t               slot_ctrls;

    /* IVP_Listener_Object: global list + per object tables
     * (IVP_Cluster_Manager::obj_callback_hash) */
    ivp_pvec_t               global_object_listeners; /* ivp_listener_object_t* */
    ivp_pvec_t               obj_callback_tables;     /* env_obj_cb_table_t*    */
    /* IVP_Listener_Collision: IVP_Environment::collision_listeners +
     * IVP_Cluster_Manager::collision_callback_hash */
    ivp_pvec_t               global_collision_listeners; /* ivp_collision_listener_t* */
    ivp_pvec_t               coll_callback_tables;       /* env_obj_cb_table_t*      */

    /* IVP_Environment::static_object: a static ball (radius 1, at the origin,
     * no collision detection) used for anchors not fixed to objects; it is
     * not part of the public object list */
    ivp_object_t            *static_object;

    /* IVP_Range_Manager (custom from the config, else own_range_manager) */
    ivp_range_manager_t     *range_manager;
    ivp_range_manager_t      own_range_manager;

    /* IVP_U_Active_Value_Manager (l_active_value_manager) */
    struct ivp_active_value_manager *active_value_manager;
    bool                     owns_active_value_manager;

    /* IVP_Cluster_Manager::an_object_to_be_checked (universe manager) */
    ivp_object_t            *an_object_to_be_checked;
    /* IVP_Cluster_Manager::root_cluster: the static object and every real
     * object (head insertion), walked by check_for_unused_objects */
    ivp_cluster_t            root_cluster;

    bool                     in_psi;

    /* IVP_Environment::statistic_manager (IVP_Statistic_Manager) */
    ivp_statistic_manager_t  statistic_manager;
    /* IVP_Environment::draw_vectors (IVP_Draw_Vector_Debug list) */
    ivp_draw_vector_debug_t *draw_vectors;

    bool                     discovery_all_anchors; /* see ivp_environment_discovery_reports_all_anchors */
} ivp_env_pipeline_t;

/* object pipeline (ivp_object_pipeline.c) */
ivp_result_t ivp_cache_obj_mgr_init(ivp_cache_obj_mgr_t *m,
                                    const ivp_allocator_t *allocator,
                                    int n);
void ivp_cache_obj_mgr_destroy(ivp_cache_obj_mgr_t *m);
#ifdef IVP_DEBUG_BUILD
void ivp_cache_obj_mgr_check(ivp_cache_obj_mgr_t *m); /* reference: #ifdef DEBUG */
#endif

bool ivp_object_pipeline_create(ivp_environment_t *env, ivp_object_t *obj, const ivp_object_template_t *tmpl);
void ivp_object_pipeline_clear_references(ivp_environment_t *env, ivp_object_t *obj);
void ivp_object_pipeline_destroy(ivp_environment_t *env, ivp_object_t *obj);

/* IVP_Controller_Phantom (ivp_phantom.c) */
struct ivp_controller_phantom;
void ivp_controller_phantom_destroy(struct ivp_controller_phantom *c);
void ivp_object_enable_collision_detection(ivp_object_t *obj, bool enable);
void ivp_object_unlink_contact_points(ivp_object_t *obj, bool silent);
void ivp_object_ensure_in_simulation_now(ivp_object_t *obj);
void ivp_object_set_pinned_ref(ivp_object_t *obj, bool pinned);
void ivp_object_change_unmovable_flag(ivp_object_t *obj, bool flag);
void ivp_object_beam_object_to_new_position(ivp_object_t *obj, const ivp_quatr_t *rotation_world_f_object,
                                            const ivp_vec3r_t *position_w_f_o, bool optimize_for_repeated_calls);
bool ivp_object_disable_simulation(ivp_object_t *obj);
void ivp_object_revive_nearest_objects_grow_fs(ivp_object_t *obj);

/* per-object collision listener tables (ivp_environment.c) */
int  ivp_env_add_listener_collision_private(ivp_environment_t *env, ivp_object_t *obj, struct ivp_collision_listener *l);
int  ivp_env_remove_listener_collision_private(ivp_environment_t *env, ivp_object_t *obj, struct ivp_collision_listener *l);
void ivp_env_fire_collision_private(ivp_environment_t *env, ivp_object_t *obj, int flag, const void *event);

/* environment helpers used by the object pipeline */
bool ivp_env_attach_default_controllers(ivp_environment_t *env, ivp_core_t *core);

#endif /* IVP_ENV_PIPELINE_H */
