/* ivp_env_internal.h — environment services used by the core pipeline port
 *
 * The collision/impact/friction port talks to the environment only through
 * these accessors (reference: the IVP_Environment getters used by
 * ivp_mindist*.cxx, ivp_impact.cxx, ivp_friction*.cxx, ivp_sim_unit.cxx).
 */
#ifndef IVP_ENV_INTERNAL_H
#define IVP_ENV_INTERNAL_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_time.h>
#include <ivp/base/ivp_legacy.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/diagnostics/ivp_performance_counter.h>

struct ivp_environment;
struct ivp_object;
struct ivp_core;
struct ivp_time_manager;
struct ivp_mindist_mgr;
struct ivp_ov_tree;
struct ivp_range_manager;
struct ivp_collision_delegator_root;
struct ivp_sim_units_mgr;
struct ivp_anomaly_manager;
struct ivp_anomaly_limits;
struct ivp_fr_system;
struct ivp_impact_solver_lt;
struct ivp_contact_situation_lt;
struct ivp_mindist;
struct ivp_mindist_settings;
struct ivp_sim_ctrl;
struct ivp_cluster;

/* IVP_ENV_STATE */
typedef enum ivp_env_state {
    IVP_ES_PSI,
    IVP_ES_PSI_INTEGRATOR,
    IVP_ES_PSI_HULL,
    IVP_ES_PSI_SHORT,
    IVP_ES_PSI_CRITIC,
    IVP_ES_AT
} ivp_env_state_t;

ivp_time_t      ivp_env_get_current_time(const struct ivp_environment *env);
ivp_time_t      ivp_env_get_next_psi_time(const struct ivp_environment *env);
ivp_time_t      ivp_env_get_old_time_of_last_psi(const struct ivp_environment *env);
float           ivp_env_get_delta_psi_time(const struct ivp_environment *env);
float           ivp_env_get_inv_delta_psi_time(const struct ivp_environment *env);
double          ivp_env_get_integrated_energy_damp(const struct ivp_environment *env);
ivp_time_code_t ivp_env_get_current_time_code(const struct ivp_environment *env);
#ifdef IVP_DEBUG_BUILD
void            ivp_env_invalid_time_code(struct ivp_environment *env); /* invalid current time code */
#endif
ivp_env_state_t ivp_env_get_state(const struct ivp_environment *env);
void            ivp_env_set_state(struct ivp_environment *env, ivp_env_state_t s);
ivp_time_code_t ivp_env_get_mindist_event_timestamp_reference(const struct ivp_environment *env);
void            ivp_env_inc_mindist_event_timestamp_reference(struct ivp_environment *env);
void            ivp_env_get_gravity_vec(const struct ivp_environment *env, double out[3]);
double          ivp_env_get_gravity_scalar(const struct ivp_environment *env);

const ivp_allocator_t           *ivp_env_get_allocator(const struct ivp_environment *env);
struct ivp_time_manager         *ivp_env_get_time_manager(struct ivp_environment *env);
struct ivp_mindist_mgr          *ivp_env_get_mindist_mgr(struct ivp_environment *env);
struct ivp_ov_tree              *ivp_env_get_ov_tree(struct ivp_environment *env);
struct ivp_sim_units_mgr        *ivp_env_get_sim_units_mgr(struct ivp_environment *env);
struct ivp_anomaly_limits       *ivp_env_get_anomaly_limits(struct ivp_environment *env);
const struct ivp_mindist_settings *ivp_env_get_mindist_settings(const struct ivp_environment *env);
struct ivp_mindist             *ivp_env_get_current_mindist(const struct ivp_environment *env);
void                            ivp_env_set_current_mindist(struct ivp_environment *env,
                                                           struct ivp_mindist *mindist);
bool                            ivp_env_get_defer_delete_mindist(const struct ivp_environment *env);
void                            ivp_env_set_defer_delete_mindist(struct ivp_environment *env,
                                                                bool defer_delete);
int                             ivp_env_next_friction_debug_counter(struct ivp_environment *env);
int                             ivp_env_next_global_friction_debug_counter(struct ivp_environment *env);

/* IVP_Range_Manager */
void   ivp_env_get_coll_range_intra_objects(struct ivp_environment *env, const struct ivp_object *o0,
                                            const struct ivp_object *o1, double *range0, double *range1);
double ivp_env_get_coll_range_in_world(struct ivp_environment *env, const struct ivp_object *o);

/* IVP_Collision_Filter::check_objects_for_collision_detection */
bool ivp_env_check_objects_for_collision_detection(struct ivp_environment *env,
                                                   struct ivp_object *o0, struct ivp_object *o1);

/* collision delegator roots (reference order: iterate from last to first) */
int  ivp_env_get_collision_delegator_root_count(const struct ivp_environment *env);
struct ivp_collision_delegator_root *ivp_env_get_collision_delegator_root(struct ivp_environment *env, int i);
void ivp_env_fire_object_is_removed_from_collision_detection(struct ivp_environment *env,
                                                             struct ivp_object *obj);

/* IVP_Universe_Manager (optional) */
bool ivp_env_has_universe_manager(const struct ivp_environment *env);
void ivp_env_universe_ensure_objects(struct ivp_environment *env, struct ivp_object *obj,
                                     const float center[3], double radius);

/* revive list (IVP_Environment::add_revive_core / remove_revive_core) */
bool ivp_env_add_revive_core(struct ivp_environment *env, struct ivp_core *core);
void ivp_env_remove_revive_core(struct ivp_environment *env, struct ivp_core *core);

/* IVP_Anomaly_Manager callbacks */
int  ivp_env_get_max_collision_checks_per_psi(const struct ivp_environment *env);

/* environment sim memory (sim_unit_mem transactions) */
void *ivp_env_sim_mem_alloc(struct ivp_environment *env, size_t size);
void  ivp_env_sim_mem_begin(struct ivp_environment *env);
void  ivp_env_sim_mem_end(struct ivp_environment *env);

/* IVP_Environment::get_memory_manager() / short_term_mem */
struct ivp_u_memory *ivp_env_get_short_term_mem(struct ivp_environment *env);
#ifndef IVP_CANONICAL_INTERNAL_STATE
#error "IVP_CANONICAL_INTERNAL_STATE must be supplied by the build"
#endif
static inline bool ivp_build_uses_canonical_internal_state(void)
{
    return IVP_CANONICAL_INTERNAL_STATE != 0;
}
void ivp_env_register_sim_controller(struct ivp_environment *env,
                                     struct ivp_sim_ctrl *controller);
uint64_t ivp_env_next_contact_sequence(struct ivp_environment *env);
uint64_t ivp_env_register_cluster(struct ivp_environment *env);
void ivp_env_unregister_cluster(struct ivp_environment *env,
                                uint64_t creation_sequence);
/* Restore cluster children before an engine traversal exposes their order. */
void ivp_cluster_restore_child_order(struct ivp_cluster *root);

/* IVP_Environment::get_statistic_manager / get_debug_manager
 * (debug_information) */
struct ivp_statistic_manager *ivp_env_get_statistic_manager(struct ivp_environment *env);
struct ivp_debug_manager     *ivp_env_get_debug_manager(struct ivp_environment *env);

/* IVP_Environment::add_draw_vector (IVP_IF(1): debug builds only) and
 * delete_draw_vector_debug */
void ivp_env_add_draw_vector(struct ivp_environment *env, const double start_p[3], const float dir_vec[3],
                             const char *debug_text, int color);
void ivp_env_delete_draw_vector_debug(struct ivp_environment *env);

/* ivp_rand() for this environment */
float ivp_env_rand(struct ivp_environment *env);

/* env->get_performancecounter()->start_pcount() / pcount(el) / stop_pcount() */
void ivp_env_start_pcount(struct ivp_environment *env);
void ivp_env_pcount(struct ivp_environment *env, ivp_performance_element_t el);
void ivp_env_stop_pcount(struct ivp_environment *env);

#endif /* IVP_ENV_INTERNAL_H */
