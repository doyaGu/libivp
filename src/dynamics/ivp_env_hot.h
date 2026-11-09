/* ivp_env_hot.h -- private direct accessors for the single-threaded pipeline.
 *
 * The exported ivp_env_* functions remain the ABI.  Core translation units
 * include this after ivp_env_internal.h so hot reads can be inlined without
 * changing call-site expressions or floating-point evaluation order.
 */
#ifndef IVP_ENV_HOT_H
#define IVP_ENV_HOT_H

#include "dynamics/ivp_env_pipeline.h"

static inline ivp_time_t ivp_env_hot_current_time(const ivp_environment_t *env)
{
    return env->current_time;
}

static inline ivp_time_t ivp_env_hot_next_psi_time(const ivp_environment_t *env)
{
    return env->pl->time_of_next_psi;
}

static inline ivp_time_t ivp_env_hot_old_time_of_last_psi(const ivp_environment_t *env)
{
    return env->pl->time_of_last_psi;
}

/* IVP_Environment::get_delta_PSI_time / get_inv_delta_PSI_time return the
 * IVP_DOUBLE members (retail; the later SDK rounded them to float) */
static inline double ivp_env_hot_delta_psi_time(const ivp_environment_t *env)
{
    return env->pl->delta_psi_time;
}

static inline double ivp_env_hot_inv_delta_psi_time(const ivp_environment_t *env)
{
    return env->pl->inv_delta_psi_time;
}

static inline double ivp_env_hot_integrated_energy_damp(const ivp_environment_t *env)
{
    return env->pl->integrated_energy_damp;
}

static inline ivp_time_code_t ivp_env_hot_current_time_code(const ivp_environment_t *env)
{
    return env->pl->current_time_code;
}

static inline ivp_env_state_t ivp_env_hot_state(const ivp_environment_t *env)
{
    return env->pl->state;
}

static inline const ivp_allocator_t *ivp_env_hot_allocator(const ivp_environment_t *env)
{
    return env->allocator;
}

static inline ivp_time_manager_t *ivp_env_hot_time_manager(ivp_environment_t *env)
{
    return &env->pl->time_manager;
}

static inline ivp_mindist_mgr_t *ivp_env_hot_mindist_mgr(ivp_environment_t *env)
{
    return &env->pl->mindist_mgr;
}

static inline ivp_ov_tree_t *ivp_env_hot_ov_tree(ivp_environment_t *env)
{
    return &env->pl->ov_tree;
}

static inline ivp_sim_units_manager_t *ivp_env_hot_sim_units_manager(ivp_environment_t *env)
{
    return &env->pl->sim_units_manager;
}

static inline ivp_arena_t *ivp_env_hot_short_term_mem(ivp_environment_t *env)
{
    return &env->pl->short_term_mem;
}

static inline ivp_statistic_manager_t *ivp_env_hot_statistic_manager(ivp_environment_t *env)
{
    return &env->pl->statistic_manager;
}

static inline struct ivp_debug_manager *ivp_env_hot_debug_manager(ivp_environment_t *env)
{
    return env->debug_manager;
}

static inline int ivp_env_hot_collision_root_count(const ivp_environment_t *env)
{
    return env->collision_delegator_root_count;
}

static inline ivp_collision_delegator_root_t *ivp_env_hot_collision_root(ivp_environment_t *env, int i)
{
    return env->collision_delegator_roots[i];
}

static inline bool ivp_env_hot_has_universe_manager(const ivp_environment_t *env)
{
    return env->universe_manager != NULL;
}

static inline void ivp_env_hot_start_pcount(ivp_environment_t *env)
{
    if (env && env->performance_counter) {
        ivp_performance_counter_start_pcount(env->performance_counter);
    }
}

static inline void ivp_env_hot_pcount(ivp_environment_t *env, ivp_performance_element_t element)
{
    if (env && env->performance_counter) {
        ivp_performance_counter_pcount(env->performance_counter, element);
    }
}

static inline void ivp_env_hot_stop_pcount(ivp_environment_t *env)
{
    if (env && env->performance_counter) {
        ivp_performance_counter_stop_pcount(env->performance_counter);
    }
}

#define ivp_env_get_current_time(env) ivp_env_hot_current_time(env)
#define ivp_env_get_next_psi_time(env) ivp_env_hot_next_psi_time(env)
#define ivp_env_get_old_time_of_last_psi(env) ivp_env_hot_old_time_of_last_psi(env)
#define ivp_env_get_delta_psi_time(env) ivp_env_hot_delta_psi_time(env)
#define ivp_env_get_inv_delta_psi_time(env) ivp_env_hot_inv_delta_psi_time(env)
#define ivp_env_get_integrated_energy_damp(env) ivp_env_hot_integrated_energy_damp(env)
#define ivp_env_get_current_time_code(env) ivp_env_hot_current_time_code(env)
#define ivp_env_get_state(env) ivp_env_hot_state(env)
#define ivp_env_get_allocator(env) ivp_env_hot_allocator(env)
#define ivp_env_get_time_manager(env) ivp_env_hot_time_manager(env)
#define ivp_env_get_mindist_mgr(env) ivp_env_hot_mindist_mgr(env)
#define ivp_env_get_ov_tree(env) ivp_env_hot_ov_tree(env)
#define ivp_env_get_sim_units_manager(env) ivp_env_hot_sim_units_manager(env)
#define ivp_env_get_short_term_mem(env) ivp_env_hot_short_term_mem(env)
#define ivp_env_get_statistic_manager(env) ivp_env_hot_statistic_manager(env)
#define ivp_env_get_debug_manager(env) ivp_env_hot_debug_manager(env)
#define ivp_env_get_collision_delegator_root_count(env) ivp_env_hot_collision_root_count(env)
#define ivp_env_get_collision_delegator_root(env, i) ivp_env_hot_collision_root((env), (i))
#define ivp_env_has_universe_manager(env) ivp_env_hot_has_universe_manager(env)
#define ivp_env_start_pcount(env) ivp_env_hot_start_pcount(env)
#define ivp_env_pcount(env, element) ivp_env_hot_pcount((env), (element))
#define ivp_env_stop_pcount(env) ivp_env_hot_stop_pcount(env)

#endif /* IVP_ENV_HOT_H */
