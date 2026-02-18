/* ivp_universe_manager.c — LOD/streaming callback implementation
 *
 * Provides default initialization and environment integration for the
 * universe manager callback interface.
 */

#include <ivp/dynamics/ivp_universe_manager.h>
#include <ivp/dynamics/ivp_environment.h>
#include <string.h>

void ivp_universe_manager_init(ivp_universe_manager_t *mgr)
{
    memset(mgr, 0, sizeof(*mgr));
    mgr->evict_interval       = 60;
    mgr->evict_near_threshold = 10.0f;
    mgr->evict_far_threshold  = 100.0f;
}

void ivp_environment_set_universe_manager(ivp_environment_t *env,
                                          const ivp_universe_manager_t *mgr)
{
    if (!env) return;

    if (mgr) {
        if (!env->universe_manager) {
            env->universe_manager = (ivp_universe_manager_t *)
                ivp_alloc(env->allocator, sizeof(ivp_universe_manager_t));
            if (!env->universe_manager) return;
        }
        memcpy(env->universe_manager, mgr, sizeof(ivp_universe_manager_t));
    } else {
        if (env->universe_manager) {
            ivp_free(env->allocator, env->universe_manager,
                     sizeof(ivp_universe_manager_t));
            env->universe_manager = NULL;
        }
    }
}

const ivp_universe_manager_t *ivp_environment_get_universe_manager(
    const ivp_environment_t *env)
{
    return env ? env->universe_manager : NULL;
}
