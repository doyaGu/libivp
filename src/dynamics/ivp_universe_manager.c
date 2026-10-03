/* ivp_universe_manager.c — LOD/streaming callback implementation
 *
 * Provides default initialization and environment integration for the
 * universe manager callback interface.
 */

#include <ivp/dynamics/ivp_universe_manager.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/collision/ivp_collision_core.h>
#include <string.h>

int ivp_object_get_collision_check_reference_count(const ivp_object_t *obj)
{
    const ivp_ov_element_t *ov = obj->ov_element;
    if (!ov) return -1; /* reference count for non colliding objects */
    return ov->collision_fvector.n;
}

void ivp_universe_manager_init(ivp_universe_manager_t *mgr)
{
    memset(mgr, 0, sizeof(*mgr));
    /* IVP_Universe_Manager_Settings::IVP_Universe_Manager_Settings */
    mgr->num_objects_in_environment_threshold_0 = 1;
    mgr->check_objects_per_second_threshold_0 = 1;
    mgr->num_objects_in_environment_threshold_1 = 1000000;
    mgr->check_objects_per_second_threshold_1 = 10;
    mgr->provide_settings = true;
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
