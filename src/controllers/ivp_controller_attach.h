/* ivp_controller_attach.h -- internal helpers for IVP_Controller_Independent
 * style controllers that are attached to explicit cores
 * (IVP_Controller_Manager::add_controller_to_core / remove_controller_from_core):
 * ivp_environment_create_core_controller registers the controller with no
 * core, ivp_environment_add_controller_to_core / remove_controller_from_core
 * manage its cores.
 */
#ifndef IVP_CONTROLLER_ATTACH_INTERNAL_H
#define IVP_CONTROLLER_ATTACH_INTERNAL_H

#include <ivp/dynamics/ivp_environment.h>

/* Register fn/ctx as a controller attached to no core (priority: the
 * built-in priority of fn, else the given one). */
static inline ivp_controller_handle_t ivp_ctrl_attach_create(ivp_environment_t *env, int priority,
                                                             ivp_controller_fn_t fn, void *ctx)
{
    return ivp_environment_create_core_controller(env, priority, fn, ctx);
}

#endif /* IVP_CONTROLLER_ATTACH_INTERNAL_H */
