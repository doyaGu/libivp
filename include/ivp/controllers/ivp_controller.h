/* ivp_controller.h — Controller callback types
 *
 * Public controller concepts are declared here so they can evolve
 * independently from environment lifecycle APIs.
 */
#ifndef IVP_CONTROLLER_H
#define IVP_CONTROLLER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_forward_decls.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/events/ivp_event.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IVP controller priorities. */
typedef enum ivp_controller_priority {
    IVP_CP_NONE             = -1,
    IVP_CP_STATIC_FRICTION  = 0,
    IVP_CP_CONSTRAINTS_MIN  = 400,
    IVP_CP_CONSTRAINTS      = 405,
    IVP_CP_CONSTRAINTS_MAX  = 410,
    IVP_CP_STIFF_SPRINGS    = 460,
    IVP_CP_FLOATING         = 470,
    IVP_CP_MOTION           = 500,
    IVP_CP_DYNAMIC_FRICTION = 600,
    IVP_CP_GRAVITY          = 1000,
    IVP_CP_SPRING           = 1400,
    IVP_CP_ACTUATOR         = 1500,
    IVP_CP_FORCEFIELDS      = 1600,
    IVP_CP_ENERGY_FRICTION  = 2000
} ivp_controller_priority_t;

/* Controller callback ABI: equivalent to do_simulation_controller(event_sim, ...).
 * Receives per-PSI simulation payload including delta_time, environment pointer,
 * and the set of associated core handles for dependent controllers. */
typedef void (*ivp_controller_event_fn_t)(const ivp_event_sim_t *event_sim,
                                          ivp_object_t *object,
                                          void *ctx);

/* Canonical alias. */
typedef ivp_controller_event_fn_t ivp_controller_fn_t;

/* Dynamic associated-core discovery for dependent controllers.
 * Return value is the number of handles written to out_handles (0..max_handles),
 * or a negative value on failure. */
typedef int (*ivp_controller_discovery_fn_t)(void *ctx,
                                             ivp_environment_t *environment,
                                             ivp_object_handle_t *out_handles,
                                             int max_handles);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_H */
