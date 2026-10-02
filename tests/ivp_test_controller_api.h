/* ivp_test_controller_api.h -- Test-only controller registration hooks
 *
 * These entry points are intentionally NOT part of the public environment API.
 * They exist only for unit-test probes/custom callbacks.
 */
#ifndef IVP_TEST_CONTROLLER_API_H
#define IVP_TEST_CONTROLLER_API_H

#include <ivp/dynamics/ivp_environment.h>

#ifdef __cplusplus
extern "C" {
#endif

ivp_controller_handle_t ivp_test_environment_add_custom_controller(ivp_environment_t *env,
                                                              int priority,
                                                              ivp_controller_fn_t fn,
                                                              void *ctx);

ivp_controller_handle_t ivp_test_environment_announce_custom_dependent_controller(
    ivp_environment_t *env,
    int priority,
    ivp_controller_fn_t fn,
    void *ctx,
    ivp_controller_discovery_fn_t discovery_fn);

#ifdef __cplusplus
}
#endif

#endif /* IVP_TEST_CONTROLLER_API_H */
