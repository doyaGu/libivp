/* ivp_contact_order.h -- Shock propagation contact ordering
 *
 * Sorts contacts bottom-up by gravity-projected height for improved
 * stacking stability. Only used when enable_shock_propagation is true.
 */
#ifndef IVP_CONTACT_ORDER_H
#define IVP_CONTACT_ORDER_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/solver/ivp_friction_system.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Sort contacts bottom-up by gravity-projected height.
 * gravity_dir should be the normalized gravity direction (e.g., (0,1,0) for Y-down).
 * Returns the number of contacts sorted (same as count on success). */
int ivp_contact_order_shock_propagation(ivp_persistent_contact_t **contacts,
                                         int count,
                                         const ivp_vec3r_t *gravity_dir);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTACT_ORDER_H */
