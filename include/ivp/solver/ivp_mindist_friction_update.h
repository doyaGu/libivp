/* ivp_mindist_friction_update.h — Synapse-driven friction contact refresh */
#ifndef IVP_MINDIST_FRICTION_UPDATE_H
#define IVP_MINDIST_FRICTION_UPDATE_H

#include <ivp/base/ivp_types.h>
#include <ivp/solver/ivp_friction_system.h>
#include <ivp/collision/ivp_collision_tolerance.h>

#ifdef __cplusplus
extern "C" {
#endif

bool ivp_mindist_friction_refresh_contact(ivp_persistent_contact_t *pc,
                                          const ivp_collision_settings_t *settings);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MINDIST_FRICTION_UPDATE_H */
