/* ivp_mindist_friction_update.h — Synapse-driven friction contact refresh */
#ifndef IVP_MINDIST_FRICTION_UPDATE_H
#define IVP_MINDIST_FRICTION_UPDATE_H

#include <ivp/base/ivp_types.h>
#include <ivp/solver/ivp_friction_system.h>
#include <ivp/collision/ivp_collision_tolerance.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Refresh geometry and integrate span_friction_s by velocity.
 * dt = time since last refresh (typically delta_psi_time).
 * Pass dt=0 to skip velocity integration (geometry-only refresh). */
bool ivp_mindist_friction_refresh_contact(ivp_persistent_contact_t *pc,
                                          const ivp_collision_settings_t *settings,
                                          ivp_real_t dt);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MINDIST_FRICTION_UPDATE_H */
