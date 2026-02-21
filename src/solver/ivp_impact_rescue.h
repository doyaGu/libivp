#ifndef IVP_IMPACT_RESCUE_H
#define IVP_IMPACT_RESCUE_H

#include <ivp/base/ivp_types.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/collision/ivp_collision_tolerance.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reference parity: rotational uncertainty term used by rescue speed. */
ivp_real_t ivp_impact_rot_speed_uncertainty(const ivp_contact_point_t *cp,
                                            const ivp_collision_settings_t *settings);

/* Reference parity: 2 * (penetration_speed + rot_uncertainty). */
ivp_real_t ivp_impact_rescue_speed_addon(const ivp_contact_point_t *cp,
                                         const ivp_collision_settings_t *settings);

/* Target separation speed used by do_rescue_push:
 * (min_vertical_speed_at_collision + rescue_addon) * safety_factor. */
ivp_real_t ivp_impact_rescue_speed_target(const ivp_contact_point_t *cp,
                                          const ivp_collision_settings_t *settings);

/* Delay decision threshold speed:
 * rescue_speed_target / safety_factor = min_vertical + rescue_addon. */
ivp_real_t ivp_impact_delay_threshold_speed(const ivp_contact_point_t *cp,
                                            const ivp_collision_settings_t *settings);

#ifdef __cplusplus
}
#endif

#endif /* IVP_IMPACT_RESCUE_H */
