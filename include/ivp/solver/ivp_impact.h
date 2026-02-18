/* ivp_impact.h — Impact and friction resolution for IVP C17
 *
 * Implements impulse-based collision response and Coulomb friction.
 *
 * Impact model (matching original IVP):
 *   1. Iterative impulse to separate objects (max IVP_IMPACT_MAX_PUSHES)
 *   2. Elasticity bounce (energy conservation with degradation)
 *   3. Rescue push for safety gap
 *
 * Friction model:
 *   - Per-contact Coulomb friction (static/dynamic)
 *   - 2D tangent plane constraint
 *   - Distance keeper to maintain gap_length near keeper_dist
 *
 * Integration with PSI loop:
 *   ivp_environment_resolve_collisions() is called after calc_next_psi_matrix.
 */
#ifndef IVP_IMPACT_H
#define IVP_IMPACT_H

#include <ivp/base/ivp_types.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/collision/ivp_collision_tolerance.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Impact resolution ──────────────────────────────────────────────────── */

/* Resolve impact for a single contact point.
 * Applies impulse to separate objects and applies elasticity bounce.
 * Returns the total impulse magnitude applied along the normal. */
ivp_real_t ivp_impact_resolve(ivp_contact_point_t *cp,
                               const ivp_collision_settings_t *settings);

/* System-aware impact resolution.
 * pushes_while_system: number of preceding impacts in the cascade.
 * Energy conservation is scaled: more cascade pushes → higher effective
 * elasticity, converging to 1.0 to prevent energy drain in multi-body
 * collisions. */
ivp_real_t ivp_impact_resolve_system(ivp_contact_point_t *cp,
                                       const ivp_collision_settings_t *settings,
                                       int pushes_while_system);

/* ── Friction resolution ────────────────────────────────────────────────── */

/* Apply Coulomb friction for a single contact point.
 * Computes tangential velocity and applies friction impulse limited by μ·N.
 * delta_time: PSI time step for distance keeper.
 * Returns the friction impulse magnitude. */
ivp_real_t ivp_friction_resolve(ivp_contact_point_t *cp,
                                 const ivp_collision_settings_t *settings,
                                 ivp_real_t delta_time);

/* ── Distance keeper ────────────────────────────────────────────────────── */

/* Apply distance keeper push to maintain gap at keeper_dist.
 * Prevents objects from interpenetrating beyond the contact surface.
 * delta_time: PSI time step. */
void ivp_distance_keeper_push(ivp_contact_point_t *cp,
                               const ivp_collision_settings_t *settings,
                               ivp_real_t delta_time);

#ifdef __cplusplus
}
#endif

#endif /* IVP_IMPACT_H */
