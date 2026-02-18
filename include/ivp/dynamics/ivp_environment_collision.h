/* ivp_environment_collision.h — Environment-level collision orchestration
 *
 * Functions that coordinate collision detection and resolution across
 * all objects in an environment. These operate on ivp_environment_t
 * and delegate to the pure solver functions in solver/ivp_impact.h
 * and solver/ivp_contact.h.
 *
 * Separated from solver/ to keep solver headers free of dynamics/ deps.
 */
#ifndef IVP_ENVIRONMENT_COLLISION_H
#define IVP_ENVIRONMENT_COLLISION_H

#include <ivp/base/ivp_types.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/collision/ivp_collision_tolerance.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Environment-level collision resolution ─────────────────────────────── */

/* Full collision processing for one PSI step.
 * Called from ivp_environment_simulate_one_psi() after integration.
 *
 * Hard-switch parity pipeline:
 *   1. Broadphase/narrowphase populate persistent mindist entries.
 *   2. Mindist event solver predicts future feature-pair events.
 *   3. Predicted events are registered in the time-event queue.
 *   4. Due events trigger do-impact semantics (impact system).
 *   5. Friction persistence is maintained via synapse-linked contacts.
 *
 * Returns: number of event-time impacts resolved. */
int ivp_environment_resolve_collisions(ivp_environment_t *env);

/* ── Broadphase ─────────────────────────────────────────────────────────── */

/* N² broadphase with adaptive speed-based margins from range manager.
 * Uses core positions, upper_limit_radius, collision tolerance, and
 * speed-based lookahead to determine the broadphase margin per pair.
 * Fills contacts array with detected pairs.
 * Returns number of contacts found (0 if none). */
int ivp_broadphase_detect(ivp_environment_t *env,
                          const ivp_collision_settings_t *coll_settings,
                          ivp_contact_point_t *contacts,
                          int max_contacts);

/* ── Energy queries ─────────────────────────────────────────────────────── */

/* Compute kinetic energy of a single object.
 * KE = 0.5 * m * v² + 0.5 * ω · (I * ω) */
ivp_real_t ivp_object_get_kinetic_energy(const ivp_object_t *obj);

/* Compute total kinetic energy of all objects in environment. */
ivp_real_t ivp_environment_get_total_kinetic_energy(const ivp_environment_t *env);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ENVIRONMENT_COLLISION_H */
