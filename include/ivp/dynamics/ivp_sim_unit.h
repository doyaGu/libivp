/* ivp_sim_unit.h -- Simulation unit for grouping interacting objects
 *
 * A simulation unit groups objects that interact with each other
 * (via contacts, constraints, or proximity). Objects in the same
 * sim unit are integrated and solved together.
 *
 * The original IVP uses connected components of the contact graph
 * to form sim units dynamically. This C17 version provides a simpler
 * static grouping with object-to-unit assignment.
 *
 * Ported from ivp_sim_unit.hxx.
 */
#ifndef IVP_SIM_UNIT_H
#define IVP_SIM_UNIT_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Sim unit constants ─────────────────────────────────────────────────── */

#define IVP_SIM_UNIT_DEFAULT    0     /* all objects start in unit 0 */
#define IVP_SIM_UNIT_MAX       255   /* maximum unit index */

/* ── Sim unit info (per-object) ─────────────────────────────────────────── */

/* Each object carries a sim_unit_id that determines which simulation
 * group it belongs to. Objects in different groups do not collide
 * or interact with each other during a single PSI substep.
 *
 * Usage patterns:
 *   - Single sim unit (default): all objects interact, simple case.
 *   - Multiple sim units: partition objects by region/type for
 *     independent solving (e.g., separate rooms, distant groups).
 *   - Wake/sleep propagation: when an object is woken, all objects
 *     in its sim unit should be checked for interaction. */

/* ── Sim unit API (object-level) ────────────────────────────────────────── */

/* Get the sim unit ID of an object.
 * Requires ivp_object.h to be included before this header. */
struct ivp_object;
int ivp_object_get_sim_unit_id(const struct ivp_object *obj);

/* Set the sim unit ID of an object. Returns IVP_ERR_INVALID_ARG if
 * unit_id is out of range [0, IVP_SIM_UNIT_MAX]. */
ivp_result_t ivp_object_set_sim_unit(struct ivp_object *obj, int unit_id);

#ifdef __cplusplus
}
#endif

#endif /* IVP_SIM_UNIT_H */
