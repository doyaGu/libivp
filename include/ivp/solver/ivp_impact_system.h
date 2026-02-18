/* ivp_impact_system.h — Multi-body impact cascade system
 *
 * Cascades impulses through connected contacts. When one contact is resolved,
 * velocity changes may cause new collisions in connected contacts.
 * Iterates until stable or push limit reached.
 *
 * Ported from IVP_Impact_System.
 */
#ifndef IVP_IMPACT_SYSTEM_H
#define IVP_IMPACT_SYSTEM_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_forward_decls.h>
#include <ivp/containers/ivp_array.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/solver/ivp_friction_system.h>
#include <ivp/collision/ivp_collision_tolerance.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Constants IVP_MAXIMUM_SYSTEM_PUSH_NUMBER and IVP_SPEED_ADDON_SYSTEM_IMPACT
 * are defined in ivp_solver_constants.h (included via ivp_contact.h). */

/* ── Impact system ─────────────────────────────────────────────────────── */

typedef struct ivp_impact_system {
    int sum_of_pushes;
    ivp_real_t start_contact_impulse_sum;

    ivp_array_t pushed_cores;
    ivp_array_t known_cores;
    ivp_array_t pairs;

    const ivp_allocator_t *alloc;
} ivp_impact_system_t;

/* ── API ───────────────────────────────────────────────────────────────── */

void ivp_impact_system_init(ivp_impact_system_t *sys,
                              const ivp_allocator_t *alloc);
void ivp_impact_system_destroy(ivp_impact_system_t *sys);

/* Cascade solver: resolve impacts propagating through friction core pairs.
 * Starts from start_contact and cascades through connected pairs in the
 * friction system. */
void ivp_impact_system_solve(ivp_impact_system_t *sys,
                               ivp_persistent_contact_t *start_contact,
                               ivp_friction_system_t *fs,
                               ivp_friction_core_pair_t *start_pair,
                               const ivp_collision_settings_t *settings);

#ifdef __cplusplus
}
#endif

#endif /* IVP_IMPACT_SYSTEM_H */
