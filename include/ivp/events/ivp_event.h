/* ivp_event.h — Event types for IVP C17
 *
 * Defines the event enum, event struct, and listener callback type.
 * Extracted from ivp_environment.h so that the events/ subsystem can
 * reference these types without pulling in the full environment definition.
 */
#ifndef IVP_EVENT_H
#define IVP_EVENT_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/dynamics/ivp_handle.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_environment;

/* ── Event types ──────────────────────────────────────────────────────── */

typedef enum ivp_event_type {
    IVP_EVENT_SIM_UNIT_BEGIN = 0,
    IVP_EVENT_SIM_UNIT_END,
    IVP_EVENT_CORE_REVIVED,
    IVP_EVENT_CONTROLLER_ADDED,
    IVP_EVENT_CONTROLLER_REMOVED,
    IVP_EVENT_OBJECT_CREATED,
    IVP_EVENT_OBJECT_DESTROYED,
    IVP_EVENT_OBJECT_FROZEN,
    IVP_EVENT_OBJECT_UNFROZEN,
    IVP_EVENT_COLLISION_DETECTED,
    IVP_EVENT_CONSTRAINT_BROKEN          /* reference: IVP_Listener_Constraint::event_constraint_broken */
} ivp_event_type_t;

/* Per-controller simulation payload parity with legacy IVP_Event_Sim.
 * For dependent controllers, associated_core_handles contains the discovered
 * current core/object set used for this dispatch. */
typedef struct ivp_event_sim {
    ivp_real_t                    delta_time;
    ivp_real_t                    inverse_delta_time;
    struct ivp_environment       *environment;
    int                           sim_unit_index;
    int                           island_identity;
    const ivp_object_handle_t    *associated_core_handles;
    int                           associated_core_count;
} ivp_event_sim_t;

typedef struct ivp_event {
    ivp_event_type_t    type;
    ivp_time_t           time;
    ivp_real_t           dt;
    int                  sim_unit_index;
    /* Dense object index for live-object events; -1 when no stable index is available
     * (e.g. IVP_EVENT_OBJECT_DESTROYED or non-object events). */
    int                  object_index;
    int                  controller_id;
} ivp_event_t;

typedef void (*ivp_event_listener_fn_t)(void *ctx,
                                            struct ivp_environment *env,
                                            const ivp_event_t *event_data);

#ifdef __cplusplus
}
#endif

#endif /* IVP_EVENT_H */
