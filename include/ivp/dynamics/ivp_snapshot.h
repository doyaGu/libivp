/* ivp_snapshot.h — World snapshots at PSI boundaries (libivp extension)
 *
 * A snapshot captures a world between two simulate calls so that
 * ivp_world_snapshot_restore can build a new world that continues bit for
 * bit like the original would have.  Coverage grows in steps; the
 * continuation test (tests/test_snapshot.c) records which scenes already
 * continue exactly.
 *
 * Captured: the time manager clock and its queue, the engine sequences and
 * movement-check state, and for every object its collision settings, its
 * core (mass properties, pose, velocities, movement and calm state) and its
 * collision state: hull manager, OV element and node, every mindist with
 * its synapses, lists and time event.  Simulation units keep their cores,
 * flags and controller lists, and friction keeps every system, contact
 * point, core pair and per-core friction info with all their lists.
 * Springs (ivp_controller_spring) and constraints (ivp_controller_constraint)
 * keep a copy of their state; the caller provides new storage for it at
 * restore (see ivp_world_snapshot_bindings_t), like at registration.
 * Restore rebuilds these structures as they were instead of running the
 * creation logic, so a restored world continues bit for bit.  That holds
 * for the normal build; a reference-internals build
 * (IVP_REFERENCE_INTERNAL_STATE) keeps address- and history-dependent state,
 * such as reused arena bytes and simulation-unit list order, that a
 * snapshot does not carry.
 *
 * capture returns NULL with IVP_ERR_INVALID_STATE, rather than dropping
 * state, while the world holds anything not captured yet: recursive
 * mindists (objects with several convex ledges), OO watchers, phantoms,
 * controllers other than gravity, friction, springs and constraints
 * (including user controllers), springs bound to active values, merged or
 * attached cores, car wheels, sub-clusters or extra collision delegator
 * roots.  Capturing never changes the world.
 *
 * Built only with IVP_ENABLE_SNAPSHOTS (CMake option, default off).
 *
 * Geometry is referenced, not copied: the surface managers, names, client
 * data and collision group strings of the captured objects must outlive the
 * snapshot and every world restored from it.
 */
#ifndef IVP_SNAPSHOT_H
#define IVP_SNAPSHOT_H

#include <ivp/base/ivp_result.h>
#include <ivp/dynamics/ivp_environment.h>

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_world_snapshot ivp_world_snapshot_t;

/* Controller state storage for a restored world.  Controllers are listed
 * in registration order; states[i] must point to storage of the
 * controller's state type (ivp_world_snapshot_controller_state_size bytes,
 * e.g. an ivp_spring_state_t) that outlives the restored world.  Restore
 * copies the captured state there, points it at the restored objects and
 * registers the controller; broken-event callbacks are kept as captured.
 * handles may be NULL; otherwise it receives the new controller handles. */
typedef struct ivp_world_snapshot_bindings {
    void *const *controller_states;
    ivp_controller_handle_t *controller_handles;
    int controller_count;
} ivp_world_snapshot_bindings_t;

/* Capture env, which must not be inside a simulate call.  Returns NULL and
 * sets *result (if non-NULL) on failure; IVP_ERR_INVALID_STATE marks state
 * the snapshot does not support yet. */
ivp_world_snapshot_t *ivp_world_snapshot_capture(const ivp_environment_t *env,
                                                 ivp_result_t *result);

/* Build a new world from snapshot.  config must be the configuration the
 * captured world was created with; the allocator and the parallel worker
 * count may differ.  bindings may be NULL when the snapshot holds no
 * controllers.  Restored objects keep their creation sequence
 * (ivp_object_order_key) and are listed in creation order.  Returns NULL
 * and sets *result (if non-NULL) on failure. */
ivp_environment_t *ivp_world_snapshot_restore(
    const ivp_world_snapshot_t *snapshot,
    const ivp_environment_config_t *config,
    const ivp_world_snapshot_bindings_t *bindings,
    ivp_result_t *result);

/* Number of objects in the snapshot (the static object excluded). */
int ivp_world_snapshot_object_count(const ivp_world_snapshot_t *snapshot);

/* Controllers in registration order: their callback and state size. */
int ivp_world_snapshot_controller_count(const ivp_world_snapshot_t *snapshot);
ivp_controller_fn_t ivp_world_snapshot_controller_fn(
    const ivp_world_snapshot_t *snapshot, int index);
size_t ivp_world_snapshot_controller_state_size(
    const ivp_world_snapshot_t *snapshot, int index);

void ivp_world_snapshot_destroy(ivp_world_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* IVP_SNAPSHOT_H */
