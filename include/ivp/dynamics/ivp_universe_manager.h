/* ivp_universe_manager.h — LOD/streaming callback interface
 *
 * Callback-based interface for managing large scenes with LOD streaming.
 * The environment calls ensure_fn for each moving object during broadphase
 * and evict_fn for objects with zero collision references at configurable
 * intervals.
 *
 * Ported from ivp_collision/ivp_universe_manager.hxx.
 */
#ifndef IVP_UNIVERSE_MANAGER_H
#define IVP_UNIVERSE_MANAGER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Callback types ────────────────────────────────────────────────────── */

/* Called for each moving object during broadphase.
 * Use to ensure detail geometry is loaded for nearby objects.
 * sphere_center/radius describe the object's bounding sphere. */
typedef void (*ivp_universe_ensure_fn_t)(void *ctx,
                                         ivp_object_t *object,
                                         const float *sphere_center,
                                         float sphere_radius);

/* Called periodically for objects with zero collision references.
 * Use to evict/unload detail geometry for distant objects. */
typedef void (*ivp_universe_evict_fn_t)(void *ctx,
                                        ivp_object_t *object);

/* Called when an object is about to be destroyed.
 * Use to clean up any streaming resources associated with this object. */
typedef void (*ivp_universe_deleted_fn_t)(void *ctx,
                                          ivp_object_t *object);

/* ── Universe manager ──────────────────────────────────────────────────── */

typedef struct ivp_universe_manager {
    ivp_universe_ensure_fn_t  ensure_fn;
    ivp_universe_evict_fn_t   evict_fn;
    ivp_universe_deleted_fn_t deleted_fn;
    void                     *ctx;

    /* Eviction timing: evict_fn is called for eligible objects every
     * evict_interval PSI steps. Two tiers allow different rates for
     * near vs. far objects. */
    int   evict_interval;         /* PSI steps between eviction sweeps, default: 60 */
    float evict_near_threshold;   /* objects closer than this are never evicted */
    float evict_far_threshold;    /* objects farther than this are evicted first */

    /* Internal counter */
    int   psi_counter;
} ivp_universe_manager_t;

/* Fill with defaults (all callbacks NULL, interval 60). */
void ivp_universe_manager_init(ivp_universe_manager_t *mgr);

/* Check if any callbacks are installed. */
static inline bool ivp_universe_manager_is_active(const ivp_universe_manager_t *mgr) {
    return mgr && (mgr->ensure_fn || mgr->evict_fn || mgr->deleted_fn);
}

/* ── Environment integration ───────────────────────────────────────────── */

/* Set the universe manager on an environment. Pass NULL to disable.
 * The manager struct is copied; the original need not persist. */
void ivp_environment_set_universe_manager(ivp_environment_t *env,
                                          const ivp_universe_manager_t *mgr);

/* Get the current universe manager (NULL if none). */
const ivp_universe_manager_t *ivp_environment_get_universe_manager(
    const ivp_environment_t *env);

#ifdef __cplusplus
}
#endif

#endif /* IVP_UNIVERSE_MANAGER_H */
