/* ivp_universe_manager.h — IVP_Universe_Manager (ivp_universe_manager.hxx)
 *
 * Callback-based interface for managing large scenes:
 *   ensure_fn  - IVP_Universe_Manager::ensure_objects_in_environment, called
 *                when an object that can collide (re)enters the OV tree
 *                (IVP_Mindist_Manager::recheck_ov_element) with the sphere
 *                inside which all objects should be in the environment;
 *   evict_fn   - IVP_Universe_Manager::object_no_longer_needed, called at the
 *                start of each PSI (IVP_Cluster_Manager::check_for_unused_objects)
 *                for objects nobody checks for collision
 *                (ivp_object_get_collision_check_reference_count() == 0, or
 *                == 1 above the second object count threshold);
 *   deleted_fn - IVP_Universe_Manager::event_object_deleted.
 * The settings are IVP_Universe_Manager_Settings
 * (provide_universe_settings()).
 */
#ifndef IVP_UNIVERSE_MANAGER_H
#define IVP_UNIVERSE_MANAGER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Callback types ────────────────────────────────────────────────────── */

/* IVP_Universe_Manager::ensure_objects_in_environment: called when an object
 * that can collide (re)enters the OV tree, with the sphere (float center,
 * IVP_DOUBLE radius) inside which all objects it may collide with should be
 * in the environment.  Objects created here are inserted with a minimal
 * radius and rechecked as soon as possible (no recursive calls). */
typedef void (*ivp_universe_ensure_fn_t)(void *ctx,
                                         ivp_object_t *object,
                                         const float *sphere_center,
                                         double sphere_radius);

/* IVP_Universe_Manager::object_no_longer_needed: called at the start of a
 * PSI for objects nobody checks for collision; the objects are visited in
 * cluster tree order (ivp_environment_get_root_cluster), a limited number
 * per PSI, resuming where the previous PSI stopped.  The callback may
 * delete the object. */
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

    /* IVP_Universe_Manager_Settings */
    int num_objects_in_environment_threshold_0; /* default 1: fewer objects -> no evict_fn calls       */
    int check_objects_per_second_threshold_0;   /* default 1: objects checked per second (ref count 0) */
    int num_objects_in_environment_threshold_1; /* default 1000000: above, also check ref count 1      */
    int check_objects_per_second_threshold_1;   /* default 10                                          */
    bool provide_settings;                      /* default true; false = provide_universe_settings() returns NULL */
} ivp_universe_manager_t;

/* IVP_Real_Object::get_collision_check_reference_count: number of objects
 * checking this object for collision, -1 when collision detection is off. */
int ivp_object_get_collision_check_reference_count(const ivp_object_t *obj);

/* Fill with defaults (all callbacks NULL, IVP_Universe_Manager_Settings defaults). */
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
