/* ivp_hull_manager.h — Hull expansion tracking
 *
 * Tracks bounding sphere expansion over time for each object.
 * When an object's hull has expanded significantly (from fast movement
 * or rotation), nearby mindist entries are invalidated for re-evaluation.
 *
 * Ported from legacy ivp_hull_manager.hxx.
 */
#ifndef IVP_HULL_MANAGER_H
#define IVP_HULL_MANAGER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Hull entry ────────────────────────────────────────────────────────── */

typedef struct ivp_hull_entry {
    ivp_object_t  *obj;
    ivp_vec3r_t    last_checked_pos;   /* position at last hull check    */
    ivp_real_t     accumulated_travel; /* distance traveled since check  */
    ivp_real_t     hull_radius;        /* current expanded hull radius   */
    ivp_real_t     base_radius;        /* upper_limit_radius at start    */
    bool           active;
} ivp_hull_entry_t;

/* ── Hull manager ──────────────────────────────────────────────────────── */

typedef struct ivp_hull_manager {
    ivp_hull_entry_t *entries;
    int  entry_count;
    int  entry_capacity;
    const ivp_allocator_t *alloc;
} ivp_hull_manager_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

void ivp_hull_manager_init(ivp_hull_manager_t *mgr,
                             const ivp_allocator_t *alloc);

void ivp_hull_manager_destroy(ivp_hull_manager_t *mgr);

/* ── Operations ────────────────────────────────────────────────────────── */

/* Register an object for hull tracking. */
void ivp_hull_manager_add_object(ivp_hull_manager_t *mgr,
                                   ivp_object_t *obj);

/* Remove an object from hull tracking. */
void ivp_hull_manager_remove_object(ivp_hull_manager_t *mgr,
                                      ivp_object_t *obj);

/* Update hull expansion for all tracked objects. Call once per PSI.
 * Returns the number of objects whose hulls have expanded significantly. */
int ivp_hull_manager_update(ivp_hull_manager_t *mgr, ivp_real_t dt);

/* Collect objects whose accumulated hull expansion exceeds threshold.
 * Returns total expanded count. If out_objects is NULL, only returns count. */
int ivp_hull_manager_collect_expanded_objects(const ivp_hull_manager_t *mgr,
                                              ivp_object_t **out_objects,
                                              int max_objects);

/* Get current hull radius for an object (base_radius + expansion).
 * Returns base radius if object is not tracked. */
ivp_real_t ivp_hull_manager_get_radius(const ivp_hull_manager_t *mgr,
                                         const ivp_object_t *obj);

/* Reset hull expansion for an object (after mindist recompute). */
void ivp_hull_manager_reset(ivp_hull_manager_t *mgr,
                              ivp_object_t *obj);

#ifdef __cplusplus
}
#endif

#endif /* IVP_HULL_MANAGER_H */
