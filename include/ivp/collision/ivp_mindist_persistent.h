/* ivp_mindist_persistent.h — Persistent minimum distance tracking
 *
 * Tracks closest feature pairs between object pairs across PSI steps.
 * Uses cached Voronoi region state to avoid restarting the GJK/mindist
 * search from scratch each frame.  Features:
 *   - State machine: EXACT → COLL_RANGE → HULL_RANGE → HULL_EXPANDED
 *   - Voronoi region walking from cached closest features
 *   - Hull expansion triggers re-evaluation
 *   - Synapse events when objects approach/separate
 *
 * Ported from legacy ivp_mindist.hxx.
 */
#ifndef IVP_MINDIST_PERSISTENT_H
#define IVP_MINDIST_PERSISTENT_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_time.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/base/ivp_forward_decls.h>
#include <ivp/collision/ivp_synapse.h>
#include <ivp/geometry/ivp_compact_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define IVP_MINDIST_MAX_PAIRS       4096
#define IVP_MINDIST_RECALC_AGE       10   /* recompute exact every N PSIs */

/* ── Mindist state ─────────────────────────────────────────────────────── */

typedef enum ivp_mindist_state {
    IVP_MDIST_UNINITIALIZED = 0,
    IVP_MDIST_EXACT = 1,
    IVP_MDIST_HULL = 2,
    IVP_MDIST_HULL_RECURSIVE = 3,
    IVP_MDIST_INVALID = 4
} ivp_mindist_state_t;

/* ── Cached feature pair ───────────────────────────────────────────────── */

typedef struct ivp_mindist_feature {
    int vertex_index;    /* closest vertex on this object (-1 = none) */
    int edge_index;      /* closest edge (-1 = none)                  */
    int face_index;      /* closest face (-1 = none)                  */
} ivp_mindist_feature_t;

/* ── Persistent mindist entry ──────────────────────────────────────────── */

typedef struct ivp_mindist_entry {
    ivp_object_t *obj_a;
    ivp_object_t *obj_b;

    ivp_mindist_state_t state;
    ivp_synapse_link_t   synapse[2];

    ivp_real_t          last_distance;     /* cached distance            */
    ivp_vec3r_t         last_normal;       /* cached contact normal      */
    ivp_vec3r_t         closest_point_a;   /* closest point on A (ws)    */
    ivp_vec3r_t         closest_point_b;   /* closest point on B (ws)    */

    ivp_mindist_feature_t feature_a;       /* cached feature on A        */
    ivp_mindist_feature_t feature_b;       /* cached feature on B        */

    /* Cached closest edges from last exact computation.
     * Used as initial guess for incremental Voronoi walk instead of
     * restarting from centroid.
     * Reference: IVP_Mindist::contact_situation (ivp_mindist.hxx) */
    const ivp_compact_edge_t *last_edge_a;
    const ivp_compact_edge_t *last_edge_b;

    /* Halfspace optimization hint.
     * Records which side of the contact plane each object was on
     * at the last exact computation.  Used for fast O(1) consistency
     * check before doing a full recalc.
     * Reference: IVP_Mindist::mindist_function_hint (ivp_mindist.hxx)
     *   0 = unknown, 1 = normal side, -1 = opposite */
    int halfspace_hint;

    int  age;                              /* PSIs since last exact calc */
    ivp_time_t event_time;                 /* predicted next event time  */
    bool event_registered;
    bool active;                           /* entry in use               */

    /* Progressive collision distance tightening.
     * Reference: IVP_Mindist::coll_dist_selector (ivp_mindist_intern.hxx)
     * Starts at IVP_MAX_STEPS_FOR_COLLDIST_DECREASE-1 (largest shell),
     * decrements every ~3 event recalculations toward 0 (tightest shell).
     * get_coll_dist() returns coll_dists[coll_dist_selector]. */
    uint8_t coll_dist_selector;
    uint8_t coll_dist_recalc_count;
} ivp_mindist_entry_t;

/* ── Persistent mindist manager ────────────────────────────────────────── */

typedef struct ivp_mindist_manager {
    ivp_mindist_entry_t *entries;
    int  entry_count;
    int  entry_capacity;
    const ivp_allocator_t *alloc;
} ivp_mindist_manager_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

void ivp_mindist_manager_init(ivp_mindist_manager_t *mgr,
                                const ivp_allocator_t *alloc);

void ivp_mindist_manager_destroy(ivp_mindist_manager_t *mgr);

/* ── Operations ────────────────────────────────────────────────────────── */

/* Find or create a persistent entry for an object pair. */
ivp_mindist_entry_t *ivp_mindist_manager_get_or_create(
    ivp_mindist_manager_t *mgr,
    ivp_object_t *obj_a,
    ivp_object_t *obj_b);

/* Remove all entries involving a specific object (e.g. on destroy). */
void ivp_mindist_manager_remove_object(ivp_mindist_manager_t *mgr,
                                         ivp_object_t *obj);

/* Age all entries and invalidate stale ones. Called once per PSI. */
void ivp_mindist_manager_age_entries(ivp_mindist_manager_t *mgr);

/* Recompute exact distance for an entry (full GJK/mindist restart). */
void ivp_mindist_entry_recompute(ivp_mindist_entry_t *entry);

/* Update entry using cached Voronoi walk (fast path). */
bool ivp_mindist_entry_update_cached(ivp_mindist_entry_t *entry);

/* Mark all active entries as INVALID (e.g. hull wakeup sweep). */
void ivp_mindist_manager_invalidate_all(ivp_mindist_manager_t *mgr);

/* Mark entries touching the given object as INVALID. */
void ivp_mindist_manager_invalidate_object(ivp_mindist_manager_t *mgr,
                                           const ivp_object_t *obj);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MINDIST_PERSISTENT_H */
