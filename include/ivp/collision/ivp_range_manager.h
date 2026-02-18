/* ivp_range_manager.h — Collision range prediction
 *
 * Implements speed-based adaptive collision range computation matching
 * the original IVP_Range_Manager. Controls how far ahead the broadphase
 * looks for potential collisions based on object velocities and radii.
 *
 * Two range modes:
 *   - Intra-object: range between two dynamic objects
 *   - World: range for a single object against static world geometry
 *
 * Ported from ivp_range_manager.hxx / ivp_range_manager.cxx.
 */
#ifndef IVP_RANGE_MANAGER_H
#define IVP_RANGE_MANAGER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Range manager parameters ──────────────────────────────────────────── */

typedef struct ivp_range_manager {
    /* Object vs object (intra) parameters */
    ivp_real_t look_ahead_time_intra;           /* [s] lookahead time window         */
    ivp_real_t look_ahead_max_radius_intra;     /* factor of smaller object radius   */
    ivp_real_t look_ahead_min_distance_intra;   /* [m] minimum range                 */
    ivp_real_t look_ahead_max_distance_intra;   /* [m] maximum range                 */
    ivp_real_t look_ahead_min_seconds_intra;    /* [s] minimum time-based range      */

    /* Object vs world parameters */
    ivp_real_t look_ahead_time_world;           /* [s] lookahead time window         */
    ivp_real_t look_ahead_max_radius_world;     /* factor of object radius           */
    ivp_real_t look_ahead_min_distance_world;   /* [m] minimum range                 */
    ivp_real_t look_ahead_max_distance_world;   /* [m] maximum range                 */
    ivp_real_t look_ahead_min_seconds_world;    /* [s] minimum time-based range      */

    /* Environment delta PSI time */
    ivp_real_t delta_psi_time;                  /* [s] environment timestep           */
} ivp_range_manager_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Initialize with default parameters matching original IVP. */
void ivp_range_manager_init(ivp_range_manager_t *rm, ivp_real_t delta_psi_time);

/* ── Range computation ─────────────────────────────────────────────────── */

/* Compute collision range for two dynamic objects.
 * Returns per-object ranges in range0 and range1.
 * Total margin = range0 + range1. */
void ivp_range_manager_get_coll_range_intra(
    const ivp_range_manager_t *rm,
    const ivp_core_t *core0,
    const ivp_core_t *core1,
    ivp_real_t *range0,
    ivp_real_t *range1);

/* Compute collision range for a single object against world geometry.
 * Returns the search radius for the object. */
ivp_real_t ivp_range_manager_get_coll_range_world(
    const ivp_range_manager_t *rm,
    const ivp_core_t *core0);

/* Compute total broadphase margin for an object pair.
 * Convenience wrapper: returns range0 + range1. */
ivp_real_t ivp_range_manager_get_pair_margin(
    const ivp_range_manager_t *rm,
    const ivp_core_t *core0,
    const ivp_core_t *core1);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RANGE_MANAGER_H */
