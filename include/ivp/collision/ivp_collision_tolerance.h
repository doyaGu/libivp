/* ivp_collision_tolerance.h — Collision distance thresholds
 *
 * Replicates original IVP's set_collision_tolerance(t) formula.
 * Default tolerance: t = 0.01 (meters).
 *
 * All threshold distances are derived from a single tolerance parameter,
 * matching the original IVP exactly (ivp_mindist.cxx:52-88).
 */
#ifndef IVP_COLLISION_TOLERANCE_H
#define IVP_COLLISION_TOLERANCE_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_fast_math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ──────────────────────────────────────────────────────────── */

#define IVP_DEFAULT_COLLISION_TOLERANCE         IVP_REAL(0.01)
#define IVP_DEFAULT_GRAVITY_FOR_KEEPER_SPEED    IVP_REAL(9.81)
#define IVP_MAX_STEPS_FOR_COLLDIST_DECREASE     64
#define IVP_MAX_SPAWNED_MINDIST_COUNT           1000

/* ── Collision settings structure ───────────────────────────────────────── */

typedef struct ivp_collision_settings {
    ivp_real_t tolerance;              /* base tolerance (default 0.01m) */

    /* Derived distances (from set_collision_tolerance formula) */
    ivp_real_t real_coll_dist;         /* 0.1t — actual collision distance */
    ivp_real_t min_coll_dist;          /* real_coll_dist + 0.9t — minimum collision dist */
    ivp_real_t minimum_friction_dist;  /* min_coll_dist + 0.0t — minimum friction dist */
    ivp_real_t friction_dist;          /* minimum_friction_dist + 1.0t — friction activation */
    ivp_real_t keeper_dist;            /* friction_dist + 0.3t — tracking distance */
    ivp_real_t distance_keepers_safety;/* 0.01t — keeper safety margin */
    ivp_real_t max_distance_for_friction; /* friction_dist + 2.5t — max friction range */
    ivp_real_t max_distance_for_impact;   /* friction_dist + 20.0t — max impact range */

    /* Speed thresholds */
    ivp_real_t min_vertical_speed_at_collision; /* 2.0t */
    ivp_real_t speed_after_keeper_dist;         /* sqrt(2*(keeper-min_coll)*g) */
    ivp_real_t mindist_change_force_dist;       /* min_coll_dist * 0.1 */

    /* Gravity magnitude for friction normal-force estimate */
    ivp_real_t gravity_magnitude;

    /* Inverse delta PSI time (1/dt), set per-simulation-step for rescue
     * speed computation in the impact solver.  Legacy: env->get_inv_delta_PSI_time(). */
    ivp_real_t inv_delta_psi_time;

    /* Interpolated collision distance array */
    ivp_real_t coll_dists[IVP_MAX_STEPS_FOR_COLLDIST_DECREASE];
} ivp_collision_settings_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/* Initialize collision settings with given tolerance.
 * Replicates set_collision_tolerance(t) formula exactly. */
void ivp_collision_settings_init(ivp_collision_settings_t *settings,
                                  ivp_real_t tolerance);

/* Initialize with default tolerance (0.01). */
void ivp_collision_settings_init_default(ivp_collision_settings_t *settings);

#ifdef __cplusplus
}
#endif

#endif /* IVP_COLLISION_TOLERANCE_H */
