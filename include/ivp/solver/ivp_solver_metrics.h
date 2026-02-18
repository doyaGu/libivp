/* ivp_solver_metrics.h -- Per-PSI solver metrics accumulator
 *
 * Lightweight optional instrumentation.  Uses the same active-pointer
 * pattern as ivp_solver_config: set before PSI, NULL in production
 * (zero overhead when not active).
 */
#ifndef IVP_SOLVER_METRICS_H
#define IVP_SOLVER_METRICS_H

#include <ivp/base/ivp_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Metrics struct ──────────────────────────────────────────────────── */

typedef struct ivp_solver_metrics {
    /* Unilateral solver */
    int        unilateral_deactivation_steps;

    /* PGS solver */
    int        pgs_iterations_used;
    ivp_real_t pgs_final_residual;

    /* Impact */
    int        impact_push_count;
    int        impact_system_pushes;

    /* Penetration */
    ivp_real_t max_penetration_depth;
    ivp_real_t avg_penetration_depth;
    int        penetration_count;

    /* Energy */
    ivp_real_t total_kinetic_energy;
    ivp_real_t energy_drift_percent;

    /* Stability */
    ivp_real_t max_position_jitter;
    int        contact_count;
} ivp_solver_metrics_t;

/* ── Lifecycle ────────────────────────────────────────────────────────── */

/* Zero all fields. */
void ivp_solver_metrics_init(ivp_solver_metrics_t *m);

/* ── Active-pointer (same pattern as solver_config) ───────────────────── */

void ivp_solver_metrics_set_active(ivp_solver_metrics_t *m);
ivp_solver_metrics_t *ivp_solver_metrics_get_active(void);

#ifdef __cplusplus
}
#endif

#endif /* IVP_SOLVER_METRICS_H */
