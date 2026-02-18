/* ivp_solver_config.h -- Runtime solver configuration
 *
 * All solver tuning parameters as a runtime struct, with defaults
 * matching the original #define constants.  Enables per-environment
 * tuning and opt-in solver enhancements without changing function
 * signatures (uses file-scope active-config pointer pattern).
 */
#ifndef IVP_SOLVER_CONFIG_H
#define IVP_SOLVER_CONFIG_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Solver configuration struct ─────────────────────────────────────── */

typedef struct ivp_solver_config {
    /* Matrix solver */
    ivp_real_t matrix_eps;
    ivp_real_t static_friction_eps;
    int        unilateral_max_steps;

    /* Impact */
    ivp_real_t impact_eps;
    int        impact_max_pushes;
    ivp_real_t impact_initial_fraction;
    int        max_system_pushes;

    /* Friction */
    int        max_friction_matrix;
    int        pgs_base_iterations;
    int        pgs_max_iterations;
    ivp_real_t pgs_residual_eps;
    ivp_real_t pgs_sor_omega;
    int        ease_every_nth_psi;
    int        slowly_turn_on_keeper;
    ivp_real_t max_energy_destroy;

    /* Contact limits */
    int        max_contacts_per_pair;
    int        max_contact_pairs;

    /* Opt-in enhancements (all default false / no-op) */
    bool       enable_shock_propagation;
    bool       enable_post_stabilization;
    ivp_real_t post_stabilize_factor;
    ivp_real_t post_stabilize_max_impulse;
    bool       enable_adaptive_pgs;
    bool       enable_warm_start_impulse;

    /* Cached gravity direction (normalized), set at PSI start.
     * Used by shock propagation contact ordering. */
    ivp_vec3r_t gravity_dir;
} ivp_solver_config_t;

/* ── Presets ──────────────────────────────────────────────────────────── */

/* Fill with defaults matching current #define constants (no behavior change). */
void ivp_solver_config_init_default(ivp_solver_config_t *cfg);

/* Tuned for stacking/high-contact scenarios. */
void ivp_solver_config_init_stacking(ivp_solver_config_t *cfg);

/* ── Active-config pointer (file-scope, set/cleared at PSI boundaries) ── */

/* Set the active solver config for the current PSI.
 * Solver functions read this via ivp_solver_config_get_active().
 * Pass NULL to clear (between PSIs). */
void ivp_solver_config_set_active(const ivp_solver_config_t *cfg);

/* Get the currently active solver config, or NULL if not in a PSI. */
const ivp_solver_config_t *ivp_solver_config_get_active(void);

#ifdef __cplusplus
}
#endif

#endif /* IVP_SOLVER_CONFIG_H */
