/* ivp_solver_config.c -- Runtime solver configuration implementation */

#include <ivp/solver/ivp_solver_config.h>
#include <ivp/solver/ivp_solver_constants.h>
#include <string.h>

/* ── File-scope active-config pointer ────────────────────────────────── */

static const ivp_solver_config_t *s_active_config = NULL;

void ivp_solver_config_set_active(const ivp_solver_config_t *cfg)
{
    s_active_config = cfg;
}

const ivp_solver_config_t *ivp_solver_config_get_active(void)
{
    return s_active_config;
}

/* ── Presets ──────────────────────────────────────────────────────────── */

void ivp_solver_config_init_default(ivp_solver_config_t *cfg)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));

    /* Matrix solver */
    cfg->matrix_eps            = IVP_SOLVER_MATRIX_EPS;
    cfg->static_friction_eps   = IVP_SOLVER_STATIC_FRICTION_EPS;
    cfg->unilateral_max_steps  = IVP_SOLVER_UNILATERAL_MAX_STEP;

    /* Impact */
    cfg->impact_eps            = IVP_IMPACT_EPS;
    cfg->impact_max_pushes     = IVP_IMPACT_MAX_PUSHES;
    cfg->impact_initial_fraction = IVP_IMPACT_INITIAL_FRACTION;
    cfg->max_system_pushes     = IVP_MAXIMUM_SYSTEM_PUSH_NUMBER;

    /* Friction */
    cfg->max_friction_matrix   = IVP_MAX_FRICTION_MATRIX;
    cfg->pgs_base_iterations   = IVP_PGS_BASE_ITERATIONS;
    cfg->pgs_max_iterations    = IVP_PGS_MAX_ITERATIONS;
    cfg->pgs_residual_eps      = IVP_PGS_RESIDUAL_EPS;
    cfg->pgs_sor_omega         = IVP_REAL(1.0);
    cfg->ease_every_nth_psi    = IVP_EASE_EVERY_NTH_PSI;
    cfg->slowly_turn_on_keeper = IVP_SLOWLY_TURN_ON_KEEPER;
    cfg->max_energy_destroy    = IVP_MAX_ENERGY_DESTROY;

    /* Contact limits */
    cfg->max_contacts_per_pair = IVP_MAX_CONTACTS_PER_PAIR;
    cfg->max_contact_pairs     = IVP_MAX_CONTACT_PAIRS;

    /* Opt-in enhancements: all disabled by default */
    cfg->enable_shock_propagation  = false;
    cfg->enable_post_stabilization = false;
    cfg->post_stabilize_factor     = IVP_POST_STABILIZE_FACTOR;
    cfg->post_stabilize_max_impulse = IVP_POST_STABILIZE_MAX_IMPULSE;
    cfg->enable_adaptive_pgs       = false;
    cfg->enable_warm_start_impulse = false;
}

void ivp_solver_config_init_stacking(ivp_solver_config_t *cfg)
{
    ivp_solver_config_init_default(cfg);
    cfg->pgs_base_iterations       = 16;
    cfg->pgs_max_iterations        = 64;
    cfg->pgs_sor_omega             = IVP_REAL(1.3);
    cfg->enable_adaptive_pgs       = true;
    cfg->enable_shock_propagation  = true;
    cfg->enable_post_stabilization = true;
    cfg->enable_warm_start_impulse = true;
}
