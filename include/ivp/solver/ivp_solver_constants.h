/* ivp_solver_constants.h — Solver tuning constants for IVP C17
 *
 * All epsilon values and size limits, extracted from the original IVP code.
 * Centralized here for deterministic control.
 */
#ifndef IVP_SOLVER_CONSTANTS_H
#define IVP_SOLVER_CONSTANTS_H

#include <ivp/base/ivp_types.h>

/* ── Matrix solver epsilons ─────────────────────────────────────────────── */

#define IVP_SOLVER_MATRIX_EPS           ((ivp_real_t)1e-9)
#define IVP_SOLVER_EPS                  ((ivp_real_t)1e-7)
#define IVP_SOLVER_GAUSS_EPS            ((ivp_real_t)1e-7)
#define IVP_SOLVER_TEST_EPS             ((ivp_real_t)1e-4)
#define IVP_SOLVER_LU_EPS              ((ivp_real_t)1e-6)
#define IVP_SOLVER_STABLE_LU_EPS        ((ivp_real_t)1e-10)
#define IVP_SOLVER_STATIC_FRICTION_EPS  ((ivp_real_t)1e-5)
#define IVP_SOLVER_COMPLEX_EPS          ((ivp_real_t)1e-5)

/* ── Matrix size limits ─────────────────────────────────────────────────── */

#define IVP_MAX_GREAT_MATRIX_SIZE       2048
#define IVP_SOLVER_UNILATERAL_MAX_STEP  250
#define IVP_SOLVER_NUMERIC_TEST_EVERY_N 7

/* ── Impact constants ───────────────────────────────────────────────────── */

#define IVP_IMPACT_EPS                  ((ivp_real_t)1e-4)
#define IVP_IMPACT_MAX_PUSHES           100
#define IVP_IMPACT_INITIAL_FRACTION     ((ivp_real_t)0.1)
#define IVP_INV_HALF_CONSERVATION_STEPS ((ivp_real_t)0.5)
#define IVP_SPEED_ADDON_SYSTEM_IMPACT   ((ivp_real_t)0.01)
#define IVP_MAXIMUM_SYSTEM_PUSH_NUMBER  5000
#define IVP_SAFETY_FACTOR_FOR_DELAY     ((ivp_real_t)1.2)

/* ── Friction constants ─────────────────────────────────────────────────── */

#define IVP_MAX_FRICTION_MATRIX         150
#define IVP_MAX_ADHESION_GAUSS          ((ivp_real_t)0.01)
#define IVP_MINIMAL_REAL_FRICTION_LEN   ((ivp_real_t)0.005)
#define IVP_MAX_ENERGY_DESTROY          ((ivp_real_t)0.1)
#define IVP_EASE_EVERY_NTH_PSI          5
#define IVP_SLOWLY_TURN_ON_KEEPER       20
#define IVP_DESCENDING_SPEEDUP          ((ivp_real_t)20.0)

/* ── PGS solver constants ──────────────────────────────────────────────── */

#define IVP_PGS_BASE_ITERATIONS         8
#define IVP_PGS_MAX_ITERATIONS          32
#define IVP_PGS_RESIDUAL_EPS            ((ivp_real_t)1e-4)

/* ── Easing / energy removal decay factors ─────────────────────────────── */

#define IVP_EASE_DECAY_FACTOR           ((ivp_real_t)0.9)
#define IVP_ENERGY_REMOVAL_DECAY        ((ivp_real_t)0.95)

/* ── Impact cascade constants ──────────────────────────────────────────── */

#define IVP_IMPACT_CASCADE_MAX_ITERS    50

/* ── Post-stabilization defaults ───────────────────────────────────────── */

#define IVP_POST_STABILIZE_FACTOR       ((ivp_real_t)0.2)
#define IVP_POST_STABILIZE_MAX_IMPULSE  ((ivp_real_t)10.0)

/* ── Contact / broadphase constants ─────────────────────────────────────── */

#define IVP_MAX_CONTACTS_PER_PAIR       4
#define IVP_MAX_CONTACT_PAIRS           1024

#endif /* IVP_SOLVER_CONSTANTS_H */
