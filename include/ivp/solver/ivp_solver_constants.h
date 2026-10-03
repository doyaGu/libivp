/* ivp_solver_constants.h — constants of the libivp dense matrix helpers
 *
 * The simulation pipeline (src/solver/ivp_fr_system.c, ivp_impact_core.c,
 * ivp_linear_constraint_solver.c) uses the reference constants directly
 * where they are defined in the reference sources; the values below are
 * kept for the public great-matrix API and for tests.
 */
#ifndef IVP_SOLVER_CONSTANTS_H
#define IVP_SOLVER_CONSTANTS_H

#include <ivp/base/ivp_types.h>

/* ── Matrix solver epsilons ─────────────────────────────────────────────── */

#define IVP_SOLVER_MATRIX_EPS           ((ivp_real_t)1e-9)
#define IVP_SOLVER_STATIC_FRICTION_EPS  ((ivp_real_t)1e-5)

/* ── Matrix size limits ─────────────────────────────────────────────────── */

#define IVP_MAX_GREAT_MATRIX_SIZE       2048
#define IVP_SOLVER_UNILATERAL_MAX_STEP  250   /* MAX_STEP_LEN of IVP_Linear_Constraint_Solver */

/* ── Reference limits (informational) ───────────────────────────────────── */

#define IVP_IMPACT_EPS                  ((ivp_real_t)1e-4)
#define IVP_MAXIMUM_SYSTEM_PUSH_NUMBER  5000  /* IVP_Impact_System */
#define IVP_MAX_FRICTION_MATRIX         150   /* IVP_Friction_System */

#endif /* IVP_SOLVER_CONSTANTS_H */
