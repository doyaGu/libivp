/* ivp_linear_constraint_solver.h — Unilateral linear constraint solver
 *
 * Bit-faithful C17 port of the reference IVP_Linear_Constraint_Solver
 * (ivp_great_matrix.cxx), the complementarity solver the friction system
 * falls back to when plain Gauss gives negative impulses.
 *
 * Problem: given an n x n matrix A (symmetric positive semi-definite in
 * practice) and b, find x with
 *     x >= 0,   accel = A*x - b >= 0,   x[i] * accel[i] = 0.
 * The solver is an active-set pivoting method: variables are processed one
 * by one ("ignored" -> "active" or "inactive"), the active sub-system is
 * kept as an incremental LU (ivp_incr_lu_t) with a Gauss fallback
 * (ivp_gmmz_t), a numerical stability check runs every 7 steps, and it
 * gives up after 250 steps.  All arithmetic is double and reproduces the
 * 64-bit reference release build bit for bit, including its VecFPU row
 * overruns and FMA contraction sites (see ivp_great_matrix.h).
 */
#ifndef IVP_LINEAR_CONSTRAINT_SOLVER_H
#define IVP_LINEAR_CONSTRAINT_SOLVER_H

#include <ivp/base/ivp_result.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/solver/ivp_great_matrix.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Constants of the reference solver. */
#define IVP_LCS_MAX_STEP             250   /* IVP_UNILATERAL_SOLVER_MAX_STEP */
#define IVP_LCS_NUMERIC_TEST_EVERY_N 7     /* IVP_NUMERIC_TEST_EVERY_N       */
#define IVP_LCS_SOLVER_EPS           ((double)1E-7f)
#define IVP_LCS_GAUSS_EPS            IVP_LCS_SOLVER_EPS
#define IVP_LCS_TEST_EPS             (IVP_LCS_SOLVER_EPS * 1000.0)
#define IVP_LCS_LU_EPS               (IVP_LCS_SOLVER_EPS * 10)
#define IVP_LCS_MAX_STEP_LEN         10000.0

struct ivp_u_memory;

/* IVP_Linear_Constraint_Solver::init_and_solve_lc on an IVP_U_Memory arena,
 * exactly as the reference friction solver calls it:
 *   A_in              n x n, row stride ivp_gm_aligned_len(n) (padding is
 *                     read where the reference reads it)
 *   b_in              ivp_gm_aligned_len(n) entries (all are read)
 *   x_out             ivp_gm_aligned_len(n) entries; receives x (also on
 *                     failure: the reference's state at that point)
 *   mem               the solver takes its 19 scratch arrays from here
 *                     (alloc_memory order, uninitialised); the caller owns
 *                     the transaction around the call
 * A_in and b_in are not modified.  Returns IVP_OK / IVP_GM_FAULT like the
 * reference, IVP_ERR_INVALID_ARG or IVP_ERR_OUT_OF_MEMORY otherwise. */
ivp_result_t ivp_lcs_init_and_solve_in_memory(double *A_in, double *b_in, double *x_out,
                                              int n, int actives_at_begin,
                                              struct ivp_u_memory *mem);

/* init_and_solve_lc on a fresh zero-filled arena (the reference started on
 * zeroed IVP_U_Memory).
 *   A                 n x n, row-major with row stride n
 *   b                 n entries
 *   x_out             n entries; receives x (also written when the solver
 *                     fails, holding the reference's state at that point)
 *   actives_at_begin  the first `actives_at_begin` variables are tried as
 *                     active in the initial setup (0 <= actives_at_begin <= n)
 *   alloc             workspace allocator (NULL = default)
 * Returns IVP_OK when the reference returns IVP_OK, IVP_GM_FAULT
 * (IVP_ERR_SINGULAR_MATRIX) when it returns IVP_FAULT, IVP_ERR_INVALID_ARG
 * or IVP_ERR_OUT_OF_MEMORY for bad input / allocation failure.
 * A and b are not modified. */
ivp_result_t ivp_lcs_init_and_solve(const double *A, const double *b, double *x_out,
                                    int n, int actives_at_begin,
                                    const ivp_allocator_t *alloc);

/* Same with an explicit row stride for A (a_stride >= n), e.g. the padded
 * aligned_row_len layout of an ivp_gmmz_t. */
ivp_result_t ivp_lcs_init_and_solve_strided(const double *A, int a_stride,
                                            const double *b, double *x_out,
                                            int n, int actives_at_begin,
                                            const ivp_allocator_t *alloc);

#ifdef __cplusplus
}
#endif

#endif /* IVP_LINEAR_CONSTRAINT_SOLVER_H */
