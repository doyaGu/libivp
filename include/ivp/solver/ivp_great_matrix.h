/* ivp_great_matrix.h — Dense linear system solver for IVP C17
 *
 * Gauss elimination with partial pivoting, matching the original IVP
 * IVP_Great_Matrix_Many_Zero solver.
 *
 * Pivot rule: largest absolute value in column; ties broken by index
 * (deterministic).
 *
 * Solves Ax = b for dense N×N systems up to IVP_MAX_GREAT_MATRIX_SIZE.
 */
#ifndef IVP_GREAT_MATRIX_H
#define IVP_GREAT_MATRIX_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/solver/ivp_solver_constants.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Solver scalar ──────────────────────────────────────────────────────── */

/* The reference engine runs IVP_Great_Matrix_Many_Zero entirely in
 * IVP_DOUBLE even when the rest of the simulation is single precision:
 * contact matrices for stacked bodies are badly conditioned and a float
 * elimination injects enough error to blow stacks apart.  The public API
 * stays in ivp_real_t; storage and elimination use this type. */
typedef double ivp_gm_real_t;

/* ── Dense matrix (augmented [A|b]) ─────────────────────────────────────── */

typedef struct ivp_great_matrix {
    ivp_gm_real_t *data;    /* n × (n+1) row-major augmented matrix (double)    */
    int        *pivot;      /* row permutation (pivot[row] = original_row)      */
    int         n;          /* matrix dimension                                  */
    int         capacity;   /* allocated columns (n+1)                          */
} ivp_great_matrix_t;

/* ── Lifecycle ──────────────────────────────────────────────────────────── */

/* Initialize a matrix of dimension n.
 * alloc may be NULL for default allocator.
 * Returns IVP_OK on success, IVP_ERR_OUT_OF_MEMORY on failure. */
ivp_result_t ivp_great_matrix_init(ivp_great_matrix_t *m, int n,
                                    const ivp_allocator_t *alloc);

/* Free allocated storage. */
void ivp_great_matrix_destroy(ivp_great_matrix_t *m, const ivp_allocator_t *alloc);

/* Zero out the matrix and reset pivot permutation. */
void ivp_great_matrix_clear(ivp_great_matrix_t *m);

/* ── Element access ─────────────────────────────────────────────────────── */

/* Set matrix element A[row][col]. */
static inline void ivp_great_matrix_set(ivp_great_matrix_t *m,
                                         int row, int col, ivp_real_t val) {
    m->data[row * m->capacity + col] = (ivp_gm_real_t)val;
}

/* Get matrix element A[row][col]. */
static inline ivp_real_t ivp_great_matrix_get(const ivp_great_matrix_t *m,
                                               int row, int col) {
    return (ivp_real_t)m->data[row * m->capacity + col];
}

/* Set right-hand-side b[row] (last column of augmented matrix). */
static inline void ivp_great_matrix_set_rhs(ivp_great_matrix_t *m,
                                             int row, ivp_real_t val) {
    m->data[row * m->capacity + m->n] = (ivp_gm_real_t)val;
}

/* Get right-hand-side b[row]. */
static inline ivp_real_t ivp_great_matrix_get_rhs(const ivp_great_matrix_t *m,
                                                    int row) {
    return (ivp_real_t)m->data[row * m->capacity + m->n];
}

/* ── Solver ─────────────────────────────────────────────────────────────── */

/* Solve Ax = b via Gauss elimination with partial pivoting.
 * Results are written to x[0..n-1].
 * Returns IVP_OK on success, IVP_ERR_DEGENERATE_GEOMETRY if singular. */
ivp_result_t ivp_great_matrix_solve(ivp_great_matrix_t *m, ivp_real_t *x);

/* Solve Ax = b and also return the determinant.
 * det_out may be NULL if determinant is not needed. */
ivp_result_t ivp_great_matrix_solve_det(ivp_great_matrix_t *m,
                                         ivp_real_t *x,
                                         ivp_real_t *det_out);

/* ── Unilateral solver ──────────────────────────────────────────────────── */

/* Solve Ax = b subject to x[i] >= 0 for all i (unilateral constraints).
 * Uses iterative Gauss elimination with contact deactivation:
 *   1. Solve unconstrained system via Gauss elimination
 *   2. If any x[i] < 0, deactivate the most negative (set x[i] = 0)
 *   3. Rebuild and re-solve the reduced system
 *   4. Iterate until all x[i] >= 0 or max_steps reached
 *
 * active_initial[0..n-1]: initial active set.
 *   If active_initial[i] != 0, contact i starts in the active set.
 *   If active_initial[i] == 0, contact i starts deactivated (x[i] = 0).
 * active_initial may be NULL (all contacts initially active).
 *
 * Results are written to x[0..n-1].
 * Returns IVP_OK on success. */
ivp_result_t ivp_great_matrix_solve_unilateral(ivp_great_matrix_t *m,
                                                  ivp_real_t *x,
                                                  const int *active_initial,
                                                  const ivp_allocator_t *alloc);

/* Solve only the active subset with plain Gauss and map solution to full size.
 * Inactive entries are forced to zero. */
ivp_result_t ivp_great_matrix_solve_active_subset(const ivp_great_matrix_t *m,
                                                   const int *active_mask,
                                                   ivp_real_t *x,
                                                   const ivp_allocator_t *alloc);

/* ── Utility ────────────────────────────────────────────────────────────── */

/* Compute determinant without solving (destroys matrix contents). */
ivp_result_t ivp_great_matrix_determinant(ivp_great_matrix_t *m,
                                           ivp_real_t *det_out);

#ifdef __cplusplus
}
#endif

#endif /* IVP_GREAT_MATRIX_H */
