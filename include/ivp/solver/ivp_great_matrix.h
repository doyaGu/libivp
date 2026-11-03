/* ivp_great_matrix.h — Dense linear system solver for IVP C17
 *
 * Gauss elimination with partial pivoting, matching the original IVP
 * IVP_Great_Matrix_Many_Zero solver.
 *
 * Pivot rule: largest absolute value in column; ties broken by index
 * (deterministic).
 *
 * Solves Ax = b for dense N×N systems up to IVP_MAX_GREAT_MATRIX_SIZE.
 *
 * The second half of this header (ivp_gmmz_*, ivp_incr_lu_*, ivp_vecfpu_*)
 * is a bit-faithful port of the reference dense solvers; see the comment
 * block there.
 */
#ifndef IVP_GREAT_MATRIX_H
#define IVP_GREAT_MATRIX_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/solver/ivp_solver_constants.h>
#include <stdbool.h>
#include <stddef.h>

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

/* ══════════════════════════════════════════════════════════════════════════
 * Reference-faithful dense solvers (port of ivp_great_matrix.cxx)
 * ══════════════════════════════════════════════════════════════════════════
 *
 * Everything below is a bit-faithful C17 port of the reference classes
 * IVP_Great_Matrix_Many_Zero ("gmmz"), IVP_Incr_L_U_Matrix ("incr_lu") and
 * the IVP_VecFPU row helpers ("vecfpu"), all in double.  The functions above
 * (ivp_great_matrix_*) are an independent, simplified dense solver kept as a
 * public utility; the simulation does not use them (the friction system
 * uses this section and ivp_linear_constraint_solver.h).
 *
 * What "faithful" means here — the reference behaviour that is reproduced:
 *
 *  - Release builds reproduce the reference release build (NDEBUG: every
 *    IVP_IF block is dead code, including the zero-fills in
 *    IVP_Linear_Constraint_Solver::alloc_memory and align_matrix_values).
 *    Debug / IVP_ENABLE_ASSERTS builds reproduce the reference debug build:
 *    the IVP_IF blocks are live, with their prints, consistency checks and
 *    side effects (zero-fills, the LCS's extra debug_mat arena allocation,
 *    the result check that overwrites delta_accel).
 *
 *  - On 64-bit hosts the reference compiles with PLATFORM_64BITS, which
 *    selects IVP_VECFPU_SIZE 4, a 16-byte IVP_VECFPU_MEM_MASK and
 *    IVP_VECFPU_MEMSHIFT 2 (the comment says "taken from vphysics retail").
 *    Rows are padded to a multiple of 4 doubles, row helpers work in whole
 *    blocks of 4, and an "unaligned" helper call first moves its start down
 *    to the previous 16-byte boundary (one double) while growing the count
 *    by 8 >> 2 = 2.  The helpers therefore touch up to 4 doubles past the
 *    logical end of a row, i.e. padding and sometimes the first entries of
 *    the NEXT row.  This visibly changes results (a 4x4 Gauss solve can
 *    return a wrong answer); it is reproduced exactly.  Array bases are
 *    treated as 16-byte aligned (the reference arena aligns to 32 bytes), so
 *    the emulation depends on element offsets only, never on addresses.
 *    Arrays allocated by this module (ivp_gmmz_init, ivp_incr_lu_init) carry
 *    the same trailing slack as the reference arena (IVP_U_Memory rounding,
 *    see ivp_gm_arena_next) so that overruns land in owned, zero-initialised
 *    memory.  The friction solver does not use those: like the reference it
 *    takes the arrays from the environment's persistent arena, whose layout
 *    matches IVP_U_Memory.  A reference-internals build preserves the stale
 *    contents read through overruns and unwritten padding; the stable-internals build
 *    clears the complete reserved span.
 *    Define IVP_GM_VECFPU_SIZE=1 to get the 32-bit reference build instead
 *    (plain scalar helpers, no overruns).
 *
 *  - Floating-point contraction: the parity target is the reference built
 *    with -ffp-contract=off (like libivp itself), where every product is
 *    rounded separately; that is the default (IVP_GM_USE_FMA=0).  A
 *    reference compiled by clang -O3 with the default -ffp-contract=on
 *    instead fuses `a*b + c` written inside one expression into an FMA (the
 *    row helpers multiply and add in separate statements and are not
 *    fused), and its contiguous dot-product style reductions were
 *    auto-vectorized as strict in-order reductions whose vector body is NOT
 *    fused: for a trip count >= 8 the first (count & ~7) terms are rounded
 *    product + rounded add and only the scalar tail is an FMA (the
 *    out-of-line solve_lower_null_matrix uses threshold 14).  Define
 *    IVP_GM_USE_FMA=1 to reproduce that build: fma() is then called
 *    explicitly at exactly the fused sites.
 *
 * Return codes: IVP_OK where the reference returns IVP_OK, IVP_GM_FAULT
 * (IVP_ERR_SINGULAR_MATRIX) where it returns IVP_FAULT. */

#ifndef IVP_GM_VECFPU_SIZE
#define IVP_GM_VECFPU_SIZE 4
#endif

#ifndef IVP_GM_USE_FMA
#define IVP_GM_USE_FMA 0
#endif

#if IVP_GM_VECFPU_SIZE != 4 && IVP_GM_VECFPU_SIZE != 1
#error "IVP_GM_VECFPU_SIZE must be 4 (64-bit reference) or 1 (32-bit reference)"
#endif

#define IVP_GM_FAULT IVP_ERR_SINGULAR_MATRIX

/* MATRIX_EPS set by the IVP_Great_Matrix_Many_Zero constructor: 1.0f/10E8f
 * evaluated in float, then widened. */
#define IVP_GMMZ_DEFAULT_EPS ((double)(1.0f / 10E8f))

/* Row length rounded up to the vector block size (calc_aligned_row_len). */
static inline int ivp_gm_aligned_len(int n) {
    return (n + IVP_GM_VECFPU_SIZE - 1) & ~(IVP_GM_VECFPU_SIZE - 1);
}

/* Offset following an allocation of `bytes` at `off` inside an
 * IVP_U_Memory block (64-bit: +0x50-1 then 32-byte mask; 32-bit: 32-byte
 * round-up).  Every gap is >= 48 bytes on 64-bit, which is more than the
 * largest vector-helper overrun (32 bytes). */
static inline size_t ivp_gm_arena_next(size_t off, size_t bytes) {
#if IVP_GM_VECFPU_SIZE == 4
    return (off + bytes + (size_t)0x4F) & ~(size_t)0x1F;
#else
    return (off + bytes + (size_t)0x1F) & ~(size_t)0x1F;
#endif
}

/* ── Vector-FPU row helpers (IVP_VecFPU) ─────────────────────────────────
 * Each array is passed as (base, element offset).  `base` is the start of
 * the allocation and is treated as 16-byte aligned; `aligned` mirrors the
 * reference's adress_aligned flag. */

void   ivp_vecfpu_add_multiple_row(double *target_base, int target_off,
                                   const double *source_base, int source_off,
                                   double factor, int size, bool aligned);
void   ivp_vecfpu_multiply_row(double *base, int off, double factor,
                               int size, bool aligned);
void   ivp_vecfpu_exchange_rows(double *base1, int off1,
                                double *base2, int off2,
                                int size, bool aligned);
void   ivp_vecfpu_copy_rows(double *target_base, int target_off,
                            const double *source_base, int source_off,
                            int size, bool aligned);
void   ivp_vecfpu_set_row_to_zero(double *base, int off, int size, bool aligned);
double ivp_vecfpu_large_dot_product(const double *base_a, int off_a,
                                    const double *base_b, int off_b,
                                    int size, bool aligned);

/* ── IVP_Great_Matrix_Many_Zero ──────────────────────────────────────────
 * NxN matrix, N = columns.  The Gauss solver uses row stride
 * aligned_row_len; lu_crout / lu_solve / lu_inverse / mult /
 * matrix_multiplication use row stride `columns` (as in the reference). */

typedef struct ivp_gmmz {
    double  matrix_eps;       /* MATRIX_EPS                                  */
    int     columns;
    int     aligned_row_len;
    double *matrix_values;    /* columns * aligned_row_len, row-major        */
    double *desired_vector;   /* aligned_row_len                             */
    double *result_vector;    /* aligned_row_len                             */
    void   *storage;          /* owned block (NULL if arrays are external)   */
    size_t  storage_size;
} ivp_gmmz_t;

/* Allocate an n x n matrix (zero-filled, reference arena layout and slack),
 * columns = n, aligned_row_len = ivp_gm_aligned_len(n),
 * matrix_eps = IVP_GMMZ_DEFAULT_EPS.  alloc may be NULL. */
ivp_result_t ivp_gmmz_init(ivp_gmmz_t *m, int n, const ivp_allocator_t *alloc);
void         ivp_gmmz_destroy(ivp_gmmz_t *m, const ivp_allocator_t *alloc);
void         ivp_gmmz_calc_aligned_row_len(ivp_gmmz_t *m);

void   ivp_gmmz_set_value(ivp_gmmz_t *m, double val, int col, int row);
double ivp_gmmz_get_value(const ivp_gmmz_t *m, int col, int row);

/* solve_great_matrix_many_zero: Gauss with pivot search on every column.
 * Input matrix_values/desired_vector (both destroyed), output result_vector. */
ivp_result_t ivp_gmmz_solve(ivp_gmmz_t *m);
void         ivp_gmmz_transform_to_lower_null_triangle(ivp_gmmz_t *m);
ivp_result_t ivp_gmmz_solve_lower_null_matrix(ivp_gmmz_t *m);
void         ivp_gmmz_add_multiple_line(ivp_gmmz_t *m, int first, int second,
                                        double factor);
void         ivp_gmmz_exchange_rows(ivp_gmmz_t *m, int a, int b);
void         ivp_gmmz_find_pivot_in_column(ivp_gmmz_t *m, int col);

/* Crout LU with implicit scaling (stride = columns; desired_vector is used
 * as scratch).  index_vec needs `columns` entries. */
ivp_result_t ivp_gmmz_lu_crout(ivp_gmmz_t *m, int *index_vec, double *sign);
ivp_result_t ivp_gmmz_lu_solve(ivp_gmmz_t *m, const int *index_vec);
ivp_result_t ivp_gmmz_lu_inverse(ivp_gmmz_t *m, ivp_gmmz_t *matrix_out,
                                 const int *index_vec);
/* invert: destroys m (and its desired/result vectors); out must have the
 * same `columns` and room for columns*columns values. */
ivp_result_t ivp_gmmz_invert(ivp_gmmz_t *m, ivp_gmmz_t *out);

void ivp_gmmz_mult(ivp_gmmz_t *m);          /* result = A*desired, stride columns */
void ivp_gmmz_mult_aligned(ivp_gmmz_t *m);  /* same, stride aligned_row_len       */
void ivp_gmmz_matrix_multiplication(ivp_gmmz_t *m, const double *first,
                                    const double *second);
void ivp_gmmz_copy_matrix(ivp_gmmz_t *dst, const ivp_gmmz_t *src);
void ivp_gmmz_copy_matrix_to(const ivp_gmmz_t *m, double *values,
                             double *desired);
/* values_big_matrix: packed m->columns x m->columns, row stride m->columns */
void ivp_gmmz_copy_to_sub_matrix(const ivp_gmmz_t *m,
                                 const double *values_big_matrix,
                                 ivp_gmmz_t *sub_matrix,
                                 const int *original_pos);
void ivp_gmmz_fill_from_bigger_matrix(ivp_gmmz_t *m, const ivp_gmmz_t *big,
                                      const int *original_pos, int n_column);
ivp_result_t ivp_gmmz_check_unequation_line(const ivp_gmmz_t *m, int linenum);
void ivp_gmmz_debug_fill_zero(ivp_gmmz_t *m);

/* align_matrix_values: moves matrix_values up to the next vector-block
 * boundary (the caller allocated IVP_GM_VECFPU_SIZE - 1 extra values; arena
 * memory is already aligned and does not move).  Debug builds also clear the
 * row padding (columns..aligned_row_len) and the padding of desired_vector,
 * like the reference's IVP_IF(1) block. */
void ivp_gmmz_align_matrix_values(ivp_gmmz_t *m);

/* Debug output of the reference (printed through ivp_debug_printf /
 * ivp_message, see ivp/base/ivp_message.h; the IVP_IF parts print only in
 * debug builds). */
void ivp_gmmz_matrix_test_unequation(const ivp_gmmz_t *m);   /* matrix_test_unequation */
void ivp_gmmz_matrix_out_before_gauss(const ivp_gmmz_t *m);  /* matrix_out_before_gauss */
/* test_result: old_matrix (stride columns) / old_desired = the input of the
 * solve; returns 0 */
int  ivp_gmmz_test_result(const ivp_gmmz_t *m, const double *old_matrix,
                          const double *old_desired);
int  ivp_gmmz_print_great_matrix(const ivp_gmmz_t *m, const char *comment); /* returns 0 */
#ifdef IVP_DEBUG_BUILD
/* plausible_input_check (reference #ifdef DEBUG): asserts |values| < 1e20 */
void ivp_gmmz_plausible_input_check(const ivp_gmmz_t *m);
#endif

/* ── IVP_Incr_L_U_Matrix ─────────────────────────────────────────────────
 * Incremental LU: L_matrix * A = U_matrix for the leading n_sub x n_sub
 * block; both matrices are n_max rows of aligned_row_len doubles. */

typedef struct ivp_incr_lu {
    double  matrix_eps;
    double *L_matrix;
    double *U_matrix;
    double *input_vec;
    double *out_vec;
    double *temp_vec;
    double *mult_vec;
    int     aligned_row_len;
    int     n_sub;
    void   *storage;          /* owned block (NULL if arrays are external)   */
    size_t  storage_size;
} ivp_incr_lu_t;

/* Allocate for up to n_max variables (zero-filled, reference arena layout),
 * n_sub = 0, matrix_eps = 1e-6 (the value the LCS uses). */
ivp_result_t ivp_incr_lu_init(ivp_incr_lu_t *lu, int n_max,
                              const ivp_allocator_t *alloc);
void         ivp_incr_lu_destroy(ivp_incr_lu_t *lu, const ivp_allocator_t *alloc);

/* l_u_decomposition_with_pivoting: decompose the n_sub x n_sub matrix
 * stored in U_matrix (L is rebuilt). */
ivp_result_t ivp_incr_lu_decompose(ivp_incr_lu_t *lu);
/* increment_l_u: new last row of A already in U_matrix row n_sub (length
 * n_sub+1), new last column in input_vec (length n_sub). */
ivp_result_t ivp_incr_lu_increment(ivp_incr_lu_t *lu);
/* decrement_l_u: remove variable del_nr (swapped with the last one). */
ivp_result_t ivp_incr_lu_decrement(ivp_incr_lu_t *lu, int del_nr);
/* solve_lin_equ: out_vec = A^-1 * input_vec. */
void         ivp_incr_lu_solve_lin_equ(ivp_incr_lu_t *lu);
void         ivp_incr_lu_mult_vec_with_L(ivp_incr_lu_t *lu);
void         ivp_incr_lu_solve_vec_with_U(ivp_incr_lu_t *lu);
ivp_result_t ivp_incr_lu_normize_row(ivp_incr_lu_t *lu, int row_nr);
ivp_result_t ivp_incr_lu_normize_row_L(ivp_incr_lu_t *lu, int row_nr);
void         ivp_incr_lu_pivot_search(ivp_incr_lu_t *lu, int col_nr);
void         ivp_incr_lu_exchange_rows(ivp_incr_lu_t *lu, int pivot_col,
                                       int exchange);
void         ivp_incr_lu_add_neg_row_to_row(ivp_incr_lu_t *lu, int pivot_row,
                                            int dest_row, double factor);
void         ivp_incr_lu_add_neg_row_upwards(ivp_incr_lu_t *lu, int row_down,
                                             int dest_row, double factor);
void         ivp_incr_lu_subtract_row_L(ivp_incr_lu_t *lu, int from, int dest,
                                        double factor);
void         ivp_incr_lu_add_neg_row_L(ivp_incr_lu_t *lu, int source_row,
                                       int dest_row, double factor);
void         ivp_incr_lu_add_neg_col_L(ivp_incr_lu_t *lu, int col0,
                                       int col_dest, double factor);
void         ivp_incr_lu_exchange_columns_L(ivp_incr_lu_t *lu, int col0, int col1);
void         ivp_incr_lu_exchange_columns_U(ivp_incr_lu_t *lu, int col0, int col1);
/* debug_print_l_u / debug_print_a (reference debug helpers, not called by
 * the solvers) */
void         ivp_incr_lu_debug_print_l_u(const ivp_incr_lu_t *lu);
void         ivp_incr_lu_debug_print_a(const ivp_incr_lu_t *lu);

#ifdef __cplusplus
}
#endif

#endif /* IVP_GREAT_MATRIX_H */
