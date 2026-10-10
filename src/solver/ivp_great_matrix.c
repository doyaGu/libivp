/* ivp_great_matrix.c — Dense linear system solver
 *
 * Gauss elimination with partial pivoting + back substitution.
 * Matches original IVP IVP_Great_Matrix_Many_Zero behavior:
 *   - Column-wise partial pivoting (max absolute value)
 *   - Ties broken by lower index (deterministic)
 *   - MATRIX_EPS threshold for singularity detection
 *
 * The second half of the file is the bit-faithful port of the reference
 * IVP_VecFPU / IVP_Great_Matrix_Many_Zero / IVP_Incr_L_U_Matrix code
 * (ivp_gmmz_*, ivp_incr_lu_*, ivp_vecfpu_*); see ivp_great_matrix.h.
 */
#include <ivp/solver/ivp_great_matrix.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "base/ivp_simd_internal.h"

/* ── Lifecycle ──────────────────────────────────────────────────────────── */

ivp_result_t ivp_great_matrix_init(ivp_great_matrix_t *m, int n,
                                    const ivp_allocator_t *alloc)
{
    if (n <= 0 || n > IVP_MAX_GREAT_MATRIX_SIZE) {
        return IVP_ERR_INVALID_ARG;
    }

    int cols = n + 1; /* augmented [A|b] */
    size_t data_size = (size_t)n * (size_t)cols * sizeof(ivp_gm_real_t);
    size_t pivot_size = (size_t)n * sizeof(int);

    const ivp_allocator_t *a = alloc ? alloc : ivp_default_allocator();
    m->data = (ivp_gm_real_t *)ivp_alloc(a, data_size);
    if (!m->data) return IVP_ERR_OUT_OF_MEMORY;

    m->pivot = (int *)ivp_alloc(a, pivot_size);
    if (!m->pivot) {
        ivp_free(a, m->data, data_size);
        m->data = NULL;
        return IVP_ERR_OUT_OF_MEMORY;
    }

    m->n = n;
    m->capacity = cols;

    ivp_great_matrix_clear(m);
    return IVP_OK;
}

void ivp_great_matrix_destroy(ivp_great_matrix_t *m, const ivp_allocator_t *alloc)
{
    if (!m) return;
    const ivp_allocator_t *a = alloc ? alloc : ivp_default_allocator();
    if (m->data) {
        size_t data_size = (size_t)m->n * (size_t)m->capacity * sizeof(ivp_gm_real_t);
        ivp_free(a, m->data, data_size);
        m->data = NULL;
    }
    if (m->pivot) {
        size_t pivot_size = (size_t)m->n * sizeof(int);
        ivp_free(a, m->pivot, pivot_size);
        m->pivot = NULL;
    }
    m->n = 0;
}

void ivp_great_matrix_clear(ivp_great_matrix_t *m)
{
    memset(m->data, 0, (size_t)m->n * (size_t)m->capacity * sizeof(ivp_gm_real_t));
    for (int i = 0; i < m->n; i++) {
        m->pivot[i] = i;
    }
}

/* ── Internal: swap rows ────────────────────────────────────────────────── */

static void swap_rows(ivp_great_matrix_t *m, int a, int b)
{
    if (a == b) return;
    int cols = m->capacity;
    ivp_gm_real_t *row_a = m->data + a * cols;
    ivp_gm_real_t *row_b = m->data + b * cols;
    for (int j = 0; j < cols; j++) {
        ivp_gm_real_t tmp = row_a[j];
        row_a[j] = row_b[j];
        row_b[j] = tmp;
    }
    int tmp = m->pivot[a];
    m->pivot[a] = m->pivot[b];
    m->pivot[b] = tmp;
}

/* ── Internal: find pivot ───────────────────────────────────────────────── */

/* Find the row with largest |a[row][col]| for row >= start_row.
 * Ties broken by lower row index (deterministic). */
static int find_pivot(const ivp_great_matrix_t *m, int col, int start_row)
{
    int best = start_row;
    ivp_gm_real_t best_val = fabs(m->data[start_row * m->capacity + col]);

    for (int i = start_row + 1; i < m->n; i++) {
        ivp_gm_real_t val = fabs(m->data[i * m->capacity + col]);
        if (val > best_val) {
            best_val = val;
            best = i;
        }
    }
    return best;
}

/* ── Gauss elimination + back-substitution ──────────────────────────────── */

/* Reference alignment: normize_constraint_equ (ivp_friction_gaps.cxx ~L93).
 * Normalizes matrix by largest diagonal and RHS by largest element,
 * storing a correction factor to rescale results afterward.
 * This dramatically improves condition number for friction matrices. */
static ivp_gm_real_t normalize_matrix(ivp_great_matrix_t *m)
{
    int n = m->n;
    int cols = m->capacity;

    /* Find max diagonal */
    ivp_gm_real_t max_diag = 0.0;
    for (int i = 0; i < n; i++) {
        ivp_gm_real_t d = fabs(m->data[i * cols + i]);
        if (d > max_diag) max_diag = d;
    }

    /* Find max |RHS| */
    ivp_gm_real_t max_rhs = 0.0;
    for (int i = 0; i < n; i++) {
        ivp_gm_real_t r = fabs(m->data[i * cols + n]);
        if (r > max_rhs) max_rhs = r;
    }

    if (max_diag < (ivp_gm_real_t)IVP_REAL_EPS || max_rhs < (ivp_gm_real_t)IVP_REAL_EPS) {
        return 1.0;  /* nothing to normalize */
    }

    ivp_gm_real_t a_norm = 1.0 / max_diag;
    ivp_gm_real_t b_norm = 1.0 / max_rhs;
    ivp_gm_real_t correct_x_factor = a_norm * max_rhs;  /* = max_rhs / max_diag */

    /* Scale matrix elements by a_norm */
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            m->data[i * cols + j] *= a_norm;
        }
    }

    /* Scale RHS by b_norm */
    for (int i = 0; i < n; i++) {
        m->data[i * cols + n] *= b_norm;
    }

    return correct_x_factor;
}

ivp_result_t ivp_great_matrix_solve(ivp_great_matrix_t *m, ivp_real_t *x)
{
    return ivp_great_matrix_solve_det(m, x, NULL);
}

ivp_result_t ivp_great_matrix_solve_det(ivp_great_matrix_t *m,
                                         ivp_real_t *x,
                                         ivp_real_t *det_out)
{
    int n = m->n;
    int cols = m->capacity;

    /* Reference alignment: normalize matrix before solving */
    ivp_gm_real_t correct_x_factor = normalize_matrix(m);

    ivp_gm_real_t det = 1.0;
    int swaps = 0;

    /* Forward elimination with partial pivoting */
    for (int j = 0; j < n; j++) {
        /* Find pivot in column j */
        int pivot_row = find_pivot(m, j, j);
        if (pivot_row != j) {
            swap_rows(m, j, pivot_row);
            swaps++;
        }

        ivp_gm_real_t diag = m->data[j * cols + j];
        det *= diag;

        /* Reference alignment: graceful singular row handling
         * (ivp_great_matrix.cxx solve_lower_null_matrix ~L619).
         * If both diagonal and RHS are near-zero, set x[i] = 0
         * instead of failing. Only fail if truly inconsistent. */
        if (fabs(diag) < (ivp_gm_real_t)IVP_SOLVER_MATRIX_EPS) {
            ivp_gm_real_t rhs = m->data[j * cols + n];
            if (fabs(rhs) < (ivp_gm_real_t)IVP_SOLVER_MATRIX_EPS * 1000.0) {
                /* Degenerate row: set result to 0 */
                m->data[j * cols + n] = 0.0;
                m->data[j * cols + j] = 1.0;  /* prevent div-by-zero */
                continue;
            }
            if (det_out) *det_out = (ivp_real_t)0.0;
            return IVP_ERR_SINGULAR_MATRIX;
        }

        /* Eliminate below */
        ivp_gm_real_t inv_diag = 1.0 / diag;
        for (int i = j + 1; i < n; i++) {
            ivp_gm_real_t factor = m->data[i * cols + j] * inv_diag;
            m->data[i * cols + j] = 0.0;
            for (int k = j + 1; k < cols; k++) {
                m->data[i * cols + k] -= factor * m->data[j * cols + k];
            }
        }
    }

    /* Back substitution, kept in solver precision until the final store */
    ivp_gm_real_t xd[IVP_MAX_GREAT_MATRIX_SIZE];
    for (int i = n - 1; i >= 0; i--) {
        ivp_gm_real_t sum = m->data[i * cols + n]; /* RHS */
        for (int j = i + 1; j < n; j++) {
            sum -= m->data[i * cols + j] * xd[j];
        }
        xd[i] = sum / m->data[i * cols + i];
    }

    /* Reference alignment: rescale results by correct_x_factor */
    for (int i = 0; i < n; i++) {
        x[i] = (ivp_real_t)(xd[i] * correct_x_factor);
    }

    if (det_out) {
        *det_out = (ivp_real_t)((swaps & 1) ? -det : det);
    }
    return IVP_OK;
}

/* ── Unilateral solver ──────────────────────────────────────────────────── */

ivp_result_t ivp_great_matrix_solve_unilateral(ivp_great_matrix_t *m,
                                                  ivp_real_t *x,
                                                  const int *active_initial,
                                                  const ivp_allocator_t *alloc)
{
    int n = m->n;
    if (n <= 0) return IVP_ERR_INVALID_ARG;

    const ivp_allocator_t *a = alloc ? alloc : ivp_default_allocator();

    /* Track which contacts are active (x[i] may be > 0) vs
     * deactivated (x[i] forced to 0). */
    size_t bool_size = (size_t)n * sizeof(int);
    int *active = (int *)ivp_alloc(a, bool_size);
    if (!active) return IVP_ERR_OUT_OF_MEMORY;

    for (int i = 0; i < n; i++) {
        active[i] = active_initial ? (active_initial[i] != 0 ? 1 : 0) : 1;
    }

    /* We need a working copy of the original matrix since Gauss elimination
     * is destructive.  Store the original in a flat buffer. */
    size_t orig_size = (size_t)n * (size_t)m->capacity * sizeof(ivp_gm_real_t);
    ivp_gm_real_t *orig_data = (ivp_gm_real_t *)ivp_alloc(a, orig_size);
    if (!orig_data) {
        ivp_free(a, active, bool_size);
        return IVP_ERR_OUT_OF_MEMORY;
    }
    memcpy(orig_data, m->data, orig_size);

    /* Iterative deactivation loop */
    for (int step = 0; step < IVP_SOLVER_UNILATERAL_MAX_STEP; step++) {
        /* Count active contacts */
        int n_active = 0;
        for (int i = 0; i < n; i++) {
            if (active[i]) n_active++;
        }

        if (n_active == 0) {
            /* All deactivated */
            for (int i = 0; i < n; i++) x[i] = (ivp_real_t)0.0;
            break;
        }

        /* Build reduced system for active contacts only.
         * Map active[k] → position in reduced system. */
        size_t map_size = (size_t)n * sizeof(int);
        int *full_to_reduced = (int *)ivp_alloc(a, map_size);
        int *reduced_to_full = (int *)ivp_alloc(a, map_size);
        if (!full_to_reduced || !reduced_to_full) {
            if (full_to_reduced) ivp_free(a, full_to_reduced, map_size);
            if (reduced_to_full) ivp_free(a, reduced_to_full, map_size);
            ivp_free(a, orig_data, orig_size);
            ivp_free(a, active, bool_size);
            return IVP_ERR_OUT_OF_MEMORY;
        }

        int ri = 0;
        for (int i = 0; i < n; i++) {
            if (active[i]) {
                full_to_reduced[i] = ri;
                reduced_to_full[ri] = i;
                ri++;
            } else {
                full_to_reduced[i] = -1;
            }
        }

        /* Build reduced matrix from original */
        ivp_great_matrix_t rm;
        ivp_result_t res = ivp_great_matrix_init(&rm, n_active, a);
        if (res != IVP_OK) {
            ivp_free(a, full_to_reduced, map_size);
            ivp_free(a, reduced_to_full, map_size);
            ivp_free(a, orig_data, orig_size);
            ivp_free(a, active, bool_size);
            return res;
        }

        for (int ri2 = 0; ri2 < n_active; ri2++) {
            int fi = reduced_to_full[ri2];
            for (int rj = 0; rj < n_active; rj++) {
                int fj = reduced_to_full[rj];
                ivp_gm_real_t val = orig_data[fi * m->capacity + fj];
                rm.data[ri2 * rm.capacity + rj] = val; /* keep solver precision */
            }
            /* RHS from original (last column) */
            ivp_gm_real_t rhs = orig_data[fi * m->capacity + n];
            rm.data[ri2 * rm.capacity + rm.n] = rhs;
        }

        /* Solve reduced system */
        size_t xr_size = (size_t)n_active * sizeof(ivp_real_t);
        ivp_real_t *x_reduced = (ivp_real_t *)ivp_alloc(a, xr_size);
        if (!x_reduced) {
            ivp_great_matrix_destroy(&rm, a);
            ivp_free(a, full_to_reduced, map_size);
            ivp_free(a, reduced_to_full, map_size);
            ivp_free(a, orig_data, orig_size);
            ivp_free(a, active, bool_size);
            return IVP_ERR_OUT_OF_MEMORY;
        }

        res = ivp_great_matrix_solve(&rm, x_reduced);
        ivp_great_matrix_destroy(&rm, a);

        if (res != IVP_OK) {
            /* Singular: fall back to all zeros for active set */
            for (int i = 0; i < n; i++) x[i] = (ivp_real_t)0.0;
            ivp_free(a, x_reduced, xr_size);
            ivp_free(a, full_to_reduced, map_size);
            ivp_free(a, reduced_to_full, map_size);
            ivp_free(a, orig_data, orig_size);
            ivp_free(a, active, bool_size);
            return IVP_OK; /* graceful degradation */
        }

        /* Map back to full-size result */
        for (int i = 0; i < n; i++) x[i] = (ivp_real_t)0.0;
        for (int ri2 = 0; ri2 < n_active; ri2++) {
            x[reduced_to_full[ri2]] = x_reduced[ri2];
        }

        ivp_free(a, x_reduced, xr_size);
        ivp_free(a, full_to_reduced, map_size);
        ivp_free(a, reduced_to_full, map_size);

        /* Check for negative impulses */
        int worst_idx = -1;
        ivp_real_t worst_val = (ivp_real_t)0.0;
        for (int i = 0; i < n; i++) {
            if (active[i] && x[i] < -IVP_SOLVER_STATIC_FRICTION_EPS) {
                if (x[i] < worst_val) {
                    worst_val = x[i];
                    worst_idx = i;
                }
            }
        }

        if (worst_idx < 0) {
            /* All non-negative: solution is valid */
            break;
        }

        /* Deactivate the most negative contact and re-solve */
        active[worst_idx] = 0;
        x[worst_idx] = (ivp_real_t)0.0;
    }


    ivp_free(a, orig_data, orig_size);
    ivp_free(a, active, bool_size);
    return IVP_OK;
}

ivp_result_t ivp_great_matrix_solve_active_subset(const ivp_great_matrix_t *m,
                                                   const int *active_mask,
                                                   ivp_real_t *x,
                                                   const ivp_allocator_t *alloc)
{
    if (!m || !x || !active_mask) return IVP_ERR_INVALID_ARG;

    int n = m->n;
    const ivp_allocator_t *a = alloc ? alloc : ivp_default_allocator();

    int n_active = 0;
    for (int i = 0; i < n; i++) {
        if (active_mask[i]) n_active++;
        x[i] = (ivp_real_t)0.0;
    }
    if (n_active <= 0) return IVP_OK;

    size_t map_size = (size_t)n * sizeof(int);
    int *full_to_reduced = (int *)ivp_alloc(a, map_size);
    int *reduced_to_full = (int *)ivp_alloc(a, map_size);
    if (!full_to_reduced || !reduced_to_full) {
        if (full_to_reduced) ivp_free(a, full_to_reduced, map_size);
        if (reduced_to_full) ivp_free(a, reduced_to_full, map_size);
        return IVP_ERR_OUT_OF_MEMORY;
    }

    int ri = 0;
    for (int i = 0; i < n; i++) {
        if (active_mask[i]) {
            full_to_reduced[i] = ri;
            reduced_to_full[ri] = i;
            ri++;
        } else {
            full_to_reduced[i] = -1;
        }
    }

    ivp_great_matrix_t rm;
    ivp_result_t res = ivp_great_matrix_init(&rm, n_active, a);
    if (res != IVP_OK) {
        ivp_free(a, full_to_reduced, map_size);
        ivp_free(a, reduced_to_full, map_size);
        return res;
    }

    for (int i = 0; i < n_active; i++) {
        int fi = reduced_to_full[i];
        for (int j = 0; j < n_active; j++) {
            int fj = reduced_to_full[j];
            rm.data[i * rm.capacity + j] = m->data[fi * m->capacity + fj]; /* keep solver precision */
        }
        rm.data[i * rm.capacity + rm.n] = m->data[fi * m->capacity + n];
    }

    size_t x_sub_size = (size_t)n_active * sizeof(ivp_real_t);
    ivp_real_t *x_sub = (ivp_real_t *)ivp_alloc(a, x_sub_size);
    if (!x_sub) {
        ivp_great_matrix_destroy(&rm, a);
        ivp_free(a, full_to_reduced, map_size);
        ivp_free(a, reduced_to_full, map_size);
        return IVP_ERR_OUT_OF_MEMORY;
    }

    res = ivp_great_matrix_solve(&rm, x_sub);
    if (res == IVP_OK) {
        for (int i = 0; i < n_active; i++) {
            x[reduced_to_full[i]] = x_sub[i];
        }
    }

    ivp_free(a, x_sub, x_sub_size);
    ivp_great_matrix_destroy(&rm, a);
    ivp_free(a, full_to_reduced, map_size);
    ivp_free(a, reduced_to_full, map_size);
    return res;
}

/* ── Determinant only ───────────────────────────────────────────────────── */

ivp_result_t ivp_great_matrix_determinant(ivp_great_matrix_t *m,
                                           ivp_real_t *det_out)
{
    int n = m->n;
    int cols = m->capacity;
    ivp_gm_real_t det = 1.0;
    int swaps = 0;

    for (int j = 0; j < n; j++) {
        int pivot_row = find_pivot(m, j, j);
        if (pivot_row != j) {
            swap_rows(m, j, pivot_row);
            swaps++;
        }

        ivp_gm_real_t diag = m->data[j * cols + j];
        det *= diag;

        if (fabs(diag) < (ivp_gm_real_t)IVP_SOLVER_MATRIX_EPS) {
            *det_out = (ivp_real_t)0.0;
            return IVP_OK; /* singular but determinant is valid (= 0) */
        }

        ivp_gm_real_t inv_diag = 1.0 / diag;
        for (int i = j + 1; i < n; i++) {
            ivp_gm_real_t factor = m->data[i * cols + j] * inv_diag;
            m->data[i * cols + j] = 0.0;
            for (int k = j + 1; k < cols; k++) {
                m->data[i * cols + k] -= factor * m->data[j * cols + k];
            }
        }
    }

    *det_out = (ivp_real_t)((swaps & 1) ? -det : det);
    return IVP_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Reference-faithful port of ivp_great_matrix.cxx
 *   IVP_VecFPU, IVP_Great_Matrix_Many_Zero, IVP_Incr_L_U_Matrix
 * See the comment block in ivp_great_matrix.h for the semantics that are
 * reproduced (64-bit VecFPU windows, FMA contraction sites).
 * Statement order and operation order follow the reference line by line.
 * The reference's debug code (IVP_IF blocks, #ifdef DEBUG) is ported in
 * place (ivp/base/ivp_assert.h): dead in release builds, live in Debug /
 * IVP_ENABLE_ASSERTS builds like the reference debug build.
 * ══════════════════════════════════════════════════════════════════════════ */

/* fma at the sites where the reference compiler contracted `a*b + c`. */
#if IVP_GM_USE_FMA
#define GM_FMA(a, b, c) fma((a), (b), (c))
#else
#define GM_FMA(a, b, c) ((a) * (b) + (c))
#endif

#define GM_BLOCK IVP_GM_VECFPU_SIZE

/* IVP_VECFPU_MEMSHIFT / IVP_VECFPU_MEM_MASK (align_matrix_values) */
#if IVP_GM_VECFPU_SIZE == 4
#define GM_MEMSHIFT 2
#define GM_MEM_MASK (~(uintptr_t)0xf)
#else
#define GM_MEMSHIFT 3
#define GM_MEM_MASK (~(uintptr_t)0x7)
#endif

#define GM_P_DOUBLE_MAX 1e20 /* P_DOUBLE_MAX (double build) */

#if IVP_GM_VECFPU_SIZE == 4
/* IVP_VECFPU_MEM_MASK = ~0xf: an unaligned start moves down to the previous
 * 16-byte boundary (2 doubles) and the count grows by
 * (byte_shift >> IVP_VECFPU_MEMSHIFT) = (8 * elems) >> 2. */
#define GM_ALIGN_ELEMS 2
#define GM_SIZE_ADJUST(elems) ((elems) * 2)
#else
/* 32-bit build: 8-byte mask, MEMSHIFT 3 -> never shifts. */
#define GM_ALIGN_ELEMS 1
#define GM_SIZE_ADJUST(elems) (elems)
#endif

#define GM_STATIC_FRICTION_TEST_EPS ((double)0.00001f)

/* Reductions `sum += a[k]*b[k]` over contiguous data were auto-vectorized
 * by the reference compiler as an in-order (strict) reduction: when the
 * trip count reaches a threshold, the first (count & ~7) terms are computed
 * as a rounded product followed by a rounded add (fmul.2d + fadd), and only
 * the scalar tail uses the contracted FMA.  Shorter loops are fully fused.
 * Thresholds read from the reference binary: 8 everywhere, except the
 * out-of-line solve_lower_null_matrix (14); strided/indexed loops (lu_crout,
 * solve_vec_with_U, the LCS's indexed products) were not vectorized. */
#define GM_VEC_BLOCK 8
#define GM_VEC_MIN 8
#define GM_VEC_MIN_SOLVE_LOWER_NULL 14

static inline int gm_unfused_terms(int count, int threshold)
{
#if IVP_GM_USE_FMA
    return count >= threshold ? (count & ~(GM_VEC_BLOCK - 1)) : 0;
#else
    (void)threshold;
    return count;
#endif
}

/* sum + a[0]*b[0] + ... + a[count-1]*b[count-1], in order */
static inline double gm_reduce_add(double sum, const double *a, const double *b, int count)
{
    int k = 0;
    int unfused = gm_unfused_terms(count, GM_VEC_MIN);
    /* Small LU systems repeatedly traverse this reduction. Unroll only the
     * loop control: keep one accumulator, every product/add rounding point,
     * the original ascending order and the FMA flavour's scalar tail. */
    for (; k <= unfused - 4; k += 4) {
        double p = a[k] * b[k];
        sum = sum + p;
        p = a[k + 1] * b[k + 1];
        sum = sum + p;
        p = a[k + 2] * b[k + 2];
        sum = sum + p;
        p = a[k + 3] * b[k + 3];
        sum = sum + p;
    }
    for (; k < unfused; k++) {
        double p = a[k] * b[k];
        sum = sum + p;
    }
    for (; k < count; k++) {
        sum = GM_FMA(a[k], b[k], sum);
    }
    return sum;
}

/* sum - a[0]*b[0] - ... - a[count-1]*b[count-1], in order */
static inline double gm_reduce_sub(double sum, const double *a, const double *b, int count)
{
    int k = 0;
    int unfused = gm_unfused_terms(count, GM_VEC_MIN);
    for (; k < unfused; k++) {
        double p = (-a[k]) * b[k];
        sum = sum + p;
    }
    for (; k < count; k++) {
        sum = GM_FMA(-a[k], b[k], sum);
    }
    return sum;
}

static inline int gm_misalign(int off)
{
    return off & (GM_ALIGN_ELEMS - 1);
}

/* ── IVP_VecFPU ─────────────────────────────────────────────────────────── */

void ivp_vecfpu_add_multiple_row(double *target_base, int target_off,
                                 const double *source_base, int source_off,
                                 double factor, int size, bool aligned)
{
    if (!aligned) {
        int shift = gm_misalign(source_off);
        target_off -= gm_misalign(target_off);
        size += GM_SIZE_ADJUST(shift);
        source_off -= shift;
    }
    double *t = target_base + target_off;
    const double *s = source_base + source_off;
    /* Blocks never overlap (target and source rows are >= one aligned row
     * apart), so element order inside a block is immaterial. */
#if IVP_ARM64_NEON && IVP_GM_VECFPU_SIZE == 4
    const float64x2_t factors = vdupq_n_f64(factor);
    for (int i = size; i > 0; i -= GM_BLOCK) {
        float64x2_t a0 = vmulq_f64(vld1q_f64(s), factors);
        float64x2_t a1 = vmulq_f64(vld1q_f64(s + 2), factors);
        a0 = vaddq_f64(a0, vld1q_f64(t));
        a1 = vaddq_f64(a1, vld1q_f64(t + 2));
        vst1q_f64(t, a0);
        vst1q_f64(t + 2, a1);
        t += GM_BLOCK;
        s += GM_BLOCK;
    }
#else
    for (int i = size; i > 0; i -= GM_BLOCK) {
        for (int k = 0; k < GM_BLOCK; k++) {
            double a = s[k] * factor;
            a += t[k];
            t[k] = a;
        }
        t += GM_BLOCK;
        s += GM_BLOCK;
    }
#endif
}

void ivp_vecfpu_multiply_row(double *base, int off, double factor,
                             int size, bool aligned)
{
    if (!aligned) {
        int shift = gm_misalign(off);
        size += GM_SIZE_ADJUST(shift);
        off -= shift;
    }
    double *t = base + off;
    for (int i = size; i > 0; i -= GM_BLOCK) {
        for (int k = 0; k < GM_BLOCK; k++) {
            double a = t[k] * factor;
            t[k] = a;
        }
        t += GM_BLOCK;
    }
}

void ivp_vecfpu_exchange_rows(double *base1, int off1, double *base2, int off2,
                              int size, bool aligned)
{
    if (!aligned) {
        int shift = gm_misalign(off1);
        size += GM_SIZE_ADJUST(shift);
        off1 -= shift;
        off2 -= gm_misalign(off2);
    }
    double *t1 = base1 + off1;
    double *t2 = base2 + off2;
    for (int i = size; i > 0; i -= GM_BLOCK) {
        for (int k = 0; k < GM_BLOCK; k++) {
            double h = t1[k];
            t1[k] = t2[k];
            t2[k] = h;
        }
        t1 += GM_BLOCK;
        t2 += GM_BLOCK;
    }
}

void ivp_vecfpu_copy_rows(double *target_base, int target_off,
                          const double *source_base, int source_off,
                          int size, bool aligned)
{
    if (!aligned) {
        int shift = gm_misalign(source_off);
        size += GM_SIZE_ADJUST(shift);
        source_off -= shift;
        target_off -= gm_misalign(target_off);
    }
    double *t = target_base + target_off;
    const double *s = source_base + source_off;
    for (int i = size - 1; i >= 0; i -= GM_BLOCK) {
        for (int j = 0; j < GM_BLOCK; j++) {
            t[j] = s[j];
        }
        t += GM_BLOCK;
        s += GM_BLOCK;
    }
}

void ivp_vecfpu_set_row_to_zero(double *base, int off, int size, bool aligned)
{
    if (!aligned) {
        int shift = gm_misalign(off);
        size += GM_SIZE_ADJUST(shift);
        off -= shift;
    }
    double *t = base + off;
    for (int i = size - 1; i >= 0; i -= GM_BLOCK) {
        for (int j = 0; j < GM_BLOCK; j++) {
            t[j] = 0.0;
        }
        t += GM_BLOCK;
    }
}

double ivp_vecfpu_large_dot_product(const double *base_a, int off_a,
                                    const double *base_b, int off_b,
                                    int size, bool aligned)
{
    if (!aligned) {
        int shift = gm_misalign(off_a);
        off_b -= gm_misalign(off_b);
        size += GM_SIZE_ADJUST(shift);
        off_a -= shift;
    }
    if (size <= 0) return 0.0;
    return gm_reduce_add(0.0, base_a + off_a, base_b + off_b, size);
}

/* ── Block allocation with reference arena layout ───────────────────────── */

/* Lays out `count` arrays of the given byte sizes like consecutive
 * IVP_U_Memory::get_mem calls and returns one zeroed block. */
static void *gm_alloc_layout(const ivp_allocator_t *a, const size_t *bytes,
                             size_t *offsets, int count, size_t *total_out)
{
    size_t off = 0;
    for (int i = 0; i < count; i++) {
        offsets[i] = off;
        off = ivp_gm_arena_next(off, bytes[i]);
    }
    void *block = ivp_alloc(a, off);
    if (!block) return NULL;
    memset(block, 0, off);
    *total_out = off;
    return block;
}

/* ── IVP_Great_Matrix_Many_Zero ─────────────────────────────────────────── */

static ivp_result_t gmmz_solve_lower_null_impl(ivp_gmmz_t *m, int vec_min);

ivp_result_t ivp_gmmz_init(ivp_gmmz_t *m, int n, const ivp_allocator_t *alloc)
{
    if (!m || n < 0 || n > IVP_MAX_GREAT_MATRIX_SIZE) return IVP_ERR_INVALID_ARG;
    const ivp_allocator_t *a = alloc ? alloc : ivp_default_allocator();
    int arl = ivp_gm_aligned_len(n);
    size_t bytes[3];
    size_t offs[3];
    bytes[0] = sizeof(double) * (size_t)n * (size_t)arl;
    bytes[1] = sizeof(double) * (size_t)arl;
    bytes[2] = sizeof(double) * (size_t)arl;
    size_t total = 0;
    char *block = (char *)gm_alloc_layout(a, bytes, offs, 3, &total);
    if (!block) return IVP_ERR_OUT_OF_MEMORY;

    m->matrix_eps      = IVP_GMMZ_DEFAULT_EPS;
    m->columns         = n;
    m->aligned_row_len = arl;
    m->matrix_values   = (double *)(void *)(block + offs[0]);
    m->desired_vector  = (double *)(void *)(block + offs[1]);
    m->result_vector   = (double *)(void *)(block + offs[2]);
    m->storage         = block;
    m->storage_size    = total;
    return IVP_OK;
}

void ivp_gmmz_destroy(ivp_gmmz_t *m, const ivp_allocator_t *alloc)
{
    if (!m) return;
    if (m->storage) {
        const ivp_allocator_t *a = alloc ? alloc : ivp_default_allocator();
        ivp_free(a, m->storage, m->storage_size);
    }
    m->storage        = NULL;
    m->storage_size   = 0;
    m->matrix_values  = NULL;
    m->desired_vector = NULL;
    m->result_vector  = NULL;
    m->columns        = 0;
    m->aligned_row_len = 0;
}

void ivp_gmmz_calc_aligned_row_len(ivp_gmmz_t *m)
{
    m->aligned_row_len = ivp_gm_aligned_len(m->columns);
}

void ivp_gmmz_set_value(ivp_gmmz_t *m, double val, int col, int row)
{
    /* ATT: slow through mults */
    IVP_ASSERT(col >= 0 && col < m->columns);
    IVP_ASSERT(row >= 0 && row < m->columns);
    m->matrix_values[col + m->aligned_row_len * row] = val;
}

double ivp_gmmz_get_value(const ivp_gmmz_t *m, int col, int row)
{
    /* ATT: slow through mults */
    IVP_ASSERT(col >= 0 && col < m->columns);
    IVP_ASSERT(row >= 0 && row < m->columns);
    return m->matrix_values[col + m->aligned_row_len * row];
}

/* tests a line to be greater equal result value */
ivp_result_t ivp_gmmz_check_unequation_line(const ivp_gmmz_t *m, int linenum)
{
    const double *dline = &m->matrix_values[m->aligned_row_len * linenum];
    double left_side = 0.0;
    if (m->columns > 0) {
        left_side = gm_reduce_add(left_side, m->result_vector, dline, m->columns);
    }
    double eps = fabs(m->desired_vector[linenum] * GM_STATIC_FRICTION_TEST_EPS);
    if (left_side + eps >= m->desired_vector[linenum]) {
        return IVP_OK;
    }
    return IVP_GM_FAULT;
}

/* matrix_test_unequation: text out for visualizing */
void ivp_gmmz_matrix_test_unequation(const ivp_gmmz_t *m)
{
    for (int i = 0; i < m->columns; i++) {
        double left = gm_reduce_add(0.0, m->result_vector,
                                    &m->matrix_values[i * m->aligned_row_len], m->columns);
        IVP_IF(1) { ivp_debug_printf("unequation_test left %f right %f\n", left, m->desired_vector[i]); }
    }
}

void ivp_gmmz_matrix_out_before_gauss(const ivp_gmmz_t *m)
{
    IVP_IF(1) {
        for (int i = 0; i < m->columns; i++) {
            for (int j = 0; j < m->columns; j++) {
                ivp_debug_printf("%.5f  ", m->matrix_values[i * m->aligned_row_len + j]);
            }
            ivp_debug_printf("=  %.5f\n", m->desired_vector[i]);
        }
    }
}

/* add factor * line `first` to line `second` (entries left of `first` in
 * both lines are zero) */
void ivp_gmmz_add_multiple_line(ivp_gmmz_t *m, int first, int second, double factor)
{
    int arl = m->aligned_row_len;
    ivp_vecfpu_add_multiple_row(m->matrix_values, second * arl + first,
                                m->matrix_values, first * arl + first,
                                factor, m->columns - first, false);
    m->desired_vector[second] = GM_FMA(m->desired_vector[first], factor,
                                       m->desired_vector[second]);
}

void ivp_gmmz_debug_fill_zero(ivp_gmmz_t *m)
{
    for (int i = 0; i < m->columns; i++) {
        for (int j = 0; j < m->aligned_row_len; j++) {
            m->matrix_values[i * m->aligned_row_len + j] = 0.0;
        }
    }
}

ivp_result_t ivp_gmmz_solve(ivp_gmmz_t *m)
{
#ifdef IVP_DEBUG_BUILD
    IVP_IF(0) {
        ivp_gmmz_plausible_input_check(m);
    }
#endif
    ivp_gmmz_transform_to_lower_null_triangle(m);
    /* the reference inlines this copy: vectorization threshold 8 */
    return gmmz_solve_lower_null_impl(m, GM_VEC_MIN);
}

void ivp_gmmz_copy_matrix(ivp_gmmz_t *dst, const ivp_gmmz_t *src)
{
    IVP_ASSERT(dst->columns == src->columns);
    for (int i = 0; i < dst->columns; i++) {
        for (int j = 0; j < dst->columns; j++) {
            dst->matrix_values[i * dst->aligned_row_len + j] =
                src->matrix_values[i * src->aligned_row_len + j];
        }
        dst->desired_vector[i] = src->desired_vector[i];
    }
}

/* align_matrix_values: shift matrix_values to aligned memory (the caller
 * allocated IVP_VECFPU_SIZE - 1 extra values); the debug build also clears
 * the row padding and the padding of desired_vector. */
void ivp_gmmz_align_matrix_values(ivp_gmmz_t *m)
{
    uintptr_t adress = (uintptr_t)m->matrix_values;
    uintptr_t maximal_shift = (uintptr_t)(IVP_GM_VECFPU_SIZE - 1) << GM_MEMSHIFT;
    uintptr_t result_adress = (adress + maximal_shift) & GM_MEM_MASK;
    m->matrix_values = (double *)result_adress;

    IVP_IF(1) {
        for (int i = 0; i < m->columns; i++) {
            double *x = &m->matrix_values[i * m->aligned_row_len];
            for (int j = m->columns; j < m->aligned_row_len; j++) {
                x[j] = 0.0;
            }
        }
        for (int j = m->columns; j < m->aligned_row_len; j++) {
            m->desired_vector[j] = 0.0;
        }
    }
}

void ivp_gmmz_copy_matrix_to(const ivp_gmmz_t *m, double *values, double *desired)
{
    for (int i = 0; i < m->columns; i++) {
        for (int j = 0; j < m->columns; j++) {
            values[i * m->columns + j] = m->matrix_values[i * m->aligned_row_len + j];
        }
    }
    for (int j = 0; j < m->columns; j++) {
        desired[j] = m->desired_vector[j];
    }
}

void ivp_gmmz_find_pivot_in_column(ivp_gmmz_t *m, int col)
{
    int arl = m->aligned_row_len;
    double best_val = fabs(m->matrix_values[col * arl + col]);
    int pos = -1;
    for (int i = m->columns - 1; i > col; i--) {
        double h = m->matrix_values[i * arl + col];
        if (fabs(h) > best_val) {
            best_val = fabs(h);
            pos = i;
        }
    }
    if (pos >= 0) {
        ivp_gmmz_exchange_rows(m, col, pos);
    }
}

void ivp_gmmz_exchange_rows(ivp_gmmz_t *m, int a, int b)
{
    int arl = m->aligned_row_len;
    ivp_vecfpu_exchange_rows(m->matrix_values, b * arl, m->matrix_values, a * arl,
                             m->columns, true);
    double h = m->desired_vector[a];
    m->desired_vector[a] = m->desired_vector[b];
    m->desired_vector[b] = h;
}

#if IVP_ARM64_NEON && IVP_GM_VECFPU_SIZE == 4
#define GMMZ_BATCHED_ROWS_MIN_COLUMNS 64

/* Apply the same pivot row to two target rows while loading each source
 * block once.  This is only used from column four onward: the reference
 * VecFPU window can write into columns 0..3 of the following row, but those
 * entries are already dead by then and never overlap another target window.
 * Each live element keeps the original rounded multiply followed by add. */
static void gmmz_add_two_multiple_lines(ivp_gmmz_t *m, int first,
                                        int second, double factor0,
                                        double factor1)
{
    int arl = m->aligned_row_len;
    int source_off = first * arl + first;
    int target_off = second * arl + first;
    int size = m->columns - first;
    int shift = gm_misalign(source_off);
    target_off -= gm_misalign(target_off);
    size += GM_SIZE_ADJUST(shift);
    source_off -= shift;

    const double *source = m->matrix_values + source_off;
    double *target0 = m->matrix_values + target_off;
    double *target1 = target0 + arl;
    const float64x2_t factors0 = vdupq_n_f64(factor0);
    const float64x2_t factors1 = vdupq_n_f64(factor1);
    for (int remaining = size; remaining > 0; remaining -= GM_BLOCK) {
        float64x2_t source0 = vld1q_f64(source);
        float64x2_t source1 = vld1q_f64(source + 2);

        float64x2_t value00 = vmulq_f64(source0, factors0);
        float64x2_t value01 = vmulq_f64(source1, factors0);
        value00 = vaddq_f64(value00, vld1q_f64(target0));
        value01 = vaddq_f64(value01, vld1q_f64(target0 + 2));
        vst1q_f64(target0, value00);
        vst1q_f64(target0 + 2, value01);

        float64x2_t value10 = vmulq_f64(source0, factors1);
        float64x2_t value11 = vmulq_f64(source1, factors1);
        value10 = vaddq_f64(value10, vld1q_f64(target1));
        value11 = vaddq_f64(value11, vld1q_f64(target1 + 2));
        vst1q_f64(target1, value10);
        vst1q_f64(target1 + 2, value11);

        source += GM_BLOCK;
        target0 += GM_BLOCK;
        target1 += GM_BLOCK;
    }
    double *desired = m->desired_vector;
    desired[second] = GM_FMA(desired[first], factor0, desired[second]);
    desired[second + 1] =
        GM_FMA(desired[first], factor1, desired[second + 1]);
}

/* Four target rows are independent once the pivot row is fixed.  Reuse the
 * two source vectors across all four rows without changing the multiply/add
 * sequence of any target element. */
static void gmmz_add_four_multiple_lines(ivp_gmmz_t *m, int first,
                                         int second, double factor0,
                                         double factor1, double factor2,
                                         double factor3)
{
    int arl = m->aligned_row_len;
    int source_off = first * arl + first;
    int target_off = second * arl + first;
    int size = m->columns - first;
    int shift = gm_misalign(source_off);
    target_off -= gm_misalign(target_off);
    size += GM_SIZE_ADJUST(shift);
    source_off -= shift;

    const double *source = m->matrix_values + source_off;
    double *target0 = m->matrix_values + target_off;
    double *target1 = target0 + arl;
    double *target2 = target1 + arl;
    double *target3 = target2 + arl;
    const float64x2_t factors0 = vdupq_n_f64(factor0);
    const float64x2_t factors1 = vdupq_n_f64(factor1);
    const float64x2_t factors2 = vdupq_n_f64(factor2);
    const float64x2_t factors3 = vdupq_n_f64(factor3);
    for (int remaining = size; remaining > 0; remaining -= GM_BLOCK) {
        float64x2_t source0 = vld1q_f64(source);
        float64x2_t source1 = vld1q_f64(source + 2);

        float64x2_t value00 = vmulq_f64(source0, factors0);
        float64x2_t value01 = vmulq_f64(source1, factors0);
        value00 = vaddq_f64(value00, vld1q_f64(target0));
        value01 = vaddq_f64(value01, vld1q_f64(target0 + 2));
        vst1q_f64(target0, value00);
        vst1q_f64(target0 + 2, value01);

        float64x2_t value10 = vmulq_f64(source0, factors1);
        float64x2_t value11 = vmulq_f64(source1, factors1);
        value10 = vaddq_f64(value10, vld1q_f64(target1));
        value11 = vaddq_f64(value11, vld1q_f64(target1 + 2));
        vst1q_f64(target1, value10);
        vst1q_f64(target1 + 2, value11);

        float64x2_t value20 = vmulq_f64(source0, factors2);
        float64x2_t value21 = vmulq_f64(source1, factors2);
        value20 = vaddq_f64(value20, vld1q_f64(target2));
        value21 = vaddq_f64(value21, vld1q_f64(target2 + 2));
        vst1q_f64(target2, value20);
        vst1q_f64(target2 + 2, value21);

        float64x2_t value30 = vmulq_f64(source0, factors3);
        float64x2_t value31 = vmulq_f64(source1, factors3);
        value30 = vaddq_f64(value30, vld1q_f64(target3));
        value31 = vaddq_f64(value31, vld1q_f64(target3 + 2));
        vst1q_f64(target3, value30);
        vst1q_f64(target3 + 2, value31);

        source += GM_BLOCK;
        target0 += GM_BLOCK;
        target1 += GM_BLOCK;
        target2 += GM_BLOCK;
        target3 += GM_BLOCK;
    }
    double *desired = m->desired_vector;
    desired[second] = GM_FMA(desired[first], factor0, desired[second]);
    desired[second + 1] =
        GM_FMA(desired[first], factor1, desired[second + 1]);
    desired[second + 2] =
        GM_FMA(desired[first], factor2, desired[second + 2]);
    desired[second + 3] =
        GM_FMA(desired[first], factor3, desired[second + 3]);
}

/* Keep the common small-matrix reference loop compact.  This separate large
 * path pays for its extra branching only where source-row reuse is material. */
__attribute__((noinline))
static void gmmz_transform_batched_rows(ivp_gmmz_t *m)
{
    int arl = m->aligned_row_len;
    for (int j = 0; j < m->columns; j++) {
        ivp_gmmz_find_pivot_in_column(m, j);
        double diagonal_elem = m->matrix_values[j * arl + j];
        if (fabs(diagonal_elem) < m->matrix_eps) continue;
        double negativ_inv_diagonal = -1.0 / diagonal_elem;
        int i = j + 1;
        for (; j >= IVP_GM_VECFPU_SIZE && i + 3 < m->columns;
             i += 4) {
            double value0 = m->matrix_values[i * arl + j];
            double value1 = m->matrix_values[(i + 1) * arl + j];
            double value2 = m->matrix_values[(i + 2) * arl + j];
            double value3 = m->matrix_values[(i + 3) * arl + j];
            if (fabs(value0) > m->matrix_eps &&
                fabs(value1) > m->matrix_eps &&
                fabs(value2) > m->matrix_eps &&
                fabs(value3) > m->matrix_eps) {
                gmmz_add_four_multiple_lines(
                    m, j, i,
                    value0 * negativ_inv_diagonal,
                    value1 * negativ_inv_diagonal,
                    value2 * negativ_inv_diagonal,
                    value3 * negativ_inv_diagonal);
                continue;
            }
            for (int row = i; row < i + 4; row++) {
                double value = m->matrix_values[row * arl + j];
                if (fabs(value) > m->matrix_eps) {
                    ivp_gmmz_add_multiple_line(
                        m, j, row, value * negativ_inv_diagonal);
                }
            }
        }
        for (; j >= IVP_GM_VECFPU_SIZE && i + 1 < m->columns;
             i += 2) {
            double value0 = m->matrix_values[i * arl + j];
            double value1 = m->matrix_values[(i + 1) * arl + j];
            if (fabs(value0) > m->matrix_eps &&
                fabs(value1) > m->matrix_eps) {
                double factor0 = value0 * negativ_inv_diagonal;
                double factor1 = value1 * negativ_inv_diagonal;
                gmmz_add_two_multiple_lines(m, j, i, factor0, factor1);
                continue;
            }
            for (int row = i; row < i + 2; row++) {
                double value = m->matrix_values[row * arl + j];
                if (fabs(value) > m->matrix_eps) {
                    ivp_gmmz_add_multiple_line(
                        m, j, row, value * negativ_inv_diagonal);
                }
            }
        }
        for (; i < m->columns; i++) {
            double value = m->matrix_values[i * arl + j];
            if (fabs(value) > m->matrix_eps) {
                ivp_gmmz_add_multiple_line(
                    m, j, i, value * negativ_inv_diagonal);
            }
        }
    }
}
#endif

/* Gauss: upper triangle with zeros in the lower part (always pivots) */
void ivp_gmmz_transform_to_lower_null_triangle(ivp_gmmz_t *m)
{
#if IVP_ARM64_NEON && IVP_GM_VECFPU_SIZE == 4
    if (m->columns >= GMMZ_BATCHED_ROWS_MIN_COLUMNS) {
        gmmz_transform_batched_rows(m);
        return;
    }
#endif
    int arl = m->aligned_row_len;
    for (int j = 0; j < m->columns; j++) {
        ivp_gmmz_find_pivot_in_column(m, j);
        double diagonal_elem = m->matrix_values[j * arl + j];
        if (fabs(diagonal_elem) < m->matrix_eps) {
            continue; /* column_done */
        }
        double negativ_inv_diagonal = -1.0 / diagonal_elem;
        for (int i = j + 1; i < m->columns; i++) {
            double col_down_elem = m->matrix_values[i * arl + j];
            if (fabs(col_down_elem) > m->matrix_eps) {
                ivp_gmmz_add_multiple_line(m, j, i, col_down_elem * negativ_inv_diagonal);
            }
        }
    }
}

#ifdef IVP_DEBUG_BUILD
void ivp_gmmz_plausible_input_check(const ivp_gmmz_t *m)
{
    int i, j;
    for (i = 0; i < m->columns; i++) {
        for (j = 0; j < m->columns; j++) {
            double a = m->matrix_values[i * m->aligned_row_len + j];
            IVP_ASSERT(a > -GM_P_DOUBLE_MAX);
            IVP_ASSERT(a < GM_P_DOUBLE_MAX);
        }
        double b = m->desired_vector[i];
        IVP_ASSERT(b > -GM_P_DOUBLE_MAX);
        IVP_ASSERT(b < GM_P_DOUBLE_MAX);
    }
}
#endif

/* matrix is upper triangular, solve from bottom to top; vec_min is the
 * vectorization threshold of the reference copy being reproduced */
static ivp_result_t gmmz_solve_lower_null_impl(ivp_gmmz_t *m, int vec_min)
{
    int columns = m->columns;
    int arl = m->aligned_row_len;
    double *mv = m->matrix_values;
    double *dv = m->desired_vector;
    int i;
    for (i = columns - 1; i >= 0; i--) {
        const double *pointer_in_line = &mv[i * arl + columns - 1];
        double right_side = dv[i];
        int unfused = gm_unfused_terms(columns - 1 - i, vec_min);
        for (int j = columns - 1; j > i; j--) {
            if (unfused > 0) {
                double p = (-dv[j]) * *pointer_in_line;
                right_side = right_side + p;
                unfused--;
            } else {
                right_side = GM_FMA(-dv[j], *pointer_in_line, right_side);
            }
            pointer_in_line--;
        }
        double left_side = *pointer_in_line;
        if (fabs(left_side) < m->matrix_eps) {
            if (fabs(right_side) < m->matrix_eps * 1000.0) {
                /* rank not maximal: choose 0.0 for this element */
                dv[i] = 0.0;
                IVP_IF(0) {
                    ivp_debug_printf("gauss_singular_matrix\n");
                }
            } else {
                /* no result possible */
                i = columns - 1;
                while (i >= 0) {
                    m->result_vector[i] = 0.0;
                    i--;
                }
                return IVP_GM_FAULT;
            }
        } else {
            dv[i] = right_side / left_side;
            if (dv[i] > (double)10E11f) {
                IVP_IF(1) { ivp_debug_printf("\ngauss_failure\n\n"); }
            }
        }
    }
    for (i = 0; i < columns; i++) {
        m->result_vector[i] = dv[i];
    }
    return IVP_OK;
}

ivp_result_t ivp_gmmz_solve_lower_null_matrix(ivp_gmmz_t *m)
{
    return gmmz_solve_lower_null_impl(m, GM_VEC_MIN_SOLVE_LOWER_NULL);
}

void ivp_gmmz_matrix_multiplication(ivp_gmmz_t *m, const double *first,
                                    const double *second)
{
    int c = m->columns;
    for (int i = 0; i < c; i++) {
        for (int j = 0; j < c; j++) {
            double sum = 0.0;
            for (int k = 0; k < c; k++) {
                double val = first[i * c + k] * second[k * c + j];
                sum += val;
            }
            m->matrix_values[i * c + j] = sum;
        }
    }
}

/* for debug only: desired_vector and the matrix are changed by the solve,
 * old_matrix (stride columns) and old_desired hold the input */
int ivp_gmmz_test_result(const ivp_gmmz_t *m, const double *old_matrix, const double *old_desired)
{
    IVP_IF(1) {
        int i, j;
        int columns = m->columns;
        for (j = 0; j < columns; j++) {
            /* walk through lines */
            double left_side = 0.0;
            for (i = 0; i < columns; i++) {
                left_side = GM_FMA(old_matrix[j * columns + i], m->result_vector[i], left_side);
                ivp_debug_printf("multip %.5f %.5f  ", old_matrix[j * columns + i], m->result_vector[i]);
            }
            ivp_debug_printf("\nwanted_res was %.5f now %.5f\n", old_desired[j], left_side);
        }
    }
    return 0;
}

/* both matrices aligned; copy the wanted rows/columns of big_matrix */
void ivp_gmmz_fill_from_bigger_matrix(ivp_gmmz_t *m, const ivp_gmmz_t *big,
                                      const int *original_pos, int n_column)
{
    if (n_column == 0) return;
    int arl = m->aligned_row_len;
    if (original_pos[n_column - 1] == n_column - 1) { /* no reorder */
        IVP_IF(1) {
            for (int i = 0; i < n_column; i++) {
                IVP_ASSERT(original_pos[i] == i);
            }
        }
        for (int i = 0; i < n_column; i++) {
            ivp_vecfpu_copy_rows(m->matrix_values, i * arl,
                                 big->matrix_values, original_pos[i] * big->aligned_row_len,
                                 arl, true);
            m->desired_vector[i] = big->desired_vector[original_pos[i]];
        }
    } else {
        for (int i = 0; i < n_column; i++) {
            double *dest = &m->matrix_values[i * arl];
            IVP_IF((original_pos[i] != i)) {
                ivp_debug_printf("original_pos_diff %d %d\n", i, original_pos[i]);
            }
            const double *source = &big->matrix_values[original_pos[i] * big->aligned_row_len];
            for (int j = 0; j < n_column; j++) {
                dest[j] = source[original_pos[j]];
            }
            m->desired_vector[i] = big->desired_vector[original_pos[i]];
        }
    }
}

/* values_big_matrix is a packed external buffer (columns x columns, row
 * stride columns, like copy_matrix_to writes it); the sub matrix keeps its
 * own aligned stride (reference cf18246) */
void ivp_gmmz_copy_to_sub_matrix(const ivp_gmmz_t *m, const double *values_big_matrix,
                                 ivp_gmmz_t *sub_matrix, const int *original_pos)
{
    for (int i = 0; i < sub_matrix->columns; i++) {
        for (int j = 0; j < sub_matrix->columns; j++) {
            sub_matrix->matrix_values[i * sub_matrix->aligned_row_len + j] =
                values_big_matrix[original_pos[i] * m->columns + original_pos[j]];
        }
    }
}

int ivp_gmmz_print_great_matrix(const ivp_gmmz_t *m, const char *comment)
{
    ivp_message("Matrix: %s\n", comment);
    int i;
    for (i = 0; i < m->columns; i++) { /* rows */
        int j;
        for (j = 0; j < m->columns; j++) { /* columns */
            double val = ivp_gmmz_get_value(m, j, i);
            if (fabs(val) < (double)1e-20f) {
                ivp_message("    0  ");
            } else {
                ivp_message("%2.6g  ", val);
            }
        }
        ivp_message("\n");
    }
    ivp_message("desired ");
    for (i = 0; i < m->columns; i++) {
        ivp_message("%.6f ", m->desired_vector[i]);
    }
    ivp_message("\n");
    return 0; /* for dbx */
}

void ivp_gmmz_mult_aligned(ivp_gmmz_t *m)
{
    for (int j = 0; j < m->columns; j++) {
        const double *source = &m->matrix_values[j * m->aligned_row_len];
        m->result_vector[j] = gm_reduce_add(0.0, source, m->desired_vector, m->columns);
    }
}

void ivp_gmmz_mult(ivp_gmmz_t *m)
{
    for (int j = 0; j < m->columns; j++) {
        m->result_vector[j] = gm_reduce_add(0.0, &m->matrix_values[j * m->columns],
                                            m->desired_vector, m->columns);
    }
}

ivp_result_t ivp_gmmz_lu_crout(ivp_gmmz_t *m, int *index_vec, double *sign)
{
    /* desired_vector stores the implicit scalings */
    int i, imax = 0, j, k, zeros = 0;
    double sum;
    double big, temp;
    int n_columns = m->columns;
    double *mv = m->matrix_values;
    double *dv = m->desired_vector;

    *sign = 1.0;

    for (i = 0; i < n_columns; i++) {
        big = 0.0;
        for (j = 0; j < n_columns; j++) {
            temp = fabs(mv[i * n_columns + j]);
            if (temp > big) big = temp;
        }
        dv[i] = (big == 0.0 ? 0.0 : 1.0 / big);
    }

    for (j = 0; j < n_columns; j++) {
        /* jth column from top to diagonal: elements of U */
        for (i = 0; i < j; i++) {
            sum = mv[i * n_columns + j];
            for (k = 0; k < i; k++) {
                sum = GM_FMA(-mv[i * n_columns + k], mv[k * n_columns + j], sum);
            }
            mv[i * n_columns + j] = sum;
        }

        /* residuals below the diagonal, largest scaled term -> imax */
        big = 0.0;
        for (i = j; i < n_columns; i++) {
            sum = mv[i * n_columns + j];
            for (k = 0; k < j; k++) {
                sum = GM_FMA(-mv[i * n_columns + k], mv[k * n_columns + j], sum);
            }
            mv[i * n_columns + j] = sum;
            temp = dv[i] * fabs(sum);
            if (temp >= big) {
                big = temp;
                imax = i;
            }
        }

        if (j != imax) {
            for (k = 0; k < n_columns; k++) {
                temp = mv[imax * n_columns + k];
                mv[imax * n_columns + k] = mv[j * n_columns + k];
                mv[j * n_columns + k] = temp;
            }
            *sign = -*sign;
            dv[imax] = dv[j];
        }
        index_vec[j] = imax;

        if (fabs(mv[j * n_columns + j]) < m->matrix_eps) {
            zeros++;
        } else if (j != n_columns - 1) {
            temp = 1.0 / mv[j * n_columns + j];
            for (i = j + 1; i < n_columns; i++) {
                mv[i * n_columns + j] *= temp;
            }
        }
    }
    if (zeros) {
        return IVP_GM_FAULT;
    }
    return IVP_OK;
}

ivp_result_t ivp_gmmz_lu_inverse(ivp_gmmz_t *m, ivp_gmmz_t *matrix_out,
                                 const int *index_vec)
{
    /* solve the identity column by column */
    for (int i = 0; i < m->columns; i++) {
        for (int j = 0; j < m->columns; j++) {
            m->desired_vector[j] = 0.0;
        }
        m->desired_vector[i] = 1.0;

        if (ivp_gmmz_lu_solve(m, index_vec) == IVP_OK) {
            for (int j = 0; j < m->columns; j++) {
                matrix_out->matrix_values[j * m->columns + i] = m->result_vector[j];
            }
        } else {
            return IVP_GM_FAULT;
        }
    }
    return IVP_OK;
}

ivp_result_t ivp_gmmz_lu_solve(ivp_gmmz_t *m, const int *index_vec)
{
    int i, nonzero = -1, iperm, j;
    double sum, zero = 0.0;
    int n_columns = m->columns;
    const double *mv = m->matrix_values;
    double *dv = m->desired_vector;

    /* check for zero diagonals */
    for (i = 0; i < n_columns; i++) {
        if (fabs(mv[i * n_columns + i]) < m->matrix_eps) {
            return IVP_GM_FAULT;
        }
    }

    /* transform b allowing for leading zeros */
    for (i = 0; i < n_columns; i++) {
        iperm = index_vec[i];
        sum = dv[iperm];
        dv[iperm] = dv[i];
        if (nonzero >= 0) {
            j = nonzero - 1;
            if (j <= i - 1) {
                sum = gm_reduce_sub(sum, &mv[i * n_columns + j], &dv[j], i - j);
            }
        } else if (sum != zero) {
            nonzero = i + 1;
        }
        dv[i] = sum;
    }

    /* backsubstitution */
    for (i = n_columns - 1; i >= 0; i--) {
        sum = dv[i];
        j = i + 1;
        if (j < n_columns) {
            sum = gm_reduce_sub(sum, &mv[i * n_columns + j], &dv[j], n_columns - j);
        }
        dv[i] = (sum / mv[i * n_columns + i]);
        m->result_vector[i] = dv[i];
    }
    return IVP_OK;
}

ivp_result_t ivp_gmmz_invert(ivp_gmmz_t *m, ivp_gmmz_t *out)
{
    /* NOTE: out memory already has to be allocated */
    IVP_ASSERT(out->columns == m->columns);
    /* reference: alloca(columns * sizeof(IVP_DOUBLE)) */
    int index_vec[IVP_MAX_GREAT_MATRIX_SIZE];
    double sign;
    if (m->columns > IVP_MAX_GREAT_MATRIX_SIZE) return IVP_ERR_INVALID_ARG;

    if (ivp_gmmz_lu_crout(m, index_vec, &sign) == IVP_GM_FAULT) {
        return IVP_GM_FAULT;
    }
    return ivp_gmmz_lu_inverse(m, out, index_vec);
}

/* ── IVP_Incr_L_U_Matrix ────────────────────────────────────────────────── */

ivp_result_t ivp_incr_lu_init(ivp_incr_lu_t *lu, int n_max, const ivp_allocator_t *alloc)
{
    if (!lu || n_max < 0 || n_max > IVP_MAX_GREAT_MATRIX_SIZE) return IVP_ERR_INVALID_ARG;
    const ivp_allocator_t *a = alloc ? alloc : ivp_default_allocator();
    int arl = ivp_gm_aligned_len(n_max);
    size_t bytes[6];
    size_t offs[6];
    bytes[0] = sizeof(double) * (size_t)arl * (size_t)n_max;
    bytes[1] = bytes[0];
    for (int i = 2; i < 6; i++) bytes[i] = sizeof(double) * (size_t)arl;
    size_t total = 0;
    char *block = (char *)gm_alloc_layout(a, bytes, offs, 6, &total);
    if (!block) return IVP_ERR_OUT_OF_MEMORY;

    lu->matrix_eps      = (double)1E-7f * 10;
    lu->L_matrix        = (double *)(void *)(block + offs[0]);
    lu->U_matrix        = (double *)(void *)(block + offs[1]);
    lu->input_vec       = (double *)(void *)(block + offs[2]);
    lu->out_vec         = (double *)(void *)(block + offs[3]);
    lu->temp_vec        = (double *)(void *)(block + offs[4]);
    lu->mult_vec        = (double *)(void *)(block + offs[5]);
    lu->aligned_row_len = arl;
    lu->n_sub           = 0;
    lu->storage         = block;
    lu->storage_size    = total;
    return IVP_OK;
}

void ivp_incr_lu_destroy(ivp_incr_lu_t *lu, const ivp_allocator_t *alloc)
{
    if (!lu) return;
    if (lu->storage) {
        const ivp_allocator_t *a = alloc ? alloc : ivp_default_allocator();
        ivp_free(a, lu->storage, lu->storage_size);
    }
    memset(lu, 0, sizeof(*lu));
}

ivp_result_t ivp_incr_lu_normize_row_L(ivp_incr_lu_t *lu, int row_nr)
{
    int row_base = row_nr * lu->aligned_row_len;
    double *base = &lu->L_matrix[row_base];
    double val = base[row_nr];
    if (fabs(val) < lu->matrix_eps) {
        return IVP_GM_FAULT;
    }
    double inv_val = 1.0 / val;
    ivp_vecfpu_multiply_row(lu->L_matrix, row_base, inv_val, lu->n_sub, false);
    base[row_nr] = 1.0;
    return IVP_OK;
}

ivp_result_t ivp_incr_lu_normize_row(ivp_incr_lu_t *lu, int row_nr)
{
    int row_base = row_nr * lu->aligned_row_len;
    double val = lu->U_matrix[row_base + row_nr];
    if (fabs(val) < lu->matrix_eps) {
        return IVP_GM_FAULT;
    }
    double inv_val = 1.0 / val;
    ivp_vecfpu_multiply_row(lu->L_matrix, row_base, inv_val, lu->n_sub, false);
    ivp_vecfpu_multiply_row(lu->U_matrix, row_base + row_nr + 1, inv_val,
                            lu->n_sub - row_nr - 1, false);
    lu->U_matrix[row_base + row_nr] = 1.0;
    return IVP_OK;
}

void ivp_incr_lu_exchange_rows(ivp_incr_lu_t *lu, int pivot_col, int exchange)
{
    int base_a = pivot_col * lu->aligned_row_len;
    int base_b = exchange * lu->aligned_row_len;
    ivp_vecfpu_exchange_rows(lu->U_matrix, base_a + pivot_col,
                             lu->U_matrix, base_b + pivot_col,
                             lu->n_sub - pivot_col, false);
    ivp_vecfpu_exchange_rows(lu->L_matrix, base_a, lu->L_matrix, base_b,
                             lu->n_sub, false);
}

/* search from col_nr|col_nr down to n_sub|col_nr */
void ivp_incr_lu_pivot_search(ivp_incr_lu_t *lu, int col_nr)
{
    int arl = lu->aligned_row_len;
    double biggest = fabs(lu->U_matrix[col_nr * arl + col_nr]);
    int pivot_row = col_nr;
    for (int i = lu->n_sub - 1; i > col_nr; i--) {
        double new_val = fabs(lu->U_matrix[i * arl + col_nr]);
        if (new_val > biggest) {
            pivot_row = i;
            biggest = new_val;
        }
    }
    if (pivot_row != col_nr) {
        ivp_incr_lu_exchange_rows(lu, col_nr, pivot_row);
    }
}

/* add row to a row more upwards (unused by the reference solvers) */
void ivp_incr_lu_add_neg_row_upwards(ivp_incr_lu_t *lu, int row_down, int dest_row,
                                     double factor)
{
    int arl = lu->aligned_row_len;
    int base_a = row_down * arl;
    int base_b = dest_row * arl;
    ivp_vecfpu_add_multiple_row(lu->U_matrix, base_b + row_down + 1,
                                lu->U_matrix, base_a + row_down + 1,
                                -factor, lu->n_sub - row_down - 1, false);
    ivp_vecfpu_add_multiple_row(lu->L_matrix, base_b, lu->L_matrix, base_a,
                                -factor, lu->n_sub, true);
    lu->L_matrix[base_b + row_down] = 0.0;
}

/* add row to a row more downwards */
void ivp_incr_lu_add_neg_row_to_row(ivp_incr_lu_t *lu, int pivot_row, int dest_row,
                                    double factor)
{
    int base_a = pivot_row * lu->aligned_row_len;
    int base_b = dest_row * lu->aligned_row_len;
    ivp_vecfpu_add_multiple_row(lu->U_matrix, base_b + pivot_row + 1,
                                lu->U_matrix, base_a + pivot_row + 1,
                                -factor, lu->n_sub - pivot_row - 1, false);
    ivp_vecfpu_add_multiple_row(lu->L_matrix, base_b, lu->L_matrix, base_a,
                                -factor, lu->n_sub, false);
    lu->U_matrix[base_b + pivot_row] = 0.0;
}

void ivp_incr_lu_subtract_row_L(ivp_incr_lu_t *lu, int from, int dest, double factor)
{
    int base_a = from * lu->aligned_row_len;
    int base_b = dest * lu->aligned_row_len;
    ivp_vecfpu_add_multiple_row(lu->L_matrix, base_b, lu->L_matrix, base_a,
                                -factor, lu->n_sub, false);
    lu->L_matrix[base_b + from] = 0.0;
}

ivp_result_t ivp_incr_lu_decompose(ivp_incr_lu_t *lu)
{
    if (lu->n_sub == 0) {
        return IVP_OK;
    }
    int arl = lu->aligned_row_len;
    int i;
    for (i = lu->n_sub - 1; i >= 0; i--) {
        ivp_vecfpu_set_row_to_zero(lu->L_matrix, i * arl, lu->n_sub, false);
        lu->L_matrix[i * arl + i] = 1.0;
    }

    for (int rows_to_do = 1; rows_to_do < lu->n_sub; rows_to_do++) {
        int pivot_elem = rows_to_do - 1;
        ivp_incr_lu_pivot_search(lu, pivot_elem);
        if (ivp_incr_lu_normize_row(lu, pivot_elem) == IVP_GM_FAULT) {
            return IVP_GM_FAULT;
        }
        for (int rows_become_zero = lu->n_sub - 1; rows_become_zero >= rows_to_do;
             rows_become_zero--) {
            double factor = lu->U_matrix[rows_become_zero * arl + pivot_elem];
            if (factor != 0.0) { /* original matrix may be filled with many zeros */
                ivp_incr_lu_add_neg_row_to_row(lu, pivot_elem, rows_become_zero, factor);
            }
        }
    }
    if (ivp_incr_lu_normize_row(lu, lu->n_sub - 1) == IVP_GM_FAULT) {
        return IVP_GM_FAULT;
    }
    return IVP_OK;
}

/* incremental decomposition: matrix is decomposed for n_sub x n_sub;
 * new last row of U (length n_sub+1) is already stored, new last column
 * of A is in input_vec (length n_sub) */
ivp_result_t ivp_incr_lu_increment(ivp_incr_lu_t *lu)
{
    int arl = lu->aligned_row_len;
    int i;
    for (i = lu->n_sub - 1; i >= 0; i--) {
        lu->mult_vec[i] = lu->input_vec[i];
    }

    ivp_incr_lu_mult_vec_with_L(lu);

    for (i = lu->n_sub - 1; i >= 0; i--) {
        lu->U_matrix[i * arl + lu->n_sub] = lu->temp_vec[i];
    }

    /* clear row and column of L */
    for (i = lu->n_sub - 1; i >= 0; i--) {
        lu->L_matrix[i * arl + lu->n_sub] = 0.0;
    }
    ivp_vecfpu_set_row_to_zero(lu->L_matrix, lu->n_sub * arl, lu->n_sub, false);

    lu->L_matrix[lu->n_sub * arl + lu->n_sub] = 1.0;

    int base_row = lu->n_sub * arl;

    lu->n_sub++;
    for (i = 0; i < lu->n_sub - 1; i++) {
        double factor = lu->U_matrix[base_row + i];
        ivp_incr_lu_add_neg_row_to_row(lu, i, lu->n_sub - 1, factor);
    }

    return ivp_incr_lu_normize_row(lu, lu->n_sub - 1);
}

/* variable del_nr is removed from the linear equation system */
ivp_result_t ivp_incr_lu_decrement(ivp_incr_lu_t *lu, int del_nr)
{
    int arl = lu->aligned_row_len;
    int i;
    ivp_incr_lu_exchange_columns_L(lu, del_nr, lu->n_sub - 1);
    ivp_incr_lu_exchange_columns_U(lu, del_nr, lu->n_sub - 1);

    for (i = lu->n_sub - 2; i > del_nr; i--) {
        double factor = lu->U_matrix[i * arl + del_nr];
        if (factor != 0.0) {
            ivp_incr_lu_add_neg_row_L(lu, lu->n_sub - 1, i, factor);
        }
        lu->U_matrix[i * arl + del_nr] = 0.0;
    }
    if (ivp_incr_lu_normize_row(lu, del_nr) != IVP_OK) {
        ivp_incr_lu_add_neg_row_L(lu, lu->n_sub - 1, del_nr, -1.0);
        lu->U_matrix[del_nr * arl + del_nr] = 1.0;
    }

    for (i = del_nr; i < lu->n_sub - 1; i++) {
        double factor = lu->U_matrix[(lu->n_sub - 1) * arl + i];
        if (factor != 0.0) {
            ivp_incr_lu_add_neg_row_to_row(lu, i, lu->n_sub - 1, factor);
        }
    }

    if (ivp_incr_lu_normize_row_L(lu, lu->n_sub - 1) != IVP_OK) {
        lu->n_sub--;
        return IVP_GM_FAULT;
    }

    del_nr = lu->n_sub - 1;
    for (i = del_nr - 1; i >= 0; i--) {
        double factor = lu->L_matrix[i * arl + del_nr];
        if (factor != 0.0) {
            ivp_incr_lu_subtract_row_L(lu, del_nr, i, factor);
        }
    }

    lu->n_sub--;
    return IVP_OK;
}

/* unused by the reference solvers */
void ivp_incr_lu_add_neg_col_L(ivp_incr_lu_t *lu, int col0, int col_dest, double factor)
{
    int arl = lu->aligned_row_len;
    for (int i = lu->n_sub - 1; i >= 0; i--) {
        lu->L_matrix[i * arl + col_dest] =
            GM_FMA(-factor, lu->L_matrix[i * arl + col0], lu->L_matrix[i * arl + col_dest]);
    }
}

void ivp_incr_lu_exchange_columns_L(ivp_incr_lu_t *lu, int col0, int col1)
{
    int arl = lu->aligned_row_len;
    for (int i = 0; i < lu->n_sub; i++) {
        double temp_val = lu->L_matrix[i * arl + col0];
        lu->L_matrix[i * arl + col0] = lu->L_matrix[i * arl + col1];
        lu->L_matrix[i * arl + col1] = temp_val;
    }
}

void ivp_incr_lu_exchange_columns_U(ivp_incr_lu_t *lu, int col0, int col1)
{
    int arl = lu->aligned_row_len;
    for (int i = 0; i < lu->n_sub; i++) {
        double temp_val = lu->U_matrix[i * arl + col0];
        lu->U_matrix[i * arl + col0] = lu->U_matrix[i * arl + col1];
        lu->U_matrix[i * arl + col1] = temp_val;
    }
}

void ivp_incr_lu_mult_vec_with_L(ivp_incr_lu_t *lu)
{
    for (int i = lu->n_sub - 1; i >= 0; i--) {
        double sum = ivp_vecfpu_large_dot_product(lu->L_matrix, i * lu->aligned_row_len,
                                                  lu->mult_vec, 0, lu->n_sub, true);
        lu->temp_vec[i] = sum;
    }
}

void ivp_incr_lu_solve_vec_with_U(ivp_incr_lu_t *lu)
{
    int arl = lu->aligned_row_len;
    for (int i = lu->n_sub - 1; i >= 0; i--) {
        double leftsum = 0.0;
        int j = lu->n_sub - 1;
        /* The reference reduces in descending column order. Do not split
         * this into partial sums or replace it with a forward dot product. */
        for (; j > i + 3; j -= 4) {
            leftsum = GM_FMA(lu->U_matrix[i * arl + j], lu->temp_vec[j], leftsum);
            leftsum = GM_FMA(lu->U_matrix[i * arl + j - 1], lu->temp_vec[j - 1], leftsum);
            leftsum = GM_FMA(lu->U_matrix[i * arl + j - 2], lu->temp_vec[j - 2], leftsum);
            leftsum = GM_FMA(lu->U_matrix[i * arl + j - 3], lu->temp_vec[j - 3], leftsum);
        }
        for (; j > i; j--) {
            leftsum = GM_FMA(lu->U_matrix[i * arl + j], lu->temp_vec[j], leftsum);
        }
        double res = lu->temp_vec[i] - leftsum;
        lu->temp_vec[i] = res;
        lu->out_vec[i] = res;
    }
}

void ivp_incr_lu_solve_lin_equ(ivp_incr_lu_t *lu)
{
    for (int i = lu->n_sub - 1; i >= 0; i--) {
        lu->mult_vec[i] = lu->input_vec[i];
    }
    ivp_incr_lu_mult_vec_with_L(lu);
    ivp_incr_lu_solve_vec_with_U(lu);
}

void ivp_incr_lu_add_neg_row_L(ivp_incr_lu_t *lu, int source_row, int dest_row,
                               double factor)
{
    int base_a = source_row * lu->aligned_row_len;
    int base_b = dest_row * lu->aligned_row_len;
    ivp_vecfpu_add_multiple_row(lu->L_matrix, base_b, lu->L_matrix, base_a,
                                -factor, lu->n_sub, false);
}

void ivp_incr_lu_debug_print_l_u(const ivp_incr_lu_t *lu)
{
    int i;
    ivp_debug_printf("  L                                      U\n");
    for (i = 0; i < lu->n_sub; i++) {
        int j;
        for (j = 0; j < lu->n_sub; j++) {
            ivp_debug_printf("%.5f  ", lu->L_matrix[i * lu->aligned_row_len + j]);
        }
        ivp_debug_printf("          ");
        for (j = 0; j < lu->n_sub; j++) {
            ivp_debug_printf("%.5f  ", lu->U_matrix[i * lu->aligned_row_len + j]);
        }
        ivp_debug_printf("\n");
    }
}

/* IVP_Great_Matrix_Many_Zero(int n) (ivp_great_matrix.cxx:728): the
 * reference constructor asserts in its debug build ("does not work any
 * longer: because of Vector-FPU adress has to be aligned"); here
 * ivp_gmmz_init, which allocates aligned storage. */
static ivp_result_t gmmz_init_n(ivp_gmmz_t *m, int n)
{
    IVP_ASSERT(0); /* does not work any longer: because of Vector-FPU adress has to be aligned */
    return ivp_gmmz_init(m, n, NULL);
}

/* prints inverse(L) * U = the original A (the reference builds its
 * temporary matrices with the IVP_Great_Matrix_Many_Zero(int n) constructor,
 * here gmmz_init_n; they are freed afterwards) */
void ivp_incr_lu_debug_print_a(const ivp_incr_lu_t *lu)
{
    int n_sub = lu->n_sub;
    ivp_gmmz_t was_l, was_u, inv_l, new_a;
    if (gmmz_init_n(&was_l, n_sub) != IVP_OK) return;
    if (gmmz_init_n(&was_u, n_sub) != IVP_OK) {
        ivp_gmmz_destroy(&was_l, NULL);
        return;
    }
    for (int i = 0; i < n_sub; i++) {
        for (int j = 0; j < n_sub; j++) {
            was_l.matrix_values[i * n_sub + j] = lu->L_matrix[i * lu->aligned_row_len + j];
            was_u.matrix_values[i * n_sub + j] = lu->U_matrix[i * lu->aligned_row_len + j];
        }
    }
    if (gmmz_init_n(&inv_l, n_sub) == IVP_OK) {
        (void)ivp_gmmz_invert(&was_l, &inv_l);
        if (gmmz_init_n(&new_a, n_sub) == IVP_OK) {
            ivp_gmmz_matrix_multiplication(&new_a, inv_l.matrix_values, was_u.matrix_values);
            (void)ivp_gmmz_print_great_matrix(&new_a, "orig_A");
            ivp_gmmz_destroy(&new_a, NULL);
        }
        ivp_gmmz_destroy(&inv_l, NULL);
    }
    ivp_gmmz_destroy(&was_u, NULL);
    ivp_gmmz_destroy(&was_l, NULL);
}
