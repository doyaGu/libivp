/* ivp_great_matrix.c — Dense linear system solver
 *
 * Gauss elimination with partial pivoting + back substitution.
 * Matches original IVP IVP_Great_Matrix_Many_Zero behavior:
 *   - Column-wise partial pivoting (max absolute value)
 *   - Ties broken by lower index (deterministic)
 *   - MATRIX_EPS threshold for singularity detection
 */
#include <ivp/solver/ivp_great_matrix.h>
#include <ivp/solver/ivp_solver_metrics.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/math/ivp_fast_math.h>
#include <string.h>
#include <math.h>

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

    /* Record metrics if active */
    {
        ivp_solver_metrics_t *met = ivp_solver_metrics_get_active();
        if (met) {
            /* Count how many contacts were deactivated */
            int deactivated = 0;
            for (int i = 0; i < n; i++) {
                if (!active[i]) deactivated++;
            }
            met->unilateral_deactivation_steps += deactivated;
        }
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
