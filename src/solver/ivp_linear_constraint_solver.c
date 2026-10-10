/* ivp_linear_constraint_solver.c — port of IVP_Linear_Constraint_Solver
 *
 * Line-by-line port of the reference (ivp_great_matrix.cxx).  Statement
 * order, operation order, constants, goto structure and the deterministic
 * "random" permutation are kept as in the reference.  `a*b + c` written in
 * one expression is evaluated with fma() (see ivp_great_matrix.h).
 *
 * The reference's debug code is ported in place as IVP_IF blocks (dead in
 * release builds, live in Debug / IVP_ENABLE_ASSERTS builds, see
 * ivp/base/ivp_assert.h), including its side effects on the arena: the
 * zero fill at the end of alloc_memory, the extra debug_mat allocation and
 * the final result check (which overwrites delta_accel with A*x).  debug_lcs
 * is 0 as in the reference, so its verbose output stays off.
 *
 * Memory: all scratch arrays come from an IVP_U_Memory arena
 * (ivp_u_memory_t), in the order and with the byte sizes of
 * IVP_Linear_Constraint_Solver::alloc_memory, and the caller's A, b and x are
 * used in place.  The friction solver passes the environment's persistent
 * short-term arena.  The reference-state build preserves the stale padding
 * and VecFPU gap bytes that the original reads; the mainline build clears the
 * complete reserved spans before the solver sees them.
 */
#include <ivp/solver/ivp_linear_constraint_solver.h>
#include <ivp/solver/ivp_great_matrix.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_u_memory.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <math.h>
#include <string.h>

/* debug helpers the reference never calls (debug_dep_var,
 * exchange_activ_inactiv) */
#if defined(__GNUC__) || defined(__clang__)
#define LCS_MAYBE_UNUSED __attribute__((unused))
#else
#define LCS_MAYBE_UNUSED
#endif
#ifdef _MSC_VER
#pragma warning(disable : 4505) /* unreferenced local function */
#endif

#if IVP_GM_USE_FMA
#define LCS_FMA(a, b, c) fma((a), (b), (c))
#else
#define LCS_FMA(a, b, c) ((a) * (b) + (c))
#endif

#define LCS_P_DOUBLE_MAX 1e20 /* P_DOUBLE_MAX (double build) */

typedef struct lcs_state {
    double  SOLVER_EPS;
    double  GAUSS_EPS;
    double  TEST_EPS;
    double  MAX_STEP_LEN;
    double *full_A;
    double *full_b;
    double *temp;
    double *full_x;       /* must be greater equal 0.0             */
    double *delta_f;      /* delta of above per step               */
    double *accel;        /* is A*x-b, must be greater equal 0.0   */
    double *delta_accel;
    double *reset_x;
    double *reset_accel;
    int    *actives_inactives_ignored;
    int    *variable_is_found_at;
    int     n_variables;
    int     aligned_size;
    int     r_actives;
    int     aligned_sub_size;
    int     ignored_pos;
    int     debug_lcs;
    int     debug_no_lu_count;
    int     first_permute_index;
    int     second_permute_index;
    int     first_permute_ignored;
    int     second_permute_ignored;
    int     sub_solver_status; /* 0 is ok, 1 is not usable */
    ivp_incr_lu_t lu_sub_solver;
    ivp_gmmz_t    sub_solver_mat;
    ivp_gmmz_t    full_solver_mat;
    ivp_gmmz_t    debug_mat;
    ivp_gmmz_t    inv_mat;
} lcs_state_t;

/* ── index bookkeeping ──────────────────────────────────────────────────── */

/* both numbers are in sub-space */
static void lcs_exchange_lcs_variables(lcs_state_t *s, int first_nr, int second_nr)
{
    IVP_IF(1) {
        IVP_ASSERT(s->variable_is_found_at[s->actives_inactives_ignored[second_nr]] == second_nr);
        IVP_ASSERT(s->variable_is_found_at[s->actives_inactives_ignored[first_nr]] == first_nr);
    }
    IVP_ASSERT(first_nr >= 0);
    IVP_ASSERT(second_nr >= 0);
    IVP_ASSERT(first_nr < s->n_variables);
    IVP_ASSERT(second_nr < s->n_variables);
    int full_space_0 = s->actives_inactives_ignored[second_nr];
    int full_space_1 = s->actives_inactives_ignored[first_nr];
    s->actives_inactives_ignored[first_nr] = full_space_0;
    s->actives_inactives_ignored[second_nr] = full_space_1;
    s->variable_is_found_at[full_space_0] = first_nr;
    s->variable_is_found_at[full_space_1] = second_nr;
}

/* please merge these two functions !!!! (unused by the reference) */
static LCS_MAYBE_UNUSED void lcs_exchange_activ_inactiv(lcs_state_t *s, int change_var)
{
    IVP_IF(1) {
        IVP_ASSERT(s->variable_is_found_at[s->actives_inactives_ignored[change_var]] == change_var);
        IVP_ASSERT(s->variable_is_found_at[s->actives_inactives_ignored[s->r_actives]] == s->r_actives);
    }
    int first_inactiv = s->r_actives;
    int full_space_0 = s->actives_inactives_ignored[change_var];
    int full_space_1 = s->actives_inactives_ignored[first_inactiv];
    s->actives_inactives_ignored[first_inactiv] = full_space_0;
    s->actives_inactives_ignored[change_var] = full_space_1;
    s->variable_is_found_at[full_space_0] = first_inactiv;
    s->variable_is_found_at[full_space_1] = change_var;
}

static void lcs_move_variable_to_end(lcs_state_t *s, int var_nr)
{
    for (int i = var_nr + 1; i < s->n_variables; i++) {
        lcs_exchange_lcs_variables(s, i - 1, i);
    }
}

/* ── sub solver maintenance ─────────────────────────────────────────────── */

static void lcs_decrement_sub_solver(lcs_state_t *s, int sub_pos)
{
    if (s->sub_solver_status == 0) {
        IVP_ASSERT(s->lu_sub_solver.n_sub == s->r_actives + 1);
        if (ivp_incr_lu_decrement(&s->lu_sub_solver, sub_pos) != IVP_OK) {
            IVP_IF(1) {
                ivp_debug_printf("\n\ndecrement_lu_failed_need_setup\n\n");
            }
            s->sub_solver_status = 2;
        }
    } else {
        s->lu_sub_solver.n_sub--;
    }
}

static void lcs_increment_sub_solver(lcs_state_t *s)
{
    int k;
    ivp_incr_lu_t *lu = &s->lu_sub_solver;
    int new_var_orig = s->actives_inactives_ignored[s->r_actives - 1];
    if (s->sub_solver_status == 0) {
        IVP_ASSERT(lu->n_sub == s->r_actives - 1);
        for (k = 0; k < s->r_actives; k++) {
            lu->U_matrix[lu->n_sub * lu->aligned_row_len + k] =
                s->full_A[new_var_orig * s->aligned_size + s->actives_inactives_ignored[k]];
        }
        for (k = 0; k < s->r_actives - 1; k++) {
            lu->input_vec[k] =
                s->full_A[s->aligned_size * s->actives_inactives_ignored[k] + new_var_orig];
        }
        if (ivp_incr_lu_increment(lu) != IVP_OK) {
            IVP_IF(s->debug_lcs) {
                ivp_debug_printf("\n\nincrement_lu_failed_need_setup\n\n");
            }
            s->sub_solver_status = 2;
        }
    } else {
        lu->n_sub++;
    }
}

/* in case numerics get unstable, never-ending loops can occur: permute a
 * little to get fresh values (deterministic counters, no PRNG) */
static void lcs_do_a_little_random_permutation(lcs_state_t *s)
{
    if (s->r_actives < 2) {
        return;
    }

    s->first_permute_index += 1;
    s->second_permute_index += 2;

    while (s->first_permute_index >= s->r_actives) {
        s->first_permute_index -= s->r_actives;
    }
    while (s->second_permute_index >= s->r_actives) {
        s->second_permute_index -= s->r_actives;
    }
    lcs_exchange_lcs_variables(s, s->first_permute_index, s->second_permute_index);

    s->first_permute_ignored += 1;
    s->second_permute_ignored += 2;

    int free_space = s->n_variables - s->ignored_pos - 1;
    if (free_space < 2) {
        return;
    }
    while (s->first_permute_ignored >= free_space) {
        s->first_permute_ignored -= free_space;
    }
    while (s->second_permute_ignored >= free_space) {
        s->second_permute_ignored -= free_space;
    }
    lcs_exchange_lcs_variables(s, s->first_permute_ignored + s->ignored_pos + 1,
                               s->second_permute_ignored + s->ignored_pos + 1);
}

static void lcs_move_not_necessary_actives_to_inactives(lcs_state_t *s)
{
    for (int i = 0; i < s->r_actives; i++) {
        if (s->full_x[s->actives_inactives_ignored[i]] < s->SOLVER_EPS) {
            s->full_x[s->actives_inactives_ignored[i]] = 0.0;
            lcs_exchange_lcs_variables(s, i, s->r_actives - 1);
            s->r_actives--;
            lcs_decrement_sub_solver(s, i);
            i--;
        }
    }
}

/* ── products ───────────────────────────────────────────────────────────── */

/* indexed access, cannot be vectorized */
static void lcs_mult_active_x_for_accel(lcs_state_t *s)
{
    for (int i = s->r_actives; i < s->n_variables; i++) {
        double sum = 0.0;
        int row_of_A = s->actives_inactives_ignored[i];
        const double *row_vals = &s->full_solver_mat.matrix_values[row_of_A * s->aligned_size];
        int real_pos;
        for (int j = 0; j < s->r_actives; j++) {
            real_pos = s->actives_inactives_ignored[j];
            sum = LCS_FMA(s->delta_f[real_pos], row_vals[real_pos], sum);
        }
        real_pos = s->actives_inactives_ignored[s->ignored_pos];
        sum = LCS_FMA(row_vals[real_pos], 1.0, sum);
        s->delta_accel[row_of_A] = sum;
    }
}

static void lcs_mult_x_with_full_A_minus_b(lcs_state_t *s)
{
    for (int i = 0; i < s->n_variables; i++) {
        double sum = ivp_vecfpu_large_dot_product(s->full_A, i * s->aligned_size,
                                                  s->full_x, 0, s->n_variables, true);
        s->temp[i] = sum - s->full_b[i];
    }
}

static bool lcs_numerical_stability_ok(lcs_state_t *s)
{
    lcs_mult_x_with_full_A_minus_b(s);
    int k;
    double val;
    double worst_error = 0.0;
    for (k = 0; k < s->r_actives; k++) {
        val = fabs(s->temp[s->actives_inactives_ignored[k]]);
        IVP_IF(1) {
            if (worst_error < val) {
                worst_error = val;
            }
        }
        if (val > s->TEST_EPS) {
            IVP_IF(s->debug_lcs) {
                ivp_debug_printf("numerical_unstable %d %.10f should be zero\n", s->ignored_pos, val);
            }
            return false;
        }
    }
    for (k = s->r_actives; k < s->n_variables; k++) {
        int idx = s->actives_inactives_ignored[k];
        val = fabs(s->temp[idx] - s->accel[idx]);
        IVP_IF(1) {
            if (worst_error < val) {
                worst_error = val;
            }
        }
        if (val > s->TEST_EPS) {
            IVP_IF(s->debug_lcs) {
                ivp_debug_printf("numerical_unstable %d %.10f should be %.10f\n", s->ignored_pos,
                                 s->temp[idx], s->accel[idx]);
            }
            return false;
        }
    }
    IVP_IF(0) {
        ivp_debug_printf("numerical_worst %.5e\n", worst_error);
    }
    return true;
}

static void lcs_update_step_vars(lcs_state_t *s, double step_size)
{
    int i;
    IVP_IF(s->debug_lcs) {
        ivp_debug_printf("deltaa  ");
    }
    for (i = 0; i < s->n_variables; i++) {
        IVP_IF(s->debug_lcs) {
            ivp_debug_printf("%f ", s->delta_accel[i]);
        }
        s->accel[i] = LCS_FMA(step_size, s->delta_accel[i], s->accel[i]);
        s->full_x[i] = LCS_FMA(step_size, s->delta_f[i], s->full_x[i]);
    }
    IVP_IF(s->debug_lcs) {
        ivp_debug_printf("\n");
    }

    /* e.g. overriding an inactive: acceleration of inactives gets negative */
    for (i = s->r_actives; i < s->ignored_pos; i++) {
        if (s->accel[s->actives_inactives_ignored[i]] < 0.0) {
            s->accel[s->actives_inactives_ignored[i]] = 0.0;
        }
    }
    /* same with actives */
    for (i = 0; i < s->r_actives; i++) {
        if (s->full_x[s->actives_inactives_ignored[i]] < 0.0) {
            s->full_x[s->actives_inactives_ignored[i]] = 0.0;
        }
    }
}

/* ── direction / setup ──────────────────────────────────────────────────── */

static ivp_result_t lcs_get_fdirection(lcs_state_t *s)
{
    int i, k;
    for (i = 0; i < s->aligned_size; i++) {
        s->delta_f[i] = 0.0;
    }

    if (s->r_actives == 0) {
        return IVP_OK;
    }

    ivp_result_t ret_val = IVP_GM_FAULT;
    ivp_incr_lu_t *lu = &s->lu_sub_solver;

    if (s->sub_solver_status == 0) {
        int ignored_index = s->actives_inactives_ignored[s->ignored_pos];
        for (k = 0; k < s->r_actives; k++) {
            int index = s->actives_inactives_ignored[k];
            lu->input_vec[k] = -s->full_A[index * s->aligned_size + ignored_index];
        }
        ivp_incr_lu_solve_lin_equ(lu);
        ret_val = IVP_OK;

        IVP_IF(s->debug_lcs) {
            ivp_gmmz_t *sub = &s->sub_solver_mat;
            sub->columns = s->r_actives;
            ivp_gmmz_calc_aligned_row_len(sub);
            /* we know our matrix is stable, so it is save to raise EPS */
            sub->matrix_eps = s->GAUSS_EPS * (double)0.001f;
            s->inv_mat.columns = s->r_actives;
            s->debug_mat.columns = s->r_actives;

            for (i = 0; i < s->r_actives; i++) {
                int index = s->actives_inactives_ignored[i];
                sub->desired_vector[i] = -s->full_A[index * s->aligned_size + ignored_index];
                s->debug_mat.desired_vector[i] = -s->full_A[index * s->aligned_size + ignored_index];
                for (int j = 0; j < s->r_actives; j++) {
                    int index2 = s->actives_inactives_ignored[j];
                    sub->matrix_values[i * sub->aligned_row_len + j] =
                        s->full_A[index * s->aligned_size + index2];
                    s->inv_mat.matrix_values[i * s->r_actives + j] =
                        s->full_A[index * s->aligned_size + index2];
                }
            }
            ivp_result_t ret_val2 = ivp_gmmz_solve(sub);
            ivp_result_t ret_val3 = IVP_GM_FAULT; /* inv_mat.invert(&debug_mat); */
            if (ret_val2 == IVP_OK) {
                for (i = 0; i < s->r_actives; i++) {
                    double diff = lu->out_vec[i] - sub->result_vector[i];
                    if (fabs(diff) > (double)0.001f) {
                        IVP_IF(s->debug_lcs) {
                            ivp_debug_printf("gauss_lu_test_fail %f %f should be equal\n",
                                             lu->out_vec[i], sub->result_vector[i]);
                        }
                    }
                }
            } else {
                IVP_IF(s->debug_lcs) {
                    ivp_debug_printf("gauss_lu_test_gauss_invalid\n");
                }
            }
            if (ret_val3 == IVP_OK) {
                ivp_gmmz_mult(&s->debug_mat);
                for (i = 0; i < s->r_actives; i++) {
                    double diff = lu->out_vec[i] - s->debug_mat.result_vector[i];
                    if (fabs(diff) > (double)0.001f) {
                        IVP_IF(s->debug_lcs) {
                            ivp_debug_printf("inv_lu_test_fail %f %f should be equal\n",
                                             lu->out_vec[i], s->debug_mat.result_vector[i]);
                        }
                    }
                }
            }
            sub->matrix_eps = s->GAUSS_EPS;
        }
    } else {
        ivp_gmmz_t *sub = &s->sub_solver_mat;
        IVP_IF((s->debug_lcs)) {
            ivp_debug_printf("sub_lu_is_off %d\n", s->r_actives);
        }
        s->debug_no_lu_count++;
        sub->columns = s->r_actives;
        ivp_gmmz_calc_aligned_row_len(sub);
        int ignored_index = s->actives_inactives_ignored[s->ignored_pos];

        for (i = 0; i < s->r_actives; i++) {
            int index = s->actives_inactives_ignored[i];
            sub->desired_vector[i] = -s->full_A[index * s->aligned_size + ignored_index];
            for (int j = 0; j < s->r_actives; j++) {
                int index2 = s->actives_inactives_ignored[j];
                sub->matrix_values[i * sub->aligned_row_len + j] =
                    s->full_A[index * s->aligned_size + index2];
            }
        }
        ret_val = ivp_gmmz_solve(sub);
        for (i = 0; i < s->r_actives; i++) {
            lu->out_vec[i] = sub->result_vector[i];
        }
    }

    /* copy clamped */
    for (i = 0; i < s->r_actives; i++) {
        s->delta_f[s->actives_inactives_ignored[i]] = lu->out_vec[i];
    }

    s->delta_f[s->actives_inactives_ignored[s->ignored_pos]] = 1.0;

    return ret_val;
}

static void lcs_start_debug_lcs(const lcs_state_t *s)
{
    ivp_debug_printf("b_vals ");
    int i;
    int j;
    for (i = 0; i < s->n_variables; i++) {
        ivp_debug_printf("%.4f ", s->full_b[i]);
    }
    ivp_debug_printf("\n");

    ivp_debug_printf("matrix:\n");
    for (i = 0; i < s->n_variables; i++) {
        for (j = 0; j < s->n_variables; j++) {
            ivp_debug_printf("%.4f ", s->full_A[i * s->aligned_size + j]);
        }
        ivp_debug_printf("\n");
    }
    ivp_debug_printf("\n");
}

static void lcs_debug_out_lcs(lcs_state_t *s)
{
    ivp_debug_printf("r_actives %d  ignored_nr %d\n", s->r_actives, s->ignored_pos);
    int i;

    if (s->ignored_pos > s->n_variables) {
        s->ignored_pos = s->n_variables;
    }

    ivp_debug_printf("actives_inactives  ");
    for (i = 0; i < s->r_actives; i++) {
        ivp_debug_printf("%d ", s->actives_inactives_ignored[i]);
    }
    ivp_debug_printf("  ");
    for (i = s->r_actives; i < s->ignored_pos; i++) {
        ivp_debug_printf("%d ", s->actives_inactives_ignored[i]);
    }
    ivp_debug_printf("  ");
    for (i = s->ignored_pos; i < s->n_variables; i++) {
        ivp_debug_printf("%d ", s->actives_inactives_ignored[i]);
    }
    ivp_debug_printf("\n");

    ivp_debug_printf("variable_is_found  ");
    for (i = 0; i < s->r_actives; i++) {
        ivp_debug_printf("%d ", s->variable_is_found_at[i]);
    }
    ivp_debug_printf("  ");
    for (i = s->r_actives; i < s->ignored_pos; i++) {
        ivp_debug_printf("%d ", s->variable_is_found_at[i]);
    }
    ivp_debug_printf("  ");
    for (i = s->ignored_pos; i < s->n_variables; i++) {
        ivp_debug_printf("%d ", s->variable_is_found_at[i]);
    }
    ivp_debug_printf("\n");

    ivp_debug_printf("x-vals ");
    for (i = 0; i < s->n_variables; i++) {
        ivp_debug_printf("%.4f ", s->full_x[i]);
    }
    ivp_debug_printf("\n");

    s->full_solver_mat.desired_vector = s->full_x;
    ivp_gmmz_mult_aligned(&s->full_solver_mat); /* temporary missuse */
    s->full_solver_mat.desired_vector = s->delta_f;

    ivp_debug_printf("accel1 ");
    for (i = 0; i < s->n_variables; i++) {
        ivp_debug_printf("%.4f ", s->full_solver_mat.result_vector[i] - s->full_b[i]);
    }
    ivp_debug_printf("\n");

    ivp_debug_printf("incr_accel ");
    for (i = 0; i < s->n_variables; i++) {
        ivp_debug_printf("%.4f ", s->accel[i]);
    }
    ivp_debug_printf("\n\n");
}

/* indexed access, cannot be vectorized */
static ivp_result_t lcs_setup_l_u_solver(lcs_state_t *s)
{
    ivp_incr_lu_t *lu = &s->lu_sub_solver;
    IVP_IF(0) {
        ivp_debug_printf("setting_up_lu\n");
    }
    for (int i = 0; i < s->r_actives; i++) {
        int index = s->actives_inactives_ignored[i];
        lu->input_vec[i] = s->full_b[index];
        const double *source = &s->full_A[index * s->aligned_size];
        double *dest = &lu->U_matrix[i * lu->aligned_row_len];
        for (int j = 0; j < s->r_actives; j++) {
            int index2 = s->actives_inactives_ignored[j];
            dest[j] = source[index2];
        }
    }
    return ivp_incr_lu_decompose(lu);
}

/* The friction solver overwhelmingly starts with a fully active 16-variable
 * system.  At startup both permutation arrays are still the identity, so the
 * indexed gather above is a straight row copy.  Keep the decomposition itself
 * shared: this shortcut changes only the copy, not a single floating-point
 * operation. */
static ivp_result_t lcs_setup_l_u_solver_identity_16(lcs_state_t *s)
{
    ivp_incr_lu_t *lu = &s->lu_sub_solver;
    for (int i = 0; i < 16; i++) {
        lu->input_vec[i] = s->full_b[i];
        memcpy(&lu->U_matrix[i * 16], &s->full_A[i * 16],
               16u * sizeof(double));
    }
    return ivp_incr_lu_decompose(lu);
}

static void lcs_bubble_sort_x_vals(lcs_state_t *s)
{
    const int *aii = s->actives_inactives_ignored;
    int r1 = 0;
    int r2;
    for (;;) {
        r2 = r1 + 1;
        if (r2 >= s->r_actives) {
            break;
        }
        if (s->full_x[aii[r1]] < s->full_x[aii[r2]]) {
            lcs_exchange_lcs_variables(s, r1, r2);
            int r_t = r1;
            while ((r_t > 0) && (s->full_x[aii[r_t - 1]] < s->full_x[aii[r_t]])) {
                lcs_exchange_lcs_variables(s, r_t, r_t - 1);
                r_t--;
            }
        }
        r1 = r2;
    }

    IVP_IF(1) {
        int i;
        for (i = 1; i < s->r_actives; i++) {
            if (s->full_x[aii[i - 1]] < s->full_x[aii[i]]) {
                ivp_debug_printf("lcs_bubble_failed\n");
            }
        }
    }
}

static void lcs_get_values_when_setup(lcs_state_t *s)
{
    s->full_solver_mat.result_vector = s->reset_accel;
    s->full_solver_mat.desired_vector = s->reset_x; /* temporary misuse */
    ivp_gmmz_mult_aligned(&s->full_solver_mat);
    s->full_solver_mat.result_vector = s->delta_accel;
    s->full_solver_mat.desired_vector = s->delta_f;

    int i;
    for (i = 0; i < s->n_variables; i++) {
        s->reset_accel[i] -= s->full_b[i];
    }

    for (i = 0; i < s->r_actives; i++) {
        s->reset_accel[s->actives_inactives_ignored[i]] = 0.0; /* remove rounding errors */
    }

    for (i = s->n_variables - 1; i >= 0; i--) {
        s->accel[i] = s->reset_accel[i];
        s->full_x[i] = s->reset_x[i];
    }
}

static void lcs_startup_setup(lcs_state_t *s, int try_actives)
{
    ivp_incr_lu_t *lu = &s->lu_sub_solver;
    s->r_actives = try_actives;
    s->aligned_sub_size = ivp_gm_aligned_len(s->r_actives);
    s->ignored_pos = try_actives;
    lu->n_sub = s->r_actives;

    int i;
    for (i = 0; i < s->aligned_size; i++) {
        s->reset_x[i] = 0.0;
    }
    ivp_result_t setup_result;
    if (s->n_variables == 16 && try_actives == 16) {
        setup_result = lcs_setup_l_u_solver_identity_16(s);
    } else {
        setup_result = lcs_setup_l_u_solver(s);
    }
    if (setup_result == IVP_OK) {
        s->sub_solver_status = 0;
        /* input vec for lu_sub_solver is filled within setup_l_u_solver */

        for (;;) {
        begin_of_while:
            ivp_incr_lu_solve_lin_equ(lu);
            for (i = 0; i < s->r_actives; i++) {
                int index = s->actives_inactives_ignored[i];
                s->reset_x[index] = lu->out_vec[i];
            }
            lcs_get_values_when_setup(s); /* fills x[i] and accel[i] */

            for (i = s->r_actives - 1; i >= 0; i--) {
                int index = s->actives_inactives_ignored[i];
                if (s->full_x[index] < 0.0) {
                    s->full_x[index] = 0.0;
                    s->reset_x[index] = 0.0;
                    s->r_actives--;
                    s->ignored_pos--;
                    lcs_exchange_lcs_variables(s, s->r_actives, i);
                    lcs_decrement_sub_solver(s, i);
                    if (s->sub_solver_status > 0) {
                        for (i = 0; i < s->n_variables; i++) {
                            s->accel[i] = 0.0;
                            s->full_x[i] = 0.0;
                        }
                        goto fast_setup_failed;
                    } else {
                        for (i = 0; i < s->r_actives; i++) {
                            index = s->actives_inactives_ignored[i];
                            lu->input_vec[i] = s->full_b[index];
                        }
                        goto begin_of_while;
                    }
                }
            }
            break;
        }
    } else {
    fast_setup_failed:
        /* sorry fast setup failed */
        s->r_actives = 0;
        s->ignored_pos = 0;
        lu->n_sub = 0;
    }
}

/* returns number of removed actives */
static int lcs_full_setup_test_ranges(lcs_state_t *s)
{
    const int *aii = s->actives_inactives_ignored;
    int illegal_vars = 0;
    int i;
    for (i = 0; i < s->r_actives; i++) {
        if (s->full_x[aii[i]] < s->SOLVER_EPS) {
            if (s->full_x[aii[i]] > -s->SOLVER_EPS) {
                s->full_x[aii[i]] = 0.0;
                lcs_exchange_lcs_variables(s, i, s->r_actives - 1);
            } else {
                lcs_move_variable_to_end(s, i);
                illegal_vars++;
                s->ignored_pos--;
            }
            i--;
            s->r_actives--;
            s->lu_sub_solver.n_sub--;
            s->sub_solver_status = 1;
        }
    }
    for (i = s->r_actives; i < s->ignored_pos; i++) {
        if (s->accel[aii[i]] < 0.0) {
            lcs_move_variable_to_end(s, i);
            i--;
            s->ignored_pos--;
        }
    }
    return illegal_vars;
}

/* the reference recurses (tail call) while illegal variables were cleared */
static ivp_result_t lcs_full_setup(lcs_state_t *s)
{
    ivp_incr_lu_t *lu = &s->lu_sub_solver;
    for (;;) {
        IVP_IF(0) {
            ivp_debug_printf("doing_fulll_setup\n");
        }

        int i;
        lu->n_sub = s->r_actives;

        for (i = 0; i < s->aligned_size; i++) {
            s->reset_x[i] = 0.0;
        }

        lcs_do_a_little_random_permutation(s);
        if (lcs_setup_l_u_solver(s) == IVP_OK) {
            s->sub_solver_status = 0;
            /* input vec for lu_sub_solver is filled within setup_l_u_solver */
            ivp_incr_lu_solve_lin_equ(lu);
            for (i = 0; i < s->r_actives; i++) {
                int index = s->actives_inactives_ignored[i];
                s->reset_x[index] = lu->out_vec[i];
            }
        } else {
            s->sub_solver_status = 2;
            IVP_IF(s->debug_lcs) {
                ivp_debug_printf("setup_lu_failed\n");
            }

            /* move actives with lowest x to the end (matrix is singular and
             * last vars get a zero) */
            lcs_bubble_sort_x_vals(s);
            lcs_do_a_little_random_permutation(s);

            ivp_gmmz_t *sub = &s->sub_solver_mat;
            sub->columns = s->r_actives;
            ivp_gmmz_calc_aligned_row_len(sub);

            for (i = 0; i < s->r_actives; i++) {
                int index = s->actives_inactives_ignored[i];
                sub->desired_vector[i] = s->full_b[index];
                for (int j = 0; j < s->r_actives; j++) {
                    int index2 = s->actives_inactives_ignored[j];
                    sub->matrix_values[i * sub->aligned_row_len + j] =
                        s->full_A[index * s->aligned_size + index2];
                }
            }
            ivp_result_t ret_val = ivp_gmmz_solve(sub);

            if (ret_val != IVP_OK) {
                IVP_IF(1) {
                    ivp_debug_printf("error_setup_gauss_failed_too\n");
                }
                return ret_val;
            }

            for (i = 0; i < s->r_actives; i++) {
                int index = s->actives_inactives_ignored[i];
                s->reset_x[index] = sub->result_vector[i];
            }
        }

        lcs_get_values_when_setup(s);

        if (lcs_full_setup_test_ranges(s) > 0) {
            IVP_IF(s->debug_lcs) {
                ivp_debug_printf("\ncleared_some_illegal_vars\n\n");
            }
            continue; /* return full_setup(); */
        }
        return IVP_OK;
    }
}

static void lcs_debug_test_all_values(lcs_state_t *s)
{
    s->sub_solver_mat.columns = s->r_actives;
    int i;

    for (i = 0; i < s->aligned_size; i++) {
        s->reset_x[i] = s->full_x[i];
    }

    s->full_solver_mat.result_vector = s->reset_accel;
    s->full_solver_mat.desired_vector = s->reset_x; /* temporary misuse */
    ivp_gmmz_mult_aligned(&s->full_solver_mat);
    s->full_solver_mat.result_vector = s->delta_accel;
    s->full_solver_mat.desired_vector = s->delta_f;

    for (i = 0; i < s->aligned_size; i++) {
        s->reset_accel[i] -= s->full_b[i];
    }

    IVP_IF(s->debug_lcs) {
        for (i = 0; i < s->n_variables; i++) {
            double diff = fabs(s->reset_accel[i] - s->accel[i]);
            if (diff > (double)0.001f) {
                ivp_debug_printf("accel_violated at %d %f should be %f\n", i, s->reset_accel[i], s->accel[i]);
            }
            diff = fabs(s->full_x[i] - s->reset_x[i]);
            if (diff > (double)0.001f) {
                ivp_debug_printf("x_violated at %d\n", i);
            }
        }
    }
}

/* when matrix is singular give dependences of a variable (unused by the
 * reference) */
static LCS_MAYBE_UNUSED void lcs_debug_dep_var(lcs_state_t *s, int full_var_nr)
{
    const int *aii = s->actives_inactives_ignored;
    ivp_gmmz_t *sub = &s->sub_solver_mat;
    const double *full_mat = s->full_solver_mat.matrix_values;
    int col_nr = 0;
    int sub_var_nr = s->variable_is_found_at[full_var_nr];
    if (sub_var_nr >= s->r_actives) {
        ivp_debug_printf("works_only_with_active\n");
        return;
    }
    int i, j;
    sub->columns = s->r_actives;
    ivp_gmmz_calc_aligned_row_len(sub);

    for (i = 0; i < sub_var_nr; i++) {
        for (j = 0; j < s->r_actives; j++) {
            int full_i = aii[i];
            int full_j = aii[j];
            sub->matrix_values[j * sub->aligned_row_len + col_nr] = full_mat[full_j * s->aligned_size + full_i];
        }
        col_nr++;
    }
    for (i = sub_var_nr + 1; i < s->r_actives; i++) {
        for (j = 0; j < s->r_actives; j++) {
            int full_i = aii[i];
            int full_j = aii[j];
            sub->matrix_values[j * sub->aligned_row_len + col_nr] = full_mat[full_j * s->aligned_size + full_i];
        }
        col_nr++;
    }
    for (j = 0; j < s->r_actives; j++) {
        sub->matrix_values[j * s->r_actives + s->r_actives - 1] = 0.0;
        int full_i = aii[sub_var_nr];
        int full_j = aii[j];
        sub->desired_vector[j] = full_mat[full_j * s->aligned_size + full_i];
    }

    if (ivp_gmmz_solve(sub) == IVP_OK) {
        ivp_debug_printf("dependency %d is ", full_var_nr);
        col_nr = 0;
        for (i = 0; i < sub_var_nr; i++) {
            ivp_debug_printf("%d:%f ", aii[i], sub->result_vector[col_nr]);
            col_nr++;
        }
        for (i = sub_var_nr + 1; i < s->r_actives; i++) {
            ivp_debug_printf("%d:%f ", aii[i], sub->result_vector[col_nr]);
            col_nr++;
        }
        ivp_debug_printf("\n");
    } else {
        ivp_debug_printf("variable %d is independent\n", full_var_nr);
    }
}

/* ── main loop ──────────────────────────────────────────────────────────────
 * actives_inactives_ignored maps the growing sub space to the full space:
 * first the active variables (accel 0, x > 0), then the inactives (x 0,
 * accel > 0), last the ignored ones (first of them is processed). */
static ivp_result_t lcs_solve_lc(lcs_state_t *s)
{
    int index;
    int step_count = 0;
    int nr_zero_steps = 0;  /* to avoid endless loops */
    int did_real_step = 0;  /* active variables have changed */
    int next_numeric_stability_test = IVP_LCS_NUMERIC_TEST_EVERY_N;
    double solver_eps = s->SOLVER_EPS;
    int limiting_var;
    int ignored_full_index;
    double smallest_step = 0.0;
    int i;
    int indx;
    double *accel = s->accel;
    double *full_x = s->full_x;
    double *delta_f = s->delta_f;
    double *delta_accel = s->delta_accel;
    const int *aii = s->actives_inactives_ignored;

    for (;;) {
        if (did_real_step) {
            step_count++;
        }
        if (step_count > IVP_LCS_MAX_STEP) {
            IVP_IF(1) {
                ivp_debug_printf("\n\ngiveup_stepcount_linear_constraint_solver\n\n\n");
            }
            return IVP_GM_FAULT;
        }

        if (next_numeric_stability_test == 0) {
            if (!lcs_numerical_stability_ok(s)) {
            perform_full_setup:
                step_count++;
                if (step_count > IVP_LCS_MAX_STEP) {
                    IVP_IF(1) {
                        ivp_debug_printf("\n\ngiveup_stepcount_linear_constraint_solver\n\n\n");
                    }
                    return IVP_GM_FAULT;
                }
                nr_zero_steps = 0;
                if (lcs_full_setup(s) != IVP_OK) {
                    return IVP_GM_FAULT;
                }
            }
            next_numeric_stability_test = IVP_LCS_NUMERIC_TEST_EVERY_N;
        } else {
            next_numeric_stability_test--;
        }

        if (s->ignored_pos >= s->n_variables) {
            return IVP_OK;
        }

        ignored_full_index = aii[s->ignored_pos];

        IVP_IF(s->debug_lcs) {
            int k;
            for (k = 0; k < s->r_actives; k++) {
                int idx = aii[k];
                IVP_IF(fabs(accel[idx]) > (double)0.01f) {
                    ivp_debug_printf("incorrect_accel %f at %d  should be zero\n", accel[idx], idx);
                }
                IVP_IF(fabs(full_x[idx]) < solver_eps) {
                    if (fabs(full_x[ignored_full_index]) < solver_eps) {
                        ivp_debug_printf("active_var %d shouldnt be active but inactive\n", idx);
                    }
                }
                if (full_x[idx] < 0.0) {
                    IVP_IF(1) {
                        ivp_debug_printf("active_var %d should be positive %f\n", idx, full_x[idx]);
                    }
                }
            }
        }
        IVP_IF(1) {
            int k;
            for (k = s->r_actives; k < s->ignored_pos; k++) {
                int idx = aii[k];
                /* (the reference prints accel[idx] in the x messages) */
                IVP_IF(fabs(full_x[idx]) > (double)0.1f) {
                    ivp_debug_printf("incorrect_x %f at %d  should be zero\n", accel[idx], idx);
                }
                if (full_x[idx] < (double)-0.001f) {
                    ivp_debug_printf("incorrect_negative_x %f at %d  should be zero\n", accel[idx], idx);
                }
                if (accel[idx] < 0.0) {
                    ivp_debug_printf("warning_inactive_neg accel %d %f\n", idx, accel[idx]);
                }
            }
        }

        IVP_IF(s->debug_lcs) {
            lcs_debug_out_lcs(s);
        }

        IVP_IF(fabs(full_x[ignored_full_index]) < solver_eps) {
            lcs_debug_test_all_values(s);
        }

        if (s->sub_solver_status > 0) {
            if ((s->sub_solver_status == 1) && (did_real_step)) {
                lcs_do_a_little_random_permutation(s);
                if (lcs_setup_l_u_solver(s) == IVP_OK) {
                    s->sub_solver_status = 0;
                } else {
                    IVP_IF(s->debug_lcs) {
                        ivp_debug_printf("setup_lu_notok\n");
                    }
                }
            } else {
                s->sub_solver_status = 1; /* ok then next time */
            }
        }

        if (fabs(accel[ignored_full_index]) < solver_eps) {
            /* no acceleration: could be inactive, unless x > 0 */
            limiting_var = s->ignored_pos;
            if (fabs(full_x[ignored_full_index]) < solver_eps) {
                goto move_ignored_to_inactive;
            } else {
                goto move_ignored_to_active;
            }
        }

        if (accel[ignored_full_index] < 0.0) {
            /* first r vars are clamped: compute x with Acc x = b */
            if (lcs_get_fdirection(s) == IVP_GM_FAULT) {
                IVP_IF(s->debug_lcs) {
                    ivp_debug_printf("lcs_fdirection_total_fail\n");
                }
                goto perform_full_setup;
            }
            delta_f[ignored_full_index] = 1.0;

            /* delta_f is now filled in original var space */

            IVP_IF(0) {
                ivp_debug_printf("fullmat:\n");
                (void)ivp_gmmz_print_great_matrix(&s->full_solver_mat, "");
                ivp_debug_printf("\n");
            }

            lcs_mult_active_x_for_accel(s);

            for (i = 0; i < s->r_actives; i++) {
                int index_full = aii[i];
                delta_accel[index_full] = 0.0; /* overwrite rounding errors */
            }

            IVP_IF(0) {
                for (int j = 0; j < s->r_actives; j++) {
                    int full_index = aii[j];
                    ivp_debug_printf("should be zero %f\n", delta_accel[full_index]);
                }
            }

            /* step to make clamped active out of ignored_pos */
            IVP_IF(accel[ignored_full_index] > 0.0) {
                ivp_debug_printf("errror_accel_ignored_negativ\n");
            }
            if (-accel[ignored_full_index] < delta_accel[ignored_full_index] * s->MAX_STEP_LEN) {
                smallest_step = -accel[ignored_full_index] / delta_accel[ignored_full_index];
                IVP_ASSERT(smallest_step > 0.0);
                limiting_var = s->ignored_pos;
            } else {
                limiting_var = -1;
                smallest_step = LCS_P_DOUBLE_MAX;
            }

            /* smallest step of clamped */
            for (i = 0; i < s->r_actives; i++) {
                index = aii[i];
                double del_f = delta_f[index];
                if (del_f < -solver_eps) {
                    double local_step = -full_x[index] / del_f;
                    if ((fabs(local_step) < solver_eps) && (full_x[index] < solver_eps)) {
                        limiting_var = i;
                        smallest_step = local_step;
                        goto limiting_var_found;
                    }
                    if (local_step < smallest_step + solver_eps) {
                        /* make it easier to move from active to inactive */
                        smallest_step = local_step;
                        limiting_var = i;
                    }
                }
            }

            /* smallest step of unclamped */
            for (i = s->r_actives; i < s->ignored_pos; i++) {
                index = aii[i];
                double del_a = delta_accel[index];
                if (del_a < -s->SOLVER_EPS) {
                    double local_step = -accel[index] / del_a;
                    if (local_step < smallest_step - solver_eps) {
                        /* make it harder to move from inactive to active
                         * (reference condition is `... || 1`) */
                        IVP_IF(s->debug_lcs) {
                            ivp_debug_printf("violate_inactive: accel %.4f  delta %.4f\n", accel[index], del_a);
                        }
                        smallest_step = local_step;
                        limiting_var = i;
                    }
                }
            }

            if (smallest_step < 0.0) {
                IVP_IF(1) {
                    ivp_debug_printf("error_stepsize_negative %f\n", smallest_step);
                }
                smallest_step = 0.0;
            }

            if (limiting_var < 0) {
                return IVP_GM_FAULT;
            }

            IVP_IF(s->debug_lcs) {
                ivp_debug_printf("smallest_step_is %f\n", smallest_step);
            }

        limiting_var_found:
            if (smallest_step > s->MAX_STEP_LEN) {
                goto perform_full_setup;
            }
            if (smallest_step < s->SOLVER_EPS) {
                nr_zero_steps++;
                if (nr_zero_steps > (s->n_variables >> 1) + 2) { /* heuristic */
                    goto perform_full_setup; /* does a little random permutation */
                }
            } else {
                nr_zero_steps = 0;
            }
            did_real_step = 1;
            lcs_update_step_vars(s, smallest_step);

            if (limiting_var < s->r_actives) {
                /* one active var becomes inactive */
                IVP_IF(s->debug_lcs) {
                    ivp_debug_printf("\nactive %d becomes inactive (relp %d)\n\n", aii[limiting_var], limiting_var);
                }
                indx = aii[limiting_var];
                IVP_ASSERT((fabs(full_x[indx]) < (double)0.001f) || (fabs(smallest_step) < solver_eps));
                full_x[indx] = 0.0;
                s->r_actives--;
                lcs_exchange_lcs_variables(s, s->r_actives, limiting_var);
                lcs_decrement_sub_solver(s, limiting_var);
            } else {
                if (limiting_var < s->ignored_pos) {
                    /* one inactive var becomes active */
                    if (smallest_step > s->SOLVER_EPS) {
                        lcs_move_not_necessary_actives_to_inactives(s);
                    }

                    IVP_IF(s->debug_lcs) {
                        ivp_debug_printf("\ninactive %d becomes active (relp %d)\n\n", aii[limiting_var], limiting_var);
                    }
                    indx = aii[limiting_var];
                    IVP_ASSERT(fabs(accel[indx]) < (double)0.001f);
                    accel[indx] = 0.0;
                    lcs_exchange_lcs_variables(s, s->r_actives, limiting_var);
                    s->r_actives++;
                    lcs_increment_sub_solver(s);
                } else {
                move_ignored_to_active:
                    /* ignored_pos comes to the active variables */
                    lcs_move_not_necessary_actives_to_inactives(s);
                    IVP_IF(s->debug_lcs) {
                        ivp_debug_printf("\nignored %d becomes active (relp %d)\n\n", aii[s->ignored_pos], s->ignored_pos);
                    }
                    indx = aii[limiting_var];
                    IVP_ASSERT(fabs(accel[indx]) < (double)0.001f);
                    accel[indx] = 0.0;
                    if (full_x[indx] < 0.0) { /* really necessary ? */
                        full_x[indx] = 0.0;
                    }
                    lcs_exchange_lcs_variables(s, s->r_actives, limiting_var);
                    s->ignored_pos++;
                    s->r_actives++;
                    if (s->ignored_pos < s->n_variables) {
                        lcs_increment_sub_solver(s);
                    } else {
                        next_numeric_stability_test = 0;
                    }
                    IVP_IF(0) {
                        lcs_debug_test_all_values(s);
                    }
                }
            }
        } else {
        move_ignored_to_inactive:
            /* ignored_pos comes to the inactive variables */
            did_real_step = 0; /* active variables are unchanged */
            IVP_IF(s->debug_lcs) {
                ivp_debug_printf("ignored %d becomes inactive (relp %d)\n\n", aii[s->ignored_pos], s->ignored_pos);
            }
            indx = aii[s->ignored_pos];
            IVP_IF(!(fabs(full_x[indx]) < (double)0.001f)) {
                ivp_debug_printf("error_move_ignored_to_inactive_x_not_zero\n");
            }

            full_x[indx] = 0.0;
            if (accel[indx] < 0.0) { /* really necessary ? */
                accel[indx] = 0.0;
            }
            s->ignored_pos++;
            if (s->ignored_pos >= s->n_variables) {
                next_numeric_stability_test = 0;
            }
        }
    }
    /* reference: IVP_IF(debug_lcs) { debug_out_lcs(); } return IVP_OK; after
     * the endless loop: unreachable, not ported */
}

/* ── public entry ───────────────────────────────────────────────────────── */

ivp_result_t ivp_lcs_init_and_solve_in_memory(double *A_in, double *b_in, double *x_out,
                                              int n, int actives_at_begin,
                                              ivp_u_memory_t *mem)
{
    if (!A_in || !b_in || !x_out || !mem || n < 0 || n > IVP_MAX_GREAT_MATRIX_SIZE ||
        actives_at_begin < 0 || actives_at_begin > n) {
        return IVP_ERR_INVALID_ARG;
    }

    lcs_state_t st;
    memset(&st, 0, sizeof(st));
    lcs_state_t *s = &st;

    s->n_variables = n;
    int as = ivp_gm_aligned_len(n);
    s->aligned_size = as;

    /* alloc_memory(): uninitialised arena memory, reference order and sizes
     * (the IVP_IF(1) zero fill at its end runs in debug builds only) */
    size_t vec_d = sizeof(double) * (size_t)as;
    size_t vec_i = sizeof(int) * (size_t)as;
    size_t mat_d = sizeof(double) * (size_t)as * (size_t)n;
    s->actives_inactives_ignored = (int *)ivp_u_memory_get_mem(mem, vec_i);
    s->variable_is_found_at = (int *)ivp_u_memory_get_mem(mem, vec_i);
    s->accel = (double *)ivp_u_memory_get_mem(mem, vec_d);
    s->delta_accel = (double *)ivp_u_memory_get_mem(mem, vec_d);
    s->delta_f = (double *)ivp_u_memory_get_mem(mem, vec_d);
    s->reset_x = (double *)ivp_u_memory_get_mem(mem, vec_d);
    s->reset_accel = (double *)ivp_u_memory_get_mem(mem, vec_d);
    s->lu_sub_solver.L_matrix = (double *)ivp_u_memory_get_mem(mem, mat_d);
    s->lu_sub_solver.U_matrix = (double *)ivp_u_memory_get_mem(mem, mat_d);
    s->lu_sub_solver.input_vec = (double *)ivp_u_memory_get_mem(mem, vec_d);
    s->lu_sub_solver.out_vec = (double *)ivp_u_memory_get_mem(mem, vec_d);
    s->lu_sub_solver.temp_vec = (double *)ivp_u_memory_get_mem(mem, vec_d);
    s->lu_sub_solver.mult_vec = (double *)ivp_u_memory_get_mem(mem, vec_d);
    s->sub_solver_mat.matrix_values = (double *)ivp_u_memory_get_mem(mem, mat_d);
    s->sub_solver_mat.desired_vector = (double *)ivp_u_memory_get_mem(mem, vec_d);
    s->sub_solver_mat.result_vector = (double *)ivp_u_memory_get_mem(mem, vec_d);
    s->inv_mat.matrix_values = (double *)ivp_u_memory_get_mem(mem, mat_d);
    s->inv_mat.desired_vector = (double *)ivp_u_memory_get_mem(mem, vec_d);
    s->inv_mat.result_vector = (double *)ivp_u_memory_get_mem(mem, vec_d);
    if (!s->actives_inactives_ignored || !s->variable_is_found_at || !s->accel ||
        !s->delta_accel || !s->delta_f || !s->reset_x || !s->reset_accel ||
        !s->lu_sub_solver.L_matrix || !s->lu_sub_solver.U_matrix ||
        !s->lu_sub_solver.input_vec || !s->lu_sub_solver.out_vec ||
        !s->lu_sub_solver.temp_vec || !s->lu_sub_solver.mult_vec ||
        !s->sub_solver_mat.matrix_values || !s->sub_solver_mat.desired_vector ||
        !s->sub_solver_mat.result_vector || !s->inv_mat.matrix_values ||
        !s->inv_mat.desired_vector || !s->inv_mat.result_vector) {
        return IVP_ERR_OUT_OF_MEMORY;
    }

    IVP_IF(1) { /* fill LU for fast align access */
        for (int j = 0; j < n; j++) {
            double *L = &s->lu_sub_solver.L_matrix[j * as];
            double *U = &s->lu_sub_solver.U_matrix[j * as];
            double *S = &s->sub_solver_mat.matrix_values[j * as];
            double *M = &s->inv_mat.matrix_values[j * as];
            for (int i = 0 /*n_variables*/; i < as; i++) {
                L[i] = 0.0;
                U[i] = 0.0;
                S[i] = 0.0;
                M[i] = 0.0;
            }
        }
        for (int k = n; k < as; k++) {
            s->reset_accel[k] = 0.0;
        }
    }

    /* IVP_Great_Matrix_Many_Zero constructors */
    s->sub_solver_mat.matrix_eps = IVP_GMMZ_DEFAULT_EPS;
    s->full_solver_mat.matrix_eps = IVP_GMMZ_DEFAULT_EPS;
    s->debug_mat.matrix_eps = IVP_GMMZ_DEFAULT_EPS;
    s->inv_mat.matrix_eps = IVP_GMMZ_DEFAULT_EPS;

    /* init_and_solve_lc */
    s->r_actives = 0;
    s->ignored_pos = 0;
    s->SOLVER_EPS = IVP_LCS_SOLVER_EPS; /* do not make this one bigger */
    s->GAUSS_EPS = s->SOLVER_EPS;
    s->TEST_EPS = s->SOLVER_EPS * 1000.0;
    s->sub_solver_status = 0;
    s->MAX_STEP_LEN = IVP_LCS_MAX_STEP_LEN;

    s->lu_sub_solver.aligned_row_len = as;
    s->lu_sub_solver.n_sub = 0;
    s->lu_sub_solver.matrix_eps = s->SOLVER_EPS * 10.0;

    s->temp = s->lu_sub_solver.temp_vec;

    s->sub_solver_mat.matrix_eps = s->GAUSS_EPS;
    s->inv_mat.matrix_eps = s->SOLVER_EPS;

    IVP_IF(1) {
        /* debug build: one more matrix from the arena (shifts every later
         * allocation of the transaction) */
        s->debug_mat.matrix_values = (double *)ivp_u_memory_get_mem(mem, mat_d);
        s->debug_mat.desired_vector = (double *)ivp_u_memory_get_mem(mem, vec_d);
        s->debug_mat.result_vector = (double *)ivp_u_memory_get_mem(mem, vec_d);
        s->debug_mat.matrix_eps = s->SOLVER_EPS;
        if (!s->debug_mat.matrix_values || !s->debug_mat.desired_vector || !s->debug_mat.result_vector) {
            return IVP_ERR_OUT_OF_MEMORY;
        }
    }

    s->full_solver_mat.columns = n;
    s->full_solver_mat.aligned_row_len = as;
    s->full_solver_mat.matrix_values = A_in;
    s->full_solver_mat.desired_vector = s->delta_f;
    s->full_solver_mat.result_vector = s->delta_accel;
    s->full_solver_mat.matrix_eps = s->SOLVER_EPS;

    for (int i = 0; i < as; i++) {
        s->actives_inactives_ignored[i] = i;
        s->variable_is_found_at[i] = i;
        x_out[i] = 0.0;
        s->accel[i] = -b_in[i];
    }

    s->full_A = A_in;
    s->full_b = b_in;
    s->full_x = x_out;

    s->first_permute_index = 0;
    s->second_permute_index = 0;
    s->first_permute_ignored = 0;
    s->second_permute_ignored = 0;

    s->debug_no_lu_count = 0;
    s->debug_lcs = 0;

    IVP_IF(s->debug_lcs) {
        ivp_debug_printf("\n***********************************************************\n");
        lcs_start_debug_lcs(s);
    }

    lcs_startup_setup(s, actives_at_begin);
    ivp_result_t ret_val = lcs_solve_lc(s);

    IVP_IF((s->debug_no_lu_count > 0) && (s->debug_lcs)) {
        ivp_debug_printf("sum_lu_failed %d\n", s->debug_no_lu_count);
    }

    IVP_IF(ret_val == IVP_OK) {
        /* the check writes A*x into delta_accel (full_solver_mat.result_vector) */
        int j;
        s->full_solver_mat.desired_vector = s->full_x;
        ivp_gmmz_mult_aligned(&s->full_solver_mat);
        for (j = 0; j < s->n_variables; j++) {
            if (s->full_solver_mat.result_vector[j] + (double)0.0001f < s->full_b[j]) {
                ivp_debug_printf("linear_constraint_failed %d %f should be greater %f\n", j,
                                 s->full_solver_mat.result_vector[j], s->full_b[j]);
            }
        }
        for (j = 0; j < s->r_actives; j++) {
            int index_full = s->actives_inactives_ignored[j];
            double diff = fabs(s->full_solver_mat.result_vector[index_full] - s->full_b[index_full]);
            if (diff > (double)0.0001f) {
                ivp_debug_printf("linear_constraint_failed %d %f should be equal %f\n", j,
                                 s->full_solver_mat.result_vector[index_full], s->full_b[index_full]);
            }
        }
    }

    s->full_solver_mat.result_vector = NULL;
    s->full_solver_mat.desired_vector = NULL;
    s->full_solver_mat.matrix_values = NULL;
    s->sub_solver_mat.result_vector = NULL;
    s->sub_solver_mat.desired_vector = NULL;
    s->sub_solver_mat.matrix_values = NULL;
    s->inv_mat.result_vector = NULL;
    s->inv_mat.desired_vector = NULL;
    s->inv_mat.matrix_values = NULL;
    IVP_IF(1) {
        s->debug_mat.result_vector = NULL;
        s->debug_mat.desired_vector = NULL;
        s->debug_mat.matrix_values = NULL;
    }

    return ret_val;
}

ivp_result_t ivp_lcs_init_and_solve_strided(const double *A, int a_stride,
                                            const double *b, double *x_out,
                                            int n, int actives_at_begin,
                                            const ivp_allocator_t *alloc)
{
    if (n < 0 || n > IVP_MAX_GREAT_MATRIX_SIZE || a_stride < n ||
        actives_at_begin < 0 || actives_at_begin > n) {
        return IVP_ERR_INVALID_ARG;
    }
    if (n == 0) {
        return IVP_OK; /* reference: nothing to do, IVP_OK */
    }
    if (!A || !b || !x_out) {
        return IVP_ERR_INVALID_ARG;
    }

    /* A fresh (zero-filled) arena laid out like the friction solver's: the
     * caller's matrix and vectors first (IVP_Friction_Solver::dist_change_mat,
     * zero padding), then the solver's own arrays. */
    ivp_u_memory_t mem;
    ivp_result_t ret_val = ivp_u_memory_init(&mem, alloc);
    if (ret_val != IVP_OK) {
        return ret_val;
    }
    int as = ivp_gm_aligned_len(n);
    double *full_A = (double *)ivp_u_memory_get_mem(
        &mem, sizeof(double) * ((size_t)as * (size_t)n + (size_t)(IVP_GM_VECFPU_SIZE - 1)));
    double *full_b = (double *)ivp_u_memory_get_mem(&mem, sizeof(double) * (size_t)as);
    double *full_x = (double *)ivp_u_memory_get_mem(&mem, sizeof(double) * (size_t)as);
    if (!full_A || !full_b || !full_x) {
        ivp_u_memory_destroy(&mem);
        return IVP_ERR_OUT_OF_MEMORY;
    }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            full_A[i * as + j] = A[(size_t)i * (size_t)a_stride + (size_t)j];
        }
        full_b[i] = b[i];
    }

    ret_val = ivp_lcs_init_and_solve_in_memory(full_A, full_b, full_x, n, actives_at_begin, &mem);

    for (int i = 0; i < n; i++) {
        x_out[i] = full_x[i];
    }
    ivp_u_memory_destroy(&mem);
    return ret_val;
}

ivp_result_t ivp_lcs_init_and_solve(const double *A, const double *b, double *x_out,
                                    int n, int actives_at_begin,
                                    const ivp_allocator_t *alloc)
{
    return ivp_lcs_init_and_solve_strided(A, n, b, x_out, n, actives_at_begin, alloc);
}
