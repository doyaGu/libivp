/* test_lcs_parity.c — bit parity of the ported dense solvers with the
 * reference IVP engine.
 *
 * tests/data/lcs_cases_strict.h (reference built with -ffp-contract=off,
 * matching IVP_GM_USE_FMA 0, the default) or tests/data/lcs_cases.h
 * (default-contraction reference, IVP_GM_USE_FMA 1) holds the outputs of the
 * reference IVP_Linear_Constraint_Solver::init_and_solve_lc,
 * IVP_Great_Matrix_Many_Zero::solve_great_matrix_many_zero and ::invert for
 * generated problems (contact stacks, singular / low-rank / sparse /
 * ill-conditioned / scaled SPD, non-symmetric; n = 1..40).  The inputs are
 * regenerated here by lcs_parity_gen.h and checked against a stored hash.
 * Every output value and every return code must match bit for bit.
 */
#include "ivp_test.h"
#include "lcs_parity_gen.h"
#include <ivp/solver/ivp_great_matrix.h>
#if IVP_GM_USE_FMA
#include "data/lcs_cases.h"
#else
#include "data/lcs_cases_strict.h"
#endif

#include <ivp/solver/ivp_linear_constraint_solver.h>
#include <stdint.h>
#include <string.h>

static uint64_t to_bits(double d)
{
    uint64_t u;
    memcpy(&u, &d, sizeof(u));
    return u;
}

static double from_bits(uint64_t u)
{
    double d;
    memcpy(&d, &u, sizeof(d));
    return d;
}

static double rel_diff(double a, double b)
{
    if (a == b) return 0.0;
    double d = fabs(a - b);
    double s = fabs(a) > fabs(b) ? fabs(a) : fabs(b);
    return s > 0.0 ? d / s : d;
}

/* compares n values against the reference bits; returns mismatch count */
static int compare_bits(const char *what, int case_idx, const double *got,
                        const uint64_t *want, int count, double *max_rel)
{
    int bad = 0;
    for (int i = 0; i < count; i++) {
        if (to_bits(got[i]) != want[i]) {
            double r = rel_diff(got[i], from_bits(want[i]));
            if (r > *max_rel || r != r) *max_rel = r;
            if (bad < 3) {
                fprintf(stderr, "  %s case %d [%d]: got %a want %a\n", what, case_idx, i,
                        got[i], from_bits(want[i]));
            }
            bad++;
        }
    }
    return bad;
}

IVP_TEST(lcs_parity_inputs_regenerate)
{
    double A[LCSGEN_MAX_N * LCSGEN_MAX_N];
    double b[LCSGEN_MAX_N];
    int bad = 0;
    for (int c = 0; c < LCS_PARITY_N_CASES; c++) {
        const lcs_parity_case_t *pc = &lcs_parity_cases[c];
        lcsgen_make(pc->family, pc->seed, pc->n, A, b);
        if (lcsgen_input_hash(A, b, pc->n) != pc->input_hash) bad++;
    }
    for (int c = 0; c < LCS_PARITY_N_INV_CASES; c++) {
        const lcs_parity_inv_case_t *ic = &lcs_parity_inv_cases[c];
        lcsgen_make(ic->family, ic->seed, ic->n, A, b);
        if (lcsgen_input_hash(A, b, ic->n) != ic->input_hash) bad++;
    }
    if (bad) fprintf(stderr, "  %d generated inputs differ from the reference inputs\n", bad);
    IVP_ASSERT_EQ(bad, 0);
}

IVP_TEST(lcs_parity_init_and_solve)
{
    double A[LCSGEN_MAX_N * LCSGEN_MAX_N];
    double b[LCSGEN_MAX_N];
    double x[LCSGEN_MAX_N];
    int ret_mismatch = 0, val_mismatch = 0, cases_bad = 0, n_ok = 0;
    double max_rel = 0.0;

    for (int c = 0; c < LCS_PARITY_N_CASES; c++) {
        const lcs_parity_case_t *pc = &lcs_parity_cases[c];
        lcsgen_make(pc->family, pc->seed, pc->n, A, b);
        if (lcsgen_input_hash(A, b, pc->n) != pc->input_hash) continue; /* reported above */
        double A_copy[LCSGEN_MAX_N * LCSGEN_MAX_N];
        memcpy(A_copy, A, sizeof(double) * (size_t)(pc->n * pc->n));

        ivp_result_t r = ivp_lcs_init_and_solve(A, b, x, pc->n, pc->actives_at_begin, NULL);
        int ok = r == IVP_OK;
        IVP_ASSERT_TRUE(r == IVP_OK || r == IVP_GM_FAULT);
        IVP_ASSERT_TRUE(memcmp(A_copy, A, sizeof(double) * (size_t)(pc->n * pc->n)) == 0);
        n_ok += ok;
        if (ok != pc->lcs_ok) {
            if (ret_mismatch < 5) {
                fprintf(stderr, "  lcs case %d (family %d n %d): ret %d want %d\n", c,
                        pc->family, pc->n, ok, pc->lcs_ok);
            }
            ret_mismatch++;
        }
        int bad = compare_bits("lcs", c, x, &lcs_parity_bits[pc->bits_offset], pc->n, &max_rel);
        val_mismatch += bad;
        if (bad || ok != pc->lcs_ok) cases_bad++;
    }
    printf("\n  lcs: %d cases (%d IVP_OK), %d differ (ret %d, values %d, max rel %g)\n  ",
           LCS_PARITY_N_CASES, n_ok, cases_bad, ret_mismatch, val_mismatch, max_rel);
    IVP_ASSERT_EQ(ret_mismatch, 0);
    IVP_ASSERT_EQ(val_mismatch, 0);
}

IVP_TEST(lcs_parity_gauss_many_zero)
{
    double A[LCSGEN_MAX_N * LCSGEN_MAX_N];
    double b[LCSGEN_MAX_N];
    int ret_mismatch = 0, val_mismatch = 0;
    double max_rel = 0.0;

    for (int c = 0; c < LCS_PARITY_N_CASES; c++) {
        const lcs_parity_case_t *pc = &lcs_parity_cases[c];
        int n = pc->n;
        lcsgen_make(pc->family, pc->seed, n, A, b);
        if (lcsgen_input_hash(A, b, n) != pc->input_hash) continue;

        ivp_gmmz_t m;
        IVP_ASSERT_EQ(ivp_gmmz_init(&m, n, NULL), IVP_OK);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) m.matrix_values[i * m.aligned_row_len + j] = A[i * n + j];
            m.desired_vector[i] = b[i];
        }
        int ok = ivp_gmmz_solve(&m) == IVP_OK;
        if (ok != pc->gauss_ok) {
            if (ret_mismatch < 5) {
                fprintf(stderr, "  gauss case %d (family %d n %d): ret %d want %d\n", c,
                        pc->family, n, ok, pc->gauss_ok);
            }
            ret_mismatch++;
        }
        val_mismatch += compare_bits("gauss", c, m.result_vector,
                                     &lcs_parity_bits[pc->bits_offset + n], n, &max_rel);
        ivp_gmmz_destroy(&m, NULL);
    }
    printf("\n  gauss: %d cases, ret mismatches %d, value mismatches %d (max rel %g)\n  ",
           LCS_PARITY_N_CASES, ret_mismatch, val_mismatch, max_rel);
    IVP_ASSERT_EQ(ret_mismatch, 0);
    IVP_ASSERT_EQ(val_mismatch, 0);
}

static void misc_fill(ivp_gmmz_t *m, const double *A, const double *b, int stride)
{
    int n = m->columns;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) m->matrix_values[i * stride + j] = A[i * n + j];
        m->desired_vector[i] = b[i];
    }
}

/* see lcsgen_misc_hash_add in lcs_parity_gen.h */
static int port_misc_hash(const double *A, const double *b, int n, const double *x_gauss,
                          uint64_t *out)
{
    uint64_t h = 0xcbf29ce484222325ULL;
    ivp_gmmz_t t;
    if (ivp_gmmz_init(&t, n, NULL) != IVP_OK) return 0;
    misc_fill(&t, A, b, t.aligned_row_len);
    ivp_gmmz_transform_to_lower_null_triangle(&t);
    int ok = ivp_gmmz_solve_lower_null_matrix(&t) == IVP_OK;
    h = lcsgen_misc_hash_add(h, (double)ok);
    for (int i = 0; i < n; i++) h = lcsgen_misc_hash_add(h, t.result_vector[i]);
    ivp_gmmz_destroy(&t, NULL);

    if (ivp_gmmz_init(&t, n, NULL) != IVP_OK) return 0;
    misc_fill(&t, A, b, t.aligned_row_len);
    ivp_gmmz_mult_aligned(&t);
    for (int i = 0; i < n; i++) h = lcsgen_misc_hash_add(h, t.result_vector[i]);
    ivp_gmmz_destroy(&t, NULL);

    if (ivp_gmmz_init(&t, n, NULL) != IVP_OK) return 0;
    misc_fill(&t, A, b, n);
    ivp_gmmz_mult(&t);
    for (int i = 0; i < n; i++) h = lcsgen_misc_hash_add(h, t.result_vector[i]);
    ivp_gmmz_destroy(&t, NULL);

    if (ivp_gmmz_init(&t, n, NULL) != IVP_OK) return 0;
    misc_fill(&t, A, b, t.aligned_row_len);
    for (int i = 0; i < n; i++) t.result_vector[i] = x_gauss[i];
    for (int i = 0; i < n; i++) {
        ok = ivp_gmmz_check_unequation_line(&t, i) == IVP_OK;
        h = lcsgen_misc_hash_add(h, (double)ok);
    }
    ivp_gmmz_destroy(&t, NULL);
    *out = h;
    return 1;
}

IVP_TEST(lcs_parity_gauss_split_mult_unequation)
{
    double A[LCSGEN_MAX_N * LCSGEN_MAX_N];
    double b[LCSGEN_MAX_N];
    double xg[LCSGEN_MAX_N];
    int bad = 0;
    for (int c = 0; c < LCS_PARITY_N_CASES; c++) {
        const lcs_parity_case_t *pc = &lcs_parity_cases[c];
        lcsgen_make(pc->family, pc->seed, pc->n, A, b);
        if (lcsgen_input_hash(A, b, pc->n) != pc->input_hash) continue;
        for (int i = 0; i < pc->n; i++) xg[i] = from_bits(lcs_parity_bits[pc->bits_offset + pc->n + i]);
        uint64_t h = 0;
        IVP_ASSERT_TRUE(port_misc_hash(A, b, pc->n, xg, &h));
        if (h != pc->misc_hash) {
            if (bad < 5) fprintf(stderr, "  misc case %d (family %d n %d) differs\n", c, pc->family, pc->n);
            bad++;
        }
    }
    printf("\n  split gauss / mult / mult_aligned / unequation: %d cases, %d differ\n  ",
           LCS_PARITY_N_CASES, bad);
    IVP_ASSERT_EQ(bad, 0);
}

IVP_TEST(lcs_parity_lu_invert)
{
    double A[LCSGEN_MAX_N * LCSGEN_MAX_N];
    double b[LCSGEN_MAX_N];
    double inv[LCSGEN_MAX_N * LCSGEN_MAX_N];
    int ret_mismatch = 0, val_mismatch = 0;
    double max_rel = 0.0;

    for (int c = 0; c < LCS_PARITY_N_INV_CASES; c++) {
        const lcs_parity_inv_case_t *ic = &lcs_parity_inv_cases[c];
        int n = ic->n;
        lcsgen_make(ic->family, ic->seed, n, A, b);
        if (lcsgen_input_hash(A, b, n) != ic->input_hash) continue;

        ivp_gmmz_t m, out;
        IVP_ASSERT_EQ(ivp_gmmz_init(&m, n, NULL), IVP_OK);
        IVP_ASSERT_EQ(ivp_gmmz_init(&out, n, NULL), IVP_OK);
        for (int i = 0; i < n * n; i++) m.matrix_values[i] = A[i]; /* stride = columns */
        int ok = ivp_gmmz_invert(&m, &out) == IVP_OK;
        if (ok != ic->inv_ok) ret_mismatch++;
        for (int i = 0; i < n * n; i++) inv[i] = out.matrix_values[i];
        val_mismatch += compare_bits("invert", c, inv, &lcs_parity_inv_bits[ic->bits_offset],
                                     n * n, &max_rel);
        ivp_gmmz_destroy(&m, NULL);
        ivp_gmmz_destroy(&out, NULL);
    }
    printf("\n  invert: %d cases, ret mismatches %d, value mismatches %d (max rel %g)\n  ",
           LCS_PARITY_N_INV_CASES, ret_mismatch, val_mismatch, max_rel);
    IVP_ASSERT_EQ(ret_mismatch, 0);
    IVP_ASSERT_EQ(val_mismatch, 0);
}

int main(void)
{
    IVP_REGISTER(lcs_parity_inputs_regenerate);
    IVP_REGISTER(lcs_parity_init_and_solve);
    IVP_REGISTER(lcs_parity_gauss_many_zero);
    IVP_REGISTER(lcs_parity_gauss_split_mult_unequation);
    IVP_REGISTER(lcs_parity_lu_invert);
    IVP_RUN_ALL();
}
