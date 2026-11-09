/* test_great_matrix_port.c — unit tests for the reference-faithful dense
 * solvers: ivp_gmmz_* (IVP_Great_Matrix_Many_Zero), ivp_incr_lu_*
 * (IVP_Incr_L_U_Matrix), ivp_vecfpu_* and ivp_lcs_* (linear constraint
 * solver).  Bit parity with the reference is checked in test_lcs_parity.c.
 */
#include "ivp_test.h"

#include <ivp/solver/ivp_great_matrix.h>
#include <ivp/solver/ivp_linear_constraint_solver.h>
#include <fenv.h>
#include <string.h>

static void gmmz_fill(ivp_gmmz_t *m, const double *A, const double *b)
{
    int n = m->columns;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) ivp_gmmz_set_value(m, A[i * n + j], j, i);
        m->desired_vector[i] = b[i];
    }
}

/* ── Many_Zero Gauss ────────────────────────────────────────────────────── */

IVP_TEST(gmmz_init_layout)
{
    ivp_gmmz_t m;
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, 5, NULL), IVP_OK);
    IVP_ASSERT_EQ(m.columns, 5);
    IVP_ASSERT_EQ(m.aligned_row_len, IVP_GM_VECFPU_SIZE == 4 ? 8 : 5);
    IVP_ASSERT_TRUE(m.matrix_eps == 1.0e-9);
    for (int i = 0; i < 5 * m.aligned_row_len; i++) IVP_ASSERT_TRUE(m.matrix_values[i] == 0.0);
    ivp_gmmz_destroy(&m, NULL);
    IVP_ASSERT_NULL(m.matrix_values);
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, -1, NULL), IVP_ERR_INVALID_ARG);
}

IVP_TEST(gmmz_solve_1x1_2x2)
{
    ivp_gmmz_t m;
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, 1, NULL), IVP_OK);
    m.matrix_values[0] = 4.0;
    m.desired_vector[0] = 2.0;
    IVP_ASSERT_EQ(ivp_gmmz_solve(&m), IVP_OK);
    IVP_ASSERT_TRUE(m.result_vector[0] == 0.5);
    ivp_gmmz_destroy(&m, NULL);

    /* 2x + y = 5, x + 3y = 7  ->  x = 1.6, y = 1.8 */
    const double A[4] = {2.0, 1.0, 1.0, 3.0};
    const double b[2] = {5.0, 7.0};
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, 2, NULL), IVP_OK);
    gmmz_fill(&m, A, b);
    IVP_ASSERT_EQ(ivp_gmmz_solve(&m), IVP_OK);
    IVP_ASSERT_NEAR(m.result_vector[0], 1.6, 1e-15);
    IVP_ASSERT_NEAR(m.result_vector[1], 1.8, 1e-15);
    ivp_gmmz_destroy(&m, NULL);
}

IVP_TEST(gmmz_solve_3x3_needs_pivot)
{
    /* zero on the first diagonal: pivot search must swap rows */
    const double A[9] = {0.0, 1.0, 1.0,
                         3.0, 2.0, 1.0,
                         1.0, 1.0, 4.0};
    const double x_true[3] = {1.0, 2.0, 3.0};
    double b[3];
    for (int i = 0; i < 3; i++) {
        b[i] = A[i * 3] * x_true[0] + A[i * 3 + 1] * x_true[1] + A[i * 3 + 2] * x_true[2];
    }
    ivp_gmmz_t m;
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, 3, NULL), IVP_OK);
    gmmz_fill(&m, A, b);
    IVP_ASSERT_EQ(ivp_gmmz_solve(&m), IVP_OK);
    for (int i = 0; i < 3; i++) IVP_ASSERT_NEAR(m.result_vector[i], x_true[i], 1e-13);
    ivp_gmmz_destroy(&m, NULL);
}

IVP_TEST(gmmz_solve_singular_consistent_picks_zero)
{
    /* rank 1, consistent: the free variable is set to 0 */
    const double A[4] = {1.0, 1.0, 1.0, 1.0};
    const double b[2] = {2.0, 2.0};
    ivp_gmmz_t m;
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, 2, NULL), IVP_OK);
    gmmz_fill(&m, A, b);
    IVP_ASSERT_EQ(ivp_gmmz_solve(&m), IVP_OK);
    IVP_ASSERT_TRUE(m.result_vector[0] == 2.0);
    IVP_ASSERT_TRUE(m.result_vector[1] == 0.0);
    ivp_gmmz_destroy(&m, NULL);
}

IVP_TEST(gmmz_solve_inconsistent_faults_and_clears)
{
    const double A[4] = {1.0, 1.0, 1.0, 1.0};
    const double b[2] = {2.0, 3.0};
    ivp_gmmz_t m;
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, 2, NULL), IVP_OK);
    gmmz_fill(&m, A, b);
    m.result_vector[0] = 7.0;
    m.result_vector[1] = 7.0;
    IVP_ASSERT_EQ(ivp_gmmz_solve(&m), IVP_GM_FAULT);
    IVP_ASSERT_TRUE(m.result_vector[0] == 0.0);
    IVP_ASSERT_TRUE(m.result_vector[1] == 0.0);
    ivp_gmmz_destroy(&m, NULL);
}

/* The 64-bit reference row helpers overrun into the next row: for this
 * 4x4 system the reference returns IVP_OK with a solution whose last row
 * residual is ~0.24.  Values printed by the reference library (identical
 * with and without FP contraction). */
IVP_TEST(gmmz_solve_4x4_reproduces_reference_overrun)
{
    const double A[16] = {4, 1, 2, 1,
                          1, 5, 1, 2,
                          2, 1, 6, 1,
                          1, 2, 1, 7};
    const double b[4] = {1, 2, 3, 4};
    ivp_gmmz_t m;
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, 4, NULL), IVP_OK);
    gmmz_fill(&m, A, b);
    IVP_ASSERT_EQ(ivp_gmmz_solve(&m), IVP_OK);
#if IVP_GM_VECFPU_SIZE == 4
    IVP_ASSERT_TRUE(m.result_vector[0] == -0.13098812544795924);
    IVP_ASSERT_TRUE(m.result_vector[1] == 0.1288669778974488);
    IVP_ASSERT_TRUE(m.result_vector[2] == 0.43450593727602038);
    IVP_ASSERT_TRUE(m.result_vector[3] == 0.52607364934234735);
#else
    for (int i = 0; i < 4; i++) {
        double r = -b[i];
        for (int j = 0; j < 4; j++) r += A[i * 4 + j] * m.result_vector[j];
        IVP_ASSERT_NEAR(r, 0.0, 1e-12);
    }
#endif
    ivp_gmmz_destroy(&m, NULL);
}

IVP_TEST(gmmz_mult_and_mult_aligned)
{
    const double A[9] = {1, 2, 3, 4, 5, 6, 7, 8, 10};
    ivp_gmmz_t m;
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, 3, NULL), IVP_OK);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) ivp_gmmz_set_value(&m, A[i * 3 + j], j, i);
        m.desired_vector[i] = (double)(i + 1);
    }
    ivp_gmmz_mult_aligned(&m);
    IVP_ASSERT_TRUE(m.result_vector[0] == 14.0);
    IVP_ASSERT_TRUE(m.result_vector[1] == 32.0);
    IVP_ASSERT_TRUE(m.result_vector[2] == 53.0);
    /* mult() uses stride `columns` */
    for (int i = 0; i < 9; i++) m.matrix_values[i] = A[i];
    m.result_vector[0] = m.result_vector[1] = m.result_vector[2] = 0.0;
    ivp_gmmz_mult(&m);
    IVP_ASSERT_TRUE(m.result_vector[0] == 14.0);
    IVP_ASSERT_TRUE(m.result_vector[2] == 53.0);
    ivp_gmmz_destroy(&m, NULL);
}

IVP_TEST(gmmz_check_unequation_line)
{
    const double A[4] = {2.0, 0.0, 0.0, 1.0};
    const double b[2] = {1.0, 1.0};
    ivp_gmmz_t m;
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, 2, NULL), IVP_OK);
    gmmz_fill(&m, A, b);
    m.result_vector[0] = 0.5;
    m.result_vector[1] = 0.9;
    IVP_ASSERT_EQ(ivp_gmmz_check_unequation_line(&m, 0), IVP_OK);
    IVP_ASSERT_EQ(ivp_gmmz_check_unequation_line(&m, 1), IVP_GM_FAULT);
    ivp_gmmz_destroy(&m, NULL);
}

IVP_TEST(gmmz_fill_from_bigger_matrix)
{
    ivp_gmmz_t big, sub;
    IVP_ASSERT_EQ(ivp_gmmz_init(&big, 3, NULL), IVP_OK);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) ivp_gmmz_set_value(&big, (double)(10 * i + j), j, i);
        big.desired_vector[i] = (double)i;
    }
    IVP_ASSERT_EQ(ivp_gmmz_init(&sub, 2, NULL), IVP_OK);
    const int pos[2] = {2, 0};
    ivp_gmmz_fill_from_bigger_matrix(&sub, &big, pos, 2);
    IVP_ASSERT_TRUE(ivp_gmmz_get_value(&sub, 0, 0) == 22.0);
    IVP_ASSERT_TRUE(ivp_gmmz_get_value(&sub, 1, 0) == 20.0);
    IVP_ASSERT_TRUE(ivp_gmmz_get_value(&sub, 0, 1) == 2.0);
    IVP_ASSERT_TRUE(ivp_gmmz_get_value(&sub, 1, 1) == 0.0);
    IVP_ASSERT_TRUE(sub.desired_vector[0] == 2.0);
    ivp_gmmz_destroy(&sub, NULL);
    ivp_gmmz_destroy(&big, NULL);
}

/* ── LU (Crout) and inverse ─────────────────────────────────────────────── */

IVP_TEST(gmmz_invert_3x3)
{
    const double A[9] = {4, 7, 2,
                         3, 6, 1,
                         2, 5, 3}; /* det = 9 */
    ivp_gmmz_t m, inv;
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, 3, NULL), IVP_OK);
    IVP_ASSERT_EQ(ivp_gmmz_init(&inv, 3, NULL), IVP_OK);
    for (int i = 0; i < 9; i++) m.matrix_values[i] = A[i]; /* stride = columns */
    IVP_ASSERT_EQ(ivp_gmmz_invert(&m, &inv), IVP_OK);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double s = 0.0;
            for (int k = 0; k < 3; k++) s += A[i * 3 + k] * inv.matrix_values[k * 3 + j];
            IVP_ASSERT_NEAR(s, i == j ? 1.0 : 0.0, 1e-13);
        }
    }
    /* known inverse entry: (A^-1)[0][0] = (6*3 - 1*5) / 9 = 13/9 */
    IVP_ASSERT_NEAR(inv.matrix_values[0], 13.0 / 9.0, 1e-14);
    ivp_gmmz_destroy(&m, NULL);
    ivp_gmmz_destroy(&inv, NULL);
}

IVP_TEST(gmmz_invert_singular_faults)
{
    const double A[4] = {1, 2, 2, 4};
    ivp_gmmz_t m, inv;
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, 2, NULL), IVP_OK);
    IVP_ASSERT_EQ(ivp_gmmz_init(&inv, 2, NULL), IVP_OK);
    for (int i = 0; i < 4; i++) m.matrix_values[i] = A[i];
    IVP_ASSERT_EQ(ivp_gmmz_invert(&m, &inv), IVP_GM_FAULT);
    ivp_gmmz_destroy(&m, NULL);
    ivp_gmmz_destroy(&inv, NULL);
}

IVP_TEST(gmmz_lu_crout_solve_5x5)
{
    const int n = 5;
    double A[25];
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) A[i * n + j] = (i == j) ? 10.0 + i : 1.0 / (1.0 + i + 2.0 * j);
    }
    const double b[5] = {1, -2, 3, -4, 5};
    ivp_gmmz_t m;
    IVP_ASSERT_EQ(ivp_gmmz_init(&m, n, NULL), IVP_OK);
    for (int i = 0; i < n * n; i++) m.matrix_values[i] = A[i];
    int index_vec[5];
    double sign = 0.0;
    IVP_ASSERT_EQ(ivp_gmmz_lu_crout(&m, index_vec, &sign), IVP_OK);
    IVP_ASSERT_TRUE(sign == 1.0 || sign == -1.0);
    for (int i = 0; i < n; i++) m.desired_vector[i] = b[i];
    IVP_ASSERT_EQ(ivp_gmmz_lu_solve(&m, index_vec), IVP_OK);
    for (int i = 0; i < n; i++) {
        double r = -b[i];
        for (int j = 0; j < n; j++) r += A[i * n + j] * m.result_vector[j];
        IVP_ASSERT_NEAR(r, 0.0, 1e-13);
    }
    ivp_gmmz_destroy(&m, NULL);
}

/* ── Vector-FPU helpers ─────────────────────────────────────────────────── */

IVP_TEST(vecfpu_unaligned_window)
{
    double t[16], s[16];
    for (int i = 0; i < 16; i++) { t[i] = 0.0; s[i] = (double)(i + 1); }
    /* logical range: elements 1..3 */
    ivp_vecfpu_add_multiple_row(t, 1, s, 1, 1.0, 3, false);
#if IVP_GM_VECFPU_SIZE == 4
    /* start moves to 0, count 3 + 2 -> two blocks of 4: elements 0..7 */
    for (int i = 0; i < 8; i++) IVP_ASSERT_TRUE(t[i] == s[i]);
    for (int i = 8; i < 16; i++) IVP_ASSERT_TRUE(t[i] == 0.0);
#else
    IVP_ASSERT_TRUE(t[0] == 0.0);
    for (int i = 1; i < 4; i++) IVP_ASSERT_TRUE(t[i] == s[i]);
    IVP_ASSERT_TRUE(t[4] == 0.0);
#endif
    IVP_ASSERT_TRUE(ivp_vecfpu_large_dot_product(s, 0, s, 0, 3, true) == 14.0);
    ivp_vecfpu_set_row_to_zero(s, 0, 2, true);
    IVP_ASSERT_TRUE(s[0] == 0.0 && s[1] == 0.0);
}

/* ── incremental LU ─────────────────────────────────────────────────────── */

/* Literal scalar reductions compare every finite result bit for bit.  NaN
 * payload propagation and floating-point status flags are compiler/ISA state,
 * not solver output; exceptional inputs therefore compare by NaN class.
 * In particular, four independent partial sums are not equivalent.  Keep the
 * FMA-reference flavour's unfused prefix and contracted tail. */
static double scalar_madd(double a, double b, double c);

/* IVP_VecFPU::fpu_large_dot_product: the retail build accumulates the
 * software dot from the last element */
static double scalar_dot(const double *a, const double *b, int count)
{
    double sum = 0.0;
    for (int k = count - 1; k >= 0; k--) sum = scalar_madd(a[k], b[k], sum);
    return sum;
}

static double scalar_madd(double a, double b, double c)
{
#if IVP_GM_USE_FMA
    return fma(a, b, c);
#else
    return a * b + c;
#endif
}

static double reduction_value(unsigned int index)
{
    static const uint64_t bits[] = {
        UINT64_C(0), UINT64_C(0x8000000000000000),
        UINT64_C(0x3ff0000000000001), UINT64_C(0xbfefffffffffffff),
        UINT64_C(0x0010000000000000), UINT64_C(1),
        UINT64_C(0x7fefffffffffffff), UINT64_C(0xffefffffffffffff),
        UINT64_C(0x3cb0000000000000), UINT64_C(0xbcb0000000000000),
        UINT64_C(0x7ff0000000000000), UINT64_C(0xfff0000000000000),
        UINT64_C(0x7ff8000000000042), UINT64_C(0x7ff0000000000001)
    };
    double value;
    uint64_t word = bits[index % (sizeof(bits) / sizeof(bits[0]))];
    memcpy(&value, &word, sizeof(value));
    return value;
}

static bool same_double_result(double actual, double expected)
{
    uint64_t actual_bits, expected_bits;
    memcpy(&actual_bits, &actual, sizeof(actual_bits));
    memcpy(&expected_bits, &expected, sizeof(expected_bits));
    if (actual_bits == expected_bits) return true;
    return (actual_bits & UINT64_C(0x7fffffffffffffff)) >
               UINT64_C(0x7ff0000000000000) &&
           (expected_bits & UINT64_C(0x7fffffffffffffff)) >
               UINT64_C(0x7ff0000000000000);
}

static bool same_double_array(const double *actual, const double *expected,
                              size_t count)
{
    for (size_t i = 0; i < count; i++) {
        if (!same_double_result(actual[i], expected[i])) return false;
    }
    return true;
}

static const int k_rounding_modes[] = {
    FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO
};

typedef struct fp_rounding_fixture {
    fenv_t saved;
} fp_rounding_fixture_t;

static bool fp_rounding_fixture_begin(fp_rounding_fixture_t *fixture)
{
    return fegetenv(&fixture->saved) == 0;
}

static bool fp_rounding_fixture_select(size_t mode_index)
{
    return fesetround(k_rounding_modes[mode_index]) == 0;
}

static bool fp_rounding_fixture_end(const fp_rounding_fixture_t *fixture)
{
    return fesetenv(&fixture->saved) == 0;
}

IVP_TEST(vecfpu_add_multiple_row_matches_scalar_rounding)
{
    enum { capacity = 128 };
    fp_rounding_fixture_t rounding;
    IVP_ASSERT_TRUE(fp_rounding_fixture_begin(&rounding));
    bool equal = true;
    for (size_t mode_index = 0;
         mode_index < sizeof(k_rounding_modes) / sizeof(k_rounding_modes[0]);
         mode_index++) {
        equal &= fp_rounding_fixture_select(mode_index);
        for (int test_case = 0; equal && test_case < 32; test_case++) {
            double source[capacity], expected[capacity], actual[capacity];
            for (int i = 0; i < capacity; i++) {
                source[i] = reduction_value((unsigned int)(i + test_case));
                expected[i] = actual[i] =
                    reduction_value((unsigned int)(3 * i + test_case + 5));
            }
            bool aligned = (test_case & 1) != 0;
            int source_off = aligned ? 16 : 16 + ((test_case >> 1) & 1);
            int target_off = aligned ? 48 : 48 + ((test_case >> 2) & 1);
            int size = 1 + (test_case % 23);
            double factor = reduction_value((unsigned int)(test_case + 7));

            int scalar_source_off = source_off;
            int scalar_target_off = target_off;
            int scalar_size = size;
#if IVP_GM_VECFPU_SIZE == 4
            if (!aligned) {
                int shift = scalar_source_off & 1;
                scalar_target_off -= scalar_target_off & 1;
                scalar_size += shift * 2;
                scalar_source_off -= shift;
            }
#endif
            int processed = ((scalar_size + IVP_GM_VECFPU_SIZE - 1) /
                             IVP_GM_VECFPU_SIZE) * IVP_GM_VECFPU_SIZE;
            for (int i = 0; i < processed; i++) {
                double value = source[scalar_source_off + i] * factor;
                value += expected[scalar_target_off + i];
                expected[scalar_target_off + i] = value;
            }
            ivp_vecfpu_add_multiple_row(actual, target_off, source, source_off,
                                        factor, size, aligned);
            equal &= same_double_array(actual, expected, capacity);
        }
    }
    IVP_ASSERT_TRUE(fp_rounding_fixture_end(&rounding));
    IVP_ASSERT_TRUE(equal);
}

IVP_TEST(vecfpu_reduction_bitwise_scalar)
{
    fp_rounding_fixture_t rounding;
    IVP_ASSERT_TRUE(fp_rounding_fixture_begin(&rounding));
    bool equal = true;
    for (size_t mode_index = 0;
         mode_index < sizeof(k_rounding_modes) / sizeof(k_rounding_modes[0]);
         mode_index++) {
        equal &= fp_rounding_fixture_select(mode_index);
        for (int sequence_shift = 0; sequence_shift < 14; sequence_shift++) {
            double a[80], b[80];
            for (int k = 0; k < 80; k++) {
                a[k] = reduction_value((unsigned int)(k + sequence_shift));
                b[k] = reduction_value((unsigned int)(3 * k + sequence_shift + 1));
            }
            for (int count = 0; count <= 65; count++) {
                for (int aligned = 0; aligned < 2; aligned++) {
                    int off_a = 1, off_b = 3, size = count;
#if IVP_GM_VECFPU_SIZE == 4
                    if (!aligned) { off_a = 0; off_b = 2; size += 2; }
#endif
                    double expected = scalar_dot(a + off_a, b + off_b, size);
                    double actual = ivp_vecfpu_large_dot_product(a, 1, b, 3, count, aligned != 0);
                    equal &= same_double_result(actual, expected);
                }
            }
        }
    }
    IVP_ASSERT_TRUE(fp_rounding_fixture_end(&rounding));
    IVP_ASSERT_TRUE(equal);
}

static void gmmz_transform_serial_reference(ivp_gmmz_t *m)
{
    int arl = m->aligned_row_len;
    for (int column = 0; column < m->columns; column++) {
        ivp_gmmz_find_pivot_in_column(m, column);
        double diagonal = m->matrix_values[column * arl + column];
        if (fabs(diagonal) < m->matrix_eps) continue;
        double negative_inverse = -1.0 / diagonal;
        for (int row = column + 1; row < m->columns; row++) {
            double value = m->matrix_values[row * arl + column];
            if (fabs(value) > m->matrix_eps) {
                ivp_gmmz_add_multiple_line(
                    m, column, row, value * negative_inverse);
            }
        }
    }
}

static double gmmz_dense_value(int row, int column, int seed)
{
    unsigned int bits = (unsigned int)(row + 1) * 2654435761u;
    bits ^= (unsigned int)(column + seed + 1) * 2246822519u;
    int centered = (int)(bits % 2047u) - 1023;
    double value = (double)centered / 2048.0;
    if (row == column) value += 4.0;
    if (column % 11 == 0 && row == column + 1) value += 8.0;
    return value;
}

IVP_TEST(gmmz_large_transform_matches_serial_reference)
{
    static const int sizes[] = {63, 64, 65, 96, 127, 128, 149, 150};
    fp_rounding_fixture_t rounding;
    IVP_ASSERT_TRUE(fp_rounding_fixture_begin(&rounding));
    bool equal = true;
    for (size_t mode = 0;
         mode < sizeof(k_rounding_modes) / sizeof(k_rounding_modes[0]);
         mode++) {
        equal &= fp_rounding_fixture_select(mode);
        for (size_t case_index = 0;
             equal && case_index < sizeof(sizes) / sizeof(sizes[0]);
             case_index++) {
            ivp_gmmz_t expected, actual;
            int n = sizes[case_index];
            memset(&expected, 0, sizeof(expected));
            memset(&actual, 0, sizeof(actual));
            bool initialized =
                ivp_gmmz_init(&expected, n, NULL) == IVP_OK &&
                ivp_gmmz_init(&actual, n, NULL) == IVP_OK;
            if (!initialized) {
                if (expected.storage) ivp_gmmz_destroy(&expected, NULL);
                if (actual.storage) ivp_gmmz_destroy(&actual, NULL);
                equal = false;
                break;
            }
            for (int row = 0; row < n; row++) {
                for (int column = 0; column < expected.aligned_row_len;
                     column++) {
                    double value = gmmz_dense_value(
                        row, column, (int)case_index + 3);
                    if ((case_index & 1u) != 0u && column < n &&
                        (column < row - 2 || column > row + 2)) {
                        value = 0.0;
                    }
                    expected.matrix_values[
                        row * expected.aligned_row_len + column] = value;
                    actual.matrix_values[
                        row * actual.aligned_row_len + column] = value;
                }
            }
            for (int row = 0; row < expected.aligned_row_len; row++) {
                double value = gmmz_dense_value(
                    row, n + 1, (int)case_index + 17);
                expected.desired_vector[row] = value;
                actual.desired_vector[row] = value;
            }

            gmmz_transform_serial_reference(&expected);
            ivp_gmmz_transform_to_lower_null_triangle(&actual);
            equal &= expected.storage_size == actual.storage_size;
            equal &= memcmp(expected.storage, actual.storage,
                            expected.storage_size) == 0;
            ivp_gmmz_destroy(&actual, NULL);
            ivp_gmmz_destroy(&expected, NULL);
        }
    }
    IVP_ASSERT_TRUE(fp_rounding_fixture_end(&rounding));
    IVP_ASSERT_TRUE(equal);
}

IVP_TEST(incr_lu_solve_bitwise_scalar)
{
    enum { stride = 40, max_n = 33 };
    double lower[stride * stride], upper[stride * stride], input[stride];
    double expected_mult[stride], expected_temp[stride], expected_out[stride];
    double actual_mult[stride], actual_temp[stride], actual_out[stride];
    fp_rounding_fixture_t rounding;
    IVP_ASSERT_TRUE(fp_rounding_fixture_begin(&rounding));
    bool equal = true;
    for (size_t mode_index = 0;
         mode_index < sizeof(k_rounding_modes) / sizeof(k_rounding_modes[0]);
         mode_index++) {
        equal &= fp_rounding_fixture_select(mode_index);
        for (int use_exceptional_values = 0; use_exceptional_values < 2;
             use_exceptional_values++) {
            for (int k = 0; k < stride * stride; k++) {
                lower[k] = use_exceptional_values
                    ? reduction_value((unsigned int)k)
                    : (double)(k % 17 - 8) / 19.0;
                upper[k] = use_exceptional_values
                    ? reduction_value((unsigned int)(k + 3))
                    : (double)(k % 13 - 6) / 23.0;
            }
            for (int k = 0; k < stride; k++) input[k] = (double)(k % 7 - 3) / 11.0;
            for (int n = 0; n <= max_n; n++) {
                for (int k = 0; k < stride; k++) {
                    expected_mult[k] = actual_mult[k] = -13.0;
                    expected_temp[k] = actual_temp[k] = -17.0;
                    expected_out[k] = actual_out[k] = -19.0;
                }
                for (int i = n - 1; i >= 0; i--) expected_mult[i] = input[i];
                for (int i = n - 1; i >= 0; i--)
                    expected_temp[i] = scalar_dot(lower + i * stride, expected_mult, n);
                for (int i = n - 1; i >= 0; i--) {
                    double sum = 0.0;
                    for (int j = n - 1; j > i; j--)
                        sum = scalar_madd(upper[i * stride + j], expected_temp[j], sum);
                    double result = expected_temp[i] - sum;
                    expected_temp[i] = result;
                    expected_out[i] = result;
                }
                ivp_incr_lu_t lu = {0};
                lu.L_matrix = lower; lu.U_matrix = upper; lu.input_vec = input;
                lu.mult_vec = actual_mult; lu.temp_vec = actual_temp; lu.out_vec = actual_out;
                lu.n_sub = n; lu.aligned_row_len = stride;
                ivp_incr_lu_solve_lin_equ(&lu);
                equal &= same_double_array(actual_mult, expected_mult,
                                           stride);
                equal &= same_double_array(actual_temp, expected_temp,
                                           stride);
                equal &= same_double_array(actual_out, expected_out, stride);
            }
        }
    }
    IVP_ASSERT_TRUE(fp_rounding_fixture_end(&rounding));
    IVP_ASSERT_TRUE(equal);
}

/* Matrix whose elimination pivots are all exactly 1 and needs no row
 * exchange, so the reference's row overruns (which scale neighbouring
 * rows by 1/pivot) are harmless and the result is exact. */
static const double k_unit_pivot[16] = {1.0,  0.5,  0.0,  0.0,
                                        0.5,  1.25, 0.5,  0.0,
                                        0.0,  0.5,  1.25, 0.5,
                                        0.0,  0.0,  0.5,  1.25};

IVP_TEST(incr_lu_decompose_increment_decrement)
{
    ivp_incr_lu_t lu;
    IVP_ASSERT_EQ(ivp_incr_lu_init(&lu, 4, NULL), IVP_OK);
    int arl = lu.aligned_row_len;

    /* decompose the leading 3x3 */
    lu.n_sub = 3;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) lu.U_matrix[i * arl + j] = k_unit_pivot[i * 4 + j];
    }
    IVP_ASSERT_EQ(ivp_incr_lu_decompose(&lu), IVP_OK);
    const double b3[3] = {1.0, 2.0, 3.0};
    for (int i = 0; i < 3; i++) lu.input_vec[i] = b3[i];
    ivp_incr_lu_solve_lin_equ(&lu);
    for (int i = 0; i < 3; i++) {
        double r = -b3[i];
        for (int j = 0; j < 3; j++) r += k_unit_pivot[i * 4 + j] * lu.out_vec[j];
        IVP_ASSERT_NEAR(r, 0.0, 1e-14);
    }

    /* grow to 4x4: new row in U row n_sub, new column in input_vec */
    for (int j = 0; j < 4; j++) lu.U_matrix[3 * arl + j] = k_unit_pivot[3 * 4 + j];
    for (int i = 0; i < 3; i++) lu.input_vec[i] = k_unit_pivot[i * 4 + 3];
    IVP_ASSERT_EQ(ivp_incr_lu_increment(&lu), IVP_OK);
    IVP_ASSERT_EQ(lu.n_sub, 4);
    const double b4[4] = {1.0, -1.0, 2.0, 0.5};
    for (int i = 0; i < 4; i++) lu.input_vec[i] = b4[i];
    ivp_incr_lu_solve_lin_equ(&lu);
    for (int i = 0; i < 4; i++) {
        double r = -b4[i];
        for (int j = 0; j < 4; j++) r += k_unit_pivot[i * 4 + j] * lu.out_vec[j];
        IVP_ASSERT_NEAR(r, 0.0, 1e-14);
    }

    /* drop the last variable again */
    IVP_ASSERT_EQ(ivp_incr_lu_decrement(&lu, 3), IVP_OK);
    IVP_ASSERT_EQ(lu.n_sub, 3);
    for (int i = 0; i < 3; i++) lu.input_vec[i] = b3[i];
    ivp_incr_lu_solve_lin_equ(&lu);
    for (int i = 0; i < 3; i++) {
        double r = -b3[i];
        for (int j = 0; j < 3; j++) r += k_unit_pivot[i * 4 + j] * lu.out_vec[j];
        IVP_ASSERT_NEAR(r, 0.0, 1e-14);
    }
    ivp_incr_lu_destroy(&lu, NULL);
}

IVP_TEST(incr_lu_singular_faults)
{
    ivp_incr_lu_t lu;
    IVP_ASSERT_EQ(ivp_incr_lu_init(&lu, 2, NULL), IVP_OK);
    int arl = lu.aligned_row_len;
    lu.n_sub = 2;
    lu.U_matrix[0] = 1.0; lu.U_matrix[1] = 2.0;
    lu.U_matrix[arl] = 2.0; lu.U_matrix[arl + 1] = 4.0;
    IVP_ASSERT_EQ(ivp_incr_lu_decompose(&lu), IVP_GM_FAULT);
    ivp_incr_lu_destroy(&lu, NULL);
}

/* ── linear constraint solver ───────────────────────────────────────────── */

static int lcp_holds(const double *A, const double *b, const double *x, int n, double tol)
{
    for (int i = 0; i < n; i++) {
        double acc = -b[i];
        for (int j = 0; j < n; j++) acc += A[i * n + j] * x[j];
        if (x[i] < 0.0 || acc < -tol || fabs(x[i] * acc) > tol) return 0;
    }
    return 1;
}

IVP_TEST(lcs_scalar_cases)
{
    double A = 2.0, b = 4.0, x = -1.0;
    IVP_ASSERT_EQ(ivp_lcs_init_and_solve(&A, &b, &x, 1, 0, NULL), IVP_OK);
    IVP_ASSERT_NEAR(x, 2.0, 1e-15);
    b = -1.0; /* separating: stays inactive */
    IVP_ASSERT_EQ(ivp_lcs_init_and_solve(&A, &b, &x, 1, 1, NULL), IVP_OK);
    IVP_ASSERT_TRUE(x == 0.0);
}

IVP_TEST(lcs_mixed_active_inactive)
{
    /* x1 alone would need a negative push: it must end inactive */
    const double A[9] = {2.0, 1.0, 0.0,
                         1.0, 2.0, 0.5,
                         0.0, 0.5, 1.0};
    const double b[3] = {1.0, -2.0, 0.5};
    double x[3];
    for (int act = 0; act <= 3; act++) {
        IVP_ASSERT_EQ(ivp_lcs_init_and_solve(A, b, x, 3, act, NULL), IVP_OK);
        IVP_ASSERT_TRUE(lcp_holds(A, b, x, 3, 1e-9));
        IVP_ASSERT_TRUE(x[1] == 0.0);
        IVP_ASSERT_NEAR(x[0], 0.5, 1e-12);
        IVP_ASSERT_NEAR(x[2], 0.5, 1e-12);
    }
}

IVP_TEST(lcs_strided_matches_dense)
{
    const int n = 3, stride = 8;
    const double A[9] = {3.0, 1.0, 0.5, 1.0, 2.0, 0.25, 0.5, 0.25, 1.5};
    const double b[3] = {1.0, 0.5, -0.25};
    double As[24];
    for (int i = 0; i < 24; i++) As[i] = 99.0; /* padding must be ignored */
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) As[i * stride + j] = A[i * n + j];
    }
    double x0[3], x1[3];
    IVP_ASSERT_EQ(ivp_lcs_init_and_solve(A, b, x0, n, 0, NULL), IVP_OK);
    IVP_ASSERT_EQ(ivp_lcs_init_and_solve_strided(As, stride, b, x1, n, 0, NULL), IVP_OK);
    IVP_ASSERT_TRUE(memcmp(x0, x1, sizeof(x0)) == 0);
    IVP_ASSERT_TRUE(lcp_holds(A, b, x0, n, 1e-9));
}

IVP_TEST(lcs_invalid_args)
{
    double A = 1.0, b = 1.0, x = 0.0;
    IVP_ASSERT_EQ(ivp_lcs_init_and_solve(&A, &b, &x, -1, 0, NULL), IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_lcs_init_and_solve(&A, &b, &x, 1, 2, NULL), IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_lcs_init_and_solve(NULL, &b, &x, 1, 0, NULL), IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_lcs_init_and_solve_strided(&A, 0, &b, &x, 1, 0, NULL), IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_lcs_init_and_solve(NULL, NULL, NULL, 0, 0, NULL), IVP_OK);
}

/* Reference cf18246: the source of copy_to_sub_matrix is a packed
 * columns x columns buffer (row stride columns), even when the matrix rows
 * are padded to aligned_row_len; the sub matrix uses its own stride. */
IVP_TEST(gmmz_copy_to_sub_matrix_reads_packed_source)
{
    ivp_gmmz_t big, sub;
    IVP_ASSERT_EQ(ivp_gmmz_init(&big, 5, NULL), IVP_OK);
    IVP_ASSERT_EQ(ivp_gmmz_init(&sub, 2, NULL), IVP_OK);
    double packed[64];
    for (int k = 0; k < 64; k++) packed[k] = -1.0;
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++) packed[i * 5 + j] = (double)(10 * i + j);
    const int original_pos[2] = {3, 1};
    ivp_gmmz_copy_to_sub_matrix(&big, packed, &sub, original_pos);
    IVP_ASSERT_TRUE(sub.matrix_values[0] == 33.0);
    IVP_ASSERT_TRUE(sub.matrix_values[1] == 31.0);
    IVP_ASSERT_TRUE(sub.matrix_values[sub.aligned_row_len + 0] == 13.0);
    IVP_ASSERT_TRUE(sub.matrix_values[sub.aligned_row_len + 1] == 11.0);
    ivp_gmmz_destroy(&sub, NULL);
    ivp_gmmz_destroy(&big, NULL);
}

int main(void)
{
    IVP_REGISTER(gmmz_init_layout);
    IVP_REGISTER(gmmz_solve_1x1_2x2);
    IVP_REGISTER(gmmz_solve_3x3_needs_pivot);
    IVP_REGISTER(gmmz_solve_singular_consistent_picks_zero);
    IVP_REGISTER(gmmz_solve_inconsistent_faults_and_clears);
    IVP_REGISTER(gmmz_solve_4x4_reproduces_reference_overrun);
    IVP_REGISTER(gmmz_mult_and_mult_aligned);
    IVP_REGISTER(gmmz_check_unequation_line);
    IVP_REGISTER(gmmz_fill_from_bigger_matrix);
    IVP_REGISTER(gmmz_invert_3x3);
    IVP_REGISTER(gmmz_invert_singular_faults);
    IVP_REGISTER(gmmz_lu_crout_solve_5x5);
    IVP_REGISTER(vecfpu_unaligned_window);
    IVP_REGISTER(vecfpu_add_multiple_row_matches_scalar_rounding);
    IVP_REGISTER(vecfpu_reduction_bitwise_scalar);
    IVP_REGISTER(gmmz_large_transform_matches_serial_reference);
    IVP_REGISTER(incr_lu_solve_bitwise_scalar);
    IVP_REGISTER(incr_lu_decompose_increment_decrement);
    IVP_REGISTER(incr_lu_singular_faults);
    IVP_REGISTER(lcs_scalar_cases);
    IVP_REGISTER(lcs_mixed_active_inactive);
    IVP_REGISTER(lcs_strided_matches_dense);
    IVP_REGISTER(lcs_invalid_args);
    IVP_REGISTER(gmmz_copy_to_sub_matrix_reads_packed_source);
    IVP_RUN_ALL();
}
