/* Reference-precision private matrix helpers: preserve scalar evaluation,
 * dynamic rounding, signed zeros and in-place input/output. */
#include "ivp_test.h"
#include "collision/ivp_lvec.h"
#include "dynamics/ivp_sim_internal.h"
#include "solver/ivp_scr_internal.h"
#include <fenv.h>
#include <stdint.h>

typedef void (*transform_fn_t)(const ivp_transform_r_t *, const ivp_vec3f_t *,
                               ivp_vec3f_t *);
/* Exercise the actual helpers even when a compiler could fold a comparison
 * between identical inline implementations. */
static transform_fn_t volatile transforms[] = {lvm_vmult3_f, lvm_vimult3_f};
typedef void (*point_transform_fn_t)(const ivp_transform_r_t *,
                                     const ivp_vec3r_t *, ivp_vec3f_t *);
static point_transform_fn_t volatile inverse_point_transform = lvm_vimult4_pf;
typedef void (*real_transform_fn_t)(const ivp_transform_r_t *,
                                    const ivp_vec3r_t *, ivp_vec3r_t *);
static real_transform_fn_t volatile real_transforms[] = {lvm_vmult3, lvm_vmult4};
typedef void (*cross_fn_t)(ivp_vec3f_t *, const ivp_vec3f_t *, const ivp_vec3f_t *);
static cross_fn_t volatile cross_product = lvf_cross;
typedef D (*dot_fn_t)(const ivp_vec3f_t *, const ivp_vec3f_t *);
static dot_fn_t volatile vector_dot = lvf_dot;
typedef D (*length_fn_t)(const ivp_vec3f_t *);
static length_fn_t volatile vector_quad_length = lvf_quad_length;
typedef void (*scale_fn_t)(ivp_vec3f_t *, D);
static scale_fn_t volatile vector_scale = lvf_mult;
typedef void (*binary_fn_t)(ivp_vec3f_t *, const ivp_vec3f_t *,
                            const ivp_vec3f_t *);
static binary_fn_t volatile vector_add = lvf_add2;
static binary_fn_t volatile vector_subtract = lvf_subtract2;
static binary_fn_t volatile vector_pairwise_mult = lvf_set_pairwise_mult;
typedef void (*combine_fn_t)(ivp_vec3f_t *, const ivp_vec3f_t *,
                             const ivp_vec3f_t *, D);
static combine_fn_t volatile vector_add_multiple = lvf_add_multiple2;
static combine_fn_t volatile vector_subtract_and_mult = lvf_subtract_and_mult;
static bool same_vec_and_flags(const ivp_vec3f_t *actual,
                               const ivp_vec3f_t *expected,
                               int actual_flags, int expected_flags);

/* IVP_U_Matrix3::{inline_vmult3,inline_vimult3} from
 * ivu_linear_software.hxx. Keep repeated conversions in the reference path. */
static void scalar_transform(const ivp_transform_r_t *m, const ivp_vec3f_t *in,
                             ivp_vec3f_t *out, int transpose)
{
    D a, b, c;
    if (transpose) {
        a = m->rot.rows[0].k[0] * (D)in->k[0] + m->rot.rows[1].k[0] * (D)in->k[1] + m->rot.rows[2].k[0] * (D)in->k[2];
        b = m->rot.rows[0].k[1] * (D)in->k[0] + m->rot.rows[1].k[1] * (D)in->k[1] + m->rot.rows[2].k[1] * (D)in->k[2];
        c = m->rot.rows[0].k[2] * (D)in->k[0] + m->rot.rows[1].k[2] * (D)in->k[1] + m->rot.rows[2].k[2] * (D)in->k[2];
    } else {
        a = m->rot.rows[0].k[0] * (D)in->k[0] + m->rot.rows[0].k[1] * (D)in->k[1] + m->rot.rows[0].k[2] * (D)in->k[2];
        b = m->rot.rows[1].k[0] * (D)in->k[0] + m->rot.rows[1].k[1] * (D)in->k[1] + m->rot.rows[1].k[2] * (D)in->k[2];
        c = m->rot.rows[2].k[0] * (D)in->k[0] + m->rot.rows[2].k[1] * (D)in->k[1] + m->rot.rows[2].k[2] * (D)in->k[2];
    }
    out->k[0] = (float)a;
    out->k[1] = (float)b;
    out->k[2] = (float)c;
}

static float float_from_bits(uint32_t bits)
{
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static float random_float(uint32_t *state)
{
    *state = *state * 1664525u + 1013904223u;
    uint32_t bits = (*state & 0x807fffffu) | ((96u + ((*state >> 23) % 64u)) << 23);
    return float_from_bits(bits);
}

static bool compare_transform(const ivp_transform_r_t *matrix,
                              const ivp_vec3f_t *input, int transpose, bool inplace)
{
    ivp_vec3f_t expected, actual = *input;
    if (feclearexcept(FE_ALL_EXCEPT)) return false;
    scalar_transform(matrix, input, &expected, transpose);
    int expected_flags = fetestexcept(FE_ALL_EXCEPT);
    if (feclearexcept(FE_ALL_EXCEPT)) return false;
    transforms[transpose](matrix, inplace ? &actual : input, &actual);
    int actual_flags = fetestexcept(FE_ALL_EXCEPT);
    /* Compare only components, never the alignment padding. */
    return memcmp(actual.k, expected.k, sizeof(actual.k)) == 0 &&
           actual_flags == expected_flags;
}

static bool exercise_transforms(bool edge_cases, bool inplace)
{
    static const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
    static const uint32_t edges[] = {
        0x00000000u, 0x80000000u, 0x00000001u, 0x00800000u,
        0x7f7fffffu, 0xff7fffffu, 0x7f800000u, 0x7fc01234u
    };
    fenv_t saved;
    if (feholdexcept(&saved)) return false;
    bool ok = true;
    uint32_t random_state = 0x7529173bu;
    for (size_t mode = 0; ok && mode < sizeof(modes) / sizeof(modes[0]); mode++) {
        ok = fesetround(modes[mode]) == 0;
        for (int n = 0; ok && n < (edge_cases ? 512 : 2048); n++) {
            ivp_transform_r_t matrix = {0};
            ivp_vec3f_t input;
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    matrix.rot.rows[row].k[col] = (D)random_float(&random_state) +
                        (D)random_float(&random_state) * (D)0x1p-26;
                }
                input.k[row] = edge_cases ? float_from_bits(edges[(n >> (row * 3)) & 7])
                                          : random_float(&random_state);
            }
            for (int transpose = 0; ok && transpose < 2; transpose++) {
                ok = compare_transform(&matrix, &input, transpose, inplace);
                if (!ok) fprintf(stderr, "matrix mismatch: rounding=%zu case=%d transpose=%d inplace=%d\n",
                                 mode, n, transpose, (int)inplace);
            }
        }
    }
    /* Restore before assertions, including failure paths. */
    return fesetenv(&saved) == 0 && ok;
}

IVP_TEST(float_matrix_products_match_scalar_rounding)
{
    IVP_ASSERT_TRUE(exercise_transforms(false, false));
}

IVP_TEST(float_matrix_products_preserve_edge_values)
{
    IVP_ASSERT_TRUE(exercise_transforms(true, false));
}

IVP_TEST(float_matrix_products_allow_inplace_input)
{
    IVP_ASSERT_TRUE(exercise_transforms(false, true));
    IVP_ASSERT_TRUE(exercise_transforms(true, true));
}

static void scalar_real_transform(const ivp_transform_r_t *m,
                                  const ivp_vec3r_t *in,
                                  ivp_vec3r_t *out, bool translate)
{
    D h = in->k[0];
    D a = h * m->rot.rows[0].k[0];
    D b = h * m->rot.rows[1].k[0];
    D c = h * m->rot.rows[2].k[0];
    h = in->k[1];
    a += h * m->rot.rows[0].k[1];
    b += h * m->rot.rows[1].k[1];
    c += h * m->rot.rows[2].k[1];
    h = in->k[2];
    a += h * m->rot.rows[0].k[2];
    b += h * m->rot.rows[1].k[2];
    c += h * m->rot.rows[2].k[2];
    if (translate) {
        a += m->pos.k[0];
        b += m->pos.k[1];
        c += m->pos.k[2];
    }
    out->k[0] = a;
    out->k[1] = b;
    out->k[2] = c;
}

static bool exercise_real_transforms(void)
{
    static const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
    fenv_t saved;
    if (feholdexcept(&saved)) return false;
    bool ok = true;
    uint32_t random_state = 0xa4093822u;
    for (size_t mode = 0; ok && mode < sizeof(modes) / sizeof(modes[0]); mode++) {
        ok = fesetround(modes[mode]) == 0;
        for (int n = 0; ok && n < 2048; n++) {
            ivp_transform_r_t matrix = {0};
            ivp_vec3r_t input;
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    matrix.rot.rows[row].k[col] =
                        (D)random_float(&random_state) +
                        (D)random_float(&random_state) * (D)0x1p-26;
                }
                matrix.pos.k[row] = (D)random_float(&random_state) +
                                     (D)random_float(&random_state) * (D)0x1p-27;
                input.k[row] = (D)random_float(&random_state) +
                               (D)random_float(&random_state) * (D)0x1p-25;
            }
            for (int translate = 0; ok && translate < 2; translate++) {
                ok = feclearexcept(FE_ALL_EXCEPT) == 0;
                ivp_vec3r_t expected;
                scalar_real_transform(&matrix, &input, &expected, translate != 0);
                int expected_flags = fetestexcept(FE_ALL_EXCEPT);
                for (int inplace = 0; ok && inplace < 2; inplace++) {
                    ivp_vec3r_t actual = input, separate;
                    ivp_vec3r_t *out = inplace ? &actual : &separate;
                    ok = feclearexcept(FE_ALL_EXCEPT) == 0;
                    real_transforms[translate](&matrix, &actual, out);
                    ok = ok && memcmp(out->k, expected.k, sizeof(expected.k)) == 0 &&
                         fetestexcept(FE_ALL_EXCEPT) == expected_flags;
                    if (!ok) {
                        fprintf(stderr,
                                "real matrix mismatch: rounding=%zu case=%d translate=%d inplace=%d\n",
                                mode, n, translate, inplace);
                    }
                }
            }
        }
    }
    return fesetenv(&saved) == 0 && ok;
}

IVP_TEST(real_matrix_products_match_scalar_rounding_and_aliases)
{
    IVP_ASSERT_TRUE(exercise_real_transforms());
}

static bool exercise_inverse_point_transforms(void)
{
    static const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
    fenv_t saved;
    if (feholdexcept(&saved)) return false;
    bool ok = true;
    uint32_t random_state = 0x13198a2eu;
    for (size_t mode = 0; ok && mode < sizeof(modes) / sizeof(modes[0]); mode++) {
        ok = fesetround(modes[mode]) == 0;
        for (int n = 0; ok && n < 2048; n++) {
            ivp_transform_r_t matrix = {0};
            ivp_vec3r_t input;
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    matrix.rot.rows[row].k[col] = (D)random_float(&random_state) +
                        (D)random_float(&random_state) * (D)0x1p-26;
                }
                matrix.pos.k[row] = (D)random_float(&random_state);
                input.k[row] = (D)random_float(&random_state) +
                               (D)random_float(&random_state) * (D)0x1p-27;
            }
            ok = feclearexcept(FE_ALL_EXCEPT) == 0;
            D a = input.k[0] - matrix.pos.k[0];
            D b = input.k[1] - matrix.pos.k[1];
            D c = input.k[2] - matrix.pos.k[2];
            ivp_vec3f_t expected;
            expected.k[0] = (float)(matrix.rot.rows[0].k[0] * a +
                                    matrix.rot.rows[1].k[0] * b +
                                    matrix.rot.rows[2].k[0] * c);
            expected.k[1] = (float)(matrix.rot.rows[0].k[1] * a +
                                    matrix.rot.rows[1].k[1] * b +
                                    matrix.rot.rows[2].k[1] * c);
            expected.k[2] = (float)(matrix.rot.rows[0].k[2] * a +
                                    matrix.rot.rows[1].k[2] * b +
                                    matrix.rot.rows[2].k[2] * c);
            int expected_flags = fetestexcept(FE_ALL_EXCEPT);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            ivp_vec3f_t actual;
            inverse_point_transform(&matrix, &input, &actual);
            ok = ok && same_vec_and_flags(&actual, &expected,
                                          fetestexcept(FE_ALL_EXCEPT), expected_flags);
            if (!ok) fprintf(stderr, "point transform mismatch: rounding=%zu case=%d\n", mode, n);
        }
    }
    return fesetenv(&saved) == 0 && ok;
}

IVP_TEST(float_inverse_point_transform_matches_scalar_rounding)
{
    IVP_ASSERT_TRUE(exercise_inverse_point_transforms());
}

/* The reference multiplies floats, subtracts in D, then rounds to float.
 * Replacing this with an all-float cross product changes the rounding. */
static void scalar_cross(ivp_vec3f_t *out, const ivp_vec3f_t *lhs,
                         const ivp_vec3f_t *rhs)
{
    D a = (D)(lhs->k[1] * rhs->k[2]), a2 = (D)(rhs->k[1] * lhs->k[2]);
    D b = (D)(lhs->k[2] * rhs->k[0]), b2 = (D)(rhs->k[2] * lhs->k[0]);
    D c = (D)(lhs->k[0] * rhs->k[1]), c2 = (D)(rhs->k[0] * lhs->k[1]);
    a -= a2; b -= b2; c -= c2;
    out->k[0] = (float)a; out->k[1] = (float)b; out->k[2] = (float)c;
}

static bool exercise_cross_products(void)
{
    static const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
    static const uint32_t edges[] = {
        0x00000000u, 0x80000000u, 0x00000001u, 0x00800000u,
        0x7f7fffffu, 0xff7fffffu, 0x7f800000u, 0x7fc01234u
    };
    fenv_t saved;
    if (feholdexcept(&saved)) return false;
    bool ok = true;
    uint32_t random_state = 0x95721aceu;
    for (size_t mode = 0; ok && mode < sizeof(modes) / sizeof(modes[0]); mode++) {
        ok = fesetround(modes[mode]) == 0;
        for (int n = 0; ok && n < 4096; n++) {
            ivp_vec3f_t lhs, rhs, expected;
            for (int i = 0; i < 3; i++) {
                lhs.k[i] = n < 512 ? float_from_bits(edges[(n >> (i * 3)) & 7])
                                  : random_float(&random_state);
                rhs.k[i] = n < 512 ? float_from_bits(edges[((n + 37) >> (i * 3)) & 7])
                                  : random_float(&random_state);
            }
            ok = feclearexcept(FE_ALL_EXCEPT) == 0;
            scalar_cross(&expected, &lhs, &rhs);
            int expected_flags = fetestexcept(FE_ALL_EXCEPT);
            for (int alias = 0; ok && alias < 3; alias++) {
                ivp_vec3f_t a = lhs, b = rhs, separate;
                ivp_vec3f_t *out = alias == 0 ? &separate : alias == 1 ? &a : &b;
                ok = feclearexcept(FE_ALL_EXCEPT) == 0;
                cross_product(out, &a, &b);
                ok = ok && fetestexcept(FE_ALL_EXCEPT) == expected_flags &&
                     memcmp(out->k, expected.k, sizeof(expected.k)) == 0;
                if (!ok) fprintf(stderr, "cross mismatch: rounding=%zu case=%d alias=%d\n",
                                 mode, n, alias);
            }
        }
    }
    return fesetenv(&saved) == 0 && ok;
}

IVP_TEST(float_cross_products_match_scalar_rounding_and_aliases)
{
    IVP_ASSERT_TRUE(exercise_cross_products());
}

static void scalar_add_multiple(ivp_vec3f_t *out, const ivp_vec3f_t *lhs,
                                const ivp_vec3f_t *rhs, D factor)
{
    D x = (D)rhs->k[0] * factor + (D)lhs->k[0];
    D y = (D)rhs->k[1] * factor + (D)lhs->k[1];
    D z = (D)rhs->k[2] * factor + (D)lhs->k[2];
    out->k[0] = (float)x; out->k[1] = (float)y; out->k[2] = (float)z;
}

static void scalar_subtract_and_mult(ivp_vec3f_t *out,
                                     const ivp_vec3f_t *lhs,
                                     const ivp_vec3f_t *rhs, D factor)
{
    D x = (D)(lhs->k[0] - rhs->k[0]) * factor;
    D y = (D)(lhs->k[1] - rhs->k[1]) * factor;
    D z = (D)(lhs->k[2] - rhs->k[2]) * factor;
    out->k[0] = (float)x; out->k[1] = (float)y; out->k[2] = (float)z;
}

static bool same_vec_and_flags(const ivp_vec3f_t *actual,
                               const ivp_vec3f_t *expected,
                               int actual_flags, int expected_flags)
{
    return memcmp(actual->k, expected->k, sizeof(actual->k)) == 0 &&
           actual_flags == expected_flags;
}

static bool exercise_vector_arithmetic(void)
{
    static const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
    static const uint32_t edges[] = {
        0x00000000u, 0x80000000u, 0x00000001u, 0x00800000u,
        0x7f7fffffu, 0xff7fffffu, 0x7f800000u, 0x7fc01234u
    };
    fenv_t saved;
    if (feholdexcept(&saved)) return false;
    bool ok = true;
    uint32_t random_state = 0x61c88647u;
    for (size_t mode = 0; ok && mode < sizeof(modes) / sizeof(modes[0]); mode++) {
        ok = fesetround(modes[mode]) == 0;
        for (int n = 0; ok && n < 2048; n++) {
            ivp_vec3f_t lhs, rhs;
            for (int i = 0; i < 3; i++) {
                lhs.k[i] = n < 512 ? float_from_bits(edges[(n >> (i * 3)) & 7])
                                   : random_float(&random_state);
                rhs.k[i] = n < 512 ? float_from_bits(edges[((n + 19) >> (i * 3)) & 7])
                                   : random_float(&random_state);
            }
            D factor = (D)random_float(&random_state) +
                       (D)random_float(&random_state) * (D)0x1p-26;

            ok = feclearexcept(FE_ALL_EXCEPT) == 0;
            D expected_dot = (D)(lhs.k[0] * rhs.k[0] + lhs.k[1] * rhs.k[1] +
                                 lhs.k[2] * rhs.k[2]);
            int expected_flags = fetestexcept(FE_ALL_EXCEPT);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            D actual_dot = vector_dot(&lhs, &rhs);
            ok = ok && memcmp(&actual_dot, &expected_dot, sizeof(actual_dot)) == 0 &&
                 fetestexcept(FE_ALL_EXCEPT) == expected_flags;

            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            D expected_length = (D)(lhs.k[0] * lhs.k[0] + lhs.k[1] * lhs.k[1] +
                                    lhs.k[2] * lhs.k[2]);
            expected_flags = fetestexcept(FE_ALL_EXCEPT);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            D actual_length = vector_quad_length(&lhs);
            ok = ok && memcmp(&actual_length, &expected_length, sizeof(actual_length)) == 0 &&
                 fetestexcept(FE_ALL_EXCEPT) == expected_flags;

            ivp_vec3f_t expected, actual;
            for (int operation = 0; ok && operation < 3; operation++) {
                ok = feclearexcept(FE_ALL_EXCEPT) == 0;
                for (int i = 0; i < 3; i++) {
                    expected.k[i] = operation == 0 ? lhs.k[i] + rhs.k[i] :
                                    operation == 1 ? lhs.k[i] - rhs.k[i] :
                                                     lhs.k[i] * rhs.k[i];
                }
                expected_flags = fetestexcept(FE_ALL_EXCEPT);
                for (int alias = 0; ok && alias < 3; alias++) {
                    ivp_vec3f_t a = lhs, b = rhs;
                    ivp_vec3f_t *out = alias == 0 ? &actual : alias == 1 ? &a : &b;
                    ok = feclearexcept(FE_ALL_EXCEPT) == 0;
                    binary_fn_t fn = operation == 0 ? vector_add :
                                     operation == 1 ? vector_subtract : vector_pairwise_mult;
                    fn(out, &a, &b);
                    ok = ok && same_vec_and_flags(out, &expected,
                                                  fetestexcept(FE_ALL_EXCEPT), expected_flags);
                }
            }

            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            scalar_add_multiple(&expected, &lhs, &rhs, factor);
            expected_flags = fetestexcept(FE_ALL_EXCEPT);
            for (int alias = 0; ok && alias < 3; alias++) {
                ivp_vec3f_t a = lhs, b = rhs;
                ivp_vec3f_t *out = alias == 0 ? &actual : alias == 1 ? &a : &b;
                ok = feclearexcept(FE_ALL_EXCEPT) == 0;
                vector_add_multiple(out, &a, &b, factor);
                ok = ok && same_vec_and_flags(out, &expected,
                                              fetestexcept(FE_ALL_EXCEPT), expected_flags);
            }

            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            scalar_subtract_and_mult(&expected, &lhs, &rhs, factor);
            expected_flags = fetestexcept(FE_ALL_EXCEPT);
            for (int alias = 0; ok && alias < 3; alias++) {
                ivp_vec3f_t a = lhs, b = rhs;
                ivp_vec3f_t *out = alias == 0 ? &actual : alias == 1 ? &a : &b;
                ok = feclearexcept(FE_ALL_EXCEPT) == 0;
                vector_subtract_and_mult(out, &a, &b, factor);
                ok = ok && same_vec_and_flags(out, &expected,
                                              fetestexcept(FE_ALL_EXCEPT), expected_flags);
            }

            actual = lhs;
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            expected.k[0] = (float)((D)lhs.k[0] * factor);
            expected.k[1] = (float)((D)lhs.k[1] * factor);
            expected.k[2] = (float)((D)lhs.k[2] * factor);
            expected_flags = fetestexcept(FE_ALL_EXCEPT);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            vector_scale(&actual, factor);
            ok = ok && same_vec_and_flags(&actual, &expected,
                                          fetestexcept(FE_ALL_EXCEPT), expected_flags);

#ifdef IVP_REAL_DOUBLE
            ivp_vec3r_t point_a, point_b;
            for (int i = 0; i < 3; i++) {
                point_a.k[i] = (D)lhs.k[i] + (D)rhs.k[i] * (D)0x1p-26;
                point_b.k[i] = (D)rhs.k[i] - (D)lhs.k[i] * (D)0x1p-27;
            }
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            for (int i = 0; i < 3; i++) expected.k[i] = (float)point_a.k[i];
            expected_flags = fetestexcept(FE_ALL_EXCEPT);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            lvf_set_p(&actual, &point_a);
            ok = ok && same_vec_and_flags(&actual, &expected,
                                          fetestexcept(FE_ALL_EXCEPT), expected_flags);

            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            for (int i = 0; i < 3; i++) expected.k[i] = (float)(point_a.k[i] - point_b.k[i]);
            expected_flags = fetestexcept(FE_ALL_EXCEPT);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            lvf_subtract2_p(&actual, &point_a, &point_b);
            ok = ok && same_vec_and_flags(&actual, &expected,
                                          fetestexcept(FE_ALL_EXCEPT), expected_flags);

            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            for (int i = 0; i < 3; i++) expected.k[i] = (float)(point_a.k[i] * factor);
            expected_flags = fetestexcept(FE_ALL_EXCEPT);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            lvf_set_multiple_p(&actual, &point_a, factor);
            ok = ok && same_vec_and_flags(&actual, &expected,
                                          fetestexcept(FE_ALL_EXCEPT), expected_flags);
#endif
            if (!ok) fprintf(stderr, "vector mismatch: rounding=%zu case=%d\n", mode, n);
        }
    }
    return fesetenv(&saved) == 0 && ok;
}

IVP_TEST(float_vector_arithmetic_matches_scalar_rounding_and_aliases)
{
    IVP_ASSERT_TRUE(exercise_vector_arithmetic());
}

static bool exercise_core_float_views(void)
{
    static const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
    fenv_t saved;
    if (feholdexcept(&saved)) return false;
    bool ok = true;
    uint32_t random_state = 0x243f6a88u;
    for (size_t mode = 0; ok && mode < sizeof(modes) / sizeof(modes[0]); mode++) {
        ok = fesetround(modes[mode]) == 0;
        for (int n = 0; ok && n < 2048; n++) {
            ivp_core_t core = {0};
            ivp_vec3f_t source, expected;
            for (int i = 0; i < 3; i++) {
                D value = (D)random_float(&random_state) +
                          (D)random_float(&random_state) * (D)0x1p-26;
                core.speed.k[i] = value;
                core.inv_rot_inertia.k[i] = value * (D)0.75;
                core.rot_inertia.k[i] = value * (D)-1.25;
                source.k[i] = random_float(&random_state);
            }

            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            for (int i = 0; i < 3; i++) expected.k[i] = (float)core.speed.k[i];
            int expected_flags = fetestexcept(FE_ALL_EXCEPT);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            ivp_vec3f_t actual = ivp_core_speed_f(&core);
            ok = ok && same_vec_and_flags(&actual, &expected,
                                          fetestexcept(FE_ALL_EXCEPT), expected_flags);

            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            for (int i = 0; i < 3; i++) expected.k[i] = (float)core.inv_rot_inertia.k[i];
            expected_flags = fetestexcept(FE_ALL_EXCEPT);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            actual = ivp_core_inv_rot_inertia_f(&core);
            ok = ok && same_vec_and_flags(&actual, &expected,
                                          fetestexcept(FE_ALL_EXCEPT), expected_flags);

            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            for (int i = 0; i < 3; i++) expected.k[i] = (float)core.rot_inertia.k[i];
            expected_flags = fetestexcept(FE_ALL_EXCEPT);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            actual = ivp_core_rot_inertia_f(&core);
            ok = ok && same_vec_and_flags(&actual, &expected,
                                          fetestexcept(FE_ALL_EXCEPT), expected_flags);

            ivp_core_set_speed_f(&core, &source);
            for (int i = 0; ok && i < 3; i++) {
                D expected_value = (D)source.k[i];
                ok = memcmp(&core.speed.k[i], &expected_value, sizeof(D)) == 0;
            }
            if (!ok) fprintf(stderr, "core float view mismatch: rounding=%zu case=%d\n", mode, n);
        }
    }
    return fesetenv(&saved) == 0 && ok;
}

IVP_TEST(core_float_views_match_scalar_rounding)
{
    IVP_ASSERT_TRUE(exercise_core_float_views());
}

static bool exercise_scr_pairwise_products(void)
{
    static const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
    fenv_t saved;
    if (feholdexcept(&saved)) return false;
    bool ok = true;
    uint32_t random_state = 0xb7e15162u;
    for (size_t mode = 0; ok && mode < sizeof(modes) / sizeof(modes[0]); mode++) {
        ok = fesetround(modes[mode]) == 0;
        for (int n = 0; ok && n < 2048; n++) {
            ivp_scr_p4_t lhs, rhs, expected, actual;
            for (int i = 0; i < 3; i++) {
                lhs.k[i] = random_float(&random_state);
                rhs.k[i] = random_float(&random_state);
            }
            lhs.hesse_val = random_float(&random_state);
            rhs.hesse_val = random_float(&random_state);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            for (int i = 0; i < 3; i++) expected.k[i] = lhs.k[i] * rhs.k[i];
            expected.hesse_val = lhs.hesse_val * rhs.hesse_val;
            int expected_flags = fetestexcept(FE_ALL_EXCEPT);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            ivp_scr_f_line_wise_mult4(&actual, &lhs, &rhs);
            ok = ok && memcmp(&actual, &expected, sizeof(actual)) == 0 &&
                 fetestexcept(FE_ALL_EXCEPT) == expected_flags;

            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            double expected_dot3 = (double)(lhs.k[0] * rhs.k[0] +
                                            lhs.k[1] * rhs.k[1] +
                                            lhs.k[2] * rhs.k[2]);
            expected_flags = fetestexcept(FE_ALL_EXCEPT);
            int expected_dot3_flags = expected_flags;
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            double actual_dot = ivp_scr_f_dot3(&lhs, &rhs);
            ok = ok && memcmp(&actual_dot, &expected_dot3, sizeof(actual_dot)) == 0 &&
                 fetestexcept(FE_ALL_EXCEPT) == expected_flags;

            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            double expected_dot4 = (double)(lhs.k[0] * rhs.k[0] +
                                            lhs.k[1] * rhs.k[1] +
                                            lhs.k[2] * rhs.k[2] +
                                            lhs.hesse_val * rhs.hesse_val);
            expected_flags = fetestexcept(FE_ALL_EXCEPT);
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            actual_dot = ivp_scr_f_dot4(&lhs, &rhs);
            ok = ok && memcmp(&actual_dot, &expected_dot4, sizeof(actual_dot)) == 0 &&
                 fetestexcept(FE_ALL_EXCEPT) == expected_flags;

            ivp_vec3f_t vector = {{lhs.k[0], lhs.k[1], lhs.k[2]}};
            ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
            actual_dot = ivp_scr_f_dot_vp(&vector, &rhs);
            ok = ok && memcmp(&actual_dot, &expected_dot3, sizeof(actual_dot)) == 0 &&
                 fetestexcept(FE_ALL_EXCEPT) == expected_dot3_flags;
            if (!ok) fprintf(stderr, "SCR product mismatch: rounding=%zu case=%d\n", mode, n);
        }
    }
    return fesetenv(&saved) == 0 && ok;
}

IVP_TEST(scr_pairwise_products_match_scalar_rounding)
{
    IVP_ASSERT_TRUE(exercise_scr_pairwise_products());
}

static bool exercise_surface_speed_aliases(void)
{
    static const int modes[] = {FE_TONEAREST, FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
    fenv_t saved;
    if (feholdexcept(&saved)) return false;
    bool ok = true;
    uint32_t random_state = 0x9e3779b9u;
    for (size_t mode = 0; ok && mode < sizeof(modes) / sizeof(modes[0]); mode++) {
        ok = fesetround(modes[mode]) == 0;
        for (int n = 0; ok && n < 1024; n++) {
            ivp_core_t core = {0};
            ivp_vec3f_t point, center, rotation;
            for (int row = 0; row < 3; row++) {
                for (int col = 0; col < 3; col++) {
                    core.m_world_f_core_last_psi.rot.rows[row].k[col] =
                        (D)random_float(&random_state) +
                        (D)random_float(&random_state) * (D)0x1p-26;
                }
                point.k[row] = random_float(&random_state);
                center.k[row] = random_float(&random_state);
                rotation.k[row] = random_float(&random_state);
            }
            for (int alias = 0; ok && alias < 4; alias++) {
                ivp_vec3f_t p = point, c = center, r = rotation, actual;
                ivp_vec3f_t cross, transformed, expected;
                ok = feclearexcept(FE_ALL_EXCEPT) == 0;
                scalar_cross(&cross, &rotation, &point);
                scalar_transform(&core.m_world_f_core_last_psi, &cross,
                                 &transformed, 0);
                if (alias == 1) {
                    expected.k[0] = transformed.k[0] + transformed.k[0];
                    expected.k[1] = transformed.k[1] + transformed.k[1];
                    expected.k[2] = transformed.k[2] + transformed.k[2];
                } else {
                    expected.k[0] = transformed.k[0] + center.k[0];
                    expected.k[1] = transformed.k[1] + center.k[1];
                    expected.k[2] = transformed.k[2] + center.k[2];
                }
                int expected_flags = fetestexcept(FE_ALL_EXCEPT);
                ivp_vec3f_t *out = alias == 0 ? &actual : alias == 1 ? &c : alias == 2 ? &p : &r;
                ok = ok && feclearexcept(FE_ALL_EXCEPT) == 0;
                ivp_core_get_surface_speed_on_test(&core, &p, &c, &r, out);
                ok = ok && same_vec_and_flags(out, &expected,
                                              fetestexcept(FE_ALL_EXCEPT), expected_flags);
                if (!ok) fprintf(stderr, "surface speed mismatch: rounding=%zu case=%d alias=%d\n",
                                 mode, n, alias);
            }
        }
    }
    return fesetenv(&saved) == 0 && ok;
}

IVP_TEST(surface_speed_preserves_scalar_rounding_and_aliases)
{
    IVP_ASSERT_TRUE(exercise_surface_speed_aliases());
}

int main(void)
{
    IVP_REGISTER(float_matrix_products_match_scalar_rounding);
    IVP_REGISTER(float_matrix_products_preserve_edge_values);
    IVP_REGISTER(float_matrix_products_allow_inplace_input);
    IVP_REGISTER(real_matrix_products_match_scalar_rounding_and_aliases);
    IVP_REGISTER(float_inverse_point_transform_matches_scalar_rounding);
    IVP_REGISTER(float_cross_products_match_scalar_rounding_and_aliases);
    IVP_REGISTER(float_vector_arithmetic_matches_scalar_rounding_and_aliases);
    IVP_REGISTER(core_float_views_match_scalar_rounding);
    IVP_REGISTER(scr_pairwise_products_match_scalar_rounding);
    IVP_REGISTER(surface_speed_preserves_scalar_rounding_and_aliases);
    IVP_RUN_ALL();
}
