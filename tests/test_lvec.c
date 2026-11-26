/* Reference-precision private matrix helpers: preserve scalar evaluation,
 * dynamic rounding, signed zeros and in-place input/output. */
#include "ivp_test.h"
#include "collision/ivp_lvec.h"
#include "dynamics/ivp_sim_internal.h"
#include "solver/ivp_scr_internal.h"
#include <fenv.h>
#include <float.h>
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

static bool same_float_result(float actual, float expected)
{
    uint32_t actual_bits, expected_bits;
    memcpy(&actual_bits, &actual, sizeof(actual_bits));
    memcpy(&expected_bits, &expected, sizeof(expected_bits));
    if (actual_bits == expected_bits) return true;
    return (actual_bits & UINT32_C(0x7fffffff)) > UINT32_C(0x7f800000) &&
           (expected_bits & UINT32_C(0x7fffffff)) > UINT32_C(0x7f800000);
}

static bool same_real_result(D actual, D expected)
{
    if (memcmp(&actual, &expected, sizeof(actual)) == 0) return true;
#ifdef IVP_REAL_DOUBLE
    uint64_t actual_bits, expected_bits;
    memcpy(&actual_bits, &actual, sizeof(actual_bits));
    memcpy(&expected_bits, &expected, sizeof(expected_bits));
    return (actual_bits & UINT64_C(0x7fffffffffffffff)) >
               UINT64_C(0x7ff0000000000000) &&
           (expected_bits & UINT64_C(0x7fffffffffffffff)) >
               UINT64_C(0x7ff0000000000000);
#else
    return same_float_result(actual, expected);
#endif
}

static bool same_vec_result(const ivp_vec3f_t *actual,
                            const ivp_vec3f_t *expected)
{
    for (int i = 0; i < 3; i++) {
        if (!same_float_result(actual->k[i], expected->k[i])) return false;
    }
    return true;
}

static bool compare_transform(const ivp_transform_r_t *matrix,
                              const ivp_vec3f_t *input, int transpose, bool inplace)
{
    ivp_vec3f_t expected, actual = *input;
    scalar_transform(matrix, input, &expected, transpose);
    transforms[transpose](matrix, inplace ? &actual : input, &actual);
    /* Compare only components, never the alignment padding. */
    return same_vec_result(&actual, &expected);
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
                ivp_vec3r_t expected;
                scalar_real_transform(&matrix, &input, &expected, translate != 0);
                for (int inplace = 0; ok && inplace < 2; inplace++) {
                    ivp_vec3r_t actual = input, separate;
                    ivp_vec3r_t *out = inplace ? &actual : &separate;
                    real_transforms[translate](&matrix, &actual, out);
                    ok = ok && memcmp(out->k, expected.k,
                                      sizeof(expected.k)) == 0;
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
            ivp_vec3f_t actual;
            inverse_point_transform(&matrix, &input, &actual);
            ok = ok && same_vec_result(&actual, &expected);
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
            scalar_cross(&expected, &lhs, &rhs);
            for (int alias = 0; ok && alias < 3; alias++) {
                ivp_vec3f_t a = lhs, b = rhs, separate;
                ivp_vec3f_t *out = alias == 0 ? &separate : alias == 1 ? &a : &b;
                cross_product(out, &a, &b);
                ok = ok && same_vec_result(out, &expected);
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

            /* IVP_U_Float_Point::dot_product: double products summed z, y, x */
            D expected_dot = (D)((double)lhs.k[2] * (double)rhs.k[2] +
                                 (double)lhs.k[1] * (double)rhs.k[1] +
                                 (double)lhs.k[0] * (double)rhs.k[0]);
            D actual_dot = vector_dot(&lhs, &rhs);
            ok = ok && same_real_result(actual_dot, expected_dot);

            D expected_length = (D)(lhs.k[0] * lhs.k[0] + lhs.k[1] * lhs.k[1] +
                                    lhs.k[2] * lhs.k[2]);
            D actual_length = vector_quad_length(&lhs);
            ok = ok && same_real_result(actual_length, expected_length);

            ivp_vec3f_t expected, actual;
            for (int operation = 0; ok && operation < 3; operation++) {
                for (int i = 0; i < 3; i++) {
                    expected.k[i] = operation == 0 ? lhs.k[i] + rhs.k[i] :
                                    operation == 1 ? lhs.k[i] - rhs.k[i] :
                                                     lhs.k[i] * rhs.k[i];
                }
                for (int alias = 0; ok && alias < 3; alias++) {
                    ivp_vec3f_t a = lhs, b = rhs;
                    ivp_vec3f_t *out = alias == 0 ? &actual : alias == 1 ? &a : &b;
                    binary_fn_t fn = operation == 0 ? vector_add :
                                     operation == 1 ? vector_subtract : vector_pairwise_mult;
                    fn(out, &a, &b);
                    ok = ok && same_vec_result(out, &expected);
                }
            }

            scalar_add_multiple(&expected, &lhs, &rhs, factor);
            for (int alias = 0; ok && alias < 3; alias++) {
                ivp_vec3f_t a = lhs, b = rhs;
                ivp_vec3f_t *out = alias == 0 ? &actual : alias == 1 ? &a : &b;
                vector_add_multiple(out, &a, &b, factor);
                ok = ok && same_vec_result(out, &expected);
            }

            scalar_subtract_and_mult(&expected, &lhs, &rhs, factor);
            for (int alias = 0; ok && alias < 3; alias++) {
                ivp_vec3f_t a = lhs, b = rhs;
                ivp_vec3f_t *out = alias == 0 ? &actual : alias == 1 ? &a : &b;
                vector_subtract_and_mult(out, &a, &b, factor);
                ok = ok && same_vec_result(out, &expected);
            }

            actual = lhs;
            expected.k[0] = (float)((D)lhs.k[0] * factor);
            expected.k[1] = (float)((D)lhs.k[1] * factor);
            expected.k[2] = (float)((D)lhs.k[2] * factor);
            vector_scale(&actual, factor);
            ok = ok && same_vec_result(&actual, &expected);

#ifdef IVP_REAL_DOUBLE
            ivp_vec3r_t point_a, point_b;
            for (int i = 0; i < 3; i++) {
                point_a.k[i] = (D)lhs.k[i] + (D)rhs.k[i] * (D)0x1p-26;
                point_b.k[i] = (D)rhs.k[i] - (D)lhs.k[i] * (D)0x1p-27;
            }
            for (int i = 0; i < 3; i++) expected.k[i] = (float)point_a.k[i];
            lvf_set_p(&actual, &point_a);
            ok = ok && same_vec_result(&actual, &expected);

            for (int i = 0; i < 3; i++) expected.k[i] = (float)(point_a.k[i] - point_b.k[i]);
            lvf_subtract2_p(&actual, &point_a, &point_b);
            ok = ok && same_vec_result(&actual, &expected);

            for (int i = 0; i < 3; i++) expected.k[i] = (float)(point_a.k[i] * factor);
            lvf_set_multiple_p(&actual, &point_a, factor);
            ok = ok && same_vec_result(&actual, &expected);
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

            for (int i = 0; i < 3; i++) expected.k[i] = (float)core.speed.k[i];
            ivp_vec3f_t actual = ivp_core_speed_f(&core);
            ok = ok && same_vec_result(&actual, &expected);

            for (int i = 0; i < 3; i++) expected.k[i] = (float)core.inv_rot_inertia.k[i];
            actual = ivp_core_inv_rot_inertia_f(&core);
            ok = ok && same_vec_result(&actual, &expected);

            for (int i = 0; i < 3; i++) expected.k[i] = (float)core.rot_inertia.k[i];
            actual = ivp_core_rot_inertia_f(&core);
            ok = ok && same_vec_result(&actual, &expected);

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
            for (int i = 0; i < 3; i++) expected.k[i] = lhs.k[i] * rhs.k[i];
            expected.hesse_val = lhs.hesse_val * rhs.hesse_val;
            ivp_scr_f_line_wise_mult4(&actual, &lhs, &rhs);
            for (int i = 0; ok && i < 3; i++) {
                ok = same_float_result(actual.k[i], expected.k[i]);
            }
            ok = ok && same_float_result(actual.hesse_val,
                                         expected.hesse_val);

            double expected_dot3 = (double)lhs.k[2] * (double)rhs.k[2] +
                                   (double)lhs.k[1] * (double)rhs.k[1] +
                                   (double)lhs.k[0] * (double)rhs.k[0];
            double actual_dot = ivp_scr_f_dot3(&lhs, &rhs);
            ok = ok && same_real_result((D)actual_dot, (D)expected_dot3);

            /* IVP_U_Point_4::dot_product4: double products summed w, z, y, x */
            double expected_dot4 = (double)lhs.hesse_val * (double)rhs.hesse_val +
                                   (double)lhs.k[2] * (double)rhs.k[2] +
                                   (double)lhs.k[1] * (double)rhs.k[1] +
                                   (double)lhs.k[0] * (double)rhs.k[0];
            actual_dot = ivp_scr_f_dot4(&lhs, &rhs);
            ok = ok && same_real_result((D)actual_dot, (D)expected_dot4);

            ivp_vec3f_t vector = {{lhs.k[0], lhs.k[1], lhs.k[2]}};
            actual_dot = ivp_scr_f_dot_vp(&vector, &rhs);
            ok = ok && same_real_result((D)actual_dot, (D)expected_dot3);
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
#if IVP_X87_ARITHMETIC
            ivp_vec3f_t expected;
            ivp_core_get_surface_speed_on_test(&core, &point, &center, &rotation, &expected);
            const int first_alias = 1;
#else
            const int first_alias = 0;
#endif
            for (int alias = first_alias; ok && alias < 4; alias++) {
                ivp_vec3f_t p = point, c = center, r = rotation, actual;
#if !IVP_X87_ARITHMETIC
                ivp_vec3f_t cross, transformed, expected;
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
#endif
                ivp_vec3f_t *out = alias == 0 ? &actual : alias == 1 ? &c : alias == 2 ? &p : &r;
                ivp_core_get_surface_speed_on_test(&core, &p, &c, &r, out);
                ok = ok && same_vec_result(out, &expected);
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
#if IVP_X87_ARITHMETIC
    ivp_core_t core = {0};
    for (int axis = 0; axis < 3; ++axis)
        core.m_world_f_core_last_psi.rot.rows[axis].k[axis] = (D)1;
    /* (1 + 2^-23)^2 - (1 + 2^-22) leaves a nonzero cross product
     * only when the products are kept wide through the subtraction. */
    const ivp_vec3f_t rotation = {{0.0f, 0x1.000002p0f, 1.0f}};
    const ivp_vec3f_t point = {{0.0f, 0x1.000004p0f, 0x1.000002p0f}};
    const ivp_vec3f_t center = {{0.0f, 0.0f, 0.0f}};
    const ivp_vec3f_t expected = {{0x1p-46f, 0.0f, 0.0f}};
    ivp_vec3f_t velocity;
    ivp_core_get_surface_speed_on_test(&core, &point, &center, &rotation, &velocity);
    IVP_ASSERT_TRUE(same_vec_result(&velocity, &expected));
#endif
}

#ifdef IVP_REAL_DOUBLE
IVP_TEST(triangle_inverse_length_preserves_binary64_precision)
{
#if defined(_MSC_VER) && defined(_M_IX86)
    unsigned int saved_control, current_control;
    IVP_ASSERT_EQ(_controlfp_s(&saved_control, 0, 0), 0);
    IVP_ASSERT_EQ(_controlfp_s(&current_control, _PC_24, _MCW_PC), 0);
#endif
    const volatile double area[] = {-0.22909351823696511, 3.184356229850664, -0.14557008071278688};
    const ivp_vec3r_t normal = {{area[0], area[1], area[2]}};
    const float inverse_length = (float)(1.0 / lvp_real_length(&normal));
#if defined(_MSC_VER) && defined(_M_IX86)
    IVP_ASSERT_EQ(_controlfp_s(&current_control, saved_control, _MCW_PC), 0);
#endif
    IVP_ASSERT_EQ(inverse_length, 0.31290057301521301f);
}
#endif

#if IVP_X87_ARITHMETIC && defined(IVP_REAL_DOUBLE)
IVP_TEST(x87_collision_speed_bound_retains_both_surface_speeds)
{
    IVP_ASSERT_EQ(ivp_x87_collision_speed_bound(0.125f, 0.125f, 1.0f, 0x1p-24f), 0x1.400001p+0);
}

IVP_TEST(x87_vertex_walk_compares_before_storing_gradient)
{
    const ivp_vec3r_t offset = {{0.0, -0x1.7687b3p+3, 0.0}};
    const ivp_vec3f_t vertex = {{0.0f, 2.5f, 0.0f}};
    const ivp_vec3f_t neighbor = {{0.0f, -2.5f, 0.0f}};
    const double projection = ivp_x87_vertex_walk_projection(&offset, &vertex);
    double gradient;
    IVP_ASSERT_TRUE(ivp_x87_better_vertex_walk_gradient(&offset, &vertex, &neighbor,
                                                        projection, 0.0, &gradient));
    IVP_ASSERT_EQ(gradient, 0x1.7687b2ffffffdp+3);
    IVP_ASSERT_TRUE(ivp_x87_better_vertex_walk_gradient(&offset, &vertex, &neighbor,
                                                        projection, gradient, &gradient));
    IVP_ASSERT_EQ(gradient, 0x1.7687b2ffffffdp+3);
}

IVP_TEST(x87_edge_face_projection_retains_near_parallel_component)
{
    const ivp_mat3r_t edge_rotation = {{{{1.0, 0x1p-60, -1.0}},
                                        {{0.0, 1.0, 0.0}},
                                        {{0.0, 0.0, 1.0}}}};
    const ivp_vec3r_t edge = {{1.0, 1.0, 1.0}};
    const ivp_vec3r_t normal = {{1.0, 0.0, 0.0}};
    ivp_transform_r_t face;
    ivp_transform_identity(&face);
    IVP_ASSERT_EQ(ivp_x87_edge_face_projection(&edge_rotation, &edge, &face.rot, &normal), 0x1p-60);
}

IVP_TEST(x87_spring_impulse_preserves_elastic_residual_after_damping)
{
    const ivp_vec3f_t direction = {{1.0f, 0.0f, 0.0f}};
    const ivp_vec3f_t relative = {{0x1.000002p+0f, 0.0f, 0.0f}};
    ivp_vec3f_t impulse;
    const double force = ivp_x87_spring_impulse(&direction, &relative, 0x1.000002p+0f, 0x1p-25f,
                                               1.0f, 1.0f, 0.0f, 1.0, 0.0, &impulse);
    IVP_ASSERT_EQ(force, -0x1p-25);
    IVP_ASSERT_EQ(impulse.k[0], -0x1p-25f);
    const double clamped = ivp_x87_spring_impulse(&direction, &relative, 0x1.000002p+0f, 0x1p-25f,
                                                 1.0f, 1.0f, 0.0f, 0.5, 0x1p-26, &impulse);
    IVP_ASSERT_EQ(clamped, -0x1p-26);
    IVP_ASSERT_EQ(impulse.k[0], -0x1p-27f);
}

IVP_TEST(x87_world_impulse_preserves_torque_under_cancellation)
{
    ivp_transform_r_t transform;
    ivp_transform_identity(&transform);
    const ivp_vec3r_t point = {{0x1.000002p+0, 1.0, 0.0}};
    const ivp_vec3f_t impulse = {{1.0f, 0x1.fffffep-1f, 0.0f}};
    const ivp_vec3f_t inverse_inertia = {{1.0f, 1.0f, 2.0f}};
    ivp_vec3f_t linear = {{0.0f, 0.0f, 0.0f}};
    ivp_vec3f_t angular = {{0.0f, 0.0f, -0x1p-23f}};
    ivp_x87_accumulate_world_impulse(&transform, &point, &impulse, 0.5f, &inverse_inertia, &linear, &angular);
    IVP_ASSERT_EQ(angular.k[2], -0x1p-46f);
    IVP_ASSERT_EQ(linear.k[0], 0.5f);
}

IVP_TEST(x87_inverse_point_transform_preserves_anchor_offset)
{
    const double rotation[3][3] = {
        {-0x1.fffffffffffdcp-1, 0.0, 0x1.777a5bffffff3p-24},
        {0.0, 1.0, 0.0},
        {-0x1.777a5bffffff3p-24, 0.0, -0x1.fffffffffffdcp-1}
    };
    const double origin[3] = {0x1.6de49ap+9, -0x1.ac6252p+3, -0x1.b4ded2p+9};
    const double point[3] = {0x1.6cd3aap+9, -0x1.a45292p+3, -0x1.b0e044p+9};
    double local[3];
    ivp_x87_inverse_transform_point3d(rotation, origin, point, local);
    IVP_ASSERT_EQ(local[0], 0x1.10effa24353dap+1);
    IVP_ASSERT_EQ(local[1], 0x1.01f8p-2);
    IVP_ASSERT_EQ(local[2], -0x1.ff4700c829000p+2);
}

IVP_TEST(x87_edge_edge_separation_retains_small_gaps)
{
    ivp_transform_r_t transform;
    ivp_transform_identity(&transform);
    const ivp_vec3r_t first_edge[2] = {{{1.0, 0.0, 1.0}}, {{1.0, 0.0, -1.0}}};
    const ivp_vec3r_t second_edge[2] = {{{0x1.0000000000001p+0, 0.0, 1.0}}, {{0.0, 1.0, 0.0}}};
    const uint16_t precision = ivp_x87_enter_step();
    const double gap = ivp_x87_edge_edge_separation(&transform, first_edge, &transform, second_edge, 1.0);
    ivp_x87_leave_step(precision);
    IVP_ASSERT_EQ(gap, -0x1p-52 * ivp_isqrt_double(2.0, 2));
}

IVP_TEST(x87_point_plane_separation_retains_small_gaps)
{
    ivp_transform_r_t transform;
    ivp_transform_identity(&transform);
    transform.pos.k[2] = 1.0;
    const ivp_vec3r_t origin = {{0.0, 0.0, 0.0}};
    const ivp_vec3r_t plane_point = {{0.0, 0.0, 0x1p-54}};
    const ivp_vec3r_t normal = {{0.0, 0.0, 1.0}};
    const uint16_t precision = ivp_x87_enter_step();
    const double gap = ivp_x87_point_plane_separation(&transform, &origin, &transform, &plane_point, &normal);
    ivp_x87_leave_step(precision);
    IVP_ASSERT_EQ(gap, -0x1p-54);
}

IVP_TEST(x87_position_advance_retains_precision_to_writeback)
{
    ivp_vec3r_t position = {{0.0, 0.0, -0x1.2fe2ea0000fd7p+5}};
    const ivp_vec3r_t velocity = {{0.0, 0.0, -0x1.c92dfep-6}};
    const double elapsed = 0x1.a4689a6970000p+6 - 0x1.a467055980000p+6;
    const uint16_t precision = ivp_x87_enter_step();
    ivp_x87_advance_position(&position, &velocity, elapsed, &position);
    ivp_x87_leave_step(precision);
    IVP_ASSERT_EQ(position.k[2], -0x1.2fe3009b12c68p+5);
}

IVP_TEST(point_face_distance_preserves_projection_difference)
{
    const ivp_vec3r_t normal = {{0x1.0000002p0, 1.0, 0.0}};
    const ivp_vec3r_t point = {{0x1.ffffffcp-1, 0x1p-27, 0.0}};
    const ivp_vec3f_t face_point = {{1.0f, 0.0f, 0.0f}};
    const uint16_t precision = ivp_x87_enter_step();
    const float distance = ivp_x87_point_face_distance(&normal, &point, &face_point);
    ivp_x87_leave_step(precision);
    IVP_ASSERT_EQ(distance, -0x1p-54f);
}
#endif

#if IVP_X87_ARITHMETIC
IVP_TEST(x87_gyroscopic_update_retains_inertia_difference_precision)
{
    const ivp_vec3f_t inertia = {{0x1.a7c4c4p-2f, 0x1.c6ba5cp-1f, 0x1.93e218p+0f}};
    const ivp_vec3f_t inverse_inertia = {{0x1.354d06p+1f, 0x1.203e20p+0f, 0x1.44878cp-1f}};
    ivp_vec3f_t angular_velocity = {{0x1.5e37a4p+1f, 0x1.95482ap+1f, 0x1.e370d4p+1f}};
    double coefficients[3];
    const uint16_t precision = ivp_x87_enter_step();
    ivp_x87_gyroscopic_coefficients(&inertia, &inverse_inertia, coefficients);
    ivp_x87_add_gyroscopic_velocity(&angular_velocity, coefficients, 0x1.f07c1f07c1f08p-7);
    ivp_x87_leave_step(precision);
#ifdef IVP_REAL_DOUBLE
    IVP_ASSERT_EQ(coefficients[0], -0x1.aa8b151efef80p+0);
    IVP_ASSERT_EQ(angular_velocity.k[0], 0x1.379310p+1f);
#else
    IVP_ASSERT_EQ(coefficients[0], -0x1.aa8b16p+0);
    IVP_ASSERT_EQ(angular_velocity.k[0], 0x1.37930ep+1f);
#endif
    IVP_ASSERT_EQ(angular_velocity.k[1], 0x1.af8b5cp+1f);
    IVP_ASSERT_EQ(angular_velocity.k[2], 0x1.de63c2p+1f);
}

IVP_TEST(minimum_rotational_inertia_retains_squared_length_precision)
{
    const ivp_vec3f_t inertia = {{0x1.041426p+7f, 0x1.0412e0p+7f, 0x1.a6ee24p-1f}};

    const uint16_t precision = ivp_x87_enter_step();
    const float minimum = ivp_x87_minimum_rotational_inertia(&inertia, 0x1.eb851ep-6f);
    ivp_x87_leave_step(precision);

    IVP_ASSERT_EQ(minimum, 0x1.611840p+2f);
}

IVP_TEST(worst_case_inverse_mass_rounds_after_the_second_reciprocal)
{
    const ivp_vec3f_t point = {{0.0f, 1.0f, 0.0f}};
    const ivp_vec3f_t inertia = {{0x1.d99702p+1f, 0.0f, 0.0f}};

    const uint16_t precision = ivp_x87_enter_step();
    const float inverse = ivp_x87_worst_case_inverse_mass(&point, &inertia, 0x1p-23f, 1e-19);
    ivp_x87_leave_step(precision);

    IVP_ASSERT_EQ(inverse, 0x1.d99702p+1f);
}

IVP_TEST(x87_constraint_rotation_error_preserves_small_axes)
{
    const double rotation[3][3] = {
        {0x1.0000000000001p+0, 0x1.2ea8p-52, 0x1.0268c21c992p-37},
        {-0x1.7b6p-53, 0x1.0p+0, -0x1.efa1701a885p-36},
        {-0x1.0268c48ap-37, 0x1.efa1625cabdp-36, 0x1.0000000000001p+0},
    };
    double error[3];
    const uint16_t precision = ivp_x87_enter_step();
    ivp_x87_constraint_rotation_error(rotation, error);
    ivp_x87_leave_step(precision);
    IVP_ASSERT_EQ(error[0], 0x1.efa16ap-36);
    IVP_ASSERT_EQ(error[1], 0x1.0268c3534c9p-37);
    IVP_ASSERT_EQ(error[2], -0x1.ec58p-53);
}

IVP_TEST(x87_constraint_back_substitution_preserves_small_impulses)
{
    double matrix[6][6] = {
        {-0x1.8072820000000p+1, 0x1.66c7e20000000p+1, -0x1.6105460000000p-4, 0.0, 0x1.a00c1c0000000p-4},
        {0x1.66c7e00000000p+1, -0x1.3f119a0000000p+2, -0x1.f511c40000000p-5, -0x1.a18ebc0000000p-4, 0.0},
        {-0x1.6105460000000p-4, -0x1.f511c40000000p-5, -0x1.8524160000000p+0, -0x1.2897940000000p-2, 0x1.a1eb2c0000000p-2},
        {0.0, -0x1.a18ebc0000000p-4, -0x1.2897940000000p-2, -0x1.ee51e20000000p-2, 0.0},
        {0x1.a00c1c0000000p-4, 0.0, 0x1.a1eb2c0000000p-2, 0.0, -0x1.ec882e0000000p-2},
    };
    double desired[6] = {0x1.d228a67790856p-7, -0x1.476c2ab6c9c8bp-2, 0x1.60c5c0ae8983bp-33, 0x1.27d5edf00e7e7p-30, 0x1.7ab636d57aaeep-34};
    const double expected[5] = {0x1.e6deb34301469p-4, 0x1.0d6196a898323p-3, 0x1.1e2e9931bd447p-33, -0x1.c71945d50a37fp-6, 0x1.9b43b7a96e7e7p-6};
    double result[6];
    const uint16_t precision = ivp_x87_enter_step();
    const bool solved = ivp_x87_solve_constraint_system(5, matrix, desired, result, (double)1e-9f);
    ivp_x87_leave_step(precision);
    IVP_ASSERT_TRUE(solved);
    for (int i = 0; i < 5; ++i) {
        IVP_ASSERT_EQ(result[i], expected[i]);
    }
}

IVP_TEST(x87_hull_gauss_offset_rounds_after_each_subtraction)
{
    const double point[] = {1.0, 1.0 + 0x1p-31, 1.0};
    const double normal[] = {-1.0, 1.0 - 0x1p-31, 0.0};
    IVP_ASSERT_EQ(ivp_x87_hull_plane_gauss_offset(point, normal, 3), 0x1p-62);

    const double unit[] = {1.0, 1.0, 1.0};
    const double cancellation[] = {-1.0, -0x1p-55, 1.0};
    IVP_ASSERT_EQ(ivp_x87_hull_plane_gauss_offset(unit, cancellation, 3), 0.0);
}

IVP_TEST(x87_hull_plane_checks_vertices_before_rounding_the_offset)
{
    double origin[3] = {-0x1.4000e1fffd200p+1, 0x1.4000e1fffd200p+1, -0x1.4000ee0005c00p+1};
    double second[3] = {0x1.4000ee0005c00p+1, 0x1.4000e1fffd200p+1, -0x1.4000ee0005c00p+1};
    double third[3] = {-0x1.4000e60000000p+1, -0x1.4000e60000000p+1, -0x1.4000e60000000p+1};
    double *vertices[3] = {origin, second, third};
    const double normal[3] = {0.0, -0x1.99999c28f5be7p-23, -0x1.fffffffffff5cp-1};
    double offset;
    const uint16_t precision = ivp_x87_enter_step();
    const bool degenerate = ivp_x87_set_hull_plane_offset(vertices, origin, normal, 1e-18, &offset);
    ivp_x87_leave_step(precision);
    IVP_ASSERT_FALSE(degenerate);
    IVP_ASSERT_EQ(offset, -0x1.4000ea0002e00p+1);
}

IVP_TEST(hull_simplex_compares_before_rounding_the_determinant)
{
    double first[2] = {1.0, -0x1p-54};
    double second[2] = {1.0, 1.0};
    double *rows[2] = {first, second};
    const uint16_t precision = ivp_x87_enter_step();
    volatile double rounded = ivp_x87_hull_determinant(rows, 2);
    const int exceeds = ivp_x87_hull_determinant_exceeds(rows, 2, rounded);
    ivp_x87_leave_step(precision);
    IVP_ASSERT_EQ(rounded, 1.0);
    IVP_ASSERT_TRUE(exceeds);
}

IVP_TEST(inverse_rotation_retains_small_axis_components)
{
    const double rotation[3][3] = {
        {-0x1.f838b8def7c8ap-1, 0x1.63a1a5d9422c5p-3, 0x1.777a5bc17ee2dp-24},
        {0x1.63a1a5d9422ddp-3, 0x1.f838b8def7cabp-1, 0x1.028f334p-52},
        {-0x1.71c60b453070fp-24, 0x1.04cde2786eafap-26, -0x1.fffffffffffdep-1},
    };
    const double world[3] = {0x1.777a5cp-24, 0.0, -1.0};
    double local[3];
    const uint16_t precision = ivp_x87_enter_step();
    ivp_x87_inverse_rotate3d(rotation, world, local);
    ivp_x87_leave_step(precision);
    IVP_ASSERT_EQ(local[0], -0x1.01718e721p-50);
    IVP_ASSERT_EQ(local[1], -0x1.43e265cf4p-54);
    IVP_ASSERT_EQ(local[2], 0x1.0000000000011p+0);
}

IVP_TEST(linear_damping_rounds_after_multiplying_velocity)
{
    ivp_vec3f_t speed = {{-0x1.49048cp-1f, 0.0f, 0.0f}};
    ivp_vec3f_t angular = {{0.0f, 0.0f, 0.0f}};
    const ivp_vec3f_t angular_damping = {{0.1f, 0.1f, 0.1f}};
    const uint16_t precision = ivp_x87_enter_step();
    ivp_x87_damp_velocity(&speed, &angular, &angular_damping, 1.5, 0x1.f07c1f07c1f08p-7);
    ivp_x87_leave_step(precision);
    IVP_ASSERT_EQ(speed.k[0], -0x1.418a42p-1f);
}

IVP_TEST(slow_core_preserves_linear_damping_coefficient)
{
    ivp_core_t core = {0};
    core.movement_state = IVP_MT_SLOW;
    core.speed_damp_factor = 0.6f;
    core.speed.k[2] = 0x1.d32ed8p-6f;
    const uint16_t precision = ivp_x87_enter_step();
    ivp_core_global_damp_core(&core, 0x1.f07c1f07c1f08p-7);
    ivp_x87_leave_step(precision);
    IVP_ASSERT_EQ(core.speed.k[2], 0x1.ce3a60p-6f);
}

#endif

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
#ifdef IVP_REAL_DOUBLE
    IVP_REGISTER(triangle_inverse_length_preserves_binary64_precision);
#endif
#if IVP_X87_ARITHMETIC && defined(IVP_REAL_DOUBLE)
    IVP_REGISTER(x87_position_advance_retains_precision_to_writeback);
    IVP_REGISTER(x87_point_plane_separation_retains_small_gaps);
    IVP_REGISTER(x87_collision_speed_bound_retains_both_surface_speeds);
    IVP_REGISTER(x87_vertex_walk_compares_before_storing_gradient);
    IVP_REGISTER(x87_edge_face_projection_retains_near_parallel_component);
    IVP_REGISTER(x87_spring_impulse_preserves_elastic_residual_after_damping);
    IVP_REGISTER(x87_world_impulse_preserves_torque_under_cancellation);
    IVP_REGISTER(x87_inverse_point_transform_preserves_anchor_offset);
    IVP_REGISTER(x87_edge_edge_separation_retains_small_gaps);
    IVP_REGISTER(point_face_distance_preserves_projection_difference);
#endif
#if IVP_X87_ARITHMETIC
    IVP_REGISTER(x87_gyroscopic_update_retains_inertia_difference_precision);
    IVP_REGISTER(minimum_rotational_inertia_retains_squared_length_precision);
    IVP_REGISTER(worst_case_inverse_mass_rounds_after_the_second_reciprocal);
    IVP_REGISTER(x87_constraint_rotation_error_preserves_small_axes);
    IVP_REGISTER(x87_constraint_back_substitution_preserves_small_impulses);
    IVP_REGISTER(x87_hull_gauss_offset_rounds_after_each_subtraction);
    IVP_REGISTER(x87_hull_plane_checks_vertices_before_rounding_the_offset);
    IVP_REGISTER(hull_simplex_compares_before_rounding_the_determinant);
    IVP_REGISTER(inverse_rotation_retains_small_axis_components);
    IVP_REGISTER(linear_damping_rounds_after_multiplying_velocity);
    IVP_REGISTER(slow_core_preserves_linear_damping_coefficient);
#endif
    IVP_RUN_ALL();
}
