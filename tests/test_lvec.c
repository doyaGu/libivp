/* Reference-precision private matrix helpers: preserve scalar evaluation,
 * dynamic rounding, signed zeros and in-place input/output. */
#include "ivp_test.h"
#include "collision/ivp_lvec.h"
#include <fenv.h>
#include <stdint.h>

typedef void (*transform_fn_t)(const ivp_transform_r_t *, const ivp_vec3f_t *,
                               ivp_vec3f_t *);
/* Exercise the actual helpers even when a compiler could fold a comparison
 * between identical inline implementations. */
static transform_fn_t volatile transforms[] = {lvm_vmult3_f, lvm_vimult3_f};
typedef void (*cross_fn_t)(ivp_vec3f_t *, const ivp_vec3f_t *, const ivp_vec3f_t *);
static cross_fn_t volatile cross_product = lvf_cross;

/* IVP_U_Matrix3::{inline_vmult3,inline_vimult3} from
 * ivu_linear_software.hxx. Keep repeated conversions in the oracle. */
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

/* The reference multiplies floats, subtracts in D, then rounds to float.
 * Replacing this with an all-float cross product is not the same contract. */
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

int main(void)
{
    IVP_REGISTER(float_matrix_products_match_scalar_rounding);
    IVP_REGISTER(float_matrix_products_preserve_edge_values);
    IVP_REGISTER(float_matrix_products_allow_inplace_input);
    IVP_REGISTER(float_cross_products_match_scalar_rounding_and_aliases);
    IVP_RUN_ALL();
}
