/* ivp_fast_math.c — Fast math implementations replicating original IVP */
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ── IEEE double bit union (matching original p_double_ieee) ────────────── */

typedef union {
    double val;
    struct {
        int l;
        int h;
    } ln2;
} ivp_double_ieee_t;

/* ── IEEE float bit union ───────────────────────────────────────────────── */

typedef union {
    float    val;
    uint32_t bits;
} ivp_float_ieee_t;

/* ── Float inverse square root ──────────────────────────────────────────── */

/* Reference: IVP_DOUBLE IVP_Inline_Math::isqrt_float(IVP_FLOAT quad)
 *            { return IVP_Fast_Math::isqrt(quad, 2); }   (ivu_linear.cxx)
 * The retail build returns the unrounded value; callers that store it in a
 * float narrow it themselves. */
double ivp_isqrt_float(float x) {
    return ivp_isqrt_double((double)x, 2);
}

/* Reference: IVP_DOUBLE IVP_Inline_Math::isqrt_double(IVP_DOUBLE quad)
 *            { return IVP_Fast_Math::isqrt(quad, 3); }   (ivu_linear.cxx) */
double ivp_isqrt_double_inline(double x) {
    return ivp_isqrt_double(x, 3);
}

/* ── IVP_Inline_Math approximations (ivu_linear_macros.hxx) ─────────────── */

/* IVP_Inline_Math::fast_approx_asin: angle + x^3/6 + 3x^5/40 (always below
 * |asin|; error < 0.0005 for |x| < 0.5, 0.2 at |x| == 1). */
float ivp_fast_approx_asinf(float angle) {
    float quad_angle = angle * angle;
    float triple_angle = angle * quad_angle;
    float quint_angle = triple_angle * quad_angle;
    return angle + (1.0f / 6.0f) * triple_angle + (3.0f / 40.0f) * quint_angle;
}

/* IVP_Inline_Math::approx5_sin: fifth order Taylor series of sin. */
float ivp_approx5_sinf(float angle) {
    float quad_angle = angle * angle;
    float triple_angle = angle * quad_angle;
    float quint_angle = triple_angle * quad_angle;
    return angle - (1.0f / 6.0f) * triple_angle + (1.0f / 120.0f) * quint_angle;
}

/* IVP_Inline_Math::approx5_cos: fourth order Taylor series of cos. */
float ivp_approx5_cosf(float angle) {
    float quad_angle = angle * angle;
    return 1.0f - (1.0f / 2.0f) * quad_angle + (1.0f / 24.0f) * quad_angle * quad_angle;
}

/* IVP_Inline_Math::upper_limit_asin: angle + x^3/6 + 0.40414 x^5. */
float ivp_upper_limit_asinf(float angle) {
    float quad_angle = angle * angle;
    float triple_angle = angle * quad_angle;
    float quint_angle = triple_angle * quad_angle;
    return angle + (1.0f / 6.0f) * triple_angle + 0.40414f * quint_angle;
}

/* IVP_Inline_Math::fast_asin: angle + 0.12 x^3 + 0.29 x^5
 * (error < 0.0016 for |x| < 0.57, < 0.0521 everywhere). */
float ivp_fast_asinf(float angle) {
    float quad_angle = angle * angle;
    float triple_angle = angle * quad_angle;
    float quint_angle = triple_angle * quad_angle;
    return angle + 0.12f * triple_angle + 0.29f * quint_angle;
}

/* IVP_Inline_Math::fast_anywhere_asin: 1.1x - 0.419x^3 + 0.76x^5
 * (error < 0.022 everywhere, average ~0.012). */
float ivp_fast_anywhere_asinf(float angle) {
    float quad_angle = angle * angle;
    float triple_angle = angle * quad_angle;
    float quint_angle = triple_angle * quad_angle;
    return 1.1f * angle - 0.419f * triple_angle + 0.76f * quint_angle;
}

/* IVP_Inline_Math::invert_2x2_matrix.  Returns false (IVP_FAULT) and leaves
 * the outputs untouched when D*D < P_DOUBLE_EPS^2 (D = a1*b2 - b1*a2).
 * Note: the reference stores the inverse "transposed" as
 *   i_a1 = b2/D, i_b1 = -a2/D, i_a2 = -b1/D, i_b2 = a1/D. */
bool ivp_invert_2x2_matrix(double a1_in, double b1_in,
                                         double a2_in, double b2_in,
                                         double *i_a1_out, double *i_b1_out,
                                         double *i_a2_out, double *i_b2_out) {
    double D = (a1_in * b2_in) - (b1_in * a2_in);
    if (D * D < IVP_REF_DOUBLE_EPS * IVP_REF_DOUBLE_EPS) {
        return false;
    }
    double DI = 1.0 / D;
    i_a1_out[0] = b2_in * DI;
    i_b1_out[0] = -a2_in * DI;
    i_a2_out[0] = -b1_in * DI;
    i_b2_out[0] = a1_in * DI;
    return true;
}

/* ── Double inverse square root (matching original IVP_Fast_Math::isqrt) ── */

/* No domain guard: like the reference, square > 0 is only asserted (debug
 * builds).  Its release results (0 -> NaN, negative -> exponent wrap) are
 * reproduced. */
double ivp_isqrt_double(double square, int resolution_steps) {
    IVP_ASSERT(ivp_fabsd(square) > (double)0.0f);
    /* Bit-hack initial estimate (matching original IVP); the int
     * subtraction is done on unsigned to stay defined for any input. */
    ivp_double_ieee_t h;
    ivp_double_ieee_t ie;
    ie.val = square;
    h.val = 1.0;
    int32_t diff = (int32_t)(0x7ff00000u - (uint32_t)ie.ln2.h);
    h.ln2.h = (int)((uint32_t)(diff >> 1) + 0x1ff00000u);

    double squareh = square * 0.5;
    double inv_sqrt = h.val;

    /* 2 base Newton-Raphson iterations (always performed) */
    inv_sqrt += inv_sqrt * (0.5 - inv_sqrt * inv_sqrt * squareh);
    inv_sqrt += inv_sqrt * (0.5 - inv_sqrt * inv_sqrt * squareh);

    /* Additional iterations based on resolution_steps */
    if (resolution_steps > 0)
        inv_sqrt += inv_sqrt * (0.5 - (inv_sqrt * inv_sqrt * squareh));
    if (resolution_steps > 1)
        inv_sqrt += inv_sqrt * (0.5 - (inv_sqrt * inv_sqrt * squareh));
    if (resolution_steps > 2)
        inv_sqrt += inv_sqrt * (0.5 - (inv_sqrt * inv_sqrt * squareh));

    IVP_ASSERT(ivp_fabsd((double)1.0f - inv_sqrt * inv_sqrt * square) < (double)0.001f);
    return inv_sqrt;
}

/* ── IEEE exponent extraction (matching original PFM_LD) ────────────────── */

int ivp_ilog2_float(float x) {
    ivp_float_ieee_t conv;
    conv.val = fabsf(x);
    int biased_exp = (int)((conv.bits >> 23) & 0xFF);
    return biased_exp - 127;
}

int ivp_ilog2_double(double x) {
    ivp_double_ieee_t conv;
    conv.val = fabs(x);
    int biased_exp = (conv.ln2.h >> 20) & 0x7FF;
    return biased_exp - 1023;
}
