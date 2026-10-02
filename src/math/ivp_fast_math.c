/* ivp_fast_math.c — Fast math implementations replicating original IVP */
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>
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

float ivp_isqrt_float(float x) {
    IVP_ASSERT(x > 0.0f);

    /* Quake-style bit hack for initial estimate */
    ivp_float_ieee_t conv;
    conv.val = x;
    conv.bits = 0x5f3759dfU - (conv.bits >> 1);
    float inv_sqrt = conv.val;

    /* 2 Newton-Raphson iterations */
    float xhalf = x * 0.5f;
    inv_sqrt = inv_sqrt * (1.5f - inv_sqrt * inv_sqrt * xhalf);
    inv_sqrt = inv_sqrt * (1.5f - inv_sqrt * inv_sqrt * xhalf);

    return inv_sqrt;
}

/* ── Double inverse square root (matching original IVP_Fast_Math::isqrt) ── */

double ivp_isqrt_double(double x, int resolution_steps) {
    if (x == 0.0) return HUGE_VAL;
    if (x < 0.0) return nan("");

    /* Bit-hack initial estimate (matching original IVP) */
    ivp_double_ieee_t h;
    ivp_double_ieee_t ie;
    ie.val = x;
    h.val = 1.0;
    h.ln2.h = ((0x7ff00000 - ie.ln2.h) >> 1) + 0x1ff00000;

    double squareh = x * 0.5;
    double inv_sqrt = h.val;

    /* 2 base Newton-Raphson iterations (always performed) */
    inv_sqrt += inv_sqrt * (0.5 - inv_sqrt * inv_sqrt * squareh);
    inv_sqrt += inv_sqrt * (0.5 - inv_sqrt * inv_sqrt * squareh);

    /* Additional iterations based on resolution_steps */
    if (resolution_steps > 0)
        inv_sqrt += inv_sqrt * (0.5 - inv_sqrt * inv_sqrt * squareh);
    if (resolution_steps > 1)
        inv_sqrt += inv_sqrt * (0.5 - inv_sqrt * inv_sqrt * squareh);
    if (resolution_steps > 2)
        inv_sqrt += inv_sqrt * (0.5 - inv_sqrt * inv_sqrt * squareh);

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
