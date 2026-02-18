/* ivp_fast_math.h — Fast math approximations for IVP C17
 *
 * Replicates original IVP's Newton-Raphson inverse square root and
 * other deterministic math operations.
 */
#ifndef IVP_FAST_MATH_H
#define IVP_FAST_MATH_H

#include <ivp/base/ivp_types.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Inverse square root (Newton-Raphson) ───────────────────────────────── */

/* Float isqrt: bit-hack initial estimate + 2 NR iterations.
 * precision ~1e-3 (suitable for fast normalization). */
float ivp_isqrt_float(float x);

/* Double isqrt: bit-hack initial estimate + 2 base + N extra NR iterations.
 * resolution_steps:
 *   0 → ~1e-3 precision (2 iterations total)
 *   1 → ~1e-7 precision (3 iterations total)
 *   2 → ~1e-14 precision (4 iterations total)
 *   3 → ~1e-16 precision (5 iterations total) */
double ivp_isqrt_double(double x, int resolution_steps);

/* ── Deterministic math wrappers ────────────────────────────────────────── */

/* These wrappers use system libm by default.
 * When IVP_USE_DETERMINISTIC_LIBM is defined, they could be redirected
 * to a deterministic implementation (SLEEF, CRlibm, etc). */

static inline ivp_real_t ivp_sqrt(ivp_real_t x) {
#ifdef IVP_REAL_DOUBLE
    return sqrt(x);
#else
    return sqrtf(x);
#endif
}

static inline ivp_real_t ivp_fabs(ivp_real_t x) {
#ifdef IVP_REAL_DOUBLE
    return fabs(x);
#else
    return fabsf(x);
#endif
}

static inline ivp_real_t ivp_sin(ivp_real_t x) {
#ifdef IVP_REAL_DOUBLE
    return sin(x);
#else
    return sinf(x);
#endif
}

static inline ivp_real_t ivp_cos(ivp_real_t x) {
#ifdef IVP_REAL_DOUBLE
    return cos(x);
#else
    return cosf(x);
#endif
}

static inline ivp_real_t ivp_asin(ivp_real_t x) {
#ifdef IVP_REAL_DOUBLE
    return asin(x);
#else
    return asinf(x);
#endif
}

static inline ivp_real_t ivp_acos(ivp_real_t x) {
#ifdef IVP_REAL_DOUBLE
    return acos(x);
#else
    return acosf(x);
#endif
}

static inline ivp_real_t ivp_atan2(ivp_real_t y, ivp_real_t x) {
#ifdef IVP_REAL_DOUBLE
    return atan2(y, x);
#else
    return atan2f(y, x);
#endif
}

static inline ivp_real_t ivp_exp(ivp_real_t x) {
#ifdef IVP_REAL_DOUBLE
    return exp(x);
#else
    return expf(x);
#endif
}

/* ── Min / Max / Clamp ──────────────────────────────────────────────────── */

static inline ivp_real_t ivp_min(ivp_real_t a, ivp_real_t b) {
    return a < b ? a : b;
}

static inline ivp_real_t ivp_max(ivp_real_t a, ivp_real_t b) {
    return a > b ? a : b;
}

static inline ivp_real_t ivp_clamp(ivp_real_t x, ivp_real_t lo, ivp_real_t hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

/* ── Log2 via IEEE bit extraction (matching original PFM_LD) ────────────── */

/* Returns floor(log2(|x|)) via IEEE exponent extraction. */
int ivp_ilog2_float(float x);
int ivp_ilog2_double(double x);

#ifdef __cplusplus
}
#endif

#endif /* IVP_FAST_MATH_H */
