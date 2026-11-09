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

/* ── Reference double-build constants (ivu_types.hxx, !IVP_NO_DOUBLE) ───── *
 * Code that reproduces an IVP_DOUBLE computation of the original engine uses
 * these, independent of IVP_REAL_DOUBLE (which only selects the storage type
 * of ivp_real_t).  P_RES_EPS = P_MAX_WORLD_DOUBLE * P_DOUBLE_RES = 1e-8. */
#define IVP_REF_DOUBLE_EPS  1e-10        /* P_DOUBLE_EPS                   */
#define IVP_REF_DOUBLE_RES  1e-12        /* P_DOUBLE_RES                   */
#define IVP_REF_DOUBLE_MAX  1e20         /* P_DOUBLE_MAX                   */
#define IVP_REF_MAX_WORLD   10000.0      /* P_MAX_WORLD_DOUBLE             */
#define IVP_REF_RES_EPS     (IVP_REF_MAX_WORLD * IVP_REF_DOUBLE_RES) /* P_RES_EPS */

/* ── Inverse square root (Newton-Raphson) ───────────────────────────────── */

/* Double isqrt — IVP_Fast_Math::isqrt(square, resolution_steps)
 * (ivu_float.hxx, non-IVP_NO_DOUBLE build): IEEE exponent bit-hack initial
 * estimate + 2 base + N extra Newton-Raphson iterations, all in double.
 * resolution_steps:
 *   0 → ~1e-3 precision (2 iterations total)
 *   1 → ~1e-7 precision (3 iterations total)
 *   2 → ~1e-14 precision (4 iterations total)
 *   3 → ~1e-16 precision (5 iterations total) */
double ivp_isqrt_double(double square, int resolution_steps);

/* Float isqrt — IVP_Inline_Math::isqrt_float(quad) (ivu_linear.cxx):
 * IVP_Fast_Math::isqrt(quad, 2), i.e. the double Newton iteration with
 * resolution 2 (~1e-14), returned unrounded like the retail build. */
double ivp_isqrt_float(float x);

/* IVP_Inline_Math::isqrt_double(quad) == IVP_Fast_Math::isqrt(quad, 3). */
double ivp_isqrt_double_inline(double x);

/* IVP_Fast_Math::sqrt(x): exact libm sqrt in the double build. */
static inline double ivp_fast_sqrt(double x) { return sqrt(x); }

/* ── IVP_Inline_Math double-precision wrappers (reference double build) ─── *
 * In the reference (non-IVP_NO_DOUBLE) build every IVP_Inline_Math trig /
 * sqrt helper takes and returns IVP_DOUBLE and calls the double libm
 * function — including the ones spelled ivp_sinf / ivp_cosf / ivp_sqrtf /
 * ivp_expf.  These wrappers are the C equivalents; they are always double,
 * independent of IVP_REAL_DOUBLE. */
static inline double ivp_fabsd(double f)   { return fabs(f); }
static inline double ivp_sqrtd(double f)   { return sqrt(f); }  /* also IVP_Inline_Math::ivp_sqrtf */
static inline double ivp_sind(double f)    { return sin(f); }   /* also IVP_Inline_Math::ivp_sinf  */
static inline double ivp_cosd(double f)    { return cos(f); }   /* also IVP_Inline_Math::ivp_cosf  */
static inline double ivp_acosd(double f)   { return acos(f); }
static inline double ivp_asind(double f)   { return asin(f); }
static inline double ivp_atand(double f)   { return atan(f); }
static inline double ivp_expd(double f)    { return exp(f); }   /* IVP_Inline_Math::ivp_expf */
static inline double ivp_atan2d(double f1, double f2) { return atan2(f1, f2); }

/* ── IVP_Inline_Math float approximations (ivu_linear_macros.hxx) ──────── *
 * All of these are IVP_FLOAT in, IVP_FLOAT out, evaluated in float. */

/* IVP_Inline_Math::clamp */
static inline float ivp_clampf(float r, float mn, float mx) {
    return ((r < mn) ? mn : ((r > mx) ? mx : r));
}

/* IVP_Inline_Math::save_acosf: acos(angle) in double, 0 for angle >= 1. */
static inline float ivp_save_acosf(float angle) {
    return angle >= 1.0f ? 0.0f : (float)acos((double)angle);
}

/* IVP_Inline_Math::int_div_2 (non-PSXII build: a / 2). */
static inline int ivp_int_div_2(int a) { return a / 2; }

/* The approximations below are implemented out of line (ivp_fast_math.c)
 * so that they are always evaluated with the library's strict FP flags
 * (no FMA contraction), independent of how the including TU is compiled. */

/* IVP_Inline_Math::fast_approx_asin: angle + x^3/6 + 3x^5/40 (always below
 * |asin|; error < 0.0005 for |x| < 0.5, 0.2 at |x| == 1). */
float ivp_fast_approx_asinf(float angle);

/* IVP_Inline_Math::approx5_sin: fifth order Taylor series of sin. */
float ivp_approx5_sinf(float angle);

/* IVP_Inline_Math::approx5_cos: fourth order Taylor series of cos. */
float ivp_approx5_cosf(float angle);

/* IVP_Inline_Math::upper_limit_asin: angle + x^3/6 + 0.40414 x^5. */
float ivp_upper_limit_asinf(float angle);

/* IVP_Inline_Math::fast_asin: angle + 0.12 x^3 + 0.29 x^5
 * (error < 0.0016 for |x| < 0.57, < 0.0521 everywhere). */
float ivp_fast_asinf(float angle);

/* IVP_Inline_Math::fast_anywhere_asin: 1.1x - 0.419x^3 + 0.76x^5
 * (error < 0.022 everywhere, average ~0.012). */
float ivp_fast_anywhere_asinf(float angle);

/* IVP_Inline_Math::invert_2x2_matrix.  Returns false (IVP_FAULT) and leaves
 * the outputs untouched when D*D < P_DOUBLE_EPS^2 (D = a1*b2 - b1*a2).
 * Note: the reference stores the inverse "transposed" as
 *   i_a1 = b2/D, i_b1 = -a2/D, i_a2 = -b1/D, i_b2 = a1/D. */
bool ivp_invert_2x2_matrix(double a1_in, double b1_in, double a2_in, double b2_in,
                           double *i_a1_out, double *i_b1_out,
                           double *i_a2_out, double *i_b2_out);

/* ── Deterministic math wrappers ────────────────────────────────────────── */

/* These wrappers use system libm. */

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
