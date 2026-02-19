/* ivp_types.h — Fundamental types for IVP C17 rewrite
 *
 * Compile-time switch:
 *   -DIVP_REAL_DOUBLE   → ivp_real_t = double
 *   (default)           → ivp_real_t = float
 */
#ifndef IVP_TYPES_H
#define IVP_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <float.h>

/* ── ivp_real_t: configurable precision ─────────────────────────────────── */

#ifdef IVP_REAL_DOUBLE
    typedef double ivp_real_t;
    #define IVP_REAL_FMT  "%.17g"
    #define IVP_REAL_EPS  1e-10
    #define IVP_REAL_RES  1e-12
    #define IVP_REAL_MAX  1e20
    #define IVP_REAL(x)   x
#else
    typedef float ivp_real_t;
    #define IVP_REAL_FMT  "%.9g"
    #define IVP_REAL_EPS  1e-10f
    #define IVP_REAL_RES  1e-6f
    #define IVP_REAL_MAX  1e16f
    #define IVP_REAL(x)   x##f
#endif

/* ── Fixed-precision constants (matching original IVP) ──────────────────── */

#define IVP_FLOAT_EPS       1e-10f
#define IVP_FLOAT_RES       1e-6f
#define IVP_FLOAT_MAX       1e16f

#ifdef IVP_REAL_DOUBLE
    #define IVP_DOUBLE_EPS  1e-10
    #define IVP_DOUBLE_RES  1e-12
    #define IVP_DOUBLE_MAX  1e20
    #define IVP_MAX_WORLD   10000.0
#else
    #define IVP_DOUBLE_EPS  IVP_FLOAT_EPS
    #define IVP_DOUBLE_RES  IVP_FLOAT_RES
    #define IVP_DOUBLE_MAX  IVP_FLOAT_MAX
    #define IVP_MAX_WORLD   3000.0f
#endif

/* Precision-correct aliases: always match ivp_real_t */
#define IVP_REAL_PRECISION_EPS  IVP_DOUBLE_EPS
#define IVP_REAL_PRECISION_RES  IVP_DOUBLE_RES
#define IVP_REAL_PRECISION_MAX  IVP_DOUBLE_MAX

#define IVP_MAX_OBJECT_SIZE  1000.0f
#define IVP_MIN_EDGE_LEN     0.01f
#define IVP_RES_EPS          (IVP_MAX_WORLD * IVP_DOUBLE_RES)

/* ── Math constants ─────────────────────────────────────────────────────── */

#ifdef IVP_REAL_DOUBLE
    #define IVP_PI     3.14159265358979323846
    #define IVP_PI_2   1.57079632679489661923
#else
    #define IVP_PI     3.14159265358979323846f
    #define IVP_PI_2   1.57079632679489661923f
#endif

/* Double-precision pi for code that always operates in double. */
#define IVP_PI_D   3.14159265358979323846

/* ── 3D solver thresholds (precision-dependent, from original IVP) ──────── */

#ifdef IVP_REAL_DOUBLE
    #define IVP_3D_SOLVER_NULLSTELLE_EPS 1e-8
#else
    #define IVP_3D_SOLVER_NULLSTELLE_EPS 3e-3f
#endif

/* ── Coordinate index ───────────────────────────────────────────────────── */

typedef enum ivp_coord_index {
    IVP_INDEX_X = 0,
    IVP_INDEX_Y = 1,
    IVP_INDEX_Z = 2
} ivp_coord_index_t;

/* ── Alignment helpers ──────────────────────────────────────────────────── */

#if defined(_MSC_VER)
    #define IVP_ALIGN(n) __declspec(align(n))
#elif defined(__GNUC__) || defined(__clang__)
    #define IVP_ALIGN(n) __attribute__((aligned(n)))
#else
    #define IVP_ALIGN(n)
#endif

#define IVP_ALIGN16 IVP_ALIGN(16)

/* ── Inline hint ────────────────────────────────────────────────────────── */

#if defined(_MSC_VER)
    #define IVP_FORCEINLINE static __forceinline
#elif defined(__GNUC__) || defined(__clang__)
    #define IVP_FORCEINLINE __attribute__((always_inline)) static inline
#else
    #define IVP_FORCEINLINE static inline
#endif

/* ── Unused parameter ───────────────────────────────────────────────────── */

#define IVP_UNUSED(x) ((void)(x))

#endif /* IVP_TYPES_H */
