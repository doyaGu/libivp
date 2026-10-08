/* ivp_simd_internal.h -- architecture-specific helpers for exact lane-wise
 * arithmetic.  These intrinsics only batch independent scalar expressions;
 * callers remain responsible for preserving each expression's operation and
 * rounding order.
 */
#ifndef IVP_SIMD_INTERNAL_H
#define IVP_SIMD_INTERNAL_H

#if defined(__aarch64__) && (defined(__clang__) || defined(__GNUC__))
#include <arm_neon.h>
#define IVP_ARM64_NEON 1
#else
#define IVP_ARM64_NEON 0
#endif

#endif /* IVP_SIMD_INTERNAL_H */
