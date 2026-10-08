/* ivp_lvec.h — reference-precision vector helpers for the core pipeline port
 *
 * Mirrors IVP_U_Float_Point (float storage, ivu_linear_software.hxx) and
 * IVP_U_Point (IVP_DOUBLE storage, ivu_linear_double.hxx) plus the
 * IVP_U_Matrix / IVP_U_Matrix3 vector products, preserving where the
 * reference rounds to float.  "D" == IVP_DOUBLE == ivp_real_t.
 *
 *   lvf_*  operate on ivp_vec3f_t  (IVP_U_Float_Point)
 *   lvp_*  operate on ivp_vec3r_t  (IVP_U_Point)
 *   lvm_*  operate on ivp_transform_r_t (IVP_U_Matrix: rot + vv)
 */
#ifndef IVP_LVEC_H
#define IVP_LVEC_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_legacy.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_fast_math.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "base/ivp_simd_internal.h"

typedef ivp_real_t D;   /* IVP_DOUBLE */

#ifdef IVP_REAL_DOUBLE
#define LV_SQRT(x) sqrt(x)
#else
#define LV_SQRT(x) sqrtf(x)
#endif

/* IVP_Fast_Math::isqrt(square, resolution_steps) of the double reference
 * build, bit for bit, including its unguarded release behaviour for
 * square == 0 (NaN) and negative input (32-bit wrap). Inline twin of
 * ivp_isqrt_double. */
static inline double lv_ref_isqrt(double square, int resolution_steps) {
    uint64_t bits;
    memcpy(&bits, &square, sizeof(bits));
    uint32_t ie_h = (uint32_t)(bits >> 32);
    /* ((0x07ff00000 - ie.ln2.h) >> 1) + 0x1ff00000 on int (ln2.h is int) */
    int32_t diff = (int32_t)(0x7ff00000u - ie_h);
    uint32_t h_h = (uint32_t)(diff >> 1) + 0x1ff00000u;
    uint64_t hbits = (uint64_t)h_h << 32;   /* h.val = 1.0 -> low word 0 */
    double inv_sqrt;
    memcpy(&inv_sqrt, &hbits, sizeof(inv_sqrt));
    double squareh = square * 0.5f;
    inv_sqrt += inv_sqrt * (0.5f - inv_sqrt * inv_sqrt * squareh);
    inv_sqrt += inv_sqrt * (0.5f - inv_sqrt * inv_sqrt * squareh);
    if (resolution_steps > 0) inv_sqrt += inv_sqrt * (0.5f - (inv_sqrt * inv_sqrt * squareh));
    if (resolution_steps > 1) inv_sqrt += inv_sqrt * (0.5f - (inv_sqrt * inv_sqrt * squareh));
    if (resolution_steps > 2) inv_sqrt += inv_sqrt * (0.5f - (inv_sqrt * inv_sqrt * squareh));
    return inv_sqrt;
}

/* IVP_Fast_Math::isqrt(x, n) (computed in double like the reference) */
static inline D lv_isqrt(D x, int n) {
    return (D)lv_ref_isqrt((double)x, n);
}
/* IVP_Inline_Math::isqrt_float == IVP_Fast_Math::isqrt(quad, 2) */
static inline float lv_isqrt_float(float x) {
    return (float)lv_ref_isqrt((double)x, 2);
}
/* IVP_Inline_Math::isqrt_double == IVP_Fast_Math::isqrt(quad, 3) */
static inline D lv_isqrt_double(D x) {
    return (D)lv_ref_isqrt((double)x, 3);
}

/* ── IVP_U_Float_Point ──────────────────────────────────────────────────── */

static inline void lvf_zero(ivp_vec3f_t *a) { a->k[0] = 0.0f; a->k[1] = 0.0f; a->k[2] = 0.0f; }
static inline void lvf_set3(ivp_vec3f_t *a, float x, float y, float z) { a->k[0] = x; a->k[1] = y; a->k[2] = z; }
static inline void lvf_setd3(ivp_vec3f_t *a, D x, D y, D z) { a->k[0] = (float)x; a->k[1] = (float)y; a->k[2] = (float)z; }
static inline void lvf_set(ivp_vec3f_t *a, const ivp_vec3f_t *b) { *a = *b; }
static inline void lvf_set_p(ivp_vec3f_t *a, const ivp_vec3r_t *b) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    vst1_f32(a->k, vcvt_f32_f64(vld1q_f64(b->k)));
    a->k[2] = (float)b->k[2];
#else
    a->k[0] = (float)b->k[0]; a->k[1] = (float)b->k[1]; a->k[2] = (float)b->k[2];
#endif
}
static inline D lvf_quad_length(const ivp_vec3f_t *a) {
#if IVP_ARM64_NEON
    float32x2_t products = vmul_f32(vld1_f32(a->k), vld1_f32(a->k));
    float sum = vget_lane_f32(products, 0) + vget_lane_f32(products, 1);
    sum += a->k[2] * a->k[2];
    return (D)sum;
#else
    return (D)(a->k[0] * a->k[0] + a->k[1] * a->k[1] + a->k[2] * a->k[2]);
#endif
}
static inline D lvf_dot(const ivp_vec3f_t *a, const ivp_vec3f_t *b) {
#if IVP_ARM64_NEON
    float32x2_t products = vmul_f32(vld1_f32(a->k), vld1_f32(b->k));
    float sum = vget_lane_f32(products, 0) + vget_lane_f32(products, 1);
    sum += a->k[2] * b->k[2];
    return (D)sum;
#else
    return (D)(a->k[0] * b->k[0] + a->k[1] * b->k[1] + a->k[2] * b->k[2]);
#endif
}
static inline D lvf_quad_distance(const ivp_vec3f_t *a, const ivp_vec3f_t *p) {
    D x = (D)(a->k[0] - p->k[0]), y = (D)(a->k[1] - p->k[1]), z = (D)(a->k[2] - p->k[2]);
    return x * x + y * y + z * z;
}
static inline void lvf_mult(ivp_vec3f_t *a, D f) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    float64x2_t values = vcvt_f64_f32(vld1_f32(a->k));
    values = vmulq_f64(values, vdupq_n_f64(f));
    vst1_f32(a->k, vcvt_f32_f64(values));
    a->k[2] = (float)((D)a->k[2] * f);
#else
    a->k[0] = (float)((D)a->k[0] * f); a->k[1] = (float)((D)a->k[1] * f); a->k[2] = (float)((D)a->k[2] * f);
#endif
}
static inline void lvf_add2(ivp_vec3f_t *r, const ivp_vec3f_t *a, const ivp_vec3f_t *b) {
#if IVP_ARM64_NEON
    vst1_f32(r->k, vadd_f32(vld1_f32(a->k), vld1_f32(b->k)));
    r->k[2] = a->k[2] + b->k[2];
#else
    r->k[0] = a->k[0] + b->k[0]; r->k[1] = a->k[1] + b->k[1]; r->k[2] = a->k[2] + b->k[2];
#endif
}
static inline void lvf_add(ivp_vec3f_t *r, const ivp_vec3f_t *b) { lvf_add2(r, r, b); }
static inline void lvf_subtract2(ivp_vec3f_t *r, const ivp_vec3f_t *a, const ivp_vec3f_t *b) {
#if IVP_ARM64_NEON
    vst1_f32(r->k, vsub_f32(vld1_f32(a->k), vld1_f32(b->k)));
    r->k[2] = a->k[2] - b->k[2];
#else
    r->k[0] = a->k[0] - b->k[0]; r->k[1] = a->k[1] - b->k[1]; r->k[2] = a->k[2] - b->k[2];
#endif
}
static inline void lvf_subtract(ivp_vec3f_t *r, const ivp_vec3f_t *b) { lvf_subtract2(r, r, b); }
static inline void lvf_subtract2_p(ivp_vec3f_t *r, const ivp_vec3r_t *a, const ivp_vec3r_t *b) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    float64x2_t values = vsubq_f64(vld1q_f64(a->k), vld1q_f64(b->k));
    vst1_f32(r->k, vcvt_f32_f64(values));
    r->k[2] = (float)(a->k[2] - b->k[2]);
#else
    r->k[0] = (float)(a->k[0] - b->k[0]); r->k[1] = (float)(a->k[1] - b->k[1]); r->k[2] = (float)(a->k[2] - b->k[2]);
#endif
}
/* this = v1 + v2 * f */
static inline void lvf_add_multiple2(ivp_vec3f_t *r, const ivp_vec3f_t *v1, const ivp_vec3f_t *v2, D f) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    float64x2_t values = vcvt_f64_f32(vld1_f32(v2->k));
    values = vmulq_f64(values, vdupq_n_f64(f));
    values = vaddq_f64(values, vcvt_f64_f32(vld1_f32(v1->k)));
    vst1_f32(r->k, vcvt_f32_f64(values));
    D c = (D)v2->k[2] * f + (D)v1->k[2];
    r->k[2] = (float)c;
#else
    D a = (D)v2->k[0] * f + (D)v1->k[0];
    D b = (D)v2->k[1] * f + (D)v1->k[1];
    D c = (D)v2->k[2] * f + (D)v1->k[2];
    r->k[0] = (float)a; r->k[1] = (float)b; r->k[2] = (float)c;
#endif
}
static inline void lvf_add_multiple(ivp_vec3f_t *r, const ivp_vec3f_t *v, D f) { lvf_add_multiple2(r, r, v, f); }
static inline void lvf_set_multiple(ivp_vec3f_t *r, const ivp_vec3f_t *v, D f) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    float64x2_t values = vmulq_f64(vcvt_f64_f32(vld1_f32(v->k)), vdupq_n_f64(f));
    vst1_f32(r->k, vcvt_f32_f64(values));
    r->k[2] = (float)((D)v->k[2] * f);
#else
    r->k[0] = (float)((D)v->k[0] * f); r->k[1] = (float)((D)v->k[1] * f); r->k[2] = (float)((D)v->k[2] * f);
#endif
}
static inline void lvf_set_multiple_p(ivp_vec3f_t *r, const ivp_vec3r_t *v, D f) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    float64x2_t values = vmulq_f64(vld1q_f64(v->k), vdupq_n_f64(f));
    vst1_f32(r->k, vcvt_f32_f64(values));
    r->k[2] = (float)(v->k[2] * f);
#else
    r->k[0] = (float)(v->k[0] * f); r->k[1] = (float)(v->k[1] * f); r->k[2] = (float)(v->k[2] * f);
#endif
}
static inline void lvf_set_pairwise_mult(ivp_vec3f_t *r, const ivp_vec3f_t *a, const ivp_vec3f_t *b) {
#if IVP_ARM64_NEON
    vst1_f32(r->k, vmul_f32(vld1_f32(a->k), vld1_f32(b->k)));
    r->k[2] = a->k[2] * b->k[2];
#else
    r->k[0] = a->k[0] * b->k[0]; r->k[1] = a->k[1] * b->k[1]; r->k[2] = a->k[2] * b->k[2];
#endif
}
static inline void lvf_set_negative(ivp_vec3f_t *r, const ivp_vec3f_t *a) {
    r->k[0] = -a->k[0]; r->k[1] = -a->k[1]; r->k[2] = -a->k[2];
}
/* this = (v1 - v2) * f */
static inline void lvf_subtract_and_mult(ivp_vec3f_t *r, const ivp_vec3f_t *v1, const ivp_vec3f_t *v2, D f) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    float32x2_t delta = vsub_f32(vld1_f32(v1->k), vld1_f32(v2->k));
    vst1_f32(r->k, vcvt_f32_f64(vmulq_n_f64(vcvt_f64_f32(delta), f)));
    D c = (D)(v1->k[2] - v2->k[2]) * f;
    r->k[2] = (float)c;
#else
    D a = (D)(v1->k[0] - v2->k[0]) * f, b = (D)(v1->k[1] - v2->k[1]) * f, c = (D)(v1->k[2] - v2->k[2]) * f;
    r->k[0] = (float)a; r->k[1] = (float)b; r->k[2] = (float)c;
#endif
}
static inline void lvf_cross(ivp_vec3f_t *r, const ivp_vec3f_t *v1, const ivp_vec3f_t *v2) {
    D a = (D)(v1->k[1] * v2->k[2]); D a2 = (D)(v2->k[1] * v1->k[2]);
    D b = (D)(v1->k[2] * v2->k[0]); D b2 = (D)(v2->k[2] * v1->k[0]);
    D c = (D)(v1->k[0] * v2->k[1]); D c2 = (D)(v2->k[0] * v1->k[1]);
    a -= a2; b -= b2; c -= c2;
    r->k[0] = (float)a; r->k[1] = (float)b; r->k[2] = (float)c;
}
static inline D lvf_real_length(const ivp_vec3f_t *a) { return LV_SQRT(lvf_quad_length(a)); }
static inline bool lvf_normize(ivp_vec3f_t *a) {
    D l = lvf_quad_length(a);
    if (l < P_DOUBLE_EPS) return false;
    lvf_mult(a, lv_isqrt(l, 2));
    return true;
}
static inline bool lvf_fast_normize(ivp_vec3f_t *a) { return lvf_normize(a); }
static inline D lvf_real_length_plus_normize(ivp_vec3f_t *a) {
    D q = lvf_quad_length(a);
    if (q < P_DOUBLE_EPS) return 0.0f;
    D f = lv_isqrt(q, 3);
    lvf_mult(a, f);
    return f * q;
}
static inline void lvf_cross_and_normize(ivp_vec3f_t *r, const ivp_vec3f_t *v1, const ivp_vec3f_t *v2) {
    lvf_cross(r, v1, v2);
    (void)lvf_normize(r);
}
static inline void lvf_set_orthogonal_part(ivp_vec3f_t *r, const ivp_vec3f_t *v, const ivp_vec3f_t *n) {
    D part = lvf_dot(v, n);
    lvf_add_multiple2(r, v, n, -part);
}
/* inline_set_vert_to_area_defined_by_three_points (float) */
static inline void lvf_set_vert_to_area(ivp_vec3f_t *r, const ivp_vec3f_t *t0, const ivp_vec3f_t *t1,
                                        const ivp_vec3f_t *t2) {
    D fp0 = t0->k[0], fp1 = t0->k[1], fp2 = t0->k[2];
    D a0 = (D)t1->k[0] - fp0, a1 = (D)t1->k[1] - fp1, a2 = (D)t1->k[2] - fp2;
    D b0 = (D)t2->k[0] - fp0, b1 = (D)t2->k[1] - fp1, b2 = (D)t2->k[2] - fp2;
    r->k[0] = (float)(b1 * a2 - a1 * b2);
    r->k[1] = (float)(b2 * a0 - a2 * b0);
    r->k[2] = (float)(b0 * a1 - a0 * b1);
}

/* ── IVP_U_Point (IVP_DOUBLE) ───────────────────────────────────────────── */

static inline void lvp_zero(ivp_vec3r_t *a) { a->k[0] = 0; a->k[1] = 0; a->k[2] = 0; }
static inline void lvp_set3(ivp_vec3r_t *a, D x, D y, D z) { a->k[0] = x; a->k[1] = y; a->k[2] = z; }
static inline void lvp_set(ivp_vec3r_t *a, const ivp_vec3r_t *b) { *a = *b; }
static inline void lvp_set_f(ivp_vec3r_t *a, const ivp_vec3f_t *b) {
    a->k[0] = (D)b->k[0]; a->k[1] = (D)b->k[1]; a->k[2] = (D)b->k[2];
}
static inline void lvp_set_fa(ivp_vec3r_t *a, const float p[3]) {
    a->k[0] = (D)p[0]; a->k[1] = (D)p[1]; a->k[2] = (D)p[2];
}
/* IVP_U_Point::set(const IVP_U_Quat*) rounds to float (ivu_linear_double.hxx) */
static inline void lvp_set_q(ivp_vec3r_t *a, const ivp_quatr_t *q) {
    a->k[0] = (D)(float)q->x; a->k[1] = (D)(float)q->y; a->k[2] = (D)(float)q->z;
}
static inline D lvp_dot(const ivp_vec3r_t *a, const ivp_vec3r_t *b) {
    return a->k[0] * b->k[0] + a->k[1] * b->k[1] + a->k[2] * b->k[2];
}
static inline D lvp_dot_f(const ivp_vec3r_t *a, const ivp_vec3f_t *b) {
    return a->k[0] * (D)b->k[0] + a->k[1] * (D)b->k[1] + a->k[2] * (D)b->k[2];
}
static inline D lvp_quad_length(const ivp_vec3r_t *a) { return lvp_dot(a, a); }
static inline D lvp_quad_distance(const ivp_vec3r_t *a, const ivp_vec3r_t *p) {
    D x = a->k[0] - p->k[0], y = a->k[1] - p->k[1], z = a->k[2] - p->k[2];
    return x * x + y * y + z * z;
}
static inline D lvp_quad_distance_f(const ivp_vec3r_t *a, const ivp_vec3f_t *p) {
    D x = a->k[0] - (D)p->k[0], y = a->k[1] - (D)p->k[1], z = a->k[2] - (D)p->k[2];
    return x * x + y * y + z * z;
}
static inline void lvp_mult(ivp_vec3r_t *a, D f) { a->k[0] *= f; a->k[1] *= f; a->k[2] *= f; }
static inline void lvp_add(ivp_vec3r_t *a, const ivp_vec3r_t *b) {
    a->k[0] += b->k[0]; a->k[1] += b->k[1]; a->k[2] += b->k[2];
}
static inline void lvp_add_f(ivp_vec3r_t *a, const ivp_vec3f_t *b) {
    a->k[0] += (D)b->k[0]; a->k[1] += (D)b->k[1]; a->k[2] += (D)b->k[2];
}
static inline void lvp_add2(ivp_vec3r_t *r, const ivp_vec3r_t *a, const ivp_vec3r_t *b) {
    r->k[0] = a->k[0] + b->k[0]; r->k[1] = a->k[1] + b->k[1]; r->k[2] = a->k[2] + b->k[2];
}
static inline void lvp_add2_ff(ivp_vec3r_t *r, const ivp_vec3f_t *a, const ivp_vec3f_t *b) {
    r->k[0] = (D)(a->k[0] + b->k[0]); r->k[1] = (D)(a->k[1] + b->k[1]); r->k[2] = (D)(a->k[2] + b->k[2]);
}
static inline void lvp_add_multiple(ivp_vec3r_t *a, const ivp_vec3r_t *v, D f) {
    a->k[0] = v->k[0] * f + a->k[0]; a->k[1] = v->k[1] * f + a->k[1]; a->k[2] = v->k[2] * f + a->k[2];
}
static inline void lvp_add_multiple_f(ivp_vec3r_t *a, const ivp_vec3f_t *v, D f) {
    a->k[0] = (D)v->k[0] * f + a->k[0]; a->k[1] = (D)v->k[1] * f + a->k[1]; a->k[2] = (D)v->k[2] * f + a->k[2];
}
static inline void lvp_add_multiple2(ivp_vec3r_t *r, const ivp_vec3r_t *v1, const ivp_vec3r_t *v2, D f) {
    D a = v2->k[0] * f + v1->k[0], b = v2->k[1] * f + v1->k[1], c = v2->k[2] * f + v1->k[2];
    r->k[0] = a; r->k[1] = b; r->k[2] = c;
}
static inline void lvp_add_multiple2_pf(ivp_vec3r_t *r, const ivp_vec3r_t *v1, const ivp_vec3f_t *v2, D f) {
    D a = (D)v2->k[0] * f + v1->k[0], b = (D)v2->k[1] * f + v1->k[1], c = (D)v2->k[2] * f + v1->k[2];
    r->k[0] = a; r->k[1] = b; r->k[2] = c;
}
static inline void lvp_subtract2(ivp_vec3r_t *r, const ivp_vec3r_t *a, const ivp_vec3r_t *b) {
    r->k[0] = a->k[0] - b->k[0]; r->k[1] = a->k[1] - b->k[1]; r->k[2] = a->k[2] - b->k[2];
}
static inline void lvp_subtract(ivp_vec3r_t *r, const ivp_vec3r_t *b) { lvp_subtract2(r, r, b); }
static inline void lvp_subtract_f(ivp_vec3r_t *r, const ivp_vec3f_t *b) {
    r->k[0] -= (D)b->k[0]; r->k[1] -= (D)b->k[1]; r->k[2] -= (D)b->k[2];
}
static inline void lvp_subtract2_pf(ivp_vec3r_t *r, const ivp_vec3r_t *a, const ivp_vec3f_t *b) {
    r->k[0] = a->k[0] - (D)b->k[0]; r->k[1] = a->k[1] - (D)b->k[1]; r->k[2] = a->k[2] - (D)b->k[2];
}
static inline void lvp_subtract2_fp(ivp_vec3r_t *r, const ivp_vec3f_t *a, const ivp_vec3r_t *b) {
    r->k[0] = (D)a->k[0] - b->k[0]; r->k[1] = (D)a->k[1] - b->k[1]; r->k[2] = (D)a->k[2] - b->k[2];
}
/* IVP_U_Point::subtract(Float, Float): float subtraction promoted */
static inline void lvp_subtract2_ff(ivp_vec3r_t *r, const ivp_vec3f_t *a, const ivp_vec3f_t *b) {
    r->k[0] = (D)(a->k[0] - b->k[0]); r->k[1] = (D)(a->k[1] - b->k[1]); r->k[2] = (D)(a->k[2] - b->k[2]);
}
static inline void lvp_subtract_and_mult(ivp_vec3r_t *r, const ivp_vec3r_t *v1, const ivp_vec3r_t *v2, D f) {
    D a = (v1->k[0] - v2->k[0]) * f, b = (v1->k[1] - v2->k[1]) * f, c = (v1->k[2] - v2->k[2]) * f;
    r->k[0] = a; r->k[1] = b; r->k[2] = c;
}
static inline void lvp_set_multiple(ivp_vec3r_t *r, const ivp_vec3r_t *p, D f) {
    r->k[0] = p->k[0] * f; r->k[1] = p->k[1] * f; r->k[2] = p->k[2] * f;
}
static inline void lvp_set_multiple_f(ivp_vec3r_t *r, const ivp_vec3f_t *p, D f) {
    r->k[0] = (D)p->k[0] * f; r->k[1] = (D)p->k[1] * f; r->k[2] = (D)p->k[2] * f;
}
static inline void lvp_set_multiple_q(ivp_vec3r_t *r, const ivp_quatr_t *q, D f) {
    r->k[0] = q->x * f; r->k[1] = q->y * f; r->k[2] = q->z * f;
}
static inline void lvp_set_negative(ivp_vec3r_t *r, const ivp_vec3r_t *a) {
    r->k[0] = -a->k[0]; r->k[1] = -a->k[1]; r->k[2] = -a->k[2];
}
static inline void lvp_set_pairwise_mult(ivp_vec3r_t *r, const ivp_vec3r_t *a, const ivp_vec3r_t *b) {
    r->k[0] = a->k[0] * b->k[0]; r->k[1] = a->k[1] * b->k[1]; r->k[2] = a->k[2] * b->k[2];
}
static inline void lvp_set_pairwise_mult_f(ivp_vec3r_t *r, const ivp_vec3r_t *a, const ivp_vec3f_t *b) {
    r->k[0] = a->k[0] * (D)b->k[0]; r->k[1] = a->k[1] * (D)b->k[1]; r->k[2] = a->k[2] * (D)b->k[2];
}
static inline void lvp_cross(ivp_vec3r_t *r, const ivp_vec3r_t *v1, const ivp_vec3r_t *v2) {
    D a = v1->k[1] * v2->k[2] - v2->k[1] * v1->k[2];
    D b = v1->k[2] * v2->k[0] - v2->k[2] * v1->k[0];
    D c = v1->k[0] * v2->k[1] - v2->k[0] * v1->k[1];
    r->k[0] = a; r->k[1] = b; r->k[2] = c;
}
/* IVP_U_Point::inline_calc_cross_product_and_normize: the reference stores the
 * cross product components through (IVP_FLOAT) casts before normize() */
static inline bool lvp_normize(ivp_vec3r_t *a);
static inline void lvp_cross_and_normize(ivp_vec3r_t *r, const ivp_vec3r_t *v1, const ivp_vec3r_t *v2) {
    D a = v1->k[1] * v2->k[2] - v2->k[1] * v1->k[2];
    D b = v1->k[2] * v2->k[0] - v2->k[2] * v1->k[0];
    D c = v1->k[0] * v2->k[1] - v2->k[0] * v1->k[1];
    r->k[0] = (D)(float)a; r->k[1] = (D)(float)b; r->k[2] = (D)(float)c;
    (void)lvp_normize(r);
}
static inline D lvp_real_length(const ivp_vec3r_t *a) { return LV_SQRT(lvp_quad_length(a)); }
static inline D lvp_fast_real_length(const ivp_vec3r_t *a) { return LV_SQRT(lvp_quad_length(a)); }
static inline bool lvp_normize(ivp_vec3r_t *a) {
    D l = lvp_quad_length(a);
    if (l < P_DOUBLE_EPS) return false;
    lvp_mult(a, lv_isqrt(l, 3));
    return true;
}
static inline bool lvp_fast_normize(ivp_vec3r_t *a) {
    D l = lvp_quad_length(a);
    if (l < P_DOUBLE_EPS) return false;
    lvp_mult(a, lv_isqrt(l, 2));
    return true;
}
static inline D lvp_real_length_plus_normize(ivp_vec3r_t *a) {
    D q = lvp_quad_length(a);
    if (q < P_DOUBLE_EPS) return 0.0;
    D f = lv_isqrt(q, 3);
    lvp_mult(a, f);
    return f * q;
}
static inline void lvp_set_orthogonal_part(ivp_vec3r_t *r, const ivp_vec3r_t *v, const ivp_vec3r_t *n) {
    D part = lvp_dot(n, v);
    lvp_add_multiple2(r, v, n, -part);
}
static inline void lvp_set_orthogonal_part_f(ivp_vec3r_t *r, const ivp_vec3r_t *v, const ivp_vec3f_t *n) {
    D part = lvp_dot_f(v, n);
    lvp_add_multiple2_pf(r, v, n, -part);
}
static inline void lvp_set_vert_to_area(ivp_vec3r_t *r, const ivp_vec3r_t *t0, const ivp_vec3r_t *t1,
                                        const ivp_vec3r_t *t2) {
    D fp0 = t0->k[0], fp1 = t0->k[1], fp2 = t0->k[2];
    D a0 = t1->k[0] - fp0, a1 = t1->k[1] - fp1, a2 = t1->k[2] - fp2;
    D b0 = t2->k[0] - fp0, b1 = t2->k[1] - fp1, b2 = t2->k[2] - fp2;
    r->k[0] = b1 * a2 - a1 * b2;
    r->k[1] = b2 * a0 - a2 * b0;
    r->k[2] = b0 * a1 - a0 * b1;
}
static inline void lvp_set_vert_to_area_fff(ivp_vec3r_t *r, const ivp_vec3f_t *t0, const ivp_vec3f_t *t1,
                                            const ivp_vec3f_t *t2) {
    D fp0 = t0->k[0], fp1 = t0->k[1], fp2 = t0->k[2];
    D a0 = (D)t1->k[0] - fp0, a1 = (D)t1->k[1] - fp1, a2 = (D)t1->k[2] - fp2;
    D b0 = (D)t2->k[0] - fp0, b1 = (D)t2->k[1] - fp1, b2 = (D)t2->k[2] - fp2;
    r->k[0] = b1 * a2 - a1 * b2;
    r->k[1] = b2 * a0 - a2 * b0;
    r->k[2] = b0 * a1 - a0 * b1;
}
static inline void lvp_set_vert_to_area_ffp(ivp_vec3r_t *r, const ivp_vec3f_t *t0, const ivp_vec3f_t *t1,
                                            const ivp_vec3r_t *t2) {
    D fp0 = t0->k[0], fp1 = t0->k[1], fp2 = t0->k[2];
    D a0 = (D)t1->k[0] - fp0, a1 = (D)t1->k[1] - fp1, a2 = (D)t1->k[2] - fp2;
    D b0 = t2->k[0] - fp0, b1 = t2->k[1] - fp1, b2 = t2->k[2] - fp2;
    r->k[0] = b1 * a2 - a1 * b2;
    r->k[1] = b2 * a0 - a2 * b0;
    r->k[2] = b0 * a1 - a0 * b1;
}

/* ── IVP_U_Hesse (point + hesse_val, IVP_DOUBLE) ────────────────────────── */

typedef struct lv_hesse {
    ivp_vec3r_t n;
    D hesse_val;
} lv_hesse_t;

static inline D lv_hesse_get_dist(const lv_hesse_t *h, const ivp_vec3r_t *p) {
    return lvp_dot(&h->n, p) + h->hesse_val;
}
static inline D lv_hesse_get_dist_f(const lv_hesse_t *h, const ivp_vec3f_t *p) {
    return lvp_dot_f(&h->n, p) + h->hesse_val;
}
static inline void lv_hesse_calc_fff(lv_hesse_t *h, const ivp_vec3f_t *t0, const ivp_vec3f_t *t1,
                                     const ivp_vec3f_t *t2) {
    lvp_set_vert_to_area_fff(&h->n, t0, t1, t2);
    h->hesse_val = -(h->n.k[0] * (D)t0->k[0] + h->n.k[1] * (D)t0->k[1] + h->n.k[2] * (D)t0->k[2]);
}
static inline void lv_hesse_calc_ppp(lv_hesse_t *h, const ivp_vec3r_t *t0, const ivp_vec3r_t *t1,
                                     const ivp_vec3r_t *t2) {
    lvp_set_vert_to_area(&h->n, t0, t1, t2);
    h->hesse_val = -(h->n.k[0] * t0->k[0] + h->n.k[1] * t0->k[1] + h->n.k[2] * t0->k[2]);
}
static inline void lv_hesse_normize(lv_hesse_t *h) {
    D len = lvp_real_length(&h->n);
    if (len < P_DOUBLE_EPS) { lvp_zero(&h->n); h->hesse_val = 0.0; return; }
    D l = 1.0f / len;
    lvp_mult(&h->n, l);
    h->hesse_val *= l;
}

/* ── IVP_U_Matrix / IVP_U_Matrix3 ───────────────────────────────────────── */

#define LVM_E(m, r, c) ((m)->rot.rows[r].k[c])

/* vmult3: out = R * in */
static inline void lvm_vmult3(const ivp_transform_r_t *m, const ivp_vec3r_t *in, ivp_vec3r_t *out) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    const D x = in->k[0], y = in->k[1], z = in->k[2];
    const float64x2_t column0 = { LVM_E(m, 0, 0), LVM_E(m, 1, 0) };
    const float64x2_t column1 = { LVM_E(m, 0, 1), LVM_E(m, 1, 1) };
    const float64x2_t column2 = { LVM_E(m, 0, 2), LVM_E(m, 1, 2) };
    float64x2_t values = vmulq_n_f64(column0, x);
    values = vaddq_f64(values, vmulq_n_f64(column1, y));
    values = vaddq_f64(values, vmulq_n_f64(column2, z));
    D c = LVM_E(m, 2, 0) * x + LVM_E(m, 2, 1) * y + LVM_E(m, 2, 2) * z;
    vst1q_f64(out->k, values);
    out->k[2] = c;
#else
    D a = LVM_E(m, 0, 0) * in->k[0] + LVM_E(m, 0, 1) * in->k[1] + LVM_E(m, 0, 2) * in->k[2];
    D b = LVM_E(m, 1, 0) * in->k[0] + LVM_E(m, 1, 1) * in->k[1] + LVM_E(m, 1, 2) * in->k[2];
    D c = LVM_E(m, 2, 0) * in->k[0] + LVM_E(m, 2, 1) * in->k[1] + LVM_E(m, 2, 2) * in->k[2];
    out->k[0] = a; out->k[1] = b; out->k[2] = c;
#endif
}
static inline void lvm_vmult3_f(const ivp_transform_r_t *m, const ivp_vec3f_t *in, ivp_vec3f_t *out) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    const D x = (D)in->k[0], y = (D)in->k[1], z = (D)in->k[2];
    const float64x2_t column0 = { LVM_E(m, 0, 0), LVM_E(m, 1, 0) };
    const float64x2_t column1 = { LVM_E(m, 0, 1), LVM_E(m, 1, 1) };
    const float64x2_t column2 = { LVM_E(m, 0, 2), LVM_E(m, 1, 2) };
    float64x2_t values = vmulq_n_f64(column0, x);
    values = vaddq_f64(values, vmulq_n_f64(column1, y));
    values = vaddq_f64(values, vmulq_n_f64(column2, z));
    D c = LVM_E(m, 2, 0) * x + LVM_E(m, 2, 1) * y + LVM_E(m, 2, 2) * z;
    vst1_f32(out->k, vcvt_f32_f64(values));
    out->k[2] = (float)c;
#else
    D a = LVM_E(m, 0, 0) * (D)in->k[0] + LVM_E(m, 0, 1) * (D)in->k[1] + LVM_E(m, 0, 2) * (D)in->k[2];
    D b = LVM_E(m, 1, 0) * (D)in->k[0] + LVM_E(m, 1, 1) * (D)in->k[1] + LVM_E(m, 1, 2) * (D)in->k[2];
    D c = LVM_E(m, 2, 0) * (D)in->k[0] + LVM_E(m, 2, 1) * (D)in->k[1] + LVM_E(m, 2, 2) * (D)in->k[2];
    out->k[0] = (float)a; out->k[1] = (float)b; out->k[2] = (float)c;
#endif
}
/* vimult3: out = R^T * in */
static inline void lvm_vimult3(const ivp_transform_r_t *m, const ivp_vec3r_t *in, ivp_vec3r_t *out) {
    D a = LVM_E(m, 0, 0) * in->k[0] + LVM_E(m, 1, 0) * in->k[1] + LVM_E(m, 2, 0) * in->k[2];
    D b = LVM_E(m, 0, 1) * in->k[0] + LVM_E(m, 1, 1) * in->k[1] + LVM_E(m, 2, 1) * in->k[2];
    D c = LVM_E(m, 0, 2) * in->k[0] + LVM_E(m, 1, 2) * in->k[1] + LVM_E(m, 2, 2) * in->k[2];
    out->k[0] = a; out->k[1] = b; out->k[2] = c;
}
static inline void lvm_vimult3_f(const ivp_transform_r_t *m, const ivp_vec3f_t *in, ivp_vec3f_t *out) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    const D x = (D)in->k[0], y = (D)in->k[1], z = (D)in->k[2];
    const float64x2_t row0 = { LVM_E(m, 0, 0), LVM_E(m, 0, 1) };
    const float64x2_t row1 = { LVM_E(m, 1, 0), LVM_E(m, 1, 1) };
    const float64x2_t row2 = { LVM_E(m, 2, 0), LVM_E(m, 2, 1) };
    float64x2_t values = vmulq_n_f64(row0, x);
    values = vaddq_f64(values, vmulq_n_f64(row1, y));
    values = vaddq_f64(values, vmulq_n_f64(row2, z));
    D c = LVM_E(m, 0, 2) * x + LVM_E(m, 1, 2) * y + LVM_E(m, 2, 2) * z;
    vst1_f32(out->k, vcvt_f32_f64(values));
    out->k[2] = (float)c;
#else
    D a = LVM_E(m, 0, 0) * (D)in->k[0] + LVM_E(m, 1, 0) * (D)in->k[1] + LVM_E(m, 2, 0) * (D)in->k[2];
    D b = LVM_E(m, 0, 1) * (D)in->k[0] + LVM_E(m, 1, 1) * (D)in->k[1] + LVM_E(m, 2, 1) * (D)in->k[2];
    D c = LVM_E(m, 0, 2) * (D)in->k[0] + LVM_E(m, 1, 2) * (D)in->k[1] + LVM_E(m, 2, 2) * (D)in->k[2];
    out->k[0] = (float)a; out->k[1] = (float)b; out->k[2] = (float)c;
#endif
}
/* vimult3 point in -> float out */
static inline void lvm_vimult3_pf(const ivp_transform_r_t *m, const ivp_vec3r_t *in, ivp_vec3f_t *out) {
    ivp_vec3r_t t; lvm_vimult3(m, in, &t);
    out->k[0] = (float)t.k[0]; out->k[1] = (float)t.k[1]; out->k[2] = (float)t.k[2];
}
/* vmult4: out = R * in + vv */
static inline void lvm_vmult4(const ivp_transform_r_t *m, const ivp_vec3r_t *in, ivp_vec3r_t *out) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    const D x = in->k[0], y = in->k[1], z = in->k[2];
    const float64x2_t column0 = { LVM_E(m, 0, 0), LVM_E(m, 1, 0) };
    const float64x2_t column1 = { LVM_E(m, 0, 1), LVM_E(m, 1, 1) };
    const float64x2_t column2 = { LVM_E(m, 0, 2), LVM_E(m, 1, 2) };
    float64x2_t values = vmulq_n_f64(column0, x);
    values = vaddq_f64(values, vmulq_n_f64(column1, y));
    values = vaddq_f64(values, vmulq_n_f64(column2, z));
    values = vaddq_f64(values, vld1q_f64(m->pos.k));
    D c = x * LVM_E(m, 2, 0);
    c += y * LVM_E(m, 2, 1);
    c += z * LVM_E(m, 2, 2);
    c += m->pos.k[2];
    vst1q_f64(out->k, values);
    out->k[2] = c;
#else
    D h = in->k[0];
    D a = h * LVM_E(m, 0, 0), b = h * LVM_E(m, 1, 0), c = h * LVM_E(m, 2, 0);
    h = in->k[1];
    a += h * LVM_E(m, 0, 1); b += h * LVM_E(m, 1, 1); c += h * LVM_E(m, 2, 1);
    h = in->k[2];
    a += h * LVM_E(m, 0, 2); b += h * LVM_E(m, 1, 2); c += h * LVM_E(m, 2, 2);
    a += m->pos.k[0]; b += m->pos.k[1]; c += m->pos.k[2];
    out->k[0] = a; out->k[1] = b; out->k[2] = c;
#endif
}
static inline void lvm_vmult4_fp(const ivp_transform_r_t *m, const ivp_vec3f_t *in, ivp_vec3r_t *out) {
    ivp_vec3r_t p; lvp_set_f(&p, in);
    lvm_vmult4(m, &p, out);
}
static inline void lvm_vmult4_ff(const ivp_transform_r_t *m, const ivp_vec3f_t *in, ivp_vec3f_t *out) {
    ivp_vec3r_t p, o; lvp_set_f(&p, in);
    lvm_vmult4(m, &p, &o);
    out->k[0] = (float)o.k[0]; out->k[1] = (float)o.k[1]; out->k[2] = (float)o.k[2];
}
/* vimult4: out = R^T * (in - vv) */
static inline void lvm_vimult4(const ivp_transform_r_t *m, const ivp_vec3r_t *in, ivp_vec3r_t *out) {
    D a = in->k[0] - m->pos.k[0], b = in->k[1] - m->pos.k[1], c = in->k[2] - m->pos.k[2];
    D x = LVM_E(m, 0, 0) * a + LVM_E(m, 1, 0) * b + LVM_E(m, 2, 0) * c;
    D y = LVM_E(m, 0, 1) * a + LVM_E(m, 1, 1) * b + LVM_E(m, 2, 1) * c;
    D z = LVM_E(m, 0, 2) * a + LVM_E(m, 1, 2) * b + LVM_E(m, 2, 2) * c;
    out->k[0] = x; out->k[1] = y; out->k[2] = z;
}
static inline void lvm_vimult4_pf(const ivp_transform_r_t *m, const ivp_vec3r_t *in, ivp_vec3f_t *out) {
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    D a = in->k[0] - m->pos.k[0], b = in->k[1] - m->pos.k[1], c = in->k[2] - m->pos.k[2];
    const float64x2_t row0 = { LVM_E(m, 0, 0), LVM_E(m, 0, 1) };
    const float64x2_t row1 = { LVM_E(m, 1, 0), LVM_E(m, 1, 1) };
    const float64x2_t row2 = { LVM_E(m, 2, 0), LVM_E(m, 2, 1) };
    float64x2_t values = vmulq_n_f64(row0, a);
    values = vaddq_f64(values, vmulq_n_f64(row1, b));
    values = vaddq_f64(values, vmulq_n_f64(row2, c));
    vst1_f32(out->k, vcvt_f32_f64(values));
    D z = LVM_E(m, 0, 2) * a + LVM_E(m, 1, 2) * b + LVM_E(m, 2, 2) * c;
    out->k[2] = (float)z;
#else
    ivp_vec3r_t o; lvm_vimult4(m, in, &o);
    out->k[0] = (float)o.k[0]; out->k[1] = (float)o.k[1]; out->k[2] = (float)o.k[2];
#endif
}
static inline void lvm_vimult4_ff(const ivp_transform_r_t *m, const ivp_vec3f_t *in, ivp_vec3f_t *out) {
    ivp_vec3r_t p; lvp_set_f(&p, in);
    lvm_vimult4_pf(m, &p, out);
}
/* get_col */
static inline void lvm_get_col(const ivp_transform_r_t *m, int col, ivp_vec3r_t *out) {
    out->k[0] = LVM_E(m, 0, col); out->k[1] = LVM_E(m, 1, col); out->k[2] = LVM_E(m, 2, col);
}
/* mimult4: m_out = this^-1 * mb (orthonormal) */
static inline void lvm_mimult4(const ivp_transform_r_t *a, const ivp_transform_r_t *mb, ivp_transform_r_t *out) {
    ivp_vec3r_t vsk, vv;
    lvp_subtract2(&vsk, &mb->pos, &a->pos);
    lvm_vimult3(a, &vsk, &vv);
    ivp_vec3r_t col[3];
    for (int c = 0; c < 3; c++) {
        ivp_vec3r_t t; lvm_get_col(mb, c, &t);
        lvm_vimult3(a, &t, &col[c]);
    }
    /* inline_mimult3: out rows = vimult3(this, cols of mb) — reference stores as rows */
    ivp_transform_r_t r;
    for (int i = 0; i < 3; i++) {
        r.rot.rows[i].k[0] = col[0].k[i];
        r.rot.rows[i].k[1] = col[1].k[i];
        r.rot.rows[i].k[2] = col[2].k[i];
    }
    r.pos = vv;
    *out = r;
}

/* quaternion -> matrix (IVP_U_Quat::set_matrix) */
static inline void lvq_set_matrix(const ivp_quatr_t *q, ivp_transform_r_t *m) {
    ivp_quat_to_mat3(q, &m->rot);
}

/* IVP_U_Quat::inline_set_mult_quat */
static inline void lvq_mult(ivp_quatr_t *res, const ivp_quatr_t *q1, const ivp_quatr_t *q2) {
    D xx = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    D yy = q1->w * q2->y + q1->y * q2->w + q1->z * q2->x - q1->x * q2->z;
    D zz = q1->w * q2->z + q1->z * q2->w + q1->x * q2->y - q1->y * q2->x;
    D ww = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    res->x = xx; res->y = yy; res->z = zz; res->w = ww;
}

/* IVP_U_Quat::inline_estimate_q_diff_to (reference argument is a float quat) */
static inline D lvq_estimate_q_diff_to(const ivp_quatr_t *a, const ivp_quatf_t *to) {
    D cosom2 = a->x * (D)to->x + a->y * (D)to->y + a->z * (D)to->z + a->w * (D)to->w;
    D qcosom2 = cosom2 * cosom2;
    D qsin2 = 1 - qcosom2;
    return qsin2 * 2;
}

/* IVP_Inline_Math helpers (float) */
static inline float lv_upper_limit_asin(float angle) {
    float quad_angle = angle * angle;
    float triple_angle = angle * quad_angle;
    float quint_angle = triple_angle * quad_angle;
    return angle + (1.0f / 6.0f) * triple_angle + 0.40414f * quint_angle;
}
static inline float lv_approx5_sin(float angle) {
    float q = angle * angle, t = angle * q, p = t * q;
    return angle - (1.0f / 6.0f) * t + (1.0f / 120.0f) * p;
}
static inline float lv_approx5_cos(float angle) {
    float q = angle * angle;
    return 1.0f - (1.0f / 2.0f) * q + (1.0f / 24.0f) * q * q;
}

#endif /* IVP_LVEC_H */
