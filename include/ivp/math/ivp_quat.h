/* ivp_quat.h — Quaternion type for IVP C17
 *
 * Layout: (x, y, z, w) where w is the scalar part.
 * Matches original IVP_U_Quat layout.
 */
#ifndef IVP_QUAT_H
#define IVP_QUAT_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/math/ivp_vec3.h>

/* Forward declare mat3 */
typedef struct ivp_mat3r ivp_mat3r_t;

#ifdef __cplusplus
extern "C" {
#endif

/* ── Types ──────────────────────────────────────────────────────────────── */

typedef struct IVP_ALIGN16 ivp_quatr {
    ivp_real_t x, y, z, w;
} ivp_quatr_t;

typedef struct IVP_ALIGN16 ivp_quatf {
    float x, y, z, w;
} ivp_quatf_t;

/* ── Construction ───────────────────────────────────────────────────────── */

static inline ivp_quatr_t ivp_quatr_identity(void) {
    ivp_quatr_t q;
    q.x = q.y = q.z = IVP_REAL(0.0);
    q.w = IVP_REAL(1.0);
    return q;
}

static inline ivp_quatr_t ivp_quatr_make(ivp_real_t x, ivp_real_t y,
                                          ivp_real_t z, ivp_real_t w) {
    ivp_quatr_t q;
    q.x = x; q.y = y; q.z = z; q.w = w;
    return q;
}

/* ── Conversion ─────────────────────────────────────────────────────────── */

/* Quaternion → 3×3 rotation matrix */
void ivp_quat_to_mat3(const ivp_quatr_t *q, ivp_mat3r_t *m);

/* 3×3 rotation matrix → quaternion */
void ivp_quat_from_mat3(ivp_quatr_t *q, const ivp_mat3r_t *m);

/* ── Arithmetic ─────────────────────────────────────────────────────────── */

/* out = q1 * q2 (Hamilton product) */
static inline void ivp_quat_mult(ivp_quatr_t *out, const ivp_quatr_t *q1,
                                  const ivp_quatr_t *q2) {
    ivp_real_t x = q1->w*q2->x + q1->x*q2->w + q1->y*q2->z - q1->z*q2->y;
    ivp_real_t y = q1->w*q2->y - q1->x*q2->z + q1->y*q2->w + q1->z*q2->x;
    ivp_real_t z = q1->w*q2->z + q1->x*q2->y - q1->y*q2->x + q1->z*q2->w;
    ivp_real_t w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
    out->x = x; out->y = y; out->z = z; out->w = w;
}

/* Inline multiply with float quaternion (for integration path) */
static inline void ivp_quat_mult_fq(ivp_quatr_t *out, const ivp_quatr_t *q1,
                                     const ivp_quatf_t *q2) {
    ivp_real_t q2x = (ivp_real_t)q2->x;
    ivp_real_t q2y = (ivp_real_t)q2->y;
    ivp_real_t q2z = (ivp_real_t)q2->z;
    ivp_real_t q2w = (ivp_real_t)q2->w;
    ivp_real_t x = q1->w*q2x + q1->x*q2w + q1->y*q2z - q1->z*q2y;
    ivp_real_t y = q1->w*q2y - q1->x*q2z + q1->y*q2w + q1->z*q2x;
    ivp_real_t z = q1->w*q2z + q1->x*q2y - q1->y*q2x + q1->z*q2w;
    ivp_real_t w = q1->w*q2w - q1->x*q2x - q1->y*q2y - q1->z*q2z;
    out->x = x; out->y = y; out->z = z; out->w = w;
}

/* Conjugate (inverse for unit quaternion) */
static inline ivp_quatr_t ivp_quat_conjugate(const ivp_quatr_t *q) {
    ivp_quatr_t r;
    r.x = -q->x; r.y = -q->y; r.z = -q->z; r.w = q->w;
    return r;
}

/* Dot product of two quaternions */
static inline ivp_real_t ivp_quat_dot(const ivp_quatr_t *a, const ivp_quatr_t *b) {
    return a->x*b->x + a->y*b->y + a->z*b->z + a->w*b->w;
}

/* ── Normalization ──────────────────────────────────────────────────────── */

/* Precise normalization. Returns error if |q| ≈ 0. */
ivp_result_t ivp_quat_normalize(ivp_quatr_t *q);

/* Fast normalization (Newton-Raphson rsqrt). Must be approx unit length.
 * Matches original fast_normize_quat() used in integration path. */
void ivp_quat_fast_normalize(ivp_quatr_t *q);

/* ── Integration helpers (matching original IVP) ────────────────────────── */

/* Small-angle approximation: q = (sin(angle*0.5)*axis, cos(angle*0.5))
 * with clipping for angles > PI.
 * Matches IVP_U_Quat::set_fast_multiple_with_clip() */
void ivp_quat_set_fast_multiple_with_clip(ivp_quatr_t *q,
                                           const ivp_vec3f_t *angles,
                                           ivp_real_t factor);

/* Very fast small-angle approximation (no clip, no trig).
 * Matches IVP_U_Quat::set_very_fast_multiple() */
void ivp_quat_set_very_fast_multiple(ivp_quatr_t *q,
                                      const ivp_vec3f_t *angles,
                                      ivp_real_t factor);

/* ── Interpolation ──────────────────────────────────────────────────────── */

/* Spherical linear interpolation */
void ivp_quat_slerp(ivp_quatr_t *out, const ivp_quatr_t *a,
                     const ivp_quatr_t *b, ivp_real_t t);

/* Linear interpolation (then normalize) */
void ivp_quat_nlerp(ivp_quatr_t *out, const ivp_quatr_t *a,
                     const ivp_quatr_t *b, ivp_real_t t);

/* ── Comparison ─────────────────────────────────────────────────────────── */

/* Angular difference in radians: 2*acos(min(1, |q0·q1|)) */
ivp_real_t ivp_quat_angle_diff(const ivp_quatr_t *a, const ivp_quatr_t *b);

/* Rough estimate of quadratic angle difference (fast, no trig).
 * Float-precision reference variant (legacy). */
static inline ivp_real_t ivp_quat_estimate_qdiff(const ivp_quatr_t *a,
                                                   const ivp_quatf_t *ref) {
    ivp_real_t dx = a->x - (ivp_real_t)ref->x;
    ivp_real_t dy = a->y - (ivp_real_t)ref->y;
    ivp_real_t dz = a->z - (ivp_real_t)ref->z;
    ivp_real_t dw = a->w - (ivp_real_t)ref->w;
    return dx*dx + dy*dy + dz*dz + dw*dw;
}

/* Full-precision variant — no truncation when IVP_REAL_DOUBLE is defined. */
static inline ivp_real_t ivp_quat_estimate_qdiff_r(const ivp_quatr_t *a,
                                                     const ivp_quatr_t *ref) {
    ivp_real_t dx = a->x - ref->x;
    ivp_real_t dy = a->y - ref->y;
    ivp_real_t dz = a->z - ref->z;
    ivp_real_t dw = a->w - ref->w;
    return dx*dx + dy*dy + dz*dz + dw*dw;
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_QUAT_H */
