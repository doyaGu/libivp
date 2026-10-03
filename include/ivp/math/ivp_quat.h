/* ivp_quat.h — Quaternion type for IVP C17
 *
 * Layout: (x, y, z, w) where w is the scalar part.
 * Matches original IVP_U_Quat layout.
 */
#ifndef IVP_QUAT_H
#define IVP_QUAT_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/base/ivp_forward_decls.h>
#include <ivp/math/ivp_vec3.h>

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

/* ── Precision note ─────────────────────────────────────────────────────── *
 * The reference IVP_U_Quat stores IVP_DOUBLE components.  Every function
 * below evaluates the reference formula in double and rounds to ivp_real_t
 * on store, so the IVP_REAL_DOUBLE build is bit-identical to the reference
 * (strict FP, no FMA contraction) and the float build only differs by the
 * final rounding of the stored components. */

/* ── Conversion ─────────────────────────────────────────────────────────── */

/* Quaternion → 3×3 rotation matrix (IVP_U_Quat::set_matrix(IVP_U_Matrix3*)) */
void ivp_quat_to_mat3(const ivp_quatr_t *q, ivp_mat3r_t *m);

/* Quaternion → 4×4 matrix (IVP_U_Quat::set_matrix(IVP_DOUBLE m[4][4])),
 * rotation in m[0..2][0..2] (row-major like set_matrix(Matrix3*)), zero
 * shift, m[3][3] = 1. */
void ivp_quat_to_mat4x4(const ivp_quatr_t *q, double m[4][4]);

/* 3×3 rotation matrix → quaternion
 * (IVP_U_Quat::set_quaternion(const IVP_U_Matrix3*), ends with normize_quat) */
void ivp_quat_from_mat3(ivp_quatr_t *q, const ivp_mat3r_t *m);

/* 4×4 column-major (OpenGL) matrix → quaternion
 * (IVP_U_Quat::set_quaternion(const IVP_DOUBLE m[4][4]); no normize). */
void ivp_quat_from_mat4x4(ivp_quatr_t *q, const double m[4][4]);

/* IVP_U_Float_Quat::set(const IVP_U_Quat*) */
static inline void ivp_quatf_set(ivp_quatf_t *out, const ivp_quatr_t *src) {
    out->x = (float)src->x;
    out->y = (float)src->y;
    out->z = (float)src->z;
    out->w = (float)src->w;
}

/* ── Arithmetic ─────────────────────────────────────────────────────────── */

/* out = q1 * q2 (Hamilton product), IVP_U_Quat::inline_set_mult_quat.
 * Evaluation order matches the reference exactly; out may alias q1/q2. */
static inline void ivp_quat_mult(ivp_quatr_t *out, const ivp_quatr_t *q1,
                                  const ivp_quatr_t *q2) {
    double x1 = (double)q1->x, y1 = (double)q1->y, z1 = (double)q1->z, w1 = (double)q1->w;
    double x2 = (double)q2->x, y2 = (double)q2->y, z2 = (double)q2->z, w2 = (double)q2->w;
    double xx = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    double yy = w1*y2 + y1*w2 + z1*x2 - x1*z2;
    double zz = w1*z2 + z1*w2 + x1*y2 - y1*x2;
    double ww = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    out->x = (ivp_real_t)xx; out->y = (ivp_real_t)yy;
    out->z = (ivp_real_t)zz; out->w = (ivp_real_t)ww;
}

/* out = q1 * q2 with a float quaternion
 * (IVP_U_Quat::inline_set_mult_quat(const IVP_U_Quat*, const IVP_U_Float_Quat*)) */
static inline void ivp_quat_mult_fq(ivp_quatr_t *out, const ivp_quatr_t *q1,
                                     const ivp_quatf_t *q2) {
    double x1 = (double)q1->x, y1 = (double)q1->y, z1 = (double)q1->z, w1 = (double)q1->w;
    double x2 = (double)q2->x, y2 = (double)q2->y, z2 = (double)q2->z, w2 = (double)q2->w;
    double xx = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    double yy = w1*y2 + y1*w2 + z1*x2 - x1*z2;
    double zz = w1*z2 + z1*w2 + x1*y2 - y1*x2;
    double ww = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    out->x = (ivp_real_t)xx; out->y = (ivp_real_t)yy;
    out->z = (ivp_real_t)zz; out->w = (ivp_real_t)ww;
}

/* res = q1 / q2 = q1 * conj(q2) for unit quaternions (set_div_unit_quat) */
void ivp_quat_set_div_unit_quat(ivp_quatr_t *out, const ivp_quatr_t *q1,
                                const ivp_quatr_t *q2);

/* res = conj(q1) * q2 for unit quaternions (set_invert_mult) */
void ivp_quat_set_invert_mult(ivp_quatr_t *out, const ivp_quatr_t *q1,
                              const ivp_quatr_t *q2);

/* Conjugate (inverse for unit quaternion), IVP_U_Quat::set_invert_unit_quat */
static inline ivp_quatr_t ivp_quat_conjugate(const ivp_quatr_t *q) {
    ivp_quatr_t r;
    r.x = -q->x; r.y = -q->y; r.z = -q->z; r.w = q->w;
    return r;
}

/* General inverse 1/q (IVP_U_Quat::invert_quat): conj(q) / |q|^2, in place */
void ivp_quat_invert(ivp_quatr_t *q);

/* Dot product of two quaternions (IVP_U_Quat::acos_quat) */
static inline ivp_real_t ivp_quat_dot(const ivp_quatr_t *a, const ivp_quatr_t *b) {
    return a->x*b->x + a->y*b->y + a->z*b->z + a->w*b->w;
}

/* ── Normalization ──────────────────────────────────────────────────────── */

/* IVP_U_Quat::normize_quat: if |q|^2 > P_DOUBLE_EPS, q *= 1/sqrt(|q|^2).
 * Returns IVP_ERR_NORMALIZATION_FAILED (q unchanged) otherwise. */
ivp_result_t ivp_quat_normalize(ivp_quatr_t *q);

/* IVP_U_Quat::fast_normize_quat: no-op if |1 - |q|^2| <= P_DOUBLE_RES,
 * otherwise Newton iteration factor += 0.5*(1 - factor^2 |q|^2) (start
 * 1.5 - 0.5|q|^2) until |1 - factor^2 |q|^2| <= P_DOUBLE_RES (1e-12).
 * Only valid for |q| = 1 +- 0.1. */
void ivp_quat_fast_normalize(ivp_quatr_t *q);

/* IVP_U_Quat::normize_correct_step(steps): fixed number (1..6) of
 * correction steps of factor = 1.5 - |q|^2/2; factor += 0.5 - factor^2|q|^2/2 */
void ivp_quat_normize_correct_step(ivp_quatr_t *q, int steps);

/* ── Construction from rotation angles ──────────────────────────────────── */

/* IVP_U_Quat::set(rot_x, rot_y, rot_z): xyz = sin(rot/2),
 * w = sqrt(1 - |xyz|^2) (0 if |xyz|^2 >= 1).  Note: rot_ < IVP_PI_2. */
void ivp_quat_set_rotation(ivp_quatr_t *q, double rot_x, double rot_y, double rot_z);

/* IVP_U_Quat::set_fast_multiple(const IVP_U_Point *angles, factor):
 * xyz = sin(angles*factor/2), w = sqrt(1 - min(|xyz|^2, 1)). */
void ivp_quat_set_fast_multiple(ivp_quatr_t *q, const ivp_vec3r_t *angles,
                                double factor);

/* IVP_U_Quat::set_fast_multiple_with_clip (ivu_quat.cxx):
 * xyz = sin(angles*factor/2) (double sin);  if |xyz|^2 > 1 the vector is
 * rescaled to length (1 - P_RES_EPS) (P_RES_EPS = 1e-8);
 * w = sqrt(1 - |xyz|^2).  Unit quaternion, never crashes. */
void ivp_quat_set_fast_multiple_with_clip(ivp_quatr_t *q,
                                           const ivp_vec3f_t *angles,
                                           ivp_real_t factor);

/* IVP_U_Quat::set_very_fast_multiple (ivu_quat.cxx):
 * xyz = s - s^3/6 with s = (IVP_FLOAT)(angles*factor/2) evaluated in float,
 * w = sqrt(1 - |xyz|^2) (0 if |xyz|^2 >= 1).  Unit quaternion. */
void ivp_quat_set_very_fast_multiple(ivp_quatr_t *q,
                                      const ivp_vec3f_t *angles,
                                      ivp_real_t factor);

/* IVP_U_Quat::get_angles: angles_out = 2 * asin(xyz) (IVP_FLOAT result) */
void ivp_quat_get_angles(const ivp_quatr_t *q, ivp_vec3f_t *angles_out);

/* IVP_U_Quat::set_from_rotation_vectors: quaternion rotating the UNIT
 * vector (x1,y1,z1) onto the UNIT vector (x2,y2,z2). */
void ivp_quat_set_from_rotation_vectors(ivp_quatr_t *q,
                                        double x1, double y1, double z1,
                                        double x2, double y2, double z2);

/* ── Interpolation ──────────────────────────────────────────────────────── */

/* IVP_U_Quat::set_interpolate_smoothly(from, to, t) (ivu_quat.cxx):
 * shortest-path slerp while cos(omega) < (float)(1 - 0.001f), otherwise
 * linear interpolation followed by normize_correct_step(3).  t is NOT
 * clamped (extrapolation allowed).  out may alias from/to. */
void ivp_quat_set_interpolate_smoothly(ivp_quatr_t *out, const ivp_quatr_t *from,
                                       const ivp_quatr_t *to, ivp_real_t t);

/* IVP_U_Quat::set_interpolate_linear: shortest-path component-wise lerp,
 * NOT normalized. */
void ivp_quat_set_interpolate_linear(ivp_quatr_t *out, const ivp_quatr_t *from,
                                     const ivp_quatr_t *to, ivp_real_t t);

/* Spherical linear interpolation — same as ivp_quat_set_interpolate_smoothly
 * (the reference has no separate slerp; its threshold is used). */
void ivp_quat_slerp(ivp_quatr_t *out, const ivp_quatr_t *a,
                     const ivp_quatr_t *b, ivp_real_t t);

/* Linear interpolation (then normalize).  libivp helper without a direct
 * reference equivalent (t is used as given, shortest path, normize_quat). */
void ivp_quat_nlerp(ivp_quatr_t *out, const ivp_quatr_t *a,
                     const ivp_quatr_t *b, ivp_real_t t);

/* ── Comparison ─────────────────────────────────────────────────────────── */

/* Angular difference in radians: 2*acos(min(1, |q0·q1|)) */
ivp_real_t ivp_quat_angle_diff(const ivp_quatr_t *a, const ivp_quatr_t *b);

/* IVP_U_Quat::inline_estimate_q_diff_to(const IVP_U_Float_Quat *reference)
 * (ivu_matrix_macros.hxx): roughly estimates the quad angle,
 *   cosom2 = q . ref;  return 2 * (1 - cosom2^2)   (= 2 sin^2(alpha/2)). */
static inline ivp_real_t ivp_quat_estimate_qdiff(const ivp_quatr_t *a,
                                                   const ivp_quatf_t *ref) {
    double cosom2 = (double)a->x * (double)ref->x + (double)a->y * (double)ref->y +
                    (double)a->z * (double)ref->z + (double)a->w * (double)ref->w;
    double qcosom2 = cosom2 * cosom2;
    double qsin2 = 1 - qcosom2;
    return (ivp_real_t)(qsin2 * 2);
}

/* Same estimate against a full-precision reference quaternion. */
static inline ivp_real_t ivp_quat_estimate_qdiff_r(const ivp_quatr_t *a,
                                                     const ivp_quatr_t *ref) {
    double cosom2 = (double)a->x * (double)ref->x + (double)a->y * (double)ref->y +
                    (double)a->z * (double)ref->z + (double)a->w * (double)ref->w;
    double qcosom2 = cosom2 * cosom2;
    double qsin2 = 1 - qcosom2;
    return (ivp_real_t)(qsin2 * 2);
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_QUAT_H */
