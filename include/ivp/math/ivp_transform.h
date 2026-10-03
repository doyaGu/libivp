/* ivp_transform.h — Rigid transform (rotation + translation) for IVP C17
 *
 * ivp_transform_r_t = ivp_mat3r_t (rotation) + ivp_vec3r_t (translation)
 * Matches original IVP_U_Matrix layout.
 */
#ifndef IVP_TRANSFORM_H
#define IVP_TRANSFORM_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Type ───────────────────────────────────────────────────────────────── */

typedef struct ivp_transform_r {
    ivp_mat3r_t rot;    /* rotation matrix (row-major) */
    ivp_vec3r_t pos;    /* translation vector */
} ivp_transform_r_t;

/* ── Construction ───────────────────────────────────────────────────────── */

/* Set to identity transform */
void ivp_transform_identity(ivp_transform_r_t *t);

/* Set from rotation matrix and translation vector */
static inline void ivp_transform_set(ivp_transform_r_t *t,
                                      const ivp_mat3r_t *rot,
                                      const ivp_vec3r_t *pos) {
    t->rot = *rot;
    t->pos = *pos;
}

/* ── Point transformation ───────────────────────────────────────────────── */

/* out = rot * p + pos */
static inline void ivp_transform_point(ivp_vec3r_t *out,
                                        const ivp_transform_r_t *t,
                                        const ivp_vec3r_t *p) {
    ivp_mat3r_vmult(out, &t->rot, p);
    ivp_vec3r_add(out, out, &t->pos);
}

/* out = transpose(rot) * (p - pos) */
static inline void ivp_transform_inv_point(ivp_vec3r_t *out,
                                            const ivp_transform_r_t *t,
                                            const ivp_vec3r_t *p) {
    ivp_vec3r_t tmp;
    ivp_vec3r_sub(&tmp, p, &t->pos);
    ivp_mat3r_vimult(out, &t->rot, &tmp);
}

/* ── Direction transformation (no translation) ──────────────────────────── */

/* out = rot * d */
static inline void ivp_transform_dir(ivp_vec3r_t *out,
                                      const ivp_transform_r_t *t,
                                      const ivp_vec3r_t *d) {
    ivp_mat3r_vmult(out, &t->rot, d);
}

/* out = transpose(rot) * d */
static inline void ivp_transform_inv_dir(ivp_vec3r_t *out,
                                          const ivp_transform_r_t *t,
                                          const ivp_vec3r_t *d) {
    ivp_mat3r_vimult(out, &t->rot, d);
}

/* ── Composition ────────────────────────────────────────────────────────── *
 * Reference: IVP_U_Matrix (ivu_linear.cxx / ivu_matrix_macros.hxx).
 * All composition functions allow in-place operation (out == a or b). */

/* out = a * b (first b, then a)  — IVP_U_Matrix::mmult4 */
void ivp_transform_compose(ivp_transform_r_t *out,
                            const ivp_transform_r_t *a,
                            const ivp_transform_r_t *b);

/* out = inv(a) * b  — IVP_U_Matrix::mimult4 (a orthonormal) */
void ivp_transform_inv_compose(ivp_transform_r_t *out,
                                const ivp_transform_r_t *a,
                                const ivp_transform_r_t *b);

/* out = a * inv(b)  — IVP_U_Matrix::mi2mult4 (b orthonormal) */
void ivp_transform_mi2mult(ivp_transform_r_t *out,
                           const ivp_transform_r_t *a,
                           const ivp_transform_r_t *b);

/* out = R(axis, angle) * t  — IVP_U_Matrix::rotate (angle is IVP_FLOAT) */
void ivp_transform_rotate(ivp_transform_r_t *out, const ivp_transform_r_t *t,
                          ivp_coord_index_t axis, float angle);

/* out = t * R(axis, angle)^T  — IVP_U_Matrix::rotate_invers */
void ivp_transform_rotate_invers(ivp_transform_r_t *out, const ivp_transform_r_t *t,
                                 ivp_coord_index_t axis, float angle);

/* ── Inversion ──────────────────────────────────────────────────────────── */

/* Compute the inverse of an orthonormal transform (transpose + shift)
 * — IVP_U_Matrix::set_transpose / transpose */
void ivp_transform_invert(ivp_transform_r_t *out, const ivp_transform_r_t *t);

/* General inverse using the determinant algorithm — IVP_U_Matrix::real_invert.
 * Returns IVP_ERR_SINGULAR_MATRIX (out untouched) if |det| < epsilon
 * (reference default epsilon: P_DOUBLE_EPS = IVP_REF_DOUBLE_EPS). */
ivp_result_t ivp_transform_real_invert(ivp_transform_r_t *out,
                                       const ivp_transform_r_t *t,
                                       double epsilon);

/* ── Shift ──────────────────────────────────────────────────────────────── */

/* Shift in world space: pos += v */
static inline void ivp_transform_shift_ws(ivp_transform_r_t *t,
                                            const ivp_vec3r_t *v) {
    ivp_vec3r_add(&t->pos, &t->pos, v);
}

/* Shift in object space: pos += rot * v */
static inline void ivp_transform_shift_os(ivp_transform_r_t *t,
                                            const ivp_vec3r_t *v) {
    ivp_vec3r_t ws;
    ivp_mat3r_vmult(&ws, &t->rot, v);
    ivp_vec3r_add(&t->pos, &t->pos, &ws);
}

/* ── Construction ───────────────────────────────────────────────────────── */

/* IVP_U_Matrix::init_rot_multiple(angles, factor): Euler rotation
 * m = m_k[1] * m_k[0] * m_k[2] (gamma about y, beta about x, alpha about
 * z) of angles*factor, zero shift. */
void ivp_transform_init_rot_multiple(ivp_transform_r_t *t, const ivp_vec3r_t *angles,
                                     double factor);

/* IVP_U_Matrix::init_columns4 / init_rows4 (shift may be NULL → zero) */
void ivp_transform_init_columns(ivp_transform_r_t *t, const ivp_vec3r_t *v0,
                                const ivp_vec3r_t *v1, const ivp_vec3r_t *v2,
                                const ivp_vec3r_t *shift);
void ivp_transform_init_rows(ivp_transform_r_t *t, const ivp_vec3r_t *v0,
                             const ivp_vec3r_t *v1, const ivp_vec3r_t *v2,
                             const ivp_vec3r_t *shift);

/* IVP_U_Matrix::get_4x4_column_major (OpenGL layout) */
void ivp_transform_get_4x4_column_major(const ivp_transform_r_t *t, float out[16]);

/* ── Interpolation ──────────────────────────────────────────────────────── */

/* IVP_U_Matrix::interpolate(m1, m2, f): element-wise linear interpolation
 * of the rotation rows and the shift (this = m1*(1-f) + m2*f).  The result
 * is generally NOT orthonormal (reference behaviour). */
void ivp_transform_interpolate(ivp_transform_r_t *out,
                                const ivp_transform_r_t *t1,
                                const ivp_transform_r_t *t2,
                                ivp_real_t f);

/* libivp helper (no reference equivalent): position linear, rotation via
 * quaternion nlerp (stays orthonormal, also between opposite orientations). */
void ivp_transform_interpolate_quat(ivp_transform_r_t *out,
                                    const ivp_transform_r_t *t1,
                                    const ivp_transform_r_t *t2,
                                    ivp_real_t f);

/* ── Distance ───────────────────────────────────────────────────────────── */

/* Quadratic distance (position + rotation elements) — quad_distance_to */
ivp_real_t ivp_transform_quad_distance(const ivp_transform_r_t *a,
                                        const ivp_transform_r_t *b);

#ifdef __cplusplus
}
#endif

#endif /* IVP_TRANSFORM_H */
