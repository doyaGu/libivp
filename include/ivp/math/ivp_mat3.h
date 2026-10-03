/* ivp_mat3.h — 3×3 rotation matrix for IVP C17
 *
 * Row-major storage (rows[3] of ivp_vec3r_t), matching original IVP layout.
 */
#ifndef IVP_MAT3_H
#define IVP_MAT3_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/math/ivp_vec3.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Type ───────────────────────────────────────────────────────────────── */

typedef struct ivp_mat3r {
    ivp_vec3r_t rows[3];   /* row-major: rows[row].k[col] */
} ivp_mat3r_t;

/* ── Element access ─────────────────────────────────────────────────────── */

static inline ivp_real_t ivp_mat3r_get(const ivp_mat3r_t *m, int row, int col) {
    return m->rows[row].k[col];
}

static inline void ivp_mat3r_set(ivp_mat3r_t *m, int row, int col, ivp_real_t val) {
    m->rows[row].k[col] = val;
}

/* ── Construction ───────────────────────────────────────────────────────── */

/* Set to identity */
void ivp_mat3r_identity(ivp_mat3r_t *m);

/* Set from 3 row vectors */
static inline void ivp_mat3r_set_rows(ivp_mat3r_t *m,
                                       const ivp_vec3r_t *r0,
                                       const ivp_vec3r_t *r1,
                                       const ivp_vec3r_t *r2) {
    m->rows[0] = *r0;
    m->rows[1] = *r1;
    m->rows[2] = *r2;
}

/* Set from 3 column vectors */
void ivp_mat3r_set_cols(ivp_mat3r_t *m,
                         const ivp_vec3r_t *c0,
                         const ivp_vec3r_t *c1,
                         const ivp_vec3r_t *c2);

/* ── Row / Column extraction ────────────────────────────────────────────── */

static inline void ivp_mat3r_get_row(const ivp_mat3r_t *m, int row, ivp_vec3r_t *out) {
    *out = m->rows[row];
}

void ivp_mat3r_get_col(const ivp_mat3r_t *m, int col, ivp_vec3r_t *out);

/* ── Matrix × Matrix ───────────────────────────────────────────────────── */

/* out = a * b */
void ivp_mat3r_mult(ivp_mat3r_t *out, const ivp_mat3r_t *a, const ivp_mat3r_t *b);

/* out = transpose(a) * b  (inverse for orthonormal a) */
void ivp_mat3r_imult(ivp_mat3r_t *out, const ivp_mat3r_t *a, const ivp_mat3r_t *b);

/* ── Matrix × Vector ───────────────────────────────────────────────────── */

/* out = m * v */
static inline void ivp_mat3r_vmult(ivp_vec3r_t *out, const ivp_mat3r_t *m,
                                    const ivp_vec3r_t *v) {
    ivp_real_t x = m->rows[0].k[0]*v->k[0] + m->rows[0].k[1]*v->k[1] + m->rows[0].k[2]*v->k[2];
    ivp_real_t y = m->rows[1].k[0]*v->k[0] + m->rows[1].k[1]*v->k[1] + m->rows[1].k[2]*v->k[2];
    ivp_real_t z = m->rows[2].k[0]*v->k[0] + m->rows[2].k[1]*v->k[1] + m->rows[2].k[2]*v->k[2];
    out->k[0] = x; out->k[1] = y; out->k[2] = z;
}

/* out = transpose(m) * v  (inverse rotation) */
static inline void ivp_mat3r_vimult(ivp_vec3r_t *out, const ivp_mat3r_t *m,
                                     const ivp_vec3r_t *v) {
    ivp_real_t x = m->rows[0].k[0]*v->k[0] + m->rows[1].k[0]*v->k[1] + m->rows[2].k[0]*v->k[2];
    ivp_real_t y = m->rows[0].k[1]*v->k[0] + m->rows[1].k[1]*v->k[1] + m->rows[2].k[1]*v->k[2];
    ivp_real_t z = m->rows[0].k[2]*v->k[0] + m->rows[1].k[2]*v->k[1] + m->rows[2].k[2]*v->k[2];
    out->k[0] = x; out->k[1] = y; out->k[2] = z;
}

/* ── Transpose ──────────────────────────────────────────────────────────── */

void ivp_mat3r_transpose(ivp_mat3r_t *out, const ivp_mat3r_t *m);

/* In-place transpose */
void ivp_mat3r_transpose_inplace(ivp_mat3r_t *m);

/* out = a * transpose(b)  (IVP_U_Matrix3::mi2mult3: this * mb^-1 for
 * orthonormal mb).  out may alias a or b. */
void ivp_mat3r_mi2mult(ivp_mat3r_t *out, const ivp_mat3r_t *a, const ivp_mat3r_t *b);

/* IVP_U_Matrix3::set_row / set_col */
static inline void ivp_mat3r_set_row(ivp_mat3r_t *m, int row, const ivp_vec3r_t *v) {
    m->rows[row] = *v;
}
static inline void ivp_mat3r_set_col(ivp_mat3r_t *m, int col, const ivp_vec3r_t *v) {
    m->rows[0].k[col] = v->k[0];
    m->rows[1].k[col] = v->k[1];
    m->rows[2].k[col] = v->k[2];
}

/* ── Orthogonalize / Normalize ──────────────────────────────────────────── *
 * Reference: IVP_U_Matrix3 (ivu_linear.cxx).  Matrix rows are IVP_U_Point
 * (double) in the reference; these helpers evaluate in double. */

/* IVP_U_Matrix3::normize: IVP_U_Point::normize() on every ROW
 * (isqrt(ql, 3)); stops at the first row with quad_length < P_DOUBLE_EPS
 * and returns IVP_ERR_NORMALIZATION_FAILED. */
ivp_result_t ivp_mat3r_normalize(ivp_mat3r_t *m);

/* IVP_U_Matrix3::orthogonize: works on COLUMNS and keeps the z column:
 *   x = y × z,  y = z × x,  columns = (x, y, z)  — a right-handed system
 * (det > 0 for any input with det > 0; identity stays identity). */
void ivp_mat3r_orthogonize(ivp_mat3r_t *m);

/* IVP_U_Matrix3::orthonormize: orthogonize() then normize() (rows). */
ivp_result_t ivp_mat3r_orthonormalize(ivp_mat3r_t *m);
static inline ivp_result_t ivp_mat3r_orthonormize(ivp_mat3r_t *m) {
    return ivp_mat3r_orthonormalize(m);
}

/* IVP_U_Matrix3::calc_eigen_vector: one eigenvector of m for the given
 * eigen value (the function assumes one exists).  Returns the degree of
 * freedom of the eigenvector (1, 2 or 3; for 3 the result is (0,0,1)). */
int ivp_mat3r_calc_eigen_vector(const ivp_mat3r_t *m, double eigen_value,
                                ivp_vec3r_t *eigen_vector_out);

/* IVP_U_Matrix3::init_normized3_col(v, index): orthonormal matrix whose
 * column `index` is normize(v); the other columns follow the reference
 * construction v2 = normize(v0.y, v0.z - v0.x, -v0.y) (fallback
 * (v0.z, -v0.z, v0.y - v0.x)), v1 = v2 × v0. */
void ivp_mat3r_init_normized_col(ivp_mat3r_t *m, const ivp_vec3r_t *v,
                                 ivp_coord_index_t index);
/* Same, but the rows are set (IVP_U_Matrix3::init_normized3_row). */
void ivp_mat3r_init_normized_row(ivp_mat3r_t *m, const ivp_vec3r_t *v,
                                 ivp_coord_index_t index);
/* init_normized3_col(vb, index_b, vc): column index_b = normize(vb),
 * v2 = normize(v0 × vc), v1 = v2 × v0. */
void ivp_mat3r_init_normized_col2(ivp_mat3r_t *m, const ivp_vec3r_t *vb,
                                  ivp_coord_index_t index_b, const ivp_vec3r_t *vc);
/* init_normized3_row(vb, index_b, vc) */
void ivp_mat3r_init_normized_row2(ivp_mat3r_t *m, const ivp_vec3r_t *vb,
                                  ivp_coord_index_t index_b, const ivp_vec3r_t *vc);

/* IVP_U_Matrix3::get_angles: Euler angles (gamma about y, beta about x,
 * alpha about z) of an orthonormized copy of m.  Returns
 * IVP_ERR_NORMALIZATION_FAILED if the copy cannot be orthonormized. */
ivp_result_t ivp_mat3r_get_angles(const ivp_mat3r_t *m, float *alpha,
                                  float *beta, float *gamma);

/* ── Determinant / Inversion ────────────────────────────────────────────── */

ivp_real_t ivp_mat3r_determinant(const ivp_mat3r_t *m);

/* General inverse (using determinant). Returns error if singular. */
ivp_result_t ivp_mat3r_invert(ivp_mat3r_t *out, const ivp_mat3r_t *m);

/* ── Rotation construction ──────────────────────────────────────────────── */

/* IVP_U_Matrix3::init_rotated3(axis, angle): angle is IVP_FLOAT in the
 * reference (rounded to float, then sin/cos in double). */
void ivp_mat3r_init_rotation(ivp_mat3r_t *m, ivp_coord_index_t axis, ivp_real_t angle);

/* IVP_U_Matrix3::quad_rot_distance_to: sum of the per-row quad distances */
ivp_real_t ivp_mat3r_quad_rot_distance(const ivp_mat3r_t *a, const ivp_mat3r_t *b);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MAT3_H */
