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

/* ── Orthogonalize / Normalize ──────────────────────────────────────────── */

/* Normalize all rows to unit length */
ivp_result_t ivp_mat3r_normalize(ivp_mat3r_t *m);

/* Orthogonalize (Gram-Schmidt, keeping row 2) */
void ivp_mat3r_orthogonize(ivp_mat3r_t *m);

/* Both: orthogonize + normalize */
ivp_result_t ivp_mat3r_orthonormalize(ivp_mat3r_t *m);

/* ── Determinant / Inversion ────────────────────────────────────────────── */

ivp_real_t ivp_mat3r_determinant(const ivp_mat3r_t *m);

/* General inverse (using determinant). Returns error if singular. */
ivp_result_t ivp_mat3r_invert(ivp_mat3r_t *out, const ivp_mat3r_t *m);

/* ── Rotation construction ──────────────────────────────────────────────── */

/* Init as rotation around a given axis */
void ivp_mat3r_init_rotation(ivp_mat3r_t *m, ivp_coord_index_t axis, ivp_real_t angle);

/* Quadratic rotation distance (sum of squared differences) */
ivp_real_t ivp_mat3r_quad_rot_distance(const ivp_mat3r_t *a, const ivp_mat3r_t *b);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MAT3_H */
