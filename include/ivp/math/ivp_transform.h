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

/* ── Composition ────────────────────────────────────────────────────────── */

/* out = a * b (first b, then a) */
void ivp_transform_compose(ivp_transform_r_t *out,
                            const ivp_transform_r_t *a,
                            const ivp_transform_r_t *b);

/* out = inv(a) * b */
void ivp_transform_inv_compose(ivp_transform_r_t *out,
                                const ivp_transform_r_t *a,
                                const ivp_transform_r_t *b);

/* ── Inversion ──────────────────────────────────────────────────────────── */

/* Compute the inverse of an orthonormal transform (transpose + shift) */
void ivp_transform_invert(ivp_transform_r_t *out, const ivp_transform_r_t *t);

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

/* ── Interpolation ──────────────────────────────────────────────────────── */

/* Interpolate position linearly and rotation in quaternion space (nlerp). */
void ivp_transform_interpolate(ivp_transform_r_t *out,
                                const ivp_transform_r_t *t1,
                                const ivp_transform_r_t *t2,
                                ivp_real_t f);

/* ── Distance ───────────────────────────────────────────────────────────── */

/* Quadratic distance (position + rotation elements) */
ivp_real_t ivp_transform_quad_distance(const ivp_transform_r_t *a,
                                        const ivp_transform_r_t *b);

#ifdef __cplusplus
}
#endif

#endif /* IVP_TRANSFORM_H */
