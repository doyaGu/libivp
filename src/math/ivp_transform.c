/* ivp_transform.c — Rigid transform implementation */
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_quat.h>

void ivp_transform_identity(ivp_transform_r_t *t) {
    ivp_mat3r_identity(&t->rot);
    ivp_vec3r_set_zero(&t->pos);
}

void ivp_transform_compose(ivp_transform_r_t *out,
                            const ivp_transform_r_t *a,
                            const ivp_transform_r_t *b) {
    /* out.rot = a.rot * b.rot */
    ivp_mat3r_mult(&out->rot, &a->rot, &b->rot);
    /* out.pos = a.rot * b.pos + a.pos */
    ivp_mat3r_vmult(&out->pos, &a->rot, &b->pos);
    ivp_vec3r_add(&out->pos, &out->pos, &a->pos);
}

void ivp_transform_inv_compose(ivp_transform_r_t *out,
                                const ivp_transform_r_t *a,
                                const ivp_transform_r_t *b) {
    /* out = inv(a) * b
     * out.rot = transpose(a.rot) * b.rot
     * out.pos = transpose(a.rot) * (b.pos - a.pos) */
    ivp_mat3r_imult(&out->rot, &a->rot, &b->rot);
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff, &b->pos, &a->pos);
    ivp_mat3r_vimult(&out->pos, &a->rot, &diff);
}

void ivp_transform_invert(ivp_transform_r_t *out, const ivp_transform_r_t *t) {
    /* For orthonormal transforms:
     * inv(rot) = transpose(rot)
     * inv(pos) = -transpose(rot) * pos */
    ivp_mat3r_transpose(&out->rot, &t->rot);
    ivp_vec3r_t neg_pos;
    ivp_vec3r_negate(&neg_pos, &t->pos);
    ivp_mat3r_vmult(&out->pos, &out->rot, &neg_pos);
}

void ivp_transform_interpolate(ivp_transform_r_t *out,
                                const ivp_transform_r_t *t1,
                                const ivp_transform_r_t *t2,
                                ivp_real_t f) {
    /* Interpolate position linearly */
    ivp_vec3r_lerp(&out->pos, &t1->pos, &t2->pos, f);

    /* Interpolate rotation in quaternion space to avoid matrix degeneracy
     * near opposite orientations (e.g. identity <-> 180 degree rotation). */
    ivp_quatr_t q1, q2, q;
    ivp_quat_from_mat3(&q1, &t1->rot);
    ivp_quat_from_mat3(&q2, &t2->rot);

    if (ivp_quat_normalize(&q1) != IVP_OK || ivp_quat_normalize(&q2) != IVP_OK) {
        /* Fallback for invalid input transforms. */
        out->rot = (f <= IVP_REAL(0.5)) ? t1->rot : t2->rot;
        return;
    }

    ivp_quat_nlerp(&q, &q1, &q2, f);
    ivp_quat_to_mat3(&q, &out->rot);
}

ivp_real_t ivp_transform_quad_distance(const ivp_transform_r_t *a,
                                        const ivp_transform_r_t *b) {
    return ivp_vec3r_quad_distance(&a->pos, &b->pos) +
           ivp_mat3r_quad_rot_distance(&a->rot, &b->rot);
}
