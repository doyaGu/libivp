/* ivp_transform.c — Rigid transform implementation
 *
 * Reference: IVP_U_Matrix (ivu_linear.cxx, ivu_matrix_macros.hxx).
 */
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_quat.h>
#include <string.h>

void ivp_transform_identity(ivp_transform_r_t *t) {
    ivp_mat3r_identity(&t->rot);
    ivp_vec3r_set_zero(&t->pos);
}

void ivp_transform_compose(ivp_transform_r_t *out,
                            const ivp_transform_r_t *a,
                            const ivp_transform_r_t *b) {
    /* inline_mmult4: v_shift_korr = this->vmult3(mb->vv) (computed before
     * the rotation is overwritten, so in-place use is allowed);
     * m_out->rot = this->rot * mb->rot; m_out->vv = this->vv + v_shift_korr */
    ivp_vec3r_t v_shift_korr;
    ivp_mat3r_vmult(&v_shift_korr, &a->rot, &b->pos);
    ivp_vec3r_t a_pos = a->pos;
    ivp_mat3r_mult(&out->rot, &a->rot, &b->rot);
    ivp_vec3r_add(&out->pos, &a_pos, &v_shift_korr);
}

void ivp_transform_inv_compose(ivp_transform_r_t *out,
                                const ivp_transform_r_t *a,
                                const ivp_transform_r_t *b) {
    /* inline_mimult4: v_shift_korr = mb->vv - this->vv;
     * m_out->vv = this->vimult3(v_shift_korr); m_out->rot = this^T * mb */
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff, &b->pos, &a->pos);
    ivp_vec3r_t new_pos;
    ivp_mat3r_vimult(&new_pos, &a->rot, &diff);
    ivp_mat3r_imult(&out->rot, &a->rot, &b->rot);
    out->pos = new_pos;
}

void ivp_transform_mi2mult(ivp_transform_r_t *out,
                           const ivp_transform_r_t *a,
                           const ivp_transform_r_t *b) {
    /* m_out = this * mb^T:  mi2mult3, then v_shift_korr = m_out->vmult3(mb->vv),
     * m_out->vv = this->vv - v_shift_korr */
    ivp_vec3r_t a_pos = a->pos;
    ivp_vec3r_t b_pos = b->pos;
    ivp_mat3r_mi2mult(&out->rot, &a->rot, &b->rot);
    ivp_vec3r_t v_shift_korr;
    ivp_mat3r_vmult(&v_shift_korr, &out->rot, &b_pos);
    ivp_vec3r_sub(&out->pos, &a_pos, &v_shift_korr);
}

static void build_m_rot(ivp_transform_r_t *m_rot, ivp_coord_index_t axis, float angle,
                        bool invers) {
    static const int pm[5] = {1, 2, 0, 1, 2};
    const int *p = &pm[axis];
    double SA = sin((double)angle);
    double CA = cos((double)angle);
    memset(m_rot, 0, sizeof(*m_rot)); /* P_MEM_CLEAR */
    m_rot->rot.rows[p[2]].k[p[2]] = IVP_REAL(1.0);
    m_rot->rot.rows[p[0]].k[p[0]] = (ivp_real_t)CA;
    m_rot->rot.rows[p[1]].k[p[1]] = (ivp_real_t)CA;
    if (!invers) {
        m_rot->rot.rows[p[0]].k[p[1]] = (ivp_real_t)-SA;
        m_rot->rot.rows[p[1]].k[p[0]] = (ivp_real_t)SA;
    } else {
        m_rot->rot.rows[p[0]].k[p[1]] = (ivp_real_t)SA;
        m_rot->rot.rows[p[1]].k[p[0]] = (ivp_real_t)-SA;
    }
}

void ivp_transform_rotate(ivp_transform_r_t *out, const ivp_transform_r_t *t,
                          ivp_coord_index_t axis, float angle) {
    ivp_transform_r_t m_rot;
    build_m_rot(&m_rot, axis, angle, false);
    ivp_transform_compose(out, &m_rot, t); /* m_rot.mmult4(this, m_out) */
}

void ivp_transform_rotate_invers(ivp_transform_r_t *out, const ivp_transform_r_t *t,
                                 ivp_coord_index_t axis, float angle) {
    ivp_transform_r_t m_rot;
    build_m_rot(&m_rot, axis, angle, true);
    ivp_transform_mi2mult(out, t, &m_rot); /* this->mi2mult4(&m_rot, m_out) */
}

void ivp_transform_invert(ivp_transform_r_t *out, const ivp_transform_r_t *t) {
    /* For orthonormal transforms:
     * inv(rot) = transpose(rot)
     * inv(pos) = -transpose(rot) * pos */
    ivp_vec3r_t neg_pos;
    ivp_vec3r_negate(&neg_pos, &t->pos);
    ivp_mat3r_transpose(&out->rot, &t->rot);
    ivp_mat3r_vmult(&out->pos, &out->rot, &neg_pos);
}

static inline void cross_d(double *out, const double *v1, const double *v2) {
    /* IVP_U_Point::inline_calc_cross_product */
    double a = v1[1] * v2[2];
    double a2 = v2[1] * v1[2];
    double b = v1[2] * v2[0];
    double b2 = v2[2] * v1[0];
    a -= a2;
    double c = v1[0] * v2[1];
    double c2 = v2[0] * v1[1];
    b -= b2;
    out[0] = a;
    out[1] = b;
    c -= c2;
    out[2] = c;
}

ivp_result_t ivp_transform_real_invert(ivp_transform_r_t *out,
                                       const ivp_transform_r_t *t,
                                       double epsilon) {
    double rows[3][3], vv[3];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) rows[r][c] = (double)t->rot.rows[r].k[c];
        vv[r] = (double)t->pos.k[r];
    }
    double r0[3], r1[3], r2[3];
    cross_d(r0, rows[1], rows[2]);
    cross_d(r1, rows[2], rows[0]);
    cross_d(r2, rows[0], rows[1]);

    double D = rows[0][0] * r0[0] + rows[0][1] * r0[1] + rows[0][2] * r0[2];
    if (fabs(D) < epsilon) {
        return IVP_ERR_SINGULAR_MATRIX; /* cannot invert, may happen */
    }
    double DI = 1.0 / D;
    for (int i = 0; i < 3; i++) {
        r0[i] *= DI;
        r1[i] *= DI;
        r2[i] *= DI;
    }
    double m[3][3];
    for (int i = 0; i < 3; i++) {
        m[i][0] = r0[i]; /* set_col(IVP_INDEX_X, &r0) */
        m[i][1] = r1[i];
        m[i][2] = r2[i];
    }
    /* p = this->vmult3(&vv); vv = -p */
    double p[3];
    for (int i = 0; i < 3; i++) {
        p[i] = m[i][0] * vv[0] + m[i][1] * vv[1] + m[i][2] * vv[2];
    }
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) out->rot.rows[r].k[c] = (ivp_real_t)m[r][c];
        out->pos.k[r] = (ivp_real_t)-p[r];
    }
    return IVP_OK;
}

void ivp_transform_init_rot_multiple(ivp_transform_r_t *t, const ivp_vec3r_t *angles,
                                     double factor) {
    /* angles.k[0] = beta, [1] = gamma, [2] = alpha;
     * mout = m_gamma * m_beta * m_alpha = m_k[1] * m_k[0] * m_k[2] */
    double a0 = (double)angles->k[0];
    double a1 = (double)angles->k[1];
    double a2 = (double)angles->k[2];
    double x, y;
    if (a0 != 0.0) {
        x = cos(a0 * factor);
        y = sin(a0 * factor);
    } else {
        x = 1.0;
        y = 0.0;
    }
    double u, w;
    if (a1 != 0.0) {
        u = cos(a1 * factor);
        w = sin(a1 * factor);
    } else {
        u = 1.0;
        w = 0.0;
    }
    double c, s;
    if (a2 != 0.0) {
        c = cos(a2 * factor);
        s = sin(a2 * factor);
    } else {
        c = 1.0;
        s = 0.0;
    }

    ivp_vec3r_set_zero(&t->pos);

    t->rot.rows[0].k[0] = (ivp_real_t)(u * c + w * y * s);
    t->rot.rows[0].k[1] = (ivp_real_t)(w * y * c - u * s);
    t->rot.rows[0].k[2] = (ivp_real_t)(w * x);
    t->rot.rows[1].k[0] = (ivp_real_t)(x * s);
    t->rot.rows[1].k[1] = (ivp_real_t)(x * c);
    t->rot.rows[1].k[2] = (ivp_real_t)(-y);
    t->rot.rows[2].k[0] = (ivp_real_t)(y * u * s - w * c);
    t->rot.rows[2].k[1] = (ivp_real_t)(u * y * c + w * s);
    t->rot.rows[2].k[2] = (ivp_real_t)(u * x);
}

void ivp_transform_init_columns(ivp_transform_r_t *t, const ivp_vec3r_t *v0,
                                const ivp_vec3r_t *v1, const ivp_vec3r_t *v2,
                                const ivp_vec3r_t *shift) {
    ivp_mat3r_set_cols(&t->rot, v0, v1, v2);
    if (shift) {
        t->pos = *shift;
    } else {
        ivp_vec3r_set_zero(&t->pos);
    }
}

void ivp_transform_init_rows(ivp_transform_r_t *t, const ivp_vec3r_t *v0,
                             const ivp_vec3r_t *v1, const ivp_vec3r_t *v2,
                             const ivp_vec3r_t *shift) {
    t->rot.rows[0] = *v0;
    t->rot.rows[1] = *v1;
    t->rot.rows[2] = *v2;
    if (shift) {
        t->pos = *shift;
    } else {
        ivp_vec3r_set_zero(&t->pos);
    }
}

void ivp_transform_get_4x4_column_major(const ivp_transform_r_t *t, float o[16]) {
    o[0] = (float)t->rot.rows[0].k[0];
    o[1] = (float)t->rot.rows[1].k[0];
    o[2] = (float)t->rot.rows[2].k[0];
    o[3] = 0.0f;

    o[4] = (float)t->rot.rows[0].k[1];
    o[5] = (float)t->rot.rows[1].k[1];
    o[6] = (float)t->rot.rows[2].k[1];
    o[7] = 0.0f;

    o[8] = (float)t->rot.rows[0].k[2];
    o[9] = (float)t->rot.rows[1].k[2];
    o[10] = (float)t->rot.rows[2].k[2];
    o[11] = 0.0f;

    o[12] = (float)t->pos.k[0];
    o[13] = (float)t->pos.k[1];
    o[14] = (float)t->pos.k[2];
    o[15] = 1.0f;
}

void ivp_transform_interpolate(ivp_transform_r_t *out,
                                const ivp_transform_r_t *t1,
                                const ivp_transform_r_t *t2,
                                ivp_real_t f) {
    /* for r: rows[r].set_interpolate(&m1->rows[r], &m2->rows[r], i_factor);
     * vv.set_interpolate(m1->vv, m2->vv, i_factor) */
    for (int r = 0; r < 3; r++) {
        ivp_vec3r_lerp(&out->rot.rows[r], &t1->rot.rows[r], &t2->rot.rows[r], f);
    }
    ivp_vec3r_lerp(&out->pos, &t1->pos, &t2->pos, f);
}

void ivp_transform_interpolate_quat(ivp_transform_r_t *out,
                                    const ivp_transform_r_t *t1,
                                    const ivp_transform_r_t *t2,
                                    ivp_real_t f) {
    /* Interpolate position linearly */
    ivp_vec3r_t pos;
    ivp_vec3r_lerp(&pos, &t1->pos, &t2->pos, f);

    /* Interpolate rotation in quaternion space to avoid matrix degeneracy
     * near opposite orientations (e.g. identity <-> 180 degree rotation). */
    ivp_quatr_t q1, q2, q;
    ivp_quat_from_mat3(&q1, &t1->rot);
    ivp_quat_from_mat3(&q2, &t2->rot);

    if (ivp_quat_normalize(&q1) != IVP_OK || ivp_quat_normalize(&q2) != IVP_OK) {
        /* Fallback for invalid input transforms. */
        out->rot = (f <= IVP_REAL(0.5)) ? t1->rot : t2->rot;
        out->pos = pos;
        return;
    }

    ivp_quat_nlerp(&q, &q1, &q2, f);
    ivp_quat_to_mat3(&q, &out->rot);
    out->pos = pos;
}

ivp_real_t ivp_transform_quad_distance(const ivp_transform_r_t *a,
                                        const ivp_transform_r_t *b) {
    /* quad_rot_distance_to(m2) + vv.quad_distance_to(&m2->vv) */
    double dx = (double)a->pos.k[0] - (double)b->pos.k[0];
    dx *= dx;
    double dy = (double)a->pos.k[1] - (double)b->pos.k[1];
    dy *= dy;
    double dz = (double)a->pos.k[2] - (double)b->pos.k[2];
    dz *= dz;
    double vv = dx + dy + dz;
    return (ivp_real_t)((double)ivp_mat3r_quad_rot_distance(&a->rot, &b->rot) + vv);
}
