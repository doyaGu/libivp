/* ivp_mat3.c — 3×3 rotation matrix implementation */
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>
#include <string.h>

void ivp_mat3r_identity(ivp_mat3r_t *m) {
    memset(m, 0, sizeof(*m));
    m->rows[0].k[0] = IVP_REAL(1.0);
    m->rows[1].k[1] = IVP_REAL(1.0);
    m->rows[2].k[2] = IVP_REAL(1.0);
}

void ivp_mat3r_set_cols(ivp_mat3r_t *m,
                         const ivp_vec3r_t *c0,
                         const ivp_vec3r_t *c1,
                         const ivp_vec3r_t *c2) {
    m->rows[0].k[0] = c0->k[0]; m->rows[0].k[1] = c1->k[0]; m->rows[0].k[2] = c2->k[0];
    m->rows[1].k[0] = c0->k[1]; m->rows[1].k[1] = c1->k[1]; m->rows[1].k[2] = c2->k[1];
    m->rows[2].k[0] = c0->k[2]; m->rows[2].k[1] = c1->k[2]; m->rows[2].k[2] = c2->k[2];
}

void ivp_mat3r_get_col(const ivp_mat3r_t *m, int col, ivp_vec3r_t *out) {
    out->k[0] = m->rows[0].k[col];
    out->k[1] = m->rows[1].k[col];
    out->k[2] = m->rows[2].k[col];
}

void ivp_mat3r_mult(ivp_mat3r_t *out, const ivp_mat3r_t *a, const ivp_mat3r_t *b) {
    ivp_mat3r_t tmp;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            tmp.rows[r].k[c] =
                a->rows[r].k[0] * b->rows[0].k[c] +
                a->rows[r].k[1] * b->rows[1].k[c] +
                a->rows[r].k[2] * b->rows[2].k[c];
        }
    }
    *out = tmp;
}

void ivp_mat3r_imult(ivp_mat3r_t *out, const ivp_mat3r_t *a, const ivp_mat3r_t *b) {
    /* out = transpose(a) * b */
    ivp_mat3r_t tmp;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            tmp.rows[r].k[c] =
                a->rows[0].k[r] * b->rows[0].k[c] +
                a->rows[1].k[r] * b->rows[1].k[c] +
                a->rows[2].k[r] * b->rows[2].k[c];
        }
    }
    *out = tmp;
}

void ivp_mat3r_transpose(ivp_mat3r_t *out, const ivp_mat3r_t *m) {
    ivp_mat3r_t tmp;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            tmp.rows[r].k[c] = m->rows[c].k[r];
    *out = tmp;
}

void ivp_mat3r_transpose_inplace(ivp_mat3r_t *m) {
    ivp_real_t t;
    t = m->rows[0].k[1]; m->rows[0].k[1] = m->rows[1].k[0]; m->rows[1].k[0] = t;
    t = m->rows[0].k[2]; m->rows[0].k[2] = m->rows[2].k[0]; m->rows[2].k[0] = t;
    t = m->rows[1].k[2]; m->rows[1].k[2] = m->rows[2].k[1]; m->rows[2].k[1] = t;
}

ivp_result_t ivp_mat3r_normalize(ivp_mat3r_t *m) {
    for (int r = 0; r < 3; r++) {
        ivp_result_t res = ivp_vec3r_normalize(&m->rows[r]);
        if (res != IVP_OK) return res;
    }
    return IVP_OK;
}

void ivp_mat3r_orthogonize(ivp_mat3r_t *m) {
    /* Keep row 2 (z-axis), orthogonalize others via Gram-Schmidt */
    /* row1 = row1 - dot(row1, row2) * row2 */
    ivp_real_t d12 = ivp_vec3r_dot(&m->rows[1], &m->rows[2]);
    ivp_vec3r_add_scaled(&m->rows[1], &m->rows[2], -d12);

    /* row0 = row2 x row1 (right-hand system) */
    ivp_vec3r_cross(&m->rows[0], &m->rows[2], &m->rows[1]);
}

ivp_result_t ivp_mat3r_orthonormalize(ivp_mat3r_t *m) {
    ivp_mat3r_orthogonize(m);
    return ivp_mat3r_normalize(m);
}

ivp_real_t ivp_mat3r_determinant(const ivp_mat3r_t *m) {
    return m->rows[0].k[0] * (m->rows[1].k[1]*m->rows[2].k[2] - m->rows[1].k[2]*m->rows[2].k[1])
         - m->rows[0].k[1] * (m->rows[1].k[0]*m->rows[2].k[2] - m->rows[1].k[2]*m->rows[2].k[0])
         + m->rows[0].k[2] * (m->rows[1].k[0]*m->rows[2].k[1] - m->rows[1].k[1]*m->rows[2].k[0]);
}

ivp_result_t ivp_mat3r_invert(ivp_mat3r_t *out, const ivp_mat3r_t *m) {
    ivp_real_t det = ivp_mat3r_determinant(m);
    if (ivp_fabs(det) < IVP_DOUBLE_EPS) return IVP_ERR_SINGULAR_MATRIX;

    ivp_real_t inv_det = IVP_REAL(1.0) / det;

    ivp_mat3r_t tmp;
    tmp.rows[0].k[0] = (m->rows[1].k[1]*m->rows[2].k[2] - m->rows[1].k[2]*m->rows[2].k[1]) * inv_det;
    tmp.rows[0].k[1] = (m->rows[0].k[2]*m->rows[2].k[1] - m->rows[0].k[1]*m->rows[2].k[2]) * inv_det;
    tmp.rows[0].k[2] = (m->rows[0].k[1]*m->rows[1].k[2] - m->rows[0].k[2]*m->rows[1].k[1]) * inv_det;

    tmp.rows[1].k[0] = (m->rows[1].k[2]*m->rows[2].k[0] - m->rows[1].k[0]*m->rows[2].k[2]) * inv_det;
    tmp.rows[1].k[1] = (m->rows[0].k[0]*m->rows[2].k[2] - m->rows[0].k[2]*m->rows[2].k[0]) * inv_det;
    tmp.rows[1].k[2] = (m->rows[0].k[2]*m->rows[1].k[0] - m->rows[0].k[0]*m->rows[1].k[2]) * inv_det;

    tmp.rows[2].k[0] = (m->rows[1].k[0]*m->rows[2].k[1] - m->rows[1].k[1]*m->rows[2].k[0]) * inv_det;
    tmp.rows[2].k[1] = (m->rows[0].k[1]*m->rows[2].k[0] - m->rows[0].k[0]*m->rows[2].k[1]) * inv_det;
    tmp.rows[2].k[2] = (m->rows[0].k[0]*m->rows[1].k[1] - m->rows[0].k[1]*m->rows[1].k[0]) * inv_det;

    *out = tmp;
    return IVP_OK;
}

void ivp_mat3r_init_rotation(ivp_mat3r_t *m, ivp_coord_index_t axis, ivp_real_t angle) {
    ivp_real_t c = ivp_cos(angle);
    ivp_real_t s = ivp_sin(angle);
    ivp_mat3r_identity(m);

    switch (axis) {
        case IVP_INDEX_X:
            m->rows[1].k[1] =  c; m->rows[1].k[2] = -s;
            m->rows[2].k[1] =  s; m->rows[2].k[2] =  c;
            break;
        case IVP_INDEX_Y:
            m->rows[0].k[0] =  c; m->rows[0].k[2] =  s;
            m->rows[2].k[0] = -s; m->rows[2].k[2] =  c;
            break;
        case IVP_INDEX_Z:
            m->rows[0].k[0] =  c; m->rows[0].k[1] = -s;
            m->rows[1].k[0] =  s; m->rows[1].k[1] =  c;
            break;
    }
}

ivp_real_t ivp_mat3r_quad_rot_distance(const ivp_mat3r_t *a, const ivp_mat3r_t *b) {
    ivp_real_t sum = IVP_REAL(0.0);
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            ivp_real_t d = a->rows[r].k[c] - b->rows[r].k[c];
            sum += d * d;
        }
    }
    return sum;
}
