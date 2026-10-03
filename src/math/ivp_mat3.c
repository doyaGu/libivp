/* ivp_mat3.c — 3×3 rotation matrix implementation
 *
 * Reference: IVP_U_Matrix3 (ivu_linear.cxx, ivu_matrix_macros.hxx).
 * Reference matrix rows are IVP_U_Point (double); the non-inline helpers
 * below evaluate in double and round to ivp_real_t on store.
 */
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
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

/* ── Double working copies (reference rows are IVP_U_Point = double) ──── */

typedef struct mat3d {
    double m[3][3]; /* [row][col] */
} mat3d_t;

static inline void m3d_load(mat3d_t *d, const ivp_mat3r_t *m) {
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            d->m[r][c] = (double)m->rows[r].k[c];
}

static inline void m3d_store(ivp_mat3r_t *m, const mat3d_t *d) {
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            m->rows[r].k[c] = (ivp_real_t)d->m[r][c];
}

static inline void v3d_load(ivp_vec3d_t *d, const ivp_vec3r_t *v) {
    d->k[0] = (double)v->k[0];
    d->k[1] = (double)v->k[1];
    d->k[2] = (double)v->k[2];
}

/* IVP_U_Point::normize on a double row */
static ivp_result_t row_normize(double *k) {
    double length = k[0] * k[0] + k[1] * k[1] + k[2] * k[2];
    if (length < IVP_REF_DOUBLE_EPS) return IVP_ERR_NORMALIZATION_FAILED;
    double f = ivp_isqrt_double(length, 3);
    k[0] *= f;
    k[1] *= f;
    k[2] *= f;
    return IVP_OK;
}

static ivp_result_t m3d_normize(mat3d_t *d) {
    for (int r = 0; r < 3; r++) {
        ivp_result_t res = row_normize(d->m[r]);
        if (res != IVP_OK) return res;
    }
    return IVP_OK;
}

static void m3d_orthogonize(mat3d_t *d) {
    ivp_vec3d_t vz, vy, v2, v3;
    /* z-vector shall remain */
    vz.k[0] = d->m[0][2]; vz.k[1] = d->m[1][2]; vz.k[2] = d->m[2][2];
    /* y-vector */
    vy.k[0] = d->m[0][1]; vy.k[1] = d->m[1][1]; vy.k[2] = d->m[2][1];

    ivp_vec3d_cross(&v2, &vy, &vz);
    ivp_vec3d_cross(&v3, &vz, &v2);

    /* init_columns3(&v2, &v3, &vz): vz, v2, v3 are in a right system */
    for (int r = 0; r < 3; r++) {
        d->m[r][0] = v2.k[r];
        d->m[r][1] = v3.k[r];
        d->m[r][2] = vz.k[r];
    }
}

/* IVP_U_Matrix3::inline_vimult3 on doubles: out_j = sum_i m(i,j) * p_i */
static inline void m3d_vimult(const mat3d_t *mb, const double *p_in, double *p_out) {
    double a = mb->m[0][0] * p_in[0] + mb->m[1][0] * p_in[1] + mb->m[2][0] * p_in[2];
    double b = mb->m[0][1] * p_in[0] + mb->m[1][1] * p_in[1] + mb->m[2][1] * p_in[2];
    double c = mb->m[0][2] * p_in[0] + mb->m[1][2] * p_in[1] + mb->m[2][2] * p_in[2];
    p_out[0] = a;
    p_out[1] = b;
    p_out[2] = c;
}

/* IVP_U_Matrix3::inline_mimult3: m_out = this^T * mb */
static void m3d_mimult(const mat3d_t *self, const mat3d_t *mb, mat3d_t *m_out) {
    double col[3][3];
    for (int c = 0; c < 3; c++) {
        col[c][0] = self->m[0][c];
        col[c][1] = self->m[1][c];
        col[c][2] = self->m[2][c];
    }
    for (int c = 0; c < 3; c++) {
        m3d_vimult(mb, col[c], col[c]);
    }
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            m_out->m[r][c] = col[r][c];
}

/* em.init(); em.rotate(axis, angle, &m_out) of IVP_U_Matrix:
 * m_out = m_rot * em via inline_mmult3 (reproduces the signs of zeros). */
static void m3d_identity_rotate(ivp_coord_index_t axis, float angle, mat3d_t *m_out) {
    static const int pm[5] = {1, 2, 0, 1, 2};
    const int *p = &pm[axis];
    double SA = sin((double)angle);
    double CA = cos((double)angle);
    mat3d_t m_rot;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            m_rot.m[r][c] = 0.0;
    m_rot.m[p[2]][p[2]] = 1;
    m_rot.m[p[0]][p[0]] = CA;
    m_rot.m[p[1]][p[1]] = CA;
    m_rot.m[p[0]][p[1]] = -SA;
    m_rot.m[p[1]][p[0]] = SA;

    mat3d_t em;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            em.m[r][c] = (r == c) ? 1.0 : 0.0;
    /* inline_mmult3: row[r] = em->inline_vimult3(m_rot.rows[r]) */
    for (int r = 0; r < 3; r++) {
        m3d_vimult(&em, m_rot.m[r], m_out->m[r]);
    }
}

ivp_result_t ivp_mat3r_normalize(ivp_mat3r_t *m) {
    for (int r = 0; r < 3; r++) {
        double k[3] = { (double)m->rows[r].k[0], (double)m->rows[r].k[1],
                        (double)m->rows[r].k[2] };
        ivp_result_t res = row_normize(k);
        if (res != IVP_OK) return res; /* earlier rows stay normized */
        m->rows[r].k[0] = (ivp_real_t)k[0];
        m->rows[r].k[1] = (ivp_real_t)k[1];
        m->rows[r].k[2] = (ivp_real_t)k[2];
    }
    return IVP_OK;
}

void ivp_mat3r_orthogonize(ivp_mat3r_t *m) {
    mat3d_t d;
    m3d_load(&d, m);
    m3d_orthogonize(&d);
    m3d_store(m, &d);
}

ivp_result_t ivp_mat3r_orthonormalize(ivp_mat3r_t *m) {
    mat3d_t d;
    m3d_load(&d, m);
    m3d_orthogonize(&d);
    ivp_result_t res = m3d_normize(&d); /* stops at the first failing row */
    m3d_store(m, &d);
    return res;
}

void ivp_mat3r_mi2mult(ivp_mat3r_t *out, const ivp_mat3r_t *a, const ivp_mat3r_t *b) {
    /* IVP_U_Matrix3::mi2mult3: row[r] = mb->inline_vmult3(rows[r]) */
    mat3d_t da, db, r;
    m3d_load(&da, a);
    m3d_load(&db, b);
    for (int i = 0; i < 3; i++) {
        const double *p = da.m[i];
        double x = db.m[0][0] * p[0] + db.m[0][1] * p[1] + db.m[0][2] * p[2];
        double y = db.m[1][0] * p[0] + db.m[1][1] * p[1] + db.m[1][2] * p[2];
        double z = db.m[2][0] * p[0] + db.m[2][1] * p[1] + db.m[2][2] * p[2];
        r.m[i][0] = x;
        r.m[i][1] = y;
        r.m[i][2] = z;
    }
    m3d_store(out, &r);
}

int ivp_mat3r_calc_eigen_vector(const ivp_mat3r_t *mat, double eigen_value,
                                ivp_vec3r_t *eigen_vector_out) {
    mat3d_t matrix;
    m3d_load(&matrix, mat);
    double m[3][3];   /* row, column */
    int col_index[3]; /* the original index of a local col */
    int row_index[3]; /* the original index of a local row */
    double ev[3];
    for (int i = 0; i < 3; i++) ev[i] = (double)eigen_vector_out->k[i];

    col_index[0] = 0; col_index[1] = 1; col_index[2] = 2;
    row_index[0] = 0; row_index[1] = 1; row_index[2] = 2;

    matrix.m[0][0] -= eigen_value;
    matrix.m[1][1] -= eigen_value;
    matrix.m[2][2] -= eigen_value;

    /* find maximum value in all values */
    {
        double max = -1.0; /* real maximum is always positive */
        int max_row = 0;
        int max_col = 0;
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                double v = matrix.m[row][col];
                v *= v;
                if (v > max) {
                    max = v;
                    max_col = col;
                    max_row = row;
                }
            }
        }
        row_index[0] = max_row; /* exchange to shift pivot element into m[0][0] */
        row_index[max_row] = 0;
        col_index[0] = max_col;
        col_index[max_col] = 0;

        if (max < (IVP_REF_DOUBLE_RES * IVP_REF_DOUBLE_RES)) { /* 3 degrees of freedom */
            eigen_vector_out->k[0] = IVP_REAL(0.0);
            eigen_vector_out->k[1] = IVP_REAL(0.0);
            eigen_vector_out->k[2] = IVP_REAL(1.0);
            return 3;
        }
    }

    /* copy reordered matrix into m[][] with row and column permutation */
    for (int i = 0; i < 3; i++) {
        const double *source = matrix.m[row_index[i]];
        for (int j = 0; j < 3; j++) {
            m[i][j] = source[col_index[j]];
        }
    }

    /* get rid of first column values */
    for (int r = 1; r <= 2; r++) {
        double f = m[r][0] / m[0][0];
        m[r][1] -= f * m[0][1];
        m[r][2] -= f * m[0][2];
    }

    /* get index of pivot element in 1 or 2nd row -> flip rows */
    {
        double max = -1.0;
        int max_row = 0, max_col = 0;
        for (int row = 1; row < 3; row++) {
            const double *pm = &m[row][0];
            for (int col = 1; col < 3; col++) {
                double v = pm[col];
                v *= v;
                if (v > max) {
                    max = v;
                    max_col = col;
                    max_row = row;
                }
            }
        }
        if (max_row != 1) { /* switch rows 1 and 2 */
            double f;
            f = m[1][1]; m[1][1] = m[2][1]; m[2][1] = f;
            f = m[1][2]; m[1][2] = m[2][2]; m[2][2] = f;
        }
        if (max_col != 1) { /* switch columns 1 and 2 and index pointer */
            double h;
            h = m[0][1]; m[0][1] = m[0][2]; m[0][2] = h;
            h = m[1][1]; m[1][1] = m[1][2]; m[1][2] = h;
            h = m[2][1]; m[2][1] = m[2][2]; m[2][2] = h;
            int i = col_index[2];
            col_index[2] = col_index[1];
            col_index[1] = i;
        }

        /* check for 2 degrees of freedom */
        if (max < (IVP_REF_DOUBLE_RES * IVP_REF_DOUBLE_RES)) {
            ev[col_index[0]] = m[0][1] / m[0][0];
            ev[col_index[1]] = 1.0;
            ev[col_index[2]] = 0.0;
            for (int i = 0; i < 3; i++) eigen_vector_out->k[i] = (ivp_real_t)ev[i];
            return 2;
        }
    }

    double f2 = m[1][2] / m[1][1];

    /* get rid of second column values (should be zero for an eigenvalue) */
    m[2][2] -= f2 * m[2][1];

    /* now we got an upper diagonal matrix */
    ev[col_index[2]] = 1.0;
    ev[col_index[1]] = -f2;
    ev[col_index[0]] = (f2 * m[0][1] - m[0][2]) / m[0][0];
    for (int i = 0; i < 3; i++) eigen_vector_out->k[i] = (ivp_real_t)ev[i];
    return 1;
}

/* Shared part of init_normized3_col/row(v, index) */
static void normized3_vectors(const ivp_vec3r_t *vp, ivp_vec3d_t *v0,
                              ivp_vec3d_t *v1, ivp_vec3d_t *v2) {
    v3d_load(v0, vp);
    ivp_vec3d_normize(v0);
    v2->k[0] = v0->k[1];
    v2->k[1] = v0->k[2] - v0->k[0];
    v2->k[2] = -v0->k[1]; /* orthogonal vector */
    if (ivp_vec3d_normize(v2) != IVP_OK) {
        v2->k[0] = v0->k[2];
        v2->k[1] = -v0->k[2];
        v2->k[2] = v0->k[1] - v0->k[0];
        ivp_vec3d_normize(v2);
    }
    ivp_vec3d_cross(v1, v2, v0);
}

/* Shared part of init_normized3_col/row(vb, index_b, vc) */
static void normized3_vectors2(const ivp_vec3r_t *vb, const ivp_vec3r_t *vc_in,
                               ivp_vec3d_t *v0, ivp_vec3d_t *v1, ivp_vec3d_t *v2) {
    ivp_vec3d_t vc;
    v3d_load(v0, vb);
    v3d_load(&vc, vc_in);
    ivp_vec3d_normize(v0);
    /* IVP_U_Point::inline_calc_cross_product_and_normize stores the cross
     * product through (IVP_FLOAT) before normize() (reference quirk) */
    ivp_vec3d_t c;
    ivp_vec3d_cross(&c, v0, &vc);
    v2->k[0] = (double)(float)c.k[0];
    v2->k[1] = (double)(float)c.k[1];
    v2->k[2] = (double)(float)c.k[2];
    ivp_vec3d_normize(v2);
    ivp_vec3d_cross(v1, v2, v0);
}

static void set_col_d(ivp_mat3r_t *m, int col, const ivp_vec3d_t *v) {
    m->rows[0].k[col] = (ivp_real_t)v->k[0];
    m->rows[1].k[col] = (ivp_real_t)v->k[1];
    m->rows[2].k[col] = (ivp_real_t)v->k[2];
}

static void set_row_d(ivp_mat3r_t *m, int row, const ivp_vec3d_t *v) {
    m->rows[row].k[0] = (ivp_real_t)v->k[0];
    m->rows[row].k[1] = (ivp_real_t)v->k[1];
    m->rows[row].k[2] = (ivp_real_t)v->k[2];
}

static const int s_cyc[5] = {0, 1, 2, 0, 1};

void ivp_mat3r_init_normized_col(ivp_mat3r_t *m, const ivp_vec3r_t *v,
                                 ivp_coord_index_t index) {
    ivp_vec3d_t v0, v1, v2;
    normized3_vectors(v, &v0, &v1, &v2);
    int i0 = (int)index;
    set_col_d(m, s_cyc[i0], &v0);
    set_col_d(m, s_cyc[i0 + 1], &v1);
    set_col_d(m, s_cyc[i0 + 2], &v2);
}

void ivp_mat3r_init_normized_row(ivp_mat3r_t *m, const ivp_vec3r_t *v,
                                 ivp_coord_index_t index) {
    ivp_vec3d_t v0, v1, v2;
    normized3_vectors(v, &v0, &v1, &v2);
    int i0 = (int)index;
    set_row_d(m, s_cyc[i0], &v0);
    set_row_d(m, s_cyc[i0 + 1], &v1);
    set_row_d(m, s_cyc[i0 + 2], &v2);
}

void ivp_mat3r_init_normized_col2(ivp_mat3r_t *m, const ivp_vec3r_t *vb,
                                  ivp_coord_index_t index_b, const ivp_vec3r_t *vc) {
    ivp_vec3d_t v0, v1, v2;
    normized3_vectors2(vb, vc, &v0, &v1, &v2);
    int i0 = (int)index_b;
    set_col_d(m, s_cyc[i0], &v0);
    set_col_d(m, s_cyc[i0 + 1], &v1);
    set_col_d(m, s_cyc[i0 + 2], &v2);
}

void ivp_mat3r_init_normized_row2(ivp_mat3r_t *m, const ivp_vec3r_t *vb,
                                  ivp_coord_index_t index_b, const ivp_vec3r_t *vc) {
    ivp_vec3d_t v0, v1, v2;
    normized3_vectors2(vb, vc, &v0, &v1, &v2);
    int i0 = (int)index_b;
    set_row_d(m, s_cyc[i0], &v0);
    set_row_d(m, s_cyc[i0 + 1], &v1);
    set_row_d(m, s_cyc[i0 + 2], &v2);
}

ivp_result_t ivp_mat3r_get_angles(const ivp_mat3r_t *m, float *alpha,
                                  float *beta, float *gamma) {
    /* third column of 'this' is my looking direction */
    mat3d_t mat;
    m3d_load(&mat, m);
    m3d_orthogonize(&mat);
    if (m3d_normize(&mat) != IVP_OK) {
        ivp_debug_printf("No valid matrix in get_angles!\n");
        return IVP_ERR_NORMALIZATION_FAILED; /* could not be normized */
    }

    ivp_vec3d_t v;
    mat3d_t rot, mat_no_g, mat_no_g_no_b;

    /* calc gamma */
    v.k[0] = mat.m[0][2]; v.k[1] = 0.0; v.k[2] = mat.m[2][2];
    if (ivp_vec3d_normize(&v) != IVP_OK) {
        *gamma = 0.0f;
    } else {
        *gamma = (float)atan2(v.k[0], v.k[2]);
    }
    m3d_identity_rotate(IVP_INDEX_Y, *gamma, &rot);
    m3d_mimult(&rot, &mat, &mat_no_g); /* no gamma */

    /* calc beta */
    v.k[0] = 0.0; v.k[1] = mat_no_g.m[1][2]; v.k[2] = mat_no_g.m[2][2];
    if (ivp_vec3d_normize(&v) != IVP_OK) {
        *beta = 0.0f;
    } else {
        *beta = (float)-atan2(v.k[1], v.k[2]);
    }
    m3d_identity_rotate(IVP_INDEX_X, *beta, &rot);
    m3d_mimult(&rot, &mat_no_g, &mat_no_g_no_b); /* no gamma no beta */

    /* calc alpha */
    v.k[0] = mat_no_g_no_b.m[0][0]; v.k[1] = mat_no_g_no_b.m[1][0]; v.k[2] = 0.0;
    if (ivp_vec3d_normize(&v) != IVP_OK) {
        *alpha = 0.0f;
        ivp_debug_printf("very strange: no alpha vec in get_angles!\n");
    } else {
        *alpha = (float)atan2(v.k[1], v.k[0]);
    }
    return IVP_OK;
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

void ivp_mat3r_init_rotation(ivp_mat3r_t *m, ivp_coord_index_t axis, ivp_real_t angle_in) {
    /* IVP_U_Matrix3::init_rotated3(IVP_COORDINATE_INDEX axis, IVP_FLOAT angle) */
    static const int pm[5] = {1, 2, 0, 1, 2};
    const int *p = &pm[axis];
    double angle = (double)(float)angle_in;
    double SA = sin(angle);
    double CA = cos(angle);
    memset(m, 0, sizeof(*m));
    m->rows[p[2]].k[p[2]] = IVP_REAL(1.0);
    m->rows[p[0]].k[p[0]] = (ivp_real_t)CA;
    m->rows[p[1]].k[p[1]] = (ivp_real_t)CA;
    m->rows[p[0]].k[p[1]] = (ivp_real_t)-SA;
    m->rows[p[1]].k[p[0]] = (ivp_real_t)SA;
}

ivp_real_t ivp_mat3r_quad_rot_distance(const ivp_mat3r_t *a, const ivp_mat3r_t *b) {
    /* rows[0].quad_distance_to(&m2->rows[0]) + rows[1]... + rows[2]...,
     * IVP_U_Point::quad_distance_to: a = d0; a *= a; ...; return a + b + c */
    double row_sum[3];
    for (int r = 0; r < 3; r++) {
        double x = (double)a->rows[r].k[0] - (double)b->rows[r].k[0];
        x *= x;
        double y = (double)a->rows[r].k[1] - (double)b->rows[r].k[1];
        y *= y;
        double z = (double)a->rows[r].k[2] - (double)b->rows[r].k[2];
        z *= z;
        row_sum[r] = x + y + z;
    }
    return (ivp_real_t)(row_sum[0] + row_sum[1] + row_sum[2]);
}
