/* ivp_quat.c — Quaternion operations implementation
 *
 * Reference: IVP_U_Quat (ivu_quat.cxx, ivu_linear_software.hxx,
 * ivu_matrix_macros.hxx), double build.  All formulas are evaluated in
 * double (IVP_DOUBLE) in the reference operation order and rounded to
 * ivp_real_t when stored.
 */
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>
#include <math.h>

#define IVP_QUAT_DELTA 1e-6f /* error tolerance (ivu_quat.cxx) */

/* Local double working copy of a quaternion (IVP_U_Quat). */
typedef struct quatd {
    double x, y, z, w;
} quatd_t;

static inline quatd_t qd_load(const ivp_quatr_t *q) {
    quatd_t r;
    r.x = (double)q->x;
    r.y = (double)q->y;
    r.z = (double)q->z;
    r.w = (double)q->w;
    return r;
}

static inline void qd_store(ivp_quatr_t *q, const quatd_t *r) {
    q->x = (ivp_real_t)r->x;
    q->y = (ivp_real_t)r->y;
    q->z = (ivp_real_t)r->z;
    q->w = (ivp_real_t)r->w;
}

static void qd_normize_quat(quatd_t *quat) {
    double square = quat->x * quat->x + quat->y * quat->y +
                    quat->z * quat->z + quat->w * quat->w;
    if (square > IVP_REF_DOUBLE_EPS) {
        double dist = 1.0 / sqrt(square);
        quat->x *= dist;
        quat->y *= dist;
        quat->z *= dist;
        quat->w *= dist;
    }
}

static void qd_normize_correct_step(quatd_t *quat, int steps) {
    double square = quat->x * quat->x + quat->y * quat->y +
                    quat->z * quat->z + quat->w * quat->w;
    square *= 0.5;
    double factor = 1.5 - square;
    if (steps > 1) factor += 0.5 - (factor * factor * square);
    if (steps > 2) factor += 0.5 - (factor * factor * square);
    if (steps > 3) factor += 0.5 - (factor * factor * square);
    if (steps > 4) factor += 0.5 - (factor * factor * square);
    if (steps > 5) factor += 0.5 - (factor * factor * square);
    quat->x *= factor;
    quat->y *= factor;
    quat->z *= factor;
    quat->w *= factor;
}

static inline void qd_mult(quatd_t *res, const quatd_t *q1, const quatd_t *q2) {
    double xx = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    double yy = q1->w * q2->y + q1->y * q2->w + q1->z * q2->x - q1->x * q2->z;
    double zz = q1->w * q2->z + q1->z * q2->w + q1->x * q2->y - q1->y * q2->x;
    double ww = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    res->x = xx;
    res->y = yy;
    res->z = zz;
    res->w = ww;
}

/* ── Conversion: quaternion → matrix ────────────────────────────────────── */

void ivp_quat_to_mat3(const ivp_quatr_t *q, ivp_mat3r_t *m) {
    quatd_t quat = qd_load(q);
    double wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;

    x2 = quat.x + quat.x; y2 = quat.y + quat.y; z2 = quat.z + quat.z;
    xx = quat.x * x2; xy = quat.x * y2; xz = quat.x * z2;
    yy = quat.y * y2; yz = quat.y * z2; zz = quat.z * z2;
    wx = quat.w * x2; wy = quat.w * y2; wz = quat.w * z2;

    m->rows[0].k[0] = (ivp_real_t)(1.0 - (yy + zz));
    m->rows[0].k[1] = (ivp_real_t)(xy - wz);
    m->rows[0].k[2] = (ivp_real_t)(xz + wy);

    m->rows[1].k[0] = (ivp_real_t)(xy + wz);
    m->rows[1].k[1] = (ivp_real_t)(1.0 - (xx + zz));
    m->rows[1].k[2] = (ivp_real_t)(yz - wx);

    m->rows[2].k[0] = (ivp_real_t)(xz - wy);
    m->rows[2].k[1] = (ivp_real_t)(yz + wx);
    m->rows[2].k[2] = (ivp_real_t)(1.0 - (xx + yy));
}

void ivp_quat_to_mat4x4(const ivp_quatr_t *q, double m[4][4]) {
    quatd_t quat = qd_load(q);
    double wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;

    x2 = quat.x + quat.x; y2 = quat.y + quat.y; z2 = quat.z + quat.z;
    xx = quat.x * x2; xy = quat.x * y2; xz = quat.x * z2;
    yy = quat.y * y2; yz = quat.y * z2; zz = quat.z * z2;
    wx = quat.w * x2; wy = quat.w * y2; wz = quat.w * z2;

    m[0][0] = 1.0 - (yy + zz);
    m[0][1] = xy - wz;
    m[0][2] = xz + wy;
    m[0][3] = 0.0;

    m[1][0] = xy + wz;
    m[1][1] = 1.0 - (xx + zz);
    m[1][2] = yz - wx;
    m[1][3] = 0.0;

    m[2][0] = xz - wy;
    m[2][1] = yz + wx;
    m[2][2] = 1.0 - (xx + yy);
    m[2][3] = 0.0;

    m[3][0] = 0.0;
    m[3][1] = 0.0;
    m[3][2] = 0.0;
    m[3][3] = 1.0;
}

/* ── Conversion: matrix → quaternion ────────────────────────────────────── */

void ivp_quat_from_mat3(ivp_quatr_t *q, const ivp_mat3r_t *mat) {
    double m[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            m[r][c] = (double)mat->rows[r].k[c];

    quatd_t quat;
    double tr = m[0][0] + m[1][1] + m[2][2];

    if (tr > 0.0) {
        double s = sqrt(tr + 1.0);
        quat.w = 0.5 * s;
        s = 0.5 / s;
        quat.x = (m[2][1] - m[1][2]) * s;
        quat.y = (m[0][2] - m[2][0]) * s;
        quat.z = (m[1][0] - m[0][1]) * s;
    } else {
        /* diagonal is negative */
        double qq[4];
        static const int nxt[3] = {1, 2, 0};
        int i = 0;
        if (m[1][1] > m[0][0]) i = 1;
        if (m[2][2] > m[i][i]) i = 2;
        int j = nxt[i];
        int k = nxt[j];

        double s = sqrt(m[i][i] - (m[j][j] + m[k][k]) + 1.0);
        qq[i] = s * 0.5;
        if (s != 0.0) s = 0.5 / s;

        qq[3] = (m[k][j] - m[j][k]) * s;
        qq[j] = (m[j][i] + m[i][j]) * s;
        qq[k] = (m[k][i] + m[i][k]) * s;

        quat.x = qq[0];
        quat.y = qq[1];
        quat.z = qq[2];
        quat.w = qq[3];
    }
    qd_normize_quat(&quat);
    qd_store(q, &quat);
}

void ivp_quat_from_mat4x4(ivp_quatr_t *q, const double m[4][4]) {
    quatd_t quat;
    double tr, s;

    tr = m[0][0] + m[1][1] + m[2][2];

    if (tr > 0.0) {
        s = sqrt(tr + 1.0);
        quat.w = s * .5;
        s = 0.5 / s;
        quat.x = (m[1][2] - m[2][1]) * s;
        quat.y = (m[2][0] - m[0][2]) * s;
        quat.z = (m[0][1] - m[1][0]) * s;
    } else {
        double qq[4];
        static const int nxt[3] = {1, 2, 0};
        int i = 0;
        if (m[1][1] > m[0][0]) i = 1;
        if (m[2][2] > m[i][i]) i = 2;
        int j = nxt[i];
        int k = nxt[j];

        s = sqrt((m[i][i] - (m[j][j] + m[k][k])) + 1.0);
        qq[i] = s * 0.5;
        if (s != 0.0) s = 0.5 / s;

        qq[3] = (m[j][k] - m[k][j]) * s;
        qq[j] = (m[i][j] + m[j][i]) * s;
        qq[k] = (m[i][k] + m[k][i]) * s;

        quat.x = qq[0];
        quat.y = qq[1];
        quat.z = qq[2];
        quat.w = qq[3];
    }
    qd_store(q, &quat);
}

/* ── Arithmetic ─────────────────────────────────────────────────────────── */

void ivp_quat_set_div_unit_quat(ivp_quatr_t *out, const ivp_quatr_t *q1,
                                const ivp_quatr_t *q2) {
    quatd_t a = qd_load(q1);
    quatd_t b = qd_load(q2);
    quatd_t inv = { -b.x, -b.y, -b.z, b.w };
    quatd_t r;
    qd_mult(&r, &a, &inv);
    qd_store(out, &r);
}

void ivp_quat_set_invert_mult(ivp_quatr_t *out, const ivp_quatr_t *q1,
                              const ivp_quatr_t *q2) {
    quatd_t a = qd_load(q1);
    quatd_t b = qd_load(q2);
    quatd_t inv = { -a.x, -a.y, -a.z, a.w };
    quatd_t r;
    qd_mult(&r, &inv, &b);
    qd_store(out, &r);
}

void ivp_quat_invert(ivp_quatr_t *q) {
    quatd_t quat = qd_load(q);
    double norm = quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w;
    double inv_norm = 1.0 / norm;
    quat.x = -quat.x * inv_norm;
    quat.y = -quat.y * inv_norm;
    quat.z = -quat.z * inv_norm;
    quat.w = quat.w * inv_norm;
    qd_store(q, &quat);
}

/* ── Normalization ──────────────────────────────────────────────────────── */

ivp_result_t ivp_quat_normalize(ivp_quatr_t *q) {
    quatd_t quat = qd_load(q);
    double square = quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w;
    if (!(square > IVP_REF_DOUBLE_EPS)) return IVP_ERR_NORMALIZATION_FAILED;
    qd_normize_quat(&quat);
    qd_store(q, &quat);
    return IVP_OK;
}

void ivp_quat_fast_normalize(ivp_quatr_t *q) {
    quatd_t quat = qd_load(q);
    double square = quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w;
    if (fabs(1.0 - square) > IVP_REF_DOUBLE_RES) {
        double factor = 1.5 - 0.5 * square;
        /* reference: goto loop; while (|1 - f^2 s| > P_DOUBLE_RES) { loop: ... }
         * The iteration cap only guards against non-convergent input
         * (|q|^2 > ~4); it never triggers for |q| = 1 +- 0.1. */
        int guard = 0;
        do {
            factor += 0.5 * (1.0 - (factor * factor * square));
        } while (fabs(1.0 - (factor * factor * square)) > IVP_REF_DOUBLE_RES &&
                 ++guard < 200);
        quat.x *= factor;
        quat.y *= factor;
        quat.z *= factor;
        quat.w *= factor;
        qd_store(q, &quat);
    }
}

void ivp_quat_normize_correct_step(ivp_quatr_t *q, int steps) {
    quatd_t quat = qd_load(q);
    qd_normize_correct_step(&quat, steps);
    qd_store(q, &quat);
}

/* ── Construction from angles ───────────────────────────────────────────── */

void ivp_quat_set_rotation(ivp_quatr_t *q, double rot_x, double rot_y, double rot_z) {
    double f = .5;
    quatd_t r;
    r.x = sin(rot_x * f);
    r.y = sin(rot_y * f);
    r.z = sin(rot_z * f);
    double qlen_set = r.x * r.x + r.y * r.y + r.z * r.z;
    r.w = (qlen_set < 1.0) ? sqrt(1.0 - qlen_set) : 0.0;
    qd_store(q, &r);
}

void ivp_quat_set_fast_multiple(ivp_quatr_t *q, const ivp_vec3r_t *angles,
                                double factor) {
    double f = factor * .5;
    quatd_t r;
    r.x = sin((double)angles->k[0] * f);
    r.y = sin((double)angles->k[1] * f);
    r.z = sin((double)angles->k[2] * f);
    double n = r.x * r.x + r.y * r.y + r.z * r.z;
    r.w = sqrt(1.0 - ((n > 1.0) ? 1.0 : n));
    qd_store(q, &r);
}

void ivp_quat_set_fast_multiple_with_clip(ivp_quatr_t *q,
                                           const ivp_vec3f_t *angles,
                                           ivp_real_t factor) {
    double f = (double)factor * .5;
    quatd_t r;
    r.x = sin((double)angles->k[0] * f);  /* IVP_Inline_Math::ivp_sinf (double) */
    r.y = sin((double)angles->k[1] * f);
    r.z = sin((double)angles->k[2] * f);

    double qlen = r.x * r.x + r.y * r.y + r.z * r.z;
    if (qlen > 1.0) { /* reverse quat needed */
        double ilen = (1.0 - IVP_REF_RES_EPS) / sqrt(qlen);
        r.x *= ilen;
        r.y *= ilen;
        r.z *= ilen;
        qlen = r.x * r.x + r.y * r.y + r.z * r.z;
    }
    r.w = sqrt(1.0 - qlen);
    qd_store(q, &r);
}

/* ivp_very_fast_sin (ivu_quat.cxx): IVP_FLOAT in/out, float arithmetic */
static inline float ivp_very_fast_sin(float x) {
    return x - x * x * x * (1.0f / 6.0f);
}

void ivp_quat_set_very_fast_multiple(ivp_quatr_t *q,
                                      const ivp_vec3f_t *angles,
                                      ivp_real_t factor) {
    double f = (double)factor * .5;
    quatd_t r;
    r.x = (double)ivp_very_fast_sin((float)((double)angles->k[0] * f));
    r.y = (double)ivp_very_fast_sin((float)((double)angles->k[1] * f));
    r.z = (double)ivp_very_fast_sin((float)((double)angles->k[2] * f));
    double qlen = r.x * r.x + r.y * r.y + r.z * r.z;
    r.w = (qlen < 1.0) ? sqrt(1.0 - qlen) : 0.0;
    qd_store(q, &r);
}

void ivp_quat_get_angles(const ivp_quatr_t *q, ivp_vec3f_t *angles_out) {
    double f = 2.0;
    angles_out->k[0] = (float)(f * asin((double)q->x));
    angles_out->k[1] = (float)(f * asin((double)q->y));
    angles_out->k[2] = (float)(f * asin((double)q->z));
}

void ivp_quat_set_from_rotation_vectors(ivp_quatr_t *q,
                                        double x1, double y1, double z1,
                                        double x2, double y2, double z2) {
    double tx, ty, tz, temp, dist;
    double cost, len, ss;
    quatd_t r;

    /* get dot product of two vectors */
    double s1, s2, s3;
    s2 = y2 * y1;
    s3 = z1 * z2;
    s1 = x1 * x2;
    cost = s1 + s2 + s3;

    /* check if parallel */
    if (cost > (double)0.99999f) {
        r.x = r.y = r.z = 0.0;
        r.w = 1.0;
        qd_store(q, &r);
        return;
    } else if (cost < (double)-0.99999f) { /* check if opposite */
        /* check if we can use cross product of from vector with [1, 0, 0] */
        tx = 0.0;
        ty = x1;
        tz = -y1;

        len = sqrt(ty * ty + tz * tz);

        if (len < (double)IVP_QUAT_DELTA) {
            /* nope! we need cross product of from vector with [0, 1, 0] */
            tx = -z1;
            ty = 0.0;
            tz = x1;
        }

        /* normalize */
        temp = tx * tx + ty * ty + tz * tz;
        dist = 1.0 / sqrt(temp);
        tx *= dist;
        ty *= dist;
        tz *= dist;

        r.x = tx;
        r.y = ty;
        r.z = tz;
        r.w = 0.0;
        qd_store(q, &r);
        return;
    }

    /* ... else we can just cross two vectors */
    tx = y1 * z2 - z1 * y2;
    ty = z1 * x2 - x1 * z2;
    tz = x1 * y2 - y1 * x2;

    temp = tx * tx + ty * ty + tz * tz;
    dist = 1.0 / sqrt(temp);
    tx *= dist;
    ty *= dist;
    tz *= dist;

    /* half-angle formulae (sin^2 t = ( 1 - cos (2t) ) /2) */
    ss = sqrt(0.5 * (1.0 - cost));
    tx *= ss;
    ty *= ss;
    tz *= ss;

    r.x = tx;
    r.y = ty;
    r.z = tz;
    /* cos^2 t = ( 1 + cos (2t) ) / 2 */
    r.w = sqrt(0.5 * (1.0 + cost));
    qd_store(q, &r);
}

/* ── Interpolation ──────────────────────────────────────────────────────── */

void ivp_quat_set_interpolate_smoothly(ivp_quatr_t *out, const ivp_quatr_t *from_in,
                                       const ivp_quatr_t *to_in, ivp_real_t t_in) {
    quatd_t from = qd_load(from_in);
    quatd_t to = qd_load(to_in);
    double t = (double)t_in;
    quatd_t res;

    /* calc cosine */
    double cosom = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;

    /* adjust signs (if necessary) */
    float sign;
    if (cosom > 0.0) {
        sign = 1.0f;
    } else {
        cosom = -cosom;
        sign = -1.0f;
    }

    /* 1.0f - 0.001f is evaluated in float by the reference */
    const float threshold = 1.0f - 0.001f;
    if (cosom < (double)threshold) {
        /* standard case (slerp) */
        double omega = acos(cosom);
        double i_sinom = 1.0 / sqrt(1.0 - cosom * cosom);
        double scale0 = sin((1.0 - t) * omega) * i_sinom;
        double scale1 = sin(t * omega) * i_sinom * (double)sign;
        res.x = scale0 * from.x + scale1 * to.x;
        res.y = scale0 * from.y + scale1 * to.y;
        res.z = scale0 * from.z + scale1 * to.z;
        res.w = scale0 * from.w + scale1 * to.w;
    } else {
        /* "from" and "to" quaternions are very close: linear interpolation */
        res.x = from.x + t * ((double)sign * to.x - from.x);
        res.y = from.y + t * ((double)sign * to.y - from.y);
        res.z = from.z + t * ((double)sign * to.z - from.z);
        res.w = from.w + t * ((double)sign * to.w - from.w);
        qd_normize_correct_step(&res, 3);
    }
    qd_store(out, &res);
}

void ivp_quat_set_interpolate_linear(ivp_quatr_t *out, const ivp_quatr_t *from_in,
                                     const ivp_quatr_t *to_in, ivp_real_t t_in) {
    quatd_t from = qd_load(from_in);
    quatd_t to = qd_load(to_in);
    double t = (double)t_in;
    double to1[4];

    double cosom = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;
    if (cosom < 0.0) {
        to1[0] = -to.x;
        to1[1] = -to.y;
        to1[2] = -to.z;
        to1[3] = -to.w;
    } else {
        to1[0] = to.x;
        to1[1] = to.y;
        to1[2] = to.z;
        to1[3] = to.w;
    }

    double scale0 = 1.0 - t;
    double scale1 = t;
    quatd_t res;
    res.x = scale0 * from.x + scale1 * to1[0];
    res.y = scale0 * from.y + scale1 * to1[1];
    res.z = scale0 * from.z + scale1 * to1[2];
    res.w = scale0 * from.w + scale1 * to1[3];
    qd_store(out, &res);
}

void ivp_quat_slerp(ivp_quatr_t *out, const ivp_quatr_t *a,
                     const ivp_quatr_t *b, ivp_real_t t) {
    ivp_quat_set_interpolate_smoothly(out, a, b, t);
}

void ivp_quat_nlerp(ivp_quatr_t *out, const ivp_quatr_t *a,
                     const ivp_quatr_t *b, ivp_real_t t) {
    ivp_real_t omt = IVP_REAL(1.0) - t;

    /* Ensure shortest path */
    ivp_real_t d = ivp_quat_dot(a, b);
    ivp_real_t sign = d < IVP_REAL(0.0) ? IVP_REAL(-1.0) : IVP_REAL(1.0);

    out->x = omt * a->x + t * sign * b->x;
    out->y = omt * a->y + t * sign * b->y;
    out->z = omt * a->z + t * sign * b->z;
    out->w = omt * a->w + t * sign * b->w;

    ivp_quat_normalize(out);
}

/* ── Angle difference ───────────────────────────────────────────────────── */

ivp_real_t ivp_quat_angle_diff(const ivp_quatr_t *a, const ivp_quatr_t *b) {
    ivp_real_t d = ivp_fabs(ivp_quat_dot(a, b));
    if (d > IVP_REAL(1.0)) d = IVP_REAL(1.0);
    return IVP_REAL(2.0) * ivp_acos(d);
}
