/* ivp_controller_ref_math.h -- private double-precision helpers mirroring
 * reference IVP_U_Quat / IVP_U_Matrix3 / IVP_Inline_Math routines used by
 * the controller ports (quaternion <-> matrix, smooth interpolation,
 * fast_asin).  Quaternions are stored as {x, y, z, w}. */
#ifndef IVP_CONTROLLER_REF_MATH_H
#define IVP_CONTROLLER_REF_MATH_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>
#include <math.h>

typedef double ivp_rm_m3_t[3][3];

/* reference IVP_U_Quat::set_matrix */
static inline void ivp_rm_quat_to_m3(const double q[4], ivp_rm_m3_t m)
{
    double x2 = q[0] + q[0], y2 = q[1] + q[1], z2 = q[2] + q[2];
    double xx = q[0] * x2, xy = q[0] * y2, xz = q[0] * z2;
    double yy = q[1] * y2, yz = q[1] * z2, zz = q[2] * z2;
    double wx = q[3] * x2, wy = q[3] * y2, wz = q[3] * z2;
    m[0][0] = 1.0 - (yy + zz); m[0][1] = xy - wz;         m[0][2] = xz + wy;
    m[1][0] = xy + wz;         m[1][1] = 1.0 - (xx + zz); m[1][2] = yz - wx;
    m[2][0] = xz - wy;         m[2][1] = yz + wx;         m[2][2] = 1.0 - (xx + yy);
}

/* reference IVP_U_Quat::set_quaternion(const IVP_U_Matrix3 *) + normize_quat */
static inline void ivp_rm_m3_to_quat(const ivp_rm_m3_t m, double q_out[4])
{
    double tr = m[0][0] + m[1][1] + m[2][2];
    double x, y, z, w;
    if (tr > 0.0) {
        double s = sqrt(tr + 1.0);
        w = 0.5 * s;
        s = 0.5 / s;
        x = (m[2][1] - m[1][2]) * s;
        y = (m[0][2] - m[2][0]) * s;
        z = (m[1][0] - m[0][1]) * s;
    } else {
        double q[4];
        static const int nxt[3] = { 1, 2, 0 };
        int i = 0;
        if (m[1][1] > m[0][0]) i = 1;
        if (m[2][2] > m[i][i]) i = 2;
        int j = nxt[i], k = nxt[j];
        double s = sqrt(m[i][i] - (m[j][j] + m[k][k]) + 1.0);
        q[i] = s * 0.5;
        if (s != 0.0) s = 0.5 / s;
        q[3] = (m[k][j] - m[j][k]) * s;
        q[j] = (m[j][i] + m[i][j]) * s;
        q[k] = (m[k][i] + m[i][k]) * s;
        x = q[0]; y = q[1]; z = q[2]; w = q[3];
    }
    double len = sqrt(x * x + y * y + z * z + w * w);
    if (len > 0.0) {
        double il = 1.0 / len;
        x *= il; y *= il; z *= il; w *= il;
    }
    q_out[0] = x; q_out[1] = y; q_out[2] = z; q_out[3] = w;
}

/* reference IVP_U_Quat::set_interpolate_smoothly (t not clamped) */
static inline void ivp_rm_interpolate_smoothly(const double from[4], const double to[4],
                                               double t, double res[4])
{
    double cosom = from[0] * to[0] + from[1] * to[1] + from[2] * to[2] + from[3] * to[3];
    double sign = 1.0;
    if (!(cosom > 0.0)) {
        cosom = -cosom;
        sign = -1.0;
    }
    if (cosom < 1.0 - 0.001) {
        double omega = acos(cosom);
        double i_sinom = 1.0 / sqrt(1.0 - cosom * cosom);
        double s0 = sin((1.0 - t) * omega) * i_sinom;
        double s1 = sin(t * omega) * i_sinom * sign;
        for (int i = 0; i < 4; i++) res[i] = s0 * from[i] + s1 * to[i];
    } else {
        double r[4];
        for (int i = 0; i < 4; i++) r[i] = from[i] + t * (sign * to[i] - from[i]);
        double square = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]) * 0.5;
        double factor = 1.5 - square;
        factor += 0.5 - (factor * factor * square);
        factor += 0.5 - (factor * factor * square);
        for (int i = 0; i < 4; i++) res[i] = r[i] * factor;
    }
}

/* reference IVP_Inline_Math::fast_asin (float polynomial) */
static inline double ivp_rm_fast_asin(double a)
{
    float angle = (float)a;
    float q = angle * angle;
    float t = angle * q;
    float qi = t * q;
    return (double)(angle + 0.12f * t + 0.29f * qi);
}

static inline void ivp_rm_quat_from_quatr(const ivp_quatr_t *q, double out[4])
{
    out[0] = (double)q->x; out[1] = (double)q->y; out[2] = (double)q->z; out[3] = (double)q->w;
}

static inline void ivp_rm_quatr_from_quat(const double q[4], ivp_quatr_t *out)
{
    out->x = (ivp_real_t)q[0]; out->y = (ivp_real_t)q[1];
    out->z = (ivp_real_t)q[2]; out->w = (ivp_real_t)q[3];
}

static inline void ivp_rm_m3_from_transform(const ivp_transform_r_t *t, ivp_rm_m3_t m)
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) m[i][j] = (double)t->rot.rows[i].k[j];
}

#endif /* IVP_CONTROLLER_REF_MATH_H */
