/* ivp_quat.c — Quaternion operations implementation */
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>
#include <math.h>

/* ── Conversion: quaternion → matrix ────────────────────────────────────── */

void ivp_quat_to_mat3(const ivp_quatr_t *q, ivp_mat3r_t *m) {
    ivp_real_t xx = q->x * q->x;
    ivp_real_t yy = q->y * q->y;
    ivp_real_t zz = q->z * q->z;
    ivp_real_t xy = q->x * q->y;
    ivp_real_t xz = q->x * q->z;
    ivp_real_t yz = q->y * q->z;
    ivp_real_t wx = q->w * q->x;
    ivp_real_t wy = q->w * q->y;
    ivp_real_t wz = q->w * q->z;

    m->rows[0].k[0] = IVP_REAL(1.0) - IVP_REAL(2.0) * (yy + zz);
    m->rows[0].k[1] = IVP_REAL(2.0) * (xy - wz);
    m->rows[0].k[2] = IVP_REAL(2.0) * (xz + wy);

    m->rows[1].k[0] = IVP_REAL(2.0) * (xy + wz);
    m->rows[1].k[1] = IVP_REAL(1.0) - IVP_REAL(2.0) * (xx + zz);
    m->rows[1].k[2] = IVP_REAL(2.0) * (yz - wx);

    m->rows[2].k[0] = IVP_REAL(2.0) * (xz - wy);
    m->rows[2].k[1] = IVP_REAL(2.0) * (yz + wx);
    m->rows[2].k[2] = IVP_REAL(1.0) - IVP_REAL(2.0) * (xx + yy);
}

/* ── Conversion: matrix → quaternion ────────────────────────────────────── */

void ivp_quat_from_mat3(ivp_quatr_t *q, const ivp_mat3r_t *m) {
    ivp_real_t trace = m->rows[0].k[0] + m->rows[1].k[1] + m->rows[2].k[2];

    if (trace > IVP_REAL(0.0)) {
        ivp_real_t s = ivp_sqrt(trace + IVP_REAL(1.0)) * IVP_REAL(2.0);
        ivp_real_t inv_s = IVP_REAL(1.0) / s;
        q->w = IVP_REAL(0.25) * s;
        q->x = (m->rows[2].k[1] - m->rows[1].k[2]) * inv_s;
        q->y = (m->rows[0].k[2] - m->rows[2].k[0]) * inv_s;
        q->z = (m->rows[1].k[0] - m->rows[0].k[1]) * inv_s;
    } else if (m->rows[0].k[0] > m->rows[1].k[1] && m->rows[0].k[0] > m->rows[2].k[2]) {
        ivp_real_t s = ivp_sqrt(IVP_REAL(1.0) + m->rows[0].k[0] - m->rows[1].k[1] - m->rows[2].k[2]) * IVP_REAL(2.0);
        ivp_real_t inv_s = IVP_REAL(1.0) / s;
        q->w = (m->rows[2].k[1] - m->rows[1].k[2]) * inv_s;
        q->x = IVP_REAL(0.25) * s;
        q->y = (m->rows[0].k[1] + m->rows[1].k[0]) * inv_s;
        q->z = (m->rows[0].k[2] + m->rows[2].k[0]) * inv_s;
    } else if (m->rows[1].k[1] > m->rows[2].k[2]) {
        ivp_real_t s = ivp_sqrt(IVP_REAL(1.0) + m->rows[1].k[1] - m->rows[0].k[0] - m->rows[2].k[2]) * IVP_REAL(2.0);
        ivp_real_t inv_s = IVP_REAL(1.0) / s;
        q->w = (m->rows[0].k[2] - m->rows[2].k[0]) * inv_s;
        q->x = (m->rows[0].k[1] + m->rows[1].k[0]) * inv_s;
        q->y = IVP_REAL(0.25) * s;
        q->z = (m->rows[1].k[2] + m->rows[2].k[1]) * inv_s;
    } else {
        ivp_real_t s = ivp_sqrt(IVP_REAL(1.0) + m->rows[2].k[2] - m->rows[0].k[0] - m->rows[1].k[1]) * IVP_REAL(2.0);
        ivp_real_t inv_s = IVP_REAL(1.0) / s;
        q->w = (m->rows[1].k[0] - m->rows[0].k[1]) * inv_s;
        q->x = (m->rows[0].k[2] + m->rows[2].k[0]) * inv_s;
        q->y = (m->rows[1].k[2] + m->rows[2].k[1]) * inv_s;
        q->z = IVP_REAL(0.25) * s;
    }
}

/* ── Normalization ──────────────────────────────────────────────────────── */

ivp_result_t ivp_quat_normalize(ivp_quatr_t *q) {
    ivp_real_t ql = q->x*q->x + q->y*q->y + q->z*q->z + q->w*q->w;
    if (ql < IVP_REAL_EPS) return IVP_ERR_NORMALIZATION_FAILED;
    ivp_real_t inv_len = IVP_REAL(1.0) / ivp_sqrt(ql);
    q->x *= inv_len;
    q->y *= inv_len;
    q->z *= inv_len;
    q->w *= inv_len;
    return IVP_OK;
}

void ivp_quat_fast_normalize(ivp_quatr_t *q) {
    /* Matching original fast_normize_quat():
     * Uses Newton-Raphson rsqrt for fast normalization.
     * Assumes quaternion is approximately unit length (|q| ≈ 1.0 ± 0.1). */
    ivp_real_t ql = q->x*q->x + q->y*q->y + q->z*q->z + q->w*q->w;
    IVP_ASSERT(ql > IVP_REAL_EPS);

#ifdef IVP_REAL_DOUBLE
    ivp_real_t inv_len = ivp_isqrt_double(ql, 1); /* 3 iterations for double */
#else
    ivp_real_t inv_len = (ivp_real_t)ivp_isqrt_float((float)ql);
#endif
    q->x *= inv_len;
    q->y *= inv_len;
    q->z *= inv_len;
    q->w *= inv_len;
}

/* ── Integration helpers ────────────────────────────────────────────────── */

void ivp_quat_set_fast_multiple_with_clip(ivp_quatr_t *q,
                                           const ivp_vec3f_t *angles,
                                           ivp_real_t factor) {
    /* Small angle approximation:
     * q.xyz = sin(angle * factor * 0.5) ≈ angle * factor * 0.5
     * q.w   = sqrt(1 - |q.xyz|²)
     * With clipping if angle too large. */
    ivp_real_t hx = (ivp_real_t)angles->k[0] * factor * IVP_REAL(0.5);
    ivp_real_t hy = (ivp_real_t)angles->k[1] * factor * IVP_REAL(0.5);
    ivp_real_t hz = (ivp_real_t)angles->k[2] * factor * IVP_REAL(0.5);

    ivp_real_t sum2 = hx*hx + hy*hy + hz*hz;

    if (sum2 < IVP_REAL(1.0)) {
        /* Use sin approximation (for small angles sin(x) ≈ x) */
        q->x = hx;
        q->y = hy;
        q->z = hz;
        q->w = ivp_sqrt(IVP_REAL(1.0) - sum2);
    } else {
        /* Clip: normalize to unit hemisphere */
        ivp_real_t inv_len = IVP_REAL(1.0) / ivp_sqrt(sum2);
        /* Clamp to |angle| < PI/2 */
        ivp_real_t clip = IVP_REAL(0.9999);
        q->x = hx * inv_len * clip;
        q->y = hy * inv_len * clip;
        q->z = hz * inv_len * clip;
        sum2 = q->x*q->x + q->y*q->y + q->z*q->z;
        q->w = ivp_sqrt(IVP_REAL(1.0) - sum2);
    }
}

void ivp_quat_set_very_fast_multiple(ivp_quatr_t *q,
                                      const ivp_vec3f_t *angles,
                                      ivp_real_t factor) {
    /* Very fast: no trig, just linear approximation
     * q.xyz = angles * factor * 0.5
     * q.w = 1  (will be corrected by fast_normalize later) */
    q->x = (ivp_real_t)angles->k[0] * factor * IVP_REAL(0.5);
    q->y = (ivp_real_t)angles->k[1] * factor * IVP_REAL(0.5);
    q->z = (ivp_real_t)angles->k[2] * factor * IVP_REAL(0.5);
    q->w = IVP_REAL(1.0);
}

/* ── Interpolation ──────────────────────────────────────────────────────── */

void ivp_quat_slerp(ivp_quatr_t *out, const ivp_quatr_t *a,
                     const ivp_quatr_t *b, ivp_real_t t) {
    ivp_real_t d = ivp_quat_dot(a, b);

    /* Ensure shortest path */
    ivp_quatr_t b2 = *b;
    if (d < IVP_REAL(0.0)) {
        d = -d;
        b2.x = -b2.x; b2.y = -b2.y; b2.z = -b2.z; b2.w = -b2.w;
    }

    if (d > IVP_REAL(0.9995)) {
        /* Nearly parallel: use linear interpolation */
        ivp_quat_nlerp(out, a, &b2, t);
        return;
    }

    ivp_real_t theta = ivp_acos(ivp_clamp(d, IVP_REAL(-1.0), IVP_REAL(1.0)));
    ivp_real_t sin_theta = ivp_sin(theta);
    ivp_real_t wa = ivp_sin((IVP_REAL(1.0) - t) * theta) / sin_theta;
    ivp_real_t wb = ivp_sin(t * theta) / sin_theta;

    out->x = wa * a->x + wb * b2.x;
    out->y = wa * a->y + wb * b2.y;
    out->z = wa * a->z + wb * b2.z;
    out->w = wa * a->w + wb * b2.w;
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
