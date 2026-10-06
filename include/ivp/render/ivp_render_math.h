/* ivp_render_math.h -- Column-major mat4 and float vec3 helpers for rendering */
#ifndef IVP_RENDER_MATH_H
#define IVP_RENDER_MATH_H

#include <math.h>
#include <string.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define IVP_RENDER_PI_F 3.14159265358979323846f

/* ── Mat4 type (column-major, 16 floats) ───────────────────────────────── */

typedef float ivp_render_mat4_t[16];

/* ── Mat4 operations ───────────────────────────────────────────────────── */

static inline void ivp_render_mat4_identity(float m[16])
{
    memset(m, 0, 16 * sizeof(float));
    m[0] = m[5] = m[10] = m[15] = 1.0f;
}

static inline void ivp_render_mat4_mul(float out[16],
                                       const float a[16],
                                       const float b[16])
{
    float tmp[16];
    for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
            tmp[c*4 + r] = 0;
            for (int k = 0; k < 4; k++) {
                tmp[c*4 + r] += a[k*4 + r] * b[c*4 + k];
            }
        }
    }
    memcpy(out, tmp, sizeof(tmp));
}

static inline void ivp_render_mat4_perspective(float out[16],
                                               float fov_rad,
                                               float aspect,
                                               float zn, float zf)
{
    float f = 1.0f / tanf(fov_rad * 0.5f);
    memset(out, 0, 16 * sizeof(float));
    out[0]  = f / aspect;
    out[5]  = f;
    out[10] = (zf + zn) / (zn - zf);
    out[11] = -1.0f;
    out[14] = (2.0f * zf * zn) / (zn - zf);
}

static inline void ivp_render_mat4_ortho(float out[16],
                                         float l, float r,
                                         float b, float t,
                                         float n, float f)
{
    memset(out, 0, 16 * sizeof(float));
    out[0]  = 2.0f / (r - l);
    out[5]  = 2.0f / (t - b);
    out[10] = -2.0f / (f - n);
    out[12] = -(r + l) / (r - l);
    out[13] = -(t + b) / (t - b);
    out[14] = -(f + n) / (f - n);
    out[15] = 1.0f;
}

/* ── Mat4 inverse (cofactor / adjugate method) ─────────────────────────── */

static inline bool ivp_render_mat4_invert(float out[16], const float m[16])
{
    /* Laplace expansion over 2x2 sub-determinants.  Reading a column-major
     * array as row-major gives the transpose, and (A^T)^-1 = (A^-1)^T, so
     * the same code inverts either layout. */
#define IVP_A(r, c) m[(r) * 4 + (c)]
    float s0 = IVP_A(0,0)*IVP_A(1,1) - IVP_A(1,0)*IVP_A(0,1);
    float s1 = IVP_A(0,0)*IVP_A(1,2) - IVP_A(1,0)*IVP_A(0,2);
    float s2 = IVP_A(0,0)*IVP_A(1,3) - IVP_A(1,0)*IVP_A(0,3);
    float s3 = IVP_A(0,1)*IVP_A(1,2) - IVP_A(1,1)*IVP_A(0,2);
    float s4 = IVP_A(0,1)*IVP_A(1,3) - IVP_A(1,1)*IVP_A(0,3);
    float s5 = IVP_A(0,2)*IVP_A(1,3) - IVP_A(1,2)*IVP_A(0,3);

    float c5 = IVP_A(2,2)*IVP_A(3,3) - IVP_A(3,2)*IVP_A(2,3);
    float c4 = IVP_A(2,1)*IVP_A(3,3) - IVP_A(3,1)*IVP_A(2,3);
    float c3 = IVP_A(2,1)*IVP_A(3,2) - IVP_A(3,1)*IVP_A(2,2);
    float c2 = IVP_A(2,0)*IVP_A(3,3) - IVP_A(3,0)*IVP_A(2,3);
    float c1 = IVP_A(2,0)*IVP_A(3,2) - IVP_A(3,0)*IVP_A(2,2);
    float c0 = IVP_A(2,0)*IVP_A(3,1) - IVP_A(3,0)*IVP_A(2,1);

    float det = s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0;
    if (det > -1e-12f && det < 1e-12f) return false;
    float d = 1.0f / det;

    float t[16];
    t[0]  = ( IVP_A(1,1)*c5 - IVP_A(1,2)*c4 + IVP_A(1,3)*c3) * d;
    t[1]  = (-IVP_A(0,1)*c5 + IVP_A(0,2)*c4 - IVP_A(0,3)*c3) * d;
    t[2]  = ( IVP_A(3,1)*s5 - IVP_A(3,2)*s4 + IVP_A(3,3)*s3) * d;
    t[3]  = (-IVP_A(2,1)*s5 + IVP_A(2,2)*s4 - IVP_A(2,3)*s3) * d;

    t[4]  = (-IVP_A(1,0)*c5 + IVP_A(1,2)*c2 - IVP_A(1,3)*c1) * d;
    t[5]  = ( IVP_A(0,0)*c5 - IVP_A(0,2)*c2 + IVP_A(0,3)*c1) * d;
    t[6]  = (-IVP_A(3,0)*s5 + IVP_A(3,2)*s2 - IVP_A(3,3)*s1) * d;
    t[7]  = ( IVP_A(2,0)*s5 - IVP_A(2,2)*s2 + IVP_A(2,3)*s1) * d;

    t[8]  = ( IVP_A(1,0)*c4 - IVP_A(1,1)*c2 + IVP_A(1,3)*c0) * d;
    t[9]  = (-IVP_A(0,0)*c4 + IVP_A(0,1)*c2 - IVP_A(0,3)*c0) * d;
    t[10] = ( IVP_A(3,0)*s4 - IVP_A(3,1)*s2 + IVP_A(3,3)*s0) * d;
    t[11] = (-IVP_A(2,0)*s4 + IVP_A(2,1)*s2 - IVP_A(2,3)*s0) * d;

    t[12] = (-IVP_A(1,0)*c3 + IVP_A(1,1)*c1 - IVP_A(1,2)*c0) * d;
    t[13] = ( IVP_A(0,0)*c3 - IVP_A(0,1)*c1 + IVP_A(0,2)*c0) * d;
    t[14] = (-IVP_A(3,0)*s3 + IVP_A(3,1)*s1 - IVP_A(3,2)*s0) * d;
    t[15] = ( IVP_A(2,0)*s3 - IVP_A(2,1)*s1 + IVP_A(2,2)*s0) * d;
#undef IVP_A
    memcpy(out, t, sizeof(t));
    return true;
}

/* ── Float vec3 helpers ────────────────────────────────────────────────── */

static inline void ivp_render_vec3_sub(float out[3],
                                       const float a[3],
                                       const float b[3])
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

static inline void ivp_render_vec3_add(float out[3],
                                       const float a[3],
                                       const float b[3])
{
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
}

static inline void ivp_render_vec3_scale(float out[3],
                                         const float v[3],
                                         float s)
{
    out[0] = v[0] * s;
    out[1] = v[1] * s;
    out[2] = v[2] * s;
}

static inline float ivp_render_vec3_dot(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static inline void ivp_render_vec3_cross(float out[3],
                                         const float a[3],
                                         const float b[3])
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

static inline void ivp_render_vec3_normalize(float v[3])
{
    float l = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (l > 1e-8f) { v[0] /= l; v[1] /= l; v[2] /= l; }
}

static inline void ivp_render_mat4_lookat(float out[16],
                                          const float eye[3],
                                          const float target[3],
                                          const float up[3])
{
    float f[3], s[3], u[3];
    ivp_render_vec3_sub(f, target, eye);
    ivp_render_vec3_normalize(f);
    ivp_render_vec3_cross(s, f, up);
    ivp_render_vec3_normalize(s);
    ivp_render_vec3_cross(u, s, f);

    ivp_render_mat4_identity(out);
    out[0] = s[0]; out[4] = s[1]; out[8]  = s[2];
    out[1] = u[0]; out[5] = u[1]; out[9]  = u[2];
    out[2] =-f[0]; out[6] =-f[1]; out[10] =-f[2];
    out[12] = -(s[0]*eye[0] + s[1]*eye[1] + s[2]*eye[2]);
    out[13] = -(u[0]*eye[0] + u[1]*eye[1] + u[2]*eye[2]);
    out[14] =  (f[0]*eye[0] + f[1]*eye[1] + f[2]*eye[2]);
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_MATH_H */
