/* ivp_vec3.h — 3D vector types for IVP C17
 *
 * ivp_vec3r_t — follows ivp_real_t (solver/dynamics precision)
 * ivp_vec3f_t — always float (compact geometry / storage)
 * ivp_vec3d_t — always double (extra precision when needed)
 */
#ifndef IVP_VEC3_H
#define IVP_VEC3_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Types ──────────────────────────────────────────────────────────────── */

typedef struct IVP_ALIGN16 ivp_vec3f {
    float k[3];
} ivp_vec3f_t;

typedef struct ivp_vec3d {
    double k[3];
} ivp_vec3d_t;

typedef struct IVP_ALIGN16 ivp_vec3r {
    ivp_real_t k[3];
} ivp_vec3r_t;

/* 4-component (for planes: normal + distance) */
typedef struct IVP_ALIGN16 ivp_vec4f {
    float k[3];
    float w;
} ivp_vec4f_t;

typedef struct ivp_vec4r {
    ivp_real_t k[3];
    ivp_real_t w;
} ivp_vec4r_t;

/* ── Construction ───────────────────────────────────────────────────────── */

static inline ivp_vec3r_t ivp_vec3r(ivp_real_t x, ivp_real_t y, ivp_real_t z) {
    ivp_vec3r_t v;
    v.k[0] = x; v.k[1] = y; v.k[2] = z;
    return v;
}

static inline ivp_vec3r_t ivp_vec3r_zero(void) {
    ivp_vec3r_t v;
    v.k[0] = v.k[1] = v.k[2] = IVP_REAL(0.0);
    return v;
}

static inline ivp_vec3f_t ivp_vec3f(float x, float y, float z) {
    ivp_vec3f_t v;
    v.k[0] = x; v.k[1] = y; v.k[2] = z;
    return v;
}

/* ── Conversion ─────────────────────────────────────────────────────────── */

static inline ivp_vec3r_t ivp_vec3r_from_f(const ivp_vec3f_t *v) {
    return ivp_vec3r((ivp_real_t)v->k[0], (ivp_real_t)v->k[1], (ivp_real_t)v->k[2]);
}

static inline ivp_vec3f_t ivp_vec3f_from_r(const ivp_vec3r_t *v) {
    return ivp_vec3f((float)v->k[0], (float)v->k[1], (float)v->k[2]);
}

/* ── Component access ───────────────────────────────────────────────────── */

#define ivp_vec3r_x(v) ((v)->k[0])
#define ivp_vec3r_y(v) ((v)->k[1])
#define ivp_vec3r_z(v) ((v)->k[2])

/* ── Arithmetic (inline) ────────────────────────────────────────────────── */

static inline void ivp_vec3r_set(ivp_vec3r_t *out, ivp_real_t x, ivp_real_t y, ivp_real_t z) {
    out->k[0] = x; out->k[1] = y; out->k[2] = z;
}

static inline void ivp_vec3r_set_zero(ivp_vec3r_t *out) {
    out->k[0] = out->k[1] = out->k[2] = IVP_REAL(0.0);
}

static inline void ivp_vec3r_copy(ivp_vec3r_t *out, const ivp_vec3r_t *a) {
    out->k[0] = a->k[0]; out->k[1] = a->k[1]; out->k[2] = a->k[2];
}

static inline void ivp_vec3r_negate(ivp_vec3r_t *out, const ivp_vec3r_t *a) {
    out->k[0] = -a->k[0]; out->k[1] = -a->k[1]; out->k[2] = -a->k[2];
}

static inline void ivp_vec3r_add(ivp_vec3r_t *out, const ivp_vec3r_t *a,
                                  const ivp_vec3r_t *b) {
    out->k[0] = a->k[0] + b->k[0];
    out->k[1] = a->k[1] + b->k[1];
    out->k[2] = a->k[2] + b->k[2];
}

static inline void ivp_vec3r_sub(ivp_vec3r_t *out, const ivp_vec3r_t *a,
                                  const ivp_vec3r_t *b) {
    out->k[0] = a->k[0] - b->k[0];
    out->k[1] = a->k[1] - b->k[1];
    out->k[2] = a->k[2] - b->k[2];
}

static inline void ivp_vec3r_scale(ivp_vec3r_t *out, const ivp_vec3r_t *a,
                                    ivp_real_t s) {
    out->k[0] = a->k[0] * s;
    out->k[1] = a->k[1] * s;
    out->k[2] = a->k[2] * s;
}

/* out += v * factor */
static inline void ivp_vec3r_add_scaled(ivp_vec3r_t *out, const ivp_vec3r_t *v,
                                         ivp_real_t factor) {
    out->k[0] += v->k[0] * factor;
    out->k[1] += v->k[1] * factor;
    out->k[2] += v->k[2] * factor;
}

static inline ivp_real_t ivp_vec3r_dot(const ivp_vec3r_t *a, const ivp_vec3r_t *b) {
    return a->k[0]*b->k[0] + a->k[1]*b->k[1] + a->k[2]*b->k[2];
}

static inline void ivp_vec3r_cross(ivp_vec3r_t *out, const ivp_vec3r_t *a,
                                    const ivp_vec3r_t *b) {
    ivp_real_t x = a->k[1]*b->k[2] - a->k[2]*b->k[1];
    ivp_real_t y = a->k[2]*b->k[0] - a->k[0]*b->k[2];
    ivp_real_t z = a->k[0]*b->k[1] - a->k[1]*b->k[0];
    out->k[0] = x; out->k[1] = y; out->k[2] = z;
}

static inline ivp_real_t ivp_vec3r_quad_length(const ivp_vec3r_t *v) {
    return v->k[0]*v->k[0] + v->k[1]*v->k[1] + v->k[2]*v->k[2];
}

static inline ivp_real_t ivp_vec3r_quad_distance(const ivp_vec3r_t *a,
                                                   const ivp_vec3r_t *b) {
    ivp_real_t dx = a->k[0] - b->k[0];
    ivp_real_t dy = a->k[1] - b->k[1];
    ivp_real_t dz = a->k[2] - b->k[2];
    return dx*dx + dy*dy + dz*dz;
}

/* out->k[i] = a->k[i] * b->k[i] */
static inline void ivp_vec3r_pairwise_mult(ivp_vec3r_t *out, const ivp_vec3r_t *a,
                                            const ivp_vec3r_t *b) {
    out->k[0] = a->k[0] * b->k[0];
    out->k[1] = a->k[1] * b->k[1];
    out->k[2] = a->k[2] * b->k[2];
}

/* out = a * (1 - t) + b * t */
static inline void ivp_vec3r_lerp(ivp_vec3r_t *out, const ivp_vec3r_t *a,
                                   const ivp_vec3r_t *b, ivp_real_t t) {
    ivp_real_t omt = IVP_REAL(1.0) - t;
    out->k[0] = a->k[0] * omt + b->k[0] * t;
    out->k[1] = a->k[1] * omt + b->k[1] * t;
    out->k[2] = a->k[2] * omt + b->k[2] * t;
}

/* ── Non-inline functions (in ivp_vec3.c) ───────────────────────────────── */

/* Precise length = sqrt(quad_length) */
ivp_real_t ivp_vec3r_length(const ivp_vec3r_t *v);

/* Fast length approximation (0.1% error) */
ivp_real_t ivp_vec3r_fast_length(const ivp_vec3r_t *v);

/* Normalize (precise). Returns IVP_ERR_NORMALIZATION_FAILED if zero-length. */
ivp_result_t ivp_vec3r_normalize(ivp_vec3r_t *v);

/* Fast normalize (Newton-Raphson rsqrt, ~0.1% error). No failure check. */
void ivp_vec3r_fast_normalize(ivp_vec3r_t *v);

/* Normalize and return former length. */
ivp_real_t ivp_vec3r_normalize_and_length(ivp_vec3r_t *v);

#ifdef __cplusplus
}
#endif

#endif /* IVP_VEC3_H */
