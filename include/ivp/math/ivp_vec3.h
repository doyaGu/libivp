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

/* out = a * (1 - t) + b * t
 * Reference: IVP_U_Point / IVP_U_Float_Point::set_interpolate(p0, p1, s):
 *   is = 1 - s; k = p0*is + p1*s, evaluated in IVP_DOUBLE (the float
 *   variant rounds the double result to IVP_FLOAT). */
static inline void ivp_vec3r_lerp(ivp_vec3r_t *out, const ivp_vec3r_t *a,
                                   const ivp_vec3r_t *b, ivp_real_t t) {
    double s  = (double)t;
    double is = 1.0 - s;
    double x = (double)a->k[0] * is + (double)b->k[0] * s;
    double y = (double)a->k[1] * is + (double)b->k[1] * s;
    double z = (double)a->k[2] * is + (double)b->k[2] * s;
    out->k[0] = (ivp_real_t)x;
    out->k[1] = (ivp_real_t)y;
    out->k[2] = (ivp_real_t)z;
}

/* ── Non-inline functions (in ivp_vec3.c) ───────────────────────────────── *
 *
 * ivp_vec3r_* helpers follow IVP_U_Float_Point semantics in the default
 * (float) build and IVP_U_Point semantics with IVP_REAL_DOUBLE.  The
 * explicit ivp_vec3f_* (IVP_U_Float_Point) and ivp_vec3d_* (IVP_U_Point)
 * variants below always use the respective reference class semantics.
 * The two classes differ in:
 *   quad_length  float arithmetic (Float_Point) vs double (Point)
 *   normize      isqrt(ql, 2)     (Float_Point) vs isqrt(ql, 3) (Point)
 * fast_normize uses isqrt(ql, 2), real_length_plus_normize isqrt(ql, 3)
 * and real_length / fast_real_length an exact sqrt in both classes. */

/* real_length(): sqrt(quad_length) */
ivp_real_t ivp_vec3r_length(const ivp_vec3r_t *v);

/* fast_real_length(): IVP_Fast_Math::sqrt(quad_length) — an exact sqrt in
 * the reference double build (ivu_linear.cxx), identical to real_length. */
ivp_real_t ivp_vec3r_fast_length(const ivp_vec3r_t *v);

/* normize(): returns IVP_ERR_NORMALIZATION_FAILED (vector unchanged) if
 * quad_length < P_DOUBLE_EPS, else v *= isqrt(quad_length, 2 or 3). */
ivp_result_t ivp_vec3r_normalize(ivp_vec3r_t *v);

/* fast_normize(): like normize but always isqrt(quad_length, 2). */
ivp_result_t ivp_vec3r_fast_normalize(ivp_vec3r_t *v);

/* real_length_plus_normize(): returns 0 (vector unchanged) if
 * quad_length < P_DOUBLE_EPS, else f = isqrt(ql, 3); v *= f; return f*ql. */
ivp_real_t ivp_vec3r_normalize_and_length(ivp_vec3r_t *v);

/* inline_calc_cross_product_and_normize(): out = normize(a x b).
 * Note: the IVP_U_Point variant rounds the cross product components to
 * IVP_FLOAT before normizing (reference quirk, reproduced). */
ivp_result_t ivp_vec3r_cross_and_normalize(ivp_vec3r_t *out,
                                            const ivp_vec3r_t *a,
                                            const ivp_vec3r_t *b);

/* inline_set_vert_to_area_defined_by_three_points(): out = (tp2-tp0) x
 * (tp1-tp0) (not normized), evaluated in IVP_DOUBLE. */
void ivp_vec3r_set_vert_to_area(ivp_vec3r_t *out, const ivp_vec3r_t *tp0,
                                const ivp_vec3r_t *tp1, const ivp_vec3r_t *tp2);

/* set_orthogonal_part(vector, normal_v): out = vector - normal_v *
 * (vector . normal_v)  (project vector on the plane given by normal_v). */
void ivp_vec3r_set_orthogonal_part(ivp_vec3r_t *out, const ivp_vec3r_t *vector,
                                   const ivp_vec3r_t *normal_v);

/* IVP_U_Point::is_parallel(v2, eps): |v1 x v2|^2 <= eps^2 |v1|^2 |v2|^2,
 * evaluated on IVP_U_Point (double) copies. */
bool ivp_vec3r_is_parallel(const ivp_vec3r_t *v1, const ivp_vec3r_t *v2,
                           double eps);

/* IVP_U_Point::calc_an_orthogonal(ip): copy ip, negate the coordinate with
 * the biggest |value| (first found scanning z, y, x), swap it with the
 * preceding coordinate (cyclic), then out = out x ip. */
void ivp_vec3r_calc_an_orthogonal(ivp_vec3r_t *out, const ivp_vec3r_t *ip);

/* IVP_U_Point::solve_quadratic_equation_fast / _accurate(a):
 * solves a.k[0]*x^2 + a.k[1]*x + a.k[2] = 0.
 * out.k[0] = discriminant (< 0: no result), out.k[1] = smaller root
 * (for a.k[0] > 0), out.k[2] = greater root.  Linear equations
 * (|a.k[0]| < P_DOUBLE_EPS) set k[0] = 0 and k[1] = k[2] = -c/b, or
 * k[0] = -1 if |a.k[1]| < P_DOUBLE_EPS too.  The fast variant only writes
 * the roots for discriminant > 0, the accurate one for >= 0. */
void ivp_vec3r_solve_quadratic_fast(ivp_vec3r_t *out, const ivp_vec3r_t *a);
void ivp_vec3r_solve_quadratic_accurate(ivp_vec3r_t *out, const ivp_vec3r_t *a);

/* IVP_U_Point::set_crossing(h0, h1, h2): intersection point of three
 * planes given in reference Hesse form (k = normal, w = hesse_val, points
 * satisfy k.p + w = 0).  Solved via IVP_U_Matrix::real_invert (P_DOUBLE_EPS)
 * in double.  Returns IVP_ERR_SINGULAR_MATRIX when the inversion fails —
 * like the reference, `out` is then still computed with the NON-inverted
 * matrix. */
ivp_result_t ivp_vec3r_set_crossing(ivp_vec3r_t *out, const ivp_vec4r_t *h0,
                                    const ivp_vec4r_t *h1, const ivp_vec4r_t *h2);

/* rotate(axis, angle): rotate v around the origin; angle is IVP_FLOAT. */
void ivp_vec3r_rotate(ivp_vec3r_t *v, ivp_coord_index_t axis, float angle);

/* line_min / line_max: component-wise min / max into v. */
void ivp_vec3r_line_min(ivp_vec3r_t *v, const ivp_vec3r_t *p);
void ivp_vec3r_line_max(ivp_vec3r_t *v, const ivp_vec3r_t *p);

/* IVP_U_Float_Point::line_sqrt: k[i] = sqrt(k[i]). */
void ivp_vec3r_line_sqrt(ivp_vec3r_t *v);

/* ── IVP_U_Float_Point (always float) ───────────────────────────────────── */

double       ivp_vec3f_quad_length(const ivp_vec3f_t *v);   /* float arithmetic */
double       ivp_vec3f_real_length(const ivp_vec3f_t *v);
double       ivp_vec3f_fast_real_length(const ivp_vec3f_t *v);
ivp_result_t ivp_vec3f_normize(ivp_vec3f_t *v);              /* isqrt(ql, 2) */
ivp_result_t ivp_vec3f_fast_normize(ivp_vec3f_t *v);         /* isqrt(ql, 2) */
double       ivp_vec3f_real_length_plus_normize(ivp_vec3f_t *v); /* isqrt(ql, 3) */
void         ivp_vec3f_mult(ivp_vec3f_t *v, double factor);  /* k = (float)(k*factor) */

/* ── IVP_U_Point (always double) ────────────────────────────────────────── */

double       ivp_vec3d_quad_length(const ivp_vec3d_t *v);
double       ivp_vec3d_real_length(const ivp_vec3d_t *v);
double       ivp_vec3d_fast_real_length(const ivp_vec3d_t *v);
ivp_result_t ivp_vec3d_normize(ivp_vec3d_t *v);              /* isqrt(ql, 3) */
ivp_result_t ivp_vec3d_fast_normize(ivp_vec3d_t *v);         /* isqrt(ql, 2) */
double       ivp_vec3d_real_length_plus_normize(ivp_vec3d_t *v); /* isqrt(ql, 3) */
void         ivp_vec3d_cross(ivp_vec3d_t *out, const ivp_vec3d_t *a,
                             const ivp_vec3d_t *b);

#ifdef __cplusplus
}
#endif

#endif /* IVP_VEC3_H */
