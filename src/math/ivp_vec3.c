/* ivp_vec3.c — Vector operations implementation
 *
 * Reference: IVP_U_Point / IVP_U_Float_Point (ivu_linear.cxx,
 * ivu_linear_software.hxx, ivu_linear_double.hxx).  The reference library
 * is the double build (P_DOUBLE_EPS = 1e-10, IVP_U_Point = double).
 *
 * ivp_vec3r_* follows IVP_U_Float_Point in the float build and IVP_U_Point
 * with IVP_REAL_DOUBLE; helpers that only exist on IVP_U_Point
 * (is_parallel, calc_an_orthogonal, solve_quadratic_*) are always evaluated
 * in double like the reference.
 */
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>

/* isqrt resolution of normize(): IVP_U_Float_Point uses 2, IVP_U_Point 3 */
#ifdef IVP_REAL_DOUBLE
#define IVP_VEC3R_NORMIZE_STEPS 3
#else
#define IVP_VEC3R_NORMIZE_STEPS 2
#endif

/* quad_length() as the reference class computes it: float arithmetic for
 * IVP_U_Float_Point (promoted to IVP_DOUBLE on return), double for
 * IVP_U_Point. */
static inline double vec3r_quad_length_ref(const ivp_vec3r_t *v) {
    return (double)(v->k[0] * v->k[0] + v->k[1] * v->k[1] + v->k[2] * v->k[2]);
}

/* mult(IVP_DOUBLE factor): k = (real)(k * factor) evaluated in double. */
static inline void vec3r_mult_ref(ivp_vec3r_t *v, double f) {
    double a = (double)v->k[0] * f;
    double b = (double)v->k[1] * f;
    double c = (double)v->k[2] * f;
    v->k[0] = (ivp_real_t)a;
    v->k[1] = (ivp_real_t)b;
    v->k[2] = (ivp_real_t)c;
}

ivp_real_t ivp_vec3r_length(const ivp_vec3r_t *v) {
    /* IVP_Inline_Math::sqrtd(this->quad_length()) */
    return (ivp_real_t)sqrt(vec3r_quad_length_ref(v));
}

ivp_real_t ivp_vec3r_fast_length(const ivp_vec3r_t *v) {
    /* IVP_Fast_Math::sqrt(quad_length) == ::sqrt (ivu_float.hxx) */
    return (ivp_real_t)ivp_fast_sqrt(vec3r_quad_length_ref(v));
}

ivp_result_t ivp_vec3r_normalize(ivp_vec3r_t *v) {
    double length = vec3r_quad_length_ref(v);
    if (length < IVP_REF_DOUBLE_EPS) return IVP_ERR_NORMALIZATION_FAILED;
    double f = ivp_isqrt_double(length, IVP_VEC3R_NORMIZE_STEPS);
    vec3r_mult_ref(v, f);
    return IVP_OK;
}

ivp_result_t ivp_vec3r_fast_normalize(ivp_vec3r_t *v) {
    double length = vec3r_quad_length_ref(v);
    if (length < IVP_REF_DOUBLE_EPS) return IVP_ERR_NORMALIZATION_FAILED;
    double f = ivp_isqrt_double(length, 2);
    vec3r_mult_ref(v, f);
    return IVP_OK;
}

ivp_real_t ivp_vec3r_normalize_and_length(ivp_vec3r_t *v) {
    double qlength = vec3r_quad_length_ref(v);
    if (qlength < IVP_REF_DOUBLE_EPS) {
        return IVP_REAL(0.0);
    }
    double f = ivp_isqrt_double(qlength, 3);
    vec3r_mult_ref(v, f);
    return (ivp_real_t)(f * qlength);
}

ivp_result_t ivp_vec3r_cross_and_normalize(ivp_vec3r_t *out,
                                            const ivp_vec3r_t *v1,
                                            const ivp_vec3r_t *v2) {
#ifdef IVP_REAL_DOUBLE
    /* IVP_U_Point::inline_calc_cross_product_and_normize: components are
     * stored through (IVP_FLOAT) before normize() (ivu_linear_double.hxx). */
    double a = v1->k[1] * v2->k[2];
    double a2 = v2->k[1] * v1->k[2];
    double b = v1->k[2] * v2->k[0];
    double b2 = v2->k[2] * v1->k[0];
    a -= a2;
    double c = v1->k[0] * v2->k[1];
    double c2 = v2->k[0] * v1->k[1];
    b -= b2;
    c -= c2;
    out->k[0] = (double)(float)a;
    out->k[1] = (double)(float)b;
    out->k[2] = (double)(float)c;
#else
    /* IVP_U_Float_Point::inline_calc_cross_product_and_normize: float
     * products, double differences, rounded to float. */
    double a = (double)(v1->k[1] * v2->k[2]);
    double a2 = (double)(v2->k[1] * v1->k[2]);
    double b = (double)(v1->k[2] * v2->k[0]);
    double b2 = (double)(v2->k[2] * v1->k[0]);
    a -= a2;
    double c = (double)(v1->k[0] * v2->k[1]);
    double c2 = (double)(v2->k[0] * v1->k[1]);
    b -= b2;
    c -= c2;
    out->k[0] = (float)a;
    out->k[1] = (float)b;
    out->k[2] = (float)c;
#endif
    return ivp_vec3r_normalize(out);
}

void ivp_vec3r_set_vert_to_area(ivp_vec3r_t *out, const ivp_vec3r_t *tp0,
                                const ivp_vec3r_t *tp1, const ivp_vec3r_t *tp2) {
    double fp0 = (double)tp0->k[0];
    double fp1 = (double)tp0->k[1];
    double fp2 = (double)tp0->k[2];
    double a0 = (double)tp1->k[0] - fp0;
    double a1 = (double)tp1->k[1] - fp1;
    double a2 = (double)tp1->k[2] - fp2;
    double b0 = (double)tp2->k[0] - fp0;
    double b1 = (double)tp2->k[1] - fp1;
    double b2 = (double)tp2->k[2] - fp2;

    double c0 = b1 * a2 - a1 * b2;
    double c1 = b2 * a0 - a2 * b0;
    double c2 = b0 * a1 - a0 * b1;

    out->k[0] = (ivp_real_t)c0;
    out->k[1] = (ivp_real_t)c1;
    out->k[2] = (ivp_real_t)c2;
}

void ivp_vec3r_set_orthogonal_part(ivp_vec3r_t *out, const ivp_vec3r_t *vector,
                                   const ivp_vec3r_t *normal_v) {
    /* part_direction = vector . normal_v (Float_Point: float arithmetic) */
    double part_direction = (double)(vector->k[0] * normal_v->k[0] +
                                     vector->k[1] * normal_v->k[1] +
                                     vector->k[2] * normal_v->k[2]);
    /* add_multiple(vector, normal_v, -part_direction) */
    double f = -part_direction;
    double a = (double)normal_v->k[0] * f;
    double b = (double)normal_v->k[1] * f;
    a += (double)vector->k[0];
    double c = (double)normal_v->k[2] * f;
    b += (double)vector->k[1];
    c += (double)vector->k[2];
    out->k[0] = (ivp_real_t)a;
    out->k[1] = (ivp_real_t)b;
    out->k[2] = (ivp_real_t)c;
}

bool ivp_vec3r_is_parallel(const ivp_vec3r_t *v_this, const ivp_vec3r_t *v_in,
                           double eps) {
    ivp_vec3d_t v1, v2, c;
    for (int i = 0; i < 3; i++) {
        v1.k[i] = (double)v_this->k[i];
        v2.k[i] = (double)v_in->k[i];
    }
    ivp_vec3d_cross(&c, &v1, &v2);
    return ivp_vec3d_quad_length(&c) <=
           eps * eps * ivp_vec3d_quad_length(&v1) * ivp_vec3d_quad_length(&v2);
}

void ivp_vec3r_calc_an_orthogonal(ivp_vec3r_t *out, const ivp_vec3r_t *ip_in) {
    ivp_vec3d_t ip, k;
    for (int i = 0; i < 3; i++) {
        ip.k[i] = (double)ip_in->k[i];
    }
    k = ip;

    int max_pos = 0;
    double max = 0.0;
    for (int i = 2; i >= 0; i--) {
        double h = fabs(k.k[i]);
        if (h > max) {
            max = h;
            max_pos = i;
        }
    }
    k.k[max_pos] *= -1;
    int other_pos = max_pos - 1;
    if (other_pos < 0) other_pos = 2;
    double h = k.k[max_pos];
    k.k[max_pos] = k.k[other_pos];
    k.k[other_pos] = h;

    ivp_vec3d_t r;
    ivp_vec3d_cross(&r, &k, &ip);
    out->k[0] = (ivp_real_t)r.k[0];
    out->k[1] = (ivp_real_t)r.k[1];
    out->k[2] = (ivp_real_t)r.k[2];
}

static void solve_quadratic(ivp_vec3r_t *out, const ivp_vec3r_t *p, bool accurate) {
    double pa = (double)p->k[0];
    double pb = (double)p->k[1];
    double pc = (double)p->k[2];
    if (fabs(pa) < IVP_REF_DOUBLE_EPS) { /* linear equation */
        out->k[0] = IVP_REAL(0.0);
        if (fabs(pb) < IVP_REF_DOUBLE_EPS) {
            out->k[0] = IVP_REAL(-1.0); /* no result */
            return;
        }
        ivp_real_t r = (ivp_real_t)(-pc / pb);
        out->k[1] = r;
        out->k[2] = r;
        return;
    }
    double b2 = pb * pb;
    double ac4 = pa * pc * 4.0;
    double k0 = b2 - ac4;
    out->k[0] = (ivp_real_t)k0;
    if (accurate ? (k0 >= 0) : (k0 > 0)) {
        double i2a = .5 / pa;
        double sr = accurate ? sqrt(k0) : ivp_fast_sqrt(k0);
        double l1 = (-pb - sr) * i2a;
        double l2 = (-pb + sr) * i2a;
        out->k[1] = (ivp_real_t)l1;
        out->k[2] = (ivp_real_t)l2;
    }
}

void ivp_vec3r_solve_quadratic_fast(ivp_vec3r_t *out, const ivp_vec3r_t *a) {
    solve_quadratic(out, a, false);
}

void ivp_vec3r_solve_quadratic_accurate(ivp_vec3r_t *out, const ivp_vec3r_t *a) {
    solve_quadratic(out, a, true);
}

ivp_result_t ivp_vec3r_set_crossing(ivp_vec3r_t *out, const ivp_vec4r_t *h0,
                                    const ivp_vec4r_t *h1, const ivp_vec4r_t *h2) {
    /* lin_equ_matrix.init_rows4(h0, h1, h2, 0); rval = real_invert(); */
    ivp_vec3d_t rows[3];
    const ivp_vec4r_t *hs[3] = { h0, h1, h2 };
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) rows[r].k[c] = (double)hs[r]->k[c];
    }
    double m[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) m[r][c] = rows[r].k[c];

    ivp_result_t rval = IVP_ERR_SINGULAR_MATRIX;
    ivp_vec3d_t r0, r1, r2;
    ivp_vec3d_cross(&r0, &rows[1], &rows[2]);
    ivp_vec3d_cross(&r1, &rows[2], &rows[0]);
    ivp_vec3d_cross(&r2, &rows[0], &rows[1]);
    double D = rows[0].k[0] * r0.k[0] + rows[0].k[1] * r0.k[1] + rows[0].k[2] * r0.k[2];
    if (!(fabs(D) < IVP_REF_DOUBLE_EPS)) {
        double DI = 1.0 / D;
        for (int i = 0; i < 3; i++) {
            m[i][0] = r0.k[i] * DI; /* set_col(IVP_INDEX_X, r0 * DI) */
            m[i][1] = r1.k[i] * DI;
            m[i][2] = r2.k[i] * DI;
        }
        rval = IVP_OK;
    }
    /* right_side = (-h0.hesse_val, -h1.hesse_val, -h2.hesse_val);
     * lin_equ_matrix.vmult3(&right_side, this) */
    double rs[3] = { -(double)h0->w, -(double)h1->w, -(double)h2->w };
    for (int i = 0; i < 3; i++) {
        out->k[i] = (ivp_real_t)(m[i][0] * rs[0] + m[i][1] * rs[1] + m[i][2] * rs[2]);
    }
    return rval;
}

void ivp_vec3r_rotate(ivp_vec3r_t *v, ivp_coord_index_t axis, float angle) {
    static const int hp[5] = {0, 1, 2, 0, 1};
    const int *p = &hp[axis];
    double c = cos((double)angle);
    double s = sin((double)angle);
    double x_new = c * (double)v->k[p[1]] - s * (double)v->k[p[2]];
    v->k[p[2]] = (ivp_real_t)(s * (double)v->k[p[1]] + c * (double)v->k[p[2]]);
    v->k[p[1]] = (ivp_real_t)x_new;
}

void ivp_vec3r_line_min(ivp_vec3r_t *v, const ivp_vec3r_t *p) {
    for (int i = 2; i >= 0; i--) {
        if (p->k[i] < v->k[i]) v->k[i] = p->k[i];
    }
}

void ivp_vec3r_line_max(ivp_vec3r_t *v, const ivp_vec3r_t *p) {
    for (int i = 2; i >= 0; i--) {
        if (p->k[i] > v->k[i]) v->k[i] = p->k[i];
    }
}

void ivp_vec3r_line_sqrt(ivp_vec3r_t *v) {
    v->k[0] = (ivp_real_t)sqrt((double)v->k[0]);
    v->k[1] = (ivp_real_t)sqrt((double)v->k[1]);
    v->k[2] = (ivp_real_t)sqrt((double)v->k[2]);
}

/* ── IVP_U_Float_Point ──────────────────────────────────────────────────── */

double ivp_vec3f_quad_length(const ivp_vec3f_t *v) {
    return (double)(v->k[0] * v->k[0] + v->k[1] * v->k[1] + v->k[2] * v->k[2]);
}

double ivp_vec3f_real_length(const ivp_vec3f_t *v) {
    return sqrt(ivp_vec3f_quad_length(v));
}

double ivp_vec3f_fast_real_length(const ivp_vec3f_t *v) {
    return ivp_fast_sqrt(ivp_vec3f_quad_length(v));
}

void ivp_vec3f_mult(ivp_vec3f_t *v, double factor) {
    double a = (double)v->k[0] * factor;
    double b = (double)v->k[1] * factor;
    double c = (double)v->k[2] * factor;
    v->k[0] = (float)a;
    v->k[1] = (float)b;
    v->k[2] = (float)c;
}

ivp_result_t ivp_vec3f_normize(ivp_vec3f_t *v) {
    double length = ivp_vec3f_quad_length(v);
    if (length < IVP_REF_DOUBLE_EPS) return IVP_ERR_NORMALIZATION_FAILED;
    ivp_vec3f_mult(v, ivp_isqrt_double(length, 2));
    return IVP_OK;
}

ivp_result_t ivp_vec3f_fast_normize(ivp_vec3f_t *v) {
    double length = ivp_vec3f_quad_length(v);
    if (length < IVP_REF_DOUBLE_EPS) return IVP_ERR_NORMALIZATION_FAILED;
    ivp_vec3f_mult(v, ivp_isqrt_double(length, 2));
    return IVP_OK;
}

double ivp_vec3f_real_length_plus_normize(ivp_vec3f_t *v) {
    double qlength = ivp_vec3f_quad_length(v);
    if (qlength < IVP_REF_DOUBLE_EPS) return 0.0;
    double f = ivp_isqrt_double(qlength, 3);
    ivp_vec3f_mult(v, f);
    return f * qlength;
}

/* ── IVP_U_Point ────────────────────────────────────────────────────────── */

double ivp_vec3d_quad_length(const ivp_vec3d_t *v) {
    return v->k[0] * v->k[0] + v->k[1] * v->k[1] + v->k[2] * v->k[2];
}

double ivp_vec3d_real_length(const ivp_vec3d_t *v) {
    return sqrt(ivp_vec3d_quad_length(v));
}

double ivp_vec3d_fast_real_length(const ivp_vec3d_t *v) {
    return ivp_fast_sqrt(ivp_vec3d_quad_length(v));
}

static inline void vec3d_mult(ivp_vec3d_t *v, double factor) {
    v->k[0] *= factor;
    v->k[1] *= factor;
    v->k[2] *= factor;
}

ivp_result_t ivp_vec3d_normize(ivp_vec3d_t *v) {
    double length = ivp_vec3d_quad_length(v);
    if (length < IVP_REF_DOUBLE_EPS) return IVP_ERR_NORMALIZATION_FAILED;
    vec3d_mult(v, ivp_isqrt_double(length, 3));
    return IVP_OK;
}

ivp_result_t ivp_vec3d_fast_normize(ivp_vec3d_t *v) {
    double length = ivp_vec3d_quad_length(v);
    if (length < IVP_REF_DOUBLE_EPS) return IVP_ERR_NORMALIZATION_FAILED;
    vec3d_mult(v, ivp_isqrt_double(length, 2));
    return IVP_OK;
}

double ivp_vec3d_real_length_plus_normize(ivp_vec3d_t *v) {
    double qlength = ivp_vec3d_quad_length(v);
    if (qlength < IVP_REF_DOUBLE_EPS) return 0.0;
    double f = ivp_isqrt_double(qlength, 3);
    vec3d_mult(v, f);
    return f * qlength;
}

void ivp_vec3d_cross(ivp_vec3d_t *out, const ivp_vec3d_t *v1, const ivp_vec3d_t *v2) {
    /* IVP_U_Point::inline_calc_cross_product (aliasing-safe) */
    double a = v1->k[1] * v2->k[2];
    double a2 = v2->k[1] * v1->k[2];
    double b = v1->k[2] * v2->k[0];
    double b2 = v2->k[2] * v1->k[0];
    a -= a2;
    double c = v1->k[0] * v2->k[1];
    double c2 = v2->k[0] * v1->k[1];
    b -= b2;
    out->k[0] = a;
    out->k[1] = b;
    c -= c2;
    out->k[2] = c;
}
