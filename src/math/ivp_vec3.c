/* ivp_vec3.c — Vector operations implementation */
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>

ivp_real_t ivp_vec3r_length(const ivp_vec3r_t *v) {
    return ivp_sqrt(ivp_vec3r_quad_length(v));
}

ivp_real_t ivp_vec3r_fast_length(const ivp_vec3r_t *v) {
    ivp_real_t ql = ivp_vec3r_quad_length(v);
    if (ql < IVP_REAL_EPS) return IVP_REAL(0.0);
#ifdef IVP_REAL_DOUBLE
    return ql * ivp_isqrt_double(ql, 0);  /* ~0.1% error */
#else
    return ql * ivp_isqrt_float(ql);
#endif
}

ivp_result_t ivp_vec3r_normalize(ivp_vec3r_t *v) {
    ivp_real_t ql = ivp_vec3r_quad_length(v);
    if (ql < IVP_REAL_EPS) return IVP_ERR_NORMALIZATION_FAILED;
    ivp_real_t inv_len = IVP_REAL(1.0) / ivp_sqrt(ql);
    v->k[0] *= inv_len;
    v->k[1] *= inv_len;
    v->k[2] *= inv_len;
    return IVP_OK;
}

void ivp_vec3r_fast_normalize(ivp_vec3r_t *v) {
    ivp_real_t ql = ivp_vec3r_quad_length(v);
    IVP_ASSERT(ql > IVP_REAL_EPS);
#ifdef IVP_REAL_DOUBLE
    /* Reference IVP_Inline_Math::isqrt_double uses 3 resolution steps. */
    ivp_real_t inv_len = ivp_isqrt_double(ql, 3);
#else
    ivp_real_t inv_len = (ivp_real_t)ivp_isqrt_float((float)ql);
#endif
    v->k[0] *= inv_len;
    v->k[1] *= inv_len;
    v->k[2] *= inv_len;
}

ivp_real_t ivp_vec3r_normalize_and_length(ivp_vec3r_t *v) {
    ivp_real_t ql = ivp_vec3r_quad_length(v);
    if (ql < IVP_REAL_EPS) return IVP_REAL(0.0);
    ivp_real_t len = ivp_sqrt(ql);
    ivp_real_t inv_len = IVP_REAL(1.0) / len;
    v->k[0] *= inv_len;
    v->k[1] *= inv_len;
    v->k[2] *= inv_len;
    return len;
}
