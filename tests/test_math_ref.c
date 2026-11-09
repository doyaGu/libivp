/* test_math_ref.c — math layer parity against the original IVP engine
 *
 * Every expected value below was produced by running the ORIGINAL C++
 * implementation (ivp_utility/ivu_quat.cxx, ivu_linear.cxx, ivu_float.hxx,
 * ivu_linear_*.hxx, double build) compiled with -ffp-contract=off, and is
 * embedded as a hex-float literal.
 *
 * Comparison policy:
 *   REF_EQ   IVP_REAL_DOUBLE: bit-exact;  float build: bit-exact to the
 *            reference value rounded to float (libivp evaluates the
 *            reference IVP_DOUBLE formula in double and rounds once).
 *   REF_NEAR bit-exact with IVP_REAL_DOUBLE; float build within `ftol`
 *            (used where the float build feeds float-rounded intermediate
 *            state back into the formula).
 *   EXACT    bit-exact in every build (always-double / always-float API).
 *   LIBM_EQ  bit-exact on the macOS arm64 parity platform; elsewhere within
 *            16 ULP for values produced from system trigonometric functions.
 */
#include "ivp_test.h"
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_fast_math.h>
#include <math.h>
#include <stdio.h>

static int check_val(double actual, double ref, int mode, double ftol,
                     const char *what, int line) {
    bool ok;
#ifdef IVP_REAL_DOUBLE
    (void)ftol;
    ok = (mode == 2) ? (actual == ref) : (actual == ref);
#else
    if (mode == 0)      ok = ((float)actual == (float)ref);   /* REF_EQ   */
    else if (mode == 1) ok = fabs(actual - ref) <= ftol;      /* REF_NEAR */
    else                ok = (actual == ref);                 /* EXACT    */
#endif
    if (!ok) {
        fprintf(stderr, "  MISMATCH %s: got %a (%.17g) expected %a (%.17g) line %d\n",
                what, actual, actual, ref, ref, line);
        return 1;
    }
    return 0;
}

#define IVP_LIBM_MAX_ULPS 16

static bool libm_value_matches(double actual, double ref) {
#ifdef IVP_REAL_DOUBLE
#if defined(__APPLE__) && (defined(__aarch64__) || defined(__arm64__))
    return actual == ref;
#else
    if (actual == ref) return true;
    if (!isfinite(actual) || !isfinite(ref)) return false;
    double lower = ref;
    double upper = ref;
    for (int i = 0; i < IVP_LIBM_MAX_ULPS; ++i) {
        lower = nextafter(lower, -INFINITY);
        upper = nextafter(upper, INFINITY);
    }
    return actual >= lower && actual <= upper;
#endif
#else
    float actual_f = (float)actual;
    float ref_f = (float)ref;
#if defined(__APPLE__) && (defined(__aarch64__) || defined(__arm64__))
    return actual_f == ref_f;
#else
    if (actual_f == ref_f) return true;
    if (!isfinite(actual_f) || !isfinite(ref_f)) return false;
    float lower = (float)ref;
    float upper = (float)ref;
    for (int i = 0; i < IVP_LIBM_MAX_ULPS; ++i) {
        lower = nextafterf(lower, -INFINITY);
        upper = nextafterf(upper, INFINITY);
    }
    return actual_f >= lower && actual_f <= upper;
#endif
#endif
}

static int check_libm_val(double actual, double ref, double ftol,
                          const char *what, int line) {
#ifndef IVP_REAL_DOUBLE
    if (ftol > 0.0 && fabs(actual - ref) <= ftol) return 0;
#else
    (void)ftol;
#endif
    if (libm_value_matches(actual, ref)) return 0;
    fprintf(stderr,
            "  MISMATCH %s: got %a (%.17g) expected %a (%.17g), "
            "limit %d ULP, line %d\n",
            what, actual, actual, ref, ref, IVP_LIBM_MAX_ULPS, line);
    return 1;
}

#define REF_EQ(a, r)        do { if (check_val((double)(a), (r), 0, 0.0, #a, __LINE__)) { (*_fail_count)++; return; } } while (0)
#define REF_NEAR(a, r, tol) do { if (check_val((double)(a), (r), 1, (tol), #a, __LINE__)) { (*_fail_count)++; return; } } while (0)
#define EXACT(a, r)         do { if (check_val((double)(a), (r), 2, 0.0, #a, __LINE__)) { (*_fail_count)++; return; } } while (0)
#define LIBM_EQ(a, r)       do { if (check_libm_val((double)(a), (r), 0.0, #a, __LINE__)) { (*_fail_count)++; return; } } while (0)
#define LIBM_NEAR(a, r, t)  do { if (check_libm_val((double)(a), (r), (t), #a, __LINE__)) { (*_fail_count)++; return; } } while (0)

#define Q_EQ(q, r)  do { REF_EQ((q).x, (r)[0]); REF_EQ((q).y, (r)[1]); REF_EQ((q).z, (r)[2]); REF_EQ((q).w, (r)[3]); } while (0)
#define Q_NEAR(q, r, t) do { REF_NEAR((q).x, (r)[0], t); REF_NEAR((q).y, (r)[1], t); REF_NEAR((q).z, (r)[2], t); REF_NEAR((q).w, (r)[3], t); } while (0)
#define Q_LIBM_EQ(q, r) do { LIBM_EQ((q).x, (r)[0]); LIBM_EQ((q).y, (r)[1]); LIBM_EQ((q).z, (r)[2]); LIBM_EQ((q).w, (r)[3]); } while (0)
#define Q_LIBM_NEAR(q, r, t) do { LIBM_NEAR((q).x, (r)[0], t); LIBM_NEAR((q).y, (r)[1], t); LIBM_NEAR((q).z, (r)[2], t); LIBM_NEAR((q).w, (r)[3], t); } while (0)
#define V_EQ(v, r)  do { REF_EQ((v).k[0], (r)[0]); REF_EQ((v).k[1], (r)[1]); REF_EQ((v).k[2], (r)[2]); } while (0)
#define V_NEAR(v, r, t) do { REF_NEAR((v).k[0], (r)[0], t); REF_NEAR((v).k[1], (r)[1], t); REF_NEAR((v).k[2], (r)[2], t); } while (0)
#define V_EXACT(v, r) do { EXACT((v).k[0], (r)[0]); EXACT((v).k[1], (r)[1]); EXACT((v).k[2], (r)[2]); } while (0)
#define V_LIBM_EQ(v, r) do { LIBM_EQ((v).k[0], (r)[0]); LIBM_EQ((v).k[1], (r)[1]); LIBM_EQ((v).k[2], (r)[2]); } while (0)
#define V_LIBM_NEAR(v, r, t) do { LIBM_NEAR((v).k[0], (r)[0], t); LIBM_NEAR((v).k[1], (r)[1], t); LIBM_NEAR((v).k[2], (r)[2], t); } while (0)
#define M_EQ(m, r)  do { for (int i_ = 0; i_ < 9; i_++) REF_EQ((m).rows[i_ / 3].k[i_ % 3], (r)[i_]); } while (0)
#define M_NEAR(m, r, t) do { for (int i_ = 0; i_ < 9; i_++) REF_NEAR((m).rows[i_ / 3].k[i_ % 3], (r)[i_], t); } while (0)
#define M_LIBM_EQ(m, r) do { for (int i_ = 0; i_ < 9; i_++) LIBM_EQ((m).rows[i_ / 3].k[i_ % 3], (r)[i_]); } while (0)
#define M_LIBM_NEAR(m, r, t) do { for (int i_ = 0; i_ < 9; i_++) LIBM_NEAR((m).rows[i_ / 3].k[i_ % 3], (r)[i_], t); } while (0)

/* Vector semantics of ivp_vec3r_*: IVP_U_Point with IVP_REAL_DOUBLE,
 * IVP_U_Float_Point in the float build. */
#ifdef IVP_REAL_DOUBLE
#define PT_OR_FP(pt, fp) (pt)
#else
#define PT_OR_FP(pt, fp) (fp)
#endif

static ivp_quatr_t mkq(double x, double y, double z, double w) {
    ivp_quatr_t q;
    q.x = (ivp_real_t)x; q.y = (ivp_real_t)y; q.z = (ivp_real_t)z; q.w = (ivp_real_t)w;
    return q;
}

static void set_m(ivp_mat3r_t *m, const float v[9]) {
    for (int i = 0; i < 9; i++) m->rows[i / 3].k[i % 3] = (ivp_real_t)v[i];
}

/* ═══ IVP_Fast_Math / IVP_Inline_Math ════════════════════════════════════ */

IVP_TEST(ref_isqrt_double) {
    static const double xs[6] = { 0.01, 0.5, 1.0, 2.0, 3.7, 1e6 };
    static const double ref[6][4] = {
        { 0x1.3fee80bc9ba5bp+3, 0x1.3ffffe90a5a3p+3, 0x1.3ffffffffffd9p+3, 0x1.4p+3 },
        { 0x1.6a058p+0, 0x1.6a09e6536af66p+0, 0x1.6a09e667f3bcdp+0, 0x1.6a09e667f3bccp+0 },
        { 0x1p+0, 0x1p+0, 0x1p+0, 0x1p+0 },
        { 0x1.6a058p-1, 0x1.6a09e6536af66p-1, 0x1.6a09e667f3bcdp-1, 0x1.6a09e667f3bccp-1 },
        { 0x1.0a2cc7101d3d4p-1, 0x1.0a2d168da35a5p-1, 0x1.0a2d168dc6f62p-1, 0x1.0a2d168dc6f62p-1 },
        { 0x1.0624cfe512482p-10, 0x1.0624dd2f199d1p-10, 0x1.0624dd2f1a9fcp-10, 0x1.0624dd2f1a9fcp-10 },
    };
    for (int i = 0; i < 6; i++)
        for (int s = 0; s <= 3; s++)
            EXACT(ivp_isqrt_double(xs[i], s), ref[i][s]);
}

IVP_TEST(ref_isqrt_float_and_inline_double) {
    /* IVP_Inline_Math::isqrt_float == IVP_Fast_Math::isqrt(x, 2) (returned
     * unrounded), IVP_Inline_Math::isqrt_double == IVP_Fast_Math::isqrt(x, 3) */
    static const float fx[4] = { 2.0f, 0.37f, 123.5f, 1e-4f };
    static const double rf[4] = { 0x1.6a09e667f3bcdp-1, 0x1.a4dc850638c51p+0,
                                  0x1.7093544a82ba3p-4, 0x1.90000054c3ffep+6 };
    static const double rd[4] = { 0x1.6a09e667f3bccp-1, 0x1.a4dc850638c51p+0,
                                  0x1.7093544a82ba3p-4, 0x1.90000054c4001p+6 };
    for (int i = 0; i < 4; i++) {
        EXACT(ivp_isqrt_float(fx[i]), rf[i]);
        EXACT(ivp_isqrt_double_inline((double)fx[i]), rd[i]);
    }
}

IVP_TEST(ref_inline_math_approximations) {
    static const float ax[6] = { -0.9f, -0.3f, 0.1f, 0.5f, 0.99f, 1.0f };
    /* approx5_sin, approx5_cos, fast_asin, upper_limit_asin,
     * fast_approx_asin, fast_anywhere_asin, save_acosf */
    static const double ref[6][7] = {
        { -0x1.911c84p-1, 0x1.3ea306p-1, -0x1.28a202p+0, -0x1.429892p+0, -0x1.10d766p+0, -0x1.222158p+0, 0x1.586476p+1 },
        { -0x1.2e9cdcp-2, 0x1.e921fep-1, -0x1.373d48p-2, -0x1.38d04ap-2, -0x1.37feap-2, -0x1.483a04p-2, 0x1.e0200cp+0 },
        { 0x1.98eaecp-4, 0x1.fd713p-1, 0x1.9a1a78p-4, 0x1.9a4c9ap-4, 0x1.9a4928p-4, 0x1.c0dffcp-4, 0x1.787b22p+0 },
        { 0x1.eaeeeep-2, 0x1.c15556p-1, 0x1.0c51ecp-1, 0x1.112206p-1, 0x1.0bdddep-1, 0x1.0af1aap-1, 0x1.0c1524p+0 },
        { 0x1.ac2382p-1, 0x1.19964cp-1, 0x1.61d95ep+0, 0x1.893a8cp+0, 0x1.391932p+0, 0x1.67bbp+0, 0x1.21df6ap-3 },
        { 0x1.aeeeeep-1, 0x1.155556p-1, 0x1.68f5c2p+0, 0x1.922062p+0, 0x1.3ddddep+0, 0x1.70e56p+0, 0x0p+0 },
    };
    for (int i = 0; i < 6; i++) {
        float a = ax[i];
        EXACT(ivp_approx5_sinf(a), ref[i][0]);
        EXACT(ivp_approx5_cosf(a), ref[i][1]);
        EXACT(ivp_fast_asinf(a), ref[i][2]);
        EXACT(ivp_upper_limit_asinf(a), ref[i][3]);
        EXACT(ivp_fast_approx_asinf(a), ref[i][4]);
        EXACT(ivp_fast_anywhere_asinf(a), ref[i][5]);
        EXACT(ivp_save_acosf(a), ref[i][6]);
    }
}

IVP_TEST(ref_invert_2x2) {
    double a = 0, b = 0, c = 0, d = 0;
    IVP_ASSERT_TRUE(ivp_invert_2x2_matrix(1.5, -0.25, 0.75, 2.0, &a, &b, &c, &d));
    EXACT(a, 0x1.4141414141414p-1);
    EXACT(b, -0x1.e1e1e1e1e1e1ep-3);
    EXACT(c, 0x1.4141414141414p-4);
    EXACT(d, 0x1.e1e1e1e1e1e1ep-2);
    double s = 7.0;
    IVP_ASSERT_FALSE(ivp_invert_2x2_matrix(1.0, 2.0, 2.0, 4.0, &s, &s, &s, &s));
    EXACT(s, 7.0);
}

IVP_TEST(ref_res_eps_constant) {
    /* P_RES_EPS = P_MAX_WORLD_DOUBLE * P_DOUBLE_RES = 10000 * 1e-12 (double build) */
    EXACT(IVP_REF_RES_EPS, 10000 * 1e-12);
    /* 1 - P_RES_EPS as baked into the legacy set_fast_multiple_with_clip
     * (immediate 0x3FEFFFFFFAA19C47 in libivp_utility.a) */
    EXACT(1.0 - IVP_REF_RES_EPS, 0x1.ffffffaa19c47p-1);
}

/* ═══ IVP_U_Quat ════════════════════════════════════════════════════════ */

IVP_TEST(ref_quat_fast_multiple_with_clip) {
    ivp_vec3f_t w = {{ 1.3f, -0.7f, 2.1f }};
    ivp_quatr_t q;
    ivp_quat_set_fast_multiple_with_clip(&q, &w, (ivp_real_t)0.016666668f);
    static const double r1[4] = { 0x1.62facf7e43d54p-7, -0x1.7e4a8aaf5ebdep-8,
                                  0x1.1eb492f6bcd16p-6, 0x1.ffe201d2a58dbp-1 };
    Q_LIBM_EQ(q, r1);
    /* |sin(angle/2)|^2 > 1: clipped by (1 - P_RES_EPS) */
    ivp_vec3f_t big = {{ 200.0f, -150.0f, 90.0f }};
    ivp_quat_set_fast_multiple_with_clip(&q, &big, (ivp_real_t)0.02f);
    static const double r2[4] = { 0x1.2a53193a6dd58p-1, -0x1.4742bb9a9d603p-1,
                                  0x1.00fef1329d06ap-1, 0x1.289503240a61cp-13 };
    Q_LIBM_EQ(q, r2);
}

IVP_TEST(ref_quat_very_fast_multiple) {
    ivp_vec3f_t w = {{ 1.3f, -0.7f, 2.1f }};
    ivp_quatr_t q;
    ivp_quat_set_very_fast_multiple(&q, &w, (ivp_real_t)0.0055555557f);
    static const double r1[4] = { 0x1.d95084724e2ap-9, -0x1.fdb9606186842p-10, 0x1.7e4a8946119ddp-8,
                                  0x1.fffcaae55e3c4p-1 };
    Q_EQ(q, r1);
    /* reference result is a unit quaternion (not w = 1) */
    double n = (double)q.x * (double)q.x + (double)q.y * (double)q.y +
               (double)q.z * (double)q.z + (double)q.w * (double)q.w;
    IVP_ASSERT_NEAR(n, 1.0, 1e-6);
    ivp_vec3f_t big = {{ 30.0f, -20.0f, 25.0f }};
    ivp_quat_set_very_fast_multiple(&q, &big, (ivp_real_t)0.1f);
    /* the retail w has no clamp: sqrt of a negative argument */
    static const double r2[3] = { 0x1.dfffff57ffff6p-1, -0x1.aaaaaabfffffdp-1, 0x1.d9555524ffffap-1 };
    REF_EQ(q.x, r2[0]);
    REF_EQ(q.y, r2[1]);
    REF_EQ(q.z, r2[2]);
    IVP_ASSERT_TRUE(isnan((double)q.w));
}

IVP_TEST(ref_quat_set_variants_and_angles) {
    ivp_vec3r_t wd = {{ (ivp_real_t)1.3f, (ivp_real_t)-0.7f, (ivp_real_t)2.1f }};
    ivp_quatr_t q;
    ivp_quat_set_fast_multiple(&q, &wd, 0.25);
    static const double r1[4] = { 0x1.4b565645599abp-3, -0x1.65f15dacf9a55p-4,
                                  0x1.09b93e45d9c32p-2, 0x1.e56a641643c7ep-1 };
    Q_LIBM_EQ(q, r1);
    ivp_quat_set_rotation(&q, 0.3f, -0.45f, 1.1f);
    static const double r2[4] = { 0x1.320c9f687ee6ep-3, -0x1.c8ebfc3f71f8dp-3,
                                  0x1.0b9da96be2f87p-1, 0x1.9e462a3fdd71bp-1 };
    Q_LIBM_EQ(q, r2);
    ivp_vec3f_t ang;
    ivp_quat_get_angles(&q, &ang);
    static const double r3[3] = { 0x1.333334p-2, -0x1.ccccccp-2, 0x1.19999ap+0 };
    V_LIBM_NEAR(ang, r3, 1e-6);
}

static const double QA[4] = { 0x1.042e219937314p-3, -0x1.042e219937314p-2,
                              0x1.042e219937314p-1, 0x1.a049cfc39df5dp-1 };
static const double QB[4] = { -0x1.3b2e391e8a202p-2, 0x1.3b2e391e8a202p-1,
                              0x1.a43da0cef3d2bp-4, 0x1.6fb5ebf018753p-1 };

IVP_TEST(ref_quat_normize_and_products) {
    ivp_quatr_t a = mkq(0.125f, -0.25f, 0.5f, 0.8f);
    IVP_ASSERT_EQ(ivp_quat_normalize(&a), IVP_OK);
    Q_EQ(a, QA);
    ivp_quatr_t b = mkq(-0.3f, 0.6f, 0.1f, 0.7f);
    ivp_quat_normalize(&b);
    Q_EQ(b, QB);

    ivp_quatr_t r;
    ivp_quat_mult(&r, &a, &b);
    static const double mult_ab[4] = { -0x1.fdda9f8887c0ap-2, 0x1.304f903bd5557p-3,
                                       0x1.cb22b09107b64p-2, 0x1.746158879f029p-1 };
    Q_NEAR(r, mult_ab, 1e-6);
    ivp_quat_set_div_unit_quat(&r, &a, &b);
    static const double div_ab[4] = { 0x1.5c5b1168f4f9ep-1, -0x1.06ef675857888p-1,
                                      0x1.204b5c9481163p-2, 0x1.c3209857c8e09p-2 };
    Q_NEAR(r, div_ab, 1e-6);
    ivp_quat_set_invert_mult(&r, &a, &b);
    static const double invmult_ab[4] = { -0x1.55aea14ca41p-9, 0x1.b47218bc2bb36p-1,
                                          -0x1.204b5c9481163p-2, 0x1.c3209857c8e09p-2 };
    Q_NEAR(r, invmult_ab, 1e-6);
    /* in-place multiply (res aliases q1) */
    ivp_quatr_t ip = a;
    ivp_quat_mult(&ip, &ip, &b);
    Q_NEAR(ip, mult_ab, 1e-6);

    ivp_quatf_t fb, fa;
    ivp_quatf_set(&fb, &b);
    ivp_quatf_set(&fa, &a);
    REF_NEAR(ivp_quat_estimate_qdiff(&a, &fb), 0x1.9ca084a1d0786p+0, 1e-6);
    REF_NEAR(ivp_quat_estimate_qdiff(&a, &fa), -0x1.d6986d4p-25, 1e-6);
}

IVP_TEST(ref_quat_interpolate_smoothly) {
    ivp_quatr_t a = mkq(QA[0], QA[1], QA[2], QA[3]);
    ivp_quatr_t b = mkq(QB[0], QB[1], QB[2], QB[3]);
    ivp_quatr_t r;
    ivp_quat_set_interpolate_smoothly(&r, &a, &b, (ivp_real_t)0.3f);
    static const double s03[4] = { -0x1.a92151485df78p-7, 0x1.a92151485df78p-6,
                                   0x1.be246bc4fb237p-2, 0x1.cc9c753c7cf28p-1 };
    Q_LIBM_NEAR(r, s03, 2e-6);
    /* t is NOT clamped */
    ivp_quat_set_interpolate_smoothly(&r, &a, &b, (ivp_real_t)1.4f);
    static const double s14[4] = { -0x1.9d8e360e59072p-2, 0x1.9d8e360e59072p-1,
                                   -0x1.09d92f5076773p-3, 0x1.a33eb6d66f0dbp-2 };
    Q_LIBM_NEAR(r, s14, 2e-6);
    /* opposite hemisphere: shortest path through sign flip */
    ivp_quatr_t nb = mkq(-QB[0], -QB[1], -QB[2], -QB[3]);
    ivp_quat_set_interpolate_smoothly(&r, &a, &nb, (ivp_real_t)0.6f);
    static const double s06[4] = { -0x1.3660d7534adb1p-3, 0x1.3660d7534adb1p-2,
                                   0x1.428177ebe75a7p-2, 0x1.c5e9b41e2bc1p-1 };
    Q_LIBM_NEAR(r, s06, 2e-6);
    ivp_quat_set_interpolate_linear(&r, &a, &nb, (ivp_real_t)0.6f);
    static const double l06[4] = { -0x1.1225054b93e7ap-3, 0x1.1225054b93e7ap-2,
                                   0x1.0f2e258d1862ap-2, 0x1.832446fe12b3ep-1 };
    Q_NEAR(r, l06, 2e-6);
    /* cos(omega) >= 1 - 0.001f: linear + normize_correct_step(3) */
    static const double QC[4] = { 0x1.17eeb13f31cdfp-3, -0x1.089cbab00ca49p-2,
                                  0x1.038168805596dp-1, 0x1.9f357468564e4p-1 };
    ivp_quatr_t c = mkq(QC[0], QC[1], QC[2], QC[3]);
    ivp_quat_set_interpolate_smoothly(&r, &a, &c, (ivp_real_t)0.7f);
    static const double c07[4] = { 0x1.120297b4fee39p-3, -0x1.07492f39734b2p-2,
                                   0x1.03b60c659a186p-1, 0x1.9f89add73f5f2p-1 };
    Q_NEAR(r, c07, 2e-6);
    ivp_quat_set_interpolate_smoothly(&r, &a, &c, (ivp_real_t)-0.5f);
    static const double cm05[4] = { 0x1.f49608b68a5fcp-4, -0x1.01f3e96fa1deep-2,
                                    0x1.04818b2116193p-1, 0x1.a0cf4569f0c65p-1 };
    Q_NEAR(r, cm05, 2e-6);
    /* ivp_quat_slerp uses the same reference algorithm */
    ivp_quat_slerp(&r, &a, &b, (ivp_real_t)0.3f);
    Q_LIBM_NEAR(r, s03, 2e-6);
}

IVP_TEST(ref_quat_fast_normize_and_correct_steps) {
    ivp_quatr_t q = mkq(0.5f, 0.5f, 0.5f, 0.52f);
    ivp_quat_fast_normalize(&q);
    static const double f1[4] = { 0x1.fadb2e29328dep-2, 0x1.fadb2e29328dep-2,
                                  0x1.fadb2e29328dep-2, 0x1.0790b0f859454p-1 };
    Q_EQ(q, f1);

    const ivp_quatr_t q0 = mkq(0.1f, -0.2f, 0.3f, 0.927f);
    q = q0;
    ivp_quat_fast_normalize(&q);
    static const double f2[4] = { 0x1.99bcccc5d7d91p-4, -0x1.99bcccc5d7d91p-3,
                                  0x1.334d9a146ce2bp-2, 0x1.dac8873b9230ap-1 };
    Q_EQ(q, f2);
    q = q0;
    ivp_quat_normalize(&q);
    static const double n2[4] = { 0x1.99bcccc5d7e19p-4, -0x1.99bcccc5d7e19p-3,
                                  0x1.334d9a146ce91p-2, 0x1.dac8873b923a7p-1 };
    Q_EQ(q, n2);

    static const double cs[6][4] = {
        { 0x1.99bcc83cdf80fp-4, -0x1.99bcc83cdf80fp-3, 0x1.334d96adb29f3p-2, 0x1.dac881fa4c08p-1 },
        { 0x1.99bcccc5741fbp-4, -0x1.99bcccc5741fbp-3, 0x1.334d9a142217ap-2, 0x1.dac8873b1ea27p-1 },
        { 0x1.99bcccc5d7d91p-4, -0x1.99bcccc5d7d91p-3, 0x1.334d9a146ce2bp-2, 0x1.dac8873b9230ap-1 },
        { 0x1.99bcccc5d7e1bp-4, -0x1.99bcccc5d7e1bp-3, 0x1.334d9a146ce92p-2, 0x1.dac8873b923a9p-1 },
        { 0x1.99bcccc5d7e1bp-4, -0x1.99bcccc5d7e1bp-3, 0x1.334d9a146ce92p-2, 0x1.dac8873b923a9p-1 },
        { 0x1.99bcccc5d7e1bp-4, -0x1.99bcccc5d7e1bp-3, 0x1.334d9a146ce92p-2, 0x1.dac8873b923a9p-1 },
    };
    for (int s = 1; s <= 6; s++) {
        q = q0;
        ivp_quat_normize_correct_step(&q, s);
        Q_EQ(q, cs[s - 1]);
    }
    q = q0;
    ivp_quat_invert(&q);
    static const double inv[4] = { -0x1.99e00292035d5p-4, 0x1.99e00292035d5p-3,
                                   -0x1.3368026d9886bp-2, 0x1.daf153f864af1p-1 };
    Q_EQ(q, inv);

    /* exactly unit (|1 - |q|^2| <= P_DOUBLE_RES): untouched */
    q = mkq(0.0, 0.0, 0.0, 1.0);
    ivp_quat_fast_normalize(&q);
    EXACT(q.w, 1.0);
    /* normize_quat refuses |q|^2 <= P_DOUBLE_EPS and leaves q unchanged */
    q = mkq(1e-10, 0.0, 0.0, 0.0);
    IVP_ASSERT_TRUE(ivp_quat_normalize(&q) != IVP_OK);
    REF_EQ(q.x, (double)(ivp_real_t)1e-10);
}

IVP_TEST(ref_quat_from_rotation_vectors) {
    ivp_quatr_t q;
    ivp_quat_set_from_rotation_vectors(&q, 0.6f, 0.0f, 0.8f, 0.0f, 1.0f, 0.0f);
    static const double g[4] = { -0x1.21a184f4845cbp-1, 0x0p+0, 0x1.b2724823cb7dfp-2, 0x1.6a09e667f3bcdp-1 };
    Q_EQ(q, g);
    ivp_quat_set_from_rotation_vectors(&q, 0.6f, 0.0f, 0.8f, 0.6f, 0.0f, 0.8f);
    static const double p[4] = { 0.0, 0.0, 0.0, 1.0 };
    Q_EQ(q, p);
    ivp_quat_set_from_rotation_vectors(&q, 0.6f, 0.0f, 0.8f, -0.6f, 0.0f, -0.8f);
    static const double o[4] = { 0.0, 1.0, -0.0, 0.0 };
    Q_EQ(q, o);
    ivp_quat_set_from_rotation_vectors(&q, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -1.0f);
    static const double oz[4] = { -1.0, 0.0, 0.0, 0.0 };
    Q_EQ(q, oz);
}

IVP_TEST(ref_quat_matrix_conversions) {
    ivp_quatr_t a = mkq(QA[0], QA[1], QA[2], QA[3]);
    ivp_mat3r_t m;
    ivp_quat_to_mat3(&a, &m);
    static const double qa_m[9] = {
        0x1.6aeda899b90c2p-2, -0x1.c823b27a952cp-1, -0x1.22df1ca1036fp-2,
        0x1.86084356c146cp-1, 0x1.ce16cf4f76e3ep-2, -0x1.dbf8ba03a5317p-2,
        0x1.15a66c982981ep-1, -0x1.a715f8d7cfc08p-5, 0x1.ad5db51337218p-1 };
    M_NEAR(m, qa_m, 1e-6);
    ivp_quatr_t r;
    ivp_quat_from_mat3(&r, &m);
    static const double qa_mq[4] = { 0x1.042e219937314p-3, -0x1.042e219937314p-2,
                                     0x1.042e219937314p-1, 0x1.a049cfc39df5cp-1 };
    Q_NEAR(r, qa_mq, 1e-6);

    /* trace < 0 branch (from a normalized (0.7, 0.6, -0.3, 0.1)) */
    ivp_quatr_t b = mkq(0.7f, 0.6f, -0.3f, 0.1f);
    ivp_quat_normalize(&b);
    ivp_quat_to_mat3(&b, &m);
    static const double qb2_m[9] = {
        0x1.af284dbd573ep-5, 0x1.e50d796788dc8p-1, -0x1.435e509ed8fecp-2,
        0x1.a46102b0dc1d6p-1, -0x1.c4b737f13ff88p-3, -0x1.0d794402209ebp-1,
        -0x1.230815bcc5fd9p-1, -0x1.da46139e6c127p-3, -0x1.9435e4ab9cb88p-1 };
    M_NEAR(m, qb2_m, 1e-6);
    ivp_quat_from_mat3(&r, &m);
    static const double qb2_mq[4] = { 0x1.6fb5ebf018753p-1, 0x1.3b2e391e8a202p-1,
                                      -0x1.3b2e391e8a202p-2, 0x1.a43da0cef3d2ap-4 };
    Q_NEAR(r, qb2_mq, 1e-6);

    /* m00 == m11 > m22 tie: the reference picks i = 0 (x branch) */
    static const float tie[9] = { 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f };
    set_m(&m, tie);
    ivp_quat_from_mat3(&r, &m);
    static const double tie_q[4] = { 0x1.6a09e667f3bcdp-1, 0x1.6a09e667f3bccp-1, 0.0, 0.0 };
    Q_EQ(r, tie_q);
    /* arbitrary (non-orthonormal) matrix: result is normized */
    static const float gen[9] = { -0.25f, 0.5f, 0.125f, 0.375f, -0.25f, 0.75f,
                                  -0.625f, 0.0625f, -0.5f };
    set_m(&m, gen);
    ivp_quat_from_mat3(&r, &m);
    static const double gen_q[4] = { 0x1.8d36f1dece0bap-1, 0x1.cf6ac4d945b86p-2,
                                     -0x1.08cf4be9deb28p-2, -0x1.6c1d086192357p-2 };
    Q_EQ(r, gen_q);

    /* 4x4 variants (no normize, column-major convention) */
    double m4[4][4];
    ivp_quat_to_mat4x4(&a, m4);
    static const double qa_m4[16] = {
        0x1.6aeda899b90c2p-2, -0x1.c823b27a952cp-1, -0x1.22df1ca1036fp-2, 0.0,
        0x1.86084356c146cp-1, 0x1.ce16cf4f76e3ep-2, -0x1.dbf8ba03a5317p-2, 0.0,
        0x1.15a66c982981ep-1, -0x1.a715f8d7cfc08p-5, 0x1.ad5db51337218p-1, 0.0,
        0.0, 0.0, 0.0, 1.0 };
    for (int i = 0; i < 16; i++) REF_NEAR(m4[i / 4][i % 4], qa_m4[i], 1e-6);
    ivp_quat_from_mat4x4(&r, (const double (*)[4])m4);
    static const double qa_m4q[4] = { -0x1.042e219937314p-3, 0x1.042e219937314p-2,
                                      -0x1.042e219937314p-1, 0x1.a049cfc39df5cp-1 };
    Q_NEAR(r, qa_m4q, 1e-6);
}

/* ═══ IVP_U_Point / IVP_U_Float_Point ═══════════════════════════════════ */

static const double PT_NORM[3] = { 0x1.b14c08172f744p-4, -0x1.32eb3029937f9p-1, 0x1.96374686ecf8ap-1 };
static const double FP_NORM[3] = { 0x1.b14c0ap-4, -0x1.32eb3p-1, 0x1.963748p-1 };

IVP_TEST(ref_point_normize_lengths) {
    const ivp_vec3r_t p = {{ (ivp_real_t)0.3f, (ivp_real_t)-1.7f, (ivp_real_t)2.25f }};
    ivp_vec3r_t q = p;
    IVP_ASSERT_EQ(ivp_vec3r_normalize(&q), IVP_OK);
    V_EXACT(q, PT_OR_FP(PT_NORM, FP_NORM));
    q = p;
    IVP_ASSERT_EQ(ivp_vec3r_fast_normalize(&q), IVP_OK);
    V_EXACT(q, PT_OR_FP(PT_NORM, FP_NORM));
    q = p;
    ivp_real_t len = ivp_vec3r_normalize_and_length(&q);
    V_EXACT(q, PT_OR_FP(PT_NORM, FP_NORM));
    REF_EQ(len, PT_OR_FP(0x1.6affc2dddff0dp+1, 0x1.6affc1f076d0ep+1));
    /* fast_real_length is an exact sqrt (IVP_Fast_Math::sqrt == ::sqrt) */
    REF_EQ(ivp_vec3r_fast_length(&p), PT_OR_FP(0x1.6affc2dddff0dp+1, 0x1.6affc1f076d0fp+1));
    REF_EQ(ivp_vec3r_length(&p), PT_OR_FP(0x1.6affc2dddff0dp+1, 0x1.6affc1f076d0fp+1));

    /* explicit class variants */
    ivp_vec3f_t f = {{ 0.3f, -1.7f, 2.25f }};
    EXACT(ivp_vec3f_quad_length(&f), 0x1.015c28p+3);
    EXACT(ivp_vec3f_fast_real_length(&f), 0x1.6affc1f076d0fp+1);
    ivp_vec3f_t g = f;
    ivp_vec3f_normize(&g);
    V_EXACT(g, FP_NORM);
    g = f;
    ivp_vec3f_fast_normize(&g);
    V_EXACT(g, FP_NORM);
    g = f;
    EXACT(ivp_vec3f_real_length_plus_normize(&g), 0x1.6affc1f076d0ep+1);
    V_EXACT(g, FP_NORM);

    ivp_vec3d_t d = {{ 0.3f, -1.7f, 2.25f }};
    EXACT(ivp_vec3d_fast_real_length(&d), 0x1.6affc2dddff0dp+1);
    ivp_vec3d_t e = d;
    ivp_vec3d_normize(&e);
    V_EXACT(e, PT_NORM);
    e = d;
    ivp_vec3d_fast_normize(&e);
    V_EXACT(e, PT_NORM);
    e = d;
    EXACT(ivp_vec3d_real_length_plus_normize(&e), 0x1.6affc2dddff0dp+1);
    V_EXACT(e, PT_NORM);

    /* quad_length < P_DOUBLE_EPS: fault, vector unchanged */
    ivp_vec3r_t tiny = {{ (ivp_real_t)1e-10f, 0, 0 }};
    IVP_ASSERT_TRUE(ivp_vec3r_normalize(&tiny) != IVP_OK);
    IVP_ASSERT_TRUE(ivp_vec3r_fast_normalize(&tiny) != IVP_OK);
    EXACT(ivp_vec3r_normalize_and_length(&tiny), 0.0);
    EXACT(tiny.k[0], (double)(ivp_real_t)1e-10f);
}

IVP_TEST(ref_point_helpers) {
    const ivp_vec3r_t p = {{ (ivp_real_t)0.3f, (ivp_real_t)-1.7f, (ivp_real_t)2.25f }};
    ivp_vec3r_t o;
    ivp_vec3r_calc_an_orthogonal(&o, &p);
    static const double o1[3] = { -0x1.fcf5c33d70a4p+2, -0x1.2f5c29fd70a4p+0, 0x1.51eb84147aep-3 };
    V_EQ(o, o1);
    const ivp_vec3r_t p2 = {{ (ivp_real_t)-4.0f, (ivp_real_t)0.5f, (ivp_real_t)0.25f }};
    ivp_vec3r_calc_an_orthogonal(&o, &p2);
    static const double o2[3] = { -0x1.ep+0, -0x1.01p+4, 0x1.1p+1 };
    V_EQ(o, o2);
    const ivp_vec3r_t p3 = {{ (ivp_real_t)0.5f, (ivp_real_t)-0.5f, (ivp_real_t)0.25f }};
    ivp_vec3r_calc_an_orthogonal(&o, &p3);
    static const double o3[3] = { 0x1p-2, 0.0, -0x1p-1 };
    V_EQ(o, o3);

    ivp_vec3r_t sol;
    const ivp_vec3r_t eq = {{ (ivp_real_t)1.5f, (ivp_real_t)-2.0f, (ivp_real_t)-3.25f }};
    static const double qs[3] = { 0x1.78p+4, -0x1.e601080b632bap-1, 0x1.242aecad83759p+1 };
    ivp_vec3r_solve_quadratic_fast(&sol, &eq);
    V_EQ(sol, qs);
    ivp_vec3r_solve_quadratic_accurate(&sol, &eq);
    V_EQ(sol, qs);
    const ivp_vec3r_t eq2 = {{ 0, 2, -3 }};
    ivp_vec3r_set(&sol, 9, 9, 9);
    ivp_vec3r_solve_quadratic_accurate(&sol, &eq2);
    static const double ql[3] = { 0.0, 1.5, 1.5 };
    V_EQ(sol, ql);
    const ivp_vec3r_t eq3 = {{ 1, 2, 1 }};
    ivp_vec3r_set(&sol, 9, 9, 9);
    ivp_vec3r_solve_quadratic_fast(&sol, &eq3);
    static const double qfz[3] = { 0.0, 9.0, 9.0 }; /* fast: roots only if disc > 0 */
    V_EQ(sol, qfz);
    ivp_vec3r_set(&sol, 9, 9, 9);
    ivp_vec3r_solve_quadratic_accurate(&sol, &eq3);
    static const double qaz[3] = { 0.0, -1.0, -1.0 };
    V_EQ(sol, qaz);

    const ivp_vec3r_t n = {{ 0.0f, (ivp_real_t)0.6f, (ivp_real_t)0.8f }};
    ivp_vec3r_t op;
    ivp_vec3r_set_orthogonal_part(&op, &p, &n);
    static const double pt_op[3] = { 0x1.333334p-2, -0x1.1581067cac07fp+1, 0x1.a04189a0c49cp+0 };
    static const double fp_op[3] = { 0x1.333334p-2, -0x1.158106p+1, 0x1.a0418ap+0 };
    V_EXACT(op, PT_OR_FP(pt_op, fp_op));

    const ivp_vec3r_t par = {{ (ivp_real_t)0.6f, (ivp_real_t)-3.4f, (ivp_real_t)4.5f }};
    IVP_ASSERT_TRUE(ivp_vec3r_is_parallel(&p, &par, 1e-6));
    IVP_ASSERT_FALSE(ivp_vec3r_is_parallel(&p, &n, 0.1));

    ivp_vec3r_t r = p;
    ivp_vec3r_rotate(&r, IVP_INDEX_Y, 0.7f);
    static const double pt_ry[3] = { 0x1.adcf2c062badbp+0, -0x1.b33334p+0, 0x1.8712bc3abe61fp+0 };
    static const double fp_ry[3] = { 0x1.adcf2cp+0, -0x1.b33334p+0, 0x1.8712bcp+0 };
    V_LIBM_EQ(r, PT_OR_FP(pt_ry, fp_ry));
    r = p;
    ivp_vec3r_rotate(&r, IVP_INDEX_Z, -1.2f);
    static const double pt_rz[3] = { -0x1.79cb5ae812751p+0, -0x1.ca8eafe7d852fp-1, 0x1.2p+1 };
    static const double fp_rz[3] = { -0x1.79cb5ap+0, -0x1.ca8ebp-1, 0x1.2p+1 };
    V_LIBM_EQ(r, PT_OR_FP(pt_rz, fp_rz));

    const ivp_vec3r_t vb = {{ (ivp_real_t)1.25f, (ivp_real_t)0.5f, (ivp_real_t)-0.75f }};
    ivp_vec3r_t c;
    ivp_vec3r_cross_and_normalize(&c, &p, &vb);
    /* IVP_U_Point variant stores the cross product through IVP_FLOAT */
    static const double pt_cn[3] = { 0x1.438ab62982d46p-5, 0x1.997b874ee9a8dp-1, 0x1.32b0ce0ef29a9p-1 };
    static const double fp_cn[3] = { 0x1.438abep-5, 0x1.997b86p-1, 0x1.32b0cep-1 };
    V_EXACT(c, PT_OR_FP(pt_cn, fp_cn));

    const ivp_vec3r_t t0 = {{ (ivp_real_t)0.1f, (ivp_real_t)0.2f, (ivp_real_t)0.3f }};
    const ivp_vec3r_t t1 = {{ (ivp_real_t)1.5f, (ivp_real_t)-0.5f, (ivp_real_t)0.25f }};
    const ivp_vec3r_t t2 = {{ (ivp_real_t)-0.75f, (ivp_real_t)2.0f, (ivp_real_t)1.0f }};
    ivp_vec3r_t area;
    ivp_vec3r_set_vert_to_area(&area, &t0, &t1, &t2);
    static const double va[3] = { 0x1.999997cp-2, 0x1.dfffff1p-1, -0x1.ecccccap+0 };
    V_EQ(area, va);

    ivp_vec3r_t li;
    ivp_vec3r_lerp(&li, &p, &vb, (ivp_real_t)0.3f);
    static const double pt_li[3] = { 0x1.2b851f6147aep-1, -0x1.0a3d70c28f5cp+0, 0x1.599999p+0 };
    static const double fp_li[3] = { 0x1.2b852p-1, -0x1.0a3d7p+0, 0x1.599998p+0 };
    V_EXACT(li, PT_OR_FP(pt_li, fp_li));
}

IVP_TEST(ref_point_set_crossing) {
    ivp_vec4r_t h0 = {{ 1.0f, (ivp_real_t)0.25f, (ivp_real_t)-0.5f }, (ivp_real_t)-2.0f };
    ivp_vec4r_t h1 = {{ (ivp_real_t)-0.3f, (ivp_real_t)1.5f, (ivp_real_t)0.2f }, (ivp_real_t)0.75f };
    ivp_vec4r_t h2 = {{ (ivp_real_t)0.1f, (ivp_real_t)-0.4f, 2.0f }, (ivp_real_t)1.25f };
    ivp_vec3r_t x;
    IVP_ASSERT_EQ(ivp_vec3r_set_crossing(&x, &h0, &h1, &h2), IVP_OK);
    static const double cp[3] = { 0x1.a82c1c4f9f3f4p+0, -0x1.282c18a2d2726p-4, -0x1.71d2504bb02ap-1 };
    V_EQ(x, cp);
    /* h2 parallel to h0: singular, out computed with the non-inverted matrix */
    h2.k[0] = 2.0f;
    h2.k[1] = 0.5f;
    h2.k[2] = -1.0f;
    IVP_ASSERT_TRUE(ivp_vec3r_set_crossing(&x, &h0, &h1, &h2) != IVP_OK);
    static const double sp[3] = { 0x1.38p+1, -0x1.f9999a1p+0, 0x1.38p+2 };
    V_EQ(x, sp);
}

/* ═══ IVP_U_Matrix3 ═════════════════════════════════════════════════════ */

static const float M_IN[9] = { 0.9f, -0.3f, 0.2f, 0.35f, 0.95f, -0.1f, -0.15f, 0.12f, 0.97f };

IVP_TEST(ref_matrix3_orthogonize_handedness) {
    ivp_mat3r_t m;
    set_m(&m, M_IN);
    ivp_mat3r_orthogonize(&m);
    static const double og[9] = {
        0x1.ddf3b6cb020c2p-1, -0x1.287fcd77c1bebp-2, 0x1.99999ap-3,
        0x1.428f5d8000006p-2, 0x1.dfff59a32aae4p-1, -0x1.99999ap-4,
        -0x1.47ae1451eb85p-3, 0x1.40346ed47453ep-3, 0x1.f0a3d8p-1 };
    M_EQ(m, og);
    set_m(&m, M_IN);
    IVP_ASSERT_EQ(ivp_mat3r_orthonormize(&m), IVP_OK);
    static const double on[9] = {
        0x1.df16a3c2d7334p-1, -0x1.293447851de38p-2, 0x1.9a92ec00ecdabp-3,
        0x1.447e31bd1370ep-2, 0x1.e2dfb2ef97a9ep-1, -0x1.9c0df4b535f7ep-4,
        -0x1.492c74d2520bp-3, 0x1.41aa16416bf8p-3, 0x1.f2e76244029ccp-1 };
    M_EQ(m, on);
    /* handedness: right-handed result (reference det = 0x1.ffff7a4775526p-1) */
    REF_NEAR(ivp_mat3r_determinant(&m), 0x1.ffff7a4775526p-1, 1e-6);
    IVP_ASSERT_TRUE(ivp_mat3r_determinant(&m) > 0);

    /* identity stays identity (old libivp version produced det == -1) */
    ivp_mat3r_identity(&m);
    ivp_mat3r_orthogonize(&m);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            EXACT(m.rows[r].k[c], r == c ? 1.0 : 0.0);
    EXACT(ivp_mat3r_determinant(&m), 1.0);

    /* z column is kept */
    set_m(&m, M_IN);
    ivp_mat3r_orthogonize(&m);
    EXACT(m.rows[0].k[2], (double)(ivp_real_t)M_IN[2]);
    EXACT(m.rows[1].k[2], (double)(ivp_real_t)M_IN[5]);
    EXACT(m.rows[2].k[2], (double)(ivp_real_t)M_IN[8]);
}

IVP_TEST(ref_matrix3_get_angles) {
    ivp_mat3r_t m;
    set_m(&m, M_IN);
    float al = 0, be = 0, ga = 0;
    IVP_ASSERT_EQ(ivp_mat3r_get_angles(&m, &al, &be, &ga), IVP_OK);
    EXACT(al, 0x1.4b84a2p-2);
    EXACT(be, 0x1.9ccaep-4);
    EXACT(ga, 0x1.9f8d9p-3);
}

IVP_TEST(ref_matrix3_eigen_vector) {
    static const float sym_v[9] = { 2, 1, 0, 1, 2, 0, 0, 0, 5 };
    ivp_mat3r_t sym;
    set_m(&sym, sym_v);
    ivp_vec3r_t ev;
    ivp_vec3r_set(&ev, 7, 7, 7);
    IVP_ASSERT_EQ(ivp_mat3r_calc_eigen_vector(&sym, 3.0, &ev), 1);
    static const double e3[3] = { 1.0, 1.0, -0.0 };
    V_EXACT(ev, e3);
    ivp_vec3r_set(&ev, 7, 7, 7);
    IVP_ASSERT_EQ(ivp_mat3r_calc_eigen_vector(&sym, 1.0, &ev), 1);
    static const double e1[3] = { 1.0, -1.0, 0.0 };
    V_EXACT(ev, e1);
    ivp_vec3r_set(&ev, 7, 7, 7);
    IVP_ASSERT_EQ(ivp_mat3r_calc_eigen_vector(&sym, 5.0, &ev), 1);
    static const double e5[3] = { 0.0, 0.0, 1.0 };
    V_EXACT(ev, e5);

    ivp_mat3r_t dg;
    ivp_mat3r_identity(&dg);
    dg.rows[2].k[2] = 3;
    ivp_vec3r_set(&ev, 7, 7, 7);
    IVP_ASSERT_EQ(ivp_mat3r_calc_eigen_vector(&dg, 1.0, &ev), 2);
    static const double ed[3] = { 0.0, 1.0, 0.0 };
    V_EXACT(ev, ed);
    ivp_mat3r_identity(&dg);
    ivp_vec3r_set(&ev, 7, 7, 7);
    IVP_ASSERT_EQ(ivp_mat3r_calc_eigen_vector(&dg, 1.0, &ev), 3);
    static const double ei[3] = { 0.0, 0.0, 1.0 };
    V_EXACT(ev, ei);

    ivp_mat3r_t gm;
    set_m(&gm, M_IN);
    ivp_vec3r_set(&ev, 7, 7, 7);
    IVP_ASSERT_EQ(ivp_mat3r_calc_eigen_vector(&gm, 0.97, &ev), 1);
    static const double eg[3] = { 0x1.4737d3a287e3cp-2, 0x1.2f286a4485519p-1, 0x1p+0 };
    V_EQ(ev, eg);
}

IVP_TEST(ref_matrix3_init_normized) {
    const ivp_vec3r_t v = {{ (ivp_real_t)0.3f, (ivp_real_t)-1.7f, (ivp_real_t)2.25f }};
    const ivp_vec3r_t vc = {{ (ivp_real_t)1.25f, (ivp_real_t)0.5f, (ivp_real_t)-0.75f }};
    ivp_mat3r_t nm;
    ivp_mat3r_init_normized_col(&nm, &v, IVP_INDEX_Y);
    static const double cy[9] = {
        -0x1.192d4259dc353p-1, 0x1.b14c08172f744p-4, 0x1.a870de9de2254p-1,
        0x1.4286bc53dd825p-1, -0x1.32eb3029937f9p-1, 0x1.f9a79f5be8d38p-2,
        0x1.192d4259dc353p-1, 0x1.96374686ecf8ap-1, 0x1.0cddb4ea129b9p-2 };
    M_EQ(nm, cy);
    ivp_mat3r_init_normized_row(&nm, &v, IVP_INDEX_Z);
    static const double rz[9] = {
        0x1.a870de9de2254p-1, 0x1.f9a79f5be8d38p-2, 0x1.0cddb4ea129b9p-2,
        -0x1.192d4259dc353p-1, 0x1.4286bc53dd825p-1, 0x1.192d4259dc353p-1,
        0x1.b14c08172f744p-4, -0x1.32eb3029937f9p-1, 0x1.96374686ecf8ap-1 };
    M_EQ(nm, rz);
    ivp_mat3r_init_normized_col2(&nm, &v, IVP_INDEX_X, &vc);
    static const double c2x[9] = {
        0x1.b14c08172f744p-4, 0x1.fcb9b59e5479p-1, 0x1.438ab68ac0f2ep-5,
        -0x1.32eb3029937f9p-1, 0x1.066634f07ce46p-5, 0x1.997b87c5d3a9ap-1,
        0x1.96374686ecf8ap-1, -0x1.bb8309faec75p-4, 0x1.32b0cd6fc6ed9p-1 };
    M_EQ(nm, c2x);
    ivp_mat3r_init_normized_row2(&nm, &v, IVP_INDEX_Z, &vc);
    static const double r2z[9] = {
        0x1.fcb9b59e5479p-1, 0x1.066634f07ce46p-5, -0x1.bb8309faec75p-4,
        0x1.438ab68ac0f2ep-5, 0x1.997b87c5d3a9ap-1, 0x1.32b0cd6fc6ed9p-1,
        0x1.b14c08172f744p-4, -0x1.32eb3029937f9p-1, 0x1.96374686ecf8ap-1 };
    M_EQ(nm, r2z);
    const ivp_vec3r_t vd = {{ 0.5f, 0.0f, 0.5f }}; /* triggers the fallback branch */
    ivp_mat3r_init_normized_col(&nm, &vd, IVP_INDEX_Z);
    static const double fb[9] = {
        -0x1.a20bd700c2c3ep-2, 0x1.279a74590331dp-1, 0x1.6a09e667f3bccp-1,
        -0x1.a20bd700c2c3ep-1, -0x1.279a74590331dp-1, 0.0,
        0x1.a20bd700c2c3ep-2, -0x1.279a74590331dp-1, 0x1.6a09e667f3bccp-1 };
    M_EQ(nm, fb);
}

IVP_TEST(ref_matrix3_rotation_distance_mi2mult) {
    ivp_mat3r_t nm;
    ivp_mat3r_init_rotation(&nm, IVP_INDEX_Y, (ivp_real_t)0.7f);
    static const double ry[9] = {
        0x1.8799656bf1a0dp-1, 0.0, 0x1.49d6e6460fba8p-1,
        0.0, 1.0, 0.0,
        -0x1.49d6e6460fba8p-1, 0.0, 0x1.8799656bf1a0dp-1 };
    M_LIBM_EQ(nm, ry);
    ivp_mat3r_init_rotation(&nm, IVP_INDEX_X, (ivp_real_t)-1.3f);
    static const double rx[9] = {
        1.0, 0.0, 0.0,
        0.0, 0x1.11eb3997fd91dp-2, 0x1.ed577f2ec09b1p-1,
        0.0, -0x1.ed577f2ec09b1p-1, 0x1.11eb3997fd91dp-2 };
    M_LIBM_EQ(nm, rx);
    ivp_mat3r_t m;
    set_m(&m, M_IN);
    LIBM_NEAR(ivp_mat3r_quad_rot_distance(&m, &nm), 0x1.c6584ee043e3p+1, 1e-5);
    ivp_mat3r_t mo;
    ivp_mat3r_mi2mult(&mo, &m, &nm);
    static const double mi2[9] = {
        0x1.ccccccp-1, 0x1.cca4ecb1d11acp-4, 0x1.5ec9f240aea33p-2,
        0x1.666666p-2, 0x1.431bed226119ap-3, -0x1.e25ee1bb2b7ebp-1,
        -0x1.333334p-3, 0x1.eefa076759a0bp-1, 0x1.2699392d9fc28p-3 };
    M_LIBM_NEAR(mo, mi2, 1e-6);
}

/* ═══ IVP_U_Matrix ══════════════════════════════════════════════════════ */

static const double ROT_MULT[9] = {
    0x1.952dc691a7bfbp-1, -0x1.4e657b6440094p-2, -0x1.089c5f4130182p-1,
    0x1.2b36f5d438f03p-2, 0x1.e3a3cd561e225p-1, -0x1.320c9f687ee6ep-3,
    0x1.12f0886b0257fp-1, -0x1.0c5465c0837fcp-5, 0x1.af9759986816fp-1 };

static void make_m_m2(ivp_transform_r_t *m, ivp_transform_r_t *m2) {
    const ivp_vec3r_t ang = {{ (ivp_real_t)0.3f, (ivp_real_t)-1.1f, (ivp_real_t)0.6f }};
    const ivp_vec3r_t ang2 = {{ 0.0f, (ivp_real_t)0.4f, 0.0f }};
    ivp_transform_init_rot_multiple(m, &ang, 0.5);
    ivp_vec3r_set(&m->pos, 0.5f, 1.5f, -2.5f);
    ivp_transform_init_rot_multiple(m2, &ang2, 1.0);
    ivp_vec3r_set(&m2->pos, -1.0f, 0.25f, 3.0f);
}

IVP_TEST(ref_matrix_init_rot_multiple_and_invert) {
    ivp_transform_r_t m;
    const ivp_vec3r_t ang = {{ (ivp_real_t)0.3f, (ivp_real_t)-1.1f, (ivp_real_t)0.6f }};
    ivp_transform_init_rot_multiple(&m, &ang, 0.5);
    M_LIBM_EQ(m.rot, ROT_MULT);
    static const double zero[3] = { 0.0, 0.0, 0.0 };
    V_EXACT(m.pos, zero);
    const ivp_vec3r_t ang2 = {{ 0.0f, (ivp_real_t)0.4f, 0.0f }};
    ivp_transform_init_rot_multiple(&m, &ang2, 2.0);
    static const double rmy[9] = {
        0x1.64b6bd9da4771p-1, 0.0, 0x1.6f494c7357ac5p-1,
        0.0, 1.0, -0.0,
        -0x1.6f494c7357ac5p-1, 0.0, 0x1.64b6bd9da4771p-1 };
    M_LIBM_EQ(m.rot, rmy);

    static const float g_v[9] = { 2.0f, -0.5f, 0.25f, 0.75f, 1.5f, -1.0f, 0.125f, 0.3f, 1.25f };
    ivp_transform_r_t g, gi;
    set_m(&g.rot, g_v);
    ivp_vec3r_set(&g.pos, 1.0f, -2.0f, 0.5f);
    IVP_ASSERT_EQ(ivp_transform_real_invert(&gi, &g, IVP_REF_DOUBLE_EPS), IVP_OK);
    static const double gim[9] = {
        0x1.c766e40855abdp-2, 0x1.2521dc51bd2cfp-3, 0x1.a2c2a856dbc67p-6,
        -0x1.bceed2dc4982dp-3, 0x1.027423e59ba48p-1, 0x1.ca04e81f00611p-2,
        0x1.f683384247afcp-8, -0x1.156dc3610c43ap-3, 0x1.61543e09496f7p-1 };
    static const double giv[3] = { -0x1.5eb639f29eba2p-3, 0x1.00d1613d44c8cp+0, -0x1.3f4e0725af68ep-1 };
    M_EQ(gi.rot, gim);
    V_NEAR(gi.pos, giv, 1e-6);

    float o[16];
    ivp_transform_get_4x4_column_major(&g, o);
    EXACT(o[1], 0.75);
    EXACT(o[4], -0.5);
    EXACT(o[13], -2.0);
    EXACT(o[15], 1.0);
}

IVP_TEST(ref_matrix_products_rotate_interpolate) {
    ivp_transform_r_t m, m2, out;
    make_m_m2(&m, &m2);

    ivp_transform_rotate(&out, &m, IVP_INDEX_Z, 0.9f);
    static const double rz[9] = {
        0x1.0d57ee2fa91fbp-2, -0x1.e2c7b9cab9bffp-1, -0x1.a233c8aeb5328p-3,
        0x1.9a6287737b39ap-1, 0x1.53543a9e30a48p-2, -0x1.fdacda92ff27bp-2,
        0x1.12f0886b0257fp-1, -0x1.0c5465c0837fcp-5, 0x1.af9759986816fp-1 };
    static const double rzv[3] = { -0x1.ba76808504dd8p-1, 0x1.52f6cdc8d7543p+0, -0x1.4p+1 };
    M_LIBM_NEAR(out.rot, rz, 1e-6);
    V_LIBM_NEAR(out.pos, rzv, 1e-6);

    ivp_transform_rotate_invers(&out, &m, IVP_INDEX_X, 0.4f);
    static const double rix[9] = {
        0x1.952dc691a7bfbp-1, -0x1.010b4762ac59dp-1, -0x1.6539aee7b3d86p-2,
        0x1.2b36f5d438f03p-2, 0x1.9faa9f9872a63p-1, -0x1.02cf8d6c8a6cep-1,
        0x1.12f0886b0257fp-1, 0x1.313eddd964b87p-2, 0x1.940d77d7e54cdp-1 };
    static const double rixv[3] = { 0x1p-1, 0x1.8p+0, -0x1.4p+1 };
    M_LIBM_NEAR(out.rot, rix, 1e-6);
    V_LIBM_NEAR(out.pos, rixv, 1e-6);

    /* IVP_U_Matrix::interpolate is element-wise (not orthonormal) */
    ivp_transform_interpolate(&out, &m, &m2, (ivp_real_t)0.3f);
    static const double ip[9] = {
        0x1.a919a292f9ee4p-1, -0x1.d427ac39caa86p-3, -0x1.f5a6d37f8393fp-3,
        0x1.a2e68ae4a0212p-3, 0x1.ec25dc8ec1222p-1, -0x1.ac7811e4ac9d1p-4,
        0x1.09493cf62cdfap-2, -0x1.77a95ad56323cp-6, 0x1.bb96bcac050dep-1 };
    static const double ipv[3] = { 0x1.99999p-5, 0x1.1fffffcp+0, -0x1.b33331p-1 };
    M_LIBM_NEAR(out.rot, ip, 1e-6);
    V_LIBM_NEAR(out.pos, ipv, 1e-6);

    ivp_transform_compose(&out, &m, &m2);
    static const double mm[9] = {
        0x1.dc3d1c0cccfdp-1, -0x1.4e657b6440094p-2, -0x1.57c12d2b700d8p-3,
        0x1.4f2f82894a8cep-2, 0x1.e3a3cd561e225p-1, -0x1.86ce3b685af9p-6,
        0x1.54aa95e2f38a4p-3, -0x1.0c5465c0837fcp-5, 0x1.f8969ddcaa2bep-1 };
    static const double mmv[3] = { -0x1.ec67c9e0e004ap+0, 0x1.fdc400dd0be35p-1, -0x1.085bcd38cc214p-1 };
    M_LIBM_NEAR(out.rot, mm, 1e-6);
    V_LIBM_NEAR(out.pos, mmv, 1e-6);
    /* in-place (m_out == this) is allowed in the reference */
    ivp_transform_r_t inplace = m;
    ivp_transform_compose(&inplace, &inplace, &m2);
    M_LIBM_NEAR(inplace.rot, mm, 1e-6);
    V_LIBM_NEAR(inplace.pos, mmv, 1e-6);

    ivp_transform_inv_compose(&out, &m, &m2);
    static const double mi[9] = {
        0x1.0a20bcbed0199p-1, 0x1.2b36f5d438f03p-2, 0x1.9b052af76a8f2p-1,
        -0x1.26f01c32e8aaep-2, 0x1.e3a3cd561e225p-1, -0x1.423a29a3f59e2p-3,
        -0x1.9bcad0c989affp-1, -0x1.320c9f687ee6ep-3, 0x1.267a3e8ead487p-1 };
    static const double miv[3] = { 0x1.66b1f566b6d71p+0, -0x1.bdfda71aa2d7ep-1, 0x1.6649ddaf15895p+2 };
    M_LIBM_NEAR(out.rot, mi, 1e-6);
    V_LIBM_NEAR(out.pos, miv, 1e-5);

    ivp_transform_mi2mult(&out, &m, &m2);
    static const double m2i[9] = {
        0x1.0e266c092368ep-1, -0x1.4e657b6440094p-2, -0x1.9181bc06825ebp-1,
        0x1.b0023cc7468afp-3, 0x1.e3a3cd561e225p-1, -0x1.0176f879a7959p-2,
        0x1.a54e3fba71e06p-1, -0x1.0c5465c0837fcp-5, 0x1.22748f4459f92p-1 };
    static const double m2iv[3] = { 0x1.bb1e13e24ca18p+1, 0x1.1d528424b15eap+1, -0x1.af77871ee67f4p+1 };
    M_LIBM_NEAR(out.rot, m2i, 1e-6);
    V_LIBM_NEAR(out.pos, m2iv, 1e-5);

    LIBM_NEAR(ivp_transform_quad_distance(&m, &m2), 0x1.1fde09bae323ep+5, 1e-4);
}

int main(void) {
    IVP_REGISTER(ref_isqrt_double);
    IVP_REGISTER(ref_isqrt_float_and_inline_double);
    IVP_REGISTER(ref_inline_math_approximations);
    IVP_REGISTER(ref_invert_2x2);
    IVP_REGISTER(ref_res_eps_constant);
    IVP_REGISTER(ref_quat_fast_multiple_with_clip);
    IVP_REGISTER(ref_quat_very_fast_multiple);
    IVP_REGISTER(ref_quat_set_variants_and_angles);
    IVP_REGISTER(ref_quat_normize_and_products);
    IVP_REGISTER(ref_quat_interpolate_smoothly);
    IVP_REGISTER(ref_quat_fast_normize_and_correct_steps);
    IVP_REGISTER(ref_quat_from_rotation_vectors);
    IVP_REGISTER(ref_quat_matrix_conversions);
    IVP_REGISTER(ref_point_normize_lengths);
    IVP_REGISTER(ref_point_helpers);
    IVP_REGISTER(ref_point_set_crossing);
    IVP_REGISTER(ref_matrix3_orthogonize_handedness);
    IVP_REGISTER(ref_matrix3_get_angles);
    IVP_REGISTER(ref_matrix3_eigen_vector);
    IVP_REGISTER(ref_matrix3_init_normized);
    IVP_REGISTER(ref_matrix3_rotation_distance_mi2mult);
    IVP_REGISTER(ref_matrix_init_rot_multiple_and_invert);
    IVP_REGISTER(ref_matrix_products_rotate_interpolate);
    IVP_RUN_ALL();
}
