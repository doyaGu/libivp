/* test_math.c — Unit tests for IVP C17 math module */
#include "ivp_test.h"
#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_fast_math.h>

/* Tolerance for float/double comparisons */
#ifdef IVP_REAL_DOUBLE
    #define TOL  1e-10
    #define LTOL 1e-6   /* loose tolerance for fast-math */
#else
    #define TOL  1e-5
    #define LTOL 1e-2   /* loose tolerance for fast-math */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Vec3 tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(vec3_construct) {
    ivp_vec3r_t v = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    IVP_ASSERT_NEAR(v.k[0], 1.0, TOL);
    IVP_ASSERT_NEAR(v.k[1], 2.0, TOL);
    IVP_ASSERT_NEAR(v.k[2], 3.0, TOL);
}

IVP_TEST(vec3_zero) {
    ivp_vec3r_t v = ivp_vec3r_zero();
    IVP_ASSERT_NEAR(v.k[0], 0.0, TOL);
    IVP_ASSERT_NEAR(v.k[1], 0.0, TOL);
    IVP_ASSERT_NEAR(v.k[2], 0.0, TOL);
}

IVP_TEST(vec3_add_sub) {
    ivp_vec3r_t a = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    ivp_vec3r_t b = ivp_vec3r(IVP_REAL(4.0), IVP_REAL(5.0), IVP_REAL(6.0));
    ivp_vec3r_t r;

    ivp_vec3r_add(&r, &a, &b);
    IVP_ASSERT_NEAR(r.k[0], 5.0, TOL);
    IVP_ASSERT_NEAR(r.k[1], 7.0, TOL);
    IVP_ASSERT_NEAR(r.k[2], 9.0, TOL);

    ivp_vec3r_sub(&r, &a, &b);
    IVP_ASSERT_NEAR(r.k[0], -3.0, TOL);
    IVP_ASSERT_NEAR(r.k[1], -3.0, TOL);
    IVP_ASSERT_NEAR(r.k[2], -3.0, TOL);
}

IVP_TEST(vec3_scale) {
    ivp_vec3r_t a = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(-3.0), IVP_REAL(4.0));
    ivp_vec3r_t r;
    ivp_vec3r_scale(&r, &a, IVP_REAL(0.5));
    IVP_ASSERT_NEAR(r.k[0], 1.0, TOL);
    IVP_ASSERT_NEAR(r.k[1], -1.5, TOL);
    IVP_ASSERT_NEAR(r.k[2], 2.0, TOL);
}

IVP_TEST(vec3_dot) {
    ivp_vec3r_t a = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    ivp_vec3r_t b = ivp_vec3r(IVP_REAL(4.0), IVP_REAL(-5.0), IVP_REAL(6.0));
    ivp_real_t d = ivp_vec3r_dot(&a, &b);
    IVP_ASSERT_NEAR(d, 12.0, TOL);  /* 4 - 10 + 18 = 12 */
}

IVP_TEST(vec3_cross) {
    ivp_vec3r_t x = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t y = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_vec3r_t z;
    ivp_vec3r_cross(&z, &x, &y);
    IVP_ASSERT_NEAR(z.k[0], 0.0, TOL);
    IVP_ASSERT_NEAR(z.k[1], 0.0, TOL);
    IVP_ASSERT_NEAR(z.k[2], 1.0, TOL);
}

IVP_TEST(vec3_quad_length) {
    ivp_vec3r_t v = ivp_vec3r(IVP_REAL(3.0), IVP_REAL(4.0), IVP_REAL(0.0));
    IVP_ASSERT_NEAR(ivp_vec3r_quad_length(&v), 25.0, TOL);
}

IVP_TEST(vec3_length) {
    ivp_vec3r_t v = ivp_vec3r(IVP_REAL(3.0), IVP_REAL(4.0), IVP_REAL(0.0));
    IVP_ASSERT_NEAR(ivp_vec3r_length(&v), 5.0, TOL);
}

IVP_TEST(vec3_normalize) {
    ivp_vec3r_t v = ivp_vec3r(IVP_REAL(3.0), IVP_REAL(4.0), IVP_REAL(0.0));
    ivp_result_t r = ivp_vec3r_normalize(&v);
    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_NEAR(ivp_vec3r_length(&v), 1.0, TOL);
    IVP_ASSERT_NEAR(v.k[0], 0.6, TOL);
    IVP_ASSERT_NEAR(v.k[1], 0.8, TOL);
}

IVP_TEST(vec3_fast_normalize) {
    ivp_vec3r_t v = ivp_vec3r(IVP_REAL(3.0), IVP_REAL(4.0), IVP_REAL(0.0));
    ivp_vec3r_fast_normalize(&v);
    IVP_ASSERT_NEAR(ivp_vec3r_length(&v), 1.0, LTOL);
}

IVP_TEST(vec3_lerp) {
    ivp_vec3r_t a = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t b = ivp_vec3r(IVP_REAL(10.0), IVP_REAL(20.0), IVP_REAL(30.0));
    ivp_vec3r_t r;
    ivp_vec3r_lerp(&r, &a, &b, IVP_REAL(0.25));
    IVP_ASSERT_NEAR(r.k[0], 2.5, TOL);
    IVP_ASSERT_NEAR(r.k[1], 5.0, TOL);
    IVP_ASSERT_NEAR(r.k[2], 7.5, TOL);
}

IVP_TEST(vec3_add_scaled) {
    ivp_vec3r_t a = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    ivp_vec3r_t v = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(4.0), IVP_REAL(6.0));
    ivp_vec3r_add_scaled(&a, &v, IVP_REAL(0.5));
    IVP_ASSERT_NEAR(a.k[0], 2.0, TOL);
    IVP_ASSERT_NEAR(a.k[1], 4.0, TOL);
    IVP_ASSERT_NEAR(a.k[2], 6.0, TOL);
}

IVP_TEST(vec3_conversion) {
    ivp_vec3f_t f = ivp_vec3f(1.5f, 2.5f, 3.5f);
    ivp_vec3r_t r = ivp_vec3r_from_f(&f);
    IVP_ASSERT_NEAR(r.k[0], 1.5, TOL);
    IVP_ASSERT_NEAR(r.k[1], 2.5, TOL);
    IVP_ASSERT_NEAR(r.k[2], 3.5, TOL);

    ivp_vec3f_t f2 = ivp_vec3f_from_r(&r);
    IVP_ASSERT_NEAR(f2.k[0], 1.5, 1e-5);
    IVP_ASSERT_NEAR(f2.k[1], 2.5, 1e-5);
    IVP_ASSERT_NEAR(f2.k[2], 3.5, 1e-5);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Mat3 tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(mat3_identity) {
    ivp_mat3r_t m;
    ivp_mat3r_identity(&m);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ivp_real_t expected = (i == j) ? IVP_REAL(1.0) : IVP_REAL(0.0);
            IVP_ASSERT_NEAR(ivp_mat3r_get(&m, i, j), expected, TOL);
        }
    }
}

IVP_TEST(mat3_vmult) {
    ivp_mat3r_t m;
    ivp_mat3r_identity(&m);
    ivp_vec3r_t v = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    ivp_vec3r_t out;
    ivp_mat3r_vmult(&out, &m, &v);
    IVP_ASSERT_NEAR(out.k[0], 1.0, TOL);
    IVP_ASSERT_NEAR(out.k[1], 2.0, TOL);
    IVP_ASSERT_NEAR(out.k[2], 3.0, TOL);
}

IVP_TEST(mat3_transpose) {
    ivp_mat3r_t m;
    ivp_mat3r_identity(&m);
    ivp_mat3r_set(&m, 0, 1, IVP_REAL(2.0));
    ivp_mat3r_set(&m, 1, 0, IVP_REAL(3.0));

    ivp_mat3r_t mt;
    ivp_mat3r_transpose(&mt, &m);
    IVP_ASSERT_NEAR(ivp_mat3r_get(&mt, 0, 1), 3.0, TOL);
    IVP_ASSERT_NEAR(ivp_mat3r_get(&mt, 1, 0), 2.0, TOL);
}

IVP_TEST(mat3_mult) {
    ivp_mat3r_t a, b, out;
    ivp_mat3r_identity(&a);
    ivp_mat3r_identity(&b);

    /* a = identity, b = identity → out = identity */
    ivp_mat3r_mult(&out, &a, &b);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ivp_real_t expected = (i == j) ? IVP_REAL(1.0) : IVP_REAL(0.0);
            IVP_ASSERT_NEAR(ivp_mat3r_get(&out, i, j), expected, TOL);
        }
    }
}

IVP_TEST(mat3_determinant) {
    ivp_mat3r_t m;
    ivp_mat3r_identity(&m);
    IVP_ASSERT_NEAR(ivp_mat3r_determinant(&m), 1.0, TOL);

    /* Scale row 0 by 2 → det = 2 */
    ivp_mat3r_set(&m, 0, 0, IVP_REAL(2.0));
    IVP_ASSERT_NEAR(ivp_mat3r_determinant(&m), 2.0, TOL);
}

IVP_TEST(mat3_invert) {
    ivp_mat3r_t m;
    ivp_mat3r_identity(&m);
    ivp_mat3r_set(&m, 0, 0, IVP_REAL(2.0));
    ivp_mat3r_set(&m, 1, 1, IVP_REAL(3.0));
    ivp_mat3r_set(&m, 2, 2, IVP_REAL(4.0));

    ivp_mat3r_t inv;
    ivp_result_t r = ivp_mat3r_invert(&inv, &m);
    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_NEAR(ivp_mat3r_get(&inv, 0, 0), 0.5, TOL);
    IVP_ASSERT_NEAR(ivp_mat3r_get(&inv, 1, 1), 1.0/3.0, TOL);
    IVP_ASSERT_NEAR(ivp_mat3r_get(&inv, 2, 2), 0.25, TOL);
}

IVP_TEST(mat3_rotation_x) {
    ivp_mat3r_t m;
    ivp_mat3r_init_rotation(&m, IVP_INDEX_X, IVP_PI_2);

    /* Rotate Y-axis around X → should become Z-axis */
    ivp_vec3r_t v = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_vec3r_t out;
    ivp_mat3r_vmult(&out, &m, &v);
    IVP_ASSERT_NEAR(out.k[0], 0.0, TOL);
    IVP_ASSERT_NEAR(out.k[1], 0.0, TOL);
    IVP_ASSERT_NEAR(out.k[2], 1.0, TOL);
}

IVP_TEST(mat3_orthonormalize) {
    ivp_mat3r_t m;
    ivp_mat3r_identity(&m);
    /* Perturb slightly */
    ivp_mat3r_set(&m, 0, 0, IVP_REAL(1.01));
    ivp_mat3r_set(&m, 1, 1, IVP_REAL(0.99));

    ivp_result_t r = ivp_mat3r_orthonormalize(&m);
    IVP_ASSERT_TRUE(ivp_ok(r));

    /* Check rows are unit length */
    for (int i = 0; i < 3; i++) {
        IVP_ASSERT_NEAR(ivp_vec3r_quad_length(&m.rows[i]), 1.0, TOL);
    }
    /* Check rows are orthogonal */
    IVP_ASSERT_NEAR(ivp_vec3r_dot(&m.rows[0], &m.rows[1]), 0.0, TOL);
    IVP_ASSERT_NEAR(ivp_vec3r_dot(&m.rows[0], &m.rows[2]), 0.0, TOL);
    IVP_ASSERT_NEAR(ivp_vec3r_dot(&m.rows[1], &m.rows[2]), 0.0, TOL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Quaternion tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(quat_identity) {
    ivp_quatr_t q = ivp_quatr_identity();
    IVP_ASSERT_NEAR(q.x, 0.0, TOL);
    IVP_ASSERT_NEAR(q.y, 0.0, TOL);
    IVP_ASSERT_NEAR(q.z, 0.0, TOL);
    IVP_ASSERT_NEAR(q.w, 1.0, TOL);
}

IVP_TEST(quat_mult_identity) {
    ivp_quatr_t a = ivp_quatr_identity();
    ivp_quatr_t b = ivp_quatr_make(IVP_REAL(0.5), IVP_REAL(0.5),
                                    IVP_REAL(0.5), IVP_REAL(0.5));
    ivp_quatr_t r;
    ivp_quat_mult(&r, &a, &b);
    IVP_ASSERT_NEAR(r.x, 0.5, TOL);
    IVP_ASSERT_NEAR(r.y, 0.5, TOL);
    IVP_ASSERT_NEAR(r.z, 0.5, TOL);
    IVP_ASSERT_NEAR(r.w, 0.5, TOL);
}

IVP_TEST(quat_conjugate) {
    ivp_quatr_t q = ivp_quatr_make(IVP_REAL(1.0), IVP_REAL(2.0),
                                    IVP_REAL(3.0), IVP_REAL(4.0));
    ivp_quatr_t c = ivp_quat_conjugate(&q);
    IVP_ASSERT_NEAR(c.x, -1.0, TOL);
    IVP_ASSERT_NEAR(c.y, -2.0, TOL);
    IVP_ASSERT_NEAR(c.z, -3.0, TOL);
    IVP_ASSERT_NEAR(c.w,  4.0, TOL);
}

IVP_TEST(quat_normalize) {
    ivp_quatr_t q = ivp_quatr_make(IVP_REAL(1.0), IVP_REAL(2.0),
                                    IVP_REAL(3.0), IVP_REAL(4.0));
    ivp_result_t r = ivp_quat_normalize(&q);
    IVP_ASSERT_TRUE(ivp_ok(r));

    ivp_real_t len2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    IVP_ASSERT_NEAR(len2, 1.0, TOL);
}

IVP_TEST(quat_fast_normalize) {
    ivp_quatr_t q = ivp_quatr_make(IVP_REAL(0.49), IVP_REAL(0.51),
                                    IVP_REAL(0.49), IVP_REAL(0.51));
    ivp_quat_fast_normalize(&q);

    ivp_real_t len2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    IVP_ASSERT_NEAR(len2, 1.0, LTOL);
}

IVP_TEST(quat_to_mat3_identity) {
    ivp_quatr_t q = ivp_quatr_identity();
    ivp_mat3r_t m;
    ivp_quat_to_mat3(&q, &m);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ivp_real_t expected = (i == j) ? IVP_REAL(1.0) : IVP_REAL(0.0);
            IVP_ASSERT_NEAR(ivp_mat3r_get(&m, i, j), expected, TOL);
        }
    }
}

IVP_TEST(quat_roundtrip_mat3) {
    /* Create a rotation quat: 90° around Z */
    ivp_real_t half = IVP_PI / IVP_REAL(4.0);
    ivp_quatr_t q = ivp_quatr_make(IVP_REAL(0.0), IVP_REAL(0.0),
                                    ivp_sin(half), ivp_cos(half));

    ivp_mat3r_t m;
    ivp_quat_to_mat3(&q, &m);

    ivp_quatr_t q2;
    ivp_quat_from_mat3(&q2, &m);

    /* Should match (up to sign) */
    ivp_real_t dot = ivp_quat_dot(&q, &q2);
    IVP_ASSERT_NEAR(fabs((double)dot), 1.0, TOL);
}

IVP_TEST(quat_slerp_endpoints) {
    ivp_quatr_t a = ivp_quatr_identity();
    ivp_real_t half = IVP_PI / IVP_REAL(4.0);
    ivp_quatr_t b = ivp_quatr_make(IVP_REAL(0.0), IVP_REAL(0.0),
                                    ivp_sin(half), ivp_cos(half));

    ivp_quatr_t r;
    ivp_quat_slerp(&r, &a, &b, IVP_REAL(0.0));
    IVP_ASSERT_NEAR(r.x, a.x, TOL);
    IVP_ASSERT_NEAR(r.y, a.y, TOL);
    IVP_ASSERT_NEAR(r.z, a.z, TOL);
    IVP_ASSERT_NEAR(r.w, a.w, TOL);

    ivp_quat_slerp(&r, &a, &b, IVP_REAL(1.0));
    ivp_real_t dot = ivp_quat_dot(&r, &b);
    IVP_ASSERT_NEAR(fabs((double)dot), 1.0, TOL);
}

IVP_TEST(quat_angle_diff_identity) {
    ivp_quatr_t a = ivp_quatr_identity();
    ivp_quatr_t b = ivp_quatr_identity();
    IVP_ASSERT_NEAR(ivp_quat_angle_diff(&a, &b), 0.0, TOL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Transform tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(transform_identity) {
    ivp_transform_r_t t;
    ivp_transform_identity(&t);

    ivp_vec3r_t p = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    ivp_vec3r_t out;
    ivp_transform_point(&out, &t, &p);
    IVP_ASSERT_NEAR(out.k[0], 1.0, TOL);
    IVP_ASSERT_NEAR(out.k[1], 2.0, TOL);
    IVP_ASSERT_NEAR(out.k[2], 3.0, TOL);
}

IVP_TEST(transform_translation) {
    ivp_transform_r_t t;
    ivp_transform_identity(&t);
    t.pos = ivp_vec3r(IVP_REAL(10.0), IVP_REAL(20.0), IVP_REAL(30.0));

    ivp_vec3r_t p = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    ivp_vec3r_t out;
    ivp_transform_point(&out, &t, &p);
    IVP_ASSERT_NEAR(out.k[0], 11.0, TOL);
    IVP_ASSERT_NEAR(out.k[1], 22.0, TOL);
    IVP_ASSERT_NEAR(out.k[2], 33.0, TOL);

    /* Direction should NOT be affected by translation */
    ivp_transform_dir(&out, &t, &p);
    IVP_ASSERT_NEAR(out.k[0], 1.0, TOL);
    IVP_ASSERT_NEAR(out.k[1], 2.0, TOL);
    IVP_ASSERT_NEAR(out.k[2], 3.0, TOL);
}

IVP_TEST(transform_inv_roundtrip) {
    ivp_transform_r_t t;
    ivp_transform_identity(&t);
    ivp_mat3r_init_rotation(&t.rot, IVP_INDEX_Z, IVP_PI / IVP_REAL(4.0));
    t.pos = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(10.0), IVP_REAL(15.0));

    ivp_transform_r_t inv;
    ivp_transform_invert(&inv, &t);

    ivp_vec3r_t p = ivp_vec3r(IVP_REAL(3.0), IVP_REAL(7.0), IVP_REAL(-2.0));
    ivp_vec3r_t q, r;
    ivp_transform_point(&q, &t, &p);
    ivp_transform_point(&r, &inv, &q);

    IVP_ASSERT_NEAR(r.k[0], 3.0, TOL);
    IVP_ASSERT_NEAR(r.k[1], 7.0, TOL);
    IVP_ASSERT_NEAR(r.k[2], -2.0, TOL);
}

IVP_TEST(transform_compose) {
    /* Compose translation only */
    ivp_transform_r_t a, b;
    ivp_transform_identity(&a);
    ivp_transform_identity(&b);
    a.pos = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    b.pos = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));

    ivp_transform_r_t c;
    ivp_transform_compose(&c, &a, &b);

    ivp_vec3r_t origin = ivp_vec3r_zero();
    ivp_vec3r_t out;
    ivp_transform_point(&out, &c, &origin);
    IVP_ASSERT_NEAR(out.k[0], 1.0, TOL);
    IVP_ASSERT_NEAR(out.k[1], 2.0, TOL);
    IVP_ASSERT_NEAR(out.k[2], 0.0, TOL);
}

IVP_TEST(transform_interpolate_halfturn) {
    ivp_transform_r_t t1, t2, out;
    ivp_transform_identity(&t1);
    ivp_transform_identity(&t2);

    t1.pos = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t2.pos = ivp_vec3r(IVP_REAL(10.0), IVP_REAL(20.0), IVP_REAL(30.0));
    ivp_mat3r_init_rotation(&t2.rot, IVP_INDEX_X, IVP_PI);

    ivp_transform_interpolate(&out, &t1, &t2, IVP_REAL(0.5));

    /* Position should be linearly interpolated. */
    IVP_ASSERT_NEAR(out.pos.k[0], 5.0, TOL);
    IVP_ASSERT_NEAR(out.pos.k[1], 10.0, TOL);
    IVP_ASSERT_NEAR(out.pos.k[2], 15.0, TOL);

    /* Rotation should remain orthonormal (no collapsed rows). */
    for (int i = 0; i < 3; i++) {
        IVP_ASSERT_NEAR(ivp_vec3r_quad_length(&out.rot.rows[i]), 1.0, LTOL);
    }
    IVP_ASSERT_NEAR(ivp_vec3r_dot(&out.rot.rows[0], &out.rot.rows[1]), 0.0, LTOL);
    IVP_ASSERT_NEAR(ivp_vec3r_dot(&out.rot.rows[0], &out.rot.rows[2]), 0.0, LTOL);
    IVP_ASSERT_NEAR(ivp_vec3r_dot(&out.rot.rows[1], &out.rot.rows[2]), 0.0, LTOL);

    /* Halfway between identity and 180 degrees around X is 90 degrees around X. */
    ivp_vec3r_t y = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_vec3r_t y_rot;
    ivp_mat3r_vmult(&y_rot, &out.rot, &y);
    IVP_ASSERT_NEAR(y_rot.k[0], 0.0, LTOL);
    IVP_ASSERT_NEAR(y_rot.k[1], 0.0, LTOL);
    IVP_ASSERT_NEAR(fabs((double)y_rot.k[2]), 1.0, LTOL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Fast math tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(isqrt_float_basic) {
    /* isqrt(4) ≈ 0.5 */
    float val = ivp_isqrt_float(4.0f);
    IVP_ASSERT_NEAR(val, 0.5, 1e-3);

    /* isqrt(1) ≈ 1.0 */
    val = ivp_isqrt_float(1.0f);
    IVP_ASSERT_NEAR(val, 1.0, 1e-3);

    /* isqrt(100) ≈ 0.1 */
    val = ivp_isqrt_float(100.0f);
    IVP_ASSERT_NEAR(val, 0.1, 1e-3);
}

IVP_TEST(isqrt_double_precision) {
    /* resolution_steps=0: ~1e-3 precision */
    double v0 = ivp_isqrt_double(4.0, 0);
    IVP_ASSERT_NEAR(v0, 0.5, 1e-3);

    /* resolution_steps=2: ~1e-14 precision */
    double v2 = ivp_isqrt_double(4.0, 2);
    IVP_ASSERT_NEAR(v2, 0.5, 1e-12);

    /* resolution_steps=3: ~1e-16 precision */
    double v3 = ivp_isqrt_double(4.0, 3);
    IVP_ASSERT_NEAR(v3, 0.5, 1e-14);
}

IVP_TEST(isqrt_various_values) {
    /* Test with several values to verify robustness */
    double vals[] = {0.01, 0.1, 0.5, 1.0, 2.0, 10.0, 100.0, 10000.0};
    int count = (int)(sizeof(vals) / sizeof(vals[0]));
    for (int i = 0; i < count; i++) {
        double result = ivp_isqrt_double(vals[i], 2);
        double expected = 1.0 / sqrt(vals[i]);
        IVP_ASSERT_NEAR(result, expected, 1e-12);
    }
}

IVP_TEST(isqrt_double_domain) {
    double z = ivp_isqrt_double(0.0, 0);
    IVP_ASSERT_TRUE(isinf(z));
    IVP_ASSERT_TRUE(z > 0.0);

    double n = ivp_isqrt_double(-4.0, 0);
    IVP_ASSERT_TRUE(isnan(n));
}

IVP_TEST(sqrt_wrapper) {
    IVP_ASSERT_NEAR(ivp_sqrt(IVP_REAL(4.0)), 2.0, TOL);
    IVP_ASSERT_NEAR(ivp_sqrt(IVP_REAL(9.0)), 3.0, TOL);
}

IVP_TEST(fabs_wrapper) {
    IVP_ASSERT_NEAR(ivp_fabs(IVP_REAL(-5.0)), 5.0, TOL);
    IVP_ASSERT_NEAR(ivp_fabs(IVP_REAL(3.0)), 3.0, TOL);
}

IVP_TEST(min_max_clamp) {
    IVP_ASSERT_NEAR(ivp_min(IVP_REAL(3.0), IVP_REAL(5.0)), 3.0, TOL);
    IVP_ASSERT_NEAR(ivp_max(IVP_REAL(3.0), IVP_REAL(5.0)), 5.0, TOL);
    IVP_ASSERT_NEAR(ivp_clamp(IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(1.0)), 0.0, TOL);
    IVP_ASSERT_NEAR(ivp_clamp(IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(1.0)), 1.0, TOL);
    IVP_ASSERT_NEAR(ivp_clamp(IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(1.0)), 0.5, TOL);
}

IVP_TEST(trig_wrappers) {
    IVP_ASSERT_NEAR(ivp_sin(IVP_REAL(0.0)), 0.0, TOL);
    IVP_ASSERT_NEAR(ivp_cos(IVP_REAL(0.0)), 1.0, TOL);
    IVP_ASSERT_NEAR(ivp_sin(IVP_PI_2), 1.0, TOL);
    IVP_ASSERT_NEAR(ivp_cos(IVP_PI_2), 0.0, TOL);
}

/* ── Main ───────────────────────────────────────────────────────────────── */

int main(void) {
    /* Vec3 */
    IVP_REGISTER(vec3_construct);
    IVP_REGISTER(vec3_zero);
    IVP_REGISTER(vec3_add_sub);
    IVP_REGISTER(vec3_scale);
    IVP_REGISTER(vec3_dot);
    IVP_REGISTER(vec3_cross);
    IVP_REGISTER(vec3_quad_length);
    IVP_REGISTER(vec3_length);
    IVP_REGISTER(vec3_normalize);
    IVP_REGISTER(vec3_fast_normalize);
    IVP_REGISTER(vec3_lerp);
    IVP_REGISTER(vec3_add_scaled);
    IVP_REGISTER(vec3_conversion);

    /* Mat3 */
    IVP_REGISTER(mat3_identity);
    IVP_REGISTER(mat3_vmult);
    IVP_REGISTER(mat3_transpose);
    IVP_REGISTER(mat3_mult);
    IVP_REGISTER(mat3_determinant);
    IVP_REGISTER(mat3_invert);
    IVP_REGISTER(mat3_rotation_x);
    IVP_REGISTER(mat3_orthonormalize);

    /* Quaternion */
    IVP_REGISTER(quat_identity);
    IVP_REGISTER(quat_mult_identity);
    IVP_REGISTER(quat_conjugate);
    IVP_REGISTER(quat_normalize);
    IVP_REGISTER(quat_fast_normalize);
    IVP_REGISTER(quat_to_mat3_identity);
    IVP_REGISTER(quat_roundtrip_mat3);
    IVP_REGISTER(quat_slerp_endpoints);
    IVP_REGISTER(quat_angle_diff_identity);

    /* Transform */
    IVP_REGISTER(transform_identity);
    IVP_REGISTER(transform_translation);
    IVP_REGISTER(transform_inv_roundtrip);
    IVP_REGISTER(transform_compose);
    IVP_REGISTER(transform_interpolate_halfturn);

    /* Fast math */
    IVP_REGISTER(isqrt_float_basic);
    IVP_REGISTER(isqrt_double_precision);
    IVP_REGISTER(isqrt_various_values);
    IVP_REGISTER(isqrt_double_domain);
    IVP_REGISTER(sqrt_wrapper);
    IVP_REGISTER(fabs_wrapper);
    IVP_REGISTER(min_max_clamp);
    IVP_REGISTER(trig_wrappers);

    IVP_RUN_ALL();
}
