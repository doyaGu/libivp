/* test_solver.c �?Tests for M5: solver / impact / friction / contact
 *
 * Test suites:
 *   1. Great matrix solver (Ax=b, determinant, singular)
 *   2. Virtual mass computation
 *   3. Contact geometry (sphere-sphere)
 *   4. Material combination
 *   5. Impulse application (momentum conservation)
 *   6. Impact resolution (elastic/inelastic two-ball)
 *   7. Friction (static/dynamic)
 *   8. Two-ball scenario (multi-step)
 *   9. Kinetic energy queries
 *  10. Broadphase detection
 */
#include "ivp_test.h"

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/collision/ivp_collision_tolerance.h>
#include <ivp/solver/ivp_solver_constants.h>
#include <ivp/solver/ivp_great_matrix.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/solver/ivp_impact.h>
#include <ivp/dynamics/ivp_environment_collision.h>
#include <ivp/math/ivp_fast_math.h>
#include <string.h>

/* ════════════════════════════════════════════════════════════════════════════
 * Great Matrix Tests
 * ════════════════════════════════════════════════════════════════════════════ */

/* 1x1: trivial case */
IVP_TEST(matrix_1x1) {
    ivp_great_matrix_t m;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m, 1, NULL), IVP_OK);
    ivp_great_matrix_set(&m, 0, 0, IVP_REAL(2.0));
    ivp_great_matrix_set_rhs(&m, 0, IVP_REAL(6.0));
    ivp_real_t x;
    IVP_ASSERT_EQ(ivp_great_matrix_solve(&m, &x), IVP_OK);
    IVP_ASSERT_NEAR(x, 3.0, 1e-6);
    ivp_great_matrix_destroy(&m, NULL);
}

/* 2x2 system */
IVP_TEST(matrix_2x2) {
    ivp_great_matrix_t m;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m, 2, NULL), IVP_OK);
    /* [2 1 | 5]
     * [1 3 | 7]  �?x=8/5=1.6, y=9/5=1.8 */
    ivp_great_matrix_set(&m, 0, 0, IVP_REAL(2.0));
    ivp_great_matrix_set(&m, 0, 1, IVP_REAL(1.0));
    ivp_great_matrix_set_rhs(&m, 0, IVP_REAL(5.0));
    ivp_great_matrix_set(&m, 1, 0, IVP_REAL(1.0));
    ivp_great_matrix_set(&m, 1, 1, IVP_REAL(3.0));
    ivp_great_matrix_set_rhs(&m, 1, IVP_REAL(7.0));
    ivp_real_t x[2];
    IVP_ASSERT_EQ(ivp_great_matrix_solve(&m, x), IVP_OK);
    IVP_ASSERT_NEAR(x[0], 1.6, 1e-6);
    IVP_ASSERT_NEAR(x[1], 1.8, 1e-6);
    ivp_great_matrix_destroy(&m, NULL);
}

/* 3x3 system with pivoting */
IVP_TEST(matrix_3x3_pivot) {
    ivp_great_matrix_t m;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m, 3, NULL), IVP_OK);
    /* [0  1  1 | 6]
     * [3  2  1 | 11]
     * [1  1  1 | 7]   �?x=1, y=2, z=4
     * Requires pivoting: column 0 row 0 is 0, pivot to row 1. */
    ivp_great_matrix_set(&m, 0, 0, IVP_REAL(0.0));
    ivp_great_matrix_set(&m, 0, 1, IVP_REAL(1.0));
    ivp_great_matrix_set(&m, 0, 2, IVP_REAL(1.0));
    ivp_great_matrix_set_rhs(&m, 0, IVP_REAL(6.0));
    ivp_great_matrix_set(&m, 1, 0, IVP_REAL(3.0));
    ivp_great_matrix_set(&m, 1, 1, IVP_REAL(2.0));
    ivp_great_matrix_set(&m, 1, 2, IVP_REAL(1.0));
    ivp_great_matrix_set_rhs(&m, 1, IVP_REAL(11.0));
    ivp_great_matrix_set(&m, 2, 0, IVP_REAL(1.0));
    ivp_great_matrix_set(&m, 2, 1, IVP_REAL(1.0));
    ivp_great_matrix_set(&m, 2, 2, IVP_REAL(1.0));
    ivp_great_matrix_set_rhs(&m, 2, IVP_REAL(7.0));
    ivp_real_t x[3];
    IVP_ASSERT_EQ(ivp_great_matrix_solve(&m, x), IVP_OK);
    IVP_ASSERT_NEAR(x[0], 1.0, 1e-5);
    IVP_ASSERT_NEAR(x[1], 2.0, 1e-5);
    IVP_ASSERT_NEAR(x[2], 4.0, 1e-5);
    ivp_great_matrix_destroy(&m, NULL);
}

/* Singular matrix �?error */
IVP_TEST(matrix_singular) {
    ivp_great_matrix_t m;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m, 2, NULL), IVP_OK);
    ivp_great_matrix_set(&m, 0, 0, IVP_REAL(1.0));
    ivp_great_matrix_set(&m, 0, 1, IVP_REAL(2.0));
    ivp_great_matrix_set_rhs(&m, 0, IVP_REAL(3.0));
    ivp_great_matrix_set(&m, 1, 0, IVP_REAL(2.0));
    ivp_great_matrix_set(&m, 1, 1, IVP_REAL(4.0));
    ivp_great_matrix_set_rhs(&m, 1, IVP_REAL(6.0));
    ivp_real_t x[2];
    IVP_ASSERT_EQ(ivp_great_matrix_solve(&m, x), IVP_ERR_SINGULAR_MATRIX);
    ivp_great_matrix_destroy(&m, NULL);
}

/* Determinant computation */
IVP_TEST(matrix_determinant) {
    ivp_great_matrix_t m;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m, 3, NULL), IVP_OK);
    /* det of [[1,2,3],[4,5,6],[7,8,0]] = 1*(0-48) - 2*(0-42) + 3*(32-35)
     *       = -48 + 84 - 9 = 27 */
    ivp_great_matrix_set(&m, 0, 0, IVP_REAL(1.0));
    ivp_great_matrix_set(&m, 0, 1, IVP_REAL(2.0));
    ivp_great_matrix_set(&m, 0, 2, IVP_REAL(3.0));
    ivp_great_matrix_set(&m, 1, 0, IVP_REAL(4.0));
    ivp_great_matrix_set(&m, 1, 1, IVP_REAL(5.0));
    ivp_great_matrix_set(&m, 1, 2, IVP_REAL(6.0));
    ivp_great_matrix_set(&m, 2, 0, IVP_REAL(7.0));
    ivp_great_matrix_set(&m, 2, 1, IVP_REAL(8.0));
    ivp_great_matrix_set(&m, 2, 2, IVP_REAL(0.0));
    ivp_real_t det;
    IVP_ASSERT_EQ(ivp_great_matrix_determinant(&m, &det), IVP_OK);
    IVP_ASSERT_NEAR(det, 27.0, 1e-4);
    ivp_great_matrix_destroy(&m, NULL);
}

/* Identity matrix �?x = b */
IVP_TEST(matrix_identity) {
    ivp_great_matrix_t m;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m, 4, NULL), IVP_OK);
    for (int i = 0; i < 4; i++) {
        ivp_great_matrix_set(&m, i, i, IVP_REAL(1.0));
        ivp_great_matrix_set_rhs(&m, i, (ivp_real_t)(i + 1));
    }
    ivp_real_t x[4];
    IVP_ASSERT_EQ(ivp_great_matrix_solve(&m, x), IVP_OK);
    for (int i = 0; i < 4; i++) {
        IVP_ASSERT_NEAR(x[i], (double)(i + 1), 1e-6);
    }
    ivp_great_matrix_destroy(&m, NULL);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Virtual Mass Tests
 * ════════════════════════════════════════════════════════════════════════════ */

/* Sphere at origin: virtual mass along any direction = mass (r_world = 0) */
IVP_TEST(virtual_mass_centered) {
    ivp_core_t core;
    ivp_vec3r_t inertia = {{IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0)}};
    ivp_core_init(&core, IVP_REAL(5.0), &inertia);

    ivp_vec3r_t r_zero = {{0, 0, 0}};
    ivp_vec3r_t dir = {{IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0)}};

    ivp_real_t vm = ivp_calc_virtual_mass(&core, &r_zero, &dir);
    /* With r=0, angular term vanishes: virt_mass = mass */
    IVP_ASSERT_NEAR(vm, 5.0, 1e-5);
}

/* Contact at surface of sphere */
IVP_TEST(virtual_mass_surface) {
    ivp_core_t core;
    ivp_vec3r_t inertia = {{IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4)}};
    ivp_core_init(&core, IVP_REAL(1.0), &inertia);
    core.upper_limit_radius = IVP_REAL(1.0);

    /* Contact at (1,0,0), normal along (1,0,0) */
    ivp_vec3r_t r = {{IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0)}};
    ivp_vec3r_t n = {{IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0)}};

    ivp_real_t vm = ivp_calc_virtual_mass(&core, &r, &n);
    /* r × n = 0 for collinear r and n, so virt_mass = mass */
    IVP_ASSERT_NEAR(vm, 1.0, 1e-5);
}

/* Unmoveable core �?large virtual mass */
IVP_TEST(virtual_mass_unmoveable) {
    ivp_core_t core;
    ivp_vec3r_t inertia = {{IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0)}};
    ivp_core_init(&core, IVP_REAL(1.0), &inertia);
    core.physical_unmoveable = true;

    ivp_vec3r_t r = {{IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0)}};
    ivp_vec3r_t n = {{IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0)}};

    ivp_real_t vm = ivp_calc_virtual_mass(&core, &r, &n);
    IVP_ASSERT_TRUE(vm > IVP_REAL(1e20));
}

/* ════════════════════════════════════════════════════════════════════════════
 * Contact Geometry Tests
 * ════════════════════════════════════════════════════════════════════════════ */

/* Two spheres separated */
IVP_TEST(contact_sphere_separated) {
    /* Create two objects with a utility environment */
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));

    /* Ball A at (-3, 0, 0) */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &tmpl, NULL);

    /* Ball B at (3, 0, 0) */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tmpl, NULL);

    /* Set radii */
    obj_a->core.upper_limit_radius = IVP_REAL(1.0);
    obj_b->core.upper_limit_radius = IVP_REAL(1.0);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = obj_a;
    cp.obj_b = obj_b;
    ivp_contact_compute_sphere_sphere(&cp);

    /* Distance between centers = 6, radii sum = 2, gap = 4 */
    IVP_ASSERT_NEAR(cp.gap_length, 4.0, 1e-5);
    /* Normal points from A to B: (1, 0, 0) */
    IVP_ASSERT_NEAR(cp.normal.k[0], 1.0, 1e-5);
    IVP_ASSERT_NEAR(cp.normal.k[1], 0.0, 1e-5);
    IVP_ASSERT_NEAR(cp.normal.k[2], 0.0, 1e-5);

    ivp_environment_destroy(env);
}

/* Two spheres penetrating */
IVP_TEST(contact_sphere_penetrating) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));

    /* Overlapping: distance between centers = 1.5, radii sum = 2 */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-0.75), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &tmpl, NULL);

    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.75), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tmpl, NULL);

    obj_a->core.upper_limit_radius = IVP_REAL(1.0);
    obj_b->core.upper_limit_radius = IVP_REAL(1.0);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = obj_a;
    cp.obj_b = obj_b;
    ivp_contact_compute_sphere_sphere(&cp);

    /* gap = 1.5 - 2.0 = -0.5 (penetrating) */
    IVP_ASSERT_NEAR(cp.gap_length, -0.5, 1e-5);
    IVP_ASSERT_NEAR(cp.normal.k[0], 1.0, 1e-5);

    ivp_environment_destroy(env);
}

IVP_TEST(contact_generic_polygon_ball_keeps_order) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);

    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(surface);
    ivp_surman_polygon_t surman;
    ivp_surman_polygon_init(&surman, surface);

    ivp_object_template_t poly_t;
    ivp_object_template_init(&poly_t);
    poly_t.type = IVP_OBJECT_POLYGON;
    poly_t.mass = IVP_REAL(2.0);
    ivp_vec3r_set(&poly_t.rot_inertia, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_vec3r_set(&poly_t.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *poly = ivp_environment_create_object(env, &poly_t, &surman.base);
    IVP_ASSERT_NOT_NULL(poly);

    ivp_object_template_t ball_t;
    ivp_object_template_init(&ball_t);
    ball_t.type = IVP_OBJECT_BALL;
    ball_t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&ball_t.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
    ivp_vec3r_set(&ball_t.position, IVP_REAL(1.6), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ball = ivp_environment_create_object(env, &ball_t, NULL);
    IVP_ASSERT_NOT_NULL(ball);
    ball->core.upper_limit_radius = IVP_REAL(0.5);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = poly;
    cp.obj_b = ball;

    ivp_contact_compute_generic(&cp);

    /* Generic dispatch must preserve caller ordering. */
    IVP_ASSERT_TRUE(cp.obj_a == poly);
    IVP_ASSERT_TRUE(cp.obj_b == ball);
    IVP_ASSERT_TRUE(cp.virtual_mass > IVP_REAL(0.0));

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(contact_generic_ball_polygon_keeps_order) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);

    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(surface);
    ivp_surman_polygon_t surman;
    ivp_surman_polygon_init(&surman, surface);

    ivp_object_template_t ball_t;
    ivp_object_template_init(&ball_t);
    ball_t.type = IVP_OBJECT_BALL;
    ball_t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&ball_t.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
    ivp_vec3r_set(&ball_t.position, IVP_REAL(1.6), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ball = ivp_environment_create_object(env, &ball_t, NULL);
    IVP_ASSERT_NOT_NULL(ball);
    ball->core.upper_limit_radius = IVP_REAL(0.5);

    ivp_object_template_t poly_t;
    ivp_object_template_init(&poly_t);
    poly_t.type = IVP_OBJECT_POLYGON;
    poly_t.mass = IVP_REAL(2.0);
    ivp_vec3r_set(&poly_t.rot_inertia, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_vec3r_set(&poly_t.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *poly = ivp_environment_create_object(env, &poly_t, &surman.base);
    IVP_ASSERT_NOT_NULL(poly);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = ball;
    cp.obj_b = poly;

    ivp_contact_compute_generic(&cp);

    IVP_ASSERT_TRUE(cp.obj_a == ball);
    IVP_ASSERT_TRUE(cp.obj_b == poly);
    IVP_ASSERT_TRUE(cp.virtual_mass > IVP_REAL(0.0));

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(contact_generic_polygon_ball_cross_consistent) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);

    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(surface);
    ivp_surman_polygon_t surman;
    ivp_surman_polygon_init(&surman, surface);

    ivp_object_template_t poly_t;
    ivp_object_template_init(&poly_t);
    poly_t.type = IVP_OBJECT_POLYGON;
    poly_t.mass = IVP_REAL(2.0);
    ivp_vec3r_set(&poly_t.rot_inertia, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_vec3r_set(&poly_t.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *poly = ivp_environment_create_object(env, &poly_t, &surman.base);
    IVP_ASSERT_NOT_NULL(poly);

    ivp_object_template_t ball_t;
    ivp_object_template_init(&ball_t);
    ball_t.type = IVP_OBJECT_BALL;
    ball_t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&ball_t.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
    ivp_vec3r_set(&ball_t.position, IVP_REAL(1.6), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ball = ivp_environment_create_object(env, &ball_t, NULL);
    IVP_ASSERT_NOT_NULL(ball);
    ball->core.upper_limit_radius = IVP_REAL(0.5);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = poly;
    cp.obj_b = ball;
    ivp_contact_compute_generic(&cp);

    ivp_vec3r_t expected_r_a, expected_r_b;
    ivp_vec3r_sub(&expected_r_a, &cp.point_a, &poly->core.pos_world_f_core_last_psi);
    ivp_vec3r_sub(&expected_r_b, &cp.point_b, &ball->core.pos_world_f_core_last_psi);

    IVP_ASSERT_NEAR(cp.r_a.k[0], expected_r_a.k[0], 1e-5);
    IVP_ASSERT_NEAR(cp.r_a.k[1], expected_r_a.k[1], 1e-5);
    IVP_ASSERT_NEAR(cp.r_a.k[2], expected_r_a.k[2], 1e-5);
    IVP_ASSERT_NEAR(cp.r_b.k[0], expected_r_b.k[0], 1e-5);
    IVP_ASSERT_NEAR(cp.r_b.k[1], expected_r_b.k[1], 1e-5);
    IVP_ASSERT_NEAR(cp.r_b.k[2], expected_r_b.k[2], 1e-5);

    ivp_vec3r_t r_a_cs, n_a_cs, cross_a_expected;
    ivp_mat3r_vimult(&r_a_cs, &poly->core.m_world_f_core_last_psi.rot, &cp.r_a);
    ivp_mat3r_vimult(&n_a_cs, &poly->core.m_world_f_core_last_psi.rot, &cp.normal);
    ivp_vec3r_cross(&cross_a_expected, &r_a_cs, &n_a_cs);

    ivp_vec3r_t r_b_cs, n_b_cs, cross_b_expected;
    ivp_mat3r_vimult(&r_b_cs, &ball->core.m_world_f_core_last_psi.rot, &cp.r_b);
    ivp_mat3r_vimult(&n_b_cs, &ball->core.m_world_f_core_last_psi.rot, &cp.normal);
    ivp_vec3r_cross(&cross_b_expected, &r_b_cs, &n_b_cs);

    IVP_ASSERT_NEAR(cp.cross_normal_cs_a.k[0], cross_a_expected.k[0], 1e-5);
    IVP_ASSERT_NEAR(cp.cross_normal_cs_a.k[1], cross_a_expected.k[1], 1e-5);
    IVP_ASSERT_NEAR(cp.cross_normal_cs_a.k[2], cross_a_expected.k[2], 1e-5);
    IVP_ASSERT_NEAR(cp.cross_normal_cs_b.k[0], cross_b_expected.k[0], 1e-5);
    IVP_ASSERT_NEAR(cp.cross_normal_cs_b.k[1], cross_b_expected.k[1], 1e-5);
    IVP_ASSERT_NEAR(cp.cross_normal_cs_b.k[2], cross_b_expected.k[2], 1e-5);

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(contact_generic_poly_poly_manifold_up_to_4) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);

    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(surface);

    ivp_surman_polygon_t surman_a;
    ivp_surman_polygon_t surman_b;
    ivp_surman_polygon_init(&surman_a, surface);
    ivp_surman_polygon_init(&surman_b, surface);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.8), IVP_REAL(0.8), IVP_REAL(0.8));
    tmpl.material.friction = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.2);

    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, &surman_a.base);
    IVP_ASSERT_NOT_NULL(a);

    /* Pure face-face overlap to deterministically exercise manifold emission. */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-1.7), IVP_REAL(0.0));
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, &surman_b.base);
    IVP_ASSERT_NOT_NULL(b);

    ivp_contact_point_t contacts[8];
    int n = ivp_contact_compute_generic_multi(a, b, contacts, 8);

    IVP_ASSERT_TRUE(n >= 1);
    IVP_ASSERT_TRUE(n <= 4);

    bool found_distinct_pair = false;
    for (int i = 0; i < n; i++) {
        IVP_ASSERT_TRUE(contacts[i].obj_a == a);
        IVP_ASSERT_TRUE(contacts[i].obj_b == b);
        IVP_ASSERT_TRUE(contacts[i].virtual_mass > IVP_REAL(0.0));
        IVP_ASSERT_TRUE(contacts[i].gap_length < IVP_REAL(0.2));

        for (int j = i + 1; j < n; j++) {
            if (ivp_vec3r_quad_distance(&contacts[i].point_a, &contacts[j].point_a) > IVP_REAL(1e-4)) {
                found_distinct_pair = true;
            }
        }
    }

    if (n > 1) {
        IVP_ASSERT_TRUE(found_distinct_pair);
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, surface);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Material Combination Tests
 * ════════════════════════════════════════════════════════════════════════════ */

typedef struct solver_material_probe_ctx {
    int                 calls;
    ivp_vec3r_t         point;
    ivp_vec3r_t         normal;
    const ivp_object_t *obj_a;
    const ivp_object_t *obj_b;
} solver_material_probe_ctx_t;

static void solver_material_probe_cb(void *ctx,
                                     const ivp_contact_situation_t *situation,
                                     ivp_real_t *out_friction,
                                     ivp_real_t *out_elasticity,
                                     ivp_real_t *out_adhesion)
{
    solver_material_probe_ctx_t *probe = (solver_material_probe_ctx_t *)ctx;
    probe->calls++;
    probe->point = situation->contact_point_ws;
    probe->normal = situation->contact_normal_ws;
    probe->obj_a = situation->obj_a;
    probe->obj_b = situation->obj_b;

    *out_friction = IVP_REAL(0.77);
    *out_elasticity = IVP_REAL(0.33);
    *out_adhesion = IVP_REAL(0.11);
}

IVP_TEST(material_combination) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));

    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.5);
    tmpl.material.adhesion   = IVP_REAL(0.1);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &tmpl, NULL);

    tmpl.material.friction   = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.4);
    tmpl.material.adhesion   = IVP_REAL(0.2);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tmpl, NULL);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = obj_a;
    cp.obj_b = obj_b;
    ivp_contact_combine_materials(&cp);

    /* friction: 0.8 * 0.6 = 0.48 */
    IVP_ASSERT_NEAR(cp.friction, 0.48, 1e-6);
    /* elasticity: 0.5 * 0.4 = 0.2 */
    IVP_ASSERT_NEAR(cp.elasticity, 0.2, 1e-6);
    /* adhesion: 0.1 + 0.2 = 0.3 */
    IVP_ASSERT_NEAR(cp.adhesion, 0.3, 1e-6);

    ivp_environment_destroy(env);
}

IVP_TEST(material_callback_uses_point_a_contact) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);

    solver_material_probe_ctx_t probe;
    memset(&probe, 0, sizeof(probe));
    ivp_environment_set_material_manager(env, solver_material_probe_cb, &probe);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj_a);
    obj_a->core.upper_limit_radius = IVP_REAL(1.0);

    ivp_vec3r_set(&tmpl.position, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj_b);
    obj_b->core.upper_limit_radius = IVP_REAL(3.0);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = obj_a;
    cp.obj_b = obj_b;
    ivp_contact_compute_sphere_sphere(&cp);
    ivp_contact_combine_materials(&cp);

    IVP_ASSERT_TRUE(probe.calls == 1);
    IVP_ASSERT_TRUE(probe.obj_a == obj_a);
    IVP_ASSERT_TRUE(probe.obj_b == obj_b);
    IVP_ASSERT_NEAR(probe.point.k[0], cp.point_a.k[0], 1e-6);
    IVP_ASSERT_NEAR(probe.point.k[1], cp.point_a.k[1], 1e-6);
    IVP_ASSERT_NEAR(probe.point.k[2], cp.point_a.k[2], 1e-6);
    IVP_ASSERT_NEAR(probe.normal.k[0], cp.normal.k[0], 1e-6);
    IVP_ASSERT_NEAR(probe.normal.k[1], cp.normal.k[1], 1e-6);
    IVP_ASSERT_NEAR(probe.normal.k[2], cp.normal.k[2], 1e-6);
    IVP_ASSERT_NEAR(cp.friction, 0.77, 1e-6);
    IVP_ASSERT_NEAR(cp.elasticity, 0.33, 1e-6);
    IVP_ASSERT_NEAR(cp.adhesion, 0.11, 1e-6);

    ivp_environment_destroy(env);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Impulse Application Tests
 * ════════════════════════════════════════════════════════════════════════════ */

/* Two equal-mass objects: impulse should change velocities symmetrically */
IVP_TEST(impulse_momentum_conservation) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &tmpl, NULL);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tmpl, NULL);

    obj_a->core.upper_limit_radius = IVP_REAL(1.0);
    obj_b->core.upper_limit_radius = IVP_REAL(1.0);

    /* Set initial velocities: A moving +x at 2 m/s, B stationary */
    ivp_vec3r_set(&obj_a->core.speed, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Total momentum before */
    ivp_real_t p_before = obj_a->core.speed.k[0] * obj_a->core.mass +
                          obj_b->core.speed.k[0] * obj_b->core.mass;

    /* Create contact and apply impulse */
    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = obj_a;
    cp.obj_b = obj_b;
    ivp_contact_compute_sphere_sphere(&cp);

    /* Apply 1.0 impulse along normal (x-direction) */
    ivp_contact_apply_impulse(&cp, IVP_REAL(1.0));

    /* Total momentum after (should be conserved) */
    ivp_real_t p_after = obj_a->core.speed.k[0] * obj_a->core.mass +
                         obj_b->core.speed.k[0] * obj_b->core.mass;

    IVP_ASSERT_NEAR(p_before, p_after, 1e-6);

    /* A should decelerate, B should accelerate */
    IVP_ASSERT_TRUE(obj_a->core.speed.k[0] < IVP_REAL(2.0));
    IVP_ASSERT_TRUE(obj_b->core.speed.k[0] > IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Impact Resolution Tests
 * ════════════════════════════════════════════════════════════════════════════ */

/* Elastic collision (e=1): two equal-mass balls, velocity exchange */
IVP_TEST(impact_elastic_exchange) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
    tmpl.material.elasticity = IVP_REAL(1.0);
    tmpl.material.friction = IVP_REAL(0.0);

    /* Ball A at (-0.5, 0, 0) moving +x at 1 m/s */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &tmpl, NULL);
    obj_a->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&obj_a->core.speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Ball B at (0.5, 0, 0) stationary (touching!) */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tmpl, NULL);
    obj_b->core.upper_limit_radius = IVP_REAL(0.5);

    /* Create contact */
    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = obj_a;
    cp.obj_b = obj_b;
    ivp_contact_compute_sphere_sphere(&cp);
    ivp_contact_combine_materials(&cp);
    ivp_contact_compute_virtual_mass(&cp);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);
    ivp_impact_resolve(&cp, &settings);

    /* After elastic collision of equal masses:
     * A should be ~0 m/s, B should be ~1 m/s.
     * Note: the iterative solver may not be perfectly exact. */
    IVP_ASSERT_NEAR(obj_a->core.speed.k[0], 0.0, 0.15);
    IVP_ASSERT_NEAR(obj_b->core.speed.k[0], 1.0, 0.15);

    /* Momentum conservation */
    ivp_real_t p = obj_a->core.speed.k[0] + obj_b->core.speed.k[0];
    IVP_ASSERT_NEAR(p, 1.0, 1e-5);

    ivp_environment_destroy(env);
}

/* Inelastic collision (e=0): objects stick together */
IVP_TEST(impact_inelastic) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.material.friction = IVP_REAL(0.0);

    /* A at (-0.4, 0, 0) moving +x at 2 m/s */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-0.4), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &tmpl, NULL);
    obj_a->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&obj_a->core.speed, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* B at (0.4, 0, 0) stationary */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.4), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tmpl, NULL);
    obj_b->core.upper_limit_radius = IVP_REAL(0.5);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = obj_a;
    cp.obj_b = obj_b;
    ivp_contact_compute_sphere_sphere(&cp);
    ivp_contact_combine_materials(&cp);
    ivp_contact_compute_virtual_mass(&cp);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);
    ivp_impact_resolve(&cp, &settings);

    /* Perfectly inelastic: both should move at v_cm = 2/(1+1) = 1 m/s */
    ivp_real_t p = obj_a->core.speed.k[0] + obj_b->core.speed.k[0];
    IVP_ASSERT_NEAR(p, 2.0, 1e-5); /* momentum conservation */

    /* Both approximately equal (center-of-mass velocity) */
    IVP_ASSERT_NEAR(obj_a->core.speed.k[0], 1.0, 0.25);
    IVP_ASSERT_NEAR(obj_b->core.speed.k[0], 1.0, 0.25);

    ivp_environment_destroy(env);
}

/* Impact with one static object */
IVP_TEST(impact_static_wall) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);

    /* Wall (static) */
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    tmpl.material.elasticity = IVP_REAL(1.0);
    tmpl.material.friction = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *wall = ivp_environment_create_object(env, &tmpl, NULL);
    wall->core.upper_limit_radius = IVP_REAL(0.5);

    /* Ball moving toward wall */
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl, NULL);
    ball->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&ball->core.speed, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = ball;
    cp.obj_b = wall;
    ivp_contact_compute_sphere_sphere(&cp);
    ivp_contact_combine_materials(&cp);
    ivp_contact_compute_virtual_mass(&cp);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);
    ivp_impact_resolve(&cp, &settings);

    /* Ball should bounce back (e=1), wall stays still */
    IVP_ASSERT_TRUE(ball->core.speed.k[0] < IVP_REAL(0.0));
    IVP_ASSERT_NEAR(wall->core.speed.k[0], 0.0, 1e-6);

    ivp_environment_destroy(env);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Friction Tests
 * ════════════════════════════════════════════════════════════════════════════ */

/* Static friction: μ large enough prevents sliding */
IVP_TEST(friction_static_arrests_motion) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
    tmpl.material.friction = IVP_REAL(1.0);  /* high friction */
    tmpl.material.elasticity = IVP_REAL(0.0);

    /* Ground (static) */
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, NULL);
    ground->core.upper_limit_radius = IVP_REAL(0.5);

    /* Object just above ground, sliding in X direction */
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    obj->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&obj->core.speed, IVP_REAL(0.1), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Create contact with normal pointing from obj to ground: (0,1,0) */
    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = obj;
    cp.obj_b = ground;
    ivp_contact_compute_sphere_sphere(&cp);
    ivp_contact_combine_materials(&cp);
    ivp_contact_compute_virtual_mass(&cp);

    /* Apply friction */
    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);
    ivp_real_t dt = (ivp_real_t)IVP_DEFAULT_DELTA_TIME;
    ivp_friction_resolve(&cp, &settings, dt);

    /* Tangential velocity should be reduced (friction opposes motion) */
    IVP_ASSERT_TRUE(ivp_fabs(obj->core.speed.k[0]) < IVP_REAL(0.1));

    ivp_environment_destroy(env);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Two-Ball Scenario (Environment-Level)
 * ════════════════════════════════════════════════════════════════════════════ */

/* Two equal-mass balls head-on collision via environment simulation */
IVP_TEST(two_balls_head_on) {
    /* Create environment with no gravity for clean collision */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
    tmpl.material.elasticity = IVP_REAL(1.0);
    tmpl.material.friction = IVP_REAL(0.0);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Ball A at (-2, 0, 0) moving +x at 1 m/s */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ball_a = ivp_environment_create_object(env, &tmpl, NULL);
    ball_a->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&ball_a->core.speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ball_a->core.delta_world_f_core_psis = ball_a->core.speed;

    /* Ball B at (2, 0, 0) moving -x at 1 m/s */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ball_b = ivp_environment_create_object(env, &tmpl, NULL);
    ball_b->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&ball_b->core.speed, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ball_b->core.delta_world_f_core_psis = ball_b->core.speed;

    /* Simulate until they collide and separate.
     * Approach speed = 2 m/s, distance = 4-1=3m �?time to contact �?1.5s
     * Run 200 PSI steps (~3 seconds) */
    ivp_real_t initial_momentum =
        ball_a->core.speed.k[0] * ball_a->core.mass +
        ball_b->core.speed.k[0] * ball_b->core.mass;

    for (int i = 0; i < 200; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* Check momentum conservation */
    ivp_real_t final_momentum =
        ball_a->core.speed.k[0] * ball_a->core.mass +
        ball_b->core.speed.k[0] * ball_b->core.mass;
    IVP_ASSERT_NEAR(initial_momentum, final_momentum, 0.05);

    /* After elastic head-on collision, velocities should have reversed:
     * A should now be moving in -x, B in +x.
     * May not be perfectly ±1 due to numerical effects. */
    IVP_ASSERT_TRUE(ball_a->core.speed.k[0] < IVP_REAL(0.0));
    IVP_ASSERT_TRUE(ball_b->core.speed.k[0] > IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

/* Two balls with different masses */
IVP_TEST(two_balls_different_mass) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.material.elasticity = IVP_REAL(1.0);
    tmpl.material.friction = IVP_REAL(0.0);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Heavy ball (mass=3) at -2, moving +x at 1 */
    tmpl.mass = IVP_REAL(3.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1.2), IVP_REAL(1.2), IVP_REAL(1.2));
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *heavy = ivp_environment_create_object(env, &tmpl, NULL);
    heavy->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&heavy->core.speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    heavy->core.delta_world_f_core_psis = heavy->core.speed;

    /* Light ball (mass=1) at +2, stationary */
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
    ivp_vec3r_set(&tmpl.position, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *light = ivp_environment_create_object(env, &tmpl, NULL);
    light->core.upper_limit_radius = IVP_REAL(0.5);
    light->core.delta_world_f_core_psis = light->core.speed;

    ivp_real_t p0 = heavy->core.speed.k[0] * heavy->core.mass +
                    light->core.speed.k[0] * light->core.mass;

    for (int i = 0; i < 300; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    ivp_real_t p1 = heavy->core.speed.k[0] * heavy->core.mass +
                    light->core.speed.k[0] * light->core.mass;

    /* Momentum conservation */
    IVP_ASSERT_NEAR(p0, p1, 0.1);

    /* Heavy ball should still move forward (slower) */
    IVP_ASSERT_TRUE(heavy->core.speed.k[0] > IVP_REAL(0.0));

    /* Light ball should be pushed forward faster */
    IVP_ASSERT_TRUE(light->core.speed.k[0] > heavy->core.speed.k[0]);

    ivp_environment_destroy(env);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Energy Tests
 * ════════════════════════════════════════════════════════════════════════════ */

IVP_TEST(kinetic_energy_basic) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(2.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.8), IVP_REAL(0.8), IVP_REAL(0.8));

    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    ivp_vec3r_set(&obj->core.speed, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* KE = 0.5 * 2 * 9 = 9 */
    ivp_real_t ke = ivp_object_get_kinetic_energy(obj);
    IVP_ASSERT_NEAR(ke, 9.0, 1e-5);

    ivp_environment_destroy(env);
}

IVP_TEST(total_energy_conservation) {
    /* Two-ball elastic collision should conserve kinetic energy */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
    tmpl.material.elasticity = IVP_REAL(1.0);
    tmpl.material.friction = IVP_REAL(0.0);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-1.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, NULL);
    a->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&a->core.speed, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    a->core.delta_world_f_core_psis = a->core.speed;

    ivp_vec3r_set(&tmpl.position, IVP_REAL(1.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, NULL);
    b->core.upper_limit_radius = IVP_REAL(0.5);
    b->core.delta_world_f_core_psis = b->core.speed;

    ivp_real_t ke_before = ivp_environment_get_total_kinetic_energy(env);

    /* Run until collision and separation */
    for (int i = 0; i < 200; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    ivp_real_t ke_after = ivp_environment_get_total_kinetic_energy(env);

    /* Energy should be approximately conserved (elastic collision, no damping) */
    IVP_ASSERT_NEAR(ke_before, ke_after, 0.5);

    ivp_environment_destroy(env);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Broadphase Detection Tests
 * ════════════════════════════════════════════════════════════════════════════ */

IVP_TEST(broadphase_detects_close_pair) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));

    /* Two objects close together */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, NULL);
    a->core.upper_limit_radius = IVP_REAL(0.5);

    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.8), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, NULL);
    b->core.upper_limit_radius = IVP_REAL(0.5);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);
    ivp_contact_point_t contacts[16];
    int n = ivp_broadphase_detect(env, &settings, contacts, 16);

    IVP_ASSERT_TRUE(n > 0);
    IVP_ASSERT_TRUE(contacts[0].obj_a == a || contacts[0].obj_b == a);

    ivp_environment_destroy(env);
}

IVP_TEST(broadphase_ignores_distant_pair) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));

    /* Two objects far apart */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, NULL);
    a->core.upper_limit_radius = IVP_REAL(0.5);

    ivp_vec3r_set(&tmpl.position, IVP_REAL(100.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, NULL);
    b->core.upper_limit_radius = IVP_REAL(0.5);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);
    ivp_contact_point_t contacts[16];
    int n = ivp_broadphase_detect(env, &settings, contacts, 16);

    IVP_ASSERT_EQ(n, 0);

    ivp_environment_destroy(env);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Collision Parameters Tests
 * ════════════════════════════════════════════════════════════════════════════ */

IVP_TEST(collision_settings_defaults) {
    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);

    IVP_ASSERT_NEAR(settings.tolerance, 0.01, 1e-6);
    IVP_ASSERT_NEAR(settings.real_coll_dist, 0.001, 1e-6);
    IVP_ASSERT_NEAR(settings.friction_dist, 0.02, 1e-6);
    IVP_ASSERT_TRUE(settings.keeper_dist > settings.friction_dist);
    IVP_ASSERT_TRUE(settings.max_distance_for_impact > settings.keeper_dist);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Static Object Tests
 * ════════════════════════════════════════════════════════════════════════════ */

IVP_TEST(static_object_not_moved) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.physical_unmoveable = true;

    ivp_vec3r_set(&tmpl.position, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *wall = ivp_environment_create_object(env, &tmpl, NULL);
    wall->core.upper_limit_radius = IVP_REAL(0.5);

    /* Ball hitting wall */
    tmpl.physical_unmoveable = false;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
    tmpl.material.elasticity = IVP_REAL(1.0);
    tmpl.material.friction = IVP_REAL(0.0);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl, NULL);
    ball->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&ball->core.speed, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ball->core.delta_world_f_core_psis = ball->core.speed;

    /* Run simulation */
    for (int i = 0; i < 100; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* Wall should not have moved */
    IVP_ASSERT_NEAR(wall->core.pos_world_f_core_last_psi.k[0], 1.0, 1e-6);
    IVP_ASSERT_NEAR(wall->core.speed.k[0], 0.0, 1e-6);

    ivp_environment_destroy(env);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Solver Constants Sanity
 * ════════════════════════════════════════════════════════════════════════════ */

IVP_TEST(solver_constants_values) {
    IVP_ASSERT_NEAR((double)IVP_SOLVER_MATRIX_EPS, 1e-9, 1e-15);
    IVP_ASSERT_NEAR((double)IVP_IMPACT_EPS, 1e-4, 1e-10);
    IVP_ASSERT_EQ(IVP_MAX_GREAT_MATRIX_SIZE, 2048);
    IVP_ASSERT_EQ(IVP_MAXIMUM_SYSTEM_PUSH_NUMBER, 5000);
    IVP_ASSERT_EQ(IVP_MAX_FRICTION_MATRIX, 150);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Closing Speed Tests
 * ════════════════════════════════════════════════════════════════════════════ */

IVP_TEST(closing_speed_approaching) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, NULL);
    a->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&a->core.speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_set(&tmpl.position, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, NULL);
    b->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&b->core.speed, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = a;
    cp.obj_b = b;
    ivp_contact_compute_sphere_sphere(&cp);

    ivp_real_t cs = ivp_contact_closing_speed(&cp);
    /* Objects approaching at 2 m/s relative */
    IVP_ASSERT_NEAR(cs, 2.0, 1e-5);

    ivp_environment_destroy(env);
}

IVP_TEST(closing_speed_separating) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, NULL);
    a->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&a->core.speed, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_set(&tmpl.position, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, NULL);
    b->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&b->core.speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = a;
    cp.obj_b = b;
    ivp_contact_compute_sphere_sphere(&cp);

    ivp_real_t cs = ivp_contact_closing_speed(&cp);
    /* Objects separating at -2 m/s relative */
    IVP_ASSERT_TRUE(cs < IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Test Registration and Main
 * ════════════════════════════════════════════════════════════════════════════ */

int main(void) {
    /* Matrix solver */
    IVP_REGISTER(matrix_1x1);
    IVP_REGISTER(matrix_2x2);
    IVP_REGISTER(matrix_3x3_pivot);
    IVP_REGISTER(matrix_singular);
    IVP_REGISTER(matrix_determinant);
    IVP_REGISTER(matrix_identity);

    /* Virtual mass */
    IVP_REGISTER(virtual_mass_centered);
    IVP_REGISTER(virtual_mass_surface);
    IVP_REGISTER(virtual_mass_unmoveable);

    /* Contact geometry */
    IVP_REGISTER(contact_sphere_separated);
    IVP_REGISTER(contact_sphere_penetrating);
    IVP_REGISTER(contact_generic_polygon_ball_keeps_order);
    IVP_REGISTER(contact_generic_ball_polygon_keeps_order);
    IVP_REGISTER(contact_generic_polygon_ball_cross_consistent);
    IVP_REGISTER(contact_generic_poly_poly_manifold_up_to_4);

    /* Material */
    IVP_REGISTER(material_combination);
    IVP_REGISTER(material_callback_uses_point_a_contact);

    /* Impulse */
    IVP_REGISTER(impulse_momentum_conservation);

    /* Impact */
    IVP_REGISTER(impact_elastic_exchange);
    IVP_REGISTER(impact_inelastic);
    IVP_REGISTER(impact_static_wall);

    /* Friction */
    IVP_REGISTER(friction_static_arrests_motion);

    /* Two-ball scenarios */
    IVP_REGISTER(two_balls_head_on);
    IVP_REGISTER(two_balls_different_mass);

    /* Energy */
    IVP_REGISTER(kinetic_energy_basic);
    IVP_REGISTER(total_energy_conservation);

    /* Broadphase */
    IVP_REGISTER(broadphase_detects_close_pair);
    IVP_REGISTER(broadphase_ignores_distant_pair);

    /* Collision settings */
    IVP_REGISTER(collision_settings_defaults);

    /* Static objects */
    IVP_REGISTER(static_object_not_moved);

    /* Constants */
    IVP_REGISTER(solver_constants_values);

    /* Closing speed */
    IVP_REGISTER(closing_speed_approaching);
    IVP_REGISTER(closing_speed_separating);

    IVP_RUN_ALL();
}
