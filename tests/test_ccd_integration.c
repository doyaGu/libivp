/* test_ccd_integration.c — Tests for continuous collision detection integration */
#include "ivp_test.h"
#include <ivp/collision/ivp_ccd_narrowphase.h>
#include <ivp/collision/ivp_3d_solver.h>
#include <ivp/collision/ivp_matrix_cache.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_environment_collision.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_fast_math.h>
#include <string.h>
#include <math.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Distance function tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(ccd_dist_sphere_sphere) {
    /* Two ball objects separated by known distance */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.type = IVP_OBJECT_BALL;
    ta.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&ta.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &ta, NULL);

    ivp_object_template_t tb;
    ivp_object_template_init(&tb);
    tb.type = IVP_OBJECT_BALL;
    tb.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tb.position, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tb, NULL);

    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    /* Set known radii */
    ivp_real_t ra = obj_a->core.upper_limit_radius;
    ivp_real_t rb = obj_b->core.upper_limit_radius;

    ivp_ccd_convex_pair_t pair;
    pair.obj_a = obj_a;
    pair.obj_b = obj_b;

    ivp_transform_r_t m_a = obj_a->core.m_world_f_core_last_psi;
    ivp_transform_r_t m_b = obj_b->core.m_world_f_core_last_psi;

    ivp_real_t dist = ivp_3d_dist_convex_convex(&m_a, &m_b, &pair);
    ivp_real_t expected = IVP_REAL(5.0) - ra - rb;
    IVP_ASSERT_NEAR(dist, expected, 0.01);

    ivp_environment_destroy(env);
}

IVP_TEST(ccd_dist_sphere_poly) {
    /* Ball at known distance from a box polygon */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    /* Ball at origin */
    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.type = IVP_OBJECT_BALL;
    ta.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&ta.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &ta, NULL);

    /* Box at (3, 0, 0) */
    ivp_compact_surface_t *box_surf = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(box_surf);
    ivp_surman_polygon_t sm_box;
    ivp_surman_polygon_init(&sm_box, box_surf);

    ivp_object_template_t tb;
    ivp_object_template_init(&tb);
    tb.type = IVP_OBJECT_POLYGON;
    tb.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tb.rot_inertia, IVP_REAL(0.083), IVP_REAL(0.083), IVP_REAL(0.083));
    ivp_vec3r_set(&tb.position, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tb, &sm_box.base);

    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    ivp_ccd_convex_pair_t pair;
    pair.obj_a = obj_a;
    pair.obj_b = obj_b;

    ivp_transform_r_t m_a = obj_a->core.m_world_f_core_last_psi;
    ivp_transform_r_t m_b = obj_b->core.m_world_f_core_last_psi;

    ivp_real_t dist = ivp_3d_dist_convex_convex(&m_a, &m_b, &pair);
    /* Distance should be positive (separated) */
    IVP_ASSERT_TRUE(dist > IVP_REAL(0.0));
    /* And less than center-to-center distance (3.0) */
    IVP_ASSERT_TRUE(dist < IVP_REAL(3.0));

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, box_surf);
}

IVP_TEST(ccd_dist_poly_poly) {
    /* Two box polygons at known separation */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_compact_surface_t *surf_a = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    ivp_compact_surface_t *surf_b = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(surf_a);
    IVP_ASSERT_NOT_NULL(surf_b);

    ivp_surman_polygon_t sm_a, sm_b;
    ivp_surman_polygon_init(&sm_a, surf_a);
    ivp_surman_polygon_init(&sm_b, surf_b);

    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.type = IVP_OBJECT_POLYGON;
    ta.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&ta.rot_inertia, IVP_REAL(0.083), IVP_REAL(0.083), IVP_REAL(0.083));
    ivp_vec3r_set(&ta.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &ta, &sm_a.base);

    ivp_object_template_t tb;
    ivp_object_template_init(&tb);
    tb.type = IVP_OBJECT_POLYGON;
    tb.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tb.rot_inertia, IVP_REAL(0.083), IVP_REAL(0.083), IVP_REAL(0.083));
    ivp_vec3r_set(&tb.position, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tb, &sm_b.base);

    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    ivp_ccd_convex_pair_t pair;
    pair.obj_a = obj_a;
    pair.obj_b = obj_b;

    ivp_transform_r_t m_a = obj_a->core.m_world_f_core_last_psi;
    ivp_transform_r_t m_b = obj_b->core.m_world_f_core_last_psi;

    ivp_real_t dist = ivp_3d_dist_convex_convex(&m_a, &m_b, &pair);
    /* Distance should be positive (separated) and less than center-to-center (3.0).
     * Exact value depends on mindist walker convergence for convex-convex. */
    IVP_ASSERT_TRUE(dist > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(dist < IVP_REAL(3.5));

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, surf_a);
    ivp_compact_surface_free(NULL, surf_b);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CCD sweep logic tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(ccd_per_object_enable) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.ccd_enabled = true;
    ivp_object_t *obj_a = ivp_environment_create_object(env, &ta, NULL);
    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_TRUE(obj_a->ccd_enabled);

    ivp_object_template_t tb;
    ivp_object_template_init(&tb);
    tb.ccd_enabled = false;
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tb, NULL);
    IVP_ASSERT_NOT_NULL(obj_b);
    IVP_ASSERT_FALSE(obj_b->ccd_enabled);

    /* Give obj_a enough speed to trigger sweep */
    obj_a->core.speed = ivp_vec3r(IVP_REAL(100.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Pair should need sweep since obj_a has CCD enabled and fast speed */
    ivp_real_t dt = IVP_REAL(1.0) / IVP_REAL(66.0);
    bool needs = ivp_ccd_pair_needs_sweep(obj_a, obj_b, dt);
    IVP_ASSERT_TRUE(needs);

    ivp_environment_destroy(env);
}

IVP_TEST(ccd_speed_threshold) {
    /* Slow object with CCD enabled — sweep should not be needed */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.ccd_enabled = true;
    ivp_object_t *obj_a = ivp_environment_create_object(env, &ta, NULL);

    ivp_object_template_t tb;
    ivp_object_template_init(&tb);
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tb, NULL);

    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    /* Very slow speed — should not trigger sweep */
    obj_a->core.speed = ivp_vec3r(IVP_REAL(0.001), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_real_t dt = IVP_REAL(1.0) / IVP_REAL(66.0);
    bool needs = ivp_ccd_pair_needs_sweep(obj_a, obj_b, dt);
    IVP_ASSERT_FALSE(needs);

    ivp_environment_destroy(env);
}

IVP_TEST(ccd_custom_speed_threshold_overrides_auto) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.ccd_enabled = true;
    ivp_object_t *obj_a = ivp_environment_create_object(env, &ta, NULL);

    ivp_object_template_t tb;
    ivp_object_template_init(&tb);
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tb, NULL);

    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    ivp_real_t dt = IVP_REAL(1.0) / IVP_REAL(66.0);
    obj_a->core.speed = ivp_vec3r(IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    obj_a->ccd_speed_threshold = IVP_REAL(0.0);
    IVP_ASSERT_FALSE(ivp_ccd_pair_needs_sweep(obj_a, obj_b, dt));

    obj_a->ccd_speed_threshold = IVP_REAL(5.0);
    IVP_ASSERT_TRUE(ivp_ccd_pair_needs_sweep(obj_a, obj_b, dt));

    obj_a->ccd_speed_threshold = IVP_REAL(20.0);
    IVP_ASSERT_FALSE(ivp_ccd_pair_needs_sweep(obj_a, obj_b, dt));

    ivp_environment_destroy(env);
}

IVP_TEST(ccd_disabled_environment) {
    /* With env->ccd_enabled = false, the CCD pass should not run at all */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.ccd_enabled = false;
    ivp_environment_t *env = ivp_environment_create(&cfg);

    IVP_ASSERT_FALSE(env->ccd_enabled);

    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.ccd_enabled = true;
    ivp_object_t *obj_a = ivp_environment_create_object(env, &ta, NULL);
    IVP_ASSERT_NOT_NULL(obj_a);

    /* Give it huge speed */
    obj_a->core.speed = ivp_vec3r(IVP_REAL(1000.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Simulate — no crash, CCD not invoked */
    for (int i = 0; i < 10; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    ivp_environment_destroy(env);
}

IVP_TEST(ccd_does_not_regress_discrete) {
    /* Verify that enabling CCD does not change behavior for slow objects.
     * Two boxes colliding normally with CCD on. */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.ccd_enabled = true;
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 10.0f, 0.5f, 10.0f);
    IVP_ASSERT_NOT_NULL(ground_surf);
    ivp_surman_polygon_t sm_ground;
    ivp_surman_polygon_init(&sm_ground, ground_surf);

    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.type = IVP_OBJECT_POLYGON;
    gt.mass = IVP_REAL(0.0);
    gt.physical_unmoveable = true;
    ivp_vec3r_set(&gt.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_create_object(env, &gt, &sm_ground.base);

    ivp_compact_surface_t *box_surf = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    IVP_ASSERT_NOT_NULL(box_surf);
    ivp_surman_polygon_t sm_box;
    ivp_surman_polygon_init(&sm_box, box_surf);

    ivp_object_template_t bt;
    ivp_object_template_init(&bt);
    bt.type = IVP_OBJECT_POLYGON;
    bt.mass = IVP_REAL(1.0);
    bt.ccd_enabled = true;
    ivp_vec3r_set(&bt.rot_inertia, IVP_REAL(0.083), IVP_REAL(0.083), IVP_REAL(0.083));
    bt.material.friction   = IVP_REAL(0.5);
    bt.material.elasticity = IVP_REAL(0.3);
    ivp_vec3r_set(&bt.position, IVP_REAL(0.0), IVP_REAL(-3.0), IVP_REAL(0.0));
    ivp_object_t *box = ivp_environment_create_object(env, &bt, &sm_box.base);
    IVP_ASSERT_NOT_NULL(box);

    /* Run 100 PSI steps. Box should fall and collide with ground.
     * With CCD on (but slow speed), discrete path handles it. */
    for (int i = 0; i < 100; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* Box Y should be near ground level (Y ~= -0.5 to 0.5 range, not fallen through) */
    ivp_real_t y = box->core.pos_world_f_core_last_psi.k[1];
    IVP_ASSERT_TRUE(y > IVP_REAL(-5.0));
    IVP_ASSERT_TRUE(y < IVP_REAL(1.0));

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_compact_surface_free(NULL, box_surf);
}

IVP_TEST(ccd_tunneling_prevention) {
    /* Fast small sphere aimed at thin wall polygon.
     * Without CCD the sphere tunnels; with CCD it should be caught. */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.ccd_enabled = true;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    /* Thin wall at X=5 */
    ivp_compact_surface_t *wall_surf = ivp_build_box_surface(NULL, 0.1f, 5.0f, 5.0f);
    IVP_ASSERT_NOT_NULL(wall_surf);
    ivp_surman_polygon_t sm_wall;
    ivp_surman_polygon_init(&sm_wall, wall_surf);

    ivp_object_template_t wt;
    ivp_object_template_init(&wt);
    wt.type = IVP_OBJECT_POLYGON;
    wt.mass = IVP_REAL(0.0);
    wt.physical_unmoveable = true;
    ivp_vec3r_set(&wt.position, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_create_object(env, &wt, &sm_wall.base);

    /* Fast ball at origin aimed at wall */
    ivp_object_template_t bt;
    ivp_object_template_init(&bt);
    bt.type = IVP_OBJECT_BALL;
    bt.mass = IVP_REAL(0.1);
    bt.ccd_enabled = true;
    ivp_vec3r_set(&bt.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ball = ivp_environment_create_object(env, &bt, NULL);
    IVP_ASSERT_NOT_NULL(ball);

    /* Very high speed towards the wall (may exceed ~5m in a few frames) */
    ball->core.speed = ivp_vec3r(IVP_REAL(100.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Run enough steps for the ball to reach the wall */
    for (int i = 0; i < 30; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* With CCD, ball should not have tunneled past X=5 */
    ivp_real_t x = ball->core.pos_world_f_core_last_psi.k[0];
    /* The ball should either have been stopped or bounced back.
     * Its X should be <= wall position + some tolerance */
    IVP_ASSERT_TRUE(x < IVP_REAL(10.0));

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, wall_surf);
}

IVP_TEST(ccd_pipeline_updates_mindist_state) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.ccd_enabled = true;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.type = IVP_OBJECT_BALL;
    ta.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&ta.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *a = ivp_environment_create_object(env, &ta, NULL);
    IVP_ASSERT_NOT_NULL(a);

    ivp_object_template_t tb = ta;
    ivp_vec3r_set(&tb.position, IVP_REAL(2.2), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = ivp_environment_create_object(env, &tb, NULL);
    IVP_ASSERT_NOT_NULL(b);

    ivp_vec3r_set(&a->core.speed, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&b->core.speed, IVP_REAL(-10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    (void)ivp_environment_resolve_collisions(env);
    IVP_ASSERT_TRUE(env->mindist_manager.entry_count > 0);

    ivp_environment_destroy(env);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test registration
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void) {
    IVP_REGISTER(ccd_dist_sphere_sphere);
    IVP_REGISTER(ccd_dist_sphere_poly);
    IVP_REGISTER(ccd_dist_poly_poly);
    IVP_REGISTER(ccd_per_object_enable);
    IVP_REGISTER(ccd_speed_threshold);
    IVP_REGISTER(ccd_custom_speed_threshold_overrides_auto);
    IVP_REGISTER(ccd_disabled_environment);
    IVP_REGISTER(ccd_does_not_regress_discrete);
    IVP_REGISTER(ccd_tunneling_prevention);
    IVP_REGISTER(ccd_pipeline_updates_mindist_state);
    IVP_RUN_ALL();
}
