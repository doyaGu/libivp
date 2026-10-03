/* test_polygon_collision_debug.c — Tests for polygon-polygon collision. */
#include "ivp_test.h"
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <math.h>

static ivp_environment_t *make_env(void) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    return ivp_environment_create(&cfg);
}

static void step(ivp_environment_t *env, int n) {
    for (int i = 0; i < n; i++) {
        ivp_environment_simulate_one_psi(env);
    }
}

/* ── Polygon box falls onto static polygon ground ────────────────────── */

IVP_TEST(polygon_box_falls_on_ground) {
    ivp_environment_t *env = make_env();

    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 10.0f, 0.5f, 10.0f);
    ivp_surman_polygon_t sm_ground;
    ivp_surman_polygon_init(&sm_ground, ground_surf);

    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.type = IVP_OBJECT_POLYGON;
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    ivp_environment_create_object(env, &gt, &sm_ground.base);

    ivp_compact_surface_t *box_surf = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    ivp_surman_polygon_t sm_box;
    ivp_surman_polygon_init(&sm_box, box_surf);

    ivp_object_template_t bt;
    ivp_object_template_init(&bt);
    bt.type = IVP_OBJECT_POLYGON;
    bt.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&bt.rot_inertia, IVP_REAL(0.083), IVP_REAL(0.083), IVP_REAL(0.083));
    bt.rot_inertia_is_factor = false; /* absolute inertia */
    bt.material.friction   = IVP_REAL(0.5);
    bt.material.elasticity = IVP_REAL(0.3);
    ivp_vec3r_set(&bt.position, IVP_REAL(0.0), IVP_REAL(-3.0), IVP_REAL(0.0));

    ivp_object_t *box = ivp_environment_create_object(env, &bt, &sm_box.base);
    IVP_ASSERT_NOT_NULL(box);

    step(env, 120);

    ivp_vec3r_t final_pos;
    ivp_object_get_position(box, &final_pos);

    /* Box should rest on the ground, not fall through.
     * Ground top at y=-0.5, box half-size 0.5 → expected y ≈ -1.0.
     * Allow generous tolerance since the box may still be slightly bouncing. */
    IVP_ASSERT_TRUE(final_pos.k[1] < IVP_REAL(0.0));

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, box_surf);
    ivp_compact_surface_free(NULL, ground_surf);
}

/* ── Two dynamic polygon boxes collide ───────────────────────────────── */

IVP_TEST(polygon_box_hits_static_box) {
    ivp_environment_t *env = make_env();

    ivp_compact_surface_t *box_surf = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);

    /* Static box at origin */
    ivp_surman_polygon_t sm_b;
    ivp_surman_polygon_init(&sm_b, box_surf);

    ivp_object_template_t tb;
    ivp_object_template_init(&tb);
    tb.type = IVP_OBJECT_POLYGON;
    tb.mass = 0.0f;
    tb.physical_unmoveable = true;

    ivp_environment_create_object(env, &tb, &sm_b.base);

    /* Falling box */
    ivp_surman_polygon_t sm_a;
    ivp_surman_polygon_init(&sm_a, box_surf);

    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.type = IVP_OBJECT_POLYGON;
    ta.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&ta.rot_inertia, IVP_REAL(0.083), IVP_REAL(0.083), IVP_REAL(0.083));
    ta.rot_inertia_is_factor = false; /* absolute inertia */
    ta.material.friction   = IVP_REAL(0.5);
    ta.material.elasticity = IVP_REAL(0.2);
    ivp_vec3r_set(&ta.position, IVP_REAL(0.0), IVP_REAL(-3.0), IVP_REAL(0.0));

    ivp_object_t *box_a = ivp_environment_create_object(env, &ta, &sm_a.base);
    IVP_ASSERT_NOT_NULL(box_a);

    step(env, 240);

    ivp_vec3r_t final_pos;
    ivp_object_get_position(box_a, &final_pos);

    /* Box A should stop above box B, not fall through.
     * Box B top at y=-0.5, box A half-size 0.5 → expected y ≈ -1.0. */
    IVP_ASSERT_TRUE(final_pos.k[1] < IVP_REAL(0.0));

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, box_surf);
}

/* ── Ball vs polygon ground comparison ───────────────────────────────── */

IVP_TEST(ball_falls_on_polygon_ground) {
    ivp_environment_t *env = make_env();

    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 10.0f, 0.5f, 10.0f);
    ivp_surman_polygon_t sm_ground;
    ivp_surman_polygon_init(&sm_ground, ground_surf);

    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.type = IVP_OBJECT_POLYGON;
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    ivp_environment_create_object(env, &gt, &sm_ground.base);

    ivp_surman_ball_t sm_ball;
    ivp_surman_ball_init(&sm_ball, 0.5f);

    ivp_object_template_t bt;
    ivp_object_template_init(&bt);
    bt.type = IVP_OBJECT_BALL;
    bt.mass = IVP_REAL(1.0);
    ivp_real_t I = IVP_REAL(0.4) * IVP_REAL(1.0) * IVP_REAL(0.25);
    ivp_vec3r_set(&bt.rot_inertia, I, I, I);
    bt.rot_inertia_is_factor = false; /* absolute inertia */
    bt.material.friction   = IVP_REAL(0.5);
    bt.material.elasticity = IVP_REAL(0.3);
    ivp_vec3r_set(&bt.position, IVP_REAL(0.0), IVP_REAL(-3.0), IVP_REAL(0.0));

    ivp_object_t *ball = ivp_environment_create_object(env, &bt, &sm_ball.base);
    IVP_ASSERT_NOT_NULL(ball);

    step(env, 240);

    ivp_vec3r_t final_pos;
    ivp_object_get_position(ball, &final_pos);

    /* Ball should rest on ground */
    IVP_ASSERT_TRUE(final_pos.k[1] < IVP_REAL(0.0));

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, ground_surf);
}

int main(void) {
    IVP_REGISTER(polygon_box_falls_on_ground);
    IVP_REGISTER(polygon_box_hits_static_box);
    IVP_REGISTER(ball_falls_on_polygon_ground);
    IVP_RUN_ALL();
}
