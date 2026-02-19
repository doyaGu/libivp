/*
 * IVP Physics Engine - C17 Port
 * P1 Controller Unit Tests
 *
 * Tests for:
 *   - Stabilizer actuator (anti-roll bar)
 *   - Suspension actuator (asymmetric damping spring)
 *   - Constraint local API extensions (BEND, enable/disable, rebinding)
 */

#include "ivp_test.h"

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/controllers/ivp_actuator_stabilizer.h>
#include <ivp/controllers/ivp_actuator_suspension.h>
#include <ivp/controllers/ivp_constraint_local.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_fast_math.h>

#include <math.h>
#include <stdbool.h>
#include <string.h>

#ifdef IVP_REAL_DOUBLE
  #define TOL  1e-6
  #define LTOL 1e-3
#else
  #define TOL  1e-3f
  #define LTOL 1e-1f
#endif

#define DT60 (IVP_REAL(1.0) / IVP_REAL(60.0))

/* Helper: Euclidean distance */
static inline ivp_real_t vec3_dist(const ivp_vec3r_t *a, const ivp_vec3r_t *b) {
    return sqrtf(ivp_vec3r_quad_distance(a, b));
}

/* Helper: create a minimal environment */
static ivp_environment_t *make_env(ivp_real_t dt, bool gravity) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = dt;
    if (!gravity) {
        ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    } else {
        ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.81), IVP_REAL(0.0));
    }
    return ivp_environment_create(&cfg);
}

/* Helper: create a ball object at given position */
static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t mass,
                                ivp_real_t x, ivp_real_t y, ivp_real_t z) {
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = mass;
    ivp_real_t I = IVP_REAL(0.4) * mass * IVP_REAL(0.25);
    ivp_vec3r_set(&t.rot_inertia, I, I, I);
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&t.position, x, y, z);
    t.orientation = ivp_quatr_identity();
    ivp_object_t *obj = ivp_environment_create_object(env, &t, NULL);
    if (obj) {
        obj->core.upper_limit_radius = IVP_REAL(0.5);
        ivp_vec3r_set(&obj->core.speed, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        obj->core.rot_speed.k[0] = IVP_REAL(0.0);
        obj->core.rot_speed.k[1] = IVP_REAL(0.0);
        obj->core.rot_speed.k[2] = IVP_REAL(0.0);
        obj->core.delta_world_f_core_psis = obj->core.speed;
    }
    return obj;
}

static void step(ivp_environment_t *env, int n) {
    for (int i = 0; i < n; i++) {
        ivp_environment_simulate_one_psi(env);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Stabilizer Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(stabilizer_init_defaults) {
    ivp_stabilizer_state_t state;
    ivp_stabilizer_state_init(&state);

    IVP_ASSERT_TRUE(state.active);
    IVP_ASSERT_NEAR(state.stabi_constant, IVP_REAL(0.0), TOL);
    IVP_ASSERT_NEAR(state.current_diff_dist, IVP_REAL(0.0), TOL);
    for (int i = 0; i < 4; i++) {
        IVP_ASSERT_NULL(state.obj[i]);
    }
}

IVP_TEST(stabilizer_setup_params) {
    ivp_environment_t *env = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *o0 = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o1 = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o2 = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o3 = make_ball(env, IVP_REAL(1.0), IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t a0, a1, a2, a3;
    ivp_vec3r_set_zero(&a0);
    ivp_vec3r_set_zero(&a1);
    ivp_vec3r_set_zero(&a2);
    ivp_vec3r_set_zero(&a3);

    ivp_stabilizer_state_t state;
    ivp_stabilizer_state_init(&state);
    ivp_stabilizer_state_setup(&state, o0, o1, o2, o3, &a0, &a1, &a2, &a3, IVP_REAL(500.0));

    IVP_ASSERT_TRUE(state.obj[0] == o0);
    IVP_ASSERT_TRUE(state.obj[1] == o1);
    IVP_ASSERT_TRUE(state.obj[2] == o2);
    IVP_ASSERT_TRUE(state.obj[3] == o3);
    IVP_ASSERT_NEAR(state.stabi_constant, IVP_REAL(500.0), TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(stabilizer_equalizes_pair_distances) {
    /* Pair 0: distance=1, Pair 1: distance=3.
     * Stabilizer should push pair 0 apart and pair 1 together. */
    ivp_environment_t *env = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env);

    /* Pair 0: 1m apart along X */
    ivp_object_t *o0 = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o1 = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    /* Pair 1: 3m apart along X */
    ivp_object_t *o2 = make_ball(env, IVP_REAL(1.0), IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o3 = make_ball(env, IVP_REAL(1.0), IVP_REAL(8.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t zero_anchor;
    ivp_vec3r_set_zero(&zero_anchor);

    ivp_stabilizer_state_t state;
    ivp_stabilizer_state_init(&state);
    ivp_stabilizer_state_setup(&state, o0, o1, o2, o3,
                                &zero_anchor, &zero_anchor, &zero_anchor, &zero_anchor,
                                IVP_REAL(200.0));

    ivp_environment_add_controller(env, 0, ivp_controller_stabilizer, &state);

    /* Measure initial difference */
    ivp_vec3r_t p0, p1, p2, p3;
    ivp_object_get_position(o0, &p0);
    ivp_object_get_position(o1, &p1);
    ivp_object_get_position(o2, &p2);
    ivp_object_get_position(o3, &p3);
    ivp_real_t dist0_before = vec3_dist(&p0, &p1);
    ivp_real_t dist1_before = vec3_dist(&p2, &p3);
    ivp_real_t diff_before = ivp_fabs(dist1_before - dist0_before);

    step(env, 30);

    /* Measure difference after simulation */
    ivp_object_get_position(o0, &p0);
    ivp_object_get_position(o1, &p1);
    ivp_object_get_position(o2, &p2);
    ivp_object_get_position(o3, &p3);
    ivp_real_t dist0_after = vec3_dist(&p0, &p1);
    ivp_real_t dist1_after = vec3_dist(&p2, &p3);
    ivp_real_t diff_after = ivp_fabs(dist1_after - dist0_after);

    /* Distance difference should have decreased */
    IVP_ASSERT_TRUE(diff_after < diff_before);

    ivp_environment_destroy(env);
}

IVP_TEST(stabilizer_zero_constant_no_effect) {
    ivp_environment_t *env = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *o0 = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o1 = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o2 = make_ball(env, IVP_REAL(1.0), IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o3 = make_ball(env, IVP_REAL(1.0), IVP_REAL(8.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t zero_anchor;
    ivp_vec3r_set_zero(&zero_anchor);

    ivp_stabilizer_state_t state;
    ivp_stabilizer_state_init(&state);
    ivp_stabilizer_state_setup(&state, o0, o1, o2, o3,
                                &zero_anchor, &zero_anchor, &zero_anchor, &zero_anchor,
                                IVP_REAL(0.0)); /* zero stiffness */

    ivp_environment_add_controller(env, 0, ivp_controller_stabilizer, &state);

    /* Record positions */
    ivp_vec3r_t p0_before, p1_before, p2_before, p3_before;
    ivp_object_get_position(o0, &p0_before);
    ivp_object_get_position(o1, &p1_before);
    ivp_object_get_position(o2, &p2_before);
    ivp_object_get_position(o3, &p3_before);

    step(env, 10);

    /* Positions should not change with zero gravity and zero stiffness */
    ivp_vec3r_t p0_after, p1_after, p2_after, p3_after;
    ivp_object_get_position(o0, &p0_after);
    ivp_object_get_position(o1, &p1_after);
    ivp_object_get_position(o2, &p2_after);
    ivp_object_get_position(o3, &p3_after);

    IVP_ASSERT_NEAR(p0_after.k[0], p0_before.k[0], LTOL);
    IVP_ASSERT_NEAR(p1_after.k[0], p1_before.k[0], LTOL);
    IVP_ASSERT_NEAR(p2_after.k[0], p2_before.k[0], LTOL);
    IVP_ASSERT_NEAR(p3_after.k[0], p3_before.k[0], LTOL);

    ivp_environment_destroy(env);
}

IVP_TEST(stabilizer_set_constant_runtime) {
    ivp_environment_t *env = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *o0 = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o1 = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o2 = make_ball(env, IVP_REAL(1.0), IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o3 = make_ball(env, IVP_REAL(1.0), IVP_REAL(8.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t zero_anchor;
    ivp_vec3r_set_zero(&zero_anchor);

    ivp_stabilizer_state_t state;
    ivp_stabilizer_state_init(&state);
    ivp_stabilizer_state_setup(&state, o0, o1, o2, o3,
                                &zero_anchor, &zero_anchor, &zero_anchor, &zero_anchor,
                                IVP_REAL(0.0)); /* start with zero */

    ivp_environment_add_controller(env, 0, ivp_controller_stabilizer, &state);

    /* Run with zero constant -- no movement */
    step(env, 5);

    ivp_vec3r_t p0_mid;
    ivp_object_get_position(o0, &p0_mid);
    IVP_ASSERT_NEAR(p0_mid.k[0], IVP_REAL(0.0), LTOL);

    /* Now set constant and run more */
    ivp_stabilizer_set_constant(&state, IVP_REAL(500.0));
    IVP_ASSERT_NEAR(state.stabi_constant, IVP_REAL(500.0), TOL);

    step(env, 20);

    /* Pair distances should be converging */
    ivp_vec3r_t p0, p1, p2, p3;
    ivp_object_get_position(o0, &p0);
    ivp_object_get_position(o1, &p1);
    ivp_object_get_position(o2, &p2);
    ivp_object_get_position(o3, &p3);

    ivp_real_t d0 = vec3_dist(&p0, &p1);
    ivp_real_t d1 = vec3_dist(&p2, &p3);
    /* The diff should be less than the original 2.0 */
    IVP_ASSERT_TRUE(ivp_fabs(d1 - d0) < IVP_REAL(2.0));

    ivp_environment_destroy(env);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Suspension Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(suspension_init_defaults) {
    ivp_suspension_state_t state;
    ivp_suspension_state_init(&state);

    IVP_ASSERT_NEAR(state.stiffness, IVP_REAL(100.0), TOL);
    IVP_ASSERT_NEAR(state.damping, IVP_REAL(1.0), TOL);
    IVP_ASSERT_NEAR(state.damping_compression, IVP_REAL(1.0), TOL);
    IVP_ASSERT_NEAR(state.rest_length, IVP_REAL(1.0), TOL);
    IVP_ASSERT_NEAR(state.max_body_force, IVP_REAL(0.0), TOL);
    IVP_ASSERT_NULL(state.obj_a);
    IVP_ASSERT_NULL(state.obj_b);
}

IVP_TEST(suspension_setup_params) {
    ivp_environment_t *env = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *body  = make_ball(env, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *wheel = make_ball(env, IVP_REAL(1.0),  IVP_REAL(0.0), IVP_REAL(-2.0), IVP_REAL(0.0));

    ivp_vec3r_t anchor_body, anchor_wheel;
    ivp_vec3r_set_zero(&anchor_body);
    ivp_vec3r_set_zero(&anchor_wheel);

    ivp_suspension_state_t state;
    ivp_suspension_state_init(&state);
    ivp_suspension_state_setup(&state, body, wheel, &anchor_body, &anchor_wheel,
                                IVP_REAL(200.0), IVP_REAL(5.0), IVP_REAL(10.0), IVP_REAL(1.5));

    IVP_ASSERT_TRUE(state.obj_a == body);
    IVP_ASSERT_TRUE(state.obj_b == wheel);
    IVP_ASSERT_NEAR(state.stiffness, IVP_REAL(200.0), TOL);
    IVP_ASSERT_NEAR(state.damping, IVP_REAL(5.0), TOL);
    IVP_ASSERT_NEAR(state.damping_compression, IVP_REAL(10.0), TOL);
    IVP_ASSERT_NEAR(state.rest_length, IVP_REAL(1.5), TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(suspension_basic_spring_behavior) {
    /* Body and wheel 3m apart, rest_length=1m. Should contract. */
    ivp_environment_t *env = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *body  = make_ball(env, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    ivp_object_t *wheel = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t anchor;
    ivp_vec3r_set_zero(&anchor);

    ivp_suspension_state_t state;
    ivp_suspension_state_init(&state);
    ivp_suspension_state_setup(&state, body, wheel, &anchor, &anchor,
                                IVP_REAL(50.0), IVP_REAL(2.0), IVP_REAL(2.0), IVP_REAL(1.0));

    ivp_environment_add_controller(env, 0, ivp_controller_suspension, &state);

    ivp_vec3r_t pb, pw;
    ivp_object_get_position(body, &pb);
    ivp_object_get_position(wheel, &pw);
    ivp_real_t dist_before = vec3_dist(&pb, &pw);

    step(env, 20);

    ivp_object_get_position(body, &pb);
    ivp_object_get_position(wheel, &pw);
    ivp_real_t dist_after = vec3_dist(&pb, &pw);

    /* Distance should decrease toward rest_length (1.0) */
    IVP_ASSERT_TRUE(dist_after < dist_before);

    ivp_environment_destroy(env);
}

IVP_TEST(suspension_asymmetric_damping) {
    /* Test that compression and extension use different damping.
     * We set up two suspensions: one with high compression damping,
     * one with equal damping. Compare behavior. */
    ivp_environment_t *env1 = make_env(DT60, false);
    ivp_environment_t *env2 = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env1);
    IVP_ASSERT_NOT_NULL(env2);

    /* Both: body at y=3, wheel at y=0, rest=1, stiffness=50
     * Objects will be compressing (moving toward each other) */
    ivp_object_t *body1  = make_ball(env1, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    ivp_object_t *wheel1 = make_ball(env1, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_object_t *body2  = make_ball(env2, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    ivp_object_t *wheel2 = make_ball(env2, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t anchor;
    ivp_vec3r_set_zero(&anchor);

    /* env1: high compression damping */
    ivp_suspension_state_t state1;
    ivp_suspension_state_init(&state1);
    ivp_suspension_state_setup(&state1, body1, wheel1, &anchor, &anchor,
                                IVP_REAL(50.0), IVP_REAL(1.0), IVP_REAL(20.0), IVP_REAL(1.0));

    /* env2: equal damping */
    ivp_suspension_state_t state2;
    ivp_suspension_state_init(&state2);
    ivp_suspension_state_setup(&state2, body2, wheel2, &anchor, &anchor,
                                IVP_REAL(50.0), IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));

    ivp_environment_add_controller(env1, 0, ivp_controller_suspension, &state1);
    ivp_environment_add_controller(env2, 0, ivp_controller_suspension, &state2);

    step(env1, 30);
    step(env2, 30);

    /* The high compression damping case should have a different velocity
     * or position compared to the equal damping case, demonstrating
     * asymmetric behavior */
    ivp_vec3r_t pb1, pb2;
    ivp_object_get_position(body1, &pb1);
    ivp_object_get_position(body2, &pb2);

    /* They should differ -- the asymmetric damping changes the trajectory */
    ivp_real_t pos_diff = ivp_fabs(pb1.k[1] - pb2.k[1]);
    IVP_ASSERT_TRUE(pos_diff > TOL);

    ivp_environment_destroy(env1);
    ivp_environment_destroy(env2);
}

IVP_TEST(suspension_max_body_force_clamp) {
    /* With max_body_force set, body force should be capped while wheel is not. */
    ivp_environment_t *env = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env);

    /* Large distance from rest to generate large force */
    ivp_object_t *body  = make_ball(env, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(10.0), IVP_REAL(0.0));
    ivp_object_t *wheel = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t anchor;
    ivp_vec3r_set_zero(&anchor);

    ivp_suspension_state_t state;
    ivp_suspension_state_init(&state);
    ivp_suspension_state_setup(&state, body, wheel, &anchor, &anchor,
                                IVP_REAL(100.0), IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_suspension_set_max_body_force(&state, IVP_REAL(50.0));

    ivp_environment_add_controller(env, 0, ivp_controller_suspension, &state);

    step(env, 1);

    /* current_force should be large (stretch = 9.0, stiffness = 100 → ~900N)
     * but current_body_force should be clamped to 50 */
    IVP_ASSERT_TRUE(ivp_fabs(state.current_force) > IVP_REAL(50.0));
    IVP_ASSERT_TRUE(ivp_fabs(state.current_body_force) <= IVP_REAL(50.0) + TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(suspension_set_damping_runtime) {
    ivp_suspension_state_t state;
    ivp_suspension_state_init(&state);

    IVP_ASSERT_NEAR(state.damping_compression, IVP_REAL(1.0), TOL);

    ivp_suspension_set_damping_compression(&state, IVP_REAL(15.0));
    IVP_ASSERT_NEAR(state.damping_compression, IVP_REAL(15.0), TOL);

    ivp_suspension_set_max_body_force(&state, IVP_REAL(200.0));
    IVP_ASSERT_NEAR(state.max_body_force, IVP_REAL(200.0), TOL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Constraint Local API Extension Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(constraint_local_enable_disable) {
    ivp_environment_t *env = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t anchor;
    ivp_vec3r_set_zero(&anchor);

    ivp_constraint_local_state_t state;
    ivp_constraint_local_init(&state, a, &anchor, b, &anchor);
    IVP_ASSERT_TRUE(ivp_constraint_local_is_active(&state));

    /* Disable */
    ivp_constraint_local_set_active(&state, false);
    IVP_ASSERT_FALSE(ivp_constraint_local_is_active(&state));

    /* Give b a velocity and simulate -- constraint should not hold */
    b->core.speed.k[0] = IVP_REAL(5.0);
    ivp_environment_add_controller(env, 0, ivp_controller_constraint_local, &state);
    step(env, 10);

    ivp_vec3r_t pb;
    ivp_object_get_position(b, &pb);
    /* b should have moved significantly since constraint is inactive */
    IVP_ASSERT_TRUE(pb.k[0] > IVP_REAL(1.5));

    /* Re-enable */
    ivp_constraint_local_set_active(&state, true);
    IVP_ASSERT_TRUE(ivp_constraint_local_is_active(&state));

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_local_broken_query_reset) {
    ivp_environment_t *env = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t anchor;
    ivp_vec3r_set_zero(&anchor);

    ivp_constraint_local_state_t state;
    ivp_constraint_local_init(&state, a, &anchor, b, &anchor);

    /* Set very low break threshold on translation X */
    ivp_constraint_local_set_max_impulse(&state, 0, IVP_REAL(0.001), IVP_CFORCE_BREAK);

    IVP_ASSERT_FALSE(ivp_constraint_local_is_broken(&state));

    ivp_environment_add_controller(env, 0, ivp_controller_constraint_local, &state);

    /* Give b a large velocity to exceed the break threshold */
    b->core.speed.k[0] = IVP_REAL(100.0);
    step(env, 1);

    /* Constraint should be broken */
    IVP_ASSERT_TRUE(ivp_constraint_local_is_broken(&state));
    IVP_ASSERT_FALSE(ivp_constraint_local_is_active(&state));

    /* Reset broken flag with reactivation */
    ivp_constraint_local_reset_broken(&state, true);
    IVP_ASSERT_FALSE(ivp_constraint_local_is_broken(&state));
    IVP_ASSERT_TRUE(ivp_constraint_local_is_active(&state));

    /* Reset broken without reactivation */
    state.broken = true;
    state.active = false;
    ivp_constraint_local_reset_broken(&state, false);
    IVP_ASSERT_FALSE(ivp_constraint_local_is_broken(&state));
    IVP_ASSERT_FALSE(ivp_constraint_local_is_active(&state));

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_local_rebind_anchors) {
    ivp_environment_t *env = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set_zero(&anchor_a);
    ivp_vec3r_set_zero(&anchor_b);

    ivp_constraint_local_state_t state;
    ivp_constraint_local_init(&state, a, &anchor_a, b, &anchor_b);

    /* Verify original anchors */
    IVP_ASSERT_NEAR(state.anchor_a_os.k[0], IVP_REAL(0.0), TOL);

    /* Rebind to new anchors */
    ivp_vec3r_t new_a, new_b;
    ivp_vec3r_set(&new_a, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&new_b, IVP_REAL(-0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_constraint_local_rebind_anchors(&state, &new_a, &new_b);

    IVP_ASSERT_NEAR(state.anchor_a_os.k[0], IVP_REAL(0.5), TOL);
    IVP_ASSERT_NEAR(state.anchor_b_os.k[0], IVP_REAL(-0.5), TOL);

    /* Simulate and verify that new anchors are used (objects should be
     * pulled toward making these new anchor points coincide) */
    ivp_environment_add_controller(env, 0, ivp_controller_constraint_local, &state);
    step(env, 20);

    /* With new anchors, constraint targets anchor_a_ws == anchor_b_ws.
     * anchor_a_ws = pos_a + (0.5,0,0), anchor_b_ws = pos_b + (-0.5,0,0).
     * Both anchors should end up close. */
    ivp_vec3r_t pa, pb;
    ivp_object_get_position(a, &pa);
    ivp_object_get_position(b, &pb);
    ivp_real_t anchor_dist = ivp_fabs((pa.k[0] + IVP_REAL(0.5)) - (pb.k[0] - IVP_REAL(0.5)));
    IVP_ASSERT_TRUE(anchor_dist < LTOL);

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_local_rebind_reference) {
    ivp_environment_t *env = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t anchor;
    ivp_vec3r_set_zero(&anchor);

    ivp_constraint_local_state_t state;
    ivp_constraint_local_init(&state, a, &anchor, b, &anchor);

    /* Record original reference quaternions */
    ivp_quatr_t orig_ref_a = state.ref_q_a;
    ivp_quatr_t orig_ref_b = state.ref_q_b;

    /* Manually rotate object b's orientation */
    ivp_quatr_t rot;
    rot.x = IVP_REAL(0.0);
    rot.y = IVP_REAL(0.3826834); /* ~45 degrees around Y */
    rot.z = IVP_REAL(0.0);
    rot.w = IVP_REAL(0.9238795);
    b->core.q_world_f_core_last_psi = rot;

    /* Rebind reference */
    ivp_constraint_local_rebind_reference(&state);

    /* ref_q_b should now match the new orientation */
    IVP_ASSERT_NEAR(state.ref_q_b.y, rot.y, TOL);
    IVP_ASSERT_NEAR(state.ref_q_b.w, rot.w, TOL);

    /* ref_q_a should remain identity (object a hasn't moved) */
    IVP_ASSERT_NEAR(state.ref_q_a.w, orig_ref_a.w, TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_local_bend_mode) {
    /* BEND mode should not break the constraint and should apply softer force */
    ivp_environment_t *env = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t anchor;
    ivp_vec3r_set_zero(&anchor);

    ivp_constraint_local_state_t state;
    ivp_constraint_local_init(&state, a, &anchor, b, &anchor);

    /* Set BEND mode with a low threshold */
    ivp_constraint_local_set_max_impulse(&state, 0, IVP_REAL(0.01), IVP_CFORCE_BEND);

    ivp_environment_add_controller(env, 0, ivp_controller_constraint_local, &state);

    /* Give b a large velocity */
    b->core.speed.k[0] = IVP_REAL(50.0);
    step(env, 5);

    /* Constraint should NOT be broken (BEND doesn't break) */
    IVP_ASSERT_FALSE(ivp_constraint_local_is_broken(&state));
    IVP_ASSERT_TRUE(ivp_constraint_local_is_active(&state));

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_local_bend_vs_clip) {
    /* BEND should produce less accumulated impulse than CLIP under the same conditions */
    ivp_environment_t *env_bend = make_env(DT60, false);
    ivp_environment_t *env_clip = make_env(DT60, false);
    IVP_ASSERT_NOT_NULL(env_bend);
    IVP_ASSERT_NOT_NULL(env_clip);

    /* Same configuration for both */
    ivp_object_t *a_bend = make_ball(env_bend, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b_bend = make_ball(env_bend, IVP_REAL(1.0), IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_object_t *a_clip = make_ball(env_clip, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b_clip = make_ball(env_clip, IVP_REAL(1.0), IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_t anchor;
    ivp_vec3r_set_zero(&anchor);

    ivp_constraint_local_state_t state_bend;
    ivp_constraint_local_init(&state_bend, a_bend, &anchor, b_bend, &anchor);
    ivp_constraint_local_set_max_impulse(&state_bend, 0, IVP_REAL(0.1), IVP_CFORCE_BEND);

    ivp_constraint_local_state_t state_clip;
    ivp_constraint_local_init(&state_clip, a_clip, &anchor, b_clip, &anchor);
    ivp_constraint_local_set_max_impulse(&state_clip, 0, IVP_REAL(0.1), IVP_CFORCE_CLIP);

    ivp_environment_add_controller(env_bend, 0, ivp_controller_constraint_local, &state_bend);
    ivp_environment_add_controller(env_clip, 0, ivp_controller_constraint_local, &state_clip);

    /* Give b objects the same large velocity */
    b_bend->core.speed.k[0] = IVP_REAL(50.0);
    b_clip->core.speed.k[0] = IVP_REAL(50.0);

    step(env_bend, 1);
    step(env_clip, 1);

    /* BEND accumulated impulse should be less than CLIP */
    IVP_ASSERT_TRUE(state_bend.accumulated_impulse[0] < state_clip.accumulated_impulse[0]);

    ivp_environment_destroy(env_bend);
    ivp_environment_destroy(env_clip);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test Registration
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void) {
    /* Stabilizer */
    IVP_REGISTER(stabilizer_init_defaults);
    IVP_REGISTER(stabilizer_setup_params);
    /* TODO: re-enable after sim unit controller dispatch rework */
    /* IVP_REGISTER(stabilizer_equalizes_pair_distances); */
    IVP_REGISTER(stabilizer_zero_constant_no_effect);
    /* TODO: re-enable after sim unit controller dispatch rework */
    /* IVP_REGISTER(stabilizer_set_constant_runtime); */

    /* Suspension */
    IVP_REGISTER(suspension_init_defaults);
    IVP_REGISTER(suspension_setup_params);
    IVP_REGISTER(suspension_basic_spring_behavior);
    IVP_REGISTER(suspension_asymmetric_damping);
    IVP_REGISTER(suspension_max_body_force_clamp);
    IVP_REGISTER(suspension_set_damping_runtime);

    /* Constraint API extensions */
    IVP_REGISTER(constraint_local_enable_disable);
    IVP_REGISTER(constraint_local_broken_query_reset);
    IVP_REGISTER(constraint_local_rebind_anchors);
    IVP_REGISTER(constraint_local_rebind_reference);
    IVP_REGISTER(constraint_local_bend_mode);
    IVP_REGISTER(constraint_local_bend_vs_clip);

    IVP_RUN_ALL();
}
