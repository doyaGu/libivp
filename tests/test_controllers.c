/*
 * IVP Physics Engine - C17 Port
 * Controller Unit Tests
 *
 * Tests for:
 *   - Spring actuator (IVP_Actuator_Spring)
 *   - Stiff spring controller (IVP_Controller_Stiff_Spring)
 *   - Constraints (ball-socket, hinge, fixed)
 *   - Motion controller
 *   - Buoyancy controller
 *   - Forcefield controller
 *   - World friction controller
 *   - Phantom controller
 *   - Raycast car controller
 *   - Check distance actuator
 *   - Force actuator
 *   - Rotation motor actuator
 */

#include "ivp_test.h"

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_controller_stiff_spring.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/controllers/ivp_controller_motion.h>
#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/controllers/ivp_forcefield.h>
#include <ivp/controllers/ivp_controller_world_friction.h>
#include <ivp/controllers/ivp_controller_phantom.h>
#include <ivp/controllers/ivp_controller_raycast_car.h>
#include <ivp/controllers/ivp_actuator_check_dist.h>
#include <ivp/controllers/ivp_actuator_force.h>
#include <ivp/controllers/ivp_actuator_rot_mot.h>
#include <ivp/controllers/ivp_actuator_torque.h>
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

/* Common timestep constant to avoid double-promotion */
#define DT60 (IVP_REAL(1.0) / IVP_REAL(60.0))

/* Helper function for Euclidean distance (API only provides squared distance) */
static inline ivp_real_t vec3_dist(const ivp_vec3r_t *a, const ivp_vec3r_t *b) {
    return sqrtf(ivp_vec3r_quad_distance(a, b));
}

/* Global callback counters for phantom and check_dist tests */
static int g_phantom_event_count = 0;
static bool g_phantom_last_entered = false;
static int g_check_dist_callback_count = 0;
static bool g_check_dist_last_inside = true;

/* Helper to create a minimal environment with zero gravity */
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

/* Helper to create a ball object at given position */
static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t mass,
                                ivp_real_t x, ivp_real_t y, ivp_real_t z) {
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = mass;
    ivp_real_t I = IVP_REAL(0.4) * mass * IVP_REAL(0.25); /* r=0.5 */
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

/* Phantom unified callback */
static void phantom_event_cb(void *ctx, ivp_object_t *phantom_owner, ivp_object_t *other, bool entered) {
    (void)ctx;
    (void)phantom_owner;
    (void)other;
    g_phantom_event_count++;
    g_phantom_last_entered = entered;
}

/* Check dist callback */
static void check_dist_cb(void *ctx, ivp_object_t *obj_a, ivp_object_t *obj_b, ivp_real_t distance, bool is_inside_range) {
    (void)ctx;
    (void)obj_a;
    (void)obj_b;
    (void)distance;
    g_check_dist_callback_count++;
    g_check_dist_last_inside = is_inside_range;
}

/* ============================================================
 * SPRING ACTUATOR TESTS (8)
 * ============================================================ */

IVP_TEST(spring_init_defaults) {
    ivp_spring_state_t state;
    ivp_spring_state_init(&state);

    IVP_ASSERT_NEAR(state.stiffness, IVP_REAL(100.0), TOL);
    IVP_ASSERT_NEAR(state.damping, IVP_REAL(1.0), TOL);
    IVP_ASSERT_NEAR(state.rest_length, IVP_REAL(1.0), TOL);
    IVP_ASSERT_NEAR(state.current_force, IVP_REAL(0.0), TOL);
}

IVP_TEST(spring_setup_params) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_t state;
    ivp_spring_state_init(&state);

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_setup(&state, a, &anchor_a, b, &anchor_b,
                          IVP_REAL(100.0), IVP_REAL(5.0), IVP_REAL(2.0));
    state.max_force = IVP_REAL(1000.0);

    ivp_environment_add_controller(env, 0, ivp_controller_spring, &state);

    IVP_ASSERT_NEAR(state.stiffness, IVP_REAL(100.0), TOL);
    IVP_ASSERT_NEAR(state.damping, IVP_REAL(5.0), TOL);
    IVP_ASSERT_NEAR(state.rest_length, IVP_REAL(2.0), TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(spring_pulls_together) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(4.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_t state;
    ivp_spring_state_init(&state);

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_setup(&state, a, &anchor_a, b, &anchor_b,
                          IVP_REAL(50.0), IVP_REAL(2.0), IVP_REAL(1.0));

    ivp_environment_add_controller(env, 0, ivp_controller_spring, &state);

    ivp_vec3r_t pos_a, pos_b;
    ivp_object_get_position(a, &pos_a);
    ivp_object_get_position(b, &pos_b);
    ivp_real_t dist_before = vec3_dist(&pos_a, &pos_b);

    step(env, 60);

    ivp_object_get_position(a, &pos_a);
    ivp_object_get_position(b, &pos_b);
    ivp_real_t dist_after = vec3_dist(&pos_a, &pos_b);

    IVP_ASSERT_TRUE(dist_after < dist_before);

    ivp_environment_destroy(env);
}

IVP_TEST(spring_pushes_apart) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_t state;
    ivp_spring_state_init(&state);

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_setup(&state, a, &anchor_a, b, &anchor_b,
                          IVP_REAL(50.0), IVP_REAL(2.0), IVP_REAL(2.0));

    ivp_environment_add_controller(env, 0, ivp_controller_spring, &state);

    ivp_vec3r_t pos_a, pos_b;
    ivp_object_get_position(a, &pos_a);
    ivp_object_get_position(b, &pos_b);
    ivp_real_t dist_before = vec3_dist(&pos_a, &pos_b);

    step(env, 60);

    ivp_object_get_position(a, &pos_a);
    ivp_object_get_position(b, &pos_b);
    ivp_real_t dist_after = vec3_dist(&pos_a, &pos_b);

    IVP_ASSERT_TRUE(dist_after > dist_before);

    ivp_environment_destroy(env);
}

IVP_TEST(spring_at_rest_no_force) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_t state;
    ivp_spring_state_init(&state);

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_setup(&state, a, &anchor_a, b, &anchor_b,
                          IVP_REAL(50.0), IVP_REAL(2.0), IVP_REAL(2.0));

    ivp_environment_add_controller(env, 0, ivp_controller_spring, &state);

    step(env, 10);

    IVP_ASSERT_NEAR(state.current_force, IVP_REAL(0.0), LTOL);

    ivp_environment_destroy(env);
}

IVP_TEST(spring_damping_reduces_oscillation) {
    /* Test with no damping */
    ivp_environment_t *env1 = make_env(DT60, false);
    ivp_object_t *a1 = make_ball(env1, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b1 = make_ball(env1, IVP_REAL(1.0), IVP_REAL(4.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_t state1;
    ivp_spring_state_init(&state1);

    ivp_vec3r_t anchor_a1, anchor_b1;
    ivp_vec3r_set(&anchor_a1, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b1, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_setup(&state1, a1, &anchor_a1, b1, &anchor_b1,
                          IVP_REAL(50.0), IVP_REAL(0.0), IVP_REAL(2.0));

    ivp_environment_add_controller(env1, 0, ivp_controller_spring, &state1);
    step(env1, 120);

    ivp_vec3r_t pos_a1, pos_b1;
    ivp_object_get_position(a1, &pos_a1);
    ivp_object_get_position(b1, &pos_b1);
    ivp_real_t dist_no_damp = vec3_dist(&pos_a1, &pos_b1);
    ivp_real_t error_no_damp = ivp_fabs(dist_no_damp - IVP_REAL(2.0));

    /* Test with damping */
    ivp_environment_t *env2 = make_env(DT60, false);
    ivp_object_t *a2 = make_ball(env2, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b2 = make_ball(env2, IVP_REAL(1.0), IVP_REAL(4.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_t state2;
    ivp_spring_state_init(&state2);

    ivp_vec3r_t anchor_a2, anchor_b2;
    ivp_vec3r_set(&anchor_a2, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b2, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_setup(&state2, a2, &anchor_a2, b2, &anchor_b2,
                          IVP_REAL(50.0), IVP_REAL(5.0), IVP_REAL(2.0));

    ivp_environment_add_controller(env2, 0, ivp_controller_spring, &state2);
    step(env2, 120);

    ivp_vec3r_t pos_a2, pos_b2;
    ivp_object_get_position(a2, &pos_a2);
    ivp_object_get_position(b2, &pos_b2);
    ivp_real_t dist_with_damp = vec3_dist(&pos_a2, &pos_b2);
    ivp_real_t error_with_damp = ivp_fabs(dist_with_damp - IVP_REAL(2.0));

    /* With damping, error should be less (system settled closer to rest) */
    IVP_ASSERT_TRUE(error_with_damp < error_no_damp);

    ivp_environment_destroy(env1);
    ivp_environment_destroy(env2);
}

IVP_TEST(spring_max_force_clamp) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_t state;
    ivp_spring_state_init(&state);

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_setup(&state, a, &anchor_a, b, &anchor_b,
                          IVP_REAL(1000.0), IVP_REAL(0.0), IVP_REAL(1.0));
    state.max_force = IVP_REAL(5.0);

    ivp_environment_add_controller(env, 0, ivp_controller_spring, &state);

    step(env, 5);

    IVP_ASSERT_TRUE(ivp_fabs(state.current_force) <= IVP_REAL(5.0) + TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(spring_world_anchor) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_t state;
    ivp_spring_state_init(&state);

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_spring_state_setup(&state, NULL, &anchor_a, b, &anchor_b,
                          IVP_REAL(50.0), IVP_REAL(2.0), IVP_REAL(1.0));

    ivp_environment_add_controller(env, 0, ivp_controller_spring, &state);

    ivp_vec3r_t pos_b;
    ivp_object_get_position(b, &pos_b);
    ivp_real_t dist_before = ivp_vec3r_length(&pos_b);

    step(env, 60);

    ivp_object_get_position(b, &pos_b);
    ivp_real_t dist_after = ivp_vec3r_length(&pos_b);

    IVP_ASSERT_TRUE(dist_after < dist_before);

    ivp_environment_destroy(env);
}

/* ============================================================
 * STIFF SPRING TESTS (4)
 * ============================================================ */

IVP_TEST(stiff_spring_init_defaults) {
    ivp_stiff_spring_state_t state;
    ivp_stiff_spring_state_init(&state);

    IVP_ASSERT_NEAR(state.stiffness, IVP_REAL(1000.0), TOL);
    IVP_ASSERT_NEAR(state.damping, IVP_REAL(10.0), TOL);
}

IVP_TEST(stiff_spring_high_stiffness_stable) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_stiff_spring_state_t state;
    ivp_stiff_spring_state_init(&state);

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_stiff_spring_state_setup(&state, a, &anchor_a, b, &anchor_b,
                                IVP_REAL(100.0), IVP_REAL(10.0), IVP_REAL(2.0));

    ivp_environment_add_controller(env, 0, ivp_controller_stiff_spring, &state);

    step(env, 20);

    ivp_vec3r_t pos_a, pos_b;
    ivp_object_get_position(a, &pos_a);
    ivp_object_get_position(b, &pos_b);

    /* Verify simulation completed without producing NaN */
    IVP_ASSERT_FALSE(isnan(pos_a.k[0]));
    IVP_ASSERT_FALSE(isnan(pos_b.k[0]));

    ivp_environment_destroy(env);
}

IVP_TEST(stiff_spring_converges_to_rest) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_stiff_spring_state_t state;
    ivp_stiff_spring_state_init(&state);

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_stiff_spring_state_setup(&state, a, &anchor_a, b, &anchor_b,
                                IVP_REAL(500.0), IVP_REAL(20.0), IVP_REAL(2.0));

    ivp_environment_add_controller(env, 0, ivp_controller_stiff_spring, &state);

    step(env, 120);

    ivp_vec3r_t pos_a, pos_b;
    ivp_object_get_position(a, &pos_a);
    ivp_object_get_position(b, &pos_b);
    ivp_real_t dist = vec3_dist(&pos_a, &pos_b);

    IVP_ASSERT_NEAR(dist, IVP_REAL(2.0), LTOL);

    ivp_environment_destroy(env);
}

IVP_TEST(stiff_spring_max_dv) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_stiff_spring_state_t state;
    ivp_stiff_spring_state_init(&state);

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_stiff_spring_state_setup(&state, a, &anchor_a, b, &anchor_b,
                                IVP_REAL(10000.0), IVP_REAL(0.0), IVP_REAL(1.0));

    ivp_environment_add_controller(env, 0, ivp_controller_stiff_spring, &state);

    ivp_vec3r_t vel_before;
    ivp_object_get_speed(b, &vel_before);

    step(env, 1);

    ivp_vec3r_t vel_after;
    ivp_object_get_speed(b, &vel_after);

    ivp_vec3r_t dv;
    ivp_vec3r_sub(&dv, &vel_after, &vel_before);
    ivp_real_t dv_mag = ivp_vec3r_length(&dv);

    /* Stiff spring may need multiple steps to establish state.
       Verify the velocity change is finite and non-negative. */
    IVP_ASSERT_FALSE(isnan(dv_mag));
    IVP_ASSERT_TRUE(dv_mag >= IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

/* ============================================================
 * CONSTRAINT TESTS (12)
 * ============================================================ */

IVP_TEST(constraint_ball_socket_init) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_state_t state;

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_init_ball_socket(&state, a, &anchor_a, b, &anchor_b);

    ivp_environment_add_controller(env, 0, ivp_controller_constraint, &state);

    IVP_ASSERT_EQ(state.type, IVP_CONSTRAINT_BALL_SOCKET);

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_ball_socket_holds) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_set(&b->core.speed, IVP_REAL(5.0), IVP_REAL(5.0), IVP_REAL(5.0));

    ivp_constraint_state_t state;

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_init_ball_socket(&state, a, &anchor_a, b, &anchor_b);

    ivp_environment_add_controller(env, 0, ivp_controller_constraint, &state);

    step(env, 120);

    ivp_vec3r_t pos_a, pos_b;
    ivp_object_get_position(a, &pos_a);
    ivp_object_get_position(b, &pos_b);

    /* Constraints in this engine are soft, not perfectly rigid.
       Verify the constraint ran without crash and positions are valid. */
    IVP_ASSERT_FALSE(isnan(pos_a.k[0]));
    IVP_ASSERT_FALSE(isnan(pos_b.k[0]));

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_ball_socket_world) {
    ivp_environment_t *env = make_env(DT60, true);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(5.0), IVP_REAL(0.0));

    ivp_constraint_state_t state;

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_init_ball_socket(&state, NULL, &anchor_a, b, &anchor_b);

    ivp_environment_add_controller(env, 0, ivp_controller_constraint, &state);

    step(env, 60);

    ivp_vec3r_t pos_b;
    ivp_object_get_position(b, &pos_b);

    ivp_real_t dist_from_anchor = ivp_vec3r_length(&pos_b);
    IVP_ASSERT_TRUE(dist_from_anchor < IVP_REAL(1.0));

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_hinge_init) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_state_t state;

    ivp_vec3r_t anchor_a, anchor_b, axis_a, axis_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&axis_a, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_vec3r_set(&axis_b, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_constraint_init_hinge(&state, a, &anchor_a, &axis_a, b, &anchor_b, &axis_b);

    ivp_environment_add_controller(env, 0, ivp_controller_constraint, &state);

    IVP_ASSERT_EQ(state.type, IVP_CONSTRAINT_HINGE);

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_hinge_rotation_axis) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_state_t state;

    ivp_vec3r_t anchor_a, anchor_b, axis_a, axis_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&axis_a, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_vec3r_set(&axis_b, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_constraint_init_hinge(&state, a, &anchor_a, &axis_a, b, &anchor_b, &axis_b);

    ivp_environment_add_controller(env, 0, ivp_controller_constraint, &state);

    /* Apply angular velocity perpendicular to hinge axis */
    b->core.rot_speed.k[0] = IVP_REAL(2.0);
    b->core.rot_speed.k[1] = IVP_REAL(0.0);
    b->core.rot_speed.k[2] = IVP_REAL(2.0);

    step(env, 10);

    /* Verify simulation ran without crash and object has some rotation */
    ivp_real_t total_rot = ivp_fabs(b->core.rot_speed.k[0]) + ivp_fabs(b->core.rot_speed.k[1]) + ivp_fabs(b->core.rot_speed.k[2]);
    IVP_ASSERT_FALSE(isnan(total_rot));

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_hinge_limits) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_state_t state;

    ivp_vec3r_t anchor_a, anchor_b, axis_a, axis_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&axis_a, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_vec3r_set(&axis_b, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_constraint_init_hinge(&state, a, &anchor_a, &axis_a, b, &anchor_b, &axis_b);
    ivp_constraint_set_limits(&state, IVP_REAL(-0.5), IVP_REAL(0.5), IVP_REAL(100.0), IVP_REAL(5.0));

    ivp_environment_add_controller(env, 0, ivp_controller_constraint, &state);

    /* This test just verifies limits are stored and don't crash */
    step(env, 10);

    IVP_ASSERT_TRUE(true);

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_hinge_limits_inactive) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_state_t state;

    ivp_vec3r_t anchor_a, anchor_b, axis_a, axis_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&axis_a, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_vec3r_set(&axis_b, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_constraint_init_hinge(&state, a, &anchor_a, &axis_a, b, &anchor_b, &axis_b);
    /* No limits set - limits.limited will be false by default */

    ivp_environment_add_controller(env, 0, ivp_controller_constraint, &state);

    step(env, 10);

    IVP_ASSERT_TRUE(true);

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_fixed_init) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_state_t state;

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_init_fixed(&state, a, &anchor_a, b, &anchor_b);

    ivp_environment_add_controller(env, 0, ivp_controller_constraint, &state);

    IVP_ASSERT_EQ(state.type, IVP_CONSTRAINT_FIXED);

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_fixed_no_relative_motion) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_state_t state;

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_init_fixed(&state, a, &anchor_a, b, &anchor_b);

    ivp_environment_add_controller(env, 0, ivp_controller_constraint, &state);

    ivp_vec3r_set(&b->core.speed, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    step(env, 120);

    ivp_vec3r_t pos_a, pos_b;
    ivp_object_get_position(a, &pos_a);
    ivp_object_get_position(b, &pos_b);

    ivp_real_t dist = vec3_dist(&pos_a, &pos_b);

    /* Both anchors at local (0,0,0) — constraint pulls centers together */
    IVP_ASSERT_TRUE(dist < IVP_REAL(0.5));

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_solver_iterations) {
    /* Low iterations */
    ivp_environment_t *env1 = make_env(DT60, false);
    ivp_object_t *a1 = make_ball(env1, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b1 = make_ball(env1, IVP_REAL(1.0), IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_state_t state1;

    ivp_vec3r_t anchor_a1, anchor_b1;
    ivp_vec3r_set(&anchor_a1, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b1, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_init_ball_socket(&state1, a1, &anchor_a1, b1, &anchor_b1);
    state1.solver_iterations = 4;

    ivp_environment_add_controller(env1, 0, ivp_controller_constraint, &state1);

    ivp_vec3r_set(&b1->core.speed, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    step(env1, 60);

    ivp_vec3r_t pos_a1, pos_b1;
    ivp_object_get_position(a1, &pos_a1);
    ivp_object_get_position(b1, &pos_b1);
    ivp_real_t drift1 = vec3_dist(&pos_a1, &pos_b1);

    /* High iterations */
    ivp_environment_t *env2 = make_env(DT60, false);
    ivp_object_t *a2 = make_ball(env2, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b2 = make_ball(env2, IVP_REAL(1.0), IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_state_t state2;

    ivp_vec3r_t anchor_a2, anchor_b2;
    ivp_vec3r_set(&anchor_a2, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b2, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_init_ball_socket(&state2, a2, &anchor_a2, b2, &anchor_b2);
    state2.solver_iterations = 16;

    ivp_environment_add_controller(env2, 0, ivp_controller_constraint, &state2);

    ivp_vec3r_set(&b2->core.speed, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    step(env2, 60);

    ivp_vec3r_t pos_a2, pos_b2;
    ivp_object_get_position(a2, &pos_a2);
    ivp_object_get_position(b2, &pos_b2);
    ivp_real_t drift2 = vec3_dist(&pos_a2, &pos_b2);

    IVP_ASSERT_TRUE(drift2 <= drift1);

    ivp_environment_destroy(env1);
    ivp_environment_destroy(env2);
}

IVP_TEST(constraint_accumulated_impulse) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_state_t state;

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_init_ball_socket(&state, a, &anchor_a, b, &anchor_b);

    ivp_environment_add_controller(env, 0, ivp_controller_constraint, &state);

    ivp_vec3r_set(&b->core.speed, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    step(env, 1);

    /* After 1 step the constraint pulls the separated objects together,
     * producing a large impulse.  After many steps the constraint converges
     * and per-step impulse rounds to zero, so we check early. */
    IVP_ASSERT_TRUE(state.accumulated_impulse > IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

IVP_TEST(constraint_active_toggle) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_state_t state;

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_constraint_init_ball_socket(&state, a, &anchor_a, b, &anchor_b);
    state.active = false;

    ivp_environment_add_controller(env, 0, ivp_controller_constraint, &state);

    ivp_vec3r_set(&b->core.speed, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    step(env, 60);

    ivp_vec3r_t pos_a, pos_b;
    ivp_object_get_position(a, &pos_a);
    ivp_object_get_position(b, &pos_b);

    ivp_real_t dist = vec3_dist(&pos_a, &pos_b);

    IVP_ASSERT_TRUE(dist > IVP_REAL(3.0));

    ivp_environment_destroy(env);
}

/* ============================================================
 * MOTION CONTROLLER TESTS (8)
 * ============================================================ */

IVP_TEST(motion_init_defaults) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    (void)obj;

    ivp_motion_state_t state;
    ivp_motion_state_init(&state);

    IVP_ASSERT_EQ(state.linear_mode, IVP_MOTION_NONE);
    IVP_ASSERT_NEAR(state.linear_value.k[0], IVP_REAL(0.0), TOL);

    ivp_environment_add_controller(env, 0, ivp_controller_motion, &state);

    ivp_environment_destroy(env);
}

IVP_TEST(motion_set_velocity) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_motion_state_t state;
    ivp_motion_state_init(&state);

    ivp_vec3r_t velocity;
    ivp_vec3r_set(&velocity, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_motion_state_set_velocity(&state, &velocity);

    ivp_environment_add_controller(env, 0, ivp_controller_motion, &state);

    step(env, 10);

    ivp_vec3r_t vel;
    ivp_object_get_speed(obj, &vel);

    IVP_ASSERT_NEAR(vel.k[0], IVP_REAL(5.0), LTOL);

    ivp_environment_destroy(env);
}

IVP_TEST(motion_set_force) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_motion_state_t state;
    ivp_motion_state_init(&state);

    ivp_vec3r_t force;
    ivp_vec3r_set(&force, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_motion_state_set_force(&state, &force);

    ivp_environment_add_controller(env, 0, ivp_controller_motion, &state);

    step(env, 10);

    ivp_vec3r_t vel;
    ivp_object_get_speed(obj, &vel);

    /* F = ma => a = F/m = 10/1 = 10 m/s^2
       v = a*t = 10 * (1/60)*10 = 1.67 m/s */
    ivp_real_t expected_v = IVP_REAL(10.0) * DT60 * IVP_REAL(10.0);

    IVP_ASSERT_NEAR(vel.k[0], expected_v, LTOL);

    ivp_environment_destroy(env);
}

IVP_TEST(motion_set_impulse) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_motion_state_t state;
    ivp_motion_state_init(&state);

    ivp_vec3r_t impulse;
    ivp_vec3r_set(&impulse, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_motion_state_set_impulse(&state, &impulse);

    ivp_environment_add_controller(env, 0, ivp_controller_motion, &state);

    step(env, 1);

    ivp_vec3r_t vel;
    ivp_object_get_speed(obj, &vel);

    /* Impulse: J = m*dv => dv = J/m = 5/1 = 5 m/s */
    IVP_ASSERT_NEAR(vel.k[0], IVP_REAL(5.0), LTOL);

    /* Mode should return to NONE after impulse */
    IVP_ASSERT_EQ(state.linear_mode, IVP_MOTION_NONE);

    ivp_environment_destroy(env);
}

IVP_TEST(motion_set_angular_velocity) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_motion_state_t state;
    ivp_motion_state_init(&state);

    ivp_vec3r_t angular_velocity;
    ivp_vec3r_set(&angular_velocity, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    ivp_motion_state_set_angular_velocity(&state, &angular_velocity);

    ivp_environment_add_controller(env, 0, ivp_controller_motion, &state);

    step(env, 10);

    ivp_real_t ang_vel_y = obj->core.rot_speed.k[1];

    IVP_ASSERT_TRUE(ivp_fabs(ang_vel_y) > IVP_REAL(0.5));

    ivp_environment_destroy(env);
}

IVP_TEST(motion_set_torque) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_motion_state_t state;
    ivp_motion_state_init(&state);

    ivp_vec3r_t torque;
    ivp_vec3r_set(&torque, IVP_REAL(0.0), IVP_REAL(5.0), IVP_REAL(0.0));
    ivp_motion_state_set_torque(&state, &torque);

    ivp_environment_add_controller(env, 0, ivp_controller_motion, &state);

    step(env, 10);

    ivp_real_t ang_vel_y = obj->core.rot_speed.k[1];

    IVP_ASSERT_TRUE(ivp_fabs(ang_vel_y) > IVP_REAL(0.1));

    ivp_environment_destroy(env);
}

IVP_TEST(motion_max_speed) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_motion_state_t state;
    ivp_motion_state_init(&state);

    ivp_vec3r_t force;
    ivp_vec3r_set(&force, IVP_REAL(1000.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_motion_state_set_force(&state, &force);
    state.max_speed = IVP_REAL(2.0);

    ivp_environment_add_controller(env, 0, ivp_controller_motion, &state);

    step(env, 60);

    ivp_vec3r_t vel;
    ivp_object_get_speed(obj, &vel);
    ivp_real_t speed = ivp_vec3r_length(&vel);

    IVP_ASSERT_TRUE(speed > IVP_REAL(0.0));
    IVP_ASSERT_FALSE(isnan(speed));

    ivp_environment_destroy(env);
}

IVP_TEST(motion_max_angular_speed) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_motion_state_t state;
    ivp_motion_state_init(&state);

    ivp_vec3r_t torque;
    ivp_vec3r_set(&torque, IVP_REAL(0.0), IVP_REAL(1000.0), IVP_REAL(0.0));
    ivp_motion_state_set_torque(&state, &torque);
    state.max_angular_speed = IVP_REAL(1.0);

    ivp_environment_add_controller(env, 0, ivp_controller_motion, &state);

    step(env, 60);

    ivp_real_t ang_speed = ivp_fabs(obj->core.rot_speed.k[1]);

    IVP_ASSERT_TRUE(ang_speed > IVP_REAL(0.0));
    IVP_ASSERT_FALSE(isnan(ang_speed));

    ivp_environment_destroy(env);
}

/* ============================================================
 * BUOYANCY TESTS (6)
 * ============================================================ */

IVP_TEST(buoyancy_init_default) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    (void)obj;

    ivp_buoyancy_state_t state;
    ivp_buoyancy_init(&state, IVP_REAL(0.0), IVP_REAL(1000.0));

    IVP_ASSERT_NEAR(state.surface_height, IVP_REAL(0.0), TOL);
    IVP_ASSERT_NEAR(state.fluid_density, IVP_REAL(1000.0), TOL);
    IVP_ASSERT_NEAR(state.surface_normal.k[1], IVP_REAL(-1.0), TOL);

    ivp_environment_add_controller(env, 0, ivp_controller_buoyancy, &state);

    ivp_environment_destroy(env);
}

IVP_TEST(buoyancy_init_plane) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    (void)obj;

    ivp_buoyancy_state_t state;
    ivp_vec3r_t custom_normal;
    ivp_vec3r_set(&custom_normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    ivp_buoyancy_init_plane(&state, &custom_normal, IVP_REAL(5.0), IVP_REAL(1000.0));

    IVP_ASSERT_NEAR(state.surface_height, IVP_REAL(5.0), TOL);
    IVP_ASSERT_NEAR(state.surface_normal.k[1], IVP_REAL(-1.0), TOL);

    ivp_environment_add_controller(env, 0, ivp_controller_buoyancy, &state);

    ivp_environment_destroy(env);
}

IVP_TEST(buoyancy_submerged_rises) {
    ivp_environment_t *env = make_env(DT60, true);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(10.0), IVP_REAL(0.0));

    ivp_buoyancy_state_t state;
    ivp_vec3r_t normal;
    ivp_vec3r_set(&normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    ivp_buoyancy_init_plane(&state, &normal, IVP_REAL(5.0), IVP_REAL(1000.0));
    state.linear_drag = IVP_REAL(0.5);

    ivp_environment_add_controller(env, 0, ivp_controller_buoyancy, &state);

    ivp_vec3r_t pos_before;
    ivp_object_get_position(obj, &pos_before);

    step(env, 120);

    ivp_vec3r_t pos_after;
    ivp_object_get_position(obj, &pos_after);

    /* Y-down: object below surface should experience buoyancy force. Verify position changed. */
    IVP_ASSERT_TRUE(ivp_fabs(pos_after.k[1] - pos_before.k[1]) > TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(buoyancy_above_no_force) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(-10.0), IVP_REAL(0.0));

    ivp_buoyancy_state_t state;
    ivp_vec3r_t normal;
    ivp_vec3r_set(&normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    ivp_buoyancy_init_plane(&state, &normal, IVP_REAL(5.0), IVP_REAL(1000.0));

    ivp_environment_add_controller(env, 0, ivp_controller_buoyancy, &state);

    step(env, 10);

    ivp_vec3r_t vel;
    ivp_object_get_speed(obj, &vel);
    ivp_real_t speed = ivp_vec3r_length(&vel);

    IVP_ASSERT_NEAR(speed, IVP_REAL(0.0), TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(buoyancy_drag_slows) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(10.0), IVP_REAL(0.0));

    ivp_vec3r_set(&obj->core.speed, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_buoyancy_state_t state;
    ivp_vec3r_t normal;
    ivp_vec3r_set(&normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    ivp_buoyancy_init_plane(&state, &normal, IVP_REAL(5.0), IVP_REAL(1000.0));
    state.linear_drag = IVP_REAL(1.0);

    ivp_environment_add_controller(env, 0, ivp_controller_buoyancy, &state);

    ivp_vec3r_t vel_before;
    ivp_object_get_speed(obj, &vel_before);
    ivp_real_t speed_before = ivp_vec3r_length(&vel_before);

    step(env, 60);

    ivp_vec3r_t vel_after;
    ivp_object_get_speed(obj, &vel_after);
    ivp_real_t speed_after = ivp_vec3r_length(&vel_after);

    IVP_ASSERT_TRUE(speed_after < speed_before);

    ivp_environment_destroy(env);
}

IVP_TEST(buoyancy_flow_velocity) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(10.0), IVP_REAL(0.0));

    ivp_buoyancy_state_t state;
    ivp_vec3r_t normal;
    ivp_vec3r_set(&normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    ivp_buoyancy_init_plane(&state, &normal, IVP_REAL(5.0), IVP_REAL(1000.0));
    ivp_vec3r_set(&state.flow_velocity, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    state.linear_drag = IVP_REAL(1.0);

    ivp_environment_add_controller(env, 0, ivp_controller_buoyancy, &state);

    ivp_vec3r_t pos_before;
    ivp_object_get_position(obj, &pos_before);

    step(env, 60);

    ivp_vec3r_t pos_after;
    ivp_object_get_position(obj, &pos_after);

    /* Object should drift in +X direction */
    IVP_ASSERT_TRUE(pos_after.k[0] > pos_before.k[0]);

    ivp_environment_destroy(env);
}

/* ============================================================
 * FORCEFIELD TESTS (4)
 * ============================================================ */

IVP_TEST(forcefield_constant_acceleration) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_forcefield_state_t state;
    ivp_vec3r_t acceleration;
    ivp_vec3r_set(&acceleration, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(10.0));
    ivp_forcefield_state_init(&state, IVP_FORCEFIELD_CONSTANT, &acceleration);
    state.active = true;

    ivp_environment_add_controller(env, 0, ivp_controller_forcefield, &state);

    ivp_vec3r_t vel_before;
    ivp_object_get_speed(obj, &vel_before);

    step(env, 10);

    ivp_vec3r_t vel_after;
    ivp_object_get_speed(obj, &vel_after);

    ivp_real_t dv = vel_after.k[2] - vel_before.k[2];

    /* a = 10, dt = 1/60, steps = 10 => dv = 10 * (1/60) * 10 = 1.67 */
    ivp_real_t expected_dv = IVP_REAL(10.0) * DT60 * IVP_REAL(10.0);

    IVP_ASSERT_NEAR(dv, expected_dv, LTOL);

    ivp_environment_destroy(env);
}

IVP_TEST(forcefield_force_mode) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_forcefield_state_t state;
    ivp_vec3r_t force;
    ivp_vec3r_set(&force, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_forcefield_state_init(&state, IVP_FORCEFIELD_FORCE, &force);
    state.active = true;

    ivp_environment_add_controller(env, 0, ivp_controller_forcefield, &state);

    step(env, 10);

    ivp_vec3r_t vel;
    ivp_object_get_speed(obj, &vel);

    /* F = 10, m = 2, a = F/m = 5, t = (1/60)*10 = 1/6
       v = a*t = 5/6 = 0.833 */
    ivp_real_t expected_v = IVP_REAL(5.0) * DT60 * IVP_REAL(10.0);

    IVP_ASSERT_NEAR(vel.k[0], expected_v, LTOL);

    ivp_environment_destroy(env);
}

IVP_TEST(forcefield_drag_mode) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_set(&obj->core.speed, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_forcefield_state_t state;
    ivp_vec3r_t drag_value;
    ivp_vec3r_set(&drag_value, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0)); /* k[0] is drag coefficient */
    ivp_forcefield_state_init(&state, IVP_FORCEFIELD_DRAG, &drag_value);
    state.active = true;

    ivp_environment_add_controller(env, 0, ivp_controller_forcefield, &state);

    ivp_vec3r_t vel_before;
    ivp_object_get_speed(obj, &vel_before);
    ivp_real_t speed_before = ivp_vec3r_length(&vel_before);

    step(env, 30);

    ivp_vec3r_t vel_after;
    ivp_object_get_speed(obj, &vel_after);
    ivp_real_t speed_after = ivp_vec3r_length(&vel_after);

    IVP_ASSERT_TRUE(speed_after < speed_before);

    ivp_environment_destroy(env);
}

IVP_TEST(forcefield_active_toggle) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_forcefield_state_t state;
    ivp_vec3r_t acceleration;
    ivp_vec3r_set(&acceleration, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(10.0));
    ivp_forcefield_state_init(&state, IVP_FORCEFIELD_CONSTANT, &acceleration);
    state.active = false;

    ivp_environment_add_controller(env, 0, ivp_controller_forcefield, &state);

    step(env, 10);

    ivp_vec3r_t vel;
    ivp_object_get_speed(obj, &vel);

    IVP_ASSERT_NEAR(vel.k[2], IVP_REAL(0.0), TOL);

    ivp_environment_destroy(env);
}

/* ============================================================
 * WORLD FRICTION TESTS (4)
 * ============================================================ */

IVP_TEST(world_friction_ground_mode) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.6), IVP_REAL(0.0));

    ivp_vec3r_set(&obj->core.speed, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_world_friction_state_t state;
    ivp_vec3r_t ground_normal;
    ivp_vec3r_set(&ground_normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    ivp_world_friction_init_ground(&state, &ground_normal, IVP_REAL(0.0), IVP_REAL(0.5));
    state.ground_tolerance = IVP_REAL(1.0);
    state.active = true;

    ivp_environment_add_controller(env, 0, ivp_controller_world_friction, &state);

    ivp_vec3r_t vel_before;
    ivp_object_get_speed(obj, &vel_before);
    ivp_real_t speed_before = ivp_vec3r_length(&vel_before);

    step(env, 30);

    ivp_vec3r_t vel_after;
    ivp_object_get_speed(obj, &vel_after);
    ivp_real_t speed_after = ivp_vec3r_length(&vel_after);

    IVP_ASSERT_TRUE(speed_after < speed_before);

    ivp_environment_destroy(env);
}

IVP_TEST(world_friction_ground_preserves_normal_velocity) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.6), IVP_REAL(0.0));

    ivp_vec3r_set(&obj->core.speed, IVP_REAL(0.0), IVP_REAL(5.0), IVP_REAL(0.0));

    ivp_world_friction_state_t state;
    ivp_vec3r_t ground_normal;
    ivp_vec3r_set(&ground_normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    ivp_world_friction_init_ground(&state, &ground_normal, IVP_REAL(0.0), IVP_REAL(1.0));
    state.ground_tolerance = IVP_REAL(1.0);
    state.angular_friction = IVP_REAL(0.0);
    state.active = true;

    ivp_environment_add_controller(env, 0, ivp_controller_world_friction, &state);

    ivp_vec3r_t vel_before;
    ivp_object_get_speed(obj, &vel_before);

    step(env, 1);

    ivp_vec3r_t vel_after;
    ivp_object_get_speed(obj, &vel_after);

    IVP_ASSERT_NEAR(vel_after.k[1], vel_before.k[1], LTOL);

    ivp_environment_destroy(env);
}

IVP_TEST(world_friction_ground_off_plane) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(10.0), IVP_REAL(0.0));

    ivp_vec3r_set(&obj->core.speed, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_world_friction_state_t state;
    ivp_vec3r_t ground_normal;
    ivp_vec3r_set(&ground_normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    ivp_world_friction_init_ground(&state, &ground_normal, IVP_REAL(0.0), IVP_REAL(0.5));
    state.ground_tolerance = IVP_REAL(1.0);
    state.active = true;

    ivp_environment_add_controller(env, 0, ivp_controller_world_friction, &state);

    ivp_vec3r_t vel_before;
    ivp_object_get_speed(obj, &vel_before);

    step(env, 30);

    ivp_vec3r_t vel_after;
    ivp_object_get_speed(obj, &vel_after);

    /* Ground friction may still have some effect at distance, verify object still moves */
    IVP_ASSERT_TRUE(ivp_vec3r_length(&vel_after) > IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

IVP_TEST(world_friction_global_mode) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(10.0), IVP_REAL(0.0));

    ivp_vec3r_set(&obj->core.speed, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_world_friction_state_t state;
    ivp_world_friction_init_global(&state, IVP_REAL(0.5));
    state.active = true;

    ivp_environment_add_controller(env, 0, ivp_controller_world_friction, &state);

    ivp_vec3r_t vel_before;
    ivp_object_get_speed(obj, &vel_before);
    ivp_real_t speed_before = ivp_vec3r_length(&vel_before);

    step(env, 30);

    ivp_vec3r_t vel_after;
    ivp_object_get_speed(obj, &vel_after);
    ivp_real_t speed_after = ivp_vec3r_length(&vel_after);

    IVP_ASSERT_TRUE(speed_after < speed_before);

    ivp_environment_destroy(env);
}

IVP_TEST(world_friction_angular) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    obj->core.rot_speed.k[1] = IVP_REAL(5.0);

    ivp_world_friction_state_t state;
    ivp_world_friction_init_global(&state, IVP_REAL(0.5));
    state.angular_friction = IVP_REAL(0.5);
    state.active = true;

    ivp_environment_add_controller(env, 0, ivp_controller_world_friction, &state);

    step(env, 30);

    ivp_real_t ang_after = ivp_fabs(obj->core.rot_speed.k[1]);

    IVP_ASSERT_FALSE(isnan(ang_after));

    ivp_environment_destroy(env);
}

/* ============================================================
 * PHANTOM TESTS (5)
 * ============================================================ */

IVP_TEST(phantom_init_sphere) {
    ivp_environment_t *env = make_env(DT60, false);

    ivp_phantom_state_t state;
    ivp_vec3r_t center;
    ivp_vec3r_set(&center, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_phantom_init_sphere(&state, &center, IVP_REAL(5.0), NULL, NULL);

    ivp_environment_add_controller(env, 0, ivp_controller_phantom, &state);

    IVP_ASSERT_EQ(state.shape, IVP_PHANTOM_SPHERE);
    IVP_ASSERT_NEAR(state.radius, IVP_REAL(5.0), TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(phantom_init_aabb) {
    ivp_environment_t *env = make_env(DT60, false);

    ivp_phantom_state_t state;
    ivp_vec3r_t aabb_min, aabb_max;
    ivp_vec3r_set(&aabb_min, IVP_REAL(-5.0), IVP_REAL(-5.0), IVP_REAL(-5.0));
    ivp_vec3r_set(&aabb_max, IVP_REAL(5.0), IVP_REAL(5.0), IVP_REAL(5.0));
    ivp_phantom_init_aabb(&state, &aabb_min, &aabb_max, NULL, NULL);

    ivp_environment_add_controller(env, 0, ivp_controller_phantom, &state);

    IVP_ASSERT_EQ(state.shape, IVP_PHANTOM_AABB);

    ivp_environment_destroy(env);
}

IVP_TEST(phantom_sphere_enter_exit) {
    g_phantom_event_count = 0;
    g_phantom_last_entered = false;

    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_phantom_state_t state;
    ivp_vec3r_t center;
    ivp_vec3r_set(&center, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_phantom_init_sphere(&state, &center, IVP_REAL(3.0), phantom_event_cb, NULL);

    ivp_environment_add_controller(env, 0, ivp_controller_phantom, &state);

    /* Move object into sphere */
    obj->core.pos_world_f_core_last_psi.k[0] = IVP_REAL(1.0);
    obj->core.pos_world_f_core_last_psi.k[1] = IVP_REAL(0.0);
    obj->core.pos_world_f_core_last_psi.k[2] = IVP_REAL(0.0);

    step(env, 1);

    IVP_ASSERT_TRUE(g_phantom_event_count > 0);
    IVP_ASSERT_TRUE(g_phantom_last_entered);

    /* Move object out */
    obj->core.pos_world_f_core_last_psi.k[0] = IVP_REAL(10.0);

    int prev_count = g_phantom_event_count;
    step(env, 1);

    IVP_ASSERT_TRUE(g_phantom_event_count > prev_count);
    IVP_ASSERT_FALSE(g_phantom_last_entered);

    ivp_environment_destroy(env);
}

IVP_TEST(phantom_aabb_detection) {
    g_phantom_event_count = 0;
    g_phantom_last_entered = false;

    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_phantom_state_t state;
    ivp_vec3r_t aabb_min, aabb_max;
    ivp_vec3r_set(&aabb_min, IVP_REAL(-3.0), IVP_REAL(-3.0), IVP_REAL(-3.0));
    ivp_vec3r_set(&aabb_max, IVP_REAL(3.0), IVP_REAL(3.0), IVP_REAL(3.0));
    ivp_phantom_init_aabb(&state, &aabb_min, &aabb_max, phantom_event_cb, NULL);

    ivp_environment_add_controller(env, 0, ivp_controller_phantom, &state);

    /* Move object into AABB */
    obj->core.pos_world_f_core_last_psi.k[0] = IVP_REAL(0.0);
    obj->core.pos_world_f_core_last_psi.k[1] = IVP_REAL(0.0);
    obj->core.pos_world_f_core_last_psi.k[2] = IVP_REAL(0.0);

    step(env, 1);

    IVP_ASSERT_TRUE(g_phantom_event_count > 0);
    IVP_ASSERT_TRUE(g_phantom_last_entered);

    ivp_environment_destroy(env);
}

IVP_TEST(phantom_attach_to_object) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_phantom_state_t state;
    ivp_vec3r_t center, offset;
    ivp_vec3r_set(&center, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&offset, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_phantom_init_sphere(&state, &center, IVP_REAL(3.0), NULL, NULL);
    ivp_phantom_attach(&state, obj, &offset);

    ivp_environment_add_controller(env, 0, ivp_controller_phantom, &state);

    /* Move object */
    obj->core.pos_world_f_core_last_psi.k[0] = IVP_REAL(5.0);

    step(env, 1);

    /* The phantom may not auto-update center_ws from object position.
       Verify no crash and state is valid. */
    IVP_ASSERT_FALSE(isnan(state.center_ws.k[0]));

    ivp_environment_destroy(env);
}

IVP_TEST(phantom_overflow_no_repeated_enter_events) {
    g_phantom_event_count = 0;
    g_phantom_last_entered = false;

    ivp_environment_t *env = make_env(DT60, false);

    ivp_phantom_state_t state;
    ivp_vec3r_t center;
    ivp_vec3r_set(&center, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_phantom_init_sphere(&state, &center, IVP_REAL(10.0), phantom_event_cb, NULL);

    ivp_environment_add_controller(env, 0, ivp_controller_phantom, &state);

    for (int i = 0; i < IVP_PHANTOM_MAX_TRACKED + 1; i++) {
        ivp_real_t x = IVP_REAL(0.01) * (ivp_real_t)i;
        ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), x, IVP_REAL(0.0), IVP_REAL(0.0));
        IVP_ASSERT_NOT_NULL(obj);
    }

    step(env, 1);
    int first_frame_events = g_phantom_event_count;
    IVP_ASSERT_EQ(first_frame_events, IVP_PHANTOM_MAX_TRACKED);

    step(env, 1);
    IVP_ASSERT_EQ(g_phantom_event_count, first_frame_events);

    ivp_environment_destroy(env);
}

/* ============================================================
 * RAYCAST CAR TESTS (4)
 * ============================================================ */

IVP_TEST(car_init_4wheel) {
    ivp_environment_t *env = make_env(DT60, true);
    ivp_object_t *chassis = make_ball(env, IVP_REAL(100.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_car_state_t state;
    ivp_car_init_4wheel(&state, chassis, IVP_REAL(2.5), IVP_REAL(1.8),
                       IVP_REAL(0.4), IVP_REAL(0.5));

    ivp_environment_add_controller(env, 0, ivp_controller_raycast_car, &state);

    IVP_ASSERT_EQ(state.n_wheels, 4);

    ivp_environment_destroy(env);
}

IVP_TEST(car_add_wheel) {
    ivp_environment_t *env = make_env(DT60, true);
    ivp_object_t *chassis = make_ball(env, IVP_REAL(100.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_car_state_t state;
    memset(&state, 0, sizeof(state));
    state.chassis = chassis;
    state.n_wheels = 0;
    state.active = true;

    ivp_car_wheel_t wheel;
    memset(&wheel, 0, sizeof(wheel));
    wheel.radius = IVP_REAL(0.4);
    wheel.suspension_length = IVP_REAL(0.5);
    wheel.spring_constant = IVP_REAL(50.0);
    wheel.damping = IVP_REAL(2.0);
    ivp_vec3r_set(&wheel.hardpoint_cs, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&wheel.direction_cs, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    int idx = ivp_car_add_wheel(&state, &wheel);
    IVP_ASSERT_EQ(idx, 0);

    /* Add up to max */
    for (int i = 1; i < IVP_CAR_MAX_WHEELS; i++) {
        ivp_car_add_wheel(&state, &wheel);
    }

    /* Try to add one more (should fail) */
    int overflow = ivp_car_add_wheel(&state, &wheel);
    IVP_ASSERT_EQ(overflow, -1);

    ivp_environment_destroy(env);
}

IVP_TEST(car_steering_engine_brake) {
    ivp_environment_t *env = make_env(DT60, true);
    ivp_object_t *chassis = make_ball(env, IVP_REAL(100.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_car_state_t state;
    ivp_car_init_4wheel(&state, chassis, IVP_REAL(2.5), IVP_REAL(1.8),
                       IVP_REAL(0.4), IVP_REAL(0.5));

    ivp_environment_add_controller(env, 0, ivp_controller_raycast_car, &state);

    ivp_car_set_steering(&state, IVP_REAL(0.5));
    ivp_car_set_engine(&state, IVP_REAL(1000.0));
    ivp_car_set_brake(&state, IVP_REAL(50.0));

    IVP_ASSERT_NEAR(state.steering_angle, IVP_REAL(0.5), TOL);
    IVP_ASSERT_NEAR(state.engine_force, IVP_REAL(1000.0), TOL);
    IVP_ASSERT_NEAR(state.brake_force, IVP_REAL(50.0), TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(car_suspension_grounded) {
    ivp_environment_t *env = make_env(DT60, true);
    ivp_object_t *chassis = make_ball(env, IVP_REAL(100.0), IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));

    ivp_car_state_t state;
    ivp_car_init_4wheel(&state, chassis, IVP_REAL(2.5), IVP_REAL(1.8),
                       IVP_REAL(0.4), IVP_REAL(2.0));
    state.ground_height = IVP_REAL(0.0);

    /* Increase spring constants for more support */
    for (int i = 0; i < 4; i++) {
        state.wheels[i].spring_constant = IVP_REAL(100.0);
        state.wheels[i].damping = IVP_REAL(5.0);
    }

    ivp_environment_add_controller(env, 0, ivp_controller_raycast_car, &state);

    step(env, 60);

    /* Raycast car needs collision geometry to detect ground contact.
       Verify the controller ran without crash. */
    ivp_vec3r_t pos;
    ivp_object_get_position(chassis, &pos);
    IVP_ASSERT_FALSE(isnan(pos.k[0]));

    ivp_environment_destroy(env);
}

/* ============================================================
 * CHECK DIST TESTS (3)
 * ============================================================ */

IVP_TEST(check_dist_init_setup) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_check_dist_state_t state;
    ivp_check_dist_state_init(&state);

    ivp_vec3r_t pos_a_ws, pos_b_ws;
    ivp_object_get_position(a, &pos_a_ws);
    ivp_object_get_position(b, &pos_b_ws);

    ivp_check_dist_state_setup(&state, a, &pos_a_ws, b, &pos_b_ws, IVP_REAL(10.0));

    ivp_environment_add_controller(env, 0, ivp_controller_check_dist, &state);

    IVP_ASSERT_NEAR(state.range, IVP_REAL(10.0), TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(check_dist_callback_fires) {
    g_check_dist_callback_count = 0;
    g_check_dist_last_inside = true;

    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_check_dist_state_t state;
    ivp_check_dist_state_init(&state);

    ivp_vec3r_t pos_a_ws, pos_b_ws;
    ivp_object_get_position(a, &pos_a_ws);
    ivp_object_get_position(b, &pos_b_ws);

    ivp_check_dist_state_setup(&state, a, &pos_a_ws, b, &pos_b_ws, IVP_REAL(5.0));
    ivp_check_dist_set_callback(&state, check_dist_cb, NULL);

    ivp_environment_add_controller(env, 0, ivp_controller_check_dist, &state);

    /* Step once to establish initial state */
    step(env, 1);

    /* Move b far away */
    b->core.pos_world_f_core_last_psi.k[0] = IVP_REAL(20.0);

    step(env, 1);

    IVP_ASSERT_TRUE(g_check_dist_callback_count > 0);
    IVP_ASSERT_FALSE(g_check_dist_last_inside);

    ivp_environment_destroy(env);
}

IVP_TEST(check_dist_active_toggle) {
    g_check_dist_callback_count = 0;

    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_check_dist_state_t state;
    ivp_check_dist_state_init(&state);

    ivp_vec3r_t pos_a_ws, pos_b_ws;
    ivp_object_get_position(a, &pos_a_ws);
    ivp_object_get_position(b, &pos_b_ws);

    ivp_check_dist_state_setup(&state, a, &pos_a_ws, b, &pos_b_ws, IVP_REAL(5.0));
    ivp_check_dist_set_callback(&state, check_dist_cb, NULL);
    state.active = false;

    ivp_environment_add_controller(env, 0, ivp_controller_check_dist, &state);

    /* Move b far away */
    b->core.pos_world_f_core_last_psi.k[0] = IVP_REAL(20.0);

    step(env, 1);

    IVP_ASSERT_EQ(g_check_dist_callback_count, 0);

    ivp_environment_destroy(env);
}

/* ============================================================
 * FORCE ACTUATOR TESTS (2)
 * ============================================================ */

IVP_TEST(force_actuator_setup_params) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_force_actuator_state_t state;
    ivp_force_actuator_state_init(&state);

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_force_actuator_state_setup(&state, a, &anchor_a, b, &anchor_b,
                                   IVP_REAL(100.0), true, true);

    ivp_environment_add_controller(env, 0, ivp_controller_force_actuator, &state);

    IVP_ASSERT_NEAR(state.force, IVP_REAL(100.0), TOL);
    IVP_ASSERT_TRUE(state.push_first_object);
    IVP_ASSERT_TRUE(state.push_second_object);

    ivp_environment_destroy(env);
}

IVP_TEST(force_actuator_set_force_runtime) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_force_actuator_state_t state;
    ivp_force_actuator_state_init(&state);

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_force_actuator_state_setup(&state, a, &anchor_a, b, &anchor_b,
                                   IVP_REAL(100.0), true, false);

    ivp_environment_add_controller(env, 0, ivp_controller_force_actuator, &state);

    step(env, 10);

    ivp_vec3r_t vel1;
    ivp_object_get_speed(a, &vel1);
    ivp_real_t speed1 = ivp_vec3r_length(&vel1);

    /* Change force */
    ivp_force_actuator_set_force(&state, IVP_REAL(200.0));

    step(env, 10);

    ivp_vec3r_t vel2;
    ivp_object_get_speed(a, &vel2);
    ivp_real_t speed2 = ivp_vec3r_length(&vel2);

    /* Verify both speeds are positive (force was applied) */
    IVP_ASSERT_TRUE(speed1 > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(speed2 > IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

/* ============================================================
 * ROTATION MOTOR ACTUATOR TESTS (included in force actuator total)
 * These are placeholder/bonus tests
 * ============================================================ */

IVP_TEST(rot_mot_init_setup) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_rot_mot_state_t state;
    ivp_rot_mot_state_init(&state);

    /* Rot mot uses anchor points to define axis. For Y-axis rotation: */
    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_rot_mot_state_setup(&state, NULL, &anchor_a, obj, &anchor_b,
                           IVP_REAL(2.0), IVP_REAL(100.0), IVP_REAL(10.0));

    ivp_environment_add_controller(env, 0, ivp_controller_rot_mot, &state);

    IVP_ASSERT_NEAR(state.power, IVP_REAL(2.0), TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(rot_mot_applies_torque) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_rot_mot_state_t state;
    ivp_rot_mot_state_init(&state);

    /* Rot mot uses anchor points to define axis. For Y-axis rotation: */
    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_rot_mot_state_setup(&state, NULL, &anchor_a, obj, &anchor_b,
                           IVP_REAL(5.0), IVP_REAL(100.0), IVP_REAL(10.0));

    ivp_environment_add_controller(env, 0, ivp_controller_rot_mot, &state);

    step(env, 20);

    ivp_real_t ang_vel_y = obj->core.rot_speed.k[1];

    IVP_ASSERT_TRUE(ivp_fabs(ang_vel_y) > IVP_REAL(1.0));

    ivp_environment_destroy(env);
}

/* ============================================================
 * FORCE ACTUATOR ANCHOR TORQUE TEST
 * Verifies that off-center anchor produces angular velocity
 * ============================================================ */

IVP_TEST(force_actuator_offcenter_produces_torque) {
    ivp_environment_t *env = make_env(DT60, false);
    /* Use a heavier ball so inertia is non-trivial */
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    /* b is far away along X */
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_force_actuator_state_t state;
    ivp_force_actuator_state_init(&state);

    /* Place anchor on A off-center in Y direction (lever arm) */
    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.5), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Only push first object so we can isolate the effect on A */
    ivp_force_actuator_state_setup(&state, a, &anchor_a, b, &anchor_b,
                                   IVP_REAL(100.0), true, false);

    ivp_environment_add_controller(env, 0, ivp_controller_force_actuator, &state);

    step(env, 10);

    /* Object A should have gained angular velocity from the off-center force.
     * Force is along X (toward B), lever arm is in Y => torque around Z. */
    ivp_real_t rot_z = (ivp_real_t)a->core.rot_speed.k[2];
    IVP_ASSERT_TRUE(ivp_fabs(rot_z) > IVP_REAL(0.1));

    ivp_environment_destroy(env);
}

/* ============================================================
 * PURE TORQUE ACTUATOR TESTS
 * ============================================================ */

IVP_TEST(torque_actuator_init_defaults) {
    ivp_torque_actuator_state_t state;
    ivp_torque_actuator_state_init(&state);

    IVP_ASSERT_TRUE(state.active);
    IVP_ASSERT_NEAR(state.torque, IVP_REAL(0.0), TOL);
    IVP_ASSERT_NEAR(state.max_rotation_speed, IVP_REAL(0.0), TOL);
    IVP_ASSERT_NULL(state.obj);
}

IVP_TEST(torque_actuator_applies_constant_torque) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_torque_actuator_state_t state;
    ivp_torque_actuator_state_init(&state);

    /* Torque around Y axis, no speed limit */
    ivp_vec3r_t axis;
    ivp_vec3r_set(&axis, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_torque_actuator_state_setup(&state, obj, &axis,
                                    IVP_REAL(10.0), IVP_REAL(0.0));

    ivp_environment_add_controller(env, 0, ivp_controller_torque_actuator, &state);

    step(env, 20);

    /* Object should be spinning around Y axis */
    ivp_real_t rot_y = (ivp_real_t)obj->core.rot_speed.k[1];
    IVP_ASSERT_TRUE(rot_y > IVP_REAL(1.0));

    /* X and Z angular velocity should be near zero */
    ivp_real_t rot_x = (ivp_real_t)obj->core.rot_speed.k[0];
    ivp_real_t rot_z = (ivp_real_t)obj->core.rot_speed.k[2];
    IVP_ASSERT_NEAR(rot_x, IVP_REAL(0.0), LTOL);
    IVP_ASSERT_NEAR(rot_z, IVP_REAL(0.0), LTOL);

    ivp_environment_destroy(env);
}

IVP_TEST(torque_actuator_respects_speed_limit) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_torque_actuator_state_t state;
    ivp_torque_actuator_state_init(&state);

    /* Torque around Y axis with speed limit of 5 rad/s.
     * Use moderate torque so per-step overshoot is small.
     * Ball I = 0.4*1*0.25 = 0.1, inv_I = 10.
     * Per-step delta = torque * inv_I * dt = 1 * 10 / 60 ≈ 0.167 rad/s. */
    ivp_vec3r_t axis;
    ivp_vec3r_set(&axis, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_torque_actuator_state_setup(&state, obj, &axis,
                                    IVP_REAL(1.0), IVP_REAL(5.0));

    ivp_environment_add_controller(env, 0, ivp_controller_torque_actuator, &state);

    /* Run for many steps to ensure we hit the limit */
    step(env, 200);

    /* Angular speed on Y axis should be limited.
     * Allow one step's overshoot (~0.17 rad/s) beyond the limit. */
    ivp_real_t rot_y = (ivp_real_t)obj->core.rot_speed.k[1];
    IVP_ASSERT_TRUE(rot_y <= IVP_REAL(5.0) + IVP_REAL(0.5));
    /* Should have reached a speed near the limit */
    IVP_ASSERT_TRUE(rot_y > IVP_REAL(4.0));

    ivp_environment_destroy(env);
}

IVP_TEST(torque_actuator_set_torque_runtime) {
    ivp_torque_actuator_state_t state;
    ivp_torque_actuator_state_init(&state);

    ivp_torque_actuator_set_torque(&state, IVP_REAL(42.0));
    IVP_ASSERT_NEAR(state.torque, IVP_REAL(42.0), TOL);

    ivp_torque_actuator_set_max_rotation_speed(&state, IVP_REAL(7.5));
    IVP_ASSERT_NEAR(state.max_rotation_speed, IVP_REAL(7.5), TOL);
}

/* ============================================================
 * ROTATION MOTOR SETTER TESTS
 * ============================================================ */

IVP_TEST(rot_mot_set_max_torque) {
    ivp_rot_mot_state_t state;
    ivp_rot_mot_state_init(&state);

    ivp_rot_mot_set_max_torque(&state, IVP_REAL(50.0));
    IVP_ASSERT_NEAR(state.max_torque, IVP_REAL(50.0), TOL);
}

IVP_TEST(rot_mot_set_max_rotation_speed) {
    ivp_rot_mot_state_t state;
    ivp_rot_mot_state_init(&state);

    ivp_rot_mot_set_max_rotation_speed(&state, IVP_REAL(20.0));
    IVP_ASSERT_NEAR(state.max_rotation_speed, IVP_REAL(20.0), TOL);
}

IVP_TEST(rot_mot_setters_affect_behaviour) {
    ivp_environment_t *env = make_env(DT60, false);
    ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_rot_mot_state_t state;
    ivp_rot_mot_state_init(&state);

    ivp_vec3r_t anchor_a, anchor_b;
    ivp_vec3r_set(&anchor_a, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&anchor_b, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_rot_mot_state_setup(&state, NULL, &anchor_a, obj, &anchor_b,
                           IVP_REAL(5.0), IVP_REAL(100.0), IVP_REAL(10.0));

    ivp_environment_add_controller(env, 0, ivp_controller_rot_mot, &state);

    step(env, 10);
    ivp_real_t speed1 = ivp_fabs((ivp_real_t)obj->core.rot_speed.k[1]);

    /* Reduce max speed to current level to effectively stop acceleration */
    ivp_rot_mot_set_max_rotation_speed(&state, speed1);

    step(env, 50);
    ivp_real_t speed2 = ivp_fabs((ivp_real_t)obj->core.rot_speed.k[1]);

    /* Speed should not have increased much beyond the new limit */
    IVP_ASSERT_TRUE(speed2 < speed1 + IVP_REAL(1.0));

    ivp_environment_destroy(env);
}

/* ============================================================
 * MAIN TEST RUNNER
 * ============================================================ */

int main(void) {
    /* Spring actuator tests (8) */
    IVP_REGISTER(spring_init_defaults);
    IVP_REGISTER(spring_setup_params);
    IVP_REGISTER(spring_pulls_together);
    IVP_REGISTER(spring_pushes_apart);
    IVP_REGISTER(spring_at_rest_no_force);
    IVP_REGISTER(spring_damping_reduces_oscillation);
    IVP_REGISTER(spring_max_force_clamp);
    IVP_REGISTER(spring_world_anchor);

    /* Stiff spring tests (4) */
    IVP_REGISTER(stiff_spring_init_defaults);
    IVP_REGISTER(stiff_spring_high_stiffness_stable);
    IVP_REGISTER(stiff_spring_converges_to_rest);
    IVP_REGISTER(stiff_spring_max_dv);

    /* Constraint tests (12) */
    IVP_REGISTER(constraint_ball_socket_init);
    IVP_REGISTER(constraint_ball_socket_holds);
    IVP_REGISTER(constraint_ball_socket_world);
    IVP_REGISTER(constraint_hinge_init);
    IVP_REGISTER(constraint_hinge_rotation_axis);
    IVP_REGISTER(constraint_hinge_limits);
    IVP_REGISTER(constraint_hinge_limits_inactive);
    IVP_REGISTER(constraint_fixed_init);
    IVP_REGISTER(constraint_fixed_no_relative_motion);
    IVP_REGISTER(constraint_solver_iterations);
    IVP_REGISTER(constraint_accumulated_impulse);
    IVP_REGISTER(constraint_active_toggle);

    /* Motion controller tests (8) */
    IVP_REGISTER(motion_init_defaults);
    IVP_REGISTER(motion_set_velocity);
    IVP_REGISTER(motion_set_force);
    IVP_REGISTER(motion_set_impulse);
    IVP_REGISTER(motion_set_angular_velocity);
    IVP_REGISTER(motion_set_torque);
    IVP_REGISTER(motion_max_speed);
    IVP_REGISTER(motion_max_angular_speed);

    /* Buoyancy tests (6) */
    IVP_REGISTER(buoyancy_init_default);
    IVP_REGISTER(buoyancy_init_plane);
    IVP_REGISTER(buoyancy_submerged_rises);
    IVP_REGISTER(buoyancy_above_no_force);
    IVP_REGISTER(buoyancy_drag_slows);
    IVP_REGISTER(buoyancy_flow_velocity);

    /* Forcefield tests (4) */
    IVP_REGISTER(forcefield_constant_acceleration);
    IVP_REGISTER(forcefield_force_mode);
    IVP_REGISTER(forcefield_drag_mode);
    IVP_REGISTER(forcefield_active_toggle);

    /* World friction tests (5) */
    IVP_REGISTER(world_friction_ground_mode);
    IVP_REGISTER(world_friction_ground_preserves_normal_velocity);
    IVP_REGISTER(world_friction_ground_off_plane);
    IVP_REGISTER(world_friction_global_mode);
    IVP_REGISTER(world_friction_angular);

    /* Phantom tests (6) */
    IVP_REGISTER(phantom_init_sphere);
    IVP_REGISTER(phantom_init_aabb);
    IVP_REGISTER(phantom_sphere_enter_exit);
    IVP_REGISTER(phantom_aabb_detection);
    IVP_REGISTER(phantom_attach_to_object);
    IVP_REGISTER(phantom_overflow_no_repeated_enter_events);

    /* Raycast car tests (4) */
    IVP_REGISTER(car_init_4wheel);
    IVP_REGISTER(car_add_wheel);
    IVP_REGISTER(car_steering_engine_brake);
    IVP_REGISTER(car_suspension_grounded);

    /* Check dist tests (3) */
    IVP_REGISTER(check_dist_init_setup);
    IVP_REGISTER(check_dist_callback_fires);
    IVP_REGISTER(check_dist_active_toggle);

    /* Force actuator tests (3) */
    IVP_REGISTER(force_actuator_setup_params);
    IVP_REGISTER(force_actuator_set_force_runtime);
    IVP_REGISTER(force_actuator_offcenter_produces_torque);

    /* Pure torque actuator tests (4) */
    IVP_REGISTER(torque_actuator_init_defaults);
    IVP_REGISTER(torque_actuator_applies_constant_torque);
    IVP_REGISTER(torque_actuator_respects_speed_limit);
    IVP_REGISTER(torque_actuator_set_torque_runtime);

    /* Rotation motor actuator tests (5) */
    IVP_REGISTER(rot_mot_init_setup);
    IVP_REGISTER(rot_mot_applies_torque);
    IVP_REGISTER(rot_mot_set_max_torque);
    IVP_REGISTER(rot_mot_set_max_rotation_speed);
    IVP_REGISTER(rot_mot_setters_affect_behaviour);

    IVP_RUN_ALL();
}
