/* IVP fake jetski controller tests */

#include "ivp_test.h"

#include <ivp/controllers/ivp_controller_fake_jetski.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>

#include <string.h>

#ifdef IVP_REAL_DOUBLE
  #define TOL 1e-6
#else
  #define TOL 1e-3f
#endif

#define DT60 (IVP_REAL(1.0) / IVP_REAL(60.0))

static ivp_environment_t *make_env(ivp_real_t dt) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.81), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t mass,
                               ivp_real_t x, ivp_real_t y, ivp_real_t z) {
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = mass;
    {
        const ivp_real_t inertia = IVP_REAL(0.4) * mass * IVP_REAL(0.25); /* r=0.5 */
        ivp_vec3r_set(&t.rot_inertia, inertia, inertia, inertia);
    }
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&t.position, x, y, z);
    t.orientation = ivp_quatr_identity();
    return ivp_environment_create_object(env, &t, NULL);
}

static void step(ivp_environment_t *env, int n) {
    for (int i = 0; i < n; ++i) {
        ivp_environment_simulate_one_psi(env);
    }
}

static int g_raycast_hits = 0;

static void ground_plane_raycast(void *ctx,
                                 const ivp_vec3r_t *start_ws,
                                 const ivp_vec3r_t *dir_ws,
                                 ivp_real_t max_dist,
                                 ivp_jetski_raycast_result_t *result) {
    (void)ctx;
    memset(result, 0, sizeof(*result));

    const ivp_real_t denom = dir_ws->k[1];
    if (ivp_fabs(denom) <= IVP_REAL_EPS) {
        return;
    }

    const ivp_real_t t = (IVP_REAL(0.0) - start_ws->k[1]) / denom;
    if (t < IVP_REAL(0.0) || t > max_dist) {
        return;
    }

    result->hit = true;
    ++g_raycast_hits;
    result->distance = t;
    result->hit_point.k[0] = start_ws->k[0] + dir_ws->k[0] * t;
    result->hit_point.k[1] = start_ws->k[1] + dir_ws->k[1] * t;
    result->hit_point.k[2] = start_ws->k[2] + dir_ws->k[2] * t;
    ivp_vec3r_set(&result->hit_normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    result->friction = IVP_REAL(1.0);
}

IVP_TEST(jetski_init_defaults) {
    ivp_environment_t *env = make_env(DT60);
    ivp_object_t *body = make_ball(env, IVP_REAL(50.0), IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_jetski_state_t state;
    ivp_jetski_init(&state, body, IVP_REAL(2.0), IVP_REAL(1.0), IVP_REAL(1.0));

    IVP_ASSERT_EQ(state.n_wheels, 4);
    IVP_ASSERT_EQ(state.n_axis, 2);
    IVP_ASSERT_EQ(state.wheels_per_axis, 2);
    IVP_ASSERT_TRUE(state.wheels[0].wheel_is_fixed);
    IVP_ASSERT_NEAR(state.wheels[0].spring_constant, IVP_REAL(50000.0), TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(jetski_steering_front_wheels_only) {
    ivp_environment_t *env = make_env(DT60);
    ivp_object_t *body = make_ball(env, IVP_REAL(50.0), IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_jetski_state_t state;
    ivp_jetski_init(&state, body, IVP_REAL(2.0), IVP_REAL(1.0), IVP_REAL(1.0));

    ivp_jetski_do_steering(&state, IVP_REAL(0.35));

    IVP_ASSERT_TRUE(ivp_fabs(state.wheels[IVP_JETSKI_FRONT_LEFT].axis_direction_cs.k[2]) > IVP_REAL(1e-4));
    IVP_ASSERT_TRUE(ivp_fabs(state.wheels[IVP_JETSKI_FRONT_RIGHT].axis_direction_cs.k[2]) > IVP_REAL(1e-4));
    IVP_ASSERT_NEAR(state.wheels[IVP_JETSKI_REAR_LEFT].axis_direction_cs.k[2], IVP_REAL(0.0), IVP_REAL(1e-4));
    IVP_ASSERT_NEAR(state.wheels[IVP_JETSKI_REAR_RIGHT].axis_direction_cs.k[2], IVP_REAL(0.0), IVP_REAL(1e-4));

    ivp_environment_destroy(env);
}

IVP_TEST(jetski_booster_changes_body_speed) {
    ivp_environment_t *env = make_env(DT60);
    ivp_object_t *body = make_ball(env, IVP_REAL(50.0), IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));

    ivp_jetski_state_t state;
    ivp_jetski_init(&state, body, IVP_REAL(2.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_jetski_activate_booster(&state, IVP_REAL(8.0), IVP_REAL(0.5), IVP_REAL(0.0));

    ivp_environment_add_controller(env, 0, ivp_controller_fake_jetski, &state);
    step(env, 3);

    IVP_ASSERT_TRUE(ivp_jetski_get_body_speed(&state, state.index_z) > IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

IVP_TEST(jetski_grounded_torque_path) {
    ivp_environment_t *env = make_env(DT60);
    ivp_object_t *body = make_ball(env, IVP_REAL(60.0), IVP_REAL(0.0), IVP_REAL(-0.6), IVP_REAL(0.0));

    ivp_jetski_state_t state;
    ivp_jetski_init(&state, body, IVP_REAL(2.4), IVP_REAL(1.4), IVP_REAL(1.0));
    ivp_jetski_set_raycast(&state, ground_plane_raycast, NULL);
    g_raycast_hits = 0;

    for (int i = 0; i < 4; ++i) {
        ivp_jetski_fix_wheel(&state, (ivp_jetski_wheel_pos_t)i, false);
        ivp_jetski_change_wheel_torque(&state, (ivp_jetski_wheel_pos_t)i, IVP_REAL(120.0));
    }

    ivp_environment_add_controller(env, 0, ivp_controller_fake_jetski, &state);
    step(env, 1);

    IVP_ASSERT_TRUE(g_raycast_hits > 0);
    step(env, 10);
    IVP_ASSERT_TRUE(ivp_fabs(ivp_jetski_get_body_speed(&state, state.index_z)) > IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

int main(void) {
    IVP_REGISTER(jetski_init_defaults);
    IVP_REGISTER(jetski_steering_front_wheels_only);
    IVP_REGISTER(jetski_booster_changes_body_speed);
    IVP_REGISTER(jetski_grounded_torque_path);
    IVP_RUN_ALL();
}
