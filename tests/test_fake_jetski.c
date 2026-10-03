/* IVP fake jetski controller tests */

#include "ivp_test.h"

#include <ivp/controllers/ivp_controller_fake_jetski.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>

#include <math.h>
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
        t.rot_inertia_is_factor = false; /* absolute inertia */
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

/* reference do_raycasts: ground plane y = 0 (normal pointing up, -y) */
static void ground_plane_raycast(void *ctx, const ivp_event_sim_t *es, int n_wheels,
                                 const ivp_ray_solver_template_t *rays,
                                 ivp_ray_hit_t *hits_out, float *friction_out) {
    (void)es;
    ivp_object_t *ground = (ivp_object_t *)ctx;
    for (int i = 0; i < n_wheels; ++i) {
        memset(&hits_out[i], 0, sizeof(hits_out[i]));
        friction_out[i] = 1.0f;
        const ivp_real_t denom = rays[i].ray_normized_direction.k[1];
        if (ivp_fabs(denom) <= IVP_REAL_EPS) continue;
        const ivp_real_t t = (IVP_REAL(0.0) - rays[i].ray_start_point.k[1]) / denom;
        if (t < IVP_REAL(0.0) || t > rays[i].ray_length) continue;
        ++g_raycast_hits;
        hits_out[i].hit_object = ground;
        hits_out[i].hit_distance = t;
        ivp_vec3r_set(&hits_out[i].hit_normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    }
}

/* IVP_Template_Car_System(4, 2) with wheels at (+-w, 0, +-l) */
static void make_template(ivp_car_system_template_t *t, ivp_object_t *body,
                          ivp_real_t length, ivp_real_t width) {
    ivp_car_system_template_init(t, 4, 2);
    t->car_body = body;
    for (int i = 0; i < 4; ++i) {
        const float wx = (float)((i & 1) ? width * IVP_REAL(0.5) : -width * IVP_REAL(0.5));
        const float wz = (float)((i & 2) ? -length * IVP_REAL(0.5) : length * IVP_REAL(0.5));
        t->wheel_pos_Bos[i] = ivp_vec3f(wx, 0.0f, wz);
        t->trace_pos_Bos[i] = ivp_vec3f(wx, 0.0f, wz);
        t->wheel_radius[i] = (ivp_real_t)0.3f;
        t->spring_constant[i] = IVP_REAL(50000.0);
        t->spring_damping[i] = IVP_REAL(3000.0);
        t->spring_damping_compression[i] = IVP_REAL(3000.0);
        t->spring_pre_tension[i] = (ivp_real_t)-0.3f;
    }
    t->stabilizer_constant[0] = IVP_REAL(500.0);
    t->stabilizer_constant[1] = IVP_REAL(500.0);
    t->wheel_max_rotation_speed[0] = IVP_REAL(50.0);
    t->wheel_max_rotation_speed[1] = IVP_REAL(50.0);
}

IVP_TEST(jetski_init_defaults) {
    ivp_environment_t *env = make_env(DT60);
    ivp_object_t *body = make_ball(env, IVP_REAL(50.0), IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_car_system_template_t t;
    make_template(&t, body, IVP_REAL(2.0), IVP_REAL(1.0));
    ivp_jetski_state_t state;
    IVP_ASSERT_TRUE(ivp_jetski_init(&state, &t));

    IVP_ASSERT_EQ(state.n_wheels, 4);
    IVP_ASSERT_EQ(state.n_axis, 2);
    IVP_ASSERT_EQ(state.wheels_per_axis, 2);
    IVP_ASSERT_TRUE(state.wheels[0].wheel_is_fixed); /* reference: all wheels start fixed */
    IVP_ASSERT_NEAR(state.wheels[0].spring_constant, 50000.0f, TOL);
    IVP_ASSERT_NEAR(state.wheels[0].spring_len, 0.3f, TOL); /* -pre_tension */
    IVP_ASSERT_NEAR(state.steering_rate, 0.005f, TOL);
    IVP_ASSERT_NEAR(ivp_jetski_get_orig_axles_distance(&state), 2.0, TOL);

    ivp_jetski_destroy(&state);
    ivp_environment_destroy(env);
}

IVP_TEST(jetski_steering_front_wheels_only) {
    ivp_environment_t *env = make_env(DT60);
    ivp_object_t *body = make_ball(env, IVP_REAL(50.0), IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_car_system_template_t t;
    make_template(&t, body, IVP_REAL(2.0), IVP_REAL(1.0));
    ivp_jetski_state_t state;
    IVP_ASSERT_TRUE(ivp_jetski_init(&state, &t));

    ivp_jetski_do_steering(&state, 0.35f);

    IVP_ASSERT_TRUE(fabsf(state.wheels[IVP_JETSKI_FRONT_LEFT].axis_direction_cs.k[2]) > 1e-4f);
    IVP_ASSERT_TRUE(fabsf(state.wheels[IVP_JETSKI_FRONT_RIGHT].axis_direction_cs.k[2]) > 1e-4f);
    IVP_ASSERT_NEAR(state.wheels[IVP_JETSKI_REAR_LEFT].axis_direction_cs.k[2], 0.0f, 1e-4f);
    IVP_ASSERT_NEAR(state.wheels[IVP_JETSKI_REAR_RIGHT].axis_direction_cs.k[2], 0.0f, 1e-4f);

    ivp_jetski_destroy(&state);
    ivp_environment_destroy(env);
}

IVP_TEST(jetski_booster_changes_body_speed) {
    ivp_environment_t *env = make_env(DT60);
    ivp_object_t *body = make_ball(env, IVP_REAL(50.0), IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));

    ivp_car_system_template_t t;
    make_template(&t, body, IVP_REAL(2.0), IVP_REAL(1.0));
    ivp_jetski_state_t state;
    IVP_ASSERT_TRUE(ivp_jetski_init(&state, &t)); /* registers the controller */
    ivp_jetski_activate_booster(&state, 8.0f, 0.5f, 0.0f);

    step(env, 3);

    IVP_ASSERT_TRUE(ivp_jetski_get_body_speed(&state, state.index_z) > 0.0);

    ivp_jetski_destroy(&state);
    ivp_environment_destroy(env);
}

IVP_TEST(jetski_grounded_torque_path) {
    ivp_environment_t *env = make_env(DT60);
    ivp_object_t *body = make_ball(env, IVP_REAL(60.0), IVP_REAL(0.0), IVP_REAL(-0.5), IVP_REAL(0.0));

    ivp_car_system_template_t t;
    make_template(&t, body, IVP_REAL(2.4), IVP_REAL(1.4));
    ivp_jetski_state_t state;
    IVP_ASSERT_TRUE(ivp_jetski_init(&state, &t));
    /* the hit object: the static object, whose object space is world space */
    ivp_jetski_set_raycast(&state, ground_plane_raycast, ivp_environment_get_static_object(env));
    g_raycast_hits = 0;

    for (int i = 0; i < 4; ++i) {
        ivp_jetski_fix_wheel(&state, (ivp_jetski_wheel_pos_t)i, false);
        ivp_jetski_change_wheel_torque(&state, (ivp_jetski_wheel_pos_t)i, 120.0f);
    }

    step(env, 1);

    IVP_ASSERT_TRUE(g_raycast_hits > 0);
    step(env, 10);
    IVP_ASSERT_TRUE(fabs(ivp_jetski_get_body_speed(&state, state.index_z)) > 0.0);

    ivp_jetski_destroy(&state);
    ivp_environment_destroy(env);
}

int main(void) {
    IVP_REGISTER(jetski_init_defaults);
    IVP_REGISTER(jetski_steering_front_wheels_only);
    IVP_REGISTER(jetski_booster_changes_body_speed);
    IVP_REGISTER(jetski_grounded_torque_path);
    IVP_RUN_ALL();
}
