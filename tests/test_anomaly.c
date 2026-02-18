#include "ivp_test.h"
#include <ivp/dynamics/ivp_anomaly_manager.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <math.h>

/* Callback tracking for tests */
static int s_cb_count = 0;
static ivp_anomaly_type_t s_cb_type = 0;
static void anomaly_cb(void *ctx, ivp_object_t *obj, ivp_anomaly_type_t type, const char *msg) {
    (void)ctx;
    (void)obj;
    (void)msg;
    s_cb_count++;
    s_cb_type = type;
}

/* Helper: create environment with zero gravity for testing */
static ivp_environment_t *create_test_env(void) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = IVP_REAL(1.0) / IVP_REAL(60.0);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

/* Helper: create a basic test object */
static ivp_object_t *create_test_object(ivp_environment_t *env) {
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t.orientation = ivp_quatr_identity();
    ivp_object_t *obj = ivp_environment_create_object(env, &t, NULL);
    obj->core.upper_limit_radius = IVP_REAL(0.5);
    return obj;
}

IVP_TEST(anomaly_detect_nan_velocity) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj = create_test_object(env);

    ivp_anomaly_manager_t mgr;
    ivp_anomaly_config_t cfg;
    ivp_anomaly_config_init(&cfg);
    cfg.auto_correct = false;
    ivp_anomaly_manager_init(&mgr, &cfg);

    /* Inject NaN into velocity */
    obj->core.speed.k[0] = NAN;

    int result = ivp_anomaly_check_object(&mgr, obj);

    IVP_ASSERT(result & IVP_ANOMALY_NAN_VELOCITY);
    IVP_ASSERT(mgr.anomalies_detected > 0);

    ivp_environment_destroy(env);
}

IVP_TEST(anomaly_detect_nan_position) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj = create_test_object(env);

    ivp_anomaly_manager_t mgr;
    ivp_anomaly_config_t cfg;
    ivp_anomaly_config_init(&cfg);
    cfg.auto_correct = false;
    ivp_anomaly_manager_init(&mgr, &cfg);

    /* Inject NaN into position */
    obj->core.pos_world_f_core_last_psi.k[1] = NAN;

    int result = ivp_anomaly_check_object(&mgr, obj);

    IVP_ASSERT(result & IVP_ANOMALY_NAN_POSITION);
    IVP_ASSERT(mgr.anomalies_detected > 0);

    ivp_environment_destroy(env);
}

IVP_TEST(anomaly_detect_nan_angular) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj = create_test_object(env);

    ivp_anomaly_manager_t mgr;
    ivp_anomaly_config_t cfg;
    ivp_anomaly_config_init(&cfg);
    cfg.auto_correct = false;
    ivp_anomaly_manager_init(&mgr, &cfg);

    /* Inject NaN into angular velocity */
    obj->core.rot_speed.k[2] = NAN;

    int result = ivp_anomaly_check_object(&mgr, obj);

    IVP_ASSERT(result & IVP_ANOMALY_NAN_VELOCITY);
    IVP_ASSERT(mgr.anomalies_detected > 0);

    ivp_environment_destroy(env);
}

IVP_TEST(anomaly_detect_excessive_speed) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj = create_test_object(env);

    ivp_anomaly_manager_t mgr;
    ivp_anomaly_config_t cfg;
    ivp_anomaly_config_init(&cfg);
    cfg.auto_correct = false;
    cfg.max_speed = IVP_REAL(1000.0);
    ivp_anomaly_manager_init(&mgr, &cfg);

    /* Set excessive linear speed */
    obj->core.speed.k[0] = IVP_REAL(5000.0);

    int result = ivp_anomaly_check_object(&mgr, obj);

    IVP_ASSERT(result & IVP_ANOMALY_EXCESSIVE_SPEED);
    IVP_ASSERT(mgr.anomalies_detected > 0);

    ivp_environment_destroy(env);
}

IVP_TEST(anomaly_detect_excessive_angular) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj = create_test_object(env);

    ivp_anomaly_manager_t mgr;
    ivp_anomaly_config_t cfg;
    ivp_anomaly_config_init(&cfg);
    cfg.auto_correct = false;
    cfg.max_angular_speed = IVP_REAL(100.0);
    ivp_anomaly_manager_init(&mgr, &cfg);

    /* Set excessive angular speed */
    obj->core.rot_speed.k[0] = IVP_REAL(500.0);
    obj->core.rot_speed.k[1] = IVP_REAL(500.0);
    obj->core.rot_speed.k[2] = IVP_REAL(500.0);

    int result = ivp_anomaly_check_object(&mgr, obj);

    IVP_ASSERT(result & IVP_ANOMALY_EXCESSIVE_SPEED);
    IVP_ASSERT(mgr.anomalies_detected > 0);

    ivp_environment_destroy(env);
}

IVP_TEST(anomaly_callback_fires) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj = create_test_object(env);

    /* Reset callback tracking */
    s_cb_count = 0;
    s_cb_type = 0;

    ivp_anomaly_manager_t mgr;
    ivp_anomaly_config_t cfg;
    ivp_anomaly_config_init(&cfg);
    cfg.auto_correct = false;
    cfg.callback = anomaly_cb;
    cfg.callback_ctx = NULL;
    ivp_anomaly_manager_init(&mgr, &cfg);

    /* Inject NaN to trigger callback */
    obj->core.speed.k[0] = NAN;

    ivp_anomaly_check_object(&mgr, obj);

    IVP_ASSERT(s_cb_count > 0);
    IVP_ASSERT(s_cb_type & IVP_ANOMALY_NAN_VELOCITY);

    ivp_environment_destroy(env);
}

IVP_TEST(anomaly_auto_correct_zeros_nan) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj = create_test_object(env);

    ivp_anomaly_manager_t mgr;
    ivp_anomaly_config_t cfg;
    ivp_anomaly_config_init(&cfg);
    cfg.auto_correct = true;
    ivp_anomaly_manager_init(&mgr, &cfg);

    /* Inject NaN into velocity */
    obj->core.speed.k[0] = NAN;

    int result = ivp_anomaly_check_object(&mgr, obj);

    IVP_ASSERT(result & IVP_ANOMALY_NAN_VELOCITY);
    IVP_ASSERT(!isnan(obj->core.speed.k[0]));
    IVP_ASSERT(mgr.anomalies_corrected > 0);

    ivp_environment_destroy(env);
}

IVP_TEST(anomaly_auto_correct_clamps_speed) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj = create_test_object(env);

    ivp_anomaly_manager_t mgr;
    ivp_anomaly_config_t cfg;
    ivp_anomaly_config_init(&cfg);
    cfg.auto_correct = true;
    cfg.max_speed = IVP_REAL(1000.0);
    ivp_anomaly_manager_init(&mgr, &cfg);

    /* Set excessive speed */
    obj->core.speed.k[0] = IVP_REAL(5000.0);
    obj->core.speed.k[1] = IVP_REAL(0.0);
    obj->core.speed.k[2] = IVP_REAL(0.0);

    int result = ivp_anomaly_check_object(&mgr, obj);

    IVP_ASSERT(result & IVP_ANOMALY_EXCESSIVE_SPEED);

    /* Verify speed was clamped */
    ivp_real_t speed_mag = ivp_vec3r_length(&obj->core.speed);
    IVP_ASSERT(speed_mag <= cfg.max_speed + IVP_REAL(0.1));
    IVP_ASSERT(mgr.anomalies_corrected > 0);

    ivp_environment_destroy(env);
}

IVP_TEST(anomaly_collision_cap_per_object) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj_a = create_test_object(env);
    ivp_object_t *obj_b = create_test_object(env);

    ivp_anomaly_manager_t mgr;
    ivp_anomaly_config_t cfg;
    ivp_anomaly_config_init(&cfg);
    cfg.max_collisions_per_psi = 1;
    cfg.freeze_on_max_collisions = true;
    ivp_anomaly_manager_init(&mgr, &cfg);

    ivp_anomaly_manager_begin_psi(&mgr);
    IVP_ASSERT_TRUE(ivp_anomaly_manager_count_collision(&mgr, obj_a));
    IVP_ASSERT_TRUE(ivp_anomaly_manager_count_collision(&mgr, obj_b));
    IVP_ASSERT_TRUE(obj_a->core.movement_state != IVP_MT_NOT_SIM);
    IVP_ASSERT_TRUE(obj_b->core.movement_state != IVP_MT_NOT_SIM);

    IVP_ASSERT_FALSE(ivp_anomaly_manager_count_collision(&mgr, obj_a));
    IVP_ASSERT_TRUE(obj_a->core.movement_state == IVP_MT_NOT_SIM);
    IVP_ASSERT_TRUE(obj_b->core.movement_state != IVP_MT_NOT_SIM);

    obj_a->core.movement_state = IVP_MT_MOVING;
    ivp_anomaly_manager_begin_psi(&mgr);
    IVP_ASSERT_TRUE(ivp_anomaly_manager_count_collision(&mgr, obj_a));
    IVP_ASSERT_TRUE(ivp_anomaly_manager_count_collision(&mgr, obj_b));

    ivp_environment_destroy(env);
}

int main(void) {
    IVP_REGISTER(anomaly_detect_nan_velocity);
    IVP_REGISTER(anomaly_detect_nan_position);
    IVP_REGISTER(anomaly_detect_nan_angular);
    IVP_REGISTER(anomaly_detect_excessive_speed);
    IVP_REGISTER(anomaly_detect_excessive_angular);
    IVP_REGISTER(anomaly_callback_fires);
    IVP_REGISTER(anomaly_auto_correct_zeros_nan);
    IVP_REGISTER(anomaly_auto_correct_clamps_speed);
    IVP_REGISTER(anomaly_collision_cap_per_object);

    IVP_RUN_ALL();
}
