/* Example display poses must use the engine's AT trajectory, including
 * core/object offsets, rather than integrating the current velocities. */
#include "ivp_test.h"
#include "demo_app.h"

static ivp_environment_t *pose_world(ivp_object_t **object)
{
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    config.gravity = ivp_vec3r(0.0, 0.0, 0.0);
    ivp_environment_t *env = ivp_environment_create(&config);
    if (!env) return NULL;
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.ball_radius = IVP_REAL(0.5);
    t.use_mass_center_override = true;
    t.mass_center_override = ivp_vec3r(0.25, -0.5, 0.125);
    *object = ivp_environment_create_object(env, &t, NULL);
    if (!*object) {
        ivp_environment_destroy(env);
        return NULL;
    }
    ivp_environment_simulate_dtime(env, IVP_REAL(0.005));
    ivp_core_t *core = (*object)->physical_core;
    core->delta_world_f_core_psis = ivp_vec3r(4.0, 2.0, -1.0);
    core->q_world_f_core_next_psi = ivp_quatr_make(0.0, 0.0, 0.6, 0.8);
    /* After an impulse, velocity need not equal the cached PSI trajectory. */
    core->speed = ivp_vec3r(20.0, -10.0, 5.0);
    core->rot_speed = ivp_vec3f(30.0f, 10.0f, -20.0f);
    return env;
}

IVP_TEST(display_object_pose_matches_engine_at) {
    ivp_object_t *object = NULL;
    ivp_environment_t *env = pose_world(&object);
    IVP_ASSERT_NOT_NULL(env);
    ivp_transform_r_t expected;
    ivp_object_get_m_world_f_object_AT(object, &expected);
    g_demo_display_poses = true;
    float pos[3], rot[9];
    demo_object_pose(object, pos, rot);
    g_demo_display_poses = false;
    ivp_environment_destroy(env);
    for (int r = 0; r < 3; r++) {
        IVP_ASSERT_NEAR(pos[r], expected.pos.k[r], 1e-6);
        for (int c = 0; c < 3; c++)
            IVP_ASSERT_NEAR(rot[c * 3 + r], expected.rot.rows[r].k[c], 1e-6);
    }
}

IVP_TEST(display_core_center_matches_engine_at) {
    ivp_object_t *object = NULL;
    ivp_environment_t *env = pose_world(&object);
    IVP_ASSERT_NOT_NULL(env);
    ivp_vec3r_t expected;
    ivp_core_get_at_position(object->physical_core, ivp_environment_get_time(env), &expected);
    g_demo_display_poses = true;
    float center[3];
    demo_object_center(object, center);
    g_demo_display_poses = false;
    ivp_environment_destroy(env);
    for (int k = 0; k < 3; k++) IVP_ASSERT_NEAR(center[k], expected.k[k], 1e-6);
}

IVP_TEST(display_rotation_uses_cached_psi_trajectory) {
    ivp_object_t *object = NULL;
    ivp_environment_t *env = pose_world(&object);
    IVP_ASSERT_NOT_NULL(env);
    ivp_transform_r_t expected;
    ivp_object_get_m_world_f_object_AT(object, &expected);
    g_demo_display_poses = true;
    float pos[3], rot[9];
    demo_object_pose(object, pos, rot);
    g_demo_display_poses = false;
    ivp_environment_destroy(env);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            IVP_ASSERT_NEAR(rot[c * 3 + r], expected.rot.rows[r].k[c], 1e-6);
}

IVP_TEST(display_anchor_matches_engine_at) {
    ivp_object_t *object = NULL;
    ivp_environment_t *env = pose_world(&object);
    IVP_ASSERT_NOT_NULL(env);
    const ivp_vec3r_t anchor = ivp_vec3r(0.5, -0.25, 1.0);
    ivp_transform_r_t m;
    ivp_core_calc_at_matrix(object->physical_core, ivp_environment_get_time(env), &m);
    ivp_vec3r_t expected;
    ivp_transform_point(&expected, &m, &anchor);
    g_demo_display_poses = true;
    float actual[3];
    demo_core_point_ws(object, &anchor, actual);
    g_demo_display_poses = false;
    ivp_environment_destroy(env);
    for (int k = 0; k < 3; k++) IVP_ASSERT_NEAR(actual[k], expected.k[k], 1e-6);
}

IVP_TEST(headless_object_pose_keeps_psi_time_and_object_offset) {
    ivp_object_t *object = NULL;
    ivp_environment_t *env = pose_world(&object);
    IVP_ASSERT_NOT_NULL(env);
    ivp_transform_r_t expected = object->physical_core->m_world_f_core_last_psi;
    ivp_vec3r_t origin;
    ivp_transform_point(&origin, &expected, &object->shift_core_f_object);
    float pos[3];
    g_demo_display_poses = false;
    demo_object_pose(object, pos, NULL);
    ivp_environment_destroy(env);
    for (int k = 0; k < 3; k++) IVP_ASSERT_NEAR(pos[k], origin.k[k], 1e-6);
}

int main(void)
{
    IVP_REGISTER(display_object_pose_matches_engine_at);
    IVP_REGISTER(display_core_center_matches_engine_at);
    IVP_REGISTER(display_rotation_uses_cached_psi_trajectory);
    IVP_REGISTER(display_anchor_matches_engine_at);
    IVP_REGISTER(headless_object_pose_keeps_psi_time_and_object_offset);
    IVP_RUN_ALL();
}
