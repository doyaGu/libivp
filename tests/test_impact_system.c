/* test_impact_system.c — Tests for multi-body impact cascade system */
#include "ivp_test.h"
#include <ivp/solver/ivp_impact_system.h>
#include <ivp/solver/ivp_friction_system.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_environment_collision.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/math/ivp_vec3.h>
#include <string.h>

static ivp_environment_t *make_env(void) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t x) {
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.position, x, IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create_object(env, &tmpl, NULL);
}

IVP_TEST(impact_system_init_destroy) {
    ivp_impact_system_t sys;
    ivp_impact_system_init(&sys, NULL);
    IVP_ASSERT_EQ(sys.sum_of_pushes, 0);
    IVP_ASSERT_EQ(ivp_array_len(&sys.pushed_cores), 0);
    IVP_ASSERT_EQ(ivp_array_len(&sys.known_cores), 0);
    IVP_ASSERT_EQ(ivp_array_len(&sys.pairs), 0);
    IVP_ASSERT_NOT_NULL(sys.alloc);
    ivp_impact_system_destroy(&sys);
}

IVP_TEST(impact_system_push_limit_constant) {
    /* Verify the constant is defined and positive */
    IVP_ASSERT_TRUE(IVP_MAXIMUM_SYSTEM_PUSH_NUMBER > 0);
}

IVP_TEST(impact_system_solve_null_safe) {
    /* Null start_contact should not crash */
    ivp_impact_system_t sys;
    ivp_impact_system_init(&sys, NULL);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);

    ivp_impact_system_solve(&sys, NULL, NULL, NULL, &settings);
    IVP_ASSERT_EQ(sys.sum_of_pushes, 0);

    ivp_impact_system_destroy(&sys);
}

IVP_TEST(impact_system_single_contact) {
    /* Single contact: cascade should terminate after one resolution */
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    /* Give them approaching velocities */
    ivp_vec3r_set(&a->core.speed, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&b->core.speed, IVP_REAL(-5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Set up a persistent contact */
    ivp_persistent_contact_t pc;
    memset(&pc, 0, sizeof(pc));
    pc.cp.obj_a = a;
    pc.cp.obj_b = b;
    ivp_vec3r_set(&pc.cp.normal, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&pc.cp.point_a, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&pc.cp.point_b, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    pc.cp.gap_length = IVP_REAL(0.0);
    ivp_contact_compute_virtual_mass(&pc.cp);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);

    ivp_impact_system_t sys;
    ivp_impact_system_init(&sys, NULL);
    ivp_impact_system_solve(&sys, &pc, NULL, NULL, &settings);

    /* Should have resolved at least one push */
    IVP_ASSERT_TRUE(sys.sum_of_pushes >= 1);

    ivp_impact_system_destroy(&sys);
    ivp_environment_destroy(env);
}

IVP_TEST(impact_system_double_init_destroy) {
    /* Init, destroy, re-init: should work cleanly */
    ivp_impact_system_t sys;
    ivp_impact_system_init(&sys, NULL);
    ivp_impact_system_destroy(&sys);

    ivp_impact_system_init(&sys, NULL);
    IVP_ASSERT_EQ(sys.sum_of_pushes, 0);
    ivp_impact_system_destroy(&sys);
}

IVP_TEST(impact_triggered_from_event_pipeline) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(2.2));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_vec3r_set(&a->core.speed, IVP_REAL(8.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&b->core.speed, IVP_REAL(-8.0), IVP_REAL(0.0), IVP_REAL(0.0));

    int resolved = ivp_environment_resolve_collisions(env);
    IVP_ASSERT_TRUE(resolved >= 0);
    IVP_ASSERT_TRUE(env->mindist_manager.entry_count > 0);

    ivp_environment_destroy(env);
}

/* ── Registration ──────────────────────────────────────────────────────── */

int main(void) {
    IVP_REGISTER(impact_system_init_destroy);
    IVP_REGISTER(impact_system_push_limit_constant);
    IVP_REGISTER(impact_system_solve_null_safe);
    IVP_REGISTER(impact_system_single_contact);
    IVP_REGISTER(impact_system_double_init_destroy);
    IVP_REGISTER(impact_triggered_from_event_pipeline);
    IVP_RUN_ALL();
}
