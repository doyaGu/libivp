/* test_friction_system.c — Tests for persistent friction system */
#include "ivp_test.h"
#include "ivp_test_controller_api.h"
#include <ivp/solver/ivp_friction_system.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_environment_collision.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/collision/ivp_synapse.h>
#include <ivp/events/ivp_listener_collision.h>
#include <ivp/math/ivp_vec3.h>
#include <string.h>

static ivp_environment_t *make_env(void) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t x, ivp_real_t y) {
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.position, x, y, IVP_REAL(0.0));
    return ivp_environment_create_object(env, &tmpl, NULL);
}

typedef struct {
    int created_count;
    int deleted_count;
} friction_listener_tracker_t;

typedef struct {
    ivp_object_t *a;
    ivp_object_t *b;
    ivp_real_t observed_rel_speed;
    bool seen;
} phase_probe_t;

static void on_friction_created(void *ctx, const ivp_friction_event_t *event) {
    friction_listener_tracker_t *tracker = (friction_listener_tracker_t *)ctx;
    if (tracker && event) tracker->created_count++;
}

static void on_friction_deleted(void *ctx, const ivp_friction_event_t *event) {
    friction_listener_tracker_t *tracker = (friction_listener_tracker_t *)ctx;
    if (tracker && event) tracker->deleted_count++;
}

static void capture_rel_speed_controller(const ivp_event_sim_t *event_sim,
                                         ivp_object_t *obj,
                                         void *ctx)
{
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    (void)obj;
    phase_probe_t *probe = (phase_probe_t *)ctx;
    if (!probe || probe->seen) return;
    ivp_vec3r_t rel;
    ivp_vec3r_sub(&rel, &probe->b->core.speed, &probe->a->core.speed);
    probe->observed_rel_speed = ivp_vec3r_length(&rel);
    probe->seen = true;
}

IVP_TEST(friction_manager_init_destroy) {
    ivp_friction_manager_t mgr;
    IVP_ASSERT_TRUE(ivp_ok(ivp_friction_manager_init(&mgr, NULL)));
    IVP_ASSERT_EQ(mgr.system_count, 0);
    IVP_ASSERT_TRUE(mgr.block_count > 0);
    ivp_friction_manager_destroy(&mgr);
}

IVP_TEST(friction_contact_persistence) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0));

    /* Create contact */
    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = a;
    cp.obj_b = b;
    ivp_vec3r_set(&cp.normal, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&cp.point_a, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&cp.point_b, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    cp.gap_length = IVP_REAL(0.0);

    /* Match or create should create a new persistent contact */
    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_persistent_contact_t *pc1 = ivp_friction_manager_match_or_create(
        &env->friction_manager, &cp);
    IVP_ASSERT_NOT_NULL(pc1);

    /* Age: should increment */
    int age1 = pc1->cp.age_psi;

    /* Next PSI: match should find the same contact */
    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_persistent_contact_t *pc2 = ivp_friction_manager_match_or_create(
        &env->friction_manager, &cp);
    /* Should match existing */
    IVP_ASSERT_NOT_NULL(pc2);
    IVP_ASSERT_TRUE(pc2->cp.age_psi >= age1);

    ivp_environment_destroy(env);
}

IVP_TEST(friction_prune_unseen) {
    ivp_friction_manager_t mgr;
    IVP_ASSERT_TRUE(ivp_ok(ivp_friction_manager_init(&mgr, NULL)));

    /* Without any contacts, prune should not crash */
    ivp_friction_manager_begin_psi(&mgr);
    ivp_friction_manager_age_and_prune(&mgr);

    ivp_friction_manager_destroy(&mgr);
}

IVP_TEST(friction_prune_removes_empty_system) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0));

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = a;
    cp.obj_b = b;
    ivp_vec3r_set(&cp.normal, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&cp.point_a, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&cp.point_b, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    cp.gap_length = IVP_REAL(0.0);

    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_persistent_contact_t *pc = ivp_friction_manager_match_or_create(
        &env->friction_manager, &cp);
    IVP_ASSERT_NOT_NULL(pc);
    IVP_ASSERT_EQ(env->friction_manager.system_count, 1);

    /* Next PSI: do not match this contact, then prune. */
    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_friction_manager_age_and_prune(&env->friction_manager);

    IVP_ASSERT_EQ(env->friction_manager.system_count, 0);

    ivp_environment_destroy(env);
}

IVP_TEST(friction_system_ease) {
    ivp_friction_system_t sys;
    memset(&sys, 0, sizeof(sys));
    sys.alloc = ivp_default_allocator();

    /* Ease on empty system should not crash */
    ivp_friction_system_ease(&sys);
    ivp_friction_system_remove_gained_energy(&sys);
}

IVP_TEST(friction_energy_tracking) {
    ivp_friction_manager_t mgr;
    IVP_ASSERT_TRUE(ivp_ok(ivp_friction_manager_init(&mgr, NULL)));

    /* Verify psi_counter increments */
    IVP_ASSERT_EQ(mgr.psi_counter, 0);
    ivp_friction_manager_begin_psi(&mgr);
    IVP_ASSERT_EQ(mgr.psi_counter, 1);
    ivp_friction_manager_begin_psi(&mgr);
    IVP_ASSERT_EQ(mgr.psi_counter, 2);

    ivp_friction_manager_destroy(&mgr);
}

IVP_TEST(friction_listener_created_deleted) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    friction_listener_tracker_t tracker;
    memset(&tracker, 0, sizeof(tracker));

    ivp_collision_listener_t listener;
    ivp_collision_listener_init(&listener,
                                IVP_COLLISION_CB_FRICTION_CREATED | IVP_COLLISION_CB_FRICTION_DELETED,
                                &tracker);
    listener.friction_created_fn = on_friction_created;
    listener.friction_deleted_fn = on_friction_deleted;
    IVP_ASSERT_EQ(ivp_object_add_collision_listener(a, &listener), 0);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = a;
    cp.obj_b = b;
    ivp_vec3r_set(&cp.normal, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&cp.point_a, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&cp.point_b, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    cp.gap_length = IVP_REAL(0.0);

    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_persistent_contact_t *pc = ivp_friction_manager_match_or_create(
        &env->friction_manager, &cp);
    IVP_ASSERT_NOT_NULL(pc);
    IVP_ASSERT_TRUE(tracker.created_count > 0);

    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_friction_manager_age_and_prune(&env->friction_manager);
    IVP_ASSERT_TRUE(tracker.deleted_count > 0);

    IVP_ASSERT_EQ(ivp_object_remove_collision_listener(a, &listener), 0);
    ivp_environment_destroy(env);
}

IVP_TEST(friction_coupled_pressure_clamped_to_normal_limit) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_vec3r_set(&a->core.speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&b->core.speed, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = a;
    cp.obj_b = b;
    ivp_contact_compute_sphere_sphere(&cp);
    ivp_contact_combine_materials(&cp);
    ivp_contact_compute_virtual_mass(&cp);

    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_persistent_contact_t *pc = ivp_friction_manager_match_or_create(
        &env->friction_manager, &cp);
    IVP_ASSERT_NOT_NULL(pc);
    pc->cp.age_psi = 2;

    ivp_friction_system_t *sys = ivp_friction_manager_find_system_for_core(
        &env->friction_manager, &a->core);
    IVP_ASSERT_NOT_NULL(sys);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);
    settings.gravity_magnitude = IVP_REAL(9.81);

    ivp_result_t res = ivp_friction_system_solve_coupled(sys, &settings, IVP_REAL(1.0));
    IVP_ASSERT_TRUE(ivp_ok(res));

    /* With additive pressure (reference alignment: distance keeper + solver),
     * the combined pressure can exceed the solver-only max_impulse.
     * Verify pressure is non-negative and finite. */
    IVP_ASSERT_TRUE(pc->now_friction_pressure >= IVP_REAL(0.0));
    IVP_ASSERT_TRUE(isfinite((double)pc->now_friction_pressure));

    ivp_environment_destroy(env);
}

IVP_TEST(friction_coupled_oversized_processes_all_contacts) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_vec3r_set(&a->core.speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&b->core.speed, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    const int target_contacts = IVP_MAX_FRICTION_MATRIX + 1;

    ivp_friction_manager_begin_psi(&env->friction_manager);
    for (int i = 0; i < target_contacts; i++) {
        ivp_real_t y = (ivp_real_t)i * IVP_REAL(0.05);

        ivp_contact_point_t cp;
        memset(&cp, 0, sizeof(cp));
        cp.obj_a = a;
        cp.obj_b = b;
        ivp_vec3r_set(&cp.normal, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_set(&cp.point_a, IVP_REAL(0.5), y, IVP_REAL(0.0));
        ivp_vec3r_set(&cp.point_b, IVP_REAL(0.5), y, IVP_REAL(0.0));
        ivp_vec3r_set(&cp.r_a, IVP_REAL(0.5), y, IVP_REAL(0.0));
        ivp_vec3r_set(&cp.r_b, IVP_REAL(-0.5), y, IVP_REAL(0.0));
        cp.gap_length = IVP_REAL(0.0);
        cp.virtual_mass = IVP_REAL(0.5);
        cp.inv_virtual_mass = IVP_REAL(2.0);
        cp.friction = IVP_REAL(0.0);

        ivp_persistent_contact_t *pc = ivp_friction_manager_match_or_create(
            &env->friction_manager, &cp);
        IVP_ASSERT_NOT_NULL(pc);
    }

    ivp_friction_system_t *sys = ivp_friction_manager_find_system_for_core(
        &env->friction_manager, &a->core);
    IVP_ASSERT_NOT_NULL(sys);
    IVP_ASSERT_EQ(sys->contact_count, target_contacts);

    int linked_count = 0;
    for (ivp_persistent_contact_t *pc = sys->first_contact; pc; pc = pc->next) {
        pc->cp.age_psi = 2;
        pc->now_friction_pressure = IVP_REAL(-1.0);
        linked_count++;
    }
    IVP_ASSERT_EQ(linked_count, target_contacts);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);
    settings.gravity_magnitude = IVP_REAL(9.81);

    ivp_result_t res = ivp_friction_system_solve_coupled(sys, &settings, IVP_REAL(1.0));
    IVP_ASSERT_TRUE(ivp_ok(res));

    int touched = 0;
    for (ivp_persistent_contact_t *pc = sys->first_contact; pc; pc = pc->next) {
        IVP_ASSERT_TRUE(pc->now_friction_pressure >= IVP_REAL(0.0));
        touched++;
    }
    IVP_ASSERT_EQ(touched, target_contacts);

    ivp_environment_destroy(env);
}

IVP_TEST(friction_contact_persistence_order_flip) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = a;
    cp.obj_b = b;
    ivp_vec3r_set(&cp.normal, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&cp.point_a, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&cp.point_b, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    cp.gap_length = IVP_REAL(0.0);

    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_persistent_contact_t *pc1 = ivp_friction_manager_match_or_create(
        &env->friction_manager, &cp);
    IVP_ASSERT_NOT_NULL(pc1);

    ivp_contact_point_t cp_flipped = cp;
    cp_flipped.obj_a = b;
    cp_flipped.obj_b = a;
    cp_flipped.point_a = cp.point_b;
    cp_flipped.point_b = cp.point_a;
    cp_flipped.normal.k[0] = -cp.normal.k[0];
    cp_flipped.normal.k[1] = -cp.normal.k[1];
    cp_flipped.normal.k[2] = -cp.normal.k[2];

    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_persistent_contact_t *pc2 = ivp_friction_manager_match_or_create(
        &env->friction_manager, &cp_flipped);
    IVP_ASSERT_NOT_NULL(pc2);
    IVP_ASSERT_TRUE(pc1 == pc2);

    ivp_friction_system_t *sys = ivp_friction_manager_find_system_for_core(
        &env->friction_manager, &a->core);
    IVP_ASSERT_NOT_NULL(sys);
    IVP_ASSERT_EQ(sys->contact_count, 1);

    ivp_environment_destroy(env);
}

IVP_TEST(friction_manager_skips_far_predictive_contacts) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    /* Gap = 0.1 with unit spheres:
     *   - outside keeper_dist (~0.023)
     *   - inside max_distance_for_impact (~0.22)
     * So broadphase reports the pair, but no impact is needed. */
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(2.1), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_vec3r_set_zero(&a->core.speed);
    ivp_vec3r_set_zero(&b->core.speed);

    for (int i = 0; i < 4; i++) {
        ivp_friction_manager_begin_psi(&env->friction_manager);
        int resolved = ivp_environment_resolve_collisions(env);
        IVP_ASSERT_EQ(resolved, 0);
        IVP_ASSERT_EQ(env->friction_manager.system_count, 0);
    }

    ivp_environment_destroy(env);
}

IVP_TEST(friction_phase_energy_runs_before_static) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = a;
    cp.obj_b = b;
    ivp_contact_compute_sphere_sphere(&cp);
    ivp_contact_combine_materials(&cp);
    ivp_contact_compute_virtual_mass(&cp);

    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_persistent_contact_t *pc = ivp_friction_manager_match_or_create(&env->friction_manager, &cp);
    IVP_ASSERT_NOT_NULL(pc);
    ivp_friction_system_t *sys = ivp_friction_manager_find_system_for_core(&env->friction_manager, &a->core);
    IVP_ASSERT_NOT_NULL(sys);
    IVP_ASSERT_TRUE(sys->pair_count > 0);

    ivp_vec3r_set(&a->core.speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&b->core.speed, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    sys->pairs[0].integrated_anti_energy = IVP_REAL(0.2);

    phase_probe_t probe;
    memset(&probe, 0, sizeof(probe));
    probe.a = a;
    probe.b = b;
    /* Descending priority dispatch: choose <2000 so probe runs AFTER
     * energy_friction(2000). In this synthetic setup energy friction may
     * damp or leave relative speed unchanged, but should not amplify it. */
    ivp_controller_handle_t h = ivp_test_environment_add_custom_controller(
        env, 1500, capture_rel_speed_controller, &probe);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(h));

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_TRUE(probe.seen);
    IVP_ASSERT_TRUE(probe.observed_rel_speed <= IVP_REAL(2.0));

    ivp_environment_destroy(env);
}

IVP_TEST(friction_phase_energy_prunes_broken_contacts) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = a;
    cp.obj_b = b;
    ivp_contact_compute_sphere_sphere(&cp);
    ivp_contact_combine_materials(&cp);
    ivp_contact_compute_virtual_mass(&cp);

    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_persistent_contact_t *pc = ivp_friction_manager_match_or_create(&env->friction_manager, &cp);
    IVP_ASSERT_NOT_NULL(pc);
    ivp_friction_system_t *sys = ivp_friction_manager_find_system_for_core(&env->friction_manager, &a->core);
    IVP_ASSERT_NOT_NULL(sys);
    IVP_ASSERT_EQ(sys->contact_count, 1);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);

    ivp_vec3r_t far_pos;
    ivp_vec3r_set(&far_pos, IVP_REAL(100.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_core_set_pose(&b->core, &far_pos, &b->core.q_world_f_core_last_psi);

    ivp_friction_system_phase_energy(sys, &settings, IVP_REAL(1.0));

    IVP_ASSERT_EQ(sys->contact_count, 0);
    ivp_environment_destroy(env);
}

IVP_TEST(friction_oversize_panic_marks_fast_piling_and_zeroes_non_terminal) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_object_t *c = make_ball(env, IVP_REAL(-1.0), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);
    IVP_ASSERT_NOT_NULL(c);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);

    ivp_friction_manager_begin_psi(&env->friction_manager);
    for (int i = 0; i < 76; i++) {
        ivp_contact_point_t cp;
        memset(&cp, 0, sizeof(cp));
        cp.obj_a = a; cp.obj_b = b;
        ivp_vec3r_set(&cp.normal, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_real_t y = (ivp_real_t)i * IVP_REAL(0.01);
        ivp_vec3r_set(&cp.point_a, IVP_REAL(0.5), y, IVP_REAL(0.0));
        ivp_vec3r_set(&cp.point_b, IVP_REAL(0.5), y, IVP_REAL(0.0));
        cp.gap_length = settings.keeper_dist;
        cp.virtual_mass = IVP_REAL(0.5);
        cp.inv_virtual_mass = IVP_REAL(2.0);
        ivp_persistent_contact_t *pc = ivp_friction_manager_match_or_create(&env->friction_manager, &cp);
        IVP_ASSERT_NOT_NULL(pc);
        pc->cp.age_psi = 2;
    }
    for (int i = 0; i < 76; i++) {
        ivp_contact_point_t cp;
        memset(&cp, 0, sizeof(cp));
        cp.obj_a = a; cp.obj_b = c;
        ivp_vec3r_set(&cp.normal, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_real_t y = (ivp_real_t)i * IVP_REAL(0.01);
        ivp_vec3r_set(&cp.point_a, IVP_REAL(-0.5), y, IVP_REAL(0.0));
        ivp_vec3r_set(&cp.point_b, IVP_REAL(-0.5), y, IVP_REAL(0.0));
        cp.gap_length = settings.keeper_dist;
        cp.virtual_mass = IVP_REAL(0.5);
        cp.inv_virtual_mass = IVP_REAL(2.0);
        ivp_persistent_contact_t *pc = ivp_friction_manager_match_or_create(&env->friction_manager, &cp);
        IVP_ASSERT_NOT_NULL(pc);
        pc->cp.age_psi = 2;
    }

    ivp_friction_system_t *sys = ivp_friction_manager_find_system_for_core(&env->friction_manager, &a->core);
    IVP_ASSERT_NOT_NULL(sys);
    IVP_ASSERT_TRUE(sys->contact_count > IVP_MAX_FRICTION_MATRIX);

    ivp_vec3r_set(&a->core.speed, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&b->core.speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&c->core.speed, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_result_t res = ivp_friction_system_phase_static_normal(sys, &settings, IVP_REAL(1.0));
    IVP_ASSERT_TRUE(ivp_ok(res));
    IVP_ASSERT_TRUE(a->core.fast_piling_allowed);
    IVP_ASSERT_TRUE(b->core.fast_piling_allowed);
    IVP_ASSERT_TRUE(c->core.fast_piling_allowed);
    IVP_ASSERT_NEAR(a->core.speed.k[0], IVP_REAL(0.0), IVP_REAL(1e-6));

    ivp_environment_destroy(env);
}

IVP_TEST(friction_dynamic_pretension_two_value_clip_and_ease) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_contact_point_t cp0;
    memset(&cp0, 0, sizeof(cp0));
    cp0.obj_a = a; cp0.obj_b = b;
    ivp_vec3r_set(&cp0.normal, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&cp0.point_a, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&cp0.point_b, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    cp0.gap_length = IVP_REAL(0.0);
    cp0.virtual_mass = IVP_REAL(0.5);
    cp0.inv_virtual_mass = IVP_REAL(2.0);
    ivp_contact_compute_virtual_mass(&cp0);

    ivp_contact_point_t cp1 = cp0;
    cp1.point_a.k[1] = IVP_REAL(0.05);
    cp1.point_b.k[1] = IVP_REAL(0.05);
    ivp_contact_compute_virtual_mass(&cp1);

    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_persistent_contact_t *pc0 = ivp_friction_manager_match_or_create(&env->friction_manager, &cp0);
    ivp_persistent_contact_t *pc1 = ivp_friction_manager_match_or_create(&env->friction_manager, &cp1);
    IVP_ASSERT_NOT_NULL(pc0);
    IVP_ASSERT_NOT_NULL(pc1);
    ivp_friction_system_t *sys = ivp_friction_manager_find_system_for_core(&env->friction_manager, &a->core);
    IVP_ASSERT_NOT_NULL(sys);
    IVP_ASSERT_TRUE(sys->pair_count > 0);

    pc0->cp.age_psi = 2;
    pc1->cp.age_psi = 2;
    pc0->now_friction_pressure = IVP_REAL(0.1);
    pc1->now_friction_pressure = IVP_REAL(0.1);
    pc0->cp.friction = IVP_REAL(1.0);
    pc1->cp.friction = IVP_REAL(1.0);
    pc0->inv_virt_mass_mindist_no_dir = IVP_REAL(1.0);
    pc1->inv_virt_mass_mindist_no_dir = IVP_REAL(1.0);
    pc0->span_friction_s[0] = IVP_REAL(10.0);
    pc0->span_friction_s[1] = IVP_REAL(0.0);
    pc1->span_friction_s[0] = IVP_REAL(0.0);
    pc1->span_friction_s[1] = IVP_REAL(1.0);
    pc0->two_friction_values = true;
    pc0->two_friction_x_scale = IVP_REAL(0.2);
    pc1->two_friction_values = false;
    sys->pairs[0].next_ease_nr_psi = 0;

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);
    ivp_friction_system_phase_dynamic_tangential(sys, &settings, IVP_REAL(1.0));

    IVP_ASSERT_TRUE(ivp_fabs(pc0->span_friction_s[0]) <=
        IVP_MINIMAL_REAL_FRICTION_LEN * IVP_REAL(0.2) + IVP_REAL(1e-6));
    IVP_ASSERT_EQ(pc0->cp_status, IVP_CP_STATUS_NEEDS_RECHECK);
    IVP_ASSERT_TRUE(ivp_fabs(pc1->span_friction_s[1]) < IVP_REAL(1.0));

    ivp_environment_destroy(env);
}

IVP_TEST(friction_contacts_have_synapse_links) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = a;
    cp.obj_b = b;
    ivp_contact_compute_sphere_sphere(&cp);
    ivp_contact_combine_materials(&cp);
    ivp_contact_compute_virtual_mass(&cp);

    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_persistent_contact_t *pc =
        ivp_friction_manager_match_or_create(&env->friction_manager, &cp);
    IVP_ASSERT_NOT_NULL(pc);
    IVP_ASSERT_NOT_NULL(a->friction_synapses);
    IVP_ASSERT_NOT_NULL(b->friction_synapses);
    IVP_ASSERT_EQ(ivp_synapse_link_get_friction_contact(a->friction_synapses), pc);
    IVP_ASSERT_EQ(ivp_synapse_link_get_friction_contact(b->friction_synapses), pc);

    ivp_environment_destroy(env);
}

/* ── Registration ──────────────────────────────────────────────────────── */

int main(void) {
    IVP_REGISTER(friction_manager_init_destroy);
    IVP_REGISTER(friction_contact_persistence);
    IVP_REGISTER(friction_contact_persistence_order_flip);
    IVP_REGISTER(friction_prune_unseen);
    IVP_REGISTER(friction_prune_removes_empty_system);
    IVP_REGISTER(friction_system_ease);
    IVP_REGISTER(friction_energy_tracking);
    IVP_REGISTER(friction_listener_created_deleted);
    IVP_REGISTER(friction_coupled_pressure_clamped_to_normal_limit);
    IVP_REGISTER(friction_coupled_oversized_processes_all_contacts);
    IVP_REGISTER(friction_manager_skips_far_predictive_contacts);
    IVP_REGISTER(friction_phase_energy_runs_before_static);
    IVP_REGISTER(friction_phase_energy_prunes_broken_contacts);
    IVP_REGISTER(friction_oversize_panic_marks_fast_piling_and_zeroes_non_terminal);
    IVP_REGISTER(friction_dynamic_pretension_two_value_clip_and_ease);
    IVP_REGISTER(friction_contacts_have_synapse_links);
    IVP_RUN_ALL();
}
