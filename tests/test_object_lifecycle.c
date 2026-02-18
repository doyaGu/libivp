#include "ivp_test.h"
#include <ivp/dynamics/ivp_object_attach.h>
#include <ivp/dynamics/ivp_merge_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_result.h>

#ifdef IVP_REAL_DOUBLE
  #define TOL 1e-6
#else
  #define TOL 1e-3f
#endif

static int s_break_count = 0;
static void break_cb(void *ctx, ivp_merge_group_t *group) {
    (void)ctx;
    (void)group;
    s_break_count++;
}

/* Helper: create basic environment with zero gravity */
static ivp_environment_t *create_test_env(void) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

/* Helper: create basic object at position */
static ivp_object_t *create_test_object(ivp_environment_t *env, ivp_real_t x, ivp_real_t y, ivp_real_t z) {
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&t.position, x, y, z);
    t.orientation = ivp_quatr_identity();
    ivp_object_t *obj = ivp_environment_create_object(env, &t, NULL);
    obj->core.upper_limit_radius = IVP_REAL(0.5);
    return obj;
}

/* ========== ATTACHMENT TESTS (6) ========== */

IVP_TEST(attachment_init) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *leader = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_attachment_t att;
    ivp_attachment_init(&att, leader);

    IVP_ASSERT_TRUE(att.leader == leader);
    IVP_ASSERT_TRUE(att.n_followers == 0);
    IVP_ASSERT_TRUE(att.active == true);

    ivp_environment_destroy(env);
}

IVP_TEST(attachment_add_follower) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *leader = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *follower = create_test_object(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_attachment_t att;
    ivp_attachment_init(&att, leader);

    ivp_result_t res = ivp_attachment_add(&att, follower);

    IVP_ASSERT_TRUE(res == IVP_OK);
    IVP_ASSERT_TRUE(att.n_followers == 1);
    IVP_ASSERT_TRUE(att.followers[0].obj == follower);

    /* Verify offset was computed (follower is at +1 x from leader) */
    ivp_real_t offset_x = att.followers[0].offset_pos.k[0];
    IVP_ASSERT_TRUE(ivp_fabs(offset_x - IVP_REAL(1.0)) < TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(attachment_sync) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *leader = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *follower = create_test_object(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_attachment_t att;
    ivp_attachment_init(&att, leader);
    ivp_attachment_add(&att, follower);

    /* Move leader's core position */
    ivp_vec3r_set(&leader->core.pos_world_f_core_last_psi, IVP_REAL(5.0), IVP_REAL(3.0), IVP_REAL(2.0));

    /* Sync should update follower to leader_pos + offset */
    ivp_attachment_sync(&att);

    /* Follower should now be at (5+1, 3+0, 2+0) = (6, 3, 2) */
    ivp_real_t expected_x = IVP_REAL(6.0);
    ivp_real_t expected_y = IVP_REAL(3.0);
    ivp_real_t expected_z = IVP_REAL(2.0);

    IVP_ASSERT_TRUE(ivp_fabs(follower->core.pos_world_f_core_last_psi.k[0] - expected_x) < TOL);
    IVP_ASSERT_TRUE(ivp_fabs(follower->core.pos_world_f_core_last_psi.k[1] - expected_y) < TOL);
    IVP_ASSERT_TRUE(ivp_fabs(follower->core.pos_world_f_core_last_psi.k[2] - expected_z) < TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(attachment_remove) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *leader = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *follower = create_test_object(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_attachment_t att;
    ivp_attachment_init(&att, leader);
    ivp_attachment_add(&att, follower);

    IVP_ASSERT_TRUE(att.n_followers == 1);

    ivp_result_t res = ivp_attachment_remove(&att, follower);

    IVP_ASSERT_TRUE(res == IVP_OK);
    IVP_ASSERT_TRUE(att.n_followers == 0);

    /* Follower should retain its independent position */
    IVP_ASSERT_TRUE(ivp_fabs(follower->core.pos_world_f_core_last_psi.k[0] - IVP_REAL(1.0)) < TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(attachment_max_followers) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *leader = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_attachment_t att;
    ivp_attachment_init(&att, leader);

    /* Add exactly IVP_ATTACH_MAX_FOLLOWERS (16) followers */
    for (int i = 0; i < IVP_ATTACH_MAX_FOLLOWERS; i++) {
        ivp_object_t *follower = create_test_object(env, (ivp_real_t)i, IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_result_t res = ivp_attachment_add(&att, follower);
        IVP_ASSERT_TRUE(res == IVP_OK);
    }

    IVP_ASSERT_TRUE(att.n_followers == IVP_ATTACH_MAX_FOLLOWERS);

    /* Try to add one more - should fail with capacity exceeded */
    ivp_object_t *extra = create_test_object(env, IVP_REAL(99.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_result_t res = ivp_attachment_add(&att, extra);

    IVP_ASSERT_TRUE(res == IVP_ERR_OUT_OF_MEMORY);
    IVP_ASSERT_TRUE(att.n_followers == IVP_ATTACH_MAX_FOLLOWERS);

    ivp_environment_destroy(env);
}

IVP_TEST(attachment_controller) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *leader = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *follower = create_test_object(env, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_attachment_t att;
    ivp_attachment_init(&att, leader);
    ivp_attachment_add(&att, follower);

    /* Register controller for leader */
    ivp_environment_add_controller(env, 0, ivp_controller_attachment, &att);

    /* Move leader manually */
    ivp_vec3r_set(&leader->core.pos_world_f_core_last_psi, IVP_REAL(10.0), IVP_REAL(5.0), IVP_REAL(0.0));

    /* Step environment - controller should sync */
    for (int i = 0; i < 10; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* Follower should be at leader + offset (10+2, 5+0, 0+0) = (12, 5, 0) */
    IVP_ASSERT_TRUE(ivp_fabs(follower->core.pos_world_f_core_last_psi.k[0] - IVP_REAL(12.0)) < TOL);
    IVP_ASSERT_TRUE(ivp_fabs(follower->core.pos_world_f_core_last_psi.k[1] - IVP_REAL(5.0)) < TOL);

    ivp_environment_destroy(env);
}

/* ========== MERGE CORE TESTS (6) ========== */

IVP_TEST(merge_group_init) {
    ivp_merge_group_t group;
    ivp_merge_group_init(&group, IVP_REAL(100.0));

    IVP_ASSERT_TRUE(group.n_members == 0);
    IVP_ASSERT_TRUE(group.merged == false);
    IVP_ASSERT_TRUE(ivp_fabs(group.break_impulse - IVP_REAL(100.0)) < TOL);
}

IVP_TEST(merge_group_add_merge) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj1 = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj2 = create_test_object(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj3 = create_test_object(env, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Each has mass = 1.0 */
    obj1->core.mass = IVP_REAL(1.0);
    obj2->core.mass = IVP_REAL(1.0);
    obj3->core.mass = IVP_REAL(1.0);

    ivp_merge_group_t group;
    ivp_merge_group_init(&group, IVP_REAL(100.0));

    ivp_merge_group_add(&group, obj1);
    ivp_merge_group_add(&group, obj2);
    ivp_merge_group_add(&group, obj3);

    IVP_ASSERT_TRUE(group.n_members == 3);
    IVP_ASSERT_TRUE(group.merged == false);

    ivp_merge_group_merge(&group);

    IVP_ASSERT_TRUE(group.merged == true);
    /* Merged core mass should be sum = 3.0 */
    IVP_ASSERT_TRUE(ivp_fabs(group.merged_core.mass - IVP_REAL(3.0)) < TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(merge_group_split) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj1 = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj2 = create_test_object(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_merge_group_t group;
    ivp_merge_group_init(&group, IVP_REAL(100.0));

    ivp_merge_group_add(&group, obj1);
    ivp_merge_group_add(&group, obj2);
    ivp_merge_group_merge(&group);

    IVP_ASSERT_TRUE(group.merged == true);

    ivp_merge_group_split(&group);

    IVP_ASSERT_TRUE(group.merged == false);
    /* Individual objects should exist with their original cores */
    IVP_ASSERT_TRUE(obj1->core.mass > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(obj2->core.mass > IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

IVP_TEST(merge_group_velocity_inheritance) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj1 = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj2 = create_test_object(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_merge_group_t group;
    ivp_merge_group_init(&group, IVP_REAL(100.0));

    ivp_merge_group_add(&group, obj1);
    ivp_merge_group_add(&group, obj2);
    ivp_merge_group_merge(&group);

    /* Set merged core velocity */
    ivp_vec3r_set(&group.merged_core.speed, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_merge_group_split(&group);

    /* After split, members should have inherited some velocity */
    /* At minimum, verify they have non-zero velocity in x direction */
    IVP_ASSERT_TRUE(ivp_fabs(obj1->core.speed.k[0]) > TOL || ivp_fabs(obj2->core.speed.k[0]) > TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(merge_group_break_threshold) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj1 = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj2 = create_test_object(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_merge_group_t group;
    ivp_merge_group_init(&group, IVP_REAL(100.0));

    s_break_count = 0;
    ivp_merge_group_set_break_callback(&group, break_cb, NULL);

    ivp_merge_group_add(&group, obj1);
    ivp_merge_group_add(&group, obj2);
    ivp_merge_group_merge(&group);

    IVP_ASSERT_TRUE(group.merged == true);

    /* Apply impulse above threshold (150 > 100) */
    ivp_merge_group_check_break(&group, IVP_REAL(150.0));

    /* Should have split */
    IVP_ASSERT_TRUE(group.merged == false);
    /* Callback should have fired */
    IVP_ASSERT_TRUE(s_break_count == 1);

    ivp_environment_destroy(env);
}

IVP_TEST(merge_group_unbreakable) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj1 = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj2 = create_test_object(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_merge_group_t group;
    /* break_impulse = 0 means unbreakable */
    ivp_merge_group_init(&group, IVP_REAL(0.0));

    s_break_count = 0;
    ivp_merge_group_set_break_callback(&group, break_cb, NULL);

    ivp_merge_group_add(&group, obj1);
    ivp_merge_group_add(&group, obj2);
    ivp_merge_group_merge(&group);

    IVP_ASSERT_TRUE(group.merged == true);

    /* Apply huge impulse - should not break */
    ivp_merge_group_check_break(&group, IVP_REAL(1000.0));

    /* Should still be merged */
    IVP_ASSERT_TRUE(group.merged == true);
    /* Callback should NOT have fired */
    IVP_ASSERT_TRUE(s_break_count == 0);

    ivp_environment_destroy(env);
}

/* ========== HANDLE SYSTEM TESTS (4) ========== */

IVP_TEST(handle_invalid_default) {
    ivp_object_handle_t h = ivp_object_handle_invalid();

    IVP_ASSERT_TRUE(ivp_object_handle_is_valid(h) == false);
    IVP_ASSERT_TRUE(h.slot == UINT32_MAX);
    IVP_ASSERT_TRUE(h.generation == 0);
}

IVP_TEST(handle_is_valid) {
    ivp_environment_t *env = create_test_env();

    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t.orientation = ivp_quatr_identity();

    ivp_object_handle_t handle;
    ivp_object_t *obj = ivp_environment_create_object_with_handle(env, &t, NULL, &handle);

    IVP_ASSERT_TRUE(obj != NULL);
    IVP_ASSERT_TRUE(ivp_object_handle_is_valid(handle) == true);

    /* Destroy object */
    ivp_environment_destroy_object(env, obj);

    /* Handle should now be stale/invalid (generation mismatch) */
    ivp_object_t *retrieved = ivp_environment_get_object_by_handle(env, handle);
    IVP_ASSERT_TRUE(retrieved == NULL);

    ivp_environment_destroy(env);
}

IVP_TEST(handle_get_object) {
    ivp_environment_t *env = create_test_env();

    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    ivp_vec3r_set(&t.position, IVP_REAL(5.0), IVP_REAL(3.0), IVP_REAL(2.0));
    t.orientation = ivp_quatr_identity();

    ivp_object_handle_t handle;
    ivp_object_t *obj = ivp_environment_create_object_with_handle(env, &t, NULL, &handle);

    /* Retrieve via handle */
    ivp_object_t *retrieved = ivp_environment_get_object_by_handle(env, handle);

    IVP_ASSERT_TRUE(retrieved == obj);
    IVP_ASSERT_TRUE(ivp_fabs(retrieved->core.pos_world_f_core_last_psi.k[0] - IVP_REAL(5.0)) < TOL);
    IVP_ASSERT_TRUE(ivp_fabs(retrieved->core.pos_world_f_core_last_psi.k[1] - IVP_REAL(3.0)) < TOL);
    IVP_ASSERT_TRUE(ivp_fabs(retrieved->core.pos_world_f_core_last_psi.k[2] - IVP_REAL(2.0)) < TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(handle_destroy_by_handle) {
    ivp_environment_t *env = create_test_env();

    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t.orientation = ivp_quatr_identity();

    ivp_object_handle_t handle;
    ivp_environment_create_object_with_handle(env, &t, NULL, &handle);

    IVP_ASSERT_TRUE(ivp_object_handle_is_valid(handle) == true);

    /* Destroy by handle */
    bool success = ivp_environment_destroy_object_by_handle(env, handle);

    IVP_ASSERT_TRUE(success == true);

    /* Object should be gone */
    ivp_object_t *retrieved = ivp_environment_get_object_by_handle(env, handle);
    IVP_ASSERT_TRUE(retrieved == NULL);

    /* Try to destroy again - should fail */
    bool success2 = ivp_environment_destroy_object_by_handle(env, handle);
    IVP_ASSERT_TRUE(success2 == false);

    ivp_environment_destroy(env);
}

/* ========== MAIN ========== */

int main(void) {
    /* Attachment tests */
    IVP_REGISTER(attachment_init);
    IVP_REGISTER(attachment_add_follower);
    IVP_REGISTER(attachment_sync);
    IVP_REGISTER(attachment_remove);
    IVP_REGISTER(attachment_max_followers);
    IVP_REGISTER(attachment_controller);

    /* Merge core tests */
    IVP_REGISTER(merge_group_init);
    IVP_REGISTER(merge_group_add_merge);
    IVP_REGISTER(merge_group_split);
    IVP_REGISTER(merge_group_velocity_inheritance);
    IVP_REGISTER(merge_group_break_threshold);
    IVP_REGISTER(merge_group_unbreakable);

    /* Handle system tests */
    IVP_REGISTER(handle_invalid_default);
    IVP_REGISTER(handle_is_valid);
    IVP_REGISTER(handle_get_object);
    IVP_REGISTER(handle_destroy_by_handle);

    IVP_RUN_ALL();
}
