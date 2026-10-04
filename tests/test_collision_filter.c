/* test_collision_filter.c — Tests for collision filter system */
#include "ivp_test.h"
#include <ivp/collision/ivp_collision_filter.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/builder/ivp_build_surface.h>
#include <stdint.h>
#include <string.h>

/* Helper: create a simple environment with zero gravity */
static ivp_environment_t *make_env(void) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

/* Helper: create a simple ball object */
static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t x) {
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.ball_radius = IVP_REAL(0.5);
    tmpl.mass = IVP_REAL(1.0);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor,
                  IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&tmpl.position, x, IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create_object(env, &tmpl, NULL);
}

static void step(ivp_environment_t *env, int count) {
    for (int i = 0; i < count; ++i) {
        ivp_environment_simulate_one_psi(env);
    }
}

IVP_TEST(allow_all_default) {
    ivp_collision_filter_t f = ivp_collision_filter_allow_all();
    IVP_ASSERT_EQ(f.type, IVP_CFILTER_ALLOW_ALL);
    /* Should always return true */
    IVP_ASSERT_TRUE(ivp_collision_filter_check(&f, NULL, NULL));
}

IVP_TEST(group_filter_same_group_skips) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    a->collision_group = 5;
    b->collision_group = 5;

    ivp_collision_filter_t f = ivp_collision_filter_group();
    /* Same group → skip */
    IVP_ASSERT_FALSE(ivp_collision_filter_check(&f, a, b));

    /* Different group → allow */
    b->collision_group = 3;
    IVP_ASSERT_TRUE(ivp_collision_filter_check(&f, a, b));

    /* Zero group → allow (zero is "no group") */
    a->collision_group = 0;
    b->collision_group = 0;
    IVP_ASSERT_TRUE(ivp_collision_filter_check(&f, a, b));

    ivp_environment_destroy(env);
}

IVP_TEST(layer_mask_filter) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    /* Both default 0xFFFFFFFF → shared bits → allow */
    ivp_collision_filter_t f = ivp_collision_filter_layer_mask();
    IVP_ASSERT_TRUE(ivp_collision_filter_check(&f, a, b));

    /* No shared bits → skip */
    a->layer_mask = 0x00FF;
    b->layer_mask = 0xFF00;
    IVP_ASSERT_FALSE(ivp_collision_filter_check(&f, a, b));

    /* Some shared bits → allow */
    b->layer_mask = 0x0F00;
    a->layer_mask = 0x0F00;
    IVP_ASSERT_TRUE(ivp_collision_filter_check(&f, a, b));

    ivp_environment_destroy(env);
}

IVP_TEST(exclusive_pair_filter) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0));
    ivp_object_t *c = make_ball(env, IVP_REAL(2.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);
    IVP_ASSERT_NOT_NULL(c);

    ivp_exclusive_pair_filter_t epf;
    IVP_ASSERT_TRUE(ivp_ok(ivp_exclusive_pair_filter_init(&epf, NULL)));

    /* Initially all pairs allowed */
    IVP_ASSERT_TRUE(ivp_collision_filter_check(&epf.base, a, b));
    IVP_ASSERT_TRUE(ivp_collision_filter_check(&epf.base, a, c));

    /* Disable a-b pair */
    ivp_exclusive_pair_filter_disable(&epf, a, b);
    IVP_ASSERT_FALSE(ivp_collision_filter_check(&epf.base, a, b));
    IVP_ASSERT_FALSE(ivp_collision_filter_check(&epf.base, b, a)); /* order-independent */
    IVP_ASSERT_TRUE(ivp_collision_filter_check(&epf.base, a, c));  /* other pairs unaffected */

    /* Re-enable a-b pair */
    ivp_exclusive_pair_filter_enable(&epf, a, b);
    IVP_ASSERT_TRUE(ivp_collision_filter_check(&epf.base, a, b));

    ivp_exclusive_pair_filter_destroy(&epf);
    ivp_environment_destroy(env);
}

IVP_TEST(exclusive_pair_filter_hash_collision_regression) {
    /* Regression for old 64-bit key compression collisions. */
#if UINTPTR_MAX <= 0xFFFFFFFFu
    return;
#else

    const struct ivp_object *a0 = (const struct ivp_object *)(uintptr_t)0x0000000000000000ULL;
    const struct ivp_object *b0 = (const struct ivp_object *)(uintptr_t)0x0000000200000001ULL;
    const struct ivp_object *a1 = (const struct ivp_object *)(uintptr_t)0x0000000000000001ULL;
    const struct ivp_object *b1 = (const struct ivp_object *)(uintptr_t)0x0000000300000001ULL;

    ivp_exclusive_pair_filter_t epf;
    IVP_ASSERT_TRUE(ivp_ok(ivp_exclusive_pair_filter_init(&epf, NULL)));

    ivp_exclusive_pair_filter_disable(&epf, a0, b0);
    IVP_ASSERT_FALSE(ivp_collision_filter_check(&epf.base, a0, b0));
    IVP_ASSERT_TRUE(ivp_collision_filter_check(&epf.base, a1, b1));

    ivp_exclusive_pair_filter_destroy(&epf);
#endif
}

IVP_TEST(meta_filter) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_collision_filter_t filters[2];
    filters[0] = ivp_collision_filter_group();
    filters[1] = ivp_collision_filter_layer_mask();

    ivp_meta_collision_filter_t meta_storage;
    ivp_collision_filter_t meta = ivp_collision_filter_meta(&meta_storage, filters, 2);

    /* Both pass → allow */
    a->collision_group = 1;
    b->collision_group = 2;
    a->layer_mask = 0xFF;
    b->layer_mask = 0xFF;
    IVP_ASSERT_TRUE(ivp_collision_filter_check(&meta, a, b));

    /* Group fails → skip */
    b->collision_group = 1;
    IVP_ASSERT_FALSE(ivp_collision_filter_check(&meta, a, b));

    /* Group passes but layer fails → skip */
    b->collision_group = 2;
    a->layer_mask = 0x0F;
    b->layer_mask = 0xF0;
    IVP_ASSERT_FALSE(ivp_collision_filter_check(&meta, a, b));

    ivp_environment_destroy(env);
}

IVP_TEST(custom_callback_filter) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0));

    /* Custom callback that always rejects */
    ivp_collision_filter_t f = ivp_collision_filter_callback(
        (ivp_collision_check_fn)(void (*)(void))NULL, NULL);
    /* NULL fn → falls through to return true */
    IVP_ASSERT_TRUE(ivp_collision_filter_check(&f, a, b));

    ivp_environment_destroy(env);
}

IVP_TEST(runtime_layer_mask_setter_rechecks_existing_pair) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    /* Start close enough that the broad phase creates a mindist, then let it
     * schedule the approaching pair before changing the filter state. */
    ivp_object_t *a = make_ball(env, IVP_REAL(-0.75));
    ivp_object_t *b = make_ball(env, IVP_REAL(0.75));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);
    ivp_vec3r_set(&a->physical_core->speed,
                  IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&b->physical_core->speed,
                  IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    step(env, 1);

    ivp_object_set_layer_mask(a, 0);
    step(env, 60);

    ivp_vec3r_t pa, pb;
    ivp_object_get_position(a, &pa);
    ivp_object_get_position(b, &pb);
    IVP_ASSERT_TRUE(pa.k[0] > pb.k[0]);
    IVP_ASSERT_TRUE(a->physical_core->speed.k[0] > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(b->physical_core->speed.k[0] < IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

IVP_TEST(runtime_collision_group_setter_recreates_filtered_pair) {
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(-0.75));
    ivp_object_t *b = make_ball(env, IVP_REAL(0.75));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);
    ivp_vec3r_set(&a->physical_core->speed,
                  IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&b->physical_core->speed,
                  IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_object_set_collision_group(a, 1);
    ivp_object_set_collision_group(b, 1);
    step(env, 1);
    ivp_object_set_collision_group(b, 2);
    step(env, 60);

    ivp_vec3r_t pa, pb;
    ivp_object_get_position(a, &pa);
    ivp_object_get_position(b, &pb);
    IVP_ASSERT_TRUE(pa.k[0] < pb.k[0]);
    IVP_ASSERT_TRUE(a->physical_core->speed.k[0] < IVP_REAL(0.0));
    IVP_ASSERT_TRUE(b->physical_core->speed.k[0] > IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

/* ── Registration ──────────────────────────────────────────────────────── */

int main(void) {
    IVP_REGISTER(allow_all_default);
    IVP_REGISTER(group_filter_same_group_skips);
    IVP_REGISTER(layer_mask_filter);
    IVP_REGISTER(exclusive_pair_filter);
    IVP_REGISTER(exclusive_pair_filter_hash_collision_regression);
    IVP_REGISTER(meta_filter);
    IVP_REGISTER(custom_callback_filter);
    IVP_REGISTER(runtime_layer_mask_setter_rechecks_existing_pair);
    IVP_REGISTER(runtime_collision_group_setter_recreates_filtered_pair);
    IVP_RUN_ALL();
}
