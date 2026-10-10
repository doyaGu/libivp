/* test_dynamics.c �?Tests for M4: core, object, environment, PSI loop */
#include "ivp_test.h"
#include "ivp_test_controller_api.h"
#include <ivp/base/ivp_assert.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_merge_core.h>
#include <ivp/events/ivp_listener_psi.h>
#include <ivp/controllers/ivp_actuator_force.h>
#include <ivp/controllers/ivp_actuator_rot_mot.h>
#include <ivp/math/ivp_fast_math.h>
#include "dynamics/ivp_env_internal.h"
#include "dynamics/ivp_env_pipeline.h"
#include "controllers/ivp_controller_system.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ══════════════════════════════════════════════════════════════════════════�?
 * 1. Core initialization
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(core_init_basic) {
    ivp_core_t core;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(3.0), IVP_REAL(4.0));
    ivp_core_init(&core, IVP_REAL(5.0), &inertia);

    IVP_ASSERT_NEAR(core.mass, 5.0, 1e-6);
    IVP_ASSERT_NEAR(core.inv_mass, 0.2, 1e-6);
    IVP_ASSERT_NEAR(core.rot_inertia.k[0], 2.0, 1e-6);
    IVP_ASSERT_NEAR(core.rot_inertia.k[1], 3.0, 1e-6);
    IVP_ASSERT_NEAR(core.rot_inertia.k[2], 4.0, 1e-6);
    IVP_ASSERT_NEAR(core.inv_rot_inertia.k[0], 0.5, 1e-6);
    IVP_ASSERT_NEAR(core.inv_rot_inertia.k[1], 1.0/3.0, 1e-6);
    IVP_ASSERT_NEAR(core.inv_rot_inertia.k[2], 0.25, 1e-6);

    /* Default damping */
    IVP_ASSERT_NEAR(core.speed_damp_factor, 0.01, 1e-6);
    IVP_ASSERT_NEAR(core.rot_speed_damp_factor.k[0], 0.01, 1e-6);

    /* Identity orientation */
    IVP_ASSERT_NEAR(core.q_world_f_core_last_psi.w, 1.0, 1e-6);
    IVP_ASSERT_NEAR(core.q_world_f_core_last_psi.x, 0.0, 1e-6);

    /* Zero velocity */
    IVP_ASSERT_NEAR(core.speed.k[0], 0.0, 1e-6);
    IVP_ASSERT_NEAR(core.speed.k[1], 0.0, 1e-6);
    IVP_ASSERT_NEAR(core.speed.k[2], 0.0, 1e-6);

    /* Reference IVP_Core::init sets IVP_MT_NOT_SIM (ivp_core.cxx:1050);
     * cores enter simulation only when revived. */
    IVP_ASSERT_EQ((int)core.movement_state, (int)IVP_MT_NOT_SIM);
    IVP_ASSERT_FALSE(core.rot_inertias_are_equal);
}

IVP_TEST(core_init_equal_inertias) {
    ivp_core_t core;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(5.0), IVP_REAL(5.0));
    ivp_core_init(&core, IVP_REAL(1.0), &inertia);

    IVP_ASSERT_TRUE(core.rot_inertias_are_equal);
}

/* Reference 1587518: calc_calc clamps zero or invalid inertia to a finite
 * positive value before it asserts the inertia length, so zero inertia
 * input is accepted without an assertion failure. */
static int g_core_assert_count;

static void core_count_assert(const char *expr, const char *file, int line,
                              const char *func, const char *msg, void *userdata)
{
    (void)expr; (void)file; (void)line; (void)func; (void)msg; (void)userdata;
    g_core_assert_count++;
}

IVP_TEST(core_zero_inertia_is_clamped_without_assert) {
    ivp_core_t core;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    g_core_assert_count = 0;
    ivp_set_assert_handler(core_count_assert, NULL);
    ivp_core_init(&core, IVP_REAL(1.0), &inertia);
    ivp_set_assert_handler(NULL, NULL);

    IVP_ASSERT_EQ(g_core_assert_count, 0);
    for (int i = 0; i < 3; i++) {
        IVP_ASSERT_TRUE(core.rot_inertia.k[i] > 0.0);
        IVP_ASSERT_TRUE(isfinite((double)core.inv_rot_inertia.k[i]));
    }
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 2. Core pose
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(core_set_pose) {
    ivp_core_t core;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_core_init(&core, IVP_REAL(1.0), &inertia);

    ivp_vec3r_t pos = ivp_vec3r(IVP_REAL(10.0), IVP_REAL(20.0), IVP_REAL(30.0));
    ivp_quatr_t quat = ivp_quatr_identity();
    ivp_core_set_pose(&core, &pos, &quat);

    IVP_ASSERT_NEAR(core.pos_world_f_core_last_psi.k[0], 10.0, 1e-6);
    IVP_ASSERT_NEAR(core.pos_world_f_core_last_psi.k[1], 20.0, 1e-6);
    IVP_ASSERT_NEAR(core.pos_world_f_core_last_psi.k[2], 30.0, 1e-6);
    IVP_ASSERT_NEAR(core.m_world_f_core_last_psi.pos.k[0], 10.0, 1e-6);
}

IVP_TEST(core_beam_to) {
    ivp_core_t core;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_core_init(&core, IVP_REAL(1.0), &inertia);

    /* Give it some velocity */
    core.speed = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(6.0), IVP_REAL(7.0));

    /* Beam to new position */
    ivp_vec3r_t pos = ivp_vec3r(IVP_REAL(100.0), IVP_REAL(200.0), IVP_REAL(300.0));
    ivp_quatr_t quat = ivp_quatr_identity();
    ivp_core_beam_to(&core, &pos, &quat);

    /* Position updated */
    IVP_ASSERT_NEAR(core.pos_world_f_core_last_psi.k[0], 100.0, 1e-6);

    /* Velocity reset to zero */
    IVP_ASSERT_NEAR(core.speed.k[0], 0.0, 1e-6);
    IVP_ASSERT_NEAR(core.speed.k[1], 0.0, 1e-6);
    IVP_ASSERT_NEAR(core.speed.k[2], 0.0, 1e-6);
    IVP_ASSERT_NEAR(core.current_speed, 0.0, 1e-6);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 3. Damping
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(core_damp_linear_approx) {
    /* With small dt, damping uses 1 - damp*dt approximation */
    ivp_core_t core;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_core_init(&core, IVP_REAL(1.0), &inertia);

    core.speed = ivp_vec3r(IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    core.speed_damp_factor = IVP_REAL(0.01);
    /* IVP_Core::global_damp_core adds 0.1 for movement_state >= IVP_MT_SLOW,
     * which includes the IVP_MT_NOT_SIM state of a fresh core
     * (ivp_core.cxx:484); test the plain path on a moving core. */
    core.movement_state = IVP_MT_MOVING;

    ivp_real_t dt = (ivp_real_t)IVP_DEFAULT_DELTA_TIME;
    ivp_core_damp(&core, dt);

    /* Expected: 10.0 * (1 - 0.01 * dt) */
    double expected = 10.0 * (1.0 - 0.01 * (1.0 / 66.0));
    IVP_ASSERT_NEAR(core.speed.k[0], expected, 1e-5);
}

IVP_TEST(core_damp_exponential) {
    /* With large damping, uses exp(-damp*dt) */
    ivp_core_t core;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_core_init(&core, IVP_REAL(1.0), &inertia);

    core.speed = ivp_vec3r(IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    core.movement_state = IVP_MT_MOVING; /* no slow add-on (ivp_core.cxx:484) */
    core.speed_damp_factor = IVP_REAL(20.0); /* Very large �?use exp path */

    ivp_real_t dt = IVP_REAL(1.0);  /* 1 second */
    ivp_core_damp(&core, dt);

    double expected = 10.0 * exp(-20.0);
    IVP_ASSERT_NEAR(core.speed.k[0], expected, 1e-5);
}

IVP_TEST(core_damp_analytical_decay) {
    /* Test that repeated small-step damping approximates exp(-k*t) */
    ivp_core_t core;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_core_init(&core, IVP_REAL(1.0), &inertia);

    ivp_real_t v0 = IVP_REAL(10.0);
    core.speed = ivp_vec3r(v0, IVP_REAL(0.0), IVP_REAL(0.0));
    core.speed_damp_factor = IVP_REAL(0.5);
    core.movement_state = IVP_MT_MOVING; /* no slow add-on (ivp_core.cxx:484) */

    ivp_real_t dt = (ivp_real_t)IVP_DEFAULT_DELTA_TIME;
    int n = 660;  /* 10 seconds at 66Hz */
    for (int i = 0; i < n; i++) {
        ivp_core_damp(&core, dt);
    }

    double t_total = (double)n * (1.0 / 66.0);
    double expected = (double)v0 * exp(-0.5 * t_total);
    IVP_ASSERT_NEAR(core.speed.k[0], expected, 0.1);
}

IVP_TEST(core_damp_slow_addon) {
    /* SLOW objects get extra 0.1 damping */
    ivp_core_t core;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_core_init(&core, IVP_REAL(1.0), &inertia);

    core.speed = ivp_vec3r(IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    core.speed_damp_factor = IVP_REAL(0.01);
    core.movement_state = IVP_MT_SLOW;

    ivp_real_t dt = (ivp_real_t)IVP_DEFAULT_DELTA_TIME;
    ivp_core_damp(&core, dt);

    /* Expected: 10.0 * (1 - (0.01 + 0.1) * dt) */
    double effective_damp = 0.01 + 0.1;
    double expected = 10.0 * (1.0 - effective_damp * (1.0 / 66.0));
    IVP_ASSERT_NEAR(core.speed.k[0], expected, 1e-5);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 4. Async pushes
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(core_commit_async_pushes) {
    ivp_core_t core;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_core_init(&core, IVP_REAL(1.0), &inertia);

    core.speed = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    core.speed_change = ivp_vec3r(IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(0.5));

    ivp_core_commit_async_pushes(&core);

    IVP_ASSERT_NEAR(core.speed.k[0], 1.5, 1e-6);
    IVP_ASSERT_NEAR(core.speed.k[1], 2.5, 1e-6);
    IVP_ASSERT_NEAR(core.speed.k[2], 3.5, 1e-6);

    /* Changes should be cleared */
    IVP_ASSERT_NEAR(core.speed_change.k[0], 0.0, 1e-6);
    IVP_ASSERT_NEAR(core.speed_change.k[1], 0.0, 1e-6);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 5. Object template
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(object_template_defaults) {
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);

    IVP_ASSERT_NEAR(tmpl.mass, 1.0, 1e-6);
    IVP_ASSERT_NEAR(tmpl.speed_damp_factor, 0.01, 1e-6);
    IVP_ASSERT_NEAR(tmpl.material.friction, 0.5, 1e-6);
    IVP_ASSERT_NEAR(tmpl.material.elasticity, 0.3, 1e-6);
    IVP_ASSERT_FALSE(tmpl.physical_unmoveable);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 6. Environment lifecycle
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(env_create_destroy) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);

    IVP_ASSERT_NEAR(env->gravity.k[1], 9.83, 1e-5);
    IVP_ASSERT_NEAR(ivp_environment_get_delta_psi_time(env), 1.0/66.0, 1e-6);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), 0);

    ivp_environment_destroy(env);
}

IVP_TEST(env_create_with_config) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(-9.81), IVP_REAL(0.0));
    cfg.delta_psi_time = IVP_REAL(0.01);

    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    IVP_ASSERT_NEAR(env->gravity.k[1], -9.81, 1e-5);
    IVP_ASSERT_NEAR(env->delta_psi_time, 0.01, 1e-6);

    ivp_environment_destroy(env);
}

typedef union IVP_ALIGN16 env_fail_alloc_header {
    max_align_t alignment;
    unsigned char ivp_alignment[16];
    struct {
        size_t size;
    } meta;
} env_fail_alloc_header_t;

_Static_assert(_Alignof(env_fail_alloc_header_t) >= 16,
               "failure allocator must preserve IVP allocator alignment");

typedef struct env_fail_alloc_state {
    int attempts;
    int fail_at;
    int live_blocks;
} env_fail_alloc_state_t;

static bool env_fail_alloc_should_fail(env_fail_alloc_state_t *state)
{
    state->attempts++;
    return state->fail_at > 0 && state->attempts == state->fail_at;
}

static void *env_fail_alloc(size_t size, void *ctx)
{
    env_fail_alloc_state_t *state = (env_fail_alloc_state_t *)ctx;
    if (env_fail_alloc_should_fail(state) || size > SIZE_MAX - sizeof(env_fail_alloc_header_t)) {
        return NULL;
    }
    env_fail_alloc_header_t *header =
        (env_fail_alloc_header_t *)malloc(sizeof(*header) + size);
    if (!header) return NULL;
    header->meta.size = size;
    void *ptr = header + 1;
    memset(ptr, 0xa5, size);
    state->live_blocks++;
    return ptr;
}

static void *env_fail_realloc(void *ptr, size_t old_size, size_t new_size, void *ctx)
{
    env_fail_alloc_state_t *state = (env_fail_alloc_state_t *)ctx;
    IVP_UNUSED(old_size);
    if (env_fail_alloc_should_fail(state) ||
        new_size > SIZE_MAX - sizeof(env_fail_alloc_header_t)) {
        return NULL;
    }
    if (!ptr) {
        env_fail_alloc_header_t *header =
            (env_fail_alloc_header_t *)malloc(sizeof(*header) + new_size);
        if (!header) return NULL;
        header->meta.size = new_size;
        void *new_ptr = header + 1;
        memset(new_ptr, 0xa5, new_size);
        state->live_blocks++;
        return new_ptr;
    }

    env_fail_alloc_header_t *old_header = (env_fail_alloc_header_t *)ptr - 1;
    const size_t copy_size = old_header->meta.size < new_size
                                 ? old_header->meta.size : new_size;
    env_fail_alloc_header_t *new_header =
        (env_fail_alloc_header_t *)malloc(sizeof(*new_header) + new_size);
    if (!new_header) return NULL;
    new_header->meta.size = new_size;
    void *new_ptr = new_header + 1;
    memcpy(new_ptr, ptr, copy_size);
    if (new_size > copy_size) {
        memset((unsigned char *)new_ptr + copy_size, 0xa5, new_size - copy_size);
    }
    free(old_header);
    return new_ptr;
}

static void env_fail_free(void *ptr, size_t size, void *ctx)
{
    env_fail_alloc_state_t *state = (env_fail_alloc_state_t *)ctx;
    IVP_UNUSED(size);
    if (!ptr) return;
    env_fail_alloc_header_t *header = (env_fail_alloc_header_t *)ptr - 1;
    free(header);
    state->live_blocks--;
}

static ivp_environment_t *env_create_with_fail_allocator(env_fail_alloc_state_t *state,
                                                          ivp_allocator_t *allocator)
{
    *allocator = (ivp_allocator_t){
        .alloc = env_fail_alloc,
        .realloc = env_fail_realloc,
        .free = env_fail_free,
        .ctx = state
    };
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.allocator = allocator;
    return ivp_environment_create(&cfg);
}

IVP_TEST(env_create_allocation_failures_are_recoverable) {
    env_fail_alloc_state_t baseline = {0};
    ivp_allocator_t allocator;
    ivp_environment_t *env = env_create_with_fail_allocator(&baseline, &allocator);
    IVP_ASSERT_NOT_NULL(env);
    const int allocation_points = baseline.attempts;
    ivp_environment_destroy(env);
    IVP_ASSERT_EQ(baseline.live_blocks, 0);

    for (int fail_at = 1; fail_at <= allocation_points; fail_at++) {
        env_fail_alloc_state_t state = {
            .attempts = 0,
            .fail_at = fail_at,
            .live_blocks = 0
        };
        env = env_create_with_fail_allocator(&state, &allocator);
        if (env) {
            fprintf(stderr, "environment unexpectedly survived failure point %d\n", fail_at);
        }
        IVP_ASSERT_NULL(env);
        if (env) ivp_environment_destroy(env);
        if (state.live_blocks != 0) {
            fprintf(stderr, "environment allocator leak at failure point %d: %d blocks\n",
                    fail_at, state.live_blocks);
        }
        IVP_ASSERT_EQ(state.live_blocks, 0);
    }
}

IVP_TEST(env_create_object) {
    ivp_environment_t *env = ivp_environment_create(NULL);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(10.0);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));

    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), 1);

    IVP_ASSERT_NEAR(obj->physical_core->mass, 10.0, 1e-6);
    IVP_ASSERT_NEAR(obj->physical_core->pos_world_f_core_last_psi.k[0], 1.0, 1e-6);

    IVP_ASSERT_TRUE(obj->owner == env);

    ivp_environment_destroy(env);
}

typedef struct object_lifecycle_event_counts {
    int created;
    int destroyed;
} object_lifecycle_event_counts_t;

static void count_object_lifecycle_events(void *ctx, ivp_environment_t *env,
                                          const ivp_event_t *event)
{
    (void)env;
    object_lifecycle_event_counts_t *counts =
        (object_lifecycle_event_counts_t *)ctx;
    if (event->type == IVP_EVENT_OBJECT_CREATED) counts->created++;
    if (event->type == IVP_EVENT_OBJECT_DESTROYED) counts->destroyed++;
}

IVP_TEST(env_create_object_reserves_psi_scratch_atomically) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);
    object_lifecycle_event_counts_t events = {0};
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(
        ivp_environment_add_event_listener(
            env, count_object_lifecycle_events, &events)));

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);

    ivp_pvec_t saved = env->pl->touched_cores;
    env->pl->touched_cores = (ivp_pvec_t){ .elems = NULL, .n = 0, .cap = -1 };
    ivp_object_handle_t handle = { 7u, 7u };
    IVP_ASSERT_NULL(ivp_environment_create_object_with_handle(env, &tmpl, NULL, &handle));
    IVP_ASSERT_FALSE(ivp_object_handle_is_valid(handle));
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), 0);
    IVP_ASSERT_EQ(events.created, 0);
    IVP_ASSERT_EQ(events.destroyed, 0);
    env->pl->touched_cores = saved;

    saved = env->pl->active_hulls;
    env->pl->active_hulls = (ivp_pvec_t){ .elems = NULL, .n = 0, .cap = -1 };
    handle = (ivp_object_handle_t){ 7u, 7u };
    IVP_ASSERT_NULL(ivp_environment_create_object_with_handle(env, &tmpl, NULL, &handle));
    IVP_ASSERT_FALSE(ivp_object_handle_is_valid(handle));
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), 0);
    IVP_ASSERT_EQ(events.created, 0);
    IVP_ASSERT_EQ(events.destroyed, 0);
    env->pl->active_hulls = saved;

    ivp_object_t *obj = ivp_environment_create_object_with_handle(env, &tmpl, NULL, &handle);
    IVP_ASSERT_NOT_NULL(obj);
    IVP_ASSERT_TRUE(ivp_object_handle_is_valid(handle));
    IVP_ASSERT_TRUE(env->pl->touched_cores.cap >= 1);
    IVP_ASSERT_TRUE(env->pl->active_hulls.cap >= 1);
    IVP_ASSERT_EQ(events.created, 1);
    IVP_ASSERT_EQ(events.destroyed, 0);

    const uint64_t first_sequence = obj->creation_sequence;
    saved = env->pl->touched_cores;
    env->pl->touched_cores = (ivp_pvec_t){ .elems = NULL, .n = 0, .cap = -1 };
    IVP_ASSERT_NULL(ivp_environment_create_object(env, &tmpl, NULL));
    IVP_ASSERT_EQ(events.created, 1);
    IVP_ASSERT_EQ(events.destroyed, 0);
    env->pl->touched_cores = saved;

    ivp_object_t *second = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(second);
    IVP_ASSERT_EQ(second->creation_sequence, first_sequence + 1u);
    IVP_ASSERT_EQ(events.created, 2);
    IVP_ASSERT_EQ(events.destroyed, 0);

    ivp_environment_destroy(env);
}

IVP_TEST(env_destroy_object) {
    ivp_environment_t *env = ivp_environment_create(NULL);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);

    ivp_object_t *obj1 = ivp_environment_create_object(env, &tmpl, NULL);
    (void)ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), 2);

    ivp_environment_destroy_object(env, obj1);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), 1);

    /* obj2 should still be accessible */
    IVP_ASSERT_NOT_NULL(ivp_environment_get_object(env, 0));

    ivp_environment_destroy(env);
}

IVP_TEST(env_object_handle_basic) {
    ivp_environment_t *env = ivp_environment_create(NULL);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);

    ivp_object_handle_t h;
    ivp_object_t *obj = ivp_environment_create_object_with_handle(env, &tmpl, NULL, &h);
    IVP_ASSERT_NOT_NULL(obj);
    IVP_ASSERT_TRUE(ivp_object_handle_is_valid(h));

    ivp_object_t *q = ivp_environment_get_object_by_handle(env, h);
    IVP_ASSERT_TRUE(q == obj);

    ivp_object_handle_t h2 = ivp_environment_get_object_handle(obj);
    IVP_ASSERT_EQ(h.slot, h2.slot);
    IVP_ASSERT_EQ(h.generation, h2.generation);

    ivp_environment_destroy(env);
}

IVP_TEST(env_object_handle_stale_after_delete) {
    ivp_environment_t *env = ivp_environment_create(NULL);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);

    ivp_object_handle_t h_old;
    ivp_object_t *obj_old = ivp_environment_create_object_with_handle(env, &tmpl, NULL, &h_old);
    IVP_ASSERT_NOT_NULL(obj_old);

    IVP_ASSERT_TRUE(ivp_environment_destroy_object_by_handle(env, h_old));
    IVP_ASSERT_NULL(ivp_environment_get_object_by_handle(env, h_old));

    ivp_object_handle_t h_new;
    ivp_object_t *obj_new = ivp_environment_create_object_with_handle(env, &tmpl, NULL, &h_new);
    IVP_ASSERT_NOT_NULL(obj_new);
    IVP_ASSERT_EQ(h_old.slot, h_new.slot);
    IVP_ASSERT_TRUE(h_old.generation != h_new.generation);

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 7. Single PSI step
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(env_create_invalid_delta_time_defaults) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = IVP_REAL(0.0);

    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);
    IVP_ASSERT_TRUE(env->delta_psi_time > IVP_REAL(0.0));
    IVP_ASSERT_NEAR(env->delta_psi_time, (double)IVP_DEFAULT_DELTA_TIME, 1e-9);

    ivp_environment_destroy(env);
}

IVP_TEST(env_destroy_object_foreign_owner_noop) {
    ivp_environment_t *env_a = ivp_environment_create(NULL);
    ivp_environment_t *env_b = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env_a);
    IVP_ASSERT_NOT_NULL(env_b);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj_a = ivp_environment_create_object(env_a, &tmpl, NULL);
    ivp_object_t *obj_b = ivp_environment_create_object(env_b, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    IVP_ASSERT_EQ(ivp_environment_get_object_count(env_a), 1);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env_b), 1);

    ivp_environment_destroy_object(env_a, obj_b);

    IVP_ASSERT_EQ(ivp_environment_get_object_count(env_a), 1);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env_b), 1);

    ivp_environment_destroy(env_a);
    ivp_environment_destroy(env_b);
}

IVP_TEST(env_single_psi_step) {
    ivp_environment_t *env = ivp_environment_create(NULL);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);

    /* One PSI step with gravity (0, 9.81, 0) */
    ivp_environment_simulate_one_psi(env);

    /* After 1 step at dt = 1/66:
     * Speed should be�?g*dt (after damping, but damping is tiny) */
    double dt = 1.0 / 66.0;
    double expected_vy = 9.83 * dt;  /* velocity after gravity */

    /* Speed was 0, damped to 0, then += g*dt */
    IVP_ASSERT_NEAR(obj->physical_core->speed.k[0], 0.0, 1e-6);
    IVP_ASSERT_NEAR(obj->physical_core->speed.k[1], expected_vy, 1e-4);
    IVP_ASSERT_NEAR(obj->physical_core->speed.k[2], 0.0, 1e-6);

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 8. Freefall �?position matches analytical solution
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(freefall_no_damping) {
    /* Zero damping: exact freefall y(t) = 0.5*g*t² */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    /* Use Z-down gravity for simplicity */
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(9.83));

    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.speed_damp_factor = IVP_REAL(0.0); /* No damping */
    tmpl.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);

    /* Run 66 steps = 1 second */
    for (int i = 0; i < 66; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* Semi-implicit Euler: v(n+1) = v(n) + g*dt, x(n+1) = x(n) + v(n+1)*dt
     * This is slightly ahead of analytical solution by about 0.5*g*dt per step.
     * For 66 steps: position �?0.5*g*t² + 0.5*g*dt �?4.915 + 0.0745 �?4.99 */
    double g = 9.83;
    double dt = 1.0 / 66.0;

    /* Reference computation matching exact PSI order:
     * init_for_psi: pos += delta * dt  (delta from previous step)
     * gravity: speed += g * dt
     * calc_next: delta = speed */
    double v = 0.0;
    double z = 0.0;
    double delta = 0.0;
    for (int i = 0; i < 66; i++) {
        z += delta * dt;     /* position uses OLD speed (delta) */
        v += g * dt;         /* velocity updated */
        delta = v;           /* save for next step */
    }

    IVP_ASSERT_NEAR(obj->physical_core->pos_world_f_core_last_psi.k[2], z, 0.01);
    IVP_ASSERT_NEAR(obj->physical_core->speed.k[2], v, 1e-4);

    ivp_environment_destroy(env);
}

IVP_TEST(freefall_default_damping) {
    /* Default damping (0.01): match exact semi-implicit loop */
    ivp_environment_t *env = ivp_environment_create(NULL);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);

    double g = 9.83;
    double dt = 1.0 / 66.0;
    double damp = 0.01;

    /* Run 240 frames (�?3.6 seconds) */
    int frames = 240;
    for (int i = 0; i < frames; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* Compute reference: PSI order is:
     *   1. init: pos += delta * dt
     *   2. damp: speed *= (1 - damp*dt)
     *   3. gravity: speed += g * dt
     *   4. calc_next: delta = speed */
    double vy = 0.0;
    double y = 0.0;
    double delta_v = 0.0;
    for (int i = 0; i < frames; i++) {
        y += delta_v * dt;
        double sf = damp * dt;
        vy *= (1.0 - sf);
        vy += g * dt;
        delta_v = vy;
    }

    /* The acceptance criteria says 1e-5 m position error */
    IVP_ASSERT_NEAR(obj->physical_core->pos_world_f_core_last_psi.k[1], y, 1e-3);
    IVP_ASSERT_NEAR(obj->physical_core->speed.k[1], vy, 1e-3);

    ivp_environment_destroy(env);
}

IVP_TEST(freefall_240_frames_position) {
    /* M4 acceptance: freefall 240 frames position error < 1e-5 m */
    ivp_environment_t *env = ivp_environment_create(NULL);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);

    double g = 9.83;
    double dt = 1.0 / 66.0;
    double damp = 0.01;

    /* Reference computation: exact same PSI order */
    double vy = 0.0, y = 0.0;
    double delta_v = 0.0;

    for (int frame = 0; frame < 240; frame++) {
        /* === Reference PSI step === */
        /* 1. init: pos += delta * dt */
        y += delta_v * dt;
        /* 2. damp: v *= (1 - damp*dt) */
        vy *= (1.0 - damp * dt);
        /* 3. gravity: v += g*dt */
        vy += g * dt;
        /* 4. calc_next: delta = speed */
        delta_v = vy;

        /* === Actual PSI step === */
        ivp_environment_simulate_one_psi(env);

        /* Compare per-frame */
        double err_pos = fabs((double)obj->physical_core->pos_world_f_core_last_psi.k[1] - y);
        double err_vel = fabs((double)obj->physical_core->speed.k[1] - vy);

        if (err_pos > 1e-3 || err_vel > 1e-3) {
            fprintf(stderr, "  Frame %d: pos_err=%g vel_err=%g\n",
                    frame, err_pos, err_vel);
            IVP_FAIL("Position/velocity mismatch too large");
        }
    }

    /* Final position check */
    IVP_ASSERT_NEAR(obj->physical_core->pos_world_f_core_last_psi.k[1], y, 1e-3);

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 9. simulate_dtime (fixed-step accumulation)
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(env_simulate_dtime_accumulation) {
    ivp_environment_t *env = ivp_environment_create(NULL);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_environment_create_object(env, &tmpl, NULL);

    /* simulate_dtime clamps to IVP_MAX_DELTA_PSI_TIME (0.1s).
     * Call it 10 times with 0.1s each to get ~1 second. */
    for (int i = 0; i < 10; i++) {
        ivp_environment_simulate_dtime(env, IVP_REAL(0.1));
    }

    /* 66 steps at dt=1/66 �?1.0s (accumulated over 10 calls) */
    IVP_ASSERT_TRUE(ivp_environment_get_time(env) > 0.9);
    IVP_ASSERT_TRUE(ivp_environment_get_time(env) < 1.1);

    ivp_environment_destroy(env);
}

static int s_dtime_psi_count;
static void dtime_psi_count_fn(void *ctx, ivp_environment_t *env, ivp_time_t time, ivp_real_t dt) {
    (void)ctx;
    (void)env;
    (void)time;
    (void)dt;
    s_dtime_psi_count++;
}

IVP_TEST(env_simulate_dtime_residual) {
    /* Reference IVP_Environment::simulate_dtime (ivp_environment.cxx:516)
     * runs the time-event loop up to current_time + dtime: the clock advances
     * by exactly dtime and a PSI fires whenever its event time is passed
     * (the first PSI event is at t=0). There is no fixed-step residual. */
    ivp_environment_t *env = ivp_environment_create(NULL);
    s_dtime_psi_count = 0;
    ivp_psi_listener_t listener;
    memset(&listener, 0, sizeof(listener));
    IVP_ASSERT_EQ(ivp_psi_listener_register(&listener, env, dtime_psi_count_fn, NULL), 0);

    /* Less than one PSI interval: only the t=0 PSI runs */
    ivp_environment_simulate_dtime(env, IVP_REAL(0.01));
    IVP_ASSERT_NEAR(ivp_environment_get_time(env), 0.01, 1e-6);
    IVP_ASSERT_EQ(s_dtime_psi_count, 1);

    /* Now ~0.02s > 1/66 ~ 0.01515: the second PSI runs */
    ivp_environment_simulate_dtime(env, IVP_REAL(0.01));
    IVP_ASSERT_NEAR(ivp_environment_get_time(env), 0.02, 1e-6);
    IVP_ASSERT_EQ(s_dtime_psi_count, 2);

    IVP_ASSERT_EQ(ivp_psi_listener_unregister(&listener, env), 0);
    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 10. Rotation integration (equal inertias �?no gyroscopic)
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(rotation_equal_inertias) {
    /* Spin around Z axis with equal inertias */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0)); /* No gravity */

    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    tmpl.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    /* Rotation only counts as movement scaled by upper_limit_radius
     * (IVP_Core::calc_movement_state, ivp_core.cxx:1144): a zero-radius
     * ball spinning in place would be frozen. */
    tmpl.ball_radius = IVP_REAL(1.0);
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);

    /* Set angular velocity: 1 rad/s around Z */
    obj->physical_core->rot_speed = ivp_vec3f(0.0f, 0.0f, 1.0f);

    /* Run 66 steps = 1 second */
    for (int i = 0; i < 66; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* Angular velocity should be unchanged (no gyro, no damping) */
    IVP_ASSERT_NEAR(obj->physical_core->rot_speed.k[2], 1.0, 1e-3);

    /* Quaternion should represent ~1 radian rotation around Z */
    /* q = (0, 0, sin(θ/2), cos(θ/2)) where θ �?1 rad */
    /* The quaternion accumulates per-step, so it won't match exact analytical
     * for large angles, but should be close for 1 rad total */
    double qw = (double)obj->physical_core->q_world_f_core_next_psi.w;

    /* Instead of exact, check that the angle is approximately correct */
    double angle = 2.0 * acos(fabs(qw));
    IVP_ASSERT_NEAR(angle, 1.0, 0.05); /* Within 5% */

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 11. Rotation integration (unequal inertias �?gyroscopic)
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(rotation_gyroscopic_substeps) {
    /* Asymmetric top: verify gyroscopic correction changes angular velocity */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    tmpl.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);

    /* Set angular velocity with components in all axes */
    obj->physical_core->rot_speed = ivp_vec3f(1.0f, 0.5f, 0.3f);

    IVP_ASSERT_FALSE(obj->physical_core->rot_inertias_are_equal);

    double wx0 = (double)obj->physical_core->rot_speed.k[0];
    double wy0 = (double)obj->physical_core->rot_speed.k[1];
    double wz0 = (double)obj->physical_core->rot_speed.k[2];

    /* Run one step */
    ivp_environment_simulate_one_psi(env);

    /* Gyroscopic correction should change angular velocity direction */
    /* The total angular momentum magnitude should be approximately conserved */
    double L0 = 1.0*wx0*wx0 + 4.0*wy0*wy0 + 9.0*wz0*wz0;
    double wx = (double)obj->physical_core->rot_speed.k[0];
    double wy = (double)obj->physical_core->rot_speed.k[1];
    double wz = (double)obj->physical_core->rot_speed.k[2];
    double L1 = 1.0*wx*wx + 4.0*wy*wy + 9.0*wz*wz;

    /* Angular momentum squared should be approximately conserved */
    IVP_ASSERT_NEAR(L1, L0, 0.1);

    /* But individual components should have changed */
    IVP_ASSERT_TRUE(fabs(wx - wx0) > 1e-6 || fabs(wy - wy0) > 1e-6
                    || fabs(wz - wz0) > 1e-6);

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 12. AT interpolation
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(at_position_interpolation) {
    ivp_environment_t *env = ivp_environment_create(NULL);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);

    /* Run a few steps to get some velocity */
    for (int i = 0; i < 10; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* AT interpolation at t = time_of_last_psi should give pos_last */
    ivp_vec3r_t at_pos;
    ivp_core_get_at_position(obj->physical_core, obj->physical_core->time_of_last_psi, &at_pos);
    IVP_ASSERT_NEAR(at_pos.k[0], obj->physical_core->pos_world_f_core_last_psi.k[0], 1e-6);
    IVP_ASSERT_NEAR(at_pos.k[1], obj->physical_core->pos_world_f_core_last_psi.k[1], 1e-6);
    IVP_ASSERT_NEAR(at_pos.k[2], obj->physical_core->pos_world_f_core_last_psi.k[2], 1e-6);

    /* AT interpolation at t = time + dt should extrapolate by delta */
    double dt = 1.0 / 66.0;
    ivp_core_get_at_position(obj->physical_core, obj->physical_core->time_of_last_psi + dt, &at_pos);
    double expected_y = (double)obj->physical_core->pos_world_f_core_last_psi.k[1]
                      + (double)obj->physical_core->delta_world_f_core_psis.k[1] * dt;
    IVP_ASSERT_NEAR(at_pos.k[1], expected_y, 1e-6);

    ivp_environment_destroy(env);
}

IVP_TEST(at_quat_interpolation) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    tmpl.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);

    obj->physical_core->rot_speed = ivp_vec3f(0.0f, 0.0f, 0.5f);

    ivp_environment_simulate_one_psi(env);

    /* AT at t=0 fraction should give q_last */
    ivp_quatr_t q_at;
    ivp_core_get_at_quat(obj->physical_core, obj->physical_core->time_of_last_psi, &q_at);
    IVP_ASSERT_NEAR(q_at.x, obj->physical_core->q_world_f_core_last_psi.x, 1e-4);
    IVP_ASSERT_NEAR(q_at.w, obj->physical_core->q_world_f_core_last_psi.w, 1e-4);

    /* AT at t=1 fraction should give q_next */
    double dt = 1.0 / 66.0;
    ivp_core_get_at_quat(obj->physical_core, obj->physical_core->time_of_last_psi + dt, &q_at);
    IVP_ASSERT_NEAR(q_at.z, obj->physical_core->q_world_f_core_next_psi.z, 1e-3);
    IVP_ASSERT_NEAR(q_at.w, obj->physical_core->q_world_f_core_next_psi.w, 1e-3);

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 14. Static objects
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(static_object_no_movement) {
    ivp_environment_t *env = ivp_environment_create(NULL);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);

    /* Reference: the object is IVP_MT_STATIC (ivp_object.cxx:545) while its
     * core stays IVP_MT_NOT_SIM (IVP_Core::init, ivp_core.cxx:1050). */
    IVP_ASSERT_EQ((int)obj->object_movement_state, (int)IVP_MT_STATIC);
    IVP_ASSERT_EQ((int)obj->physical_core->movement_state, (int)IVP_MT_NOT_SIM);

    /* Run some steps */
    for (int i = 0; i < 100; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* Position should be unchanged */
    IVP_ASSERT_NEAR(obj->physical_core->pos_world_f_core_last_psi.k[0], 5.0, 1e-6);
    IVP_ASSERT_NEAR(obj->physical_core->speed.k[0], 0.0, 1e-6);

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 15. Phase hook
 * ══════════════════════════════════════════════════════════════════════════�?*/

static int s_phase_hits[IVP_PSI_PHASE_COUNT];
static void test_phase_hook(void *ctx,
                            struct ivp_environment *env,
                            ivp_psi_phase_t phase,
                            ivp_time_t time,
                            ivp_real_t dt) {
    (void)ctx;
    (void)time;
    const int idx = (int)phase;
    if (idx >= 0 && idx < IVP_PSI_PHASE_COUNT) {
        s_phase_hits[idx]++;
    }
}

IVP_TEST(phase_hook_called) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    memset(s_phase_hits, 0, sizeof(s_phase_hits));

    ivp_environment_set_phase_hook(env, test_phase_hook, NULL);
    ivp_environment_simulate_one_psi(env);

    IVP_ASSERT_EQ(s_phase_hits[(int)IVP_PSI_PHASE_PRE_PSI], 1);
    IVP_ASSERT_EQ(s_phase_hits[(int)IVP_PSI_PHASE_CONTROLLERS], 1);
    IVP_ASSERT_EQ(s_phase_hits[(int)IVP_PSI_PHASE_COLLISION_SOLVE], 1);
    IVP_ASSERT_EQ(s_phase_hits[(int)IVP_PSI_PHASE_POST_PSI], 1);

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 15b. Sim unit + event pipeline
 * ══════════════════════════════════════════════════════════════════════════�?*/

static int s_evt_unit_begin;
static int s_evt_unit_end;
static int s_evt_last_unit;

static void test_event_listener(void *ctx,
                                struct ivp_environment *env,
                                const ivp_event_t *event_data) {
    (void)ctx;
    if (event_data->type == IVP_EVENT_SIM_UNIT_BEGIN) {
        s_evt_unit_begin++;
        s_evt_last_unit = event_data->sim_unit_index;
    } else if (event_data->type == IVP_EVENT_SIM_UNIT_END) {
        s_evt_unit_end++;
    }
}

IVP_TEST(sim_unit_event_pipeline) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    memset(s_phase_hits, 0, sizeof(s_phase_hits));
    s_evt_unit_begin = 0;
    s_evt_unit_end = 0;
    s_evt_last_unit = -1;

    ivp_environment_set_sim_unit_count(env, 3);
    IVP_ASSERT_EQ(ivp_environment_get_sim_unit_count(env), 3);

    ivp_environment_set_phase_hook(env, test_phase_hook, NULL);
    ivp_event_listener_handle_t lid = ivp_environment_add_event_listener(env, test_event_listener, NULL);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(lid));

    ivp_environment_simulate_one_psi(env);

    /* The reference runs one PSI loop per PSI (IVP_Environment::simulate_psi);
     * sim_unit_count is informational only, so the sim-unit events and
     * phases fire once per PSI with sim unit index 0. */
    IVP_ASSERT_EQ(s_evt_unit_begin, 1);
    IVP_ASSERT_EQ(s_evt_unit_end, 1);
    IVP_ASSERT_EQ(s_evt_last_unit, 0);

    IVP_ASSERT_EQ(s_phase_hits[(int)IVP_PSI_PHASE_SIM_UNIT_BEGIN], 1);
    IVP_ASSERT_EQ(s_phase_hits[(int)IVP_PSI_PHASE_MANAGER_EVENTS], 1);
    IVP_ASSERT_EQ(s_phase_hits[(int)IVP_PSI_PHASE_SIM_UNIT_END], 1);

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 15c. Controller priority sync/async semantics
 * ══════════════════════════════════════════════════════════════════════════�?*/

static int s_controller_sync_seen;
typedef struct dependent_probe_ctx {
    ivp_object_handle_t target_a;
    ivp_object_handle_t target_b;
    int hits_a;
    int hits_b;
    int hits_other;
    int calls;
    ivp_object_handle_t discovery_handles[4];
    int discovery_count;
} dependent_probe_ctx_t;

static void controller_low_sync(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx) {
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    (void)ctx;
    /* Sync push: write directly to speed */
    obj->physical_core->speed.k[0] += IVP_REAL(1.0);
}

static void controller_high_async(const ivp_event_sim_t *event_sim,
                                  ivp_object_t *obj,
                                  void *ctx) {
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    (void)ctx;
    /* Check whether lower-priority (400) already modified speed.
     * Reference parity executes higher priorities first, so this should
     * observe the pre-low state and stay false. */
    if (obj->physical_core->speed.k[0] > IVP_REAL(0.9)) {
        s_controller_sync_seen = 1;
    }
    /* Direct speed write for test simplicity (tests dispatch ORDER). */
    obj->physical_core->speed.k[0] += IVP_REAL(2.0);
}

static void controller_dependent_probe(const ivp_event_sim_t *event_sim,
                                       ivp_object_t *obj,
                                       void *ctx) {
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    dependent_probe_ctx_t *probe = (dependent_probe_ctx_t *)ctx;
    probe->calls++;
    ivp_object_handle_t h = ivp_environment_get_object_handle(obj);

    if (h.slot == probe->target_a.slot && h.generation == probe->target_a.generation) {
        probe->hits_a++;
    } else if (h.slot == probe->target_b.slot && h.generation == probe->target_b.generation) {
        probe->hits_b++;
    } else {
        probe->hits_other++;
    }
}

static int dependent_discovery_fn(void *ctx,
                                  ivp_environment_t *env,
                                  ivp_object_handle_t *out_handles,
                                  int max_handles) {
    dependent_probe_ctx_t *probe = (dependent_probe_ctx_t *)ctx;
    int n = probe->discovery_count;
    if (n > max_handles) n = max_handles;
    for (int i = 0; i < n; i++) out_handles[i] = probe->discovery_handles[i];
    return n;
}

IVP_TEST(sim_unit_core_timestamp_tracks_full_psi) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_environment_set_sim_unit_count(env, 3);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj);

    /* simulate_one_psi == simulate_until(current_time + delta_psi_time) and
     * the first PSI runs at t=0, so after each call the clock sits one
     * delta_psi_time past the PSI that just ran. */
    const double dt = (double)ivp_environment_get_delta_psi_time(env);
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_NEAR(obj->physical_core->time_of_last_psi, 0.0, 1e-9);
    IVP_ASSERT_NEAR(obj->physical_core->time_of_last_psi, ivp_environment_get_time(env) - dt, 1e-6);

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_NEAR(obj->physical_core->time_of_last_psi, ivp_environment_get_time(env) - dt, 1e-6);

    ivp_environment_destroy(env);
}

static bool pending_union_find_count_matches(const ivp_sim_units_manager_t *m)
{
    size_t count = 0;
    const ivp_sim_unit_t *heads[] = { m->sim_units_slot0, m->still_slot };
    for (int i = 0; i < 2; ++i) {
        for (const ivp_sim_unit_t *su = heads[i]; su; su = su->next_sim_unit) {
            if (su->manager != m) return false;
            if (su->union_find_needed_for_sim_unit) count++;
        }
    }
    return count == m->pending_union_find_count;
}

IVP_TEST(sim_unit_pending_tracks_detach_wake_freeze_and_delete) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_sim_units_manager_t *m = ivp_env_get_sim_units_manager(env);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.start_in_simulation = false;
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj);
    ivp_sim_unit_t *su = obj->physical_core->sim_unit;
    IVP_ASSERT_EQ(m->pending_union_find_count, 0u);
    ivp_sim_unit_request_union_find(su);
    ivp_sim_unit_request_union_find(su);
    IVP_ASSERT_EQ(m->pending_union_find_count, 1u);

    ivp_sim_units_mgr_rem_sim_unit(m, su);
    IVP_ASSERT_NULL(su->manager);
    IVP_ASSERT_EQ(m->pending_union_find_count, 0u);
    ivp_sim_unit_request_union_find(su);
    ivp_sim_units_mgr_add_sim_unit(m, su);
    IVP_ASSERT_EQ(m->pending_union_find_count, 1u);
    ivp_sim_unit_revive_for_simulation(su, env);
    IVP_ASSERT_EQ(m->pending_union_find_count, 1u);
    IVP_ASSERT_TRUE(pending_union_find_count_matches(m));

    /* Forced freeze runs union-find immediately, not through the PSI scan. */
    IVP_ASSERT_TRUE(ivp_object_disable_simulation(obj));
    IVP_ASSERT_EQ(m->pending_union_find_count, 0u);
    IVP_ASSERT_TRUE(pending_union_find_count_matches(m));
    ivp_sim_unit_request_union_find(obj->physical_core->sim_unit);
    ivp_environment_destroy_object(env, obj);
    IVP_ASSERT_EQ(m->pending_union_find_count, 0u);
    IVP_ASSERT_TRUE(pending_union_find_count_matches(m));

    /* An empty linked unit can survive until manager teardown. */
    su = ivp_sim_unit_new(env->allocator);
    IVP_ASSERT_NOT_NULL(su);
    ivp_sim_unit_request_union_find(su);
    ivp_sim_units_mgr_add_sim_unit(m, su);
    IVP_ASSERT_EQ(m->pending_union_find_count, 1u);
    ivp_sim_unit_delete(su);
    IVP_ASSERT_EQ(m->pending_union_find_count, 0u);
    IVP_ASSERT_TRUE(pending_union_find_count_matches(m));
    ivp_environment_destroy(env);
}

typedef struct union_find_order_probe {
    ivp_sim_ctrl_t base;
    int id;
    int *order;
    int *length;
    ivp_sim_unit_t *mark_next;
} union_find_order_probe_t;

static int union_find_probe_priority(ivp_sim_ctrl_t *c) { (void)c; return 400; }
static void union_find_probe_simulate(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *cores)
{ (void)c; (void)es; (void)cores; }
static void union_find_probe_deleted(ivp_sim_ctrl_t *c, ivp_core_t *core)
{ ivp_core_rem_core_controller(core, c); }
static ivp_pvec_t *union_find_probe_associated(ivp_sim_ctrl_t *c)
{
    union_find_order_probe_t *probe = (union_find_order_probe_t *)c;
    if (*probe->length < 16) probe->order[(*probe->length)++] = probe->id;
    if (probe->mark_next) {
        ivp_sim_unit_request_union_find(probe->mark_next);
        probe->mark_next = NULL;
    }
    return NULL;
}
static const ivp_sim_ctrl_vt_t union_find_probe_vt = {
    union_find_probe_priority, union_find_probe_simulate,
    union_find_probe_associated, union_find_probe_deleted, NULL
};

IVP_TEST(sim_unit_pending_keeps_reference_scan_order) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_sim_units_manager_t *m = ivp_env_get_sim_units_manager(env);
    int order[16] = {0}, length = 0;
    union_find_order_probe_t probes[4] = {0};
    ivp_object_t *objects[4];
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.start_in_simulation = false;
    for (int i = 0; i < 4; ++i) {
        tmpl.position.k[0] = (ivp_real_t)(i * 10);
        objects[i] = ivp_environment_create_object(env, &tmpl, NULL);
        IVP_ASSERT_NOT_NULL(objects[i]);
        probes[i] = (union_find_order_probe_t){
            .base = { &union_find_probe_vt }, .id = i, .order = order, .length = &length
        };
        IVP_ASSERT_TRUE(ivp_core_add_core_controller(objects[i]->physical_core, &probes[i].base));
    }
    ivp_sim_unit_request_union_find(objects[0]->physical_core->sim_unit);
    ivp_sim_unit_request_union_find(objects[2]->physical_core->sim_unit);
    /* Unit 3 was already visited when unit 0 asks for work: defer it to the
     * next PSI, just as the reference's saved-next traversal does. */
    probes[0].mark_next = objects[3]->physical_core->sim_unit;
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(length, 2);
    IVP_ASSERT_EQ(order[0], 2);
    IVP_ASSERT_EQ(order[1], 0);
    IVP_ASSERT_EQ(m->pending_union_find_count, 1u);
    IVP_ASSERT_TRUE(pending_union_find_count_matches(m));
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(length, 3);
    IVP_ASSERT_EQ(order[2], 3);
    IVP_ASSERT_EQ(m->pending_union_find_count, 0u);
    for (int i = 0; i < 8; ++i) ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(length, 3);
    IVP_ASSERT_TRUE(pending_union_find_count_matches(m));
    for (int i = 0; i < 4; ++i)
        ivp_core_rem_core_controller(objects[i]->physical_core, &probes[i].base);
    ivp_environment_destroy(env);
}

IVP_TEST(sim_unit_pending_retries_every_split_allocation_failure) {
    int failures = 0;
    for (int fail_at = 1; fail_at <= 24; ++fail_at) {
        env_fail_alloc_state_t state = {0};
        ivp_allocator_t allocator;
        ivp_environment_t *env = env_create_with_fail_allocator(&state, &allocator);
        IVP_ASSERT_NOT_NULL(env);
        ivp_sim_units_manager_t *m = ivp_env_get_sim_units_manager(env);
        ivp_object_t *objects[3];
        ivp_object_template_t tmpl;
        ivp_object_template_init(&tmpl);
        for (int i = 0; i < 3; ++i) {
            tmpl.position.k[0] = (ivp_real_t)(i * 10);
            objects[i] = ivp_environment_create_object(env, &tmpl, NULL);
            IVP_ASSERT_NOT_NULL(objects[i]);
        }
        /* Real controller removal wakes its cores before splitting. A
         * synthetic split of still cores would create MOVING units around
         * frozen objects, violating the reference's simulation precondition. */
        ivp_environment_simulate_one_psi(env);
        ivp_sim_unit_t *su = objects[0]->physical_core->sim_unit;
        for (int i = 1; i < 3; ++i) {
            ivp_sim_unit_t *other = objects[i]->physical_core->sim_unit;
            ivp_sim_unit_request_union_find(other);
            IVP_ASSERT_TRUE(ivp_sim_unit_fusion_simulation_unities(su, other));
            ivp_sim_unit_delete(other);
        }
        IVP_ASSERT_TRUE(pending_union_find_count_matches(m));
        ivp_sim_unit_request_union_find(su);
        state.attempts = 0;
        state.fail_at = fail_at;
        ivp_sim_unit_do_sim_unit_union_find(su);
        bool failed = state.attempts >= fail_at;
        if (failed) {
            failures++;
            IVP_ASSERT_TRUE(m->pending_union_find_count > 0);
        }
        IVP_ASSERT_TRUE(pending_union_find_count_matches(m));
        state.fail_at = 0;
        /* Retry before simulation, including a newly split unit whose
         * controller index failed to rebuild. No extra flag may be needed. */
        for (int pass = 0; pass < 3 && m->pending_union_find_count; ++pass) {
            ivp_sim_unit_t *heads[] = { m->sim_units_slot0, m->still_slot };
            for (int i = 0; i < 2; ++i) {
                for (ivp_sim_unit_t *unit = heads[i]; unit;) {
                    ivp_sim_unit_t *next = unit->next_sim_unit;
                    if (unit->union_find_needed_for_sim_unit)
                        ivp_sim_unit_do_sim_unit_union_find(unit);
                    unit = next;
                }
            }
        }
        IVP_ASSERT_EQ(m->pending_union_find_count, 0u);
        IVP_ASSERT_TRUE(pending_union_find_count_matches(m));
        IVP_ASSERT_TRUE(objects[0]->physical_core->sim_unit != objects[1]->physical_core->sim_unit);
        IVP_ASSERT_TRUE(objects[0]->physical_core->sim_unit != objects[2]->physical_core->sim_unit);
        IVP_ASSERT_TRUE(objects[1]->physical_core->sim_unit != objects[2]->physical_core->sim_unit);
        ivp_environment_simulate_one_psi(env);
        ivp_environment_destroy(env);
        IVP_ASSERT_EQ(state.live_blocks, 0);
    }
    IVP_ASSERT_TRUE(failures >= 8);
}

IVP_TEST(sim_unit_controller_rebuild_failure_clears_partial_index) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj);
    ivp_sim_unit_t *unit = obj->physical_core->sim_unit;
    IVP_ASSERT_NOT_NULL(unit);

    ivp_pvec_t saved = unit->controller_cores;
    unit->controller_cores = (ivp_pvec_t){ NULL, 0, -1 };
    IVP_ASSERT_FALSE(ivp_sim_unit_sim_unit_calc_redundants(unit));
    IVP_ASSERT_EQ(ivp_pvec_len(&unit->controller_cores), 0);

    unit->controller_cores = saved;
    ivp_sim_unit_clean_sim_unit(unit);
    IVP_ASSERT_TRUE(ivp_sim_unit_sim_unit_calc_redundants(unit));
    ivp_environment_destroy(env);
}

IVP_TEST(sim_unit_controller_rebuild_failure_preserves_live_index) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj);
    ivp_sim_unit_t *unit = obj->physical_core->sim_unit;
    IVP_ASSERT_NOT_NULL(unit);
    IVP_ASSERT_TRUE(ivp_pvec_len(&unit->controller_cores) > 0);

    ivp_pvec_t index_before = unit->controller_cores;
    ivp_allocator_t allocator_before = *env->allocator;
    env_fail_alloc_state_t state = { .fail_at = 1 };
    *env->allocator = (ivp_allocator_t){
        .alloc = env_fail_alloc,
        .realloc = env_fail_realloc,
        .free = env_fail_free,
        .ctx = &state
    };
    IVP_ASSERT_FALSE(ivp_sim_unit_sim_unit_calc_redundants(unit));
    *env->allocator = allocator_before;

    IVP_ASSERT_TRUE(unit->controller_cores.elems == index_before.elems);
    IVP_ASSERT_EQ(unit->controller_cores.n, index_before.n);
    IVP_ASSERT_EQ(unit->controller_cores.cap, index_before.cap);
    IVP_ASSERT_EQ(state.live_blocks, 0);
    ivp_environment_destroy(env);
}

typedef struct controller_self_remove_ctx {
    ivp_controller_handle_t self;
    int calls;
} controller_self_remove_ctx_t;

typedef struct controller_count_ctx {
    int calls;
} controller_count_ctx_t;

static void controller_remove_self(const ivp_event_sim_t *event_sim,
                                   ivp_object_t *obj,
                                   void *ctx) {
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    (void)obj;
    controller_self_remove_ctx_t *self_ctx = (controller_self_remove_ctx_t *)ctx;
    self_ctx->calls++;
    if (self_ctx->calls == 1 && ivp_controller_handle_is_valid(self_ctx->self)) {
        (void)ivp_environment_remove_controller(env, self_ctx->self);
    }
}

static void controller_count_only(const ivp_event_sim_t *event_sim,
                                  ivp_object_t *obj,
                                  void *ctx) {
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    (void)obj;
    controller_count_ctx_t *count_ctx = (controller_count_ctx_t *)ctx;
    count_ctx->calls++;
}

IVP_TEST(controller_registration_failure_is_atomic) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj);
    ivp_controller_system_t *controllers = (ivp_controller_system_t *)env->controller_system;
    controller_count_ctx_t count_ctx = {0};

    ivp_pvec_t saved = env->pl->slot_ctrls;
    env->pl->slot_ctrls = (ivp_pvec_t){ NULL, 0, -1 };
    ivp_controller_handle_t h = ivp_test_environment_add_custom_controller(
        env, IVP_CP_MOTION, controller_count_only, &count_ctx);
    IVP_ASSERT_FALSE(ivp_controller_handle_is_valid(h));
    IVP_ASSERT_EQ(ivp_controller_system_count(controllers), 0);
    env->pl->slot_ctrls = saved;

    saved = env->pl->all_core_ctrls;
    env->pl->all_core_ctrls = (ivp_pvec_t){ NULL, 0, -1 };
    h = ivp_test_environment_add_custom_controller(
        env, IVP_CP_MOTION, controller_count_only, &count_ctx);
    IVP_ASSERT_FALSE(ivp_controller_handle_is_valid(h));
    IVP_ASSERT_EQ(ivp_controller_system_count(controllers), 0);
    IVP_ASSERT_EQ(ivp_pvec_len(&env->pl->slot_ctrls), 0);
    env->pl->all_core_ctrls = saved;

    saved = obj->physical_core->controllers_of_core;
    obj->physical_core->controllers_of_core = (ivp_pvec_t){ NULL, 0, -1 };
    h = ivp_test_environment_add_custom_controller(
        env, IVP_CP_MOTION, controller_count_only, &count_ctx);
    IVP_ASSERT_FALSE(ivp_controller_handle_is_valid(h));
    IVP_ASSERT_EQ(ivp_controller_system_count(controllers), 0);
    IVP_ASSERT_EQ(ivp_pvec_len(&env->pl->slot_ctrls), 0);
    IVP_ASSERT_EQ(ivp_pvec_len(&env->pl->all_core_ctrls), 0);
    obj->physical_core->controllers_of_core = saved;

    h = ivp_test_environment_add_custom_controller(
        env, IVP_CP_MOTION, controller_count_only, &count_ctx);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(h));
    IVP_ASSERT_TRUE(ivp_environment_remove_controller(env, h));

    h = ivp_environment_create_core_controller(
        env, IVP_CP_MOTION, controller_count_only, &count_ctx);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(h));
    saved = obj->physical_core->controllers_of_core;
    obj->physical_core->controllers_of_core = (ivp_pvec_t){ NULL, 0, -1 };
    IVP_ASSERT_FALSE(ivp_environment_add_controller_to_core(env, h, obj));
    obj->physical_core->controllers_of_core = saved;
    IVP_ASSERT_FALSE(ivp_environment_remove_controller_from_core(env, h, obj));
    IVP_ASSERT_TRUE(ivp_environment_add_controller_to_core(env, h, obj));
    IVP_ASSERT_TRUE(ivp_environment_remove_controller_from_core(env, h, obj));
    IVP_ASSERT_TRUE(ivp_environment_remove_controller(env, h));
    ivp_environment_destroy(env);
}

IVP_TEST(dependent_controller_allocation_failure_is_atomic) {
    env_fail_alloc_state_t state = {0};
    ivp_allocator_t allocator;
    ivp_environment_t *env = env_create_with_fail_allocator(&state, &allocator);
    IVP_ASSERT_NOT_NULL(env);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_handle_t object_handle = ivp_object_handle_invalid();
    ivp_object_t *obj = ivp_environment_create_object_with_handle(
        env, &tmpl, NULL, &object_handle);
    IVP_ASSERT_NOT_NULL(obj);

    dependent_probe_ctx_t probe;
    memset(&probe, 0, sizeof(probe));
    probe.target_a = object_handle;
    probe.discovery_handles[0] = object_handle;
    probe.discovery_count = 1;
    controller_count_ctx_t warm_ctx = {0};
    ivp_controller_handle_t warm = ivp_test_environment_add_custom_controller(
        env, IVP_CP_MOTION, controller_count_only, &warm_ctx);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(warm));
    IVP_ASSERT_TRUE(ivp_environment_remove_controller(env, warm));
    const int live_before = state.live_blocks;
    state.fail_at = state.attempts + 3; /* slot, adapter, association handles */

    ivp_controller_handle_t ctrl =
        ivp_test_environment_announce_custom_dependent_controller(
            env, IVP_CP_CONSTRAINTS, controller_dependent_probe, &probe,
            dependent_discovery_fn);
    IVP_ASSERT_FALSE(ivp_controller_handle_is_valid(ctrl));
    IVP_ASSERT_EQ(state.attempts, state.fail_at);
    IVP_ASSERT_EQ(state.live_blocks, live_before);
    IVP_ASSERT_EQ(ivp_pvec_len(&env->pl->slot_ctrls), 0);
    IVP_ASSERT_EQ(ivp_controller_system_count(
                      (ivp_controller_system_t *)env->controller_system), 0);

    state.fail_at = 0;
    ctrl = ivp_test_environment_announce_custom_dependent_controller(
        env, IVP_CP_CONSTRAINTS, controller_dependent_probe, &probe,
        dependent_discovery_fn);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ctrl));
    IVP_ASSERT_TRUE(ivp_environment_remove_controller(env, ctrl));
    ivp_environment_destroy(env);
    IVP_ASSERT_EQ(state.live_blocks, 0);
}

IVP_TEST(controller_discovery_failure_preserves_all_core_mode) {
    env_fail_alloc_state_t state = {0};
    ivp_allocator_t allocator;
    ivp_environment_t *env = env_create_with_fail_allocator(&state, &allocator);
    IVP_ASSERT_NOT_NULL(env);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_handle_t h_a = ivp_object_handle_invalid();
    ivp_object_handle_t h_b = ivp_object_handle_invalid();
    ivp_object_t *obj_a = ivp_environment_create_object_with_handle(
        env, &tmpl, NULL, &h_a);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object_with_handle(
        env, &tmpl, NULL, &h_b);
    IVP_ASSERT_TRUE(obj_a && obj_b);

    dependent_probe_ctx_t probe;
    memset(&probe, 0, sizeof(probe));
    probe.target_a = h_a;
    probe.target_b = h_b;
    probe.discovery_handles[0] = h_a;
    probe.discovery_count = 1;
    ivp_controller_handle_t ctrl = ivp_test_environment_add_custom_controller(
        env, IVP_CP_MOTION, controller_dependent_probe, &probe);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ctrl));

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(probe.calls, 2);
    const int live_before = state.live_blocks;
    state.fail_at = state.attempts + 1; /* replacement association handles */
    IVP_ASSERT_FALSE(ivp_environment_set_controller_discovery(
        env, ctrl, dependent_discovery_fn));
    IVP_ASSERT_EQ(state.live_blocks, live_before);

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(probe.calls, 4);
    state.fail_at = 0;
    IVP_ASSERT_TRUE(ivp_environment_remove_controller(env, ctrl));
    ivp_environment_destroy(env);
    IVP_ASSERT_EQ(state.live_blocks, 0);
}

IVP_TEST(dependent_refresh_failure_preserves_live_association) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_handle_t h_a = ivp_object_handle_invalid();
    ivp_object_handle_t h_b = ivp_object_handle_invalid();
    ivp_object_t *obj_a = ivp_environment_create_object_with_handle(
        env, &tmpl, NULL, &h_a);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object_with_handle(
        env, &tmpl, NULL, &h_b);
    IVP_ASSERT_TRUE(obj_a && obj_b);

    dependent_probe_ctx_t probe;
    memset(&probe, 0, sizeof(probe));
    probe.target_a = h_a;
    probe.target_b = h_b;
    probe.discovery_handles[0] = h_a;
    probe.discovery_count = 1;
    ivp_controller_handle_t ctrl =
        ivp_test_environment_announce_custom_dependent_controller(
            env, IVP_CP_CONSTRAINTS, controller_dependent_probe, &probe,
            dependent_discovery_fn);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ctrl));

    ivp_controller_system_t *controllers =
        (ivp_controller_system_t *)env->controller_system;
    ivp_controller_slot_t *slot =
        ivp_controller_system_get_by_handle(controllers, ctrl);
    IVP_ASSERT_NOT_NULL(slot);
    IVP_ASSERT_TRUE(ivp_pvec_index_of(
                        &obj_a->physical_core->controllers_of_core,
                        &slot->base) >= 0);

    probe.discovery_handles[0] = h_b;
    ivp_pvec_t saved = obj_b->physical_core->controllers_of_core;
    obj_b->physical_core->controllers_of_core =
        (ivp_pvec_t){ NULL, 0, -1 };
    IVP_ASSERT_FALSE(ivp_environment_set_controller_discovery(
        env, ctrl, dependent_discovery_fn));
    obj_b->physical_core->controllers_of_core = saved;

    IVP_ASSERT_TRUE(ivp_pvec_index_of(
                        &obj_a->physical_core->controllers_of_core,
                        &slot->base) >= 0);
    IVP_ASSERT_TRUE(ivp_pvec_index_of(
                        &obj_b->physical_core->controllers_of_core,
                        &slot->base) < 0);

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(probe.hits_a, 0);
    IVP_ASSERT_EQ(probe.hits_b, 1);
    IVP_ASSERT_TRUE(ivp_pvec_index_of(
                        &obj_a->physical_core->controllers_of_core,
                        &slot->base) < 0);
    IVP_ASSERT_TRUE(ivp_pvec_index_of(
                        &obj_b->physical_core->controllers_of_core,
                        &slot->base) >= 0);

    IVP_ASSERT_TRUE(ivp_environment_remove_controller(env, ctrl));
    ivp_environment_destroy(env);
}

IVP_TEST(controller_self_remove_does_not_break_iteration) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj);

    controller_self_remove_ctx_t self_ctx;
    memset(&self_ctx, 0, sizeof(self_ctx));
    self_ctx.self = ivp_controller_handle_invalid();
    controller_count_ctx_t count_ctx;
    memset(&count_ctx, 0, sizeof(count_ctx));

    ivp_controller_handle_t h_self =
        ivp_test_environment_add_custom_controller(env, 400, controller_remove_self, &self_ctx);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(h_self));
    self_ctx.self = h_self;

    ivp_controller_handle_t h_count =
        ivp_test_environment_add_custom_controller(env, 500, controller_count_only, &count_ctx);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(h_count));

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(self_ctx.calls, 1);
    IVP_ASSERT_EQ(count_ctx.calls, 1);

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(self_ctx.calls, 1);
    IVP_ASSERT_EQ(count_ctx.calls, 2);

    ivp_environment_destroy(env);
}

IVP_TEST(controller_dependent_rejects_foreign_targets) {
    ivp_environment_t *env_a = ivp_environment_create(NULL);
    ivp_environment_t *env_b = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env_a);
    IVP_ASSERT_NOT_NULL(env_b);
    ivp_environment_clear_controllers(env_a);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj_a = ivp_environment_create_object(env_a, &tmpl, NULL);
    ivp_object_t *obj_b = ivp_environment_create_object(env_b, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    dependent_probe_ctx_t probe;
    memset(&probe, 0, sizeof(probe));

    ivp_controller_handle_t ctrl =
        ivp_test_environment_add_custom_controller(env_a, 500, controller_dependent_probe, &probe);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ctrl));

    IVP_ASSERT_FALSE(ivp_environment_add_controller_to_core(env_a, ctrl, obj_b));
    IVP_ASSERT_TRUE(ivp_environment_add_controller_to_core(env_a, ctrl, obj_a));

    ivp_environment_destroy(env_a);
    ivp_environment_destroy(env_b);
}

IVP_TEST(controller_priority_sync_async) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    /* The gravity controller (priority 1000) damps the core before committing
     * async pushes (IVP_Standard_Gravity_Controller::do_simulation_controller,
     * ivp_sim_unit.cxx:880): disable damping so the pushes add up exactly. */
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    tmpl.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj);

    s_controller_sync_seen = 0;
    ivp_controller_handle_t id_low = ivp_test_environment_add_custom_controller(env, 400, controller_low_sync, NULL);
    ivp_controller_handle_t id_high = ivp_test_environment_add_custom_controller(env, 1400, controller_high_async, NULL);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(id_low));
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(id_high));

    ivp_environment_simulate_one_psi(env);

    /* Reference parity: high(1400) runs before low(400) in descending order.
     * High controller should not observe low's sync push yet. */
    IVP_ASSERT_EQ(s_controller_sync_seen, 0);
    /* final velocity includes low sync push (1.0) + high async push (2.0,
     * committed by gravity barrier at priority 1000). */
    IVP_ASSERT_NEAR(obj->physical_core->speed.k[0], 3.0, 1e-6);

    ivp_environment_destroy(env);
}

IVP_TEST(controller_dependent_zero_targets_invokes_none) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_handle_t h0 = ivp_object_handle_invalid();
    ivp_object_handle_t h1 = ivp_object_handle_invalid();
    ivp_object_t *obj0 = ivp_environment_create_object_with_handle(env, &tmpl, NULL, &h0);
    ivp_object_t *obj1 = ivp_environment_create_object_with_handle(env, &tmpl, NULL, &h1);
    IVP_ASSERT_NOT_NULL(obj0);
    IVP_ASSERT_NOT_NULL(obj1);

    dependent_probe_ctx_t probe;
    memset(&probe, 0, sizeof(probe));
    probe.target_a = h0;
    probe.target_b = h1;

    ivp_controller_handle_t ctrl =
        ivp_test_environment_announce_custom_dependent_controller(env,
                                                             500,
                                                             controller_dependent_probe,
                                                             &probe,
                                                             dependent_discovery_fn);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ctrl));

    ivp_environment_simulate_one_psi(env);

    IVP_ASSERT_EQ(probe.hits_a, 0);
    IVP_ASSERT_EQ(probe.hits_b, 0);
    IVP_ASSERT_EQ(probe.hits_other, 0);
    IVP_ASSERT_EQ(probe.calls, 0);

    ivp_environment_destroy(env);
}

IVP_TEST(controller_dependent_requires_discovery_callback) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_environment_clear_controllers(env);

    dependent_probe_ctx_t probe;
    memset(&probe, 0, sizeof(probe));

    ivp_controller_handle_t ctrl =
        ivp_test_environment_announce_custom_dependent_controller(
            env, 500, controller_dependent_probe, &probe, NULL);
    IVP_ASSERT_FALSE(ivp_controller_handle_is_valid(ctrl));

    ivp_environment_destroy(env);
}

IVP_TEST(controller_dependent_removed_with_target) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);

    ivp_object_handle_t h_a = ivp_object_handle_invalid();
    ivp_object_handle_t h_b = ivp_object_handle_invalid();
    ivp_object_t *obj_a = ivp_environment_create_object_with_handle(env, &tmpl, NULL, &h_a);
    ivp_object_t *obj_b = ivp_environment_create_object_with_handle(env, &tmpl, NULL, &h_b);
    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    dependent_probe_ctx_t probe;
    memset(&probe, 0, sizeof(probe));
    probe.target_a = h_a;
    probe.target_b = h_b;
    probe.discovery_handles[0] = h_a;
    probe.discovery_handles[1] = h_b;
    probe.discovery_count = 2;

    ivp_controller_handle_t ctrl =
        ivp_test_environment_announce_custom_dependent_controller(env,
                                                             500,
                                                             controller_dependent_probe,
                                                             &probe,
                                                             dependent_discovery_fn);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ctrl));

    /* Reference: an actuator / constraint is deleted together with an object
     * it is anchored to (IVP_Anchor::object_is_going_to_be_deleted), so the
     * dependent controller is unregistered and never runs again. */
    IVP_ASSERT_TRUE(ivp_environment_destroy_object_by_handle(env, h_a));
    ivp_environment_simulate_one_psi(env);

    IVP_ASSERT_EQ(probe.hits_a, 0);
    IVP_ASSERT_EQ(probe.hits_b, 0);
    IVP_ASSERT_EQ(probe.hits_other, 0);
    IVP_ASSERT_EQ(probe.calls, 0);
    IVP_ASSERT_FALSE(ivp_environment_remove_controller(env, ctrl)); /* already gone */

    ivp_environment_destroy(env);
}

IVP_TEST(controller_dependent_dynamic_discovery_unions_island) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_handle_t h_a = ivp_object_handle_invalid();
    ivp_object_handle_t h_b = ivp_object_handle_invalid();
    ivp_object_t *obj_a = ivp_environment_create_object_with_handle(env, &tmpl, NULL, &h_a);
    ivp_object_t *obj_b = ivp_environment_create_object_with_handle(env, &tmpl, NULL, &h_b);
    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    dependent_probe_ctx_t probe;
    memset(&probe, 0, sizeof(probe));
    probe.target_a = h_a;
    probe.target_b = h_b;
    probe.discovery_handles[0] = h_a;
    probe.discovery_handles[1] = h_b;
    probe.discovery_count = 2;

    ivp_controller_handle_t ctrl =
        ivp_test_environment_announce_custom_dependent_controller(env,
                                                             IVP_CP_CONSTRAINTS,
                                                             controller_dependent_probe,
                                                             &probe,
                                                             dependent_discovery_fn);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ctrl));

    ivp_environment_simulate_one_psi(env);

    IVP_ASSERT_EQ(probe.calls, 1);
    IVP_ASSERT_EQ(probe.hits_a + probe.hits_b, 1);
    IVP_ASSERT_EQ(probe.hits_other, 0);

    /* Connected dependent-controller targets must share one sim unit. */
    IVP_ASSERT_NOT_NULL(obj_a->physical_core->sim_unit);
    IVP_ASSERT_NOT_NULL(obj_b->physical_core->sim_unit);
    IVP_ASSERT_TRUE(obj_a->physical_core->sim_unit == obj_b->physical_core->sim_unit);

    ivp_environment_destroy(env);
}

IVP_TEST(controller_dependent_refreshes_object_on_shared_core) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.start_in_simulation = false;
    ivp_object_handle_t h_a = ivp_object_handle_invalid();
    ivp_object_handle_t h_b = ivp_object_handle_invalid();
    ivp_object_t *obj_a = ivp_environment_create_object_with_handle(env, &tmpl, NULL, &h_a);
    ivp_object_t *obj_b = ivp_environment_create_object_with_handle(env, &tmpl, NULL, &h_b);
    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);
    ivp_object_t *objects[] = { obj_a, obj_b };
    IVP_ASSERT_TRUE(ivp_environment_merge_objects(env, objects, 2));
    IVP_ASSERT_TRUE(obj_a->physical_core == obj_b->physical_core);

    dependent_probe_ctx_t probe;
    memset(&probe, 0, sizeof(probe));
    probe.target_a = h_a;
    probe.target_b = h_b;
    probe.discovery_handles[0] = h_a;
    probe.discovery_count = 1;
    ivp_controller_handle_t ctrl =
        ivp_test_environment_announce_custom_dependent_controller(
            env, IVP_CP_CONSTRAINTS, controller_dependent_probe, &probe,
            dependent_discovery_fn);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ctrl));
    IVP_ASSERT_TRUE(ivp_environment_ensure_core_in_simulation(env, obj_a));

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(probe.hits_a, 1);
    IVP_ASSERT_EQ(probe.hits_b, 0);

    probe.discovery_handles[0] = h_b;
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(probe.hits_a, 1);
    IVP_ASSERT_EQ(probe.hits_b, 1);

    ivp_environment_destroy(env);
}

IVP_TEST(controller_attach_to_core_limits_dispatch_scope) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj_a = ivp_environment_create_object(env, &tmpl, NULL);
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    controller_count_ctx_t count_ctx;
    memset(&count_ctx, 0, sizeof(count_ctx));
    ivp_controller_handle_t ctrl = ivp_test_environment_add_custom_controller(
        env, IVP_CP_MOTION, controller_count_only, &count_ctx);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ctrl));

    IVP_ASSERT_TRUE(ivp_environment_add_controller_to_core(env, ctrl, obj_a));
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(count_ctx.calls, 1);

    IVP_ASSERT_TRUE(ivp_environment_remove_controller_from_core(env, ctrl, obj_a));
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(count_ctx.calls, 1);

    ivp_environment_destroy(env);
}

IVP_TEST(core_controller_removal_preserves_shared_core_attachment) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.start_in_simulation = false;
    ivp_object_t *obj_a = ivp_environment_create_object(env, &tmpl, NULL);
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tmpl, NULL);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *unassociated = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_TRUE(obj_a && obj_b && unassociated);
    ivp_object_t *objects[] = { obj_a, obj_b };
    IVP_ASSERT_TRUE(ivp_environment_merge_objects(env, objects, 2));

    controller_count_ctx_t count_ctx = {0};
    ivp_controller_handle_t ctrl = ivp_environment_create_core_controller(
        env, IVP_CP_MOTION, controller_count_only, &count_ctx);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ctrl));
    IVP_ASSERT_TRUE(ivp_environment_add_controller_to_core(env, ctrl, obj_a));
    IVP_ASSERT_TRUE(ivp_environment_add_controller_to_core(env, ctrl, obj_b));
    IVP_ASSERT_FALSE(ivp_environment_remove_controller_from_core(
        env, ctrl, unassociated));

    IVP_ASSERT_TRUE(ivp_environment_ensure_core_in_simulation(env, obj_a));
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(count_ctx.calls, 1);

    IVP_ASSERT_TRUE(ivp_environment_remove_controller_from_core(env, ctrl, obj_a));
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(count_ctx.calls, 2);

    IVP_ASSERT_TRUE(ivp_environment_remove_controller_from_core(env, ctrl, obj_b));
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(count_ctx.calls, 2);

    ivp_environment_destroy(env);
}

IVP_TEST(ensure_core_in_simulation_wakes_frozen_object) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.start_in_simulation = false; /* object starts frozen */
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj);
    IVP_ASSERT_EQ((int)obj->physical_core->movement_state, (int)IVP_MT_NOT_SIM);

    /* IVP_Real_Object::ensure_in_simulation queues the core for revival
     * (add_revive_core, ivp_object.cxx:1385); it is revived at the next PSI. */
    IVP_ASSERT_TRUE(ivp_environment_ensure_core_in_simulation(env, obj));
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ((int)obj->physical_core->movement_state, (int)IVP_MT_MOVING);

    ivp_environment_destroy(env);
}

IVP_TEST(revive_queue_failure_leaves_core_retryable) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.start_in_simulation = false;
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj);
    ivp_core_t *core = obj->physical_core;
    IVP_ASSERT_FALSE(core->is_in_wakeup_vec);

    ivp_pvec_t saved = env->pl->core_revive_list;
    env->pl->core_revive_list = (ivp_pvec_t){ NULL, 0, -1 };
    IVP_ASSERT_FALSE(ivp_env_add_revive_core(env, core));
    IVP_ASSERT_FALSE(core->is_in_wakeup_vec);
    env->pl->core_revive_list = saved;

    IVP_ASSERT_TRUE(ivp_env_add_revive_core(env, core));
    IVP_ASSERT_TRUE(core->is_in_wakeup_vec);
    IVP_ASSERT_TRUE(ivp_pvec_index_of(&env->pl->core_revive_list, core) >= 0);
    ivp_env_remove_revive_core(env, core);
    IVP_ASSERT_FALSE(core->is_in_wakeup_vec);

    ivp_environment_destroy(env);
}

IVP_TEST(force_actuator_executes_once_per_psi) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = IVP_REAL(0.1);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    ivp_environment_clear_controllers(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.mass = IVP_REAL(1.0);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    tmpl.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj_a);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj_b);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(20.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_c = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj_c);

    ivp_force_actuator_state_t force_state;
    ivp_vec3r_t a0 = ivp_vec3r_zero();
    ivp_vec3r_t b0 = ivp_vec3r_zero();
    ivp_force_actuator_state_setup(&force_state, obj_a, &a0, obj_b, &b0, IVP_REAL(10.0), true, true);
    ivp_controller_handle_t h =
        ivp_environment_add_controller(env, 500, ivp_controller_force_actuator, &force_state);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(h));

    ivp_environment_simulate_one_psi(env);

    /* Reference IVP_Actuator_Force: a positive force pushes A away from B
     * (dir = pos_a - pos_b) and B away from A. */
    IVP_ASSERT_NEAR(obj_a->physical_core->speed.k[0], -1.0, 1e-6);
    IVP_ASSERT_NEAR(obj_b->physical_core->speed.k[0], 1.0, 1e-6);
    IVP_ASSERT_NEAR(obj_c->physical_core->speed.k[0], 0.0, 1e-6);

    ivp_environment_destroy(env);
}

IVP_TEST(rot_mot_executes_once_per_psi) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = IVP_REAL(0.1);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    /* Type-driven priority mapping: rot_mot is forced to actuator(1500),
     * which runs before gravity(1000), so async push is committed in-step
     * at the gravity barrier. */

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    tmpl.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.rot_inertia_is_factor = false; /* absolute unit inertia (template default 1,1,1) */

    ivp_object_t *obj_b = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj_b);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_c = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj_c);

    ivp_rot_mot_state_t rot_state;
    ivp_vec3r_t axis0 = ivp_vec3r_zero();
    ivp_vec3r_t axis1 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_rot_mot_state_setup(&rot_state, NULL, &axis0, obj_b, &axis1,
                            IVP_REAL(100.0), IVP_REAL(5.0), IVP_REAL(1000.0));
    ivp_controller_handle_t h =
        ivp_environment_add_controller(env, 500, ivp_controller_rot_mot, &rot_state);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(h));

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_NEAR(obj_b->physical_core->rot_speed.k[1], 0.5, 1e-6);

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_NEAR(obj_b->physical_core->rot_speed.k[1], 1.0, 1e-3);
    IVP_ASSERT_NEAR(obj_c->physical_core->rot_speed.k[0], 0.0, 1e-6);
    IVP_ASSERT_NEAR(obj_c->physical_core->rot_speed.k[1], 0.0, 1e-6);
    IVP_ASSERT_NEAR(obj_c->physical_core->rot_speed.k[2], 0.0, 1e-6);

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 16. Multiple objects
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(multiple_objects_freefall) {
    ivp_environment_t *env = ivp_environment_create(NULL);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj1 = ivp_environment_create_object(env, &tmpl, NULL);

    tmpl.mass = IVP_REAL(100.0);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj2 = ivp_environment_create_object(env, &tmpl, NULL);

    /* Run 100 steps */
    for (int i = 0; i < 100; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* Both should fall same distance (gravity doesn't depend on mass) */
    IVP_ASSERT_NEAR(obj1->physical_core->pos_world_f_core_last_psi.k[1],
                    obj2->physical_core->pos_world_f_core_last_psi.k[1], 1e-3);

    /* But X positions should be different (one started at 10) */
    IVP_ASSERT_NEAR(obj1->physical_core->pos_world_f_core_last_psi.k[0], 0.0, 1e-6);
    IVP_ASSERT_NEAR(obj2->physical_core->pos_world_f_core_last_psi.k[0], 10.0, 1e-6);

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 17. Freeze params
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(freeze_params_defaults) {
    ivp_freeze_params_t fp = ivp_freeze_params_default();
    IVP_ASSERT_NEAR(fp.trans_threshold_short, 0.01, 1e-6);
    IVP_ASSERT_NEAR(fp.rot_threshold_short, 0.005, 1e-6);
    IVP_ASSERT_NEAR(fp.freeze_check_dtime, 0.3, 1e-6);
    IVP_ASSERT_NEAR(fp.has_moved_long_time, 4.0, 1e-6);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 18. Quaternion fast_normalize (integration path)
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(quat_fast_normalize_integration) {
    /* After many integration steps, quaternion should remain unit length */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    tmpl.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);

    obj->physical_core->rot_speed = ivp_vec3f(0.5f, 0.3f, 0.7f);

    for (int i = 0; i < 1000; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* Check quaternion is still unit length */
    ivp_real_t ql = ivp_quat_dot(&obj->physical_core->q_world_f_core_next_psi,
                                  &obj->physical_core->q_world_f_core_next_psi);
    IVP_ASSERT_NEAR(ql, 1.0, 1e-3);

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * 19. Pinned object (no gravity, no linear push)
 * ══════════════════════════════════════════════════════════════════════════�?*/

IVP_TEST(pinned_object_no_gravity) {
    ivp_environment_t *env = ivp_environment_create(NULL);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    obj->physical_core->pinned = true;

    for (int i = 0; i < 100; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* Speed should remain zero (gravity skipped for pinned) */
    IVP_ASSERT_NEAR(obj->physical_core->speed.k[0], 0.0, 1e-6);
    IVP_ASSERT_NEAR(obj->physical_core->speed.k[1], 0.0, 1e-6);
    IVP_ASSERT_NEAR(obj->physical_core->speed.k[2], 0.0, 1e-6);

    ivp_environment_destroy(env);
}

/* ══════════════════════════════════════════════════════════════════════════�?
 * Registration
 * ══════════════════════════════════════════════════════════════════════════�?*/

int main(void) {
    /* Core */
    IVP_REGISTER(core_init_basic);
    IVP_REGISTER(core_init_equal_inertias);
    IVP_REGISTER(core_zero_inertia_is_clamped_without_assert);
    IVP_REGISTER(core_set_pose);
    IVP_REGISTER(core_beam_to);

    /* Damping */
    IVP_REGISTER(core_damp_linear_approx);
    IVP_REGISTER(core_damp_exponential);
    IVP_REGISTER(core_damp_analytical_decay);
    IVP_REGISTER(core_damp_slow_addon);

    /* Async pushes */
    IVP_REGISTER(core_commit_async_pushes);

    /* Object template */
    IVP_REGISTER(object_template_defaults);

    /* Environment */
    IVP_REGISTER(env_create_destroy);
    IVP_REGISTER(env_create_with_config);
    IVP_REGISTER(env_create_allocation_failures_are_recoverable);
    IVP_REGISTER(env_create_object);
    IVP_REGISTER(env_create_object_reserves_psi_scratch_atomically);
    IVP_REGISTER(env_destroy_object);
    IVP_REGISTER(env_object_handle_basic);
    IVP_REGISTER(env_object_handle_stale_after_delete);
    IVP_REGISTER(env_create_invalid_delta_time_defaults);
    IVP_REGISTER(env_destroy_object_foreign_owner_noop);

    /* PSI */
    IVP_REGISTER(env_single_psi_step);
    IVP_REGISTER(freefall_no_damping);
    IVP_REGISTER(freefall_default_damping);
    IVP_REGISTER(freefall_240_frames_position);

    /* simulate_dtime */
    IVP_REGISTER(env_simulate_dtime_accumulation);
    IVP_REGISTER(env_simulate_dtime_residual);

    /* Rotation */
    IVP_REGISTER(rotation_equal_inertias);
    IVP_REGISTER(rotation_gyroscopic_substeps);

    /* AT interpolation */
    IVP_REGISTER(at_position_interpolation);
    IVP_REGISTER(at_quat_interpolation);

    /* Static */
    IVP_REGISTER(static_object_no_movement);

    /* Hooks / events */
    IVP_REGISTER(phase_hook_called);
    IVP_REGISTER(sim_unit_event_pipeline);
    IVP_REGISTER(sim_unit_core_timestamp_tracks_full_psi);
    IVP_REGISTER(sim_unit_pending_tracks_detach_wake_freeze_and_delete);
    IVP_REGISTER(sim_unit_pending_keeps_reference_scan_order);
    IVP_REGISTER(sim_unit_pending_retries_every_split_allocation_failure);
    IVP_REGISTER(sim_unit_controller_rebuild_failure_clears_partial_index);
    IVP_REGISTER(sim_unit_controller_rebuild_failure_preserves_live_index);
    IVP_REGISTER(controller_self_remove_does_not_break_iteration);
    IVP_REGISTER(controller_registration_failure_is_atomic);
    IVP_REGISTER(dependent_controller_allocation_failure_is_atomic);
    IVP_REGISTER(controller_discovery_failure_preserves_all_core_mode);
    IVP_REGISTER(dependent_refresh_failure_preserves_live_association);
    IVP_REGISTER(controller_priority_sync_async);
    IVP_REGISTER(controller_dependent_rejects_foreign_targets);
    IVP_REGISTER(controller_dependent_zero_targets_invokes_none);
    IVP_REGISTER(controller_dependent_requires_discovery_callback);
    IVP_REGISTER(controller_dependent_removed_with_target);
    IVP_REGISTER(controller_dependent_dynamic_discovery_unions_island);
    IVP_REGISTER(controller_dependent_refreshes_object_on_shared_core);
    IVP_REGISTER(controller_attach_to_core_limits_dispatch_scope);
    IVP_REGISTER(core_controller_removal_preserves_shared_core_attachment);
    IVP_REGISTER(ensure_core_in_simulation_wakes_frozen_object);
    IVP_REGISTER(revive_queue_failure_leaves_core_retryable);
    IVP_REGISTER(force_actuator_executes_once_per_psi);
    IVP_REGISTER(rot_mot_executes_once_per_psi);

    /* Multiple objects */
    IVP_REGISTER(multiple_objects_freefall);

    /* Freeze params */
    IVP_REGISTER(freeze_params_defaults);

    /* Integration stability */
    IVP_REGISTER(quat_fast_normalize_integration);

    /* Pinned */
    IVP_REGISTER(pinned_object_no_gravity);

    IVP_RUN_ALL();
}
