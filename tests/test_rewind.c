/* test_rewind.c -- seeking a recorded world must reproduce the recording.
 *
 * A reference run applies the same per-PSI inputs to a plain world and
 * records its state after every PSI.  The rewinder records the scene, built
 * in a memory region, with those inputs, seeks back and forth inside the
 * recording, branches with different inputs, and drops old keyframes from a
 * small ring; every world it reaches must equal the reference state of that
 * PSI bit for bit. */
#include "ivp_test.h"
#include "ivp_test_fault_allocator.h"
#include "ivp_test_scene.h"

#include <ivp/base/ivp_memory_region.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_rewind.h>
#include <ivp/geometry/ivp_surman.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define RW_MAX_OBJECTS 32
#define RW_PSI_COUNT 300
#define RW_BRANCH_PSI 100

typedef ivp_test_scene_assets_t rw_assets_t;

/* controller states of a built world; they must outlive it, and a
 * recorded world keeps them in its region */
typedef struct rw_controllers {
    ivp_spring_state_t springs[2];
    ivp_constraint_state_t constraints[2];
} rw_controllers_t;

typedef struct rw_object_state {
    ivp_real_t position[3];
    ivp_real_t speed[3];
    float rot_speed[3];
    ivp_real_t orientation[4];
    int movement_state;
} rw_object_state_t;

typedef struct rw_state {
    ivp_time_t time;
    int object_count;
    rw_object_state_t objects[RW_MAX_OBJECTS];
} rw_state_t;

/* the input of one PSI: a speed push on one object, by order key */
typedef struct rw_input {
    uint64_t object;
    float speed[3];
} rw_input_t;

/* ── Scene ───────────────────────────────────────────────────────────── */

/* gravity: a pile joined by a spring and a ball-socket constraint, a
 * spring to a rolling ball, and two bouncing balls */
static bool rw_build(ivp_environment_t *env, rw_assets_t *assets,
                     rw_controllers_t *controllers)
{
    if (!ivp_test_scene_ground(env, assets)) return false;
    const ivp_surman_t *cube = &assets->cube_surman.base;
    ivp_object_t *cubes[5];
    for (int i = 0; i < 4; i++) {
        cubes[i] = ivp_test_scene_body(env, assets, cube, (i & 1) ? 0.55 : -0.55,
                                       0.9, (i & 2) ? 0.55 : -0.55, true);
        if (!cubes[i]) return false;
    }
    cubes[4] = ivp_test_scene_body(env, assets, cube, 0.1, -0.4, -0.1, true);
    ivp_object_t *ball = ivp_test_scene_body(env, assets, NULL, 3.0, 1.05, 0.0, true);
    ivp_object_t *bouncers[2] = {
        ivp_test_scene_body(env, assets, NULL, -6.0, -2.0, 1.0, true),
        ivp_test_scene_body(env, assets, NULL, 6.0, -3.0, -1.0, true)
    };
    if (!cubes[4] || !ball || !bouncers[0] || !bouncers[1]) return false;

    ivp_vec3r_t center = ivp_vec3r_zero();
    ivp_spring_state_init(&controllers->springs[0]);
    ivp_spring_state_setup(&controllers->springs[0], cubes[0], &center,
                           cubes[1], &center, IVP_REAL(30.0), IVP_REAL(3.0),
                           IVP_REAL(0.8));
    ivp_spring_state_init(&controllers->springs[1]);
    ivp_spring_state_setup(&controllers->springs[1], cubes[4], &center, ball,
                           &center, IVP_REAL(10.0), IVP_REAL(1.0),
                           IVP_REAL(2.0));
    ivp_vec3r_t anchor;
    ivp_vec3r_set(&anchor, IVP_REAL(0.0), IVP_REAL(0.9), IVP_REAL(0.55));
    ivp_constraint_init_ball_socket_ws(&controllers->constraints[0], cubes[2],
                                       &anchor, cubes[3]);
    ivp_vec3r_set(&anchor, IVP_REAL(-0.2), IVP_REAL(0.25), IVP_REAL(-0.3));
    ivp_constraint_init_ball_socket_ws(&controllers->constraints[1], cubes[0],
                                       &anchor, cubes[4]);
    for (int i = 0; i < 2; i++) {
        if (!ivp_controller_handle_is_valid(
                ivp_environment_announce_dependent_controller(
                    env, 100, ivp_controller_spring, &controllers->springs[i],
                    ivp_spring_discovery)) ||
            !ivp_controller_handle_is_valid(
                ivp_environment_announce_dependent_controller(
                    env, 50, ivp_controller_constraint,
                    &controllers->constraints[i], ivp_constraint_discovery))) {
            return false;
        }
    }
    return true;
}

static void rw_config(ivp_environment_config_t *config)
{
    ivp_environment_config_init(config);
    config->disable_performance_counter = true;
}

/* ── Inputs ──────────────────────────────────────────────────────────── */

/* a push every 13 PSIs on a dynamic object; the branch variant differs */
static size_t rw_make_input(uint64_t psi, int variant, rw_input_t *input)
{
    if (psi % 13u != (uint64_t)variant * 5u) return 0;
    memset(input, 0, sizeof(*input));
    input->object = 2u + (psi / 13u) % 8u;   /* after the ground */
    input->speed[0] = (float)((int)(psi % 7u) - 3) * 0.25f;
    input->speed[1] = -1.5f - 0.5f * (float)variant;
    input->speed[2] = (float)((int)(psi % 5u) - 2) * 0.25f;
    return sizeof(*input);
}

/* inputs of the recording, or of the branch after RW_BRANCH_PSI */
static size_t rw_input_for(uint64_t psi, bool branch, rw_input_t *input)
{
    return rw_make_input(psi, branch && psi >= RW_BRANCH_PSI ? 1 : 0, input);
}

static ivp_object_t *rw_find(ivp_environment_t *env, uint64_t order_key)
{
    int count = ivp_environment_get_object_count(env);
    for (int i = 0; i < count; i++) {
        ivp_object_t *object = ivp_environment_get_object(env, i);
        if (ivp_object_order_key(object) == order_key) return object;
    }
    return NULL;
}

typedef struct rw_apply_log {
    uint64_t calls;
} rw_apply_log_t;

static void rw_apply(void *ctx, ivp_environment_t *env, uint64_t psi,
                     const void *data, size_t size)
{
    (void)psi;
    if (ctx) ((rw_apply_log_t *)ctx)->calls++;
    if (size != sizeof(rw_input_t)) return;
    rw_input_t input;
    memcpy(&input, data, sizeof(input));
    ivp_object_t *object = rw_find(env, input.object);
    if (!object) return;
    ivp_vec3r_t speed;
    ivp_vec3r_set(&speed, (ivp_real_t)input.speed[0],
                  (ivp_real_t)input.speed[1], (ivp_real_t)input.speed[2]);
    ivp_object_async_add_speed_ws(object, &speed);
}

/* ── States ──────────────────────────────────────────────────────────── */

static int rw_compare_order(const void *left, const void *right)
{
    uint64_t a = ivp_object_order_key(*(const ivp_object_t *const *)left);
    uint64_t b = ivp_object_order_key(*(const ivp_object_t *const *)right);
    return a < b ? -1 : a > b ? 1 : 0;
}

static void rw_capture(const ivp_environment_t *env, rw_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->time = ivp_environment_get_time(env);
    ivp_object_t *ordered[RW_MAX_OBJECTS];
    int count = ivp_environment_get_object_count(env);
    if (count > RW_MAX_OBJECTS) count = RW_MAX_OBJECTS;
    for (int i = 0; i < count; i++) ordered[i] = ivp_environment_get_object(env, i);
    qsort(ordered, (size_t)count, sizeof(ordered[0]), rw_compare_order);
    state->object_count = count;
    for (int i = 0; i < count; i++) {
        const ivp_core_t *core = ordered[i]->physical_core;
        rw_object_state_t *out = &state->objects[i];
        for (int axis = 0; axis < 3; axis++) {
            out->position[axis] = core->pos_world_f_core_last_psi.k[axis];
            out->speed[axis] = core->speed.k[axis];
            out->rot_speed[axis] = core->rot_speed.k[axis];
        }
        out->orientation[0] = core->q_world_f_core_last_psi.x;
        out->orientation[1] = core->q_world_f_core_last_psi.y;
        out->orientation[2] = core->q_world_f_core_last_psi.z;
        out->orientation[3] = core->q_world_f_core_last_psi.w;
        out->movement_state = (int)core->movement_state;
    }
}

static bool rw_matches(const rw_state_t *expected, const ivp_environment_t *env,
                       const char *what, uint64_t psi)
{
    rw_state_t actual;
    rw_capture(env, &actual);
    if (memcmp(expected, &actual, sizeof(actual)) == 0) return true;
    fprintf(stderr, "rewind: %s at psi %llu differs from the reference\n",
            what, (unsigned long long)psi);
    return false;
}

/* ── Fixtures ────────────────────────────────────────────────────────── */

typedef struct rw_fixture {
    rw_assets_t assets;
    rw_state_t reference[RW_PSI_COUNT + 1];          /* after k PSIs */
    rw_state_t branch_reference[RW_PSI_COUNT + 1];
} rw_fixture_t;

static ivp_environment_t *rw_world(rw_assets_t *assets,
                                   rw_controllers_t *controllers)
{
    ivp_environment_config_t config;
    rw_config(&config);
    ivp_environment_t *env = ivp_environment_create(&config);
    if (env && !rw_build(env, assets, controllers)) {
        ivp_environment_destroy(env);
        env = NULL;
    }
    return env;
}

/* a plain world with the recording's (or the branch's) inputs */
static bool rw_reference_run(rw_fixture_t *fixture, bool branch,
                             rw_state_t *states)
{
    rw_controllers_t *controllers =
        (rw_controllers_t *)calloc(1, sizeof(*controllers));
    ivp_environment_t *env =
        controllers ? rw_world(&fixture->assets, controllers) : NULL;
    if (!env) {
        free(controllers);
        return false;
    }
    rw_capture(env, &states[0]);
    for (uint64_t psi = 0; psi < RW_PSI_COUNT; psi++) {
        rw_input_t input;
        size_t size = rw_input_for(psi, branch, &input);
        rw_apply(NULL, env, psi, &input, size);
        ivp_environment_simulate_one_psi(env);
        rw_capture(env, &states[psi + 1]);
    }
    ivp_environment_destroy(env);
    free(controllers);
    return true;
}

static rw_fixture_t *rw_fixture_create(void)
{
    rw_fixture_t *fixture = (rw_fixture_t *)calloc(1, sizeof(*fixture));
    if (!fixture) return NULL;
    if (!ivp_test_scene_assets_init(&fixture->assets) ||
        !rw_reference_run(fixture, false, fixture->reference) ||
        !rw_reference_run(fixture, true, fixture->branch_reference)) {
        ivp_test_scene_assets_destroy(&fixture->assets);
        free(fixture);
        return NULL;
    }
    return fixture;
}

static void rw_fixture_destroy(rw_fixture_t *fixture)
{
    ivp_test_scene_assets_destroy(&fixture->assets);
    free(fixture);
}

/* a recorded world: its region, the world and controller states in it,
 * and the rewinder */
typedef struct rw_session {
    ivp_memory_region_t *region;
    ivp_environment_t *env;
    rw_controllers_t *controllers;
    ivp_rewind_t *rewind;
} rw_session_t;

static void rw_close(rw_session_t *session)
{
    ivp_rewind_destroy(session->rewind);
    if (session->env) ivp_environment_destroy(session->env);
    ivp_memory_region_destroy(session->region);
    memset(session, 0, sizeof(*session));
}

static bool rw_open_with(rw_fixture_t *fixture, int interval, int capacity,
                         rw_apply_log_t *log, const ivp_allocator_t *storage,
                         rw_session_t *session)
{
    memset(session, 0, sizeof(*session));
    session->region = ivp_memory_region_create(0, NULL);
    if (!session->region) return false;
    const ivp_allocator_t *world = ivp_memory_region_allocator(session->region);
    ivp_environment_config_t world_config;
    rw_config(&world_config);
    world_config.allocator = (ivp_allocator_t *)world;
    session->env = ivp_environment_create(&world_config);
    session->controllers =
        (rw_controllers_t *)ivp_alloc(world, sizeof(*session->controllers));
    if (!session->env || !session->controllers ||
        !rw_build(session->env, &fixture->assets, session->controllers)) {
        rw_close(session);
        return false;
    }
    ivp_rewind_config_t config;
    memset(&config, 0, sizeof(config));
    config.keyframe_interval = interval;
    config.keyframe_capacity = capacity;
    config.apply = rw_apply;
    config.apply_ctx = log;
    config.storage = storage;
    ivp_result_t result = IVP_OK;
    session->rewind = ivp_rewind_create(session->env, session->region, &config,
                                        &result);
    if (!session->rewind) {
        fprintf(stderr, "rewind: create failed: %s\n", ivp_result_str(result));
        rw_close(session);
        return false;
    }
    return true;
}

static bool rw_open(rw_fixture_t *fixture, int interval, int capacity,
                    rw_apply_log_t *log, rw_session_t *session)
{
    return rw_open_with(fixture, interval, capacity, log, NULL, session);
}

/* record psi_count PSIs, checking every world against `states` */
static bool rw_record(ivp_rewind_t *rewind, bool branch, uint64_t psi_count,
                      const rw_state_t *states)
{
    while (ivp_rewind_current_psi(rewind) < psi_count) {
        uint64_t psi = ivp_rewind_current_psi(rewind);
        rw_input_t input;
        size_t size = rw_input_for(psi, branch, &input);
        ivp_result_t result = ivp_rewind_step(rewind, &input, size);
        if (result != IVP_OK) {
            fprintf(stderr, "rewind: step at psi %llu failed: %s\n",
                    (unsigned long long)psi, ivp_result_str(result));
            return false;
        }
        if (!rw_matches(&states[psi + 1], ivp_rewind_environment(rewind),
                        "recorded world", psi + 1)) {
            return false;
        }
    }
    return true;
}

static bool rw_seek_and_check(ivp_rewind_t *rewind, uint64_t psi,
                              const rw_state_t *states)
{
    ivp_result_t result = ivp_rewind_seek(rewind, psi);
    if (result != IVP_OK) {
        fprintf(stderr, "rewind: seek to %llu failed: %s\n",
                (unsigned long long)psi, ivp_result_str(result));
        return false;
    }
    return ivp_rewind_current_psi(rewind) == psi &&
           rw_matches(&states[psi], ivp_rewind_environment(rewind),
                      "seek", psi);
}

/* ── Tests ───────────────────────────────────────────────────────────── */

IVP_TEST(recording_and_seeking_reproduce_the_reference)
{
    rw_fixture_t *fixture = rw_fixture_create();
    IVP_ASSERT_TRUE(fixture != NULL);
    rw_apply_log_t log = { 0 };
    rw_session_t session;
    IVP_ASSERT_TRUE(rw_open(fixture, 25, 64, &log, &session));
    ivp_rewind_t *rewind = session.rewind;
    IVP_ASSERT_TRUE(rw_record(rewind, false, RW_PSI_COUNT, fixture->reference));
    IVP_ASSERT_EQ(ivp_rewind_recorded_psi(rewind), (uint64_t)RW_PSI_COUNT);
    IVP_ASSERT_EQ(ivp_rewind_oldest_psi(rewind), (uint64_t)0);

    /* back, forward, between and onto keyframes, both ends */
    static const uint64_t targets[] = {
        0, 1, 37, 150, 299, 300, 120, 250, 25, 24, 26, 5, 300, 0, 300
    };
    for (size_t i = 0; i < sizeof(targets) / sizeof(targets[0]); i++) {
        IVP_ASSERT_TRUE(rw_seek_and_check(rewind, targets[i], fixture->reference));
    }
    /* the input callback ran for every recorded and replayed PSI */
    IVP_ASSERT_TRUE(log.calls > (uint64_t)RW_PSI_COUNT);
    IVP_ASSERT_EQ(ivp_rewind_seek(rewind, RW_PSI_COUNT + 1), IVP_ERR_INVALID_ARG);
    rw_close(&session);
    rw_fixture_destroy(fixture);
}

IVP_TEST(stepping_after_a_seek_branches)
{
    rw_fixture_t *fixture = rw_fixture_create();
    IVP_ASSERT_TRUE(fixture != NULL);
    /* the branch inputs change the world only after the branch point */
    IVP_ASSERT_TRUE(memcmp(&fixture->reference[RW_BRANCH_PSI],
                           &fixture->branch_reference[RW_BRANCH_PSI],
                           sizeof(rw_state_t)) == 0);
    IVP_ASSERT_TRUE(memcmp(&fixture->reference[200],
                           &fixture->branch_reference[200],
                           sizeof(rw_state_t)) != 0);
    rw_session_t session;
    IVP_ASSERT_TRUE(rw_open(fixture, 20, 64, NULL, &session));
    ivp_rewind_t *rewind = session.rewind;
    IVP_ASSERT_TRUE(rw_record(rewind, false, RW_PSI_COUNT, fixture->reference));

    /* branch off at RW_BRANCH_PSI with different inputs */
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, RW_BRANCH_PSI, fixture->reference));
    IVP_ASSERT_TRUE(rw_record(rewind, true, 200, fixture->branch_reference));
    IVP_ASSERT_EQ(ivp_rewind_recorded_psi(rewind), (uint64_t)200);
    /* the old future is gone; the branch is reachable */
    IVP_ASSERT_EQ(ivp_rewind_seek(rewind, 250), IVP_ERR_INVALID_ARG);
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 150, fixture->branch_reference));
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 60, fixture->branch_reference));
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 200, fixture->branch_reference));
    /* and continues to the end like the branch reference */
    IVP_ASSERT_TRUE(rw_record(rewind, true, RW_PSI_COUNT, fixture->branch_reference));
    rw_close(&session);
    rw_fixture_destroy(fixture);
}

IVP_TEST(the_keyframe_ring_limits_the_oldest_psi)
{
    rw_fixture_t *fixture = rw_fixture_create();
    IVP_ASSERT_TRUE(fixture != NULL);
    /* keyframes at 0, 10, ..., 90; four are kept */
    rw_session_t session;
    IVP_ASSERT_TRUE(rw_open(fixture, 10, 4, NULL, &session));
    ivp_rewind_t *rewind = session.rewind;
    IVP_ASSERT_TRUE(rw_record(rewind, false, 100, fixture->reference));
    IVP_ASSERT_EQ(ivp_rewind_oldest_psi(rewind), (uint64_t)60);
    IVP_ASSERT_EQ(ivp_rewind_recorded_psi(rewind), (uint64_t)100);
    IVP_ASSERT_EQ(ivp_rewind_seek(rewind, 59), IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_rewind_current_psi(rewind), (uint64_t)100);
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 60, fixture->reference));
    /* replays across the inputs of PSIs 78 and 91 */
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 87, fixture->reference));
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 95, fixture->reference));
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 100, fixture->reference));
    /* recording on keeps moving the window */
    IVP_ASSERT_TRUE(rw_record(rewind, false, 150, fixture->reference));
    IVP_ASSERT_EQ(ivp_rewind_oldest_psi(rewind), (uint64_t)110);
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 110, fixture->reference));
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 135, fixture->reference));
    rw_close(&session);
    rw_fixture_destroy(fixture);
}

/* Branching discards the keyframes of the old future, so they cannot take
 * ring slots from valid ones or break the order of the ring. */
IVP_TEST(branching_inside_a_full_ring)
{
    rw_fixture_t *fixture = rw_fixture_create();
    IVP_ASSERT_TRUE(fixture != NULL);
    /* keyframes 0 ... 140; six are kept: 90 ... 140 */
    rw_session_t session;
    IVP_ASSERT_TRUE(rw_open(fixture, 10, 6, NULL, &session));
    ivp_rewind_t *rewind = session.rewind;
    IVP_ASSERT_TRUE(rw_record(rewind, false, 150, fixture->reference));
    IVP_ASSERT_EQ(ivp_rewind_oldest_psi(rewind), (uint64_t)90);
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, RW_BRANCH_PSI, fixture->reference));
    IVP_ASSERT_TRUE(rw_record(rewind, true, 150, fixture->branch_reference));
    IVP_ASSERT_EQ(ivp_rewind_oldest_psi(rewind), (uint64_t)90);
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 95, fixture->branch_reference));
    /* replays across the branch input of PSI 122 */
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 125, fixture->branch_reference));
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 150, fixture->branch_reference));
    rw_close(&session);
    rw_fixture_destroy(fixture);
}

/* A step that cannot store its keyframe or input changes nothing. */
IVP_TEST(a_failed_step_changes_nothing)
{
    rw_fixture_t *fixture = rw_fixture_create();
    IVP_ASSERT_TRUE(fixture != NULL);
    ivp_test_fault_allocator_t fault;
    ivp_test_fault_allocator_init_tracked(&fault, NULL);
    rw_session_t session;
    IVP_ASSERT_TRUE(rw_open_with(fixture, 25, 64, NULL, &fault.allocator, &session));
    ivp_rewind_t *rewind = session.rewind;
    IVP_ASSERT_TRUE(rw_record(rewind, false, 75, fixture->reference));

    /* the step from PSI 75 takes a keyframe first */
    int failures = 0;
    for (int fail_at = 1;; fail_at++) {
        ivp_test_fault_allocator_reset(&fault, fail_at);
        rw_input_t input;
        size_t size = rw_input_for(75, false, &input);
        ivp_result_t result = ivp_rewind_step(rewind, &input, size);
        bool injected = fault.attempts >= fail_at;
        fault.fail_at = INT_MAX;
        if (!injected) {
            IVP_ASSERT_EQ(result, IVP_OK);
            IVP_ASSERT_TRUE(rw_matches(&fixture->reference[76],
                                       session.env, "step", 76));
            break;
        }
        IVP_ASSERT_EQ(result, IVP_ERR_OUT_OF_MEMORY);
        IVP_ASSERT_EQ(ivp_rewind_current_psi(rewind), (uint64_t)75);
        IVP_ASSERT_EQ(ivp_rewind_recorded_psi(rewind), (uint64_t)75);
        IVP_ASSERT_TRUE(rw_matches(&fixture->reference[75], session.env,
                                   "kept world", 75));
        failures++;
    }
    IVP_ASSERT_TRUE(failures > 0);
    /* the recording still seeks and replays */
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 30, fixture->reference));
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 76, fixture->reference));
    ivp_rewind_destroy(rewind);
    session.rewind = NULL;
    IVP_ASSERT_EQ(fault.live_bytes, (size_t)0);
    rw_close(&session);
    rw_fixture_destroy(fixture);
}

/* A restore keeps the world in place: the environment, objects and
 * controller states keep their addresses. */
IVP_TEST(seeking_keeps_objects_and_controller_states_in_place)
{
    rw_fixture_t *fixture = rw_fixture_create();
    IVP_ASSERT_TRUE(fixture != NULL);
    rw_session_t session;
    IVP_ASSERT_TRUE(rw_open(fixture, 25, 64, NULL, &session));
    ivp_rewind_t *rewind = session.rewind;
    ivp_spring_state_t *spring = &session.controllers->springs[0];
    ivp_object_t *spring_end = spring->obj_a;
    int object_count = ivp_environment_get_object_count(session.env);
    IVP_ASSERT_TRUE(rw_record(rewind, false, 80, fixture->reference));
    IVP_ASSERT_TRUE(rw_seek_and_check(rewind, 30, fixture->reference));
    IVP_ASSERT_TRUE(ivp_rewind_environment(rewind) == session.env);
    IVP_ASSERT_TRUE(spring->obj_a == spring_end);
    IVP_ASSERT_TRUE(spring_end->owner == session.env);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(session.env), object_count);
    rw_close(&session);
    rw_fixture_destroy(fixture);
}

/* the input callback of rw_spawning: a ball dropped in at PSI 40 */
static void rw_spawn(void *ctx, ivp_environment_t *env, uint64_t psi,
                     const void *data, size_t size)
{
    (void)data;
    (void)size;
    if (psi == 40) {
        ivp_test_scene_body(env, (rw_assets_t *)ctx, NULL, 1.5, -3.0, 0.5, true);
    }
}

/* Objects created after a keyframe are gone after seeking back before
 * them, and come back on replay. */
IVP_TEST(seeking_back_removes_later_objects)
{
    rw_fixture_t *fixture = rw_fixture_create();
    IVP_ASSERT_TRUE(fixture != NULL);
    rw_session_t session;
    IVP_ASSERT_TRUE(rw_open(fixture, 10, 64, NULL, &session));
    ivp_rewind_destroy(session.rewind);
    ivp_rewind_config_t config;
    memset(&config, 0, sizeof(config));
    config.keyframe_interval = 10;
    config.keyframe_capacity = 64;
    config.apply = rw_spawn;
    config.apply_ctx = &fixture->assets;
    ivp_result_t result = IVP_OK;
    session.rewind = ivp_rewind_create(session.env, session.region, &config, &result);
    IVP_ASSERT_TRUE(session.rewind != NULL);
    int before = ivp_environment_get_object_count(session.env);
    for (int psi = 0; psi < 60; psi++) {
        IVP_ASSERT_EQ(ivp_rewind_step(session.rewind, NULL, 0), IVP_OK);
    }
    IVP_ASSERT_EQ(ivp_environment_get_object_count(session.env), before + 1);
    rw_state_t at_60;
    rw_capture(session.env, &at_60);
    IVP_ASSERT_EQ(ivp_rewind_seek(session.rewind, 35), IVP_OK);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(session.env), before);
    IVP_ASSERT_EQ(ivp_rewind_seek(session.rewind, 60), IVP_OK);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(session.env), before + 1);
    IVP_ASSERT_TRUE(rw_matches(&at_60, session.env, "replayed spawn", 60));
    rw_close(&session);
    rw_fixture_destroy(fixture);
}

/* A world that does not live in the region cannot be recorded. */
IVP_TEST(create_requires_the_world_in_the_region)
{
    ivp_memory_region_t *region = ivp_memory_region_create(1u << 24, NULL);
    IVP_ASSERT_TRUE(region != NULL);
    ivp_environment_config_t world_config;
    rw_config(&world_config);
    ivp_environment_t *env = ivp_environment_create(&world_config);
    ivp_rewind_config_t config;
    memset(&config, 0, sizeof(config));
    config.keyframe_interval = 10;
    config.keyframe_capacity = 4;
    ivp_result_t result = IVP_OK;
    IVP_ASSERT_TRUE(ivp_rewind_create(env, region, &config, &result) == NULL);
    IVP_ASSERT_EQ(result, IVP_ERR_INVALID_ARG);
    ivp_environment_destroy(env);
    ivp_memory_region_destroy(region);
}

/* Creating a rewinder that cannot store its first keyframe fails cleanly,
 * and its storage may not be the region it checkpoints. */
IVP_TEST(create_fails_cleanly)
{
    ivp_memory_region_t *region = ivp_memory_region_create(1u << 24, NULL);
    const ivp_allocator_t *world = ivp_memory_region_allocator(region);
    ivp_environment_config_t world_config;
    rw_config(&world_config);
    world_config.allocator = (ivp_allocator_t *)world;
    ivp_environment_t *env = ivp_environment_create(&world_config);
    IVP_ASSERT_TRUE(env != NULL);
    ivp_rewind_config_t config;
    memset(&config, 0, sizeof(config));
    config.keyframe_interval = 10;
    config.keyframe_capacity = 4;
    ivp_result_t result = IVP_OK;

    config.storage = world;
    IVP_ASSERT_TRUE(ivp_rewind_create(env, region, &config, &result) == NULL);
    IVP_ASSERT_EQ(result, IVP_ERR_INVALID_ARG);

    ivp_test_fault_allocator_t fault;
    ivp_test_fault_allocator_init_tracked(&fault, NULL);
    config.storage = &fault.allocator;
    int failures = 0;
    for (int fail_at = 1;; fail_at++) {
        ivp_test_fault_allocator_reset(&fault, fail_at);
        ivp_rewind_t *rewind = ivp_rewind_create(env, region, &config, &result);
        bool injected = fault.attempts >= fail_at;
        fault.fail_at = INT_MAX;
        if (!injected) {
            IVP_ASSERT_TRUE(rewind != NULL);
            ivp_rewind_destroy(rewind);
            break;
        }
        IVP_ASSERT_TRUE(rewind == NULL);
        IVP_ASSERT_EQ(result, IVP_ERR_OUT_OF_MEMORY);
        IVP_ASSERT_EQ(fault.live_bytes, (size_t)0);
        failures++;
    }
    IVP_ASSERT_TRUE(failures > 0);
    IVP_ASSERT_EQ(fault.live_bytes, (size_t)0);
    ivp_environment_destroy(env);
    ivp_memory_region_destroy(region);
}

/* caller state in the region that grows by a block every PSI */
typedef struct rw_hog {
    const ivp_allocator_t *alloc;
    int blocks;
} rw_hog_t;

#define RW_HOG_BLOCK (64u * 1024u)

static void rw_hog_apply(void *ctx, ivp_environment_t *env, uint64_t psi,
                         const void *data, size_t size)
{
    (void)env;
    (void)psi;
    (void)data;
    (void)size;
    rw_hog_t *hog = (rw_hog_t *)ctx;
    if (ivp_alloc(hog->alloc, RW_HOG_BLOCK)) hog->blocks++;
}

/* A region that fills up during a recorded PSI is reported; the PSI stays
 * recorded, and a replay meets the same refusal and the same world. */
IVP_TEST(a_full_region_is_reported_and_replayed_alike)
{
    rw_fixture_t *fixture = rw_fixture_create();
    IVP_ASSERT_TRUE(fixture != NULL);
    rw_session_t session;
    memset(&session, 0, sizeof(session));
    session.region = ivp_memory_region_create(4u << 20, NULL);
    const ivp_allocator_t *world = ivp_memory_region_allocator(session.region);
    ivp_environment_config_t world_config;
    rw_config(&world_config);
    world_config.allocator = (ivp_allocator_t *)world;
    session.env = ivp_environment_create(&world_config);
    session.controllers = (rw_controllers_t *)ivp_alloc(world, sizeof(*session.controllers));
    rw_hog_t *hog = (rw_hog_t *)ivp_alloc(world, sizeof(*hog));
    IVP_ASSERT_TRUE(session.env && session.controllers && hog);
    hog->alloc = world;
    IVP_ASSERT_TRUE(rw_build(session.env, &fixture->assets, session.controllers));
    ivp_rewind_config_t config;
    memset(&config, 0, sizeof(config));
    config.keyframe_interval = 10;
    config.keyframe_capacity = 64;
    config.apply = rw_hog_apply;
    config.apply_ctx = hog;
    ivp_result_t result = IVP_OK;
    session.rewind = ivp_rewind_create(session.env, session.region, &config, &result);
    IVP_ASSERT_TRUE(session.rewind != NULL);

    uint64_t full = 0;
    for (int psi = 0; psi < 200 && !full; psi++) {
        result = ivp_rewind_step(session.rewind, NULL, 0);
        if (result != IVP_OK) {
            IVP_ASSERT_EQ(result, IVP_ERR_OUT_OF_MEMORY);
            full = ivp_rewind_current_psi(session.rewind);
        }
    }
    IVP_ASSERT_TRUE(full > 10);
    IVP_ASSERT_EQ(ivp_rewind_recorded_psi(session.rewind), full);
    int blocks = hog->blocks;
    rw_state_t at_full;
    rw_capture(session.env, &at_full);

    IVP_ASSERT_EQ(ivp_rewind_seek(session.rewind, full - 5), IVP_OK);
    IVP_ASSERT_TRUE(hog->blocks < blocks);
    IVP_ASSERT_EQ(ivp_rewind_seek(session.rewind, full), IVP_ERR_OUT_OF_MEMORY);
    IVP_ASSERT_EQ(hog->blocks, blocks);
    IVP_ASSERT_TRUE(rw_matches(&at_full, session.env, "replayed full region", full));
    rw_close(&session);
    rw_fixture_destroy(fixture);
}

int main(void)
{
    IVP_REGISTER(recording_and_seeking_reproduce_the_reference);
    IVP_REGISTER(stepping_after_a_seek_branches);
    IVP_REGISTER(the_keyframe_ring_limits_the_oldest_psi);
    IVP_REGISTER(branching_inside_a_full_ring);
    IVP_REGISTER(a_failed_step_changes_nothing);
    IVP_REGISTER(seeking_keeps_objects_and_controller_states_in_place);
    IVP_REGISTER(seeking_back_removes_later_objects);
    IVP_REGISTER(create_requires_the_world_in_the_region);
    IVP_REGISTER(create_fails_cleanly);
    IVP_REGISTER(a_full_region_is_reported_and_replayed_alike);
    IVP_RUN_ALL();
}
