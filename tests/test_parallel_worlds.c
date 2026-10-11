/* test_parallel_worlds.c -- true parallel stepping of independent worlds. */
#include "ivp_test.h"
#include "ivp_test_fault_allocator.h"
#include "ivp_test_worker_runtime.h"

#include "dynamics/ivp_world_scheduler_internal.h"

#include <ivp/dynamics/ivp_world_scheduler.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/utility/ivp_active_value.h>

#include <fenv.h>
#include <math.h>
#include <stdatomic.h>
#include <string.h>

#if defined(_WIN32)
#include <windows.h>
typedef DWORD test_thread_id_t;
static test_thread_id_t test_thread_current(void) { return GetCurrentThreadId(); }
static bool test_thread_equal(test_thread_id_t a, test_thread_id_t b)
{
    return a == b;
}
static void test_thread_yield(void) { Sleep(0); }
#else
#include <pthread.h>
#include <sched.h>
typedef pthread_t test_thread_id_t;
static test_thread_id_t test_thread_current(void) { return pthread_self(); }
static bool test_thread_equal(test_thread_id_t a, test_thread_id_t b)
{
    return pthread_equal(a, b) != 0;
}
static void test_thread_yield(void) { (void)sched_yield(); }
#endif

#define PARALLEL_WORLD_COUNT 8
#define PARALLEL_WORKER_COUNT 4

typedef struct test_world {
    ivp_environment_t *env;
    ivp_object_t *balls[2];
    ivp_active_float_t *time_squared;
} test_world_t;

typedef struct overlap_probe {
    atomic_int callbacks;
    atomic_int active;
    atomic_int max_active;
    atomic_bool overlapped;
    ivp_world_scheduler_t *contender;
    ivp_environment_t *unowned_environment;
    atomic_int contention_attempts;
    atomic_int contention_result;
} overlap_probe_t;

typedef struct rounding_probe {
    atomic_int expected_mode;
    atomic_int callbacks;
    atomic_int mismatches;
} rounding_probe_t;

static void rounding_hook(void *ctx, ivp_environment_t *env,
                          ivp_psi_phase_t phase, ivp_time_t time,
                          ivp_real_t dt)
{
    (void)env;
    (void)time;
    (void)dt;
    if (phase != IVP_PSI_PHASE_PRE_PSI) return;
    rounding_probe_t *probe = (rounding_probe_t *)ctx;
    int expected = atomic_load_explicit(&probe->expected_mode,
                                        memory_order_relaxed);
    if (fegetround() != expected) {
        atomic_fetch_add_explicit(&probe->mismatches, 1,
                                  memory_order_relaxed);
    }
    atomic_fetch_add_explicit(&probe->callbacks, 1, memory_order_relaxed);
}

typedef struct serial_fallback_probe {
    test_thread_id_t caller_thread;
    ivp_world_scheduler_t *scheduler;
    atomic_int callbacks;
    atomic_int next_world;
    atomic_int contention_attempts;
    atomic_int contention_result;
    atomic_bool caller_thread_only;
    atomic_bool array_order;
} serial_fallback_probe_t;

typedef struct serial_fallback_hook_context {
    serial_fallback_probe_t *probe;
    int world_index;
} serial_fallback_hook_context_t;

static void serial_fallback_hook(void *ctx, ivp_environment_t *env,
                                 ivp_psi_phase_t phase, ivp_time_t time,
                                 ivp_real_t dt)
{
    (void)env;
    (void)time;
    (void)dt;
    if (phase != IVP_PSI_PHASE_PRE_PSI) return;
    serial_fallback_hook_context_t *hook =
        (serial_fallback_hook_context_t *)ctx;
    serial_fallback_probe_t *probe = hook->probe;
    if (probe->scheduler &&
        atomic_fetch_add_explicit(&probe->contention_attempts, 1,
                                  memory_order_relaxed) == 0) {
        ivp_environment_t *same_world[] = { env };
        ivp_result_t result = ivp_world_scheduler_simulate_one_psi(
            probe->scheduler, same_world, 1);
        atomic_store_explicit(&probe->contention_result, (int)result,
                              memory_order_relaxed);
    }
    if (!test_thread_equal(test_thread_current(), probe->caller_thread)) {
        atomic_store_explicit(&probe->caller_thread_only, false,
                              memory_order_relaxed);
    }
    int next = atomic_fetch_add_explicit(&probe->next_world, 1,
                                         memory_order_relaxed);
    if (next != hook->world_index) {
        atomic_store_explicit(&probe->array_order, false,
                              memory_order_relaxed);
    }
    atomic_fetch_add_explicit(&probe->callbacks, 1, memory_order_relaxed);
}

static void overlap_hook(void *ctx, ivp_environment_t *env,
                         ivp_psi_phase_t phase, ivp_time_t time,
                         ivp_real_t dt)
{
    (void)env;
    (void)time;
    (void)dt;
    if (phase != IVP_PSI_PHASE_PRE_PSI) return;
    overlap_probe_t *probe = (overlap_probe_t *)ctx;
    atomic_fetch_add_explicit(&probe->callbacks, 1, memory_order_relaxed);
    if (probe->contender &&
        atomic_fetch_add_explicit(&probe->contention_attempts, 1,
                                  memory_order_relaxed) == 0) {
        /* The first environment is free and the second is owned by the
         * outer dispatch.  Rejection must not partially advance the first. */
        ivp_environment_t *contended_batch[] = {
            probe->unowned_environment, env
        };
        ivp_result_t result = ivp_world_scheduler_simulate_one_psi(
            probe->contender, contended_batch, 2);
        atomic_store_explicit(&probe->contention_result, (int)result,
                              memory_order_release);
    }
    int active = atomic_fetch_add_explicit(&probe->active, 1,
                                           memory_order_acq_rel) + 1;
    int old_max = atomic_load_explicit(&probe->max_active,
                                       memory_order_relaxed);
    while (active > old_max &&
           !atomic_compare_exchange_weak_explicit(
               &probe->max_active, &old_max, active,
               memory_order_relaxed, memory_order_relaxed)) {
    }
    if (active >= 2) {
        atomic_store_explicit(&probe->overlapped, true, memory_order_release);
    } else {
        for (int i = 0; i < 100000 &&
                        !atomic_load_explicit(&probe->overlapped,
                                              memory_order_acquire); i++) {
            test_thread_yield();
        }
    }
    atomic_fetch_sub_explicit(&probe->active, 1, memory_order_acq_rel);
}

static bool test_world_init(test_world_t *world, int variant)
{
    memset(world, 0, sizeof(*world));
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    ivp_vec3r_set(&config.gravity, IVP_REAL(0.0), IVP_REAL(0.0),
                  IVP_REAL(0.0));
    config.disable_performance_counter = true;
    world->env = ivp_environment_create(&config);
    if (!world->env) return false;
    ivp_active_value_manager_t *active_values =
        ivp_environment_get_active_value_manager(world->env);
    ivp_active_float_t *time_value =
        ivp_active_value_manager_current_time(active_values);
    world->time_squared = ivp_active_float_mult_create(
        NULL, "time_squared", time_value, time_value);
    if (!world->time_squared) return false;

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.5);
    object_template.mass = IVP_REAL(1.0);
    object_template.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&object_template.rot_speed_damp_factor,
                  IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    object_template.material.friction = IVP_REAL(0.6);
    object_template.material.elasticity = IVP_REAL(0.7);

    const ivp_real_t offset = (ivp_real_t)variant * IVP_REAL(0.01);
    ivp_vec3r_set(&object_template.position,
                  IVP_REAL(-1.2) - offset, IVP_REAL(0.0), IVP_REAL(0.0));
    world->balls[0] = ivp_environment_create_object(
        world->env, &object_template, NULL);
    ivp_vec3r_set(&object_template.position,
                  IVP_REAL(1.2) + offset, IVP_REAL(0.0), IVP_REAL(0.0));
    world->balls[1] = ivp_environment_create_object(
        world->env, &object_template, NULL);
    if (!world->balls[0] || !world->balls[1]) return false;

    const ivp_real_t speed = IVP_REAL(2.5) +
                             (ivp_real_t)variant * IVP_REAL(0.02);
    ivp_vec3r_set(&world->balls[0]->physical_core->speed,
                  speed, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&world->balls[1]->physical_core->speed,
                  -speed, IVP_REAL(0.0), IVP_REAL(0.0));
    return true;
}

static void test_world_destroy(test_world_t *world)
{
    ivp_active_float_remove_ref(world->time_squared);
    if (world->env) ivp_environment_destroy(world->env);
    memset(world, 0, sizeof(*world));
}

static bool vec3r_equal(const ivp_vec3r_t *a, const ivp_vec3r_t *b)
{
    return a->k[0] == b->k[0] && a->k[1] == b->k[1] &&
           a->k[2] == b->k[2];
}

static bool vec3f_equal(const ivp_vec3f_t *a, const ivp_vec3f_t *b)
{
    return a->k[0] == b->k[0] && a->k[1] == b->k[1] &&
           a->k[2] == b->k[2];
}

static bool quat_equal(const ivp_quatr_t *a, const ivp_quatr_t *b)
{
    return a->x == b->x && a->y == b->y && a->z == b->z && a->w == b->w;
}

static bool world_state_equal(const test_world_t *a, const test_world_t *b)
{
    if (ivp_environment_get_time(a->env) != ivp_environment_get_time(b->env)) {
        return false;
    }
    if (ivp_active_float_get(a->time_squared) !=
        ivp_active_float_get(b->time_squared)) {
        return false;
    }
    for (int i = 0; i < 2; i++) {
        const ivp_core_t *ca = a->balls[i]->physical_core;
        const ivp_core_t *cb = b->balls[i]->physical_core;
        if (!vec3r_equal(&ca->pos_world_f_core_last_psi,
                         &cb->pos_world_f_core_last_psi) ||
            !quat_equal(&ca->q_world_f_core_last_psi,
                        &cb->q_world_f_core_last_psi) ||
            !quat_equal(&ca->q_world_f_core_next_psi,
                        &cb->q_world_f_core_next_psi) ||
            !vec3r_equal(&ca->speed, &cb->speed) ||
            !vec3f_equal(&ca->rot_speed, &cb->rot_speed) ||
            ca->movement_state != cb->movement_state) {
            return false;
        }
    }
    return true;
}

static bool parallel_stats_are_zero(const ivp_environment_t *environment)
{
    ivp_parallel_stats_t stats;
    ivp_environment_get_parallel_stats(environment, &stats);
    return stats.island_dispatches == 0u && stats.island_tasks == 0u &&
           stats.broadphase_dispatches == 0u &&
           stats.broadphase_tasks == 0u && stats.solver_dispatches == 0u &&
           stats.solver_tasks == 0u && stats.max_workers_used == 0;
}

IVP_TEST(scheduler_cleans_partial_thread_start)
{
    ivp_test_fault_allocator_t allocator;
    ivp_test_fault_allocator_init_tracked(&allocator, NULL);
    ivp_test_worker_runtime_t runtime;
    ivp_test_worker_runtime_init(&runtime, NULL);
    atomic_store_explicit(&runtime.fail_thread_start_at, 3,
                          memory_order_relaxed);

    ivp_world_scheduler_t *scheduler =
        ivp_world_scheduler_create_with_runtime(
            PARALLEL_WORKER_COUNT, &allocator.allocator, &runtime.runtime);
    bool ok = scheduler == NULL &&
              atomic_load_explicit(&runtime.thread_start_calls,
                                   memory_order_relaxed) == 3 &&
              atomic_load_explicit(&runtime.thread_join_calls,
                                   memory_order_relaxed) == 2 &&
              allocator.live_bytes == 0u;
    ivp_world_scheduler_destroy(scheduler);
    IVP_ASSERT_TRUE(ok);
}

IVP_TEST(scheduler_setup_failures_are_atomic)
{
    test_world_t scheduled[PARALLEL_WORLD_COUNT] = {0};
    test_world_t direct[PARALLEL_WORLD_COUNT] = {0};
    ivp_environment_t *environments[PARALLEL_WORLD_COUNT] = {0};
    ivp_test_worker_runtime_t runtime;
    ivp_test_worker_runtime_init(&runtime, NULL);
    ivp_world_scheduler_t *scheduler = NULL;
    bool ok = true;
    int expected_joins = 0;
    int world_count = PARALLEL_WORLD_COUNT;
#if defined(IVP_HOME_BUILD) || defined(IVP_BENCHMARK_DIAGNOSTICS)
    world_count = 1;
#endif

    for (int i = 0; i < world_count; i++) {
        if (!test_world_init(&scheduled[i], i) ||
            !test_world_init(&direct[i], i)) {
            ok = false;
            goto cleanup;
        }
        environments[i] = scheduled[i].env;
    }
    scheduler = ivp_world_scheduler_create_with_runtime(
        PARALLEL_WORKER_COUNT, NULL, &runtime.runtime);
    if (!scheduler) {
        ok = false;
        goto cleanup;
    }

    ivp_test_worker_runtime_fail_next_capture(&runtime);
    if (ivp_world_scheduler_simulate_one_psi(
            scheduler, environments, (size_t)world_count) !=
        IVP_ERR_INVALID_STATE) {
        ok = false;
    }
    for (int i = 0; i < world_count; i++) {
        if (!world_state_equal(&scheduled[i], &direct[i])) ok = false;
    }

    ivp_test_worker_runtime_fail_next_apply(&runtime);
    if (ivp_world_scheduler_simulate_one_psi(
            scheduler, environments, (size_t)world_count) !=
        IVP_ERR_INVALID_STATE) {
        ok = false;
    }
    for (int i = 0; i < world_count; i++) {
        if (!world_state_equal(&scheduled[i], &direct[i])) ok = false;
    }

    if (ivp_world_scheduler_simulate_one_psi(
            scheduler, environments, (size_t)world_count) != IVP_OK) {
        ok = false;
    }
    for (int i = 0; i < world_count; i++) {
        ivp_environment_simulate_one_psi(direct[i].env);
        if (!world_state_equal(&scheduled[i], &direct[i])) ok = false;
    }

cleanup:
    expected_joins = scheduler ? PARALLEL_WORKER_COUNT : 0;
    ivp_world_scheduler_destroy(scheduler);
    if (atomic_load_explicit(&runtime.thread_join_calls,
                             memory_order_relaxed) != expected_joins) {
        ok = false;
    }
    for (int i = 0; i < PARALLEL_WORLD_COUNT; i++) {
        test_world_destroy(&scheduled[i]);
        test_world_destroy(&direct[i]);
    }
    IVP_ASSERT_TRUE(ok);
}

IVP_TEST(scheduler_allocation_failure_is_atomic)
{
    test_world_t scheduled = {0};
    test_world_t direct = {0};
    ivp_environment_t *environments[1] = {0};
    ivp_test_fault_allocator_t allocator;
    ivp_test_fault_allocator_init_tracked(&allocator, NULL);
    ivp_world_scheduler_t *scheduler = NULL;
    bool ok = test_world_init(&scheduled, 0) &&
              test_world_init(&direct, 0);
    if (ok) {
        environments[0] = scheduled.env;
        scheduler = ivp_world_scheduler_create(1, &allocator.allocator);
        ok = scheduler != NULL;
    }
    if (ok) {
        ivp_test_fault_allocator_reset(&allocator, 1);
        ok = ivp_world_scheduler_simulate_one_psi(
                 scheduler, environments, 1) == IVP_ERR_OUT_OF_MEMORY &&
             world_state_equal(&scheduled, &direct);
    }
    if (ok) {
        ivp_test_fault_allocator_reset(&allocator, INT_MAX);
        ok = ivp_world_scheduler_simulate_one_psi(
                 scheduler, environments, 1) == IVP_OK;
        ivp_environment_simulate_one_psi(direct.env);
        ok = ok && world_state_equal(&scheduled, &direct);
    }

    ivp_world_scheduler_destroy(scheduler);
    ok = ok && allocator.live_bytes == 0u;
    test_world_destroy(&scheduled);
    test_world_destroy(&direct);
    IVP_ASSERT_TRUE(ok);
}

IVP_TEST(scheduler_one_worker_is_fully_serial)
{
    test_world_t scheduled[PARALLEL_WORLD_COUNT] = {0};
    test_world_t direct[PARALLEL_WORLD_COUNT] = {0};
    ivp_environment_t *environments[PARALLEL_WORLD_COUNT] = {0};
    serial_fallback_hook_context_t hooks[PARALLEL_WORLD_COUNT];
    ivp_world_scheduler_t *scheduler = NULL;
    serial_fallback_probe_t probe;
    probe.caller_thread = test_thread_current();
    probe.scheduler = NULL;
    atomic_init(&probe.callbacks, 0);
    atomic_init(&probe.next_world, 0);
    atomic_init(&probe.contention_attempts, 0);
    atomic_init(&probe.contention_result, -1);
    atomic_init(&probe.caller_thread_only, true);
    atomic_init(&probe.array_order, true);
    bool ok = true;
    int world_count = PARALLEL_WORLD_COUNT;
#if defined(IVP_HOME_BUILD) || defined(IVP_BENCHMARK_DIAGNOSTICS)
    world_count = 1;
#endif

    for (int i = 0; i < world_count; i++) {
        if (!test_world_init(&scheduled[i], i) ||
            !test_world_init(&direct[i], i)) {
            ok = false;
            goto cleanup;
        }
        environments[i] = scheduled[i].env;
        hooks[i].probe = &probe;
        hooks[i].world_index = i;
        ivp_environment_set_phase_hook(scheduled[i].env,
                                       serial_fallback_hook, &hooks[i]);
        if (ivp_environment_get_parallel_worker_count(scheduled[i].env) != 1) {
            ok = false;
            goto cleanup;
        }
    }

    scheduler = ivp_world_scheduler_create(1, NULL);
    probe.scheduler = scheduler;
    if (!scheduler || ivp_world_scheduler_simulate_one_psi(
                          scheduler, environments,
                          (size_t)world_count) != IVP_OK) {
        ok = false;
        goto cleanup;
    }

    for (int i = 0; i < world_count; i++) {
        ivp_environment_set_phase_hook(scheduled[i].env, NULL, NULL);
        ivp_environment_simulate_one_psi(direct[i].env);
        if (!world_state_equal(&scheduled[i], &direct[i]) ||
            !parallel_stats_are_zero(scheduled[i].env)) {
            ok = false;
        }
    }
    if (atomic_load_explicit(&probe.callbacks, memory_order_relaxed) !=
            world_count ||
        atomic_load_explicit(&probe.next_world, memory_order_relaxed) !=
            world_count ||
        atomic_load_explicit(&probe.contention_result,
                             memory_order_relaxed) != IVP_ERR_INVALID_STATE ||
        !atomic_load_explicit(&probe.caller_thread_only,
                              memory_order_relaxed) ||
        !atomic_load_explicit(&probe.array_order, memory_order_relaxed)) {
        ok = false;
    }
    if (ok) {
        if (ivp_world_scheduler_simulate_dtime(
                scheduler, environments, (size_t)world_count,
                IVP_REAL(0.05)) != IVP_OK) {
            ok = false;
        }
        for (int i = 0; ok && i < world_count; i++) {
            ivp_environment_simulate_dtime(direct[i].env, IVP_REAL(0.05));
            if (!world_state_equal(&scheduled[i], &direct[i]) ||
                !parallel_stats_are_zero(scheduled[i].env)) {
                ok = false;
            }
        }
    }

cleanup:
    ivp_world_scheduler_destroy(scheduler);
    for (int i = 0; i < PARALLEL_WORLD_COUNT; i++) {
        test_world_destroy(&scheduled[i]);
        test_world_destroy(&direct[i]);
    }
    IVP_ASSERT_TRUE(ok);
}

IVP_TEST(scheduler_propagates_dispatch_fenv)
{
    test_world_t worlds[PARALLEL_WORLD_COUNT] = {0};
    ivp_environment_t *environments[PARALLEL_WORLD_COUNT] = {0};
    ivp_world_scheduler_t *scheduler = NULL;
    fenv_t saved_environment;
    bool environment_saved = fegetenv(&saved_environment) == 0;
    bool ok = environment_saved;
    int world_count = PARALLEL_WORLD_COUNT;
#if defined(IVP_HOME_BUILD) || defined(IVP_BENCHMARK_DIAGNOSTICS)
    world_count = 1;
#endif
    rounding_probe_t probe;
    atomic_init(&probe.expected_mode, FE_TONEAREST);
    atomic_init(&probe.callbacks, 0);
    atomic_init(&probe.mismatches, 0);

    for (int i = 0; ok && i < world_count; i++) {
        if (!test_world_init(&worlds[i], i)) {
            ok = false;
            break;
        }
        environments[i] = worlds[i].env;
        ivp_environment_set_phase_hook(worlds[i].env, rounding_hook, &probe);
    }
    scheduler = ivp_world_scheduler_create(PARALLEL_WORKER_COUNT, NULL);
    if (!scheduler) ok = false;

    const int modes[] = { FE_DOWNWARD, FE_UPWARD };
    for (size_t mode = 0; ok && mode < sizeof(modes) / sizeof(modes[0]);
         mode++) {
        atomic_store_explicit(&probe.expected_mode, modes[mode],
                              memory_order_relaxed);
        if (fesetround(modes[mode]) != 0 ||
            ivp_world_scheduler_simulate_one_psi(
                scheduler, environments, (size_t)world_count) != IVP_OK) {
            ok = false;
        }
    }
    if (atomic_load_explicit(&probe.callbacks, memory_order_relaxed) !=
            world_count * 2 ||
        atomic_load_explicit(&probe.mismatches, memory_order_relaxed) != 0) {
        ok = false;
    }

    if (environment_saved && fesetenv(&saved_environment) != 0) ok = false;
    ivp_world_scheduler_destroy(scheduler);
    for (int i = 0; i < PARALLEL_WORLD_COUNT; i++) {
        test_world_destroy(&worlds[i]);
    }
    IVP_ASSERT_TRUE(ok);
}

IVP_TEST(scheduler_parallel_execution)
{
    test_world_t parallel[PARALLEL_WORLD_COUNT] = {0};
    test_world_t serial[PARALLEL_WORLD_COUNT] = {0};
    ivp_environment_t *environments[PARALLEL_WORLD_COUNT] = {0};
    ivp_world_scheduler_t *scheduler = NULL;
    ivp_world_scheduler_t *contender = NULL;
    ivp_world_scheduler_t *invalid_scheduler = NULL;
    ivp_world_scheduler_t *negative_scheduler = NULL;
    bool ok = true;

    for (int i = 0; i < PARALLEL_WORLD_COUNT; i++) {
        if (!test_world_init(&parallel[i], i) ||
            !test_world_init(&serial[i], i)) {
            ok = false;
            goto cleanup;
        }
        environments[i] = parallel[i].env;
    }

    scheduler = ivp_world_scheduler_create(PARALLEL_WORKER_COUNT, NULL);
    contender = ivp_world_scheduler_create(1, NULL);
    if (!scheduler || !contender) {
        ok = false;
        goto cleanup;
    }
    invalid_scheduler = ivp_world_scheduler_create(0, NULL);
    negative_scheduler = ivp_world_scheduler_create(-1, NULL);
    if (invalid_scheduler != NULL ||
        negative_scheduler != NULL ||
        ivp_world_scheduler_simulate_one_psi(scheduler, NULL, 0) != IVP_OK ||
        ivp_world_scheduler_simulate_one_psi(scheduler, NULL, 1) !=
            IVP_ERR_INVALID_ARG ||
        ivp_world_scheduler_simulate_dtime(
            scheduler, environments, PARALLEL_WORLD_COUNT,
            IVP_REAL(-1.0)) != IVP_ERR_INVALID_ARG ||
        ivp_world_scheduler_simulate_dtime(
            scheduler, environments, PARALLEL_WORLD_COUNT,
            (ivp_real_t)NAN) != IVP_ERR_INVALID_ARG ||
        ivp_world_scheduler_simulate_dtime(
            scheduler, environments, PARALLEL_WORLD_COUNT,
            (ivp_real_t)INFINITY) != IVP_ERR_INVALID_ARG) {
        ok = false;
        goto cleanup;
    }

#if defined(IVP_HOME_BUILD) || defined(IVP_BENCHMARK_DIAGNOSTICS)
    if (ivp_world_scheduler_simulate_one_psi(
            scheduler, environments, PARALLEL_WORLD_COUNT) !=
        IVP_ERR_INVALID_STATE) {
        ok = false;
    }
    {
        ivp_environment_t *one_world[] = { environments[0] };
        if (ivp_world_scheduler_simulate_one_psi(
                scheduler, one_world, 1) != IVP_OK) {
            ok = false;
        } else {
            ivp_environment_simulate_one_psi(serial[0].env);
            if (!world_state_equal(&parallel[0], &serial[0])) ok = false;
        }
    }
    goto cleanup;
#endif

    overlap_probe_t probe;
    atomic_init(&probe.callbacks, 0);
    atomic_init(&probe.active, 0);
    atomic_init(&probe.max_active, 0);
    atomic_init(&probe.overlapped, false);
    probe.contender = contender;
    probe.unowned_environment = serial[0].env;
    const ivp_time_t unowned_time =
        ivp_environment_get_time(probe.unowned_environment);
    atomic_init(&probe.contention_attempts, 0);
    atomic_init(&probe.contention_result, -1);
    for (int i = 0; i < PARALLEL_WORLD_COUNT; i++) {
        ivp_environment_set_phase_hook(parallel[i].env, overlap_hook, &probe);
    }
    if (ivp_world_scheduler_simulate_one_psi(
            scheduler, environments, PARALLEL_WORLD_COUNT) != IVP_OK) {
        ok = false;
        goto cleanup;
    }
    if (ivp_environment_get_time(probe.unowned_environment) != unowned_time) {
        ok = false;
        goto cleanup;
    }
    for (int i = 0; i < PARALLEL_WORLD_COUNT; i++) {
        ivp_environment_simulate_one_psi(serial[i].env);
        ivp_environment_set_phase_hook(parallel[i].env, NULL, NULL);
    }
    if (atomic_load_explicit(&probe.callbacks, memory_order_relaxed) !=
            PARALLEL_WORLD_COUNT ||
        !atomic_load_explicit(&probe.overlapped, memory_order_acquire) ||
        atomic_load_explicit(&probe.max_active, memory_order_relaxed) < 2 ||
        atomic_load_explicit(&probe.contention_result,
                             memory_order_acquire) != IVP_ERR_INVALID_STATE) {
        ok = false;
        goto cleanup;
    }

    for (int step = 1; step < 120; step++) {
        if (ivp_world_scheduler_simulate_one_psi(
                scheduler, environments, PARALLEL_WORLD_COUNT) != IVP_OK) {
            ok = false;
            goto cleanup;
        }
        for (int i = 0; i < PARALLEL_WORLD_COUNT; i++) {
            ivp_environment_simulate_one_psi(serial[i].env);
        }
    }
    if (ivp_world_scheduler_simulate_dtime(
            scheduler, environments, PARALLEL_WORLD_COUNT,
            IVP_REAL(0.05)) != IVP_OK) {
        ok = false;
        goto cleanup;
    }
    for (int i = 0; i < PARALLEL_WORLD_COUNT; i++) {
        ivp_environment_simulate_dtime(serial[i].env, IVP_REAL(0.05));
        if (!world_state_equal(&parallel[i], &serial[i])) {
            ok = false;
            goto cleanup;
        }
    }

    {
        ivp_environment_t *duplicates[2] = {
            parallel[0].env, parallel[0].env
        };
        const ivp_time_t time_before_rejection =
            ivp_environment_get_time(parallel[0].env);
        if (ivp_world_scheduler_simulate_one_psi(
                scheduler, duplicates, 2) != IVP_ERR_INVALID_ARG ||
            ivp_environment_get_time(parallel[0].env) !=
                time_before_rejection) {
            ok = false;
        }
    }

cleanup:
    ivp_world_scheduler_destroy(negative_scheduler);
    ivp_world_scheduler_destroy(invalid_scheduler);
    ivp_world_scheduler_destroy(contender);
    ivp_world_scheduler_destroy(scheduler);
    for (int i = 0; i < PARALLEL_WORLD_COUNT; i++) {
        test_world_destroy(&parallel[i]);
        test_world_destroy(&serial[i]);
    }
    IVP_ASSERT_TRUE(ok);
}

int main(void)
{
    IVP_REGISTER(scheduler_cleans_partial_thread_start);
    IVP_REGISTER(scheduler_setup_failures_are_atomic);
    IVP_REGISTER(scheduler_allocation_failure_is_atomic);
    IVP_REGISTER(scheduler_one_worker_is_fully_serial);
    IVP_REGISTER(scheduler_propagates_dispatch_fenv);
    IVP_REGISTER(scheduler_parallel_execution);
    IVP_RUN_ALL();
}
