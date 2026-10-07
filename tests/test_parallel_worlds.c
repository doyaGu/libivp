/* test_parallel_worlds.c -- true parallel stepping of independent worlds. */
#include "ivp_test.h"

#include <ivp/dynamics/ivp_world_scheduler.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/utility/ivp_active_value.h>

#include <math.h>
#include <stdatomic.h>
#include <string.h>

#if defined(_WIN32)
#include <windows.h>
static void test_thread_yield(void) { Sleep(0); }
#else
#include <sched.h>
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
    atomic_int contention_attempts;
    atomic_int contention_result;
} overlap_probe_t;

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
        ivp_environment_t *same_world[] = { env };
        ivp_result_t result = ivp_world_scheduler_simulate_one_psi(
            probe->contender, same_world, 1);
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

IVP_TEST(scheduler_parallel_contract)
{
    test_world_t parallel[PARALLEL_WORLD_COUNT] = {0};
    test_world_t serial[PARALLEL_WORLD_COUNT] = {0};
    ivp_environment_t *environments[PARALLEL_WORLD_COUNT] = {0};
    ivp_world_scheduler_t *scheduler = NULL;
    ivp_world_scheduler_t *contender = NULL;
    ivp_world_scheduler_t *invalid_scheduler = NULL;
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
    if (invalid_scheduler != NULL ||
        ivp_world_scheduler_simulate_one_psi(scheduler, NULL, 0) != IVP_OK ||
        ivp_world_scheduler_simulate_one_psi(scheduler, NULL, 1) !=
            IVP_ERR_INVALID_ARG ||
        ivp_world_scheduler_simulate_dtime(
            scheduler, environments, PARALLEL_WORLD_COUNT,
            IVP_REAL(-1.0)) != IVP_ERR_INVALID_ARG ||
        ivp_world_scheduler_simulate_dtime(
            scheduler, environments, PARALLEL_WORLD_COUNT,
            (ivp_real_t)NAN) != IVP_ERR_INVALID_ARG) {
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
        if (ivp_world_scheduler_simulate_one_psi(
                scheduler, duplicates, 2) != IVP_ERR_INVALID_ARG) {
            ok = false;
        }
    }

cleanup:
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
    IVP_REGISTER(scheduler_parallel_contract);
    IVP_RUN_ALL();
}
