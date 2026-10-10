/* ivp_world_scheduler.c -- parallel outer loop over independent environments. */
#include <ivp/dynamics/ivp_world_scheduler.h>

#include "base/ivp_worker_group.h"
#include "dynamics/ivp_environment_parallel.h"
#include "dynamics/ivp_world_scheduler_internal.h"

#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

typedef enum ivp_world_job_kind {
    IVP_WORLD_JOB_ONE_PSI,
    IVP_WORLD_JOB_DTIME
} ivp_world_job_kind_t;

struct ivp_world_scheduler {
    ivp_allocator_t allocator;
    ivp_worker_group_t *workers;
    atomic_bool dispatching;

    /* Reused open-addressing table for duplicate detection. */
    ivp_environment_t **seen_environments;
    size_t seen_capacity;
};

static void scheduler_run_environment(ivp_world_job_kind_t kind,
                                      ivp_environment_t *environment,
                                      ivp_real_t elapsed_time)
{
    if (kind == IVP_WORLD_JOB_ONE_PSI) {
        ivp_environment_simulate_one_psi_owned(environment);
    } else {
        ivp_environment_simulate_dtime_owned(environment, elapsed_time);
    }
}

typedef struct ivp_world_job {
    ivp_environment_t *const *environments;
    size_t environment_count;
    atomic_size_t next_environment;
    ivp_world_job_kind_t kind;
    ivp_real_t elapsed_time;
} ivp_world_job_t;

static void scheduler_worker_run(void *opaque, int lane_index)
{
    (void)lane_index;
    ivp_world_job_t *job = (ivp_world_job_t *)opaque;
    for (;;) {
        size_t index = atomic_fetch_add_explicit(
            &job->next_environment, 1u, memory_order_relaxed);
        if (index >= job->environment_count) break;
        scheduler_run_environment(job->kind, job->environments[index],
                                  job->elapsed_time);
    }
}

ivp_world_scheduler_t *ivp_world_scheduler_create(
    int worker_count, const ivp_allocator_t *allocator)
{
    return ivp_world_scheduler_create_with_runtime(
        worker_count, allocator, ivp_worker_runtime_default());
}

ivp_world_scheduler_t *ivp_world_scheduler_create_with_runtime(
    int worker_count, const ivp_allocator_t *allocator,
    const ivp_worker_runtime_t *runtime)
{
    if (worker_count <= 0 || !ivp_worker_runtime_is_valid(runtime)) {
        return NULL;
    }
    if (!allocator) allocator = ivp_default_allocator();

    ivp_world_scheduler_t *s =
        (ivp_world_scheduler_t *)ivp_alloc(allocator, sizeof(*s));
    if (!s) return NULL;
    memset(s, 0, sizeof(*s));
    s->allocator = *allocator;
    atomic_init(&s->dispatching, false);
    s->workers = ivp_worker_group_create(
        worker_count == 1 ? 0 : worker_count, &s->allocator, runtime);
    if (!s->workers) {
        ivp_free(&s->allocator, s, sizeof(*s));
        return NULL;
    }
    return s;
}

void ivp_world_scheduler_destroy(ivp_world_scheduler_t *s)
{
    if (!s) return;
    ivp_worker_group_destroy(s->workers);
    if (s->seen_environments) {
        ivp_free(&s->allocator, s->seen_environments,
                 s->seen_capacity * sizeof(*s->seen_environments));
    }
    ivp_free(&s->allocator, s, sizeof(*s));
}

static size_t environment_hash(const ivp_environment_t *env)
{
    uintptr_t x = (uintptr_t)env >> 4;
#if UINTPTR_MAX > UINT32_MAX
    x ^= x >> 33;
    x *= UINT64_C(0xff51afd7ed558ccd);
    x ^= x >> 33;
#else
    x ^= x >> 16;
    x *= UINT32_C(0x7feb352d);
    x ^= x >> 15;
#endif
    return (size_t)x;
}

static ivp_result_t scheduler_validate_environments(
    ivp_world_scheduler_t *s,
    ivp_environment_t *const *environments,
    size_t environment_count)
{
    if (environment_count == 0) return IVP_OK;
    if (!environments || environment_count > SIZE_MAX / 2u) {
        return IVP_ERR_INVALID_ARG;
    }

    size_t needed = 8u;
    while (needed < environment_count * 2u) {
        if (needed > SIZE_MAX / 2u) return IVP_ERR_CAPACITY_EXCEEDED;
        needed *= 2u;
    }
    if (needed > SIZE_MAX / sizeof(*s->seen_environments)) {
        return IVP_ERR_CAPACITY_EXCEEDED;
    }
    if (needed > s->seen_capacity) {
        ivp_environment_t **new_seen = (ivp_environment_t **)ivp_realloc(
            &s->allocator, s->seen_environments,
            s->seen_capacity * sizeof(*s->seen_environments),
            needed * sizeof(*s->seen_environments));
        if (!new_seen) return IVP_ERR_OUT_OF_MEMORY;
        s->seen_environments = new_seen;
        s->seen_capacity = needed;
    }
    memset(s->seen_environments, 0,
           s->seen_capacity * sizeof(*s->seen_environments));

    const size_t mask = s->seen_capacity - 1u;
    for (size_t i = 0; i < environment_count; i++) {
        ivp_environment_t *env = environments[i];
        if (!env) return IVP_ERR_INVALID_ARG;
        size_t slot = environment_hash(env) & mask;
        while (s->seen_environments[slot]) {
            if (s->seen_environments[slot] == env) return IVP_ERR_INVALID_ARG;
            slot = (slot + 1u) & mask;
        }
        s->seen_environments[slot] = env;
    }
    return IVP_OK;
}

static ivp_result_t scheduler_dispatch(
    ivp_world_scheduler_t *s,
    ivp_environment_t *const *environments,
    size_t environment_count,
    ivp_world_job_kind_t kind,
    ivp_real_t elapsed_time)
{
    if (!s) return IVP_ERR_INVALID_ARG;
    if (kind == IVP_WORLD_JOB_DTIME &&
        (!isfinite((double)elapsed_time) || elapsed_time < IVP_REAL(0.0))) {
        return IVP_ERR_INVALID_ARG;
    }

    if (atomic_exchange_explicit(&s->dispatching, true,
                                 memory_order_acquire)) {
        return IVP_ERR_INVALID_STATE;
    }
    ivp_result_t result = scheduler_validate_environments(
        s, environments, environment_count);
    if (result != IVP_OK || environment_count == 0) {
        atomic_store_explicit(&s->dispatching, false,
                              memory_order_release);
        return result;
    }

    size_t acquired = 0;
    for (; acquired < environment_count; acquired++) {
        if (!ivp_environment_try_begin_simulation(environments[acquired])) break;
    }
    if (acquired != environment_count) {
        for (size_t i = 0; i < acquired; i++) {
            ivp_environment_end_simulation(environments[i]);
        }
        atomic_store_explicit(&s->dispatching, false,
                              memory_order_release);
        return IVP_ERR_INVALID_STATE;
    }

    ivp_world_job_t job = {
        .environments = environments,
        .environment_count = environment_count,
        .kind = kind,
        .elapsed_time = elapsed_time
    };
    atomic_init(&job.next_environment, 0u);
    if (!ivp_worker_group_dispatch(
            s->workers,
            ivp_worker_group_background_thread_count(s->workers) == 0,
            scheduler_worker_run, &job)) {
        result = IVP_ERR_INVALID_STATE;
    }

    for (size_t i = 0; i < acquired; i++) {
        ivp_environment_end_simulation(environments[i]);
    }
    atomic_store_explicit(&s->dispatching, false, memory_order_release);
    return result;
}

ivp_result_t ivp_world_scheduler_simulate_one_psi(
    ivp_world_scheduler_t *scheduler,
    ivp_environment_t *const *environments,
    size_t environment_count)
{
    return scheduler_dispatch(scheduler, environments, environment_count,
                              IVP_WORLD_JOB_ONE_PSI, IVP_REAL(0.0));
}

ivp_result_t ivp_world_scheduler_simulate_dtime(
    ivp_world_scheduler_t *scheduler,
    ivp_environment_t *const *environments,
    size_t environment_count,
    ivp_real_t elapsed_time)
{
    return scheduler_dispatch(scheduler, environments, environment_count,
                              IVP_WORLD_JOB_DTIME, elapsed_time);
}
