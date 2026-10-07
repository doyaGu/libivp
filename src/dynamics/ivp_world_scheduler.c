/* ivp_world_scheduler.c -- parallel outer loop over independent environments. */
#include <ivp/dynamics/ivp_world_scheduler.h>

#include "dynamics/ivp_environment_parallel.h"

#include <fenv.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#if defined(_WIN32)
#include <process.h>
#include <windows.h>
typedef HANDLE ivp_worker_thread_t;
typedef CRITICAL_SECTION ivp_worker_mutex_t;
typedef CONDITION_VARIABLE ivp_worker_cond_t;
#else
#include <pthread.h>
typedef pthread_t ivp_worker_thread_t;
typedef pthread_mutex_t ivp_worker_mutex_t;
typedef pthread_cond_t ivp_worker_cond_t;
#endif

typedef enum ivp_world_job_kind {
    IVP_WORLD_JOB_ONE_PSI,
    IVP_WORLD_JOB_DTIME
} ivp_world_job_kind_t;

struct ivp_world_scheduler {
    ivp_allocator_t allocator;
    ivp_worker_thread_t *threads;
    int worker_count;

    ivp_worker_mutex_t mutex;
    ivp_worker_cond_t work_ready;
    ivp_worker_cond_t work_done;
    bool sync_initialized;
    bool stopping;
    bool running;
    uint64_t generation;

    ivp_environment_t *const *environments;
    size_t environment_count;
    size_t next_environment;
    int workers_ready;
    int workers_done;
    bool worker_setup_failed;
    fenv_t floating_point_environment;
    ivp_world_job_kind_t job_kind;
    ivp_real_t elapsed_time;

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

#if defined(_WIN32)
static bool sync_init(ivp_world_scheduler_t *s)
{
    InitializeCriticalSection(&s->mutex);
    InitializeConditionVariable(&s->work_ready);
    InitializeConditionVariable(&s->work_done);
    return true;
}
static void sync_destroy(ivp_world_scheduler_t *s) { DeleteCriticalSection(&s->mutex); }
static void sync_lock(ivp_world_scheduler_t *s) { EnterCriticalSection(&s->mutex); }
static void sync_unlock(ivp_world_scheduler_t *s) { LeaveCriticalSection(&s->mutex); }
static void sync_wait(ivp_worker_cond_t *cond, ivp_world_scheduler_t *s)
{
    (void)SleepConditionVariableCS(cond, &s->mutex, INFINITE);
}
static void sync_broadcast(ivp_worker_cond_t *cond) { WakeAllConditionVariable(cond); }
#else
static bool sync_init(ivp_world_scheduler_t *s)
{
    if (pthread_mutex_init(&s->mutex, NULL) != 0) return false;
    if (pthread_cond_init(&s->work_ready, NULL) != 0) {
        (void)pthread_mutex_destroy(&s->mutex);
        return false;
    }
    if (pthread_cond_init(&s->work_done, NULL) != 0) {
        (void)pthread_cond_destroy(&s->work_ready);
        (void)pthread_mutex_destroy(&s->mutex);
        return false;
    }
    return true;
}
static void sync_destroy(ivp_world_scheduler_t *s)
{
    (void)pthread_cond_destroy(&s->work_done);
    (void)pthread_cond_destroy(&s->work_ready);
    (void)pthread_mutex_destroy(&s->mutex);
}
static void sync_lock(ivp_world_scheduler_t *s) { (void)pthread_mutex_lock(&s->mutex); }
static void sync_unlock(ivp_world_scheduler_t *s) { (void)pthread_mutex_unlock(&s->mutex); }
static void sync_wait(ivp_worker_cond_t *cond, ivp_world_scheduler_t *s)
{
    (void)pthread_cond_wait(cond, &s->mutex);
}
static void sync_broadcast(ivp_worker_cond_t *cond) { (void)pthread_cond_broadcast(cond); }
#endif

static void scheduler_worker_loop(ivp_world_scheduler_t *s)
{
    uint64_t observed_generation = 0;
    sync_lock(s);
    for (;;) {
        while (!s->stopping && observed_generation == s->generation) {
            sync_wait(&s->work_ready, s);
        }
        if (s->stopping) break;
        observed_generation = s->generation;

        const fenv_t floating_point_environment =
            s->floating_point_environment;
        sync_unlock(s);
        bool setup_ok = fesetenv(&floating_point_environment) == 0;
        sync_lock(s);
        if (!setup_ok) s->worker_setup_failed = true;
        s->workers_ready++;
        if (s->workers_ready == s->worker_count) {
            sync_broadcast(&s->work_done);
        } else {
            while (!s->stopping &&
                   s->workers_ready != s->worker_count) {
                sync_wait(&s->work_done, s);
            }
        }
        if (s->stopping) break;

        if (!s->worker_setup_failed) {
            for (;;) {
                if (s->next_environment >= s->environment_count) break;
                size_t index = s->next_environment++;
                ivp_environment_t *env = s->environments[index];
                ivp_world_job_kind_t kind = s->job_kind;
                ivp_real_t elapsed_time = s->elapsed_time;
                sync_unlock(s);

                scheduler_run_environment(kind, env, elapsed_time);

                sync_lock(s);
            }
        }

        s->workers_done++;
        if (s->workers_done == s->worker_count) {
            s->running = false;
            sync_broadcast(&s->work_done);
        }
    }
    sync_unlock(s);
}

#if defined(_WIN32)
static unsigned __stdcall scheduler_worker_main(void *opaque)
{
    scheduler_worker_loop((ivp_world_scheduler_t *)opaque);
    return 0;
}
#else
static void *scheduler_worker_main(void *opaque)
{
    scheduler_worker_loop((ivp_world_scheduler_t *)opaque);
    return NULL;
}
#endif

static void scheduler_join_threads(ivp_world_scheduler_t *s, int count)
{
    for (int i = 0; i < count; i++) {
#if defined(_WIN32)
        (void)WaitForSingleObject(s->threads[i], INFINITE);
        (void)CloseHandle(s->threads[i]);
#else
        (void)pthread_join(s->threads[i], NULL);
#endif
    }
}

ivp_world_scheduler_t *ivp_world_scheduler_create(
    int worker_count, const ivp_allocator_t *allocator)
{
    if (worker_count <= 0) return NULL;
    if ((size_t)worker_count > SIZE_MAX / sizeof(ivp_worker_thread_t)) {
        return NULL;
    }
    if (!allocator) allocator = ivp_default_allocator();

    ivp_world_scheduler_t *s =
        (ivp_world_scheduler_t *)ivp_alloc(allocator, sizeof(*s));
    if (!s) return NULL;
    memset(s, 0, sizeof(*s));
    s->allocator = *allocator;
    s->worker_count = worker_count;
    if (worker_count > 1) {
        s->threads = (ivp_worker_thread_t *)ivp_alloc(
            &s->allocator, (size_t)worker_count * sizeof(*s->threads));
        if (!s->threads) {
            ivp_free(&s->allocator, s, sizeof(*s));
            return NULL;
        }
    }
    if (!sync_init(s)) {
        if (s->threads) {
            ivp_free(&s->allocator, s->threads,
                     (size_t)worker_count * sizeof(*s->threads));
        }
        ivp_free(&s->allocator, s, sizeof(*s));
        return NULL;
    }
    s->sync_initialized = true;

    /* One is the complete serial fallback: retain validation and ownership
     * checks, but create no OS thread and execute dispatches on the caller. */
    if (worker_count == 1) return s;

    int created = 0;
    for (; created < worker_count; created++) {
#if defined(_WIN32)
        uintptr_t thread = _beginthreadex(NULL, 0, scheduler_worker_main,
                                         s, 0, NULL);
        s->threads[created] = (HANDLE)thread;
        if (!s->threads[created]) break;
#else
        if (pthread_create(&s->threads[created], NULL,
                           scheduler_worker_main, s) != 0) break;
#endif
    }
    if (created != worker_count) {
        sync_lock(s);
        s->stopping = true;
        sync_broadcast(&s->work_ready);
        sync_unlock(s);
        scheduler_join_threads(s, created);
        sync_destroy(s);
        ivp_free(&s->allocator, s->threads,
                 (size_t)worker_count * sizeof(*s->threads));
        ivp_free(&s->allocator, s, sizeof(*s));
        return NULL;
    }
    return s;
}

void ivp_world_scheduler_destroy(ivp_world_scheduler_t *s)
{
    if (!s) return;
    sync_lock(s);
    s->stopping = true;
    sync_broadcast(&s->work_ready);
    sync_unlock(s);
    if (s->worker_count > 1) {
        scheduler_join_threads(s, s->worker_count);
    }
    if (s->sync_initialized) sync_destroy(s);
    if (s->seen_environments) {
        ivp_free(&s->allocator, s->seen_environments,
                 s->seen_capacity * sizeof(*s->seen_environments));
    }
    if (s->threads) {
        ivp_free(&s->allocator, s->threads,
                 (size_t)s->worker_count * sizeof(*s->threads));
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

    sync_lock(s);
    if (s->running || s->stopping) {
        sync_unlock(s);
        return IVP_ERR_INVALID_STATE;
    }
    ivp_result_t result = scheduler_validate_environments(
        s, environments, environment_count);
    if (result != IVP_OK || environment_count == 0) {
        sync_unlock(s);
        return result;
    }
    if (s->worker_count > 1 &&
        fegetenv(&s->floating_point_environment) != 0) {
        sync_unlock(s);
        return IVP_ERR_INVALID_STATE;
    }

    size_t acquired = 0;
    for (; acquired < environment_count; acquired++) {
        if (!ivp_environment_try_begin_simulation(environments[acquired])) break;
    }
    if (acquired != environment_count) {
        for (size_t i = 0; i < acquired; i++) {
            ivp_environment_end_simulation(environments[i]);
        }
        sync_unlock(s);
        return IVP_ERR_INVALID_STATE;
    }

    s->environments = environments;
    s->environment_count = environment_count;
    s->next_environment = 0;
    s->workers_ready = 0;
    s->workers_done = 0;
    s->worker_setup_failed = false;
    s->job_kind = kind;
    s->elapsed_time = elapsed_time;
    s->running = true;
    if (s->worker_count == 1) {
        /* Release the scheduler lock around application callbacks.  A nested
         * or concurrent dispatch can then observe running and fail instead
         * of deadlocking, just as it does while worker threads are active. */
        sync_unlock(s);
        for (size_t i = 0; i < environment_count; i++) {
            scheduler_run_environment(kind, environments[i], elapsed_time);
        }
        sync_lock(s);
        s->running = false;
    } else {
        s->generation++;
        sync_broadcast(&s->work_ready);
        while (s->running) sync_wait(&s->work_done, s);
        if (s->worker_setup_failed) result = IVP_ERR_INVALID_STATE;
    }

    s->environments = NULL;
    s->environment_count = 0;
    for (size_t i = 0; i < acquired; i++) {
        ivp_environment_end_simulation(environments[i]);
    }
    sync_unlock(s);
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
