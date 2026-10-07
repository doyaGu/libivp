/* ivp_task_pool.c -- persistent threads for deterministic indexed kernels. */
#include "base/ivp_task_pool.h"

#include <ivp/base/ivp_assert.h>

#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#if defined(_WIN32)
#include <windows.h>
typedef CRITICAL_SECTION ivp_task_mutex_t;
typedef CONDITION_VARIABLE ivp_task_cond_t;
#else
#include <pthread.h>
typedef pthread_mutex_t ivp_task_mutex_t;
typedef pthread_cond_t ivp_task_cond_t;
#endif

typedef struct ivp_task_worker {
    struct ivp_task_pool *pool;
} ivp_task_worker_t;

struct ivp_task_pool {
    ivp_allocator_t allocator;
    const ivp_worker_runtime_t *runtime;
    ivp_worker_thread_t *threads;
    ivp_task_worker_t *workers;
    int worker_count;
    int background_thread_count;

    ivp_task_mutex_t mutex;
    ivp_task_cond_t work_ready;
    ivp_task_cond_t state_changed;
    bool sync_initialized;
    bool stopping;
    bool running;
    uint64_t generation;
    int setup_ready_count;
    int job_done_count;
    bool setup_failed;

    ivp_task_pool_fn function;
    void *context;
    size_t task_count;
    atomic_size_t next_task;
    size_t task_grain;
    fenv_t floating_point_environment;
};

#if defined(_WIN32)
static bool task_sync_init(ivp_task_pool_t *pool)
{
    InitializeCriticalSection(&pool->mutex);
    InitializeConditionVariable(&pool->work_ready);
    InitializeConditionVariable(&pool->state_changed);
    return true;
}
static void task_sync_destroy(ivp_task_pool_t *pool)
{
    DeleteCriticalSection(&pool->mutex);
}
static void task_lock(ivp_task_pool_t *pool) { EnterCriticalSection(&pool->mutex); }
static void task_unlock(ivp_task_pool_t *pool) { LeaveCriticalSection(&pool->mutex); }
static void task_wait(ivp_task_cond_t *condition, ivp_task_pool_t *pool)
{
    (void)SleepConditionVariableCS(condition, &pool->mutex, INFINITE);
}
static void task_broadcast(ivp_task_cond_t *condition)
{
    WakeAllConditionVariable(condition);
}
#else
static bool task_sync_init(ivp_task_pool_t *pool)
{
    if (pthread_mutex_init(&pool->mutex, NULL) != 0) return false;
    if (pthread_cond_init(&pool->work_ready, NULL) != 0) {
        (void)pthread_mutex_destroy(&pool->mutex);
        return false;
    }
    if (pthread_cond_init(&pool->state_changed, NULL) != 0) {
        (void)pthread_cond_destroy(&pool->work_ready);
        (void)pthread_mutex_destroy(&pool->mutex);
        return false;
    }
    return true;
}
static void task_sync_destroy(ivp_task_pool_t *pool)
{
    (void)pthread_cond_destroy(&pool->state_changed);
    (void)pthread_cond_destroy(&pool->work_ready);
    (void)pthread_mutex_destroy(&pool->mutex);
}
static void task_lock(ivp_task_pool_t *pool) { (void)pthread_mutex_lock(&pool->mutex); }
static void task_unlock(ivp_task_pool_t *pool) { (void)pthread_mutex_unlock(&pool->mutex); }
static void task_wait(ivp_task_cond_t *condition, ivp_task_pool_t *pool)
{
    (void)pthread_cond_wait(condition, &pool->mutex);
}
static void task_broadcast(ivp_task_cond_t *condition)
{
    (void)pthread_cond_broadcast(condition);
}
#endif

static bool task_claim_range(ivp_task_pool_t *pool, size_t task_count,
                             size_t task_grain, size_t *first_out,
                             size_t *end_out)
{
    size_t first = atomic_load_explicit(&pool->next_task,
                                        memory_order_relaxed);
    while (first < task_count) {
        size_t remaining = task_count - first;
        size_t count = remaining < task_grain ? remaining : task_grain;
        size_t end = first + count;
        if (atomic_compare_exchange_weak_explicit(
                &pool->next_task, &first, end, memory_order_relaxed,
                memory_order_relaxed)) {
            *first_out = first;
            *end_out = end;
            return true;
        }
    }
    return false;
}

static void task_run_claimed(ivp_task_pool_t *pool,
                             ivp_task_pool_fn function, void *context,
                             size_t task_count, size_t task_grain)
{
    size_t first, end;
    while (task_claim_range(pool, task_count, task_grain, &first, &end)) {
        for (size_t index = first; index < end; index++) {
            function(context, index);
        }
    }
}

static void task_worker_loop(ivp_task_worker_t *worker)
{
    ivp_task_pool_t *pool = worker->pool;
    uint64_t observed_generation = 0;
    task_lock(pool);
    for (;;) {
        while (!pool->stopping && observed_generation == pool->generation) {
            task_wait(&pool->work_ready, pool);
        }
        if (pool->stopping) break;
        observed_generation = pool->generation;

        ivp_task_pool_fn function = pool->function;
        void *context = pool->context;
        const size_t task_count = pool->task_count;
        const size_t task_grain = pool->task_grain;
        const fenv_t floating_point_environment =
            pool->floating_point_environment;
        task_unlock(pool);

        bool setup_ok = pool->runtime->floating_point_apply(
            pool->runtime->context, &floating_point_environment);
        task_lock(pool);
        if (!setup_ok) pool->setup_failed = true;
        pool->setup_ready_count++;
        if (pool->setup_ready_count == pool->background_thread_count) {
            task_broadcast(&pool->state_changed);
        } else {
            while (!pool->stopping &&
                   pool->setup_ready_count != pool->background_thread_count) {
                task_wait(&pool->state_changed, pool);
            }
        }
        if (pool->stopping) break;
        bool run_tasks = !pool->setup_failed;
        task_unlock(pool);

        if (run_tasks) {
            task_run_claimed(pool, function, context, task_count, task_grain);
        }

        task_lock(pool);
        pool->job_done_count++;
        if (pool->job_done_count == pool->background_thread_count) {
            task_broadcast(&pool->state_changed);
        }
    }
    task_unlock(pool);
}

static void task_worker_main(void *opaque)
{
    task_worker_loop((ivp_task_worker_t *)opaque);
}

static void task_join_threads(ivp_task_pool_t *pool, int count)
{
    for (int i = 0; i < count; i++) {
        pool->runtime->thread_join(pool->runtime->context,
                                   &pool->threads[i]);
    }
}

ivp_task_pool_t *ivp_task_pool_create(int worker_count,
                                      const ivp_allocator_t *allocator)
{
    return ivp_task_pool_create_with_runtime(
        worker_count, allocator, ivp_worker_runtime_default());
}

ivp_task_pool_t *ivp_task_pool_create_with_runtime(
    int worker_count, const ivp_allocator_t *allocator,
    const ivp_worker_runtime_t *runtime)
{
    if (worker_count <= 0 || !ivp_worker_runtime_is_valid(runtime)) {
        return NULL;
    }
    if ((size_t)worker_count > SIZE_MAX / sizeof(ivp_worker_thread_t) ||
        (size_t)worker_count > SIZE_MAX / sizeof(ivp_task_worker_t)) {
        return NULL;
    }
    if (!allocator) allocator = ivp_default_allocator();
    ivp_task_pool_t *pool =
        (ivp_task_pool_t *)ivp_alloc(allocator, sizeof(*pool));
    if (!pool) return NULL;
    memset(pool, 0, sizeof(*pool));
    atomic_init(&pool->next_task, 0u);
    pool->allocator = *allocator;
    pool->runtime = runtime;
    pool->worker_count = worker_count;
    pool->background_thread_count = worker_count - 1;
    pool->threads = (ivp_worker_thread_t *)ivp_alloc(
        &pool->allocator, (size_t)worker_count * sizeof(*pool->threads));
    pool->workers = (ivp_task_worker_t *)ivp_alloc(
        &pool->allocator, (size_t)worker_count * sizeof(*pool->workers));
    if (!pool->threads || !pool->workers || !task_sync_init(pool)) {
        if (pool->threads) {
            ivp_free(&pool->allocator, pool->threads,
                     (size_t)worker_count * sizeof(*pool->threads));
        }
        if (pool->workers) {
            ivp_free(&pool->allocator, pool->workers,
                     (size_t)worker_count * sizeof(*pool->workers));
        }
        ivp_free(&pool->allocator, pool, sizeof(*pool));
        return NULL;
    }
    pool->sync_initialized = true;

    int created = 0;
    for (; created < pool->background_thread_count; created++) {
        pool->workers[created].pool = pool;
        if (!pool->runtime->thread_start(
                pool->runtime->context, &pool->threads[created],
                task_worker_main, &pool->workers[created])) {
            break;
        }
    }
    if (created != pool->background_thread_count) {
        task_lock(pool);
        pool->stopping = true;
        task_broadcast(&pool->work_ready);
        task_unlock(pool);
        task_join_threads(pool, created);
        task_sync_destroy(pool);
        ivp_free(&pool->allocator, pool->workers,
                 (size_t)worker_count * sizeof(*pool->workers));
        ivp_free(&pool->allocator, pool->threads,
                 (size_t)worker_count * sizeof(*pool->threads));
        ivp_free(&pool->allocator, pool, sizeof(*pool));
        return NULL;
    }
    return pool;
}

void ivp_task_pool_destroy(ivp_task_pool_t *pool)
{
    if (!pool) return;
    task_lock(pool);
    IVP_ASSERT(!pool->running);
    while (pool->running) task_wait(&pool->state_changed, pool);
    pool->stopping = true;
    task_broadcast(&pool->work_ready);
    task_unlock(pool);
    task_join_threads(pool, pool->background_thread_count);
    if (pool->sync_initialized) task_sync_destroy(pool);
    ivp_free(&pool->allocator, pool->workers,
             (size_t)pool->worker_count * sizeof(*pool->workers));
    ivp_free(&pool->allocator, pool->threads,
             (size_t)pool->worker_count * sizeof(*pool->threads));
    ivp_free(&pool->allocator, pool, sizeof(*pool));
}

int ivp_task_pool_worker_count(const ivp_task_pool_t *pool)
{
    return pool ? pool->worker_count : 0;
}

bool ivp_task_pool_parallel_for(ivp_task_pool_t *pool, size_t task_count,
                                ivp_task_pool_fn function, void *context)
{
    if (!pool || task_count == 0 || !function) return false;
    task_lock(pool);
    IVP_ASSERT(!pool->running && !pool->stopping);
    if (!pool->runtime->floating_point_capture(
            pool->runtime->context, &pool->floating_point_environment)) {
        task_unlock(pool);
        return false;
    }
    pool->function = function;
    pool->context = context;
    pool->task_count = task_count;
    size_t lane_count = (size_t)pool->worker_count;
    size_t target_chunks = lane_count <= SIZE_MAX / 8u
                               ? lane_count * 8u
                               : SIZE_MAX;
    pool->task_grain = task_count / target_chunks;
    if (pool->task_grain == 0u) pool->task_grain = 1u;
    atomic_store_explicit(&pool->next_task, 0u, memory_order_relaxed);
    pool->setup_ready_count = 0;
    pool->job_done_count = 0;
    pool->setup_failed = false;
    pool->running = true;
    pool->generation++;
    task_broadcast(&pool->work_ready);
    ivp_task_pool_fn caller_function = pool->function;
    void *caller_context = pool->context;
    const size_t caller_task_count = pool->task_count;
    const size_t caller_task_grain = pool->task_grain;
    /* The caller lane does not start until every background lane has adopted
     * this dispatch's floating-point environment. */
    while (pool->setup_ready_count != pool->background_thread_count) {
        task_wait(&pool->state_changed, pool);
    }
    bool setup_ok = !pool->setup_failed;
    task_unlock(pool);

    if (setup_ok) {
        task_run_claimed(pool, caller_function, caller_context,
                         caller_task_count, caller_task_grain);
    }

    task_lock(pool);
    while (pool->job_done_count != pool->background_thread_count) {
        task_wait(&pool->state_changed, pool);
    }
    pool->running = false;
    task_broadcast(&pool->state_changed);
    pool->function = NULL;
    pool->context = NULL;
    pool->task_count = 0;
    task_unlock(pool);
    return setup_ok;
}
