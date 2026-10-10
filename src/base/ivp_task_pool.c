/* ivp_task_pool.c -- persistent threads for deterministic indexed kernels. */
#include "base/ivp_task_pool.h"

#include "base/ivp_worker_group.h"

#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

struct ivp_task_pool {
    ivp_allocator_t allocator;
    ivp_worker_group_t *workers;
    int worker_count;

    ivp_task_pool_fn function;
    void *context;
    size_t task_count;
    atomic_size_t next_task;
    size_t task_grain;
};

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

static void task_worker_run(void *opaque, int lane_index)
{
    (void)lane_index;
    ivp_task_pool_t *pool = (ivp_task_pool_t *)opaque;
    task_run_claimed(pool, pool->function, pool->context,
                     pool->task_count, pool->task_grain);
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
    if (!allocator) allocator = ivp_default_allocator();
    ivp_task_pool_t *pool =
        (ivp_task_pool_t *)ivp_alloc(allocator, sizeof(*pool));
    if (!pool) return NULL;
    memset(pool, 0, sizeof(*pool));
    atomic_init(&pool->next_task, 0u);
    pool->allocator = *allocator;
    pool->worker_count = worker_count;
    pool->workers = ivp_worker_group_create(
        worker_count - 1, &pool->allocator, runtime);
    if (!pool->workers) {
        ivp_free(&pool->allocator, pool, sizeof(*pool));
        return NULL;
    }
    return pool;
}

void ivp_task_pool_destroy(ivp_task_pool_t *pool)
{
    if (!pool) return;
    ivp_worker_group_destroy(pool->workers);
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
    bool setup_ok = ivp_worker_group_dispatch(
        pool->workers, true, task_worker_run, pool);
    pool->function = NULL;
    pool->context = NULL;
    pool->task_count = 0;
    return setup_ok;
}
