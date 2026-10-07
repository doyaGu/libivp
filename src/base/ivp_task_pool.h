/* ivp_task_pool.h -- private persistent worker pool for engine kernels. */
#ifndef IVP_TASK_POOL_H
#define IVP_TASK_POOL_H

#include <ivp/base/ivp_allocator.h>

#include "base/ivp_worker_runtime.h"

#include <stddef.h>

typedef struct ivp_task_pool ivp_task_pool_t;
typedef void (*ivp_task_pool_fn)(void *context, size_t task_index);

/* worker_count is the total execution width.  The calling thread owns one
 * lane; the remaining lanes are persistent background threads. */
ivp_task_pool_t *ivp_task_pool_create(int worker_count,
                                      const ivp_allocator_t *allocator);
/* Private construction seam for platform-failure tests.  runtime must remain
 * valid until the returned pool is destroyed. */
ivp_task_pool_t *ivp_task_pool_create_with_runtime(
    int worker_count, const ivp_allocator_t *allocator,
    const ivp_worker_runtime_t *runtime);
void ivp_task_pool_destroy(ivp_task_pool_t *pool);
int ivp_task_pool_worker_count(const ivp_task_pool_t *pool);

/* Run every indexed task exactly once using dynamically claimed contiguous
 * chunks.  The call blocks until every task is complete.  Calls on a pool are
 * serialized by its owner.  Returns false without running a task if the
 * caller's floating-point environment cannot be propagated to every worker. */
bool ivp_task_pool_parallel_for(ivp_task_pool_t *pool, size_t task_count,
                                ivp_task_pool_fn function, void *context);

#endif /* IVP_TASK_POOL_H */
