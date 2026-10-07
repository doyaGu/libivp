/* ivp_env_parallel_tasks.h -- environment adapter for pure indexed kernels. */
#ifndef IVP_ENV_PARALLEL_TASKS_H
#define IVP_ENV_PARALLEL_TASKS_H

#include <ivp/dynamics/ivp_environment.h>

#include <stddef.h>

typedef enum ivp_parallel_work_kind {
    IVP_PARALLEL_WORK_BROADPHASE,
    IVP_PARALLEL_WORK_SOLVER
} ivp_parallel_work_kind_t;

typedef void (*ivp_env_parallel_task_fn)(void *context, size_t task_index);

int ivp_env_parallel_worker_count(const ivp_environment_t *env);

/* Returns false without running tasks when the environment has no pool or
 * the work is below min_tasks.  Returns true after all tasks have completed. */
bool ivp_env_parallel_for(ivp_environment_t *env,
                          ivp_parallel_work_kind_t kind,
                          size_t task_count,
                          size_t min_tasks,
                          ivp_env_parallel_task_fn function,
                          void *context);

#endif /* IVP_ENV_PARALLEL_TASKS_H */
