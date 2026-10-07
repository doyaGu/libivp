/* ivp_world_scheduler_internal.h -- private scheduler construction seam. */
#ifndef IVP_WORLD_SCHEDULER_INTERNAL_H
#define IVP_WORLD_SCHEDULER_INTERNAL_H

#include <ivp/dynamics/ivp_world_scheduler.h>

#include "base/ivp_worker_runtime.h"

/* Internal construction seam; runtime must outlive the scheduler. */
ivp_world_scheduler_t *ivp_world_scheduler_create_with_runtime(
    int worker_count, const ivp_allocator_t *allocator,
    const ivp_worker_runtime_t *runtime);

#endif /* IVP_WORLD_SCHEDULER_INTERNAL_H */
