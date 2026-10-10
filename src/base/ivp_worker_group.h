/* ivp_worker_group.h -- private persistent worker execution group. */
#ifndef IVP_WORKER_GROUP_H
#define IVP_WORKER_GROUP_H

#include <ivp/base/ivp_allocator.h>

#include "base/ivp_worker_runtime.h"

#include <stdbool.h>

typedef struct ivp_worker_group ivp_worker_group_t;
typedef void (*ivp_worker_group_fn)(void *context, int lane_index);

/* background_thread_count excludes the caller.  A zero-thread group remains
 * useful for serial fallback because dispatch still guards re-entry. */
ivp_worker_group_t *ivp_worker_group_create(
    int background_thread_count, const ivp_allocator_t *allocator,
    const ivp_worker_runtime_t *runtime);
void ivp_worker_group_destroy(ivp_worker_group_t *group);
int ivp_worker_group_background_thread_count(const ivp_worker_group_t *group);

/* Run one generation and block until every background lane is complete.
 * When caller_participates is true, the caller runs one additional lane after
 * every background thread has adopted the caller's floating-point state.
 * Returns false without invoking fn if setup fails or the group is busy. */
bool ivp_worker_group_dispatch(ivp_worker_group_t *group,
                               bool caller_participates,
                               ivp_worker_group_fn fn, void *context);

#endif /* IVP_WORKER_GROUP_H */
