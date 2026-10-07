/* ivp_world_scheduler.h -- persistent worker pool for independent worlds. */
#ifndef IVP_WORLD_SCHEDULER_H
#define IVP_WORLD_SCHEDULER_H

#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <ivp/dynamics/ivp_environment.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_world_scheduler ivp_world_scheduler_t;

/* Create a scheduler with the requested execution width.  A worker_count of
 * one is the complete serial fallback: it creates no OS thread and advances
 * worlds in array order on the dispatching thread.  Values greater than one
 * create that many persistent OS worker threads.  worker_count must be
 * positive.  allocator may be NULL for ivp_default_allocator(); its context
 * must outlive the scheduler.  Returns NULL on invalid input, allocation
 * failure, or thread creation failure. */
ivp_world_scheduler_t *ivp_world_scheduler_create(
    int worker_count, const ivp_allocator_t *allocator);

/* Stop and join every worker.  Do not call this concurrently with a dispatch
 * call or from a simulation callback running on this scheduler. */
void ivp_world_scheduler_destroy(ivp_world_scheduler_t *scheduler);

/* Advance each distinct environment exactly one PSI, or by the same elapsed
 * simulation time.  Calls block until all environments finish.  The input
 * array is borrowed only for the duration of the call.
 *
 * Different array entries execute in parallel when worker_count is greater
 * than one.  A one-worker scheduler visits them serially in array order on
 * the calling thread.  An environment may also use its configured intra-world
 * worker pool; set its parallel_worker_count to one for a fully serial call.
 * Budget both layers to avoid oversubscription.  NULL or duplicate
 * environments are rejected.
 * A world already being simulated elsewhere yields IVP_ERR_INVALID_STATE.
 * A zero environment_count is a successful no-op.  simulate_dtime rejects
 * negative and non-finite elapsed time.
 * With more than one worker, controller/listener callbacks execute on worker
 * threads.  Shared callback state and custom allocator/manager implementations
 * therefore need their own synchronization.  With one worker, callbacks run
 * on the dispatching thread.  Mutable manager instances, including
 * active-value managers, must not be shared between concurrently dispatched
 * worlds.  Immutable compact geometry may be shared.  A parallel world has no
 * stable worker affinity across dispatches; callbacks must not use thread-local
 * storage as world storage.  The caller must not otherwise access an
 * environment until this call returns.
 *
 * IVP_HOME_BUILD and IVP_BENCHMARK_DIAGNOSTICS retain legacy process-wide
 * instrumentation and therefore serialize simulation; a multi-world dispatch
 * in either opt-in build mode returns IVP_ERR_INVALID_STATE.
 *
 * A scheduler accepts one dispatch at a time. */
ivp_result_t ivp_world_scheduler_simulate_one_psi(
    ivp_world_scheduler_t *scheduler,
    ivp_environment_t *const *environments,
    size_t environment_count);

ivp_result_t ivp_world_scheduler_simulate_dtime(
    ivp_world_scheduler_t *scheduler,
    ivp_environment_t *const *environments,
    size_t environment_count,
    ivp_real_t elapsed_time);

#ifdef __cplusplus
}
#endif

#endif /* IVP_WORLD_SCHEDULER_H */
