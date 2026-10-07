#ifndef IVP_TEST_WORKER_RUNTIME_H
#define IVP_TEST_WORKER_RUNTIME_H

#include "base/ivp_worker_runtime.h"

#include <limits.h>
#include <stdatomic.h>

typedef struct ivp_test_worker_runtime {
    ivp_worker_runtime_t runtime;
    const ivp_worker_runtime_t *inner;
    atomic_int thread_start_calls;
    atomic_int thread_join_calls;
    atomic_int floating_point_capture_calls;
    atomic_int floating_point_apply_calls;
    atomic_int fail_thread_start_at;
    atomic_int fail_floating_point_capture_at;
    atomic_int fail_floating_point_apply_at;
} ivp_test_worker_runtime_t;

static inline bool ivp_test_worker_thread_start(
    void *context, ivp_worker_thread_t *thread, ivp_worker_entry_fn entry,
    void *entry_context)
{
    ivp_test_worker_runtime_t *state =
        (ivp_test_worker_runtime_t *)context;
    int call = atomic_fetch_add_explicit(
                   &state->thread_start_calls, 1, memory_order_relaxed) + 1;
    if (call == atomic_load_explicit(&state->fail_thread_start_at,
                                     memory_order_relaxed)) {
        return false;
    }
    return state->inner->thread_start(
        state->inner->context, thread, entry, entry_context);
}

static inline void ivp_test_worker_thread_join(
    void *context, ivp_worker_thread_t *thread)
{
    ivp_test_worker_runtime_t *state =
        (ivp_test_worker_runtime_t *)context;
    atomic_fetch_add_explicit(&state->thread_join_calls, 1,
                              memory_order_relaxed);
    state->inner->thread_join(state->inner->context, thread);
}

static inline bool ivp_test_worker_floating_point_capture(
    void *context, fenv_t *environment_out)
{
    ivp_test_worker_runtime_t *state =
        (ivp_test_worker_runtime_t *)context;
    int call = atomic_fetch_add_explicit(
                   &state->floating_point_capture_calls, 1,
                   memory_order_relaxed) + 1;
    if (call == atomic_load_explicit(
                    &state->fail_floating_point_capture_at,
                    memory_order_relaxed)) {
        return false;
    }
    return state->inner->floating_point_capture(
        state->inner->context, environment_out);
}

static inline bool ivp_test_worker_floating_point_apply(
    void *context, const fenv_t *environment)
{
    ivp_test_worker_runtime_t *state =
        (ivp_test_worker_runtime_t *)context;
    int call = atomic_fetch_add_explicit(
                   &state->floating_point_apply_calls, 1,
                   memory_order_relaxed) + 1;
    if (call == atomic_load_explicit(&state->fail_floating_point_apply_at,
                                     memory_order_relaxed)) {
        return false;
    }
    return state->inner->floating_point_apply(
        state->inner->context, environment);
}

static inline void ivp_test_worker_runtime_init(
    ivp_test_worker_runtime_t *state,
    const ivp_worker_runtime_t *inner)
{
    state->inner = inner ? inner : ivp_worker_runtime_default();
    atomic_init(&state->thread_start_calls, 0);
    atomic_init(&state->thread_join_calls, 0);
    atomic_init(&state->floating_point_capture_calls, 0);
    atomic_init(&state->floating_point_apply_calls, 0);
    atomic_init(&state->fail_thread_start_at, INT_MAX);
    atomic_init(&state->fail_floating_point_capture_at, INT_MAX);
    atomic_init(&state->fail_floating_point_apply_at, INT_MAX);
    state->runtime = (ivp_worker_runtime_t){
        .context = state,
        .thread_start = ivp_test_worker_thread_start,
        .thread_join = ivp_test_worker_thread_join,
        .floating_point_capture = ivp_test_worker_floating_point_capture,
        .floating_point_apply = ivp_test_worker_floating_point_apply
    };
}

static inline void ivp_test_worker_runtime_fail_next_capture(
    ivp_test_worker_runtime_t *state)
{
    int next = atomic_load_explicit(&state->floating_point_capture_calls,
                                    memory_order_relaxed) + 1;
    atomic_store_explicit(&state->fail_floating_point_capture_at, next,
                          memory_order_relaxed);
}

static inline void ivp_test_worker_runtime_fail_next_apply(
    ivp_test_worker_runtime_t *state)
{
    int next = atomic_load_explicit(&state->floating_point_apply_calls,
                                    memory_order_relaxed) + 1;
    atomic_store_explicit(&state->fail_floating_point_apply_at, next,
                          memory_order_relaxed);
}

#endif /* IVP_TEST_WORKER_RUNTIME_H */
