#ifndef IVP_TEST_FAULT_ALLOCATOR_H
#define IVP_TEST_FAULT_ALLOCATOR_H

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <ivp/base/ivp_allocator.h>

typedef struct ivp_test_fault_allocator {
    ivp_allocator_t allocator;
    ivp_allocator_t inner;
    int attempts;
    int fail_at;
    bool track_live_bytes;
    size_t live_bytes;
} ivp_test_fault_allocator_t;

static inline void *ivp_test_fault_alloc(size_t size, void *ctx)
{
    ivp_test_fault_allocator_t *state =
        (ivp_test_fault_allocator_t *)ctx;
    state->attempts++;
    if (state->attempts == state->fail_at) return NULL;
    void *ptr = ivp_alloc(&state->inner, size);
    if (ptr && state->track_live_bytes) state->live_bytes += size;
    return ptr;
}

static inline void *ivp_test_fault_realloc(void *ptr, size_t old_size,
                                           size_t new_size, void *ctx)
{
    ivp_test_fault_allocator_t *state =
        (ivp_test_fault_allocator_t *)ctx;
    state->attempts++;
    if (state->attempts == state->fail_at) return NULL;
    void *new_ptr = ivp_realloc(&state->inner, ptr, old_size, new_size);
    if (new_ptr && state->track_live_bytes) {
        state->live_bytes = state->live_bytes - old_size + new_size;
    }
    return new_ptr;
}

static inline void ivp_test_fault_free(void *ptr, size_t size, void *ctx)
{
    ivp_test_fault_allocator_t *state =
        (ivp_test_fault_allocator_t *)ctx;
    if (!ptr) return;
    ivp_free(&state->inner, ptr, size);
    if (state->track_live_bytes) state->live_bytes -= size;
}

static inline void ivp_test_fault_allocator_init(
    ivp_test_fault_allocator_t *state, const ivp_allocator_t *inner)
{
    memset(state, 0, sizeof(*state));
    state->inner = inner ? *inner : *ivp_default_allocator();
    state->fail_at = INT_MAX;
    state->allocator = (ivp_allocator_t){
        .alloc = ivp_test_fault_alloc,
        .realloc = ivp_test_fault_realloc,
        .free = ivp_test_fault_free,
        .ctx = state
    };
}

static inline void ivp_test_fault_allocator_init_tracked(
    ivp_test_fault_allocator_t *state, const ivp_allocator_t *inner)
{
    ivp_test_fault_allocator_init(state, inner);
    state->track_live_bytes = true;
}

static inline void ivp_test_fault_allocator_reset(
    ivp_test_fault_allocator_t *state, int fail_at)
{
    state->attempts = 0;
    state->fail_at = fail_at;
}

#endif
