/* ivp_rewind.c -- keyframe ring and input log over memory-region
 * checkpoints (libivp extension).
 *
 * The input for the PSI that follows p PSIs is inputs[p - log_base]; the
 * log reaches log_base + input_count.  Keyframes are ascending by PSI and
 * the oldest one never lies before log_base, so every keyframe can replay
 * to any recorded PSI after it. */
#include <ivp/dynamics/ivp_rewind.h>

#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_assert.h>

#include "dynamics/ivp_env_internal.h"

#include <stdlib.h>
#include <string.h>

typedef struct rewind_input {
    void *data;
    size_t size;
} rewind_input_t;

typedef struct rewind_keyframe {
    uint64_t psi;
    ivp_memory_checkpoint_t *checkpoint;
} rewind_keyframe_t;

struct ivp_rewind {
    ivp_rewind_config_t config;
    ivp_environment_t *env;
    ivp_memory_region_t *region;
    ivp_allocator_t storage;
    uint64_t psi;
    uint64_t log_base;
    rewind_input_t *inputs;
    int input_count;
    int input_capacity;
    rewind_keyframe_t *keyframes;   /* keyframe_capacity entries */
    int keyframe_count;
};

static const ivp_allocator_t *rewind_allocator(const ivp_rewind_t *rewind)
{
    return &rewind->storage;
}

static void rewind_free_input(const ivp_rewind_t *rewind, rewind_input_t *input)
{
    if (input->data) ivp_free(rewind_allocator(rewind), input->data, input->size);
    input->data = NULL;
    input->size = 0;
}

static ivp_result_t rewind_set_result(ivp_result_t *result, ivp_result_t value)
{
    if (result) *result = value;
    return value;
}

ivp_rewind_t *ivp_rewind_create(ivp_environment_t *env,
                                ivp_memory_region_t *region,
                                const ivp_rewind_config_t *config,
                                ivp_result_t *result)
{
    if (!env || !region || !config || config->keyframe_interval < 1 ||
        config->keyframe_capacity < 1 ||
        ivp_env_get_allocator(env) != ivp_memory_region_allocator(region)) {
        rewind_set_result(result, IVP_ERR_INVALID_ARG);
        return NULL;
    }
    const ivp_allocator_t *allocator =
        config->storage ? config->storage : ivp_default_allocator();
    ivp_rewind_t *rewind = (ivp_rewind_t *)ivp_alloc(allocator, sizeof(*rewind));
    rewind_keyframe_t *keyframes = (rewind_keyframe_t *)ivp_alloc(
        allocator, sizeof(*keyframes) * (size_t)config->keyframe_capacity);
    ivp_memory_checkpoint_t *first = rewind && keyframes
        ? ivp_memory_region_checkpoint(region, allocator) : NULL;
    if (!first) {
        if (keyframes) {
            ivp_free(allocator, keyframes,
                     sizeof(*keyframes) * (size_t)config->keyframe_capacity);
        }
        if (rewind) ivp_free(allocator, rewind, sizeof(*rewind));
        rewind_set_result(result, IVP_ERR_OUT_OF_MEMORY);
        return NULL;
    }
    memset(rewind, 0, sizeof(*rewind));
    rewind->config = *config;
    rewind->env = env;
    rewind->region = region;
    rewind->storage = *allocator;
    rewind->keyframes = keyframes;
    rewind->keyframes[0].psi = 0;
    rewind->keyframes[0].checkpoint = first;
    rewind->keyframe_count = 1;
    rewind_set_result(result, IVP_OK);
    return rewind;
}

void ivp_rewind_destroy(ivp_rewind_t *rewind)
{
    if (!rewind) return;
    /* a copy: the last free releases the rewinder that holds it */
    const ivp_allocator_t storage = rewind->storage;
    const ivp_allocator_t *allocator = &storage;
    for (int i = 0; i < rewind->input_count; i++) {
        rewind_free_input(rewind, &rewind->inputs[i]);
    }
    if (rewind->inputs) {
        ivp_free(allocator, rewind->inputs,
                 sizeof(*rewind->inputs) * (size_t)rewind->input_capacity);
    }
    for (int i = 0; i < rewind->keyframe_count; i++) {
        ivp_memory_checkpoint_destroy(rewind->keyframes[i].checkpoint);
    }
    ivp_free(allocator, rewind->keyframes,
             sizeof(*rewind->keyframes) * (size_t)rewind->config.keyframe_capacity);
    ivp_free(allocator, rewind, sizeof(*rewind));
}

uint64_t ivp_rewind_recorded_psi(const ivp_rewind_t *rewind)
{
    return rewind ? rewind->log_base + (uint64_t)rewind->input_count : 0u;
}

uint64_t ivp_rewind_oldest_psi(const ivp_rewind_t *rewind)
{
    return rewind ? rewind->keyframes[0].psi : 0u;
}

uint64_t ivp_rewind_current_psi(const ivp_rewind_t *rewind)
{
    return rewind ? rewind->psi : 0u;
}

ivp_environment_t *ivp_rewind_environment(const ivp_rewind_t *rewind)
{
    return rewind ? rewind->env : NULL;
}

static void rewind_simulate(ivp_rewind_t *rewind, const rewind_input_t *input)
{
    if (rewind->config.apply) {
        rewind->config.apply(rewind->config.apply_ctx, rewind->env, rewind->psi,
                             input->size ? input->data : NULL, input->size);
    }
    ivp_environment_simulate_one_psi(rewind->env);
    rewind->psi++;
}

/* drop the recorded future after the current PSI (branching) */
static void rewind_truncate(ivp_rewind_t *rewind)
{
    int keep = (int)(rewind->psi - rewind->log_base);
    for (int i = keep; i < rewind->input_count; i++) {
        rewind_free_input(rewind, &rewind->inputs[i]);
    }
    if (keep < rewind->input_count) rewind->input_count = keep;
    while (rewind->keyframe_count > 1 &&
           rewind->keyframes[rewind->keyframe_count - 1].psi > rewind->psi) {
        ivp_memory_checkpoint_destroy(
            rewind->keyframes[--rewind->keyframe_count].checkpoint);
    }
}

/* append a keyframe, dropping the oldest one and its inputs when full */
static void rewind_add_keyframe(ivp_rewind_t *rewind,
                                ivp_memory_checkpoint_t *checkpoint)
{
    if (rewind->keyframe_count == rewind->config.keyframe_capacity) {
        ivp_memory_checkpoint_destroy(rewind->keyframes[0].checkpoint);
        memmove(rewind->keyframes, rewind->keyframes + 1,
                sizeof(*rewind->keyframes) * (size_t)(rewind->keyframe_count - 1));
        rewind->keyframe_count--;
    }
    rewind_keyframe_t *keyframe = &rewind->keyframes[rewind->keyframe_count++];
    keyframe->psi = rewind->psi;
    keyframe->checkpoint = checkpoint;

    int dropped = (int)(rewind->keyframes[0].psi - rewind->log_base);
    if (dropped > 0) {
        for (int i = 0; i < dropped; i++) rewind_free_input(rewind, &rewind->inputs[i]);
        memmove(rewind->inputs, rewind->inputs + dropped,
                sizeof(*rewind->inputs) * (size_t)(rewind->input_count - dropped));
        rewind->input_count -= dropped;
        rewind->log_base += (uint64_t)dropped;
    }
}

ivp_result_t ivp_rewind_step(ivp_rewind_t *rewind, const void *input,
                             size_t size)
{
    if (!rewind || (size && !input)) return IVP_ERR_INVALID_ARG;
    const ivp_allocator_t *allocator = rewind_allocator(rewind);

    /* everything that can fail comes before the recording changes; after a
     * seek back the keyframe of this PSI may already exist, followed by
     * the recorded future that is about to be discarded */
    bool keyframe_due =
        rewind->psi % (uint64_t)rewind->config.keyframe_interval == 0u;
    for (int i = 0; keyframe_due && i < rewind->keyframe_count; i++) {
        if (rewind->keyframes[i].psi == rewind->psi) keyframe_due = false;
    }
    ivp_memory_checkpoint_t *checkpoint = NULL;
    if (keyframe_due) {
        checkpoint = ivp_memory_region_checkpoint(rewind->region, allocator);
        if (!checkpoint) return IVP_ERR_OUT_OF_MEMORY;
    }
    rewind_input_t copy = { NULL, size };
    if (size) {
        copy.data = ivp_alloc(allocator, size);
        if (!copy.data) {
            ivp_memory_checkpoint_destroy(checkpoint);
            return IVP_ERR_OUT_OF_MEMORY;
        }
        memcpy(copy.data, input, size);
    }
    int needed = (int)(rewind->psi - rewind->log_base) + 1;
    if (needed > rewind->input_capacity) {
        int capacity = rewind->input_capacity ? rewind->input_capacity * 2 : 64;
        if (capacity < needed) capacity = needed;
        rewind_input_t *inputs = rewind->inputs
            ? (rewind_input_t *)ivp_realloc(
                  allocator, rewind->inputs,
                  sizeof(*inputs) * (size_t)rewind->input_capacity,
                  sizeof(*inputs) * (size_t)capacity)
            : (rewind_input_t *)ivp_alloc(allocator,
                                          sizeof(*inputs) * (size_t)capacity);
        if (!inputs) {
            rewind_free_input(rewind, &copy);
            ivp_memory_checkpoint_destroy(checkpoint);
            return IVP_ERR_OUT_OF_MEMORY;
        }
        rewind->inputs = inputs;
        rewind->input_capacity = capacity;
    }

    rewind_truncate(rewind);
    if (checkpoint) rewind_add_keyframe(rewind, checkpoint);
    rewind->inputs[rewind->input_count++] = copy;
    rewind_simulate(rewind, &rewind->inputs[rewind->input_count - 1]);
    return IVP_OK;
}

static void rewind_replay(ivp_rewind_t *rewind, uint64_t psi)
{
    while (rewind->psi < psi) {
        IVP_ASSERT(rewind->psi >= rewind->log_base &&
                   rewind->psi - rewind->log_base < (uint64_t)rewind->input_count);
        rewind_simulate(rewind,
                        &rewind->inputs[rewind->psi - rewind->log_base]);
    }
}

ivp_result_t ivp_rewind_seek(ivp_rewind_t *rewind, uint64_t psi)
{
    if (!rewind || psi < ivp_rewind_oldest_psi(rewind) ||
        psi > ivp_rewind_recorded_psi(rewind)) {
        return IVP_ERR_INVALID_ARG;
    }
    if (psi >= rewind->psi) {
        rewind_replay(rewind, psi);
        return IVP_OK;
    }
    const rewind_keyframe_t *keyframe = &rewind->keyframes[0];
    for (int i = 1; i < rewind->keyframe_count; i++) {
        if (rewind->keyframes[i].psi <= psi) keyframe = &rewind->keyframes[i];
    }

    ivp_result_t restored =
        ivp_memory_region_restore(rewind->region, keyframe->checkpoint);
    if (restored != IVP_OK) return restored;
    rewind->psi = keyframe->psi;
    rewind_replay(rewind, psi);
    return IVP_OK;
}
