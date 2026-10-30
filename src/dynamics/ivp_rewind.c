/* ivp_rewind.c -- keyframe ring and input log over world snapshots
 * (libivp extension).
 *
 * The input for the PSI that follows p PSIs is inputs[p - log_base]; the
 * log reaches log_base + input_count.  Keyframes are ascending by PSI and
 * the oldest one never lies before log_base, so every keyframe can replay
 * to any recorded PSI after it. */
#include <ivp/dynamics/ivp_rewind.h>

#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_snapshot.h>

#include "controllers/ivp_controller_system.h"
#include "dynamics/ivp_env_pipeline.h"

#include <stdlib.h>
#include <string.h>

typedef struct rewind_input {
    void *data;
    size_t size;
} rewind_input_t;

typedef struct rewind_keyframe {
    uint64_t psi;
    ivp_world_snapshot_t *snapshot;
} rewind_keyframe_t;

struct ivp_rewind {
    ivp_rewind_config_t config;
    ivp_environment_t *env;
    void **states;    /* controller states of a restored world, owned */
    size_t *state_sizes;
    int state_count;
    uint64_t psi;
    uint64_t log_base;
    rewind_input_t *inputs;
    int input_count;
    int input_capacity;
    rewind_keyframe_t *keyframes;   /* keyframe_capacity entries */
    int keyframe_count;
};

static const ivp_allocator_t *rewind_allocator(void)
{
    return ivp_default_allocator();
}

static void rewind_free_states(void **states, size_t *sizes, int count)
{
    size_t slots = (size_t)(count > 0 ? count : 1);
    for (int i = 0; states && sizes && i < count; i++) {
        if (states[i]) ivp_free(rewind_allocator(), states[i], sizes[i]);
    }
    if (states) ivp_free(rewind_allocator(), states, sizeof(*states) * slots);
    if (sizes) ivp_free(rewind_allocator(), sizes, sizeof(*sizes) * slots);
}

static void rewind_free_input(rewind_input_t *input)
{
    if (input->data) ivp_free(rewind_allocator(), input->data, input->size);
    input->data = NULL;
    input->size = 0;
}

static ivp_result_t rewind_set_result(ivp_result_t *result, ivp_result_t value)
{
    if (result) *result = value;
    return value;
}

ivp_rewind_t *ivp_rewind_create(ivp_environment_t *env,
                                const ivp_rewind_config_t *config,
                                ivp_result_t *result)
{
    if (!env || !config || config->keyframe_interval < 1 ||
        config->keyframe_capacity < 1) {
        rewind_set_result(result, IVP_ERR_INVALID_ARG);
        return NULL;
    }
    const ivp_allocator_t *allocator = rewind_allocator();
    ivp_rewind_t *rewind = (ivp_rewind_t *)ivp_alloc(allocator, sizeof(*rewind));
    rewind_keyframe_t *keyframes = (rewind_keyframe_t *)ivp_alloc(
        allocator, sizeof(*keyframes) * (size_t)config->keyframe_capacity);
    ivp_result_t captured = IVP_OK;
    ivp_world_snapshot_t *first =
        rewind && keyframes ? ivp_world_snapshot_capture(env, &captured) : NULL;
    if (!first) {
        if (keyframes) {
            ivp_free(allocator, keyframes,
                     sizeof(*keyframes) * (size_t)config->keyframe_capacity);
        }
        if (rewind) ivp_free(allocator, rewind, sizeof(*rewind));
        rewind_set_result(result, rewind && keyframes ? captured
                                                      : IVP_ERR_OUT_OF_MEMORY);
        return NULL;
    }
    memset(rewind, 0, sizeof(*rewind));
    rewind->config = *config;
    rewind->env = env;
    rewind->keyframes = keyframes;
    rewind->keyframes[0].psi = 0;
    rewind->keyframes[0].snapshot = first;
    rewind->keyframe_count = 1;
    rewind_set_result(result, IVP_OK);
    return rewind;
}

void ivp_rewind_destroy(ivp_rewind_t *rewind)
{
    if (!rewind) return;
    const ivp_allocator_t *allocator = rewind_allocator();
    ivp_environment_destroy(rewind->env);
    rewind_free_states(rewind->states, rewind->state_sizes, rewind->state_count);
    for (int i = 0; i < rewind->input_count; i++) {
        rewind_free_input(&rewind->inputs[i]);
    }
    if (rewind->inputs) {
        ivp_free(allocator, rewind->inputs,
                 sizeof(*rewind->inputs) * (size_t)rewind->input_capacity);
    }
    for (int i = 0; i < rewind->keyframe_count; i++) {
        ivp_world_snapshot_destroy(rewind->keyframes[i].snapshot);
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
        rewind_free_input(&rewind->inputs[i]);
    }
    if (keep < rewind->input_count) rewind->input_count = keep;
    while (rewind->keyframe_count > 1 &&
           rewind->keyframes[rewind->keyframe_count - 1].psi > rewind->psi) {
        ivp_world_snapshot_destroy(
            rewind->keyframes[--rewind->keyframe_count].snapshot);
    }
}

/* append a keyframe, dropping the oldest one and its inputs when full */
static void rewind_add_keyframe(ivp_rewind_t *rewind,
                                ivp_world_snapshot_t *snapshot)
{
    if (rewind->keyframe_count == rewind->config.keyframe_capacity) {
        ivp_world_snapshot_destroy(rewind->keyframes[0].snapshot);
        memmove(rewind->keyframes, rewind->keyframes + 1,
                sizeof(*rewind->keyframes) * (size_t)(rewind->keyframe_count - 1));
        rewind->keyframe_count--;
    }
    rewind_keyframe_t *keyframe = &rewind->keyframes[rewind->keyframe_count++];
    keyframe->psi = rewind->psi;
    keyframe->snapshot = snapshot;

    int dropped = (int)(rewind->keyframes[0].psi - rewind->log_base);
    if (dropped > 0) {
        for (int i = 0; i < dropped; i++) rewind_free_input(&rewind->inputs[i]);
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
    const ivp_allocator_t *allocator = rewind_allocator();

    /* everything that can fail comes before the recording changes; after a
     * seek back the keyframe of this PSI may already exist, followed by
     * the recorded future that is about to be discarded */
    bool keyframe_due =
        rewind->psi % (uint64_t)rewind->config.keyframe_interval == 0u;
    for (int i = 0; keyframe_due && i < rewind->keyframe_count; i++) {
        if (rewind->keyframes[i].psi == rewind->psi) keyframe_due = false;
    }
    ivp_result_t result = IVP_OK;
    ivp_world_snapshot_t *snapshot = NULL;
    if (keyframe_due) {
        snapshot = ivp_world_snapshot_capture(rewind->env, &result);
        if (!snapshot) return result;
    }
    rewind_input_t copy = { NULL, size };
    if (size) {
        copy.data = ivp_alloc(allocator, size);
        if (!copy.data) {
            ivp_world_snapshot_destroy(snapshot);
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
            rewind_free_input(&copy);
            ivp_world_snapshot_destroy(snapshot);
            return IVP_ERR_OUT_OF_MEMORY;
        }
        rewind->inputs = inputs;
        rewind->input_capacity = capacity;
    }

    rewind_truncate(rewind);
    if (snapshot) rewind_add_keyframe(rewind, snapshot);
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

    const ivp_allocator_t *allocator = rewind_allocator();
    int count = ivp_world_snapshot_controller_count(keyframe->snapshot);
    size_t slots = (size_t)(count > 0 ? count : 1);
    void **states = (void **)ivp_alloc(allocator, sizeof(*states) * slots);
    size_t *sizes = (size_t *)ivp_alloc(allocator, sizeof(*sizes) * slots);
    if (!states || !sizes) {
        if (states) ivp_free(allocator, states, sizeof(*states) * slots);
        if (sizes) ivp_free(allocator, sizes, sizeof(*sizes) * slots);
        return IVP_ERR_OUT_OF_MEMORY;
    }
    memset(states, 0, sizeof(*states) * slots);
    memset(sizes, 0, sizeof(*sizes) * slots);
    for (int i = 0; i < count; i++) {
        sizes[i] = ivp_world_snapshot_controller_state_size(keyframe->snapshot, i);
        states[i] = ivp_alloc(allocator, sizes[i]);
        if (!states[i]) {
            rewind_free_states(states, sizes, count);
            return IVP_ERR_OUT_OF_MEMORY;
        }
        memset(states[i], 0, sizes[i]);
    }
    ivp_world_snapshot_bindings_t bindings;
    bindings.controller_states = states;
    bindings.controller_handles = NULL;
    bindings.controller_count = count;
    ivp_result_t result = IVP_OK;
    ivp_environment_t *env = ivp_world_snapshot_restore(
        keyframe->snapshot, &rewind->config.environment_config, &bindings,
        &result);
    if (!env) {
        rewind_free_states(states, sizes, count);
        return result;
    }

    ivp_environment_destroy(rewind->env);
    rewind_free_states(rewind->states, rewind->state_sizes, rewind->state_count);
    rewind->env = env;
    rewind->states = states;
    rewind->state_sizes = sizes;
    rewind->state_count = count;
    rewind->psi = keyframe->psi;
    rewind_replay(rewind, psi);
    return IVP_OK;
}

ivp_object_t *ivp_rewind_find_object(const ivp_rewind_t *rewind,
                                     uint64_t order_key)
{
    if (!rewind) return NULL;
    int count = ivp_environment_get_object_count(rewind->env);
    for (int i = 0; i < count; i++) {
        ivp_object_t *object = ivp_environment_get_object(rewind->env, i);
        if (object && ivp_object_order_key(object) == order_key) return object;
    }
    return NULL;
}

void *ivp_rewind_controller_state(const ivp_rewind_t *rewind, int index)
{
    if (!rewind || index < 0) return NULL;
    const ivp_controller_system_t *cs =
        (const ivp_controller_system_t *)rewind->env->controller_system;
    int count = ivp_controller_system_count(cs);
    if (index >= count) return NULL;
    /* the index-th smallest registration sequence */
    const ivp_controller_slot_t *found = NULL;
    for (int i = 0; i < count; i++) {
        const ivp_controller_slot_t *slot = ivp_controller_system_get_at(cs, i);
        int smaller = 0;
        for (int j = 0; j < count; j++) {
            const ivp_controller_slot_t *other = ivp_controller_system_get_at(cs, j);
            if (other->base.registration_sequence <
                slot->base.registration_sequence) {
                smaller++;
            }
        }
        if (smaller == index) {
            found = slot;
            break;
        }
    }
    return found ? found->ctx : NULL;
}
