#include "controllers/ivp_controller_system.h"
#include <string.h>

static void ivp_controller_index_cb(void *elem, int new_index, void *userdata)
{
    (void)userdata;
    ((ivp_controller_slot_t *)elem)->dense_index = new_index;
}

static int ivp_controller_get_index(const void *elem)
{
    return ((const ivp_controller_slot_t *)elem)->dense_index;
}

static void ivp_controller_slot_release_associated_handles(ivp_controller_slot_t *slot,
                                                           const ivp_allocator_t *alloc)
{
    if (!slot || !alloc) return;
    if (slot->associated_handles && slot->associated_capacity > 0) {
        ivp_free(alloc, slot->associated_handles,
                 (size_t)slot->associated_capacity * sizeof(ivp_object_handle_t));
    }
    slot->associated_handles = NULL;
    slot->associated_count = 0;
    slot->associated_capacity = 0;
}

static bool ivp_controller_slot_ensure_associated_capacity(ivp_controller_slot_t *slot,
                                                           int required,
                                                           const ivp_allocator_t *alloc)
{
    if (!slot || !alloc) return false;
    if (required <= slot->associated_capacity) return true;

    int new_cap = slot->associated_capacity > 0 ? slot->associated_capacity : 4;
    while (new_cap < required) new_cap *= 2;

    ivp_object_handle_t *new_buf = (ivp_object_handle_t *)ivp_alloc(
        alloc, (size_t)new_cap * sizeof(ivp_object_handle_t));
    if (!new_buf) return false;

    if (slot->associated_handles && slot->associated_count > 0) {
        memcpy(new_buf, slot->associated_handles,
               (size_t)slot->associated_count * sizeof(ivp_object_handle_t));
    }
    if (slot->associated_handles && slot->associated_capacity > 0) {
        ivp_free(alloc, slot->associated_handles,
                 (size_t)slot->associated_capacity * sizeof(ivp_object_handle_t));
    }

    slot->associated_handles = new_buf;
    slot->associated_capacity = new_cap;
    return true;
}

static bool ivp_controller_handle_equal(ivp_object_handle_t a, ivp_object_handle_t b)
{
    return a.slot == b.slot && a.generation == b.generation;
}

static ivp_controller_handle_t ivp_controller_system_add_internal(
    ivp_controller_system_t *sys,
    const ivp_allocator_t *alloc,
    int priority,
    ivp_controller_event_fn_t event_fn,
    void *ctx)
{
    ivp_controller_handle_t invalid = ivp_controller_handle_invalid();
    if (!sys || !alloc) return invalid;
    if (!event_fn) return invalid;

    ivp_controller_slot_t *slot =
        (ivp_controller_slot_t *)ivp_alloc(alloc, sizeof(ivp_controller_slot_t));
    if (!slot) return invalid;

    memset(slot, 0, sizeof(*slot));
    slot->priority = priority;
    slot->event_fn = event_fn;
    slot->ctx = ctx;
    slot->enabled = true;
    slot->mode = IVP_CONTROLLER_INDEPENDENT;
    slot->run_for_all_cores = true;
    slot->discovery_fn = NULL;

    ivp_handle_t h;
    if (ivp_registry_insert(&sys->registry, slot, &h) != IVP_OK) {
        ivp_free(alloc, slot, sizeof(ivp_controller_slot_t));
        return invalid;
    }

    slot->handle = h;

    return h;
}

ivp_result_t ivp_controller_system_init(ivp_controller_system_t *sys,
                                        const ivp_allocator_t *alloc,
                                        int initial_capacity)
{
    return ivp_registry_init(&sys->registry,
                             alloc,
                             initial_capacity,
                             ivp_controller_index_cb,
                             NULL,
                             ivp_controller_get_index);
}

void ivp_controller_system_destroy(ivp_controller_system_t *sys,
                                   const ivp_allocator_t *alloc)
{
    if (!sys || !alloc) return;
    ivp_controller_system_clear(sys, alloc);
    ivp_registry_destroy(&sys->registry, false, sizeof(ivp_controller_slot_t));
}

ivp_controller_handle_t ivp_controller_system_add(ivp_controller_system_t *sys,
                                                  const ivp_allocator_t *alloc,
                                                  int priority,
                                                  ivp_controller_fn_t fn,
                                                  void *ctx)
{
    return ivp_controller_system_add_internal(sys, alloc, priority, fn, ctx);
}

bool ivp_controller_system_remove(ivp_controller_system_t *sys,
                                  ivp_controller_handle_t controller_handle,
                                  const ivp_allocator_t *alloc)
{
    if (!sys || !alloc || !ivp_controller_handle_is_valid(controller_handle)) return false;

    void *removed = NULL;
    if (!ivp_registry_remove_by_handle(&sys->registry, controller_handle, &removed)) {
        return false;
    }

    ivp_controller_slot_release_associated_handles((ivp_controller_slot_t *)removed, alloc);
    ivp_free(alloc, removed, sizeof(ivp_controller_slot_t));
    return true;
}

void ivp_controller_system_clear(ivp_controller_system_t *sys,
                                 const ivp_allocator_t *alloc)
{
    if (!sys || !alloc) return;

    while (ivp_registry_count(&sys->registry) > 0) {
        ivp_controller_slot_t *slot = (ivp_controller_slot_t *)ivp_registry_get_at(&sys->registry, 0);
        ivp_controller_handle_t h = slot->handle;
        if (!ivp_controller_system_remove(sys, h, alloc)) {
            break;
        }
    }
}

int ivp_controller_system_count(const ivp_controller_system_t *sys)
{
    return ivp_registry_count(&sys->registry);
}

ivp_controller_slot_t *ivp_controller_system_get_at(const ivp_controller_system_t *sys, int index)
{
    return (ivp_controller_slot_t *)ivp_registry_get_at(&sys->registry, index);
}

ivp_controller_slot_t *ivp_controller_system_get_by_handle(const ivp_controller_system_t *sys,
                                                            ivp_controller_handle_t handle)
{
    ivp_handle_t h;
    h.slot = handle.slot;
    h.generation = handle.generation;
    return (ivp_controller_slot_t *)ivp_registry_get_by_handle(&sys->registry, h);
}

void ivp_controller_system_sort(ivp_controller_system_t *sys,
                                bool use_reference_tiebreak)
{
    const int n = ivp_registry_count(&sys->registry);
    void **data = sys->registry.dense.elems;

    /* Keep the registry in descending priority order for management
     * traversals.  Simulation units maintain their own ascending array and
     * execute it from the last element to the first. */
    for (int i = 1; i < n; i++) {
        ivp_controller_slot_t *key = (ivp_controller_slot_t *)data[i];
        int j = i - 1;
        while (j >= 0) {
            ivp_controller_slot_t *cur = (ivp_controller_slot_t *)data[j];
            const bool tie_shift = use_reference_tiebreak
                ? cur->handle.slot > key->handle.slot
                : cur->registration_sequence > key->registration_sequence;
            const bool should_shift =
                (cur->priority < key->priority) ||
                (cur->priority == key->priority && tie_shift);
            if (!should_shift) break;
            data[j + 1] = data[j];
            j--;
        }
        data[j + 1] = key;
    }

    for (int i = 0; i < n; i++) {
        ivp_controller_index_cb(data[i], i, NULL);
    }
}

bool ivp_controller_slot_set_mode(ivp_controller_slot_t *slot,
                                  ivp_controller_mode_t mode)
{
    if (!slot) return false;
    slot->mode = mode;
    return true;
}

bool ivp_controller_slot_set_discovery_fn(ivp_controller_slot_t *slot,
                                          ivp_controller_discovery_fn_t fn)
{
    if (!slot) return false;
    slot->discovery_fn = fn;
    return true;
}

bool ivp_controller_slot_set_associated_handles(ivp_controller_slot_t *slot,
                                                const ivp_object_handle_t *handles,
                                                int handle_count,
                                                const ivp_allocator_t *alloc)
{
    if (!slot || !alloc) return false;
    if (handle_count < 0) return false;
    if (handle_count > 0 && !handles) return false;

    if (handle_count == 0) {
        slot->associated_count = 0;
        return true;
    }

    if (!ivp_controller_slot_ensure_associated_capacity(slot, handle_count, alloc)) {
        return false;
    }

    int count = 0;
    for (int i = 0; i < handle_count; i++) {
        if (!ivp_object_handle_is_valid(handles[i])) continue;
        bool seen = false;
        for (int j = 0; j < count; j++) {
            if (ivp_controller_handle_equal(slot->associated_handles[j], handles[i])) {
                seen = true;
                break;
            }
        }
        if (seen) continue;
        slot->associated_handles[count++] = handles[i];
    }
    slot->associated_count = count;
    return true;
}

bool ivp_controller_slot_add_associated_handle(ivp_controller_slot_t *slot,
                                               ivp_object_handle_t handle,
                                               const ivp_allocator_t *alloc)
{
    if (!slot || !alloc || !ivp_object_handle_is_valid(handle)) return false;

    for (int i = 0; i < slot->associated_count; i++) {
        if (ivp_controller_handle_equal(slot->associated_handles[i], handle)) {
            return true;
        }
    }

    if (!ivp_controller_slot_ensure_associated_capacity(slot, slot->associated_count + 1, alloc)) {
        return false;
    }

    slot->associated_handles[slot->associated_count++] = handle;
    return true;
}

bool ivp_controller_slot_remove_associated_handle(ivp_controller_slot_t *slot,
                                                  ivp_object_handle_t handle)
{
    if (!slot || !ivp_object_handle_is_valid(handle)) return false;
    for (int i = 0; i < slot->associated_count; i++) {
        if (!ivp_controller_handle_equal(slot->associated_handles[i], handle)) continue;
        slot->associated_handles[i] = slot->associated_handles[slot->associated_count - 1];
        slot->associated_count--;
        return true;
    }
    return false;
}

int ivp_controller_slot_collect_associated_handles(const ivp_controller_slot_t *slot,
                                                   ivp_environment_t *env,
                                                   ivp_object_handle_t *out_handles,
                                                   int max_handles)
{
    if (!slot || !env || !out_handles || max_handles <= 0) return 0;

    int count = 0;
    if (slot->discovery_fn) {
        count = slot->discovery_fn(slot->ctx, env, out_handles, max_handles);
        if (count < 0) count = 0;
        if (count > max_handles) count = max_handles;
    } else {
        count = slot->associated_count;
        if (count > max_handles) count = max_handles;
        for (int i = 0; i < count; i++) {
            out_handles[i] = slot->associated_handles[i];
        }
    }

    int unique = 0;
    for (int i = 0; i < count; i++) {
        if (!ivp_object_handle_is_valid(out_handles[i])) continue;
        bool seen = false;
        for (int j = 0; j < unique; j++) {
            if (ivp_controller_handle_equal(out_handles[j], out_handles[i])) {
                seen = true;
                break;
            }
        }
        if (seen) continue;
        out_handles[unique++] = out_handles[i];
    }
    return unique;
}
