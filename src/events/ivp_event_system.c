#include "events/ivp_event_system.h"
#include <string.h>

static void ivp_event_listener_index_cb(void *elem, int new_index, void *userdata)
{
    (void)userdata;
    ((ivp_event_listener_slot_t *)elem)->dense_index = new_index;
}

static int ivp_event_listener_get_index(const void *elem)
{
    return ((const ivp_event_listener_slot_t *)elem)->dense_index;
}

ivp_result_t ivp_event_system_init(ivp_event_system_t *sys,
                                   const ivp_allocator_t *alloc,
                                   int initial_capacity)
{
    return ivp_registry_init(&sys->registry,
                             alloc,
                             initial_capacity,
                             ivp_event_listener_index_cb,
                             NULL,
                             ivp_event_listener_get_index);
}

void ivp_event_system_destroy(ivp_event_system_t *sys,
                              const ivp_allocator_t *alloc)
{
    (void)alloc;
    ivp_registry_destroy(&sys->registry, true, sizeof(ivp_event_listener_slot_t));
}

ivp_event_listener_handle_t ivp_event_system_add_listener(ivp_event_system_t *sys,
                                                          const ivp_allocator_t *alloc,
                                                          ivp_event_listener_fn_t fn,
                                                          void *ctx)
{
    ivp_event_listener_handle_t invalid = ivp_event_listener_handle_invalid();
    if (!sys || !alloc || !fn) return invalid;

    ivp_event_listener_slot_t *slot =
        (ivp_event_listener_slot_t *)ivp_alloc(alloc, sizeof(ivp_event_listener_slot_t));
    if (!slot) return invalid;

    memset(slot, 0, sizeof(*slot));
    slot->fn = fn;
    slot->ctx = ctx;
    slot->enabled = true;

    ivp_handle_t h;
    if (ivp_registry_insert(&sys->registry, slot, &h) != IVP_OK) {
        ivp_free(alloc, slot, sizeof(ivp_event_listener_slot_t));
        return invalid;
    }

    slot->handle_slot = h.slot;
    slot->handle_generation = h.generation;

    ivp_event_listener_handle_t out;
    out.slot = h.slot;
    out.generation = h.generation;
    return out;
}

bool ivp_event_system_remove_listener(ivp_event_system_t *sys,
                                      ivp_event_listener_handle_t listener_handle,
                                      const ivp_allocator_t *alloc)
{
    if (!sys || !alloc || !ivp_event_listener_handle_is_valid(listener_handle)) return false;

    void *removed = NULL;
    if (!ivp_registry_remove_by_handle(&sys->registry, listener_handle, &removed)) {
        return false;
    }

    ivp_free(alloc, removed, sizeof(ivp_event_listener_slot_t));
    return true;
}

void ivp_event_system_clear(ivp_event_system_t *sys,
                            const ivp_allocator_t *alloc)
{
    if (!sys || !alloc) return;

    while (ivp_registry_count(&sys->registry) > 0) {
        ivp_event_listener_slot_t *slot =
            (ivp_event_listener_slot_t *)ivp_registry_get_at(&sys->registry, 0);
        ivp_event_listener_handle_t h;
        h.slot = slot->handle_slot;
        h.generation = slot->handle_generation;
        if (!ivp_event_system_remove_listener(sys, h, alloc)) {
            break;
        }
    }
}

int ivp_event_system_count(const ivp_event_system_t *sys)
{
    return ivp_registry_count(&sys->registry);
}

ivp_event_listener_slot_t *ivp_event_system_get_at(const ivp_event_system_t *sys, int index)
{
    return (ivp_event_listener_slot_t *)ivp_registry_get_at(&sys->registry, index);
}
