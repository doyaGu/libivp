#ifndef IVP_EVENT_SYSTEM_H
#define IVP_EVENT_SYSTEM_H

#include <ivp/events/ivp_event.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include "dynamics/ivp_registry.h"

typedef struct ivp_event_system {
    ivp_registry_t registry;
} ivp_event_system_t;

typedef struct ivp_event_listener_slot {
    uint32_t handle_slot;
    uint32_t handle_generation;
    int dense_index;
    ivp_event_listener_fn_t fn;
    void *ctx;
    bool enabled;
} ivp_event_listener_slot_t;

ivp_result_t ivp_event_system_init(ivp_event_system_t *sys,
                                   const ivp_allocator_t *alloc,
                                   int initial_capacity);

void ivp_event_system_destroy(ivp_event_system_t *sys,
                              const ivp_allocator_t *alloc);

ivp_event_listener_handle_t ivp_event_system_add_listener(ivp_event_system_t *sys,
                                                          const ivp_allocator_t *alloc,
                                                          ivp_event_listener_fn_t fn,
                                                          void *ctx);

bool ivp_event_system_remove_listener(ivp_event_system_t *sys,
                                      ivp_event_listener_handle_t listener_handle,
                                      const ivp_allocator_t *alloc);

void ivp_event_system_clear(ivp_event_system_t *sys,
                            const ivp_allocator_t *alloc);

int ivp_event_system_count(const ivp_event_system_t *sys);
ivp_event_listener_slot_t *ivp_event_system_get_at(const ivp_event_system_t *sys, int index);

#endif
