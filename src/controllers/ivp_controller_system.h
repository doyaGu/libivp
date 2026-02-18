#ifndef IVP_CONTROLLER_SYSTEM_H
#define IVP_CONTROLLER_SYSTEM_H

#include <ivp/controllers/ivp_controller.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include "dynamics/ivp_registry.h"

typedef struct ivp_controller_system {
    ivp_registry_t registry;
} ivp_controller_system_t;

/* Controller target mode:
 *   INDEPENDENT — called once per active object (e.g. gravity)
 *   DEPENDENT   — called only for specific target objects */
typedef enum ivp_controller_mode {
    IVP_CONTROLLER_INDEPENDENT = 0,
    IVP_CONTROLLER_DEPENDENT   = 1
} ivp_controller_mode_t;

#define IVP_CONTROLLER_DISCOVERY_STACK_MAX 64

typedef struct ivp_controller_slot {
    uint32_t handle_slot;
    uint32_t handle_generation;
    int dense_index;
    int priority;
    ivp_controller_event_fn_t event_fn;
    void *ctx;
    bool enabled;
    ivp_controller_mode_t mode;
    bool run_for_all_cores;
    ivp_controller_discovery_fn_t discovery_fn;

    /* Explicit associated-core/object handles. For dependent controllers
     * this is a fallback set when discovery_fn is NULL; for independent
     * controllers this powers per-core attach/remove parity APIs. */
    ivp_object_handle_t *associated_handles;
    int associated_count;
    int associated_capacity;
} ivp_controller_slot_t;

ivp_result_t ivp_controller_system_init(ivp_controller_system_t *sys,
                                        const ivp_allocator_t *alloc,
                                        int initial_capacity);

void ivp_controller_system_destroy(ivp_controller_system_t *sys,
                                   const ivp_allocator_t *alloc);

ivp_controller_handle_t ivp_controller_system_add(ivp_controller_system_t *sys,
                                                  const ivp_allocator_t *alloc,
                                                  int priority,
                                                  ivp_controller_fn_t fn,
                                                  void *ctx);

bool ivp_controller_system_remove(ivp_controller_system_t *sys,
                                  ivp_controller_handle_t controller_handle,
                                  const ivp_allocator_t *alloc);

void ivp_controller_system_clear(ivp_controller_system_t *sys,
                                 const ivp_allocator_t *alloc);

int ivp_controller_system_count(const ivp_controller_system_t *sys);
ivp_controller_slot_t *ivp_controller_system_get_at(const ivp_controller_system_t *sys, int index);
ivp_controller_slot_t *ivp_controller_system_get_by_handle(const ivp_controller_system_t *sys,
                                                            ivp_controller_handle_t handle);
void ivp_controller_system_sort(ivp_controller_system_t *sys);

bool ivp_controller_slot_set_mode(ivp_controller_slot_t *slot,
                                  ivp_controller_mode_t mode);
bool ivp_controller_slot_set_discovery_fn(ivp_controller_slot_t *slot,
                                          ivp_controller_discovery_fn_t fn);
bool ivp_controller_slot_set_associated_handles(ivp_controller_slot_t *slot,
                                                const ivp_object_handle_t *handles,
                                                int handle_count,
                                                const ivp_allocator_t *alloc);
bool ivp_controller_slot_add_associated_handle(ivp_controller_slot_t *slot,
                                               ivp_object_handle_t handle,
                                               const ivp_allocator_t *alloc);
bool ivp_controller_slot_remove_associated_handle(ivp_controller_slot_t *slot,
                                                  ivp_object_handle_t handle);
int ivp_controller_slot_collect_associated_handles(const ivp_controller_slot_t *slot,
                                                   ivp_environment_t *env,
                                                   ivp_object_handle_t *out_handles,
                                                   int max_handles);

#endif
