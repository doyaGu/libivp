#ifndef IVP_REGISTRY_H
#define IVP_REGISTRY_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/containers/ivp_array.h>
#include <ivp/containers/ivp_hashmap.h>
#include <ivp/dynamics/ivp_handle.h>

typedef int (*ivp_registry_get_index_fn_t)(const void *elem);

typedef struct ivp_registry_slot_meta {
    uint32_t generation;
    uint64_t handle_key;
    bool alive;
} ivp_registry_slot_meta_t;

typedef struct ivp_registry {
    const ivp_allocator_t *alloc;
    ivp_array_t dense;
    ivp_array_t slots;
    ivp_array_t free_slots;
    ivp_hashmap_t handle_to_elem;
    ivp_registry_get_index_fn_t get_index;
} ivp_registry_t;

ivp_result_t ivp_registry_init(ivp_registry_t *reg,
                               const ivp_allocator_t *alloc,
                               int initial_capacity,
                               ivp_array_index_cb_t index_cb,
                               void *index_userdata,
                               ivp_registry_get_index_fn_t get_index);

void ivp_registry_destroy(ivp_registry_t *reg,
                          bool free_elements,
                          size_t element_size);

int ivp_registry_count(const ivp_registry_t *reg);
void *ivp_registry_get_at(const ivp_registry_t *reg, int dense_index);

ivp_result_t ivp_registry_insert(ivp_registry_t *reg,
                                 void *elem,
                                 ivp_handle_t *out_handle);

void *ivp_registry_get_by_handle(const ivp_registry_t *reg,
                                 ivp_handle_t handle);

bool ivp_registry_remove_by_handle(ivp_registry_t *reg,
                                   ivp_handle_t handle,
                                   void **out_removed_elem);

#endif
