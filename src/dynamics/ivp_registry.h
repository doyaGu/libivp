#ifndef IVP_REGISTRY_H
#define IVP_REGISTRY_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/containers/ivp_pvec.h>
#include <ivp/dynamics/ivp_handle.h>

typedef void (*ivp_registry_set_index_fn_t)(void *elem, int new_index,
                                            void *userdata);
typedef int (*ivp_registry_get_index_fn_t)(const void *elem);

typedef struct ivp_registry_slot_meta {
    uint32_t generation;
    void    *element;
} ivp_registry_slot_meta_t;

typedef struct ivp_registry {
    const ivp_allocator_t *alloc;
    ivp_pvec_t dense;
    ivp_pvec_t slots;
    ivp_pvec_t free_slots;
    ivp_registry_set_index_fn_t set_index;
    void *index_userdata;
    ivp_registry_get_index_fn_t get_index;
} ivp_registry_t;

ivp_result_t ivp_registry_init(ivp_registry_t *reg,
                               const ivp_allocator_t *alloc,
                               int initial_capacity,
                               ivp_registry_set_index_fn_t index_cb,
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
