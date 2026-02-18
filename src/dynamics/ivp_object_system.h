#ifndef IVP_OBJECT_SYSTEM_H
#define IVP_OBJECT_SYSTEM_H

#include <ivp/dynamics/ivp_environment.h>
#include "ivp_registry.h"

typedef struct ivp_object_system {
    ivp_registry_t registry;
} ivp_object_system_t;

ivp_result_t ivp_object_system_init(ivp_object_system_t *sys,
                                    const ivp_allocator_t *alloc,
                                    int initial_capacity);

void ivp_object_system_destroy(ivp_object_system_t *sys,
                               const ivp_allocator_t *alloc);

ivp_object_t *ivp_object_system_create(ivp_object_system_t *sys,
                                       ivp_environment_t *env,
                                       const ivp_object_template_t *tmpl,
                                       const ivp_surman_t *surman,
                                       ivp_object_handle_t *out_handle);

bool ivp_object_system_destroy_by_handle(ivp_object_system_t *sys,
                                         ivp_object_handle_t handle,
                                         const ivp_allocator_t *alloc);

ivp_object_t *ivp_object_system_get_by_handle(const ivp_object_system_t *sys,
                                              ivp_object_handle_t handle);

int ivp_object_system_count(const ivp_object_system_t *sys);
ivp_object_t *ivp_object_system_get_at(const ivp_object_system_t *sys, int index);

#endif
