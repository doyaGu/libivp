#include "ivp_object_system.h"
#include <ivp/geometry/ivp_surman.h>
#include <string.h>

static void ivp_object_index_cb(void *elem, int new_index, void *userdata)
{
    (void)userdata;
    ((ivp_object_t *)elem)->dense_index = new_index;
}

static int ivp_object_get_index(const void *elem)
{
    return ((const ivp_object_t *)elem)->dense_index;
}

ivp_result_t ivp_object_system_init(ivp_object_system_t *sys,
                                    const ivp_allocator_t *alloc,
                                    int initial_capacity)
{
    return ivp_registry_init(&sys->registry,
                             alloc,
                             initial_capacity,
                             ivp_object_index_cb,
                             NULL,
                             ivp_object_get_index);
}

void ivp_object_system_destroy(ivp_object_system_t *sys)
{
    if (!sys) return;
    ivp_registry_destroy(&sys->registry, true, sizeof(ivp_object_t));
}

ivp_object_t *ivp_object_system_create(ivp_object_system_t *sys,
                                       ivp_environment_t *env,
                                       const ivp_object_template_t *tmpl,
                                       const ivp_surman_t *surman,
                                       ivp_object_handle_t *out_handle)
{
    if (!sys || !sys->registry.alloc || !env || !tmpl) return NULL;
    const ivp_allocator_t *alloc = sys->registry.alloc;
    ivp_object_t *obj = (ivp_object_t *)ivp_alloc(alloc, sizeof(ivp_object_t));
    if (!obj) {
        return NULL;
    }
    memset(obj, 0, sizeof(*obj));

    obj->type         = tmpl->type;
    obj->material     = tmpl->material;
    obj->no_collision_group = tmpl->no_collision_group;
    obj->collision_group    = 0;
    obj->layer_mask         = 0xFFFFFFFFu;
    obj->surman       = surman;
    obj->name         = tmpl->name;
    obj->client_data  = tmpl->client_data;
    obj->owner        = env;
    obj->ccd_enabled  = tmpl->ccd_enabled;
    obj->ccd_speed_threshold = tmpl->ccd_speed_threshold;
    obj->shift_core_f_object_is_zero = true;
    obj->q_core_f_object = NULL;

    ivp_handle_t h;
    if (ivp_registry_insert(&sys->registry, obj, &h) != IVP_OK) {
        ivp_free(alloc, obj, sizeof(ivp_object_t));
        return NULL;
    }

    obj->handle = h;

    if (out_handle) {
        *out_handle = h;
    }
    /* IVP_EVENT_OBJECT_CREATED is emitted by the environment once the
     * object is fully initialised */
    return obj;
}

bool ivp_object_system_destroy_by_handle(ivp_object_system_t *sys,
                                         ivp_object_handle_t handle)
{
    if (!sys || !sys->registry.alloc) return false;
    const ivp_allocator_t *alloc = sys->registry.alloc;
    void *removed = NULL;
    if (!ivp_registry_remove_by_handle(&sys->registry, handle, &removed)) {
        return false;
    }

    ivp_free(alloc, removed, sizeof(ivp_object_t));
    return true;
}

ivp_object_t *ivp_object_system_get_by_handle(const ivp_object_system_t *sys,
                                              ivp_object_handle_t handle)
{
    return (ivp_object_t *)ivp_registry_get_by_handle(&sys->registry, handle);
}

int ivp_object_system_count(const ivp_object_system_t *sys)
{
    return ivp_registry_count(&sys->registry);
}

ivp_object_t *ivp_object_system_get_at(const ivp_object_system_t *sys, int index)
{
    return (ivp_object_t *)ivp_registry_get_at(&sys->registry, index);
}
