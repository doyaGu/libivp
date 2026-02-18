#include "ivp_object_system.h"
#include "dynamics/ivp_environment_events.h"
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

void ivp_object_system_destroy(ivp_object_system_t *sys,
                               const ivp_allocator_t *alloc)
{
    (void)alloc;
    ivp_registry_destroy(&sys->registry, true, sizeof(ivp_object_t));
}

ivp_object_t *ivp_object_system_create(ivp_object_system_t *sys,
                                       ivp_environment_t *env,
                                       const ivp_object_template_t *tmpl,
                                       const ivp_surman_t *surman,
                                       ivp_object_handle_t *out_handle)
{
    ivp_object_t *obj = (ivp_object_t *)ivp_alloc(env->allocator, sizeof(ivp_object_t));
    if (!obj) {
        return NULL;
    }
    memset(obj, 0, sizeof(*obj));

    ivp_core_init(&obj->core, tmpl->mass, &tmpl->rot_inertia);
    ivp_core_set_pose(&obj->core, &tmpl->position, &tmpl->orientation);

    /* Propagate bounding radius from surface manager to core.
     * Without this, broadphase sphere overlap uses the default 1.0
     * and misses all collisions for objects larger than unit size. */
    if (surman && surman->vtable->get_radius_and_deviation) {
        float center[3] = {0.0f, 0.0f, 0.0f};
        float radius = 1.0f, deviation = 0.0f;
        surman->vtable->get_radius_and_deviation(surman, center, &radius, &deviation);
        obj->core.upper_limit_radius    = (ivp_real_t)radius;
        obj->core.max_surface_deviation = (ivp_real_t)deviation;
    }

    obj->core.speed_damp_factor     = tmpl->speed_damp_factor;
    obj->core.rot_speed_damp_factor = tmpl->rot_speed_damp_factor;
    obj->core.physical_unmoveable   = tmpl->physical_unmoveable;
    obj->core.time_of_last_psi      = env->current_time;

    if (tmpl->physical_unmoveable) {
        obj->core.movement_state = IVP_MT_STATIC;
    }

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

    ivp_handle_t h;
    if (ivp_registry_insert(&sys->registry, obj, &h) != IVP_OK) {
        ivp_free(env->allocator, obj, sizeof(ivp_object_t));
        return NULL;
    }

    obj->handle_slot = h.slot;
    obj->handle_generation = h.generation;
    obj->handle_key = 0;

    if (out_handle) {
        out_handle->slot = h.slot;
        out_handle->generation = h.generation;
    }

    ivp_environment_emit_event_internal(env, IVP_EVENT_OBJECT_CREATED, IVP_REAL(0.0),
                                        obj->dense_index, 0);

    return obj;
}

bool ivp_object_system_destroy_by_handle(ivp_object_system_t *sys,
                                         ivp_object_handle_t handle,
                                         const ivp_allocator_t *alloc)
{
    void *removed = NULL;
    if (!ivp_registry_remove_by_handle(&sys->registry, handle, &removed)) {
        return false;
    }

    ivp_object_t *obj = (ivp_object_t *)removed;
    if (obj->owner) {
        ivp_environment_emit_event_internal(obj->owner, IVP_EVENT_OBJECT_DESTROYED,
                                            IVP_REAL(0.0), -1, 0);
    }
    ivp_free(alloc, obj, sizeof(ivp_object_t));
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
