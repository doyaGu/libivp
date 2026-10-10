/* ivp_phantom.c -- IVP_Controller_Phantom (ivp_physics/ivp_phantom.cxx)
 *
 * The mindist manager calls ivp_phantom_mindist_entered_volume /
 * ivp_phantom_mindist_left_volume (IVP_Mindist_Manager::mindist_entered_phantom
 * / mindist_left_phantom) for every object of the mindist that carries a
 * phantom controller.
 */
#include <ivp/dynamics/ivp_phantom.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/events/ivp_listener_object.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/collision/ivp_hull_mgr.h>
#include <ivp/base/ivp_assert.h>

#include "dynamics/ivp_env_pipeline.h"
#include "dynamics/ivp_env_internal.h"
#include "dynamics/ivp_sim_internal.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

void ivp_phantom_mindist_entered_volume(void *controller_phantom, ivp_mindist_t *md);
void ivp_phantom_mindist_left_volume(void *controller_phantom, ivp_mindist_t *md);
float ivp_phantom_get_exit_policy_extra_radius(void *controller_phantom);

void ivp_template_phantom_init(ivp_template_phantom_t *t)
{
    t->manage_intruding_objects = false;
    t->manage_intruding_cores = false;
    t->dont_check_for_unmoveables = false;
    t->exit_policy_extra_radius = 0.5f;
    t->exit_policy_extra_time = 0.5f;
    t->manage_sleeping_cores = false;
}

ivp_object_t *ivp_controller_phantom_get_other_object(const ivp_controller_phantom_t *c, const ivp_mindist_t *md)
{
    if (md->synapse[0].l_obj == c->object) return md->synapse[1].l_obj;
    return md->synapse[0].l_obj;
}

/* IVP_Listener_Object part of the controller */
typedef struct phantom_obj_listener {
    ivp_listener_object_t    base;
    ivp_controller_phantom_t *c;
} phantom_obj_listener_t;

static ivp_listener_object_t *ctrl_obj_listener(ivp_controller_phantom_t *c)
{
    return &((phantom_obj_listener_t *)c->object_listener)->base;
}

void ivp_controller_phantom_add_listener(ivp_controller_phantom_t *c, ivp_listener_phantom_t *l)
{
    ivp_pvec_add(&c->listeners, l);
}

void ivp_controller_phantom_remove_listener(ivp_controller_phantom_t *c, ivp_listener_phantom_t *l)
{
    ivp_pvec_remove(&c->listeners, l);
}

void ivp_controller_phantom_add_sleeping_object(ivp_controller_phantom_t *c, ivp_object_t *obj)
{
    if (!c->set_of_sleeping_cores) return;
    ivp_core_t *core = obj->physical_core;   /* get_core() */
    if (ivp_set_find(c->set_of_sleeping_cores, core)) return;
    for (int i = ivp_core_n_objects(core) - 1; i >= 0; i--) {
        ivp_object_add_listener_object(ivp_core_object_at(core, i), ctrl_obj_listener(c));
    }
    ivp_set_add(c->set_of_sleeping_cores, core);
}

void ivp_controller_phantom_remove_sleeping_object(ivp_controller_phantom_t *c, ivp_object_t *obj)
{
    if (!c->set_of_sleeping_cores) return;
    ivp_core_t *core = obj->physical_core;   /* get_core() */
    if (!ivp_set_find(c->set_of_sleeping_cores, core)) return;
    for (int i = ivp_core_n_objects(core) - 1; i >= 0; i--) {
        ivp_object_remove_listener_object(ivp_core_object_at(core, i), ctrl_obj_listener(c));
    }
    ivp_set_remove(c->set_of_sleeping_cores, core);
}

void ivp_controller_phantom_wake_all_sleeping_objects(ivp_controller_phantom_t *c)
{
    if (c->set_of_sleeping_cores) {
        ivp_set_enumerator_t e;
        ivp_set_enumerator_init(&e, c->set_of_sleeping_cores);
        ivp_core_t *core;
        while ((core = (ivp_core_t *)ivp_set_enumerator_next(&e, c->set_of_sleeping_cores)) != NULL) {
            for (int i = ivp_core_n_objects(core) - 1; i >= 0; i--) ivp_object_ensure_in_simulation(ivp_core_object_at(core, i));
        }
    }
    if (c->set_of_cores) {
        ivp_set_enumerator_t e;
        ivp_set_enumerator_init(&e, &c->set_of_cores->set);
        ivp_core_t *core;
        while ((core = (ivp_core_t *)ivp_set_enumerator_next(&e, &c->set_of_cores->set)) != NULL) {
            for (int i = ivp_core_n_objects(core) - 1; i >= 0; i--) ivp_object_ensure_in_simulation(ivp_core_object_at(core, i));
        }
    }
}

static void fire_event_core_entered(ivp_controller_phantom_t *c, ivp_core_t *core)
{
    for (int i = ivp_pvec_len(&c->listeners) - 1; i >= 0; i--) {
        ivp_listener_phantom_t *l = (ivp_listener_phantom_t *)ivp_pvec_at(&c->listeners, i);
        if (l->core_entered_volume) l->core_entered_volume(l, c, core);
    }
}

static void fire_event_core_left(ivp_controller_phantom_t *c, ivp_core_t *core)
{
    for (int i = ivp_pvec_len(&c->listeners) - 1; i >= 0; i--) {
        ivp_listener_phantom_t *l = (ivp_listener_phantom_t *)ivp_pvec_at(&c->listeners, i);
        if (l->core_left_volume) l->core_left_volume(l, c, core);
    }
}

static int phantom_compare_core_identity(const void *left,
                                         const void *right)
{
    const ivp_core_t *left_core =
        *(ivp_core_t *const *)left;
    const ivp_core_t *right_core =
        *(ivp_core_t *const *)right;
    uint64_t left_key = ivp_core_stable_order_key(left_core);
    uint64_t right_key = ivp_core_stable_order_key(right_core);
    IVP_ASSERT(left_key != right_key || left_core == right_core);
    return left_key < right_key ? -1 : left_key > right_key ? 1 : 0;
}

static void phantom_fire_core_left_for_set(ivp_controller_phantom_t *c,
                                           const ivp_set_t *set)
{
    if (!ivp_build_uses_canonical_internal_state()) {
        ivp_set_enumerator_t enumerator;
        ivp_set_enumerator_init(&enumerator, set);
        ivp_core_t *core;
        while ((core = (ivp_core_t *)ivp_set_enumerator_next(
                    &enumerator, set)) != NULL) {
            fire_event_core_left(c, core);
        }
        return;
    }

    int count = ivp_set_count(set);
    if (count <= 0) return;
    const ivp_allocator_t *allocator = c->object->owner->allocator;
    ivp_core_t **cores = (ivp_core_t **)ivp_alloc(
        allocator, (size_t)count * sizeof(*cores));
    if (cores) {
        int actual_count = 0;
        ivp_set_enumerator_t enumerator;
        ivp_set_enumerator_init(&enumerator, set);
        ivp_core_t *core;
        while (actual_count < count &&
               (core = (ivp_core_t *)ivp_set_enumerator_next(
                    &enumerator, set)) != NULL) {
            cores[actual_count++] = core;
        }
        qsort(cores, (size_t)actual_count, sizeof(*cores),
              phantom_compare_core_identity);
        for (int i = 0; i < actual_count; i++) {
            fire_event_core_left(c, cores[i]);
        }
        ivp_free(allocator, cores, (size_t)count * sizeof(*cores));
        return;
    }

    /* Destruction must still have a stable callback order when temporary
     * allocation fails.  Rescan the intact set for the next identity. */
    bool have_previous = false;
    uint64_t previous_key = 0u;
    for (int emitted = 0; emitted < count; emitted++) {
        ivp_core_t *selected = NULL;
        uint64_t selected_key = UINT64_MAX;
        ivp_set_enumerator_t enumerator;
        ivp_set_enumerator_init(&enumerator, set);
        ivp_core_t *core;
        while ((core = (ivp_core_t *)ivp_set_enumerator_next(
                    &enumerator, set)) != NULL) {
            uint64_t key = ivp_core_stable_order_key(core);
            if ((!have_previous || key > previous_key) &&
                (!selected || key < selected_key)) {
                selected = core;
                selected_key = key;
            }
        }
        if (!selected) break;
        fire_event_core_left(c, selected);
        previous_key = selected_key;
        have_previous = true;
    }
}

/* IVP_Controller_Phantom::mindist_entered_volume */
static void controller_mindist_entered_volume(ivp_controller_phantom_t *c, ivp_mindist_t *mindist)
{
    ivp_set_active_add(&c->set_of_mindists, mindist);

    if (c->set_of_objects) {
        ivp_object_t *other_object = ivp_controller_phantom_get_other_object(c, mindist);
        intptr_t x = (intptr_t)ivp_vhash_store_find(c->mindist_object_counter, other_object);
        if (x) {
            ivp_vhash_store_change(c->mindist_object_counter, other_object, (void *)(x + 1));
        } else {
            ivp_vhash_store_add(c->mindist_object_counter, other_object, (void *)1);
            ivp_set_active_add(c->set_of_objects, other_object);
        }
    }
    if (c->set_of_cores) {
        ivp_object_t *other_object = ivp_controller_phantom_get_other_object(c, mindist);
        ivp_core_t *other_core = other_object->physical_core;
        intptr_t x = (intptr_t)ivp_vhash_store_find(c->mindist_core_counter, other_core);
        if (x) {
            ivp_vhash_store_change(c->mindist_core_counter, other_core, (void *)(x + 1));
        } else {
            ivp_vhash_store_add(c->mindist_core_counter, other_core, (void *)1);
            ivp_set_active_add(c->set_of_cores, other_core);
            if (c->set_of_sleeping_cores && ivp_set_find(c->set_of_sleeping_cores, other_core)) {
                ivp_controller_phantom_remove_sleeping_object(c, other_object);
            } else {
                fire_event_core_entered(c, other_core);
            }
        }
    }

    for (int i = ivp_pvec_len(&c->listeners) - 1; i >= 0; i--) {
        ivp_listener_phantom_t *l = (ivp_listener_phantom_t *)ivp_pvec_at(&c->listeners, i);
        if (l->mindist_entered_volume) l->mindist_entered_volume(l, c, mindist);
    }
}

/* IVP_Controller_Phantom::mindist_left_volume */
static void controller_mindist_left_volume(ivp_controller_phantom_t *c, ivp_mindist_t *mindist)
{
    ivp_set_active_remove(&c->set_of_mindists, mindist);
    if (c->set_of_objects) {
        ivp_object_t *other_object = ivp_controller_phantom_get_other_object(c, mindist);
        intptr_t x = (intptr_t)ivp_vhash_store_find(c->mindist_object_counter, other_object);
        if (x > 1) {
            ivp_vhash_store_change(c->mindist_object_counter, other_object, (void *)(x - 1));
        } else {
            (void)ivp_vhash_store_remove(c->mindist_object_counter, other_object);
            ivp_set_active_remove(c->set_of_objects, other_object);
        }
    }
    if (c->set_of_cores) {
        ivp_object_t *other_object = ivp_controller_phantom_get_other_object(c, mindist);
        ivp_core_t *other_core = other_object->physical_core;
        intptr_t x = (intptr_t)ivp_vhash_store_find(c->mindist_core_counter, other_core);
        if (x > 1) {
            ivp_vhash_store_change(c->mindist_core_counter, other_core, (void *)(x - 1));
        } else {
            (void)ivp_vhash_store_remove(c->mindist_core_counter, other_core);
            ivp_set_active_remove(c->set_of_cores, other_core);
            if (c->set_of_sleeping_cores && other_object->object_movement_state == IVP_MT_NOT_SIM) {
                ivp_controller_phantom_add_sleeping_object(c, other_object);
            } else {
                fire_event_core_left(c, other_core);
            }
        }
    }

    for (int i = ivp_pvec_len(&c->listeners) - 1; i >= 0; i--) {
        ivp_listener_phantom_t *l = (ivp_listener_phantom_t *)ivp_pvec_at(&c->listeners, i);
        if (l->mindist_left_volume) l->mindist_left_volume(l, c, mindist);
    }
}

/* glue for the mindist manager (ivp_mindist_core.c) */
void ivp_phantom_mindist_entered_volume(void *controller_phantom, ivp_mindist_t *md)
{
    controller_mindist_entered_volume((ivp_controller_phantom_t *)controller_phantom, md);
}

void ivp_phantom_mindist_left_volume(void *controller_phantom, ivp_mindist_t *md)
{
    controller_mindist_left_volume((ivp_controller_phantom_t *)controller_phantom, md);
}

float ivp_phantom_get_exit_policy_extra_radius(void *controller_phantom)
{
    return ((ivp_controller_phantom_t *)controller_phantom)->exit_policy_extra_radius;
}

/* IVP_Controller_Phantom::event_object_revived (also used for deleted) */
static void phantom_event_object_revived(ivp_listener_object_t *self, ivp_event_object_t *ev)
{
    ivp_controller_phantom_t *c = ((phantom_obj_listener_t *)self)->c;
    ivp_core_t *core = ev->real_object->physical_core;
    if (c->set_of_sleeping_cores && ivp_set_find(c->set_of_sleeping_cores, core)) {
        ivp_controller_phantom_remove_sleeping_object(c, ev->real_object);
        if (c->set_of_cores) {
            if (!ivp_vhash_store_find(c->mindist_core_counter, core)) fire_event_core_left(c, core);
        }
    }
}

static void controller_phantom_free_storage(ivp_controller_phantom_t *c)
{
    const ivp_allocator_t *allocator = c->object->owner->allocator;
    if (c->mindist_core_counter) {
        ivp_vhash_store_destroy(c->mindist_core_counter);
        ivp_free(allocator, c->mindist_core_counter, sizeof(*c->mindist_core_counter));
    }
    if (c->mindist_object_counter) {
        ivp_vhash_store_destroy(c->mindist_object_counter);
        ivp_free(allocator, c->mindist_object_counter, sizeof(*c->mindist_object_counter));
    }
    if (c->set_of_objects) {
        ivp_set_active_destroy(c->set_of_objects);
        ivp_free(allocator, c->set_of_objects, sizeof(*c->set_of_objects));
    }
    if (c->set_of_sleeping_cores) {
        ivp_set_destroy(c->set_of_sleeping_cores);
        ivp_free(allocator, c->set_of_sleeping_cores, sizeof(*c->set_of_sleeping_cores));
    }
    if (c->set_of_cores) {
        ivp_set_active_destroy(c->set_of_cores);
        ivp_free(allocator, c->set_of_cores, sizeof(*c->set_of_cores));
    }
    ivp_set_active_destroy(&c->set_of_mindists);
    ivp_pvec_clear(&c->listeners);
    ivp_free(allocator, c->object_listener, sizeof(phantom_obj_listener_t));
    ivp_free(allocator, c, sizeof(*c));
}

/* IVP_Controller_Phantom::IVP_Controller_Phantom */
static ivp_controller_phantom_t *controller_phantom_create(ivp_object_t *object, const ivp_template_phantom_t *templat)
{
    const ivp_allocator_t *alloc = object->owner->allocator;
    ivp_controller_phantom_t *c = (ivp_controller_phantom_t *)ivp_alloc(alloc, sizeof(*c));
    phantom_obj_listener_t *ol = (phantom_obj_listener_t *)ivp_alloc(alloc, sizeof(*ol));
    if (!c || !ol) {
        if (c) ivp_free(alloc, c, sizeof(*c));
        if (ol) ivp_free(alloc, ol, sizeof(*ol));
        return NULL;
    }
    memset(c, 0, sizeof(*c));
    memset(ol, 0, sizeof(*ol));
    ol->c = c;
    ol->base.event_object_deleted = phantom_event_object_revived;
    ol->base.event_object_revived = phantom_event_object_revived;
    c->object_listener = ol;

    c->object = object;
    ivp_pvec_init_with_allocator(&c->listeners, alloc);
    ivp_set_active_init(&c->set_of_mindists, alloc, 16);
    if (!c->set_of_mindists.set.hash.elems) goto fail;
    if (templat->manage_intruding_objects) {
        c->set_of_objects = (ivp_set_active_t *)ivp_alloc(alloc, sizeof(ivp_set_active_t));
        c->mindist_object_counter = (ivp_vhash_store_t *)ivp_alloc(alloc, sizeof(ivp_vhash_store_t));
        if (!c->set_of_objects || !c->mindist_object_counter) goto fail;
        memset(c->set_of_objects, 0, sizeof(*c->set_of_objects));
        memset(c->mindist_object_counter, 0, sizeof(*c->mindist_object_counter));
        ivp_set_active_init(c->set_of_objects, alloc, 16);
        ivp_vhash_store_init(c->mindist_object_counter, alloc, 16);
        if (!c->set_of_objects->set.hash.elems || !c->mindist_object_counter->elems) goto fail;
    }
    if (templat->manage_intruding_cores) {
        c->set_of_cores = (ivp_set_active_t *)ivp_alloc(alloc, sizeof(ivp_set_active_t));
        c->mindist_core_counter = (ivp_vhash_store_t *)ivp_alloc(alloc, sizeof(ivp_vhash_store_t));
        if (!c->set_of_cores || !c->mindist_core_counter) goto fail;
        memset(c->set_of_cores, 0, sizeof(*c->set_of_cores));
        memset(c->mindist_core_counter, 0, sizeof(*c->mindist_core_counter));
        ivp_set_active_init(c->set_of_cores, alloc, 16);
        ivp_vhash_store_init(c->mindist_core_counter, alloc, 16);
        if (!c->set_of_cores->set.hash.elems || !c->mindist_core_counter->elems) goto fail;
        if (templat->manage_sleeping_cores) {
            c->set_of_sleeping_cores = (ivp_set_t *)ivp_alloc(alloc, sizeof(ivp_set_t));
            if (!c->set_of_sleeping_cores) goto fail;
            memset(c->set_of_sleeping_cores, 0, sizeof(*c->set_of_sleeping_cores));
            ivp_set_init(c->set_of_sleeping_cores, alloc, 16);
            if (!c->set_of_sleeping_cores->hash.elems) goto fail;
        }
    }

    /* obj->controller_phantom is assigned after construction (as in
     * IVP_Real_Object::convert_to_phantom): mindists created by the recheck
     * below start as collision mindists and are converted by the loops */
    if (!templat->dont_check_for_unmoveables) {
        if (object->object_movement_state == IVP_MT_STATIC) {
            ivp_object_set_movement_state(object, IVP_MT_STATIC_PHANTOM);
            ivp_mindist_mgr_recheck_ov_element(&object->owner->pl->mindist_mgr, object);
        }
    }

    c->exit_policy_extra_radius = templat->exit_policy_extra_radius;

    /* loop through all invalid mindists and insert them into mindist set */
    for (ivp_md_synapse_t *syn = object->invalid_synapses; syn; syn = syn->next) {
        ivp_mindist_t *mindist = syn->mindist;
        mindist->mindist_function = IVP_MF_PHANTOM;
        controller_mindist_entered_volume(c, mindist);
        mindist->is_in_phantom_set = true;
    }

    /* loop through all exact mindists and mark them as phantom mindists */
    for (ivp_md_synapse_t *syn = object->exact_synapses; syn; syn = syn->next) {
        ivp_mindist_t *mindist = syn->mindist;
        mindist->mindist_function = IVP_MF_PHANTOM;
        if (ivp_mindist_get_length(mindist) < 0.0f) {
            controller_mindist_entered_volume(c, mindist);
            mindist->is_in_phantom_set = true;
        } else {
            mindist->is_in_phantom_set = false;
        }
    }

    /* loop through all hull mindists and mark them as phantom mindists */
    if (object->hull_manager) {
        ivp_minlist_enumerator_t en;
        ivp_minlist_enum_init(&en, &object->hull_manager->sorted_synapses);
        ivp_listener_hull_t *supers;
        while ((supers = (ivp_listener_hull_t *)ivp_minlist_enum_next(&en)) != NULL) {
            if (supers->vt->type != IVP_HULL_ELEM_POLYGON) continue;
            ivp_md_synapse_t *syn = (ivp_md_synapse_t *)supers;
            ivp_mindist_t *md = syn->mindist;
            if (md->mindist_function == IVP_MF_COLLISION) md->mindist_function = IVP_MF_PHANTOM;
        }
    }
    c->client_data = NULL;
    return c;

fail:
    controller_phantom_free_storage(c);
    return NULL;
}

/* IVP_Controller_Phantom::~IVP_Controller_Phantom */
void ivp_controller_phantom_destroy(ivp_controller_phantom_t *c)
{
    if (!c) return;
    if (c->set_of_cores) {
        phantom_fire_core_left_for_set(c, &c->set_of_cores->set);
    }
    if (c->set_of_sleeping_cores) {
        phantom_fire_core_left_for_set(c, c->set_of_sleeping_cores);
        ivp_set_enumerator_t enumerator;
        ivp_set_enumerator_init(&enumerator, c->set_of_sleeping_cores);
        ivp_core_t *core;
        while ((core = (ivp_core_t *)ivp_set_enumerator_next(
                    &enumerator, c->set_of_sleeping_cores)) != NULL) {
            for (int i = ivp_core_n_objects(core) - 1; i >= 0; i--) {
                ivp_object_remove_listener_object(
                    ivp_core_object_at(core, i), ctrl_obj_listener(c));
            }
        }
    }
    for (int i = ivp_pvec_len(&c->listeners) - 1; i >= 0; i--) {
        if (i >= ivp_pvec_len(&c->listeners)) continue;
        ivp_listener_phantom_t *l = (ivp_listener_phantom_t *)ivp_pvec_at(&c->listeners, i);
        if (l->phantom_is_going_to_be_deleted_event) l->phantom_is_going_to_be_deleted_event(l, c);
    }
    c->object->controller_phantom = NULL;
    controller_phantom_free_storage(c);
}

/* IVP_Real_Object::convert_to_phantom */
ivp_controller_phantom_t *ivp_object_convert_to_phantom(ivp_object_t *obj, const ivp_template_phantom_t *tmpl)
{
    if (!obj) return NULL;
    ivp_template_phantom_t def;
    if (!tmpl) {
        ivp_template_phantom_init(&def);
        tmpl = &def;
    }
    if (obj->controller_phantom) ivp_controller_phantom_destroy((ivp_controller_phantom_t *)obj->controller_phantom);
    ivp_controller_phantom_t *c = controller_phantom_create(obj, tmpl);
    obj->controller_phantom = c;
    return c;
}

ivp_controller_phantom_t *ivp_object_get_controller_phantom(const ivp_object_t *obj)
{
    return obj ? (ivp_controller_phantom_t *)obj->controller_phantom : NULL;
}
