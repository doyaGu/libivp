/* ivp_listener_collision.c — Per-object collision listener implementation */

#include <ivp/events/ivp_listener_collision.h>
#include <ivp/dynamics/ivp_object.h>
#include <string.h>

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_collision_listener_init(ivp_collision_listener_t *listener,
                                 int callback_flags,
                                 void *ctx)
{
    memset(listener, 0, sizeof(*listener));
    listener->enabled_callbacks = callback_flags;
    listener->ctx = ctx;
    listener->active = true;
}

/* ── Attach / detach ───────────────────────────────────────────────────── */

int ivp_object_add_collision_listener(ivp_object_t *obj,
                                      ivp_collision_listener_t *listener)
{
    if (!obj || !listener) return -1;

    for (int i = 0; i < IVP_MAX_COLLISION_LISTENERS_PER_OBJECT; i++) {
        if (!obj->collision_listeners[i]) {
            obj->collision_listeners[i] = listener;
            return 0;
        }
    }
    return -1;  /* no free slot */
}

int ivp_object_remove_collision_listener(ivp_object_t *obj,
                                         ivp_collision_listener_t *listener)
{
    if (!obj || !listener) return -1;

    for (int i = 0; i < IVP_MAX_COLLISION_LISTENERS_PER_OBJECT; i++) {
        if (obj->collision_listeners[i] == listener) {
            obj->collision_listeners[i] = NULL;
            return 0;
        }
    }
    return -1;  /* not found */
}

/* ── Dispatch helpers ──────────────────────────────────────────────────── */

static void fire_pre_for_object(ivp_object_t *obj,
                                const ivp_collision_event_t *event)
{
    if (!obj) return;
    for (int i = 0; i < IVP_MAX_COLLISION_LISTENERS_PER_OBJECT; i++) {
        ivp_collision_listener_t *l = obj->collision_listeners[i];
        if (!l || !l->active) continue;
        if (!(l->enabled_callbacks & IVP_COLLISION_CB_PRE_COLLISION)) continue;
        if (l->pre_collision_fn) {
            l->pre_collision_fn(l->ctx, event);
        }
    }
}

static void fire_post_for_object(ivp_object_t *obj,
                                 const ivp_collision_event_t *event)
{
    if (!obj) return;
    for (int i = 0; i < IVP_MAX_COLLISION_LISTENERS_PER_OBJECT; i++) {
        ivp_collision_listener_t *l = obj->collision_listeners[i];
        if (!l || !l->active) continue;
        if (!(l->enabled_callbacks & IVP_COLLISION_CB_POST_COLLISION)) continue;
        if (l->post_collision_fn) {
            l->post_collision_fn(l->ctx, event);
        }
    }
}

static void fire_friction_created_for_object(ivp_object_t *obj,
                                              const ivp_friction_event_t *event)
{
    if (!obj) return;
    for (int i = 0; i < IVP_MAX_COLLISION_LISTENERS_PER_OBJECT; i++) {
        ivp_collision_listener_t *l = obj->collision_listeners[i];
        if (!l || !l->active) continue;
        if (!(l->enabled_callbacks & IVP_COLLISION_CB_FRICTION_CREATED)) continue;
        if (l->friction_created_fn) {
            l->friction_created_fn(l->ctx, event);
        }
    }
}

static void fire_friction_deleted_for_object(ivp_object_t *obj,
                                              const ivp_friction_event_t *event)
{
    if (!obj) return;
    for (int i = 0; i < IVP_MAX_COLLISION_LISTENERS_PER_OBJECT; i++) {
        ivp_collision_listener_t *l = obj->collision_listeners[i];
        if (!l || !l->active) continue;
        if (!(l->enabled_callbacks & IVP_COLLISION_CB_FRICTION_DELETED)) continue;
        if (l->friction_deleted_fn) {
            l->friction_deleted_fn(l->ctx, event);
        }
    }
}

/* ── Public dispatch ───────────────────────────────────────────────────── */

void ivp_collision_listeners_fire_pre(ivp_object_t *obj_a,
                                      ivp_object_t *obj_b,
                                      const ivp_collision_event_t *event)
{
    fire_pre_for_object(obj_a, event);
    fire_pre_for_object(obj_b, event);
}

void ivp_collision_listeners_fire_post(ivp_object_t *obj_a,
                                       ivp_object_t *obj_b,
                                       const ivp_collision_event_t *event)
{
    fire_post_for_object(obj_a, event);
    fire_post_for_object(obj_b, event);
}

void ivp_collision_listeners_fire_friction_created(ivp_object_t *obj_a,
                                                    ivp_object_t *obj_b,
                                                    const ivp_friction_event_t *event)
{
    fire_friction_created_for_object(obj_a, event);
    fire_friction_created_for_object(obj_b, event);
}

void ivp_collision_listeners_fire_friction_deleted(ivp_object_t *obj_a,
                                                    ivp_object_t *obj_b,
                                                    const ivp_friction_event_t *event)
{
    fire_friction_deleted_for_object(obj_a, event);
    fire_friction_deleted_for_object(obj_b, event);
}
