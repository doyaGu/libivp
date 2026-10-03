/* ivp_listener_collision.c — IVP_Listener_Collision registration
 *
 * Per-object listener tables live in the environment
 * (IVP_Cluster_Manager::collision_callback_hash, see ivp_environment.c);
 * the simulation fires them from the impact / friction code.
 */

#include <ivp/events/ivp_listener_collision.h>
#include <ivp/dynamics/ivp_object.h>
#include <string.h>

#include "dynamics/ivp_env_pipeline.h"

/* kinds of ivp_env_fire_collision_private (ivp_environment.c) */
enum { COLL_PRE, COLL_POST, COLL_FR_CREATED, COLL_FR_DELETED, COLL_OBJ_DELETED };

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

/* ── Attach / detach (IVP_Real_Object::add/remove_listener_collision) ──── */

int ivp_object_add_collision_listener(ivp_object_t *obj,
                                      ivp_collision_listener_t *listener)
{
    if (!obj || !listener || !obj->owner) return -1;
    return ivp_env_add_listener_collision_private(obj->owner, obj, listener);
}

int ivp_object_remove_collision_listener(ivp_object_t *obj,
                                         ivp_collision_listener_t *listener)
{
    if (!obj || !listener || !obj->owner) return -1;
    return ivp_env_remove_listener_collision_private(obj->owner, obj, listener);
}

/* ── Manual dispatch to the private listeners of the given objects ─────── */

void ivp_collision_listeners_fire_pre(ivp_object_t *obj_a,
                                      ivp_object_t *obj_b,
                                      const ivp_collision_event_t *event)
{
    if (obj_a && obj_a->owner) ivp_env_fire_collision_private(obj_a->owner, obj_a, COLL_PRE, event);
    if (obj_b && obj_b->owner) ivp_env_fire_collision_private(obj_b->owner, obj_b, COLL_PRE, event);
}

void ivp_collision_listeners_fire_post(ivp_object_t *obj_a,
                                       ivp_object_t *obj_b,
                                       const ivp_collision_event_t *event)
{
    if (obj_a && obj_a->owner) ivp_env_fire_collision_private(obj_a->owner, obj_a, COLL_POST, event);
    if (obj_b && obj_b->owner) ivp_env_fire_collision_private(obj_b->owner, obj_b, COLL_POST, event);
}

void ivp_collision_listeners_fire_friction_created(ivp_object_t *obj_a,
                                                    ivp_object_t *obj_b,
                                                    const ivp_friction_event_t *event)
{
    if (obj_a && obj_a->owner) ivp_env_fire_collision_private(obj_a->owner, obj_a, COLL_FR_CREATED, event);
    if (obj_b && obj_b->owner) ivp_env_fire_collision_private(obj_b->owner, obj_b, COLL_FR_CREATED, event);
}

void ivp_collision_listeners_fire_friction_deleted(ivp_object_t *obj_a,
                                                    ivp_object_t *obj_b,
                                                    const ivp_friction_event_t *event)
{
    if (obj_a && obj_a->owner) ivp_env_fire_collision_private(obj_a->owner, obj_a, COLL_FR_DELETED, event);
    if (obj_b && obj_b->owner) ivp_env_fire_collision_private(obj_b->owner, obj_b, COLL_FR_DELETED, event);
}

void ivp_collision_listeners_fire_object_deleted(ivp_object_t *obj)
{
    if (obj && obj->owner) ivp_env_fire_collision_private(obj->owner, obj, COLL_OBJ_DELETED, (const void *)obj);
}
