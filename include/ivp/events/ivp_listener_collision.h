/* ivp_listener_collision.h — Per-object collision listener
 *
 * Provides callbacks for collision events on specific objects:
 *   - Pre-collision: called before impact resolution
 *   - Post-collision: called after impact resolution
 *   - Friction created/deleted: called when persistent contact changes
 *
 * Listeners are attached to individual objects and invoked only when
 * that object participates in a collision.
 *
 * Ported from ivp_listener_collision.hxx.
 */
#ifndef IVP_LISTENER_COLLISION_H
#define IVP_LISTENER_COLLISION_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Callback enable flags ─────────────────────────────────────────────── */

typedef enum ivp_collision_callback_flags {
    IVP_COLLISION_CB_PRE_COLLISION    = 0x01,
    IVP_COLLISION_CB_POST_COLLISION   = 0x02,
    IVP_COLLISION_CB_FRICTION_CREATED = 0x04,
    IVP_COLLISION_CB_FRICTION_DELETED = 0x08,
    IVP_COLLISION_CB_ALL             = 0x0F
} ivp_collision_callback_flags_t;

/* ── Collision event data ──────────────────────────────────────────────── */

typedef struct ivp_collision_event {
    ivp_object_t      *obj_a;          /* first colliding object */
    ivp_object_t      *obj_b;          /* second colliding object */
    ivp_vec3r_t        contact_point;  /* world-space contact point */
    ivp_vec3r_t        contact_normal; /* world-space normal (A→B) */
    ivp_real_t         closing_speed;  /* relative velocity along normal */
    ivp_real_t         impulse;        /* applied impulse magnitude */
    ivp_real_t         gap_distance;   /* gap at contact (negative = penetration) */
} ivp_collision_event_t;

/* ── Friction event data ───────────────────────────────────────────────── */

typedef struct ivp_friction_event {
    ivp_object_t      *obj_a;
    ivp_object_t      *obj_b;
    ivp_vec3r_t        contact_point;
    ivp_vec3r_t        contact_normal;
    ivp_real_t         normal_force;   /* current normal (vertical) force */
} ivp_friction_event_t;

/* ── Callback function types ───────────────────────────────────────────── */

typedef void (*ivp_collision_listener_fn_t)(void *ctx,
                                           const ivp_collision_event_t *event);

typedef void (*ivp_friction_listener_fn_t)(void *ctx,
                                          const ivp_friction_event_t *event);

/* ── Collision listener ────────────────────────────────────────────────── */

typedef struct ivp_collision_listener {
    ivp_collision_listener_fn_t  pre_collision_fn;
    ivp_collision_listener_fn_t  post_collision_fn;
    ivp_friction_listener_fn_t   friction_created_fn;
    ivp_friction_listener_fn_t   friction_deleted_fn;
    void                        *ctx;
    int                          enabled_callbacks;  /* bitmask of ivp_collision_callback_flags_t */
    bool                         active;
} ivp_collision_listener_t;

/* ── API ───────────────────────────────────────────────────────────────── */

/* Initialize a collision listener with given callback flags. */
void ivp_collision_listener_init(ivp_collision_listener_t *listener,
                                 int callback_flags,
                                 void *ctx);

/* Attach a collision listener to an object.
 * Returns 0 on success, -1 if max listeners reached.
 * The listener must remain valid for the lifetime of the attachment. */
int ivp_object_add_collision_listener(ivp_object_t *obj,
                                      ivp_collision_listener_t *listener);

/* Remove a collision listener from an object.
 * Returns 0 on success, -1 if not found. */
int ivp_object_remove_collision_listener(ivp_object_t *obj,
                                         ivp_collision_listener_t *listener);

/* ── Dispatch (called by collision/impact subsystem) ───────────────────── */

/* Fire pre-collision callbacks for both objects in a collision pair. */
void ivp_collision_listeners_fire_pre(ivp_object_t *obj_a,
                                      ivp_object_t *obj_b,
                                      const ivp_collision_event_t *event);

/* Fire post-collision callbacks for both objects. */
void ivp_collision_listeners_fire_post(ivp_object_t *obj_a,
                                       ivp_object_t *obj_b,
                                       const ivp_collision_event_t *event);

/* Fire friction created callbacks. */
void ivp_collision_listeners_fire_friction_created(ivp_object_t *obj_a,
                                                    ivp_object_t *obj_b,
                                                    const ivp_friction_event_t *event);

/* Fire friction deleted callbacks. */
void ivp_collision_listeners_fire_friction_deleted(ivp_object_t *obj_a,
                                                    ivp_object_t *obj_b,
                                                    const ivp_friction_event_t *event);

#ifdef __cplusplus
}
#endif

#endif /* IVP_LISTENER_COLLISION_H */
