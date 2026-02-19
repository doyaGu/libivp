/* ivp_collision_delegator.h — Collision delegator root (reference alignment)
 *
 * In the reference IVP, collision orchestration uses a delegator pattern:
 *   - IVP_Collision_Delegator_Root instances are registered on the environment
 *   - During broadphase, when two objects overlap, each registered root is
 *     asked to create a collision checker for that pair
 *   - The root can return NULL (pair not handled) or a collision object
 *
 * This C17 port exposes the vtable pattern via function pointers.
 * The default root (ivp_collision_delegator_root_mindist) creates
 * persistent mindist entries for overlapping convex pairs.
 *
 * Ported from IVP_Collision_Delegator_Root in ivp_collision.hxx.
 */
#ifndef IVP_COLLISION_DELEGATOR_H
#define IVP_COLLISION_DELEGATOR_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_forward_decls.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Delegator root vtable ─────────────────────────────────────────────── */

/* Forward declaration for the delegator root struct itself. */
typedef struct ivp_collision_delegator_root ivp_collision_delegator_root_t;

/* Called when two objects begin overlapping in the broadphase.
 * The delegator should create any collision tracking state it needs.
 * Return true if the pair was accepted, false to pass to next delegator. */
typedef bool (*ivp_delegate_collisions_fn_t)(
    ivp_collision_delegator_root_t *self,
    ivp_object_t *obj_a,
    ivp_object_t *obj_b);

/* Called when an object is removed from collision detection. */
typedef void (*ivp_object_removed_from_collision_fn_t)(
    ivp_collision_delegator_root_t *self,
    ivp_object_t *obj);

/* Called when the environment is being destroyed. */
typedef void (*ivp_delegator_environment_deleted_fn_t)(
    ivp_collision_delegator_root_t *self,
    struct ivp_environment *env);

struct ivp_collision_delegator_root {
    ivp_delegate_collisions_fn_t               delegate_collisions;
    ivp_object_removed_from_collision_fn_t     object_removed;
    ivp_delegator_environment_deleted_fn_t     environment_deleted;
    void                                      *ctx;   /* user data */
};

/* Initialize a delegator root with the given vtable callbacks. */
static inline void ivp_collision_delegator_root_init(
    ivp_collision_delegator_root_t *root,
    ivp_delegate_collisions_fn_t    delegate_fn,
    ivp_object_removed_from_collision_fn_t removed_fn,
    ivp_delegator_environment_deleted_fn_t env_deleted_fn,
    void *ctx)
{
    root->delegate_collisions = delegate_fn;
    root->object_removed = removed_fn;
    root->environment_deleted = env_deleted_fn;
    root->ctx = ctx;
}

/* ── Delegator list on environment ─────────────────────────────────────── */

#define IVP_MAX_COLLISION_DELEGATOR_ROOTS 8

#ifdef __cplusplus
}
#endif

#endif /* IVP_COLLISION_DELEGATOR_H */
