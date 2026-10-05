/* Private object-collision partition search; the vector owns enumeration. */
#ifndef IVP_COLLISION_DIFF_H
#define IVP_COLLISION_DIFF_H
#include <ivp/base/ivp_allocator.h>
#include <ivp/collision/ivp_collision_core.h>

typedef struct ivp_collision_diff_storage ivp_collision_diff_storage_t;
typedef struct ivp_object *(*ivp_collision_diff_key_fn)(const ivp_collision_t *,
                                                       const struct ivp_object *);
typedef struct ivp_collision_diff {
    ivp_coll_fvector_t *vector;
    int found;
    int indexed_length;
    const struct ivp_object *owner;
    ivp_collision_diff_storage_t *index;
} ivp_collision_diff_t;

/* key_fn must only read stable built-in collisions and return NULL for any
 * user implementation. NULL, duplicates and OOM retain the linear algorithm.
 * Filters with user callbacks must pass NULL: they may change the vector. */
void ivp_collision_diff_begin(ivp_collision_diff_t *diff, ivp_coll_fvector_t *vector,
                              const struct ivp_object *owner, int queries,
                              ivp_collision_diff_storage_t **storage,
                              const ivp_allocator_t *allocator,
                              ivp_collision_diff_key_fn key_fn);
bool ivp_collision_diff_check(ivp_collision_diff_t *diff, struct ivp_object *object);
void ivp_collision_diff_storage_free(ivp_collision_diff_storage_t *storage,
                                     const ivp_allocator_t *allocator);
#endif
