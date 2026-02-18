/* ivp_collision_filter.h -- Collision pair filtering
 *
 * Controls which object pairs should be checked for collision.
 * Built-in strategies plus custom callback support:
 *
 *   1. Group filter: objects sharing the same non-zero collision_group skip collision.
 *   2. Layer mask filter: objects must share at least one layer bit.
 *   3. Exclusive pair filter: specific object pairs are excluded via hash.
 *   4. Meta filter: AND-composite of multiple filters.
 *   5. Custom callback: user-provided function decides per pair.
 *
 * Ported from ivp_collision_filter.hxx.
 */
#ifndef IVP_COLLISION_FILTER_H
#define IVP_COLLISION_FILTER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ivp_object;

/* ── Filter callback type ──────────────────────────────────────────────── */

/* Return true if collision between obj_a and obj_b should be detected.
 * Return false to skip collision for this pair. */
typedef bool (*ivp_collision_check_fn)(const struct ivp_object *obj_a,
                                        const struct ivp_object *obj_b,
                                        void *user_data);

/* ── Filter types ──────────────────────────────────────────────────────── */

typedef enum ivp_collision_filter_type {
    IVP_CFILTER_ALLOW_ALL   = 0,  /* no filtering */
    IVP_CFILTER_GROUP       = 1,  /* skip same-group pairs */
    IVP_CFILTER_LAYER_MASK  = 2,  /* require shared layer bit */
    IVP_CFILTER_CALLBACK    = 3,  /* custom callback */
    IVP_CFILTER_META        = 4   /* AND-composite of multiple filters */
} ivp_collision_filter_type_t;

/* ── Collision filter ──────────────────────────────────────────────────── */

typedef struct ivp_collision_filter {
    ivp_collision_filter_type_t type;
    ivp_collision_check_fn      check_fn;
    void                       *user_data;
} ivp_collision_filter_t;

/* ── Constructors ──────────────────────────────────────────────────────── */

/* Create a filter that allows all collisions (no filtering). */
static inline ivp_collision_filter_t ivp_collision_filter_allow_all(void) {
    ivp_collision_filter_t f;
    f.type = IVP_CFILTER_ALLOW_ALL;
    f.check_fn = NULL;
    f.user_data = NULL;
    return f;
}

/* Create a filter with a custom callback. */
static inline ivp_collision_filter_t ivp_collision_filter_callback(
    ivp_collision_check_fn fn, void *user_data)
{
    ivp_collision_filter_t f;
    f.type = IVP_CFILTER_CALLBACK;
    f.check_fn = fn;
    f.user_data = user_data;
    return f;
}

/* ── Check function ────────────────────────────────────────────────────── */

/* Returns true if collision between the two objects should be detected. */
static inline bool ivp_collision_filter_check(
    const ivp_collision_filter_t *filter,
    const struct ivp_object *obj_a,
    const struct ivp_object *obj_b)
{
    if (filter->type == IVP_CFILTER_ALLOW_ALL) return true;
    if (filter->check_fn) {
        return filter->check_fn(obj_a, obj_b, filter->user_data);
    }
    return true;
}

/* ── Built-in filter constructors (implemented in ivp_collision_filter.c) ── */

/* Group filter: skip if both objects share same non-zero collision_group */
ivp_collision_filter_t ivp_collision_filter_group(void);

/* Layer mask filter: skip if (a->layer_mask & b->layer_mask) == 0 */
ivp_collision_filter_t ivp_collision_filter_layer_mask(void);

/* ── Exclusive pair filter (hash-based) ────────────────────────────────── */

typedef struct ivp_exclusive_pair {
    uintptr_t obj_a;
    uintptr_t obj_b;
} ivp_exclusive_pair_t;

typedef struct ivp_exclusive_pair_filter {
    ivp_collision_filter_t base;
    ivp_exclusive_pair_t *pairs;
    int count;
    int capacity;
    const ivp_allocator_t *alloc;
} ivp_exclusive_pair_filter_t;

ivp_result_t ivp_exclusive_pair_filter_init(ivp_exclusive_pair_filter_t *f,
                                              const ivp_allocator_t *alloc);
void ivp_exclusive_pair_filter_destroy(ivp_exclusive_pair_filter_t *f);
void ivp_exclusive_pair_filter_disable(ivp_exclusive_pair_filter_t *f,
                                         const struct ivp_object *a,
                                         const struct ivp_object *b);
void ivp_exclusive_pair_filter_enable(ivp_exclusive_pair_filter_t *f,
                                        const struct ivp_object *a,
                                        const struct ivp_object *b);

/* ── Meta filter (AND-composite) ───────────────────────────────────────── */

typedef struct ivp_meta_collision_filter {
    ivp_collision_filter_t base;
    ivp_collision_filter_t *filters;
    int count;
} ivp_meta_collision_filter_t;

/* Create a meta filter that requires ALL sub-filters to pass.
 * The caller owns the meta struct and filters array — both must remain valid
 * for the lifetime of the returned filter. */
ivp_collision_filter_t ivp_collision_filter_meta(ivp_meta_collision_filter_t *meta,
                                                  ivp_collision_filter_t *filters,
                                                  int count);

#ifdef __cplusplus
}
#endif

#endif /* IVP_COLLISION_FILTER_H */
