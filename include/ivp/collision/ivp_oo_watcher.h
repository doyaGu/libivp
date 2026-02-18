/* ivp_oo_watcher.h — Compound object collision delegator */
#ifndef IVP_OO_WATCHER_H
#define IVP_OO_WATCHER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/collision/ivp_mindist_recursive.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_oo_watcher {
    ivp_object_t *obj_a;
    ivp_object_t *obj_b;
    ivp_mindist_recursive_t recursive;
    bool active;
} ivp_oo_watcher_t;

void ivp_oo_watcher_init(ivp_oo_watcher_t *watcher,
                         ivp_object_t *obj_a,
                         ivp_object_t *obj_b,
                         const ivp_allocator_t *alloc);
void ivp_oo_watcher_destroy(ivp_oo_watcher_t *watcher);

void ivp_oo_watcher_hull_wakeup(ivp_oo_watcher_t *watcher);

#ifdef __cplusplus
}
#endif

#endif /* IVP_OO_WATCHER_H */
