/* ivp_oo_watcher.c — Compound object collision delegator */

#include <ivp/collision/ivp_oo_watcher.h>

#include <string.h>

void ivp_oo_watcher_init(ivp_oo_watcher_t *watcher,
                         ivp_object_t *obj_a,
                         ivp_object_t *obj_b,
                         const ivp_allocator_t *alloc)
{
    if (!watcher) return;
    memset(watcher, 0, sizeof(*watcher));
    watcher->obj_a = obj_a;
    watcher->obj_b = obj_b;
    watcher->active = true;
    ivp_mindist_recursive_init(&watcher->recursive, alloc);
}

void ivp_oo_watcher_destroy(ivp_oo_watcher_t *watcher)
{
    if (!watcher) return;
    ivp_mindist_recursive_destroy(&watcher->recursive);
    watcher->obj_a = NULL;
    watcher->obj_b = NULL;
    watcher->active = false;
}

void ivp_oo_watcher_hull_wakeup(ivp_oo_watcher_t *watcher)
{
    if (!watcher || !watcher->active) return;
    ivp_mindist_recursive_invalidate(&watcher->recursive);
}
