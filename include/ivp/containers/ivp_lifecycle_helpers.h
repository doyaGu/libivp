/* ivp_lifecycle_helpers.h — Shared element lifecycle dispatch for containers
 *
 * Static inline wrappers that null-check typeinfo/lifecycle/callback
 * before dispatching. Used by ivp_array, ivp_hashmap, ivp_minheap,
 * and any container that participates in element lifecycle.
 */
#ifndef IVP_LIFECYCLE_HELPERS_H
#define IVP_LIFECYCLE_HELPERS_H

#include <ivp/containers/ivp_elem_typeinfo.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline void ivp_lifecycle_init(const ivp_elem_typeinfo_t *t, void *ctx, void *elem) {
    if (t && t->lifecycle && t->lifecycle->init && elem) {
        t->lifecycle->init(elem, ctx);
    }
}

static inline void ivp_lifecycle_reset(const ivp_elem_typeinfo_t *t, void *ctx, void *elem) {
    if (t && t->lifecycle && t->lifecycle->reset && elem) {
        t->lifecycle->reset(elem, ctx);
    }
}

static inline void ivp_lifecycle_dispose(const ivp_elem_typeinfo_t *t, void *ctx, void *elem) {
    if (t && t->lifecycle && t->lifecycle->dispose && elem) {
        t->lifecycle->dispose(elem, ctx);
    }
}

static inline void ivp_lifecycle_copy(const ivp_elem_typeinfo_t *t, void *ctx,
                                       void *elem, int src_index, int dst_index) {
    if (t && t->lifecycle && t->lifecycle->copy && elem) {
        t->lifecycle->copy(elem, src_index, dst_index, ctx);
    }
}

static inline void ivp_lifecycle_move(const ivp_elem_typeinfo_t *t, void *ctx,
                                       void *elem, int src_index, int dst_index) {
    if (t && t->lifecycle && t->lifecycle->move && elem) {
        t->lifecycle->move(elem, src_index, dst_index, ctx);
    }
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_LIFECYCLE_HELPERS_H */
