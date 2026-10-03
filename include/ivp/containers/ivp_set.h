/* ivp_set.h -- set of pointers (IVP_U_Set / IVP_U_Set_Active, ivu_set.hxx)
 *
 * Built on ivp_vhash_t with pointer equality and the reference
 * IVP_VHash::fast_hash_index of the pointer.  The active variant notifies
 * listeners (most recently added first) on add / remove / destroy.
 */
#ifndef IVP_SET_H
#define IVP_SET_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/containers/ivp_vhash.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_set {
    ivp_vhash_t hash;
} ivp_set_t;

void  ivp_set_init(ivp_set_t *s, const ivp_allocator_t *alloc, int init_size); /* power of 2 */
void  ivp_set_destroy(ivp_set_t *s);
void  ivp_set_add(ivp_set_t *s, void *elem);          /* add_element (no duplicate check) */
void  ivp_set_remove(ivp_set_t *s, void *elem);       /* remove_element */
void *ivp_set_find(const ivp_set_t *s, const void *elem); /* NULL or elem */
void  ivp_set_install(ivp_set_t *s, void *elem);      /* add if not present */
static inline int ivp_set_count(const ivp_set_t *s) { return ivp_vhash_count(&s->hash); }

/* IVP_U_Set_Enumerator: walks the hash table from the last slot down */
typedef struct ivp_set_enumerator {
    int index;
} ivp_set_enumerator_t;

static inline void ivp_set_enumerator_init(ivp_set_enumerator_t *e, const ivp_set_t *s) {
    e->index = ivp_vhash_capacity(&s->hash) - 1;
}
void *ivp_set_enumerator_next(ivp_set_enumerator_t *e, const ivp_set_t *s);

/* ── IVP_U_Set_Active ───────────────────────────────────────────────────── */

typedef struct ivp_set_active ivp_set_active_t;

typedef struct ivp_set_active_listener {
    void (*element_added)(void *ctx, ivp_set_active_t *set, void *elem);
    void (*element_removed)(void *ctx, ivp_set_active_t *set, void *elem);
    void (*pset_is_going_to_be_deleted)(void *ctx, ivp_set_active_t *set);
    void *ctx;
} ivp_set_active_listener_t;

struct ivp_set_active {
    ivp_set_t                   set;
    const ivp_set_active_listener_t **listeners;
    int                         n_listeners;
    int                         cap_listeners;
    const ivp_allocator_t      *alloc;
};

void ivp_set_active_init(ivp_set_active_t *s, const ivp_allocator_t *alloc, int init_size);
/* notifies pset_is_going_to_be_deleted (elements are not removed) */
void ivp_set_active_destroy(ivp_set_active_t *s);
void ivp_set_active_add(ivp_set_active_t *s, void *elem);
void ivp_set_active_install(ivp_set_active_t *s, void *elem);
void ivp_set_active_remove(ivp_set_active_t *s, void *elem);
bool ivp_set_active_add_listener(ivp_set_active_t *s, const ivp_set_active_listener_t *l);
void ivp_set_active_remove_listener(ivp_set_active_t *s, const ivp_set_active_listener_t *l);

#ifdef __cplusplus
}
#endif

#endif /* IVP_SET_H */
