/* ivp_set.c -- IVP_U_Set / IVP_U_Set_Active port (ivu_set.hxx) */
#include <ivp/containers/ivp_set.h>
#include <ivp/utility/ivp_hash.h>
#include <ivp/base/ivp_assert.h>
#include <string.h>

static bool ptr_compare(const void *e0, const void *e1) {
    return e0 == e1;
}

static inline uint32_t elem_to_index(const void *elem) {
    return ivp_vhash_fast_index((uintptr_t)elem);
}

void ivp_set_init(ivp_set_t *s, const ivp_allocator_t *alloc, int init_size) {
    if (!alloc) alloc = ivp_default_allocator();
    ivp_vhash_init(&s->hash, alloc, init_size, ptr_compare);
}

void ivp_set_destroy(ivp_set_t *s) {
    ivp_vhash_destroy(&s->hash);
}

void ivp_set_add(ivp_set_t *s, void *elem) {
    ivp_vhash_add(&s->hash, elem, elem_to_index(elem));
}

void ivp_set_remove(ivp_set_t *s, void *elem) {
    ivp_vhash_remove(&s->hash, elem, elem_to_index(elem));
}

void *ivp_set_find(const ivp_set_t *s, const void *elem) {
    return ivp_vhash_find(&s->hash, elem, elem_to_index(elem));
}

void ivp_set_install(ivp_set_t *s, void *elem) {
    uint32_t index = elem_to_index(elem);
    if (ivp_vhash_find(&s->hash, elem, index)) return;
    ivp_vhash_add(&s->hash, elem, index);
}

void *ivp_set_enumerator_next(ivp_set_enumerator_t *e, const ivp_set_t *s) {
    while (1) {
        if (e->index < 0) return NULL;
        void *res = ivp_vhash_element_at(&s->hash, e->index--);
        if (res) return res;
    }
}

/* ── IVP_U_Set_Active ───────────────────────────────────────────────────── */

void ivp_set_active_init(ivp_set_active_t *s, const ivp_allocator_t *alloc, int init_size) {
    if (!alloc) alloc = ivp_default_allocator();
    memset(s, 0, sizeof(*s));
    s->alloc = alloc;
    ivp_set_init(&s->set, alloc, init_size);
}

void ivp_set_active_destroy(ivp_set_active_t *s) {
    for (int i = s->n_listeners - 1; i >= 0; i--) {
        const ivp_set_active_listener_t *l = s->listeners[i];
        if (l->pset_is_going_to_be_deleted) l->pset_is_going_to_be_deleted(l->ctx, s);
    }
    if (s->listeners) {
        ivp_free(s->alloc, (void *)s->listeners, (size_t)s->cap_listeners * sizeof(*s->listeners));
    }
    s->listeners = NULL;
    s->n_listeners = s->cap_listeners = 0;
    ivp_set_destroy(&s->set);
}

static void notify_added(ivp_set_active_t *s, void *elem) {
    for (int i = s->n_listeners - 1; i >= 0; i--) {
        const ivp_set_active_listener_t *l = s->listeners[i];
        if (l->element_added) l->element_added(l->ctx, s, elem);
    }
}

void ivp_set_active_add(ivp_set_active_t *s, void *elem) {
    ivp_set_add(&s->set, elem);
    notify_added(s, elem);
}

void ivp_set_active_install(ivp_set_active_t *s, void *elem) {
    if (ivp_set_find(&s->set, elem)) return;
    ivp_set_add(&s->set, elem);
    notify_added(s, elem);
}

void ivp_set_active_remove(ivp_set_active_t *s, void *elem) {
    ivp_set_remove(&s->set, elem);
    for (int i = s->n_listeners - 1; i >= 0; i--) {
        const ivp_set_active_listener_t *l = s->listeners[i];
        if (l->element_removed) l->element_removed(l->ctx, s, elem);
    }
}

bool ivp_set_active_add_listener(ivp_set_active_t *s, const ivp_set_active_listener_t *l) {
    if (s->n_listeners >= s->cap_listeners) {
        int nc = s->cap_listeners ? s->cap_listeners * 2 : 4;
        const ivp_set_active_listener_t **n = (const ivp_set_active_listener_t **)ivp_realloc(
            s->alloc, (void *)s->listeners, (size_t)s->cap_listeners * sizeof(*n),
            (size_t)nc * sizeof(*n));
        if (!n) return false;
        s->listeners = n;
        s->cap_listeners = nc;
    }
    s->listeners[s->n_listeners++] = l;
    return true;
}

void ivp_set_active_remove_listener(ivp_set_active_t *s, const ivp_set_active_listener_t *l) {
    /* IVP_U_Vector::remove: last occurrence, order preserved */
    int index;
    for (index = s->n_listeners - 1; index >= 0; index--) {
        if (s->listeners[index] == l) break;
    }
    if (index < 0) {
        IVP_ASSERT(index >= 0);
        return;
    }
    s->n_listeners--;
    for (int j = index; j < s->n_listeners; j++) s->listeners[j] = s->listeners[j + 1];
}
