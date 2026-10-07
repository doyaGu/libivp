/* ivp_min_hash.c — IVP_U_Min_Hash port (ivu_min_hash.cxx) */
#include <ivp/containers/ivp_min_hash.h>
#include <ivp/base/ivp_assert.h>
#include <stdint.h>
#include <string.h>

static inline unsigned int min_hash_index(const ivp_min_hash_t *h, const void *key) {
    /* IVP_U_Min_Hash::hash_index (64-bit platform), on intp bit patterns */
    uint64_t k = (uint64_t)(uintptr_t)key;
    unsigned int x = (unsigned int)(k * 101u);
    unsigned int y = (x >> 8) + (unsigned int)(k * 1001u);
#if UINTPTR_MAX > 0xFFFFFFFFu
    unsigned int z = (unsigned int)((int64_t)k >> 32);
    y ^= z * 1009u;
#endif
    return y & (h->size - 1);
}

ivp_result_t ivp_min_hash_init(ivp_min_hash_t *h, const ivp_allocator_t *alloc, int size) {
    memset(h, 0, sizeof(*h));
    if (!alloc) alloc = ivp_default_allocator();
    IVP_IF(1) {
        int i;
        for (i = 0; i < 32; i++) {
            if ((1u << i) == (unsigned int)size) break;
        }
        IVP_ASSERT(i != 32); /* must be power 2 */
    }
    if (size <= 0 || (size & (size - 1)) != 0) return IVP_ERR_INVALID_ARG; /* must be 2^x */
    h->alloc = alloc;
    h->size = (unsigned int)size;
    h->elems = (ivp_min_hash_elem_t **)ivp_alloc(alloc, sizeof(void *) * (size_t)size);
    h->stadel = (ivp_min_hash_elem_t **)ivp_alloc(alloc, sizeof(void *) * (size_t)size * 2);
    if (!h->elems || !h->stadel) {
        ivp_min_hash_destroy(h);
        return IVP_ERR_OUT_OF_MEMORY;
    }
    memset(h->elems, 0, sizeof(void *) * (size_t)size);
    memset(h->stadel, 0, sizeof(void *) * (size_t)size * 2);
    h->min_per_array_pos = h->stadel + size;
    h->counter = 0;
    h->sort_counter = 1;
    return IVP_OK;
}

void ivp_min_hash_destroy(ivp_min_hash_t *h) {
    if (h->elems) {
        for (unsigned int i = 0; i < h->size; i++) {
            ivp_min_hash_elem_t *next_elem;
            for (ivp_min_hash_elem_t *el = h->elems[i]; el; el = next_elem) {
                next_elem = el->next;
                ivp_free(h->alloc, el, sizeof(*el));
            }
        }
        ivp_free(h->alloc, h->elems, sizeof(void *) * h->size);
    }
    if (h->stadel) ivp_free(h->alloc, h->stadel, sizeof(void *) * h->size * 2);
    h->elems = NULL;
    h->stadel = NULL;
    h->min_per_array_pos = NULL;
    h->counter = 0;
    h->sort_counter = 1;
}

/* a new minimum element at a given hash index was set, so update stadel */
static void min_added_at_index(ivp_min_hash_t *h, ivp_min_hash_elem_t *elem, unsigned int i) {
    double val = elem->value;
    i += h->size;
    h->stadel[i] = elem; /* transport new min element from bottom to top */
    for (unsigned int height = h->size >> 1; height > 0; height >>= 1) {
        i = i >> 1;
        if (h->stadel[i] && val >= h->stadel[i]->value) {
            break;
        }
        h->stadel[i] = elem;
    }
}

static inline bool elem_less(const ivp_min_hash_elem_t *a, const ivp_min_hash_elem_t *b) {
    return a->value < b->value || (a->value == b->value && a->cmp_index < b->cmp_index);
}

/* the minimum element at an array position has been removed */
static void min_removed_at_index(ivp_min_hash_t *h, ivp_min_hash_elem_t *elem, unsigned int i) {
    { /* search hash index position for new minimal element */
        ivp_min_hash_elem_t *new_min = h->elems[i];
        if (new_min) {
            for (ivp_min_hash_elem_t *el = new_min->next; el; el = el->next) {
                if (elem_less(el, new_min)) new_min = el;
            }
        }
        h->min_per_array_pos[i] = new_min;
    }
    /* go from bottom to top and trace path of elem */
    i += h->size;
    for (unsigned int height = h->size >> 1; height > 0; height >>= 1) {
        i = i >> 1;
        if (elem != h->stadel[i]) break;
        unsigned int j = i * 2;
        if (!h->stadel[j]) {
            h->stadel[i] = h->stadel[j + 1];
        } else if (!h->stadel[j + 1]) {
            h->stadel[i] = h->stadel[j];
        } else if (elem_less(h->stadel[j], h->stadel[j + 1])) {
            h->stadel[i] = h->stadel[j];
        } else {
            h->stadel[i] = h->stadel[j + 1];
        }
    }
}

ivp_result_t ivp_min_hash_add(ivp_min_hash_t *h, void *elem, double val) {
    unsigned int i = min_hash_index(h, elem);
    ivp_min_hash_elem_t *el = (ivp_min_hash_elem_t *)ivp_alloc(h->alloc, sizeof(*el));
    if (!el) return IVP_ERR_OUT_OF_MEMORY;
    el->cmp_index = h->sort_counter++;
    el->elem = elem;
    el->next = h->elems[i];
    h->elems[i] = el;
    el->value = val;
    h->counter++;
    if (!h->min_per_array_pos[i] || val < h->min_per_array_pos[i]->value) {
        min_added_at_index(h, el, i);
    }
    return IVP_OK;
}

ivp_result_t ivp_min_hash_change_value(ivp_min_hash_t *h, void *elem, double val) {
    ivp_min_hash_remove(h, elem);
    return ivp_min_hash_add(h, elem, val);
}

void ivp_min_hash_remove(ivp_min_hash_t *h, void *elem) {
    unsigned int i = min_hash_index(h, elem);
    ivp_min_hash_elem_t *last_el = NULL;
    for (ivp_min_hash_elem_t *el = h->elems[i]; el; el = el->next) {
        if (el->elem == elem) {
            if (last_el) {
                last_el->next = el->next;
            } else {
                h->elems[i] = el->next;
            }
            if (el == h->min_per_array_pos[i]) {
                min_removed_at_index(h, el, i);
            }
            ivp_free(h->alloc, el, sizeof(*el));
            h->counter--;
            return;
        }
        last_el = el;
    }
}

void ivp_min_hash_remove_min(ivp_min_hash_t *h) {
    void *min = ivp_min_hash_find_min_elem(h);
    if (min) ivp_min_hash_remove(h, min);
}

bool ivp_min_hash_is_elem(const ivp_min_hash_t *h, const void *elem) {
    unsigned int i = min_hash_index(h, elem);
    for (const ivp_min_hash_elem_t *el = h->elems[i]; el; el = el->next) {
        if (el->elem == elem) return true;
    }
    return false;
}

void *ivp_min_hash_enumerator_next(ivp_min_hash_enumerator_t *e) {
    if (e->loop_elem) e->loop_elem = e->loop_elem->next;
    while (!e->loop_elem) {
        e->loop_index++;
        if (e->loop_index >= (int)e->min_hash->size) return NULL;
        e->loop_elem = e->min_hash->elems[e->loop_index];
    }
    return e->loop_elem->elem;
}

void *ivp_min_hash_enumerator_next_lt(ivp_min_hash_enumerator_t *e, double max_limit) {
    while (1) {
        if (e->loop_elem) e->loop_elem = e->loop_elem->next;
        while (!e->loop_elem) {
            e->loop_index++;
            if (e->loop_index >= (int)e->min_hash->size) return NULL;
            e->loop_elem = e->min_hash->elems[e->loop_index];
        }
        if (e->loop_elem->value < max_limit) return e->loop_elem->elem;
    }
}
