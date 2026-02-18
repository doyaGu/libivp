/* ivp_vhash.c -- Virtual hash implementation
 *
 * Ported from ivu_vhash.cxx (Ipion Software GmbH).
 * Open-addressing with ordered linear probing.
 */
#include <ivp/containers/ivp_vhash.h>
#include <ivp/base/ivp_assert.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  ivp_vhash_t — element-is-key variant
 * ═══════════════════════════════════════════════════════════════════════════ */

static void ivp_vhash_rehash(ivp_vhash_t *h, int new_size) {
    ivp_vhash_elem_t *old = h->elems;
    int old_size = h->size_mm + 1;
    int old_nelems = h->nelems;

    size_t bytes = (size_t)new_size * sizeof(ivp_vhash_elem_t);
    ivp_vhash_elem_t *new_elems = (ivp_vhash_elem_t *)ivp_alloc(h->alloc, bytes);
    if (!new_elems) return;
    memset(new_elems, 0, bytes);

    h->size_mm = new_size - 1;
    h->elems = new_elems;
    h->nelems = 0;

    for (int i = 0; i < old_size; i++) {
        if (old[i].elem) {
            ivp_vhash_add(h, old[i].elem, old[i].hash_index);
        }
    }
    h->nelems = old_nelems;

    ivp_free(h->alloc, old, (size_t)old_size * sizeof(ivp_vhash_elem_t));
}

void ivp_vhash_init(ivp_vhash_t *h, const ivp_allocator_t *alloc,
                     int capacity, ivp_vhash_compare_fn_t compare) {
    if (!h) return;
    h->elems = NULL;
    h->size_mm = -1;
    h->nelems = 0;
    h->compare = compare;
    h->alloc = alloc ? alloc : ivp_default_allocator();

    IVP_ASSERT(compare != NULL);
    IVP_ASSERT(capacity > 0 && (capacity & (capacity - 1)) == 0);
    if (!compare || capacity <= 0 || (capacity & (capacity - 1)) != 0) return;

    h->size_mm = capacity - 1;
    size_t bytes = (size_t)capacity * sizeof(ivp_vhash_elem_t);
    h->elems = (ivp_vhash_elem_t *)ivp_alloc(h->alloc, bytes);
    if (!h->elems) {
        h->size_mm = -1;
        return;
    }
    memset(h->elems, 0, bytes);
}

void ivp_vhash_destroy(ivp_vhash_t *h) {
    if (h->elems) {
        ivp_free(h->alloc, h->elems, (size_t)(h->size_mm + 1) * sizeof(ivp_vhash_elem_t));
        h->elems = NULL;
    }
    h->size_mm = -1;
    h->nelems = 0;
}

void ivp_vhash_add(ivp_vhash_t *h, const void *elem, uint32_t hash_index) {
    if (!h || !h->elems || h->size_mm < 0) return;
    if (h->nelems + h->nelems > h->size_mm) {
        ivp_vhash_rehash(h, (h->size_mm + 1) * 2);
        /* If rehash failed (OOM), table may be full — drop insert to avoid infinite loop */
        if (h->nelems + h->nelems > h->size_mm) return;
    }

    int capacity = h->size_mm + 1;
    int index = (int)(hash_index & (uint32_t)h->size_mm);
    int pos = index;
    h->nelems++;

    for (int searched = 0; searched < capacity; searched++, pos = (pos + 1) & h->size_mm) {
        ivp_vhash_elem_t *e = &h->elems[pos];
        if (!e->elem) break;
        int e_index = (int)(e->hash_index & (uint32_t)h->size_mm);
        if (index >= e_index) continue;

        /* Swap: current element goes further, displaced element takes this slot */
        const void *h_e = e->elem;
        uint32_t h_i = e->hash_index;
        e->elem = elem;
        e->hash_index = hash_index;
        elem = h_e;
        hash_index = h_i;
        index = e_index;
    }
    h->elems[pos].elem = elem;
    h->elems[pos].hash_index = hash_index;
}

void *ivp_vhash_find(const ivp_vhash_t *h, const void *elem, uint32_t hash_index) {
    if (!h || !h->elems || h->size_mm < 0 || !h->compare) return NULL;
    int pos = (int)(hash_index & (uint32_t)h->size_mm);

    for (int searched = 0; searched < h->size_mm + 1; searched++, pos = (pos + 1) & h->size_mm) {
        const ivp_vhash_elem_t *e = &h->elems[pos];
        if (!e->elem) break;
        if ((e->hash_index | IVP_VHASH_TOUCH_BIT) != hash_index) continue;
        if (h->compare(e->elem, elem)) {
            return (void *)e->elem;
        }
    }
    return NULL;
}

void *ivp_vhash_remove(ivp_vhash_t *h, const void *elem, uint32_t hash_index) {
    if (!h || !h->elems || h->size_mm < 0 || !h->compare) return NULL;
    int pos = (int)(hash_index & (uint32_t)h->size_mm);

    /* Find the element */
    ivp_vhash_elem_t *e = NULL;
    for (int searched = 0; searched < h->size_mm + 1; searched++, pos = (pos + 1) & h->size_mm) {
        e = &h->elems[pos];
        if (!e->elem) return NULL;
        if ((e->hash_index | IVP_VHASH_TOUCH_BIT) != hash_index) continue;
        if (h->compare(e->elem, elem)) break;
    }
    if (!e || !e->elem || !h->compare(e->elem, elem)) return NULL;

    const void *removed = e->elem;
    h->nelems--;
    int last_pos = pos;

    int ideal_pos_of_last = h->size_mm;
    if (h->elems[h->size_mm].elem) {
        ideal_pos_of_last = (int)(h->elems[h->size_mm].hash_index & (uint32_t)h->size_mm);
    }

    /* Shift subsequent elements to fill the gap */
    pos = (pos + 1) & h->size_mm;
    for (int searched = 0; searched < h->size_mm + 1; searched++, pos = (pos + 1) & h->size_mm) {
        ivp_vhash_elem_t *en = &h->elems[pos];
        if (!en->elem) break;
        int ideal_pos = (int)(en->hash_index & (uint32_t)h->size_mm);

        if (pos > last_pos) {
            if (ideal_pos < ideal_pos_of_last) {
                if (ideal_pos > last_pos) continue;
            } else if (ideal_pos == ideal_pos_of_last) {
                if (ideal_pos <= pos) {
                    if (ideal_pos > last_pos) continue;
                }
            }
        } else {
            if (last_pos != h->size_mm) break;
            if (ideal_pos < ideal_pos_of_last) continue;
        }

        h->elems[last_pos] = *en;
        last_pos = pos;
    }
    h->elems[last_pos].elem = NULL;
    h->elems[last_pos].hash_index = 0;

    return (void *)removed;
}

void *ivp_vhash_touch(ivp_vhash_t *h, const void *elem, uint32_t hash_index) {
    if (!h || !h->elems || h->size_mm < 0 || !h->compare) return NULL;
    int pos = (int)(hash_index & (uint32_t)h->size_mm);

    for (int searched = 0; searched < h->size_mm + 1; searched++, pos = (pos + 1) & h->size_mm) {
        ivp_vhash_elem_t *e = &h->elems[pos];
        if (!e->elem) break;
        if ((e->hash_index | IVP_VHASH_TOUCH_BIT) != hash_index) continue;
        if (h->compare(e->elem, elem)) {
            e->hash_index |= IVP_VHASH_TOUCH_BIT;
            return (void *)e->elem;
        }
    }
    return NULL;
}

void ivp_vhash_untouch_all(ivp_vhash_t *h) {
    if (!h || !h->elems || h->size_mm < 0) return;
    for (int i = h->size_mm; i >= 0; i--) {
        h->elems[i].hash_index &= (IVP_VHASH_TOUCH_BIT - 1);
    }
}

void ivp_vhash_gc(ivp_vhash_t *h, int preferred_size) {
    if (!h || !h->elems || h->size_mm < 0) return;
    if (preferred_size < h->nelems) return;
    int new_size = h->size_mm + 1;
    while (preferred_size * 2 + 1 < new_size) {
        new_size /= 2;
    }
    ivp_vhash_rehash(h, new_size);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  ivp_vhash_store_t — key-value variant (pointer equality)
 * ═══════════════════════════════════════════════════════════════════════════ */

static void ivp_vhash_store_rehash(ivp_vhash_store_t *h, int new_size) {
    ivp_vhash_store_elem_t *old = h->elems;
    int old_size = h->size;
    int old_nelems = h->nelems;

    size_t bytes = (size_t)new_size * sizeof(ivp_vhash_store_elem_t);
    ivp_vhash_store_elem_t *new_elems = (ivp_vhash_store_elem_t *)ivp_alloc(h->alloc, bytes);
    if (!new_elems) return;
    memset(new_elems, 0, bytes);

    h->size = new_size;
    h->size_mm = new_size - 1;
    h->elems = new_elems;
    h->nelems = 0;

    for (int i = 0; i < old_size; i++) {
        if (old[i].key_elem) {
            ivp_vhash_store_add_hash(h, old[i].key_elem, old[i].elem, old[i].hash_index);
        }
    }
    h->nelems = old_nelems;

    ivp_free(h->alloc, old, (size_t)old_size * sizeof(ivp_vhash_store_elem_t));
}

void ivp_vhash_store_init(ivp_vhash_store_t *h, const ivp_allocator_t *alloc,
                           int capacity) {
    if (!h) return;
    h->elems = NULL;
    h->size = 0;
    h->size_mm = -1;
    h->nelems = 0;
    h->alloc = alloc ? alloc : ivp_default_allocator();

    IVP_ASSERT(capacity > 0 && (capacity & (capacity - 1)) == 0);
    if (capacity <= 0 || (capacity & (capacity - 1)) != 0) return;

    h->size = capacity;
    h->size_mm = capacity - 1;
    size_t bytes = (size_t)capacity * sizeof(ivp_vhash_store_elem_t);
    h->elems = (ivp_vhash_store_elem_t *)ivp_alloc(h->alloc, bytes);
    if (!h->elems) {
        h->size = 0;
        h->size_mm = -1;
        return;
    }
    memset(h->elems, 0, bytes);
}

void ivp_vhash_store_destroy(ivp_vhash_store_t *h) {
    if (h->elems) {
        ivp_free(h->alloc, h->elems, (size_t)h->size * sizeof(ivp_vhash_store_elem_t));
        h->elems = NULL;
    }
    h->size = 0;
    h->size_mm = -1;
    h->nelems = 0;
}

void ivp_vhash_store_add(ivp_vhash_store_t *h, void *key, void *value) {
    uint32_t hi = ivp_vhash_ptr_index(key);
    ivp_vhash_store_add_hash(h, key, value, hi);
}

void ivp_vhash_store_add_hash(ivp_vhash_store_t *h, void *key, void *value,
                               uint32_t hash_index) {
    if (!h || !h->elems || h->size_mm < 0) return;
    if (h->nelems + h->nelems >= h->size) {
        ivp_vhash_store_rehash(h, h->size * 2);
        /* If rehash failed (OOM), table may be full — drop insert to avoid infinite loop */
        if (h->nelems + h->nelems >= h->size) return;
    }

    int index = (int)(hash_index & (uint32_t)h->size_mm);
    int pos = index;
    h->nelems++;

    for (int searched = 0; searched < h->size; searched++, pos = (pos + 1) & h->size_mm) {
        ivp_vhash_store_elem_t *e = &h->elems[pos];
        if (!e->key_elem) break;
        int e_index = (int)(e->hash_index & (uint32_t)h->size_mm);
        if (index >= e_index) continue;

        void *h_ke = e->key_elem;
        void *h_e = e->elem;
        uint32_t h_i = e->hash_index;
        e->key_elem = key;
        e->elem = value;
        e->hash_index = hash_index;
        key = h_ke;
        value = h_e;
        hash_index = h_i;
        index = e_index;
    }
    h->elems[pos].key_elem = key;
    h->elems[pos].elem = value;
    h->elems[pos].hash_index = hash_index;
}

void *ivp_vhash_store_find(ivp_vhash_store_t *h, void *key) {
    uint32_t hi = ivp_vhash_ptr_index(key);
    return ivp_vhash_store_find_hash(h, key, hi);
}

void *ivp_vhash_store_find_hash(ivp_vhash_store_t *h, void *key,
                                 uint32_t hash_index) {
    if (!h || !h->elems || h->size_mm < 0) return NULL;
    int pos = (int)(hash_index & (uint32_t)h->size_mm);

    for (int searched = 0; searched < h->size; searched++, pos = (pos + 1) & h->size_mm) {
        ivp_vhash_store_elem_t *e = &h->elems[pos];
        if (!e->key_elem) break;
        if ((e->hash_index | IVP_VHASH_TOUCH_BIT) != hash_index) continue;
        if (e->key_elem == key) return e->elem;
    }
    return NULL;
}

void *ivp_vhash_store_remove(ivp_vhash_store_t *h, void *key) {
    uint32_t hi = ivp_vhash_ptr_index(key);
    return ivp_vhash_store_remove_hash(h, key, hi);
}

void *ivp_vhash_store_remove_hash(ivp_vhash_store_t *h, void *key,
                                   uint32_t hash_index) {
    if (!h || !h->elems || h->size_mm < 0) return NULL;
    int pos = (int)(hash_index & (uint32_t)h->size_mm);

    ivp_vhash_store_elem_t *e;
    for (int searched = 0; searched < h->size; searched++, pos = (pos + 1) & h->size_mm) {
        e = &h->elems[pos];
        if (!e->key_elem) return NULL;
        if ((e->hash_index | IVP_VHASH_TOUCH_BIT) != hash_index) continue;
        if (e->key_elem == key) break;
    }
    if (!e->key_elem || e->key_elem != key) return NULL;

    void *removed = e->elem;
    h->nelems--;
    int last_pos = pos;

    int ideal_pos_of_last = h->size_mm;
    if (h->elems[h->size_mm].key_elem) {
        ideal_pos_of_last = (int)(h->elems[h->size_mm].hash_index & (uint32_t)h->size_mm);
    }

    pos = (pos + 1) & h->size_mm;
    for (int searched = 0; searched < h->size; searched++, pos = (pos + 1) & h->size_mm) {
        ivp_vhash_store_elem_t *en = &h->elems[pos];
        if (!en->key_elem) break;
        int ideal_pos = (int)(en->hash_index & (uint32_t)h->size_mm);

        if (pos > last_pos) {
            if (ideal_pos < ideal_pos_of_last) {
                if (ideal_pos > last_pos) continue;
            } else if (ideal_pos == ideal_pos_of_last) {
                if (ideal_pos <= pos) {
                    if (ideal_pos > last_pos) continue;
                }
            }
        } else {
            if (last_pos != h->size_mm) break;
            if (ideal_pos < ideal_pos_of_last) continue;
        }

        h->elems[last_pos] = *en;
        last_pos = pos;
    }
    h->elems[last_pos].key_elem = NULL;
    h->elems[last_pos].elem = NULL;
    h->elems[last_pos].hash_index = 0;

    return removed;
}

void ivp_vhash_store_change(ivp_vhash_store_t *h, void *key, void *new_value) {
    if (!h || !h->elems || h->size_mm < 0) return;
    uint32_t hash_index = ivp_vhash_ptr_index(key);
    int pos = (int)(hash_index & (uint32_t)h->size_mm);

    for (int searched = 0; searched < h->size; searched++, pos = (pos + 1) & h->size_mm) {
        ivp_vhash_store_elem_t *e = &h->elems[pos];
        if (!e->key_elem) break;
        if ((e->hash_index | IVP_VHASH_TOUCH_BIT) != hash_index) continue;
        if (e->key_elem == key) {
            e->elem = new_value;
            return;
        }
    }
    IVP_ASSERT(0);
}

void *ivp_vhash_store_touch(ivp_vhash_store_t *h, void *key,
                               uint32_t hash_index) {
    if (!h || !h->elems || h->size_mm < 0) return NULL;
    int pos = (int)(hash_index & (uint32_t)h->size_mm);

    for (int searched = 0; searched < h->size; searched++, pos = (pos + 1) & h->size_mm) {
        ivp_vhash_store_elem_t *e = &h->elems[pos];
        if (!e->key_elem) break;
        if ((e->hash_index | IVP_VHASH_TOUCH_BIT) != hash_index) continue;
        if (e->key_elem == key) {
            e->hash_index |= IVP_VHASH_TOUCH_BIT;
            return e->elem;
        }
    }
    return NULL;
}

void ivp_vhash_store_untouch_all(ivp_vhash_store_t *h) {
    if (!h || !h->elems || h->size <= 0) return;
    for (int i = h->size - 1; i >= 0; i--) {
        h->elems[i].hash_index &= (IVP_VHASH_TOUCH_BIT - 1);
    }
}
