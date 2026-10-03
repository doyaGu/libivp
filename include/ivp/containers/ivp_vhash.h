/* ivp_vhash.h -- Virtual hash (element-derived key) for IVP C17
 *
 * Open-addressing hash where keys are derived from elements via a callback.
 * Features:
 *   - Power-of-2 sized, auto-expands at 50% load
 *   - Touch bit (MSB of hash_index) for marking visited elements
 *   - CRC-based hash index computation
 *   - Ordered linear probing (elements shifted to preserve order)
 *
 * Ported from ivu_vhash.hxx / ivu_vhash.cxx.
 *
 * Two variants:
 *   ivp_vhash_t       -- element IS the key (compare callback on elements)
 *   ivp_vhash_store_t -- separate key pointer and value pointer
 */
#ifndef IVP_VHASH_H
#define IVP_VHASH_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/utility/ivp_hash.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

/* ── Element-is-key variant (ivp_vhash_t) ──────────────────────────────── */

typedef struct ivp_vhash_elem {
    uint32_t    hash_index;  /* MSB = touch bit */
    const void *elem;        /* NULL = empty slot */
} ivp_vhash_elem_t;

/* Compare callback: return true if elem0 and elem1 are equal. */
typedef bool (*ivp_vhash_compare_fn_t)(const void *elem0, const void *elem1);

typedef struct ivp_vhash {
    ivp_vhash_elem_t       *elems;
    int                     size_mm;   /* capacity - 1 (mask) */
    int                     nelems;
    ivp_vhash_compare_fn_t  compare;
    const ivp_allocator_t  *alloc;
} ivp_vhash_t;

/* Initialize with given capacity (must be power of 2).
 * compare must be non-NULL. */
void ivp_vhash_init(ivp_vhash_t *h, const ivp_allocator_t *alloc,
                     int capacity, ivp_vhash_compare_fn_t compare);

/* Destroy and free backing memory. */
void ivp_vhash_destroy(ivp_vhash_t *h);

/* Add an element with precomputed hash_index. Touches the element. */
void ivp_vhash_add(ivp_vhash_t *h, const void *elem, uint32_t hash_index);

/* Find an element. Returns the stored element or NULL. */
void *ivp_vhash_find(const ivp_vhash_t *h, const void *elem, uint32_t hash_index);

/* Remove an element. Returns the removed element.  Removing an element that
 * is not in the hash is an error: IVP_CORE (reference CORE in
 * IVP_VHash::remove_elem), then NULL. */
void *ivp_vhash_remove(ivp_vhash_t *h, const void *elem, uint32_t hash_index);

/* Find and touch an element. Returns the element or NULL. */
void *ivp_vhash_touch(ivp_vhash_t *h, const void *elem, uint32_t hash_index);

/* Clear all touch bits. */
void ivp_vhash_untouch_all(ivp_vhash_t *h);

/* Shrink if overallocated (preferred_size = estimated element count). */
void ivp_vhash_gc(ivp_vhash_t *h, int preferred_size);

/* Number of elements. */
static inline int ivp_vhash_count(const ivp_vhash_t *h) { return h->nelems; }

/* Capacity (table length). */
static inline int ivp_vhash_capacity(const ivp_vhash_t *h) { return h->size_mm + 1; }

/* Direct access to element at table index (may be NULL). */
static inline void *ivp_vhash_element_at(const ivp_vhash_t *h, int i) {
    return (void *)h->elems[i].elem;
}

/* Check if element at table index is touched. */
static inline bool ivp_vhash_is_touched(const ivp_vhash_t *h, int i) {
    return h->elems[i].hash_index >= IVP_VHASH_TOUCH_BIT;
}

/* ── Key-value variant (ivp_vhash_store_t) ─────────────────────────────── */

typedef struct ivp_vhash_store_elem {
    uint32_t  hash_index;
    void     *key_elem;   /* NULL = empty slot */
    void     *elem;       /* associated value */
} ivp_vhash_store_elem_t;

typedef struct ivp_vhash_store {
    ivp_vhash_store_elem_t *elems;
    int                     size;
    int                     size_mm;   /* size - 1 */
    int                     nelems;
    const ivp_allocator_t  *alloc;
} ivp_vhash_store_t;

/* Initialize with given capacity (must be power of 2). */
void ivp_vhash_store_init(ivp_vhash_store_t *h, const ivp_allocator_t *alloc,
                           int capacity);

/* Destroy and free backing memory. */
void ivp_vhash_store_destroy(ivp_vhash_store_t *h);

/* Add a key-value pair. Key comparison uses pointer equality. */
void ivp_vhash_store_add(ivp_vhash_store_t *h, void *key, void *value);

/* Add with precomputed hash index. */
void ivp_vhash_store_add_hash(ivp_vhash_store_t *h, void *key, void *value,
                               uint32_t hash_index);

/* Find value by key pointer. Returns value or NULL. */
void *ivp_vhash_store_find(ivp_vhash_store_t *h, void *key);

/* Find with precomputed hash index. */
void *ivp_vhash_store_find_hash(ivp_vhash_store_t *h, void *key,
                                 uint32_t hash_index);

/* Remove by key pointer. Returns value or NULL. */
void *ivp_vhash_store_remove(ivp_vhash_store_t *h, void *key);

/* Remove with precomputed hash index. */
void *ivp_vhash_store_remove_hash(ivp_vhash_store_t *h, void *key,
                                   uint32_t hash_index);

/* Update value for existing key. Asserts key exists. */
void ivp_vhash_store_change(ivp_vhash_store_t *h, void *key, void *new_value);

/* Find and touch. Returns value or NULL. */
void *ivp_vhash_store_touch(ivp_vhash_store_t *h, void *key,
                              uint32_t hash_index);

/* Clear all touch bits. */
void ivp_vhash_store_untouch_all(ivp_vhash_store_t *h);

/* Number of elements. */
static inline int ivp_vhash_store_count(const ivp_vhash_store_t *h) {
    return h->nelems;
}

/* Capacity. */
static inline int ivp_vhash_store_capacity(const ivp_vhash_store_t *h) {
    return h->size;
}

/* Direct access. */
static inline void *ivp_vhash_store_element_at(const ivp_vhash_store_t *h, int i) {
    return h->elems[i].elem;
}

static inline bool ivp_vhash_store_is_touched(const ivp_vhash_store_t *h, int i) {
    return h->elems[i].hash_index >= IVP_VHASH_TOUCH_BIT;
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_VHASH_H */
