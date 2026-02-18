/* ivp_minheap.h — Binary min-heap for event/time management
 *
 * Stores (key, value) pairs ordered by key (ivp_real_t).
 * Supports push, pop-min, update-key, and remove-by-index.
 * Each element can track its heap index via an optional callback.
 */
#ifndef IVP_MINHEAP_H
#define IVP_MINHEAP_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <ivp/containers/ivp_elem_typeinfo.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Types ──────────────────────────────────────────────────────────────── */

typedef struct ivp_heap_entry {
    ivp_real_t  key;
    void       *value;
} ivp_heap_entry_t;

/* Optional callback: notified whenever an element moves within the heap.
 * value = the element, new_index = its new heap position. */
typedef void (*ivp_heap_index_cb_t)(void *value, int new_index);

typedef struct ivp_minheap {
    ivp_heap_entry_t   *data;
    int                 len;
    int                 cap;
    const ivp_allocator_t *alloc;
    ivp_heap_index_cb_t index_cb;   /* may be NULL */
    const ivp_elem_typeinfo_t *elem_type;
    void                *elem_ctx;
} ivp_minheap_t;

/* ── Create / Destroy ───────────────────────────────────────────────────── */

ivp_result_t ivp_minheap_init(ivp_minheap_t *heap, const ivp_allocator_t *alloc,
                               int initial_capacity, ivp_heap_index_cb_t cb);
ivp_result_t ivp_minheap_init_ex(ivp_minheap_t *heap, const ivp_allocator_t *alloc,
                                  int initial_capacity, ivp_heap_index_cb_t cb,
                                  const ivp_elem_typeinfo_t *elem_type,
                                  void *elem_ctx);
void         ivp_minheap_destroy(ivp_minheap_t *heap);

/* ── Operations ─────────────────────────────────────────────────────────── */

/* Push element. Returns IVP_ERR_OUT_OF_MEMORY on allocation failure. */
ivp_result_t ivp_minheap_push(ivp_minheap_t *heap, ivp_real_t key, void *value);

/* Peek at minimum element without removing it. Returns NULL if empty. */
static inline ivp_heap_entry_t *ivp_minheap_peek(const ivp_minheap_t *heap) {
    return heap->len > 0 ? (ivp_heap_entry_t *)&heap->data[0] : NULL;
}

/* Pop minimum element. Stores in *out_entry if non-NULL. Returns IVP_ERR_CAPACITY if empty. */
ivp_result_t ivp_minheap_pop(ivp_minheap_t *heap, ivp_heap_entry_t *out_entry);

/* Update the key of element at given index. Re-heapifies. */
void ivp_minheap_update_key(ivp_minheap_t *heap, int index, ivp_real_t new_key);

/* Remove element at given index. */
void ivp_minheap_remove(ivp_minheap_t *heap, int index);

/* Number of elements. */
static inline int ivp_minheap_count(const ivp_minheap_t *heap) { return heap->len; }
static inline int ivp_minheap_cap(const ivp_minheap_t *heap) { return heap->cap; }

/* Is empty? */
static inline bool ivp_minheap_empty(const ivp_minheap_t *heap) { return heap->len == 0; }

/* Clear all entries. */
void ivp_minheap_clear(ivp_minheap_t *heap);

/* Ensure capacity for at least min_cap elements. */
ivp_result_t ivp_minheap_reserve(ivp_minheap_t *heap, int min_cap);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MINHEAP_H */
