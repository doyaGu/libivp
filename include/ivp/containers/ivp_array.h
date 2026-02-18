/* ivp_array.h — Dynamic array (pointer vector) for IVP C17
 *
 * Stable insertion order. Supports both ordered and swap-remove.
 * Optional index callback for tracking element positions (replaces ivp_farray).
 * Debug mode: iterator validation, bounds checking, canaries.
 */
#ifndef IVP_ARRAY_H
#define IVP_ARRAY_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/containers/ivp_elem_typeinfo.h>
#include <ivp/containers/ivp_lifecycle_helpers.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Types ──────────────────────────────────────────────────────────────── */

/* Callback invoked when an element's index changes (push, swap-remove, insert, etc.).
 * Used so elements can track their own position in the array. */
typedef void (*ivp_array_index_cb_t)(void *elem, int new_index, void *userdata);

typedef struct ivp_array {
    void                **data;
    int                   len;
    int                   cap;
    const ivp_allocator_t *alloc;
    const ivp_elem_typeinfo_t *elem_type;
    void                  *elem_ctx;
    ivp_array_index_cb_t   index_cb;
    void                  *index_cb_userdata;
#ifdef IVP_CONTAINER_DEBUG
    uint32_t              version;  /* incremented on mutation for iterator checks */
#endif
} ivp_array_t;

/* ── Create / Destroy ───────────────────────────────────────────────────── */

ivp_result_t ivp_array_init(ivp_array_t *arr, const ivp_allocator_t *alloc, int initial_cap);
ivp_result_t ivp_array_init_ex(ivp_array_t *arr, const ivp_allocator_t *alloc, int initial_cap,
                                const ivp_elem_typeinfo_t *elem_type,
                                void *elem_ctx);

/* Indexed variants: attach an index callback for tracking element positions. */
ivp_result_t ivp_array_init_indexed(ivp_array_t *arr, const ivp_allocator_t *alloc,
                                     int initial_cap,
                                     ivp_array_index_cb_t index_cb, void *cb_userdata);
ivp_result_t ivp_array_init_indexed_ex(ivp_array_t *arr, const ivp_allocator_t *alloc,
                                        int initial_cap,
                                        ivp_array_index_cb_t index_cb,
                                        void *index_cb_userdata,
                                        const ivp_elem_typeinfo_t *elem_type,
                                        void *elem_ctx);

void         ivp_array_destroy(ivp_array_t *arr);

/* ── Access ─────────────────────────────────────────────────────────────── */

static inline int ivp_array_len(const ivp_array_t *arr) { return arr->len; }
static inline int ivp_array_cap(const ivp_array_t *arr) { return arr->cap; }

static inline void *ivp_array_get(const ivp_array_t *arr, int idx) {
    IVP_ASSERT(idx >= 0 && idx < arr->len);
    return arr->data[idx];
}

static inline void *ivp_array_get_unchecked(const ivp_array_t *arr, int idx) {
    return arr->data[idx];
}

static inline void ivp_array_set(ivp_array_t *arr, int idx, void *elem) {
    IVP_ASSERT(idx >= 0 && idx < arr->len);
    ivp_lifecycle_dispose(arr->elem_type, arr->elem_ctx, arr->data[idx]);
    arr->data[idx] = elem;
    ivp_lifecycle_init(arr->elem_type, arr->elem_ctx, elem);
}

/* ── Mutation ───────────────────────────────────────────────────────────── */

/* Append element. Returns IVP_ERR_OUT_OF_MEMORY if grow fails. */
ivp_result_t ivp_array_push(ivp_array_t *arr, void *elem);

/* Ensure capacity for at least min_cap elements. */
ivp_result_t ivp_array_reserve(ivp_array_t *arr, int min_cap);

/* Remove last element and return it. Returns NULL if empty. */
void *ivp_array_pop(ivp_array_t *arr);

/* Insert at index, shifting elements right. */
ivp_result_t ivp_array_insert(ivp_array_t *arr, int idx, void *elem);

/* Remove at index, preserving order (shifts left). Returns removed element. */
void *ivp_array_remove_ordered(ivp_array_t *arr, int idx);

/* Remove at index, O(1), swaps last element in (breaks order). */
void *ivp_array_remove_swap(ivp_array_t *arr, int idx);

/* Clear all elements (doesn't free buffer). */
void ivp_array_clear(ivp_array_t *arr);

/* ── Search ─────────────────────────────────────────────────────────────── */

/* Find element by pointer equality. Returns index or -1. */
int ivp_array_find(const ivp_array_t *arr, const void *elem);

/* ── Iteration helper ───────────────────────────────────────────────────── */

#ifdef IVP_CONTAINER_DEBUG

typedef struct ivp_array_iter {
    const ivp_array_t *arr;
    uint32_t           version;
    int                idx;
} ivp_array_iter_t;

static inline ivp_array_iter_t ivp_array_iter_begin(const ivp_array_t *arr) {
    ivp_array_iter_t it;
    it.arr = arr;
    it.version = arr->version;
    it.idx = 0;
    return it;
}

static inline bool ivp_array_iter_valid(const ivp_array_iter_t *it) {
    IVP_ASSERT_MSG(it->version == it->arr->version, "Iterator invalidated by mutation");
    return it->idx < it->arr->len;
}

static inline void *ivp_array_iter_get(const ivp_array_iter_t *it) {
    IVP_ASSERT_MSG(it->version == it->arr->version, "Iterator invalidated by mutation");
    return it->arr->data[it->idx];
}

static inline void ivp_array_iter_next(ivp_array_iter_t *it) {
    it->idx++;
}

#endif /* IVP_CONTAINER_DEBUG */

#ifdef __cplusplus
}
#endif

#endif /* IVP_ARRAY_H */
