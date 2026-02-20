/* ivp_minheap.c — Binary min-heap implementation */
#include <ivp/containers/ivp_minheap.h>
#include <ivp/containers/ivp_lifecycle_helpers.h>
#include <ivp/base/ivp_assert.h>
#include <limits.h>
#include <string.h>

/* ── Internal helpers ───────────────────────────────────────────────────── */

static inline int parent(int i)      { return (i - 1) / 2; }
static inline int left_child(int i)  { return 2 * i + 1; }
static inline int right_child(int i) { return 2 * i + 2; }

static inline void notify(ivp_minheap_t *heap, int idx) {
    if (heap->index_cb) {
        heap->index_cb(heap->data[idx].value, idx);
    }
}

static inline void swap_entries(ivp_minheap_t *heap, int a, int b) {
    ivp_heap_entry_t tmp = heap->data[a];
    heap->data[a] = heap->data[b];
    heap->data[b] = tmp;
    ivp_lifecycle_move(heap->elem_type, heap->elem_ctx, heap->data[a].value, b, a);
    ivp_lifecycle_move(heap->elem_type, heap->elem_ctx, heap->data[b].value, a, b);
    notify(heap, a);
    notify(heap, b);
}

static void sift_up(ivp_minheap_t *heap, int idx) {
    while (idx > 0 && heap->data[idx].key < heap->data[parent(idx)].key) {
        swap_entries(heap, idx, parent(idx));
        idx = parent(idx);
    }
}

static void sift_down(ivp_minheap_t *heap, int idx) {
    int n = heap->len;
    while (true) {
        int smallest = idx;
        int l = left_child(idx);
        int r = right_child(idx);

        if (l < n && heap->data[l].key < heap->data[smallest].key)
            smallest = l;
        if (r < n && heap->data[r].key < heap->data[smallest].key)
            smallest = r;

        if (smallest == idx) break;

        swap_entries(heap, idx, smallest);
        idx = smallest;
    }
}

static ivp_result_t ensure_capacity(ivp_minheap_t *heap, int min_cap) {
    if (min_cap < 0) return IVP_ERR_INVALID_ARG;
    if (min_cap <= heap->cap) return IVP_OK;

    int new_cap = heap->cap < 16 ? 16 : heap->cap;
    while (new_cap < min_cap) {
        if (new_cap > INT_MAX / 2) return IVP_ERR_CAPACITY_EXCEEDED;
        new_cap *= 2;
    }
    if ((size_t)new_cap > ((size_t)-1) / sizeof(ivp_heap_entry_t)) {
        return IVP_ERR_CAPACITY_EXCEEDED;
    }
    size_t new_bytes = (size_t)new_cap * sizeof(ivp_heap_entry_t);
    size_t old_bytes = (size_t)heap->cap * sizeof(ivp_heap_entry_t);

    ivp_heap_entry_t *new_data =
        (ivp_heap_entry_t *)ivp_realloc(heap->alloc, heap->data, old_bytes, new_bytes);
    if (!new_data) return IVP_ERR_OUT_OF_MEMORY;

    if (new_data != heap->data) {
        for (int i = 0; i < heap->len; i++) {
            ivp_lifecycle_copy(heap->elem_type, heap->elem_ctx, new_data[i].value, i, i);
        }
    }

    heap->data = new_data;
    heap->cap  = new_cap;
    return IVP_OK;
}

/* ── API implementation ─────────────────────────────────────────────────── */

ivp_result_t ivp_minheap_init(ivp_minheap_t *heap, const ivp_allocator_t *alloc,
                               int initial_capacity, ivp_heap_index_cb_t cb) {
    return ivp_minheap_init_ex(heap, alloc, initial_capacity, cb, &IVP_ELEM_TYPEINFO_PTR, NULL);
}

ivp_result_t ivp_minheap_reserve(ivp_minheap_t *heap, int min_cap) {
    return ensure_capacity(heap, min_cap);
}

ivp_result_t ivp_minheap_init_ex(ivp_minheap_t *heap, const ivp_allocator_t *alloc,
                                  int initial_capacity, ivp_heap_index_cb_t cb,
                                  const ivp_elem_typeinfo_t *elem_type,
                                  void *elem_ctx) {
    if (!heap) return IVP_ERR_INVALID_ARG;
    if (!alloc) alloc = ivp_default_allocator();

    int cap = initial_capacity < 16 ? 16 : initial_capacity;
    if ((size_t)cap > ((size_t)-1) / sizeof(ivp_heap_entry_t)) {
        return IVP_ERR_CAPACITY_EXCEEDED;
    }
    size_t bytes = (size_t)cap * sizeof(ivp_heap_entry_t);
    ivp_heap_entry_t *data = (ivp_heap_entry_t *)ivp_alloc(alloc, bytes);
    if (!data) return IVP_ERR_OUT_OF_MEMORY;

    heap->data     = data;
    heap->len      = 0;
    heap->cap      = cap;
    heap->alloc    = alloc;
    heap->index_cb = cb;
    heap->elem_type = elem_type;
    heap->elem_ctx = elem_ctx;

    return IVP_OK;
}

void ivp_minheap_destroy(ivp_minheap_t *heap) {
    if (!heap) return;
    for (int i = 0; i < heap->len; i++) {
        ivp_lifecycle_dispose(heap->elem_type, heap->elem_ctx, heap->data[i].value);
    }
    if (heap->data) {
        ivp_free(heap->alloc, heap->data,
                 (size_t)heap->cap * sizeof(ivp_heap_entry_t));
    }
    heap->data = NULL;
    heap->len  = 0;
    heap->cap  = 0;
}

ivp_result_t ivp_minheap_push(ivp_minheap_t *heap, ivp_time_t key, void *value) {
    ivp_result_t r = ensure_capacity(heap, heap->len + 1);
    if (r != IVP_OK) return r;

    int idx = heap->len;
    heap->data[idx].key   = key;
    heap->data[idx].value = value;
    heap->len++;

    ivp_lifecycle_init(heap->elem_type, heap->elem_ctx, value);
    notify(heap, idx);
    sift_up(heap, idx);
    return IVP_OK;
}

ivp_result_t ivp_minheap_pop(ivp_minheap_t *heap, ivp_heap_entry_t *out_entry) {
    if (heap->len == 0) return IVP_ERR_INVALID_STATE;

    if (out_entry) {
        *out_entry = heap->data[0];
    }

    heap->len--;
    if (heap->len > 0) {
        heap->data[0] = heap->data[heap->len];
        ivp_lifecycle_move(heap->elem_type, heap->elem_ctx, heap->data[0].value, heap->len, 0);
        notify(heap, 0);
        sift_down(heap, 0);
    }

    return IVP_OK;
}

void ivp_minheap_update_key(ivp_minheap_t *heap, int index, ivp_time_t new_key) {
    IVP_ASSERT(index >= 0 && index < heap->len);

    ivp_time_t old_key = heap->data[index].key;
    heap->data[index].key = new_key;

    if (new_key < old_key) {
        sift_up(heap, index);
    } else {
        sift_down(heap, index);
    }
}

void ivp_minheap_remove(ivp_minheap_t *heap, int index) {
    IVP_ASSERT(index >= 0 && index < heap->len);

    void *removed_value = heap->data[index].value;

    heap->len--;
    if (index < heap->len) {
        heap->data[index] = heap->data[heap->len];
        ivp_lifecycle_move(heap->elem_type, heap->elem_ctx, heap->data[index].value, heap->len, index);
        notify(heap, index);

        /* Determine whether to sift up or down */
        if (index > 0 && heap->data[index].key < heap->data[parent(index)].key) {
            sift_up(heap, index);
        } else {
            sift_down(heap, index);
        }
    }

    ivp_lifecycle_dispose(heap->elem_type, heap->elem_ctx, removed_value);
}

void ivp_minheap_clear(ivp_minheap_t *heap) {
    for (int i = 0; i < heap->len; i++) {
        ivp_lifecycle_reset(heap->elem_type, heap->elem_ctx, heap->data[i].value);
        ivp_lifecycle_dispose(heap->elem_type, heap->elem_ctx, heap->data[i].value);
    }
    heap->len = 0;
}
