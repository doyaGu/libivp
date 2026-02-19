/* ivp_array.c — Dynamic array implementation */
#include <ivp/containers/ivp_array.h>
#include <ivp/containers/ivp_lifecycle_helpers.h>
#include <limits.h>
#include <string.h>

/* ── Internal: grow buffer ──────────────────────────────────────────────── */

static ivp_result_t array_grow(ivp_array_t *arr, int min_cap) {
    if (min_cap < 0) return IVP_ERR_INVALID_ARG;

    int new_cap = arr->cap > 0 ? arr->cap : 4;
    while (new_cap < min_cap) {
        if (new_cap > INT_MAX / 2) return IVP_ERR_CAPACITY_EXCEEDED;
        new_cap *= 2;
    }
    if ((size_t)new_cap > ((size_t)-1) / sizeof(void *)) {
        return IVP_ERR_CAPACITY_EXCEEDED;
    }

    size_t old_size = (size_t)arr->cap * sizeof(void *);
    size_t new_size = (size_t)new_cap * sizeof(void *);
    void **new_data = (void **)ivp_realloc(arr->alloc, arr->data, old_size, new_size);
    if (!new_data) return IVP_ERR_OUT_OF_MEMORY;

    arr->data = new_data;
    arr->cap  = new_cap;
    return IVP_OK;
}

/* ── API implementation ─────────────────────────────────────────────────── */

ivp_result_t ivp_array_init(ivp_array_t *arr, const ivp_allocator_t *alloc, int initial_cap) {
    return ivp_array_init_ex(arr, alloc, initial_cap, &IVP_ELEM_TYPEINFO_PTR, NULL);
}

ivp_result_t ivp_array_init_ex(ivp_array_t *arr, const ivp_allocator_t *alloc, int initial_cap,
                                const ivp_elem_typeinfo_t *elem_type,
                                void *elem_ctx) {
    if (!arr) return IVP_ERR_INVALID_ARG;
    if (!alloc) alloc = ivp_default_allocator();

    arr->data  = NULL;
    arr->len   = 0;
    arr->cap   = 0;
    arr->alloc = alloc;
    arr->elem_type = elem_type;
    arr->elem_ctx = elem_ctx;
    arr->index_cb = NULL;
    arr->index_cb_userdata = NULL;
#ifdef IVP_CONTAINER_DEBUG
    arr->version = 0;
#endif

    if (initial_cap > 0) {
        return array_grow(arr, initial_cap);
    }
    return IVP_OK;
}

ivp_result_t ivp_array_init_indexed(ivp_array_t *arr, const ivp_allocator_t *alloc,
                                     int initial_cap,
                                     ivp_array_index_cb_t index_cb, void *cb_userdata) {
    return ivp_array_init_indexed_ex(arr, alloc, initial_cap, index_cb, cb_userdata,
                                      &IVP_ELEM_TYPEINFO_PTR, NULL);
}

ivp_result_t ivp_array_init_indexed_ex(ivp_array_t *arr, const ivp_allocator_t *alloc,
                                        int initial_cap,
                                        ivp_array_index_cb_t index_cb,
                                        void *index_cb_userdata,
                                        const ivp_elem_typeinfo_t *elem_type,
                                        void *elem_ctx) {
    ivp_result_t r = ivp_array_init_ex(arr, alloc, initial_cap, elem_type, elem_ctx);
    if (r != IVP_OK) return r;
    arr->index_cb = index_cb;
    arr->index_cb_userdata = index_cb_userdata;
    return IVP_OK;
}

ivp_result_t ivp_array_reserve(ivp_array_t *arr, int min_cap) {
    if (min_cap <= arr->cap) return IVP_OK;
    return array_grow(arr, min_cap);
}

void ivp_array_destroy(ivp_array_t *arr) {
    if (!arr) return;
    for (int i = 0; i < arr->len; i++) {
        ivp_lifecycle_dispose(arr->elem_type, arr->elem_ctx, arr->data[i]);
    }
    if (arr->data) {
        ivp_free(arr->alloc, arr->data, (size_t)arr->cap * sizeof(void *));
    }
    arr->data = NULL;
    arr->len  = 0;
    arr->cap  = 0;
}

ivp_result_t ivp_array_push(ivp_array_t *arr, void *elem) {
    if (arr->len >= arr->cap) {
        ivp_result_t r = array_grow(arr, arr->len + 1);
        if (r != IVP_OK) return r;
    }
    int idx = arr->len;
    arr->data[idx] = elem;
    arr->len++;
    if (arr->index_cb) {
        arr->index_cb(elem, idx, arr->index_cb_userdata);
    }
    ivp_lifecycle_init(arr->elem_type, arr->elem_ctx, elem);
#ifdef IVP_CONTAINER_DEBUG
    arr->version++;
#endif
    return IVP_OK;
}

void *ivp_array_pop(ivp_array_t *arr) {
    if (arr->len == 0) return NULL;
    void *elem = arr->data[--arr->len];
#ifdef IVP_CONTAINER_DEBUG
    arr->version++;
#endif
    return elem;
}

ivp_result_t ivp_array_insert(ivp_array_t *arr, int idx, void *elem) {
    IVP_ASSERT(idx >= 0 && idx <= arr->len);
    if (arr->len >= arr->cap) {
        ivp_result_t r = array_grow(arr, arr->len + 1);
        if (r != IVP_OK) return r;
    }
    if (idx < arr->len) {
        for (int i = arr->len; i > idx; --i) {
            arr->data[i] = arr->data[i - 1];
            ivp_lifecycle_move(arr->elem_type, arr->elem_ctx, arr->data[i], i - 1, i);
            if (arr->index_cb) {
                arr->index_cb(arr->data[i], i, arr->index_cb_userdata);
            }
        }
    }
    arr->data[idx] = elem;
    arr->len++;
    if (arr->index_cb) {
        arr->index_cb(elem, idx, arr->index_cb_userdata);
    }
    ivp_lifecycle_init(arr->elem_type, arr->elem_ctx, elem);
#ifdef IVP_CONTAINER_DEBUG
    arr->version++;
#endif
    return IVP_OK;
}

void *ivp_array_remove_ordered(ivp_array_t *arr, int idx) {
    IVP_ASSERT(idx >= 0 && idx < arr->len);
    void *elem = arr->data[idx];
    if (idx < arr->len - 1) {
        for (int i = idx; i < arr->len - 1; ++i) {
            arr->data[i] = arr->data[i + 1];
            ivp_lifecycle_move(arr->elem_type, arr->elem_ctx, arr->data[i], i + 1, i);
            if (arr->index_cb) {
                arr->index_cb(arr->data[i], i, arr->index_cb_userdata);
            }
        }
    }
    arr->len--;
#ifdef IVP_CONTAINER_DEBUG
    arr->version++;
#endif
    return elem;
}

void *ivp_array_remove_swap(ivp_array_t *arr, int idx) {
    IVP_ASSERT(idx >= 0 && idx < arr->len);
    void *elem = arr->data[idx];
    int last = arr->len - 1;
    if (idx != last) {
        arr->data[idx] = arr->data[last];
        ivp_lifecycle_move(arr->elem_type, arr->elem_ctx, arr->data[idx], last, idx);
        if (arr->index_cb) {
            arr->index_cb(arr->data[idx], idx, arr->index_cb_userdata);
        }
    }
    arr->len--;
#ifdef IVP_CONTAINER_DEBUG
    arr->version++;
#endif
    return elem;
}

void ivp_array_clear(ivp_array_t *arr) {
    for (int i = 0; i < arr->len; i++) {
        ivp_lifecycle_dispose(arr->elem_type, arr->elem_ctx, arr->data[i]);
    }
    arr->len = 0;
#ifdef IVP_CONTAINER_DEBUG
    arr->version++;
#endif
}

int ivp_array_find(const ivp_array_t *arr, const void *elem) {
    for (int i = 0; i < arr->len; i++) {
        if (arr->data[i] == elem) return i;
    }
    return -1;
}
