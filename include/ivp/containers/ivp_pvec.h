/* ivp_pvec.h — Pointer vector with IVP_U_Vector semantics
 *
 * Port of ivu_vector.hxx: add appends, remove() shifts (order preserving),
 * remove_allow_resort() moves the last element into the hole, index_of()
 * searches from the end.  Ordering semantics matter for determinism, so
 * every reference call site maps 1:1 onto these helpers.
 */
#ifndef IVP_PVEC_H
#define IVP_PVEC_H

#include <ivp/base/ivp_assert.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_pvec {
    void **elems;
    int    n;
    int    cap;
} ivp_pvec_t;

#define IVP_PVEC_INIT { NULL, 0, 0 }

static inline void ivp_pvec_init(ivp_pvec_t *v) { v->elems = NULL; v->n = 0; v->cap = 0; }

static inline void ivp_pvec_clear(ivp_pvec_t *v) {
    free(v->elems);
    v->elems = NULL; v->n = 0; v->cap = 0;
}

static inline void ivp_pvec_remove_all(ivp_pvec_t *v) { v->n = 0; }

static inline int ivp_pvec_len(const ivp_pvec_t *v) { return v->n; }

static inline void *ivp_pvec_at(const ivp_pvec_t *v, int i) {
    IVP_ASSERT(i >= 0 && i < v->n);
    return v->elems[i];
}

static inline void ivp_pvec_ensure(ivp_pvec_t *v) {
    if (v->n >= v->cap) {
        /* reference increment_mem: grows by doubling (+1) */
        int nc = v->cap * 2 + 1;
        if (nc < 4) nc = 4;
        void **ne = (void **)realloc(v->elems, (size_t)nc * sizeof(void *));
        IVP_ASSERT(ne);
        if (!ne) return;
        v->elems = ne;
        v->cap = nc;
    }
}

static inline int ivp_pvec_add(ivp_pvec_t *v, void *e) {
    ivp_pvec_ensure(v);
    v->elems[v->n] = e;
    return v->n++;
}

static inline int ivp_pvec_index_of(const ivp_pvec_t *v, const void *e) {
    int i;
    for (i = v->n - 1; i >= 0; i--) {
        if (v->elems[i] == e) break;
    }
    return i;
}

static inline int ivp_pvec_install(ivp_pvec_t *v, void *e) {
    int idx = ivp_pvec_index_of(v, e);
    if (idx != -1) return idx;
    return ivp_pvec_add(v, e);
}

static inline void ivp_pvec_remove_at(ivp_pvec_t *v, int index) {
    IVP_ASSERT(index >= 0 && index < v->n);
    if (index < 0 || index >= v->n) return;
    for (int j = index; j < v->n - 1; j++) v->elems[j] = v->elems[j + 1];
    v->n--;
}

static inline void ivp_pvec_remove(ivp_pvec_t *v, const void *e) {
    int index = ivp_pvec_index_of(v, e);
    IVP_ASSERT(index >= 0);
    if (index < 0) return;
    ivp_pvec_remove_at(v, index);
}

/* remove e if it is in the vector (no assertion) */
static inline bool ivp_pvec_remove_if_present(ivp_pvec_t *v, const void *e) {
    int index = ivp_pvec_index_of(v, e);
    if (index < 0) return false;
    ivp_pvec_remove_at(v, index);
    return true;
}

static inline void ivp_pvec_remove_at_allow_resort(ivp_pvec_t *v, int index) {
    IVP_ASSERT(index >= 0 && index < v->n);
    if (index < 0 || index >= v->n) return;
    v->n--;
    v->elems[index] = v->elems[v->n];
}

static inline void ivp_pvec_remove_allow_resort(ivp_pvec_t *v, const void *e) {
    int index = ivp_pvec_index_of(v, e);
    IVP_ASSERT(index >= 0);
    if (index < 0) return;
    ivp_pvec_remove_at_allow_resort(v, index);
}

/* swap_elems (ivu_vector.hxx:141) */
static inline void ivp_pvec_swap(ivp_pvec_t *v, int index1, int index2) {
    IVP_ASSERT((index1 >= 0) && (index1 < v->n));
    IVP_ASSERT((index2 >= 0) && (index2 < v->n));
    void *t = v->elems[index1];
    v->elems[index1] = v->elems[index2];
    v->elems[index2] = t;
}

static inline void ivp_pvec_insert_after(ivp_pvec_t *v, int index, void *e) {
    IVP_ASSERT(index >= 0 && index < v->n);
    index++;
    ivp_pvec_ensure(v);
    for (int j = v->n; j > index; j--) v->elems[j] = v->elems[j - 1];
    v->elems[index] = e;
    v->n++;
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_PVEC_H */
