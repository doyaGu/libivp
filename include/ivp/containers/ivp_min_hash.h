/* ivp_min_hash.h — IVP_U_Min_Hash port (ivu_min_hash.hxx / .cxx)
 *
 * A pointer-keyed hash of (elem, value) pairs that keeps track of the
 * minimum value.  Elements live in `size` buckets (unsorted linked lists,
 * new elements prepended, bucket = hash of the element pointer); a binary
 * tournament tree ("stadel") over the per-bucket minima yields the global
 * minimum.  Tie handling is exactly the reference one (SORT_MINDIST_ELEMENTS
 * build):
 *   - adding: a new element only replaces an existing (bucket or tree)
 *     minimum if its value is strictly smaller;
 *   - after a removal the bucket / tree minima are recomputed preferring
 *     the smaller value, then the smaller insertion index.  libivp stores
 *     this counter per hash; comparisons never cross hashes, so this keeps
 *     the reference order while making independent hashes reentrant.
 * The minimum among equal values therefore depends on the element pointer
 * values (bucket placement), just like in the reference.
 *
 * libivp's ivp_minheap_t (binary heap) is a different structure and is
 * NOT a replacement for this one.
 */
#ifndef IVP_MIN_HASH_H
#define IVP_MIN_HASH_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_min_hash_elem {
    struct ivp_min_hash_elem *next;
    double                    value;
    int                       cmp_index;   /* insertion order (tie breaker) */
    void                     *elem;
} ivp_min_hash_elem_t;

typedef struct ivp_min_hash {
    unsigned int           size;               /* power of 2 */
    ivp_min_hash_elem_t  **stadel;              /* tree, [1] == root, 2*size entries */
    ivp_min_hash_elem_t  **min_per_array_pos;   /* == stadel + size */
    ivp_min_hash_elem_t  **elems;               /* bucket lists */
    int                    counter;             /* number of elements */
    int                    sort_counter;        /* next insertion tie breaker */
    const ivp_allocator_t *alloc;
} ivp_min_hash_t;

/* size must be a power of 2 (reference default 256). */
ivp_result_t ivp_min_hash_init(ivp_min_hash_t *h, const ivp_allocator_t *alloc,
                               int size);
void ivp_min_hash_destroy(ivp_min_hash_t *h);

/* IVP_U_Min_Hash::add — no duplicate check (like the reference). */
ivp_result_t ivp_min_hash_add(ivp_min_hash_t *h, void *elem, double value);

/* change_value: remove(elem) + add(elem, value) */
ivp_result_t ivp_min_hash_change_value(ivp_min_hash_t *h, void *elem, double value);

/* remove the element (first match in its bucket list); no-op if absent */
void ivp_min_hash_remove(ivp_min_hash_t *h, void *elem);

/* remove the current minimum element */
void ivp_min_hash_remove_min(ivp_min_hash_t *h);

bool ivp_min_hash_is_elem(const ivp_min_hash_t *h, const void *elem);

/* NULL / 0.0 if empty */
static inline void *ivp_min_hash_find_min_elem(const ivp_min_hash_t *h) {
    return h->stadel[1] ? h->stadel[1]->elem : NULL;
}
static inline double ivp_min_hash_find_min_value(const ivp_min_hash_t *h) {
    return h->stadel[1] ? h->stadel[1]->value : 0.0;
}
static inline int ivp_min_hash_count(const ivp_min_hash_t *h) { return h->counter; }

/* IVP_U_Min_Hash_Enumerator: bucket order, list order within a bucket */
typedef struct ivp_min_hash_enumerator {
    const ivp_min_hash_t      *min_hash;
    const ivp_min_hash_elem_t *loop_elem;
    int                        loop_index;
} ivp_min_hash_enumerator_t;

static inline void ivp_min_hash_enumerator_init(ivp_min_hash_enumerator_t *e,
                                                const ivp_min_hash_t *h) {
    e->min_hash = h;
    e->loop_elem = NULL;
    e->loop_index = -1;
}

void *ivp_min_hash_enumerator_next(ivp_min_hash_enumerator_t *e);
/* next element whose value is < max_limit */
void *ivp_min_hash_enumerator_next_lt(ivp_min_hash_enumerator_t *e, double max_limit);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MIN_HASH_H */
