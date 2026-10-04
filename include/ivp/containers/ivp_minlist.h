/* ivp_minlist.h -- Sorted minimum list for IVP C17
 *
 * A sorted linked list where elements are ordered by ascending float value.
 * Supports O(1) minimum query, O(n) insertion with skip-list acceleration.
 * Used for mindist event scheduling where minimum-time queries are critical.
 *
 * Ported from ivu_min_list.hxx / ivu_min_list.cxx.
 *
 * Indices are 32-bit so large worlds are not limited to 65532 scheduled
 * events. The highest bit remains available to the indexed implementation.
 */
#ifndef IVP_MINLIST_H
#define IVP_MINLIST_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_allocator.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define IVP_MINLIST_UNUSED       UINT32_MAX
#define IVP_MINLIST_LONG_UNUSED  (UINT32_MAX - UINT32_C(1))
#define IVP_MINLIST_MAX_ALLOC    (UINT32_C(0x7fffffff) - UINT32_C(2))
#define IVP_MINLIST_MAX_VALUE   1e10f

typedef uint32_t ivp_minlist_index_t;

/* ── Element ───────────────────────────────────────────────────────────── */

typedef struct ivp_minlist_elem {
    uint32_t long_next;   /* skip-list: next long-jump node */
    uint32_t long_prev;   /* skip-list: previous long-jump node */
    uint32_t next;        /* doubly-linked: next in sorted order */
    uint32_t prev;        /* doubly-linked: previous in sorted order */
    float    value;       /* sort key */
    void    *element;     /* user data pointer */
} ivp_minlist_elem_t;

/* ── Min list ──────────────────────────────────────────────────────────── */

typedef struct ivp_minlist {
    ivp_minlist_elem_t *elems;
    uint32_t            malloced_size;
    uint32_t            free_list;
    uint32_t            first_element;  /* head of sorted list */
    uint32_t            first_long;     /* head of skip-list chain */
    uint32_t            counter;        /* number of elements */
    float               min_value;      /* cached minimum value */
    const ivp_allocator_t *alloc;
} ivp_minlist_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

/* Initialize with given starting capacity (will grow as needed). */
void ivp_minlist_init(ivp_minlist_t *ml, const ivp_allocator_t *alloc, int start_size);

/* Destroy and free all memory. */
void ivp_minlist_destroy(ivp_minlist_t *ml);

/* ── Operations ────────────────────────────────────────────────────────── */

/* Add an element with a sort value. Returns index for later removal,
 * or IVP_MINLIST_UNUSED if allocation fails or the list is full (the
 * reference prints "Fatal error: IVP_U_Min_List reached maximum capacity"
 * through ivp_message and COREs there, so does this port: IVP_CORE). */
ivp_minlist_index_t ivp_minlist_add(ivp_minlist_t *ml, void *elem, float value);

/* Remove element by index. */
void ivp_minlist_remove(ivp_minlist_t *ml, ivp_minlist_index_t index);

/* ── Queries ───────────────────────────────────────────────────────────── */

/* Get the element with minimum value. Undefined if empty. */
static inline void *ivp_minlist_find_min_elem(const ivp_minlist_t *ml) {
    IVP_ASSERT(ml->first_element != IVP_MINLIST_UNUSED);
    return ml->elems[ml->first_element].element;
}

/* Get the minimum value. Returns IVP_MINLIST_MAX_VALUE if empty. */
static inline float ivp_minlist_find_min_value(const ivp_minlist_t *ml) {
    return ml->min_value;
}

/* Check if list has any elements. */
static inline bool ivp_minlist_has_elements(const ivp_minlist_t *ml) {
    return ml->counter > 0;
}

/* Number of elements. */
static inline uint32_t ivp_minlist_count(const ivp_minlist_t *ml) {
    return ml->counter;
}

/* ── Enumerator ────────────────────────────────────────────────────────── */

typedef struct ivp_minlist_enumerator {
    ivp_minlist_t  *ml;
    ivp_minlist_index_t current;
} ivp_minlist_enumerator_t;

static inline void ivp_minlist_enum_init(ivp_minlist_enumerator_t *en,
                                          ivp_minlist_t *ml) {
    en->ml = ml;
    en->current = ml->first_element;
}

/* Get next element in sorted order. Returns NULL when exhausted. */
static inline void *ivp_minlist_enum_next(ivp_minlist_enumerator_t *en) {
    if (en->current == IVP_MINLIST_UNUSED) return NULL;
    void *e = en->ml->elems[en->current].element;
    en->current = en->ml->elems[en->current].next;
    return e;
}

/* Get next element only if its value < max_limit. Returns NULL otherwise. */
static inline void *ivp_minlist_enum_next_lt(ivp_minlist_enumerator_t *en,
                                              float max_limit) {
    if (en->current == IVP_MINLIST_UNUSED) return NULL;
    ivp_minlist_elem_t *el = &en->ml->elems[en->current];
    if (el->value >= max_limit) return NULL;
    en->current = el->next;
    return el->element;
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_MINLIST_H */
