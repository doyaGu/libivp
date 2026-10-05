/* ivp_minlist.c -- Sorted minimum list implementation
 *
 * The reference's doubly-linked list remains authoritative for enumeration.
 * A deterministic red-black tree over finite values only replaces the
 * skip-list insertion search. Equality always descends left, preserving the
 * reference rule that a new equal-time item precedes older items.
 */
#include <ivp/containers/ivp_minlist.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_legacy.h>
#include <ivp/base/ivp_message.h>

#include <math.h>
#include "ivp_minlist_diagnostics.h"
#ifdef IVP_BENCHMARK_DIAGNOSTICS
static ivp_minlist_diagnostics_t minlist_diagnostics;
static uint64_t search_comparisons;
void ivp_minlist_diagnostics_reset(void)
{
    minlist_diagnostics = (ivp_minlist_diagnostics_t){0};
}
ivp_minlist_diagnostics_t ivp_minlist_diagnostics_get(void)
{
    return minlist_diagnostics;
}
#define COUNT_COMPARISON() (++search_comparisons)
#else
#define COUNT_COMPARISON() ((void)0)
#endif

#define IVP_MINLIST_RED UINT32_C(0x80000000)
#define IVP_MINLIST_PARENT_MASK UINT32_C(0x7fffffff)
#define IVP_MINLIST_TREE_THRESHOLD UINT32_C(1024)
#define IVP_MINLIST_CACHE_THRESHOLD UINT32_C(64)
#define IVP_MINLIST_CACHE_BACKTRACK 8u

static uint32_t rb_parent(const ivp_minlist_t *ml, uint32_t index)
{
    return ml->elems[index].tree_parent_color & IVP_MINLIST_PARENT_MASK;
}

static bool rb_is_red(const ivp_minlist_t *ml, uint32_t index)
{
    return index != IVP_MINLIST_TREE_UNUSED &&
           (ml->elems[index].tree_parent_color & IVP_MINLIST_RED) != 0;
}

static void rb_set_parent(ivp_minlist_t *ml, uint32_t index, uint32_t parent)
{
    if (index == IVP_MINLIST_TREE_UNUSED) return;
    ml->elems[index].tree_parent_color =
        (ml->elems[index].tree_parent_color & IVP_MINLIST_RED) | parent;
}

static void rb_set_red(ivp_minlist_t *ml, uint32_t index, bool red)
{
    if (index == IVP_MINLIST_TREE_UNUSED) return;
    ml->elems[index].tree_parent_color =
        (ml->elems[index].tree_parent_color & IVP_MINLIST_PARENT_MASK) |
        (red ? IVP_MINLIST_RED : 0u);
}

static uint32_t rb_left(const ivp_minlist_t *ml, uint32_t index)
{
    return index == IVP_MINLIST_TREE_UNUSED ? IVP_MINLIST_TREE_UNUSED
                                             : ml->elems[index].tree_left;
}

static uint32_t rb_right(const ivp_minlist_t *ml, uint32_t index)
{
    return index == IVP_MINLIST_TREE_UNUSED ? IVP_MINLIST_TREE_UNUSED
                                             : ml->elems[index].tree_right;
}

static bool rb_key_before(float value, const ivp_minlist_elem_t *other)
{
    return value <= other->value;
}

static void rb_rotate_left(ivp_minlist_t *ml, uint32_t x)
{
    uint32_t y = ml->elems[x].tree_right;
    ml->elems[x].tree_right = ml->elems[y].tree_left;
    rb_set_parent(ml, ml->elems[y].tree_left, x);
    uint32_t xp = rb_parent(ml, x);
    rb_set_parent(ml, y, xp);
    if (xp == IVP_MINLIST_TREE_UNUSED) {
        ml->tree_root = y;
    } else if (x == ml->elems[xp].tree_left) {
        ml->elems[xp].tree_left = y;
    } else {
        ml->elems[xp].tree_right = y;
    }
    ml->elems[y].tree_left = x;
    rb_set_parent(ml, x, y);
}

static void rb_rotate_right(ivp_minlist_t *ml, uint32_t x)
{
    uint32_t y = ml->elems[x].tree_left;
    ml->elems[x].tree_left = ml->elems[y].tree_right;
    rb_set_parent(ml, ml->elems[y].tree_right, x);
    uint32_t xp = rb_parent(ml, x);
    rb_set_parent(ml, y, xp);
    if (xp == IVP_MINLIST_TREE_UNUSED) {
        ml->tree_root = y;
    } else if (x == ml->elems[xp].tree_right) {
        ml->elems[xp].tree_right = y;
    } else {
        ml->elems[xp].tree_left = y;
    }
    ml->elems[y].tree_right = x;
    rb_set_parent(ml, x, y);
}

static void rb_insert_fixup(ivp_minlist_t *ml, uint32_t z)
{
    while (rb_is_red(ml, rb_parent(ml, z))) {
        uint32_t parent = rb_parent(ml, z);
        uint32_t grandparent = rb_parent(ml, parent);
        if (parent == ml->elems[grandparent].tree_left) {
            uint32_t uncle = ml->elems[grandparent].tree_right;
            if (rb_is_red(ml, uncle)) {
                rb_set_red(ml, parent, false);
                rb_set_red(ml, uncle, false);
                rb_set_red(ml, grandparent, true);
                z = grandparent;
            } else {
                if (z == ml->elems[parent].tree_right) {
                    z = parent;
                    rb_rotate_left(ml, z);
                    parent = rb_parent(ml, z);
                    grandparent = rb_parent(ml, parent);
                }
                rb_set_red(ml, parent, false);
                rb_set_red(ml, grandparent, true);
                rb_rotate_right(ml, grandparent);
            }
        } else {
            uint32_t uncle = ml->elems[grandparent].tree_left;
            if (rb_is_red(ml, uncle)) {
                rb_set_red(ml, parent, false);
                rb_set_red(ml, uncle, false);
                rb_set_red(ml, grandparent, true);
                z = grandparent;
            } else {
                if (z == ml->elems[parent].tree_left) {
                    z = parent;
                    rb_rotate_right(ml, z);
                    parent = rb_parent(ml, z);
                    grandparent = rb_parent(ml, parent);
                }
                rb_set_red(ml, parent, false);
                rb_set_red(ml, grandparent, true);
                rb_rotate_left(ml, grandparent);
            }
        }
    }
    rb_set_red(ml, ml->tree_root, false);
}

static void rb_insert(ivp_minlist_t *ml, uint32_t z)
{
    uint32_t parent = IVP_MINLIST_TREE_UNUSED;
    uint32_t current = ml->tree_root;
    ivp_minlist_elem_t *elem = &ml->elems[z];
    while (current != IVP_MINLIST_TREE_UNUSED) {
        parent = current;
        if (rb_key_before(elem->value, &ml->elems[current])) {
            current = ml->elems[current].tree_left;
        } else {
            current = ml->elems[current].tree_right;
        }
    }
    elem->tree_left = IVP_MINLIST_TREE_UNUSED;
    elem->tree_right = IVP_MINLIST_TREE_UNUSED;
    elem->tree_parent_color = parent | IVP_MINLIST_RED;
    if (parent == IVP_MINLIST_TREE_UNUSED) {
        ml->tree_root = z;
    } else if (rb_key_before(elem->value, &ml->elems[parent])) {
        ml->elems[parent].tree_left = z;
    } else {
        ml->elems[parent].tree_right = z;
    }
    rb_insert_fixup(ml, z);
}

static void rb_transplant(ivp_minlist_t *ml, uint32_t u, uint32_t v)
{
    uint32_t parent = rb_parent(ml, u);
    if (parent == IVP_MINLIST_TREE_UNUSED) {
        ml->tree_root = v;
    } else if (u == ml->elems[parent].tree_left) {
        ml->elems[parent].tree_left = v;
    } else {
        ml->elems[parent].tree_right = v;
    }
    rb_set_parent(ml, v, parent);
}

static uint32_t rb_minimum(const ivp_minlist_t *ml, uint32_t index)
{
    while (ml->elems[index].tree_left != IVP_MINLIST_TREE_UNUSED) {
        index = ml->elems[index].tree_left;
    }
    return index;
}

static void rb_delete_fixup(ivp_minlist_t *ml, uint32_t x, uint32_t parent)
{
    while (x != ml->tree_root && !rb_is_red(ml, x)) {
        if (parent == IVP_MINLIST_TREE_UNUSED) break;
        if (x == ml->elems[parent].tree_left) {
            uint32_t sibling = ml->elems[parent].tree_right;
            if (rb_is_red(ml, sibling)) {
                rb_set_red(ml, sibling, false);
                rb_set_red(ml, parent, true);
                rb_rotate_left(ml, parent);
                sibling = ml->elems[parent].tree_right;
            }
            if (!rb_is_red(ml, rb_left(ml, sibling)) &&
                !rb_is_red(ml, rb_right(ml, sibling))) {
                rb_set_red(ml, sibling, true);
                x = parent;
                parent = rb_parent(ml, x);
            } else {
                if (!rb_is_red(ml, rb_right(ml, sibling))) {
                    rb_set_red(ml, rb_left(ml, sibling), false);
                    rb_set_red(ml, sibling, true);
                    rb_rotate_right(ml, sibling);
                    sibling = ml->elems[parent].tree_right;
                }
                rb_set_red(ml, sibling, rb_is_red(ml, parent));
                rb_set_red(ml, parent, false);
                rb_set_red(ml, rb_right(ml, sibling), false);
                rb_rotate_left(ml, parent);
                x = ml->tree_root;
                parent = IVP_MINLIST_TREE_UNUSED;
            }
        } else {
            uint32_t sibling = ml->elems[parent].tree_left;
            if (rb_is_red(ml, sibling)) {
                rb_set_red(ml, sibling, false);
                rb_set_red(ml, parent, true);
                rb_rotate_right(ml, parent);
                sibling = ml->elems[parent].tree_left;
            }
            if (!rb_is_red(ml, rb_right(ml, sibling)) &&
                !rb_is_red(ml, rb_left(ml, sibling))) {
                rb_set_red(ml, sibling, true);
                x = parent;
                parent = rb_parent(ml, x);
            } else {
                if (!rb_is_red(ml, rb_left(ml, sibling))) {
                    rb_set_red(ml, rb_right(ml, sibling), false);
                    rb_set_red(ml, sibling, true);
                    rb_rotate_left(ml, sibling);
                    sibling = ml->elems[parent].tree_left;
                }
                rb_set_red(ml, sibling, rb_is_red(ml, parent));
                rb_set_red(ml, parent, false);
                rb_set_red(ml, rb_left(ml, sibling), false);
                rb_rotate_right(ml, parent);
                x = ml->tree_root;
                parent = IVP_MINLIST_TREE_UNUSED;
            }
        }
    }
    rb_set_red(ml, x, false);
}

static void rb_remove(ivp_minlist_t *ml, uint32_t z)
{
    uint32_t y = z;
    bool y_was_red = rb_is_red(ml, y);
    uint32_t x;
    uint32_t x_parent;
    if (ml->elems[z].tree_left == IVP_MINLIST_TREE_UNUSED) {
        x = ml->elems[z].tree_right;
        x_parent = rb_parent(ml, z);
        rb_transplant(ml, z, x);
    } else if (ml->elems[z].tree_right == IVP_MINLIST_TREE_UNUSED) {
        x = ml->elems[z].tree_left;
        x_parent = rb_parent(ml, z);
        rb_transplant(ml, z, x);
    } else {
        y = rb_minimum(ml, ml->elems[z].tree_right);
        y_was_red = rb_is_red(ml, y);
        x = ml->elems[y].tree_right;
        if (rb_parent(ml, y) == z) {
            x_parent = y;
            rb_set_parent(ml, x, y);
        } else {
            x_parent = rb_parent(ml, y);
            rb_transplant(ml, y, x);
            ml->elems[y].tree_right = ml->elems[z].tree_right;
            rb_set_parent(ml, ml->elems[y].tree_right, y);
        }
        rb_transplant(ml, z, y);
        ml->elems[y].tree_left = ml->elems[z].tree_left;
        rb_set_parent(ml, ml->elems[y].tree_left, y);
        rb_set_red(ml, y, rb_is_red(ml, z));
    }
    if (!y_was_red) rb_delete_fixup(ml, x, x_parent);
}

void ivp_minlist_init(ivp_minlist_t *ml, const ivp_allocator_t *alloc, int start_size)
{
    if (!ml) return;
    if (!alloc) alloc = ivp_default_allocator();
    if (start_size < 4) start_size = 4;
    if ((uint32_t)start_size > IVP_MINLIST_MAX_ALLOC) start_size = (int)IVP_MINLIST_MAX_ALLOC;

    ml->alloc = alloc;
    ml->malloced_size = 0;
    ml->counter = 0;
    ml->free_list = IVP_MINLIST_UNUSED;
    ml->first_element = IVP_MINLIST_UNUSED;
    ml->first_long = IVP_MINLIST_UNUSED;
    ml->tree_root = IVP_MINLIST_TREE_UNUSED;
    ml->nonfinite_count = 0;
    ml->tree_active = false;
    ml->min_value = IVP_MINLIST_MAX_VALUE;
    ml->elems = (ivp_minlist_elem_t *)ivp_alloc(
        alloc, (size_t)start_size * sizeof(ivp_minlist_elem_t));
    if (!ml->elems) return;
    ml->malloced_size = (uint32_t)start_size;
    ml->free_list = 0;
    for (int i = 0; i < start_size; i++) ml->elems[i].next = (uint32_t)(i + 1);
    ml->elems[start_size - 1].next = IVP_MINLIST_UNUSED;
}

void ivp_minlist_destroy(ivp_minlist_t *ml)
{
    if (!ml) return;
    if (ml->elems) {
        ivp_free(ml->alloc, ml->elems,
                 (size_t)ml->malloced_size * sizeof(ivp_minlist_elem_t));
        ml->elems = NULL;
    }
    ml->malloced_size = 0;
    ml->free_list = IVP_MINLIST_UNUSED;
    ml->first_element = IVP_MINLIST_UNUSED;
    ml->first_long = IVP_MINLIST_UNUSED;
    ml->tree_root = IVP_MINLIST_TREE_UNUSED;
    ml->min_value = IVP_MINLIST_MAX_VALUE;
    ml->counter = 0;
    ml->nonfinite_count = 0;
    ml->tree_active = false;
}

static bool ivp_minlist_grow(ivp_minlist_t *ml, ivp_minlist_elem_t **out_e,
                             ivp_minlist_index_t *out_index)
{
    if (ml->malloced_size >= IVP_MINLIST_MAX_ALLOC) return false;
    uint64_t grown_size = (uint64_t)ml->malloced_size * 2u + 1u;
    uint32_t new_size = grown_size > IVP_MINLIST_MAX_ALLOC
                            ? IVP_MINLIST_MAX_ALLOC : (uint32_t)grown_size;
    ivp_minlist_elem_t *new_elems = (ivp_minlist_elem_t *)ivp_alloc(
        ml->alloc, (size_t)new_size * sizeof(ivp_minlist_elem_t));
    if (!new_elems) return false;
    for (uint32_t i = 0; i < ml->malloced_size; i++) new_elems[i] = ml->elems[i];
    uint32_t old_size = ml->malloced_size;
    ml->malloced_size = new_size;
    ivp_free(ml->alloc, ml->elems, (size_t)old_size * sizeof(ivp_minlist_elem_t));
    ml->elems = new_elems;
    *out_index = old_size;
    *out_e = &ml->elems[old_size];
    if (old_size + 1u < ml->malloced_size) {
        uint32_t i = old_size + 1u;
        ml->free_list = i;
        for (; i < ml->malloced_size; i++) ml->elems[i].next = i + 1u;
        ml->elems[ml->malloced_size - 1].next = IVP_MINLIST_UNUSED;
    } else {
        ml->free_list = IVP_MINLIST_UNUSED;
    }
    return true;
}

static void link_before(ivp_minlist_t *ml, uint32_t index, uint32_t next)
{
    ivp_minlist_elem_t *e = &ml->elems[index];
    e->next = next;
    if (next == IVP_MINLIST_UNUSED) {
        IVP_ASSERT(ml->first_element == IVP_MINLIST_UNUSED);
        e->prev = IVP_MINLIST_UNUSED;
        ml->first_element = index;
        return;
    }
    uint32_t prev = ml->elems[next].prev;
    e->prev = prev;
    ml->elems[next].prev = index;
    if (prev == IVP_MINLIST_UNUSED) {
        ml->first_element = index;
    } else {
        ml->elems[prev].next = index;
    }
}

static void link_after(ivp_minlist_t *ml, uint32_t index, uint32_t prev)
{
    ivp_minlist_elem_t *e = &ml->elems[index];
    e->prev = prev;
    e->next = ml->elems[prev].next;
    if (e->next != IVP_MINLIST_UNUSED) ml->elems[e->next].prev = index;
    ml->elems[prev].next = index;
}

static void find_tree_neighbors(const ivp_minlist_t *ml, float value,
                                uint32_t *out_prev, uint32_t *out_next)
{
    uint32_t prev = IVP_MINLIST_UNUSED;
    uint32_t next = IVP_MINLIST_UNUSED;
    uint32_t current = ml->tree_root;
    while (current != IVP_MINLIST_TREE_UNUSED) {
        COUNT_COMPARISON();
        if (rb_key_before(value, &ml->elems[current])) {
            next = current;
            current = ml->elems[current].tree_left;
        } else {
            prev = current;
            current = ml->elems[current].tree_right;
        }
    }
    *out_prev = prev;
    *out_next = next;
}

static void activate_tree_index(ivp_minlist_t *ml)
{
    if (ml->tree_active || ml->nonfinite_count != 0 ||
        ml->counter < IVP_MINLIST_TREE_THRESHOLD) {
        return;
    }
    uint32_t tail = ml->first_element;
    if (tail != IVP_MINLIST_UNUSED) {
        while (ml->elems[tail].next != IVP_MINLIST_UNUSED) tail = ml->elems[tail].next;
    }
    ml->first_long = IVP_MINLIST_UNUSED;
    ml->tree_root = IVP_MINLIST_TREE_UNUSED;
    ml->tree_active = true;
    while (tail != IVP_MINLIST_UNUSED) {
        uint32_t prev = ml->elems[tail].prev;
        ml->elems[tail].tree_left = IVP_MINLIST_TREE_UNUSED;
        ml->elems[tail].tree_right = IVP_MINLIST_TREE_UNUSED;
        ml->elems[tail].tree_parent_color = IVP_MINLIST_TREE_UNUSED;
        rb_insert(ml, tail);
        tail = prev;
    }
}

static void link_with_reference_skip_search(ivp_minlist_t *ml, uint32_t index)
{
    ivp_minlist_elem_t *e = &ml->elems[index];
    e->tree_left = IVP_MINLIST_LONG_UNUSED;
    if (ml->first_element == IVP_MINLIST_UNUSED || e->value <= ml->min_value) {
        link_before(ml, index, ml->first_element);
        return;
    }

    /* Reuse the previous skip-search prefix, but keep the reference's
     * first_after, comparison limit and auxiliary-node creation unchanged.
     * Their history matters even for nonfinite comparison semantics. */
    uint32_t last = ml->first_element;
    uint32_t long_index = ml->first_long;
    uint32_t anchor = IVP_MINLIST_UNUSED;
    int prefix_length = 0;
    uint32_t cached = ml->tree_root; /* absent below the cache threshold */
    if (cached < ml->malloced_size && isfinite(e->value)) {
#ifdef IVP_BENCHMARK_DIAGNOSTICS
        minlist_diagnostics.finger_attempts++;
#endif
        prefix_length = (int)ml->elems[cached].tree_parent_color;
        uint32_t walked = 0;
        while (cached != IVP_MINLIST_UNUSED) {
            COUNT_COMPARISON();
            if (!(ml->elems[cached].value >= e->value)) break;
            if (walked++ == IVP_MINLIST_CACHE_BACKTRACK) {
                cached = IVP_MINLIST_UNUSED;
                prefix_length = 0;
                break;
            }
            cached = ml->elems[cached].tree_right;
            prefix_length--;
        }
        if (cached != IVP_MINLIST_UNUSED) {
#ifdef IVP_BENCHMARK_DIAGNOSTICS
            minlist_diagnostics.finger_hits++;
#endif
            anchor = cached;
            last = cached;
            long_index = ml->elems[cached].tree_left;
        }
    }
    int max_compare_length = 3 + prefix_length;
    while (long_index != IVP_MINLIST_UNUSED) {
        COUNT_COMPARISON();
        ivp_minlist_elem_t *long_elem = &ml->elems[long_index];
        if (long_elem->value >= e->value) break;
        max_compare_length++;
        anchor = long_index;
        last = long_index;
        long_index = long_elem->tree_left;
    }

    uint32_t first_after = last;
    int comparisons = 0;
    uint32_t cached_rank;
    uint32_t current = ml->elems[last].next;
    while (current != IVP_MINLIST_UNUSED) {
        COUNT_COMPARISON();
        comparisons++;
        if (e->value > ml->elems[current].value) {
            last = current;
            current = ml->elems[current].next;
            continue;
        }
        link_before(ml, index, current);
        goto update_skip;
    }
    link_after(ml, index, last);

update_skip:
    cached_rank = (uint32_t)(max_compare_length - 3);
    if (comparisons > max_compare_length) {
        uint32_t new_long = first_after;
        for (int k = 2; k < max_compare_length; k++) new_long = ml->elems[new_long].next;
        uint32_t next_of_first = ml->elems[first_after].tree_left;
        ivp_minlist_elem_t *new_long_elem = &ml->elems[new_long];
        IVP_ASSERT(new_long_elem->tree_left == IVP_MINLIST_LONG_UNUSED);
        anchor = new_long;
        cached_rank++;
        if (next_of_first != IVP_MINLIST_LONG_UNUSED) {
            new_long_elem->tree_left = next_of_first;
            new_long_elem->tree_right = first_after;
            if (next_of_first != IVP_MINLIST_UNUSED) {
                ml->elems[next_of_first].tree_right = new_long;
            }
            ml->elems[first_after].tree_left = new_long;
        } else {
            new_long_elem->tree_left = ml->first_long;
            new_long_elem->tree_right = IVP_MINLIST_UNUSED;
            if (ml->first_long != IVP_MINLIST_UNUSED) {
                ml->elems[ml->first_long].tree_right = new_long;
            }
            ml->first_long = new_long;
        }
    }
    if (ml->counter >= IVP_MINLIST_CACHE_THRESHOLD &&
        ml->tree_root != IVP_MINLIST_LONG_UNUSED) {
        ml->tree_root = anchor == IVP_MINLIST_UNUSED ? IVP_MINLIST_TREE_UNUSED : anchor;
        if (anchor != IVP_MINLIST_UNUSED) ml->elems[anchor].tree_parent_color = cached_rank;
    }
}

static void link_with_linear_reference_search(ivp_minlist_t *ml, uint32_t index)
{
    ivp_minlist_elem_t *e = &ml->elems[index];
    if (ml->first_element == IVP_MINLIST_UNUSED || e->value <= ml->min_value) {
        link_before(ml, index, ml->first_element);
        return;
    }
    uint32_t last = ml->first_element;
    uint32_t current = ml->elems[last].next;
    while (current != IVP_MINLIST_UNUSED) {
        COUNT_COMPARISON();
        if (e->value > ml->elems[current].value) {
            last = current;
            current = ml->elems[current].next;
            continue;
        }
        link_before(ml, index, current);
        return;
    }
    link_after(ml, index, last);
}

ivp_minlist_index_t ivp_minlist_add(ivp_minlist_t *ml, void *elem, float value)
{
    IVP_ASSERT(!isfinite(value) || value <= P_FLOAT_MAX);
    if (!ml || !ml->elems) return IVP_MINLIST_UNUSED;
    activate_tree_index(ml);
    ivp_minlist_elem_t *e;
    ivp_minlist_index_t index;
    if (ml->free_list != IVP_MINLIST_UNUSED) {
        IVP_ASSERT(ml->free_list < ml->malloced_size);
        index = ml->free_list;
        e = &ml->elems[index];
        ml->free_list = e->next;
    } else {
        IVP_ASSERT(ml->malloced_size != IVP_MINLIST_MAX_ALLOC);
        if (ml->malloced_size == IVP_MINLIST_MAX_ALLOC) {
            ivp_message("Fatal error: IVP_U_Min_List reached maximum capacity (%u)\n",
                        (unsigned int)IVP_MINLIST_MAX_ALLOC);
            IVP_CORE();
            return IVP_MINLIST_UNUSED;
        }
        if (!ivp_minlist_grow(ml, &e, &index)) return IVP_MINLIST_UNUSED;
    }

    e->element = elem;
    e->value = value;

    const bool finite = isfinite(value) != 0;
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    search_comparisons = 0;
    minlist_diagnostics.adds++;
    if (ml->counter > minlist_diagnostics.max_population)
        minlist_diagnostics.max_population = ml->counter;
    uint32_t size = ml->counter;
    int bucket = size < 16 ? 0 : size < 64 ? 1 : size < 256 ? 2 : size < 1024 ? 3 : size < 4096 ? 4 : 5;
    minlist_diagnostics.population_buckets[bucket]++;
    if (!ml->tree_active) minlist_diagnostics.skip_adds++;
    else if (finite && ml->nonfinite_count == 0) minlist_diagnostics.tree_adds++;
    else minlist_diagnostics.fallback_adds++;
#endif
    if (!ml->tree_active) {
        link_with_reference_skip_search(ml, index);
    } else if (finite && ml->nonfinite_count == 0) {
        uint32_t prev, next;
        find_tree_neighbors(ml, value, &prev, &next);
        if (next != IVP_MINLIST_UNUSED) {
            link_before(ml, index, next);
        } else if (prev != IVP_MINLIST_UNUSED) {
            link_after(ml, index, prev);
        } else {
            link_before(ml, index, ml->first_element);
        }
    } else {
        link_with_linear_reference_search(ml, index);
    }
    if (finite && ml->tree_active) {
        rb_insert(ml, index);
    } else if (!finite) {
        ml->nonfinite_count++;
        /* Its skip chain need not stay ordered even after later removal. */
        if (!ml->tree_active) ml->tree_root = IVP_MINLIST_LONG_UNUSED;
    }
    ml->counter++;
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    minlist_diagnostics.comparisons += search_comparisons;
    if (search_comparisons > minlist_diagnostics.max_comparisons)
        minlist_diagnostics.max_comparisons = search_comparisons;
#endif
    ml->min_value = ml->elems[ml->first_element].value;
    return index;
}

void ivp_minlist_remove(ivp_minlist_t *ml, ivp_minlist_index_t index)
{
    if (!ml || !ml->elems) return;
    IVP_ASSERT(index < ml->malloced_size);
    if (index >= ml->malloced_size || ml->counter == 0) return;
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    minlist_diagnostics.removes++;
#endif
    ivp_minlist_elem_t *e = &ml->elems[index];
    uint32_t prev = e->prev;
    uint32_t next = e->next;
    if (ml->tree_root < ml->malloced_size && !ml->tree_active &&
        e->tree_left != IVP_MINLIST_LONG_UNUSED) {
        /* Only skip-member removal changes the cached prefix length.
         * Equal keys do not identify their position: discard that cache. */
        uint32_t cached = ml->tree_root;
        uint32_t rank = ml->elems[cached].tree_parent_color;
        if (cached == index) {
            cached = e->tree_right;
            ml->tree_root = cached == IVP_MINLIST_UNUSED ? IVP_MINLIST_TREE_UNUSED : cached;
            if (cached != IVP_MINLIST_UNUSED) ml->elems[cached].tree_parent_color = rank - 1u;
        } else if (e->value < ml->elems[cached].value) {
            ml->elems[cached].tree_parent_color = rank - 1u;
        } else if (e->value == ml->elems[cached].value) {
            ml->tree_root = IVP_MINLIST_TREE_UNUSED;
        }
    }
    if (prev != IVP_MINLIST_UNUSED) {
        ml->elems[prev].next = next;
    } else {
        ml->first_element = next;
    }
    if (next != IVP_MINLIST_UNUSED) ml->elems[next].prev = prev;

    if (ml->tree_active && isfinite(e->value)) {
        rb_remove(ml, index);
    } else if (!isfinite(e->value)) {
        IVP_ASSERT(ml->nonfinite_count > 0);
        ml->nonfinite_count--;
    } else {
        uint32_t long_next = e->tree_left;
        if (long_next != IVP_MINLIST_LONG_UNUSED) {
            uint32_t long_prev = e->tree_right;
            if (long_next != IVP_MINLIST_UNUSED) {
                ml->elems[long_next].tree_right = long_prev;
            }
            if (long_prev == IVP_MINLIST_UNUSED) {
                ml->first_long = long_next;
            } else {
                ml->elems[long_prev].tree_left = long_next;
            }
        }
    }
    ml->counter--;
    if (ml->tree_root < ml->malloced_size && !ml->tree_active &&
        ml->counter < IVP_MINLIST_CACHE_THRESHOLD)
        ml->tree_root = IVP_MINLIST_TREE_UNUSED;
    ml->min_value = ml->counter == 0 ? IVP_MINLIST_MAX_VALUE
                                      : ml->elems[ml->first_element].value;
    e->element = NULL;
    e->value = IVP_MINLIST_MAX_VALUE;
    e->prev = IVP_MINLIST_UNUSED;
    e->tree_left = IVP_MINLIST_TREE_UNUSED;
    e->tree_right = IVP_MINLIST_TREE_UNUSED;
    e->tree_parent_color = IVP_MINLIST_TREE_UNUSED;
    e->next = ml->free_list;
    ml->free_list = index;
    activate_tree_index(ml);
}
