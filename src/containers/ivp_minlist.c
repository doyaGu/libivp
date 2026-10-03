/* ivp_minlist.c -- Sorted minimum list implementation
 *
 * Ported from ivu_min_list.cxx (Ipion Software GmbH).
 * Sorted insertion with skip-list ("long jump") acceleration.
 */
#include <ivp/containers/ivp_minlist.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_legacy.h>
#include <ivp/base/ivp_message.h>

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

void ivp_minlist_init(ivp_minlist_t *ml, const ivp_allocator_t *alloc, int start_size) {
    if (!ml) return;
    if (!alloc) alloc = ivp_default_allocator();
    if (start_size > IVP_MINLIST_MAX_ALLOC) {
        start_size = IVP_MINLIST_MAX_ALLOC;
    }
    if (start_size < 4) start_size = 4;

    ml->alloc = alloc;
    ml->malloced_size = 0;
    ml->counter = 0;
    ml->free_list = IVP_MINLIST_UNUSED;
    ml->first_element = IVP_MINLIST_UNUSED;
    ml->first_long = IVP_MINLIST_UNUSED;
    ml->min_value = IVP_MINLIST_MAX_VALUE;
    ml->elems = NULL;

    ml->elems = (ivp_minlist_elem_t *)ivp_alloc(alloc,
        (size_t)start_size * sizeof(ivp_minlist_elem_t));
    if (!ml->elems) {
        return;
    }
    ml->malloced_size = (uint16_t)start_size;

    /* Build free list */
    ml->free_list = 0;
    for (int i = 0; i < start_size; i++) {
        ml->elems[i].next = (uint16_t)(i + 1);
    }
    ml->elems[start_size - 1].next = IVP_MINLIST_UNUSED;
}

void ivp_minlist_destroy(ivp_minlist_t *ml) {
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
    ml->min_value = IVP_MINLIST_MAX_VALUE;
    ml->counter = 0;
}

/* ── Grow ──────────────────────────────────────────────────────────────── */

static bool ivp_minlist_grow(ivp_minlist_t *ml, ivp_minlist_elem_t **out_e,
                             ivp_minlist_index_t *out_index) {
    if (ml->malloced_size >= IVP_MINLIST_MAX_ALLOC) return false;

    int new_size = ml->malloced_size * 2 + 1;
    if (new_size > IVP_MINLIST_MAX_ALLOC) {
        new_size = IVP_MINLIST_MAX_ALLOC;
    }

    ivp_minlist_elem_t *new_elems = (ivp_minlist_elem_t *)ivp_alloc(ml->alloc,
        (size_t)new_size * sizeof(ivp_minlist_elem_t));
    if (!new_elems) return false;

    for (int i = 0; i < ml->malloced_size; i++) {
        new_elems[i] = ml->elems[i];
    }

    uint16_t old_size = ml->malloced_size;
    ml->malloced_size = (uint16_t)new_size;

    ivp_free(ml->alloc, ml->elems,
              (size_t)old_size * sizeof(ivp_minlist_elem_t));
    ml->elems = new_elems;

    *out_index = old_size;
    *out_e = &ml->elems[old_size];
    if ((int)old_size + 1 < ml->malloced_size) {
        uint16_t i = (uint16_t)(old_size + 1);
        ml->free_list = i;
        for (; i < ml->malloced_size; i++) {
            ml->elems[i].next = (uint16_t)(i + 1);
        }
        ml->elems[ml->malloced_size - 1].next = IVP_MINLIST_UNUSED;
    } else {
        ml->free_list = IVP_MINLIST_UNUSED;
    }
    return true;
}

/* ── Add ───────────────────────────────────────────────────────────────── */

ivp_minlist_index_t ivp_minlist_add(ivp_minlist_t *ml, void *elem, float value) {
    /* search free element first */
    IVP_ASSERT(value <= P_FLOAT_MAX);
    if (!ml || !ml->elems) return IVP_MINLIST_UNUSED;

    ivp_minlist_elem_t *e;
    ivp_minlist_index_t return_index;

    if (ml->free_list != IVP_MINLIST_UNUSED) {
        IVP_ASSERT(ml->free_list < ml->malloced_size);
        return_index = ml->free_list;
        e = &ml->elems[ml->free_list];
        ml->free_list = e->next;
    } else {
        /* If this assertion triggers, it's going to overflow... (a full list
         * has counter == malloced_size == IVP_MINLIST_MAX_ALLOC) */
        IVP_ASSERT(ml->malloced_size != IVP_MINLIST_MAX_ALLOC);
        if (ml->malloced_size == IVP_MINLIST_MAX_ALLOC) {
            ivp_message("Fatal error: IVP_U_Min_List reached maximum capacity (%u)\n",
                        (unsigned int)IVP_MINLIST_MAX_ALLOC);
            IVP_CORE();
            return IVP_MINLIST_UNUSED;
        }
        if (!ivp_minlist_grow(ml, &e, &return_index)) {
            return IVP_MINLIST_UNUSED;
        }
    }
    ml->counter++;

    e->element = elem;
    e->value = value;
    e->long_next = IVP_MINLIST_LONG_UNUSED;

    /* Fast path: insert as new minimum */
    if (value <= ml->min_value) {
        ml->min_value = value;
        e->next = ml->first_element;
        if (ml->first_element != IVP_MINLIST_UNUSED) {
            ml->elems[ml->first_element].prev = return_index;
        }
        ml->first_element = return_index;
        IVP_ASSERT((ml->first_element == IVP_MINLIST_UNUSED) || (ml->first_element < ml->malloced_size));
        e->prev = IVP_MINLIST_UNUSED;
        return return_index;
    }

    IVP_ASSERT(ml->first_element != IVP_MINLIST_UNUSED);

    /* Use long-jump skip list to find insertion region */
    int lastj = ml->first_element;
    int lo = ml->first_long;
    int max_cmp_len = 3;
    while (lo != IVP_MINLIST_UNUSED) {
        ivp_minlist_elem_t *flong = &ml->elems[lo];
        if (flong->value >= value) break;
        max_cmp_len++;
        lastj = lo;
        lo = flong->long_next;
    }

    int firstj_after = lastj;
    int count_cmp = 0;

    /* Linear search from lastj */
    ivp_minlist_elem_t *f = &ml->elems[lastj];
    int j;
    for (j = f->next; j != IVP_MINLIST_UNUSED; j = f->next) {
        f = &ml->elems[j];
        count_cmp++;
        if (value > f->value) {
            lastj = j;
            continue;
        }
        /* Insert before j */
        ivp_minlist_elem_t *l = &ml->elems[lastj];
        e->next = l->next;
        ml->elems[j].prev = return_index;
        e->prev = (uint16_t)lastj;
        l->next = return_index;
        goto end;
    }
    /* Insert at end */
    e->next = (uint16_t)j;
    e->prev = (uint16_t)lastj;
    f->next = return_index;

end:
    /* Insert long-jump node if search was too long */
    if (count_cmp > max_cmp_len) {
        int new_long_pos = firstj_after;
        for (int k = 2; k < max_cmp_len; k++) {
            new_long_pos = ml->elems[new_long_pos].next;
        }

        uint16_t next_of_first = ml->elems[firstj_after].long_next;
        ivp_minlist_elem_t *nl = &ml->elems[new_long_pos];
        IVP_ASSERT(nl->long_next == IVP_MINLIST_LONG_UNUSED);

        if (next_of_first != IVP_MINLIST_LONG_UNUSED) {
            nl->long_next = next_of_first;
            nl->long_prev = (uint16_t)firstj_after;
            if (next_of_first != IVP_MINLIST_UNUSED) {
                ml->elems[next_of_first].long_prev = (uint16_t)new_long_pos;
            }
            ml->elems[firstj_after].long_next = (uint16_t)new_long_pos;
        } else {
            nl->long_next = ml->first_long;
            nl->long_prev = IVP_MINLIST_UNUSED;
            if (ml->first_long != IVP_MINLIST_UNUSED) {
                ml->elems[ml->first_long].long_prev = (uint16_t)new_long_pos;
            }
            ml->first_long = (uint16_t)new_long_pos;
        }
    }

    return return_index;
}

/* ── Remove ────────────────────────────────────────────────────────────── */

void ivp_minlist_remove(ivp_minlist_t *ml, ivp_minlist_index_t index) {
    if (!ml || !ml->elems) return;
    IVP_ASSERT(index < ml->malloced_size);
    if (index >= ml->malloced_size || ml->counter == 0) return;

    ivp_minlist_elem_t *e = &ml->elems[index];
    uint16_t prev = e->prev;
    uint16_t next = e->next;

    if (prev != IVP_MINLIST_UNUSED) {
        ml->elems[prev].next = next;
        if (next != IVP_MINLIST_UNUSED) {
            ml->elems[next].prev = prev;
        }
    } else {
        ml->first_element = next;
        IVP_ASSERT((ml->first_element == IVP_MINLIST_UNUSED) || (ml->first_element < ml->malloced_size));
        if (next != IVP_MINLIST_UNUSED) {
            ml->elems[next].prev = prev;
            ml->min_value = ml->elems[next].value;
        } else {
            ml->min_value = IVP_MINLIST_MAX_VALUE;
        }
    }

    /* Remove from skip-list if this node is a long-jump node */
    uint16_t long_next = e->long_next;
    if (long_next != IVP_MINLIST_LONG_UNUSED) {
        uint16_t long_prev = e->long_prev;
        if (long_next != IVP_MINLIST_UNUSED) {
            ml->elems[long_next].long_prev = long_prev;
        }
        if (long_prev == IVP_MINLIST_UNUSED) {
            ml->first_long = long_next;
        } else {
            ml->elems[long_prev].long_next = long_next;
        }
    }

    ml->counter--;
    e->element = NULL;
    e->value = IVP_MINLIST_MAX_VALUE;
    e->prev = IVP_MINLIST_UNUSED;
    e->long_prev = IVP_MINLIST_UNUSED;
    e->long_next = IVP_MINLIST_LONG_UNUSED;
    e->next = ml->free_list;
    ml->free_list = index;
}
