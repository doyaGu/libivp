/* Test model of IVP_U_Min_List's original adaptive skip chain.
 * Keep the bookkeeping literal: NaN insertion can depend on its history.
 * Fixed-capacity, below the indexed threshold; no allocation/growth logic.
 */
#ifndef IVP_TEST_MINLIST_REFERENCE_MODEL_H
#define IVP_TEST_MINLIST_REFERENCE_MODEL_H
static void minlist_model_link_before(ivp_minlist_t *ml, uint32_t index, uint32_t next)
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

static void minlist_model_link_after(ivp_minlist_t *ml, uint32_t index, uint32_t prev)
{
    ivp_minlist_elem_t *e = &ml->elems[index];
    e->prev = prev;
    e->next = ml->elems[prev].next;
    if (e->next != IVP_MINLIST_UNUSED) ml->elems[e->next].prev = index;
    ml->elems[prev].next = index;
}

static void minlist_model_link(ivp_minlist_t *ml, uint32_t index)
{
    ivp_minlist_elem_t *e = &ml->elems[index];
    e->tree_left = IVP_MINLIST_LONG_UNUSED;
    if (ml->first_element == IVP_MINLIST_UNUSED || e->value <= ml->min_value) {
        minlist_model_link_before(ml, index, ml->first_element);
        return;
    }

    uint32_t last = ml->first_element;
    uint32_t long_index = ml->first_long;
    int max_compare_length = 3;
    while (long_index != IVP_MINLIST_UNUSED) {
        ivp_minlist_elem_t *long_elem = &ml->elems[long_index];
        if (long_elem->value >= e->value) break;
        max_compare_length++;
        last = long_index;
        long_index = long_elem->tree_left;
    }

    uint32_t first_after = last;
    int comparisons = 0;
    uint32_t current = ml->elems[last].next;
    while (current != IVP_MINLIST_UNUSED) {
        comparisons++;
        if (e->value > ml->elems[current].value) {
            last = current;
            current = ml->elems[current].next;
            continue;
        }
        minlist_model_link_before(ml, index, current);
        goto update_skip;
    }
    minlist_model_link_after(ml, index, last);

update_skip:
    if (comparisons > max_compare_length) {
        uint32_t new_long = first_after;
        for (int k = 2; k < max_compare_length; k++) new_long = ml->elems[new_long].next;
        uint32_t next_of_first = ml->elems[first_after].tree_left;
        ivp_minlist_elem_t *new_long_elem = &ml->elems[new_long];
        IVP_ASSERT(new_long_elem->tree_left == IVP_MINLIST_LONG_UNUSED);
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
}


static uint32_t minlist_model_add(ivp_minlist_t *ml, void *element, float value)
{
    uint32_t index = ml->free_list;
    IVP_ASSERT(index != IVP_MINLIST_UNUSED);
    ivp_minlist_elem_t *e = &ml->elems[index];
    ml->free_list = e->next;
    e->element = element;
    e->value = value;
    minlist_model_link(ml, index);
    ml->counter++;
    ml->min_value = ml->elems[ml->first_element].value;
    return index;
}
static void minlist_model_remove(ivp_minlist_t *ml, uint32_t index)
{
    ivp_minlist_elem_t *e = &ml->elems[index];
    uint32_t prev = e->prev, next = e->next;
    if (prev != IVP_MINLIST_UNUSED) ml->elems[prev].next = next;
    else ml->first_element = next;
    if (next != IVP_MINLIST_UNUSED) ml->elems[next].prev = prev;
    if (e->tree_left != IVP_MINLIST_LONG_UNUSED) {
        uint32_t skip_next = e->tree_left, skip_prev = e->tree_right;
        if (skip_next != IVP_MINLIST_UNUSED) ml->elems[skip_next].tree_right = skip_prev;
        if (skip_prev != IVP_MINLIST_UNUSED) ml->elems[skip_prev].tree_left = skip_next;
        else ml->first_long = skip_next;
    }
    ml->counter--;
    ml->min_value = ml->counter ? ml->elems[ml->first_element].value : IVP_MINLIST_MAX_VALUE;
    e->element = NULL;
    e->value = IVP_MINLIST_MAX_VALUE;
    e->tree_left = IVP_MINLIST_TREE_UNUSED;
    e->next = ml->free_list;
    ml->free_list = index;
}
#endif

