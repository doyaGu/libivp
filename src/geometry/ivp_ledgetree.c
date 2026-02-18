/* ivp_ledgetree.c — Ledgetree traversal functions */

#include <ivp/geometry/ivp_ledgetree.h>

static bool ivp_ledgetree_visit_all_impl(const ivp_compact_ledgetree_node_t *root,
                                          ivp_ledgetree_visitor_fn visitor,
                                          void *user_data)
{
    if (!root || !visitor) return true;

    if (ivp_ledgetree_is_terminal(root)) {
        const ivp_compact_ledge_t *ledge = ivp_ledgetree_get_compact_ledge(root);
        return visitor(ledge, user_data);
    }

    /* Depth-first, left then right. Stop immediately if visitor returns false. */
    if (!ivp_ledgetree_visit_all_impl(ivp_ledgetree_left_son(root), visitor, user_data)) {
        return false;
    }
    return ivp_ledgetree_visit_all_impl(ivp_ledgetree_right_son(root), visitor, user_data);
}

void ivp_ledgetree_visit_all(const ivp_compact_ledgetree_node_t *root,
                              ivp_ledgetree_visitor_fn visitor,
                              void *user_data)
{
    (void)ivp_ledgetree_visit_all_impl(root, visitor, user_data);
}

static bool count_visitor(const ivp_compact_ledge_t *ledge, void *user_data) {
    (void)ledge;
    int *count = (int *)user_data;
    (*count)++;
    return true;
}

int ivp_ledgetree_count_terminal_ledges(
    const ivp_compact_ledgetree_node_t *root)
{
    int count = 0;
    ivp_ledgetree_visit_all(root, count_visitor, &count);
    return count;
}
