/* ivp_mindist_recursive.c — Recursive/compound mindist delegator */

#include <ivp/collision/ivp_mindist_recursive.h>

#include <string.h>

static bool ensure_capacity(ivp_mindist_recursive_t *recursive, int needed)
{
    if (!recursive) return false;
    if (needed <= recursive->child_capacity) return true;

    int new_capacity = recursive->child_capacity > 0 ? recursive->child_capacity * 2 : 8;
    if (new_capacity < needed) new_capacity = needed;

    size_t new_size = (size_t)new_capacity * sizeof(ivp_mindist_entry_t *);
    ivp_mindist_entry_t **new_children =
        (ivp_mindist_entry_t **)ivp_alloc(recursive->alloc, new_size);
    if (!new_children) return false;

    memset(new_children, 0, new_size);

    if (recursive->children && recursive->child_count > 0) {
        size_t old_size = (size_t)recursive->child_count * sizeof(ivp_mindist_entry_t *);
        memcpy(new_children, recursive->children, old_size);
        ivp_free(recursive->alloc,
                 recursive->children,
                 (size_t)recursive->child_capacity * sizeof(ivp_mindist_entry_t *));
    }

    recursive->children = new_children;
    recursive->child_capacity = new_capacity;
    return true;
}

void ivp_mindist_recursive_init(ivp_mindist_recursive_t *recursive,
                                const ivp_allocator_t *alloc)
{
    if (!recursive) return;
    memset(recursive, 0, sizeof(*recursive));
    recursive->alloc = alloc ? alloc : ivp_default_allocator();
}

void ivp_mindist_recursive_destroy(ivp_mindist_recursive_t *recursive)
{
    if (!recursive) return;
    if (recursive->children) {
        ivp_free(recursive->alloc,
                 recursive->children,
                 (size_t)recursive->child_capacity * sizeof(ivp_mindist_entry_t *));
    }
    memset(recursive, 0, sizeof(*recursive));
}

bool ivp_mindist_recursive_add_child(ivp_mindist_recursive_t *recursive,
                                     ivp_mindist_entry_t *entry)
{
    if (!recursive || !entry) return false;

    for (int i = 0; i < recursive->child_count; i++) {
        if (recursive->children[i] == entry) return true;
    }

    if (!ensure_capacity(recursive, recursive->child_count + 1)) {
        return false;
    }

    recursive->children[recursive->child_count++] = entry;
    entry->state = IVP_MDIST_HULL_RECURSIVE;
    return true;
}

void ivp_mindist_recursive_remove_child(ivp_mindist_recursive_t *recursive,
                                        ivp_mindist_entry_t *entry)
{
    if (!recursive || !entry) return;

    for (int i = 0; i < recursive->child_count; i++) {
        if (recursive->children[i] != entry) continue;

        for (int j = i; j < recursive->child_count - 1; j++) {
            recursive->children[j] = recursive->children[j + 1];
        }
        recursive->child_count--;
        entry->state = IVP_MDIST_INVALID;
        return;
    }
}

void ivp_mindist_recursive_invalidate(ivp_mindist_recursive_t *recursive)
{
    if (!recursive) return;
    for (int i = 0; i < recursive->child_count; i++) {
        ivp_mindist_entry_t *entry = recursive->children[i];
        if (!entry) continue;
        entry->state = IVP_MDIST_INVALID;
    }
}
