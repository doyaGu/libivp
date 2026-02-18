/* ivp_compact_surface.c — Compact surface functions */

#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/base/ivp_allocator.h>

const ivp_compact_ledge_t *ivp_surface_get_single_convex(
    const ivp_compact_surface_t *s)
{
    const ivp_compact_ledgetree_node_t *root = ivp_surface_get_ledgetree_root(s);
    if (ivp_ledgetree_is_terminal(root)) {
        return ivp_ledgetree_get_compact_ledge(root);
    }
    return NULL;
}

void ivp_compact_surface_free(const ivp_allocator_t *alloc,
                               ivp_compact_surface_t *s)
{
    if (s) {
        if (!alloc) alloc = ivp_default_allocator();
        ivp_free_aligned(alloc, s);
    }
}
