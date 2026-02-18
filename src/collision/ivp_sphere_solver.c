/* ivp_sphere_solver.c -- Sphere overlap queries against compact geometry
 *
 * Traverses compact ledgetree BVH checking sphere-sphere overlap at each
 * node. This is the object-level spatial query used by the broadphase.
 *
 * Ported from ivp_sphere_solver.cxx.
 */
#include <ivp/collision/ivp_sphere_solver.h>
#include <ivp/base/ivp_assert.h>

/* ── Recursive BVH traversal ───────────────────────────────────────────── */

static bool sphere_vs_node(const ivp_compact_ledgetree_node_t *node,
                            const ivp_vec3r_t *center_os,
                            ivp_real_t radius,
                            int depth,
                            int max_depth)
{
    /* Check sphere-sphere overlap: query sphere vs node bounding sphere. */
    ivp_vec3r_t node_center = ivp_vec3r(
        (ivp_real_t)node->center[0],
        (ivp_real_t)node->center[1],
        (ivp_real_t)node->center[2]);

    ivp_real_t sum_r = radius + (ivp_real_t)node->radius;
    ivp_real_t qdist = ivp_vec3r_quad_distance(center_os, &node_center);

    if (qdist > sum_r * sum_r) {
        return false; /* no overlap with this subtree */
    }

    /* Overlap detected. If terminal or at max depth, report hit. */
    if (ivp_ledgetree_is_terminal(node) || depth >= max_depth) {
        return true;
    }

    /* Recurse into children. */
    if (sphere_vs_node(ivp_ledgetree_left_son(node),
                        center_os, radius, depth + 1, max_depth)) {
        return true;
    }
    return sphere_vs_node(ivp_ledgetree_right_son(node),
                           center_os, radius, depth + 1, max_depth);
}

/* ── Public API ────────────────────────────────────────────────────────── */

bool ivp_sphere_overlaps_ledgetree(
    const ivp_compact_ledgetree_node_t *root,
    const ivp_vec3r_t *center_os,
    ivp_real_t radius,
    int max_depth)
{
    IVP_ASSERT(root != NULL);
    IVP_ASSERT(center_os != NULL);
    IVP_ASSERT(radius >= IVP_REAL(0.0));

    if (max_depth < 0) max_depth = 0;

    return sphere_vs_node(root, center_os, radius, 0, max_depth);
}

bool ivp_sphere_overlaps_surface(
    const ivp_compact_surface_t *surface,
    const ivp_vec3r_t *center_os,
    ivp_real_t radius,
    int max_depth)
{
    IVP_ASSERT(surface != NULL);
    IVP_ASSERT(center_os != NULL);

    const ivp_compact_ledgetree_node_t *root =
        ivp_surface_get_ledgetree_root(surface);

    return ivp_sphere_overlaps_ledgetree(root, center_os, radius, max_depth);
}
