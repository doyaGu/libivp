/* ivp_ledgetree.h — Compact ledgetree node (28 bytes)
 *
 * Binary tree for hierarchical collision detection of compound shapes.
 * Single convex shapes have exactly one terminal node.
 *
 * Navigation:
 *   - Terminal: offset_right_node == 0 → get_compact_ledge()
 *   - Non-terminal: left_son() = this + 1, right_son() = (char*)this + offset
 *
 * Bounding volume:
 *   - center + radius → bounding sphere (object coordinates)
 *   - box_sizes[3] → AABB half-extents encoded as:
 *     box_sizes[i] = int(extent_i / (radius * IVP_COMPACT_BOUNDINGBOX_STEP_SIZE)) + 1
 */
#ifndef IVP_LEDGETREE_H
#define IVP_LEDGETREE_H

#include <ivp/geometry/ivp_compact_ledge.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ──────────────────────────────────────────────────────────── */

#define IVP_COMPACT_BOUNDINGBOX_STEP_SIZE  (1.0f / 250.0f)
#define IVP_COMPACT_SURFACE_DEVIATION_STEP_SIZE (1.0f / 250.0f)

/* ── Ledgetree node structure ───────────────────────────────────────────── */

typedef struct ivp_compact_ledgetree_node {
    int32_t  offset_right_node;      /* 0 if terminal */
    int32_t  offset_compact_ledge;   /* offset to associated ledge (or hull) */
    float    center[3];              /* bounding sphere center (object coords) */
    float    radius;                 /* bounding sphere radius */
    uint8_t  box_sizes[3];           /* AABB half-extents (encoded) */
    uint8_t  padding;                /* alignment padding */
} ivp_compact_ledgetree_node_t;

/* Size assertion */
_Static_assert(sizeof(ivp_compact_ledgetree_node_t) == 28,
               "ivp_compact_ledgetree_node_t must be 28 bytes");

/* ── Terminal check ─────────────────────────────────────────────────────── */

static inline bool ivp_ledgetree_is_terminal(
    const ivp_compact_ledgetree_node_t *n)
{
    return n->offset_right_node == 0;
}

/* ── Navigation ─────────────────────────────────────────────────────────── */

/* Get the compact ledge for a terminal node. */
static inline const ivp_compact_ledge_t *ivp_ledgetree_get_compact_ledge(
    const ivp_compact_ledgetree_node_t *n)
{
    IVP_ASSERT(ivp_ledgetree_is_terminal(n));
    return (const ivp_compact_ledge_t *)((const char *)n +
           n->offset_compact_ledge);
}

/* Get left child (always the next node in memory). */
static inline const ivp_compact_ledgetree_node_t *ivp_ledgetree_left_son(
    const ivp_compact_ledgetree_node_t *n)
{
    IVP_ASSERT(!ivp_ledgetree_is_terminal(n));
    return n + 1;
}

/* Get right child (at byte offset from this node). */
static inline const ivp_compact_ledgetree_node_t *ivp_ledgetree_right_son(
    const ivp_compact_ledgetree_node_t *n)
{
    IVP_ASSERT(!ivp_ledgetree_is_terminal(n));
    return (const ivp_compact_ledgetree_node_t *)((const char *)n +
           n->offset_right_node);
}

/* Get the convex hull ledge for a non-terminal node (may be NULL). */
static inline const ivp_compact_ledge_t *ivp_ledgetree_get_compact_hull(
    const ivp_compact_ledgetree_node_t *n)
{
    if (n->offset_compact_ledge != 0) {
        return (const ivp_compact_ledge_t *)((const char *)n +
               n->offset_compact_ledge);
    }
    return NULL;
}

/* ── Bounding box decoding ──────────────────────────────────────────────── */

/* Decode encoded box size to actual half-extent for axis i. */
static inline float ivp_ledgetree_decode_box_size(
    const ivp_compact_ledgetree_node_t *n, int axis)
{
    return (float)n->box_sizes[axis] * n->radius *
           IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;
}

/* ── Traversal (non-inline, in ivp_ledgetree.c) ─────────────────────────── */

/* Callback for ledgetree traversal. Return true to continue, false to stop. */
typedef bool (*ivp_ledgetree_visitor_fn)(const ivp_compact_ledge_t *ledge,
                                          void *user_data);

/* Visit all terminal ledges in the tree (depth-first, left-to-right). */
void ivp_ledgetree_visit_all(const ivp_compact_ledgetree_node_t *root,
                              ivp_ledgetree_visitor_fn visitor,
                              void *user_data);

/* Count all terminal ledges in the tree. */
int ivp_ledgetree_count_terminal_ledges(
    const ivp_compact_ledgetree_node_t *root);

#ifdef __cplusplus
}
#endif

#endif /* IVP_LEDGETREE_H */
