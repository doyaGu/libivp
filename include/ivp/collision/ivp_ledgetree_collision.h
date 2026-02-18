/* ivp_ledgetree_collision.h — Hierarchical poly-poly collision via ledgetree BVH
 *
 * Provides bounding-volume overlap tests, pair traversal of two ledgetrees,
 * and a top-level hierarchical contact computation function for compound
 * (multi-convex) objects.
 */
#ifndef IVP_LEDGETREE_COLLISION_H
#define IVP_LEDGETREE_COLLISION_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/solver/ivp_contact.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ──────────────────────────────────────────────────────────── */

#define IVP_LEDGETREE_MAX_DEPTH       32
#define IVP_LEDGETREE_MAX_STACK       (IVP_LEDGETREE_MAX_DEPTH * 2)
#define IVP_LEDGETREE_MAX_LEAF_PAIRS  64

/* ── Leaf pair callback ─────────────────────────────────────────────────── */

/* Called for each overlapping terminal pair found during traversal.
 * ledge_a and ledge_b are the compact ledges at the terminal nodes.
 * b_in_a is the transform from B's object space to A's object space.
 * Return true to continue traversal, false to stop early. */
typedef bool (*ivp_leaf_pair_fn)(const ivp_compact_ledge_t *ledge_a,
                                  const ivp_compact_ledge_t *ledge_b,
                                  const ivp_transform_r_t *b_in_a,
                                  void *user_data);

/* ── BV overlap test ────────────────────────────────────────────────────── */

/* Test bounding sphere overlap between two ledgetree nodes in different
 * object spaces. b_in_a transforms B-space centers into A-space.
 * Returns true if the spheres overlap (within margin). */
bool ivp_ledgetree_bv_overlap(const ivp_compact_ledgetree_node_t *node_a,
                               const ivp_compact_ledgetree_node_t *node_b,
                               const ivp_transform_r_t *b_in_a,
                               ivp_real_t margin);

/* ── Pair traversal ─────────────────────────────────────────────────────── */

/* Traverse two ledgetrees and invoke callback for each overlapping terminal
 * pair. Uses explicit stack with "descend the larger" strategy.
 * Returns the number of leaf pairs found. */
int ivp_ledgetree_traverse_pairs(const ivp_compact_ledgetree_node_t *root_a,
                                  const ivp_compact_ledgetree_node_t *root_b,
                                  const ivp_transform_r_t *b_in_a,
                                  ivp_real_t margin,
                                  ivp_leaf_pair_fn callback,
                                  void *user_data);

/* ── Hierarchical multi-contact computation ─────────────────────────────── */

/* Compute contacts between two polygon objects using hierarchical BVH
 * traversal. Handles compound (multi-convex) shapes by traversing both
 * ledgetrees and running narrowphase per leaf pair.
 *
 * For simple (single-convex) objects, creates a synthetic terminal node
 * and runs a single leaf pair.
 *
 * contacts_out must have space for max_contacts elements.
 * Returns the number of contacts written (0..max_contacts). */
int ivp_contact_compute_hierarchical_multi(ivp_object_t *obj_a,
                                            ivp_object_t *obj_b,
                                            ivp_contact_point_t *contacts_out,
                                            int max_contacts);

#ifdef __cplusplus
}
#endif

#endif /* IVP_LEDGETREE_COLLISION_H */
