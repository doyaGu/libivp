/* ivp_collision_restore.h -- internal helpers for world snapshots
 * (src/dynamics/ivp_snapshot.c): identify collision objects and rebuild
 * them in a new world without running the creation logic. */
#ifndef IVP_COLLISION_RESTORE_H
#define IVP_COLLISION_RESTORE_H

#include <ivp/collision/ivp_collision_core.h>
#include <ivp/collision/ivp_hull_mgr.h>

struct ivp_object;

/* collision created by the built-in mindist delegator (not an OO watcher) */
bool ivp_collision_is_mindist(const ivp_collision_t *c);

bool ivp_listener_hull_is_md_synapse(const ivp_listener_hull_t *lh);
bool ivp_listener_hull_is_ov_element(const ivp_listener_hull_t *lh);

/* IVP_Synapse::init_synapse_real with the given edge and status, including
 * the ledge reference of a polygon synapse (IVP_Mindist::init_mindist).
 * The synapse is left outside every list. */
void ivp_md_synapse_restore(ivp_mindist_t *md, int side, struct ivp_object *obj,
                            const ivp_compact_edge_t *edge, short status);

/* Free a mindist from ivp_mindist_create that was never initialized,
 * listed or registered anywhere. */
void ivp_mindist_discard_unpublished(ivp_mindist_t *md);

/* Insert element into the node with the given raster data, creating the
 * node like ivp_ov_tree_insert_ov_element_ex would.  The caller sets the
 * element's center and radius; no collision partners are collected. */
ivp_result_t ivp_ov_tree_insert_ov_element_at(ivp_ov_tree_t *t,
                                              ivp_ov_element_t *element,
                                              const ivp_ov_node_data_t *data);

#endif /* IVP_COLLISION_RESTORE_H */
