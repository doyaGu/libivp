/* ivp_cluster.h — Cluster (object container) for IVP C17
 *
 * A cluster is a container that groups IVP objects into a hierarchy.
 * The cluster itself is an IVP_Object with type IVP_OBJECT_CLUSTER.
 * Physics ignores the hierarchy except for the universe manager, which
 * visits the real objects in cluster tree order
 * (IVP_Cluster_Manager::get_next_real_object_in_cluster_tree): every
 * environment has a root cluster holding its static object and every real
 * object created in it (each new child is inserted at the head of the
 * list, like IVP_Cluster::add_object).
 *
 * Ported from IVP_Cluster (ivp_object.hxx).
 */
#ifndef IVP_CLUSTER_H
#define IVP_CLUSTER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/dynamics/ivp_object.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Cluster template ───────────────────────────────────────────────────── */

typedef struct ivp_cluster_template {
    const char *name;           /* optional debug name (borrowed)    */
    void       *client_data;    /* user data pointer                 */
} ivp_cluster_template_t;

static inline void ivp_cluster_template_init(ivp_cluster_template_t *tmpl) {
    tmpl->name = NULL;
    tmpl->client_data = NULL;
}

/* ── Cluster ────────────────────────────────────────────────────────────── */

typedef struct ivp_cluster {
    /* Common object header (must be first member for tagged-struct pattern) */
    ivp_object_t base;

    /* Linked list of child objects */
    ivp_object_t *first_child;
    int           child_count;

    /* Parent cluster (NULL for root) */
    struct ivp_cluster *parent;
} ivp_cluster_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

/* Initialise an empty cluster in place (no parent).  tmpl may be NULL. */
void ivp_cluster_init(ivp_cluster_t *cluster, struct ivp_environment *env,
                      const ivp_cluster_template_t *tmpl);

/* Create a cluster. parent may be NULL (a cluster outside any tree; use
 * ivp_environment_create_cluster to create one below the environment's
 * root cluster).  tmpl may be NULL for defaults. env is borrowed. */
ivp_cluster_t *ivp_cluster_create(struct ivp_environment *env,
                                  ivp_cluster_t *parent,
                                  const ivp_cluster_template_t *tmpl);

/* Destroy a cluster (children are NOT destroyed): its children move to the
 * cluster's parent (keeping their order), or are detached when it has none. */
void ivp_cluster_destroy(ivp_cluster_t *cluster);

/* ── Child management ───────────────────────────────────────────────────── */

/* Add a child object at the head of the cluster's list, removing it from
 * its previous cluster first (IVP_Object::assign_to_cluster). */
void ivp_cluster_add_child(ivp_cluster_t *cluster, ivp_object_t *child);

/* Remove a child object from the cluster. Returns true if found.  The child
 * must be in the cluster (IVP_Cluster::remove_object asserts it in debug
 * builds); release builds return false otherwise. */
bool ivp_cluster_remove_child(ivp_cluster_t *cluster, ivp_object_t *child);

/* ── Iteration ──────────────────────────────────────────────────────────── */

static inline ivp_object_t *ivp_cluster_get_first_child(const ivp_cluster_t *c) {
    return c->first_child;
}

/* Get next sibling in the cluster's child list.
 * Uses the next_in_cluster link on ivp_object_t. */
static inline ivp_object_t *ivp_cluster_get_next_child(const ivp_object_t *obj) {
    return obj->next_in_cluster;
}

static inline int ivp_cluster_get_child_count(const ivp_cluster_t *c) {
    return c->child_count;
}

/* IVP_Cluster_Manager::get_next_real_object_in_cluster_tree: the real
 * (non-cluster) object following `object` in the depth-first walk of the
 * tree below `root` (object NULL: the first one), NULL after the last one. */
ivp_object_t *ivp_cluster_get_next_real_object_in_tree(const ivp_cluster_t *root,
                                                       const ivp_object_t *object);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CLUSTER_H */
