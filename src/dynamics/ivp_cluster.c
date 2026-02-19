/* ivp_cluster.c — Cluster implementation for IVP C17
 *
 * Ported from IVP_Cluster (ivp_object.hxx / ivp_object.cxx).
 */
#include <ivp/dynamics/ivp_cluster.h>
#include <ivp/dynamics/ivp_environment.h>

#include <string.h>

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

ivp_cluster_t *ivp_cluster_create(ivp_environment_t *env,
                                  ivp_cluster_t *parent,
                                  const ivp_cluster_template_t *tmpl)
{
    const ivp_allocator_t *alloc = env->allocator;
    ivp_cluster_t *c = (ivp_cluster_t *)ivp_alloc(alloc, sizeof(ivp_cluster_t));
    if (!c) return NULL;

    memset(c, 0, sizeof(*c));

    /* Initialize the base object header */
    c->base.type = IVP_OBJECT_CLUSTER;
    c->base.owner = env;
    c->base.layer_mask = 0xFFFFFFFF;
    c->base.next_in_cluster = NULL;
    c->base.parent_cluster = parent;

    if (tmpl) {
        c->base.name = tmpl->name;
        c->base.client_data = tmpl->client_data;
    }

    c->first_child = NULL;
    c->child_count = 0;
    c->parent = parent;

    /* If there is a parent, add this cluster as a child of the parent */
    if (parent) {
        ivp_cluster_add_child(parent, &c->base);
    }

    return c;
}

void ivp_cluster_destroy(ivp_cluster_t *cluster)
{
    if (!cluster) return;

    /* Detach all children (do NOT destroy them) */
    ivp_object_t *child = cluster->first_child;
    while (child) {
        ivp_object_t *next = child->next_in_cluster;
        child->next_in_cluster = NULL;
        child->parent_cluster = NULL;
        child = next;
    }
    cluster->first_child = NULL;
    cluster->child_count = 0;

    /* Remove self from parent */
    if (cluster->parent) {
        ivp_cluster_remove_child(cluster->parent, &cluster->base);
    }

    /* Free */
    ivp_environment_t *env = cluster->base.owner;
    if (env && env->allocator) {
        ivp_free(env->allocator, cluster, sizeof(ivp_cluster_t));
    }
}

/* ── Child management ───────────────────────────────────────────────────── */

void ivp_cluster_add_child(ivp_cluster_t *cluster, ivp_object_t *child)
{
    if (!cluster || !child) return;

    /* Detach from previous parent if any */
    if (child->parent_cluster) {
        ivp_cluster_remove_child(child->parent_cluster, child);
    }

    child->parent_cluster = cluster;
    child->next_in_cluster = cluster->first_child;
    cluster->first_child = child;
    cluster->child_count++;
}

bool ivp_cluster_remove_child(ivp_cluster_t *cluster, ivp_object_t *child)
{
    if (!cluster || !child) return false;

    ivp_object_t **pp = &cluster->first_child;
    while (*pp) {
        if (*pp == child) {
            *pp = child->next_in_cluster;
            child->next_in_cluster = NULL;
            child->parent_cluster = NULL;
            cluster->child_count--;
            return true;
        }
        pp = &(*pp)->next_in_cluster;
    }
    return false;
}
