/* ivp_cluster.c — Cluster implementation for IVP C17
 *
 * Ported from IVP_Cluster (ivp_object.hxx / ivp_object.cxx) and
 * IVP_Cluster_Manager::get_next_real_object_in_cluster_tree
 * (ivp_intern/ivp_physic_private.cxx).
 */
#include <ivp/dynamics/ivp_cluster.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/base/ivp_assert.h>

#include <stdint.h>
#include <string.h>

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

void ivp_cluster_init(ivp_cluster_t *c, ivp_environment_t *env, const ivp_cluster_template_t *tmpl)
{
    memset(c, 0, sizeof(*c));
    /* IVP_Object::init + IVP_Cluster ctor: objects = NULL, type IVP_CLUSTER */
    c->base.type = IVP_OBJECT_CLUSTER;
    c->base.owner = env;
    c->base.layer_mask = 0xFFFFFFFF;
    if (tmpl) {
        c->base.name = tmpl->name;
        c->base.client_data = tmpl->client_data;
    }
}

ivp_cluster_t *ivp_cluster_create(ivp_environment_t *env,
                                  ivp_cluster_t *parent,
                                  const ivp_cluster_template_t *tmpl)
{
    const ivp_allocator_t *alloc = env->allocator;
    ivp_cluster_t *c = (ivp_cluster_t *)ivp_alloc(alloc, sizeof(ivp_cluster_t));
    if (!c) return NULL;
    ivp_cluster_init(c, env, tmpl);

    /* IVP_Object(father, templ): assign_to_cluster(father) */
    if (parent) {
        ivp_cluster_add_child(parent, &c->base);
    }

    return c;
}

void ivp_cluster_destroy(ivp_cluster_t *cluster)
{
    if (!cluster) return;

    ivp_cluster_t *parent = cluster->base.parent_cluster;
    if (parent) {
        /* move the children to the parent cluster, keeping their order (the
         * reference deletes them with the cluster; libivp keeps them in the
         * tree the universe manager walks) */
        ivp_object_t *last = cluster->first_child;
        while (last && last->next_in_cluster) last = last->next_in_cluster;
        while (last) {
            ivp_object_t *prev = last->prev_in_cluster;
            ivp_cluster_remove_child(cluster, last);
            ivp_cluster_add_child(parent, last);
            last = prev;
        }
        ivp_cluster_remove_child(parent, &cluster->base);
    } else {
        /* Detach all children (do NOT destroy them) */
        while (cluster->first_child) ivp_cluster_remove_child(cluster, cluster->first_child);
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
    IVP_ASSERT(child); /* IVP_Cluster::add_object: object */
    if (!cluster || !child) return;
    /* IVP_Object::assign_to_cluster */
    if (child->parent_cluster == cluster) return;
    IVP_ASSERT(cluster->base.owner == child->owner); /* cluster->environment == environment */
    if (child->parent_cluster) {
        ivp_cluster_remove_child(child->parent_cluster, child);
    }

    /* IVP_Cluster::add_object: insert at the head of the list */
    IVP_ASSERT(child->parent_cluster == NULL);
    child->next_in_cluster = cluster->first_child;
    child->prev_in_cluster = NULL;
    if (cluster->first_child) cluster->first_child->prev_in_cluster = child;
    cluster->first_child = child;
    child->parent_cluster = cluster;
    if (child->type == IVP_OBJECT_CLUSTER) ((ivp_cluster_t *)child)->parent = cluster;
    cluster->child_count++;
}

bool ivp_cluster_remove_child(ivp_cluster_t *cluster, ivp_object_t *child)
{
    /* IVP_Cluster::remove_object */
    IVP_ASSERT(child);
    if (!cluster || !child) return false;
    IVP_ASSERT(child->parent_cluster == cluster);
    if (child->parent_cluster != cluster) return false;
    if (!child->prev_in_cluster) {
        cluster->first_child = child->next_in_cluster;
    } else {
        child->prev_in_cluster->next_in_cluster = child->next_in_cluster;
    }
    if (child->next_in_cluster) {
        child->next_in_cluster->prev_in_cluster = child->prev_in_cluster;
    }
    child->parent_cluster = NULL;
    child->next_in_cluster = NULL;
    child->prev_in_cluster = NULL;
    if (child->type == IVP_OBJECT_CLUSTER) ((ivp_cluster_t *)child)->parent = NULL;
    cluster->child_count--;
    return true;
}

/* ── Iteration ──────────────────────────────────────────────────────────── */

ivp_object_t *ivp_cluster_get_next_real_object_in_tree(const ivp_cluster_t *root, const ivp_object_t *object_in)
{
    /* IVP_Cluster_Manager::get_next_real_object_in_cluster_tree, including
     * its walk-up quirks (an empty cluster restarts the upward search from
     * the root's first object) */
    ivp_object_t *object = (ivp_object_t *)(uintptr_t)object_in;
    ivp_object_t *next_object;
    if (!root) return NULL;
    if (object == NULL) {
        object = root->first_child;
        next_object = object; /* no object supplied: the first object in root */
        if (!object) return NULL;
    } else {
        next_object = object->next_in_cluster;
    }

    /* try to go down as deep as possible */
    if (object->type == IVP_OBJECT_CLUSTER) {
        const ivp_cluster_t *cluster = (const ivp_cluster_t *)object;
        for (;;) {
            object = cluster->first_child;
            if (object) {
                if (object->type != IVP_OBJECT_CLUSTER) return object;
                cluster = (const ivp_cluster_t *)object;
                continue;
            }
            break; /* just get next_in_cluster */
        }
    }

    /* go up until a next pointer is found */
    ivp_object_t *next;
    for (next = next_object; !next; next = object->next_in_cluster) {
        ivp_object_t *old_father = object;
        object = object ? (ivp_object_t *)object->parent_cluster : NULL;
        if (!object) {
            if ((const ivp_object_t *)&root->base == old_father) return NULL; /* end of the tree */
            object = root->first_child;
            if (!object) return NULL;
        }
    }

    /* now a next is found */
    object = next;
    if (object->type == IVP_OBJECT_CLUSTER) return ivp_cluster_get_next_real_object_in_tree(root, object);
    return object;
}
