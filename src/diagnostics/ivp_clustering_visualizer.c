/* ivp_clustering_visualizer.c — port of ivp_physics/ivp_clustering_visualizer.cxx
 * and ivp_physics/ivp_clustering_visual_hash.cxx
 */
#include <ivp/diagnostics/ivp_clustering_visualizer.h>

#include <ivp/dynamics/ivp_object.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/collision/ivp_collision_core.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ── IVP_Clustering_Visualizer_Object_Hash ─────────────────────────────── */

static bool cv_hash_compare(const void *elem0, const void *elem1)
{
    return elem0 == elem1;
}

/* The reference hashes sizeof(pointer) bytes AT the object (its vtable
 * pointer); libivp objects start with mutable state, so the pointer value
 * itself is hashed.  Membership semantics are the same. */
static uint32_t cv_hash_obj_to_index(const struct ivp_object *obj)
{
    return ivp_vhash_index(&obj, (int)sizeof(obj));
}

void ivp_clustering_visualizer_object_hash_init(ivp_clustering_visualizer_object_hash_t *h, int create_size)
{
    ivp_clustering_visualizer_object_hash_init_with_allocator(h, create_size, NULL);
}

void ivp_clustering_visualizer_object_hash_init_with_allocator(
    ivp_clustering_visualizer_object_hash_t *h, int create_size,
    const ivp_allocator_t *allocator)
{
    if (!h) return;
    int size = 1;
    while (size < create_size && size < (1 << 30)) size <<= 1;
    ivp_vhash_init(&h->hash, allocator, size, cv_hash_compare);
}

void ivp_clustering_visualizer_object_hash_destroy(ivp_clustering_visualizer_object_hash_t *h)
{
    if (!h) return;
    ivp_vhash_destroy(&h->hash);
}

void ivp_clustering_visualizer_object_hash_add(ivp_clustering_visualizer_object_hash_t *h, struct ivp_object *obj)
{
    if (!h || !obj) return;
    ivp_vhash_add(&h->hash, obj, cv_hash_obj_to_index(obj));
}

struct ivp_object *ivp_clustering_visualizer_object_hash_remove(ivp_clustering_visualizer_object_hash_t *h,
                                                                struct ivp_object *obj)
{
    if (!h || !obj) return NULL;
    return (struct ivp_object *)ivp_vhash_remove(&h->hash, obj, cv_hash_obj_to_index(obj));
}

struct ivp_object *ivp_clustering_visualizer_object_hash_find(const ivp_clustering_visualizer_object_hash_t *h,
                                                              struct ivp_object *obj)
{
    if (!h || !obj) return NULL;
    return (struct ivp_object *)ivp_vhash_find(&h->hash, obj, cv_hash_obj_to_index(obj));
}

/* add / remove with the reference "already stored?" checks */
static void cv_filter_add(ivp_clustering_visualizer_object_hash_t *accept,
                          ivp_clustering_visualizer_object_hash_t *reject, struct ivp_object *obj,
                          ivp_clustering_visualizer_filter_t filter)
{
    switch (filter) {
    case IVP_CV_FILTER_ACCEPT:
        if (!ivp_clustering_visualizer_object_hash_find(accept, obj)) {
            ivp_clustering_visualizer_object_hash_add(accept, obj);
        }
        break;
    case IVP_CV_FILTER_REJECT:
        if (!ivp_clustering_visualizer_object_hash_find(reject, obj)) {
            ivp_clustering_visualizer_object_hash_add(reject, obj);
        }
        break;
    }
}

static void cv_filter_remove(ivp_clustering_visualizer_object_hash_t *accept,
                             ivp_clustering_visualizer_object_hash_t *reject, struct ivp_object *obj,
                             ivp_clustering_visualizer_filter_t filter)
{
    switch (filter) {
    case IVP_CV_FILTER_ACCEPT:
        if (ivp_clustering_visualizer_object_hash_find(accept, obj)) {
            (void)ivp_clustering_visualizer_object_hash_remove(accept, obj);
        }
        break;
    case IVP_CV_FILTER_REJECT:
        if (ivp_clustering_visualizer_object_hash_find(reject, obj)) {
            (void)ivp_clustering_visualizer_object_hash_remove(reject, obj);
        }
        break;
    }
}

/* ── Shortrange: internal classes ──────────────────────────────────────── */

/* IVP_Clustering_Visualizer_Shortrange_Nodedata */
typedef struct ivp_cv_shortrange_nodedata {
    const ivp_compact_ledgetree_node_t *node;
    int  n_touches;
    bool reported_ledge;
} ivp_cv_shortrange_nodedata_t;

/* IVP_Clustering_Visualizer_Shortrange_Objectdata */
typedef struct ivp_cv_shortrange_objectdata {
    struct ivp_object *real_object;
    ivp_pvec_t         ledgetree_nodes; /* ivp_cv_shortrange_nodedata_t* */
    ivp_vec3r_t        center;
    ivp_real_t         radius;
} ivp_cv_shortrange_objectdata_t;

static void cv_objectdata_free(ivp_cv_shortrange_objectdata_t *od)
{
    if (!od) return;
    const ivp_allocator_t *allocator = ivp_pvec_allocator(&od->ledgetree_nodes);
    for (int i = 0; i < ivp_pvec_len(&od->ledgetree_nodes); i++) {
        ivp_free(allocator, ivp_pvec_at(&od->ledgetree_nodes, i),
                 sizeof(ivp_cv_shortrange_nodedata_t));
    }
    ivp_pvec_clear(&od->ledgetree_nodes);
    ivp_free(allocator, od, sizeof(*od));
}

/* ── Shortrange: callback ──────────────────────────────────────────────── */

void ivp_clustering_visualizer_shortrange_callback_init(
    ivp_clustering_visualizer_shortrange_callback_t *cb,
    const ivp_clustering_visualizer_shortrange_callback_vtable_t *vt)
{
    if (!cb) return;
    cb->vt = vt;
    cb->private_property = NULL;
    cb->node = NULL;
    ivp_vec3r_set(&cb->position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    cb->sphere_radius = IVP_REAL(-1.0);
    ivp_vec3r_set(&cb->box_extents, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    cb->n_touches = -1;
    cb->reported_ledge = false;
    cb->user_data = NULL;
}

/* ── Shortrange: private methods ───────────────────────────────────────── */

static void cv_add_node_to_pipeline(ivp_clustering_visualizer_shortrange_t *v,
                                    const ivp_compact_ledgetree_node_t *node, bool reported_ledge)
{
    ivp_cv_shortrange_objectdata_t *od = v->pipelined_private_property_data;
    for (int x = 0; x < ivp_pvec_len(&od->ledgetree_nodes); x++) {
        ivp_cv_shortrange_nodedata_t *nodedata = (ivp_cv_shortrange_nodedata_t *)ivp_pvec_at(&od->ledgetree_nodes, x);
        if (nodedata->node == node) {
            nodedata->reported_ledge = nodedata->reported_ledge || reported_ledge;
            if (ivp_ledgetree_is_terminal(node)) {
                nodedata->n_touches++; /* no reference counting for non-terminal nodes! */
            }
            return;
        }
    }

    ivp_cv_shortrange_nodedata_t *new_nodedata =
        (ivp_cv_shortrange_nodedata_t *)ivp_alloc(&v->allocator, sizeof(*new_nodedata));
    if (!new_nodedata) return;
    memset(new_nodedata, 0, sizeof(*new_nodedata));
    new_nodedata->node = node;
    new_nodedata->n_touches = ivp_ledgetree_is_terminal(node) ? 1 : 0; /* no reference counting for non-terminal nodes! */
    new_nodedata->reported_ledge = reported_ledge;
    if (ivp_pvec_add(&od->ledgetree_nodes, new_nodedata) < 0) {
        ivp_free(&v->allocator, new_nodedata, sizeof(*new_nodedata));
    }
}

/* returns true whenever the node has been aborted (this includes terminal
 * nodes), false whenever the node is an inner node of the traversal tree */
static bool cv_recursively_traverse_cluster(ivp_clustering_visualizer_shortrange_t *v,
                                            const ivp_compact_ledgetree_node_t *node)
{
    ivp_vec3r_t center;
    ivp_vec3r_set(&center, (ivp_real_t)node->center[0], (ivp_real_t)node->center[1], (ivp_real_t)node->center[2]);
    ivp_vec3r_t distance_between_centers;
    ivp_vec3r_sub(&distance_between_centers, &center, &v->intruder_position_in_private_property_cs);

    {   /* first check sphere against sphere */
        v->stats_n_sphere_collisiontests++;
        ivp_real_t sum_of_radii = v->intruder_radius + (ivp_real_t)node->radius;
        ivp_real_t quad_length = distance_between_centers.k[0] * distance_between_centers.k[0] +
                                 distance_between_centers.k[1] * distance_between_centers.k[1] +
                                 distance_between_centers.k[2] * distance_between_centers.k[2];
        if (quad_length >= sum_of_radii * sum_of_radii) goto abort;
    }

    /* --> sphere collision detected! */

    {   /* now check bounding box against bounding box */
        v->stats_n_box_collisiontests++;
        float work = (float)(IVP_COMPACT_BOUNDINGBOX_STEP_SIZE * (double)node->radius);
        if ((ivp_real_t)fabs((double)distance_between_centers.k[0]) >=
            (ivp_real_t)((float)node->box_sizes[0] * work) + v->intruder_radius) goto abort;
        if ((ivp_real_t)fabs((double)distance_between_centers.k[1]) >=
            (ivp_real_t)((float)node->box_sizes[1] * work) + v->intruder_radius) goto abort;
        if ((ivp_real_t)fabs((double)distance_between_centers.k[2]) >=
            (ivp_real_t)((float)node->box_sizes[2] * work) + v->intruder_radius) goto abort;
    }

    /* --> bounding box collision detected! */

    if (ivp_ledgetree_is_terminal(node)) {
        cv_add_node_to_pipeline(v, node, true);
        return true;
    } else if (v->boxmode == IVP_CV_TRAVERSED_BOXES) {
        cv_add_node_to_pipeline(v, node, false);
    }

    (void)cv_recursively_traverse_cluster(v, ivp_ledgetree_left_son(node));
    (void)cv_recursively_traverse_cluster(v, ivp_ledgetree_right_son(node));
    return false;

abort:
    if (v->boxmode != IVP_CV_TERMINALS_ONLY) { /* display for ABORTED_BOXES and TRAVERSED_BOXES */
        cv_add_node_to_pipeline(v, node, false);
    }
    return true;
}

/* The ledgetree root of a polygon (compact surface) surface manager, NULL
 * for other surface managers (balls, grids). */
static const ivp_compact_ledgetree_node_t *cv_polygon_ledgetree_root(const ivp_surman_t *sm)
{
    if (!sm) return NULL;
    ivp_surman_polygon_t probe;
    ivp_surman_polygon_init(&probe, NULL);
    if (sm->vtable != probe.base.vtable) return NULL;
    const ivp_surman_polygon_t *poly = (const ivp_surman_polygon_t *)sm;
    if (!poly->compact_surface) return NULL;
    return ivp_surface_get_ledgetree_root(poly->compact_surface);
}

/* ── Shortrange: public methods ────────────────────────────────────────── */

void ivp_clustering_visualizer_shortrange_visualize_collisions(ivp_clustering_visualizer_shortrange_t *v)
{
    if (!v || !v->enabled) return;

    v->stats_n_reported_ledges = 0;

    for (int i = 0; i < ivp_pvec_len(&v->output_callbacks); i++) {
        ivp_clustering_visualizer_shortrange_callback_t *callback =
            (ivp_clustering_visualizer_shortrange_callback_t *)ivp_pvec_at(&v->output_callbacks, i);

        for (int x = ivp_pvec_len(&v->pipelined_objects) - 1; x >= 0; x--) {
            ivp_cv_shortrange_objectdata_t *objectdata =
                (ivp_cv_shortrange_objectdata_t *)ivp_pvec_at(&v->pipelined_objects, x);

            for (int y = ivp_pvec_len(&objectdata->ledgetree_nodes) - 1; y >= 0; y--) {
                ivp_cv_shortrange_nodedata_t *nodedata =
                    (ivp_cv_shortrange_nodedata_t *)ivp_pvec_at(&objectdata->ledgetree_nodes, y);
                const ivp_compact_ledgetree_node_t *node = nodedata->node;

                callback->private_property = objectdata->real_object;
                callback->node = node;
                callback->n_touches = nodedata->n_touches;

                ivp_vec3r_set(&callback->position, (ivp_real_t)node->center[0], (ivp_real_t)node->center[1],
                              (ivp_real_t)node->center[2]);
                callback->sphere_radius = (ivp_real_t)node->radius;
                callback->reported_ledge = nodedata->reported_ledge;

                if (callback->reported_ledge) v->stats_n_reported_ledges++;

                ivp_real_t work = (ivp_real_t)(IVP_COMPACT_BOUNDINGBOX_STEP_SIZE * (double)node->radius);
                callback->box_extents.k[0] = (ivp_real_t)node->box_sizes[0] * work;
                callback->box_extents.k[1] = (ivp_real_t)node->box_sizes[1] * work;
                callback->box_extents.k[2] = (ivp_real_t)node->box_sizes[2] * work;

                if (callback->vt && callback->vt->visualize_request_for_node) {
                    callback->vt->visualize_request_for_node(callback);
                }

                ivp_pvec_remove(&objectdata->ledgetree_nodes, nodedata);
                ivp_free(&v->allocator, nodedata, sizeof(*nodedata));
            }

            if (objectdata->radius > IVP_REAL(0.0)) {
                callback->position = objectdata->center;
                callback->box_extents.k[0] = objectdata->radius;
                if (callback->vt && callback->vt->visualize_request_for_intruder_radius) {
                    callback->vt->visualize_request_for_intruder_radius(callback);
                }
            }

            ivp_pvec_remove(&v->pipelined_objects, objectdata);
            cv_objectdata_free(objectdata);
        }
    }

    /* reset values for next frame */
    v->stats_n_sphere_collisiontests = 0;
    v->stats_n_box_collisiontests = 0;
}

void ivp_clustering_visualizer_shortrange_analyze_collision(ivp_clustering_visualizer_shortrange_t *v,
                                                            struct ivp_object *object0, ivp_real_t radius0,
                                                            struct ivp_object *object1, ivp_real_t radius1)
{
    if (!v || !v->enabled) return;
    if (!object0 || !object1) return;

    for (int x = 0; x < 2; x++) {
        if (x == 0) {
            v->private_property = object1;
            v->intruder = object0;
            v->intruder_radius = radius0;
        } else {
            v->private_property = object0;
            v->intruder = object1;
            v->intruder_radius = radius1;
        }

        /* filter out any unwanted objects */
        if (v->use_intruder_accept_filter &&
            !ivp_clustering_visualizer_object_hash_find(&v->accept_filter_intruder, v->intruder)) continue;
        if (v->use_intruder_reject_filter &&
            ivp_clustering_visualizer_object_hash_find(&v->reject_filter_intruder, v->intruder)) continue;
        if (v->use_private_property_accept_filter &&
            !ivp_clustering_visualizer_object_hash_find(&v->accept_filter_private_property, v->private_property)) continue;
        if (v->use_private_property_reject_filter &&
            ivp_clustering_visualizer_object_hash_find(&v->reject_filter_private_property, v->private_property)) continue;

        const ivp_compact_ledgetree_node_t *private_property_ledgetree_rootnode =
            cv_polygon_ledgetree_root(v->private_property->surman);
        if (!private_property_ledgetree_rootnode) continue;

        ivp_object_get_m_world_f_object_AT(v->intruder, &v->intruder_matrix);

        ivp_cache_obj_t *object_cache = ivp_object_get_cache_object(v->private_property);
        /* IVP_Cache_Object::transform_position_to_object_coords */
        ivp_transform_inv_point(&v->intruder_position_in_private_property_cs, &object_cache->m_world_f_object,
                                &v->intruder_matrix.pos);
        ivp_cache_obj_remove_reference(object_cache);

        v->pipelined_private_property_data = NULL;
        for (int k = 0; k < ivp_pvec_len(&v->pipelined_objects); k++) {
            ivp_cv_shortrange_objectdata_t *objectdata =
                (ivp_cv_shortrange_objectdata_t *)ivp_pvec_at(&v->pipelined_objects, k);
            if (objectdata->real_object == v->private_property) {
                v->pipelined_private_property_data = objectdata;
                break;
            }
        }
        if (!v->pipelined_private_property_data) {
            ivp_cv_shortrange_objectdata_t *od =
                (ivp_cv_shortrange_objectdata_t *)ivp_alloc(&v->allocator, sizeof(*od));
            if (!od) continue;
            memset(od, 0, sizeof(*od));
            od->real_object = v->private_property;
            ivp_pvec_init_with_allocator(&od->ledgetree_nodes, &v->allocator);
            od->center = v->intruder_matrix.pos;
            od->radius = v->intruder_radius;
            if (ivp_pvec_add(&v->pipelined_objects, od) < 0) {
                cv_objectdata_free(od);
                continue;
            }
            v->pipelined_private_property_data = od;
        }

        (void)cv_recursively_traverse_cluster(v, private_property_ledgetree_rootnode);
    }
}

void ivp_clustering_visualizer_shortrange_add_object_to_intruder_filter(ivp_clustering_visualizer_shortrange_t *v,
                                                                        struct ivp_object *obj,
                                                                        ivp_clustering_visualizer_filter_t filter)
{
    if (!v) return;
    cv_filter_add(&v->accept_filter_intruder, &v->reject_filter_intruder, obj, filter);
}

void ivp_clustering_visualizer_shortrange_remove_object_from_intruder_filter(
    ivp_clustering_visualizer_shortrange_t *v, struct ivp_object *obj, ivp_clustering_visualizer_filter_t filter)
{
    if (!v) return;
    cv_filter_remove(&v->accept_filter_intruder, &v->reject_filter_intruder, obj, filter);
}

void ivp_clustering_visualizer_shortrange_add_object_to_private_property_filter(
    ivp_clustering_visualizer_shortrange_t *v, struct ivp_object *obj, ivp_clustering_visualizer_filter_t filter)
{
    if (!v) return;
    cv_filter_add(&v->accept_filter_private_property, &v->reject_filter_private_property, obj, filter);
}

void ivp_clustering_visualizer_shortrange_remove_object_from_private_property_filter(
    ivp_clustering_visualizer_shortrange_t *v, struct ivp_object *obj, ivp_clustering_visualizer_filter_t filter)
{
    if (!v) return;
    cv_filter_remove(&v->accept_filter_private_property, &v->reject_filter_private_property, obj, filter);
}

void ivp_clustering_visualizer_shortrange_install_visualize_callback(
    ivp_clustering_visualizer_shortrange_t *v, ivp_clustering_visualizer_shortrange_callback_t *callback)
{
    if (!v || !callback) return;
    ivp_pvec_add(&v->output_callbacks, callback);
}

void ivp_clustering_visualizer_shortrange_remove_visualize_callback(
    ivp_clustering_visualizer_shortrange_t *v, ivp_clustering_visualizer_shortrange_callback_t *callback)
{
    if (!v) return;
    (void)ivp_pvec_remove_if_present(&v->output_callbacks, callback);
}

void ivp_clustering_visualizer_shortrange_enable(ivp_clustering_visualizer_shortrange_t *v)
{
    if (v) v->enabled = true;
}

void ivp_clustering_visualizer_shortrange_disable(ivp_clustering_visualizer_shortrange_t *v)
{
    if (v) v->enabled = false;
}

bool ivp_clustering_visualizer_shortrange_get_state(const ivp_clustering_visualizer_shortrange_t *v)
{
    return v ? v->enabled : false;
}

void ivp_clustering_visualizer_shortrange_init_with_allocator(
    ivp_clustering_visualizer_shortrange_t *v, const ivp_allocator_t *allocator)
{
    if (!v) return;
    memset(v, 0, sizeof(*v));
    v->allocator = *(allocator ? allocator : ivp_default_allocator());
    v->enabled = false; /* disabled by default! */

    ivp_clustering_visualizer_object_hash_init_with_allocator(
        &v->accept_filter_intruder, 64, &v->allocator);
    ivp_clustering_visualizer_object_hash_init_with_allocator(
        &v->reject_filter_intruder, 64, &v->allocator);
    ivp_clustering_visualizer_object_hash_init_with_allocator(
        &v->accept_filter_private_property, 64, &v->allocator);
    ivp_clustering_visualizer_object_hash_init_with_allocator(
        &v->reject_filter_private_property, 64, &v->allocator);
    ivp_pvec_init_with_allocator(&v->output_callbacks, &v->allocator);
    ivp_pvec_init_with_allocator(&v->pipelined_objects, &v->allocator);

    v->private_property = NULL;
    v->intruder = NULL;
    v->intruder_radius = IVP_REAL(0.0);
    ivp_transform_identity(&v->intruder_matrix);

    v->pipelined_private_property_data = NULL;

    v->use_intruder_accept_filter = false;
    v->use_intruder_reject_filter = false;
    v->use_private_property_accept_filter = false;
    v->use_private_property_reject_filter = false;
    v->remove_longrange_visualization = false;

    v->boxmode = IVP_CV_TRAVERSED_BOXES;

    v->stats_n_reported_ledges = 0;
    v->stats_n_sphere_collisiontests = 0;
    v->stats_n_box_collisiontests = 0;
}

void ivp_clustering_visualizer_shortrange_init(ivp_clustering_visualizer_shortrange_t *v)
{
    ivp_clustering_visualizer_shortrange_init_with_allocator(v, NULL);
}

void ivp_clustering_visualizer_shortrange_destroy(ivp_clustering_visualizer_shortrange_t *v)
{
    if (!v) return;
    v->enabled = false; /* just in case... */

    ivp_clustering_visualizer_object_hash_destroy(&v->accept_filter_intruder);
    ivp_clustering_visualizer_object_hash_destroy(&v->reject_filter_intruder);
    ivp_clustering_visualizer_object_hash_destroy(&v->accept_filter_private_property);
    ivp_clustering_visualizer_object_hash_destroy(&v->reject_filter_private_property);
    for (int i = 0; i < ivp_pvec_len(&v->pipelined_objects); i++) {
        cv_objectdata_free((ivp_cv_shortrange_objectdata_t *)ivp_pvec_at(&v->pipelined_objects, i));
    }
    ivp_pvec_clear(&v->pipelined_objects);
    ivp_pvec_clear(&v->output_callbacks);
    v->pipelined_private_property_data = NULL;
}

/* ── Longrange ─────────────────────────────────────────────────────────── */

/* IVP_Clustering_Visualizer_Longrange_Data */
typedef struct ivp_cv_longrange_data {
    struct ivp_object *real_object;
    ivp_vec3f_t        center;
    ivp_real_t         radius;
} ivp_cv_longrange_data_t;

void ivp_clustering_visualizer_longrange_visualize(ivp_clustering_visualizer_longrange_t *v)
{
    if (!v || !v->enabled) return;

    for (int x = ivp_pvec_len(&v->visualize_data) - 1; x >= 0; x--) {
        ivp_cv_longrange_data_t *data = (ivp_cv_longrange_data_t *)ivp_pvec_at(&v->visualize_data, x);
        struct ivp_object *real_object = data->real_object;

        const bool unmoveable = ivp_object_get_core(real_object)->physical_unmoveable;
        if (unmoveable && !v->visualize_unmoveable_objects) continue;
        if (!unmoveable && !v->visualize_moveable_objects) continue;

        if (v->use_accept_filter) {
            if (!ivp_clustering_visualizer_object_hash_find(&v->accept_filter, real_object)) continue;
        }
        if (v->use_reject_filter) {
            if (ivp_clustering_visualizer_object_hash_find(&v->reject_filter, real_object)) continue;
        }

        for (int i = 0; i < ivp_pvec_len(&v->graphics_callbacks); i++) {
            ivp_clustering_visualizer_longrange_callback_t *callback =
                (ivp_clustering_visualizer_longrange_callback_t *)ivp_pvec_at(&v->graphics_callbacks, i);
            callback->real_object = real_object;
            callback->center = data->center;
            callback->radius = data->radius;
            if (callback->vt && callback->vt->visualize_request) callback->vt->visualize_request(callback);
        }

        ivp_pvec_remove(&v->visualize_data, data);
        ivp_free(&v->allocator, data, sizeof(*data));
    }
}

static void cv_longrange_remove_objectdata(ivp_clustering_visualizer_longrange_t *v, struct ivp_object *real_object)
{
    for (int x = 0; x < ivp_pvec_len(&v->visualize_data); x++) {
        ivp_cv_longrange_data_t *data = (ivp_cv_longrange_data_t *)ivp_pvec_at(&v->visualize_data, x);
        if (data->real_object != real_object) continue;
        ivp_pvec_remove(&v->visualize_data, data);
        ivp_free(&v->allocator, data, sizeof(*data));
        return;
    }
}

void ivp_clustering_visualizer_longrange_remove_object(ivp_clustering_visualizer_longrange_t *v,
                                                       struct ivp_object *real_object)
{
    if (!v || !v->enabled) return;

    cv_longrange_remove_objectdata(v, real_object);

    for (int i = 0; i < ivp_pvec_len(&v->graphics_callbacks); i++) {
        ivp_clustering_visualizer_longrange_callback_t *callback =
            (ivp_clustering_visualizer_longrange_callback_t *)ivp_pvec_at(&v->graphics_callbacks, i);
        callback->real_object = real_object;
        if (callback->vt && callback->vt->devisualize_request) callback->vt->devisualize_request(callback);
    }
}

void ivp_clustering_visualizer_longrange_add_object(ivp_clustering_visualizer_longrange_t *v,
                                                    struct ivp_object *real_object, const ivp_vec3f_t *center,
                                                    ivp_real_t radius)
{
    if (!v || !real_object || !center) return;
    ivp_cv_longrange_data_t *data =
        (ivp_cv_longrange_data_t *)ivp_alloc(&v->allocator, sizeof(*data));
    if (!data) return;
    memset(data, 0, sizeof(*data));
    data->real_object = real_object;
    data->center = *center;
    data->radius = radius;
    if (ivp_pvec_add(&v->visualize_data, data) < 0) {
        ivp_free(&v->allocator, data, sizeof(*data));
    }
}

void ivp_clustering_visualizer_longrange_install_visualize_callback(
    ivp_clustering_visualizer_longrange_t *v, ivp_clustering_visualizer_longrange_callback_t *callback)
{
    if (!v || !callback) return;
    ivp_pvec_add(&v->graphics_callbacks, callback);
}

void ivp_clustering_visualizer_longrange_remove_visualize_callback(
    ivp_clustering_visualizer_longrange_t *v, ivp_clustering_visualizer_longrange_callback_t *callback)
{
    if (!v) return;
    (void)ivp_pvec_remove_if_present(&v->graphics_callbacks, callback);
}

void ivp_clustering_visualizer_longrange_add_object_to_filter(ivp_clustering_visualizer_longrange_t *v,
                                                              struct ivp_object *obj,
                                                              ivp_clustering_visualizer_filter_t filter)
{
    if (!v) return;
    cv_filter_add(&v->accept_filter, &v->reject_filter, obj, filter);
}

void ivp_clustering_visualizer_longrange_remove_object_from_filter(ivp_clustering_visualizer_longrange_t *v,
                                                                   struct ivp_object *obj,
                                                                   ivp_clustering_visualizer_filter_t filter)
{
    if (!v) return;
    cv_filter_remove(&v->accept_filter, &v->reject_filter, obj, filter);
}

void ivp_clustering_visualizer_longrange_enable(ivp_clustering_visualizer_longrange_t *v)
{
    if (!v) return;
    for (int i = 0; i < ivp_pvec_len(&v->graphics_callbacks); i++) {
        ivp_clustering_visualizer_longrange_callback_t *callback =
            (ivp_clustering_visualizer_longrange_callback_t *)ivp_pvec_at(&v->graphics_callbacks, i);
        if (callback->vt && callback->vt->enable) callback->vt->enable(callback);
        callback->enabled = true;
    }
    v->enabled = true;
}

void ivp_clustering_visualizer_longrange_disable(ivp_clustering_visualizer_longrange_t *v)
{
    if (!v) return;
    for (int i = 0; i < ivp_pvec_len(&v->graphics_callbacks); i++) {
        ivp_clustering_visualizer_longrange_callback_t *callback =
            (ivp_clustering_visualizer_longrange_callback_t *)ivp_pvec_at(&v->graphics_callbacks, i);
        if (callback->vt && callback->vt->disable) callback->vt->disable(callback);
        callback->enabled = false;
    }
    v->enabled = false;
}

bool ivp_clustering_visualizer_longrange_get_state(const ivp_clustering_visualizer_longrange_t *v)
{
    return v ? v->enabled : false;
}

void ivp_clustering_visualizer_longrange_init_with_allocator(
    ivp_clustering_visualizer_longrange_t *v, const ivp_allocator_t *allocator)
{
    if (!v) return;
    memset(v, 0, sizeof(*v));
    v->allocator = *(allocator ? allocator : ivp_default_allocator());
    v->enabled = false;
    v->use_accept_filter = true;
    v->use_reject_filter = true;
    v->visualize_moveable_objects = false;
    v->visualize_unmoveable_objects = false;
    ivp_pvec_init_with_allocator(&v->graphics_callbacks, &v->allocator);
    ivp_pvec_init_with_allocator(&v->visualize_data, &v->allocator);
    ivp_clustering_visualizer_object_hash_init_with_allocator(
        &v->accept_filter, 1024, &v->allocator);
    ivp_clustering_visualizer_object_hash_init_with_allocator(
        &v->reject_filter, 1024, &v->allocator);
}

void ivp_clustering_visualizer_longrange_init(ivp_clustering_visualizer_longrange_t *v)
{
    ivp_clustering_visualizer_longrange_init_with_allocator(v, NULL);
}

void ivp_clustering_visualizer_longrange_destroy(ivp_clustering_visualizer_longrange_t *v)
{
    if (!v) return;
    ivp_clustering_visualizer_object_hash_destroy(&v->accept_filter);
    ivp_clustering_visualizer_object_hash_destroy(&v->reject_filter);
    for (int i = 0; i < ivp_pvec_len(&v->visualize_data); i++) {
        ivp_free(&v->allocator, ivp_pvec_at(&v->visualize_data, i),
                 sizeof(ivp_cv_longrange_data_t));
    }
    ivp_pvec_clear(&v->visualize_data);
    ivp_pvec_clear(&v->graphics_callbacks);
}

/* ── IVP_Clustering_Visualizer ─────────────────────────────────────────── */

void ivp_clustering_visualizer_init(ivp_clustering_visualizer_t *v)
{
    ivp_clustering_visualizer_init_with_allocator(v, NULL);
}

void ivp_clustering_visualizer_init_with_allocator(
    ivp_clustering_visualizer_t *v, const ivp_allocator_t *allocator)
{
    if (!v) return;
    ivp_clustering_visualizer_shortrange_init_with_allocator(&v->shortrange, allocator);
    ivp_clustering_visualizer_longrange_init_with_allocator(&v->longrange, allocator);
}

void ivp_clustering_visualizer_destroy(ivp_clustering_visualizer_t *v)
{
    if (!v) return;
    ivp_clustering_visualizer_longrange_destroy(&v->longrange);
    ivp_clustering_visualizer_shortrange_destroy(&v->shortrange);
}

#ifdef IVP_HOME_BUILD
/* ── ivp_global_clustering_visualizer (ivp_clustering_visualizer.cxx:57) ── */

static ivp_clustering_visualizer_t ivp_cv_global_instance;
static bool ivp_cv_global_constructed = false;

static void ivp_cv_global_destroy(void)
{
    if (!ivp_cv_global_constructed) return;
    ivp_cv_global_constructed = false;
    ivp_clustering_visualizer_destroy(&ivp_cv_global_instance);
}

ivp_clustering_visualizer_t *ivp_clustering_visualizer_get_global(void)
{
    if (!ivp_cv_global_constructed) {
        ivp_clustering_visualizer_init(&ivp_cv_global_instance);
        ivp_cv_global_constructed = true;
        (void)atexit(ivp_cv_global_destroy);
    }
    return &ivp_cv_global_instance;
}
#endif /* IVP_HOME_BUILD */
