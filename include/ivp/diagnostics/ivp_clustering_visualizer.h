/* ivp_clustering_visualizer.h — IVP_Clustering_Visualizer
 *
 * Port of ivp_physics/ivp_clustering_visualizer.{hxx,cxx} and
 * ivp_physics/ivp_clustering_visual_hash.{hxx,cxx}.
 *
 * Helpers to visualise
 *   - shortrange: the object internal clustering (compact ledgetree) of a
 *     polygon object as seen by an intruder sphere (the traversal the
 *     reference performs for IVP_Mindist_Manager::create_exact_mindists);
 *   - longrange: the "time sphere" (OV tree element sphere) of objects.
 * They perform no output themselves: install callbacks that draw / print
 * what they are handed.
 *
 * Like the reference, the engine feeds the global instance
 * (ivp_global_clustering_visualizer below) only in IVP_HOME_BUILD builds
 * (CMake option IVP_HOME_BUILD, default OFF; the shipped reference builds
 * never define it):
 *   - IVP_Mindist_Manager::create_exact_mindists -> shortrange.analyze_collision
 *     (ivp_mindist.cxx:439; returns at once unless shortrange is enabled),
 *   - IVP_OV_Tree_Manager::insert_ov_element -> longrange.add_object (queues
 *     the time sphere even when longrange is disabled, like the reference),
 *   - IVP_OV_Tree_Manager::remove_ov_element -> longrange.remove_object
 *     (ivp_clustering_longrange.cxx:755 / 890).
 * The application enables the parts it wants, installs callbacks and calls
 * visualize_collisions / visualize once per frame.  Without IVP_HOME_BUILD
 * the engine never calls them: the application calls analyze_collision /
 * add_object / remove_object itself on its own instances.
 */
#ifndef IVP_CLUSTERING_VISUALIZER_H
#define IVP_CLUSTERING_VISUALIZER_H

#include <ivp/base/ivp_types.h>
#include <ivp/containers/ivp_pvec.h>
#include <ivp/containers/ivp_vhash.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/geometry/ivp_ledgetree.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_object;

/* IVP_CLUSTERING_VISUALIZER_FILTER */
typedef enum ivp_clustering_visualizer_filter {
    IVP_CV_FILTER_ACCEPT = 1,
    IVP_CV_FILTER_REJECT = 2
} ivp_clustering_visualizer_filter_t;

/* ── IVP_Clustering_Visualizer_Object_Hash ─────────────────────────────── *
 * A set of object pointers (IVP_VHash keyed by the pointer). */
typedef struct ivp_clustering_visualizer_object_hash {
    ivp_vhash_t hash;
} ivp_clustering_visualizer_object_hash_t;

/* create_size: initial capacity (rounded up to a power of two) */
void ivp_clustering_visualizer_object_hash_init(ivp_clustering_visualizer_object_hash_t *h, int create_size);
void ivp_clustering_visualizer_object_hash_destroy(ivp_clustering_visualizer_object_hash_t *h);
void ivp_clustering_visualizer_object_hash_add(ivp_clustering_visualizer_object_hash_t *h, struct ivp_object *obj);
struct ivp_object *ivp_clustering_visualizer_object_hash_remove(ivp_clustering_visualizer_object_hash_t *h,
                                                                struct ivp_object *obj);
struct ivp_object *ivp_clustering_visualizer_object_hash_find(const ivp_clustering_visualizer_object_hash_t *h,
                                                              struct ivp_object *obj);

/* ── Longrange ─────────────────────────────────────────────────────────── */

typedef struct ivp_clustering_visualizer_longrange_callback ivp_clustering_visualizer_longrange_callback_t;

typedef struct ivp_clustering_visualizer_longrange_callback_vtable {
    void (*visualize_request)(ivp_clustering_visualizer_longrange_callback_t *cb);
    void (*devisualize_request)(ivp_clustering_visualizer_longrange_callback_t *cb);
    void (*enable)(ivp_clustering_visualizer_longrange_callback_t *cb);
    void (*disable)(ivp_clustering_visualizer_longrange_callback_t *cb);
} ivp_clustering_visualizer_longrange_callback_vtable_t;

/* IVP_Clustering_Visualizer_Longrange_Callback: the visualizer fills the
 * data members, then calls the request. */
struct ivp_clustering_visualizer_longrange_callback {
    const ivp_clustering_visualizer_longrange_callback_vtable_t *vt;
    bool enabled;
    struct ivp_object *real_object;
    ivp_vec3f_t center;  /* IVP_U_Float_Point */
    ivp_real_t  radius;
};

/* IVP_Clustering_Visualizer_Longrange */
typedef struct ivp_clustering_visualizer_longrange {
    bool enabled;
    ivp_pvec_t graphics_callbacks; /* longrange callbacks (not owned)  */
    ivp_clustering_visualizer_object_hash_t accept_filter;
    ivp_clustering_visualizer_object_hash_t reject_filter;
    ivp_pvec_t visualize_data;     /* internal: pending spheres        */

    bool use_accept_filter;            /* default true  */
    bool use_reject_filter;            /* default true  */
    bool visualize_moveable_objects;   /* default false */
    bool visualize_unmoveable_objects; /* default false */
} ivp_clustering_visualizer_longrange_t;

/* Constructor: disabled, filters as above, 1024 entry filter hashes. */
void ivp_clustering_visualizer_longrange_init(ivp_clustering_visualizer_longrange_t *v);
void ivp_clustering_visualizer_longrange_destroy(ivp_clustering_visualizer_longrange_t *v);

void ivp_clustering_visualizer_longrange_add_object_to_filter(ivp_clustering_visualizer_longrange_t *v,
                                                              struct ivp_object *obj,
                                                              ivp_clustering_visualizer_filter_t filter);
void ivp_clustering_visualizer_longrange_remove_object_from_filter(ivp_clustering_visualizer_longrange_t *v,
                                                                   struct ivp_object *obj,
                                                                   ivp_clustering_visualizer_filter_t filter);

/* Queue the time sphere of an object (also when disabled). */
void ivp_clustering_visualizer_longrange_add_object(ivp_clustering_visualizer_longrange_t *v,
                                                    struct ivp_object *real_object, const ivp_vec3f_t *center,
                                                    ivp_real_t radius);
/* When enabled: drop the queued sphere of the object and send
 * devisualize_request to every callback. */
void ivp_clustering_visualizer_longrange_remove_object(ivp_clustering_visualizer_longrange_t *v,
                                                       struct ivp_object *real_object);

/* When enabled: hand every queued sphere (newest first) that passes the
 * moveable / unmoveable switches and the filters to every callback
 * (visualize_request) and drop it; spheres that do not pass stay queued. */
void ivp_clustering_visualizer_longrange_visualize(ivp_clustering_visualizer_longrange_t *v);

void ivp_clustering_visualizer_longrange_install_visualize_callback(
    ivp_clustering_visualizer_longrange_t *v, ivp_clustering_visualizer_longrange_callback_t *callback);
void ivp_clustering_visualizer_longrange_remove_visualize_callback(
    ivp_clustering_visualizer_longrange_t *v, ivp_clustering_visualizer_longrange_callback_t *callback);

/* enable / disable also call enable / disable of every callback and set its
 * enabled flag. */
void ivp_clustering_visualizer_longrange_enable(ivp_clustering_visualizer_longrange_t *v);
void ivp_clustering_visualizer_longrange_disable(ivp_clustering_visualizer_longrange_t *v);
bool ivp_clustering_visualizer_longrange_get_state(const ivp_clustering_visualizer_longrange_t *v);

/* ── Shortrange ────────────────────────────────────────────────────────── */

typedef struct ivp_clustering_visualizer_shortrange_callback ivp_clustering_visualizer_shortrange_callback_t;

typedef struct ivp_clustering_visualizer_shortrange_callback_vtable {
    void (*visualize_request_for_node)(ivp_clustering_visualizer_shortrange_callback_t *cb);
    void (*visualize_request_for_intruder_radius)(ivp_clustering_visualizer_shortrange_callback_t *cb);
} ivp_clustering_visualizer_shortrange_callback_vtable_t;

/* IVP_Clustering_Visualizer_Shortrange_Callback.  For a node: the node, its
 * sphere (position / sphere_radius, object coordinates of private_property)
 * and box half extents.  For the intruder: position = intruder position
 * (world), box_extents.k[0] = intruder radius. */
struct ivp_clustering_visualizer_shortrange_callback {
    const ivp_clustering_visualizer_shortrange_callback_vtable_t *vt;
    struct ivp_object *private_property;
    const ivp_compact_ledgetree_node_t *node;
    ivp_vec3r_t position;
    ivp_real_t  sphere_radius;
    ivp_vec3r_t box_extents;
    int  n_touches;      /* only used when node is terminal */
    bool reported_ledge; /* terminal node whose ledge the traversal reported */
    void *user_data;
};

/* Constructor (vt + reference defaults: sphere_radius -1, n_touches -1). */
void ivp_clustering_visualizer_shortrange_callback_init(
    ivp_clustering_visualizer_shortrange_callback_t *cb,
    const ivp_clustering_visualizer_shortrange_callback_vtable_t *vt);

/* IVP_CLUSTERING_SHORTRANGE_VISUALIZER_BOXMODE */
typedef enum ivp_clustering_shortrange_visualizer_boxmode {
    IVP_CV_TERMINALS_ONLY = 1,  /* report only the bounding boxes of terminals          */
    IVP_CV_ABORTED_BOXES = 2,   /* report all boxes that have been aborted during traversal */
    IVP_CV_TRAVERSED_BOXES = 3  /* report all visited boxes                              */
} ivp_clustering_shortrange_visualizer_boxmode_t;

struct ivp_cv_shortrange_objectdata;

/* IVP_Clustering_Visualizer_Shortrange */
typedef struct ivp_clustering_visualizer_shortrange {
    bool enabled;
    ivp_clustering_visualizer_object_hash_t accept_filter_intruder;
    ivp_clustering_visualizer_object_hash_t reject_filter_intruder;
    ivp_clustering_visualizer_object_hash_t accept_filter_private_property;
    ivp_clustering_visualizer_object_hash_t reject_filter_private_property;
    ivp_pvec_t output_callbacks; /* shortrange callbacks (not owned) */

    /* temporary data */
    struct ivp_object *private_property;
    struct ivp_object *intruder;
    ivp_real_t         intruder_radius;
    ivp_transform_r_t  intruder_matrix;
    ivp_vec3r_t        intruder_position_in_private_property_cs;
    ivp_pvec_t         pipelined_objects; /* struct ivp_cv_shortrange_objectdata* */
    struct ivp_cv_shortrange_objectdata *pipelined_private_property_data;

    ivp_clustering_shortrange_visualizer_boxmode_t boxmode; /* default TRAVERSED_BOXES */
    bool use_intruder_accept_filter;          /* default false */
    bool use_intruder_reject_filter;          /* default false */
    bool use_private_property_accept_filter;  /* default false */
    bool use_private_property_reject_filter;  /* default false */
    bool remove_longrange_visualization;      /* default false (unused, as in the reference) */

    int stats_n_reported_ledges;
    int stats_n_sphere_collisiontests;
    int stats_n_box_collisiontests;
} ivp_clustering_visualizer_shortrange_t;

/* Constructor: disabled, 64 entry filter hashes. */
void ivp_clustering_visualizer_shortrange_init(ivp_clustering_visualizer_shortrange_t *v);
void ivp_clustering_visualizer_shortrange_destroy(ivp_clustering_visualizer_shortrange_t *v);

/* When enabled: hand every pipelined node (objects and nodes newest first)
 * to the callbacks (visualize_request_for_node, then
 * visualize_request_for_intruder_radius per object when its intruder radius
 * is > 0) and empty the pipeline; counts stats_n_reported_ledges and resets
 * the sphere / box test counters.  As in the reference the first callback
 * consumes the pipeline. */
void ivp_clustering_visualizer_shortrange_visualize_collisions(ivp_clustering_visualizer_shortrange_t *v);

/* When enabled: for (intruder object0 radius0 -> private property object1)
 * and (object1 radius1 -> object0), unless filtered out, traverse the
 * private property's ledgetree with the intruder sphere (intruder position
 * at the current environment time, in the private property's object
 * coordinates) and pipeline the nodes according to boxmode.  Private
 * properties without a polygon (compact surface) surface manager are
 * skipped (the reference assumes polygons). */
void ivp_clustering_visualizer_shortrange_analyze_collision(ivp_clustering_visualizer_shortrange_t *v,
                                                            struct ivp_object *object0, ivp_real_t radius0,
                                                            struct ivp_object *object1, ivp_real_t radius1);

void ivp_clustering_visualizer_shortrange_add_object_to_intruder_filter(ivp_clustering_visualizer_shortrange_t *v,
                                                                        struct ivp_object *obj,
                                                                        ivp_clustering_visualizer_filter_t filter);
void ivp_clustering_visualizer_shortrange_add_object_to_private_property_filter(
    ivp_clustering_visualizer_shortrange_t *v, struct ivp_object *obj, ivp_clustering_visualizer_filter_t filter);
void ivp_clustering_visualizer_shortrange_remove_object_from_intruder_filter(
    ivp_clustering_visualizer_shortrange_t *v, struct ivp_object *obj, ivp_clustering_visualizer_filter_t filter);
void ivp_clustering_visualizer_shortrange_remove_object_from_private_property_filter(
    ivp_clustering_visualizer_shortrange_t *v, struct ivp_object *obj, ivp_clustering_visualizer_filter_t filter);

void ivp_clustering_visualizer_shortrange_install_visualize_callback(
    ivp_clustering_visualizer_shortrange_t *v, ivp_clustering_visualizer_shortrange_callback_t *callback);
void ivp_clustering_visualizer_shortrange_remove_visualize_callback(
    ivp_clustering_visualizer_shortrange_t *v, ivp_clustering_visualizer_shortrange_callback_t *callback);

void ivp_clustering_visualizer_shortrange_enable(ivp_clustering_visualizer_shortrange_t *v);
void ivp_clustering_visualizer_shortrange_disable(ivp_clustering_visualizer_shortrange_t *v);
bool ivp_clustering_visualizer_shortrange_get_state(const ivp_clustering_visualizer_shortrange_t *v);

/* ── IVP_Clustering_Visualizer ─────────────────────────────────────────── */

typedef struct ivp_clustering_visualizer {
    ivp_clustering_visualizer_shortrange_t shortrange;
    ivp_clustering_visualizer_longrange_t  longrange;
} ivp_clustering_visualizer_t;

void ivp_clustering_visualizer_init(ivp_clustering_visualizer_t *v);
void ivp_clustering_visualizer_destroy(ivp_clustering_visualizer_t *v);

#ifdef IVP_HOME_BUILD
/* ivp_global_clustering_visualizer (reference ivp_clustering_visualizer.cxx:57
 * / .hxx:276, IVP_HOME_BUILD only): a global object the reference constructs
 * before main.  C has no global constructors, so the instance is constructed
 * on first use by ivp_clustering_visualizer_get_global() (which also
 * registers its destruction with atexit, the reference's static destructor).
 * The macro keeps the reference spelling
 * ("ivp_global_clustering_visualizer.longrange") and always goes through the
 * getter, so the instance can never be used unconstructed.  Not thread safe
 * (neither is the reference). */
ivp_clustering_visualizer_t *ivp_clustering_visualizer_get_global(void);
#define ivp_global_clustering_visualizer (*ivp_clustering_visualizer_get_global())
#endif

#ifdef __cplusplus
}
#endif

#endif /* IVP_CLUSTERING_VISUALIZER_H */
