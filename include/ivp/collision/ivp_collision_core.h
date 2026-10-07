/* ivp_collision_core.h — Event-driven collision core (port of the reference
 * mindist / synapse / hull / OV-tree pipeline)
 *
 * Reference files (ivp_physics/ unless noted):
 *   ivp_collision.hxx, ivp_mindist.hxx, ivp_mindist_macros.hxx, ivp_mindist.cxx,
 *   ivp_mindist_recursive.cxx, ivp_oo_watcher.cxx, ivp_coll_del_root_mindist.cxx,
 *   ivp_intern/ivp_clustering_longrange.hxx, ivp_collision/ivp_clustering_longrange.cxx,
 *   ivp_intern/ivp_cache_object.hxx, ivp_intern/ivp_cache_ledge_point.hxx
 *
 * C mapping of the reference class hierarchy:
 *   IVP_Time_Event        -> ivp_time_event_t           (first member)
 *   IVP_Collision         -> ivp_collision_t             (vtable + delegator)
 *   IVP_Mindist_Base/Mindist/Mindist_Recursive -> ivp_mindist_t (is_recursive flag;
 *                            the recursive part and its IVP_Collision_Delegator are embedded)
 *   IVP_OO_Watcher        -> ivp_oo_watcher_t
 *   IVP_Synapse_Real      -> ivp_md_synapse_t            (IVP_Listener_Hull first member)
 */
#ifndef IVP_COLLISION_CORE_H
#define IVP_COLLISION_CORE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_legacy.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/containers/ivp_pvec.h>
#include <ivp/collision/ivp_hull_mgr.h>
#include <ivp/dynamics/ivp_time_manager.h>
#include <ivp/geometry/ivp_compact_ledge.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_object;
struct ivp_core;
struct ivp_environment;
struct ivp_contact_pt;
struct ivp_fr_system;
struct ivp_sim_unit;
struct ivp_mindist_mgr;
struct ivp_fr_synapse;

/* ── Mindist settings (IVP_Mindist_Settings) ─────────────────────────────── */

#define IVP_MAX_STEPS_FOR_COLLDIST_DECREASE 64

typedef struct ivp_mindist_settings {
    float  real_coll_dist;
    float  min_coll_dists;
    float  coll_dists[IVP_MAX_STEPS_FOR_COLLDIST_DECREASE];
    float  minimum_friction_dist;
    float  friction_dist;
    float  keeper_dist;
    float  speed_after_keeper_dist;
    float  distance_keepers_safety;
    float  max_distance_for_friction;
    float  max_distance_for_impact_system;
    float  mindist_change_force_dist;
    float  min_vertical_speed_at_collision;
    int    max_spawned_mindist_count;
    double event_queue_min_delta_time_base;
} ivp_mindist_settings_t;

/* The reference keeps one global instance.  libivp stores one copy in every
 * environment so independent worlds can use different tolerances and can be
 * simulated concurrently. */
void ivp_mindist_settings_set_collision_tolerance(ivp_mindist_settings_t *s, double t);
void ivp_mindist_settings_init(ivp_mindist_settings_t *s);

/* ── Enums ──────────────────────────────────────────────────────────────── */

typedef enum ivp_synapse_polygon_status {
    IVP_ST_POINT     = 0,
    IVP_ST_EDGE      = 1,
    IVP_ST_TRIANGLE  = 2,
    IVP_ST_BALL      = 3,
    IVP_ST_MAX_LEGAL = 4,
    IVP_ST_BACKSIDE  = 5
} ivp_synapse_polygon_status_t;

typedef enum ivp_minimal_dist_status {
    IVP_MD_UNINITIALIZED  = 0,
    IVP_MD_INVALID        = 2,
    IVP_MD_EXACT          = 3,
    IVP_MD_HULL_RECURSIVE = 4,
    IVP_MD_HULL           = 5
} ivp_minimal_dist_status_t;

typedef enum ivp_minimal_dist_recalc_result {
    IVP_MDRR_OK        = 0,
    IVP_MDRR_INTRUSION = 1
} ivp_minimal_dist_recalc_result_t;

typedef enum ivp_coll_type {
    IVP_COLL_NONE                 = 0x00,
    IVP_COLL_PP_COLL              = 0x10,
    IVP_COLL_PP_PK                = 0x11,
    IVP_COLL_PF_COLL              = 0x20,
    IVP_COLL_PF_NPF               = 0x21,
    IVP_COLL_PK_COLL              = 0x30,
    IVP_COLL_PK_PF                = 0x31,
    IVP_COLL_PK_KK                = 0x32,
    IVP_COLL_PK_NOT_MORE_PARALLEL = 0x33,
    IVP_COLL_KK_COLL              = 0x40,
    IVP_COLL_KK_PARALLEL          = 0x41,
    IVP_COLL_KK_PF                = 0x42
} ivp_coll_type_t;

typedef enum ivp_mindist_function {
    IVP_MF_COLLISION = 0,
    IVP_MF_PHANTOM   = 1
} ivp_mindist_function_t;

typedef enum ivp_mrc_type {
    IVP_MRC_UNINITIALIZED      = 0,
    IVP_MRC_OK                 = 1,
    IVP_MRC_ENDLESS_LOOP       = 2,
    IVP_MRC_BACKSIDE           = 3,
    IVP_MRC_ALREADY_CALCULATED = 4,
    IVP_MRC_ILLEGAL            = 5
} ivp_mrc_type_t;

typedef enum ivp_mindist_event_hint {
    IVP_EH_NOW,
    IVP_EH_SMALL_DELAY,
    IVP_EH_BIG_DELAY
} ivp_mindist_event_hint_t;

typedef enum ivp_mindist_recursive_types {
    IVP_MR_NORMAL                  = -1,
    IVP_MR_FIRST_SYNAPSE_RECURSIVE  = 0,
    IVP_MR_SECOND_SYNAPSE_RECURSIVE = 1
} ivp_mindist_recursive_types_t;

/* ── Collision / delegator base ─────────────────────────────────────────── */

typedef struct ivp_collision ivp_collision_t;
typedef struct ivp_collision_delegator ivp_collision_delegator_t;

typedef struct ivp_collision_vt {
    void (*get_objects)(ivp_collision_t *c, struct ivp_object *objects_out[2]);
    void (*get_ledges)(ivp_collision_t *c, const ivp_compact_ledge_t *ledges_out[2]);
    /* deletes the collision (reference default: P_DELETE_THIS) */
    void (*delegator_is_going_to_be_deleted_event)(ivp_collision_t *c, ivp_collision_delegator_t *d);
    /* virtual destructor */
    void (*destroy)(ivp_collision_t *c);
} ivp_collision_vt_t;

struct ivp_collision {
    ivp_time_event_t           te;            /* IVP_Time_Event base (index) */
    const ivp_collision_vt_t  *cvt;
    ivp_collision_delegator_t *delegator;
    int                        fvector_index[2];  /* OV-element backlinks */
};

typedef struct ivp_collision_delegator_vt {
    void (*collision_is_going_to_be_deleted_event)(ivp_collision_delegator_t *d, ivp_collision_t *c);
    void (*change_spawned_mindist_count)(ivp_collision_delegator_t *d, int change);   /* may be NULL */
    int  (*get_spawned_mindist_count)(ivp_collision_delegator_t *d);                 /* may be NULL (-1) */
} ivp_collision_delegator_vt_t;

struct ivp_collision_delegator {
    const ivp_collision_delegator_vt_t *dvt;
};

static inline void ivp_collision_delete(ivp_collision_t *c) { c->cvt->destroy(c); }

/* IVP_Collision_Delegator_Root */
typedef struct ivp_collision_delegator_root ivp_collision_delegator_root_t;
typedef struct ivp_collision_delegator_root_vt {
    void (*object_is_removed_from_collision_detection)(ivp_collision_delegator_root_t *r, struct ivp_object *o);
    ivp_collision_t *(*delegate_collisions_for_object)(ivp_collision_delegator_root_t *r,
                                                       struct ivp_object *base_object,
                                                       struct ivp_object *colliding_element);
    void (*environment_is_going_to_be_deleted_event)(ivp_collision_delegator_root_t *r,
                                                     struct ivp_environment *env);
} ivp_collision_delegator_root_vt_t;

struct ivp_collision_delegator_root {
    ivp_collision_delegator_t                 base;   /* collision_is_going_to_be_deleted_event */
    const ivp_collision_delegator_root_vt_t  *rvt;
};

/* The built-in IVP_Collision_Delegator_Root_Mindist */
ivp_collision_delegator_root_t *ivp_collision_delegator_root_mindist_create(struct ivp_environment *env);

/* ── FVector of collisions (IVP_U_FVector<IVP_Collision>) ───────────────── */

typedef struct ivp_coll_fvector {
    ivp_collision_t **elems;
    int n;
    int cap;
    const ivp_allocator_t *allocator;
} ivp_coll_fvector_t;

void ivp_coll_fvector_init(ivp_coll_fvector_t *v);
void ivp_coll_fvector_init_with_allocator(ivp_coll_fvector_t *v,
                                          const ivp_allocator_t *allocator);
void ivp_coll_fvector_clear(ivp_coll_fvector_t *v);
bool ivp_coll_fvector_reserve(ivp_coll_fvector_t *v, int minimum);
int  ivp_coll_fvector_index_of(const ivp_coll_fvector_t *v, const ivp_collision_t *c);
int  ivp_coll_fvector_add(ivp_coll_fvector_t *v, ivp_collision_t *c);
void ivp_coll_fvector_swap(ivp_coll_fvector_t *v, int i1, int i2);
void ivp_coll_fvector_remove_allow_resort(ivp_coll_fvector_t *v, ivp_collision_t *c);

/* ── Synapse (IVP_Synapse / IVP_Synapse_Real) ───────────────────────────── */

struct ivp_mindist;

typedef struct ivp_md_synapse {
    ivp_listener_hull_t        lh;           /* IVP_Listener_Hull */
    struct ivp_md_synapse     *next, *prev;  /* per object, exact/invalid lists */
    struct ivp_object         *l_obj;
    const ivp_compact_edge_t  *edge;         /* balls share one dummy edge */
    struct ivp_mindist        *mindist;      /* reference: mindist_offset */
    short                      status;       /* ivp_synapse_polygon_status_t */
} ivp_md_synapse_t;

static inline ivp_synapse_polygon_status_t ivp_md_synapse_get_status(const ivp_md_synapse_t *s) {
    return (ivp_synapse_polygon_status_t)s->status;
}
/* IVP_Synapse_Real::check_consistency_of_ledge (ivp_mindist.cxx:198): the
 * reference returns immediately (its check is "#if 0"). */
static inline void ivp_md_synapse_check_consistency_of_ledge(const ivp_md_synapse_t *s,
                                                             const ivp_compact_edge_t *test_edge) {
    (void)s;
    (void)test_edge;
}
static inline void ivp_md_synapse_update(ivp_md_synapse_t *s, const ivp_compact_edge_t *e,
                                         ivp_synapse_polygon_status_t st) {
    IVP_IF(st != IVP_ST_BALL) {   /* ivp_mindist.hxx:258 */
        ivp_md_synapse_check_consistency_of_ledge(s, e);
    }
    s->edge = e;
    s->status = (short)st;
}
const ivp_compact_ledge_t *ivp_md_synapse_get_ledge(const ivp_md_synapse_t *s);
struct ivp_core *ivp_md_synapse_get_core(const ivp_md_synapse_t *s);

/* ── Mindist (IVP_Mindist_Base / IVP_Mindist / IVP_Mindist_Recursive) ───── */

typedef struct ivp_mindist {
    ivp_collision_t coll;                   /* IVP_Collision base */

    /* IVP_Mindist_Base */
    uint8_t   coll_type;                    /* ivp_coll_type_t */
    uint8_t   synapse_sort_flag;
    bool      is_in_phantom_set;
    uint8_t   mindist_function;             /* ivp_mindist_function_t */
    uint8_t   recalc_result;                /* ivp_minimal_dist_recalc_result_t */
    bool      disable_halfspace_optimization;
    uint8_t   mindist_status;               /* ivp_minimal_dist_status_t */
    unsigned  coll_dist_selector;
    unsigned  coll_dist_decrease_counter;
    ivp_md_synapse_t synapse[2];
    float     sum_extra_radius;
    float     len_numerator;
    float     contact_dot_diff_center;
    double    sum_angular_hull_time;
    ivp_vec3f_t contact_plane;              /* IVP_U_Float_Point */

    /* IVP_Mindist */
    ivp_time_code_t recalc_time_stamp;
    struct ivp_mindist *next, *prev;
    const ivp_compact_edge_t *last_visited_triangle;

    /* IVP_Mindist_Recursive (valid if is_recursive) */
    bool      is_recursive;
    ivp_collision_delegator_t rec_delegator;   /* IVP_Collision_Delegator part */
    int       recursive_status;                /* ivp_mindist_recursive_types_t */
    ivp_coll_fvector_t mindists;               /* spawned children */
    int       spawned_mindist_count;

    struct ivp_environment *env;
} ivp_mindist_t;

static inline ivp_md_synapse_t *ivp_mindist_get_synapse(ivp_mindist_t *md, int i) {
    return &md->synapse[i];
}
static inline ivp_md_synapse_t *ivp_mindist_get_sorted_synapse(ivp_mindist_t *md, int i) {
    return &md->synapse[md->synapse_sort_flag ^ (unsigned)i];
}
ivp_real_t ivp_mindist_get_coll_dist(const ivp_mindist_t *md);
static inline float ivp_mindist_get_length(const ivp_mindist_t *md) { return md->len_numerator; }

ivp_mindist_t *ivp_mindist_create(struct ivp_environment *env, ivp_collision_delegator_t *del, bool recursive);
void ivp_mindist_init_mindist(ivp_mindist_t *md, struct ivp_object *pop0, struct ivp_object *pop1,
                              const ivp_compact_edge_t *e0, const ivp_compact_edge_t *e1);
ivp_mrc_type_t ivp_mindist_recalc_mindist(ivp_mindist_t *md);
ivp_mrc_type_t ivp_mindist_recalc_invalid_mindist(ivp_mindist_t *md);
void ivp_mindist_update_exact_mindist_events(ivp_mindist_t *md, bool allow_hull_conversion,
                                             ivp_mindist_event_hint_t hint);
void ivp_mindist_exact_mindist_went_invalid(ivp_mindist_t *md, struct ivp_mindist_mgr *mm);
void ivp_mindist_do_impact(ivp_mindist_t *md);
void ivp_mindist_mindist_rescue_push(ivp_mindist_t *md);
struct ivp_contact_pt *ivp_mindist_try_to_generate_managed_friction(ivp_mindist_t *md,
                                                                    struct ivp_fr_system **associated_fs,
                                                                    bool *having_new,
                                                                    struct ivp_sim_unit *sim_unit_not_destroy,
                                                                    bool call_recalc_svals);
void ivp_mindist_create_cp_in_advance_pretension(ivp_mindist_t *md, struct ivp_object *robject, float gap_len);

/* ── OO watcher (IVP_OO_Watcher / IVP_Synapse_OO) ───────────────────────── */

typedef struct ivp_oo_watcher ivp_oo_watcher_t;

typedef struct ivp_synapse_oo {
    ivp_listener_hull_t lh;
    struct ivp_object  *object;
    ivp_oo_watcher_t   *watcher;
} ivp_synapse_oo_t;

struct ivp_oo_watcher {
    ivp_collision_t           coll;       /* IVP_Collision */
    ivp_collision_delegator_t del;        /* IVP_Collision_Delegator */
    ivp_synapse_oo_t          synapses[2];
    ivp_coll_fvector_t        mindists;
    struct ivp_environment   *env;
};

ivp_oo_watcher_t *ivp_oo_watcher_create(ivp_collision_delegator_t *del, struct ivp_object *obj0,
                                        struct ivp_object *obj1);

/* ── OV tree (IVP_OV_Element / IVP_OV_Node / IVP_OV_Tree_Manager) ───────── */

typedef struct ivp_ov_node_data {
    int x, y, z;
    int rasterlevel;
    int sizelevel;
} ivp_ov_node_data_t;

typedef struct ivp_ov_node {
    ivp_ov_node_data_t   data;
    struct ivp_ov_node  *parent;
    ivp_pvec_t           children;   /* ivp_ov_node_t* */
    ivp_pvec_t           elements;   /* ivp_ov_element_t* */
    struct ivp_ov_node  *hash_next;  /* lookup chain */
} ivp_ov_node_t;

typedef struct ivp_ov_element {
    ivp_listener_hull_t  lh;
    ivp_ov_node_t       *node;
    ivp_hull_mgr_t      *hull_manager;   /* set if in hull manager */
    ivp_vec3f_t          center;         /* IVP_U_Float_Point */
    float                radius;
    struct ivp_object   *real_object;
    ivp_coll_fvector_t   collision_fvector;
} ivp_ov_element_t;

#define IVP_OV_HASH_INITIAL_SIZE 1024u

typedef struct ivp_ov_tree {
    double          powerlist[81];
    ivp_ov_node_t   search_node;
    ivp_pvec_t     *collision_partners;
    ivp_ov_node_t **hash;
    size_t          hash_size;
    size_t          node_count;
    const ivp_allocator_t *allocator;
    ivp_ov_node_t  *root;
} ivp_ov_tree_t;

void   ivp_ov_tree_init(ivp_ov_tree_t *t);
ivp_result_t ivp_ov_tree_init_with_allocator(ivp_ov_tree_t *t, const ivp_allocator_t *allocator);
void   ivp_ov_tree_destroy(ivp_ov_tree_t *t);
ivp_result_t ivp_ov_tree_insert_ov_element_ex(
    ivp_ov_tree_t *t, ivp_ov_element_t *element,
    double min_radius, double max_radius,
    ivp_pvec_t *colliding_balls, double *used_radius_out);
double ivp_ov_tree_insert_ov_element(ivp_ov_tree_t *t, ivp_ov_element_t *element,
                                     double min_radius, double max_radius,
                                     ivp_pvec_t *colliding_balls /* ivp_ov_element_t*, may be NULL */);
void   ivp_ov_tree_remove_ov_element(ivp_ov_tree_t *t, ivp_ov_element_t *element);

/* IVP_OV_Tree_Manager::get_luf_coordinates_ws: world space left-upper-front
 * corner (IVP_U_Float_Point) and edge length (IVP_FLOAT) of a node's cube.
 * Used by the ray / sphere solver tree walks. */
void   ivp_ov_tree_get_luf_coordinates_ws(const ivp_ov_tree_t *t, const ivp_ov_node_t *node,
                                          float p[3], float *cubesize);
/* IVP_OV_Tree_Manager::get_center_coordinates_ws */
void   ivp_ov_tree_get_center_coordinates_ws(const ivp_ov_tree_t *t, const ivp_ov_node_t *node,
                                             float p[3], float *cubesize);

ivp_ov_element_t *ivp_ov_element_create(struct ivp_object *obj);
void ivp_ov_element_destroy(ivp_ov_element_t *el);
void ivp_ov_element_add_to_hull_manager(ivp_ov_element_t *el, ivp_hull_mgr_t *hm, double hull_time);
void ivp_ov_element_add_oo_collision(ivp_ov_element_t *el, ivp_collision_t *c);
void ivp_ov_element_remove_oo_collision(ivp_ov_element_t *el, ivp_collision_t *c);

/* ── Mindist manager (IVP_Mindist_Manager) ──────────────────────────────── */

typedef struct ivp_mindist_mgr {
    bool                   scanning_universe;
    struct ivp_environment *environment;
    ivp_mindist_t         *exact_mindists;
    ivp_pvec_t             wheel_look_ahead_mindists;
    ivp_mindist_t         *invalid_mindists;
    /* Top-level OV rechecks retain their high-water temporary storage.
     * Recursive rechecks use local vectors so traversal order is unchanged. */
    ivp_pvec_t             recheck_colliding_elements;
    ivp_pvec_t             recheck_new_objects;
    unsigned               recheck_depth;
    struct ivp_collision_diff_storage *recheck_diff_storage;
} ivp_mindist_mgr_t;

void ivp_mindist_mgr_init(ivp_mindist_mgr_t *mm, struct ivp_environment *env);
void ivp_mindist_mgr_destroy(ivp_mindist_mgr_t *mm);

void ivp_mindist_mgr_create_exact_mindists(struct ivp_object *obj0, struct ivp_object *obj1, double scan_radius,
                                           ivp_coll_fvector_t *existing_collisions,
                                           const ivp_compact_ledge_t *single_ledge0,
                                           const ivp_compact_ledge_t *single_ledge1,
                                           const ivp_compact_ledge_t *root_ledge0,
                                           const ivp_compact_ledge_t *root_ledge1,
                                           ivp_collision_delegator_t *del);
void ivp_mindist_mgr_insert_exact_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md);
void ivp_mindist_mgr_insert_and_recalc_exact_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md);
void ivp_mindist_mgr_insert_and_recalc_phantom_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md);
void ivp_mindist_mgr_remove_exact_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md);
void ivp_mindist_mgr_insert_invalid_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md);
void ivp_mindist_mgr_remove_invalid_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md);
void ivp_mindist_mgr_mindist_entered_phantom(ivp_mindist_t *md);
void ivp_mindist_mgr_mindist_left_phantom(ivp_mindist_t *md);
void ivp_mindist_mgr_remove_hull_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md);
void ivp_mindist_mgr_insert_hull_mindist2(ivp_mindist_t *md, float hull_time0, float hull_time1);
void ivp_mindist_mgr_insert_hull_mindist(ivp_mindist_t *md, float hull_time);
void ivp_mindist_mgr_insert_lazy_hull_mindist2(ivp_mindist_t *md, float hull_time0, float hull_time1);
void ivp_mindist_mgr_insert_lazy_hull_mindist(ivp_mindist_t *md, float hull_time);
void ivp_mindist_mgr_recheck_ov_element(ivp_mindist_mgr_t *mm, struct ivp_object *object);
void ivp_mindist_mgr_recalc_exact_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md);
void ivp_mindist_mgr_recalc_all_exact_mindists(ivp_mindist_mgr_t *mm);
void ivp_mindist_mgr_recalc_all_exact_wheel_mindist(ivp_mindist_mgr_t *mm);
void ivp_mindist_mgr_recalc_all_exact_mindists_events(ivp_mindist_mgr_t *mm);
void ivp_mindist_mgr_enable_collision_detection_for_object(ivp_mindist_mgr_t *mm, struct ivp_object *obj);

/* ── Cache object / cache ledge point (IVP_Cache_Object, IVP_Cache_Ledge_Point) ── */

typedef struct ivp_cache_obj {
    ivp_time_code_t  valid_until_time_code;
    int              reference_count;
    struct ivp_object *object;
    ivp_quatr_t       q_world_f_object;   /* IVP_U_Quat (double) */
    ivp_transform_r_t m_world_f_object;   /* IVP_U_Matrix */
    ivp_vec3r_t       core_pos;           /* IVP_U_Point */
} ivp_cache_obj_t;

/* get_cache_object (locks) / get_cache_object_no_lock */
ivp_cache_obj_t *ivp_object_get_cache_object(struct ivp_object *obj);
ivp_cache_obj_t *ivp_object_get_cache_object_no_lock(struct ivp_object *obj);
static inline void ivp_cache_obj_remove_reference(ivp_cache_obj_t *co)
{
    IVP_ASSERT(co->reference_count == 1);   /* ivp_cache_object.hxx:27 */
    co->reference_count--;
}
void ivp_cache_obj_update(ivp_cache_obj_t *co);
void ivp_object_invalid_cache_object(struct ivp_object *obj);

typedef struct ivp_cache_ledge_point {
    const ivp_compact_poly_point_t *compact_poly_points;
    const ivp_compact_ledge_t      *compact_ledge;
    ivp_cache_obj_t                *clp_cache_object;
    struct ivp_object              *clp_object;
    union {
        ivp_md_synapse_t      *synapse;
        struct ivp_fr_synapse *synapse_friction;
    } tmp;
} ivp_cache_ledge_point_t;

static inline void ivp_cache_ledge_point_init(ivp_cache_ledge_point_t *clp, struct ivp_object *obj,
                                              const ivp_compact_ledge_t *ledge) {
    clp->compact_ledge = ledge;
    clp->clp_cache_object = ivp_object_get_cache_object(obj);
    clp->clp_object = obj;
    clp->compact_poly_points = ivp_ledge_get_point_array(ledge);
    clp->tmp.synapse = NULL;
}
static inline void ivp_cache_ledge_point_remove_reference(ivp_cache_ledge_point_t *clp) {
    ivp_cache_obj_remove_reference(clp->clp_cache_object);
}

typedef struct ivp_cache_ball {
    ivp_cache_obj_t   *cache_object;
    struct ivp_object *object;
    union { ivp_md_synapse_t *synapse; } tmp;
} ivp_cache_ball_t;

#ifdef __cplusplus
}
#endif

#endif /* IVP_COLLISION_CORE_H */
