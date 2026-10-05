/* ivp_mindist_core.c — port of IVP_Mindist / IVP_Mindist_Manager and friends
 *
 * Reference: ivp_physics/ivp_mindist.cxx, ivp_mindist_macros.hxx,
 * ivp_mindist_recursive.cxx, ivp_oo_watcher.cxx, ivp_coll_del_root_mindist.cxx,
 * ivp_surface_manager/ivp_surman_polygon.cxx (get_all_ledges_within_radius).
 */
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/diagnostics/ivp_debug_manager.h>
#include <ivp/diagnostics/ivp_statistic_manager.h>
#ifdef IVP_HOME_BUILD
#include <ivp/diagnostics/ivp_clustering_visualizer.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "collision/ivp_lvec.h"
#include "collision/ivp_mindist_debug.h"
#include "collision/ivp_mindist_event_internal.h"
#include "collision/ivp_recheck_diagnostics.h"
#include "collision/ivp_collision_diff.h"
#include "dynamics/ivp_env_internal.h"
#include "dynamics/ivp_env_hot.h"


/* g_pCurrentMindist / g_fDeferDeleteMindist (ivp_mindist.cxx) */
ivp_mindist_t *ivp_g_current_mindist = NULL;
bool ivp_g_defer_delete_mindist = false;

#ifdef IVP_BENCHMARK_DIAGNOSTICS
static ivp_recheck_diagnostics_t recheck_diagnostics;
void ivp_recheck_diagnostics_reset(void)
{
    recheck_diagnostics = (ivp_recheck_diagnostics_t){0};
}
ivp_recheck_diagnostics_t ivp_recheck_diagnostics_get(void)
{
    return recheck_diagnostics;
}
ivp_recheck_diagnostics_t *ivp_recheck_diagnostics_mutable(void)
{
    return &recheck_diagnostics;
}
#endif

/* phantom glue (ivp_controller_phantom.c) */
void  ivp_phantom_mindist_entered_volume(void *controller_phantom, ivp_mindist_t *md);
void  ivp_phantom_mindist_left_volume(void *controller_phantom, ivp_mindist_t *md);
float ivp_phantom_get_exit_policy_extra_radius(void *controller_phantom);

/* ivp_lcls (ivp_mindist_minimize.c) */
D ivp_lcls_calc_qlen_PF_F_space(const ivp_compact_ledge_t *ledge, const ivp_compact_triangle_t *tri,
                                const ivp_vec3r_t *object_pos);

/* ── IVP_Mindist_Settings ───────────────────────────────────────────────── */

ivp_mindist_settings_t ivp_mindist_settings = {
    /* filled by ivp_mindist_settings_init on first environment creation */
    0
};

void ivp_mindist_settings_set_collision_tolerance(ivp_mindist_settings_t *s, double t)
{
    s->real_coll_dist = (float)(0.1f * t);
    s->min_coll_dists = (float)((double)s->real_coll_dist + 0.9f * t);
    s->minimum_friction_dist = (float)((double)s->min_coll_dists + 0.0f * t);
    for (int i = IVP_MAX_STEPS_FOR_COLLDIST_DECREASE - 1; i >= 0; i--) {
        s->coll_dists[i] = (float)((double)s->min_coll_dists +
                                   ((double)i / (double)IVP_MAX_STEPS_FOR_COLLDIST_DECREASE) *
                                       (double)(s->minimum_friction_dist - s->min_coll_dists));
    }
    s->friction_dist = (float)((double)s->minimum_friction_dist + 1.0f * t);
    s->keeper_dist = (float)((double)s->friction_dist + 0.3f * t);
    /* ivp_sqrtf(IVP_DOUBLE) of a float product */
    s->speed_after_keeper_dist = (float)sqrt((double)(2.0f * (s->keeper_dist - s->min_coll_dists) * 9.81f));
    s->distance_keepers_safety = (float)(0.01f * t);
    s->max_distance_for_friction = (float)((double)s->friction_dist + 2.5f * t);
    s->max_distance_for_impact_system = (float)((double)s->friction_dist + 20.0f * t);
    s->min_vertical_speed_at_collision = (float)(2.0f * t);
    s->mindist_change_force_dist = s->min_coll_dists * 0.1f;
}

void ivp_mindist_settings_init(ivp_mindist_settings_t *s)
{
    ivp_mindist_settings_set_collision_tolerance(s, 0.01f);
    s->event_queue_min_delta_time_base = 1.0f;
    s->max_spawned_mindist_count = 1000;
}

/* ── object helpers ─────────────────────────────────────────────────────── */

typedef struct IVP_ALIGN16 ball_dummy_ledge {
    unsigned char bytes[sizeof(ivp_compact_ledge_t) + sizeof(ivp_compact_triangle_t)];
} ball_dummy_ledge_t;
static ball_dummy_ledge_t ivp_ball_dummy_ledge;   /* IVP_SurfaceManager_Ball::compact_ledge (zeroed) */

static const ivp_compact_ledge_t *ball_ledge(void) { return (const ivp_compact_ledge_t *)&ivp_ball_dummy_ledge; }

static inline bool obj_is_ball(const ivp_object_t *o) { return o->type == IVP_OBJECT_BALL; }

static const ivp_compact_ledgetree_node_t *obj_ledgetree_root(const ivp_object_t *o)
{
    const ivp_surman_t *sm = o->surman;
    if (!sm) return NULL;
    /* only managers that expose a ledgetree (IVP_SurfaceManager_Polygon);
     * grid / custom managers are queried through get_all_ledges_within_radius */
    return sm->vtable->get_ledgetree_root ? sm->vtable->get_ledgetree_root(sm) : NULL;
}

/* IVP_SurfaceManager::get_single_convex */
static const ivp_compact_ledge_t *obj_get_single_convex(const ivp_object_t *o)
{
    if (obj_is_ball(o)) return ball_ledge();
    const ivp_compact_ledgetree_node_t *root = obj_ledgetree_root(o);
    if (root) {
        if (ivp_ledgetree_is_terminal(root)) return ivp_ledgetree_get_compact_ledge(root);
        return ivp_ledgetree_get_compact_hull(root);
    }
    return o->surman ? ivp_surman_get_single_convex(o->surman) : NULL;
}

/* IVP_SurfaceManager_Polygon_Solver::traverse_cluster */
static void traverse_cluster(const ivp_compact_ledgetree_node_t *node, const ivp_vec3r_t *visitor_pos_os, D radius,
                             ivp_pvec_t *out)
{
    const D work = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;
    ivp_vec3r_t dist_vec;
    ivp_vec3f_t center;
    lvf_set3(&center, node->center[0], node->center[1], node->center[2]);
    lvp_subtract2_fp(&dist_vec, &center, visitor_pos_os);
    D check_dist = radius + (D)node->radius;
    if (lvp_quad_length(&dist_vec) > check_dist * check_dist) return;
    float work2 = (float)(work * (D)node->radius);
    /* uchar * IVP_FLOAT is a float product */
    if ((D)fabs((double)dist_vec.k[0]) >= ((D)((float)node->box_sizes[0] * work2) + radius)) return;
    if ((D)fabs((double)dist_vec.k[1]) >= ((D)((float)node->box_sizes[1] * work2) + radius)) return;
    if ((D)fabs((double)dist_vec.k[2]) >= ((D)((float)node->box_sizes[2] * work2) + radius)) return;
    const ivp_compact_ledge_t *hull = ivp_ledgetree_get_compact_hull(node);
    if (hull) {
        ivp_pvec_add(out, (void *)hull);
        return;
    }
    if (ivp_ledgetree_is_terminal(node)) {
        const ivp_compact_ledge_t *ledge = ivp_ledgetree_get_compact_ledge(node);
        if (ivp_ledge_get_n_triangles(ledge) == 2) {
            D qdist = ivp_lcls_calc_qlen_PF_F_space(ledge, ivp_ledge_get_first_triangle(ledge), visitor_pos_os);
            if (qdist > radius * radius) return;
        }
        ivp_pvec_add(out, (void *)ledge);
        return;
    }
    traverse_cluster(ivp_ledgetree_left_son(node), visitor_pos_os, radius, out);
    traverse_cluster(ivp_ledgetree_right_son(node), visitor_pos_os, radius, out);
}

static const ivp_compact_ledgetree_node_t *ledge_get_ledgetree_node(const ivp_compact_ledge_t *l)
{
    IVP_ASSERT(!ivp_ledge_is_terminal(l));   /* ivp_compact_ledge.hxx:213 */
    return (const ivp_compact_ledgetree_node_t *)((const char *)l + l->node_or_client);
}

static void obj_ledge_collect_cb(void *ctx, const ivp_compact_ledge_t *ledge)
{
    ivp_pvec_add((ivp_pvec_t *)ctx, (void *)ledge);
}

/* IVP_SurfaceManager::get_all_ledges_within_radius */
static void obj_get_all_ledges_within_radius(const ivp_object_t *o, const ivp_vec3r_t *visitor_pos_os, D radius,
                                             const ivp_compact_ledge_t *root_ledge, ivp_pvec_t *out)
{
    if (obj_is_ball(o)) {
        if (lvp_quad_length(visitor_pos_os) > radius * radius) return;
        ivp_pvec_add(out, (void *)ball_ledge());
        return;
    }
    if (o->surman && o->surman->vtable->get_all_ledges_within_radius) {
        /* surface manager port of IVP_SurfaceManager::get_all_ledges_within_radius */
        double vp[3] = { (double)visitor_pos_os->k[0], (double)visitor_pos_os->k[1], (double)visitor_pos_os->k[2] };
        o->surman->vtable->get_all_ledges_within_radius(o->surman, vp, (double)radius, root_ledge, NULL,
                                                        obj_ledge_collect_cb, out);
        return;
    }
    if (!root_ledge) {
        const ivp_compact_ledgetree_node_t *root_node = obj_ledgetree_root(o);
        if (!root_node) {
            const ivp_compact_ledge_t *l = o->surman ? ivp_surman_get_single_convex(o->surman) : NULL;
            if (l) ivp_pvec_add(out, (void *)l);
            return;
        }
        traverse_cluster(root_node, visitor_pos_os, radius, out);
        return;
    }
    if (ivp_ledge_is_terminal(root_ledge)) {
        if (ivp_ledge_get_n_triangles(root_ledge) == 2) {
            D qdist = ivp_lcls_calc_qlen_PF_F_space(root_ledge, ivp_ledge_get_first_triangle(root_ledge), visitor_pos_os);
            if (qdist > radius * radius) return;
        }
        ivp_pvec_add(out, (void *)root_ledge);
        return;
    }
    const ivp_compact_ledgetree_node_t *root_node = ledge_get_ledgetree_node(root_ledge);
    if (ivp_ledgetree_is_terminal(root_node)) {
        traverse_cluster(root_node, visitor_pos_os, radius, out);
        return;
    }
    traverse_cluster(ivp_ledgetree_left_son(root_node), visitor_pos_os, radius, out);
    traverse_cluster(ivp_ledgetree_right_son(root_node), visitor_pos_os, radius, out);
}

static inline bool obj_is_moving(const ivp_object_t *o) { return IVP_MTIS_IS_MOVING(o->object_movement_state) != 0; }

/* ── synapse ────────────────────────────────────────────────────────────── */

const ivp_compact_ledge_t *ivp_md_synapse_get_ledge(const ivp_md_synapse_t *s)
{
    return ivp_edge_get_compact_ledge(s->edge);
}

struct ivp_core *ivp_md_synapse_get_core(const ivp_md_synapse_t *s) { return s->l_obj->physical_core; }

static void syn_remove_exact_from_object(ivp_md_synapse_t *s)
{
    if (s->next) s->next->prev = s->prev;
    if (s->prev) s->prev->next = s->next;
    else {
        IVP_ASSERT(s->l_obj->exact_synapses == s);
        s->l_obj->exact_synapses = s->next;
    }
    IVP_IF(1) {   /* ivp_mindist_macros.hxx:19 */
        s->next = s->prev = s;
    }
}

static void syn_insert_exact_in_object(ivp_md_synapse_t *s)
{
    IVP_ASSERT(s->next == s);
    IVP_ASSERT(s->prev == s);
    s->next = s->l_obj->exact_synapses;
    if (s->next) s->next->prev = s;
    s->prev = NULL;
    s->l_obj->exact_synapses = s;
}

static void syn_remove_invalid_from_object(ivp_md_synapse_t *s)
{
    if (s->next) s->next->prev = s->prev;
    if (s->prev) s->prev->next = s->next;
    else {
        IVP_ASSERT(s->l_obj->invalid_synapses == s);
        s->l_obj->invalid_synapses = s->next;
    }
    IVP_IF(1) {   /* ivp_mindist_macros.hxx:51 */
        s->next = s->prev = s;
    }
}

static void syn_insert_invalid_in_object(ivp_md_synapse_t *s)
{
    s->next = s->l_obj->invalid_synapses;
    if (s->next) s->next->prev = s;
    s->prev = NULL;
    s->l_obj->invalid_synapses = s;
}

static float syn_insert_in_hull_manager(ivp_md_synapse_t *s, double rel_hull_time)
{
    ivp_time_t t = ivp_env_get_current_time(s->l_obj->owner);
    return ivp_hull_mgr_insert_synapse(s->l_obj->hull_manager, &s->lh, t, rel_hull_time);
}

static float syn_insert_lazy_in_hull_manager(ivp_md_synapse_t *s, double rel_hull_time)
{
    ivp_time_t t = ivp_env_get_current_time(s->l_obj->owner);
    return ivp_hull_mgr_insert_lazy_synapse(s->l_obj->hull_manager, &s->lh, t, rel_hull_time);
}

static void mindist_hull_limit_exceeded_event(ivp_mindist_t *md, float hull_intrusion_value);
static void mindist_recursive_rec_hull_limit_exceeded_event(ivp_mindist_t *md);

static void synapse_hull_limit_exceeded(ivp_listener_hull_t *lh, ivp_hull_mgr_t *hm, float v)
{
    (void)hm;
    ivp_md_synapse_t *s = (ivp_md_synapse_t *)lh;
    mindist_hull_limit_exceeded_event(s->mindist, v);
}

static void synapse_hull_manager_deleted(ivp_listener_hull_t *lh, ivp_hull_mgr_t *hm)
{
    (void)hm;
    ivp_md_synapse_t *s = (ivp_md_synapse_t *)lh;
    ivp_collision_delete(&s->mindist->coll);
}

static void synapse_hull_manager_reset(ivp_listener_hull_t *lh, float dt, float center_dt)
{
    ivp_md_synapse_t *s = (ivp_md_synapse_t *)lh;
    s->mindist->sum_angular_hull_time += (double)(dt - center_dt);
}

static const ivp_listener_hull_vt_t synapse_vt = {
    IVP_HULL_ELEM_POLYGON,
    synapse_hull_limit_exceeded,
    synapse_hull_manager_deleted,
    synapse_hull_manager_reset
};

static void syn_init(ivp_md_synapse_t *s, ivp_mindist_t *md, ivp_object_t *obj, const ivp_compact_edge_t *e,
                     ivp_synapse_polygon_status_t st)
{
    ivp_listener_hull_init(&s->lh, &synapse_vt);
    s->next = s->prev = NULL;
    /* IVP_Synapse::init_synapse_real */
    s->mindist = md;
    s->l_obj = obj;
    IVP_IF(1) {   /* ivp_mindist.hxx:93 */
        s->next = s->prev = s;
    }
    ivp_md_synapse_update(s, e, st);
}

/* ── mindist collision vtable ───────────────────────────────────────────── */

static void mindist_get_objects(ivp_collision_t *c, struct ivp_object *out[2])
{
    ivp_mindist_t *md = (ivp_mindist_t *)c;
    out[0] = md->synapse[0].l_obj;
    out[1] = md->synapse[1].l_obj;
}

static void mindist_get_ledges(ivp_collision_t *c, const ivp_compact_ledge_t *out[2])
{
    ivp_mindist_t *md = (ivp_mindist_t *)c;
    out[0] = ivp_edge_get_compact_ledge(md->synapse[0].edge);
    out[1] = ivp_edge_get_compact_ledge(md->synapse[1].edge);
}

static void mindist_destroy(ivp_collision_t *c);

static void collision_default_delegator_deleted(ivp_collision_t *c, ivp_collision_delegator_t *d)
{
    (void)d;
    ivp_collision_delete(c);
}

static const ivp_collision_vt_t mindist_collision_vt = {
    mindist_get_objects,
    mindist_get_ledges,
    collision_default_delegator_deleted,
    mindist_destroy
};

static void mindist_simulate_time_event(ivp_time_event_t *ev, struct ivp_environment *env);

/* IVP_Mindist_Recursive delegator part */
static void rec_collision_is_going_to_be_deleted_event(ivp_collision_delegator_t *d, ivp_collision_t *c);
static void rec_change_spawned_mindist_count(ivp_collision_delegator_t *d, int change);
static int  rec_get_spawned_mindist_count(ivp_collision_delegator_t *d);

static const ivp_collision_delegator_vt_t rec_delegator_vt = {
    rec_collision_is_going_to_be_deleted_event,
    rec_change_spawned_mindist_count,
    rec_get_spawned_mindist_count
};

static inline ivp_mindist_t *rec_from_delegator(ivp_collision_delegator_t *d) {
    return (ivp_mindist_t *)((char *)d - offsetof(ivp_mindist_t, rec_delegator));
}

ivp_mindist_t *ivp_mindist_create(struct ivp_environment *env, ivp_collision_delegator_t *del, bool recursive)
{
    IVP_ASSERT(del);   /* ivp_mindist.cxx:110 */
    ivp_mindist_t *md = (ivp_mindist_t *)calloc(1, sizeof(*md));
    IVP_ASSERT(md);
    if (!md) return NULL;
    ivp_time_event_init(&md->coll.te, mindist_simulate_time_event);
    md->coll.cvt = &mindist_collision_vt;
    md->coll.delegator = del;
    md->coll.fvector_index[0] = md->coll.fvector_index[1] = -1;
    md->coll_type = IVP_COLL_NONE;
    md->synapse_sort_flag = 0;
    md->recalc_result = IVP_MDRR_OK;
    md->is_in_phantom_set = false;
    md->mindist_status = IVP_MD_UNINITIALIZED;
    md->mindist_function = IVP_MF_COLLISION;
    md->coll_dist_selector = IVP_MAX_STEPS_FOR_COLLDIST_DECREASE - 1;
    md->coll_dist_decrease_counter = 0;
    md->sum_extra_radius = 0;
    md->len_numerator = 0;
    md->contact_dot_diff_center = 0;
    md->disable_halfspace_optimization = false;
    md->sum_angular_hull_time = 0.0;
    md->recalc_time_stamp = 0;
    md->next = md->prev = NULL;
    md->last_visited_triangle = NULL;
    md->env = env;
    md->is_recursive = recursive;
    if (recursive) {
        md->rec_delegator.dvt = &rec_delegator_vt;
        md->recursive_status = IVP_MR_NORMAL;
        ivp_coll_fvector_init(&md->mindists);
        md->spawned_mindist_count = 0;
    }
    /* ivp_mindist.cxx:115 (every build) */
    ivp_env_get_statistic_manager(env)->sum_of_mindists++;
    ivp_env_get_statistic_manager(env)->mindists_generated++;
    return md;
}

static void rec_delete_all_children(ivp_mindist_t *md)
{
    for (int i = md->mindists.n - 1; i >= 0; i--) {
        if (i >= md->mindists.n) continue;
        ivp_collision_t *c = md->mindists.elems[i];
        ivp_collision_delete(c);
    }
    ivp_coll_fvector_clear(&md->mindists);
}

/* IVP_SurfaceManager::add_reference_to_ledge / remove_reference_to_ledge of
 * a polygon synapse (IVP_Mindist::init_mindist / ~IVP_Mindist). */
static void syn_add_reference_to_ledge(const ivp_md_synapse_t *syn)
{
    if (syn->l_obj->surman) ivp_surman_add_reference_to_ledge(syn->l_obj->surman, ivp_edge_get_compact_ledge(syn->edge));
}

static void syn_remove_reference_to_ledge(const ivp_md_synapse_t *syn)
{
    if (syn->l_obj->surman) ivp_surman_remove_reference_to_ledge(syn->l_obj->surman, ivp_edge_get_compact_ledge(syn->edge));
}

static void mindist_destroy(ivp_collision_t *c)
{
    ivp_mindist_t *md = (ivp_mindist_t *)c;
    if (md->is_recursive) {
        rec_delete_all_children(md);
    }
    /* ~IVP_Mindist */
#ifdef IVP_DEBUG_BUILD
    if (ivp_g_current_mindist == md) {   /* ivp_mindist.cxx:130 */
        (void)ivp_env_check_objects_for_collision_detection(md->env, md->synapse[0].l_obj, md->synapse[1].l_obj);
    }
#endif
    IVP_ASSERT(ivp_g_current_mindist != md);
    /* ivp_mindist.cxx:139 (every build) */
    ivp_env_get_statistic_manager(md->env)->sum_of_mindists--;
    ivp_env_get_statistic_manager(md->env)->mindists_deleted++;
    ivp_mindist_mgr_t *mm = ivp_env_get_mindist_mgr(md->env);
    if (md->is_in_phantom_set) ivp_mindist_mgr_mindist_left_phantom(md);
    switch (md->mindist_status) {
    case IVP_MD_EXACT:          ivp_mindist_mgr_remove_exact_mindist(mm, md); break;
    case IVP_MD_HULL:
    case IVP_MD_HULL_RECURSIVE: ivp_mindist_mgr_remove_hull_mindist(mm, md); break;
    case IVP_MD_INVALID:        ivp_mindist_mgr_remove_invalid_mindist(mm, md); break;
    default: break;
    }
    IVP_ASSERT(md->coll.te.index == IVP_MINLIST_UNUSED);   /* ivp_mindist.cxx:166 */
    if (md->coll.te.index != IVP_MINLIST_UNUSED) {
        /* the reference only asserts (its release build leaves the event
         * queued); libivp also removes it */
        ivp_time_manager_remove_event(ivp_env_get_time_manager(md->env), &md->coll.te);
        md->coll.te.index = IVP_MINLIST_UNUSED;
    }
    /* remove the ledge references of the polygon synapses (init_mindist) */
    for (int i = 0; i < 2; i++) {
        const ivp_md_synapse_t *syn = &md->synapse[i];
        if (syn->l_obj && syn->edge && !obj_is_ball(syn->l_obj)) syn_remove_reference_to_ledge(syn);
    }
    md->coll.delegator->dvt->collision_is_going_to_be_deleted_event(md->coll.delegator, &md->coll);
    free(md);
}

/* ── print (ivp_mindist.cxx:182-268, "#if defined(DEBUG) || 1") ─────────── */

void ivp_md_synapse_print(const ivp_md_synapse_t *s)
{
    const char *stat_text = NULL;
    ivp_message("(syn) ");
    switch (ivp_md_synapse_get_status(s)) {
    case IVP_ST_POINT:    stat_text = "Point"; break;
    case IVP_ST_EDGE:     stat_text = "Edge"; break;
    case IVP_ST_TRIANGLE: stat_text = "Triangle"; break;
    case IVP_ST_BALL:     stat_text = "Ball"; break;
    case IVP_ST_BACKSIDE: stat_text = "Backside"; break;
    default: IVP_CORE(); break;
    }
    ivp_message("%s.\n", stat_text);
}

void ivp_mindist_print(ivp_mindist_t *md, const char *text)
{
    (void)text;
    ivp_message("syn0: ");
    ivp_md_synapse_print(&md->synapse[0]);
    ivp_message("\nsyn1: ");
    ivp_md_synapse_print(&md->synapse[1]);
    ivp_message("\n");
}

void ivp_mindist_mgr_print_mindists(ivp_mindist_mgr_t *mm)
{
    ivp_mindist_t *mdist;
    ivp_debug_printf("\n--------------------------\n");
    for (mdist = mm->exact_mindists; mdist; mdist = mdist->next) {
        ivp_mindist_print(mdist, "exact:");
    }
    for (mdist = mm->invalid_mindists; mdist; mdist = mdist->next) {
        ivp_mindist_print(mdist, "invalid:");
    }
}

/* ── init_mindist ───────────────────────────────────────────────────────── */

void ivp_mindist_init_mindist(ivp_mindist_t *md, struct ivp_object *pop0, struct ivp_object *pop1,
                              const ivp_compact_edge_t *e0, const ivp_compact_edge_t *e1)
{
    ivp_md_synapse_t *syn0 = &md->synapse[0];
    ivp_md_synapse_t *syn1 = &md->synapse[1];
    switch (pop1->type) {
    case IVP_OBJECT_POLYGON:
        syn_init(syn1, md, pop1, e1, IVP_ST_POINT);
        syn_add_reference_to_ledge(syn1);
        break;
    case IVP_OBJECT_BALL:
        if (obj_is_ball(pop0)) {
            if ((uintptr_t)pop0->client_data < (uintptr_t)pop1->client_data) {
                ivp_md_synapse_t *h = syn0; syn0 = syn1; syn1 = h;
            }
        } else {
            ivp_md_synapse_t *h = syn0; syn0 = syn1; syn1 = h;
        }
        syn_init(syn1, md, pop1, e1, IVP_ST_BALL);
        break;
    default:
        IVP_CORE();   /* ivp_mindist.cxx:303 */
        break;
    }
    switch (pop0->type) {
    case IVP_OBJECT_POLYGON:
        syn_init(syn0, md, pop0, e0, IVP_ST_POINT);
        syn_add_reference_to_ledge(syn0);
        break;
    case IVP_OBJECT_BALL:
        syn_init(syn0, md, pop0, e0, IVP_ST_BALL);
        break;
    default:
        IVP_CORE();   /* ivp_mindist.cxx:315 */
        break;
    }
    md->sum_extra_radius = pop0->extra_radius + pop1->extra_radius;
    ivp_mindist_mgr_t *mm = ivp_env_get_mindist_mgr(pop0->owner);
    if (pop0->controller_phantom || pop1->controller_phantom) {
        md->mindist_function = IVP_MF_PHANTOM;
        ivp_mindist_mgr_insert_and_recalc_phantom_mindist(mm, md);
    } else {
        ivp_mindist_mgr_insert_and_recalc_exact_mindist(mm, md);
    }
}

/* ── IVP_Mindist_Manager ────────────────────────────────────────────────── */

void ivp_mindist_mgr_init(ivp_mindist_mgr_t *mm, struct ivp_environment *env)
{
    mm->scanning_universe = false;
    mm->environment = env;
    mm->exact_mindists = NULL;
    mm->invalid_mindists = NULL;
    ivp_pvec_init(&mm->wheel_look_ahead_mindists);
    ivp_pvec_init(&mm->recheck_colliding_elements);
    ivp_pvec_init(&mm->recheck_new_objects);
    mm->recheck_depth = 0;
    mm->recheck_diff_storage = NULL;
}

void ivp_mindist_mgr_destroy(ivp_mindist_mgr_t *mm)
{
    ivp_mindist_t *md, *next;
    for (md = mm->exact_mindists; md; md = next) {
        next = md->next;
        ivp_collision_delete(&md->coll);
    }
    for (md = mm->invalid_mindists; md; md = next) {
        next = md->next;
        ivp_collision_delete(&md->coll);
    }
    ivp_pvec_clear(&mm->wheel_look_ahead_mindists);
    ivp_pvec_clear(&mm->recheck_colliding_elements);
    ivp_pvec_clear(&mm->recheck_new_objects);
    if (mm->recheck_diff_storage)
        ivp_collision_diff_storage_free(mm->recheck_diff_storage, ivp_env_get_allocator(mm->environment));
    mm->recheck_diff_storage = NULL;
}

/* IVP_Diff_Hash replacement: same resulting order (found -> swapped to n_found) */
typedef struct diff_hash {
    ivp_coll_fvector_t *v;
    int n_found_objects;
} diff_hash_t;

static bool diff_hash_check_ledges(diff_hash_t *dh, const ivp_compact_ledge_t *l0, const ivp_compact_ledge_t *l1)
{
    for (int i = dh->n_found_objects; i < dh->v->n; i++) {
        ivp_collision_t *c = dh->v->elems[i];
        const ivp_compact_ledge_t *ls[2];
        c->cvt->get_ledges(c, ls);
        if (ls[0] == l0 && ls[1] == l1) {
            if (i > dh->n_found_objects) ivp_coll_fvector_swap(dh->v, dh->n_found_objects, i);
            dh->n_found_objects++;
            return true;
        }
    }
    return false;
}

void ivp_mindist_mgr_create_exact_mindists(struct ivp_object *pop0, struct ivp_object *pop1, double scan_radius,
                                           ivp_coll_fvector_t *mindists,
                                           const ivp_compact_ledge_t *single_ledge0,
                                           const ivp_compact_ledge_t *single_ledge1,
                                           const ivp_compact_ledge_t *root_ledge0,
                                           const ivp_compact_ledge_t *root_ledge1,
                                           ivp_collision_delegator_t *oo_watcher)
{
    struct ivp_environment *env = pop0->owner;
    ivp_pvec_t ledges0 = IVP_PVEC_INIT, ledges1 = IVP_PVEC_INIT;
    ivp_time_t now = ivp_env_get_current_time(env);
    if (!single_ledge0) {
        ivp_vec3r_t pos1, visitor_os0;
        D dt = (D)ivp_time_diff(now, pop1->physical_core->time_of_last_psi);
        lvp_add_multiple2(&pos1, &pop1->physical_core->pos_world_f_core_last_psi, &pop1->physical_core->delta_world_f_core_psis, dt);
        /* IVP_FLOAT upper_limit_radius + IVP_FLOAT extra radius, then + IVP_DOUBLE */
        D search_radius_0 = (D)((float)pop1->physical_core->upper_limit_radius + pop0->extra_radius) + (D)scan_radius;
        ivp_cache_obj_t *cache0 = ivp_object_get_cache_object_no_lock(pop0);
        lvm_vimult4(&cache0->m_world_f_object, &pos1, &visitor_os0);
        obj_get_all_ledges_within_radius(pop0, &visitor_os0, search_radius_0, root_ledge0, &ledges0);
    } else {
        ivp_pvec_add(&ledges0, (void *)single_ledge0);
    }
    if (!single_ledge1) {
        ivp_vec3r_t pos0, visitor_os1;
        D dt = (D)ivp_time_diff(now, pop0->physical_core->time_of_last_psi);
        lvp_add_multiple2(&pos0, &pop0->physical_core->pos_world_f_core_last_psi, &pop0->physical_core->delta_world_f_core_psis, dt);
        D search_radius_1 = (D)((float)pop0->physical_core->upper_limit_radius + pop1->extra_radius) + (D)scan_radius;
        ivp_cache_obj_t *cache1 = ivp_object_get_cache_object_no_lock(pop1);
        lvm_vimult4(&cache1->m_world_f_object, &pos0, &visitor_os1);
        obj_get_all_ledges_within_radius(pop1, &visitor_os1, search_radius_1, root_ledge1, &ledges1);
    } else {
        ivp_pvec_add(&ledges1, (void *)single_ledge1);
    }
#ifdef IVP_HOME_BUILD
    {
        /* IVP_SUPREME_SUPPORT (ivp_mindist.cxx:439-442):
         *   ivp_global_clustering_visualizer.shortrange.analyze_collision(pop0, search_radius_1,
         *                                                                pop1, search_radius_0);
         * As written the reference hook does not compile (search_radius_0 /
         * search_radius_1 are declared inside the two "if (!single_ledgeX)"
         * blocks above).  The evidently intended values are the radii of the
         * two ledge searches: search_radius_1 is the sphere around pop0 within
         * which pop1's ledges are searched (pop0 is the intruder of pop1) and
         * vice versa.  They are recomputed here with the same formula, also
         * for a side whose search a single_ledge skipped (recursive children,
         * where the visualizer still traverses the whole ledgetree of the
         * private property): the radius only depends on the core radius, the
         * extra radius and scan_radius, it touches no engine state, and the
         * visualizer then shows the same intruder sphere as for a full
         * search.  Passing 0 instead would traverse with a zero radius
         * intruder and skip the intruder sphere (visualize_request_for_intruder_radius
         * is only sent for radius > 0), hiding the pair. */
        D vis_search_radius_0 = (D)((float)pop1->physical_core->upper_limit_radius + pop0->extra_radius) + (D)scan_radius;
        D vis_search_radius_1 = (D)((float)pop0->physical_core->upper_limit_radius + pop1->extra_radius) + (D)scan_radius;
        ivp_clustering_visualizer_shortrange_analyze_collision(&ivp_global_clustering_visualizer.shortrange, pop0,
                                                               (ivp_real_t)vis_search_radius_1, pop1,
                                                               (ivp_real_t)vis_search_radius_0);
    }
#endif
    diff_hash_t dh = { mindists, 0 };
    ivp_pvec_t new_collisions = IVP_PVEC_INIT;
    for (int i = ivp_pvec_len(&ledges0) - 1; i >= 0; i--) {
        const ivp_compact_ledge_t *l0 = (const ivp_compact_ledge_t *)ivp_pvec_at(&ledges0, i);
        const ivp_compact_triangle_t *tri0 = ivp_ledge_get_first_triangle(l0);
        for (int j = ivp_pvec_len(&ledges1) - 1; j >= 0; j--) {
            const ivp_compact_ledge_t *l1 = (const ivp_compact_ledge_t *)ivp_pvec_at(&ledges1, j);
            if (diff_hash_check_ledges(&dh, l0, l1)) {
#ifdef IVP_DEBUG_BUILD
                {   /* IVP_MM_CMP::are_equal (ivp_mindist.cxx:367) */
                    ivp_collision_t *found = dh.v->elems[dh.n_found_objects - 1];
                    ivp_object_t *object[2];
                    found->cvt->get_objects(found, object);
                    IVP_ASSERT(object[0] == pop0);
                    IVP_ASSERT(object[1] == pop1);
                }
#endif
                continue;
            }
            bool rec = !ivp_ledge_is_terminal(l0) || !ivp_ledge_is_terminal(l1);
            ivp_mindist_t *nm = ivp_mindist_create(env, oo_watcher, rec);
            /* ivp_mindist.cxx:486-491: IVP_ASSERT(new_collisions.n_elems <= new_collisions.memsize) */
            IVP_ASSERT(new_collisions.n <= new_collisions.cap);
            const ivp_compact_triangle_t *tri1 = ivp_ledge_get_first_triangle(l1);
            ivp_mindist_init_mindist(nm, pop0, pop1, ivp_tri_get_first_edge(tri0), ivp_tri_get_first_edge(tri1));
            IVP_ASSERT(new_collisions.n <= new_collisions.cap);
            ivp_pvec_add(&new_collisions, nm);
            IVP_ASSERT(new_collisions.n <= new_collisions.cap);
        }
    }
    /* delete unused collisions */
    for (int x = mindists->n - 1; x >= dh.n_found_objects; x--) {
        if (x >= mindists->n) continue;
        ivp_collision_t *c = mindists->elems[x];
        ivp_collision_delete(c);
    }
    /* add new collisions */
    for (int x = ivp_pvec_len(&new_collisions) - 1; x >= 0; x--) {
        ivp_coll_fvector_add(mindists, (ivp_collision_t *)ivp_pvec_at(&new_collisions, x));
    }
    ivp_pvec_clear(&new_collisions);
    ivp_pvec_clear(&ledges0);
    ivp_pvec_clear(&ledges1);
}

void ivp_mindist_mgr_insert_exact_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md)
{
    IVP_ASSERT(md->mindist_status == IVP_MD_UNINITIALIZED);   /* ivp_mindist.cxx:542 */
    md->mindist_status = IVP_MD_EXACT;
    md->next = mm->exact_mindists;
    md->prev = NULL;
    if (md->next) md->next->prev = md;
    mm->exact_mindists = md;
    syn_insert_exact_in_object(&md->synapse[0]);
    syn_insert_exact_in_object(&md->synapse[1]);
    if (md->synapse[0].l_obj->physical_core->car_wheel || md->synapse[1].l_obj->physical_core->car_wheel) {
        if (md->mindist_function != IVP_MF_PHANTOM) ivp_pvec_add(&mm->wheel_look_ahead_mindists, md);
    }
}

void ivp_mindist_mgr_insert_invalid_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md)
{
    IVP_ASSERT(md->mindist_status == IVP_MD_UNINITIALIZED);   /* ivp_mindist.cxx:572 */
    md->mindist_status = IVP_MD_INVALID;
    md->next = mm->invalid_mindists;
    md->prev = NULL;
    if (md->next) md->next->prev = md;
    mm->invalid_mindists = md;
    syn_insert_invalid_in_object(&md->synapse[0]);
    syn_insert_invalid_in_object(&md->synapse[1]);
}

void ivp_mindist_exact_mindist_went_invalid(ivp_mindist_t *md, struct ivp_mindist_mgr *mm)
{
    if (md->is_recursive) {
        /* IVP_Mindist_Recursive::exact_mindist_went_invalid */
        if (rec_get_spawned_mindist_count(&md->rec_delegator) > ivp_mindist_settings.max_spawned_mindist_count) {
            ivp_mindist_mgr_remove_exact_mindist(mm, md);
            ivp_mindist_mgr_insert_invalid_mindist(mm, md);
            return;
        }
        const ivp_compact_ledge_t *l0 = ivp_edge_get_compact_ledge(md->synapse[0].edge);
        const ivp_compact_ledge_t *l1 = ivp_edge_get_compact_ledge(md->synapse[1].edge);
        if (ivp_ledge_is_terminal(l0)) {
            md->recursive_status = IVP_MR_SECOND_SYNAPSE_RECURSIVE;
        } else if (ivp_ledge_is_terminal(l1)) {
            md->recursive_status = IVP_MR_FIRST_SYNAPSE_RECURSIVE;
        } else {
            D rad0 = (D)ledge_get_ledgetree_node(l0)->radius;
            D rad1 = (D)ledge_get_ledgetree_node(l1)->radius;
            md->recursive_status = rad0 > rad1 ? IVP_MR_FIRST_SYNAPSE_RECURSIVE : IVP_MR_SECOND_SYNAPSE_RECURSIVE;
        }
        ivp_mindist_mgr_remove_exact_mindist(mm, md);
        double dist0, dist1;
        ivp_env_get_coll_range_intra_objects(md->env, md->synapse[0].l_obj, md->synapse[1].l_obj, &dist0, &dist1);
        ivp_mindist_mgr_insert_lazy_hull_mindist(md, (float)(dist0 + dist1));
        md->mindist_status = IVP_MD_HULL_RECURSIVE;
        /* recheck_recursive_childs */
        extern void ivp_mindist_recursive_recheck_childs(ivp_mindist_t *md, double dist_intra);
        ivp_mindist_recursive_recheck_childs(md, dist0 + dist1);
        return;
    }
    ivp_mindist_mgr_remove_exact_mindist(mm, md);
    ivp_mindist_mgr_insert_invalid_mindist(mm, md);
}

void ivp_mindist_mgr_insert_and_recalc_exact_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md)
{
    IVP_ASSERT(md->mindist_status == IVP_MD_UNINITIALIZED);   /* ivp_mindist.cxx:597 */
    IVP_ASSERT(md->mindist_function != IVP_MF_PHANTOM);
    md->mindist_status = IVP_MD_EXACT;
    md->next = mm->exact_mindists;
    md->prev = NULL;
    if (md->next) md->next->prev = md;
    ivp_md_synapse_t *syn0 = &md->synapse[0];
    ivp_md_synapse_t *syn1 = &md->synapse[1];
    syn_insert_exact_in_object(syn0);
    syn_insert_exact_in_object(syn1);
    mm->exact_mindists = md;
    ivp_mindist_recalc_mindist(md);
    if (syn0->l_obj->physical_core->car_wheel || syn1->l_obj->physical_core->car_wheel) {
        ivp_pvec_add(&mm->wheel_look_ahead_mindists, md);
    }
    if (md->recalc_result == IVP_MDRR_OK) {
        unsigned allow_hull_bit = (unsigned)syn0->l_obj->physical_core->movement_state | (unsigned)syn1->l_obj->physical_core->movement_state;
        bool allow_hull_conversion = !(allow_hull_bit >= IVP_MT_GET_MINDIST);
        ivp_mindist_update_exact_mindist_events(md, allow_hull_conversion, IVP_EH_NOW);
    } else {
        ivp_mindist_exact_mindist_went_invalid(md, mm);
    }
}

static float phantom_exit_extra(ivp_mindist_t *md)
{
    ivp_object_t *o0 = md->synapse[0].l_obj;
    if (o0->controller_phantom) return ivp_phantom_get_exit_policy_extra_radius(o0->controller_phantom);
    ivp_object_t *o1 = md->synapse[1].l_obj;
    if (o1->controller_phantom) return ivp_phantom_get_exit_policy_extra_radius(o1->controller_phantom);
    return 0.0f;
}

void ivp_mindist_mgr_insert_and_recalc_phantom_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md)
{
    IVP_ASSERT(md->mindist_status == IVP_MD_UNINITIALIZED);   /* ivp_mindist.cxx:652 */
    IVP_ASSERT(md->mindist_function == IVP_MF_PHANTOM);
    ivp_mindist_recalc_invalid_mindist(md);
    float uncertanty;
    if (md->recalc_result == IVP_MDRR_OK) {
        if (md->len_numerator > 0.0f) {
            if (md->is_in_phantom_set) ivp_mindist_mgr_mindist_left_phantom(md);
            ivp_mindist_mgr_insert_exact_mindist(mm, md);
            ivp_mindist_update_exact_mindist_events(md, true, IVP_EH_BIG_DELAY);
            return;
        }
        uncertanty = -md->len_numerator;
    } else {
        uncertanty = md->sum_extra_radius;
    }
    if (!md->is_in_phantom_set) ivp_mindist_mgr_mindist_entered_phantom(md);
    uncertanty += phantom_exit_extra(md);
    ivp_mindist_mgr_insert_lazy_hull_mindist(md, uncertanty);
}

/* remove_exact / remove_invalid / remove_hull_mindist reset mindist_status
 * only in IVP_IF(1) like the reference: its release build keeps the stale
 * status.  Never observable: every removal is followed by an insert (which
 * sets the status) or by the destruction of the mindist; the only status
 * read in between (recalc_invalid_mindist in insert_and_recalc_phantom_mindist
 * after remove_hull_mindist) short-circuits on IVP_MF_PHANTOM. */
void ivp_mindist_mgr_remove_exact_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md)
{
    if (md->coll.te.index != IVP_MINLIST_UNUSED) {
        ivp_time_manager_remove_event(ivp_env_get_time_manager(mm->environment), &md->coll.te);
        md->coll.te.index = IVP_MINLIST_UNUSED;
    }
    IVP_ASSERT(md->mindist_status == IVP_MD_EXACT);
    IVP_IF(1) {   /* ivp_mindist.cxx:1499 */
        md->mindist_status = IVP_MD_UNINITIALIZED;
    }
    if (md->next) md->next->prev = md->prev;
    if (md->prev) md->prev->next = md->next;
    else mm->exact_mindists = md->next;
    syn_remove_exact_from_object(&md->synapse[0]);
    syn_remove_exact_from_object(&md->synapse[1]);
    int i = ivp_pvec_index_of(&mm->wheel_look_ahead_mindists, md);
    if (i >= 0) ivp_pvec_remove_at_allow_resort(&mm->wheel_look_ahead_mindists, i);
}

void ivp_mindist_mgr_remove_invalid_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md)
{
    IVP_ASSERT(md->mindist_status == IVP_MD_INVALID);
    IVP_IF(1) {   /* ivp_mindist.cxx:1532 */
        md->mindist_status = IVP_MD_UNINITIALIZED;
    }
    if (md->next) md->next->prev = md->prev;
    if (md->prev) md->prev->next = md->next;
    else mm->invalid_mindists = md->next;
    syn_remove_invalid_from_object(&md->synapse[0]);
    syn_remove_invalid_from_object(&md->synapse[1]);
}

void ivp_mindist_mgr_remove_hull_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md)
{
    (void)mm;
    IVP_ASSERT(md->mindist_status == IVP_MD_HULL || md->mindist_status == IVP_MD_HULL_RECURSIVE);
    IVP_IF(1) {   /* ivp_mindist.cxx:1557 */
        md->mindist_status = IVP_MD_UNINITIALIZED;
    }
    ivp_md_synapse_t *syn0 = &md->synapse[0];
    ivp_md_synapse_t *syn1 = &md->synapse[1];
    ivp_hull_mgr_remove_synapse(syn0->l_obj->hull_manager, &syn0->lh);
    ivp_hull_mgr_remove_synapse(syn1->l_obj->hull_manager, &syn1->lh);
}

void ivp_mindist_mgr_insert_hull_mindist2(ivp_mindist_t *md, float hull_time0, float hull_time1)
{
    IVP_ASSERT(md->mindist_status == IVP_MD_UNINITIALIZED);   /* ivp_mindist.cxx:1569 */
    md->mindist_status = IVP_MD_HULL;
    float center0 = syn_insert_in_hull_manager(&md->synapse[0], (double)hull_time0);
    float center1 = syn_insert_in_hull_manager(&md->synapse[1], (double)hull_time1);
    md->sum_angular_hull_time = (double)center0 + (double)center1;
}

static void split_hull_time(ivp_mindist_t *md, float hull_time, float eps, float *t0, float *t1)
{
    ivp_object_t *obj0 = md->synapse[0].l_obj;
    ivp_object_t *obj1 = md->synapse[1].l_obj;
    if (!obj_is_moving(obj0)) { *t0 = eps; *t1 = hull_time; return; }
    if (!obj_is_moving(obj1)) { *t0 = hull_time; *t1 = eps; return; }
    float speed0 = obj0->physical_core->max_surface_rot_speed + obj0->physical_core->current_speed + P_FLOAT_EPS;
    float speed1 = obj1->physical_core->max_surface_rot_speed + obj1->physical_core->current_speed + P_FLOAT_EPS;
    float speed_corr_0 = speed0 + 0.1f * speed1;
    float speed_corr_1 = speed1 + 0.1f * speed0;
    float sum_speed = speed_corr_0 + speed_corr_1;
    float factor = hull_time / sum_speed;
    *t0 = factor * speed_corr_0;
    *t1 = factor * speed_corr_1;
}

void ivp_mindist_mgr_insert_hull_mindist(ivp_mindist_t *md, float hull_time)
{
    float t0, t1;
    split_hull_time(md, hull_time, 0.0f, &t0, &t1);
    ivp_mindist_mgr_insert_hull_mindist2(md, t0, t1);
}

void ivp_mindist_mgr_insert_lazy_hull_mindist2(ivp_mindist_t *md, float hull_time0, float hull_time1)
{
    IVP_ASSERT(md->mindist_status == IVP_MD_UNINITIALIZED);   /* ivp_mindist.cxx:1615 */
    md->mindist_status = IVP_MD_HULL;
    syn_insert_lazy_in_hull_manager(&md->synapse[0], (double)hull_time0);
    syn_insert_lazy_in_hull_manager(&md->synapse[1], (double)hull_time1);
}

void ivp_mindist_mgr_insert_lazy_hull_mindist(ivp_mindist_t *md, float hull_time)
{
    float t0, t1;
    split_hull_time(md, hull_time, P_FLOAT_EPS, &t0, &t1);
    ivp_mindist_mgr_insert_lazy_hull_mindist2(md, t0, t1);
    /* ivp_mindist.cxx:1630 / 1638 / 1656: one trace per branch of the split */
    if (!obj_is_moving(md->synapse[0].l_obj)) {
        IVP_IF(ivp_check_debug_mindist(md)) {
            ivp_message("%32s into hull, htimes %f: %f\n", "insert_lazy_hull_mindist", P_DOUBLE_EPS, (double)hull_time);
        }
    } else if (!obj_is_moving(md->synapse[1].l_obj)) {
        IVP_IF(ivp_check_debug_mindist(md)) {
            ivp_message("%32s into hull, htimes %f: %f\n", "insert_lazy_hull_mindist", (double)hull_time, P_DOUBLE_EPS);
        }
    } else {
        IVP_IF(ivp_check_debug_mindist(md)) {
            ivp_message("%32s into hull, htimes %f: %f\n", "insert_lazy_hull_mindist", (double)t0, (double)t1);
        }
    }
}

void ivp_mindist_mgr_mindist_entered_phantom(ivp_mindist_t *md)
{
    if (md->is_in_phantom_set) return;
    md->is_in_phantom_set = true;
    md->disable_halfspace_optimization = true;
    ivp_object_t *o0 = md->synapse[0].l_obj;
    if (o0->controller_phantom) ivp_phantom_mindist_entered_volume(o0->controller_phantom, md);
    ivp_object_t *o1 = md->synapse[1].l_obj;
    if (o1->controller_phantom) ivp_phantom_mindist_entered_volume(o1->controller_phantom, md);
}

void ivp_mindist_mgr_mindist_left_phantom(ivp_mindist_t *md)
{
    if (!md->is_in_phantom_set) return;
    md->disable_halfspace_optimization = false;
    md->is_in_phantom_set = false;
    ivp_object_t *o0 = md->synapse[0].l_obj;
    if (o0->controller_phantom) ivp_phantom_mindist_left_volume(o0->controller_phantom, md);
    ivp_object_t *o1 = md->synapse[1].l_obj;
    if (o1->controller_phantom) ivp_phantom_mindist_left_volume(o1->controller_phantom, md);
}

void ivp_mindist_mgr_recalc_exact_mindist(ivp_mindist_mgr_t *mm, ivp_mindist_t *md)
{
    ivp_mindist_recalc_mindist(md);
    if (md->mindist_function == IVP_MF_COLLISION) {
        if (md->recalc_result == IVP_MDRR_OK) return;
        ivp_mindist_exact_mindist_went_invalid(md, mm);
        return;
    }
    float uncertanty;
    if (md->recalc_result == IVP_MDRR_OK) {
        if (md->len_numerator > 0.0f) {
            if (md->is_in_phantom_set) ivp_mindist_mgr_mindist_left_phantom(md);
            return;
        }
        uncertanty = -md->len_numerator;
    } else {
        uncertanty = md->sum_extra_radius;
    }
    ivp_object_t *o0 = md->synapse[0].l_obj;
    ivp_object_t *o1 = md->synapse[1].l_obj;
    if (o0->controller_phantom && o1->controller_phantom) return;
    uncertanty += phantom_exit_extra(md);
    if (!md->is_in_phantom_set) {
        ivp_mindist_mgr_mindist_entered_phantom(md);
        ivp_mindist_mgr_remove_exact_mindist(mm, md);
        ivp_mindist_mgr_insert_lazy_hull_mindist(md, uncertanty);
    }
}

void ivp_mindist_mgr_recalc_all_exact_mindists(ivp_mindist_mgr_t *mm)
{
    ivp_mindist_t *md, *next;
    for (md = mm->exact_mindists; md; md = next) {
        next = md->next;
        IVP_ASSERT(md->mindist_status == IVP_MD_EXACT);   /* ivp_mindist.cxx:977 */
        ivp_mindist_mgr_recalc_exact_mindist(mm, md);
    }
}

void ivp_mindist_mgr_recalc_all_exact_mindists_events(ivp_mindist_mgr_t *mm)
{
    ivp_mindist_t *md, *next;
    for (md = mm->exact_mindists; md; md = next) {
        next = md->next;
        IVP_ASSERT(md->mindist_status == IVP_MD_EXACT);   /* ivp_mindist.cxx:920 */
        ivp_mindist_update_exact_mindist_events(md, true, IVP_EH_SMALL_DELAY);
    }
}

void ivp_mindist_mgr_recalc_all_exact_wheel_mindist(ivp_mindist_mgr_t *mm)
{
    for (int i = ivp_pvec_len(&mm->wheel_look_ahead_mindists) - 1; i >= 0; i--) {
        if (i >= ivp_pvec_len(&mm->wheel_look_ahead_mindists)) continue;
        ivp_mindist_t *md = (ivp_mindist_t *)ivp_pvec_at(&mm->wheel_look_ahead_mindists, i);
        IVP_ASSERT(md->mindist_function != IVP_MF_PHANTOM);   /* ivp_mindist.cxx:936 */
        ivp_mindist_mgr_recalc_exact_mindist(mm, md);
        ivp_md_synapse_t *syn0 = ivp_mindist_get_sorted_synapse(md, 0);
        ivp_md_synapse_t *syn1 = ivp_mindist_get_sorted_synapse(md, 1);
        if (ivp_md_synapse_get_status(syn0) != IVP_ST_BALL) { ivp_md_synapse_t *t = syn0; syn0 = syn1; syn1 = t; }
        if (syn1->edge == md->last_visited_triangle) continue;
        if (ivp_md_synapse_get_status(syn1) == IVP_ST_TRIANGLE && ivp_md_synapse_get_status(syn0) == IVP_ST_BALL) {
            float md_len = md->len_numerator;
            if (md_len < ivp_mindist_settings.max_distance_for_friction) {
                if (ivp_ledge_is_terminal(ivp_md_synapse_get_ledge(syn1))) {
                    ivp_mindist_create_cp_in_advance_pretension(md, syn0->l_obj, md_len);
                    md->last_visited_triangle = syn1->edge;
                }
            }
        }
    }
}

/* ── update_exact_mindist_events ────────────────────────────────────────── */

void ivp_mindist_update_exact_mindist_events(ivp_mindist_t *md, bool allow_hull_conversion,
                                             ivp_mindist_event_hint_t event_hint)
{
    IVP_IF(1) {   /* ivp_mindist.cxx:990 */
        IVP_ASSERT(md->mindist_status == IVP_MD_EXACT);
        ivp_debug_manager_t *dm = ivp_env_get_debug_manager(md->env);
        if (dm->file_out_impacts) {
            fprintf(dm->out_deb_file, "doing_mindist_events %x at %f\n", (unsigned)((uintptr_t)md & 0x0000ffffU),
                    (double)ivp_env_get_current_time(md->env));
        }
    }
    ivp_md_synapse_t *syn0 = ivp_mindist_get_sorted_synapse(md, 0);
    ivp_md_synapse_t *syn1 = ivp_mindist_get_sorted_synapse(md, 1);
    struct ivp_core *core0 = syn0->l_obj->physical_core;
    struct ivp_core *core1 = syn1->l_obj->physical_core;
    struct ivp_environment *env = md->env;
    ivp_mindist_event_solver_t mim;
    /* IVP_FLOAT + IVP_FLOAT */
    mim.sum_max_surface_rot_speed = (D)(core0->max_surface_rot_speed + core1->max_surface_rot_speed);
    mim.worst_case_speed = mim.sum_max_surface_rot_speed + (D)core0->current_speed + (D)core1->current_speed;
    mim.event_type_out = IVP_COLL_NONE;
    if (md->coll.te.index != IVP_MINLIST_UNUSED) {
        ivp_time_manager_remove_event(ivp_env_get_time_manager(env), &md->coll.te);
        md->coll.te.index = IVP_MINLIST_UNUSED;
    }
    {
        D coll_dist = ivp_mindist_get_coll_dist(md);
        D hull_check_len = coll_dist + (D)ivp_env_get_delta_psi_time(env) * mim.worst_case_speed * 2.1f;
        ivp_mindist_mgr_t *mm = ivp_env_get_mindist_mgr(env);
        if ((D)md->len_numerator > hull_check_len) {
            if (allow_hull_conversion) {
                ivp_mindist_mgr_remove_exact_mindist(mm, md);
                ivp_mindist_mgr_insert_hull_mindist(md, (float)((D)md->len_numerator - coll_dist));
            }
            return;
        }
    }
    {
        /* core->speed is an IVP_U_Float_Point: float dot products */
        ivp_vec3f_t sp1, sp0;
        lvf_set_p(&sp1, &core1->speed);
        lvf_set_p(&sp0, &core0->speed);
        D proj = lvf_dot(&sp1, &md->contact_plane) - lvf_dot(&sp0, &md->contact_plane);
        mim.projected_center_speed = proj;
        D rot0 = lvf_dot(&md->contact_plane, &core0->rotation_axis_world_space);
        rot0 *= rot0;
        rot0 = LV_SQRT(1.001f - rot0) * (D)core0->max_surface_rot_speed;
        D rot1 = lvf_dot(&md->contact_plane, &core1->rotation_axis_world_space);
        rot1 *= rot1;
        rot1 = LV_SQRT(1.001f - rot1) * (D)core1->max_surface_rot_speed;
        mim.max_coll_speed = rot0 + rot1 + mim.projected_center_speed;
        mim.t_now = ivp_env_get_current_time(env);
        mim.t_max = ivp_env_get_next_psi_time(env);
    }
    D len = (D)md->len_numerator;
    if (mim.max_coll_speed < (D)ivp_mindist_settings.speed_after_keeper_dist) {
        if (mim.max_coll_speed < P_DOUBLE_EPS) return;
    }
    D d_time = (D)ivp_time_diff(mim.t_max, mim.t_now);
    D d_len = d_time * mim.max_coll_speed;
    D sec_dist = ivp_mindist_get_coll_dist(md) + d_len;
    IVP_IF(ivp_check_debug_mindist(md)) {   /* ivp_mindist.cxx:1071 */
        ivp_message("%32s  event  len_now %g, sec_dist:%g\n", "update_exact_mindist_events", (double)md->len_numerator,
                    (double)sec_dist);
    }
    if (len >= sec_dist) return;
    if (md->mindist_function == IVP_MF_PHANTOM) return;
    mim.mindist = md;
    mim.environment = env;
    if (md->coll_dist_selector > 0) {
        if (md->coll_dist_decrease_counter++ > 2) {
            md->coll_dist_selector -= 1;
            md->coll_dist_decrease_counter = 0;
        }
    }
    ivp_mindist_event_solver_calc_time_of_next_event(&mim);
    ivp_time_t next_event = mim.event_time_out;
    IVP_IF(ivp_check_debug_mindist(md)) {   /* ivp_mindist.cxx:1108 */
        ivp_message("%32s     calc_time_of_next_event called: ", "''");
        if (mim.event_type_out != IVP_COLL_NONE) ivp_message("event at %f\n", (double)next_event);
        else ivp_message("no\n");
    }
    if (mim.event_type_out != IVP_COLL_NONE) {
        if (ivp_time_diff(next_event, mim.t_now) < (double)P_FLOAT_RES) {
            if (event_hint == IVP_EH_NOW) {
                next_event = mim.t_now;
            } else {
                D mdist = len - (D)ivp_mindist_settings.real_coll_dist;
                D dt = (D)ivp_env_get_delta_psi_time(env);
                D base = (D)ivp_mindist_settings.event_queue_min_delta_time_base;
                ivp_time_t now = ivp_env_get_current_time(env);
                if ((event_hint == IVP_EH_BIG_DELAY) || (mim.event_type_out & 0x0f)) {
                    /* IVP_Time + a + b == (IVP_Time + a) + b */
                    if (mdist >= P_DOUBLE_RES) next_event = (now + (double)(mdist / mim.worst_case_speed)) + (double)((base * 1e-4f) * dt);
                    else next_event = now + (double)((base * 1e-3f) * dt);
                } else {
                    if (mdist >= P_DOUBLE_RES) next_event = (now + (double)(0.1f * mdist / mim.worst_case_speed)) + (double)((base * 1e-7f) * dt);
                    else next_event = now + (double)((base * 1e-5f) * dt);
                }
                if (ivp_time_diff(next_event, ivp_env_get_next_psi_time(env)) >= 0.0) return;
            }
        }
        ivp_time_manager_t *tm = ivp_env_get_time_manager(env);
        float event_time = (float)ivp_time_diff(next_event, tm->base_time);
        md->coll.te.index = ivp_minlist_add(&tm->min_hash, &md->coll.te, event_time);
        IVP_ASSERT(md->coll.te.index != IVP_MINLIST_UNUSED);   /* ivp_mindist.cxx:1171 */
        md->coll_type = (uint8_t)mim.event_type_out;
    }
}

/* ── hull events ────────────────────────────────────────────────────────── */

static void mindist_hull_limit_exceeded_event(ivp_mindist_t *md, float hull_intrusion_value)
{
    if (md->mindist_status == IVP_MD_HULL_RECURSIVE) {
        mindist_recursive_rec_hull_limit_exceeded_event(md);
        return;
    }
    IVP_ASSERT(md->mindist_status == IVP_MD_HULL);   /* ivp_mindist.cxx:1215 */
    ivp_md_synapse_t *syn0 = ivp_mindist_get_sorted_synapse(md, 0);
    ivp_md_synapse_t *syn1 = ivp_mindist_get_sorted_synapse(md, 1);
    ivp_object_t *obj0 = syn0->l_obj;
    ivp_object_t *obj1 = syn1->l_obj;
    ivp_hull_mgr_t *m0 = obj0->hull_manager;
    ivp_hull_mgr_t *m1 = obj1->hull_manager;
    struct ivp_core *core0 = obj0->physical_core;
    struct ivp_core *core1 = obj1->physical_core;
    ivp_time_t current_time = ivp_env_get_current_time(obj0->owner);
    ivp_vec3r_t center0, center1, diff;
    lvp_add_multiple2(&center0, &core0->pos_world_f_core_last_psi, &core0->delta_world_f_core_psis,
                      (D)ivp_time_diff(current_time, core0->time_of_last_psi));
    lvp_add_multiple2(&center1, &core1->pos_world_f_core_last_psi, &core1->delta_world_f_core_psis,
                      (D)ivp_time_diff(current_time, core1->time_of_last_psi));
    lvp_subtract2(&diff, &center0, &center1);
    /* IVP_FLOAT + IVP_FLOAT, then + P_DOUBLE_EPS */
    D speed0 = (D)(core0->current_speed + core0->max_surface_rot_speed) + P_DOUBLE_EPS;
    D speed1 = (D)(core1->current_speed + core1->max_surface_rot_speed) + P_DOUBLE_EPS;
    D speed = speed0 + speed1;
    /* IVP_DOUBLE qdistance = diff.quad_length() (only read by the traces) */
    IVP_IF(ivp_check_debug_mindist(md)) {   /* ivp_mindist.cxx:1255 */
        D qdistance = lvp_quad_length(&diff);
        ivp_message("%32s center_distance %f old_dist %f\n", "hull_limit_exceeded_event", (double)LV_SQRT(qdistance),
                    (double)md->len_numerator);
    }
    if (!md->disable_halfspace_optimization) {
        D new_contact_dot_diff_center = lvp_dot_f(&diff, &md->contact_plane);
        D delta_center_distance = (D)md->contact_dot_diff_center - new_contact_dot_diff_center;
        D a0 = (D)ivp_hull_mgr_get_angular_hull_time(m0, current_time);
        D a1 = (D)ivp_hull_mgr_get_angular_hull_time(m1, current_time);
        D sum_angular = a0 + a1;
        D angular_movement = sum_angular - (D)md->sum_angular_hull_time;
        D estimated_distance = (D)md->len_numerator - angular_movement - delta_center_distance + (D)hull_intrusion_value;
        const D look_ahead_factor = 6.0f;
        IVP_IF(0 && md->contact_plane.k[1] == -1.0f && !md->mindist_function) {   /* ivp_mindist.cxx:1281 */
            ivp_message("sum_angular %f   angular_movement %f    sum_angular_hull_time %f\n", (double)sum_angular,
                        (double)angular_movement, md->sum_angular_hull_time);
        }
        if (estimated_distance > speed * (D)ivp_env_get_delta_psi_time(obj0->owner) * look_ahead_factor) {
            IVP_IF(ivp_check_debug_mindist(md)) {   /* ivp_mindist.cxx:1295 */
                D qdistance = lvp_quad_length(&diff);
                ivp_message("%32s estimated_distance into hull dist: %f error %f speed %f\n", "hull_limit_exceeded_event",
                            (double)LV_SQRT(qdistance), (double)estimated_distance, (double)speed);
            }
            md->sum_angular_hull_time = (double)sum_angular;
            md->contact_dot_diff_center = (float)new_contact_dot_diff_center;
            md->len_numerator = (float)(estimated_distance - (D)hull_intrusion_value);
            D dist_factor = estimated_distance / speed;
            D dist0 = dist_factor * speed0;
            D dist1 = dist_factor * speed1;
            IVP_IF(0 && md->contact_plane.k[1] == -1.0f && !md->mindist_function) {   /* ivp_mindist.cxx:1307 */
                ivp_message("est_dist %f, center_dist %f   sum_angular %f\n", (double)md->len_numerator,
                            (double)delta_center_distance, (double)sum_angular);
            }
            ivp_hull_mgr_update_synapse(syn0->l_obj->hull_manager, &syn0->lh, current_time, (double)dist0);
            ivp_hull_mgr_update_synapse(syn1->l_obj->hull_manager, &syn1->lh, current_time, (double)dist1);
            return;
        }
    }
    /* ivp_mindist.cxx:1325 "if (0) { ... }" (dead small object sphere check,
     * incl. its IVP_IF(ivp_check_debug_mindist(this)) trace at :1338): not
     * ported, dead code in every reference build. */
    ivp_mindist_mgr_t *mgr = ivp_env_get_mindist_mgr(obj0->owner);
    ivp_mindist_mgr_remove_hull_mindist(mgr, md);
    if (md->mindist_function != IVP_MF_PHANTOM) ivp_mindist_mgr_insert_and_recalc_exact_mindist(mgr, md);
    else ivp_mindist_mgr_insert_and_recalc_phantom_mindist(mgr, md);
}

/* ── simulate_time_event ────────────────────────────────────────────────── */

static void mindist_simulate_time_event(ivp_time_event_t *ev, struct ivp_environment *env)
{
    (void)env;
    ivp_env_pcount(env, IVP_PE_AT_INIT);
    ivp_mindist_t *md = (ivp_mindist_t *)ev;
    IVP_ASSERT(md->coll.te.index == IVP_MINLIST_UNUSED);   /* ivp_mindist.cxx:1414 */
    IVP_IF(ivp_check_debug_mindist(md)) {   /* ivp_mindist.cxx:1418 */
        D md_len = (D)md->len_numerator;
        ivp_message("%32s COLL_TYPE %X: old_len=%g\n", "Time_Event_Mindist_Coll", (unsigned)md->coll_type, (double)md_len);
    }
    ivp_mindist_recalc_mindist(md);
    IVP_ASSERT(md->mindist_function != IVP_MF_PHANTOM);   /* ivp_mindist.cxx:1425 */
    IVP_IF(ivp_check_debug_mindist(md)) {   /* ivp_mindist.cxx:1428 */
        D md_len = (D)md->len_numerator;
        ivp_message("%32s COLL_TYPE %X: current_len=%g\n", "Time_Event_Mindist_Coll", (unsigned)md->coll_type,
                    (double)md_len);
    }
    if (md->recalc_result == IVP_MDRR_OK) {
        if ((md->coll_type & 0x0f) == 0) {
            IVP_ASSERT(md->mindist_status == IVP_MD_EXACT);   /* ivp_mindist.cxx:1438 */
            float md_len = md->len_numerator;
            float sec_dist = (float)(ivp_mindist_get_coll_dist(md) + (D)ivp_mindist_settings.mindist_change_force_dist);
            if (md_len < sec_dist) {
                ivp_mindist_do_impact(md);
            } else {
                ivp_mindist_update_exact_mindist_events(md, false, IVP_EH_BIG_DELAY);
            }
        } else {
            ivp_mindist_update_exact_mindist_events(md, false, IVP_EH_SMALL_DELAY);
        }
    }
    ivp_env_pcount(env, IVP_PE_AT_END);
}

/* ── IVP_Mindist_Recursive ──────────────────────────────────────────────── */

static void rec_change_spawned_mindist_count(ivp_collision_delegator_t *d, int change)
{
    ivp_mindist_t *md = rec_from_delegator(d);
    md->spawned_mindist_count += change;
    if (md->coll.delegator->dvt->change_spawned_mindist_count)
        md->coll.delegator->dvt->change_spawned_mindist_count(md->coll.delegator, change);
}

static int rec_get_spawned_mindist_count(ivp_collision_delegator_t *d)
{
    ivp_mindist_t *md = rec_from_delegator(d);
    int parent = md->coll.delegator->dvt->get_spawned_mindist_count
                     ? md->coll.delegator->dvt->get_spawned_mindist_count(md->coll.delegator) : -1;
    if (parent > 0) return parent;
    return md->spawned_mindist_count;
}

static void rec_collision_is_going_to_be_deleted_event(ivp_collision_delegator_t *d, ivp_collision_t *c)
{
    ivp_mindist_t *md = rec_from_delegator(d);
    if (ivp_coll_fvector_index_of(&md->mindists, c) < 0) return;
    ivp_coll_fvector_remove_allow_resort(&md->mindists, c);
    rec_change_spawned_mindist_count(d, -1);
}

void ivp_mindist_recursive_recheck_childs(ivp_mindist_t *md, double dist_intra)
{
    ivp_object_t *obj0 = md->synapse[0].l_obj;
    ivp_object_t *obj1 = md->synapse[1].l_obj;
    if (rec_get_spawned_mindist_count(&md->rec_delegator) > ivp_mindist_settings.max_spawned_mindist_count) return;
    const ivp_compact_ledge_t *sl[2] = { NULL, NULL };
    const ivp_compact_ledge_t *rl[2] = { NULL, NULL };
    int rs = md->recursive_status;
    if (rs < 0 || rs > 1) return;
    sl[1 - rs] = ivp_edge_get_compact_ledge(md->synapse[1 - rs].edge);
    rl[rs] = ivp_edge_get_compact_ledge(md->synapse[rs].edge);
    int l = md->mindists.n;
    ivp_mindist_mgr_create_exact_mindists(obj0, obj1, dist_intra, &md->mindists, sl[0], sl[1], rl[0], rl[1],
                                          &md->rec_delegator);
    l = md->mindists.n - l;
    rec_change_spawned_mindist_count(&md->rec_delegator, l);
}

static void rec_invalid_mindist_went_exact(ivp_mindist_t *md)
{
    rec_delete_all_children(md);
    ivp_mindist_mgr_t *mm = ivp_env_get_mindist_mgr(md->env);
    ivp_mindist_mgr_remove_hull_mindist(mm, md);
    ivp_mindist_mgr_insert_exact_mindist(mm, md);
    md->mindist_function = IVP_MF_COLLISION;
}

static void mindist_recursive_rec_hull_limit_exceeded_event(ivp_mindist_t *md)
{
    ivp_mindist_recalc_invalid_mindist(md);
    if (md->recalc_result == IVP_MDRR_OK) {
        if (md->len_numerator > ivp_mindist_settings.friction_dist) {
            rec_invalid_mindist_went_exact(md);
            return;
        }
    }
    ivp_object_t *obj0 = md->synapse[0].l_obj;
    ivp_object_t *obj1 = md->synapse[1].l_obj;
    double dist0, dist1;
    ivp_env_get_coll_range_intra_objects(md->env, obj0, obj1, &dist0, &dist1);
    ivp_env_get_statistic_manager(md->env)->range_intra_exceeded++;   /* ivp_mindist_recursive.cxx:281 (every build) */
    ivp_mindist_recursive_recheck_childs(md, dist0 + dist1);
    ivp_time_t t_now = ivp_env_get_current_time(md->env);
    ivp_hull_mgr_update_synapse(obj0->hull_manager, &md->synapse[0].lh, t_now, dist0);
    ivp_hull_mgr_update_synapse(obj1->hull_manager, &md->synapse[1].lh, t_now, dist1);
}

/* IVP_Mindist_Recursive::do_impact; returns true if handled recursively */
bool ivp_mindist_recursive_do_impact(ivp_mindist_t *md)
{
    ivp_md_synapse_t *syn0 = &md->synapse[0];
    ivp_md_synapse_t *syn1 = &md->synapse[1];
    if (rec_get_spawned_mindist_count(&md->rec_delegator) > ivp_mindist_settings.max_spawned_mindist_count) return false;
    IVP_ASSERT(md->mindist_status == IVP_MD_EXACT);   /* ivp_mindist_recursive.cxx:34 */
    int s0 = (int)ivp_md_synapse_get_status(syn0), s1 = (int)ivp_md_synapse_get_status(syn1);
    switch (s0) {
    case IVP_ST_BALL:
    case IVP_ST_POINT:
        switch (s1) {
        case IVP_ST_BALL:
        case IVP_ST_POINT:
            return false;
        case IVP_ST_EDGE:
            if (!ivp_edge_get_is_virtual(syn1->edge)) return false;
            md->recursive_status = IVP_MR_SECOND_SYNAPSE_RECURSIVE;
            break;
        case IVP_ST_TRIANGLE:
            if (!ivp_tri_get_is_virtual(ivp_edge_get_triangle(syn1->edge))) return false;
            md->recursive_status = IVP_MR_SECOND_SYNAPSE_RECURSIVE;
            break;
        default:
            IVP_CORE();   /* ivp_mindist_recursive.cxx:56 (the reference then continues with the recursive split) */
            return false;
        }
        break;
    case IVP_ST_EDGE:
        switch (s1) {
        case IVP_ST_BALL:
        case IVP_ST_POINT:
            if (!ivp_tri_get_is_virtual(ivp_edge_get_triangle(syn0->edge))) return false;
            md->recursive_status = IVP_MR_FIRST_SYNAPSE_RECURSIVE;
            break;
        case IVP_ST_EDGE:
            if (!ivp_edge_get_is_virtual(syn1->edge)) {
                if (!ivp_edge_get_is_virtual(syn0->edge)) return false;
                md->recursive_status = IVP_MR_FIRST_SYNAPSE_RECURSIVE;
                break;
            }
            if (!ivp_edge_get_is_virtual(syn0->edge)) {
                md->recursive_status = IVP_MR_SECOND_SYNAPSE_RECURSIVE;
                break;
            }
            {
                const ivp_compact_ledge_t *l0 = ivp_edge_get_compact_ledge(syn0->edge);
                const ivp_compact_ledge_t *l1 = ivp_edge_get_compact_ledge(syn1->edge);
                D rad0 = (D)ledge_get_ledgetree_node(l0)->radius;
                D rad1 = (D)ledge_get_ledgetree_node(l1)->radius;
                md->recursive_status = rad0 > rad1 ? IVP_MR_FIRST_SYNAPSE_RECURSIVE : IVP_MR_SECOND_SYNAPSE_RECURSIVE;
            }
            break;
        default:
            IVP_CORE();   /* ivp_mindist_recursive.cxx:108 */
            return false;
        }
        break;
    case IVP_ST_TRIANGLE:
        if (!ivp_tri_get_is_virtual(ivp_edge_get_triangle(syn0->edge))) return false;
        md->recursive_status = IVP_MR_FIRST_SYNAPSE_RECURSIVE;
        break;
    default:
        IVP_CORE();   /* ivp_mindist_recursive.cxx:117 */
        return false;
    }
    ivp_mindist_mgr_t *mm = ivp_env_get_mindist_mgr(md->env);
    ivp_mindist_mgr_remove_exact_mindist(mm, md);
    double dist0, dist1;
    ivp_env_get_coll_range_intra_objects(md->env, syn0->l_obj, syn1->l_obj, &dist0, &dist1);
    ivp_mindist_mgr_insert_lazy_hull_mindist(md, (float)(dist0 + dist1));
    md->mindist_status = IVP_MD_HULL_RECURSIVE;
    ivp_mindist_recursive_recheck_childs(md, dist0 + dist1);
    return true;
}

/* ── IVP_OO_Watcher ─────────────────────────────────────────────────────── */

static void oo_hull_limit_exceeded(ivp_oo_watcher_t *w);

static void synapse_oo_limit_exceeded(ivp_listener_hull_t *lh, ivp_hull_mgr_t *hm, float v)
{
    (void)hm; (void)v;
    oo_hull_limit_exceeded(((ivp_synapse_oo_t *)lh)->watcher);
}

static void oo_watcher_destroy(ivp_collision_t *c);

/* IVP_OO_Watcher::simulate_time_event (ivp_mindist.hxx:225): never queued */
static void oo_watcher_simulate_time_event(ivp_time_event_t *ev, struct ivp_environment *env)
{
    (void)ev;
    (void)env;
    IVP_CORE();
}

static void synapse_oo_manager_deleted(ivp_listener_hull_t *lh, ivp_hull_mgr_t *hm)
{
    (void)hm;
    oo_watcher_destroy(&((ivp_synapse_oo_t *)lh)->watcher->coll);
}

static const ivp_listener_hull_vt_t synapse_oo_vt = {
    IVP_HULL_ELEM_OO_WATCHER,
    synapse_oo_limit_exceeded,
    synapse_oo_manager_deleted,
    NULL
};

static void oo_get_objects(ivp_collision_t *c, struct ivp_object *out[2])
{
    ivp_oo_watcher_t *w = (ivp_oo_watcher_t *)c;
    out[0] = w->synapses[0].object;
    out[1] = w->synapses[1].object;
}

static void oo_get_ledges(ivp_collision_t *c, const ivp_compact_ledge_t *out[2])
{
    (void)c;
    out[0] = NULL;
    out[1] = NULL;
}

static const ivp_collision_vt_t oo_collision_vt = {
    oo_get_objects,
    oo_get_ledges,
    collision_default_delegator_deleted,
    oo_watcher_destroy
};

static void oo_del_collision_is_going_to_be_deleted_event(ivp_collision_delegator_t *d, ivp_collision_t *c)
{
    ivp_oo_watcher_t *w = (ivp_oo_watcher_t *)((char *)d - offsetof(ivp_oo_watcher_t, del));
    ivp_coll_fvector_remove_allow_resort(&w->mindists, c);
}

static const ivp_collision_delegator_vt_t oo_delegator_vt = {
    oo_del_collision_is_going_to_be_deleted_event,
    NULL,
    NULL
};

static void oo_hull_limit_exceeded(ivp_oo_watcher_t *w)
{
    ivp_object_t *obj0 = w->synapses[0].object;
    ivp_object_t *obj1 = w->synapses[1].object;
    double dist0, dist1;
    ivp_env_get_statistic_manager(w->env)->range_intra_exceeded++;   /* ivp_oo_watcher.cxx:23 (every build) */
    ivp_env_get_coll_range_intra_objects(w->env, obj0, obj1, &dist0, &dist1);
    ivp_mindist_mgr_create_exact_mindists(obj0, obj1, dist0 + dist1, &w->mindists, NULL, NULL, NULL, NULL, &w->del);
    ivp_time_t t_now = ivp_env_get_current_time(w->env);
    ivp_hull_mgr_update_synapse(obj0->hull_manager, &w->synapses[0].lh, t_now, dist0);
    ivp_hull_mgr_update_synapse(obj1->hull_manager, &w->synapses[1].lh, t_now, dist1);
}

ivp_oo_watcher_t *ivp_oo_watcher_create(ivp_collision_delegator_t *del, struct ivp_object *obj0,
                                        struct ivp_object *obj1)
{
    ivp_oo_watcher_t *w = (ivp_oo_watcher_t *)calloc(1, sizeof(*w));
    IVP_ASSERT(w);
    if (!w) return NULL;
    w->coll.te.index = IVP_MINLIST_UNUSED;
    w->coll.te.simulate_time_event = oo_watcher_simulate_time_event;
    w->coll.cvt = &oo_collision_vt;
    w->coll.delegator = del;
    w->coll.fvector_index[0] = w->coll.fvector_index[1] = -1;
    w->del.dvt = &oo_delegator_vt;
    w->env = obj0->owner;
    ivp_coll_fvector_init(&w->mindists);
    struct ivp_object *objs[2] = { obj0, obj1 };
    for (int i = 0; i < 2; i++) {
        ivp_synapse_oo_t *s = &w->synapses[i];
        ivp_listener_hull_init(&s->lh, &synapse_oo_vt);
        s->object = objs[i];
        ivp_hull_mgr_insert_synapse(objs[i]->hull_manager, &s->lh, ivp_env_get_current_time(w->env), P_FLOAT_MAX);
        s->watcher = w;
    }
    oo_hull_limit_exceeded(w);
    return w;
}

static void oo_watcher_destroy(ivp_collision_t *c)
{
    ivp_oo_watcher_t *w = (ivp_oo_watcher_t *)c;
    for (int i = w->mindists.n - 1; i >= 0; i--) {
        if (i >= w->mindists.n) continue;
        ivp_collision_delete(w->mindists.elems[i]);
    }
    ivp_coll_fvector_clear(&w->mindists);
    w->coll.delegator->dvt->collision_is_going_to_be_deleted_event(w->coll.delegator, &w->coll);
    /* ~IVP_Synapse_OO */
    for (int i = 0; i < 2; i++) {
        ivp_hull_mgr_remove_synapse(w->synapses[i].object->hull_manager, &w->synapses[i].lh);
    }
    free(w);
}

/* ── IVP_Collision_Delegator_Root_Mindist ───────────────────────────────── */

static void root_object_is_removed(ivp_collision_delegator_root_t *r, struct ivp_object *o)
{
    ivp_ov_element_t *ov = o->ov_element;
    if (!ov) return;
    for (int i = ov->collision_fvector.n - 1; i >= 0; i--) {
        if (i >= ov->collision_fvector.n) continue;
        ivp_collision_t *coll = ov->collision_fvector.elems[i];
        coll->cvt->delegator_is_going_to_be_deleted_event(coll, &r->base);
    }
}

static void root_env_deleted(ivp_collision_delegator_root_t *r, struct ivp_environment *env)
{
    ivp_free(ivp_env_get_allocator(env), r, sizeof(*r));
}

static void root_collision_is_going_to_be_deleted_event(ivp_collision_delegator_t *d, ivp_collision_t *t)
{
    (void)d;
    ivp_object_t *objs[2];
    t->cvt->get_objects(t, objs);
    if (objs[0]->ov_element) ivp_ov_element_remove_oo_collision(objs[0]->ov_element, t);
    if (objs[1]->ov_element) ivp_ov_element_remove_oo_collision(objs[1]->ov_element, t);
}

static ivp_collision_t *root_delegate_collisions_for_object(ivp_collision_delegator_root_t *r,
                                                            struct ivp_object *obj0, struct ivp_object *obj1)
{
    ivp_collision_t *coll;
    const ivp_compact_ledge_t *l0 = obj_get_single_convex(obj0);
    const ivp_compact_ledge_t *l1 = obj_get_single_convex(obj1);
    if (l0 && l1) {
        const ivp_compact_triangle_t *tri0 = ivp_ledge_get_first_triangle(l0);
        const ivp_compact_triangle_t *tri1 = ivp_ledge_get_first_triangle(l1);
        bool rec = !ivp_ledge_is_terminal(l0) || !ivp_ledge_is_terminal(l1);
        ivp_mindist_t *md = ivp_mindist_create(obj0->owner, &r->base, rec);
        /* reference: ov elements get the collision AFTER init_mindist */
        ivp_mindist_init_mindist(md, obj0, obj1, ivp_tri_get_first_edge(tri0), ivp_tri_get_first_edge(tri1));
        coll = &md->coll;
    } else {
        coll = &ivp_oo_watcher_create(&r->base, obj0, obj1)->coll;
    }
    ivp_ov_element_add_oo_collision(obj0->ov_element, coll);
    ivp_ov_element_add_oo_collision(obj1->ov_element, coll);
    return coll;
}

static const ivp_collision_delegator_vt_t root_delegator_vt = {
    root_collision_is_going_to_be_deleted_event,
    NULL,
    NULL
};

static const ivp_collision_delegator_root_vt_t root_vt = {
    root_object_is_removed,
    root_delegate_collisions_for_object,
    root_env_deleted
};

ivp_collision_delegator_root_t *ivp_collision_delegator_root_mindist_create(struct ivp_environment *env)
{
    if (!env) return NULL;
    const ivp_allocator_t *allocator = ivp_env_get_allocator(env);
    ivp_collision_delegator_root_t *r =
        (ivp_collision_delegator_root_t *)ivp_alloc(allocator, sizeof(*r));
    if (!r) return NULL;
    memset(r, 0, sizeof(*r));
    r->base.dvt = &root_delegator_vt;
    r->rvt = &root_vt;
    return r;
}

/* ── recheck_ov_element / enable_collision_detection ────────────────────── */

/* Only the two built-in, pure getters are stable during the partition. User
 * collision implementations keep their original callback sequence. */
static ivp_object_t *recheck_collision_key(const ivp_collision_t *c, const ivp_object_t *owner)
{
    ivp_object_t *a, *b;
    if (c->cvt == &mindist_collision_vt) {
        const ivp_mindist_t *md = (const ivp_mindist_t *)c;
        a = md->synapse[0].l_obj; b = md->synapse[1].l_obj;
    } else if (c->cvt == &oo_collision_vt) {
        const ivp_oo_watcher_t *w = (const ivp_oo_watcher_t *)c;
        a = w->synapses[0].object; b = w->synapses[1].object;
    } else return NULL;
    return a == owner ? b : b == owner ? a : NULL;
}

void ivp_mindist_mgr_recheck_ov_element(ivp_mindist_mgr_t *mm, struct ivp_object *object)
{
    ivp_ov_element_t *elem = object->ov_element;
    if (!elem) return;
    const bool top_level = mm->recheck_depth++ == 0;
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    recheck_diagnostics.rechecks++;
    if (!top_level) recheck_diagnostics.recursive_rechecks++;
    if (mm->recheck_depth > recheck_diagnostics.max_depth)
        recheck_diagnostics.max_depth = mm->recheck_depth;
#endif
    ivp_pvec_t local_colliding_elements = IVP_PVEC_INIT;
    ivp_pvec_t local_new_objects = IVP_PVEC_INIT;
    ivp_collision_diff_storage_t *local_diff_storage = NULL;
    ivp_pvec_t *colliding_elements = top_level ? &mm->recheck_colliding_elements
                                                : &local_colliding_elements;
    ivp_pvec_t *new_objects = top_level ? &mm->recheck_new_objects : &local_new_objects;
    ivp_pvec_remove_all(colliding_elements);
    ivp_pvec_remove_all(new_objects);
    struct ivp_environment *env = mm->environment;
    struct ivp_core *core = object->physical_core;
    ivp_ov_tree_t *ovt = ivp_env_get_ov_tree(env);
    ivp_ov_tree_remove_ov_element(ovt, elem);
    ivp_env_get_statistic_manager(env)->range_world_exceeded++;   /* ivp_mindist.cxx:787 (every build) */
    lvf_set_p(&elem->center, &core->m_world_f_core_last_psi.pos);   /* core->get_position_PSI() */
    ivp_hull_mgr_t *hm = object->hull_manager;
    if (!mm->scanning_universe) {
        double hull_time = ivp_env_get_coll_range_in_world(env, object);
        double real_check_sphere = hull_time + (double)core->upper_limit_radius;
        if (ivp_env_has_universe_manager(env)) {
            if (IVP_MTIS_CAN_COLLIDE(object->object_movement_state)) {
                mm->scanning_universe = true;
                ivp_env_universe_ensure_objects(env, object, elem->center.k, real_check_sphere);
                mm->scanning_universe = false;
            }
        }
        ivp_pvec_t *saved_collision_partners = ovt->collision_partners;
        double radius = ivp_ov_tree_insert_ov_element(ovt, elem, real_check_sphere, real_check_sphere,
                                                      colliding_elements);
        ovt->collision_partners = saved_collision_partners;
        double real_hull_time = radius - (double)core->upper_limit_radius;
        ivp_ov_element_add_to_hull_manager(elem, hm, real_hull_time);
    } else {
        double real_check_sphere = (double)core->upper_limit_radius;
        ivp_pvec_t *saved_collision_partners = ovt->collision_partners;
        ivp_ov_tree_insert_ov_element(ovt, elem, real_check_sphere, real_check_sphere, NULL);
        ovt->collision_partners = saved_collision_partners;
        ivp_ov_element_add_to_hull_manager(elem, hm, (double)P_DOUBLE_EPS);
        goto cleanup;
    }
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    uint64_t candidates = (uint64_t)ivp_pvec_len(colliding_elements);
    uint64_t old_collisions = (uint64_t)elem->collision_fvector.n;
    recheck_diagnostics.candidates += candidates;
    recheck_diagnostics.old_collisions += old_collisions;
    recheck_diagnostics.potential_search_pairs += candidates * old_collisions;
    if (candidates > recheck_diagnostics.max_candidates)
        recheck_diagnostics.max_candidates = candidates;
    if (old_collisions > recheck_diagnostics.max_old_collisions)
        recheck_diagnostics.max_old_collisions = old_collisions;
#endif
#ifdef IVP_DEBUG_BUILD
    /* IVP_Diff_Hash constructor: IVP_OO_CMP::calc_hash_index(c, object) of
     * every old collision (ivp_mindist.cxx:720) */
    for (int i = elem->collision_fvector.n - 1; i >= 0; i--) {
        ivp_collision_t *c = elem->collision_fvector.elems[i];
        ivp_object_t *objects[2];
        c->cvt->get_objects(c, objects);
        IVP_ASSERT(objects[0] == object || objects[1] == object);
    }
#endif
    ivp_collision_diff_t dh;
    /* User filters can mutate or recursively recheck this very vector. Only
     * allow-all filtering has a callback-free partition interval. */
    ivp_collision_diff_begin(&dh, &elem->collision_fvector, object,
                             ivp_pvec_len(colliding_elements),
                             top_level ? &mm->recheck_diff_storage : &local_diff_storage,
                             ivp_env_get_allocator(env),
                             env->collision_filter.type == IVP_CFILTER_ALLOW_ALL
                                 ? recheck_collision_key : NULL);
    for (int i = ivp_pvec_len(colliding_elements) - 1; i >= 0; i--) {
        ivp_ov_element_t *el = (ivp_ov_element_t *)ivp_pvec_at(colliding_elements, i);
        ivp_object_t *obj0 = object;
        ivp_object_t *obj1 = el->real_object;
        if (!IVP_MTIS_CAN_COLLIDE(obj0->object_movement_state) && !IVP_MTIS_CAN_COLLIDE(obj1->object_movement_state))
            continue;
        if (obj0->friction_core == obj1->friction_core) continue;
        struct ivp_core *c0 = obj0->physical_core, *c1 = obj1->physical_core;
        if ((c0->pinned && c1->pinned) || (c0->pinned && c1->physical_unmoveable) ||
            (c0->physical_unmoveable && c1->pinned))
            continue;
        if (!ivp_env_check_objects_for_collision_detection(env, obj0, obj1)) continue;
        if (ivp_collision_diff_check(&dh, obj1)) continue;
        ivp_pvec_add(new_objects, obj1);
    }
    /* delete unused collisions */
    for (int x = elem->collision_fvector.n - 1; x >= dh.found; x--) {
        if (x >= elem->collision_fvector.n) continue;
        ivp_collision_t *c = elem->collision_fvector.elems[x];
        /* reference HACKHACK (E3 2003): never delete the mindist that is calling us up the stack */
        if ((void *)c != (void *)ivp_g_current_mindist) {
            ivp_collision_delete(c);
        } else {
            ivp_g_defer_delete_mindist = true;
        }
    }
    /* create new ones */
    for (int x = ivp_pvec_len(new_objects) - 1; x >= 0; x--) {
        ivp_object_t *obj1 = (ivp_object_t *)ivp_pvec_at(new_objects, x);
        for (int k = ivp_env_get_collision_delegator_root_count(env) - 1; k >= 0; k--) {
            ivp_collision_delegator_root_t *cdr = ivp_env_get_collision_delegator_root(env, k);
            ivp_collision_t *coll = cdr->rvt->delegate_collisions_for_object(cdr, object, obj1);
            if (coll) break;
        }
    }
cleanup:
    if (top_level) {
        ivp_pvec_remove_all(&mm->recheck_new_objects);
        ivp_pvec_remove_all(&mm->recheck_colliding_elements);
    } else {
        ivp_collision_diff_storage_free(local_diff_storage, ivp_env_get_allocator(env));
        ivp_pvec_clear(&local_new_objects);
        ivp_pvec_clear(&local_colliding_elements);
    }
    IVP_ASSERT(mm->recheck_depth > 0);
    mm->recheck_depth--;
}

void ivp_mindist_mgr_enable_collision_detection_for_object(ivp_mindist_mgr_t *mm, struct ivp_object *object)
{
    if (object->ov_element) {
        ivp_ov_element_destroy(object->ov_element);
        object->ov_element = NULL;
    }
    object->ov_element = ivp_ov_element_create(object);
    ivp_mindist_mgr_recheck_ov_element(mm, object);
}
