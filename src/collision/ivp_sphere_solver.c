/* ivp_sphere_solver.c -- Sphere overlap queries against compact geometry
 *
 * Traverses compact ledgetree BVH checking sphere-sphere overlap at each
 * node, and the environment query (IVP_Sphere_Solver), which walks the
 * OV tree like recursively_collect_intruding_objects.
 *
 * Ported from ivp_sphere_solver.cxx.
 */
#include <ivp/collision/ivp_sphere_solver.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_cache_object.h>
#include <ivp/base/ivp_assert.h>
#include "dynamics/ivp_env_internal.h"

#include <math.h>

/* ── Recursive BVH traversal ───────────────────────────────────────────── */

static bool sphere_vs_node(const ivp_compact_ledgetree_node_t *node,
                            const ivp_vec3r_t *center_os,
                            ivp_real_t radius,
                            int depth,
                            int max_depth)
{
    /* Check sphere-sphere overlap: query sphere vs node bounding sphere. */
    ivp_vec3r_t node_center = ivp_vec3r(
        (ivp_real_t)node->center[0],
        (ivp_real_t)node->center[1],
        (ivp_real_t)node->center[2]);

    ivp_real_t sum_r = radius + (ivp_real_t)node->radius;
    ivp_real_t qdist = ivp_vec3r_quad_distance(center_os, &node_center);

    if (qdist > sum_r * sum_r) {
        return false; /* no overlap with this subtree */
    }

    /* Overlap detected. If terminal or at max depth, report hit. */
    if (ivp_ledgetree_is_terminal(node) || depth >= max_depth) {
        return true;
    }

    /* Recurse into children. */
    if (sphere_vs_node(ivp_ledgetree_left_son(node),
                        center_os, radius, depth + 1, max_depth)) {
        return true;
    }
    return sphere_vs_node(ivp_ledgetree_right_son(node),
                           center_os, radius, depth + 1, max_depth);
}

/* ── Public API ────────────────────────────────────────────────────────── */

bool ivp_sphere_overlaps_ledgetree(
    const ivp_compact_ledgetree_node_t *root,
    const ivp_vec3r_t *center_os,
    ivp_real_t radius,
    int max_depth)
{
    IVP_ASSERT(root != NULL);
    IVP_ASSERT(center_os != NULL);
    IVP_ASSERT(radius >= IVP_REAL(0.0));

    if (max_depth < 0) max_depth = 0;

    return sphere_vs_node(root, center_os, radius, 0, max_depth);
}

bool ivp_sphere_overlaps_surface(
    const ivp_compact_surface_t *surface,
    const ivp_vec3r_t *center_os,
    ivp_real_t radius,
    int max_depth)
{
    IVP_ASSERT(surface != NULL);
    IVP_ASSERT(center_os != NULL);

    const ivp_compact_ledgetree_node_t *root =
        ivp_surface_get_ledgetree_root(surface);

    return ivp_sphere_overlaps_ledgetree(root, center_os, radius, max_depth);
}

/* ── Environment / object queries ──────────────────────────────────────── */

/* sphere_overlaps_sphere_d (ivp_sphere_solver.cxx:33) */
static bool overlaps_d(const double p1[3], double r1, const double p2[3], double r2)
{
    float d[3];
    for (int i = 0; i < 3; i++) d[i] = (float)(p1[i] - p2[i]);
    float qd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    return !((double)qd > (r1 + r2) * (r1 + r2));
}

static bool node_rec(const ivp_compact_ledgetree_node_t *cln, const double q[3], double r,
                     int depth, int max_depth)
{
    /* sphere_against_ledgetree_node (ivp_sphere_solver.cxx:48) */
    double c[3] = { (double)cln->center[0], (double)cln->center[1], (double)cln->center[2] };
    if (!overlaps_d(q, r, c, (double)cln->radius)) return false;
    if (depth < max_depth && !ivp_ledgetree_is_terminal(cln)) {
        if (node_rec(ivp_ledgetree_left_son(cln), q, r, depth + 1, max_depth)) return true;
        return node_rec(ivp_ledgetree_right_son(cln), q, r, depth + 1, max_depth);
    }
    return true;
}

static void to_object_space(const ivp_object_t *obj, const ivp_vec3r_t *p_ws, double out[3])
{
    ivp_transform_r_t m;
    ivp_time_t t = obj->owner ? ivp_environment_get_time(obj->owner) : obj->physical_core->time_of_last_psi;
    ivp_object_get_at_matrix(obj, t, &m);
    double d[3];
    for (int i = 0; i < 3; i++) d[i] = (double)p_ws->k[i] - (double)m.pos.k[i];
    for (int c = 0; c < 3; c++) {
        double s = 0.0;
        for (int r = 0; r < 3; r++) s += (double)m.rot.rows[r].k[c] * d[r];
        out[c] = s;
    }
}

/* Ledge tree root used by sphere_against_object.  The vtable's
 * get_ledgetree_root only exposes the root of compound (non-terminal)
 * surfaces, while the reference also tests the terminal root sphere of a
 * single convex polygon, so the compact surface of a genuine polygon
 * surface manager is read directly.  The manager is identified by its
 * vtable (the one ivp_surman_polygon_init installs), never by
 * get_type(): other managers (e.g. the grid managers) report
 * IVP_SURMAN_POLYGON as well but are different structs. */
static const ivp_compact_ledgetree_node_t *polygon_ledgetree_root(const ivp_surman_t *sm)
{
    ivp_surman_polygon_t probe;
    ivp_surman_polygon_init(&probe, NULL);
    if (sm->vtable == probe.base.vtable) {
        const ivp_compact_surface_t *cs = ((const ivp_surman_polygon_t *)sm)->compact_surface;
        return cs ? ivp_surface_get_ledgetree_root(cs) : NULL;
    }
    return ivp_surman_get_ledgetree_root(sm);
}

static void any_ledge_cb(void *ctx, const ivp_compact_ledge_t *ledge)
{
    (void)ledge;
    *(bool *)ctx = true;
}

static bool sphere_against_object_os(const ivp_object_t *obj, const double q_os[3],
                                     double r, int max_depth)
{
    /* sphere_against_object (ivp_sphere_solver.cxx:74) */
    if (obj->type == IVP_OBJECT_POLYGON) {
        const ivp_surman_t *sm = obj->surman;
        if (!sm) return false;
        const ivp_compact_ledgetree_node_t *root = polygon_ledgetree_root(sm);
        if (root) return node_rec(root, q_os, r, 0, max_depth);
        /* other managers (grids): intruding when the manager reports a
         * ledge within the query radius (reference 6623a1d) */
        bool any = false;
        ivp_surman_get_all_ledges_within_radius(sm, q_os, r, NULL, NULL, any_ledge_cb, &any);
        return any;
    }
    if (obj->type == IVP_OBJECT_BALL) {
        /* Reference quirk kept for parity: the object space query center is
         * compared with the ball's WORLD core position (get_position_PSI). */
        const ivp_vec3r_t *wp = &obj->physical_core->m_world_f_core_last_psi.pos;   /* get_position_PSI() */
        double p[3] = { (double)wp->k[0], (double)wp->k[1], (double)wp->k[2] };
        return overlaps_d(q_os, r, p, (double)(float)obj->physical_core->upper_limit_radius);
    }
    return false;
}

bool ivp_sphere_solver_check_object(const ivp_sphere_solver_template_t *tmpl,
                                    const struct ivp_object *object)
{
    if (!tmpl || !object) return false;
    double q[3];
    to_object_space(object, &tmpl->center, q);
    return sphere_against_object_os(object, q, (double)tmpl->radius, tmpl->max_traversal_depth);
}

/* AABB_overlaps_AABB_d (ivp_sphere_solver.cxx:14): IVP_U_Float_Point
 * half extents (rlb) and centers (pos) */
static bool aabb_overlaps_aabb(const float rlb1[3], const float pos1[3],
                               const float rlb2[3], const float pos2[3])
{
    for (int i = 0; i < 3; i++) {
        /* fabsd(float - float) > (float + float) */
        if (fabs((double)(pos1[i] - pos2[i])) > (double)(rlb1[i] + rlb2[i])) return false;
    }
    return true;
}

typedef struct sphere_env_query {
    const ivp_ov_tree_t                *tree;          /* m_tree_manager */
    const ivp_sphere_solver_template_t *tmpl;
    double                              center[3];     /* m_center (IVP_U_Point) */
    double                              radius;        /* m_radius */
    int                                 max_depth;     /* m_max_traversal_depth */
    struct ivp_object                 **out;
    int                                 capacity;
    int                                 found;
} sphere_env_query_t;

static void collect_intruding_objects(sphere_env_query_t *q, const ivp_ov_node_t *node,
                                      const float query_rlb[3], const float query_pos[3])
{
    /* IVP_Sphere_Solver::recursively_collect_intruding_objects (ivp_sphere_solver.cxx:139) */
    float node_luf[3], node_size;
    ivp_ov_tree_get_luf_coordinates_ws(q->tree, node, node_luf, &node_size);
    float half = node_size / 2.0f;
    const float node_rlb[3] = { half, half, half };
    const float node_pos[3] = { node_luf[0] + half, node_luf[1] + half, node_luf[2] + half };

    if (!aabb_overlaps_aabb(node_rlb, node_pos, query_rlb, query_pos)) return;

    /* objects of this node, first to last */
    for (int i = 0; i < ivp_pvec_len(&node->elements); i++) {
        const ivp_ov_element_t *elem = (const ivp_ov_element_t *)ivp_pvec_at(&node->elements, i);
        ivp_object_t *obj = elem->real_object;
        const ivp_vec3r_t *pos = &obj->physical_core->m_world_f_core_last_psi.pos;   /* get_position_PSI() */
        double p[3] = { (double)pos->k[0], (double)pos->k[1], (double)pos->k[2] };

        /* object bounding sphere against the query sphere */
        if (!overlaps_d(q->center, q->radius, p, (double)(float)obj->physical_core->upper_limit_radius)) continue;

        /* query sphere center in object space */
        double q_os[3];
        to_object_space(obj, &q->tmpl->center, q_os);
        if (sphere_against_object_os(obj, q_os, q->radius, q->max_depth)) {
            if (q->out && q->found < q->capacity) q->out[q->found] = obj;
            q->found++;
        }
    }

    /* children, first to last */
    for (int i = 0; i < ivp_pvec_len(&node->children); i++) {
        collect_intruding_objects(q, (const ivp_ov_node_t *)ivp_pvec_at(&node->children, i),
                                  query_rlb, query_pos);
    }
}

int ivp_sphere_solver_check_environment(const ivp_sphere_solver_template_t *tmpl,
                                        const struct ivp_environment *env,
                                        struct ivp_object **intruding_objects,
                                        int capacity)
{
    /* IVP_Sphere_Solver::check_sphere_against_environment (ivp_sphere_solver.cxx:114) */
    if (!tmpl || !env) return 0;
    sphere_env_query_t q;
    q.tree = ivp_env_get_ov_tree((struct ivp_environment *)env);
    if (!q.tree) return 0;
    const ivp_ov_node_t *root = q.tree->root;
    if (!root) return 0;
    q.tmpl = tmpl;
    q.center[0] = (double)tmpl->center.k[0];
    q.center[1] = (double)tmpl->center.k[1];
    q.center[2] = (double)tmpl->center.k[2];
    q.radius = (double)tmpl->radius;
    q.max_depth = tmpl->max_traversal_depth;
    q.out = intruding_objects;
    q.capacity = capacity;
    q.found = 0;

    /* sphere_bb_rlb.set(m_radius, ...), sphere_bb_pos.set(m_center.k[0], ...) */
    const float sphere_bb_rlb[3] = { (float)q.radius, (float)q.radius, (float)q.radius };
    const float sphere_bb_pos[3] = { (float)q.center[0], (float)q.center[1], (float)q.center[2] };
    collect_intruding_objects(&q, root, sphere_bb_rlb, sphere_bb_pos);
    return q.found;
}
