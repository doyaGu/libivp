/* ivp_build_ledge_soup.c -- Ledge soup builder implementation
 *
 * Faithful port of ivp_compact_builder/ivp_surbuild_ledge_soup.cxx
 * (Ipion Software GmbH) with IVP_CLUSTER_SHORTRANGE_TOPDOWN and
 * IVP_CLUSTER_SHORTRANGE_BALANCED defined (the reference build flags):
 *
 *   1. ledges_to_boxes_and_spheres(): one terminal sphere per ledge
 *      (bounding box center, half diagonal, encoded box sizes);
 *   2. cluster_spheres_topdown_mediancut(): balanced mean split over all
 *      three axes with the reference handling of spheres on the median;
 *   3. optional build_root_convex_hull() (IVP_Compact_Recursive);
 *   4. allocate_compact_surface() / insert_compact_ledges() (optional point
 *      merging) / create_compact_ledgetree() (preorder);
 *   5. insert_radius_in_compact_surface(): IVP_Rot_Inertia_Solver mass
 *      center + unit inertia, radius and deviation measured from the mass
 *      center, max_factor = int(1 + dev / (r / 250)).
 */
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/builder/ivp_build_compact_recursive.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_rot_inertia.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_compact_ledge_geom.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/containers/ivp_hashmap.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/diagnostics/ivp_debug_manager.h>

#include <float.h>
#include <math.h>
#include <string.h>

#define LS_DOUBLE_EPS 1e-10            /* P_DOUBLE_EPS */
#define LS_FLOAT_RES  ((double)1e-6f)  /* P_FLOAT_RES  */

/* ── IVV_Sphere ────────────────────────────────────────────────────────── */

typedef struct ls_sphere {
    double center[3];
    double radius;
    uint8_t box_sizes[3];
    const ivp_compact_ledge_t *compact_ledge; /* source ledge (terminal or hull) */
    ivp_compact_ledge_t *dst_ledge;           /* copy inside the surface */
    struct ls_sphere *child_1;
    struct ls_sphere *child_2;
} ls_sphere_t;

typedef struct ls_ctx {
    const ivp_allocator_t *alloc;
    ls_sphere_t *spheres;      /* arena */
    int n_spheres;
    int cap_spheres;
    int number_of_nodes;
    bool failed;
    char *clt_highmem; /* debugging (set in IVP_IF(1), checked by the IVP_ASSERTs) */
    char *clt_lowmem;  /* debugging */
} ls_ctx_t;

/* debug_sphere_output (ivp_surbuild_ledge_soup.cxx:60), only called from an
 * IVP_IF(0) block in create_compact_ledgetree() */
static int ivp_debug_sf_indent = 0;
static int ivp_debug_sf_treedepth = 0;
static int ivp_debug_sf_max_treedepth = 0;
static int ivp_debug_sf_n_nodes_with_one_terminal = 0;
static bool ivp_debug_sf_last_node_was_terminal = false;

static void ls_debug_sphere_output(const ls_sphere_t *sphere)
{
    ivp_debug_sf_treedepth++;
    if (ivp_debug_sf_max_treedepth < ivp_debug_sf_treedepth)
        ivp_debug_sf_max_treedepth = ivp_debug_sf_treedepth;

    if (sphere->child_1) {
        IVP_ASSERT(sphere->child_1); /* prevent nodes with one single child */
        IVP_ASSERT(sphere->child_2);

        int n_terminal_children = 0;

        ls_debug_sphere_output(sphere->child_1);
        if (ivp_debug_sf_last_node_was_terminal) n_terminal_children++;

        ls_debug_sphere_output(sphere->child_2);
        if (ivp_debug_sf_last_node_was_terminal) n_terminal_children++;

        if (n_terminal_children == 1) ivp_debug_sf_n_nodes_with_one_terminal++;

        ivp_debug_sf_last_node_was_terminal = false;
    } else {
        IVP_ASSERT(!sphere->child_1); /* prevent nodes with one single child */
        IVP_ASSERT(!sphere->child_2);
        ivp_debug_sf_last_node_was_terminal = true;
    }
    ivp_debug_sf_treedepth--;
}

static ls_sphere_t *ls_new_sphere(ls_ctx_t *c)
{
    IVP_ASSERT(c->n_spheres < c->cap_spheres);
    ls_sphere_t *s = &c->spheres[c->n_spheres++];
    memset(s, 0, sizeof(*s));
    return s;
}

static uint8_t ls_box_size(double extent, double work)
{
    int bs = (int)(extent / work) + 1;
    if (bs > 255) bs = 255;
    if (bs < 0) bs = 0;
    return (uint8_t)bs;
}

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

void ivp_ledge_soup_template_init(ivp_ledge_soup_template_t *templ)
{
    if (!templ) return;
    templ->force_convex_hull = NULL;
    templ->build_root_convex_hull = false;
    templ->free_input_compact_ledges = false;
    templ->link_to_input_compact_ledges = false;
    templ->merge_points = IVP_SLMP_NO_MERGE;
}

void ivp_ledge_soup_init(ivp_ledge_soup_t *soup, const ivp_allocator_t *alloc)
{
    if (!soup) return;
    if (!alloc) alloc = ivp_default_allocator();
    soup->alloc = alloc;
    soup->n_ledges = 0;
    soup->capacity = 16;
    soup->ledges = (const ivp_compact_ledge_t **)ivp_alloc(alloc,
        (size_t)soup->capacity * sizeof(ivp_compact_ledge_t *));
    if (!soup->ledges) soup->capacity = 0;
}

void ivp_ledge_soup_destroy(ivp_ledge_soup_t *soup)
{
    if (!soup) return;
    if (soup->ledges) {
        ivp_free(soup->alloc, soup->ledges,
                 (size_t)soup->capacity * sizeof(ivp_compact_ledge_t *));
        soup->ledges = NULL;
    }
    soup->n_ledges = 0;
    soup->capacity = 0;
}

int ivp_ledge_soup_add(ivp_ledge_soup_t *soup, const ivp_compact_ledge_t *ledge)
{
    if (!ledge) {
        /* IVP_SurfaceBuilder_Ledge_Soup::insert_ledge (not inside IVP_IF) */
        IVP_IFDEBUG(IVP_DM_SURBUILD_LEDGESOUP) {
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_LEDGESOUP, "warning: tried to add NULL ledge in IVP_SurfaceBuilder_Ledge_Soup::insert_ledge()\n");
        }
        return -1;
    }
    if (!soup) return -1;
    if (!soup->ledges || soup->capacity <= 0) return -1;
    if (soup->n_ledges >= IVP_LEDGE_SOUP_MAX_LEDGES) return -1;
    if (ivp_ledge_get_n_triangles(ledge) <= 0) return -1;

    if (soup->n_ledges >= soup->capacity) {
        int new_cap = soup->capacity * 2;
        const ivp_compact_ledge_t **arr = (const ivp_compact_ledge_t **)ivp_alloc(
            soup->alloc, (size_t)new_cap * sizeof(ivp_compact_ledge_t *));
        if (!arr) return -1;
        memcpy(arr, soup->ledges, (size_t)soup->n_ledges * sizeof(ivp_compact_ledge_t *));
        ivp_free(soup->alloc, soup->ledges,
                 (size_t)soup->capacity * sizeof(ivp_compact_ledge_t *));
        soup->ledges = arr;
        soup->capacity = new_cap;
    }
    soup->ledges[soup->n_ledges++] = ledge;
    return 0;
}

/* ── Spheres (ledges_to_boxes_and_spheres) ─────────────────────────────── */

static void ls_ledge_to_sphere(ls_sphere_t *s, const ivp_compact_ledge_t *ledge)
{
    /* ivp_surbuild_ledge_soup.cxx:485 */
    IVP_ASSERT(ivp_ledge_get_n_triangles(ledge) > 0);
    const double dimension_steps = (double)IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;
    double mn[3], mx[3];
    ivp_ledge_calc_bounding_box_d(ledge, mn, mx);
    double rad[3];
    for (int i = 0; i < 3; i++) {
        /* center.set_interpolate(&max, &min, 0.5f) */
        s->center[i] = mx[i] * (1.0 - (double)0.5f) + mn[i] * (double)0.5f;
        rad[i] = mx[i] - s->center[i];
    }
    s->radius = sqrt(rad[0] * rad[0] + rad[1] * rad[1] + rad[2] * rad[2]);
    double work = s->radius * dimension_steps;
    if (work < LS_DOUBLE_EPS) work = LS_DOUBLE_EPS;
    for (int i = 0; i < 3; i++) s->box_sizes[i] = ls_box_size(mx[i] - s->center[i], work);
    s->compact_ledge = ledge;
}

/* calculate_boundingbox(): float extents (IVP_U_Float_Point) */
static void ls_calc_bbox(ls_sphere_t *const *t, int n, float ext_min[3], float ext_max[3])
{
    const double dimension_steps = (double)IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;
    for (int i = 0; i < 3; i++) {
        ext_min[i] = 1000000.0f;
        ext_max[i] = -1000000.0f;
    }
    for (int x = 0; x < n; x++) {
        const ls_sphere_t *s = t[x];
        double work = dimension_steps * s->radius;
        for (int i = 0; i < 3; i++) {
            double lo = s->center[i] - ((double)s->box_sizes[i] * work);
            double hi = s->center[i] + ((double)s->box_sizes[i] * work);
            if (lo < (double)ext_min[i]) ext_min[i] = (float)lo;
            if (hi > (double)ext_max[i]) ext_max[i] = (float)hi;
        }
    }
}

static float ls_bbox_volume(const float mn[3], const float mx[3])
{
    float d0 = (float)fabs((double)mx[0] - (double)mn[0]);
    float d1 = (float)fabs((double)mx[1] - (double)mn[1]);
    float d2 = (float)fabs((double)mx[2] - (double)mn[2]);
    return d0 * d1 * d2;
}

/* cluster_spheres_topdown_mediancut_recursively (ivp_surbuild_ledge_soup.cxx:1079) */
static ls_sphere_t *ls_cluster(ls_ctx_t *c, ls_sphere_t **terminals, int len)
{
    IVP_ASSERT(len > 0);
    if (len == 1) return terminals[0];
    if (c->failed) return terminals[0];

    float ext_min[3], ext_max[3];
    ls_calc_bbox(terminals, len, ext_min, ext_max);

    const double dimension_steps = (double)IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;
    double new_center[3], rad[3];
    for (int i = 0; i < 3; i++) {
        /* set_interpolate(&extents_max, &extents_min, 0.5f) */
        new_center[i] = (double)ext_max[i] * (1.0 - (double)0.5f) + (double)ext_min[i] * (double)0.5f;
        rad[i] = (double)ext_max[i] - new_center[i];
    }
    double new_radius = sqrt(rad[0] * rad[0] + rad[1] * rad[1] + rad[2] * rad[2]);
    double work = new_radius * dimension_steps;
    if (work < LS_DOUBLE_EPS) work = LS_DOUBLE_EPS;

    ls_sphere_t *ns = ls_new_sphere(c);
    ns->radius = new_radius;
    memcpy(ns->center, new_center, sizeof(new_center));
    ns->compact_ledge = NULL;
    for (int i = 0; i < 3; i++) {
        ns->box_sizes[i] = ls_box_size((double)ext_max[i] - new_center[i], work);
    }

    if (len == 2) {
        ns->child_1 = ls_cluster(c, &terminals[0], 1);
        ns->child_2 = ls_cluster(c, &terminals[1], 1);

        IVP_ASSERT(ns->child_1 != 0);
        IVP_ASSERT(ns->child_2 != 0);

        c->number_of_nodes++;
        return ns;
    }

    size_t vec_sz = (size_t)len * 6u * sizeof(ls_sphere_t *);
    ls_sphere_t **buf = (ls_sphere_t **)ivp_alloc(c->alloc, vec_sz);
    if (!buf) {
        c->failed = true;
        ns->child_1 = terminals[0];
        ns->child_2 = terminals[1];
        return ns;
    }
    ls_sphere_t **lh[3], **rh[3];
    int n_l[3] = { 0, 0, 0 }, n_r[3] = { 0, 0, 0 };
    for (int a = 0; a < 3; a++) {
        lh[a] = buf + (size_t)(2 * a) * (size_t)len;
        rh[a] = buf + (size_t)(2 * a + 1) * (size_t)len;
    }
    float diff[3];

    for (int axis = 0; axis < 3; axis++) {
        double median = 0.0;
        for (int m = 0; m < len; m++) median += terminals[m]->center[axis];
        median /= (double)len;

        IVP_IF(0) {
            /* ivp_message("Median on axis %d: %f\n", axis, median); */
        }

        bool toggle = true;
        for (int x = 0; x < len; x++) {
            ls_sphere_t *ts = terminals[x];
            double v = ts->center[axis];
            if (v < median - LS_FLOAT_RES) {
                lh[axis][n_l[axis]++] = ts;
            } else if (v > median + LS_FLOAT_RES) {
                rh[axis][n_r[axis]++] = ts;
            } else {
                const ls_sphere_t *ref = (x == len - 1) ? terminals[x - 1] : terminals[x + 1];
                double rv = ref->center[axis];
                if (rv < median - LS_FLOAT_RES) {
                    rh[axis][n_r[axis]++] = ts;
                    continue;
                }
                if (rv > median + LS_FLOAT_RES) {
                    lh[axis][n_l[axis]++] = ts;
                    continue;
                }
                if (n_l[axis] == 0) {
                    lh[axis][n_l[axis]++] = ts;
                    continue;
                }
                if (n_r[axis] == 0) {
                    rh[axis][n_r[axis]++] = ts;
                    continue;
                }
                if (toggle) {
                    lh[axis][n_l[axis]++] = ts;
                    toggle = false;
                } else {
                    rh[axis][n_r[axis]++] = ts;
                    toggle = true;
                }
            }
        }

        float mn0[3], mx0[3], mn1[3], mx1[3];
        ls_calc_bbox(lh[axis], n_l[axis], mn0, mx0);
        ls_calc_bbox(rh[axis], n_r[axis], mn1, mx1);
        float volume1 = ls_bbox_volume(mn0, mx0);
        float volume2 = ls_bbox_volume(mn1, mx1);
        diff[axis] = volume2 + volume1;
    }

    int chosen_axis = 0;
    {
        double max_diff = DBL_MAX;
        for (int i = 0; i < 3; i++) {
            if ((double)diff[i] < max_diff && n_l[i] != 0 && n_r[i] != 0) {
                chosen_axis = i;
                max_diff = (double)diff[i];
            }
        }
    }
    IVP_ASSERT(n_l[chosen_axis] != 0); /* lefthand_terminals[chosen_axis]->len() != 0 */
    IVP_ASSERT(n_r[chosen_axis] != 0); /* righthand_terminals[chosen_axis]->len() != 0 */
    if (n_l[chosen_axis] == 0 || n_r[chosen_axis] == 0) {
        /* cannot happen with the tie rules above (reference asserts) */
        c->failed = true;
        ivp_free(c->alloc, buf, vec_sz);
        ns->child_1 = terminals[0];
        ns->child_2 = terminals[1];
        return ns;
    }

    ns->child_1 = ls_cluster(c, lh[chosen_axis], n_l[chosen_axis]);
    ns->child_2 = ls_cluster(c, rh[chosen_axis], n_r[chosen_axis]);
    ivp_free(c->alloc, buf, vec_sz);

    IVP_ASSERT(ns->child_1 != 0);
    IVP_ASSERT(ns->child_2 != 0);

    c->number_of_nodes++;
    return ns;
}

/* add_ledge_tree_to_convex_hull: terminal ledges in tree order */
static int ls_collect_tree_ledges(const ls_sphere_t *node, const ivp_compact_ledge_t **out, int pos)
{
    if (!node) return pos;
    if (node->compact_ledge) {
        out[pos++] = node->compact_ledge;
        return pos;
    }
    pos = ls_collect_tree_ledges(node->child_1, out, pos);
    pos = ls_collect_tree_ledges(node->child_2, out, pos);
    return pos;
}

/* ── Point merging (recompile_point_indizes_of_compact_ledge) ──────────── */

typedef struct ls_point_merge {
    ivp_hashmap_t map;                 /* key: 3 floats bitwise -> index+1 */
    ivp_compact_poly_point_t *first;   /* shared point array */
    int n_allocated;
    bool active;
} ls_point_merge_t;

/* Size of a compact copy of `l`: header + triangles + every referenced
 * point.  Equals get_size() for compact ledges (the reference memcpy's
 * get_size() bytes); also handles ledges whose points live elsewhere
 * (merged surfaces). */
static int ls_ledge_copy_size(const ivp_compact_ledge_t *l)
{
    int n_tri = ivp_ledge_get_n_triangles(l);
    const ivp_compact_triangle_t *t = ivp_ledge_get_first_triangle(l);
    int max_index = -1;
    for (int i = 0; i < n_tri; i++) {
        for (int e = 0; e < 3; e++) {
            int pi = ivp_edge_get_start_point_index(&t[i].edges[e]);
            if (pi > max_index) max_index = pi;
        }
    }
    int sz = (int)sizeof(ivp_compact_ledge_t) + n_tri * (int)sizeof(ivp_compact_triangle_t)
           + (max_index + 1) * (int)sizeof(ivp_compact_poly_point_t);
    int own = ivp_ledge_get_size(l);
    return (ivp_ledge_get_is_compact(l) && own >= sz) ? own : sz;
}

static int ls_copy_ledge(ls_point_merge_t *pm, const ivp_compact_ledge_t *src, char *dest)
{
    if (!pm->active) {
        /* recompile_point_indizes_of_compact_ledge without point hash:
         * plain copy (points placed right behind the triangles). */
        int size = ls_ledge_copy_size(src);
        int n_tri = ivp_ledge_get_n_triangles(src);
        size_t head = sizeof(ivp_compact_ledge_t) + (size_t)n_tri * sizeof(ivp_compact_triangle_t);
        memcpy(dest, src, head);
        memcpy(dest + head, ivp_ledge_get_point_array(src), (size_t)size - head);
        ivp_compact_ledge_t *dl = (ivp_compact_ledge_t *)dest;
        IVP_ASSERT(((int)head & 3) == 0);   /* set_offset_ledge_points (ivp_compact_ledge.hxx:173) */
        dl->c_point_offset = (int32_t)head;
        ivp_ledge_set_size(dl, size);
        ivp_ledge_set_is_compact(dl, 1);
        return size;
    }

    ivp_compact_ledge_t *dl = (ivp_compact_ledge_t *)dest;
    int n_tri = ivp_ledge_get_n_triangles(src);
    int size = (int)sizeof(ivp_compact_ledge_t) + (int)sizeof(ivp_compact_triangle_t) * n_tri;
    memcpy(dest, src, (size_t)size);

    ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle_mut(dl);
    const ivp_compact_poly_point_t *sp = ivp_ledge_get_point_array(src);
    for (int t = 0; t < n_tri; t++, tri++) {
        for (int e = 0; e < 3; e++) {
            ivp_compact_edge_t *edge = &tri->edges[e];
            const ivp_compact_poly_point_t *p_in = &sp[ivp_edge_get_start_point_index(edge)];
            void *found = ivp_hashmap_get(&pm->map, p_in->k);
            if (found) {
                ivp_edge_set_start_point_index(edge, (int)((uintptr_t)found - 1u));
            } else {
                int idx = pm->n_allocated++;
                ivp_edge_set_start_point_index(edge, idx);
                pm->first[idx] = *p_in;
                (void)ivp_hashmap_put(&pm->map, pm->first[idx].k,
                                      (void *)((uintptr_t)idx + 1u));
            }
        }
    }
    {
        /* dl->set_offset_ledge_points(offset) (ivp_compact_ledge.hxx:173) */
        int32_t offset = (int32_t)((char *)pm->first - (char *)dl);
        IVP_ASSERT((offset & 3) == 0);
        dl->c_point_offset = offset;
    }
    return size;
}

/* ── Ledgetree (build_ledgetree, preorder) ─────────────────────────────── */

static ivp_compact_ledgetree_node_t *ls_build_ledgetree(const ls_ctx_t *c, ls_sphere_t *node,
                                                        ivp_compact_ledgetree_node_t **work)
{
    ivp_compact_ledgetree_node_t *cur = *work;

    IVP_ASSERT((intptr_t)cur < (intptr_t)c->clt_highmem); /* ledgetree memory overwrite! */
    IVP_ASSERT((intptr_t)cur >= (intptr_t)c->clt_lowmem); /* ledgetree memory underwrite! */
    IVP_USE(c);

    (*work)++;
    cur->center[0] = (float)node->center[0];
    cur->center[1] = (float)node->center[1];
    cur->center[2] = (float)node->center[2];
    cur->radius = (float)node->radius;
    cur->box_sizes[0] = node->box_sizes[0];
    cur->box_sizes[1] = node->box_sizes[1];
    cur->box_sizes[2] = node->box_sizes[2];
    cur->padding = 0;

    if (node->child_1) {
        IVP_ASSERT(node->child_2);
        if (node->dst_ledge) {
            cur->offset_compact_ledge = (int32_t)((char *)node->dst_ledge - (char *)cur);
            node->dst_ledge->node_or_client = (int32_t)((char *)cur - (char *)node->dst_ledge);
            ivp_ledge_set_has_children(node->dst_ledge, 1);
        } else {
            cur->offset_compact_ledge = 0;
        }
        ls_build_ledgetree(c, node->child_1, work);
        ivp_compact_ledgetree_node_t *right = ls_build_ledgetree(c, node->child_2, work);
        cur->offset_right_node = (int32_t)((char *)right - (char *)cur);
    } else {
        IVP_ASSERT(!node->child_2);
        ivp_ledge_set_has_children(node->dst_ledge, 0);
        cur->offset_compact_ledge = (int32_t)((char *)node->dst_ledge - (char *)cur);
        cur->offset_right_node = 0;
    }
    IVP_ASSERT(cur->box_sizes[0] != 0);
    return cur;
}

/* ledgetree_debug_output (ivp_surbuild_ledge_soup.cxx:1645): DEBUGGING ONLY,
 * walks the tree checking the node addresses (its printing is commented
 * out in the reference) */
static void ls_ledgetree_debug_output(const ls_ctx_t *c, const ivp_compact_ledgetree_node_t *node)
{
    IVP_IF(1) {
        IVP_ASSERT((intptr_t)node < (intptr_t)c->clt_highmem); /* ledgetree memory overread! */
        IVP_ASSERT((intptr_t)node >= (intptr_t)c->clt_lowmem); /* ledgetree memory underread! */
    }
    if (node->offset_right_node == 0) return;
    if (!ivp_ledgetree_is_terminal(node)) {
        ls_ledgetree_debug_output(c, ivp_ledgetree_left_son(node));
        ls_ledgetree_debug_output(c, ivp_ledgetree_right_son(node));
    }
}

/* ── insert_radius_in_compact_surface ──────────────────────────────────── */

void ivp_ledge_soup_insert_radius(ivp_compact_surface_t *surface)
{
    if (!surface) return;
    double mass_center[3], rot_inertia[3];
    ivp_calc_mass_center_and_rot_inertia_d(surface, mass_center, rot_inertia);

    double mass_radius = 0.0, mass_radius_dev = 0.0;
    ivp_surface_calc_radius_to_given_center(surface, mass_center, &mass_radius, &mass_radius_dev);

    for (int i = 0; i < 3; i++) {
        surface->rotation_inertia[i] = (float)rot_inertia[i];
        surface->mass_center[i] = (float)mass_center[i];
    }
    surface->upper_limit_radius = (float)mass_radius;
    IVP_ASSERT(mass_radius_dev >= 0.0);
    IVP_ASSERT(mass_radius_dev <= mass_radius + LS_FLOAT_RES);

    /* int(1.0f + dev / (r * IVP_COMPACT_SURFACE_DEVIATION_STEP_SIZE)) */
    int factor = 1;
    double denom = mass_radius * (double)IVP_COMPACT_SURFACE_DEVIATION_STEP_SIZE;
    if (denom > 0.0) {
        double f = (double)1.0f + mass_radius_dev / denom;
        if (f > 255.0) f = 255.0;
        factor = (int)f;
    }
    ivp_surface_set_max_deviation(surface, factor);
}

/* ── Compile ───────────────────────────────────────────────────────────── */

ivp_compact_surface_t *ivp_ledge_soup_compile(ivp_ledge_soup_t *soup)
{
    return ivp_ledge_soup_compile_ex(soup, NULL);
}

static uint32_t ls_hash_point(const void *key, size_t len)
{
    /* FNV-1a over the 12 key bytes */
    const unsigned char *b = (const unsigned char *)key;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h ^= b[i];
        h *= 16777619u;
    }
    return h;
}

ivp_compact_surface_t *ivp_ledge_soup_compile_ex(ivp_ledge_soup_t *soup,
                                                 const ivp_ledge_soup_template_t *templ_in)
{
    if (!soup || soup->n_ledges <= 0 || !soup->ledges) return NULL;

    ivp_ledge_soup_template_t templ;
    if (templ_in) templ = *templ_in;
    else ivp_ledge_soup_template_init(&templ);

    const ivp_allocator_t *alloc = soup->alloc;
    const int n_ledges = soup->n_ledges;

    ls_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.alloc = alloc;
    ctx.cap_spheres = 2 * n_ledges;
    size_t spheres_sz = (size_t)ctx.cap_spheres * sizeof(ls_sphere_t);
    ctx.spheres = (ls_sphere_t *)ivp_alloc(alloc, spheres_sz);
    size_t term_sz = (size_t)n_ledges * sizeof(ls_sphere_t *);
    ls_sphere_t **terminals = (ls_sphere_t **)ivp_alloc(alloc, term_sz);
    ivp_compact_ledge_t *hull = NULL;
    ivp_compact_surface_t *surface = NULL;
    void *mem = NULL;
    size_t mem_sz = 0;

    if (!ctx.spheres || !terminals) goto cleanup;

    /* 1. terminal spheres */
    for (int i = 0; i < n_ledges; i++) {
        ls_sphere_t *s = ls_new_sphere(&ctx);
        ls_ledge_to_sphere(s, soup->ledges[i]);
        terminals[i] = s;
    }

    /* 2. top-down median cut */
    ctx.number_of_nodes = n_ledges;
    ls_sphere_t *root;
    {
        ls_sphere_t **work = (ls_sphere_t **)ivp_alloc(alloc, term_sz);
        if (!work) goto cleanup;
        memcpy(work, terminals, term_sz);
        root = ls_cluster(&ctx, work, n_ledges);
        ivp_free(alloc, work, term_sz);
    }
    if (ctx.failed) goto cleanup;

    IVP_ASSERT(surface == NULL); /* compact_surface == 0 */

    /* 3. root convex hull (only for real compounds) */
    ls_sphere_t *rec_sphere = NULL;
    if (templ.build_root_convex_hull && n_ledges > 1) {
        const ivp_compact_ledge_t **tree_ledges =
            (const ivp_compact_ledge_t **)ivp_alloc(alloc, (size_t)n_ledges * sizeof(*tree_ledges));
        if (!tree_ledges) goto cleanup;
        int nt = ls_collect_tree_ledges(root, tree_ledges, 0);
        if (templ.force_convex_hull) {
            hull = ivp_compact_ledge_clone(alloc, templ.force_convex_hull);
            if (hull) ivp_compact_recursive_set_virtual_flags(hull, tree_ledges, nt);
        } else {
            hull = ivp_compact_recursive_build_hull(alloc, tree_ledges, nt);
        }
        ivp_free(alloc, (void *)tree_ledges, (size_t)n_ledges * sizeof(*tree_ledges));
        if (hull) {
            root->compact_ledge = hull;
            rec_sphere = root;
        }
    }

    /* 4. allocate_compact_surface */
    const int cs_header_size = (int)sizeof(ivp_compact_surface_t);
    long est_ledge_list = 0;
    int n_tri_compiled = 0;
    int n_ledges_compiled = 0;
    for (int i = 0; i < n_ledges; i++) {
        est_ledge_list += ls_ledge_copy_size(terminals[i]->compact_ledge);
        n_tri_compiled += ivp_ledge_get_n_triangles(terminals[i]->compact_ledge);
        n_ledges_compiled++;
    }
    if (rec_sphere) {
        est_ledge_list += ls_ledge_copy_size(hull);
        n_tri_compiled += ivp_ledge_get_n_triangles(hull);
        n_ledges_compiled++;
    }
    const int cs_ledgetree_size = ctx.number_of_nodes * (int)sizeof(ivp_compact_ledgetree_node_t);
    long cs_size = (long)cs_header_size + est_ledge_list + cs_ledgetree_size;
    if (cs_size > (long)INT32_MAX - 16) goto cleanup;
    cs_size = (cs_size + 15) & ~15L;

    IVP_IF(1) { /*printf("Compact surface size : %d\n", cs_size);*/ }

    mem_sz = (size_t)cs_size;
    mem = ivp_alloc_aligned(alloc, mem_sz, 16);
    if (!mem) goto cleanup;
    memset(mem, 0, mem_sz);
    surface = (ivp_compact_surface_t *)mem;

    ls_point_merge_t pm;
    memset(&pm, 0, sizeof(pm));
    if (n_ledges_compiled > 1 && templ.merge_points != IVP_SLMP_NO_MERGE) {
        int hash_cap = 16;
        while (hash_cap < n_tri_compiled * 2) hash_cap <<= 1;
        if (!ivp_ok(ivp_hashmap_init(&pm.map, alloc, hash_cap, ls_hash_point, ivp_eq_memcmp,
                                     3 * sizeof(float)))) {
            goto cleanup;
        }
        pm.active = true;
        pm.first = (ivp_compact_poly_point_t *)((char *)mem + cs_header_size +
                   (size_t)n_tri_compiled * sizeof(ivp_compact_triangle_t) +
                   (size_t)n_ledges_compiled * sizeof(ivp_compact_ledge_t));
    }

    /* insert_compact_ledges: terminals in insertion order, then the hull */
    {
        char *dest = (char *)mem + cs_header_size;
        for (int i = 0; i < n_ledges; i++) {
            IVP_ASSERT(((uintptr_t)dest & 15u) == 0);
            terminals[i]->dst_ledge = (ivp_compact_ledge_t *)dest;
            dest += ls_copy_ledge(&pm, terminals[i]->compact_ledge, dest);
        }
        if (rec_sphere) {
            IVP_ASSERT(((uintptr_t)dest & 15u) == 0);
            rec_sphere->dst_ledge = (ivp_compact_ledge_t *)dest;
            dest += ls_copy_ledge(&pm, hull, dest);
        }
        IVP_ASSERT(!pm.active || dest == (char *)pm.first);
    }

    long cs_ledgelist_size;
    if (pm.active) {
        ivp_hashmap_destroy(&pm.map);
        cs_ledgelist_size = (long)n_tri_compiled * (long)sizeof(ivp_compact_triangle_t)
                          + (long)n_ledges_compiled * (long)sizeof(ivp_compact_ledge_t)
                          + (long)pm.n_allocated * (long)sizeof(ivp_compact_poly_point_t);
    } else {
        cs_ledgelist_size = est_ledge_list;
    }

    const int cs_real_size = cs_header_size + (int)cs_ledgelist_size + cs_ledgetree_size;
    ivp_surface_set_byte_size(surface, cs_real_size);
    surface->offset_ledgetree_root = cs_header_size + (int32_t)cs_ledgelist_size;
    memset(surface->_pad, 0, sizeof(surface->_pad));
    IVP_IF(1) {
        ctx.clt_lowmem = (char *)mem + surface->offset_ledgetree_root; /* ledgetree_work */
        ctx.clt_highmem = (char *)mem + cs_real_size;
    }

    /* 5. create_compact_ledgetree */
    {
        IVP_IF(0) {
            ivp_debug_sf_indent = 0;
            ls_debug_sphere_output(root);
        }
        ivp_compact_ledgetree_node_t *work =
            (ivp_compact_ledgetree_node_t *)((char *)mem + surface->offset_ledgetree_root);
        ivp_compact_ledgetree_node_t *r = ls_build_ledgetree(&ctx, root, &work);
        IVP_ASSERT(r == ivp_surface_get_ledgetree_root(surface));
        IVP_UNUSED(r);
        IVP_IF(1) {
            ls_ledgetree_debug_output(&ctx, ivp_surface_get_ledgetree_root(surface));
        }
    }

    /* 6. insert_radius_in_compact_surface */
    ivp_ledge_soup_insert_radius(surface);

    /* optional reallocation for merged points (ivp_surbuild_ledge_soup.cxx:228) */
    if (n_ledges > 1 && templ.merge_points == IVP_SLMP_MERGE_AND_REALLOCATE &&
        (size_t)cs_real_size < mem_sz) {
        size_t new_sz = ((size_t)cs_real_size + 15u) & ~(size_t)15u;
        void *res = ivp_alloc_aligned(alloc, new_sz, 16);
        if (res) {
            memcpy(res, mem, (size_t)cs_real_size);
            if (new_sz > (size_t)cs_real_size) {
                memset((char *)res + cs_real_size, 0, new_sz - (size_t)cs_real_size);
            }
            ivp_free_aligned(alloc, mem);
            mem = res;
            surface = (ivp_compact_surface_t *)res;
        }
    }

    if (templ.free_input_compact_ledges) {
        for (int i = 0; i < n_ledges; i++) {
            ivp_free_aligned(alloc, (void *)soup->ledges[i]);
        }
    }
    soup->n_ledges = 0; /* c_ledge_vec.clear() */
    mem = NULL;         /* ownership passes to the caller */

cleanup:
    if (mem) {
        ivp_free_aligned(alloc, mem);
        surface = NULL;
    }
    if (hull) ivp_compact_ledge_free(alloc, hull);
    if (terminals) ivp_free(alloc, terminals, term_sz);
    if (ctx.spheres) ivp_free(alloc, ctx.spheres, spheres_sz);
    return surface;
}
