/* ivp_build_polygon_convex.c -- reference convex polygon -> compact ledge
 *
 * Port of (see ivp_build_polygon_convex_internal.h for the call chain):
 *   ivp_compact_builder/ivp_surbuild_pointsoup.cxx     planes_to_template,
 *                                                      get_offset_from_pointlist,
 *                                                      get_offset_from_lineslist
 *   ivp_compact_builder/ivp_surbuild_polygon_convex.cxx IVP_SurfaceBuilder_Polygon_Convex
 *   ivp_compact_builder/ivp_object_polygon_tetra.cxx   make_triangles, P_Sur_2D*,
 *                                                      calc_hesse, insert_pierce_info,
 *                                                      generate_double_triangle
 *   ivp_compact_builder/ivp_compact_ledge_gen.cxx      IVP_Compact_Ledge_Generator
 *
 * Only the code reachable from convert_template_to_ledge is ported:
 * init_surface_manager_polygon never calls convexify(), so IVP_Tetra_Intrude
 * and the pop / min-hash machinery of IVP_Object_Polygon_Tetra are dead on
 * this path.  Pieces that cannot influence the generated ledge are left out
 * and noted where they would be:
 *   - the hidden backside triangles of make_triangles (skipped by
 *     insert_pierce_info and fill_list_with_all_triangles; they are inserted
 *     in front of the visible ones, so the visible order is unchanged),
 *   - points_to_edge_hash and the tetra point bookkeeping,
 *   - p_count_reachable() (its result is overridden: having_islands = 1).
 *
 * Containers: P_List::insert prepends; IVP_U_Min_Hash (SORT_MINDIST_ELEMENTS
 * is defined) yields its elements by ascending value with ties in insertion
 * order, i.e. a stable sort; IVP_Hash / IVP_point_hash only provide lookups,
 * their iteration order is never used.
 */

#include "ivp_build_polygon_convex_internal.h"

#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/base/ivp_arena.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_triangle.h>

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ── Reference constants (IVP_DOUBLE build) ────────────────────────────── */

#define PC_P_DOUBLE_EPS 1e-10 /* P_DOUBLE_EPS */
/* P_Pop_Too_Flat_Eps is a zero initialised global that only convexify()
 * assigns; convexify() is not reachable from convert_template_to_ledge. */
#define PC_P_POP_TOO_FLAT_EPS 0.0
#define PC_P_CROSS_EPS 0.0001f                                /* P_CROSS_EPS */
#define PC_P_CROSS_EPS_QUAD ((double)(PC_P_CROSS_EPS * PC_P_CROSS_EPS)) /* float product */
#define PC_P_OVERLAP_EPS ((double)1E-8f)                      /* P_OVERLAP_EPS */
#define PC_DET_EPS ((double)0.000000001f)                     /* DET_EPS */
#define PC_MAX_SCAL_VAL ((double)-1E-6f)                      /* IVP_MAX_SCAL_VAL */
#define PC_TRI_LOOP_LIMIT 100 /* calc_triangle_representation loop_counter */

/* ── Template polygon ──────────────────────────────────────────────────── */

/* IVP_SurfaceBuilder_Pointsoup::get_offset_from_pointlist: first point that
 * compares equal (==, so -0.0 matches +0.0). */
static int pc_offset_from_pointlist(const double *pts, int n, const double *p)
{
    for (int i = 0; i < n; i++) {
        if (pts[3 * i + 0] == p[0] && pts[3 * i + 1] == p[1] && pts[3 * i + 2] == p[2]) {
            return i;
        }
    }
    return -1;
}

/* IVP_SurfaceBuilder_Pointsoup::get_offset_from_lineslist */
static int pc_offset_from_lineslist(const ivp_pc_templ_line_t *lines, int n,
                                    int p1, int p2, char *reverse)
{
    for (int x = 0; x < n; x++) {
        if (p1 == lines[x].p[0] && p2 == lines[x].p[1]) {
            *reverse = 0;
            return x;
        }
        if (p1 == lines[x].p[1] && p2 == lines[x].p[0]) {
            *reverse = 1;
            return x;
        }
    }
    return -1;
}

double ivp_pc_plane_area_size(const double *pts, const int *ids, int n, const double *normal)
{
    double sum = 0.0;
    const double *p0 = &pts[3 * ids[0]];
    for (int i = 0; i < n; i++) {
        int in = (i + 1) % n;
        const double *a = &pts[3 * ids[i]];
        const double *b = &pts[3 * ids[in]];
        double d0[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };    /* diff0 = b - a */
        double d1[3] = { a[0] - p0[0], a[1] - p0[1], a[2] - p0[2] }; /* diff1 = a - p0 */
        /* cross.calc_cross_product(&diff0, &diff1) (inline_calc_cross_product) */
        double c0 = d0[1] * d1[2];
        double c0b = d1[1] * d0[2];
        double c1 = d0[2] * d1[0];
        double c1b = d1[2] * d0[0];
        c0 -= c0b;
        double c2 = d0[0] * d1[1];
        double c2b = d1[0] * d0[1];
        c1 -= c1b;
        c2 -= c2b;
        sum += c0 * normal[0] + c1 * normal[1] + c2 * normal[2];
    }
    return sum;
}

void ivp_pc_template_destroy(ivp_pc_templ_polygon_t *t)
{
    if (!t || !t->alloc) return;
    const ivp_allocator_t *a = t->alloc;
    int cap = t->surface_line_capacity;
    if (t->points) ivp_free(a, t->points, (size_t)t->point_capacity * 3u * sizeof(double));
    if (t->lines) ivp_free(a, t->lines, (size_t)cap * sizeof(ivp_pc_templ_line_t));
    if (t->surfaces) ivp_free(a, t->surfaces, (size_t)t->n_surfaces * sizeof(ivp_pc_templ_surface_t));
    if (t->surface_line_storage) ivp_free(a, t->surface_line_storage, (size_t)cap * sizeof(uint16_t));
    if (t->surface_revert_storage) ivp_free(a, t->surface_revert_storage, (size_t)cap);
    memset(t, 0, sizeof(*t));
}

bool ivp_pc_planes_to_template(ivp_pc_templ_polygon_t *templ,
                               const ivp_allocator_t *alloc,
                               const double *points, int n_points,
                               const ivp_pc_plane_t *planes, int n_planes)
{
    memset(templ, 0, sizeof(*templ));
    if (!points || !planes || n_points <= 0 || n_planes <= 0) return false;
    if (n_points > INT_MAX / 3 ||
        (size_t)n_points > SIZE_MAX / (3u * sizeof(double)) ||
        (size_t)n_planes > SIZE_MAX / sizeof(ivp_pc_templ_surface_t)) {
        return false;
    }
    if (!alloc) alloc = ivp_default_allocator();
    templ->alloc = alloc;

    int total = 0;
    for (int x = 0; x < n_planes; x++) {
        if (planes[x].n_points < 0 || planes[x].n_points > (1 << 20)) return false;
        if (planes[x].n_points > INT_MAX - total) return false;
        total += planes[x].n_points;
    }
    if (total <= 0) return false;

    /* Build Pointlist: merge equal points (all input points, also the
     * ones no plane uses). */
    double *tp = (double *)ivp_alloc(alloc, (size_t)n_points * 3u * sizeof(double));
    if (!tp) return false;
    templ->points = tp;
    templ->point_capacity = n_points;
    int np = 0;
    for (int x = 0; x < n_points; x++) {
        const double *p = &points[3 * x];
        if (pc_offset_from_pointlist(tp, np, p) < 0) {
            tp[3 * np + 0] = p[0];
            tp[3 * np + 1] = p[1];
            tp[3 * np + 2] = p[2];
            np++;
        }
    }
    templ->n_points = np;
    if (np > 0xffff) goto fail; /* ushort point offsets */

    templ->surface_line_capacity = total;
    templ->lines = (ivp_pc_templ_line_t *)ivp_alloc(alloc, (size_t)total * sizeof(ivp_pc_templ_line_t));
    templ->surfaces = (ivp_pc_templ_surface_t *)ivp_alloc(alloc, (size_t)n_planes * sizeof(ivp_pc_templ_surface_t));
    templ->surface_line_storage = (uint16_t *)ivp_alloc(alloc, (size_t)total * sizeof(uint16_t));
    templ->surface_revert_storage = (char *)ivp_alloc(alloc, (size_t)total);
    templ->n_surfaces = n_planes;
    if (!templ->lines || !templ->surfaces || !templ->surface_line_storage ||
        !templ->surface_revert_storage) {
        goto fail;
    }
    memset(templ->surfaces, 0, (size_t)n_planes * sizeof(ivp_pc_templ_surface_t));

    /* Build Lineslist: every polygon edge once, in order of appearance and
     * in the direction of its first appearance (lines_hash / lines_vector). */
    int n_lines = 0;
    for (int x = 0; x < n_planes; x++) {
        const ivp_pc_plane_t *pl = &planes[x];
        int fvc = pl->n_points;
        for (int pointnr = 0; pointnr < fvc; pointnr++) {
            int i1 = pl->point_idx[pointnr];
            int i2 = pl->point_idx[(pointnr + 1 == fvc) ? 0 : pointnr + 1];
            if (i1 < 0 || i1 >= n_points || i2 < 0 || i2 >= n_points) goto fail;
            int o1 = pc_offset_from_pointlist(tp, np, &points[3 * i1]);
            int o2 = pc_offset_from_pointlist(tp, np, &points[3 * i2]);
            if (o1 < 0 || o2 < 0) goto fail;
            int found = 0;
            for (int l = 0; l < n_lines; l++) {
                const ivp_pc_templ_line_t *tl = &templ->lines[l];
                if ((tl->p[0] == o1 && tl->p[1] == o2) || (tl->p[0] == o2 && tl->p[1] == o1)) {
                    found = 1;
                    break;
                }
            }
            if (!found) {
                templ->lines[n_lines].p[0] = (uint16_t)o1;
                templ->lines[n_lines].p[1] = (uint16_t)o2;
                n_lines++;
            }
        }
    }
    templ->n_lines = n_lines;

    /* Build Surfacelist */
    int off = 0;
    for (int x = 0; x < n_planes; x++) {
        const ivp_pc_plane_t *pl = &planes[x];
        ivp_pc_templ_surface_t *sur = &templ->surfaces[x];
        int punktezahl = pl->n_points;
        sur->normal[0] = pl->normal[0];
        sur->normal[1] = pl->normal[1];
        sur->normal[2] = pl->normal[2];
        sur->n_lines = punktezahl;
        sur->lines = &templ->surface_line_storage[off];
        sur->revert_line = &templ->surface_revert_storage[off];
        off += punktezahl;
        for (int pointnr = 0; pointnr < punktezahl; pointnr++) {
            int i1 = pl->point_idx[pointnr];
            int i2 = pl->point_idx[(pointnr + 1 == punktezahl) ? 0 : pointnr + 1];
            int o1 = pc_offset_from_pointlist(tp, np, &points[3 * i1]);
            int o2 = pc_offset_from_pointlist(tp, np, &points[3 * i2]);
            char reverse = 0;
            int line_index = pc_offset_from_lineslist(templ->lines, n_lines, o1, o2, &reverse);
            if (line_index < 0) goto fail;
            sur->lines[pointnr] = (uint16_t)line_index;
            sur->revert_line[pointnr] = (char)((reverse * (-1)) + 1);
        }
    }

    return true;

fail:
    ivp_pc_template_destroy(templ);
    return false;
}

/* ── IVP_Object_Polygon_Tetra (triangles) ──────────────────────────────── */

typedef struct pc_point {
    double k[3];       /* IVP_Poly_Point (IVP_U_Point) */
    int compact_index; /* tmp.compact_index */
} pc_point_t;

typedef struct pc_edge pc_edge_t;
typedef struct pc_tri pc_tri_t;

struct pc_edge { /* IVP_Tri_Edge */
    pc_point_t *start_point;
    pc_tri_t *triangle;
    pc_edge_t *next;
    pc_edge_t *prev;
    pc_edge_t *opposite;
    int lin; /* compact ledge generator edge id (i * 4 + j + 1), 0 = none */
};

struct pc_tri { /* IVP_Triangle */
    pc_tri_t *next;
    pc_tri_t *prev;
    pc_tri_t *other_side;
    pc_tri_t *pierced_triangle;
    char is_hidden;
    double hesse[4]; /* tmp.gen.hesse: k[3] + hesse_val (normized) */
    int index;
    pc_edge_t three_edges[3];
};

typedef struct pc_tetra {
    const ivp_allocator_t *alloc;
    ivp_arena_t *arena;
    const ivp_pc_templ_polygon_t *templ;
    pc_point_t *points;
    int n_points;
    pc_tri_t *first; /* P_List<IVP_Triangle> triangles */
} pc_tetra_t;

static void pc_tri_list_insert(pc_tri_t **first, pc_tri_t *t)
{
    IVP_ASSERT(!t->next);
    t->next = *first;
    if (*first) (*first)->prev = t;
    t->prev = NULL;
    *first = t;
}

static pc_tri_t *pc_new_triangle(pc_tetra_t *pop)
{
    pc_tri_t *t = (pc_tri_t *)ivp_arena_alloc(pop->arena, sizeof(pc_tri_t));
    if (t) memset(t, 0, sizeof(*t));
    return t;
}

/* ── P_Sur_2D (ivp_triangle_gen.hxx) ───────────────────────────────────── */

typedef struct pc2d_line pc2d_line_t;

typedef struct pc2d_point { /* P_Sur_2D_Point (IVP_U_Point, 2D coords) */
    double k[3];
    int point_num;
    pc2d_line_t *line_ref;
} pc2d_point_t;

struct pc2d_line { /* P_Sur_2D_Line */
    pc2d_line_t *next;
    pc2d_line_t *prev;
    pc2d_point_t *start_point;
    pc2d_point_t *end_point;
    double delta_x;
    double delta_y;
};

typedef struct pc2d_tri { /* P_Sur_2D_Triangle */
    struct pc2d_tri *next;
    int point_nums[3];
} pc2d_tri_t;

typedef struct pc2d_cand {
    pc2d_line_t *line;
    double value;
} pc2d_cand_t;

static void pc2d_line_init(pc2d_line_t *l, pc2d_point_t *sp, pc2d_point_t *ep)
{
    l->next = NULL;
    l->prev = NULL;
    l->start_point = sp;
    l->end_point = ep;
    l->delta_x = 0.0;
    l->delta_y = 0.0;
    if (sp && ep) {
        l->delta_x = ep->k[0] - sp->k[0];
        l->delta_y = ep->k[1] - sp->k[1];
    }
}

static void pc2d_list_insert(pc2d_line_t **first, pc2d_line_t *l)
{
    IVP_ASSERT(!l->next);
    l->next = *first;
    if (*first) (*first)->prev = l;
    l->prev = NULL;
    *first = l;
}

static void pc2d_list_remove(pc2d_line_t **first, pc2d_line_t *l)
{
    IVP_ASSERT(l->next != (pc2d_line_t *)-1);
    if (l->prev) {
        l->prev->next = l->next;
    } else {
        *first = l->next;
    }
    if (l->next) l->next->prev = l->prev;
    l->next = (pc2d_line_t *)-1; /* invalid flag !! (P_List::remove) */
    l->prev = NULL;
}

/* P_Sur_2D_Line::hesse_dist_to_point (not normized, may be negative) */
static double pc2d_hesse_dist(const pc2d_line_t *l, const pc2d_point_t *p)
{
    double hesse_dist = l->delta_y * (l->start_point->k[0] - p->k[0]) +
                        l->delta_x * (p->k[1] - l->start_point->k[1]);
    return hesse_dist;
}

/* P_Sur_2D_Line::point_lies_to_the_left */
static int pc2d_point_lies_to_the_left(const pc2d_line_t *l, const pc2d_point_t *p)
{
    double hesse_dist = pc2d_hesse_dist(l, p);
    if (hesse_dist <= PC_P_DOUBLE_EPS) return 0;       /* on the other side */
    if (hesse_dist <= PC_P_POP_TOO_FLAT_EPS) return 2; /* not enough distance */
    return 1;
}

/* P_Sur_2D_Line::has_points */
static int pc2d_has_points(const pc2d_line_t *l, const pc2d_point_t *a, const pc2d_point_t *b)
{
    if (!b) {
        if (l->start_point == a || l->end_point == a) return 1;
        return 0;
    }
    if (l->start_point == a && l->end_point == b) return 1;
    if (l->end_point == a && l->start_point == b) return 1;
    return 0;
}

/* P_Sur_2D_Line::dist_to_point */
static double pc2d_dist_to_point(const pc2d_line_t *l, const pc2d_point_t *p)
{
    double norm = l->delta_y * l->delta_y + l->delta_x * l->delta_x;
    if (norm < PC_P_DOUBLE_EPS) {
        double dx = l->start_point->k[0] - p->k[0];
        double dy = l->start_point->k[1] - p->k[1];
        return dx * dx + dy * dy;
    }
    double hesse_dist = l->delta_y * (l->start_point->k[0] - p->k[0]) +
                        l->delta_x * (p->k[1] - l->start_point->k[1]);
    return hesse_dist / sqrt(norm);
}

/* P_Sur_2D_Line::point_lies_in_interval */
static int pc2d_point_lies_in_interval(const pc2d_line_t *l, const pc2d_point_t *p)
{
    double line_abs_delta = fabs(l->delta_x);
    if (line_abs_delta > PC_P_DOUBLE_EPS) {
        int dx = (l->delta_x > 0) ? 1 : -1;
        double point_delta = (p->k[0] - l->start_point->k[0]) * dx;
        if (point_delta > (double)PC_P_CROSS_EPS &&
            point_delta < line_abs_delta - (double)PC_P_CROSS_EPS) {
            return 1;
        }
        return 0;
    }
    line_abs_delta = fabs(l->delta_y);
    if (line_abs_delta > PC_P_DOUBLE_EPS) {
        int dy = (l->delta_y > 0) ? 1 : -1;
        double point_delta = (p->k[1] - l->start_point->k[1]) * dy;
        if (point_delta > (double)PC_P_CROSS_EPS &&
            point_delta < line_abs_delta - (double)PC_P_CROSS_EPS) {
            return 1;
        }
        return 0;
    }
    return 0;
}

/* P_Sur_2D_Line::overlaps_with_line (lines are on a common line) */
static int pc2d_overlaps_with_line(const pc2d_line_t *line_u, const pc2d_line_t *line_v)
{
    double line_abs_delta = fabs(line_u->delta_x);
    if (line_abs_delta > PC_P_OVERLAP_EPS) {
        double u_lower, u_higher;
        if (line_u->delta_x > 0.0) {
            u_lower = line_u->start_point->k[0];
            u_higher = line_u->end_point->k[0];
        } else {
            u_lower = line_u->end_point->k[0];
            u_higher = line_u->start_point->k[0];
        }
        u_lower += PC_P_OVERLAP_EPS;
        u_higher -= PC_P_OVERLAP_EPS;
        double v_start = line_v->start_point->k[0];
        double v_end = line_v->end_point->k[0];
        if ((v_start <= u_lower && v_end <= u_lower) ||
            (v_start >= u_higher && v_end >= u_higher)) {
            return 0;
        }
        return 1;
    }
    line_abs_delta = fabs(line_u->delta_y);
    if (line_abs_delta > PC_P_OVERLAP_EPS) {
        double u_lower, u_higher;
        if (line_u->delta_y > 0.0) {
            u_lower = line_u->start_point->k[1];
            u_higher = line_u->end_point->k[1];
        } else {
            u_lower = line_u->end_point->k[1];
            u_higher = line_u->start_point->k[1];
        }
        u_lower += PC_P_OVERLAP_EPS;
        u_higher -= PC_P_OVERLAP_EPS;
        double v_start = line_v->start_point->k[1];
        double v_end = line_v->end_point->k[1];
        if ((v_start <= u_lower && v_end <= u_lower) ||
            (v_start >= u_higher && v_end >= u_higher)) {
            return 0;
        }
        return 1;
    }
    /* line_u actually is a point */
    return pc2d_point_lies_in_interval(line_v, line_u->start_point);
}

/* P_Sur_2D_Line::is_crossing_line: 1 if line_u crosses line_v (a common
 * end/start point does not count). */
static int pc2d_is_crossing_line(const pc2d_line_t *line_u, const pc2d_line_t *line_v)
{
    double u_dx = line_u->delta_x;
    double u_dy = line_u->delta_y;
    double v_dx = line_v->delta_x;
    double v_dy = line_v->delta_y;

    double D = v_dx * u_dy - u_dx * v_dy;
    if (fabs(D) < PC_DET_EPS) {
        if (fabs(pc2d_dist_to_point(line_u, line_v->start_point)) < PC_P_CROSS_EPS_QUAD) {
            return pc2d_overlaps_with_line(line_u, line_v) ? 1 : 0;
        }
        return 0; /* really parallel */
    }

    if (line_u->start_point == line_v->end_point || line_u->start_point == line_v->start_point ||
        line_u->end_point == line_v->start_point || line_u->end_point == line_v->end_point) {
        return 0;
    }

    double o_dx = line_v->start_point->k[0] - line_u->start_point->k[0];
    double o_dy = line_v->start_point->k[1] - line_u->start_point->k[1];

    double left_border, right_border;
    if (D > 0.0) {
        left_border = 0.0;
        right_border = D;
    } else {
        left_border = D;
        right_border = 0.0;
    }

    double u_t = v_dx * o_dy - v_dy * o_dx;
    if (u_t < left_border || u_t > right_border) return 0;
    double v_t = u_dx * o_dy - o_dx * u_dy;
    if (v_t < left_border || v_t > right_border) return 0;
    return 1;
}

/* IVP_U_Point::quad_distance_to */
static double pc_quad_distance(const double *a, const double *b)
{
    double x = a[0] - b[0];
    x *= x;
    double y = a[1] - b[1];
    y *= y;
    double z = a[2] - b[2];
    z *= z;
    return x + y + z;
}

typedef struct pc2d_surface {
    pc2d_line_t *lines;      /* P_List<P_Sur_2D_Line> */
    pc2d_tri_t *triangles;   /* P_List<P_Sur_2D_Triangle> */
    int n_lines;             /* orig_surface->n_lines */
} pc2d_surface_t;

/* P_Sur_2D::calc_line_representation.  Returns NULL (IVP_NO_ERROR) or an
 * error string.  The reference's only error ("no orig_surface specified")
 * cannot occur here; the other errors are libivp's checks of the template
 * (the reference would index out of bounds) and allocation failures. */
static const char *pc2d_calc_line_representation(pc_tetra_t *pop, const ivp_pc_templ_surface_t *sur,
                                                 pc2d_surface_t *s2d, pc2d_point_t **point_hash_array)
{
    const double *hesse = sur->normal;
    int max_koord = 0;
    double max_koord_val = fabs(hesse[0]);
    if (fabs(hesse[1]) > max_koord_val) {
        max_koord_val = fabs(hesse[1]);
        max_koord = 1;
    }
    if (fabs(hesse[2]) > max_koord_val) {
        max_koord = 2;
    }

    int td_k0, td_k1;
    switch (max_koord) {
    case 0: td_k0 = 1; td_k1 = 2; break;
    case 1: td_k0 = 0; td_k1 = 2; break;
    case 2: td_k0 = 0; td_k1 = 1; break;
    default:
        td_k0 = td_k1 = 0; /* compiler lullaby */
        IVP_CORE();
    }

    int trafo_is_mirrored = 0;
    if (hesse[max_koord] < 0.0) trafo_is_mirrored = 1;
    if (max_koord == 1) trafo_is_mirrored = 1 - trafo_is_mirrored;

    memset(point_hash_array, 0, (size_t)pop->n_points * sizeof(pc2d_point_t *));
    s2d->lines = NULL;
    s2d->triangles = NULL;
    s2d->n_lines = sur->n_lines;

    int point_count = 0; /* point_array[point_count++] (P_Sur_2D::point_array) */
    for (int i = sur->n_lines - 1; i >= 0; --i) {
        int line_num = sur->lines[i];
        if (line_num < 0 || line_num >= pop->templ->n_lines) return "invalid template line";
        const ivp_pc_templ_line_t *line = &pop->templ->lines[line_num];

        int revert = sur->revert_line[i];
        if (trafo_is_mirrored) revert = 1 - revert;
        if (revert < 0 || revert > 1) return "invalid template line";
        int start_point_num = line->p[revert];
        int end_point_num = line->p[1 - revert];
        if (start_point_num >= pop->n_points || end_point_num >= pop->n_points) return "invalid template line";

        pc2d_point_t *start_point = point_hash_array[start_point_num];
        if (!start_point) {
            start_point = (pc2d_point_t *)ivp_arena_alloc(pop->arena, sizeof(pc2d_point_t));
            if (!start_point) return "out of memory";
            memset(start_point, 0, sizeof(*start_point));
            start_point->point_num = start_point_num;
            point_hash_array[start_point_num] = start_point;
            point_count++;
        }
        start_point->k[0] = pop->points[start_point_num].k[td_k0];
        start_point->k[1] = pop->points[start_point_num].k[td_k1];
        start_point->k[2] = 0.0;

        pc2d_point_t *end_point = point_hash_array[end_point_num];
        if (!end_point) {
            end_point = (pc2d_point_t *)ivp_arena_alloc(pop->arena, sizeof(pc2d_point_t));
            if (!end_point) return "out of memory";
            memset(end_point, 0, sizeof(*end_point));
            end_point->point_num = end_point_num;
            point_hash_array[end_point_num] = end_point;
            point_count++;
        }
        end_point->k[0] = pop->points[end_point_num].k[td_k0];
        end_point->k[1] = pop->points[end_point_num].k[td_k1];
        end_point->k[2] = 0.0;

        pc2d_line_t *td_line = (pc2d_line_t *)ivp_arena_alloc(pop->arena, sizeof(pc2d_line_t));
        if (!td_line) return "out of memory";
        pc2d_line_init(td_line, start_point, end_point);
        start_point->line_ref = td_line;
        IVP_ASSERT(td_line);
        pc2d_list_insert(&s2d->lines, td_line);
    }

    IVP_ASSERT(point_count == sur->n_lines);
    IVP_USE(point_count);
    return NULL; /* IVP_NO_ERROR */
}

/* P_Sur_2D::calc_triangle_representation.  Returns NULL (IVP_NO_ERROR) or
 * an error string: "Cannot convert" like the reference, and libivp's
 * failure return where the reference raises SIGINT via CORE (after the
 * reference print and IVP_CORE()) or an allocation fails. */
static const char *pc2d_calc_triangle_representation(pc_tetra_t *pop, pc2d_surface_t *s2d,
                                                     pc2d_cand_t **cand_buf, int *cand_cap)
{
    int loop_counter = 0;
    pc2d_line_t *td_base_line;

    while ((td_base_line = s2d->lines) != NULL) {
        if (loop_counter++ > PC_TRI_LOOP_LIMIT) {
            for (const pc2d_line_t *td_point_line = s2d->lines; td_point_line; td_point_line = td_point_line->next) {
                const pc2d_point_t *point_c = td_point_line->start_point;
                ivp_debug_printf("  %f %f %f\n", point_c->k[0], point_c->k[1], point_c->k[2]);
            }
            return "Cannot convert";
        }

        /* put points into min_hash: ascending side length, ties in
         * insertion (= lines list) order */
        pc2d_point_t *point_a = td_base_line->start_point;
        pc2d_point_t *point_b = td_base_line->end_point;
        int n_cand = 0;
        for (pc2d_line_t *l = s2d->lines; l; l = l->next) n_cand++;
        if (n_cand > *cand_cap) {
            int ncap = *cand_cap ? *cand_cap : 16;
            while (ncap < n_cand) ncap *= 2;
            pc2d_cand_t *nb = (pc2d_cand_t *)ivp_arena_alloc(pop->arena, (size_t)ncap * sizeof(pc2d_cand_t));
            if (!nb) return "out of memory";
            *cand_buf = nb;
            *cand_cap = ncap;
        }
        pc2d_cand_t *cand = *cand_buf;
        n_cand = 0;
        for (pc2d_line_t *l = s2d->lines; l; l = l->next) {
            const pc2d_point_t *point_c = l->start_point;
            /* p_ab_quad_length */
            double v = pc_quad_distance(point_c->k, point_a->k) + pc_quad_distance(point_c->k, point_b->k);
            int j = n_cand++;
            while (j > 0 && cand[j - 1].value > v) {
                cand[j] = cand[j - 1];
                j--;
            }
            cand[j].line = l;
            cand[j].value = v;
        }

        /* now try triangles in ascending side length order */
        for (int ci = 0; ci < n_cand; ci++) {
            pc2d_line_t *td_point_line = cand[ci].line;
            if (td_point_line == td_base_line) continue;
            if (td_point_line->start_point == td_base_line->end_point) continue;

            pc2d_point_t *point_c = td_point_line->start_point;

            /* point_c must be on the left */
            if (!pc2d_point_lies_to_the_left(td_base_line, point_c)) continue;

            int skip_this_point = 0;
            pc2d_line_t td_ca_line, td_bc_line;
            pc2d_line_init(&td_ca_line, point_c, point_a);
            pc2d_line_init(&td_bc_line, point_b, point_c);

            for (pc2d_line_t *td_cross_line = s2d->lines; td_cross_line; td_cross_line = td_cross_line->next) {
                if (!pc2d_has_points(td_cross_line, point_c, point_a)) {
                    if (pc2d_is_crossing_line(td_cross_line, &td_ca_line)) {
                        skip_this_point = 1;
                        break;
                    }
                }
                if (!pc2d_has_points(td_cross_line, point_b, point_c)) {
                    if (pc2d_is_crossing_line(td_cross_line, &td_bc_line)) {
                        skip_this_point = 1;
                        break;
                    }
                }
            }
            /* reference: "if (!ab_min_hash.find_min_elem()) { printf(Couldn't
             * find a matching point to baseline!); CORE; }": never true there,
             * td_point_line is still the min hash's minimum (the loop
             * increment removes it) */
            if (skip_this_point) continue;

            /* something inside? (having_islands is forced to 1) */
            for (pc2d_line_t *td_inside_line = s2d->lines; td_inside_line; td_inside_line = td_inside_line->next) {
                const pc2d_point_t *sp = td_inside_line->start_point;
                if (pc2d_point_lies_to_the_left(td_base_line, sp) &&
                    pc2d_point_lies_to_the_left(&td_bc_line, sp) &&
                    pc2d_point_lies_to_the_left(&td_ca_line, sp)) {
                    skip_this_point = 1;
                    break;
                }
            }
            if (skip_this_point) {
                if (!td_point_line->next) {
                    ivp_debug_printf("Couldn't find a matching point to baseline!\n");
                    IVP_CORE();
                    /* the reference goes on with the next candidate when
                     * SIGINT is ignored; libivp gives up */
                    return "Couldn't find a matching point to baseline";
                }
                continue;
            }

            /* take triangle */
            pc2d_tri_t *triang = (pc2d_tri_t *)ivp_arena_alloc(pop->arena, sizeof(pc2d_tri_t));
            if (!triang) return "out of memory";
            triang->point_nums[0] = point_a->point_num;
            triang->point_nums[1] = point_c->point_num;
            triang->point_nums[2] = point_b->point_num;
            triang->next = s2d->triangles;
            s2d->triangles = triang;

            /* delete used lines */
            pc2d_list_remove(&s2d->lines, td_base_line);
            int ca_removed = 0;
            int bc_removed = 0;
            pc2d_line_t *td_ident_line_next;
            for (pc2d_line_t *td_ident_line = s2d->lines; td_ident_line; td_ident_line = td_ident_line_next) {
                td_ident_line_next = td_ident_line->next;
                if (pc2d_has_points(td_ident_line, point_c, point_a)) {
                    pc2d_list_remove(&s2d->lines, td_ident_line);
                    ca_removed = 1;
                    td_ident_line = NULL;
                }
                if (td_ident_line && pc2d_has_points(td_ident_line, point_c, point_b)) {
                    pc2d_list_remove(&s2d->lines, td_ident_line);
                    bc_removed = 1;
                }
            }

            /* insert remaining triangle sides as new lines */
            if (!ca_removed) {
                pc2d_line_t *nl = (pc2d_line_t *)ivp_arena_alloc(pop->arena, sizeof(pc2d_line_t));
                if (!nl) return "out of memory";
                pc2d_line_init(nl, point_a, point_c);
                pc2d_list_insert(&s2d->lines, nl);
            }
            if (!bc_removed) {
                pc2d_line_t *nl = (pc2d_line_t *)ivp_arena_alloc(pop->arena, sizeof(pc2d_line_t));
                if (!nl) return "out of memory";
                pc2d_line_init(nl, point_c, point_b);
                pc2d_list_insert(&s2d->lines, nl);
            }
            break; /* next base line */
        }
    }
    return NULL; /* IVP_NO_ERROR */
}

/* ── make_triangles ────────────────────────────────────────────────────── */

typedef struct pc_edge_slot {
    int a, b;          /* point indices, a < b */
    pc_edge_t *edge;
    int used;
} pc_edge_slot_t;

typedef struct pc_edge_hash { /* IVP_Hash keyed by the sorted point pair */
    pc_edge_slot_t *slots;
    int cap;  /* power of two */
    int count;
} pc_edge_hash_t;

static uint32_t pc_edge_hash_fn(int a, int b)
{
    uint32_t h = (uint32_t)a * 0x9E3779B1u ^ ((uint32_t)b + 0x7F4A7C15u) * 0x85EBCA77u;
    h ^= h >> 15;
    return h;
}

static pc_edge_slot_t *pc_edge_hash_lookup(pc_edge_hash_t *h, int a, int b)
{
    uint32_t i = pc_edge_hash_fn(a, b) & (uint32_t)(h->cap - 1);
    for (;;) {
        pc_edge_slot_t *s = &h->slots[i];
        if (!s->used || (s->a == a && s->b == b)) return s;
        i = (i + 1) & (uint32_t)(h->cap - 1);
    }
}

/* ivp_check_for_opposite: the first edge with this point pair is stored,
 * every later one is linked with it (the entry stays).  Returns false when
 * the (fixed size) table is full. */
static bool pc_check_for_opposite(pc_edge_hash_t *h, const pc_tetra_t *pop,
                                  const pc_point_t *p0, const pc_point_t *p1, pc_edge_t *edge)
{
    int i0 = (int)(p0 - pop->points);
    int i1 = (int)(p1 - pop->points);
    int a = i0 < i1 ? i0 : i1;
    int b = i0 < i1 ? i1 : i0;
    if (2 * (h->count + 1) > h->cap) return false; /* keep the probe loop finite */
    pc_edge_slot_t *s = pc_edge_hash_lookup(h, a, b);
    if (!s->used) {
        s->used = 1;
        s->a = a;
        s->b = b;
        s->edge = edge;
        h->count++;
    } else {
        edge->opposite = s->edge;
        s->edge->opposite = edge;
    }
    return true;
}

/* IVP_Object_Polygon_Tetra::make_triangles (visible triangles only) */
static bool pc_make_triangles(pc_tetra_t *pop)
{
    const ivp_pc_templ_polygon_t *templ = pop->templ;
    int total_lines = 0;
    for (int s = 0; s < templ->n_surfaces; s++) total_lines += templ->surfaces[s].n_lines;

    /* undirected edges: at most 3 per triangle, n_lines - 2 triangles per
     * simple face; the table is kept at most half full */
    pc_edge_hash_t hash;
    hash.cap = 16;
    while (hash.cap < 8 * total_lines + 16) hash.cap <<= 1;
    hash.count = 0;
    hash.slots = (pc_edge_slot_t *)ivp_arena_alloc(pop->arena, (size_t)hash.cap * sizeof(pc_edge_slot_t));
    pc2d_point_t **point_hash_array =
        (pc2d_point_t **)ivp_arena_alloc(pop->arena, (size_t)pop->n_points * sizeof(pc2d_point_t *));
    if (!hash.slots || !point_hash_array) return false;
    memset(hash.slots, 0, (size_t)hash.cap * sizeof(pc_edge_slot_t));

    pc2d_cand_t *cand_buf = NULL;
    int cand_cap = 0;

    for (int si = 0; si < templ->n_surfaces; si++) {
        const ivp_pc_templ_surface_t *sur = &templ->surfaces[si];
        if (sur->n_lines <= 0) return false;

        /* split plane into triangles and insert them */
        pc2d_surface_t s2d;
        const char *error = pc2d_calc_line_representation(pop, sur, &s2d, point_hash_array);
        if (error) {
            fprintf(stderr, "make_triangles:calc_line_representation: %s\n", error);
            return false; /* "No 2d representation" */
        }
        error = pc2d_calc_triangle_representation(pop, &s2d, &cand_buf, &cand_cap);
        if (error) {
            fprintf(stderr, "make_triangles:calc_triangle_representation: %s\n", error);
            return false; /* "no 3d representation" */
        }

        for (const pc2d_tri_t *td_tri = s2d.triangles; td_tri; td_tri = td_tri->next) {
            pc_point_t *po = &pop->points[td_tri->point_nums[2]];
            pc_point_t *po2 = &pop->points[td_tri->point_nums[1]];
            pc_point_t *po3 = &pop->points[td_tri->point_nums[0]];

            /* clockwise ? */
            {
                const double *hesse = sur->normal;
                double v0[3], v1[3], cross[3];
                v0[0] = po2->k[0] - po->k[0];
                v0[1] = po2->k[1] - po->k[1];
                v0[2] = po2->k[2] - po->k[2];
                v1[0] = po3->k[0] - po->k[0];
                v1[1] = po3->k[1] - po->k[1];
                v1[2] = po3->k[2] - po->k[2];
                /* IVP_U_Point::inline_calc_cross_product */
                {
                    double a = v0[1] * v1[2];
                    double a2 = v1[1] * v0[2];
                    double b = v0[2] * v1[0];
                    double b2 = v1[2] * v0[0];
                    a -= a2;
                    double c = v0[0] * v1[1];
                    double c2 = v1[0] * v0[1];
                    b -= b2;
                    cross[0] = a;
                    cross[1] = b;
                    c -= c2;
                    cross[2] = c;
                }
                double scal = cross[0] * hesse[0] + cross[1] * hesse[1] + cross[2] * hesse[2];
                if (scal < 0.0) {
                    pc_point_t *h = po3;
                    po3 = po2;
                    po2 = h;
                }
            }

            /* make physical triangle */
            pc_tri_t *tri = pc_new_triangle(pop);
            if (!tri) return false;
            pc_tri_list_insert(&pop->first, tri);
            tri->is_hidden = 0;

            pc_edge_t *edge = &tri->three_edges[0];
            edge->start_point = po;
            edge->triangle = tri;
            edge->next = edge + 1;
            edge->prev = edge + 2;
            if (!pc_check_for_opposite(&hash, pop, po, po2, edge)) return false;

            edge++;
            edge->start_point = po2;
            edge->triangle = tri;
            edge->next = edge + 1;
            edge->prev = edge - 1;
            if (!pc_check_for_opposite(&hash, pop, po2, po3, edge)) return false;

            edge++;
            edge->start_point = po3;
            edge->triangle = tri;
            edge->next = edge - 2;
            edge->prev = edge - 1;
            if (!pc_check_for_opposite(&hash, pop, po3, po, edge)) return false;
        }
    }
    /* The reference now prepends a hidden backside triangle for every
     * triangle (tetra preparation); they never reach the compact ledge. */
    return true;
}

/* IVP_Triangle::calc_hesse: hesse of (p0, p2, p1), normized */
static void pc_tri_calc_hesse(pc_tri_t *tri)
{
    const double *tp0 = tri->three_edges[0].start_point->k;
    const double *tp1 = tri->three_edges[0].prev->start_point->k; /* p2 */
    const double *tp2 = tri->three_edges[0].next->start_point->k; /* p1 */

    /* IVP_U_Hesse::calc_hesse(tp0, tp1, tp2) */
    double fp0 = tp0[0];
    double fp1 = tp0[1];
    double fp2 = tp0[2];
    double a0 = tp1[0] - fp0;
    double a1 = tp1[1] - fp1;
    double a2 = tp1[2] - fp2;
    double b0 = tp2[0] - fp0;
    double b1 = tp2[1] - fp1;
    double b2 = tp2[2] - fp2;
    double c0 = b1 * a2 - a1 * b2;
    double c1 = b2 * a0 - a2 * b0;
    double c2 = b0 * a1 - a0 * b1;
    double *h = tri->hesse;
    h[0] = c0;
    h[1] = c1;
    h[2] = c2;
    h[3] = -(h[0] * fp0 + h[1] * fp1 + h[2] * fp2);

    /* IVP_U_Hesse::normize */
    double len = sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
    if (len < PC_P_DOUBLE_EPS) {
        h[0] = h[1] = h[2] = 0.0;
        h[3] = 0.0;
        return;
    }
    double l = 1.0 / len;
    double m0 = h[0] * l;
    double m1 = h[1] * l;
    double m2 = h[2] * l;
    h[0] = m0;
    h[1] = m1;
    h[2] = m2;
    h[3] *= l;
}

/* IVP_Object_Polygon_Tetra::insert_pierce_info (convex objects) */
static void pc_insert_pierce_info(pc_tetra_t *pop)
{
    for (pc_tri_t *tri = pop->first; tri; tri = tri->next) tri->pierced_triangle = NULL;

    for (pc_tri_t *tri = pop->first; tri; tri = tri->next) {
        if (tri->is_hidden) continue;
        if (tri->pierced_triangle) continue;

        pc_tri_t *found_tri = NULL;
        double min_scal_val = PC_MAX_SCAL_VAL;
        for (pc_tri_t *tri2 = pop->first; tri2; tri2 = tri2->next) {
            if (tri2->is_hidden) continue;
            double scal_val = tri->hesse[0] * tri2->hesse[0] + tri->hesse[1] * tri2->hesse[1] +
                              tri->hesse[2] * tri2->hesse[2];
            if (scal_val < min_scal_val) {
                min_scal_val = scal_val;
                found_tri = tri2;
            }
        }
        /* insert pierce info */
        IVP_ASSERT(found_tri);
        if (!found_tri) continue; /* the reference dereferences it */
        tri->pierced_triangle = found_tri;
        found_tri->pierced_triangle = tri;
    }
}

/* ── IVP_Compact_Ledge_Generator ───────────────────────────────────────── */

/* prepare_compact_ledge + generate_compact_ledge for the visible triangles
 * of the list (fill_list_with_all_triangles) in list order. */
static ivp_compact_ledge_t *pc_generate_compact_ledge(const ivp_allocator_t *alloc,
                                                      pc_tri_t *first, pc_point_t *points,
                                                      int n_points)
{
    int n_triangles = 0;
    for (pc_tri_t *t = first; t; t = t->next) {
        if (!t->is_hidden) n_triangles++;
    }
    if (n_triangles <= 0 || n_triangles > IVP_BUILD_MAX_TRIANGLES_PER_LEDGE) {
        IVP_ASSERT(n_triangles > 0 && n_triangles <= IVP_MAX_TRIANGLES_PER_LEDGE);
        return NULL;
    }

    for (int i = 0; i < n_points; i++) points[i].compact_index = -1;

    /* index triangles, build up point order (first use) */
    int point_cnt = 0;
    {
        int i = 0;
        for (pc_tri_t *tri = first; tri; tri = tri->next) {
            if (tri->is_hidden) continue;
            tri->index = i++;
            pc_edge_t *e = &tri->three_edges[0];
            for (int j = 0; j < 3; e = e->next, j++) {
                if (e->start_point->compact_index < 0) {
                    e->start_point->compact_index = point_cnt++;
                }
                e->lin = 0;
            }
        }
    }
    if (point_cnt >= (1 << 16)) return NULL;

    /* edge ids (edge_hash: i * 4 + j + 1) */
    {
        int i = 0;
        for (pc_tri_t *tri = first; tri; tri = tri->next) {
            if (tri->is_hidden) continue;
            pc_edge_t *e = &tri->three_edges[0];
            for (int j = 0; j < 3; e = e->next, j++) {
                /* edge_hash->find((char *)&edge) == (void *)-1: not yet added */
                IVP_ASSERT(e->lin == 0);
                e->lin = i * 4 + j + 1;
            }
            i++; /* triangle_vec.add(c_tri) */
        }
        /* Triangle info second pass: opposites */
        IVP_ASSERT(n_triangles == i); /* triangle_vec.len() */
    }

    int size = (int)sizeof(ivp_compact_ledge_t) + (int)sizeof(ivp_compact_triangle_t) * n_triangles +
               (int)sizeof(ivp_compact_poly_point_t) * point_cnt;
    ivp_compact_ledge_t *c_ledge = (ivp_compact_ledge_t *)ivp_alloc_aligned(alloc, (size_t)size, 16);
    if (!c_ledge) return NULL;
    memset(c_ledge, 0, (size_t)size);

    ivp_ledge_init(c_ledge);
    c_ledge->n_triangles = (int16_t)n_triangles;
    ivp_compact_triangle_t *ct = ivp_ledge_get_first_triangle_mut(c_ledge);
    {
        int i = 0;
        for (pc_tri_t *tri = first; tri; tri = tri->next) {
            if (tri->is_hidden) continue;
            ivp_compact_triangle_t *c_tri = &ct[i];
            ivp_tri_init(c_tri);
            ivp_tri_set_tri_index(c_tri, tri->index);
            ivp_tri_set_pierce_index(c_tri, tri->pierced_triangle ? tri->pierced_triangle->index : 0);
            ivp_tri_set_material_index(c_tri, 0);
            pc_edge_t *e = &tri->three_edges[0];
            for (int j = 0; j < 3; e = e->next, j++) {
                /* opposites: IVP_Hash lookup, (void *)-1 when not found */
                int opp_index = (e->opposite && e->opposite->lin > 0) ? e->opposite->lin : -1;
                IVP_ASSERT(opp_index >= 0);
                int rel_index = opp_index - (i * 4 + j + 1);
                /* set_opposite_index (ivp_compact_ledge.hxx:63, via ivp_compact_ledge_gen.cxx:132) */
                IVP_ASSERT(rel_index >= -(1 << 14) + 1 && rel_index < (1 << 14) - 1);
                if (opp_index < 0 || rel_index < -(1 << 14) || rel_index >= (1 << 14)) {
                    /* open mesh / oversized ledge: the reference writes a
                     * dangling or truncated index here */
                    ivp_free_aligned(alloc, c_ledge);
                    return NULL;
                }
                ivp_edge_set(&c_tri->edges[j], e->start_point->compact_index, rel_index, 0);
            }
            i++;
        }
        /* generate_compact_ledge: copy triangle info */
        IVP_ASSERT(n_triangles == i); /* triangle_vec.len() */
    }

    /* copy point info */
    IVP_IF(1) {
        int n_point_vec = 0; /* point_vec.len() */
        for (int i = 0; i < n_points; i++) {
            if (points[i].compact_index >= 0) n_point_vec++;
        }
        IVP_ASSERT(point_cnt == n_point_vec);
        IVP_USE(n_point_vec);
    }
    {
        /* c_ledge->set_offset_ledge_points(offset) (ivp_compact_ledge.hxx:173) */
        int32_t offset =
            (int32_t)(sizeof(ivp_compact_ledge_t) + (size_t)n_triangles * sizeof(ivp_compact_triangle_t));
        IVP_ASSERT((offset & 3) == 0);
        c_ledge->c_point_offset = offset;
    }
    ivp_compact_poly_point_t *pa = ivp_ledge_get_point_array_mut(c_ledge);
    for (int i = 0; i < n_points; i++) {
        int ci = points[i].compact_index;
        if (ci < 0) continue;
        ivp_compact_poly_point_t *cpp = &pa[ci];
        cpp->hesse_val = 0.0f;
        cpp->k[0] = (float)points[i].k[0];
        cpp->k[1] = (float)points[i].k[1];
        cpp->k[2] = (float)points[i].k[2];
    }
    ivp_ledge_set_is_compact(c_ledge, 1);
    ivp_ledge_set_size(c_ledge, size);
    return c_ledge;
}

#ifdef IVP_DEBUG_BUILD
/* IVP_U_Point::quad_distance_to(const IVP_U_Float_Point *) */
static double pc_debug_qdist(const double *a, const float *b)
{
    double x = a[0] - (double)b[0];
    x *= x;
    double y = a[1] - (double)b[1];
    y *= y;
    double z = a[2] - (double)b[2];
    z *= z;
    return x + y + z;
}

/* IVP_Compact_Ledge_Generator::validate (ivp_compact_ledge_gen.cxx:186,
 * #ifdef DEBUG): checks whether the compact ledge has any similarity with
 * the original version.  orig_triangles = the visible triangles of the list
 * (fill_list_with_all_triangles), in list order. */
static bool pc_validate(const ivp_compact_ledge_t *compact_ledge, pc_tri_t *first)
{
    int n_triangles = 0;
    for (pc_tri_t *t = first; t; t = t->next) {
        if (!t->is_hidden) n_triangles++;
    }
    IVP_ASSERT(compact_ledge->n_triangles == n_triangles);

    /* all triangles */
    const ivp_compact_triangle_t *c_triangles = ivp_ledge_get_first_triangle(compact_ledge);
    const ivp_compact_poly_point_t *pa = ivp_ledge_get_point_array(compact_ledge);
    int i = 0;
    for (pc_tri_t *tri = first; tri; tri = tri->next) {
        if (tri->is_hidden) continue;
        const ivp_compact_triangle_t *c_tri = &c_triangles[i];

        /* check tri info */
        IVP_ASSERT(ivp_tri_get_tri_index(c_tri) == i);
        if (tri->pierced_triangle) {
            IVP_ASSERT(ivp_tri_get_pierce_index(c_tri) == tri->pierced_triangle->index);
        }
        IVP_ASSERT(ivp_tri_get_material_index(c_tri) == 0);

        /* all tri edges */
        pc_edge_t *edge = &tri->three_edges[0];
        for (int j = 0; j < 3; edge = edge->next, j++) {
            const ivp_compact_edge_t *c_edge = &c_tri->edges[j];

            /* edge_hash->find(&edge->opposite): (void *)-1 when not found */
            intptr_t opp_index = (edge->opposite && edge->opposite->lin > 0) ? edge->opposite->lin : -1;
            IVP_ASSERT(opp_index >= 0);
            int rel_index = (int)opp_index - (i * (int)sizeof(ivp_compact_edge_t) + j + 1);
            IVP_ASSERT(rel_index == ivp_edge_get_opposite_index(c_edge));
            IVP_USE(rel_index);

            const ivp_compact_poly_point_t *c_po = &pa[ivp_edge_get_start_point_index(ivp_edge_get_opposite(c_edge))];
            IVP_ASSERT(pc_debug_qdist(edge->opposite->start_point->k, c_po->k) < (double)1e-3f);

            IVP_ASSERT(edge->opposite->opposite == edge);
            IVP_ASSERT(ivp_edge_get_opposite(ivp_edge_get_opposite(c_edge)) == c_edge);

            IVP_ASSERT(ivp_edge_get_compact_ledge(c_edge) == compact_ledge);
            const ivp_compact_poly_point_t *c_point = &pa[ivp_edge_get_start_point_index(c_edge)];
            IVP_ASSERT(pc_debug_qdist(edge->start_point->k, c_point->k) < (double)1e-3f);

            c_po = &pa[ivp_edge_get_start_point_index(ivp_edge_get_next(c_edge))];
            IVP_ASSERT(pc_debug_qdist(edge->next->start_point->k, c_po->k) < (double)1e-3f);
            IVP_USE(c_po);
            IVP_USE(c_point);
        }
        i++;
    }
    return true; /* IVP_OK */
}
#endif /* IVP_DEBUG_BUILD */

/* ── Public (internal) entry points ────────────────────────────────────── */

ivp_compact_ledge_t *ivp_pc_template_to_ledge(const ivp_allocator_t *alloc,
                                              const ivp_pc_templ_polygon_t *templ)
{
    if (!templ || templ->n_points <= 0 || templ->n_surfaces <= 0) return NULL;
    if (!alloc) alloc = ivp_default_allocator();

    pc_tetra_t pop;
    memset(&pop, 0, sizeof(pop));
    pop.alloc = alloc;
    pop.templ = templ;
    pop.n_points = templ->n_points;
    pop.arena = ivp_arena_create(alloc);
    if (!pop.arena) return NULL;

    ivp_compact_ledge_t *ledge = NULL;
    pop.points = (pc_point_t *)ivp_arena_alloc(pop.arena, (size_t)pop.n_points * sizeof(pc_point_t));
    if (!pop.points) goto done;
    for (int i = 0; i < pop.n_points; i++) {
        pop.points[i].k[0] = templ->points[3 * i + 0];
        pop.points[i].k[1] = templ->points[3 * i + 1];
        pop.points[i].k[2] = templ->points[3 * i + 2];
        pop.points[i].compact_index = 0;
    }

    /* init_surface_manager_polygon */
    if (!pc_make_triangles(&pop)) goto done;
    for (pc_tri_t *tri = pop.first; tri; tri = tri->next) pc_tri_calc_hesse(tri);
    pc_insert_pierce_info(&pop);
    ledge = pc_generate_compact_ledge(alloc, pop.first, pop.points, pop.n_points);
#ifdef IVP_DEBUG_BUILD
    if (ledge && !pc_validate(ledge, pop.first)) {
        ivp_debug_printf("Compact ledge generation fizzled :-(\n");
    }
#endif

done:
    ivp_arena_destroy(pop.arena);
    return ledge;
}

ivp_compact_ledge_t *ivp_pc_double_triangle_ledge(const ivp_allocator_t *alloc,
                                                  const double p0[3],
                                                  const double p1[3],
                                                  const double p2[3])
{
    if (!alloc) alloc = ivp_default_allocator();

    /* IVP_SurfaceBuilder_Polygon_Convex(IVP_Template_Ledge_Polygon_Soup *):
     * generate_double_triangle(p1, p2, p3), pierced triangles = each other,
     * tri_vec = { tri, tri->other_side }. */
    pc_point_t pts[3];
    memset(pts, 0, sizeof(pts));
    for (int k = 0; k < 3; k++) {
        pts[0].k[k] = p0[k];
        pts[1].k[k] = p1[k];
        pts[2].k[k] = p2[k];
    }
    pc_tri_t tri_a, i_tri_a;
    memset(&tri_a, 0, sizeof(tri_a));
    memset(&i_tri_a, 0, sizeof(i_tri_a));

    pc_edge_t *e0 = &tri_a.three_edges[0];
    pc_edge_t *e1 = &tri_a.three_edges[1];
    pc_edge_t *e2 = &tri_a.three_edges[2];
    pc_edge_t *i_e0 = &i_tri_a.three_edges[0];
    pc_edge_t *i_e1 = &i_tri_a.three_edges[1];
    pc_edge_t *i_e2 = &i_tri_a.three_edges[2];

    e0->next = e1; e0->prev = e2; e0->start_point = &pts[0]; e0->triangle = &tri_a;
    e1->next = e2; e1->prev = e0; e1->start_point = &pts[1]; e1->triangle = &tri_a;
    e2->next = e0; e2->prev = e1; e2->start_point = &pts[2]; e2->triangle = &tri_a;

    i_e0->next = i_e2; i_e0->prev = i_e1; i_e0->start_point = &pts[1]; i_e0->triangle = &i_tri_a;
    i_e1->next = i_e0; i_e1->prev = i_e2; i_e1->start_point = &pts[2]; i_e1->triangle = &i_tri_a;
    i_e2->next = i_e1; i_e2->prev = i_e0; i_e2->start_point = &pts[0]; i_e2->triangle = &i_tri_a;

    tri_a.other_side = &i_tri_a;
    i_tri_a.other_side = &tri_a;

    /* p_link_triangle_self: opposite = IVP_Tri_Edge::other_side(), the first
     * edge of the other side (in array order) whose next edge starts at
     * this edge's start point */
    pc_tri_t *both[2] = { &tri_a, &i_tri_a };
    for (int t = 0; t < 2; t++) {
        pc_tri_t *tri = both[t];
        for (int i = 0; i < 3; i++) {
            pc_edge_t *e = &tri->three_edges[i];
            pc_edge_t *o = &tri->other_side->three_edges[0];
            pc_edge_t *found = NULL;
            for (int k = 2; k >= 0; k--, o++) {
                if (o->next && o->next->start_point == e->start_point) {
                    found = o;
                    break;
                }
            }
            e->opposite = found;
        }
    }

    tri_a.pierced_triangle = &i_tri_a;
    i_tri_a.pierced_triangle = &tri_a;
    tri_a.next = &i_tri_a;
    i_tri_a.prev = &tri_a;

    ivp_compact_ledge_t *ledge = pc_generate_compact_ledge(alloc, &tri_a, pts, 3);
#ifdef IVP_DEBUG_BUILD
    if (ledge && !pc_validate(ledge, &tri_a)) {
        ivp_debug_printf("Compact ledge generation fizzled :-(\n");
    }
#endif
    return ledge;
}
