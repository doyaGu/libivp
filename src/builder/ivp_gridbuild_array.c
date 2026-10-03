/* ivp_gridbuild_array.c — Heightfield -> compact grid (IVP_GridBuilder_Array)
 *
 * Port of ivp_surface_manager/ivp_gridbuild_array.cxx: every row strip of
 * the heightfield is scanned for maximal convex runs (flat runs of at least
 * three coplanar triangles are merged), each run becomes one non-compact
 * compact ledge (a 2 triangle "triangle" ledge, a 4 triangle "square"
 * ledge, or an extruded convex stripe closed by two reference points one
 * grid size below the lowest height), and every grid square references at
 * most two ledges.
 *
 * Kept reference quirks: unreferenced grid squares point at ledge 0 (the
 * reference field is calloc'ed), and the stripe pierce indices store the
 * loop counter u instead of the triangle index.
 * The compiled blob has the reference layout (see ivp_compact_grid.h),
 * including the 4-byte aligned shared point array and byte_size's
 * worst-case padding; bytes the reference leaves uninitialised are zero
 * (the grid points' hesse_val included).
 */
#include <ivp/builder/ivp_build_grid.h>
#include <ivp/geometry/ivp_compact_grid.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_triangle.h>
#include <ivp/base/ivp_assert.h>

#include "geometry/ivp_check_ledge_debug.h"

#include <stddef.h>
#include <string.h>
#include <math.h>

void ivp_template_compact_grid_init(ivp_template_compact_grid_t *t)
{
    memset(t, 0, sizeof(*t));
    t->height_maps_to = 0; /* IVP_INDEX_X */
    t->height_invert_axis = false;
    t->grid_field_size = 0.0f;
}

typedef struct gb {
    const ivp_allocator_t *alloc;
    int n_rows, n_cols;
    const float *height_field;
    ivp_compact_poly_point_t *height_points;
    bool is_left_handed;
    ivp_compact_grid_element_t *ledge_reference_field;
    const ivp_template_compact_grid_t *templ;
    int *grid_point_to_ledge_point_array;
    ivp_compact_poly_point_t *buffer;
    int n_used;
    int buffer_cap;

    ivp_compact_ledge_t *c_ledge;
    ivp_compact_poly_point_t *c_points;
    int c_point_to_point_index[IVP_GRID_MAX_ROWS * 2 + 2];
    int triangle_count;

    ivp_compact_ledge_t **ledges;
    int *ledge_base;   /* index of the ledge's first point in buffer */
    int n_ledges;
    int cap_ledges;
    bool failed;
} gb_t;

/* ── helpers ───────────────────────────────────────────────────────────── */

static int install_grid_point(gb_t *b, int grid_point_index)
{
    int index = b->grid_point_to_ledge_point_array[grid_point_index];
    if (index >= 0) return index;
    index = b->n_used++;
    b->grid_point_to_ledge_point_array[grid_point_index] = index;
    b->buffer[index] = b->height_points[grid_point_index];
    IVP_IF(1) {
        ivp_compact_poly_point_t *cpp = &b->buffer[index];
        intptr_t adress = (intptr_t)cpp;
        IVP_ASSERT((adress & 15) == 0);
        IVP_USE(adress);
    }
    return index;
}

static int install_point(gb_t *b, const float p[3])
{
    int index = b->n_used++;
    IVP_IF(1) {
        ivp_compact_poly_point_t *cpp = &b->buffer[index];
        intptr_t adress = (intptr_t)cpp;
        IVP_ASSERT((adress & 15) == 0);
        IVP_USE(adress);
    }
    b->buffer[index].k[0] = p[0];
    b->buffer[index].k[1] = p[1];
    b->buffer[index].k[2] = p[2];
    b->buffer[index].hesse_val = 0.0f;
    return index;
}

static ivp_compact_ledge_t *new_ledge(gb_t *b, int num_triangles)
{
    int mem_size = (int)sizeof(ivp_compact_ledge_t) + num_triangles * (int)sizeof(ivp_compact_triangle_t);
    ivp_compact_ledge_t *l = (ivp_compact_ledge_t *)ivp_alloc_aligned(b->alloc, (size_t)mem_size, 16);
    if (!l) {
        b->failed = true;
        return NULL;
    }
    memset(l, 0, (size_t)mem_size);
    IVP_ASSERT(((intptr_t)l & 15) == 0); /* make sure it is aligned */
    ivp_ledge_set_is_compact(l, 0);
    ivp_ledge_set_size(l, mem_size);
    l->n_triangles = (int16_t)num_triangles;
    ivp_ledge_set_has_children(l, 0);
    b->c_ledge = l;
    b->triangle_count = 0;
    return l;
}

/* debug convexity check of add_triangle / insert_opposite_index:
 * hesse.calc_hesse(start(edge), start(next(edge)), start(prev(edge))) and
 * hesse.get_dist(start(prev(oppo))), IVP_U_Point (double) arithmetic */
static double gb_debug_hesse_dist(const gb_t *b, const ivp_compact_edge_t *edge,
                                  const ivp_compact_edge_t *oppo)
{
    const ivp_compact_poly_point_t *pa = b->c_points;
    const float *tp0 = pa[ivp_edge_get_start_point_index(edge)].k;
    const float *tp1 = pa[ivp_edge_get_start_point_index(ivp_edge_get_next(edge))].k;
    const float *tp2 = pa[ivp_edge_get_start_point_index(ivp_edge_get_prev(edge))].k;
    const float *pp = pa[ivp_edge_get_start_point_index(ivp_edge_get_prev(oppo))].k;
    double fp0 = (double)tp0[0], fp1 = (double)tp0[1], fp2 = (double)tp0[2];
    double a0 = (double)tp1[0] - fp0, a1 = (double)tp1[1] - fp1, a2 = (double)tp1[2] - fp2;
    double b0 = (double)tp2[0] - fp0, b1 = (double)tp2[1] - fp1, b2 = (double)tp2[2] - fp2;
    double c0 = b1 * a2 - a1 * b2;
    double c1 = b2 * a0 - a2 * b0;
    double c2 = b0 * a1 - a0 * b1;
    double hesse_val = -(c0 * fp0 + c1 * fp1 + c2 * fp2);
    return c0 * (double)pp[0] + c1 * (double)pp[1] + c2 * (double)pp[2] + hesse_val;
}

static void add_triangle(gb_t *b, int s0, int s1, int s2, int opp0, int opp1, int opp2)
{
    ivp_compact_triangle_t *tri = &((ivp_compact_triangle_t *)b->c_ledge)[b->triangle_count + 1];
    ivp_tri_set_tri_index(tri, b->triangle_count++);
    ivp_edge_set_start_point_index(&tri->edges[0], b->c_point_to_point_index[s0]);
    ivp_edge_set_start_point_index(&tri->edges[1], b->c_point_to_point_index[s1]);
    ivp_edge_set_start_point_index(&tri->edges[2], b->c_point_to_point_index[s2]);
    ivp_edge_set_opposite_index(&tri->edges[0], opp0);
    ivp_edge_set_opposite_index(&tri->edges[1], opp1);
    ivp_edge_set_opposite_index(&tri->edges[2], opp2);

    IVP_IF(1) {
        /* check for opposite and convexity consistency */
        int opp[3];
        opp[0] = opp0;
        opp[1] = opp1;
        opp[2] = opp2;

        for (int e = 0; e < 3; e++) {
            const ivp_compact_edge_t *edge, *oppo;
            edge = &tri->edges[e];
            oppo = ivp_edge_get_opposite(edge);

            if ((opp[e] < 0) && (opp[e] > -1000) && (ivp_edge_get_opposite_index(oppo) > -1000)) {
                IVP_ASSERT(edge == ivp_edge_get_opposite(oppo));

                /* check for convexity */
                double dist = gb_debug_hesse_dist(b, edge, oppo);
                IVP_ASSERT(dist >= (double)-0.001f); /*- P_RES_EPS); */
                IVP_USE(dist);
            }
        }
    }
}

/* IVP_GridBuilder_Array::insert_opposite_index */
static void insert_opposite_index(gb_t *b, const ivp_compact_edge_t *edge, int index)
{
    ivp_edge_set_opposite_index((ivp_compact_edge_t *)edge, index);

    IVP_IF(1) {
        const ivp_compact_edge_t *oppo;
        oppo = ivp_edge_get_opposite(edge);
        if (ivp_edge_get_triangle(oppo) - ivp_ledge_get_first_triangle(b->c_ledge) >= b->triangle_count)
            return; /* not in list yet */
        IVP_ASSERT(edge == ivp_edge_get_opposite(oppo));

        /* check for convexity */
        double dist = gb_debug_hesse_dist(b, edge, oppo);
        IVP_ASSERT(dist >= (double)-1e-12f);
        IVP_USE(dist);
    }
}

static ivp_compact_triangle_t *ledge_tri(gb_t *b, int i)
{
    return &ivp_ledge_get_first_triangle_mut(b->c_ledge)[i];
}

static const ivp_compact_poly_point_t *strip_pos(gb_t *b, int pos)
{
    return &b->c_points[b->c_point_to_point_index[pos + 2]];
}

/* IVP_U_Hesse::calc_hesse(float, float, float) + get_dist(float) */
static double hesse_dist(const ivp_compact_poly_point_t *tp0, const ivp_compact_poly_point_t *tp1,
                         const ivp_compact_poly_point_t *tp2, const ivp_compact_poly_point_t *p)
{
    double fp0 = (double)tp0->k[0], fp1 = (double)tp0->k[1], fp2 = (double)tp0->k[2];
    double a0 = (double)tp1->k[0] - fp0, a1 = (double)tp1->k[1] - fp1, a2 = (double)tp1->k[2] - fp2;
    double b0 = (double)tp2->k[0] - fp0, b1 = (double)tp2->k[1] - fp1, b2 = (double)tp2->k[2] - fp2;
    double c0 = b1 * a2 - a1 * b2;
    double c1 = b2 * a0 - a2 * b0;
    double c2 = b0 * a1 - a0 * b1;
    double hv = -(c0 * fp0 + c1 * fp1 + c2 * fp2);
    return c0 * (double)p->k[0] + c1 * (double)p->k[1] + c2 * (double)p->k[2] + hv;
}

static void set_points(gb_t *b, int base)
{
    b->c_points = &b->buffer[base];
    {
        /* c_ledge->set_offset_ledge_points(c_points - c_ledge): the
         * reference asserts the alignment of the offset
         * (ivp_compact_ledge.hxx:173); libivp stores the offset only when
         * the grid is compiled (the work ledges and the point buffer are
         * separate allocations here) */
        intptr_t offset = (intptr_t)((uintptr_t)b->c_points - (uintptr_t)b->c_ledge);
        IVP_ASSERT((offset & 3) == 0);
        IVP_USE(offset);
    }
}

/* ── convert_convex_triangle_to_compact_ledge ──────────────────────────── */

static ivp_compact_ledge_t *convert_triangle(gb_t *b, const int strip_points[], int *base_out)
{
    if (!new_ledge(b, 2)) return NULL;
    int min = IVP_GRID_MAX_ROWS * IVP_GRID_MAX_ROWS * 3;
    int max = 0;
    for (int i = 2; i >= 0; i--) {
        int index = install_grid_point(b, strip_points[i]);
        if (index < min) min = index;
        if (index > max) max = index;
        b->c_point_to_point_index[i] = index;
    }
    for (int i = 0; i < 3; i++) b->c_point_to_point_index[i] -= min;
    IVP_ASSERT(max - min + 1 < b->n_cols * 4 + 2 * b->n_cols);
    IVP_USE(max);
    set_points(b, min);
    *base_out = min;

    add_triangle(b, 0, 1, 2, 6, 4, 2);
    add_triangle(b, 0, 2, 1, -2, -4, -6);
    ivp_tri_set_pierce_index(ledge_tri(b, 0), 1);
    ivp_tri_set_pierce_index(ledge_tri(b, 1), 0);

#ifdef IVP_DEBUG_BUILD
    ivp_debug_check_ledge_points(b->c_ledge, b->c_points);
#endif
    return b->c_ledge;
}

/* ── convert_convex_square_to_compact_ledge ────────────────────────────── */

static ivp_compact_ledge_t *convert_square(gb_t *b, const int strip_points[], bool is_right_starter,
                                           int *base_out)
{
    if (!new_ledge(b, 4)) return NULL;
    int min = IVP_GRID_MAX_ROWS * IVP_GRID_MAX_ROWS * 3;
    int max = 0;
    for (int i = 3; i >= 0; i--) {
        int index = install_grid_point(b, strip_points[i]);
        if (index < min) min = index;
        if (index > max) max = index;
        b->c_point_to_point_index[i] = index;
    }
    for (int i = 0; i < 4; i++) b->c_point_to_point_index[i] -= min;
    IVP_ASSERT(max - min + 1 < b->n_cols * 4 + 2 * b->n_cols);
    IVP_USE(max);
    set_points(b, min);
    *base_out = min;

    int p1 = 1, p2 = 2;
    if (is_right_starter) {
        p1 = 2;
        p2 = 1;
    }
    add_triangle(b, p2, p1, 0, 4, 8, 12);
    add_triangle(b, p1, p2, 3, -4, 8, 4);
    add_triangle(b, 3, 0, p1, 4, -8, -4);
    add_triangle(b, 0, 3, p2, -4, -8, -12);
    ivp_tri_set_pierce_index(ledge_tri(b, 0), 2);
    ivp_tri_set_pierce_index(ledge_tri(b, 1), 2);
    ivp_tri_set_pierce_index(ledge_tri(b, 2), 0);
    ivp_tri_set_pierce_index(ledge_tri(b, 3), 0);

#ifdef IVP_DEBUG_BUILD
    ivp_debug_check_ledge_points(b->c_ledge, b->c_points);
#endif
    return b->c_ledge;
}

/* ── convert_convex_stripe_to_compact_ledge_fast ───────────────────────── */

static ivp_compact_ledge_t *convert_stripe(gb_t *b, const int strip_points[], int n_points,
                                           const float ref_point[2][3], bool is_right_starter,
                                           int *base_out)
{
    IVP_ASSERT(n_points >= 4);
    int num_triangles = 4 + 2 * (n_points - 2);
    if (!new_ledge(b, num_triangles)) return NULL;

    {
        int min = b->c_point_to_point_index[0] = install_point(b, ref_point[0]);
        int max = b->c_point_to_point_index[1] = install_point(b, ref_point[1]);
        if (min > max) {
            int h = max;
            max = min;
            min = h;
        }
        for (int i = n_points - 1; i >= 0; i--) {
            int index = install_grid_point(b, strip_points[i]);
            if (index < min) min = index;
            if (index > max) max = index;
            b->c_point_to_point_index[i + 2] = index;
        }
        for (int j = n_points + 1; j >= 0; j--) b->c_point_to_point_index[j] -= min;
        IVP_ASSERT(max - min + 1 < b->n_cols * 4 + 1);
        set_points(b, min);
        *base_out = min;
    }

    int cp = 2;
    int end_point_idx = n_points - 1;
    int last_opp = 0;
    int left_off, mid_off, right_off;

    {
        double dist = hesse_dist(strip_pos(b, -1), strip_pos(b, -2), strip_pos(b, 0), strip_pos(b, 1));
        if (is_right_starter) {
            if (dist > 0.0) {
                add_triangle(b, 3, 2, 0, 8, 5, 19);
                add_triangle(b, 2, 1, 0, 10, -1000, -5);
                left_off = -19;
                mid_off = -8;
                right_off = -10;
            } else {
                add_triangle(b, 3, 2, 1, 8, 13, 2);
                add_triangle(b, 3, 1, 0, -2, -1000, 15);
                left_off = -15;
                mid_off = -8;
                right_off = -13;
            }
            goto lower_right;
        } else {
            if (dist < 0.0) {
                add_triangle(b, 3, 0, 2, 6, 12, 6);
                add_triangle(b, 3, 1, 0, 18, -1000, -6);
                left_off = -12;
                mid_off = -6;
                right_off = -18;
            } else {
                add_triangle(b, 2, 3, 1, 8, 21, 2);
                add_triangle(b, 2, 1, 0, -2, -1000, 7);
                left_off = -7;
                mid_off = -8;
                right_off = -21;
            }
        }
    }

    while (1) {
        /* upper left plus left neighbour */
        if (cp < end_point_idx) {
            add_triangle(b, cp + 1, cp, cp + 2, mid_off, 3, 6);
            mid_off = -6;
            add_triangle(b, cp + 2, cp, 0, -3, left_off, 15);
            left_off = -15;
            cp++;
        } else {
            double dist = hesse_dist(strip_pos(b, -2), strip_pos(b, -1), strip_pos(b, cp + 1 - 2),
                                     strip_pos(b, cp));
            if (dist < 0.0) {
                insert_opposite_index(b, &ledge_tri(b, b->triangle_count - 1)->edges[1], 13);
                add_triangle(b, cp + 1, cp, cp + 2, mid_off, 3, 6);
                add_triangle(b, cp + 2, cp, 0, -3, left_off, 7);
                add_triangle(b, cp + 1, cp + 2, 1, -6, 3, -13);
                last_opp = -1 - (num_triangles - 2) * 4;
                add_triangle(b, 1, cp + 2, 0, -3, -7, last_opp);
            } else {
                insert_opposite_index(b, &ledge_tri(b, b->triangle_count - 1)->edges[1], 17);
                add_triangle(b, cp + 1, cp, cp + 2, mid_off, 3, 6);
                add_triangle(b, cp + 2, cp, 0, -3, left_off, 3);
                add_triangle(b, cp + 1, cp + 2, 0, -6, -3, 2);
                last_opp = -(num_triangles - 2) * 4;
                add_triangle(b, cp + 1, 0, 1, -2, last_opp, -17);
            }
            break;
        }
    lower_right:
        /* lower right plus right neighbour */
        if (cp < end_point_idx) {
            add_triangle(b, cp, cp + 1, cp + 2, mid_off, 7, 2);
            mid_off = -7;
            add_triangle(b, cp, cp + 2, 1, -2, 17, right_off);
            right_off = -17;
            cp++;
        } else {
            double dist = hesse_dist(strip_pos(b, -2), strip_pos(b, -1), strip_pos(b, cp + 1 - 2),
                                     strip_pos(b, cp));
            if (dist > 0.0) {
                insert_opposite_index(b, &ledge_tri(b, b->triangle_count - 1)->edges[2], 15);
                add_triangle(b, cp, cp + 1, cp + 2, mid_off, 7, 2);
                add_triangle(b, cp, cp + 2, 1, -2, 5, right_off);
                add_triangle(b, cp + 2, cp + 1, 1, -7, 3, -5);
                last_opp = -1 - (num_triangles - 2) * 4;
                add_triangle(b, 1, cp + 1, 0, -3, -15, last_opp);
            } else {
                insert_opposite_index(b, &ledge_tri(b, b->triangle_count - 1)->edges[2], 11);
                add_triangle(b, cp, cp + 1, cp + 2, mid_off, 7, 2);
                add_triangle(b, cp, cp + 2, 1, -2, 9, right_off);
                add_triangle(b, cp + 2, cp + 1, 0, -7, -11, 2);
                last_opp = -(num_triangles - 2) * 4;
                add_triangle(b, cp + 2, 0, 1, -2, last_opp, -9);
            }
            break;
        }
    }

    /* link last with first triangle */
    {
        IVP_ASSERT(last_opp);
        IVP_ASSERT(ivp_tri_get_tri_index(ledge_tri(b, 1)) == 1);
        insert_opposite_index(b, &ledge_tri(b, 1)->edges[1], -last_opp);
    }

    /* pierce info (reference: pierce index = loop counter u) */
    {
        int found_cnt = 0;
        const double take_it_eps = (double)0.01f;
        for (int t = num_triangles - 1, ti = 0; t >= 0; t--, ti++) {
            ivp_compact_triangle_t *tri = ledge_tri(b, ti);
            double n1[3];
            {
                const float *a = b->c_points[ivp_edge_get_start_point_index(&tri->edges[0])].k;
                const float *c1 = b->c_points[ivp_edge_get_start_point_index(&tri->edges[1])].k;
                const float *c2 = b->c_points[ivp_edge_get_start_point_index(&tri->edges[2])].k;
                double x0 = (double)c1[0] - (double)a[0], x1 = (double)c1[1] - (double)a[1], x2 = (double)c1[2] - (double)a[2];
                double y0 = (double)c2[0] - (double)a[0], y1 = (double)c2[1] - (double)a[1], y2 = (double)c2[2] - (double)a[2];
                n1[0] = y1 * x2 - x1 * y2;
                n1[1] = y2 * x0 - x2 * y0;
                n1[2] = y0 * x1 - x0 * y1;
            }
            double max_h = -1.0;
            int max_tri_num = -1;
            bool found = false;
            for (int u = num_triangles - 1, ui = 0; u >= 0; u--, ui++) {
                if (t == u) continue;
                const ivp_compact_triangle_t *tri2 = ledge_tri(b, ui);
                const float *a = b->c_points[ivp_edge_get_start_point_index(&tri2->edges[0])].k;
                const float *c1 = b->c_points[ivp_edge_get_start_point_index(&tri2->edges[1])].k;
                const float *c2 = b->c_points[ivp_edge_get_start_point_index(&tri2->edges[2])].k;
                double x0 = (double)c1[0] - (double)a[0], x1 = (double)c1[1] - (double)a[1], x2 = (double)c1[2] - (double)a[2];
                double y0 = (double)c2[0] - (double)a[0], y1 = (double)c2[1] - (double)a[1], y2 = (double)c2[2] - (double)a[2];
                double n2[3] = { y1 * x2 - x1 * y2, y2 * x0 - x2 * y0, y0 * x1 - x0 * y1 };
                double h = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];
                if (h > take_it_eps) {
                    ivp_tri_set_pierce_index(tri, u);
                    found_cnt++;
                    found = true;
                    break;
                }
                if (h > max_h) {
                    max_h = h;
                    max_tri_num = u;
                }
            }
            if (!found) {
                IVP_ASSERT(max_tri_num >= 0);
                if (max_tri_num >= 0) ivp_tri_set_pierce_index(tri, max_tri_num);
                found_cnt++;
            }
        }
        IVP_ASSERT(found_cnt == num_triangles);
        IVP_USE(found_cnt);
    }

#ifdef IVP_DEBUG_BUILD
    ivp_debug_check_ledge_points(b->c_ledge, b->c_points);
#endif
    return b->c_ledge;
}

/* ── strips ────────────────────────────────────────────────────────────── */

static bool add_ledge(gb_t *b, ivp_compact_ledge_t *cl, int base)
{
    if (b->n_ledges >= b->cap_ledges) {
        int nc = b->cap_ledges ? b->cap_ledges * 2 : 64;
        ivp_compact_ledge_t **nl = (ivp_compact_ledge_t **)ivp_alloc(b->alloc, (size_t)nc * sizeof(*nl));
        int *nb = (int *)ivp_alloc(b->alloc, (size_t)nc * sizeof(int));
        if (!nl || !nb) {
            if (nl) ivp_free(b->alloc, nl, (size_t)nc * sizeof(*nl));
            if (nb) ivp_free(b->alloc, nb, (size_t)nc * sizeof(int));
            return false;
        }
        if (b->ledges) {
            memcpy(nl, b->ledges, (size_t)b->n_ledges * sizeof(*nl));
            memcpy(nb, b->ledge_base, (size_t)b->n_ledges * sizeof(int));
            ivp_free(b->alloc, b->ledges, (size_t)b->cap_ledges * sizeof(*nl));
            ivp_free(b->alloc, b->ledge_base, (size_t)b->cap_ledges * sizeof(int));
        }
        b->ledges = nl;
        b->ledge_base = nb;
        b->cap_ledges = nc;
    }
    b->ledges[b->n_ledges] = cl;
    b->ledge_base[b->n_ledges] = base;
    b->n_ledges++;
    return true;
}

static float hdiff(const gb_t *b, const int *o, int p)
{
    const float *hf = b->height_field;
    return hf[o[p + 0]] + hf[o[p - 1]] - hf[o[p - 2]] - hf[o[p + 1]];
}

static void convert_strip(gb_t *b, int row)
{
    /* ivp_gridbuild_array.cxx:627 convert_strip_to_compact_ledges */
    int strip_field_offset[IVP_GRID_MAX_COLUMNS * 2];
    int dest_stripe[IVP_GRID_MAX_COLUMNS * 2];
    const ivp_template_compact_grid_t *templ = b->templ;
    int n_cols = b->n_cols;
    {
        int low = row * n_cols;
        int high = low + n_cols;
        int d = 0;
        for (int c = 0; c < n_cols; c++) {
            strip_field_offset[d] = low++;
            strip_field_offset[d + 1] = high++;
            d += 2;
        }
    }
    int strip_size = n_cols + n_cols;
    int start_pos, end_pos = 0, dest_index;

    for (start_pos = 0; start_pos < strip_size - 2; start_pos = end_pos - 1) {
        dest_index = 0;
        int first_point = start_pos;
        dest_stripe[dest_index] = strip_field_offset[start_pos];
        dest_stripe[dest_index + 1] = strip_field_offset[start_pos + 1];
        dest_index += 2;
        start_pos += 2;
        int ep;
    check_flat_triangles:
        for (ep = start_pos; ep < strip_size - 3; ep += 2) {
            int p;
            for (p = ep; p < ep + 3 && p < strip_size - 1; p++) {
                double diff = (double)hdiff(b, strip_field_offset, p);
                if (diff < 0.0 || diff > (double)1e-3f) {
                    p = -1;
                    break;
                }
            }
            if (p == -1) break;
        }
        dest_stripe[dest_index++] = strip_field_offset[ep];

        if (ep < strip_size - 1) {
            double diff = (double)hdiff(b, strip_field_offset, ep);
            if (diff >= 0.0) {
                start_pos = ep + 1;
                goto check_flat_triangles;
            }
        }
        end_pos = ep;

        ivp_compact_ledge_t *cl = NULL;
        int base = 0;
        if (dest_index == 3) {
            cl = convert_triangle(b, dest_stripe, &base);
        } else {
            bool is_right_starter =
                !((strip_field_offset[first_point] < strip_field_offset[first_point + 1]) != b->is_left_handed);
            if (dest_index == 4) {
                cl = convert_square(b, dest_stripe, is_right_starter, &base);
            } else {
                float ref_points[2][3];
                int p0a_index = dest_stripe[0];
                int p1a_index = dest_stripe[1];
                int p0b_index = dest_stripe[(dest_index - 1) & ~1];
                int p1b_index = dest_stripe[((dest_index - 2) & ~1) + 1];
                const float *p0a = b->height_points[p0a_index].k;
                const float *p0b = b->height_points[p0b_index].k;
                const float *p1a = b->height_points[p1a_index].k;
                const float *p1b = b->height_points[p1b_index].k;
                memcpy(ref_points[0], p0a, sizeof(ref_points[0]));
                memcpy(ref_points[1], p1a, sizeof(ref_points[1]));
                int cm = templ->column_info.maps_to;
                int hm = templ->height_maps_to;
                float col_pos = 0.5f * (p0a[cm] + p0b[cm]);
                ref_points[0][cm] = col_pos;
                ref_points[1][cm] = col_pos;
                float dz = templ->height_invert_axis ? -1.0f : 1.0f;
                float minz = p0a[hm] * dz, z;
                z = p0b[hm] * dz;
                if (z < minz) minz = z;
                z = p1a[hm] * dz;
                if (z < minz) minz = z;
                z = p1b[hm] * dz;
                if (z < minz) minz = z;
                minz -= templ->grid_field_size;
                minz *= dz;
                ref_points[0][hm] = minz;
                ref_points[1][hm] = minz;
                if (is_right_starter) {
                    float hp[3];
                    memcpy(hp, ref_points[0], sizeof(hp));
                    memcpy(ref_points[0], ref_points[1], sizeof(hp));
                    memcpy(ref_points[1], hp, sizeof(hp));
                }
                cl = convert_stripe(b, dest_stripe, dest_index, (const float(*)[3])ref_points,
                                    is_right_starter, &base);
#ifdef IVP_DEBUG_BUILD
                if (cl) ivp_debug_check_ledge_points(cl, b->c_points);
#endif
            }
        }
        if (!cl || !add_ledge(b, cl, base)) {
            if (cl) ivp_free_aligned(b->alloc, cl);
            b->failed = true;
            return;
        }
        int ledge_index = b->n_ledges - 1;
        for (int p = first_point; p < ep - 1; p++) {
            int a_offset = strip_field_offset[p];
            int b_offset = strip_field_offset[p + 1];
            if (a_offset < b_offset) {
                b->ledge_reference_field[a_offset].compact_ledge_index[0] = (int16_t)ledge_index;
            } else {
                b->ledge_reference_field[b_offset - 1].compact_ledge_index[1] = (int16_t)ledge_index;
            }
        }
    }
}

/* m_object_f_grid: columns row/col/height axis, position = origin */
static void object_f_grid(const ivp_template_compact_grid_t *gp, double rot[3][3], double pos[3])
{
    double dx = gp->row_info.invert_axis ? -(double)gp->grid_field_size : (double)gp->grid_field_size;
    double dy = gp->column_info.invert_axis ? -(double)gp->grid_field_size : (double)gp->grid_field_size;
    double dz = gp->height_invert_axis ? -1.0 : 1.0;
    memset(rot, 0, 9 * sizeof(double));
    rot[gp->row_info.maps_to][0] = dx;
    rot[gp->column_info.maps_to][1] = dy;
    rot[gp->height_maps_to][2] = dz;
    for (int i = 0; i < 3; i++) pos[i] = (double)gp->position_origin_os[i];
}

static double det3(double m[3][3])
{
    double c0 = m[1][1] * m[2][2] - m[2][1] * m[1][2];
    double c1 = m[1][2] * m[2][0] - m[2][2] * m[1][0];
    double c2 = m[1][0] * m[2][1] - m[2][0] * m[1][1];
    return m[0][0] * c0 + m[0][1] * c1 + m[0][2] * c2;
}

/* ── compile_ledges_into_compact_grid ──────────────────────────────────── */

static ivp_compact_grid_t *compile_grid(gb_t *b)
{
    const ivp_template_compact_grid_t *gp = b->templ;
    int n_rows = b->n_rows, n_cols = b->n_cols;
    /* buffer size exactly as the reference computes it (worst-case 15 bytes
     * padding for the grid elements' alignment) */
    const size_t header = offsetof(ivp_compact_grid_t, offset_compact_ledge_array);
    const size_t elements_sz = (size_t)(n_rows - 1) * (size_t)(n_cols - 1) * sizeof(ivp_compact_grid_element_t);
    const size_t points_sz = (size_t)b->n_used * sizeof(ivp_compact_poly_point_t);
    size_t buffer_size = header + (size_t)b->n_ledges * sizeof(int32_t) + 15u + elements_sz + points_sz;
    buffer_size = (buffer_size + 15u) & ~(size_t)15u; /* align compact ledges */
    for (int i = b->n_ledges - 1; i >= 0; i--) {
        int l_size = ivp_ledge_get_size(b->ledges[i]);
        IVP_ASSERT(((l_size) & 0xf) == 0);
        buffer_size += (size_t)l_size;
    }
    if (buffer_size > (size_t)INT32_MAX) return NULL;
    /* actual layout: elements 16-byte aligned after the ledge offsets, the
     * points directly after the elements, the ledges 16-byte aligned */
    const size_t off_elements = (header + (size_t)b->n_ledges * sizeof(int32_t) + 15u) & ~(size_t)15u;
    const size_t off_points = off_elements + elements_sz;
    const size_t off_ledges = (off_points + points_sz + 15u) & ~(size_t)15u;

    /* ivp_malloc_aligned(buffer_size + 4, 16) */
    ivp_compact_grid_t *cg = (ivp_compact_grid_t *)ivp_alloc_aligned(b->alloc, buffer_size + 4u, 16);
    if (!cg) return NULL;
    memset(cg, 0, buffer_size + 4u);
    cg->byte_size = (intptr_t)buffer_size;
    cg->n_rows = (int16_t)(n_rows - 1);
    cg->n_columns = (int16_t)(n_cols - 1);
    cg->n_compact_ledges = b->n_ledges;

    double rot[3][3], pos[3];
    object_f_grid(gp, rot, pos);
    {   /* real_invert */
        double r[3][3];
        double D = det3(rot);
        for (int i = 0; i < 3; i++) {
            const double *a = rot[(i + 1) % 3];
            const double *c = rot[(i + 2) % 3];
            r[i][0] = a[1] * c[2] - a[2] * c[1];
            r[i][1] = a[2] * c[0] - a[0] * c[2];
            r[i][2] = a[0] * c[1] - a[1] * c[0];
        }
        double DI = (double)1.0f / D;
        for (int i = 0; i < 3; i++) {
            for (int k = 0; k < 3; k++) cg->m_grid_f_object_rot[k][i] = r[i][k] * DI;
        }
        for (int k = 0; k < 3; k++) {
            cg->m_grid_f_object_pos[k] = -(cg->m_grid_f_object_rot[k][0] * pos[0] +
                                           cg->m_grid_f_object_rot[k][1] * pos[1] +
                                           cg->m_grid_f_object_rot[k][2] * pos[2]);
        }
    }

    {   /* center from the bounding box */
        float mx = b->height_field[0], mn = b->height_field[0];
        for (int x = n_cols * n_rows - 1; x >= 0; x--) {
            if (b->height_field[x] > mx) mx = b->height_field[x];
            if (b->height_field[x] < mn) mn = b->height_field[x];
        }
        float cgs[3] = { (float)(n_rows - 1) * 0.5f, (float)(n_cols - 1) * 0.5f, (mx + mn) * 0.5f };
        for (int r = 0; r < 3; r++) {
            cg->center[r] = (float)(rot[r][0] * (double)cgs[0] + rot[r][1] * (double)cgs[1] +
                                    rot[r][2] * (double)cgs[2] + pos[r]);
        }
    }
    {   /* radius over all used points */
        double c[3] = { (double)cg->center[0], (double)cg->center[1], (double)cg->center[2] };
        double qrad = 0.0;
        for (int i = b->n_used - 1; i >= 0; i--) {
            double dx = (double)b->buffer[i].k[0] - c[0];
            double dy = (double)b->buffer[i].k[1] - c[1];
            double dz = (double)b->buffer[i].k[2] - c[2];
            double q = dx * dx + dy * dy + dz * dz;
            if (q > qrad) qrad = q;
        }
        cg->radius = (float)sqrt(qrad);
    }
    cg->inv_grid_size = 1.0f / gp->grid_field_size;
    cg->offset_grid_elements = (int32_t)off_elements;

    char *compact_points = (char *)cg + off_points;
    memcpy(compact_points, b->buffer, points_sz);
    char *dest = (char *)cg + off_ledges;
    for (int i = 0; i < b->n_ledges; i++) {
        int l_size = ivp_ledge_get_size(b->ledges[i]);
        memcpy(dest, b->ledges[i], (size_t)l_size);
        ivp_compact_ledge_t *cl = (ivp_compact_ledge_t *)dest;
        {
            /* cl_dest->set_offset_ledge_points(offset) (ivp_compact_ledge.hxx:173) */
            int32_t offset = (int32_t)((compact_points + (size_t)b->ledge_base[i] *
                                        sizeof(ivp_compact_poly_point_t)) - dest);
            IVP_ASSERT((offset & 3) == 0); /* grid points are 4-byte aligned */
            cl->c_point_offset = offset;
        }
        IVP_IF(1) {
            /* IVP_ASSERT(cl_dest->get_point_array()->quad_distance_to(cl->get_point_array()) == 0.0f) */
            const float *p0 = ivp_ledge_get_point_array(cl)->k;
            const float *p1 = b->buffer[b->ledge_base[i]].k;
            double d0 = (double)p0[0] - (double)p1[0];
            double d1 = (double)p0[1] - (double)p1[1];
            double d2 = (double)p0[2] - (double)p1[2];
            IVP_ASSERT(d0 * d0 + d1 * d1 + d2 * d2 == 0.0);
            IVP_USE(d0);
            IVP_USE(d1);
            IVP_USE(d2);
        }
        cg->offset_compact_ledge_array[i] = (int32_t)(dest - (char *)cg);
        dest += l_size;
    }
    IVP_ASSERT((char *)dest - (char *)cg <= cg->byte_size); /* byte_size is a padded estimate */

    ivp_compact_grid_element_t *ge = (ivp_compact_grid_element_t *)((char *)cg + off_elements);
    const ivp_compact_grid_element_t *src = b->ledge_reference_field;
    for (int r = 1; r < n_rows; r++) {
        for (int c = 1; c < n_cols; c++) *ge++ = *src++;
        src++;
    }
    return cg;
}

ivp_compact_grid_t *ivp_gridbuild_array_to_compact_grid(const ivp_allocator_t *alloc,
                                                        const ivp_template_compact_grid_t *gp,
                                                        const float *height_field)
{
    if (!gp || !height_field) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    int n_rows = gp->row_info.n_points;
    int n_cols = gp->column_info.n_points;
    if (n_rows < 2 || n_cols < 2 || n_rows > IVP_GRID_MAX_ROWS || n_cols > IVP_GRID_MAX_COLUMNS) return NULL;
    if (!(gp->grid_field_size > 0.0f)) return NULL;
    if (gp->row_info.maps_to < 0 || gp->row_info.maps_to > 2 || gp->column_info.maps_to < 0 ||
        gp->column_info.maps_to > 2 || gp->height_maps_to < 0 || gp->height_maps_to > 2 ||
        gp->row_info.maps_to == gp->column_info.maps_to || gp->row_info.maps_to == gp->height_maps_to ||
        gp->column_info.maps_to == gp->height_maps_to) {
        return NULL;
    }

    gb_t b;
    memset(&b, 0, sizeof(b));
    b.alloc = alloc;
    b.n_rows = n_rows;
    b.n_cols = n_cols;
    b.height_field = height_field;
    b.templ = gp;
    size_t n = (size_t)n_rows * (size_t)n_cols;
    size_t hp_sz = n * sizeof(ivp_compact_poly_point_t);
    size_t lr_sz = n * sizeof(ivp_compact_grid_element_t);
    size_t gpl_sz = n * sizeof(int);
    size_t buf_sz = n * 3u * sizeof(ivp_compact_poly_point_t);
    b.height_points = (ivp_compact_poly_point_t *)ivp_alloc_aligned(alloc, hp_sz, 16);
    b.ledge_reference_field = (ivp_compact_grid_element_t *)ivp_alloc(alloc, lr_sz);
    b.grid_point_to_ledge_point_array = (int *)ivp_alloc(alloc, gpl_sz);
    b.buffer = (ivp_compact_poly_point_t *)ivp_alloc_aligned(alloc, buf_sz, 16);
    ivp_compact_grid_t *cg = NULL;
    if (!b.height_points || !b.ledge_reference_field || !b.grid_point_to_ledge_point_array || !b.buffer) {
        goto cleanup;
    }
    memset(b.ledge_reference_field, 0, lr_sz);
    for (size_t j = 0; j < n; j++) b.grid_point_to_ledge_point_array[j] = -1;
    b.buffer_cap = (int)(n * 3u);

    {   /* height points (constructor) */
        ivp_compact_poly_point_t *dest = b.height_points;
        const float *source = height_field;
        double x = (double)gp->position_origin_os[gp->row_info.maps_to];
        double dx = gp->row_info.invert_axis ? -(double)gp->grid_field_size : (double)gp->grid_field_size;
        double dy = gp->column_info.invert_axis ? -(double)gp->grid_field_size : (double)gp->grid_field_size;
        double dz = gp->height_invert_axis ? -1.0 : 1.0;
        for (int r = 0; r < n_rows; r++) {
            double y = (double)gp->position_origin_os[gp->column_info.maps_to];
            for (int c = 0; c < n_cols; c++) {
                dest->k[gp->row_info.maps_to] = (float)x;
                dest->k[gp->column_info.maps_to] = (float)y;
                dest->k[gp->height_maps_to] = (float)((double)source[0] * dz);
                dest->hesse_val = 0.0f;
                source++;
                dest++;
                IVP_ASSERT(((intptr_t)dest & 15) == 0);
                y += dy;
            }
            x += dx;
        }
    }

    {   /* convert_array_to_compact_ledges */
        double rot[3][3], pos[3];
        object_f_grid(gp, rot, pos);
        b.is_left_handed = det3(rot) > 0.0;
        for (int r = 0; r < n_rows - 1 && !b.failed; r++) convert_strip(&b, r);
    }
    if (!b.failed) cg = compile_grid(&b);

cleanup:
    for (int i = 0; i < b.n_ledges; i++) ivp_free_aligned(alloc, b.ledges[i]);
    if (b.ledges) ivp_free(alloc, b.ledges, (size_t)b.cap_ledges * sizeof(*b.ledges));
    if (b.ledge_base) ivp_free(alloc, b.ledge_base, (size_t)b.cap_ledges * sizeof(int));
    if (b.height_points) ivp_free_aligned(alloc, b.height_points);
    if (b.ledge_reference_field) ivp_free(alloc, b.ledge_reference_field, lr_sz);
    if (b.grid_point_to_ledge_point_array) ivp_free(alloc, b.grid_point_to_ledge_point_array, gpl_sz);
    if (b.buffer) ivp_free_aligned(alloc, b.buffer);
    return cg;
}

void ivp_compact_grid_free(const ivp_allocator_t *alloc, ivp_compact_grid_t *grid)
{
    if (!grid) return;
    if (!alloc) alloc = ivp_default_allocator();
    ivp_free_aligned(alloc, grid);
}
