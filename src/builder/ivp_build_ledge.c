/* ivp_build_ledge.c -- Compact ledge generation (convex polyhedra)
 *
 * Polygon / triangle faces go through the reference
 * IVP_Template_Polygon -> IVP_SurfaceBuilder_Polygon_Convex pipeline
 * (ivp_build_polygon_convex.c).  Also ported here:
 *   ivp_compact_builder/ivp_object_polygon_tetra.cxx:2505 (insert_pierce_info,
 *     on a finished compact ledge)
 *   ivp_compact_builder/ivp_surbuild_pointsoup.cxx:715 (convert_triangle_to_compace_ledge)
 */
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_triangle.h>
#include <ivp/base/ivp_assert.h>
#include "ivp_build_polygon_convex_internal.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <string.h>

/* ── Helpers ───────────────────────────────────────────────────────────── */

static void sub3(const double *a, const double *b, double out[3])
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

static void cross3(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static double dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/* ── Pierce info ───────────────────────────────────────────────────────── */

void ivp_compact_ledge_compute_pierce_info(ivp_compact_ledge_t *ledge)
{
    /* IVP_Object_Polygon_Tetra::insert_pierce_info (ivp_object_polygon_tetra.cxx:2505):
     * every triangle not yet treated is paired with the triangle whose
     * normalised normal minimises the scalar product (first minimum wins),
     * and the partner is overwritten symmetrically. */
    if (!ledge) return;
    int n_tri = ivp_ledge_get_n_triangles(ledge);
    if (n_tri <= 0) return;
    ivp_compact_triangle_t *tris = ivp_ledge_get_first_triangle_mut(ledge);
    const ivp_compact_poly_point_t *pa = ivp_ledge_get_point_array(ledge);

    enum { STACK_TRIS = 64 };
    double stack_h[STACK_TRIS * 3];
    unsigned char stack_done[STACK_TRIS];
    const ivp_allocator_t *alloc = ivp_default_allocator();
    double *h = stack_h;
    unsigned char *done = stack_done;
    size_t h_sz = (size_t)n_tri * 3u * sizeof(double);
    size_t d_sz = (size_t)n_tri;
    if (n_tri > STACK_TRIS) {
        h = (double *)ivp_alloc(alloc, h_sz);
        done = (unsigned char *)ivp_alloc(alloc, d_sz);
        if (!h || !done) {
            if (h) ivp_free(alloc, h, h_sz);
            if (done) ivp_free(alloc, done, d_sz);
            return;
        }
    }

    for (int t = 0; t < n_tri; t++) {
        double p[3][3];
        for (int e = 0; e < 3; e++) {
            const ivp_compact_poly_point_t *pp = &pa[ivp_edge_get_start_point_index(&tris[t].edges[e])];
            p[e][0] = (double)pp->k[0];
            p[e][1] = (double)pp->k[1];
            p[e][2] = (double)pp->k[2];
        }
        double e0[3], e1[3], n[3];
        sub3(p[1], p[0], e0);
        sub3(p[2], p[0], e1);
        cross3(e0, e1, n);
        double l = sqrt(dot3(n, n));
        if (l > 0.0) {
            n[0] /= l; n[1] /= l; n[2] /= l;
        }
        memcpy(&h[3 * t], n, sizeof(n));
        done[t] = 0;
    }
    for (int t = 0; t < n_tri; t++) {
        if (done[t]) continue;
        int found = t;
        double min_scal = DBL_MAX;
        for (int t2 = 0; t2 < n_tri; t2++) {
            double sc = dot3(&h[3 * t], &h[3 * t2]);
            if (sc < min_scal) {
                min_scal = sc;
                found = t2;
            }
        }
        ivp_tri_set_pierce_index(&tris[t], found);
        ivp_tri_set_pierce_index(&tris[found], t);
        done[t] = 1;
        done[found] = 1;
    }

    if (h != stack_h) ivp_free(alloc, h, h_sz);
    if (done != stack_done) ivp_free(alloc, done, d_sz);
}

/* ── Polygon faces -> template -> IVP_SurfaceBuilder_Polygon_Convex ───── */

/* Faces given as vertex index lists (CCW seen from outside) become the
 * planes of an IVP_Template_Polygon exactly like the qhull facets of the
 * pointsoup builder (planes_to_template).  qh_facet3vertex lists a facet's
 * vertices clockwise around the outward facet normal, so every face is
 * walked in reverse (first vertex kept) with its outward, normalised
 * Newell normal; the pointsoup's get_area_size() orientation check is
 * applied as well. */
static ivp_compact_ledge_t *ledge_from_faces(const ivp_allocator_t *alloc,
                                             const double *points, int n_points,
                                             const int *face_indices,
                                             const int *face_sizes, int n_faces)
{
    if (n_points >= (1 << 16)) return NULL;
    int total = 0;
    for (int f = 0; f < n_faces; f++) total += face_sizes[f];
    size_t planes_sz = (size_t)n_faces * sizeof(ivp_pc_plane_t);
    size_t idx_sz = (size_t)total * sizeof(int);
    ivp_pc_plane_t *planes = (ivp_pc_plane_t *)ivp_alloc(alloc, planes_sz);
    int *rev = (int *)ivp_alloc(alloc, idx_sz);
    if (!planes || !rev) {
        if (planes) ivp_free(alloc, planes, planes_sz);
        if (rev) ivp_free(alloc, rev, idx_sz);
        return NULL;
    }

    int offset = 0;
    for (int f = 0; f < n_faces; f++) {
        int k = face_sizes[f];
        const int *poly = &face_indices[offset];
        int *cw = &rev[offset];
        cw[0] = poly[0];
        for (int i = 1; i < k; i++) cw[i] = poly[k - i];

        double nw[3] = { 0.0, 0.0, 0.0 };
        for (int i = 0; i < k; i++) {
            const double *a = &points[3 * poly[i]];
            const double *b = &points[3 * poly[(i + 1) % k]];
            nw[0] += (a[1] - b[1]) * (a[2] + b[2]);
            nw[1] += (a[2] - b[2]) * (a[0] + b[0]);
            nw[2] += (a[0] - b[0]) * (a[1] + b[1]);
        }
        double len = sqrt(dot3(nw, nw));
        if (len > 0.0) {
            nw[0] /= len;
            nw[1] /= len;
            nw[2] /= len;
        }
        /* orient like the pointsoup facets: flip when the area is negative */
        if (ivp_pc_plane_area_size(points, cw, k, nw) < 0) {
            nw[0] *= -1.0;
            nw[1] *= -1.0;
            nw[2] *= -1.0;
        }
        planes[f].normal[0] = nw[0];
        planes[f].normal[1] = nw[1];
        planes[f].normal[2] = nw[2];
        planes[f].n_points = k;
        planes[f].point_idx = cw;
        offset += k;
    }

    ivp_compact_ledge_t *ledge = NULL;
    ivp_pc_templ_polygon_t templ;
    if (ivp_pc_planes_to_template(&templ, alloc, points, n_points, planes, n_faces)) {
        ledge = ivp_pc_template_to_ledge(alloc, &templ);
        ivp_pc_template_destroy(&templ);
    }
    ivp_free(alloc, planes, planes_sz);
    ivp_free(alloc, rev, idx_sz);
    return ledge;
}

/* ── Public API ────────────────────────────────────────────────────────── */

ivp_compact_ledge_t *ivp_build_ledge_from_triangles(const ivp_allocator_t *alloc,
                                                    const double *points,
                                                    int n_points,
                                                    const int *indices,
                                                    int n_triangles)
{
    if (!points || !indices || n_points < 3 || n_triangles <= 0) return NULL;
    if (n_triangles > IVP_BUILD_MAX_TRIANGLES_PER_LEDGE) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    for (int i = 0; i < 3 * n_triangles; i++) {
        if (indices[i] < 0 || indices[i] >= n_points) return NULL;
    }
    size_t sizes_sz = (size_t)n_triangles * sizeof(int);
    int *sizes = (int *)ivp_alloc(alloc, sizes_sz);
    if (!sizes) return NULL;
    for (int t = 0; t < n_triangles; t++) sizes[t] = 3;
    ivp_compact_ledge_t *ledge = ledge_from_faces(alloc, points, n_points, indices, sizes, n_triangles);
    ivp_free(alloc, sizes, sizes_sz);
    return ledge;
}

ivp_compact_ledge_t *ivp_build_ledge_from_polygons(const ivp_allocator_t *alloc,
                                                   const double *points,
                                                   int n_points,
                                                   const int *face_indices,
                                                   const int *face_sizes,
                                                   int n_faces)
{
    if (!points || !face_indices || !face_sizes || n_points < 3 || n_faces <= 0) return NULL;
    if (!alloc) alloc = ivp_default_allocator();

    long total_tri = 0;
    int offset = 0;
    for (int f = 0; f < n_faces; f++) {
        if (face_sizes[f] < 3) return NULL;
        total_tri += face_sizes[f] - 2;
        if (total_tri > IVP_BUILD_MAX_TRIANGLES_PER_LEDGE) return NULL;
        for (int i = 0; i < face_sizes[f]; i++) {
            int v = face_indices[offset + i];
            if (v < 0 || v >= n_points) return NULL;
        }
        offset += face_sizes[f];
    }
    return ledge_from_faces(alloc, points, n_points, face_indices, face_sizes, n_faces);
}

ivp_compact_ledge_t *ivp_build_triangle_ledge(const ivp_allocator_t *alloc,
                                              const double p0[3],
                                              const double p1[3],
                                              const double p2[3])
{
    /* IVP_SurfaceBuilder_Pointsoup::convert_triangle_to_compace_ledge
     * (ivp_surbuild_pointsoup.cxx:715): IVP_U_Hesse
     * inline_set_vert_to_area_defined_by_three_points + quad_length check
     * (P_DOUBLE_RES), then a copy of the two sided template ledge with the
     * three points patched in (t0 = 0,1,2 ; t1 = 1,0,2). */
    double a[3], b[3], h[3];
    sub3(p1, p0, a);
    sub3(p2, p0, b);
    cross3(b, a, h);
    if (dot3(h, h) < 1e-12) return NULL; /* P_DOUBLE_RES */
    if (!alloc) alloc = ivp_default_allocator();
    return ivp_pc_double_triangle_ledge(alloc, p0, p1, p2);
}

ivp_compact_ledge_t *ivp_compact_ledge_clone(const ivp_allocator_t *alloc,
                                             const ivp_compact_ledge_t *src)
{
    if (!src) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    int n_tri = ivp_ledge_get_n_triangles(src);
    const ivp_compact_triangle_t *st = ivp_ledge_get_first_triangle(src);
    int max_index = -1;
    for (int t = 0; t < n_tri; t++) {
        for (int e = 0; e < 3; e++) {
            int pi = ivp_edge_get_start_point_index(&st[t].edges[e]);
            if (pi > max_index) max_index = pi;
        }
    }
    int n_pts = max_index + 1;
    int size = (int)sizeof(ivp_compact_ledge_t)
             + n_tri * (int)sizeof(ivp_compact_triangle_t)
             + n_pts * (int)sizeof(ivp_compact_poly_point_t);
    ivp_compact_ledge_t *dst = (ivp_compact_ledge_t *)ivp_alloc_aligned(alloc, (size_t)size, 16);
    if (!dst) return NULL;
    size_t head = sizeof(ivp_compact_ledge_t) + (size_t)n_tri * sizeof(ivp_compact_triangle_t);
    memcpy(dst, src, head);
    dst->c_point_offset = (int32_t)head;
    memcpy((char *)dst + head, ivp_ledge_get_point_array(src),
           (size_t)n_pts * sizeof(ivp_compact_poly_point_t));
    ivp_ledge_set_size(dst, size);
    ivp_ledge_set_is_compact(dst, 1);
    return dst;
}

void ivp_compact_ledge_free(const ivp_allocator_t *alloc, ivp_compact_ledge_t *ledge)
{
    if (!ledge) return;
    if (!alloc) alloc = ivp_default_allocator();
    ivp_free_aligned(alloc, ledge);
}
