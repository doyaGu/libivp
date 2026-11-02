/* ivp_build_convex_qhull.c — Convex compact-ledge builder via QHull
 *
 * Port of IVP_SurfaceBuilder_Pointsoup (ivp_compact_builder/
 * ivp_surbuild_pointsoup.cxx):
 *   - exact duplicate removal (IVP_I_Point_VHash compares bitwise);
 *   - 3 points -> two sided triangle ledge;
 *   - qhull "Qs Pp C-0 W1e-14 E1.0e-18" with the joggle ("QJ<eps>") retry
 *     loop and the degenerate-facet point removal of
 *     try_to_build_convex_ledge_from_qhull_result();
 *   - the facets (FORALLfacets order, qh_facet3vertex vertex order, normal
 *     flipped when the polygon area is negative) become an
 *     IVP_Template_Polygon (planes_to_template) which
 *     IVP_SurfaceBuilder_Polygon_Convex turns into the compact ledge
 *     (ivp_build_polygon_convex.c);
 *   - convert_pointsoup_to_compact_surface wraps the ledge with the ledge
 *     soup, which computes mass center / inertia / radius / deviation.
 *
 * The mainline build uses the official reentrant Qhull 2020.2 (8.0.2)
 * dependency.  Each call owns an independent qhT, so convex builders may run
 * concurrently.  A reference-development build can select the classic
 * dependency; its global Qhull state is serialized by the adapter.
 *
 * Mainline construction (not with IVP_REFERENCE_INTERNAL_STATE, which keeps
 * the reference's options and retry loop):
 *   - qhull "Qs Pp C-0": Qhull derives its roundoff and outside width from
 *     the input.  The reference's "W1e-14 E1.0e-18" claim a precision double
 *     arithmetic does not have at the coordinates' size, so Qhull 2020.2
 *     rejects valid hulls (a rotated box scaled by 100, or 10^4 from the
 *     origin) as "wide merges" and the reference falls back to joggling;
 *   - a hull whose triangles do not fit one ledge (2V - 4 above
 *     IVP_BUILD_MAX_TRIANGLES_PER_LEDGE) is no precision problem: the build
 *     stops instead of joggling until the vertex count happens to drop;
 *   - when Qhull rejects the points, flat ones become a two sided polygon
 *     ledge (their 2D hull), collinear ones none;
 *   - joggling remains the last resort, and a joggled hull is kept only when
 *     its ledge is convex in the real coordinates: the joggled facets are
 *     applied to unjoggled points.
 *
 * Not ported: after a successful build the reference copies hesse_val of
 * matching input points into the compact points, matching them by
 * reinterpreting the float compact point as an IVP_U_Point (3 doubles).
 * That comparison never matches a real point, so hesse_val stays 0.
 */

#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/containers/ivp_hashmap.h>
#include <ivp/diagnostics/ivp_debug_manager.h>
#include <ivp/utility/ivp_hash.h>

#include "ivp_build_polygon_convex_internal.h"
#include "geometry/ivp_check_ledge_debug.h"
#include "ivp_qhull_api.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PS_MIN_EDGE_LEN 0.01f /* P_MIN_EDGE_LEN */

#if IVP_REFERENCE_INTERNAL_STATE
#define PS_QHULL_COMMAND "qhull Qs Pp C-0 W1e-14 E1.0e-18"
#define PS_QHULL_JOGGLE_COMMAND "qhull Qs QJ%G C-0 Pp W1e-14 E1.0e-18"
#else
#define PS_QHULL_COMMAND "qhull Qs Pp C-0"
#define PS_QHULL_JOGGLE_COMMAND "qhull Qs QJ%G C-0 Pp"
#endif

/* ── try_to_build_convex_ledge_from_qhull_result ───────────────────────── */

typedef struct ps_facets {
    const double *pts; /* current point list (IVP_U_Vector<IVP_U_Point> points) */
    unsigned *skip_list;
    unsigned *use_list;
    bool skip_point;

    ivp_pc_plane_t *planes; /* IVP_U_Vector<IVP_SurMan_PS_Plane> planes */
    int n_planes;
    int cap_planes;
    int *idx; /* plane point indices, planes[i].point_idx point into it */
    int n_idx;
    int cap_idx;
    bool oom;
    const ivp_allocator_t *alloc;
} ps_facets_t;

static bool ps_grow(const ivp_allocator_t *alloc, void **buf, int *cap, int need, size_t elem)
{
    if (need <= *cap) return true;
    int ncap = *cap ? *cap : 16;
    while (ncap < need) ncap *= 2;
    void *nb = ivp_realloc(alloc, *buf, (size_t)*cap * elem, (size_t)ncap * elem);
    if (!nb) return false;
    *buf = nb;
    *cap = ncap;
    return true;
}

/* IVP_U_Point::quad_distance_to */
static double ps_quad_distance(const double *a, const double *b)
{
    double x = a[0] - b[0];
    x *= x;
    double y = a[1] - b[1];
    y *= y;
    double z = a[2] - b[2];
    z *= z;
    return x + y + z;
}

/* IVP_SurMan_PS_Plane::get_qlen_of_all_edges */
static double ps_qlen_of_all_edges(const double *pts, const int *ids, int n)
{
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        int in = (i + 1) % n;
        sum += ps_quad_distance(&pts[3 * ids[i]], &pts[3 * ids[in]]);
    }
    return sum;
}

/* FORALLfacets body */
static void ps_facet(void *vctx, const double normal_in[3], const int *ids, int n)
{
    ps_facets_t *ctx = (ps_facets_t *)vctx;
    if (ctx->oom) return;
    if (!ps_grow(ctx->alloc, (void **)&ctx->planes, &ctx->cap_planes, ctx->n_planes + 1,
                 sizeof(ivp_pc_plane_t)) ||
        !ps_grow(ctx->alloc, (void **)&ctx->idx, &ctx->cap_idx, ctx->n_idx + n, sizeof(int))) {
        ctx->oom = true;
        return;
    }
    ivp_pc_plane_t *plane = &ctx->planes[ctx->n_planes++];
    plane->normal[0] = normal_in[0];
    plane->normal[1] = normal_in[1];
    plane->normal[2] = normal_in[2];
    plane->n_points = n;
    plane->point_idx = NULL; /* fixed up after the walk (idx may move) */
    int *pi = &ctx->idx[ctx->n_idx];
    for (int k = 0; k < n; k++) {
        pi[k] = ids[k];
        ctx->use_list[ids[k]]++;
    }
    ctx->n_idx += n;
    if (n <= 0) goto no_point_skipped;

    const double *pts = ctx->pts;
    /* check area size of facet */
    double area_sum = ivp_pc_plane_area_size(pts, pi, n, plane->normal);
    double qedge_sum = ps_qlen_of_all_edges(pts, pi, n);

    /* swap area */
    if (area_sum < 0) {
        /* plane->mult(-1) */
        double m0 = plane->normal[0] * -1.0;
        double m1 = plane->normal[1] * -1.0;
        double m2 = plane->normal[2] * -1.0;
        plane->normal[0] = m0;
        plane->normal[1] = m1;
        plane->normal[2] = m2;
        area_sum *= -1;
    }

    if (area_sum < 0.5 * (double)PS_MIN_EDGE_LEN * sqrt(qedge_sum)) {
        /* degenerate area -> find and delete interior point:
         * search point which is farest from point 0 */
        double max_dist = -1.0;
        int max_index = 0;
        int max_index2 = 0;
        for (int i0 = 0; i0 < n; i0++) {
            int in = pi[i0];
            if (ctx->skip_list[in]) goto no_point_skipped;
            double dist = ps_quad_distance(&pts[3 * pi[0]], &pts[3 * pi[i0]]);
            if (dist > max_dist) {
                max_index = i0;
                max_dist = dist;
            }
        }
        /* search point with a maximum distance to max_index */
        max_dist = -1.0;
        for (int i1 = 0; i1 < n; i1++) {
            ctx->skip_point = true;
            double dist = ps_quad_distance(&pts[3 * pi[max_index]], &pts[3 * pi[i1]]);
            if (dist > max_dist) {
                max_index2 = i1;
                max_dist = dist;
            }
        }
        for (int i2 = 0; i2 < n; i2++) {
            if (i2 != max_index2 && i2 != max_index) {
                ctx->skip_list[pi[i2]]++;
                IVP_ASSERT(ctx->use_list[pi[i2]]);
                break;
            }
        }
    }
no_point_skipped:
    IVP_IF(0) {
        IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "n_faces: %d\n", n);
            if (n > 3) { /*@@@SF: Debug [14.12.1999] */
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "[Testing/Debug] Qhull : more than 3 points on plane!\n");
            }
        }
    }
}

/* DEBUG: print all vertices of the convex hull (FORALLvertices body) */
static void ps_debug_print_vertex(void *vctx, int point_index)
{
    const double *pts = (const double *)vctx;
    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "Point in convex hull: %f %f %f\n",
                                    pts[3 * point_index + 0], pts[3 * point_index + 1], pts[3 * point_index + 2]);
}

/* IVP_SurfaceBuilder_Pointsoup::error_output */
static void ps_error_output(const ivp_pc_templ_polygon_t *templ)
{
    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "Object points:\n");
    int i;
    for (i = 0; i < templ->n_points; i++) {
        const double *p = &templ->points[3 * i];
        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "Point [%d] : ", i);
        ivp_debug_printf("Point %f %f %f\n", p[0], p[1], p[2]); /* IVP_U_Point::print() */
    }
    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "Object edges:\n");
    for (i = 0; i < templ->n_lines; i++) {
        int p0 = templ->lines[i].p[0];
        int p1 = templ->lines[i].p[1];
        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "Distance [%d] - [%d] : %f\n", p0, p1,
                                        ps_quad_distance(&templ->points[3 * p0], &templ->points[3 * p1]));
    }
    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "\n");
}

static ivp_compact_ledge_t *ps_try_build(ivp_qhull_context_t *qhull,
                                         const ivp_allocator_t *alloc,
                                         const double *pts, int numpoints,
                                         bool *skip_point, bool *too_many,
                                         unsigned *skip_list, unsigned *use_list)
{
    ps_facets_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.pts = pts;
    ctx.skip_list = skip_list;
    ctx.use_list = use_list;
    ctx.alloc = alloc;

    *skip_point = false;
    /* DEBUG: Print all vertices of convex hull */
    IVP_IF(1) {
        IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
            ivp_qhull_for_each_vertex(qhull, ps_debug_print_vertex,
                                      (void *)(uintptr_t)pts);
        }
    }
    int n_facets = ivp_qhull_for_each_facet(qhull, ps_facet, &ctx);
    *skip_point = ctx.skip_point;
    *too_many = false;
#if !IVP_REFERENCE_INTERNAL_STATE
    /* A closed triangulated convex hull of V vertices has 2V - 4 triangles */
    if (n_facets > 0 && !ctx.oom && !ctx.skip_point) {
        int vertices = 0;
        for (int i = 0; i < numpoints; i++) {
            vertices += use_list[i] ? 1 : 0;
        }
        *too_many = 2 * vertices - 4 > IVP_BUILD_MAX_TRIANGLES_PER_LEDGE;
    }
#endif

    ivp_compact_ledge_t *ledge = NULL;
    if (n_facets > 0 && !ctx.oom && !ctx.skip_point && !*too_many) {
        /* convert convex points & planes to polygon template */
        int off = 0;
        for (int i = 0; i < ctx.n_planes; i++) {
            ctx.planes[i].point_idx = &ctx.idx[off];
            off += ctx.planes[i].n_points;
        }
        ivp_pc_templ_polygon_t templ;
        if (ivp_pc_planes_to_template(&templ, alloc, pts, numpoints, ctx.planes, ctx.n_planes)) {
            /* convert polygon template to compact ledge */
            ledge = ivp_pc_template_to_ledge(alloc, &templ);
            IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
                if (!ledge) {
                    ps_error_output(&templ);
                }
            }
            ivp_pc_template_destroy(&templ);
        }
    }
    if (ctx.planes) ivp_free(alloc, ctx.planes, (size_t)ctx.cap_planes * sizeof(ivp_pc_plane_t));
    if (ctx.idx) ivp_free(alloc, ctx.idx, (size_t)ctx.cap_idx * sizeof(int));
    return ledge;
}

#if !IVP_REFERENCE_INTERNAL_STATE
/* ── Mainline construction ──────────────────────────────────────────────── */

static void ps_sub(double r[3], const double *a, const double *b)
{
    r[0] = a[0] - b[0];
    r[1] = a[1] - b[1];
    r[2] = a[2] - b[2];
}

static void ps_cross(double r[3], const double a[3], const double b[3])
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

static double ps_dot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/* The span of the points (bounding box diagonal) and their largest coordinate */
static void ps_extent(const double *pts, int n, double *span, double *max_abs)
{
    double lo[3] = {pts[0], pts[1], pts[2]}, hi[3] = {pts[0], pts[1], pts[2]};
    *max_abs = 0.0;
    for (int i = 0; i < n; i++) {
        for (int c = 0; c < 3; c++) {
            double v = pts[3 * i + c];
            if (v < lo[c]) lo[c] = v;
            if (v > hi[c]) hi[c] = v;
            if (fabs(v) > *max_abs) *max_abs = fabs(v);
        }
    }
    double d[3] = {hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]};
    *span = sqrt(ps_dot(d, d));
}

/*
 * The points' dimension once Qhull rejected them: 1 or less when they lie
 * on a line, 2 when on a plane (within `tolerance` of the plane through
 * the three spanning points origin, origin + u, and the farthest from that
 * line; `normal` and `u` are unit vectors), else 3.
 */
static int ps_dimension(const double *pts, int n, double tolerance, int *origin,
                        double u[3], double normal[3])
{
    /* the farthest point from point 0, and the farthest from that one */
    int a = 0, b = 0;
    double best = -1.0;
    for (int i = 0; i < n; i++) {
        double d[3];
        ps_sub(d, &pts[3 * i], &pts[0]);
        if (ps_dot(d, d) > best) { best = ps_dot(d, d); a = i; }
    }
    best = -1.0;
    for (int i = 0; i < n; i++) {
        double d[3];
        ps_sub(d, &pts[3 * i], &pts[3 * a]);
        if (ps_dot(d, d) > best) { best = ps_dot(d, d); b = i; }
    }
    if (best <= tolerance * tolerance) return 0;
    ps_sub(u, &pts[3 * b], &pts[3 * a]);
    double len = sqrt(ps_dot(u, u));
    u[0] /= len; u[1] /= len; u[2] /= len;
    /* the farthest point from the line a-b */
    best = -1.0;
    double far_cross[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < n; i++) {
        double d[3], c[3];
        ps_sub(d, &pts[3 * i], &pts[3 * a]);
        ps_cross(c, u, d);
        if (ps_dot(c, c) > best) {
            best = ps_dot(c, c);
            memcpy(far_cross, c, sizeof(c));
        }
    }
    if (best <= tolerance * tolerance) return 1;
    len = sqrt(best);
    normal[0] = far_cross[0] / len; normal[1] = far_cross[1] / len; normal[2] = far_cross[2] / len;
    for (int i = 0; i < n; i++) {
        double d[3];
        ps_sub(d, &pts[3 * i], &pts[3 * a]);
        if (fabs(ps_dot(d, normal)) > tolerance) return 3;
    }
    *origin = a;
    return 2;
}

/* A point in the plane of flat points, for the 2D hull */
typedef struct ps_planar {
    double u, v;
    int index;
} ps_planar_t;

/* By u, then v */
static int ps_planar_compare(const void *a, const void *b)
{
    const ps_planar_t *p = (const ps_planar_t *)a, *q = (const ps_planar_t *)b;
    if (p->u != q->u) return p->u < q->u ? -1 : 1;
    if (p->v != q->v) return p->v < q->v ? -1 : 1;
    return 0;
}

/* A flat convex polygon as a two sided ledge: the points' 2D hull in the plane, both faces */
static ivp_compact_ledge_t *ps_flat_ledge(const ivp_allocator_t *alloc, const double *pts, int n,
                                          int origin, const double u[3], const double normal[3])
{
    double v[3];
    ps_cross(v, normal, u);
    /* the points in (u, v) coordinates, sorted by u then v (Andrew's monotone chain) */
    ps_planar_t *planar = (ps_planar_t *)ivp_alloc(alloc, (size_t)n * sizeof(ps_planar_t));
    int *hull = (int *)ivp_alloc(alloc, (size_t)(2 * n + 1) * sizeof(int));
    int *fans = (int *)ivp_alloc(alloc, (size_t)6 * (size_t)n * sizeof(int));
    ivp_compact_ledge_t *ledge = NULL;
    if (!planar || !hull || !fans) goto done;
    for (int i = 0; i < n; i++) {
        double d[3];
        ps_sub(d, &pts[3 * i], &pts[3 * origin]);
        planar[i].u = ps_dot(d, u);
        planar[i].v = ps_dot(d, v);
        planar[i].index = i;
    }
    qsort(planar, (size_t)n, sizeof(ps_planar_t), ps_planar_compare);
    int m = 0; /* hull holds positions in `planar` until the end */
    for (int pass = 0; pass < 2; pass++) { /* lower hull, then upper hull */
        int start = m;
        for (int s = 0; s < n; s++) {
            int k = pass == 0 ? s : n - 1 - s;
            while (m >= start + 2) {
                const ps_planar_t *o = &planar[hull[m - 2]], *a = &planar[hull[m - 1]], *b = &planar[k];
                double turn = (a->u - o->u) * (b->v - o->v) - (a->v - o->v) * (b->u - o->u);
                if (turn > 0.0) break;
                m--;
            }
            hull[m++] = k;
        }
        m--; /* the last point starts the other chain */
    }
    for (int i = 0; i < m; i++) hull[i] = planar[hull[i]].index;
    if (m < 3 || 2 * m - 4 > IVP_BUILD_MAX_TRIANGLES_PER_LEDGE) goto done;
    if (m == 3) {
        ledge = ivp_build_triangle_ledge(alloc, &pts[3 * hull[0]], &pts[3 * hull[1]], &pts[3 * hull[2]]);
        goto done;
    }
    /* Both faces as triangle fans, counter-clockwise seen from their side: a closed, consistently
     * oriented mesh the ledge generator links edge by edge.  The rear fan starts one corner on, so
     * no diagonal belongs to both faces (four triangles on one edge cannot be linked). */
    for (int t = 0; t < m - 2; t++) {
        int *front = &fans[6 * t], *rear = &fans[6 * t + 3];
        front[0] = hull[0]; front[1] = hull[t + 1]; front[2] = hull[t + 2];
        rear[0] = hull[1]; rear[1] = hull[(t + 3) % m]; rear[2] = hull[t + 2];
    }
    ledge = ivp_build_ledge_from_triangles(alloc, pts, n, fans, 2 * (m - 2));
done:
    if (planar) ivp_free(alloc, planar, (size_t)n * sizeof(ps_planar_t));
    if (hull) ivp_free(alloc, hull, (size_t)(2 * n + 1) * sizeof(int));
    if (fans) ivp_free(alloc, fans, (size_t)6 * (size_t)n * sizeof(int));
    return ledge;
}

/*
 * Whether the ledge is convex within `tolerance`: no point of it in front of
 * a triangle's plane (front: away from the points' centroid).
 */
static bool ps_ledge_is_convex(const ivp_compact_ledge_t *ledge, double tolerance)
{
    int n_points = ivp_ledge_get_n_points(ledge), n_triangles = ivp_ledge_get_n_triangles(ledge);
    const ivp_compact_poly_point_t *points = ivp_ledge_get_point_array(ledge);
    double centroid[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < n_points; i++) {
        for (int c = 0; c < 3; c++) centroid[c] += (double)points[i].k[c] / n_points;
    }
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
    for (int t = 0; t < n_triangles; t++, tri++) {
        double p[3][3];
        for (int j = 0; j < 3; j++) {
            const ivp_compact_poly_point_t *q = ivp_edge_get_start_point(&tri->edges[j], ledge);
            for (int c = 0; c < 3; c++) p[j][c] = (double)q->k[c];
        }
        double e1[3], e2[3], nrm[3], to_centroid[3];
        ps_sub(e1, p[1], p[0]);
        ps_sub(e2, p[2], p[0]);
        ps_cross(nrm, e1, e2);
        double len = sqrt(ps_dot(nrm, nrm));
        if (len == 0.0) return false;
        for (int c = 0; c < 3; c++) nrm[c] /= len;
        ps_sub(to_centroid, centroid, p[0]);
        double inward = ps_dot(nrm, to_centroid);
        if (fabs(inward) <= tolerance) continue; /* a flat ledge: no front */
        double sign = inward > 0.0 ? -1.0 : 1.0;
        for (int i = 0; i < n_points; i++) {
            double q[3] = {(double)points[i].k[0], (double)points[i].k[1], (double)points[i].k[2]}, d[3];
            ps_sub(d, q, p[0]);
            if (sign * ps_dot(nrm, d) > tolerance) return false;
        }
    }
    return true;
}

/*
 * The hull of the points (duplicates removed): Qhull with its own
 * precision, IVP's degenerate-facet point removal, flat and collinear
 * points handled without Qhull, joggling only when Qhull rejects points
 * spanning space, and a joggled result kept only when convex.
 */
static ivp_compact_ledge_t *ps_build_mainline(ivp_qhull_context_t *qhull,
                                              const ivp_allocator_t *alloc,
                                              double *pts, int numpoints,
                                              unsigned *skip_list, unsigned *use_list)
{
    double random_eps = 0.0; /* 0: no joggle */
    for (;;) {
        if (numpoints < 3) return NULL;
        if (numpoints == 3) return ivp_build_triangle_ledge(alloc, &pts[0], &pts[3], &pts[6]);
        double span, max_abs;
        ps_extent(pts, numpoints, &span, &max_abs);
        /* The compact ledge keeps float coordinates */
        double tolerance = 16.0 * (double)FLT_EPSILON * (max_abs > span ? max_abs : span);
        int exitcode;
        ivp_qhull_free_hull(qhull);
        if (random_eps == 0.0) {
            exitcode = ivp_qhull_new_hull(qhull, pts, numpoints, PS_QHULL_COMMAND);
            if (exitcode) {
                int origin = 0;
                double u[3], normal[3];
                int dimension = ps_dimension(pts, numpoints, tolerance, &origin, u, normal);
                if (dimension < 2) return NULL;
                if (dimension == 2) return ps_flat_ledge(alloc, pts, numpoints, origin, u, normal);
                random_eps = (double)1e-12f;
                continue;
            }
        } else {
            char flags[250];
            snprintf(flags, sizeof(flags), PS_QHULL_JOGGLE_COMMAND, random_eps);
            exitcode = ivp_qhull_new_hull(qhull, pts, numpoints, flags);
        }
        bool skip_point = false, too_many = false;
        if (!exitcode) {
            memset(use_list, 0, (size_t)numpoints * sizeof(unsigned));
            memset(skip_list, 0, (size_t)numpoints * sizeof(unsigned));
            ivp_compact_ledge_t *res = ps_try_build(qhull, alloc, pts, numpoints,
                                                    &skip_point, &too_many, skip_list, use_list);
            if (res) {
                if (random_eps == 0.0 || ps_ledge_is_convex(res, tolerance)) return res;
                ivp_compact_ledge_free(alloc, res);
                return NULL;
            }
            if (too_many) return NULL;
            /* drop the interior points and the degenerate facets' points, and build again */
            int dest = 0;
            for (int x = 0; x < numpoints; x++) {
                if (use_list[x] && !skip_list[x]) {
                    memmove(&pts[3 * dest], &pts[3 * x], 3 * sizeof(double));
                    dest++;
                }
            }
            if (dest != numpoints) {
                numpoints = dest;
                continue;
            }
        }
        /* nothing to drop: joggle (more) */
        random_eps = (random_eps + (double)1e-12f) * (double)1.2f;
        if (random_eps >= (double)0.02f) return NULL;
    }
}
#endif

/* ── convert_pointsoup_to_compact_ledge_internal ───────────────────────── */

static ivp_compact_ledge_t *ps_build_qhull_ledge(const ivp_allocator_t *alloc,
                                                 const double *pts_in, int n_in)
{
    IVP_IF(1) {
        IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "*** Starting building new convex hull from pointsoup.\n");
        }
    }

    /* copy vector and remove duplicated points */
    size_t pts_sz = (size_t)n_in * 3u * sizeof(double);
    double *pts = (double *)ivp_alloc(alloc, pts_sz); /* points == points2 */
    unsigned *skip_list = (unsigned *)ivp_alloc(alloc, (size_t)n_in * sizeof(unsigned));
    unsigned *use_list = (unsigned *)ivp_alloc(alloc, (size_t)n_in * sizeof(unsigned));
    ivp_compact_ledge_t *res = NULL;
    ivp_qhull_context_t *qhull = NULL;

    if (!pts || !skip_list || !use_list) goto cleanup;
    qhull = ivp_qhull_context_create();
    if (!qhull) goto cleanup;

    int numpoints = 0;
    {
        ivp_hashmap_t map;
        int cap = 16;
        while (cap < 2 * n_in) cap <<= 1;
        if (!ivp_ok(ivp_hashmap_init(&map, alloc, cap, ivp_hash_fnv1a, ivp_eq_memcmp,
                                     3 * sizeof(double)))) {
            goto cleanup;
        }
        for (int i = 0; i < n_in; i++) {
            const double *p = &pts_in[3 * i];
            if (ivp_hashmap_contains(&map, p)) continue;
            memcpy(&pts[3 * numpoints], p, 3 * sizeof(double));
            (void)ivp_hashmap_put(&map, &pts_in[3 * i], (void *)1);
            numpoints++;
        }
        ivp_hashmap_destroy(&map);
    }
#if !IVP_REFERENCE_INTERNAL_STATE
    res = ps_build_mainline(qhull, alloc, pts, numpoints, skip_list, use_list);
#else
    /* first loop, try to randomize point 0 until we get a result.  With
     * fewer than 4 distinct points every qhull call fails ("not enough
     * points"), and the reference retries until random_eps reaches 0.02,
     * printing qhull's error report each time, then returns NULL. */
    bool try_unjumbled = true;
    for (double random_eps = (double)1e-12f; random_eps < (double)0.02f;) {
        int exitcode = 0;
        if (try_unjumbled) {
            ivp_qhull_free_hull(qhull);
            exitcode = ivp_qhull_new_hull(qhull, pts, numpoints, PS_QHULL_COMMAND);
        }
        if (exitcode || !try_unjumbled) {
            IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "*** Qhull failed. Retrying with different parameters.\n");
            }
            char flags[250];
            snprintf(flags, sizeof(flags), PS_QHULL_JOGGLE_COMMAND, random_eps);
            ivp_qhull_free_hull(qhull);
            exitcode = ivp_qhull_new_hull(qhull, pts, numpoints, flags);
            if (exitcode) {
                random_eps = (random_eps + (double)1e-12f) * (double)1.2f;
                try_unjumbled = false;
                continue;
            }
        }

        bool skip_point = false, too_many = false;
        memset(use_list, 0, (size_t)numpoints * sizeof(unsigned));
        memset(skip_list, 0, (size_t)numpoints * sizeof(unsigned));
        res = ps_try_build(qhull, alloc, pts, numpoints,
                           &skip_point, &too_many, skip_list, use_list);
        if (res) break;

        int dest = 0;
        for (int x = 0; x < numpoints; x++) {
            if (use_list[x] && !skip_list[x]) {
                memmove(&pts[3 * dest], &pts[3 * x], 3 * sizeof(double));
                dest++;
            }
        }
        if (numpoints == dest) {
            random_eps = (random_eps + (double)1e-12f) * (double)1.2f;
            try_unjumbled = false;
        }
        numpoints = dest;
        if (numpoints == 3) {
            res = ivp_build_triangle_ledge(alloc, &pts[0], &pts[3], &pts[6]);
        }
        if (numpoints <= 3) break;
    }
#endif

cleanup:
    ivp_qhull_context_destroy(qhull);
    if (pts) ivp_free(alloc, pts, pts_sz);
    if (skip_list) ivp_free(alloc, skip_list, (size_t)n_in * sizeof(unsigned));
    if (use_list) ivp_free(alloc, use_list, (size_t)n_in * sizeof(unsigned));
    if (!res) {
        IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "*** IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_template_polygon - couldn't build convex hull! Skipping object...\n");
        }
    }
    IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "*** Done with convex pointsoup.\n\n");
    }
    return res;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

ivp_compact_ledge_t *ivp_build_convex_ledge_d(const ivp_allocator_t *alloc,
                                              const double *coords,
                                              int n_points)
{
    /* convert_pointsoup_to_compact_ledge (ivp_surbuild_pointsoup.cxx:744) */
    if (!coords || n_points < 3) return NULL;
    if (n_points > INT_MAX / 3) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    if (n_points == 3) {
        return ivp_build_triangle_ledge(alloc, &coords[0], &coords[3], &coords[6]);
    }
    return ps_build_qhull_ledge(alloc, coords, n_points);
}

ivp_compact_ledge_t *ivp_build_convex_ledge(const ivp_allocator_t *alloc,
                                            const ivp_vec3f_t *points,
                                            int n_points)
{
    if (!points || n_points < 3) return NULL;
    if (n_points > INT_MAX / 3) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    size_t sz = (size_t)n_points * 3u * sizeof(double);
    double *c = (double *)ivp_alloc(alloc, sz);
    if (!c) return NULL;
    for (int i = 0; i < n_points; i++) {
        c[3 * i + 0] = (double)points[i].k[0];
        c[3 * i + 1] = (double)points[i].k[1];
        c[3 * i + 2] = (double)points[i].k[2];
    }
    ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(alloc, c, n_points);
    ivp_free(alloc, c, sz);
    return l;
}

ivp_compact_surface_t *ivp_build_surface_from_ledge(const ivp_allocator_t *alloc,
                                                    const ivp_compact_ledge_t *ledge)
{
    if (!ledge) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, alloc);
    ivp_compact_surface_t *s = NULL;
    if (ivp_ledge_soup_add(&soup, ledge) == 0) {
        s = ivp_ledge_soup_compile(&soup);
    }
    ivp_ledge_soup_destroy(&soup);
    return s;
}

ivp_compact_surface_t *ivp_build_convex_surface_d(const ivp_allocator_t *alloc,
                                                  const double *coords,
                                                  int n_points)
{
    /* convert_pointsoup_to_compact_surface (ivp_surbuild_pointsoup.cxx:829) */
    if (!alloc) alloc = ivp_default_allocator();
    ivp_compact_ledge_t *ledge = ivp_build_convex_ledge_d(alloc, coords, n_points);
    if (!ledge) {
        IVP_IF(1) {
            IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "*** IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface - skipping ledge due to invalid topology\n");
            }
        }
        return NULL;
    }
#ifdef IVP_DEBUG_BUILD
    ivp_debug_check_ledge(ledge);
#endif
    ivp_compact_surface_t *s = ivp_build_surface_from_ledge(alloc, ledge);
    ivp_compact_ledge_free(alloc, ledge);
    return s;
}

ivp_compact_surface_t *ivp_build_convex_surface(const ivp_allocator_t *alloc,
                                                 const ivp_vec3f_t *points,
                                                 int n_points)
{
    /* convert_pointsoup_to_compact_surface (ivp_surbuild_pointsoup.cxx:829) */
    if (!alloc) alloc = ivp_default_allocator();
    ivp_compact_ledge_t *ledge = ivp_build_convex_ledge(alloc, points, n_points);
    if (!ledge) {
        IVP_IF(1) {
            IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "*** IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface - skipping ledge due to invalid topology\n");
            }
        }
        return NULL;
    }
#ifdef IVP_DEBUG_BUILD
    ivp_debug_check_ledge(ledge);
#endif
    ivp_compact_surface_t *s = ivp_build_surface_from_ledge(alloc, ledge);
    ivp_compact_ledge_free(alloc, ledge);
    return s;
}
