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
 * dependency; its global Qhull state is serialized by the adapter.  Backend
 * selection and the resulting layout domain are documented in
 * docs/PARITY_GAP_INVENTORY.md.
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

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PS_MIN_EDGE_LEN 0.01f /* P_MIN_EDGE_LEN */

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
                                         bool *skip_point,
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

    ivp_compact_ledge_t *ledge = NULL;
    if (n_facets > 0 && !ctx.oom && !ctx.skip_point) {
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
    /* first loop, try to randomize point 0 until we get a result.  With
     * fewer than 4 distinct points every qhull call fails ("not enough
     * points"), and the reference retries until random_eps reaches 0.02,
     * printing qhull's error report each time, then returns NULL. */
    bool try_unjumbled = true;
    for (double random_eps = (double)1e-12f; random_eps < (double)0.02f;) {
        int exitcode = 0;
        if (try_unjumbled) {
            ivp_qhull_free_hull(qhull);
            exitcode = ivp_qhull_new_hull(
                qhull, pts, numpoints,
                "qhull Qs Pp C-0 W1e-14 E1.0e-18");
        }
        if (exitcode || !try_unjumbled) {
            IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_SURBUILD_POINTSOUP, "*** Qhull failed. Retrying with different parameters.\n");
            }
            char flags[250];
            snprintf(flags, sizeof(flags), "qhull Qs QJ%G C-0 Pp W1e-14 E1.0e-18", random_eps);
            ivp_qhull_free_hull(qhull);
            exitcode = ivp_qhull_new_hull(qhull, pts, numpoints, flags);
            if (exitcode) {
                random_eps = (random_eps + (double)1e-12f) * (double)1.2f;
                try_unjumbled = false;
                continue;
            }
        }

        bool skip_point = false;
        memset(use_list, 0, (size_t)numpoints * sizeof(unsigned));
        memset(skip_list, 0, (size_t)numpoints * sizeof(unsigned));
        res = ps_try_build(qhull, alloc, pts, numpoints,
                           &skip_point, skip_list, use_list);
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
