/* ivp_build_halfspacesoup.c — Half-space intersection surface builder
 *
 * Port of ivp_compact_builder/ivp_surbuild_halfspacesoup.cxx
 * (Ipion Software GmbH):
 *   - every triple of planes is intersected with IVP_U_Point::set_crossing
 *     (determinant inversion, |D| < P_DOUBLE_EPS -> no intersection);
 *   - a point is dropped when it lies more than HALFSPACESOUP_TOLERANCE =
 *     0.01 * P_MIN_EDGE_LEN = 1e-4 outside any plane (ivp_halfspacesoup.hxx:15);
 *   - points closer than pointmerge_threshold to an earlier point are
 *     dropped (insert_point_into_list keeps the old one), followed by the
 *     reference's second merge pass;
 *   - the points go through the pointsoup convex builder and the ledge soup.
 */
#include <ivp/builder/ivp_build_halfspacesoup.h>
#include "ivp_build_halfspacesoup_internal.h"
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include "ivp_build_checked.h"
#include <math.h>
#include <string.h>

/* HALFSPACESOUP_TOLERANCE (0.01f * P_MIN_EDGE_LEN) */
#define IVP_HALFSPACE_TOLERANCE ((double)(0.01f * 0.01f))
#define HS_DOUBLE_EPS 1e-10 /* P_DOUBLE_EPS */

/* IVP_U_Point::set_crossing (ivu_linear.cxx:332) with the reference's
 * IVP_U_Matrix::real_invert (ivu_linear.cxx:767).  The libivp plane
 * n.x = dist corresponds to the reference hesse (n, -dist). */
static bool set_crossing(const ivp_halfspace_t *h0, const ivp_halfspace_t *h1,
                         const ivp_halfspace_t *h2, double out[3])
{
    double rows[3][3];
    const ivp_halfspace_t *h[3] = { h0, h1, h2 };
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) rows[i][k] = (double)h[i]->normal[k];
    }
    double r[3][3];
    for (int i = 0; i < 3; i++) {
        const double *a = rows[(i + 1) % 3];
        const double *b = rows[(i + 2) % 3];
        r[i][0] = a[1] * b[2] - a[2] * b[1];
        r[i][1] = a[2] * b[0] - a[0] * b[2];
        r[i][2] = a[0] * b[1] - a[1] * b[0];
    }
    double D = rows[0][0] * r[0][0] + rows[0][1] * r[0][1] + rows[0][2] * r[0][2];
    if (fabs(D) < HS_DOUBLE_EPS) return false;
    double DI = (double)1.0f / D;
    double rhs[3] = { (double)h0->dist, (double)h1->dist, (double)h2->dist };
    for (int k = 0; k < 3; k++) {
        out[k] = (r[0][k] * DI) * rhs[0] + (r[1][k] * DI) * rhs[1] + (r[2][k] * DI) * rhs[2];
    }
    return true;
}

static double plane_dist(const ivp_halfspace_t *h, const double p[3])
{
    return (double)h->normal[0] * p[0] + (double)h->normal[1] * p[1] +
           (double)h->normal[2] * p[2] - (double)h->dist;
}

static double qdist(const double *a, const double *b)
{
    double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return dx * dx + dy * dy + dz * dz;
}

/* convert_halfspacesoup_to_points (ivp_surbuild_halfspacesoup.cxx:59).
 * Returns the number of points written to *out (allocated, caller frees
 * with *out_sz bytes), or -1. */
static int halfspaces_to_points_d(const ivp_allocator_t *alloc,
                                  const ivp_halfspace_t *hs, int n,
                                  double merge_threshold,
                                  double **out, size_t *out_sz)
{
    if (n < 3) return -1;
    size_t factors[3] = { (size_t)n, (size_t)(n - 1), (size_t)(n - 2) };
    for (int divisor = 2; divisor <= 3; divisor++) {
        bool divided = false;
        for (int i = 0; i < 3; i++) {
            if (factors[i] % (size_t)divisor == 0u) {
                factors[i] /= (size_t)divisor;
                divided = true;
                break;
            }
        }
        if (!divided) return -1;
    }
    size_t max_points;
    if (!ivp_build_size_mul(factors[0], factors[1], &max_points) ||
        !ivp_build_size_mul(max_points, factors[2], &max_points) ||
        max_points == 0u || max_points > (size_t)INT_MAX) return -1;
    size_t n_coords;
    size_t sz;
    if (!ivp_build_size_mul(max_points, 3u, &n_coords) ||
        !ivp_build_size_mul(n_coords, sizeof(double), &sz)) return -1;
    double *pts = (double *)ivp_alloc(alloc, sz);
    if (!pts) return -1;
    int np = 0;
    double threshold = merge_threshold * merge_threshold;

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            for (int k = j + 1; k < n; k++) {
                double p[3];
                if (!set_crossing(&hs[i], &hs[j], &hs[k], p)) continue;
                bool skip = false;
                for (int l = 0; l < n; l++) {
                    if (plane_dist(&hs[l], p) < -IVP_HALFSPACE_TOLERANCE) {
                        skip = true;
                        break;
                    }
                }
                if (skip) continue;
                bool dup = false;
                for (int q = 0; q < np; q++) {
                    if (qdist(p, &pts[3 * q]) < threshold) {
                        dup = true;
                        break;
                    }
                }
                if (dup) continue;
                memcpy(&pts[3 * np], p, sizeof(p));
                np++;
            }
        }
    }

    /* second merge pass */
    for (int i = 0; i < np; i++) {
        for (int j = np - 1; j > i; j--) {
            if (qdist(&pts[3 * i], &pts[3 * j]) < threshold) {
                IVP_IF(1) {
                    ivp_debug_printf("*** IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_points - removing ");
                    ivp_debug_printf("Point %f %f %f\n", pts[3 * j + 0], pts[3 * j + 1], pts[3 * j + 2]); /* p2->print() */
                }
                memmove(&pts[3 * j], &pts[3 * (j + 1)], (size_t)(np - j - 1) * 3u * sizeof(double));
                np--;
            }
        }
    }
    *out = pts;
    *out_sz = sz;
    return np;
}

ivp_result_t ivp_halfspacesoup_to_points(
    const ivp_halfspace_t *halfspaces,
    int n_halfspaces,
    float merge_threshold,
    float **out_points,
    int *out_n_points,
    const ivp_allocator_t *alloc)
{
    if (!halfspaces || n_halfspaces < 4 || !out_points || !out_n_points)
        return IVP_ERR_INVALID_ARG;
    if (!alloc) alloc = ivp_default_allocator();

    double *pts = NULL;
    size_t sz = 0;
    int np = halfspaces_to_points_d(alloc, halfspaces, n_halfspaces,
                                    (double)merge_threshold, &pts, &sz);
    if (np < 0) return IVP_ERR_OUT_OF_MEMORY;
    if (np < 4) {
        ivp_free(alloc, pts, sz);
        return IVP_ERR_INVALID_ARG;
    }
    /* caller frees with ivp_free(alloc, ptr, np * 3 * sizeof(float)) */
    int n_coords;
    size_t float_sz;
    if (!ivp_build_int_mul(np, 3, &n_coords) ||
        !ivp_build_count_bytes(n_coords, sizeof(float), &float_sz)) {
        ivp_free(alloc, pts, sz);
        return IVP_ERR_CAPACITY_EXCEEDED;
    }
    float *f = (float *)ivp_alloc(alloc, float_sz);
    if (!f) {
        ivp_free(alloc, pts, sz);
        return IVP_ERR_OUT_OF_MEMORY;
    }
    for (int i = 0; i < n_coords; i++) f[i] = (float)pts[i];
    ivp_free(alloc, pts, sz);
    *out_points = f;
    *out_n_points = np;
    return IVP_OK;
}

/* convert_halfspacesoup_to_compact_ledge (ivp_surbuild_halfspacesoup.cxx:195) */
ivp_compact_ledge_t *ivp_hs_halfspacesoup_to_ledge(const ivp_allocator_t *alloc,
                                                   const ivp_halfspace_t *halfspaces,
                                                   int n_halfspaces,
                                                   float merge_threshold)
{
    if (!halfspaces || n_halfspaces < 4) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    if (merge_threshold <= 0.0f) merge_threshold = 0.01f;

    double *pts = NULL;
    size_t sz = 0;
    int np = halfspaces_to_points_d(alloc, halfspaces, n_halfspaces,
                                    (double)merge_threshold, &pts, &sz);
    if (np < 0) return NULL;
    /* convert_pointsoup_to_compact_ledge (NULL for fewer than 3 points) */
    ivp_compact_ledge_t *ledge = ivp_build_convex_ledge_d(alloc, pts, np);
    ivp_free(alloc, pts, sz);
    return ledge;
}

ivp_compact_surface_t *ivp_build_halfspacesoup_surface(
    const ivp_allocator_t *alloc,
    const ivp_halfspace_t *halfspaces,
    int n_halfspaces,
    float merge_threshold)
{
    /* convert_halfspacesoup_to_compact_surface (ivp_surbuild_halfspacesoup.cxx:216) */
    if (!halfspaces) return NULL;
    if (!alloc) alloc = ivp_default_allocator();

    ivp_compact_surface_t *surface = NULL;
    ivp_compact_ledge_t *ledge = ivp_hs_halfspacesoup_to_ledge(alloc, halfspaces, n_halfspaces,
                                                               merge_threshold);
    if (ledge) {
        /* ledge soup (free_input_compact_ledges = link_to_input_compact_ledges = IVP_FALSE) */
        surface = ivp_build_surface_from_ledge(alloc, ledge);
        ivp_compact_ledge_free(alloc, ledge);
    } else {
        ivp_debug_printf("*** IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_compact_surface - skipping ledge due to invalid topology\n");
    }
    return surface;
}
