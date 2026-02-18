/* ivp_build_halfspacesoup.c — Half-space intersection surface builder
 *
 * Ported from ivp_surbuild_halfspacesoup.cxx (Ipion Software GmbH).
 */
#include <ivp/builder/ivp_build_halfspacesoup.h>
#include <ivp/builder/ivp_build_surface.h>
#include <math.h>
#include <string.h>

#define IVP_HALFSPACE_TOLERANCE  0.001f

/* ── 3-plane intersection ──────────────────────────────────────────────── */

/* Solve the system of three plane equations for a single point.
 * Returns false if planes are parallel/degenerate. */
static bool intersect_three_planes(const ivp_halfspace_t *p0,
                                   const ivp_halfspace_t *p1,
                                   const ivp_halfspace_t *p2,
                                   float out[3])
{
    /* The system is:
     *   n0 · x = d0
     *   n1 · x = d1
     *   n2 · x = d2
     *
     * Using Cramer's rule on the 3x3 matrix [n0; n1; n2]. */
    float det = p0->normal[0] * (p1->normal[1] * p2->normal[2] -
                                 p1->normal[2] * p2->normal[1])
              - p0->normal[1] * (p1->normal[0] * p2->normal[2] -
                                 p1->normal[2] * p2->normal[0])
              + p0->normal[2] * (p1->normal[0] * p2->normal[1] -
                                 p1->normal[1] * p2->normal[0]);

    if (fabsf(det) < 1e-10f) return false;

    float inv_det = 1.0f / det;

    out[0] = (p0->dist * (p1->normal[1] * p2->normal[2] -
                           p1->normal[2] * p2->normal[1])
            - p0->normal[1] * (p1->dist * p2->normal[2] -
                                p1->normal[2] * p2->dist)
            + p0->normal[2] * (p1->dist * p2->normal[1] -
                                p1->normal[1] * p2->dist)) * inv_det;

    out[1] = (p0->normal[0] * (p1->dist * p2->normal[2] -
                                p1->normal[2] * p2->dist)
            - p0->dist * (p1->normal[0] * p2->normal[2] -
                           p1->normal[2] * p2->normal[0])
            + p0->normal[2] * (p1->normal[0] * p2->dist -
                                p1->dist * p2->normal[0])) * inv_det;

    out[2] = (p0->normal[0] * (p1->normal[1] * p2->dist -
                                p1->dist * p2->normal[1])
            - p0->normal[1] * (p1->normal[0] * p2->dist -
                                p1->dist * p2->normal[0])
            + p0->dist * (p1->normal[0] * p2->normal[1] -
                           p1->normal[1] * p2->normal[0])) * inv_det;

    return true;
}

/* ── Point containment test ────────────────────────────────────────────── */

/* Returns true if point is inside (or on) all half-spaces. */
static bool point_inside_halfspaces(const float point[3],
                                    const ivp_halfspace_t *halfspaces,
                                    int n_halfspaces)
{
    for (int i = 0; i < n_halfspaces; i++) {
        const ivp_halfspace_t *h = &halfspaces[i];
        float d = h->normal[0] * point[0] +
                  h->normal[1] * point[1] +
                  h->normal[2] * point[2] - h->dist;
        if (d < -IVP_HALFSPACE_TOLERANCE) {
            return false;
        }
    }
    return true;
}

/* ── Point deduplication ───────────────────────────────────────────────── */

/* Remove duplicate points closer than threshold^2 (squared distance). */
static int deduplicate_points(float *points, int n_points,
                              float threshold_sq)
{
    if (n_points <= 1) return n_points;

    /* Mark duplicates */
    int out_count = 0;
    for (int i = 0; i < n_points; i++) {
        bool duplicate = false;
        for (int j = 0; j < out_count; j++) {
            float dx = points[i * 3 + 0] - points[j * 3 + 0];
            float dy = points[i * 3 + 1] - points[j * 3 + 1];
            float dz = points[i * 3 + 2] - points[j * 3 + 2];
            if (dx * dx + dy * dy + dz * dz < threshold_sq) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            if (out_count != i) {
                points[out_count * 3 + 0] = points[i * 3 + 0];
                points[out_count * 3 + 1] = points[i * 3 + 1];
                points[out_count * 3 + 2] = points[i * 3 + 2];
            }
            out_count++;
        }
    }
    return out_count;
}

/* ── Convert to points ─────────────────────────────────────────────────── */

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

    /* Maximum possible intersection points: C(n, 3) = n*(n-1)*(n-2)/6 */
    int max_points = n_halfspaces * (n_halfspaces - 1) * (n_halfspaces - 2) / 6;
    if (max_points <= 0) return IVP_ERR_INVALID_ARG;
    if (max_points > 100000) max_points = 100000;  /* safety cap */

    float *points = (float *)ivp_alloc(alloc, (size_t)max_points * 3 * sizeof(float));
    if (!points) return IVP_ERR_OUT_OF_MEMORY;

    int n_points = 0;

    /* Enumerate all triple-plane intersections */
    for (int i = 0; i < n_halfspaces - 2 && n_points < max_points; i++) {
        for (int j = i + 1; j < n_halfspaces - 1 && n_points < max_points; j++) {
            for (int k = j + 1; k < n_halfspaces && n_points < max_points; k++) {
                float pt[3];
                if (!intersect_three_planes(&halfspaces[i], &halfspaces[j],
                                            &halfspaces[k], pt)) {
                    continue;
                }

                /* Test containment: point must be inside all half-spaces */
                if (point_inside_halfspaces(pt, halfspaces, n_halfspaces)) {
                    points[n_points * 3 + 0] = pt[0];
                    points[n_points * 3 + 1] = pt[1];
                    points[n_points * 3 + 2] = pt[2];
                    n_points++;
                }
            }
        }
    }

    if (n_points < 4) {
        ivp_free(alloc, points, (size_t)max_points * 3 * sizeof(float));
        return IVP_ERR_INVALID_ARG;  /* degenerate or empty intersection */
    }

    /* Deduplicate nearby points */
    float threshold_sq = merge_threshold * merge_threshold;
    n_points = deduplicate_points(points, n_points, threshold_sq);

    if (n_points < 4) {
        ivp_free(alloc, points, (size_t)max_points * 3 * sizeof(float));
        return IVP_ERR_INVALID_ARG;
    }

    *out_points = points;
    *out_n_points = n_points;
    return IVP_OK;
}

/* ── Build surface ─────────────────────────────────────────────────────── */

ivp_compact_surface_t *ivp_build_halfspacesoup_surface(
    const ivp_allocator_t *alloc,
    const ivp_halfspace_t *halfspaces,
    int n_halfspaces,
    float merge_threshold)
{
    if (!alloc) alloc = ivp_default_allocator();

    if (merge_threshold <= 0.0f) merge_threshold = 0.01f;

    /* Step 1: Convert half-spaces to point cloud */
    float *points = NULL;
    int n_points = 0;
    ivp_result_t res = ivp_halfspacesoup_to_points(
        halfspaces, n_halfspaces, merge_threshold,
        &points, &n_points, alloc);

    if (!ivp_ok(res) || n_points < 4) {
        return NULL;
    }

    /* Step 2: Build convex hull from point cloud */
    ivp_vec3f_t *verts = (ivp_vec3f_t *)ivp_alloc(
        alloc, (size_t)n_points * sizeof(ivp_vec3f_t));
    if (!verts) {
        /* Compute original alloc size for free */
        int max_points = n_halfspaces * (n_halfspaces - 1) * (n_halfspaces - 2) / 6;
        if (max_points > 100000) max_points = 100000;
        ivp_free(alloc, points, (size_t)max_points * 3 * sizeof(float));
        return NULL;
    }

    for (int i = 0; i < n_points; i++) {
        verts[i].k[0] = points[i * 3 + 0];
        verts[i].k[1] = points[i * 3 + 1];
        verts[i].k[2] = points[i * 3 + 2];
    }

    ivp_compact_surface_t *surface =
        ivp_build_convex_surface(alloc, verts, n_points);

    /* Cleanup */
    ivp_free(alloc, verts, (size_t)n_points * sizeof(ivp_vec3f_t));
    int max_points = n_halfspaces * (n_halfspaces - 1) * (n_halfspaces - 2) / 6;
    if (max_points > 100000) max_points = 100000;
    ivp_free(alloc, points, (size_t)max_points * 3 * sizeof(float));

    return surface;
}
