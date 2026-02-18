/* ivp_build_compact_recursive.c — Root convex hull builder
 *
 * Extracts all vertices from a set of compact ledges and computes
 * a single convex hull around them.
 */
#include <ivp/builder/ivp_build_compact_recursive.h>
#include <ivp/builder/ivp_build_pointsoup.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <string.h>

/* ── Vertex extraction ────────────────────────────────────────────────── */

int ivp_extract_ledge_vertices(
    const ivp_compact_ledge_t **ledges,
    int n_ledges,
    float **out_coords,
    int *out_count,
    const ivp_allocator_t *alloc)
{
    if (!ledges || n_ledges < 1 || !out_coords || !out_count) return -1;
    if (!alloc) alloc = ivp_default_allocator();

    /* Count total points across all ledges */
    int total_points = 0;
    for (int i = 0; i < n_ledges; i++) {
        if (!ledges[i]) continue;
        total_points += ivp_ledge_get_n_points(ledges[i]);
    }

    if (total_points < 4) return -1;

    /* Allocate output array */
    float *coords = (float *)ivp_alloc(alloc,
        (size_t)total_points * 3 * sizeof(float));
    if (!coords) return -1;

    /* Extract vertex coordinates */
    int idx = 0;
    for (int i = 0; i < n_ledges; i++) {
        if (!ledges[i]) continue;
        const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(ledges[i]);
        int np = ivp_ledge_get_n_points(ledges[i]);
        for (int j = 0; j < np; j++) {
            coords[idx * 3 + 0] = pts[j].k[0];
            coords[idx * 3 + 1] = pts[j].k[1];
            coords[idx * 3 + 2] = pts[j].k[2];
            idx++;
        }
    }

    *out_coords = coords;
    *out_count  = idx;
    return 0;
}

/* ── Root convex hull builder ─────────────────────────────────────────── */

ivp_compact_surface_t *ivp_build_root_convex_hull(
    const ivp_allocator_t *alloc,
    const ivp_compact_ledge_t **ledges,
    int n_ledges)
{
    if (!alloc) alloc = ivp_default_allocator();

    /* Extract all vertices */
    float *coords = NULL;
    int n_points = 0;
    if (ivp_extract_ledge_vertices(ledges, n_ledges, &coords, &n_points,
                                    alloc) != 0) {
        return NULL;
    }

    /* Build convex hull via point soup */
    ivp_pointsoup_options_t opts = ivp_pointsoup_options_default();
    opts.center_points = false;  /* keep original coordinate system */

    ivp_compact_surface_t *hull = ivp_build_pointsoup_surface(
        alloc, coords, n_points, &opts);

    /* Free temporary vertex array */
    ivp_free(alloc, coords, (size_t)n_points * 3 * sizeof(float));

    return hull;
}
