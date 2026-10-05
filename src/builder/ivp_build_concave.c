/* ivp_build_concave.c — Concave polyhedron surface builder
 *
 * Chains convex decomposition → ledge soup → compile to produce a
 * single compound surface from a concave triangle mesh.
 */
#include <ivp/builder/ivp_build_concave.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include "ivp_build_checked.h"
#include <string.h>

/* ── Build from parts ──────────────────────────────────────────────────── */

ivp_compact_surface_t *ivp_build_concave_from_parts(
    const ivp_allocator_t *alloc,
    ivp_compact_surface_t **parts,
    int n_parts)
{
    if (!parts || n_parts <= 0) return NULL;
    if (!alloc) alloc = ivp_default_allocator();

    /* Single part: return a copy directly */
    if (n_parts == 1) {
        const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(parts[0]);
        if (!ledge) return NULL;

        ivp_ledge_soup_t soup;
        ivp_ledge_soup_init(&soup, alloc);
        if (ivp_ledge_soup_add(&soup, ledge) != 0) {
            ivp_ledge_soup_destroy(&soup);
            return NULL;
        }
        ivp_compact_surface_t *result = ivp_ledge_soup_compile(&soup);
        ivp_ledge_soup_destroy(&soup);
        return result;
    }

    /* Multiple parts: extract ledge from each, compile into soup */
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, alloc);

    for (int i = 0; i < n_parts; i++) {
        if (!parts[i]) continue;
        const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(parts[i]);
        if (ledge) {
            if (ivp_ledge_soup_add(&soup, ledge) != 0) {
                ivp_ledge_soup_destroy(&soup);
                return NULL;
            }
        }
    }

    ivp_compact_surface_t *result = NULL;
    if (soup.n_ledges > 0) {
        result = ivp_ledge_soup_compile(&soup);
    }

    ivp_ledge_soup_destroy(&soup);
    return result;
}

/* ── Build from a concave polyhedron (reference path) ──────────────────── */

ivp_compact_surface_t *ivp_build_concave_polyhedron_surface(
    const ivp_allocator_t *alloc,
    const double *points,
    int n_points,
    const int *face_indices,
    const int *face_sizes,
    int n_faces,
    const ivp_concave_options_t *options)
{
    /* IVP_SurfaceBuilder_Polyhedron_Concave::
     * convert_concave_polyhedron_to_single_compact_surface
     * (ivp_surbuild_polyhdrn_cncv.cxx:83) */
    if (!points || !face_indices || !face_sizes) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    ivp_concave_options_t opts = options ? *options : ivp_concave_options_default();

    ivp_compact_ledge_t **ledges = NULL;
    int n = 0;
    if (opts.force_single_hull) {
        ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(alloc, points, n_points);
        if (!l) return NULL;
        ledges = (ivp_compact_ledge_t **)ivp_alloc(alloc, sizeof(*ledges));
        if (!ledges) {
            ivp_compact_ledge_free(alloc, l);
            return NULL;
        }
        ledges[0] = l;
        n = 1;
    } else {
        n = ivp_convex_decompose_to_ledges(alloc, points, n_points, face_indices, face_sizes,
                                           n_faces, &opts.decompose, &ledges);
        if (n <= 0) return NULL;
    }

    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, alloc);
    bool ok = true;
    for (int i = 0; i < n; i++) {
        if (ivp_ledge_soup_add(&soup, ledges[i]) != 0) ok = false;
    }
    ivp_compact_surface_t *result = NULL;
    if (ok) {
        ivp_ledge_soup_template_t t;
        ivp_ledge_soup_template_init(&t);
        t.build_root_convex_hull = opts.build_root_convex_hull;
        t.merge_points = opts.merge_points;
        result = ivp_ledge_soup_compile_ex(&soup, &t);
    }
    ivp_ledge_soup_destroy(&soup);
    for (int i = 0; i < n; i++) ivp_compact_ledge_free(alloc, ledges[i]);
    ivp_free(alloc, ledges, (size_t)n * sizeof(*ledges));
    return result;
}

/* ── Build from triangle mesh ──────────────────────────────────────────── */

ivp_compact_surface_t *ivp_build_concave_surface(
    const ivp_allocator_t *alloc,
    const float *vertices,
    int n_vertices,
    const int *indices,
    int n_triangles,
    const ivp_concave_options_t *options)
{
    if (!vertices || !indices || n_triangles < 1 || n_vertices < 3)
        return NULL;
    if (!alloc) alloc = ivp_default_allocator();

    int n_coords;
    int n_indices;
    size_t pts_sz;
    size_t sizes_sz;
    if (!ivp_build_int_mul(n_vertices, 3, &n_coords) ||
        !ivp_build_int_mul(n_triangles, 3, &n_indices) ||
        !ivp_build_count_bytes(n_coords, sizeof(double), &pts_sz) ||
        !ivp_build_count_bytes(n_triangles, sizeof(int), &sizes_sz)) return NULL;
    double *pts = (double *)ivp_alloc(alloc, pts_sz);
    int *sizes = (int *)ivp_alloc(alloc, sizes_sz);
    ivp_compact_surface_t *result = NULL;
    if (pts && sizes) {
        for (int i = 0; i < n_coords; i++) pts[i] = (double)vertices[i];
        for (int i = 0; i < n_triangles; i++) sizes[i] = 3;
        result = ivp_build_concave_polyhedron_surface(alloc, pts, n_vertices, indices, sizes,
                                                      n_triangles, options);
    }
    if (pts) ivp_free(alloc, pts, pts_sz);
    if (sizes) ivp_free(alloc, sizes, sizes_sz);
    return result;
}
