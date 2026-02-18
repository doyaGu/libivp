/* ivp_build_concave.c — Concave polyhedron surface builder
 *
 * Chains convex decomposition → ledge soup → compile to produce a
 * single compound surface from a concave triangle mesh.
 */
#include <ivp/builder/ivp_build_concave.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/geometry/ivp_compact_ledge.h>
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

    ivp_concave_options_t opts;
    if (options) opts = *options;
    else         opts = ivp_concave_options_default();

    /* If force_single_hull, just build one convex hull from all vertices */
    if (opts.force_single_hull) {
        ivp_vec3f_t *pts = (ivp_vec3f_t *)ivp_alloc(
            alloc, (size_t)n_vertices * sizeof(ivp_vec3f_t));
        if (!pts) return NULL;

        for (int i = 0; i < n_vertices; i++) {
            pts[i].k[0] = vertices[i * 3 + 0];
            pts[i].k[1] = vertices[i * 3 + 1];
            pts[i].k[2] = vertices[i * 3 + 2];
        }

        ivp_compact_surface_t *result =
            ivp_build_convex_surface(alloc, pts, n_vertices);
        ivp_free(alloc, pts, (size_t)n_vertices * sizeof(ivp_vec3f_t));
        return result;
    }

    /* Step 1: Convex decomposition */
    ivp_decompose_result_t decomp;
    ivp_result_t res = ivp_convex_decompose(
        alloc, vertices, n_vertices, indices, n_triangles,
        &opts.decompose, &decomp);

    if (!ivp_ok(res) || decomp.n_parts <= 0) {
        return NULL;
    }

    /* Step 2: Compile parts into a single compound surface */
    ivp_compact_surface_t *result =
        ivp_build_concave_from_parts(alloc, decomp.parts, decomp.n_parts);

    /* Step 3: Cleanup decomposition results */
    ivp_decompose_result_free(&decomp);

    return result;
}
