/* ivp_build_polygon.c -- Build compact surfaces from explicit polygons
 *
 * Ported from ivp_surbuild_polygon_convex.cxx (IVP_SurfaceBuilder_Polygon_Convex:
 * template -> compact ledge) followed by the ledge soup like every other
 * reference builder, and ivp_compact_modify.cxx.
 */

#include <ivp/builder/ivp_build_polygon.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_rot_inertia.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/base/ivp_assert.h>

#include <math.h>
#include <string.h>

static double *to_double(const ivp_allocator_t *alloc, const float *v, int n, size_t *sz)
{
    *sz = (size_t)n * 3u * sizeof(double);
    double *d = (double *)ivp_alloc(alloc, *sz);
    if (!d) return NULL;
    for (int i = 0; i < 3 * n; i++) d[i] = (double)v[i];
    return d;
}

static ivp_compact_surface_t *wrap_and_free(const ivp_allocator_t *alloc, ivp_compact_ledge_t *ledge)
{
    if (!ledge) return NULL;
    ivp_compact_surface_t *s = ivp_build_surface_from_ledge(alloc, ledge);
    ivp_compact_ledge_free(alloc, ledge);
    return s;
}

ivp_compact_surface_t *ivp_build_polygon_surface(
    const ivp_allocator_t *alloc,
    const float *vertices,
    int n_vertices,
    const int *indices,
    int n_triangles)
{
    if (!vertices || !indices) return NULL;
    if (n_vertices < 4 || n_triangles < 4) return NULL;
    if (n_vertices >= (1 << 16)) return NULL;
    if (n_triangles > IVP_BUILD_MAX_TRIANGLES_PER_LEDGE) return NULL;
    if (!alloc) alloc = ivp_default_allocator();

    for (int i = 0; i < n_triangles * 3; i++) {
        if (indices[i] < 0 || indices[i] >= n_vertices) return NULL;
    }
    size_t sz;
    double *pts = to_double(alloc, vertices, n_vertices, &sz);
    if (!pts) return NULL;
    ivp_compact_ledge_t *ledge = ivp_build_ledge_from_triangles(alloc, pts, n_vertices,
                                                                indices, n_triangles);
    ivp_free(alloc, pts, sz);
    return wrap_and_free(alloc, ledge);
}

ivp_compact_surface_t *ivp_build_polygon_convex_surface(
    const ivp_allocator_t *alloc,
    const float *vertices,
    int n_vertices,
    const int *face_indices,
    const int *face_sizes,
    int n_faces)
{
    if (!vertices || !face_indices || !face_sizes) return NULL;
    if (n_vertices < 4 || n_faces < 4 || n_vertices >= (1 << 16)) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    size_t sz;
    double *pts = to_double(alloc, vertices, n_vertices, &sz);
    if (!pts) return NULL;
    ivp_compact_ledge_t *ledge = ivp_build_ledge_from_polygons(alloc, pts, n_vertices,
                                                               face_indices, face_sizes, n_faces);
    ivp_free(alloc, pts, sz);
    return wrap_and_free(alloc, ledge);
}

/* ── Compact surface vertex modification ────────────────────────────────── */

int ivp_surface_modify_vertex(ivp_compact_surface_t *surface,
                                int vertex_index,
                                const float new_pos[3])
{
    if (!surface || !new_pos) return -1;

    const ivp_compact_ledgetree_node_t *root = ivp_surface_get_ledgetree_root(surface);
    if (root->offset_right_node != 0) return -1; /* compound surface */

    ivp_compact_ledge_t *ledge =
        (ivp_compact_ledge_t *)((char *)root + root->offset_compact_ledge);

    int n_points = ivp_ledge_get_n_points(ledge);
    if (vertex_index < 0 || vertex_index >= n_points) return -1;

    ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array_mut(ledge);
    pts[vertex_index].k[0] = new_pos[0];
    pts[vertex_index].k[1] = new_pos[1];
    pts[vertex_index].k[2] = new_pos[2];
    return 0;
}

void ivp_surface_recompute_bounds(ivp_compact_surface_t *surface)
{
    if (!surface) return;

    /* terminal root: refresh the bounding sphere / box exactly like
     * ledges_to_boxes_and_spheres (ivp_surbuild_ledge_soup.cxx:485) */
    ivp_compact_ledgetree_node_t *root =
        (ivp_compact_ledgetree_node_t *)((char *)surface + surface->offset_ledgetree_root);
    if (root->offset_right_node == 0) {
        const ivp_compact_ledge_t *ledge =
            (const ivp_compact_ledge_t *)((const char *)root + root->offset_compact_ledge);
        float mn[3], mx[3];
        ivp_ledge_calc_bounding_box(ledge, mn, mx);
        double c[3], r[3];
        for (int i = 0; i < 3; i++) {
            c[i] = (double)mx[i] * 0.5 + (double)mn[i] * 0.5;
            r[i] = (double)mx[i] - c[i];
        }
        double radius = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
        double work = radius * (double)IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;
        if (work < (double)IVP_DOUBLE_EPS) work = (double)IVP_DOUBLE_EPS; /* P_DOUBLE_EPS */
        for (int i = 0; i < 3; i++) {
            root->center[i] = (float)c[i];
            int bs = (int)(((double)mx[i] - c[i]) / work) + 1;
            root->box_sizes[i] = (uint8_t)(bs > 255 ? 255 : bs);
        }
        root->radius = (float)radius;
    }

    /* surface header: insert_radius_in_compact_surface */
    ivp_ledge_soup_insert_radius(surface);
}
