/* ivp_build_polygon.c -- Build compact surfaces from explicit triangles
 *
 * Ported from ivp_surbuild_polygon_convex.cxx / ivp_compact_modify.cxx.
 */

#include <ivp/builder/ivp_build_polygon.h>
#include <ivp/builder/ivp_rot_inertia.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/base/ivp_assert.h>
#include "ivp_build_edge_map_internal.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#define IVP_TRI_INDEX_LIMIT (1 << 12)

/* ── Edge hash for opposite-edge pairing ────────────────────────────────── */

static int build_opposites(const ivp_allocator_t *alloc,
                           const int *indices, int n_tri, int *out_opp) {
    if (!alloc || !indices || !out_opp || n_tri <= 0) return 0;
    if (n_tri > IVP_TRI_INDEX_LIMIT) return 0;
    if (n_tri > INT_MAX / 3) return 0;

    const int n_edges = n_tri * 3;
    if (n_edges > INT_MAX / 4) return 0;

    ivp_build_edge_map_t edge_map;
    if (!ivp_build_edge_map_init(&edge_map, alloc, n_edges)) return 0;

    for (int i = 0; i < n_edges; i++) out_opp[i] = -1;

    for (int t = 0; t < n_tri; t++) {
        for (int e = 0; e < 3; e++) {
            int a = indices[t * 3 + e];
            int b = indices[t * 3 + (e + 1) % 3];

            int this_linear = t * 4 + e;
            uint64_t rev_key = ivp_build_make_edge_key(b, a);
            int other_linear = 0;
            if (ivp_build_edge_map_get_linear(&edge_map, rev_key, &other_linear)) {
                int other_edge_idx = (other_linear / 4) * 3 + (other_linear % 4);
                int this_edge_idx  = t * 3 + e;
                out_opp[this_edge_idx]  = other_linear - this_linear;
                out_opp[other_edge_idx] = this_linear - other_linear;
            } else {
                uint64_t key = ivp_build_make_edge_key(a, b);
                if (!ivp_build_edge_map_insert_linear(&edge_map, key, this_linear)) {
                    ivp_build_edge_map_destroy(&edge_map);
                    return 0;
                }
            }
        }
    }

    ivp_build_edge_map_destroy(&edge_map);

    /* Verify all edges were paired */
    for (int i = 0; i < n_edges; i++) {
        if (out_opp[i] == -1) return 0;
    }
    return 1;
}

/* ── Public: build from triangles ──────────────────────────────────────── */

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
    if (n_triangles > IVP_TRI_INDEX_LIMIT) return NULL;
    if (n_triangles > INT_MAX / 3) return NULL;
    if (!alloc) alloc = ivp_default_allocator();

    /* Validate indices */
    int n_indices = n_triangles * 3;
    for (int i = 0; i < n_indices; i++) {
        if (indices[i] < 0 || indices[i] >= n_vertices) return NULL;
    }

    /* Build opposite edge table */
    if ((size_t)n_triangles > ((size_t)-1) / (3u * sizeof(int))) return NULL;
    int *opp_idx = (int *)ivp_alloc(alloc,
                    (size_t)n_triangles * 3u * sizeof(int));
    if (!opp_idx) return NULL;

    if (!build_opposites(alloc, indices, n_triangles, opp_idx)) {
        ivp_free(alloc, opp_idx, (size_t)n_triangles * 3u * sizeof(int));
        return NULL;
    }

    /* Memory layout */
    const int surface_header_size = (int)sizeof(ivp_compact_surface_t);
    const int ledge_header_size   = (int)sizeof(ivp_compact_ledge_t);
    const int tri_size            = (int)sizeof(ivp_compact_triangle_t);
    const int point_size          = (int)sizeof(ivp_compact_poly_point_t);
    const int node_size           = (int)sizeof(ivp_compact_ledgetree_node_t);

    if ((size_t)n_triangles > ((size_t)-1) / (size_t)tri_size) {
        ivp_free(alloc, opp_idx, (size_t)n_triangles * 3u * sizeof(int));
        return NULL;
    }
    if ((size_t)n_vertices > ((size_t)-1) / (size_t)point_size) {
        ivp_free(alloc, opp_idx, (size_t)n_triangles * 3u * sizeof(int));
        return NULL;
    }

    size_t ledge_data_size_z = (size_t)ledge_header_size
                             + (size_t)n_triangles * (size_t)tri_size
                             + (size_t)n_vertices * (size_t)point_size;
    size_t total_unpadded_z = (size_t)surface_header_size + ledge_data_size_z + (size_t)node_size;
    size_t total_padded_z = (total_unpadded_z + 15u) & ~(size_t)15u;

    if (ledge_data_size_z > (size_t)INT_MAX || total_padded_z > (size_t)INT_MAX) {
        ivp_free(alloc, opp_idx, (size_t)n_triangles * 3u * sizeof(int));
        return NULL;
    }

    const int ledge_data_size = (int)ledge_data_size_z;
    const int total_padded = (int)total_padded_z;

    void *mem = ivp_alloc_aligned(alloc, (size_t)total_padded, 16);
    if (!mem) {
        ivp_free(alloc, opp_idx, (size_t)n_triangles * 3u * sizeof(int));
        return NULL;
    }
    memset(mem, 0, (size_t)total_padded);

    ivp_compact_surface_t *surface = (ivp_compact_surface_t *)mem;
    ivp_compact_ledge_t *ledge =
        (ivp_compact_ledge_t *)((char *)mem + surface_header_size);
    ivp_compact_triangle_t *tris =
        (ivp_compact_triangle_t *)((char *)ledge + ledge_header_size);
    ivp_compact_poly_point_t *pts =
        (ivp_compact_poly_point_t *)((char *)tris + n_triangles * tri_size);
    ivp_compact_ledgetree_node_t *node =
        (ivp_compact_ledgetree_node_t *)((char *)ledge + ledge_data_size);

    /* Write vertices */
    for (int i = 0; i < n_vertices; i++) {
        pts[i].k[0] = vertices[i * 3 + 0];
        pts[i].k[1] = vertices[i * 3 + 1];
        pts[i].k[2] = vertices[i * 3 + 2];
        pts[i].hesse_val = 0.0f;
    }

    /* Write triangles + edges */
    for (int t = 0; t < n_triangles; t++) {
        ivp_compact_triangle_t *tri = &tris[t];
        ivp_tri_init(tri);
        ivp_tri_set_tri_index(tri, t);
        ivp_tri_set_pierce_index(tri, 0);
        ivp_tri_set_material_index(tri, 0);
        ivp_tri_set_is_virtual(tri, 0);

        for (int e = 0; e < 3; e++) {
            int edge_i = t * 3 + e;
            ivp_edge_set(&tri->edges[e],
                         indices[t * 3 + e],
                         opp_idx[edge_i],
                         0);
        }
    }

    /* Ledge header */
    ivp_ledge_init(ledge);
    ledge->c_point_offset = ledge_header_size + n_triangles * tri_size;
    ledge->node_or_client = ledge_data_size;
    ivp_ledge_set_has_children(ledge, 0);
    ivp_ledge_set_is_compact(ledge, 1);
    ivp_ledge_set_size(ledge, ledge_data_size);
    ledge->n_triangles = (int16_t)n_triangles;
    ledge->reserved = 0;

    /* Initialize root linkage before mass/inertia traversal. */
    node->offset_right_node = 0;
    node->offset_compact_ledge = (int32_t)((char *)ledge - (char *)node);
    surface->offset_ledgetree_root = (int32_t)((char *)node - (char *)surface);

    /* Compute center/radius/inertia */
    float mass_center[3], rot_inertia[3];
    ivp_calc_mass_center_and_rot_inertia(surface, mass_center, rot_inertia);

    float radius = 0.0f;
    for (int i = 0; i < n_vertices; i++) {
        float dx = pts[i].k[0] - mass_center[0];
        float dy = pts[i].k[1] - mass_center[1];
        float dz = pts[i].k[2] - mass_center[2];
        float r = sqrtf(dx * dx + dy * dy + dz * dz);
        if (r > radius) radius = r;
    }

    surface->mass_center[0] = mass_center[0];
    surface->mass_center[1] = mass_center[1];
    surface->mass_center[2] = mass_center[2];
    surface->rotation_inertia[0] = rot_inertia[0];
    surface->rotation_inertia[1] = rot_inertia[1];
    surface->rotation_inertia[2] = rot_inertia[2];
    surface->upper_limit_radius = radius;
    ivp_surface_set_max_deviation(surface, 1);
    ivp_surface_set_byte_size(surface, total_padded);

    /* Single terminal ledgetree node */
    node->center[0] = mass_center[0];
    node->center[1] = mass_center[1];
    node->center[2] = mass_center[2];
    node->radius = radius;

    if (radius > 0.0f) {
        float inv_step = 1.0f / (radius * IVP_COMPACT_BOUNDINGBOX_STEP_SIZE);
        for (int i = 0; i < n_vertices; i++) {
            float dx = fabsf(pts[i].k[0] - mass_center[0]);
            float dy = fabsf(pts[i].k[1] - mass_center[1]);
            float dz = fabsf(pts[i].k[2] - mass_center[2]);
            float bx = dx * inv_step + 1.0f;
            float by = dy * inv_step + 1.0f;
            float bz = dz * inv_step + 1.0f;
            if ((int)bx > node->box_sizes[0])
                node->box_sizes[0] = (uint8_t)((int)bx > 255 ? 255 : (int)bx);
            if ((int)by > node->box_sizes[1])
                node->box_sizes[1] = (uint8_t)((int)by > 255 ? 255 : (int)by);
            if ((int)bz > node->box_sizes[2])
                node->box_sizes[2] = (uint8_t)((int)bz > 255 ? 255 : (int)bz);
        }
    } else {
        node->box_sizes[0] = 1;
        node->box_sizes[1] = 1;
        node->box_sizes[2] = 1;
    }
    node->padding = 0;

    ivp_free(alloc, opp_idx, (size_t)n_triangles * 3u * sizeof(int));
    return surface;
}

/* ── Compact surface vertex modification ────────────────────────────────── */

int ivp_surface_modify_vertex(ivp_compact_surface_t *surface,
                                int vertex_index,
                                const float new_pos[3])
{
    if (!surface || !new_pos) return -1;

    /* Get the single convex ledge */
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

    /* Recompute mass center and inertia */
    float mass_center[3], rot_inertia[3];
    ivp_calc_mass_center_and_rot_inertia(surface, mass_center, rot_inertia);

    surface->mass_center[0] = mass_center[0];
    surface->mass_center[1] = mass_center[1];
    surface->mass_center[2] = mass_center[2];
    surface->rotation_inertia[0] = rot_inertia[0];
    surface->rotation_inertia[1] = rot_inertia[1];
    surface->rotation_inertia[2] = rot_inertia[2];

    /* Recompute bounding radius from mass center */
    const ivp_compact_ledgetree_node_t *root = ivp_surface_get_ledgetree_root(surface);
    /* Only handle terminal (single-ledge) surfaces */
    if (root->offset_right_node != 0) return;

    const ivp_compact_ledge_t *ledge =
        (const ivp_compact_ledge_t *)((const char *)root +
                                       root->offset_compact_ledge);
    int n_points = ivp_ledge_get_n_points(ledge);
    const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(ledge);

    float radius = 0.0f;
    for (int i = 0; i < n_points; i++) {
        float dx = pts[i].k[0] - mass_center[0];
        float dy = pts[i].k[1] - mass_center[1];
        float dz = pts[i].k[2] - mass_center[2];
        float r = sqrtf(dx * dx + dy * dy + dz * dz);
        if (r > radius) radius = r;
    }
    surface->upper_limit_radius = radius;

    /* Update ledgetree node */
    ivp_compact_ledgetree_node_t *node_mut =
        (ivp_compact_ledgetree_node_t *)((char *)surface +
                                          surface->offset_ledgetree_root);
    node_mut->center[0] = mass_center[0];
    node_mut->center[1] = mass_center[1];
    node_mut->center[2] = mass_center[2];
    node_mut->radius = radius;

    if (radius > 0.0f) {
        float inv_step = 1.0f / (radius * IVP_COMPACT_BOUNDINGBOX_STEP_SIZE);
        node_mut->box_sizes[0] = 0;
        node_mut->box_sizes[1] = 0;
        node_mut->box_sizes[2] = 0;
        for (int i = 0; i < n_points; i++) {
            int bx = (int)(fabsf(pts[i].k[0] - mass_center[0]) * inv_step) + 1;
            int by = (int)(fabsf(pts[i].k[1] - mass_center[1]) * inv_step) + 1;
            int bz = (int)(fabsf(pts[i].k[2] - mass_center[2]) * inv_step) + 1;
            if (bx > node_mut->box_sizes[0])
                node_mut->box_sizes[0] = (uint8_t)(bx > 255 ? 255 : bx);
            if (by > node_mut->box_sizes[1])
                node_mut->box_sizes[1] = (uint8_t)(by > 255 ? 255 : by);
            if (bz > node_mut->box_sizes[2])
                node_mut->box_sizes[2] = (uint8_t)(bz > 255 ? 255 : bz);
        }
    } else {
        node_mut->box_sizes[0] = 1;
        node_mut->box_sizes[1] = 1;
        node_mut->box_sizes[2] = 1;
    }
}
