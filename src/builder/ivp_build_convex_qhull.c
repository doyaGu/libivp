/* ivp_build_convex_qhull.c — Convex compact-surface builder via QHull (libqhull_r)
 */

#include <ivp/builder/ivp_build_surface.h>
#include <ivp/base/ivp_assert.h>
#include "ivp_build_edge_map_internal.h"

#include <libqhull_r/qhull_ra.h>

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ── Internal helpers ───────────────────────────────────────────────────── */

typedef struct ivp_tri_idx {
    int v[3];
} ivp_tri_idx_t;

#define IVP_TRI_INDEX_LIMIT (1 << 12)

static int ivp_build_opposites(const ivp_allocator_t *alloc,
                                const ivp_tri_idx_t *tris, int n_tri,
                                int *out_opp)
{
    if (!alloc || !tris || !out_opp || n_tri <= 0) return 0;
    if (n_tri > IVP_TRI_INDEX_LIMIT) return 0;
    if (n_tri > INT_MAX / 3) return 0;

    const int n_edges = n_tri * 3;
    if (n_edges > INT_MAX / 4) return 0;

    ivp_build_edge_map_t edge_map;
    if (!ivp_build_edge_map_init(&edge_map, alloc, n_edges)) return 0;

    for (int i = 0; i < n_edges; i++) out_opp[i] = -1;

    for (int t = 0; t < n_tri; t++) {
        for (int e = 0; e < 3; e++) {
            int a = tris[t].v[e];
            int b = tris[t].v[(e + 1) % 3];

            int this_linear = t * 4 + e;
            uint64_t rev_key = ivp_build_make_edge_key(b, a);
            int other_linear = 0;
            if (ivp_build_edge_map_get_linear(&edge_map, rev_key, &other_linear)) {
                int other_edge_idx = (other_linear / 4) * 3 + (other_linear % 4);
                int this_edge_idx  = t * 3 + e;
                out_opp[this_edge_idx] = other_linear - this_linear;
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

    for (int i = 0; i < n_edges; i++) {
        if (out_opp[i] == -1) {
            return 0;
        }
    }
    return 1;
}

static void ivp_compute_center_radius(const ivp_vec3f_t *pts, int n,
                                       float out_center[3], float *out_radius)
{
    double cx = 0.0, cy = 0.0, cz = 0.0;
    for (int i = 0; i < n; i++) {
        cx += (double)pts[i].k[0];
        cy += (double)pts[i].k[1];
        cz += (double)pts[i].k[2];
    }
    cx /= (double)n;
    cy /= (double)n;
    cz /= (double)n;

    float radius = 0.0f;
    for (int i = 0; i < n; i++) {
        float dx = pts[i].k[0] - (float)cx;
        float dy = pts[i].k[1] - (float)cy;
        float dz = pts[i].k[2] - (float)cz;
        float d2 = dx * dx + dy * dy + dz * dz;
        float d = sqrtf(d2);
        if (d > radius) radius = d;
    }

    out_center[0] = (float)cx;
    out_center[1] = (float)cy;
    out_center[2] = (float)cz;
    *out_radius = radius;
}

static void ivp_compute_diag_inertia_approx(const ivp_vec3f_t *pts, int n,
                                             const float center[3],
                                             float out_I[3])
{
    float min_x = pts[0].k[0], max_x = pts[0].k[0];
    float min_y = pts[0].k[1], max_y = pts[0].k[1];
    float min_z = pts[0].k[2], max_z = pts[0].k[2];

    for (int i = 1; i < n; i++) {
        if (pts[i].k[0] < min_x) min_x = pts[i].k[0];
        if (pts[i].k[0] > max_x) max_x = pts[i].k[0];
        if (pts[i].k[1] < min_y) min_y = pts[i].k[1];
        if (pts[i].k[1] > max_y) max_y = pts[i].k[1];
        if (pts[i].k[2] < min_z) min_z = pts[i].k[2];
        if (pts[i].k[2] > max_z) max_z = pts[i].k[2];
    }

    float hx = 0.5f * (max_x - min_x);
    float hy = 0.5f * (max_y - min_y);
    float hz = 0.5f * (max_z - min_z);

    out_I[0] = (hy * hy + hz * hz) / 3.0f;
    out_I[1] = (hx * hx + hz * hz) / 3.0f;
    out_I[2] = (hx * hx + hy * hy) / 3.0f;

    IVP_UNUSED(center);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

ivp_compact_surface_t *ivp_build_convex_surface(const ivp_allocator_t *alloc,
                                                 const ivp_vec3f_t *points,
                                                 int n_points)
{
    if (!points || n_points < 4) return NULL;
    if (n_points > INT_MAX / 3) return NULL;
    if (!alloc) alloc = ivp_default_allocator();

    ivp_compact_surface_t *surface = NULL;
    coordT *qh_points = NULL;
    ivp_tri_idx_t *triangles_idx = NULL;
    int *id_to_compact = NULL;
    int *opp_idx = NULL;
    int n_tri = 0;

    qhT qh_qh;
    qhT *qh = &qh_qh;
    FILE *errfile = stderr;
    boolT ismalloc = False;

    qh_zero(qh, errfile);

    if ((size_t)n_points > SIZE_MAX / (3u * sizeof(coordT))) goto fail;
    qh_points = (coordT *)ivp_alloc(alloc, (size_t)n_points * 3u * sizeof(coordT));
    if (!qh_points) goto fail;

    for (int i = 0; i < n_points; i++) {
        qh_points[i * 3 + 0] = (coordT)points[i].k[0];
        qh_points[i * 3 + 1] = (coordT)points[i].k[1];
        qh_points[i * 3 + 2] = (coordT)points[i].k[2];
    }

    int exitcode = qh_new_qhull(qh, 3, n_points, qh_points, ismalloc,
                                "qhull Qt", NULL, errfile);
    if (exitcode != qh_ERRnone) goto fail;

    facetT *facet;
    FORALLfacets {
        if (!facet->upperdelaunay) {
            n_tri++;
            if (n_tri > IVP_TRI_INDEX_LIMIT) goto fail;
        }
    }
    if (n_tri <= 0) goto fail;

    if ((size_t)n_tri > SIZE_MAX / sizeof(ivp_tri_idx_t)) goto fail;
    if ((size_t)n_points > SIZE_MAX / sizeof(int)) goto fail;

    triangles_idx = (ivp_tri_idx_t *)ivp_alloc(alloc,
                    (size_t)n_tri * sizeof(ivp_tri_idx_t));
    id_to_compact = (int *)ivp_alloc(alloc, (size_t)n_points * sizeof(int));
    if (!triangles_idx || !id_to_compact) goto fail;

    for (int i = 0; i < n_points; i++) id_to_compact[i] = -1;

    int tri_write = 0;
    int compact_n_points = 0;

    FORALLfacets {
        if (facet->upperdelaunay) continue;

        setT *vertices = qh_facet3vertex(qh, facet);
        if (!vertices || qh_setsize(qh, vertices) != 3) {
            qh_settempfree(qh, &vertices);
            goto fail;
        }

        for (int k = 0; k < 3; k++) {
            vertexT *v = (vertexT *)SETelem_(vertices, k);
            int src_id = qh_pointid(qh, v->point);
            if (src_id < 0 || src_id >= n_points) {
                qh_settempfree(qh, &vertices);
                goto fail;
            }

            if (id_to_compact[src_id] < 0) {
                id_to_compact[src_id] = compact_n_points++;
            }
            triangles_idx[tri_write].v[k] = id_to_compact[src_id];
        }

        qh_settempfree(qh, &vertices);
        if (tri_write >= n_tri) goto fail;
        tri_write++;
    }

    if (tri_write != n_tri || compact_n_points < 4) goto fail;
    if (compact_n_points >= (1 << 16)) goto fail;

    if ((size_t)n_tri > SIZE_MAX / (3u * sizeof(int))) goto fail;
    opp_idx = (int *)ivp_alloc(alloc, (size_t)n_tri * 3u * sizeof(int));
    if (!opp_idx) goto fail;

    if (!ivp_build_opposites(alloc, triangles_idx, n_tri, opp_idx)) goto fail;

    /* Memory layout */
    const int surface_header_size = (int)sizeof(ivp_compact_surface_t);
    const int ledge_header_size = (int)sizeof(ivp_compact_ledge_t);
    const int tri_size = (int)sizeof(ivp_compact_triangle_t);
    const int point_size = (int)sizeof(ivp_compact_poly_point_t);
    const int node_size = (int)sizeof(ivp_compact_ledgetree_node_t);

    if ((size_t)n_tri > SIZE_MAX / (size_t)tri_size) goto fail;
    if ((size_t)compact_n_points > SIZE_MAX / (size_t)point_size) goto fail;

    size_t ledge_data_size_z = (size_t)ledge_header_size
                             + (size_t)n_tri * (size_t)tri_size
                             + (size_t)compact_n_points * (size_t)point_size;
    size_t total_unpadded_z = (size_t)surface_header_size + ledge_data_size_z + (size_t)node_size;
    size_t total_padded_z = (total_unpadded_z + 15u) & ~(size_t)15u;

    if (ledge_data_size_z > (size_t)INT_MAX) goto fail;
    if (total_padded_z > (size_t)INT_MAX) goto fail;

    const int ledge_data_size = (int)ledge_data_size_z;
    const int total_padded = (int)total_padded_z;

    void *mem = ivp_alloc_aligned(alloc, (size_t)total_padded, 16);
    if (!mem) goto fail;
    memset(mem, 0, (size_t)total_padded);

    surface = (ivp_compact_surface_t *)mem;
    ivp_compact_ledge_t *ledge = (ivp_compact_ledge_t *)((char *)mem + surface_header_size);
    ivp_compact_triangle_t *tris = (ivp_compact_triangle_t *)((char *)ledge + ledge_header_size);
    ivp_compact_poly_point_t *out_pts = (ivp_compact_poly_point_t *)((char *)tris + n_tri * tri_size);
    ivp_compact_ledgetree_node_t *node = (ivp_compact_ledgetree_node_t *)((char *)ledge + ledge_data_size);

    /* Write compact points in deterministic remap order */
    for (int src = 0; src < n_points; src++) {
        int dst = id_to_compact[src];
        if (dst >= 0) {
            out_pts[dst].k[0] = points[src].k[0];
            out_pts[dst].k[1] = points[src].k[1];
            out_pts[dst].k[2] = points[src].k[2];
            out_pts[dst].hesse_val = 0.0f;
        }
    }

    /* Write triangles+edges */
    for (int t = 0; t < n_tri; t++) {
        ivp_compact_triangle_t *tri = &tris[t];
        ivp_tri_init(tri);
        ivp_tri_set_tri_index(tri, t);
        ivp_tri_set_pierce_index(tri, 0);
        ivp_tri_set_material_index(tri, 0);
        ivp_tri_set_is_virtual(tri, 0);

        for (int e = 0; e < 3; e++) {
            int edge_i = t * 3 + e;
            ivp_edge_set(&tri->edges[e],
                         triangles_idx[t].v[e],
                         opp_idx[edge_i],
                         0);
        }
    }

    /* Ledge header */
    ivp_ledge_init(ledge);
    ledge->c_point_offset = ledge_header_size + n_tri * tri_size;
    ledge->node_or_client = ledge_data_size;
    ivp_ledge_set_has_children(ledge, 0);
    ivp_ledge_set_is_compact(ledge, 1);
    ivp_ledge_set_size(ledge, ledge_data_size);
    ledge->n_triangles = (int16_t)n_tri;
    ledge->reserved = 0;

    /* Surface center/radius/inertia (approx for M2) */
    float center[3];
    float radius;
    float inertia[3];

    if ((size_t)compact_n_points > SIZE_MAX / sizeof(ivp_vec3f_t)) goto fail_after_surface;
    ivp_vec3f_t *temp_pts = (ivp_vec3f_t *)ivp_alloc(alloc,
                               (size_t)compact_n_points * sizeof(ivp_vec3f_t));
    if (!temp_pts) goto fail_after_surface;

    for (int i = 0; i < compact_n_points; i++) {
        temp_pts[i] = ivp_vec3f(out_pts[i].k[0], out_pts[i].k[1], out_pts[i].k[2]);
    }

    ivp_compute_center_radius(temp_pts, compact_n_points, center, &radius);
    ivp_compute_diag_inertia_approx(temp_pts, compact_n_points, center, inertia);
    ivp_free(alloc, temp_pts, (size_t)compact_n_points * sizeof(ivp_vec3f_t));

    surface->mass_center[0] = center[0];
    surface->mass_center[1] = center[1];
    surface->mass_center[2] = center[2];
    surface->rotation_inertia[0] = inertia[0];
    surface->rotation_inertia[1] = inertia[1];
    surface->rotation_inertia[2] = inertia[2];
    surface->upper_limit_radius = radius;
    ivp_surface_set_max_deviation(surface, 1);
    ivp_surface_set_byte_size(surface, total_padded);
    surface->offset_ledgetree_root = (int32_t)((char *)node - (char *)surface);

    /* Single terminal ledgetree node */
    node->offset_right_node = 0;
    node->offset_compact_ledge = (int32_t)((char *)ledge - (char *)node);
    node->center[0] = center[0];
    node->center[1] = center[1];
    node->center[2] = center[2];
    node->radius = radius;

    float ex = 0.0f, ey = 0.0f, ez = 0.0f;
    for (int i = 0; i < compact_n_points; i++) {
        float dx = fabsf(out_pts[i].k[0] - center[0]);
        float dy = fabsf(out_pts[i].k[1] - center[1]);
        float dz = fabsf(out_pts[i].k[2] - center[2]);
        if (dx > ex) ex = dx;
        if (dy > ey) ey = dy;
        if (dz > ez) ez = dz;
    }

    if (radius > 0.0f) {
        float inv_step = 1.0f / (radius * IVP_COMPACT_BOUNDINGBOX_STEP_SIZE);
        int bx = (int)(ex * inv_step) + 1;
        int by = (int)(ey * inv_step) + 1;
        int bz = (int)(ez * inv_step) + 1;
        node->box_sizes[0] = (uint8_t)(bx > 255 ? 255 : bx);
        node->box_sizes[1] = (uint8_t)(by > 255 ? 255 : by);
        node->box_sizes[2] = (uint8_t)(bz > 255 ? 255 : bz);
    } else {
        node->box_sizes[0] = 1;
        node->box_sizes[1] = 1;
        node->box_sizes[2] = 1;
    }
    node->padding = 0;

    /* Cleanup qhull */
    qh_freeqhull(qh, !qh_ALL);
    {
        int curlong = 0, totlong = 0;
        qh_memfreeshort(qh, &curlong, &totlong);
    }

    ivp_free(alloc, qh_points, (size_t)n_points * 3u * sizeof(coordT));
    ivp_free(alloc, triangles_idx, (size_t)n_tri * sizeof(ivp_tri_idx_t));
    ivp_free(alloc, id_to_compact, (size_t)n_points * sizeof(int));
    ivp_free(alloc, opp_idx, (size_t)n_tri * 3u * sizeof(int));
    return surface;

fail_after_surface:
    ivp_free_aligned(alloc, surface);
    surface = NULL;

fail:
    qh_settempfree_all(qh);
    qh_freeqhull(qh, !qh_ALL);
    {
        int curlong = 0, totlong = 0;
        qh_memfreeshort(qh, &curlong, &totlong);
    }
    if (qh_points) {
        ivp_free(alloc, qh_points, (size_t)n_points * 3u * sizeof(coordT));
    }
    if (triangles_idx) {
        ivp_free(alloc, triangles_idx, (size_t)n_tri * sizeof(ivp_tri_idx_t));
    }
    if (id_to_compact) {
        ivp_free(alloc, id_to_compact, (size_t)n_points * sizeof(int));
    }
    if (opp_idx) {
        ivp_free(alloc, opp_idx, (size_t)n_tri * 3u * sizeof(int));
    }
    return NULL;
}
