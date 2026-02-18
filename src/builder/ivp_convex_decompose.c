/* ivp_convex_decompose.c -- Approximate convex decomposition */

#include <ivp/builder/ivp_convex_decompose.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/math/ivp_vec3.h>
#include <string.h>
#include <math.h>

/* ── Triangle normal ───────────────────────────────────────────────────── */

static void compute_triangle_normal(const float *verts, const int *tri,
                                      float out[3])
{
    const float *a = &verts[tri[0] * 3];
    const float *b = &verts[tri[1] * 3];
    const float *c = &verts[tri[2] * 3];
    float ab[3] = { b[0]-a[0], b[1]-a[1], b[2]-a[2] };
    float ac[3] = { c[0]-a[0], c[1]-a[1], c[2]-a[2] };
    out[0] = ab[1]*ac[2] - ab[2]*ac[1];
    out[1] = ab[2]*ac[0] - ab[0]*ac[2];
    out[2] = ab[0]*ac[1] - ab[1]*ac[0];
    float len = sqrtf(out[0]*out[0] + out[1]*out[1] + out[2]*out[2]);
    if (len > 1e-10f) {
        float inv = 1.0f / len;
        out[0] *= inv; out[1] *= inv; out[2] *= inv;
    }
}

/* ── Adjacency building ────────────────────────────────────────────────── */

/* Simple edge hash: for each edge (v0, v1) where v0 < v1,
 * find the other triangle sharing that edge. */

typedef struct edge_entry {
    int v0, v1;       /* sorted vertex indices */
    int tri_index;    /* triangle that owns this edge */
    int next;         /* linked list in hash bucket */
} edge_entry_t;

static int edge_hash_fn(int v0, int v1, int table_size)
{
    unsigned int h = (unsigned int)v0 * 73856093u ^ (unsigned int)v1 * 19349663u;
    return (int)(h % (unsigned int)table_size);
}

static void build_adjacency(const int *indices, int n_tris,
                              int *adj,  /* n_tris * 3: adj[tri*3+e] = neighbor tri or -1 */
                              const ivp_allocator_t *alloc)
{
    int n_edges = n_tris * 3;
    int table_size = n_edges * 2 + 1;

    int *buckets = (int *)ivp_alloc(alloc, (size_t)table_size * sizeof(int));
    edge_entry_t *entries = (edge_entry_t *)ivp_alloc(
        alloc, (size_t)n_edges * sizeof(edge_entry_t));
    if (!buckets || !entries) {
        if (buckets) ivp_free(alloc, buckets, (size_t)table_size * sizeof(int));
        if (entries) ivp_free(alloc, entries, (size_t)n_edges * sizeof(edge_entry_t));
        for (int i = 0; i < n_tris * 3; i++) adj[i] = -1;
        return;
    }

    for (int i = 0; i < table_size; i++) buckets[i] = -1;
    for (int i = 0; i < n_tris * 3; i++) adj[i] = -1;

    int entry_count = 0;

    for (int t = 0; t < n_tris; t++) {
        for (int e = 0; e < 3; e++) {
            int va = indices[t * 3 + e];
            int vb = indices[t * 3 + (e + 1) % 3];
            int v0 = va < vb ? va : vb;
            int v1 = va < vb ? vb : va;
            int h = edge_hash_fn(v0, v1, table_size);

            /* Search for matching edge */
            int found = -1;
            for (int ei = buckets[h]; ei >= 0; ei = entries[ei].next) {
                if (entries[ei].v0 == v0 && entries[ei].v1 == v1) {
                    found = ei;
                    break;
                }
            }

            if (found >= 0) {
                /* Link adjacency both ways */
                int other_tri = entries[found].tri_index;
                adj[t * 3 + e] = other_tri;
                /* Find which edge of other_tri matches */
                for (int oe = 0; oe < 3; oe++) {
                    int oa = indices[other_tri * 3 + oe];
                    int ob = indices[other_tri * 3 + (oe + 1) % 3];
                    int o0 = oa < ob ? oa : ob;
                    int o1 = oa < ob ? ob : oa;
                    if (o0 == v0 && o1 == v1) {
                        adj[other_tri * 3 + oe] = t;
                        break;
                    }
                }
            } else {
                /* Insert new edge */
                edge_entry_t *ent = &entries[entry_count];
                ent->v0 = v0;
                ent->v1 = v1;
                ent->tri_index = t;
                ent->next = buckets[h];
                buckets[h] = entry_count;
                entry_count++;
            }
        }
    }

    ivp_free(alloc, entries, (size_t)n_edges * sizeof(edge_entry_t));
    ivp_free(alloc, buckets, (size_t)table_size * sizeof(int));
}

/* ── Flood fill decomposition ──────────────────────────────────────────── */

ivp_result_t ivp_convex_decompose(
    const ivp_allocator_t *alloc,
    const float *vertices,
    int n_vertices,
    const int *indices,
    int n_triangles,
    const ivp_decompose_options_t *options,
    ivp_decompose_result_t *result)
{
    if (!vertices || !indices || !result || n_triangles < 1 || n_vertices < 3)
        return IVP_ERR_INVALID_ARG;

    if (!alloc) alloc = ivp_default_allocator();
    memset(result, 0, sizeof(*result));
    result->allocator = alloc;

    ivp_decompose_options_t opts;
    if (options) opts = *options;
    else opts = ivp_decompose_options_default();

    float cos_threshold = cosf(opts.angle_threshold_deg * 3.14159265f / 180.0f);

    /* Compute triangle normals */
    float *normals = (float *)ivp_alloc(alloc, (size_t)n_triangles * 3 * sizeof(float));
    int *adj = (int *)ivp_alloc(alloc, (size_t)n_triangles * 3 * sizeof(int));
    int *part_ids = (int *)ivp_alloc(alloc, (size_t)n_triangles * sizeof(int));
    int *stack = (int *)ivp_alloc(alloc, (size_t)n_triangles * sizeof(int));

    if (!normals || !adj || !part_ids || !stack) {
        if (normals) ivp_free(alloc, normals, (size_t)n_triangles * 3 * sizeof(float));
        if (adj) ivp_free(alloc, adj, (size_t)n_triangles * 3 * sizeof(int));
        if (part_ids) ivp_free(alloc, part_ids, (size_t)n_triangles * sizeof(int));
        if (stack) ivp_free(alloc, stack, (size_t)n_triangles * sizeof(int));
        return IVP_ERR_OUT_OF_MEMORY;
    }

    for (int i = 0; i < n_triangles; i++) {
        compute_triangle_normal(vertices, &indices[i * 3], &normals[i * 3]);
        part_ids[i] = -1;
    }

    /* Build adjacency */
    build_adjacency(indices, n_triangles, adj, alloc);

    /* Flood fill: group triangles by normal similarity */
    int n_parts = 0;
    for (int seed = 0; seed < n_triangles && n_parts < opts.max_parts; seed++) {
        if (part_ids[seed] >= 0) continue;

        int part = n_parts++;
        int sp = 0;
        stack[sp++] = seed;
        part_ids[seed] = part;

        while (sp > 0) {
            int t = stack[--sp];
            for (int e = 0; e < 3; e++) {
                int neighbor = adj[t * 3 + e];
                if (neighbor < 0 || part_ids[neighbor] >= 0) continue;

                /* Check normal similarity */
                float dot = normals[t*3+0] * normals[neighbor*3+0] +
                            normals[t*3+1] * normals[neighbor*3+1] +
                            normals[t*3+2] * normals[neighbor*3+2];

                if (dot >= cos_threshold) {
                    part_ids[neighbor] = part;
                    stack[sp++] = neighbor;
                }
            }
        }
    }

    /* Assign remaining unassigned triangles to nearest part */
    for (int i = 0; i < n_triangles; i++) {
        if (part_ids[i] < 0) {
            /* Find an adjacent part */
            for (int e = 0; e < 3; e++) {
                int nb = adj[i * 3 + e];
                if (nb >= 0 && part_ids[nb] >= 0) {
                    part_ids[i] = part_ids[nb];
                    break;
                }
            }
            if (part_ids[i] < 0) {
                part_ids[i] = n_parts > 0 ? 0 : -1;
            }
        }
    }

    /* Build convex hull for each part */
    ivp_compact_surface_t **tmp_parts = (ivp_compact_surface_t **)ivp_alloc(
        alloc, (size_t)n_parts * sizeof(ivp_compact_surface_t *));
    if (!tmp_parts) {
        ivp_free(alloc, normals, (size_t)n_triangles * 3 * sizeof(float));
        ivp_free(alloc, adj, (size_t)n_triangles * 3 * sizeof(int));
        ivp_free(alloc, part_ids, (size_t)n_triangles * sizeof(int));
        ivp_free(alloc, stack, (size_t)n_triangles * sizeof(int));
        return IVP_ERR_OUT_OF_MEMORY;
    }
    for (int i = 0; i < n_parts; i++) tmp_parts[i] = NULL;

    /* Temporary buffer for collecting vertices per part */
    ivp_vec3f_t *part_verts = (ivp_vec3f_t *)ivp_alloc(
        alloc, (size_t)n_vertices * sizeof(ivp_vec3f_t));
    /* Mark buffer for dedup */
    int *vert_mark = (int *)ivp_alloc(alloc, (size_t)n_vertices * sizeof(int));
    if (!part_verts || !vert_mark) {
        if (vert_mark)
            ivp_free(alloc, vert_mark, (size_t)n_vertices * sizeof(int));
        if (part_verts)
            ivp_free(alloc, part_verts, (size_t)n_vertices * sizeof(ivp_vec3f_t));
        ivp_free(alloc, tmp_parts, (size_t)n_parts * sizeof(ivp_compact_surface_t *));
        ivp_free(alloc, stack, (size_t)n_triangles * sizeof(int));
        ivp_free(alloc, part_ids, (size_t)n_triangles * sizeof(int));
        ivp_free(alloc, adj, (size_t)n_triangles * 3 * sizeof(int));
        ivp_free(alloc, normals, (size_t)n_triangles * 3 * sizeof(float));
        return IVP_ERR_OUT_OF_MEMORY;
    }

    int actual_parts = 0;
    for (int p = 0; p < n_parts; p++) {
        /* Collect unique vertices for this part */
        for (int v = 0; v < n_vertices; v++) vert_mark[v] = 0;
        int pv_count = 0;

        for (int t = 0; t < n_triangles; t++) {
            if (part_ids[t] != p) continue;
            for (int vi = 0; vi < 3; vi++) {
                int idx = indices[t * 3 + vi];
                if (!vert_mark[idx]) {
                    vert_mark[idx] = 1;
                    part_verts[pv_count].k[0] = vertices[idx * 3 + 0];
                    part_verts[pv_count].k[1] = vertices[idx * 3 + 1];
                    part_verts[pv_count].k[2] = vertices[idx * 3 + 2];
                    pv_count++;
                }
            }
        }

        if (pv_count < 4) continue; /* Skip degenerate parts */

        ivp_compact_surface_t *hull =
            ivp_build_convex_surface(alloc, part_verts, pv_count);
        if (hull) {
            tmp_parts[actual_parts++] = hull;
        }
    }

    if (actual_parts > 0) {
        result->parts = (ivp_compact_surface_t **)ivp_alloc(
            alloc, (size_t)actual_parts * sizeof(ivp_compact_surface_t *));
        if (!result->parts) {
            for (int i = 0; i < actual_parts; i++) {
                if (tmp_parts[i]) {
                    ivp_compact_surface_free(alloc, tmp_parts[i]);
                }
            }
            ivp_free(alloc, tmp_parts, (size_t)n_parts * sizeof(ivp_compact_surface_t *));
            ivp_free(alloc, vert_mark, (size_t)n_vertices * sizeof(int));
            ivp_free(alloc, part_verts, (size_t)n_vertices * sizeof(ivp_vec3f_t));
            ivp_free(alloc, stack, (size_t)n_triangles * sizeof(int));
            ivp_free(alloc, part_ids, (size_t)n_triangles * sizeof(int));
            ivp_free(alloc, adj, (size_t)n_triangles * 3 * sizeof(int));
            ivp_free(alloc, normals, (size_t)n_triangles * 3 * sizeof(float));
            return IVP_ERR_OUT_OF_MEMORY;
        }
        memcpy(result->parts, tmp_parts,
               (size_t)actual_parts * sizeof(ivp_compact_surface_t *));
    } else {
        result->parts = NULL;
    }
    result->n_parts = actual_parts;
    ivp_free(alloc, tmp_parts, (size_t)n_parts * sizeof(ivp_compact_surface_t *));

    /* Cleanup */
    ivp_free(alloc, vert_mark, (size_t)n_vertices * sizeof(int));
    ivp_free(alloc, part_verts, (size_t)n_vertices * sizeof(ivp_vec3f_t));
    ivp_free(alloc, stack, (size_t)n_triangles * sizeof(int));
    ivp_free(alloc, part_ids, (size_t)n_triangles * sizeof(int));
    ivp_free(alloc, adj, (size_t)n_triangles * 3 * sizeof(int));
    ivp_free(alloc, normals, (size_t)n_triangles * 3 * sizeof(float));

    return actual_parts > 0 ? IVP_OK : IVP_ERR_INVALID_ARG;
}

void ivp_decompose_result_free(ivp_decompose_result_t *result)
{
    if (!result || !result->allocator) return;

    for (int i = 0; i < result->n_parts; i++) {
        if (result->parts[i]) {
            ivp_compact_surface_free(result->allocator, result->parts[i]);
        }
    }

    if (result->parts) {
        ivp_free(result->allocator, result->parts,
                 (size_t)result->n_parts * sizeof(ivp_compact_surface_t *));
    }

    result->parts = NULL;
    result->n_parts = 0;
}
