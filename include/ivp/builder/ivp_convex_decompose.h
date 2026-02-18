/* ivp_convex_decompose.h -- Approximate convex decomposition
 *
 * Decomposes a concave triangle mesh into convex parts suitable for
 * building a ledge soup. Uses a simple flood-fill approach based on
 * face normal clustering.
 *
 * Algorithm:
 *   1. Build adjacency graph from triangle mesh.
 *   2. Seed flood-fill: group adjacent triangles whose normals differ
 *      by less than the angle threshold.
 *   3. For each group, compute convex hull of its vertices.
 *   4. Output an array of convex hulls (compact surfaces).
 *
 * For production use, consider a more sophisticated HACD or V-HACD
 * algorithm. This implementation provides a reasonable baseline.
 *
 * Ported from ivp_convex_decompositor.hxx.
 */
#ifndef IVP_CONVEX_DECOMPOSE_H
#define IVP_CONVEX_DECOMPOSE_H

#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/base/ivp_allocator.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Decomposition options ─────────────────────────────────────────────── */

typedef struct ivp_decompose_options {
    float angle_threshold_deg;  /* max normal angle within a part (default: 30) */
    int   max_parts;            /* max convex parts (default: 64) */
    int   min_triangles_per_part; /* min tris per part (default: 4) */
} ivp_decompose_options_t;

static inline ivp_decompose_options_t ivp_decompose_options_default(void) {
    ivp_decompose_options_t o;
    o.angle_threshold_deg = 30.0f;
    o.max_parts = 64;
    o.min_triangles_per_part = 4;
    return o;
}

/* ── Decomposition result ──────────────────────────────────────────────── */

typedef struct ivp_decompose_result {
    ivp_compact_surface_t **parts;  /* array of convex hull surfaces */
    int                     n_parts;
    const ivp_allocator_t  *allocator;
} ivp_decompose_result_t;

/* Free all parts in the result. */
void ivp_decompose_result_free(ivp_decompose_result_t *result);

/* ── Decomposition ─────────────────────────────────────────────────────── */

/* Decompose a triangle mesh into convex parts.
 *
 * Parameters:
 *   alloc       -- allocator (NULL for default)
 *   vertices    -- flat float array [x0,y0,z0, x1,y1,z1, ...]
 *   n_vertices  -- number of 3D vertices
 *   indices     -- triangle index array [i0,i1,i2, i3,i4,i5, ...]
 *   n_triangles -- number of triangles
 *   options     -- decomposition config (NULL for defaults)
 *   result      -- output: array of convex hull compact surfaces
 *
 * Returns IVP_OK on success. */
ivp_result_t ivp_convex_decompose(
    const ivp_allocator_t *alloc,
    const float *vertices,
    int n_vertices,
    const int *indices,
    int n_triangles,
    const ivp_decompose_options_t *options,
    ivp_decompose_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONVEX_DECOMPOSE_H */
