/* ivp_build_concave.h — Concave polyhedron surface builder
 *
 * High-level convenience that converts a concave triangle mesh into a single
 * compound compact surface:
 *   1. Runs convex decomposition (flood-fill by normal similarity).
 *   2. Builds a convex hull for each part.
 *   3. Compiles all parts into a single ledge soup with BVH.
 *
 * This is the C17 equivalent of the legacy
 * IVP_SurfaceBuilder_Polyhedron_Concave::convert_concave_polyhedron_to_single_compact_surface().
 *
 * Ported from ivp_surbuild_polyhdrn_cncv.hxx.
 */
#ifndef IVP_BUILD_CONCAVE_H
#define IVP_BUILD_CONCAVE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/builder/ivp_convex_decompose.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Concave build options ─────────────────────────────────────────────── */

typedef struct ivp_concave_options {
    ivp_decompose_options_t decompose;  /* convex decomposition settings */
    bool  force_single_hull;            /* skip decomposition, build one hull */
    bool  build_root_convex_hull;       /* also build outer bounding convex hull */
} ivp_concave_options_t;

static inline ivp_concave_options_t ivp_concave_options_default(void) {
    ivp_concave_options_t o;
    o.decompose = ivp_decompose_options_default();
    o.force_single_hull = false;
    o.build_root_convex_hull = false;
    return o;
}

/* ── Build functions ───────────────────────────────────────────────────── */

/* Build a compound surface from a concave triangle mesh.
 *
 * Parameters:
 *   alloc       -- allocator (NULL for default)
 *   vertices    -- flat float array [x0,y0,z0, x1,y1,z1, ...]
 *   n_vertices  -- number of 3D vertices
 *   indices     -- triangle index array [i0,i1,i2, i3,i4,i5, ...]
 *   n_triangles -- number of triangles
 *   options     -- build config (NULL for defaults)
 *
 * Returns a newly allocated compact surface, or NULL on failure.
 * The caller must free with ivp_compact_surface_free(). */
ivp_compact_surface_t *ivp_build_concave_surface(
    const ivp_allocator_t *alloc,
    const float *vertices,
    int n_vertices,
    const int *indices,
    int n_triangles,
    const ivp_concave_options_t *options);

/* Build a compound surface from pre-computed convex parts.
 *
 * Each part is a convex compact surface (e.g. from ivp_build_convex_surface).
 * This function extracts the single ledge from each part and compiles them
 * into a single ledge soup with BVH.
 *
 * Input part surfaces are NOT freed by this function. */
ivp_compact_surface_t *ivp_build_concave_from_parts(
    const ivp_allocator_t *alloc,
    ivp_compact_surface_t **parts,
    int n_parts);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_CONCAVE_H */
