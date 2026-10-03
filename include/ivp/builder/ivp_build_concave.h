/* ivp_build_concave.h — Concave polyhedron surface builder
 *
 * Converts a concave polyhedron into a single compound compact surface like
 * IVP_SurfaceBuilder_Polyhedron_Concave::convert_concave_polyhedron_to_
 * single_compact_surface (ivp_surbuild_polyhdrn_cncv.cxx:83):
 *   1. GEOMPACK convex decomposition (ivp_convex_decompose.h);
 *   2. a pointsoup convex ledge per part;
 *   3. ledge soup with build_root_convex_hull = true and
 *      merge_points = IVP_SLMP_MERGE_AND_REALLOCATE (reference defaults).
 *
 * Note: with the root hull, ivp_surman_get_single_convex() returns the
 * (non-terminal) hull ledge, as in the reference.
 */
#ifndef IVP_BUILD_CONCAVE_H
#define IVP_BUILD_CONCAVE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/builder/ivp_convex_decompose.h>
#include <ivp/builder/ivp_build_ledge_soup.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Concave build options ─────────────────────────────────────────────── */

typedef struct ivp_concave_options {
    ivp_decompose_options_t decompose;   /* convex decomposition settings */
    bool  force_single_hull;             /* skip decomposition, build one hull */
    bool  build_root_convex_hull;        /* outer hull at the root node (default true) */
    ivp_ledge_soup_merge_points_t merge_points; /* default MERGE_AND_REALLOCATE */
} ivp_concave_options_t;

static inline ivp_concave_options_t ivp_concave_options_default(void) {
    ivp_concave_options_t o;
    o.decompose = ivp_decompose_options_default();
    o.force_single_hull = false;
    o.build_root_convex_hull = true;
    o.merge_points = IVP_SLMP_MERGE_AND_REALLOCATE;
    return o;
}

/* ── Build functions ───────────────────────────────────────────────────── */

/* Build a compound surface from a concave triangle mesh.
 *
 * Parameters:
 *   alloc       -- allocator (NULL for default)
 *   vertices    -- flat float array [x0,y0,z0, x1,y1,z1, ...]
 *   n_vertices  -- number of 3D vertices
 *   indices     -- triangle index array [i0,i1,i2, i3,i4,i5, ...], every
 *                  triangle CCW seen from outside (closed 2-manifold)
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

/* Reference path for polygon input: points [n_points][3], faces given by
 * concatenated 0-based indices (CCW seen from outside) and per face sizes. */
ivp_compact_surface_t *ivp_build_concave_polyhedron_surface(
    const ivp_allocator_t *alloc,
    const double *points,
    int n_points,
    const int *face_indices,
    const int *face_sizes,
    int n_faces,
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
