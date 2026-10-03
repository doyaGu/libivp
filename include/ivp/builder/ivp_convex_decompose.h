/* ivp_convex_decompose.h -- Convex decomposition of concave polyhedra
 *
 * Port of IVP_Convex_Decompositor (ivp_convex_decompositor.cxx), which runs
 * Barry Joe's GEOMPACK cvdec3 algorithm: the polyhedron is cut along planes
 * through its reflex edges (resedg / cutfac / insfac) until every edge
 * angle is <= pi, retrying with smaller acceptable dihedral angles
 * (angacc -= 5 degrees, rdacc *= 0.95, at most 36 attempts) like
 * IVP_Geompack::decompose.  Each resulting convex part is returned as the
 * point soup of its vertices; the convex hull of each part is built with
 * the pointsoup builder.
 *
 * Reference failure semantics are kept: if GEOMPACK aborts (non manifold
 * input, ...) the parts produced so far are returned (initially the whole
 * polyhedron, i.e. its convex hull).
 */
#ifndef IVP_CONVEX_DECOMPOSE_H
#define IVP_CONVEX_DECOMPOSE_H

#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Decomposition parameters (IVP_Convex_Decompositor_Parameters) ─────── */

typedef struct ivp_decompose_options {
    float tolin;   /* relative tolerance (GEOMPACK TOLIN), default 1e-9  */
    float angacc;  /* min acceptable dihedral angle of cut faces, degrees,
                      default 30 (reference callers supply their own) */
    float rdacc;   /* min acceptable relative distance between a cut plane
                      and vertices not on it, default 0.3 */
} ivp_decompose_options_t;

static inline ivp_decompose_options_t ivp_decompose_options_default(void) {
    ivp_decompose_options_t o;
    o.tolin = 1e-9f;
    o.angacc = 30.0f;
    o.rdacc = 0.3f;
    return o;
}

/* ── Low level: point soups of the convex subparts ──────────────────────── */

/* IVP_Convex_Decompositor::perform_convex_decomposition_on_concave_polyhedron.
 *
 *   points       -- [n_points][3]
 *   face_indices -- concatenated 0-based vertex indices of every face
 *                   (CCW seen from outside, faces must be simple)
 *   face_sizes   -- vertex count of every face
 *
 * On success *out_part_points receives one allocation holding the points of
 * all parts ([total][3] doubles, part i starts at out_part_offsets[i] and has
 * out_part_offsets[i+1]-out_part_offsets[i] points).  Free both arrays with
 * ivp_decompose_points_free().  Returns the number of parts (>= 0) or -1 on
 * invalid input / allocation failure. */
int ivp_convex_decompose_points(const ivp_allocator_t *alloc,
                                const double *points, int n_points,
                                const int *face_indices, const int *face_sizes, int n_faces,
                                const ivp_decompose_options_t *options,
                                double **out_part_points, int **out_part_offsets);

void ivp_decompose_points_free(const ivp_allocator_t *alloc, int n_parts,
                               double *part_points, int *part_offsets);

/* ── Decomposition result (convex part surfaces) ────────────────────────── */

typedef struct ivp_decompose_result {
    ivp_compact_surface_t **parts;  /* array of convex hull surfaces */
    int                     n_parts;
    const ivp_allocator_t  *allocator;
} ivp_decompose_result_t;

/* Free all parts in the result. */
void ivp_decompose_result_free(ivp_decompose_result_t *result);

/* Decompose a closed triangle mesh into convex parts and build the convex
 * hull surface (pointsoup) of every part.  Parts whose hull cannot be
 * built are skipped (reference convert_concave_polyhedron_to_compact_ledges).
 * Returns IVP_OK on success (n_parts >= 1). */
ivp_result_t ivp_convex_decompose(
    const ivp_allocator_t *alloc,
    const float *vertices,
    int n_vertices,
    const int *indices,
    int n_triangles,
    const ivp_decompose_options_t *options,
    ivp_decompose_result_t *result);

/* Same as above but returns the convex parts as compact ledges
 * (IVP_SurfaceBuilder_Polyhedron_Concave::convert_concave_polyhedron_to_
 * compact_ledges).  *out_ledges is an allocated array of n ledge pointers;
 * free every ledge with ivp_compact_ledge_free() and the array with
 * ivp_free(alloc, ptr, n * sizeof(ptr)).  Returns n (>= 0) or -1. */
int ivp_convex_decompose_to_ledges(const ivp_allocator_t *alloc,
                                   const double *points, int n_points,
                                   const int *face_indices, const int *face_sizes, int n_faces,
                                   const ivp_decompose_options_t *options,
                                   ivp_compact_ledge_t ***out_ledges);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONVEX_DECOMPOSE_H */
