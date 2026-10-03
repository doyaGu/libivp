/* ivp_build_polygon.h -- Build compact surfaces from explicit triangles
 *
 * Unlike ivp_build_convex_surface() which computes the convex hull of a
 * point cloud, this builder takes pre-triangulated geometry with explicit
 * vertex and index buffers. The input must describe a closed convex
 * polyhedron.
 *
 * Also provides compact surface vertex modification.
 *
 * Ported from ivp_surbuild_polygon_convex.hxx / ivp_compact_modify.hxx.
 */
#ifndef IVP_BUILD_POLYGON_H
#define IVP_BUILD_POLYGON_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Triangle-based convex builder ──────────────────────────────────────── */

/* Build a compact surface from explicit triangles.
 *
 * Parameters:
 *   alloc      -- allocator (NULL for default)
 *   vertices   -- vertex positions (3 floats each)
 *   n_vertices -- number of vertices (>= 4)
 *   indices    -- triangle indices (3 ints per triangle, CCW winding)
 *   n_triangles-- number of triangles (>= 4)
 *
 * Each index must be in [0, n_vertices). The mesh must be a closed,
 * oriented manifold. Returns NULL on degenerate or invalid input.
 *
 * The caller must free with ivp_free_aligned(alloc, result).
 */
ivp_compact_surface_t *ivp_build_polygon_surface(
    const ivp_allocator_t *alloc,
    const float *vertices,      /* [n_vertices][3] */
    int n_vertices,
    const int *indices,         /* [n_triangles][3] */
    int n_triangles);

/* Build a compact surface from convex n-gon faces (reference
 * IVP_Template_Polygon surfaces -> IVP_SurfaceBuilder_Polygon_Convex).
 *
 *   face_indices -- concatenated vertex indices of all faces (CCW seen
 *                   from outside)
 *   face_sizes   -- vertex count of each face (>= 3)
 *
 * The faces are triangulated and the ledge is wrapped by the ledge soup
 * (reference mass properties).  Returns NULL on invalid input. */
ivp_compact_surface_t *ivp_build_polygon_convex_surface(
    const ivp_allocator_t *alloc,
    const float *vertices,
    int n_vertices,
    const int *face_indices,
    const int *face_sizes,
    int n_faces);

/* ── Compact surface vertex modification ────────────────────────────────── */

/* Modify a single vertex position in a compact surface.
 *
 * This directly patches the vertex data in-place. The surface must be
 * a single-ledge surface (not compound).
 *
 * Parameters:
 *   surface       -- the surface to modify (in-place)
 *   vertex_index  -- 0-based vertex index
 *   new_pos       -- new position (3 floats)
 *
 * Returns 0 on success, -1 on failure (invalid index or compound surface).
 *
 * NOTE: Does NOT update mass_center, rotation_inertia, or bounding
 * radius/box_sizes. Call ivp_surface_recompute_bounds() after modifying
 * vertices to update these.
 */
int ivp_surface_modify_vertex(ivp_compact_surface_t *surface,
                                int vertex_index,
                                const float new_pos[3]);

/* Recompute bounding radius, box sizes, mass center, and inertia for
 * a compact surface. Call after modifying vertices. */
void ivp_surface_recompute_bounds(ivp_compact_surface_t *surface);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_POLYGON_H */
