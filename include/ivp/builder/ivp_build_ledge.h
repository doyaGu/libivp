/* ivp_build_ledge.h -- Compact ledge generation (convex polyhedra)
 *
 * Builds a single convex IVP_Compact_Ledge from an oriented polygon soup
 * through the C port of the reference pipeline
 *   IVP_Template_Polygon (planes_to_template) -> IVP_SurfaceBuilder_Polygon_Convex
 *   -> IVP_Object_Polygon_Tetra::make_triangles (P_Sur_2D face triangulation),
 *      calc_hesse, insert_pierce_info
 *   -> IVP_Compact_Ledge_Generator
 * (src/builder/ivp_build_polygon_convex.c), the same code the convex hull
 * builder feeds with the qhull facets.  Triangle order, triangle start
 * edges, point order (first use) and pierce indices are the reference ones.
 *
 * Every returned ledge is 16-byte aligned, terminal, compact, has its
 * pierce indices filled and must be freed with ivp_compact_ledge_free().
 */
#ifndef IVP_BUILD_LEDGE_H
#define IVP_BUILD_LEDGE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_ledge.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reference IVP_MAX_TRIANGLES_PER_LEDGE (ivp_compact_ledge.hxx:40). */
#define IVP_BUILD_MAX_TRIANGLES_PER_LEDGE IVP_MAX_TRIANGLES_PER_LEDGE

/* Build a convex ledge from polygons.
 *
 *   points       -- [n_points][3] vertex coordinates
 *   face_indices -- concatenated vertex indices of every face, each face
 *                   counter-clockwise when seen from outside
 *   face_sizes   -- number of vertices of each face (>= 3)
 *   n_faces      -- number of faces
 *
 * Each face becomes one template surface (normal: outward Newell normal);
 * faces are walked clockwise around it like qh_facet3vertex lists qhull
 * facets.  The polygon soup must form a closed, consistently oriented
 * 2-manifold.  Returns NULL on invalid input (open mesh, a face that cannot
 * be triangulated, too many triangles, ...). */
ivp_compact_ledge_t *ivp_build_ledge_from_polygons(const ivp_allocator_t *alloc,
                                                   const double *points,
                                                   int n_points,
                                                   const int *face_indices,
                                                   const int *face_sizes,
                                                   int n_faces);

/* Same with pre-triangulated faces (indices [n_triangles][3], CCW): every
 * triangle is a template surface of its own. */
ivp_compact_ledge_t *ivp_build_ledge_from_triangles(const ivp_allocator_t *alloc,
                                                    const double *points,
                                                    int n_points,
                                                    const int *indices,
                                                    int n_triangles);

/* Two-sided single triangle ledge (2 compact triangles), like
 * IVP_SurfaceBuilder_Pointsoup::convert_triangle_to_compace_ledge.
 * Returns NULL if the triangle area is degenerate (|n|^2 < P_DOUBLE_RES). */
ivp_compact_ledge_t *ivp_build_triangle_ledge(const ivp_allocator_t *alloc,
                                              const double p0[3],
                                              const double p1[3],
                                              const double p2[3]);

/* Fill the pierce indices of every triangle (insert_pierce_info). */
void ivp_compact_ledge_compute_pierce_info(ivp_compact_ledge_t *ledge);

/* Copy a compact ledge (point array included) into a new aligned block. */
ivp_compact_ledge_t *ivp_compact_ledge_clone(const ivp_allocator_t *alloc,
                                             const ivp_compact_ledge_t *ledge);

/* Free a ledge returned by the functions above. */
void ivp_compact_ledge_free(const ivp_allocator_t *alloc, ivp_compact_ledge_t *ledge);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_LEDGE_H */
