/* ivp_build_surface.h — Basic surface builder API
 *
 *   ivp_build_box_surface()      — axis aligned box
 *   ivp_build_convex_surface()   — convex hull from points (QHull,
 *                                  IVP_SurfaceBuilder_Pointsoup)
 *
 * Every surface is produced the reference way: a convex compact ledge is
 * wrapped by the ledge soup builder, which computes the surface header
 * (IVP_SurfaceBuilder_Ledge_Soup::insert_radius_in_compact_surface):
 *   mass_center         — volume centroid (IVP_Rot_Inertia_Solver)
 *   rotation_inertia    — unit mass inertia of the reference solver.  NOTE:
 *                         the reference converts the 2D second moments
 *                         a,b,c to I_x = sqrt(b^2 + c^2) (cyclic), so a cube
 *                         of half size h gets 0.4714 h^2, not 2/3 h^2.
 *   upper_limit_radius  — max vertex distance from mass_center
 *   max deviation       — int(1 + dev / (radius / 250)), dev = max in-plane
 *                         distance |p x n| of a vertex from mass_center
 *
 * All returned surfaces must be freed with ivp_compact_surface_free() /
 * ivp_free_aligned().  Returned ledges must be freed with
 * ivp_compact_ledge_free().
 *
 * The convex hull builders (and everything built on them: boxes, point
 * soups, half space soups, convex decomposition pieces, ledge tree hulls)
 * use the selected external Qhull dependency.  The default Qhull 2020.2
 * backend gives each call a separate qhT, so independent builds may run
 * concurrently.  The reference-development backend accepts concurrent
 * callers but serializes their classic process-global Qhull work.  Do not
 * share an allocator between threads unless that allocator is thread-safe.
 */
#ifndef IVP_BUILD_SURFACE_H
#define IVP_BUILD_SURFACE_H

#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/math/ivp_vec3.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Build a compact surface for an axis-aligned box centered at the origin
 * (8 vertices, 12 triangles): the convex hull of the 8 corners given in
 * the order sx, sy, sz = -1, +1 (the reference runner's
 * build_box_compact_surface), so ledge layout and surface header are the
 * reference ones. */
ivp_compact_surface_t *ivp_build_box_surface(const ivp_allocator_t *alloc,
                                               float half_x,
                                               float half_y,
                                               float half_z);

/* Build a compact convex surface from an input point cloud
 * (IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface).
 * n_points must be >= 3; exactly 3 points give a two sided triangle.
 * Points are used as given (never re-centered). */
ivp_compact_surface_t *ivp_build_convex_surface(const ivp_allocator_t *alloc,
                                                 const ivp_vec3f_t *points,
                                                 int n_points);

/* Same with double input (coords = [x0,y0,z0, x1,...]). */
ivp_compact_surface_t *ivp_build_convex_surface_d(const ivp_allocator_t *alloc,
                                                  const double *coords,
                                                  int n_points);

/* Convex hull ledge only (convert_pointsoup_to_compact_ledge). */
ivp_compact_ledge_t *ivp_build_convex_ledge(const ivp_allocator_t *alloc,
                                            const ivp_vec3f_t *points,
                                            int n_points);
ivp_compact_ledge_t *ivp_build_convex_ledge_d(const ivp_allocator_t *alloc,
                                              const double *coords,
                                              int n_points);

/* Wrap one convex ledge into a compact surface through the ledge soup
 * (the ledge is copied, not freed). */
ivp_compact_surface_t *ivp_build_surface_from_ledge(const ivp_allocator_t *alloc,
                                                    const ivp_compact_ledge_t *ledge);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_SURFACE_H */
