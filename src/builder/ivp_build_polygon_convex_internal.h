/* ivp_build_polygon_convex_internal.h -- reference convex polygon pipeline
 *
 * Faithful C port of the reference path that turns convex polygon faces
 * into a compact ledge:
 *
 *   IVP_SurfaceBuilder_Pointsoup::planes_to_template  (ivp_surbuild_pointsoup.cxx)
 *     -> IVP_Template_Polygon                         (ivp_templates_intern.hxx)
 *   IVP_SurfaceBuilder_Polygon_Convex::convert_template_to_ledge
 *                                                      (ivp_surbuild_polygon_convex.cxx)
 *     -> IVP_Object_Polygon_Tetra::make_triangles     (ivp_object_polygon_tetra.cxx,
 *        P_Sur_2D ear cutting of every face            ivp_triangle_gen.hxx)
 *     -> IVP_Triangle::calc_hesse, insert_pierce_info
 *     -> IVP_Compact_Ledge_Generator                  (ivp_compact_ledge_gen.cxx)
 *
 * and the two sided single triangle ledge of
 * IVP_SurfaceBuilder_Polygon_Convex(IVP_Template_Ledge_Polygon_Soup *)
 * (IVP_Object_Polygon_Tetra::generate_double_triangle).
 *
 * The triangle order, the start edge of every triangle, the point order
 * (first use) and the opposite / pierce indices are those of the
 * reference, so the ledge layout is bit identical for identical input.
 * All arithmetic uses IVP_DOUBLE (double) with the reference evaluation
 * order; compact points are IVP_FLOAT.
 */
#ifndef IVP_BUILD_POLYGON_CONVEX_INTERNAL_H
#define IVP_BUILD_POLYGON_CONVEX_INTERNAL_H

#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_ledge.h>

#include <stdbool.h>
#include <stdint.h>

/* IVP_Template_Line */
typedef struct ivp_pc_templ_line {
    uint16_t p[2]; /* end point indices */
} ivp_pc_templ_line_t;

/* IVP_Template_Surface.  Walking from point lines[i].p[revert_line[i]] to
 * lines[i].p[1 - revert_line[i]] the surface lies on the left (as seen
 * along `normal`, see P_Sur_2D::calc_line_representation). */
typedef struct ivp_pc_templ_surface {
    double normal[3];
    int n_lines;
    uint16_t *lines;   /* indices into ivp_pc_templ_polygon_t::lines */
    char *revert_line; /* direction flags */
} ivp_pc_templ_surface_t;

/* IVP_Template_Polygon */
typedef struct ivp_pc_templ_polygon {
    int n_points;
    double *points; /* [n_points][3] (IVP_Template_Point: IVP_U_Point) */
    int n_lines;
    ivp_pc_templ_line_t *lines;
    int n_surfaces;
    ivp_pc_templ_surface_t *surfaces;

    /* storage bookkeeping */
    const ivp_allocator_t *alloc;
    int point_capacity;
    int surface_line_capacity;
    uint16_t *surface_line_storage;
    char *surface_revert_storage;
} ivp_pc_templ_polygon_t;

/* One convex face (IVP_SurMan_PS_Plane): plane normal and the indices of
 * its points in the point array passed to ivp_pc_planes_to_template. */
typedef struct ivp_pc_plane {
    double normal[3];
    int n_points;
    const int *point_idx;
} ivp_pc_plane_t;

/* IVP_SurMan_PS_Plane::get_area_size: sum over the polygon edges of
 * ((b - a) x (a - p0)) . normal.  The pointsoup flips a plane normal when
 * this is negative (plane->mult(-1)), which is the orientation
 * convention IVP_SurfaceBuilder_Polygon_Convex expects for the template
 * surfaces. */
double ivp_pc_plane_area_size(const double *points, const int *point_idx, int n_points,
                              const double normal[3]);

/* IVP_SurfaceBuilder_Pointsoup::planes_to_template.
 * points: [n_points][3]; every point is copied (exactly equal points are
 * merged, like get_offset_from_pointlist).  Returns false on failure
 * (allocation, or a plane index that is out of range); *templ is then
 * empty and needs no destroy. */
bool ivp_pc_planes_to_template(ivp_pc_templ_polygon_t *templ,
                               const ivp_allocator_t *alloc,
                               const double *points, int n_points,
                               const ivp_pc_plane_t *planes, int n_planes);

/* Release a template filled by ivp_pc_planes_to_template. */
void ivp_pc_template_destroy(ivp_pc_templ_polygon_t *templ);

/* IVP_SurfaceBuilder_Polygon_Convex::convert_template_to_ledge.
 * Returns a 16-byte aligned ledge (free with ivp_compact_ledge_free) or
 * NULL when the template cannot be triangulated / linked. */
ivp_compact_ledge_t *ivp_pc_template_to_ledge(const ivp_allocator_t *alloc,
                                              const ivp_pc_templ_polygon_t *templ);

/* Two sided triangle ledge (generate_double_triangle + compact ledge
 * generator), points 0,1,2 = p0,p1,p2.  No area check. */
ivp_compact_ledge_t *ivp_pc_double_triangle_ledge(const ivp_allocator_t *alloc,
                                                  const double p0[3],
                                                  const double p1[3],
                                                  const double p2[3]);

#endif /* IVP_BUILD_POLYGON_CONVEX_INTERNAL_H */
