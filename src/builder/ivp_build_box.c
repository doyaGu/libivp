/* ivp_build_box.c — Box compact surface builder
 *
 * The reference creates boxes as a pointsoup of the 8 corners
 * (ivp_reference_runner/ref_runner_common.cxx build_box_compact_surface ->
 * IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface), so
 * libivp does the same: the convex hull builder (qhull 2.6 +
 * IVP_SurfaceBuilder_Polygon_Convex port) yields the reference ledge
 * layout and the ledge soup the reference surface header (mass center,
 * IVP_Rot_Inertia_Solver inertia, radius, deviation).
 *
 * Corner input order:
 *   for (sx = -1,+1) for (sy = -1,+1) for (sz = -1,+1)
 *
 * The compact point order is the reference one (first use in the
 * generated triangles), e.g. for every box
 *   p0=(-,+,+) p1=(+,-,+) p2=(+,+,+) p3=(-,-,+)
 *   p4=(-,+,-) p5=(+,+,-) p6=(-,-,-) p7=(+,-,-)
 */

#include <ivp/builder/ivp_build_surface.h>
#include <ivp/base/ivp_allocator.h>

#define BOX_N_VERTICES 8

ivp_compact_surface_t *ivp_build_box_surface(const ivp_allocator_t *alloc,
                                               float half_x,
                                               float half_y,
                                               float half_z)
{
    if (!alloc) alloc = ivp_default_allocator();

    /* corner coordinates exactly like IVP_U_Point::set((double)sx * hx, ...) */
    double pts[BOX_N_VERTICES * 3];
    int vi = 0;
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                pts[3 * vi + 0] = (double)sx * (double)half_x;
                pts[3 * vi + 1] = (double)sy * (double)half_y;
                pts[3 * vi + 2] = (double)sz * (double)half_z;
                vi++;
            }
        }
    }
    return ivp_build_convex_surface_d(alloc, pts, BOX_N_VERTICES);
}
