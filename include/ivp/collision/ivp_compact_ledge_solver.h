/* ivp_compact_ledge_solver.h -- Compact ledge solver utilities
 *
 * Higher-level collision functions operating on compact ledges:
 *   - Triangle hesse plane computation (normalized and unnormalized)
 *   - Bounding sphere computation per ledge
 *   - Radius and deviation to given center
 *   - Ledge collection from surfaces
 *   - Point-face and point-ledge distance queries
 *
 * These are the C17 equivalents of the original IVP_Compact_Ledge_Solver
 * (IVP_CLS) singleton functions.
 *
 * Ported from ivp_compact_ledge_solver.hxx / ivp_compact_ledge_solver.cxx.
 */
#ifndef IVP_COMPACT_LEDGE_SOLVER_H
#define IVP_COMPACT_LEDGE_SOLVER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/collision/ivp_narrowphase.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Point-face distance with full result ──────────────────────────────── */

/* Find the closest feature on a ledge to a given point.
 * Tests all triangles and returns the squared distance to the closest one.
 *
 * Parameters:
 *   ledge     -- compact ledge to test against
 *   point_os  -- query point in object space
 *
 * Returns:
 *   Squared distance from point to nearest feature on ledge.
 */
ivp_real_t ivp_cls_qdist_point_to_ledge(const ivp_compact_ledge_t *ledge,
                                         const ivp_vec3r_t *point_os);

/* ── Radius computation ────────────────────────────────────────────────── */

/* Compute the bounding radius and maximum surface deviation of a ledge
 * relative to a given center point.
 *
 * Parameters:
 *   ledge          -- compact ledge
 *   center         -- center point (object space)
 *   radius_inout   -- on input: current max radius; on output: updated
 *   deviation_out  -- on input: current max deviation; on output: updated
 *
 * Both radius_inout and deviation_out are max-accumulated (only increase).
 */
void ivp_cls_calc_radius_to_center(const ivp_compact_ledge_t *ledge,
                                    const ivp_vec3r_t *center,
                                    ivp_real_t *radius_inout,
                                    ivp_real_t *deviation_out);

/* Compute bounding radius and deviation for a full surface. */
void ivp_cls_calc_radius_for_surface(const ivp_compact_surface_t *surface,
                                      const ivp_vec3r_t *center,
                                      ivp_real_t *radius_out,
                                      ivp_real_t *deviation_out);

/* ── Hesse plane (normalized) ──────────────────────────────────────────── */

/* Compute the normalized Hesse plane of a triangle in object space.
 *
 * Parameters:
 *   ledge      -- owning ledge
 *   tri_edge   -- any edge of the triangle
 *   normal_out -- output: unit normal vector
 *   dist_out   -- output: signed distance from origin to plane
 */
void ivp_cls_calc_hesse_normalized(const ivp_compact_ledge_t *ledge,
                                    const ivp_compact_edge_t *tri_edge,
                                    ivp_vec3r_t *normal_out,
                                    ivp_real_t *dist_out);

#ifdef IVP_DEBUG_BUILD
/* IVP_Compact_Ledge_Solver::check_ledge ("#ifdef DEBUG", "for debugging",
 * ivp_compact_ledge_solver.cxx:788): consistency check of one compact ledge
 * that reports every violation through IVP_ASSERT (16 byte alignment of the
 * ledge, its point array and its triangles, tri_index sequence, opposite
 * symmetry, opposite start points, convexity within -10000 * P_FLOAT_RES,
 * edge round trips) and returns true.  The reference debug build calls it
 * from the grid / pointsoup builders.  The point-array alignment check is
 * IVP_ASSERT_NONFATAL: grid ledges have 4-byte aligned points and fail it in
 * the reference too.  (ivp_compact_ledge_check in
 * ivp/geometry/ivp_compact_ledge_geom.h is the silent, every-build
 * validator.) */
bool ivp_cls_check_ledge(const ivp_compact_ledge_t *c_ledge);
#endif

#ifdef __cplusplus
}
#endif

#endif /* IVP_COMPACT_LEDGE_SOLVER_H */
