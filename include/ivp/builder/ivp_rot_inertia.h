/* ivp_rot_inertia.h -- Mass center and rotational inertia solver
 *
 * Computes mass center and diagonal rotational inertia tensor from compact
 * ledge geometry by volume integration over triangle surfaces.
 *
 * Ported from ivp_rot_inertia_solver.hxx / ivp_rot_inertia_solver.cxx.
 */
#ifndef IVP_ROT_INERTIA_H
#define IVP_ROT_INERTIA_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/geometry/ivp_compact_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Compute mass center and rotational inertia from a compact surface.
 *
 * This integrates over all triangle surfaces of all ledges to find the
 * volumetric center of mass and the diagonal inertia tensor (for unit mass).
 *
 * If the geometry is degenerate (zero or near-zero volume), falls back to
 * bounding-box-based estimates and returns IVP_ERR_DEGENERATE_GEOMETRY.
 *
 * Parameters:
 *   surface           -- input compact surface
 *   mass_center_out   -- output: center of mass (3 floats)
 *   rot_inertia_out   -- output: diagonal inertia tensor (3 floats)
 *
 * Returns:
 *   IVP_OK on success, IVP_ERR_DEGENERATE_GEOMETRY if fallback was used.
 */
ivp_result_t ivp_calc_mass_center_and_rot_inertia(
    const ivp_compact_surface_t *surface,
    float mass_center_out[3],
    float rot_inertia_out[3]);

/* Compute bounding box of a compact ledge in object coordinates.
 *
 * Parameters:
 *   ledge      -- input compact ledge
 *   min_out    -- output: minimum extents (3 floats)
 *   max_out    -- output: maximum extents (3 floats)
 */
void ivp_ledge_calc_bounding_box(const ivp_compact_ledge_t *ledge,
                                  float min_out[3],
                                  float max_out[3]);

/* Compute geometric center and bounding radius of a compact ledge.
 *
 * Parameters:
 *   ledge       -- input compact ledge
 *   center_out  -- output: geometric center (3 floats)
 *   radius_out  -- output: bounding radius
 */
void ivp_ledge_calc_geom_center_and_radius(const ivp_compact_ledge_t *ledge,
                                            float center_out[3],
                                            float *radius_out);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ROT_INERTIA_H */
