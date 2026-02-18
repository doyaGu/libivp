/* ivp_ccd_narrowphase.h — Continuous collision detection narrowphase
 *
 * Provides distance functions for the CCD 3D solver, sweep-pair logic,
 * and integration helpers for the environment collision resolution.
 */
#ifndef IVP_CCD_NARROWPHASE_H
#define IVP_CCD_NARROWPHASE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/collision/ivp_3d_solver.h>
#include <ivp/collision/ivp_matrix_cache.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ──────────────────────────────────────────────────────────── */

#define IVP_MAX_CCD_PAIRS  64

/* ── CCD convex pair context ────────────────────────────────────────────── */

/* Context struct for the CCD distance callback.
 * Holds geometry info needed by the distance function. */
typedef struct ivp_ccd_convex_pair {
    ivp_object_t *obj_a;
    ivp_object_t *obj_b;
} ivp_ccd_convex_pair_t;

/* ── CCD hit result ─────────────────────────────────────────────────────── */

typedef struct ivp_ccd_hit {
    ivp_contact_point_t contact;
    ivp_time_t          toi;            /* time of impact within PSI */
    bool                valid;
} ivp_ccd_hit_t;

/* ── Distance function for 3D solver ───────────────────────────────────── */

/* Distance function matching ivp_3d_distance_fn signature.
 * Dispatches based on object types (sphere-sphere, sphere-poly, poly-poly).
 * user_data should point to ivp_ccd_convex_pair_t. */
ivp_real_t ivp_3d_dist_convex_convex(const ivp_transform_r_t *m_a,
                                       const ivp_transform_r_t *m_b,
                                       void *user_data);

/* ── Sweep pair query ───────────────────────────────────────────────────── */

/* Returns true if this pair needs a CCD sweep: at least one object has
 * ccd_enabled, and its speed exceeds the threshold. */
bool ivp_ccd_pair_needs_sweep(const ivp_object_t *obj_a,
                               const ivp_object_t *obj_b,
                               ivp_real_t dt);

/* Perform CCD sweep for a pair. Returns true if TOI hit found.
 * base_time: start of current PSI. dt: PSI timestep. */
bool ivp_ccd_sweep_pair(ivp_object_t *obj_a,
                          ivp_object_t *obj_b,
                          ivp_time_t base_time,
                          ivp_real_t dt,
                          ivp_ccd_hit_t *hit_out);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CCD_NARROWPHASE_H */
