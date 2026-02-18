/* ivp_matrix_cache.h — Pre-computed transform cache for CCD
 *
 * Caches object transforms at sub-PSI time steps for efficient
 * repeated evaluation during 3D solver root finding.
 *
 * Ported from IVP_Cache_Object.
 */
#ifndef IVP_MATRIX_CACHE_H
#define IVP_MATRIX_CACHE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define IVP_3D_SOLVER_MAX_STEPS  20

/* ── Matrix cache ──────────────────────────────────────────────────────── */

typedef struct ivp_matrix_cache {
    ivp_object_t      *object;
    ivp_time_t         base_time;
    ivp_time_t         dt;                                      /* PSI timestep */
    ivp_transform_r_t  transforms[IVP_3D_SOLVER_MAX_STEPS + 1];
    bool               computed[IVP_3D_SOLVER_MAX_STEPS + 1];
} ivp_matrix_cache_t;

/* Initialize cache for an object over the current PSI interval. */
void ivp_matrix_cache_init(ivp_matrix_cache_t *cache,
                             ivp_object_t *obj,
                             ivp_time_t base_time,
                             ivp_time_t dt);

/* Get transform at sub-step index [0..IVP_3D_SOLVER_MAX_STEPS].
 * Lazily computed and cached. */
const ivp_transform_r_t *ivp_matrix_cache_get(ivp_matrix_cache_t *cache,
                                                 int index);

/* Get transform at arbitrary time t (interpolated). */
const ivp_transform_r_t *ivp_matrix_cache_get_at_time(ivp_matrix_cache_t *cache,
                                                         ivp_time_t t);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MATRIX_CACHE_H */
