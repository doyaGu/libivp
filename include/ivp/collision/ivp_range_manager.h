/* ivp_range_manager.h — IVP_Range_Manager (ivp_physics/ivp_range_manager.cxx)
 *
 * Decides how far ahead the collision detection looks: the radius of the OV
 * tree hull of an object (get_coll_range_in_world) and how the look-ahead
 * distance of a pair of objects is split between their hull managers
 * (get_coll_range_intra_objects).  The environment owns a default manager
 * (ivp_environment_get_range_manager); the parameters can be changed, and
 * the two functions can be overridden (the reference methods are virtual).
 *
 * Parameters are IVP_FLOAT, all arithmetic is double, as in the reference.
 */
#ifndef IVP_RANGE_MANAGER_H
#define IVP_RANGE_MANAGER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_range_manager;

/* optional overrides of the virtual methods (NULL = reference behaviour) */
typedef void (*ivp_range_intra_fn_t)(struct ivp_range_manager *rm, double delta_psi_time,
                                     const ivp_object_t *obj0, const ivp_object_t *obj1,
                                     double *range0, double *range1);
typedef double (*ivp_range_world_fn_t)(struct ivp_range_manager *rm, double delta_psi_time,
                                       const ivp_object_t *obj0);

typedef struct ivp_range_manager {
    /* Object vs object (intra) parameters */
    float look_ahead_time_intra;           /* [s] 0.5                              */
    float look_ahead_max_radius_intra;     /* factor of the smaller radius, 0.9    */
    float look_ahead_min_distance_intra;   /* [m] 0.8                              */
    float look_ahead_max_distance_intra;   /* [m] 10                               */
    float look_ahead_min_seconds_intra;    /* [s] 0.1                              */

    /* Object vs world parameters */
    float look_ahead_time_world;           /* [s] 1                                */
    float look_ahead_max_radius_world;     /* factor of the object radius, 5       */
    float look_ahead_min_distance_world;   /* [m] 0.5                              */
    float look_ahead_max_distance_world;   /* [m] 15                               */
    float look_ahead_min_seconds_world;    /* [s] 0.1                              */

    /* delta PSI time used by the core-based helpers below (the environment
     * passes its current delta PSI time to the object-based functions) */
    ivp_real_t delta_psi_time;

    ivp_range_intra_fn_t get_coll_range_intra_objects;
    ivp_range_world_fn_t get_coll_range_in_world;
    void                *user_data;
} ivp_range_manager_t;

/* IVP_Range_Manager::IVP_Range_Manager defaults (overrides NULL). */
void ivp_range_manager_init(ivp_range_manager_t *rm, ivp_real_t delta_psi_time);

/* Reference IVP_Range_Manager::get_coll_range_intra_objects /
 * get_coll_range_in_world (calls the overrides when set). */
void   ivp_range_manager_coll_range_intra_objects(ivp_range_manager_t *rm, double delta_psi_time,
                                                  const ivp_object_t *obj0, const ivp_object_t *obj1,
                                                  double *range0, double *range1);
double ivp_range_manager_coll_range_in_world(ivp_range_manager_t *rm, double delta_psi_time,
                                             const ivp_object_t *obj0);

/* The reference implementations (for use by overrides). */
void   ivp_range_manager_default_intra_objects(const ivp_range_manager_t *rm, double delta_psi_time,
                                               const ivp_core_t *core0, const ivp_core_t *core1,
                                               double *range0, double *range1);
double ivp_range_manager_default_in_world(const ivp_range_manager_t *rm, double delta_psi_time,
                                          const ivp_core_t *core0);

/* ── Core based helpers (use rm->delta_psi_time) ────────────────────────── */

void ivp_range_manager_get_coll_range_intra(const ivp_range_manager_t *rm,
                                            const ivp_core_t *core0, const ivp_core_t *core1,
                                            ivp_real_t *range0, ivp_real_t *range1);
ivp_real_t ivp_range_manager_get_coll_range_world(const ivp_range_manager_t *rm,
                                                  const ivp_core_t *core0);
/* range0 + range1 of get_coll_range_intra */
ivp_real_t ivp_range_manager_get_pair_margin(const ivp_range_manager_t *rm,
                                             const ivp_core_t *core0, const ivp_core_t *core1);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RANGE_MANAGER_H */
