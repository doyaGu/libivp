/* ivp_collision_tolerance.c — Collision distance threshold computation
 *
 * Replicates original IVP's set_collision_tolerance(t) formula exactly
 * (from ivp_mindist.cxx lines 52-88).
 */
#include <ivp/collision/ivp_collision_tolerance.h>
#include <ivp/base/ivp_assert.h>

void ivp_collision_settings_init(ivp_collision_settings_t *settings,
                                  ivp_real_t tolerance)
{
    IVP_ASSERT(settings != NULL);
    IVP_ASSERT(tolerance > IVP_REAL(0.0));

    ivp_real_t t = tolerance;
    settings->tolerance = t;

    /* Core distance layers (from set_collision_tolerance formula) */
    settings->real_coll_dist         = IVP_REAL(0.1) * t;
    settings->min_coll_dist          = settings->real_coll_dist + IVP_REAL(0.9) * t;
    settings->minimum_friction_dist  = settings->min_coll_dist + IVP_REAL(0.0) * t;
    settings->friction_dist          = settings->minimum_friction_dist + IVP_REAL(1.0) * t;
    settings->keeper_dist            = settings->friction_dist + IVP_REAL(0.3) * t;
    settings->distance_keepers_safety = IVP_REAL(0.01) * t;
    settings->max_distance_for_friction = settings->friction_dist + IVP_REAL(2.5) * t;
    settings->max_distance_for_impact   = settings->friction_dist + IVP_REAL(20.0) * t;

    /* Speed thresholds */
    settings->min_vertical_speed_at_collision = IVP_REAL(2.0) * t;

    /* speed_after_keeper_dist = sqrt(2 * (keeper_dist - min_coll_dist) * gravity)
     * Original uses 9.81 m/s^2 as gravity constant for this computation. */
    ivp_real_t delta = settings->keeper_dist - settings->min_coll_dist;
    settings->speed_after_keeper_dist = ivp_sqrt(
        IVP_REAL(2.0) * delta * IVP_DEFAULT_GRAVITY_FOR_KEEPER_SPEED);

    settings->mindist_change_force_dist = settings->min_coll_dist * IVP_REAL(0.1);

    /* Default gravity magnitude for friction estimates */
    settings->gravity_magnitude = IVP_DEFAULT_GRAVITY_FOR_KEEPER_SPEED;

    /* Interpolated collision distance array.
     * Linear interpolation from min_coll_dist to minimum_friction_dist
     * over IVP_MAX_STEPS_FOR_COLLDIST_DECREASE steps. */
    ivp_real_t range = settings->minimum_friction_dist - settings->min_coll_dist;
    for (int i = 0; i < IVP_MAX_STEPS_FOR_COLLDIST_DECREASE; i++) {
        ivp_real_t frac = (ivp_real_t)i /
                          (ivp_real_t)(IVP_MAX_STEPS_FOR_COLLDIST_DECREASE - 1);
        settings->coll_dists[i] = settings->min_coll_dist + range * frac;
    }
}

void ivp_collision_settings_init_default(ivp_collision_settings_t *settings)
{
    ivp_collision_settings_init(settings, IVP_DEFAULT_COLLISION_TOLERANCE);
}
