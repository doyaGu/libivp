/* ivp_range_manager.c — Speed-based collision range prediction
 *
 * Implements adaptive collision range computation matching the original
 * IVP_Range_Manager. Range is based on object speed, rotation speed,
 * and radius, with configurable lookahead time and distance limits.
 *
 * The algorithm splits the total margin between two objects proportional
 * to their speeds (with cross-contamination factors based on original IVP,
 * but fixed to use independent reads: the original had a data dependency
 * bug where speed1 used the already-modified speed0).
 *
 * Ported from ivp_range_manager.cxx.
 */
#include <ivp/collision/ivp_range_manager.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/math/ivp_fast_math.h>

void ivp_range_manager_init(ivp_range_manager_t *rm, ivp_real_t delta_psi_time)
{
    /* Intra-object defaults (matching original IVP) */
    rm->look_ahead_time_intra         = IVP_REAL(0.5);
    rm->look_ahead_max_radius_intra   = IVP_REAL(0.9);
    rm->look_ahead_min_distance_intra = IVP_REAL(0.8);
    rm->look_ahead_max_distance_intra = IVP_REAL(10.0);
    rm->look_ahead_min_seconds_intra  = IVP_REAL(0.1);

    /* World defaults (matching original IVP) */
    rm->look_ahead_time_world         = IVP_REAL(1.0);
    rm->look_ahead_max_radius_world   = IVP_REAL(5.0);
    rm->look_ahead_min_distance_world = IVP_REAL(0.5);
    rm->look_ahead_max_distance_world = IVP_REAL(15.0);
    rm->look_ahead_min_seconds_world  = IVP_REAL(0.1);

    rm->delta_psi_time = delta_psi_time;
}

void ivp_range_manager_get_coll_range_intra(
    const ivp_range_manager_t *rm,
    const ivp_core_t *core0,
    const ivp_core_t *core1,
    ivp_real_t *range0,
    ivp_real_t *range1)
{
    /* Effective speed = linear + max surface rotation + epsilon */
    ivp_real_t speed0 = core0->current_speed + core0->max_surface_rot_speed + IVP_REAL_EPS;
    ivp_real_t speed1 = core1->current_speed + core1->max_surface_rot_speed + IVP_REAL_EPS;

    ivp_real_t sum_speed = speed0 + speed1;

    /* Use the smaller radius for radius-based clamping */
    ivp_real_t min_rad = core0->upper_limit_radius < core1->upper_limit_radius
                       ? core0->upper_limit_radius
                       : core1->upper_limit_radius;

    /* Base range from speed × lookahead time */
    ivp_real_t max_dist = sum_speed * rm->look_ahead_time_intra;

    /* Clamp to radius-based limit */
    if (min_rad * rm->look_ahead_max_radius_intra < max_dist) {
        max_dist = min_rad * rm->look_ahead_max_radius_intra;
    }

    /* Clamp to min/max distance */
    if (max_dist < rm->look_ahead_min_distance_intra) {
        max_dist = rm->look_ahead_min_distance_intra;
    }
    if (max_dist > rm->look_ahead_max_distance_intra) {
        max_dist = rm->look_ahead_max_distance_intra;
    }

    /* Subtract one PSI step of travel (objects will move this far next step) */
    max_dist -= sum_speed * rm->delta_psi_time;

    /* Ensure minimum time-based range */
    if (max_dist < rm->look_ahead_min_seconds_intra * sum_speed) {
        max_dist = rm->look_ahead_min_seconds_intra * sum_speed;
    }

    /* Cross-contamination factors (original IVP had a data dependency bug
     * where speed1's update used the already-modified speed0; fixed here
     * to use the original values for both updates) */
    ivp_real_t orig_speed0 = speed0;
    speed0 += IVP_REAL(0.2) * speed1;
    speed1 += IVP_REAL(0.18) * orig_speed0;

    ivp_real_t sum = speed0 + speed1;
    ivp_real_t inv_sum = IVP_REAL(1.0) / sum;

    *range0 = max_dist * speed0 * inv_sum;
    *range1 = max_dist * speed1 * inv_sum;
}

ivp_real_t ivp_range_manager_get_coll_range_world(
    const ivp_range_manager_t *rm,
    const ivp_core_t *core0)
{
    ivp_real_t speed0 = core0->current_speed + core0->max_surface_rot_speed + IVP_REAL_EPS;

    ivp_real_t min_rad = core0->upper_limit_radius;

    /* Base range from speed × lookahead time */
    ivp_real_t max_dist = speed0 * rm->look_ahead_time_world;

    /* Clamp to radius-based limit */
    if (min_rad * rm->look_ahead_max_radius_world < max_dist) {
        max_dist = min_rad * rm->look_ahead_max_radius_world;
    }

    /* Clamp to min/max distance */
    if (max_dist < rm->look_ahead_min_distance_world) {
        max_dist = rm->look_ahead_min_distance_world;
    }
    if (max_dist > rm->look_ahead_max_distance_world) {
        max_dist = rm->look_ahead_max_distance_world;
    }

    /* Subtract one PSI step of travel */
    max_dist -= speed0 * rm->delta_psi_time;

    /* Ensure minimum time-based range including object radius */
    if (max_dist < core0->upper_limit_radius + rm->look_ahead_min_seconds_world * speed0) {
        max_dist = rm->look_ahead_min_seconds_world * speed0 + core0->upper_limit_radius;
    }

    return max_dist;
}

ivp_real_t ivp_range_manager_get_pair_margin(
    const ivp_range_manager_t *rm,
    const ivp_core_t *core0,
    const ivp_core_t *core1)
{
    ivp_real_t r0, r1;
    ivp_range_manager_get_coll_range_intra(rm, core0, core1, &r0, &r1);
    return r0 + r1;
}
