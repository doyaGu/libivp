/* ivp_range_manager.c — IVP_Range_Manager (ivp_physics/ivp_range_manager.cxx) */

#include <ivp/collision/ivp_range_manager.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>

#define RM_P_DOUBLE_EPS 1e-19 /* P_DOUBLE_EPS */

void ivp_range_manager_init(ivp_range_manager_t *rm, ivp_real_t delta_psi_time)
{
    rm->look_ahead_time_world = 1.0f;
    rm->look_ahead_max_radius_world = 5.0f;
    rm->look_ahead_min_distance_world = 0.5f;
    rm->look_ahead_max_distance_world = 15.0f;
    rm->look_ahead_min_seconds_world = 0.1f;

    rm->look_ahead_time_intra = 0.5f;
    rm->look_ahead_max_radius_intra = 0.9f;
    rm->look_ahead_min_distance_intra = 0.8f;
    rm->look_ahead_max_distance_intra = 10.0f;
    rm->look_ahead_min_seconds_intra = 0.1f;

    rm->delta_psi_time = delta_psi_time;
    rm->get_coll_range_intra_objects = NULL;
    rm->get_coll_range_in_world = NULL;
    rm->user_data = NULL;
}

void ivp_range_manager_default_intra_objects(const ivp_range_manager_t *rm, double delta_psi_time,
                                             const ivp_core_t *core0, const ivp_core_t *core1,
                                             double *range0, double *range1)
{
    /* split max_dist to both hull_managers */
    double speed0 = (double)core0->current_speed + (double)core0->max_surface_rot_speed + RM_P_DOUBLE_EPS;
    double speed1 = (double)core1->current_speed + (double)core1->max_surface_rot_speed + RM_P_DOUBLE_EPS;
    double sum_speed = speed0 + speed1;
    double r0 = (double)(float)core0->upper_limit_radius, r1 = (double)(float)core1->upper_limit_radius;
    double min_rad = (r0 < r1) ? r0 : r1;
    double max_dist = sum_speed * (double)rm->look_ahead_time_intra;
    if (min_rad * (double)rm->look_ahead_max_radius_intra < max_dist) max_dist = min_rad * (double)rm->look_ahead_max_radius_intra;
    if (max_dist < (double)rm->look_ahead_min_distance_intra) max_dist = (double)rm->look_ahead_min_distance_intra;
    if (max_dist > (double)rm->look_ahead_max_distance_intra) max_dist = (double)rm->look_ahead_max_distance_intra;
    max_dist -= sum_speed * delta_psi_time;
    if (max_dist < (double)rm->look_ahead_min_seconds_intra * sum_speed) {
        max_dist = (double)rm->look_ahead_min_seconds_intra * sum_speed;
    }
    /* retail: speed1 takes the already increased speed0 */
    speed0 += (double)0.2f * speed1;
    speed1 += (double)0.18f * speed0;
    double sum = speed0 + speed1;
    double inv_sum = (double)1.0f / sum;
    *range0 = max_dist * speed0 * inv_sum;
    *range1 = max_dist * speed1 * inv_sum;
}

double ivp_range_manager_default_in_world(const ivp_range_manager_t *rm, double delta_psi_time,
                                          const ivp_core_t *core0)
{
    double speed0 = (double)core0->current_speed + (double)core0->max_surface_rot_speed + RM_P_DOUBLE_EPS;
    double min_rad = (double)(float)core0->upper_limit_radius;
    double max_dist = speed0 * (double)rm->look_ahead_time_world;
    if (min_rad * (double)rm->look_ahead_max_radius_world < max_dist) max_dist = min_rad * (double)rm->look_ahead_max_radius_world;
    if (max_dist < (double)rm->look_ahead_min_distance_world) max_dist = (double)rm->look_ahead_min_distance_world;
    if (max_dist > (double)rm->look_ahead_max_distance_world) max_dist = (double)rm->look_ahead_max_distance_world;
    max_dist -= speed0 * delta_psi_time;
    if (max_dist < min_rad + (double)rm->look_ahead_min_seconds_world * speed0) {
        max_dist = (double)rm->look_ahead_min_seconds_world * speed0 + min_rad;
    }
    return max_dist;
}

void ivp_range_manager_coll_range_intra_objects(ivp_range_manager_t *rm, double delta_psi_time,
                                                const ivp_object_t *obj0, const ivp_object_t *obj1,
                                                double *range0, double *range1)
{
    if (rm->get_coll_range_intra_objects) {
        rm->get_coll_range_intra_objects(rm, delta_psi_time, obj0, obj1, range0, range1);
        return;
    }
    ivp_range_manager_default_intra_objects(rm, delta_psi_time, obj0->physical_core, obj1->physical_core, range0, range1);
}

double ivp_range_manager_coll_range_in_world(ivp_range_manager_t *rm, double delta_psi_time, const ivp_object_t *obj0)
{
    if (rm->get_coll_range_in_world) return rm->get_coll_range_in_world(rm, delta_psi_time, obj0);
    return ivp_range_manager_default_in_world(rm, delta_psi_time, obj0->physical_core);
}

void ivp_range_manager_get_coll_range_intra(const ivp_range_manager_t *rm,
                                            const ivp_core_t *core0, const ivp_core_t *core1,
                                            ivp_real_t *range0, ivp_real_t *range1)
{
    double r0, r1;
    ivp_range_manager_default_intra_objects(rm, (double)rm->delta_psi_time, core0, core1, &r0, &r1);
    *range0 = (ivp_real_t)r0;
    *range1 = (ivp_real_t)r1;
}

ivp_real_t ivp_range_manager_get_coll_range_world(const ivp_range_manager_t *rm, const ivp_core_t *core0)
{
    return (ivp_real_t)ivp_range_manager_default_in_world(rm, (double)rm->delta_psi_time, core0);
}

ivp_real_t ivp_range_manager_get_pair_margin(const ivp_range_manager_t *rm,
                                             const ivp_core_t *core0, const ivp_core_t *core1)
{
    ivp_real_t r0, r1;
    ivp_range_manager_get_coll_range_intra(rm, core0, core1, &r0, &r1);
    return r0 + r1;
}
