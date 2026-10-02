#include "solver/ivp_impact_rescue.h"

#include <ivp/math/ivp_fast_math.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/solver/ivp_solver_constants.h>

/* Reference constants from ivp_impact.cxx */
#define IVP_ROT_MAX_UNCERTAINTY       IVP_REAL(0.5)
#define IVP_COLL_DETECT_MIN_INV_TIME  IVP_REAL(200.0)
#define IVP_COLL_DETECT_MIN_TIME      (IVP_REAL(1.0) / IVP_COLL_DETECT_MIN_INV_TIME)

ivp_real_t ivp_impact_rot_speed_uncertainty(const ivp_contact_point_t *cp,
                                            const ivp_collision_settings_t *settings)
{
    ivp_real_t rot_uncertainty = IVP_REAL(0.0);
    if (!cp || !settings) return rot_uncertainty;

    const ivp_object_t *objs[2] = { cp->obj_a, cp->obj_b };
    for (int i = 0; i < 2; i++) {
        const ivp_object_t *obj = objs[i];
        if (!obj) continue;
        if (obj->core.physical_unmoveable) continue;
        /* Reference excludes ball synapses from this uncertainty term. */
        if (obj->type == IVP_OBJECT_BALL) continue;

        ivp_real_t wx = (ivp_real_t)obj->core.rot_speed.k[0];
        ivp_real_t wy = (ivp_real_t)obj->core.rot_speed.k[1];
        ivp_real_t wz = (ivp_real_t)obj->core.rot_speed.k[2];
        ivp_real_t omega2 = wx * wx + wy * wy + wz * wz;

        ivp_real_t alpha = ivp_sqrt(omega2) * IVP_COLL_DETECT_MIN_TIME;
        if (alpha > IVP_ROT_MAX_UNCERTAINTY) {
            alpha = IVP_ROT_MAX_UNCERTAINTY;
        }

        /* Reference: IVP_Inline_Math::cosd(alpha) -- alpha is treated as
         * DEGREES there (IVP_ROT_MAX_UNCERTAINTY is "max rot degree").  Using
         * radians here inflated the uncertainty ~3000x and turned the rescue
         * speed into tens of m/s for any rotating body. */
        ivp_real_t alpha_rad = alpha * (ivp_real_t)(3.14159265358979323846 / 180.0);
        ivp_real_t uncertain_way =
            (IVP_REAL(1.0) - ivp_cos(alpha_rad)) * obj->core.upper_limit_radius;
        rot_uncertainty += uncertain_way * settings->inv_delta_psi_time;
    }
    return rot_uncertainty;
}

ivp_real_t ivp_impact_rescue_speed_addon(const ivp_contact_point_t *cp,
                                         const ivp_collision_settings_t *settings)
{
    if (!cp || !settings) return IVP_REAL(0.0);

    ivp_real_t rescue_base = IVP_REAL(0.0);
    if (cp->gap_length < settings->min_coll_dist) {
        ivp_real_t rescue_depth = settings->min_coll_dist - cp->gap_length;
        rescue_base += rescue_depth * settings->inv_delta_psi_time;
    }
    rescue_base += ivp_impact_rot_speed_uncertainty(cp, settings);

    return IVP_REAL(2.0) * rescue_base;
}

ivp_real_t ivp_impact_delay_threshold_speed(const ivp_contact_point_t *cp,
                                            const ivp_collision_settings_t *settings)
{
    if (!settings) return IVP_REAL(0.0);
    return settings->min_vertical_speed_at_collision
         + ivp_impact_rescue_speed_addon(cp, settings);
}

ivp_real_t ivp_impact_rescue_speed_target(const ivp_contact_point_t *cp,
                                          const ivp_collision_settings_t *settings)
{
    return ivp_impact_delay_threshold_speed(cp, settings)
         * IVP_SAFETY_FACTOR_FOR_DELAY;
}
