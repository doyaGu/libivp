/* ivp_radar.c — Proximity query implementation
 *
 * Iterates all objects in the environment, computes distance to source,
 * and calls the user's hit callback for matches within range.
 */

#include <ivp/collision/ivp_radar.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/collision/ivp_collision_filter.h>
#include <math.h>

void ivp_radar_config_init(ivp_radar_config_t *cfg)
{
    cfg->max_range       = IVP_REAL(100.0);
    cfg->use_filter      = true;
    cfg->skip_unmoveable = false;
    cfg->hit_fn          = NULL;
    cfg->ctx             = NULL;
}

void ivp_environment_do_radar(ivp_environment_t *env,
                              ivp_object_t *source,
                              const ivp_radar_config_t *cfg)
{
    if (!env || !source || !cfg || !cfg->hit_fn) return;

    const ivp_core_t *src_core = ivp_object_get_core_const(source);
    if (!src_core) return;

    ivp_vec3r_t src_pos;
    ivp_core_get_position(src_core, &src_pos);
    ivp_real_t src_radius = src_core->upper_limit_radius;

    int n_objects = ivp_environment_get_object_count(env);

    for (int i = 0; i < n_objects; i++) {
        ivp_object_t *target = ivp_environment_get_object(env, i);
        if (!target || target == source) continue;

        const ivp_core_t *tgt_core = ivp_object_get_core_const(target);
        if (!tgt_core) continue;

        if (cfg->skip_unmoveable && tgt_core->physical_unmoveable) continue;

        /* Apply collision filter if requested */
        if (cfg->use_filter) {
            if (!ivp_collision_filter_check(&env->collision_filter, source, target))
                continue;
        }

        /* Compute center-to-center distance */
        ivp_vec3r_t tgt_pos;
        ivp_core_get_position(tgt_core, &tgt_pos);

        ivp_vec3r_t diff;
        ivp_vec3r_sub(&diff, &tgt_pos, &src_pos);
        ivp_real_t dist_sq = ivp_vec3r_quad_length(&diff);

        /* Quick rejection: distance > max_range + both radii */
        ivp_real_t max_check = cfg->max_range + src_radius + tgt_core->upper_limit_radius;
        if (dist_sq > max_check * max_check) continue;

        ivp_real_t center_dist = (ivp_real_t)sqrt((double)dist_sq);
        ivp_real_t surface_dist = center_dist - src_radius - tgt_core->upper_limit_radius;
        if (surface_dist < IVP_REAL(0.0)) surface_dist = IVP_REAL(0.0);

        if (surface_dist > cfg->max_range) continue;

        /* Compute direction */
        ivp_vec3r_t direction;
        if (center_dist > IVP_REAL_EPS) {
            ivp_real_t inv = IVP_REAL(1.0) / center_dist;
            ivp_vec3r_set(&direction,
                          diff.k[0] * inv,
                          diff.k[1] * inv,
                          diff.k[2] * inv);
        } else {
            ivp_vec3r_set(&direction, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        }

        cfg->hit_fn(cfg->ctx, target, surface_dist, &direction);
    }
}
