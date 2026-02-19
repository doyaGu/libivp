/* ivp_anomaly_manager.c -- Anomaly detection implementation */

#include <ivp/dynamics/ivp_anomaly_manager.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include <math.h>
#include <string.h>

void ivp_anomaly_manager_init(ivp_anomaly_manager_t *mgr,
                                const ivp_anomaly_config_t *config)
{
    memset(mgr, 0, sizeof(*mgr));
    if (config) {
        mgr->config = *config;
    } else {
        ivp_anomaly_config_init(&mgr->config);
    }
    mgr->active = true;
}

static bool is_nan_or_inf_vec(const ivp_vec3r_t *v)
{
    return !isfinite(v->k[0]) || !isfinite(v->k[1]) || !isfinite(v->k[2]);
}

static void fire_callback(ivp_anomaly_manager_t *mgr,
                           ivp_object_t *obj,
                           ivp_anomaly_type_t type,
                           const char *msg)
{
    mgr->anomalies_detected++;
    if (mgr->config.callback) {
        mgr->config.callback(mgr->config.callback_ctx, obj, type, msg);
    }
}

int ivp_anomaly_check_object(ivp_anomaly_manager_t *mgr,
                               ivp_object_t *obj)
{
    if (!mgr || !mgr->active || !obj) return 0;

    ivp_core_t *core = &obj->core;
    if (core->physical_unmoveable) return 0;

    int detected = 0;

    /* Check NaN position */
    if (is_nan_or_inf_vec(&core->pos_world_f_core_last_psi)) {
        detected |= IVP_ANOMALY_NAN_POSITION;
        fire_callback(mgr, obj, IVP_ANOMALY_NAN_POSITION, "NaN/Inf position");
        if (mgr->config.auto_correct) {
            ivp_vec3r_set(&core->pos_world_f_core_last_psi,
                          IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
            ivp_vec3r_set(&core->speed,
                          IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
            core->rot_speed.k[0] = core->rot_speed.k[1] = core->rot_speed.k[2] = 0.0f;
            mgr->anomalies_corrected++;
        }
    }

    /* Check NaN velocity */
    if (is_nan_or_inf_vec(&core->speed)) {
        detected |= IVP_ANOMALY_NAN_VELOCITY;
        fire_callback(mgr, obj, IVP_ANOMALY_NAN_VELOCITY, "NaN/Inf linear velocity");
        if (mgr->config.auto_correct) {
            ivp_vec3r_set(&core->speed,
                          IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
            mgr->anomalies_corrected++;
        }
    }

    /* Check NaN angular velocity */
    if (!isfinite(core->rot_speed.k[0]) ||
        !isfinite(core->rot_speed.k[1]) ||
        !isfinite(core->rot_speed.k[2])) {
        detected |= IVP_ANOMALY_NAN_VELOCITY;
        fire_callback(mgr, obj, IVP_ANOMALY_NAN_VELOCITY, "NaN/Inf angular velocity");
        if (mgr->config.auto_correct) {
            core->rot_speed.k[0] = core->rot_speed.k[1] = core->rot_speed.k[2] = 0.0f;
            mgr->anomalies_corrected++;
        }
    }

    /* Check excessive linear speed */
    ivp_real_t speed_sq = ivp_vec3r_quad_length(&core->speed);
    ivp_real_t max_sq = mgr->config.max_speed * mgr->config.max_speed;
    if (speed_sq > max_sq) {
        detected |= IVP_ANOMALY_EXCESSIVE_SPEED;
        fire_callback(mgr, obj, IVP_ANOMALY_EXCESSIVE_SPEED, "Excessive linear speed");
        if (mgr->config.auto_correct) {
            ivp_real_t scale = mgr->config.max_speed / ivp_sqrt(speed_sq);
            core->speed.k[0] *= scale;
            core->speed.k[1] *= scale;
            core->speed.k[2] *= scale;
            mgr->anomalies_corrected++;
        }
    }

    /* Check excessive angular speed */
    ivp_real_t ang_sq = (ivp_real_t)core->rot_speed.k[0] * (ivp_real_t)core->rot_speed.k[0] +
                        (ivp_real_t)core->rot_speed.k[1] * (ivp_real_t)core->rot_speed.k[1] +
                        (ivp_real_t)core->rot_speed.k[2] * (ivp_real_t)core->rot_speed.k[2];
    ivp_real_t max_ang_sq = mgr->config.max_angular_speed *
                            mgr->config.max_angular_speed;
    if (ang_sq > max_ang_sq) {
        detected |= IVP_ANOMALY_EXCESSIVE_SPEED;
        fire_callback(mgr, obj, IVP_ANOMALY_EXCESSIVE_SPEED, "Excessive angular speed");
        if (mgr->config.auto_correct) {
            float scale = (float)(mgr->config.max_angular_speed / ivp_sqrt(ang_sq));
            core->rot_speed.k[0] *= scale;
            core->rot_speed.k[1] *= scale;
            core->rot_speed.k[2] *= scale;
            mgr->anomalies_corrected++;
        }
    }

    return detected;
}

void ivp_anomaly_manager_begin_psi(ivp_anomaly_manager_t *mgr)
{
    if (mgr) {
        mgr->collisions_this_psi = 0;
    }
}

bool ivp_anomaly_manager_count_collision(ivp_anomaly_manager_t *mgr,
                                           ivp_object_t *obj)
{
    if (!mgr || !mgr->active) return true;
    if (!obj) return true;

    ivp_core_t *core = &obj->core;
    if (core->physical_unmoveable) return true;

    core->impacts_since_last_PSI++;
    mgr->collisions_this_psi++;

    if (mgr->config.max_collisions_per_psi > 0 &&
        core->impacts_since_last_PSI > mgr->config.max_collisions_per_psi) {
        if (core->impacts_since_last_PSI == mgr->config.max_collisions_per_psi + 1) {
            fire_callback(mgr, obj, IVP_ANOMALY_MAX_COLLISIONS,
                          "Max collisions per PSI exceeded");
        }
        if (mgr->config.freeze_on_max_collisions) {
            if (core->movement_state != IVP_MT_NOT_SIM) {
                core->movement_state = IVP_MT_NOT_SIM;
                mgr->anomalies_corrected++;
            }
        }
        return false;
    }
    return true;
}

int ivp_anomaly_check_all(ivp_anomaly_manager_t *mgr,
                            struct ivp_environment *env)
{
    if (!mgr || !env) return 0;

    int total = 0;
    int n = ivp_environment_get_object_count(env);
    for (int i = 0; i < n; i++) {
        ivp_object_t *obj = ivp_environment_get_object(env, i);
        if (obj) {
            total += (ivp_anomaly_check_object(mgr, obj) != 0) ? 1 : 0;
        }
    }
    return total;
}

void ivp_controller_anomaly_check(const ivp_event_sim_t *event_sim,
                                    ivp_object_t *obj,
                                    void *ctx)
{
    ivp_anomaly_manager_t *mgr = (ivp_anomaly_manager_t *)ctx;
    ivp_anomaly_check_object(mgr, obj);
}
