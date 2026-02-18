/* ivp_matrix_cache.c — Pre-computed transform cache for CCD */

#include <ivp/collision/ivp_matrix_cache.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_fast_math.h>
#include <string.h>

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_matrix_cache_init(ivp_matrix_cache_t *cache,
                             ivp_object_t *obj,
                             ivp_time_t base_time,
                             ivp_time_t dt)
{
    memset(cache, 0, sizeof(*cache));
    cache->object = obj;
    cache->base_time = base_time;
    cache->dt = dt;

    /* Pre-compute step 0 = start of PSI (current position) */
    cache->transforms[0] = obj->core.m_world_f_core_last_psi;
    cache->computed[0] = true;
}

/* ── Lazy compute at sub-step ──────────────────────────────────────────── */

static void compute_transform_at_fraction(const ivp_core_t *core,
                                            ivp_real_t fraction,
                                            ivp_real_t dt,
                                            ivp_transform_r_t *out)
{
    /* Linear interpolation of position */
    ivp_real_t sub_dt = fraction * dt;

    /* Position: pos + speed * sub_dt */
    ivp_vec3r_t pos = core->pos_world_f_core_last_psi;
    ivp_vec3r_add_scaled(&pos, &core->speed, sub_dt);

    /* Orientation: integrate quaternion by sub_dt fraction of rot_speed */
    ivp_quatr_t q = core->q_world_f_core_last_psi;
    if (sub_dt > IVP_REAL(0.0)) {
        /* Angular velocity → quaternion derivative */
        ivp_real_t wx = (ivp_real_t)core->rot_speed.k[0] * sub_dt;
        ivp_real_t wy = (ivp_real_t)core->rot_speed.k[1] * sub_dt;
        ivp_real_t wz = (ivp_real_t)core->rot_speed.k[2] * sub_dt;
        ivp_real_t angle = ivp_sqrt(wx * wx + wy * wy + wz * wz);

        if (angle > IVP_REAL_EPS) {
            ivp_real_t half_angle = angle * IVP_REAL(0.5);
            ivp_real_t s = ivp_sin(half_angle) / angle;

            ivp_quatr_t dq;
            dq.x = wx * s;
            dq.y = wy * s;
            dq.z = wz * s;
            dq.w = ivp_cos(half_angle);

            ivp_quatr_t result;
            ivp_quat_mult(&result, &q, &dq);
            ivp_quat_fast_normalize(&result);
            q = result;
        }
    }

    /* Build transform from pos and quaternion */
    out->pos = pos;
    ivp_quat_to_mat3(&q, &out->rot);
}

const ivp_transform_r_t *ivp_matrix_cache_get(ivp_matrix_cache_t *cache,
                                                 int index)
{
    if (index < 0) index = 0;
    if (index > IVP_3D_SOLVER_MAX_STEPS) index = IVP_3D_SOLVER_MAX_STEPS;

    if (!cache->computed[index]) {
        ivp_real_t fraction = (ivp_real_t)index / (ivp_real_t)IVP_3D_SOLVER_MAX_STEPS;
        compute_transform_at_fraction(&cache->object->core, fraction,
                                       (ivp_real_t)cache->dt,
                                       &cache->transforms[index]);
        cache->computed[index] = true;
    }

    return &cache->transforms[index];
}

const ivp_transform_r_t *ivp_matrix_cache_get_at_time(ivp_matrix_cache_t *cache,
                                                         ivp_time_t t)
{
    if (cache->dt <= 0.0) {
        return &cache->transforms[0];
    }

    ivp_real_t fraction = (ivp_real_t)((t - cache->base_time) / cache->dt);
    if (fraction < IVP_REAL(0.0)) fraction = IVP_REAL(0.0);
    if (fraction > IVP_REAL(1.0)) fraction = IVP_REAL(1.0);

    int index = (int)(fraction * (ivp_real_t)IVP_3D_SOLVER_MAX_STEPS);
    return ivp_matrix_cache_get(cache, index);
}
