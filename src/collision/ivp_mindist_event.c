/* ivp_mindist_event.c — Mindist event prediction and feature-pair dispatch */

#include <ivp/collision/ivp_mindist_event.h>
#include <ivp/collision/ivp_3d_solver.h>
#include <ivp/collision/ivp_matrix_cache.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_vec3.h>

#include <string.h>

typedef bool (*mindist_case_fn_t)(const ivp_mindist_entry_t *entry,
                                  const ivp_collision_settings_t *settings,
                                  ivp_time_t now,
                                  ivp_time_t max_time,
                                  ivp_mindist_event_prediction_t *out_prediction);

typedef struct event_solver_data {
    ivp_vec3r_t anchor_a_local;
    ivp_vec3r_t anchor_b_local;
} event_solver_data_t;

static ivp_real_t distance_anchor_anchor(const ivp_transform_r_t *m_a,
                                         const ivp_transform_r_t *m_b,
                                         void *ud)
{
    event_solver_data_t *data = (event_solver_data_t *)ud;
    ivp_vec3r_t world_a;
    ivp_vec3r_t world_b;
    ivp_vec3r_t diff;
    ivp_transform_point(&world_a, m_a, &data->anchor_a_local);
    ivp_transform_point(&world_b, m_b, &data->anchor_b_local);
    ivp_vec3r_sub(&diff, &world_b, &world_a);
    return ivp_vec3r_length(&diff);
}

static bool predict_with_3d_solver(const ivp_mindist_entry_t *entry,
                                   const ivp_collision_settings_t *settings,
                                   ivp_time_t now,
                                   ivp_time_t max_time,
                                   ivp_mindist_event_prediction_t *out_prediction)
{
    if (!entry || !settings || !out_prediction) return false;
    if (!entry->obj_a || !entry->obj_b) return false;
    if (max_time <= now) return false;

    ivp_vec3r_t rel_speed;
    ivp_vec3r_sub(&rel_speed,
                  &entry->obj_b->core.speed,
                  &entry->obj_a->core.speed);

    ivp_real_t closing_speed = -ivp_vec3r_dot(&rel_speed, &entry->last_normal);
    if (closing_speed <= IVP_IMPACT_EPS && entry->last_distance > settings->min_coll_dist) {
        return false;
    }

    event_solver_data_t data;
    ivp_transform_inv_point(&data.anchor_a_local,
                            &entry->obj_a->core.m_world_f_core_last_psi,
                            &entry->closest_point_a);
    ivp_transform_inv_point(&data.anchor_b_local,
                            &entry->obj_b->core.m_world_f_core_last_psi,
                            &entry->closest_point_b);

    ivp_matrix_cache_t cache_a;
    ivp_matrix_cache_t cache_b;
    ivp_matrix_cache_init(&cache_a, entry->obj_a, now, max_time - now);
    ivp_matrix_cache_init(&cache_b, entry->obj_b, now, max_time - now);

    ivp_3d_solver_t solver;
    ivp_3d_solver_init(&solver, distance_anchor_anchor, &data, settings->tolerance);

    ivp_time_t hit_time;
    if (!ivp_3d_solver_find_first_t(&solver,
                                    settings->min_coll_dist,
                                    now,
                                    max_time,
                                    &cache_a,
                                    &cache_b,
                                    &hit_time)) {
        return false;
    }

    out_prediction->kind = IVP_MINDIST_EVENT_IMPACT;
    out_prediction->event_time = hit_time;
    out_prediction->status_a = entry->synapse[0].status;
    out_prediction->status_b = entry->synapse[1].status;
    out_prediction->expected_gap = settings->min_coll_dist;
    return true;
}

static bool predict_case_pp(const ivp_mindist_entry_t *entry,
                            const ivp_collision_settings_t *settings,
                            ivp_time_t now,
                            ivp_time_t max_time,
                            ivp_mindist_event_prediction_t *out_prediction)
{
    return predict_with_3d_solver(entry, settings, now, max_time, out_prediction);
}

static bool predict_case_pf(const ivp_mindist_entry_t *entry,
                            const ivp_collision_settings_t *settings,
                            ivp_time_t now,
                            ivp_time_t max_time,
                            ivp_mindist_event_prediction_t *out_prediction)
{
    return predict_with_3d_solver(entry, settings, now, max_time, out_prediction);
}

static bool predict_case_pk(const ivp_mindist_entry_t *entry,
                            const ivp_collision_settings_t *settings,
                            ivp_time_t now,
                            ivp_time_t max_time,
                            ivp_mindist_event_prediction_t *out_prediction)
{
    return predict_with_3d_solver(entry, settings, now, max_time, out_prediction);
}

static bool predict_case_kk(const ivp_mindist_entry_t *entry,
                            const ivp_collision_settings_t *settings,
                            ivp_time_t now,
                            ivp_time_t max_time,
                            ivp_mindist_event_prediction_t *out_prediction)
{
    return predict_with_3d_solver(entry, settings, now, max_time, out_prediction);
}

static bool predict_case_ball(const ivp_mindist_entry_t *entry,
                              const ivp_collision_settings_t *settings,
                              ivp_time_t now,
                              ivp_time_t max_time,
                              ivp_mindist_event_prediction_t *out_prediction)
{
    return predict_with_3d_solver(entry, settings, now, max_time, out_prediction);
}

static mindist_case_fn_t mim_function_table[IVP_ST_MAX_LEGAL][IVP_ST_MAX_LEGAL] = {
    { predict_case_pp,   predict_case_pk,   predict_case_pf,   predict_case_ball },
    { predict_case_pk,   predict_case_kk,   predict_case_pk,   predict_case_ball },
    { predict_case_pf,   predict_case_pk,   predict_case_pf,   predict_case_ball },
    { predict_case_ball, predict_case_ball, predict_case_ball, predict_case_ball }
};

bool ivp_mindist_event_predict(const ivp_mindist_entry_t *entry,
                               const ivp_collision_settings_t *settings,
                               ivp_time_t now,
                               ivp_time_t max_time,
                               ivp_mindist_event_prediction_t *out_prediction)
{
    if (!entry || !settings || !out_prediction) return false;

    memset(out_prediction, 0, sizeof(*out_prediction));
    out_prediction->kind = IVP_MINDIST_EVENT_NONE;

    if (!entry->active || !entry->obj_a || !entry->obj_b) return false;

    int sa = (int)entry->synapse[0].status;
    int sb = (int)entry->synapse[1].status;
    if (sa < 0 || sa >= IVP_ST_MAX_LEGAL) sa = (int)IVP_ST_POINT;
    if (sb < 0 || sb >= IVP_ST_MAX_LEGAL) sb = (int)IVP_ST_POINT;

    mindist_case_fn_t fn = mim_function_table[sa][sb];
    if (!fn) return false;

    if (fn(entry, settings, now, max_time, out_prediction)) {
        return out_prediction->kind != IVP_MINDIST_EVENT_NONE;
    }

    return false;
}

void ivp_mindist_event_to_time_event(const ivp_mindist_entry_t *entry,
                                     const ivp_mindist_event_prediction_t *prediction,
                                     ivp_time_event_t *out_event)
{
    if (!entry || !prediction || !out_event) return;

    memset(out_event, 0, sizeof(*out_event));
    out_event->trigger_time = prediction->event_time;
    out_event->type = (prediction->kind == IVP_MINDIST_EVENT_IMPACT)
        ? IVP_TIME_EVENT_MINDIST_IMPACT
        : IVP_TIME_EVENT_MINDIST_RECALC;

    out_event->obj_a = entry->obj_a->handle;
    out_event->obj_b = entry->obj_b->handle;

    out_event->status_a = prediction->status_a;
    out_event->status_b = prediction->status_b;
    out_event->expected_gap = prediction->expected_gap;
}
