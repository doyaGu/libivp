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

static ivp_real_t topology_event_speed(ivp_mindist_event_kind_t kind,
                                       ivp_real_t abs_normal_speed,
                                       ivp_real_t tangential_speed)
{
    switch (kind) {
        case IVP_MINDIST_EVENT_KK_PARALLEL:
        case IVP_MINDIST_EVENT_PK_NOT_MORE_PARALLEL:
            return tangential_speed + abs_normal_speed * IVP_REAL(0.2);

        case IVP_MINDIST_EVENT_KK_PF:
        case IVP_MINDIST_EVENT_PK_PF:
        case IVP_MINDIST_EVENT_PF_NPF:
            return abs_normal_speed + tangential_speed * IVP_REAL(0.25);

        case IVP_MINDIST_EVENT_BALL_RECALC:
            return abs_normal_speed * IVP_REAL(0.5) + tangential_speed * IVP_REAL(0.5);

        default:
            return abs_normal_speed + tangential_speed * IVP_REAL(0.5);
    }
}

static ivp_time_t topology_min_eta_fraction(ivp_mindist_event_kind_t kind,
                                            ivp_time_t horizon)
{
    ivp_time_t frac = (ivp_time_t)0.05;
    switch (kind) {
        case IVP_MINDIST_EVENT_KK_PARALLEL:
        case IVP_MINDIST_EVENT_PK_NOT_MORE_PARALLEL:
            frac = (ivp_time_t)0.03;
            break;
        case IVP_MINDIST_EVENT_KK_PF:
        case IVP_MINDIST_EVENT_PK_PF:
        case IVP_MINDIST_EVENT_PF_NPF:
            frac = (ivp_time_t)0.08;
            break;
        default:
            break;
    }

    ivp_time_t min_eta = horizon * frac;
    if (min_eta < (ivp_time_t)1e-6) min_eta = (ivp_time_t)1e-6;
    return min_eta;
}

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
                                   ivp_mindist_event_kind_t impact_kind,
                                   ivp_mindist_event_kind_t topology_kind,
                                   ivp_mindist_event_prediction_t *out_prediction)
{
    if (!entry || !settings || !out_prediction) return false;
    if (!entry->obj_a || !entry->obj_b) return false;
    if (max_time <= now) return false;

    /* Per-entry collision distance from progressive selector.
     * Reference: IVP_Mindist::get_coll_dist(). */
    int sel = entry->coll_dist_selector;
    if (sel < 0) sel = 0;
    if (sel >= IVP_MAX_STEPS_FOR_COLLDIST_DECREASE)
        sel = IVP_MAX_STEPS_FOR_COLLDIST_DECREASE - 1;
    ivp_real_t coll_dist = settings->coll_dists[sel];

    ivp_vec3r_t rel_speed;
    ivp_vec3r_sub(&rel_speed,
                  &entry->obj_b->core.speed,
                  &entry->obj_a->core.speed);
    ivp_real_t closing_speed = -ivp_vec3r_dot(&rel_speed, &entry->last_normal);
    ivp_real_t rel_speed_len = ivp_vec3r_length(&rel_speed);
    ivp_real_t tangential_speed = rel_speed_len;
    {
        ivp_real_t normal_sq = closing_speed * closing_speed;
        ivp_real_t rel_sq = rel_speed_len * rel_speed_len;
        if (rel_sq > normal_sq) {
            tangential_speed = ivp_sqrt(rel_sq - normal_sq);
        } else {
            tangential_speed = IVP_REAL(0.0);
        }
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
                                    coll_dist,
                                    now,
                                    max_time,
                                    &cache_a,
                                    &cache_b,
                                    &hit_time)) {
        ivp_real_t sec_dist = coll_dist + settings->mindist_change_force_dist;
        if (entry->last_distance <= sec_dist) {
            ivp_mindist_event_kind_t fallback_kind = topology_kind;
            if (topology_kind == IVP_MINDIST_EVENT_PK_KK) {
                if (ivp_fabs(closing_speed) <= IVP_IMPACT_EPS) {
                    fallback_kind = IVP_MINDIST_EVENT_PK_NOT_MORE_PARALLEL;
                } else if (entry->last_distance <= coll_dist + settings->mindist_change_force_dist * IVP_REAL(0.25)) {
                    fallback_kind = IVP_MINDIST_EVENT_PK_PF;
                }
            } else if (topology_kind == IVP_MINDIST_EVENT_KK_PARALLEL) {
                const ivp_real_t abs_normal = ivp_fabs(closing_speed);
                const ivp_real_t kk_parallel_band = tangential_speed * IVP_REAL(0.5) + IVP_IMPACT_EPS;
                if (abs_normal > kk_parallel_band) {
                    fallback_kind = IVP_MINDIST_EVENT_KK_PF;
                } else {
                    fallback_kind = IVP_MINDIST_EVENT_KK_PARALLEL;
                }
            }

            out_prediction->kind = fallback_kind;
            {
                ivp_real_t slack = sec_dist - entry->last_distance;
                if (slack < IVP_REAL(0.0)) slack = IVP_REAL(0.0);

                ivp_real_t event_speed = topology_event_speed(
                    fallback_kind,
                    ivp_fabs(closing_speed),
                    tangential_speed);
                if (event_speed < IVP_REAL(1e-4)) event_speed = IVP_REAL(1e-4);

                ivp_time_t horizon = max_time - now;
                ivp_time_t eta = (ivp_time_t)(slack / event_speed);
                ivp_time_t min_eta = topology_min_eta_fraction(fallback_kind, horizon);

                ivp_time_t dt = eta;
                if (dt < min_eta) dt = min_eta;
                if (dt > horizon) dt = horizon;

                out_prediction->event_time = now + dt;
            }
            out_prediction->status_a = entry->synapse[0].status;
            out_prediction->status_b = entry->synapse[1].status;
            out_prediction->expected_gap = coll_dist;
            return true;
        }
        return false;
    }

    out_prediction->kind = impact_kind;
    out_prediction->event_time = hit_time;
    out_prediction->status_a = entry->synapse[0].status;
    out_prediction->status_b = entry->synapse[1].status;
    out_prediction->expected_gap = coll_dist;
    return true;
}

static bool predict_case_pp(const ivp_mindist_entry_t *entry,
                            const ivp_collision_settings_t *settings,
                            ivp_time_t now,
                            ivp_time_t max_time,
                            ivp_mindist_event_prediction_t *out_prediction)
{
    return predict_with_3d_solver(entry, settings, now, max_time,
                                  IVP_MINDIST_EVENT_PP_COLL,
                                  IVP_MINDIST_EVENT_PP_PK,
                                  out_prediction);
}

static bool predict_case_pf(const ivp_mindist_entry_t *entry,
                            const ivp_collision_settings_t *settings,
                            ivp_time_t now,
                            ivp_time_t max_time,
                            ivp_mindist_event_prediction_t *out_prediction)
{
    return predict_with_3d_solver(entry, settings, now, max_time,
                                  IVP_MINDIST_EVENT_PF_COLL,
                                  IVP_MINDIST_EVENT_PF_NPF,
                                  out_prediction);
}

static bool predict_case_pk(const ivp_mindist_entry_t *entry,
                            const ivp_collision_settings_t *settings,
                            ivp_time_t now,
                            ivp_time_t max_time,
                            ivp_mindist_event_prediction_t *out_prediction)
{
    return predict_with_3d_solver(entry, settings, now, max_time,
                                  IVP_MINDIST_EVENT_PK_COLL,
                                  IVP_MINDIST_EVENT_PK_KK,
                                  out_prediction);
}

static bool predict_case_kk(const ivp_mindist_entry_t *entry,
                            const ivp_collision_settings_t *settings,
                            ivp_time_t now,
                            ivp_time_t max_time,
                            ivp_mindist_event_prediction_t *out_prediction)
{
    return predict_with_3d_solver(entry, settings, now, max_time,
                                  IVP_MINDIST_EVENT_KK_COLL,
                                  IVP_MINDIST_EVENT_KK_PARALLEL,
                                  out_prediction);
}

static bool predict_case_ball(const ivp_mindist_entry_t *entry,
                              const ivp_collision_settings_t *settings,
                              ivp_time_t now,
                              ivp_time_t max_time,
                              ivp_mindist_event_prediction_t *out_prediction)
{
    return predict_with_3d_solver(entry, settings, now, max_time,
                                  IVP_MINDIST_EVENT_BALL_COLL,
                                  IVP_MINDIST_EVENT_BALL_RECALC,
                                  out_prediction);
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
    out_event->type = ivp_mindist_event_is_impact(prediction->kind)
        ? IVP_TIME_EVENT_MINDIST_IMPACT
        : IVP_TIME_EVENT_MINDIST_RECALC;
    out_event->mindist_kind = (uint32_t)prediction->kind;

    out_event->obj_a = entry->obj_a->handle;
    out_event->obj_b = entry->obj_b->handle;

    out_event->status_a = prediction->status_a;
    out_event->status_b = prediction->status_b;
    out_event->expected_gap = prediction->expected_gap;
}
