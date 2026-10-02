/* ivp_environment_collision.c — Environment-level collision orchestration
 *
 * Broadphase detection and collision resolution that operate on
 * ivp_environment_t. Moved here from solver/ to break the
 * dynamics ↔ solver circular dependency.
 */
#include <ivp/dynamics/ivp_environment_collision.h>
#include <ivp/dynamics/ivp_anomaly_manager.h>
#include <ivp/collision/ivp_collision_filter.h>
#include <ivp/collision/ivp_broadphase.h>
#include <ivp/collision/ivp_ccd_narrowphase.h>
#include <ivp/collision/ivp_hull_manager.h>
#include <ivp/collision/ivp_mindist_event.h>
#include <ivp/collision/ivp_mindist_persistent.h>
#include <ivp/collision/ivp_time_event_queue.h>
#include <ivp/solver/ivp_impact.h>
#include <ivp/solver/ivp_impact_system.h>
#include <ivp/solver/ivp_friction_system.h>
#include <ivp/solver/ivp_solver_metrics.h>
#include <ivp/events/ivp_listener_collision.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/collision/ivp_mindist.h>
#include <ivp/collision/ivp_compact_ledge_solver.h>
#include <ivp/collision/ivp_range_manager.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/dynamics/ivp_sim_unit.h>
#include <string.h>
#include <math.h>
#include "dynamics/ivp_environment_events.h"

static ivp_collision_event_t ivp_make_collision_event(const ivp_contact_point_t *cp,
                                                      ivp_real_t closing_speed,
                                                      ivp_real_t impulse)
{
    ivp_collision_event_t event;
    event.obj_a = cp->obj_a;
    event.obj_b = cp->obj_b;
    event.contact_point = cp->point_a;
    event.contact_normal = cp->normal;
    event.closing_speed = closing_speed;
    event.impulse = impulse;
    event.gap_distance = cp->gap_length;
    return event;
}

static ivp_real_t ivp_environment_get_effective_step_dt(const ivp_environment_t *env)
{
    if (!env) return IVP_REAL(0.0);
    if (env->delta_psi_time <= IVP_REAL_EPS) return IVP_REAL(0.0);

    if (env->current_sim_unit_index >= 0) {
        const int sim_units = env->sim_unit_count > 0 ? env->sim_unit_count : 1;
        return env->delta_psi_time / (ivp_real_t)sim_units;
    }

    return env->delta_psi_time;
}

/* ── Environment collision resolution ───────────────────────────────────── */

static ivp_friction_core_pair_t *find_pair_for_cores(ivp_friction_system_t *fs,
                                                      ivp_core_t *core_a,
                                                      ivp_core_t *core_b)
{
    if (!fs || !core_a || !core_b) return NULL;
    for (int p = 0; p < fs->pair_count; p++) {
        ivp_friction_core_pair_t *fp = &fs->pairs[p];
        bool same_order = fp->cores[0] == core_a && fp->cores[1] == core_b;
        bool rev_order = fp->cores[0] == core_b && fp->cores[1] == core_a;
        if (same_order || rev_order) {
            return fp;
        }
    }
    return NULL;
}

static void wake_objects_after_impulse(ivp_environment_t *env, ivp_real_t step_dt)
{
    const int n = ivp_environment_get_object_count(env);
    for (int i = 0; i < n; i++) {
        ivp_object_t *obj = ivp_environment_get_object(env, i);
        if (!obj || obj->core.physical_unmoveable) continue;
        if (obj->core.movement_state != IVP_MT_NOT_SIM) continue;

        /* Revive: transfer core from still_slot to a new active sim unit
         * so it participates in subsequent PSI steps (integration, etc.).
         * Reference: frozen objects hit by impulses must be fully revived,
         * not just have their state flag flipped. */
        if (env->sim_units_manager &&
            obj->core.sim_unit == &env->sim_units_manager->still_slot) {
            ivp_sim_unit_revive_core(env->sim_units_manager, &obj->core);
        } else {
            obj->core.movement_state = IVP_MT_SLOW;
        }
        obj->core.movement_check_counter = 0;
        ivp_environment_emit_event_internal(env, IVP_EVENT_OBJECT_UNFROZEN,
                                            step_dt, obj->dense_index, 0);
    }
}

/* Retrieve the per-entry collision distance using the progressive
 * selector.  Reference: IVP_Mindist::get_coll_dist(). */
static inline ivp_real_t get_entry_coll_dist(const ivp_mindist_entry_t *entry,
                                              const ivp_collision_settings_t *settings)
{
    int sel = entry->coll_dist_selector;
    if (sel < 0) sel = 0;
    if (sel >= IVP_MAX_STEPS_FOR_COLLDIST_DECREASE)
        sel = IVP_MAX_STEPS_FOR_COLLDIST_DECREASE - 1;
    return settings->coll_dists[sel];
}

/* Progressive collision distance tightening.
 * Reference: IVP_Mindist::update_exact_mindist_events — decrements
 * coll_dist_selector every 3 event recalculations (ivp_mindist.cxx). */
static void update_coll_dist_selector(ivp_mindist_entry_t *entry)
{
    if (entry->coll_dist_selector > 0) {
        entry->coll_dist_recalc_count++;
        if (entry->coll_dist_recalc_count > 2) {
            entry->coll_dist_selector--;
            entry->coll_dist_recalc_count = 0;
        }
    }
}

static bool time_event_is_impact(const ivp_time_event_t *event)
{
    if (!event) return false;
    if (event->mindist_kind != 0u) {
        return (event->mindist_kind & 0x0Fu) == 0u;
    }
    return event->type == IVP_TIME_EVENT_MINDIST_IMPACT;
}

static bool topology_event_should_tighten(uint32_t kind)
{
    switch (kind) {
        case IVP_MINDIST_EVENT_PP_PK:
        case IVP_MINDIST_EVENT_PF_NPF:
        case IVP_MINDIST_EVENT_PK_PF:
        case IVP_MINDIST_EVENT_PK_KK:
        case IVP_MINDIST_EVENT_KK_PF:
            return true;
        case IVP_MINDIST_EVENT_PK_NOT_MORE_PARALLEL:
        case IVP_MINDIST_EVENT_KK_PARALLEL:
            return false;
        default:
            return true;
    }
}

static ivp_mindist_event_kind_t impact_kind_from_status(ivp_synapse_status_t sa,
                                                        ivp_synapse_status_t sb)
{
    if (sa == IVP_ST_BALL || sb == IVP_ST_BALL) {
        return IVP_MINDIST_EVENT_BALL_COLL;
    }
    if (sa == IVP_ST_EDGE && sb == IVP_ST_EDGE) {
        return IVP_MINDIST_EVENT_KK_COLL;
    }
    if ((sa == IVP_ST_POINT && sb == IVP_ST_EDGE) ||
        (sa == IVP_ST_EDGE && sb == IVP_ST_POINT)) {
        return IVP_MINDIST_EVENT_PK_COLL;
    }
    if ((sa == IVP_ST_POINT && sb == IVP_ST_TRIANGLE) ||
        (sa == IVP_ST_TRIANGLE && sb == IVP_ST_POINT)) {
        return IVP_MINDIST_EVENT_PF_COLL;
    }
    if (sa == IVP_ST_POINT && sb == IVP_ST_POINT) {
        return IVP_MINDIST_EVENT_PP_COLL;
    }
    return IVP_MINDIST_EVENT_PF_COLL;
}

static void apply_topology_kind_to_synapse(ivp_mindist_entry_t *entry,
                                           uint32_t kind,
                                           ivp_synapse_status_t fallback_a,
                                           ivp_synapse_status_t fallback_b)
{
    if (!entry) return;

    ivp_synapse_status_t sa = fallback_a;
    ivp_synapse_status_t sb = fallback_b;

    switch (kind) {
        case IVP_MINDIST_EVENT_PP_PK:
            sa = IVP_ST_POINT;
            sb = IVP_ST_EDGE;
            break;
        case IVP_MINDIST_EVENT_PF_NPF:
        case IVP_MINDIST_EVENT_PK_PF:
        case IVP_MINDIST_EVENT_KK_PF:
            sa = IVP_ST_POINT;
            sb = IVP_ST_TRIANGLE;
            break;
        case IVP_MINDIST_EVENT_PK_KK:
        case IVP_MINDIST_EVENT_KK_PARALLEL:
            sa = IVP_ST_EDGE;
            sb = IVP_ST_EDGE;
            break;
        case IVP_MINDIST_EVENT_PK_NOT_MORE_PARALLEL:
            sa = IVP_ST_POINT;
            sb = IVP_ST_EDGE;
            break;
        case IVP_MINDIST_EVENT_BALL_RECALC:
            sa = IVP_ST_BALL;
            sb = IVP_ST_BALL;
            break;
        default:
            break;
    }

    entry->synapse[0].status = sa;
    entry->synapse[1].status = sb;
}

static void schedule_followup_mindist_event(ivp_environment_t *env,
                                            const ivp_collision_settings_t *settings,
                                            ivp_mindist_entry_t *entry,
                                            ivp_time_t now,
                                            ivp_time_t horizon_end)
{
    if (!env || !settings || !entry) return;
    if (horizon_end <= now) return;

    /* Debounce: if this entry already has a pending event in the current
     * horizon window, avoid re-scheduling to reduce jittery re-queues. */
    if (entry->event_registered && entry->event_time > now && entry->event_time <= horizon_end) {
        return;
    }

    ivp_mindist_event_prediction_t prediction;
    if (!ivp_mindist_event_predict(entry, settings, now, horizon_end, &prediction)) {
        return;
    }
    if (!isfinite((double)prediction.event_time)) {
        return;
    }

    /* Keep events strictly in the future to avoid immediate pop/requeue loops
     * when prediction lands exactly on current time due to numeric noise. */
    {
        const ivp_time_t horizon = horizon_end - now;
        const ivp_time_t min_ahead = horizon > (ivp_time_t)1e-6
            ? horizon * (ivp_time_t)0.1
            : (ivp_time_t)1e-6;
        if (prediction.event_time <= now + min_ahead) {
            prediction.event_time = now + min_ahead;
            if (prediction.event_time > horizon_end) {
                prediction.event_time = horizon_end;
            }
        }
        if (!isfinite((double)prediction.event_time)) {
            return;
        }
    }

    ivp_time_event_t time_event;
    ivp_mindist_event_to_time_event(entry, &prediction, &time_event);
    if (ivp_time_event_queue_push(&env->time_event_queue, &time_event)) {
        entry->event_registered = true;
        entry->event_time = prediction.event_time;
    }
}

/* Reference parity for impact timing.  The legacy engine resolves an impact
 * at the exact event time: cores are synchronized to that time (so the pair
 * is at coll_dist, not penetrating), pushed, and then re-integrated to the
 * next PSI.  Here impacts are resolved after the PSI integration, when the
 * pair may already have moved v*dt into each other; the rescue-speed term
 * (depth / dt) would then demand a huge separation speed.  Rewind both cores
 * along their own velocity to the moment the gap equalled coll_dist, resolve
 * there, and replay the remaining time with the post-impact velocity. */
static ivp_real_t impact_rewind_time(const ivp_contact_point_t *cp,
                                     ivp_real_t coll_dist,
                                     ivp_real_t closing_speed,
                                     ivp_real_t step_dt)
{
    if (closing_speed <= IVP_IMPACT_EPS) return IVP_REAL(0.0);
    if (cp->gap_length >= coll_dist) return IVP_REAL(0.0);
    ivp_real_t dt_back = (coll_dist - cp->gap_length) / closing_speed;
    if (dt_back > step_dt) dt_back = step_dt;
    if (dt_back < IVP_REAL(0.0)) dt_back = IVP_REAL(0.0);
    return dt_back;
}

static void impact_shift_core_along_velocity(ivp_core_t *core, ivp_real_t dt)
{
    if (!core || core->physical_unmoveable) return;
    ivp_vec3r_add_scaled(&core->pos_world_f_core_last_psi, &core->speed, dt);
    ivp_vec3r_add_scaled(&core->m_world_f_core_last_psi.pos, &core->speed, dt);
}

static int dispatch_due_mindist_events(ivp_environment_t *env,
                                       const ivp_collision_settings_t *settings,
                                       ivp_time_t now,
                                       ivp_time_t max_time,
                                       ivp_real_t step_dt,
                                       bool *any_impulse_applied)
{
    int resolved = 0;
    ivp_time_event_t event;

    ivp_impact_system_t impact_sys;
    ivp_impact_system_init(&impact_sys, env->allocator);

    while (ivp_time_event_queue_pop_due(&env->time_event_queue, max_time, &event)) {
        ivp_object_t *obj_a = ivp_environment_get_object_by_handle(env, event.obj_a);
        ivp_object_t *obj_b = ivp_environment_get_object_by_handle(env, event.obj_b);
        if (!obj_a || !obj_b) continue;

        ivp_mindist_entry_t *entry =
            ivp_mindist_manager_get_or_create(&env->mindist_manager, obj_a, obj_b);
        if (entry) {
            entry->event_registered = false;
            entry->event_time = now;
        }

        /* ── RECALC events (sector-change / topology) ──
         * Reference: IVP_Mindist::simulate_time_event — when
         * (coll_type & 0x0f) != 0, just recalc and reschedule.
         * No narrowphase, no impact resolution.
         * Note: rescheduling is deferred to the broadphase in the next
         * collision resolution call to avoid infinite dispatch loops. */
        /* Important: follow-up events must be scheduled strictly AFTER the
         * current dispatch horizon (`max_time`), otherwise pop_due(max_time)
         * can repeatedly consume newly requeued events in the same pass. */
        const ivp_time_t small_delay_end = max_time + (ivp_time_t)step_dt;
        const ivp_time_t big_delay_end = max_time + (ivp_time_t)(step_dt * IVP_REAL(4.0));

        if (!time_event_is_impact(&event)) {
            if (entry) {
                apply_topology_kind_to_synapse(entry,
                                               event.mindist_kind,
                                               event.status_a,
                                               event.status_b);
                ivp_mindist_entry_recompute(entry);
                if (topology_event_should_tighten(event.mindist_kind)) {
                    update_coll_dist_selector(entry);
                }
                schedule_followup_mindist_event(env, settings, entry, max_time, small_delay_end);
            }
            continue;
        }

        /* ── IMPACT events ──
         * Reference: IVP_Mindist::simulate_time_event — when
         * (coll_type & 0x0f) == 0:
         *   1. recalc_mindist() to get current distance
         *   2. If distance < get_coll_dist() + mindist_change_force_dist → impact
         *   3. Otherwise → false alarm, reschedule with big delay */

        /* Step 1: recompute to get the actual current situation */
        if (entry) {
            ivp_mindist_entry_recompute(entry);
            update_coll_dist_selector(entry);
        }

        /* Step 2: false-alarm check using reference-like secondary distance:
         * sec_dist = coll_dist + mindist_change_force_dist.
         * If current distance exceeds sec_dist, schedule a big-delay recheck. */
        {
            ivp_real_t coll_dist = entry
                ? get_entry_coll_dist(entry, settings)
                : settings->min_coll_dist;
            ivp_real_t sec_dist = coll_dist + settings->mindist_change_force_dist;
            bool false_alarm = entry && entry->last_distance > sec_dist;
            bool approaching = false;
            if (false_alarm && entry) {
                ivp_vec3r_t rel_speed;
                ivp_vec3r_sub(&rel_speed,
                              &entry->obj_b->core.speed,
                              &entry->obj_a->core.speed);
                ivp_real_t closing_estimate =
                    -ivp_vec3r_dot(&rel_speed, &entry->last_normal);
                approaching = closing_estimate > IVP_IMPACT_EPS;
                if (closing_estimate > IVP_IMPACT_EPS) {
                    ivp_real_t projected_gap =
                        entry->last_distance - closing_estimate * step_dt;
                    if (projected_gap <= coll_dist) {
                        false_alarm = false;
                    }
                }
            }
            if (false_alarm) {
                schedule_followup_mindist_event(
                    env, settings, entry, max_time,
                    approaching ? small_delay_end : big_delay_end);
                continue;
            }
        }

        /* Additional coarse far-separation guard. */
        if (entry && entry->last_distance > settings->max_distance_for_impact) {
            schedule_followup_mindist_event(env, settings, entry, max_time, big_delay_end);
            continue;
        }

        bool any_need_impact = false;
        bool any_impact_resolved = false;
        int impact_passes = 0;
        const bool pair_has_ball =
            obj_a->type == IVP_OBJECT_BALL || obj_b->type == IVP_OBJECT_BALL;
        const int max_impact_passes =
            pair_has_ball ? IVP_MAX_CONTACTS_PER_PAIR : 1;

        /* Reference-like event-driven loop:
         * resolve one most urgent contact, refresh narrowphase, repeat. */
        while (impact_passes < max_impact_passes) {
            ivp_contact_point_t generated[IVP_MAX_CONTACTS_PER_PAIR];
            int n_generated = ivp_contact_compute_generic_multi(obj_a, obj_b,
                                                                generated,
                                                                IVP_MAX_CONTACTS_PER_PAIR);
            if (n_generated <= 0) {
                if (entry && !any_need_impact) {
                    schedule_followup_mindist_event(env, settings, entry, max_time, big_delay_end);
                }
                break;
            }

            ivp_persistent_contact_t *impact_candidate_pc = NULL;
            ivp_real_t impact_candidate_metric = IVP_REAL(1e30);

            for (int ci = 0; ci < n_generated; ci++) {
                ivp_contact_point_t *cp = &generated[ci];
                ivp_real_t closing_speed = ivp_contact_closing_speed(cp);
                bool need_impact = false;

                ivp_real_t coll_dist = entry
                    ? get_entry_coll_dist(entry, settings)
                    : settings->min_coll_dist;

                if (cp->gap_length <= coll_dist) {
                    /* Keep resting contacts out of impact path unless they are
                     * still approaching or already penetrating. */
                    need_impact = (closing_speed > IVP_IMPACT_EPS) ||
                                  (cp->gap_length < IVP_REAL(0.0));
                } else if (closing_speed > IVP_IMPACT_EPS &&
                           cp->gap_length <= settings->max_distance_for_impact) {
                    const ivp_real_t projected_gap =
                        cp->gap_length - closing_speed * step_dt;
                    need_impact = projected_gap <= coll_dist;
                }

                ivp_persistent_contact_t *pc = NULL;
                if (cp->gap_length <= settings->keeper_dist || need_impact) {
                    pc = ivp_friction_manager_match_or_create(&env->friction_manager, cp);
                }

                if (!need_impact) {
                    if (pc) pc->last_gap_len = cp->gap_length;
                    continue;
                }

                any_need_impact = true;
                if (!pc) continue;

                ivp_real_t candidate_metric = cp->gap_length - closing_speed * step_dt;
                if (!impact_candidate_pc || candidate_metric < impact_candidate_metric) {
                    impact_candidate_metric = candidate_metric;
                    impact_candidate_pc = pc;
                }
            }

            if (!impact_candidate_pc) {
                break;
            }

            ivp_contact_point_t *cp = &impact_candidate_pc->cp;
            ivp_persistent_contact_t *pc = impact_candidate_pc;

            ivp_environment_emit_event_internal(env, IVP_EVENT_COLLISION_DETECTED,
                                                step_dt, cp->obj_a->dense_index, 0);

            ivp_real_t closing_speed = ivp_contact_closing_speed(cp);
            ivp_collision_event_t pre_event =
                ivp_make_collision_event(cp, closing_speed, IVP_REAL(0.0));
            ivp_collision_listeners_fire_pre(cp->obj_a, cp->obj_b, &pre_event);

            /* Rewind to the moment of first contact (see impact_rewind_time). */
            const ivp_real_t rewind_coll_dist = entry
                ? get_entry_coll_dist(entry, settings)
                : settings->min_coll_dist;
            const ivp_real_t dt_back =
                impact_rewind_time(cp, rewind_coll_dist, closing_speed, step_dt);
            if (dt_back > IVP_REAL(0.0)) {
                impact_shift_core_along_velocity(&cp->obj_a->core, -dt_back);
                impact_shift_core_along_velocity(&cp->obj_b->core, -dt_back);
                cp->gap_length += closing_speed * dt_back;
            }
            if (cp->gap_length < rewind_coll_dist) {
                ivp_contact_project_out_of_penetration(cp, rewind_coll_dist);
            }

            ivp_real_t impact_impulse = IVP_REAL(0.0);
            const bool involves_ball =
                cp->obj_a->type == IVP_OBJECT_BALL ||
                cp->obj_b->type == IVP_OBJECT_BALL;
            /* Reference: every polygon impact runs through IVP_Impact_System so
             * that pushes propagate through touching neighbours (stacks) instead
             * of being resolved pairwise with rescue pushes. */
            const bool use_system_solver = !involves_ball;

            if (use_system_solver) {
                ivp_core_t *core_a = &cp->obj_a->core;
                ivp_core_t *core_b = &cp->obj_b->core;
                ivp_friction_system_t *fs =
                    ivp_friction_manager_find_system_for_core(&env->friction_manager, core_a);
                ivp_friction_core_pair_t *pair = find_pair_for_cores(fs, core_a, core_b);

                ivp_impact_system_solve(&impact_sys, pc, fs, pair, settings);
                impact_impulse = impact_sys.start_contact_impulse_sum;
            } else {
                ivp_real_t primary_impulse = ivp_impact_resolve(&pc->cp, settings);
                if (primary_impulse > IVP_IMPACT_EPS) {
                    impact_impulse += primary_impulse;
                }
            }

            /* Replay the rewound time with the post-impact velocities. */
            if (dt_back > IVP_REAL(0.0)) {
                impact_shift_core_along_velocity(&cp->obj_a->core, dt_back);
                impact_shift_core_along_velocity(&cp->obj_b->core, dt_back);
                cp->gap_length -= ivp_contact_closing_speed(cp) * dt_back;
            }

            if (impact_impulse > IVP_IMPACT_EPS) {
                *any_impulse_applied = true;
                pc->impact_resolved = true;
                any_impact_resolved = true;

                /* Reference: recalc_all_affected_cores re-integrates every
                 * pushed core so the next PSI moves with the post-impact
                 * velocity instead of the stale pre-impact delta. */
                ivp_core_resync_after_impulse(&cp->obj_a->core);
                ivp_core_resync_after_impulse(&cp->obj_b->core);
            }

            ivp_real_t cs_now = ivp_contact_closing_speed(cp);
            pc->now_friction_pressure = cs_now > IVP_REAL(0.0)
                ? cs_now * cp->virtual_mass
                : IVP_REAL(0.0);
            pc->last_gap_len = cp->gap_length;

            ivp_collision_event_t post_event =
                ivp_make_collision_event(cp, ivp_contact_closing_speed(cp), impact_impulse);
            ivp_collision_listeners_fire_post(cp->obj_a, cp->obj_b, &post_event);
            resolved++;
            impact_passes++;
        }

        /* Recompute entry after impact for subsequent broadphase tracking */
        if (entry) {
            ivp_mindist_entry_recompute(entry);
            if (!any_need_impact) {
                schedule_followup_mindist_event(env, settings, entry, max_time, big_delay_end);
            } else if (!any_impact_resolved) {
                schedule_followup_mindist_event(env, settings, entry, max_time, small_delay_end);
            }
        }
    }

    ivp_impact_system_destroy(&impact_sys);
    return resolved;
}

int ivp_environment_resolve_collisions(ivp_environment_t *env)
{
    if (ivp_environment_get_object_count(env) < 2) {
        ivp_friction_manager_age_and_prune(&env->friction_manager);
        return 0;
    }

    const ivp_real_t step_dt = ivp_environment_get_effective_step_dt(env);
    const ivp_time_t psi_now = env->current_time +
        (env->current_sim_unit_index >= 0
             ? (ivp_time_t)((ivp_real_t)env->current_sim_unit_index * step_dt)
             : (ivp_time_t)0.0);
    const ivp_time_t psi_end_time = psi_now + (ivp_time_t)step_dt;

    /* Initialize collision settings */
    ivp_collision_settings_t coll_settings;
    ivp_collision_settings_init_default(&coll_settings);
    coll_settings.gravity_magnitude = ivp_vec3r_length(&env->gravity);
    coll_settings.inv_delta_psi_time = (step_dt > IVP_REAL_EPS)
        ? (IVP_REAL(1.0) / step_dt) : IVP_REAL(0.0);

    ivp_mindist_manager_age_entries(&env->mindist_manager);
    if (ivp_hull_manager_update(&env->hull_manager, step_dt) > 0) {
        ivp_object_t *expanded_local[64];
        int expanded_count = ivp_hull_manager_collect_expanded_objects(
            &env->hull_manager,
            expanded_local,
            (int)(sizeof(expanded_local) / sizeof(expanded_local[0])));

        if (expanded_count > (int)(sizeof(expanded_local) / sizeof(expanded_local[0]))) {
            ivp_mindist_manager_invalidate_all(&env->mindist_manager);
        } else {
            for (int ei = 0; ei < expanded_count; ei++) {
                ivp_mindist_manager_invalidate_object(&env->mindist_manager,
                                                      expanded_local[ei]);
            }
        }
    }

    /* Broadphase: detect all overlapping pairs.
     * Use sweep-and-prune for larger scenes, brute-force for small. */
    ivp_contact_point_t contacts[IVP_MAX_CONTACT_PAIRS];
    int n_contacts;
    if (ivp_environment_get_object_count(env) >= IVP_SAP_THRESHOLD) {
        n_contacts = ivp_broadphase_sap_detect(&env->broadphase_sap, env,
                                                &coll_settings,
                                                contacts, IVP_MAX_CONTACT_PAIRS);
    } else {
        n_contacts = ivp_broadphase_detect(env, &coll_settings,
                                            contacts, IVP_MAX_CONTACT_PAIRS);
    }

    bool any_impulse_applied = false;
    int resolved = 0;

    /* CCD contributes event-time impacts by inserting TOI events. */
    if (env->ccd_enabled) {
        for (int i = 0; i < n_contacts; i++) {
            ivp_contact_point_t *cp = &contacts[i];
            if (ivp_ccd_pair_needs_sweep(cp->obj_a, cp->obj_b, step_dt)) {
                ivp_ccd_hit_t hit;
                if (ivp_ccd_sweep_pair(cp->obj_a, cp->obj_b,
                                        psi_now,
                                        step_dt, &hit)) {
                    if (hit.valid) {
                        ivp_time_event_t evt;
                        memset(&evt, 0, sizeof(evt));
                        evt.type = IVP_TIME_EVENT_MINDIST_IMPACT;
                        evt.mindist_kind = (uint32_t)IVP_MINDIST_EVENT_BALL_COLL;
                        evt.trigger_time = psi_now + (ivp_time_t)hit.toi;
                        evt.obj_a.slot = cp->obj_a->handle.slot;
                        evt.obj_a.generation = cp->obj_a->handle.generation;
                        evt.obj_b.slot = cp->obj_b->handle.slot;
                        evt.obj_b.generation = cp->obj_b->handle.generation;
                        evt.status_a = cp->obj_a->type == IVP_OBJECT_BALL ? IVP_ST_BALL : IVP_ST_TRIANGLE;
                        evt.status_b = cp->obj_b->type == IVP_OBJECT_BALL ? IVP_ST_BALL : IVP_ST_TRIANGLE;
                        evt.expected_gap = coll_settings.min_coll_dist;
                        (void)ivp_time_event_queue_push(&env->time_event_queue, &evt);
                    }
                }
            }
        }
    }

    for (int i = 0; i < n_contacts; i++) {
        ivp_contact_point_t *cp = &contacts[i];

        bool allow_a = ivp_anomaly_manager_count_collision(&env->anomaly_manager, cp->obj_a);
        bool allow_b = ivp_anomaly_manager_count_collision(&env->anomaly_manager, cp->obj_b);
        if (!allow_a || !allow_b) continue;

        /* B5: Record penetration metrics */
        {
            ivp_solver_metrics_t *met = ivp_solver_metrics_get_active();
            if (met) {
                met->contact_count++;
                if (cp->gap_length < IVP_REAL(0.0)) {
                    ivp_real_t depth = -cp->gap_length;
                    if (depth > met->max_penetration_depth) {
                        met->max_penetration_depth = depth;
                    }
                    met->avg_penetration_depth += depth;
                    met->penetration_count++;
                }
            }
        }

        ivp_mindist_entry_t *entry =
            ivp_mindist_manager_get_or_create(&env->mindist_manager, cp->obj_a, cp->obj_b);
        if (!entry) continue;

        /* The entry stores its data relative to (entry->obj_a, entry->obj_b),
         * which may be the reverse of the contact point's (obj_a, obj_b). */
        const bool swapped = entry->obj_a != cp->obj_a;
        entry->last_distance = cp->gap_length;
        if (swapped) {
            ivp_vec3r_scale(&entry->last_normal, &cp->normal, IVP_REAL(-1.0));
            entry->closest_point_a = cp->point_b;
            entry->closest_point_b = cp->point_a;
        } else {
            entry->last_normal = cp->normal;
            entry->closest_point_a = cp->point_a;
            entry->closest_point_b = cp->point_b;
        }
        entry->state = cp->gap_length <= coll_settings.max_distance_for_impact
            ? IVP_MDIST_EXACT
            : IVP_MDIST_HULL;
        entry->synapse[0].status = entry->obj_a->type == IVP_OBJECT_BALL ? IVP_ST_BALL : IVP_ST_TRIANGLE;
        entry->synapse[1].status = entry->obj_b->type == IVP_OBJECT_BALL ? IVP_ST_BALL : IVP_ST_TRIANGLE;

        if (cp->gap_length <= coll_settings.keeper_dist) {
            ivp_persistent_contact_t *pc =
                ivp_friction_manager_match_or_create(&env->friction_manager, cp);
            if (pc) {
                pc->last_gap_len = cp->gap_length;
            }
        }

        {
            ivp_real_t coll_dist = get_entry_coll_dist(entry, &coll_settings);
            ivp_real_t closing_speed = ivp_contact_closing_speed(cp);
            bool immediate_impact = false;
            const bool has_ball =
                cp->obj_a->type == IVP_OBJECT_BALL ||
                cp->obj_b->type == IVP_OBJECT_BALL;

            if (cp->gap_length <= coll_dist) {
                immediate_impact = true;
            } else if (has_ball &&
                       closing_speed > IVP_IMPACT_EPS &&
                       cp->gap_length <= coll_settings.max_distance_for_impact) {
                const ivp_real_t projected_gap =
                    cp->gap_length - closing_speed * step_dt;
                immediate_impact = projected_gap <= coll_dist;
            }

            if (immediate_impact) {
                ivp_time_event_t time_event;
                memset(&time_event, 0, sizeof(time_event));
                time_event.trigger_time = psi_end_time;
                time_event.type = IVP_TIME_EVENT_MINDIST_IMPACT;
                time_event.mindist_kind = (uint32_t)impact_kind_from_status(
                    entry->synapse[0].status,
                    entry->synapse[1].status);
                time_event.obj_a = entry->obj_a->handle;
                time_event.obj_b = entry->obj_b->handle;
                time_event.status_a = entry->synapse[0].status;
                time_event.status_b = entry->synapse[1].status;
                time_event.expected_gap = coll_dist;
                if (ivp_time_event_queue_push(&env->time_event_queue, &time_event)) {
                    entry->event_registered = true;
                    entry->event_time = time_event.trigger_time;
                }
                continue;
            }
        }

        ivp_mindist_event_prediction_t prediction;
        if (!ivp_mindist_event_predict(entry, &coll_settings,
                                       psi_now, psi_end_time, &prediction)) {
            continue;
        }

        ivp_time_event_t time_event;
        ivp_mindist_event_to_time_event(entry, &prediction, &time_event);
        if (ivp_time_event_queue_push(&env->time_event_queue, &time_event)) {
            entry->event_registered = true;
            entry->event_time = prediction.event_time;
        }
    }

    resolved += dispatch_due_mindist_events(env, &coll_settings, psi_now, psi_end_time,
                                            step_dt, &any_impulse_applied);

    /* Age and prune (coupled solver, easing, energy removal now run
     * in ivp_environment_run_friction_phase before controllers). */
    ivp_friction_manager_age_and_prune(&env->friction_manager);

    if (any_impulse_applied) {
        wake_objects_after_impulse(env, step_dt);
    }

    return resolved;
}

/* ── Broadphase ─────────────────────────────────────────────────────────── */

int ivp_broadphase_detect(ivp_environment_t *env,
                          const ivp_collision_settings_t *coll_settings,
                          ivp_contact_point_t *contacts,
                          int max_contacts)
{
    int count = 0;
    int n = ivp_environment_get_object_count(env);

    /* Initialize range manager with environment's delta PSI time */
    ivp_range_manager_t rm;
    ivp_range_manager_init(&rm, ivp_environment_get_effective_step_dt(env));

    for (int i = 0; i < n && count < max_contacts; i++) {
        ivp_object_t *obj_a = ivp_environment_get_object(env, i);
        if (!obj_a) continue;

        for (int j = i + 1; j < n && count < max_contacts; j++) {
            ivp_object_t *obj_b = ivp_environment_get_object(env, j);
            if (!obj_b) continue;

            /* Skip static-static pairs */
            if (obj_a->core.physical_unmoveable && obj_b->core.physical_unmoveable) {
                continue;
            }

            /* Skip pairs explicitly assigned to the same no-collision group (legacy) */
            if (ivp_objects_share_no_collision_group(obj_a, obj_b)) {
                continue;
            }

            /* Collision filter check (new system) */
            if (!ivp_collision_filter_check(&env->collision_filter, obj_a, obj_b)) {
                continue;
            }

            /* Adaptive margin from range manager (speed-based) */
            ivp_real_t pair_margin = ivp_range_manager_get_pair_margin(
                &rm, &obj_a->core, &obj_b->core);

            /* Use the larger of range-manager margin and collision tolerance */
            ivp_real_t margin = pair_margin > coll_settings->max_distance_for_impact
                              ? pair_margin
                              : coll_settings->max_distance_for_impact;

            /* Bounding sphere overlap check */
            const ivp_vec3r_t *pa = &obj_a->core.pos_world_f_core_last_psi;
            const ivp_vec3r_t *pb = &obj_b->core.pos_world_f_core_last_psi;
            ivp_real_t ra = obj_a->core.upper_limit_radius;
            ivp_real_t rb = obj_b->core.upper_limit_radius;

            ivp_vec3r_t diff;
            ivp_vec3r_sub(&diff, pb, pa);
            ivp_real_t qdist = ivp_vec3r_quad_length(&diff);
            ivp_real_t threshold = ra + rb + margin;

            if (qdist < threshold * threshold) {
                int free_slots = max_contacts - count;
                if (free_slots <= 0) break;

                int produced = ivp_contact_compute_generic_multi(
                    obj_a, obj_b, &contacts[count], free_slots);

                int valid = 0;
                for (int k = 0; k < produced; k++) {
                    ivp_contact_point_t *cp = &contacts[count + k];

                    /* Ignore invalid/non-contact results from narrowphase.
                     * This filters backside/degenerate outcomes before entering
                     * impact/friction processing. */
                    if (!isfinite(cp->gap_length)) {
                        continue;
                    }

                    if (obj_a->type == IVP_OBJECT_POLYGON &&
                        obj_b->type == IVP_OBJECT_POLYGON &&
                        cp->gap_length > margin) {
                        continue;
                    }

                    if (valid != k) {
                        contacts[count + valid] = *cp;
                    }
                    valid++;
                }

                count += valid;
            }
        }
    }

    return count;
}

/* ── Energy queries ─────────────────────────────────────────────────────── */

ivp_real_t ivp_object_get_kinetic_energy(const ivp_object_t *obj)
{
    const ivp_core_t *c = &obj->core;
    if (c->physical_unmoveable) return IVP_REAL(0.0);

    /* Translational: 0.5 * m * v² */
    ivp_real_t v2 = ivp_vec3r_quad_length(&c->speed);
    ivp_real_t ke_trans = IVP_REAL(0.5) * c->mass * v2;

    /* Rotational: 0.5 * ω · (I * ω) [core space, diagonal I] */
    ivp_real_t ke_rot = IVP_REAL(0.5) * (
        c->rot_inertia.k[0] * (ivp_real_t)c->rot_speed.k[0] * (ivp_real_t)c->rot_speed.k[0] +
        c->rot_inertia.k[1] * (ivp_real_t)c->rot_speed.k[1] * (ivp_real_t)c->rot_speed.k[1] +
        c->rot_inertia.k[2] * (ivp_real_t)c->rot_speed.k[2] * (ivp_real_t)c->rot_speed.k[2]
    );

    return ke_trans + ke_rot;
}

ivp_real_t ivp_environment_get_total_kinetic_energy(const ivp_environment_t *env)
{
    ivp_real_t total = IVP_REAL(0.0);
    const int n = ivp_environment_get_object_count(env);
    for (int i = 0; i < n; i++) {
        ivp_object_t *obj = ivp_environment_get_object(env, i);
        if (obj) {
            total += ivp_object_get_kinetic_energy(obj);
        }
    }
    return total;
}
