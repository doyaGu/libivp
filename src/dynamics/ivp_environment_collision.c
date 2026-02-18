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

        obj->core.movement_state = IVP_MT_SLOW;
        obj->core.movement_check_counter = 0;
        ivp_environment_emit_event_internal(env, IVP_EVENT_OBJECT_UNFROZEN,
                                            step_dt, obj->dense_index, 0);
    }
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

        ivp_contact_point_t generated[IVP_MAX_CONTACTS_PER_PAIR];
        int n_generated = ivp_contact_compute_generic_multi(obj_a, obj_b,
                                                            generated,
                                                            IVP_MAX_CONTACTS_PER_PAIR);
        if (n_generated <= 0) continue;

        for (int ci = 0; ci < n_generated; ci++) {
            ivp_contact_point_t *cp = &generated[ci];
            ivp_real_t closing_speed = ivp_contact_closing_speed(cp);
            bool need_impact = false;

            if (closing_speed > IVP_IMPACT_EPS &&
                cp->gap_length <= settings->max_distance_for_impact) {
                if (cp->gap_length <= settings->min_coll_dist) {
                    need_impact = true;
                } else {
                    const ivp_real_t projected_gap =
                        cp->gap_length - closing_speed * step_dt;
                    need_impact = projected_gap <= settings->min_coll_dist;
                }
            }

            ivp_persistent_contact_t *pc = NULL;
            if (cp->gap_length <= settings->keeper_dist || need_impact) {
                pc = ivp_friction_manager_match_or_create(&env->friction_manager, cp);
            }

            if (!need_impact) {
                if (pc) pc->last_gap_len = cp->gap_length;
                continue;
            }

            if (!pc) {
                /* Hard switch: event-time impacts are anchored on persistent
                 * synapse contacts only; no immediate single-contact fallback. */
                continue;
            }

            ivp_environment_emit_event_internal(env, IVP_EVENT_COLLISION_DETECTED,
                                                step_dt, cp->obj_a->dense_index, 0);

            ivp_collision_event_t pre_event =
                ivp_make_collision_event(cp, closing_speed, IVP_REAL(0.0));
            ivp_collision_listeners_fire_pre(cp->obj_a, cp->obj_b, &pre_event);

            ivp_real_t impact_impulse = IVP_REAL(0.0);
            ivp_core_t *core_a = &cp->obj_a->core;
            ivp_core_t *core_b = &cp->obj_b->core;
            ivp_friction_system_t *fs =
                ivp_friction_manager_find_system_for_core(&env->friction_manager, core_a);
            ivp_friction_core_pair_t *pair = find_pair_for_cores(fs, core_a, core_b);

            ivp_impact_system_solve(&impact_sys, pc, fs, pair, settings);
            impact_impulse = impact_sys.start_contact_impulse_sum;

            if (impact_sys.sum_of_pushes > 0) {
                *any_impulse_applied = true;
                pc->impact_resolved = true;
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
        }

        ivp_mindist_entry_t *entry =
            ivp_mindist_manager_get_or_create(&env->mindist_manager, obj_a, obj_b);
        if (entry) {
            entry->event_registered = false;
            entry->event_time = now;
            ivp_mindist_entry_recompute(entry);
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
    const ivp_time_t now = env->current_time +
        (env->current_sim_unit_index >= 0
             ? (ivp_time_t)((ivp_real_t)env->current_sim_unit_index * step_dt)
             : (ivp_time_t)0.0);
    const ivp_time_t max_time = now + (ivp_time_t)step_dt;

    /* Initialize collision settings */
    ivp_collision_settings_t coll_settings;
    ivp_collision_settings_init_default(&coll_settings);
    coll_settings.gravity_magnitude = ivp_vec3r_length(&env->gravity);
    coll_settings.inv_delta_psi_time = (step_dt > IVP_REAL_EPS)
        ? (IVP_REAL(1.0) / step_dt) : IVP_REAL(0.0);

    ivp_mindist_manager_age_entries(&env->mindist_manager);
    if (ivp_hull_manager_update(&env->hull_manager, step_dt) > 0) {
        ivp_mindist_manager_invalidate_all(&env->mindist_manager);
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
                                        now,
                                        step_dt, &hit)) {
                    if (hit.valid) {
                        ivp_time_event_t evt;
                        memset(&evt, 0, sizeof(evt));
                        evt.type = IVP_TIME_EVENT_MINDIST_IMPACT;
                        evt.trigger_time = now + (ivp_time_t)hit.toi;
                        evt.obj_a.slot = cp->obj_a->handle_slot;
                        evt.obj_a.generation = cp->obj_a->handle_generation;
                        evt.obj_b.slot = cp->obj_b->handle_slot;
                        evt.obj_b.generation = cp->obj_b->handle_generation;
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

        entry->last_distance = cp->gap_length;
        entry->last_normal = cp->normal;
        entry->closest_point_a = cp->point_a;
        entry->closest_point_b = cp->point_b;
        entry->state = cp->gap_length <= coll_settings.max_distance_for_impact
            ? IVP_MDIST_EXACT
            : IVP_MDIST_HULL;
        entry->synapse[0].status = cp->obj_a->type == IVP_OBJECT_BALL ? IVP_ST_BALL : IVP_ST_TRIANGLE;
        entry->synapse[1].status = cp->obj_b->type == IVP_OBJECT_BALL ? IVP_ST_BALL : IVP_ST_TRIANGLE;

        if (cp->gap_length <= coll_settings.keeper_dist) {
            ivp_persistent_contact_t *pc =
                ivp_friction_manager_match_or_create(&env->friction_manager, cp);
            if (pc) {
                pc->last_gap_len = cp->gap_length;
            }
        }

        ivp_mindist_event_prediction_t prediction;
        if (!ivp_mindist_event_predict(entry, &coll_settings, now, max_time, &prediction)) {
            continue;
        }

        ivp_time_event_t time_event;
        ivp_mindist_event_to_time_event(entry, &prediction, &time_event);
        if (ivp_time_event_queue_push(&env->time_event_queue, &time_event)) {
            entry->event_registered = true;
            entry->event_time = prediction.event_time;
        }
    }

    resolved += dispatch_due_mindist_events(env, &coll_settings, now, max_time,
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

            /* Skip objects in different sim units */
            if (obj_a->sim_unit_id != obj_b->sim_unit_id) {
                continue;
            }

            /* Skip pairs explicitly assigned to the same no-collision group (legacy) */
            if (obj_a->no_collision_group && obj_b->no_collision_group &&
                obj_a->no_collision_group == obj_b->no_collision_group) {
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
