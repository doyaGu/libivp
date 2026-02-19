/* ivp_sim_unit.c — Dynamic simulation island implementation
 *
 * Aligned with reference IVP_Simulation_Unit / IVP_Sim_Units_Manager
 * (ivp_sim_unit.hxx / ivp_sim_unit.cxx).
 */
#include <ivp/dynamics/ivp_sim_unit.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/base/ivp_assert.h>
#include <string.h>

/* ── Internal helpers ──────────────────────────────────────────────────── */

static ivp_sim_unit_core_entry_t *alloc_core_entry(ivp_sim_units_manager_t *mgr)
{
    ivp_sim_unit_core_entry_t *e = (ivp_sim_unit_core_entry_t *)ivp_alloc(
        mgr->allocator, sizeof(ivp_sim_unit_core_entry_t));
    if (e) memset(e, 0, sizeof(*e));
    return e;
}

static void free_core_entry(ivp_sim_units_manager_t *mgr, ivp_sim_unit_core_entry_t *e)
{
    ivp_free(mgr->allocator, e, sizeof(ivp_sim_unit_core_entry_t));
}

/* Unlink a sim unit from the manager's active list. */
static void unlink_from_active(ivp_sim_units_manager_t *mgr, ivp_sim_unit_t *su)
{
    if (su->prev_unit) {
        su->prev_unit->next_unit = su->next_unit;
    } else {
        mgr->active_units = su->next_unit;
    }
    if (su->next_unit) {
        su->next_unit->prev_unit = su->prev_unit;
    }
    su->next_unit = NULL;
    su->prev_unit = NULL;
    mgr->active_count--;
}

/* Link a sim unit into the manager's active list (at head). */
static void link_to_active(ivp_sim_units_manager_t *mgr, ivp_sim_unit_t *su)
{
    su->prev_unit = NULL;
    su->next_unit = mgr->active_units;
    if (mgr->active_units) {
        mgr->active_units->prev_unit = su;
    }
    mgr->active_units = su;
    mgr->active_count++;
}

/* ── Manager lifecycle ─────────────────────────────────────────────────── */

ivp_sim_units_manager_t *ivp_sim_units_manager_create(ivp_environment_t *env,
                                                       const ivp_allocator_t *alloc)
{
    if (!alloc) alloc = ivp_default_allocator();

    ivp_sim_units_manager_t *mgr = (ivp_sim_units_manager_t *)ivp_alloc(
        alloc, sizeof(ivp_sim_units_manager_t));
    if (!mgr) return NULL;

    memset(mgr, 0, sizeof(*mgr));
    mgr->env = env;
    mgr->allocator = alloc;
    mgr->active_units = NULL;
    mgr->active_count = 0;

    /* Initialize still_slot as a pseudo-unit for frozen cores */
    memset(&mgr->still_slot, 0, sizeof(mgr->still_slot));
    mgr->still_slot.movement_type = IVP_SUM_STILL;
    mgr->still_slot.manager = mgr;

    return mgr;
}

void ivp_sim_units_manager_destroy(ivp_sim_units_manager_t *mgr)
{
    if (!mgr) return;

    /* Free all active units and their core entries */
    ivp_sim_unit_t *su = mgr->active_units;
    while (su) {
        ivp_sim_unit_t *next = su->next_unit;

        /* Free core entries */
        ivp_sim_unit_core_entry_t *ce = su->core_list;
        while (ce) {
            ivp_sim_unit_core_entry_t *cn = ce->next;
            ce->core->sim_unit = NULL;
            ce->core->sim_unit_entry = NULL;
            free_core_entry(mgr, ce);
            ce = cn;
        }

        ivp_free(mgr->allocator, su, sizeof(ivp_sim_unit_t));
        su = next;
    }

    /* Free still_slot core entries */
    {
        ivp_sim_unit_core_entry_t *ce = mgr->still_slot.core_list;
        while (ce) {
            ivp_sim_unit_core_entry_t *cn = ce->next;
            ce->core->sim_unit = NULL;
            ce->core->sim_unit_entry = NULL;
            free_core_entry(mgr, ce);
            ce = cn;
        }
        mgr->still_slot.core_list = NULL;
    }

    const ivp_allocator_t *alloc = mgr->allocator;
    ivp_free(alloc, mgr, sizeof(ivp_sim_units_manager_t));
}

/* ── Sim unit lifecycle ────────────────────────────────────────────────── */

ivp_sim_unit_t *ivp_sim_unit_create(ivp_sim_units_manager_t *mgr)
{
    ivp_sim_unit_t *su = (ivp_sim_unit_t *)ivp_alloc(
        mgr->allocator, sizeof(ivp_sim_unit_t));
    if (!su) return NULL;

    memset(su, 0, sizeof(*su));
    su->movement_type = IVP_SUM_MOVING;
    su->manager = mgr;

    link_to_active(mgr, su);
    return su;
}

void ivp_sim_unit_destroy(ivp_sim_units_manager_t *mgr, ivp_sim_unit_t *su)
{
    if (!su || su == &mgr->still_slot) return;
    IVP_ASSERT(su->core_count == 0);
    IVP_ASSERT(su->controller_count == 0);

    unlink_from_active(mgr, su);
    ivp_free(mgr->allocator, su, sizeof(ivp_sim_unit_t));
}

/* ── Core management ───────────────────────────────────────────────────── */

void ivp_sim_unit_add_core(ivp_sim_unit_t *su, ivp_core_t *core)
{
    IVP_ASSERT(su && core);

    /* If already in a different unit, remove first */
    if (core->sim_unit && core->sim_unit != su) {
        ivp_sim_unit_remove_core(su->manager, core);
    }

    /* Already in this unit? */
    if (core->sim_unit == su) return;

    ivp_sim_unit_core_entry_t *entry = alloc_core_entry(su->manager);
    if (!entry) return;

    entry->core = core;
    entry->next = su->core_list;
    entry->prev = NULL;
    if (su->core_list) {
        su->core_list->prev = entry;
    }
    su->core_list = entry;
    su->core_count++;

    core->sim_unit = su;
    core->sim_unit_entry = entry;
}

void ivp_sim_unit_remove_core(ivp_sim_units_manager_t *mgr, ivp_core_t *core)
{
    if (!core || !core->sim_unit) return;

    ivp_sim_unit_t *su = core->sim_unit;
    ivp_sim_unit_core_entry_t *entry = core->sim_unit_entry;
    IVP_ASSERT(entry);

    /* Unlink from DLL */
    if (entry->prev) {
        entry->prev->next = entry->next;
    } else {
        su->core_list = entry->next;
    }
    if (entry->next) {
        entry->next->prev = entry->prev;
    }
    su->core_count--;

    core->sim_unit = NULL;
    core->sim_unit_entry = NULL;
    free_core_entry(mgr, entry);

    /* Destroy unit if empty (and not still_slot) */
    if (su->core_count == 0 && su->controller_count == 0 && su != &mgr->still_slot) {
        ivp_sim_unit_destroy(mgr, su);
    }
}

/* ── Sim unit merge ────────────────────────────────────────────────────── */

ivp_sim_unit_t *ivp_sim_unit_merge(ivp_sim_units_manager_t *mgr,
                                    ivp_sim_unit_t *su_a,
                                    ivp_sim_unit_t *su_b)
{
    if (su_a == su_b) return su_a;
    IVP_ASSERT(su_a && su_b);
    IVP_ASSERT(su_a != &mgr->still_slot && su_b != &mgr->still_slot);

    /* Move all cores from su_b to su_a */
    ivp_sim_unit_core_entry_t *ce = su_b->core_list;
    while (ce) {
        ivp_sim_unit_core_entry_t *next = ce->next;
        ce->core->sim_unit = su_a;

        /* Unlink from su_b */
        ce->prev = NULL;
        ce->next = su_a->core_list;
        if (su_a->core_list) {
            su_a->core_list->prev = ce;
        }
        su_a->core_list = ce;

        ce = next;
    }
    su_a->core_count += su_b->core_count;
    su_b->core_list = NULL;
    su_b->core_count = 0;

    /* Move all controllers from su_b to su_a (re-insert sorted) */
    ivp_controller_core_t *cc = su_b->controller_list;
    while (cc) {
        ivp_controller_core_t *next = cc->next_in_unit;
        cc->next_in_unit = NULL;
        cc->prev_in_unit = NULL;
        cc->sim_unit = NULL;
        su_b->controller_count--;
        ivp_sim_unit_add_controller(su_a, cc);
        cc = next;
    }
    su_b->controller_list = NULL;
    su_b->controller_count = 0;

    /* Destroy the now-empty su_b */
    ivp_sim_unit_destroy(mgr, su_b);

    return su_a;
}

/* ── Controller registration ───────────────────────────────────────────── */

void ivp_sim_unit_add_controller(ivp_sim_unit_t *su, ivp_controller_core_t *cc)
{
    IVP_ASSERT(su && cc);

    /* Remove from previous unit if any */
    if (cc->sim_unit) {
        ivp_sim_unit_remove_controller(cc);
    }

    cc->sim_unit = su;

    /* Insert sorted by priority (ascending) */
    ivp_controller_core_t **pp = &su->controller_list;
    while (*pp && (*pp)->priority <= cc->priority) {
        pp = &(*pp)->next_in_unit;
    }

    cc->next_in_unit = *pp;
    cc->prev_in_unit = NULL;
    if (*pp) {
        cc->prev_in_unit = (*pp)->prev_in_unit;
        (*pp)->prev_in_unit = cc;
    } else {
        /* Inserting at end: find the last element for prev_in_unit */
        ivp_controller_core_t *last = su->controller_list;
        while (last && last->next_in_unit) {
            last = last->next_in_unit;
        }
        cc->prev_in_unit = (last != cc) ? last : NULL;
    }
    if (cc->prev_in_unit) {
        cc->prev_in_unit->next_in_unit = cc;
    } else {
        su->controller_list = cc;
    }

    su->controller_count++;
}

void ivp_sim_unit_remove_controller(ivp_controller_core_t *cc)
{
    if (!cc || !cc->sim_unit) return;

    ivp_sim_unit_t *su = cc->sim_unit;

    if (cc->prev_in_unit) {
        cc->prev_in_unit->next_in_unit = cc->next_in_unit;
    } else {
        su->controller_list = cc->next_in_unit;
    }
    if (cc->next_in_unit) {
        cc->next_in_unit->prev_in_unit = cc->prev_in_unit;
    }

    cc->next_in_unit = NULL;
    cc->prev_in_unit = NULL;
    cc->sim_unit = NULL;

    su->controller_count--;
}

/* ── PSI execution ─────────────────────────────────────────────────────── */

/* Collect core pointers from the unit into a flat array for controller dispatch.
 * Returns count. Caller provides buffer. */
static int collect_cores(ivp_sim_unit_t *su, ivp_core_t **buf, int buf_size)
{
    int n = 0;
    ivp_sim_unit_core_entry_t *ce = su->core_list;
    while (ce && n < buf_size) {
        buf[n++] = ce->core;
        ce = ce->next;
    }
    return n;
}

void ivp_sim_unit_simulate_psi(ivp_sim_unit_t *su,
                                ivp_environment_t *env,
                                ivp_real_t delta_time,
                                ivp_time_t step_end_time)
{
    if (!su || su->core_count == 0) return;

    /* Step 1: Init all cores for PSI + reset cascade fields */
    for (ivp_sim_unit_core_entry_t *ce = su->core_list; ce; ce = ce->next) {
        ivp_core_t *core = ce->core;
        core->temporarily_unmovable = false;
        core->impacts_since_last_PSI = 0;
        ivp_core_init_for_psi_ex(core, delta_time, env->use_forward_projection);
    }

    /* Step 2: Commit all async pushes BEFORE controllers run
     * (reference: ivp_sim_unit.cxx:890,908) */
    for (ivp_sim_unit_core_entry_t *ce = su->core_list; ce; ce = ce->next) {
        ivp_core_commit_async_pushes(ce->core);
    }

    /* Step 3: Run ALL controllers for this unit's cores.
     *
     * Reference ordering (ivp_sim_unit.cxx:907-952):
     *   - Global controllers (from environment controller system) dispatch
     *     only for cores in this unit, in ascending priority order.
     *   - Per-unit controller_core entries dispatch after global controllers
     *     (or interleaved by priority once all controllers are per-unit).
     */
    ivp_environment_run_controllers_for_unit(env, su, delta_time);

    /* Per-unit controllers (registered directly on this sim unit) */
    if (su->controller_count > 0) {
        ivp_core_t *local_cores[64];
        ivp_core_t **cores = local_cores;
        bool cores_allocated = false;
        if (su->core_count > 64) {
            cores = (ivp_core_t **)ivp_alloc(
                su->manager->allocator,
                (size_t)su->core_count * sizeof(ivp_core_t *));
            if (!cores) return;
            cores_allocated = true;
        }
        int core_count = collect_cores(su, cores, su->core_count);

        for (ivp_controller_core_t *cc = su->controller_list; cc; cc = cc->next_in_unit) {
            if (cc->fn) {
                cc->fn(cc->ctx, env, cores, core_count, delta_time);
            }
        }

        if (cores_allocated) {
            ivp_free(su->manager->allocator, cores,
                     (size_t)su->core_count * sizeof(ivp_core_t *));
        }
    }

    /* Step 3.5: Commit async pushes accumulated by controllers.
     * C17 controllers write to speed_change; this merges them into speed
     * before integration reads speed. */
    for (ivp_sim_unit_core_entry_t *ce = su->core_list; ce; ce = ce->next) {
        ivp_core_commit_async_pushes(ce->core);
    }

    /* Step 4: Integration — calc_next_PSI_matrix for all cores
     * (reference: ivp_sim_unit.cxx:953-961) */
    for (ivp_sim_unit_core_entry_t *ce = su->core_list; ce; ce = ce->next) {
        ivp_core_t *core = ce->core;
        ivp_core_calc_next_psi_matrix(core, delta_time);
        core->time_of_last_psi = step_end_time;
    }

    /* Step 5: Movement state / freeze check */
    ivp_sim_unit_calc_movement_state(su, env);

    /* Step 6: Union find if needed (deferred split) */
    if (su->union_find_needed) {
        /* TODO: Implement union_find_for_sim_unit to detect disconnected
         * components and split them into separate units.
         * Reference: ivp_sim_unit.cxx do_sim_unit_union_find() */
        su->union_find_needed = false;
    }
}

void ivp_sim_units_manager_simulate_all_psi(ivp_sim_units_manager_t *mgr,
                                             ivp_real_t delta_time,
                                             ivp_time_t step_end_time)
{
    if (!mgr) return;

    /* Iterate active units. We snapshot the list because simulate_psi
     * may freeze a unit and move it to still_slot. */
    ivp_sim_unit_t *su = mgr->active_units;
    while (su) {
        ivp_sim_unit_t *next = su->next_unit;
        ivp_sim_unit_simulate_psi(su, mgr->env, delta_time, step_end_time);
        su = next;
    }
}

/* ── Freeze / wake ─────────────────────────────────────────────────────── */

void ivp_sim_unit_calc_movement_state(ivp_sim_unit_t *su,
                                       ivp_environment_t *env)
{
    if (!su || su->core_count == 0) return;

    bool all_calm = true;
    bool any_moving = false;

    for (ivp_sim_unit_core_entry_t *ce = su->core_list; ce; ce = ce->next) {
        ivp_core_t *core = ce->core;
        if (core->physical_unmoveable) continue;

        /* Periodic movement check */
        core->movement_check_counter++;
        if (core->movement_check_counter >= IVP_MOVEMENT_CHECK_COUNT) {
            core->movement_check_counter = 0;
            ivp_core_calc_movement_state(core, &env->freeze_params);
        }

        if (core->movement_state == IVP_MT_MOVING || core->movement_state == IVP_MT_SLOW) {
            any_moving = true;
            all_calm = false;
        } else if (core->movement_state != IVP_MT_CALM &&
                   core->movement_state != IVP_MT_NOT_SIM) {
            all_calm = false;
        }
    }

    if (any_moving) {
        su->movement_type = IVP_SUM_MOVING;
    } else if (all_calm) {
        /* All cores are calm — mark the unit as still.
         * Note: We do NOT yet move cores to still_slot here.
         * In the reference IVP, freezing requires sustained calm over multiple
         * checks and integrates with the mindist/controller dependency graph.
         * The still_slot migration will be implemented when the full freeze
         * pipeline (including revive-on-proximity) is aligned.
         * For now, cores remain in the active unit but with movement_type
         * set to STILL, which prevents unnecessary controller work. */
        su->movement_type = IVP_SUM_STILL;
    } else {
        su->movement_type = IVP_SUM_CALM;
    }
}

void ivp_sim_unit_revive_core(ivp_sim_units_manager_t *mgr, ivp_core_t *core)
{
    if (!mgr || !core) return;
    if (core->sim_unit != &mgr->still_slot) return;

    /* Remove from still_slot */
    ivp_sim_unit_core_entry_t *entry = core->sim_unit_entry;
    if (!entry) return;

    if (entry->prev) entry->prev->next = entry->next;
    else mgr->still_slot.core_list = entry->next;
    if (entry->next) entry->next->prev = entry->prev;
    mgr->still_slot.core_count--;

    /* Create a new active unit for this core */
    ivp_sim_unit_t *su = ivp_sim_unit_create(mgr);
    if (!su) {
        /* Fallback: just relink into still_slot */
        entry->prev = NULL;
        entry->next = mgr->still_slot.core_list;
        if (mgr->still_slot.core_list) {
            mgr->still_slot.core_list->prev = entry;
        }
        mgr->still_slot.core_list = entry;
        mgr->still_slot.core_count++;
        return;
    }

    /* Link entry into new unit */
    entry->prev = NULL;
    entry->next = su->core_list;
    if (su->core_list) {
        su->core_list->prev = entry;
    }
    su->core_list = entry;
    su->core_count++;
    core->sim_unit = su;

    core->movement_state = IVP_MT_MOVING;
}

/* ── Queries ───────────────────────────────────────────────────────────── */

bool ivp_sim_unit_core_is_frozen(const ivp_sim_units_manager_t *mgr,
                                   const ivp_core_t *core)
{
    return core->sim_unit == &mgr->still_slot;
}
