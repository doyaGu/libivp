/* ivp_sim_unit.c — Dynamic simulation island implementation
 *
 * Aligned with reference IVP_Simulation_Unit / IVP_Sim_Units_Manager
 * (ivp_sim_unit.hxx / ivp_sim_unit.cxx).
 */
#include <ivp/dynamics/ivp_sim_unit.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/utility/ivp_union_find.h>
#include "controllers/ivp_controller_system.h"
#include <string.h>

static int core_index_in_array(ivp_core_t **cores, int count, const ivp_core_t *core)
{
    for (int i = 0; i < count; i++) {
        if (cores[i] == core) return i;
    }
    return -1;
}

static void unlink_core_entry_from_unit(ivp_sim_unit_t *su,
                                        ivp_sim_unit_core_entry_t *entry)
{
    if (entry->prev) {
        entry->prev->next = entry->next;
    } else {
        su->core_list = entry->next;
    }
    if (entry->next) {
        entry->next->prev = entry->prev;
    }
    su->core_count--;
}

static void link_core_entry_to_unit(ivp_sim_unit_t *su,
                                    ivp_sim_unit_core_entry_t *entry)
{
    entry->prev = NULL;
    entry->next = su->core_list;
    if (su->core_list) {
        su->core_list->prev = entry;
    }
    su->core_list = entry;
    su->core_count++;
}

static void move_core_entry_between_units(ivp_sim_unit_t *from,
                                          ivp_sim_unit_t *to,
                                          ivp_sim_unit_core_entry_t *entry)
{
    unlink_core_entry_from_unit(from, entry);
    link_core_entry_to_unit(to, entry);

    entry->core->sim_unit = to;
    entry->core->sim_unit_entry = entry;
}

static ivp_controller_system_t *controller_system_from_env(ivp_environment_t *env)
{
    return (ivp_controller_system_t *)env->controller_system;
}

static bool has_sync_all_cores_controller_below_gravity(ivp_environment_t *env)
{
    ivp_controller_system_t *cs = controller_system_from_env(env);
    int sc = ivp_controller_system_count(cs);
    for (int si = 0; si < sc; si++) {
        ivp_controller_slot_t *slot = ivp_controller_system_get_at(cs, si);
        if (!slot || !slot->enabled) continue;
        if (slot->mode != IVP_CONTROLLER_INDEPENDENT) continue;
        if (!slot->run_for_all_cores) continue;
        if (slot->priority < IVP_CP_GRAVITY) return true;
    }
    return false;
}

static void union_friction_connectivity(ivp_friction_manager_t *fm,
                                        ivp_core_t **cores,
                                        int n,
                                        int *parent)
{
    for (int si = 0; si < fm->system_count; si++) {
        ivp_friction_system_t *sys = fm->systems[si];
        if (!sys) continue;
        for (int pi = 0; pi < sys->pair_count; pi++) {
            const ivp_friction_core_pair_t *pair = &sys->pairs[pi];
            int ia = core_index_in_array(cores, n, pair->cores[0]);
            int ib = core_index_in_array(cores, n, pair->cores[1]);
            if (ia >= 0 && ib >= 0) {
                ivp_union_find_union(parent, ia, ib);
            }
        }
    }
}

static void union_dependent_controller_connectivity(ivp_environment_t *env,
                                                    ivp_core_t **cores,
                                                    int n,
                                                    int *parent)
{
    ivp_controller_system_t *cs = controller_system_from_env(env);
    ivp_object_handle_t associated_handles[IVP_CONTROLLER_DISCOVERY_STACK_MAX];
    int sc = ivp_controller_system_count(cs);
    for (int si = 0; si < sc; si++) {
        ivp_controller_slot_t *slot = ivp_controller_system_get_at(cs, si);
        if (!slot || !slot->enabled) continue;
        if (slot->mode != IVP_CONTROLLER_DEPENDENT) continue;

        int associated_count = ivp_controller_slot_collect_associated_handles(
            slot, env, associated_handles, IVP_CONTROLLER_DISCOVERY_STACK_MAX);
        if (associated_count <= 1) continue;

        int base_idx = -1;
        for (int ai = 0; ai < associated_count; ai++) {
            ivp_object_t *obj = ivp_environment_get_object_by_handle(env, associated_handles[ai]);
            if (!obj) continue;
            int idx = core_index_in_array(cores, n, &obj->core);
            if (idx < 0) continue;
            if (base_idx < 0) {
                base_idx = idx;
            } else {
                ivp_union_find_union(parent, base_idx, idx);
            }
        }
    }
}

static int label_connected_components(int *parent,
                                      int *component,
                                      int *component_size,
                                      int n)
{
    int component_count = 0;
    for (int i = 0; i < n; i++) {
        int root = ivp_union_find_find(parent, i);
        int cid = -1;
        for (int j = 0; j < component_count; j++) {
            if (component[j] == root) {
                cid = j;
                break;
            }
        }
        if (cid < 0) {
            cid = component_count;
            component[component_count] = root;
            component_size[component_count] = 0;
            component_count++;
        }
        parent[i] = cid;
        component_size[cid]++;
    }
    return component_count;
}

static void rehome_friction_controllers_after_split(ivp_sim_unit_t *su,
                                                    ivp_friction_manager_t *fm)
{
    for (int si = 0; si < fm->system_count; si++) {
        ivp_friction_system_t *sys = fm->systems[si];
        if (!sys || sys->core_count <= 0 || !sys->cores) continue;

        ivp_sim_unit_t *target_su = NULL;
        for (int ci = 0; ci < sys->core_count; ci++) {
            ivp_core_t *core = sys->cores[ci];
            if (core && core->sim_unit && core->sim_unit != &su->manager->still_slot) {
                target_su = core->sim_unit;
                break;
            }
        }
        if (!target_su) continue;

        ivp_controller_core_t *ccs[3] = { &sys->static_cc, &sys->dynamic_cc, &sys->energy_cc };
        for (int ci = 0; ci < 3; ci++) {
            ivp_controller_core_t *cc = ccs[ci];
            if (cc->sim_unit == target_su) continue;
            if (cc->sim_unit) {
                ivp_sim_unit_remove_controller(cc);
            }
            ivp_sim_unit_add_controller(target_su, cc);
        }
    }
}

static void maybe_split_disconnected_components(ivp_sim_unit_t *su,
                                                ivp_environment_t *env)
{
    if (!su || !env) return;
    if (su->core_count <= 1) return;
    if (has_sync_all_cores_controller_below_gravity(env)) return;

    ivp_core_t *local_cores[64];
    int local_parent[64];
    int local_component[64];
    int local_component_size[64];
    ivp_core_t **cores = local_cores;
    int *parent = NULL;
    int *component = NULL;
    int *component_size = NULL;
    bool heap_alloc = false;

    if (su->core_count > 64) {
        size_t core_bytes = (size_t)su->core_count * sizeof(ivp_core_t *);
        size_t int_bytes = (size_t)su->core_count * sizeof(int);
        cores = (ivp_core_t **)ivp_alloc(su->manager->allocator, core_bytes);
        parent = (int *)ivp_alloc(su->manager->allocator, int_bytes);
        component = (int *)ivp_alloc(su->manager->allocator, int_bytes);
        component_size = (int *)ivp_alloc(su->manager->allocator, int_bytes);
        if (!cores || !parent || !component || !component_size) {
            if (component_size) ivp_free(su->manager->allocator, component_size, int_bytes);
            if (component) ivp_free(su->manager->allocator, component, int_bytes);
            if (parent) ivp_free(su->manager->allocator, parent, int_bytes);
            if (cores) ivp_free(su->manager->allocator, cores, core_bytes);
            return;
        }
        heap_alloc = true;
    } else {
        parent = local_parent;
        component = local_component;
        component_size = local_component_size;
    }

    int n = 0;
    for (ivp_sim_unit_core_entry_t *ce = su->core_list; ce; ce = ce->next) {
        cores[n] = ce->core;
        component[n] = -1;
        component_size[n] = 0;
        n++;
    }
    ivp_union_find_init(parent, n);

    ivp_friction_manager_t *fm = &env->friction_manager;
    union_friction_connectivity(fm, cores, n, parent);
    union_dependent_controller_connectivity(env, cores, n, parent);

    int component_count = label_connected_components(parent, component, component_size, n);

    if (component_count > 1) {
        int keep_component = 0;
        for (int c = 1; c < component_count; c++) {
            if (component_size[c] > component_size[keep_component]) {
                keep_component = c;
            }
        }

        for (int c = 0; c < component_count; c++) {
            if (c == keep_component) continue;

            ivp_sim_unit_t *new_su = ivp_sim_unit_create(su->manager);
            if (!new_su) continue;

            for (ivp_sim_unit_core_entry_t *ce = su->core_list; ce; ) {
                ivp_sim_unit_core_entry_t *next = ce->next;
                int idx = core_index_in_array(cores, n, ce->core);
                if (idx >= 0 && parent[idx] == c) {
                    move_core_entry_between_units(su, new_su, ce);
                }
                ce = next;
            }
        }

        rehome_friction_controllers_after_split(su, fm);
    }

    if (heap_alloc) {
        size_t core_bytes = (size_t)n * sizeof(ivp_core_t *);
        size_t int_bytes = (size_t)n * sizeof(int);
        ivp_free(su->manager->allocator, component_size, int_bytes);
        ivp_free(su->manager->allocator, component, int_bytes);
        ivp_free(su->manager->allocator, parent, int_bytes);
        ivp_free(su->manager->allocator, cores, core_bytes);
    }
}

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
    link_core_entry_to_unit(su, entry);

    core->sim_unit = su;
    core->sim_unit_entry = entry;
}

void ivp_sim_unit_remove_core(ivp_sim_units_manager_t *mgr, ivp_core_t *core)
{
    if (!core || !core->sim_unit) return;

    ivp_sim_unit_t *su = core->sim_unit;
    ivp_sim_unit_core_entry_t *entry = core->sim_unit_entry;
    IVP_ASSERT(entry);

    unlink_core_entry_from_unit(su, entry);

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
        move_core_entry_between_units(su_b, su_a, ce);
        ce = next;
    }

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
        maybe_split_disconnected_components(su, env);
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
