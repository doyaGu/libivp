/* ivp_friction_system.c — Persistent friction system implementation
 *
 * Implements frame-to-frame contact matching via object-space positions,
 * coupled matrix friction solver, energy tracking, and system merge/split.
 */

#include <ivp/solver/ivp_friction_system.h>
#include <ivp/solver/ivp_impact.h>
#include <ivp/solver/ivp_great_matrix.h>
#include <ivp/solver/ivp_solver_config.h>
#include <ivp/solver/ivp_solver_metrics.h>
#include <ivp/solver/ivp_solver_constants.h>
#include <ivp/solver/ivp_contact_order.h>
#include <ivp/solver/ivp_mindist_friction_update.h>
#include <ivp/collision/ivp_synapse.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/events/ivp_listener_collision.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>
#include <math.h>

/* ── Helpers ────────────────────────────────────────────────────────────── */

/* Transform world point to object-space using inverse of core transform */
static void world_to_object_space(const ivp_core_t *core,
                                   const ivp_vec3r_t *p_world,
                                   ivp_vec3r_t *p_os) {
    ivp_transform_inv_point(p_os, &core->m_world_f_core_last_psi, p_world);
}

static void world_dir_to_object_space(const ivp_core_t *core,
                                      const ivp_vec3r_t *dir_world,
                                      ivp_vec3r_t *dir_os) {
    ivp_mat3r_vimult(dir_os, &core->m_world_f_core_last_psi.rot, dir_world);
    ivp_vec3r_normalize(dir_os);
}

static ivp_friction_event_t make_friction_event(const ivp_contact_point_t *cp,
                                                ivp_real_t normal_force)
{
    ivp_friction_event_t event;
    event.obj_a = cp->obj_a;
    event.obj_b = cp->obj_b;
    event.contact_point = cp->point_a;
    event.contact_normal = cp->normal;
    event.normal_force = normal_force;
    return event;
}

/* Find system containing a given core */
ivp_friction_system_t *ivp_friction_manager_find_system_for_core(
    ivp_friction_manager_t *mgr, ivp_core_t *core) {
    for (int i = 0; i < mgr->system_count; i++) {
        ivp_friction_system_t *sys = mgr->systems[i];
        for (int j = 0; j < sys->core_count; j++) {
            if (sys->cores[j] == core) {
                return sys;
            }
        }
    }
    return NULL;
}

/* Add a core to a system if not already present */
static void system_add_core(ivp_friction_system_t *sys, ivp_core_t *core) {
    /* Check if already present */
    for (int i = 0; i < sys->core_count; i++) {
        if (sys->cores[i] == core) {
            return;
        }
    }

    /* Grow array if needed */
    if (sys->core_count >= sys->core_capacity) {
        int new_cap = sys->core_capacity * 2;
        if (new_cap < 8) new_cap = 8;
        size_t old_size = (size_t)sys->core_capacity * sizeof(ivp_core_t*);
        size_t new_size = (size_t)new_cap * sizeof(ivp_core_t*);
        ivp_core_t **new_cores = ivp_realloc(sys->alloc, sys->cores, old_size, new_size);
        if (new_cores) {
            sys->cores = new_cores;
            sys->core_capacity = new_cap;
        } else {
            return; /* allocation failed */
        }
    }

    sys->cores[sys->core_count++] = core;
}

/* Find or create friction core pair */
static ivp_friction_core_pair_t *get_or_create_pair(ivp_friction_system_t *sys,
                                                      ivp_core_t *core_a,
                                                      ivp_core_t *core_b) {
    /* Search existing pairs */
    for (int i = 0; i < sys->pair_count; i++) {
        ivp_friction_core_pair_t *p = &sys->pairs[i];
        if ((p->cores[0] == core_a && p->cores[1] == core_b) ||
            (p->cores[0] == core_b && p->cores[1] == core_a)) {
            return p;
        }
    }

    /* Need to create new pair */
    if (sys->pair_count >= sys->pair_capacity) {
        int new_cap = sys->pair_capacity * 2;
        if (new_cap < 8) new_cap = 8;
        size_t old_size = (size_t)sys->pair_capacity * sizeof(ivp_friction_core_pair_t);
        size_t new_size = (size_t)new_cap * sizeof(ivp_friction_core_pair_t);
        ivp_friction_core_pair_t *new_pairs = ivp_realloc(sys->alloc, sys->pairs,
                                                            old_size, new_size);
        if (new_pairs) {
            sys->pairs = new_pairs;
            sys->pair_capacity = new_cap;
        } else {
            return NULL;
        }
    }

    ivp_friction_core_pair_t *pair = &sys->pairs[sys->pair_count++];
    memset(pair, 0, sizeof(*pair));
    pair->cores[0] = core_a;
    pair->cores[1] = core_b;
    pair->alloc = sys->alloc;
    pair->next_ease_nr_psi = IVP_EASE_EVERY_NTH_PSI;
    pair->contact_capacity = 8;
    pair->contacts = ivp_alloc(pair->alloc,
                                (size_t)pair->contact_capacity *
                                sizeof(ivp_persistent_contact_t*));
    if (!pair->contacts) {
        sys->pair_count--;
        return NULL;
    }
    return pair;
}

/* Add contact to pair. Returns false on allocation failure. */
static bool pair_add_contact(ivp_friction_core_pair_t *pair,
                             ivp_persistent_contact_t *contact) {
    if (!pair || !contact) return false;
    for (int i = 0; i < pair->contact_count; i++) {
        if (pair->contacts[i] == contact) {
            return true;
        }
    }

    if (pair->contact_count >= pair->contact_capacity) {
        int new_cap = pair->contact_capacity * 2;
        size_t old_size = (size_t)pair->contact_capacity * sizeof(ivp_persistent_contact_t*);
        size_t new_size = (size_t)new_cap * sizeof(ivp_persistent_contact_t*);
        ivp_persistent_contact_t **new_contacts =
            ivp_realloc(pair->alloc, pair->contacts, old_size, new_size);
        if (new_contacts) {
            pair->contacts = new_contacts;
            pair->contact_capacity = new_cap;
        } else {
            return false;
        }
    }
    pair->contacts[pair->contact_count++] = contact;
    return true;
}

static void pair_remove_contact(ivp_friction_core_pair_t *pair,
                                 ivp_persistent_contact_t *contact) {
    if (!pair || !contact) return;

    for (int i = 0; i < pair->contact_count; i++) {
        if (pair->contacts[i] == contact) {
            for (int j = i; j < pair->contact_count - 1; j++) {
                pair->contacts[j] = pair->contacts[j + 1];
            }
            pair->contact_count--;
            return;
        }
    }
}

static void system_compact_empty_pairs(ivp_friction_system_t *sys) {
    if (!sys || !sys->pairs) return;

    int write_idx = 0;
    for (int read_idx = 0; read_idx < sys->pair_count; read_idx++) {
        ivp_friction_core_pair_t *pair = &sys->pairs[read_idx];
        if (pair->contact_count > 0) {
            if (write_idx != read_idx) {
                sys->pairs[write_idx] = *pair;
            }
            write_idx++;
        } else {
            if (pair->contacts) {
                ivp_free(sys->alloc, pair->contacts,
                         (size_t)pair->contact_capacity * sizeof(ivp_persistent_contact_t*));
            }
        }
    }
    sys->pair_count = write_idx;
}

/* Free a system's internal arrays (pairs with their contact arrays, and cores).
 * Does NOT free the system struct itself — caller is responsible for that. */
static void friction_system_free_internals(ivp_friction_system_t *sys) {
    if (sys->pairs) {
        for (int i = 0; i < sys->pair_count; i++) {
            if (sys->pairs[i].contacts) {
                ivp_free(sys->alloc, sys->pairs[i].contacts,
                         (size_t)sys->pairs[i].contact_capacity *
                         sizeof(ivp_persistent_contact_t*));
            }
        }
        ivp_free(sys->alloc, sys->pairs,
                 (size_t)sys->pair_capacity * sizeof(ivp_friction_core_pair_t));
    }
    if (sys->cores) {
        ivp_free(sys->alloc, sys->cores,
                 (size_t)sys->core_capacity * sizeof(ivp_core_t*));
    }
}

/* ── Contact release helper ─────────────────────────────────────────────── */

/* Return a freshly-allocated contact that could not be used.
 * If the contact came from the pool tail, decrement current_block_used;
 * otherwise push it onto the free list. */
static void release_new_contact(ivp_friction_manager_t *mgr,
                                 ivp_persistent_contact_t *pc,
                                 bool from_pool_tail)
{
    if (from_pool_tail) {
        mgr->current_block_used--;
    } else {
        pc->next = mgr->free_contacts;
        mgr->free_contacts = pc;
    }
}

/* ── Per-system friction controller callbacks ─────────────────────────── */

/* These are ivp_controller_core_fn_t callbacks registered in each friction
 * system's sim unit.  They replace the old global friction controllers.
 * ctx is the ivp_friction_system_t*. */

static void friction_cc_static(void *ctx, ivp_environment_t *env,
                                ivp_core_t **cores, int core_count,
                                ivp_real_t delta_time)
{
    (void)cores; (void)core_count;
    ivp_friction_system_t *sys = (ivp_friction_system_t *)ctx;
    if (sys->core_count < 2) return;

    ivp_collision_settings_t coll_settings;
    ivp_collision_settings_init_default(&coll_settings);
    coll_settings.gravity_magnitude = ivp_vec3r_length(&env->gravity);

    ivp_friction_system_phase_static_normal(sys, &coll_settings, delta_time);
}

static void friction_cc_dynamic(void *ctx, ivp_environment_t *env,
                                 ivp_core_t **cores, int core_count,
                                 ivp_real_t delta_time)
{
    (void)cores; (void)core_count;
    ivp_friction_system_t *sys = (ivp_friction_system_t *)ctx;
    if (sys->core_count < 2) return;

    ivp_collision_settings_t coll_settings;
    ivp_collision_settings_init_default(&coll_settings);
    coll_settings.gravity_magnitude = ivp_vec3r_length(&env->gravity);

    ivp_friction_system_phase_dynamic_tangential(sys, &coll_settings, delta_time);
}

static void friction_cc_energy(void *ctx, ivp_environment_t *env,
                                ivp_core_t **cores, int core_count,
                                ivp_real_t delta_time)
{
    (void)cores; (void)core_count;
    ivp_friction_system_t *sys = (ivp_friction_system_t *)ctx;

    ivp_collision_settings_t coll_settings;
    ivp_collision_settings_init_default(&coll_settings);
    coll_settings.gravity_magnitude = ivp_vec3r_length(&env->gravity);

    ivp_friction_system_phase_energy(sys, &coll_settings, delta_time);
}

/* Register the three friction controller_cores in the given sim unit.
 * The system's cc fields must already be initialized. */
static void friction_system_register_controllers(ivp_friction_system_t *sys,
                                                   ivp_sim_unit_t *su)
{
    if (!su || su == &su->manager->still_slot) return;
    ivp_sim_unit_add_controller(su, &sys->static_cc);
    ivp_sim_unit_add_controller(su, &sys->dynamic_cc);
    ivp_sim_unit_add_controller(su, &sys->energy_cc);
}

/* Unregister the three friction controller_cores from their sim unit. */
static void friction_system_unregister_controllers(ivp_friction_system_t *sys)
{
    ivp_sim_unit_remove_controller(&sys->static_cc);
    ivp_sim_unit_remove_controller(&sys->dynamic_cc);
    ivp_sim_unit_remove_controller(&sys->energy_cc);
}

/* Find the appropriate sim unit for a friction system:
 * use the first non-static core's sim unit. */
static ivp_sim_unit_t *friction_system_find_sim_unit(ivp_friction_system_t *sys)
{
    for (int i = 0; i < sys->core_count; i++) {
        if (!sys->cores[i]->physical_unmoveable && sys->cores[i]->sim_unit) {
            return sys->cores[i]->sim_unit;
        }
    }
    return NULL;
}

/* ── New friction system allocation ────────────────────────────────────── */

/* Allocate a new friction system with initial cores and pairs arrays,
 * register it in the manager, and add the two cores.
 * Returns NULL on any allocation failure (manager arrays may still grow). */
static ivp_friction_system_t *friction_system_create(
    ivp_friction_manager_t *mgr,
    ivp_core_t *core_a,
    ivp_core_t *core_b)
{
    /* Grow manager systems array if needed */
    if (mgr->system_count >= mgr->system_capacity) {
        int new_cap = mgr->system_capacity * 2;
        size_t old_size = (size_t)mgr->system_capacity * sizeof(ivp_friction_system_t*);
        size_t new_size = (size_t)new_cap * sizeof(ivp_friction_system_t*);
        ivp_friction_system_t **new_systems = ivp_realloc(mgr->alloc, mgr->systems,
                                                            old_size, new_size);
        if (!new_systems) return NULL;
        mgr->systems = new_systems;
        mgr->system_capacity = new_cap;
    }

    ivp_friction_system_t *sys = ivp_alloc(mgr->alloc, sizeof(ivp_friction_system_t));
    if (!sys) return NULL;
    memset(sys, 0, sizeof(*sys));
    sys->alloc = mgr->alloc;
    sys->manager = mgr;

    sys->core_capacity = 8;
    sys->cores = ivp_alloc(mgr->alloc,
                             (size_t)sys->core_capacity * sizeof(ivp_core_t*));
    if (!sys->cores) {
        ivp_free(mgr->alloc, sys, sizeof(*sys));
        return NULL;
    }

    sys->pair_capacity = 8;
    sys->pairs = ivp_alloc(mgr->alloc,
                             (size_t)sys->pair_capacity *
                             sizeof(ivp_friction_core_pair_t));
    if (!sys->pairs) {
        ivp_free(mgr->alloc, sys->cores,
                 (size_t)sys->core_capacity * sizeof(ivp_core_t*));
        ivp_free(mgr->alloc, sys, sizeof(*sys));
        return NULL;
    }

    mgr->systems[mgr->system_count++] = sys;

    system_add_core(sys, core_a);
    system_add_core(sys, core_b);

    /* Initialize the three per-system controller_cores. */
    ivp_controller_core_init(&sys->static_cc, friction_cc_static, sys,
                              IVP_CP_STATIC_FRICTION);
    ivp_controller_core_init(&sys->dynamic_cc, friction_cc_dynamic, sys,
                              IVP_CP_DYNAMIC_FRICTION);
    ivp_controller_core_init(&sys->energy_cc, friction_cc_energy, sys,
                              IVP_CP_ENERGY_FRICTION);

    /* Register controllers in the first non-static core's sim unit. */
    ivp_sim_unit_t *su = friction_system_find_sim_unit(sys);
    if (su) {
        friction_system_register_controllers(sys, su);
    }

    return sys;
}

/* ── Manager lifecycle ──────────────────────────────────────────────────── */

ivp_result_t ivp_friction_manager_init(ivp_friction_manager_t *mgr,
                                         const ivp_allocator_t *alloc) {
    if (!mgr) return IVP_ERR_INVALID_ARG;
    if (!alloc) alloc = ivp_default_allocator();

    memset(mgr, 0, sizeof(*mgr));
    mgr->alloc = alloc;

    /* Allocate first contact pool block */
    mgr->pool_blocks[0] = ivp_alloc(alloc, (size_t)IVP_FRICTION_POOL_BLOCK_SIZE *
                                     sizeof(ivp_persistent_contact_t));
    if (!mgr->pool_blocks[0]) return IVP_ERR_OUT_OF_MEMORY;
    mgr->block_count = 1;
    mgr->current_block_used = 0;

    /* Allocate systems array */
    mgr->system_capacity = 16;
    mgr->systems = ivp_alloc(alloc, (size_t)mgr->system_capacity *
                              sizeof(ivp_friction_system_t*));
    if (!mgr->systems) {
        ivp_free(alloc, mgr->pool_blocks[0], (size_t)IVP_FRICTION_POOL_BLOCK_SIZE *
                  sizeof(ivp_persistent_contact_t));
        return IVP_ERR_OUT_OF_MEMORY;
    }

    mgr->psi_counter = 0;
    return IVP_OK;
}

void ivp_friction_manager_destroy(ivp_friction_manager_t *mgr) {
    if (!mgr) return;

    /* Free all systems */
    for (int i = 0; i < mgr->system_count; i++) {
        ivp_friction_system_t *sys = mgr->systems[i];
        if (sys) {
            /* Unregister per-unit controllers before destroying */
            friction_system_unregister_controllers(sys);

            ivp_persistent_contact_t *pc = sys->first_contact;
            while (pc) {
                ivp_synapse_detach_friction_contact(pc);
                pc = pc->next;
            }
            friction_system_free_internals(sys);
            ivp_free(mgr->alloc, sys, sizeof(*sys));
        }
    }

    if (mgr->systems) {
        ivp_free(mgr->alloc, mgr->systems,
                 (size_t)mgr->system_capacity * sizeof(ivp_friction_system_t*));
    }

    /* Free all pool blocks */
    for (int b = 0; b < mgr->block_count; b++) {
        if (mgr->pool_blocks[b]) {
            ivp_free(mgr->alloc, mgr->pool_blocks[b],
                     (size_t)IVP_FRICTION_POOL_BLOCK_SIZE * sizeof(ivp_persistent_contact_t));
        }
    }

    memset(mgr, 0, sizeof(*mgr));
}

/* ── PSI lifecycle ──────────────────────────────────────────────────────── */

void ivp_friction_manager_begin_psi(ivp_friction_manager_t *mgr) {
    if (!mgr) return;

    mgr->psi_counter++;

    /* Mark all contacts as unseen */
    for (int i = 0; i < mgr->system_count; i++) {
        ivp_friction_system_t *sys = mgr->systems[i];
        ivp_persistent_contact_t *pc = sys->first_contact;
        while (pc) {
            pc->seen_this_psi = false;
            pc = pc->next;
        }
    }
}

/* ── Contact matching ───────────────────────────────────────────────────── */

ivp_persistent_contact_t *ivp_friction_manager_match_or_create(
    ivp_friction_manager_t *mgr,
    const ivp_contact_point_t *new_cp) {

    if (!mgr || !new_cp) return NULL;

    ivp_object_t *obj_a = new_cp->obj_a;
    ivp_object_t *obj_b = new_cp->obj_b;
    if (!obj_a || !obj_b) return NULL;

    ivp_core_t *core_a = &obj_a->core;
    ivp_core_t *core_b = &obj_b->core;

    /* Compute object-space positions for matching */
    ivp_vec3r_t new_os_a, new_os_b;
    world_to_object_space(core_a, &new_cp->point_a, &new_os_a);
    world_to_object_space(core_b, &new_cp->point_b, &new_os_b);
    ivp_vec3r_t new_normal_os_a, new_normal_os_b;
    world_dir_to_object_space(core_a, &new_cp->normal, &new_normal_os_a);
    world_dir_to_object_space(core_b, &new_cp->normal, &new_normal_os_b);

    ivp_persistent_contact_t *best_match = NULL;
    ivp_real_t best_score = IVP_REAL(1e30);

    /* Search for matching contact in existing systems */
    for (int i = 0; i < mgr->system_count; i++) {
        ivp_friction_system_t *sys = mgr->systems[i];
        ivp_persistent_contact_t *pc = sys->first_contact;

        while (pc) {
            if (pc->seen_this_psi) {
                pc = pc->next;
                continue;
            }

            bool same_order = (pc->cp.obj_a == obj_a && pc->cp.obj_b == obj_b);
            bool reverse_order = (pc->cp.obj_a == obj_b && pc->cp.obj_b == obj_a);

            if (same_order || reverse_order) {
                /* Support both orderings so SAP axis order changes do not
                 * break persistence matching. */
                const ivp_vec3r_t *match_os0 = same_order ? &new_os_a : &new_os_b;
                const ivp_vec3r_t *match_os1 = same_order ? &new_os_b : &new_os_a;

                ivp_real_t dist_sq_a = ivp_vec3r_quad_distance(&pc->contact_point_os[0], match_os0);
                ivp_real_t dist_sq_b = ivp_vec3r_quad_distance(&pc->contact_point_os[1], match_os1);

                if (dist_sq_a < IVP_FRICTION_MATCH_DIST_SQ &&
                    dist_sq_b < IVP_FRICTION_MATCH_DIST_SQ) {
                    ivp_real_t local_normal_dot_a;
                    ivp_real_t local_normal_dot_b;
                    if (same_order) {
                        local_normal_dot_a = ivp_vec3r_dot(&pc->contact_normal_os[0], &new_normal_os_a);
                        local_normal_dot_b = ivp_vec3r_dot(&pc->contact_normal_os[1], &new_normal_os_b);
                    } else {
                        local_normal_dot_a = -ivp_vec3r_dot(&pc->contact_normal_os[0], &new_normal_os_b);
                        local_normal_dot_b = -ivp_vec3r_dot(&pc->contact_normal_os[1], &new_normal_os_a);
                    }
                    if (local_normal_dot_a < IVP_FRICTION_MATCH_NORMAL_OS_DOT ||
                        local_normal_dot_b < IVP_FRICTION_MATCH_NORMAL_OS_DOT) {
                        pc = pc->next;
                        continue;
                    }

                    ivp_real_t normal_dot = ivp_vec3r_dot(&pc->cp.normal, &new_cp->normal);
                    if (reverse_order) {
                        normal_dot = -normal_dot;
                    }
                    if (normal_dot > IVP_REAL(1.0)) normal_dot = IVP_REAL(1.0);
                    if (normal_dot < IVP_REAL(0.0)) normal_dot = IVP_REAL(0.0);

                    ivp_real_t gap_diff = ivp_fabs(pc->cp.gap_length - new_cp->gap_length);
                    ivp_real_t score = dist_sq_a + dist_sq_b
                                     + (IVP_REAL(1.0) - normal_dot) * IVP_REAL(0.001)
                                     + (IVP_REAL(1.0) - local_normal_dot_a) * IVP_REAL(0.001)
                                     + (IVP_REAL(1.0) - local_normal_dot_b) * IVP_REAL(0.001)
                                     + gap_diff * IVP_REAL(0.01);

                    if (score < best_score) {
                        best_score = score;
                        best_match = pc;
                    }
                }
            }

            pc = pc->next;
        }
    }

    if (best_match) {
        /* Match found! Update geometry and mark as seen.
         * Preserve age_psi across matches so the distance
         * keeper ramp-up accumulates correctly. */
        int saved_age = best_match->cp.age_psi;
        int saved_keeper_countdown = best_match->slowly_turn_on_keeper;
        ivp_real_t saved_old_energy = best_match->old_energy_dynamic_fr;
        ivp_real_t saved_inv_vm = best_match->inv_virt_mass_mindist_no_dir;
        bool saved_two_values = best_match->two_friction_values;
        ivp_real_t saved_two_value_scale = best_match->two_friction_x_scale;
        best_match->cp = *new_cp;
        best_match->cp.age_psi = saved_age;
        best_match->contact_point_os[0] = new_os_a;
        best_match->contact_point_os[1] = new_os_b;
        best_match->contact_normal_os[0] = new_normal_os_a;
        best_match->contact_normal_os[1] = new_normal_os_b;
        best_match->seen_this_psi = true;
        best_match->impacts_while_system = 0;
        best_match->slowly_turn_on_keeper = saved_keeper_countdown;
        best_match->old_energy_dynamic_fr = saved_old_energy;
        best_match->inv_virt_mass_mindist_no_dir = saved_inv_vm;
        best_match->two_friction_values = saved_two_values;
        best_match->two_friction_x_scale = saved_two_value_scale;
        best_match->psi_scratch.index_in_fs = -1;
        best_match->psi_scratch.has_negative_pull_since = best_match->has_negative_pull_since;
        best_match->psi_scratch.dist_len = best_match->cp.gap_length;
        best_match->psi_scratch.friction_is_broken = best_match->friction_is_broken;
        best_match->last_gap_len = new_cp->gap_length;
        return best_match;
    }

    /* No match found — allocate new contact from free list or pool tail.
     * Pool uses fixed-size blocks: existing blocks are never reallocated,
     * so all live pointers remain stable.  New blocks are appended when
     * the current block is full. */
    bool new_pc_from_pool_tail = false;
    ivp_persistent_contact_t *new_pc = NULL;

    if (mgr->free_contacts) {
        new_pc = mgr->free_contacts;
        mgr->free_contacts = new_pc->next;
    } else {
        if (mgr->current_block_used >= IVP_FRICTION_POOL_BLOCK_SIZE) {
            /* Current block exhausted — allocate a new one */
            if (mgr->block_count >= IVP_FRICTION_POOL_MAX_BLOCKS) {
                return NULL;  /* hard limit reached */
            }
            ivp_persistent_contact_t *new_block =
                ivp_alloc(mgr->alloc,
                          (size_t)IVP_FRICTION_POOL_BLOCK_SIZE *
                          sizeof(ivp_persistent_contact_t));
            if (!new_block) return NULL;
            mgr->pool_blocks[mgr->block_count++] = new_block;
            mgr->current_block_used = 0;
        }
        new_pc = &mgr->pool_blocks[mgr->block_count - 1][mgr->current_block_used++];
        new_pc_from_pool_tail = true;
    }

    memset(new_pc, 0, sizeof(*new_pc));

    new_pc->cp = *new_cp;
    new_pc->cp.age_psi = 0;
    new_pc->contact_point_os[0] = new_os_a;
    new_pc->contact_point_os[1] = new_os_b;
    new_pc->contact_normal_os[0] = new_normal_os_a;
    new_pc->contact_normal_os[1] = new_normal_os_b;
    new_pc->seen_this_psi = true;
    new_pc->last_gap_len = new_cp->gap_length;
    new_pc->friction_is_broken = false;
    new_pc->cp_status = IVP_CP_STATUS_NORMAL;
    new_pc->has_negative_pull_since = 0;
    new_pc->slowly_turn_on_keeper = IVP_SLOWLY_TURN_ON_KEEPER;
    new_pc->impacts_while_system = 0;
    new_pc->impact_resolved = false;
    new_pc->old_energy_dynamic_fr = IVP_REAL(0.0);
    new_pc->inv_virt_mass_mindist_no_dir = new_cp->virtual_mass > IVP_REAL_EPS
        ? (IVP_REAL(1.0) / new_cp->virtual_mass)
        : IVP_REAL(0.0);
    new_pc->two_friction_values =
        (new_cp->obj_a->material.second_friction_x_enabled ||
         new_cp->obj_b->material.second_friction_x_enabled);
    new_pc->two_friction_x_scale = IVP_REAL(1.0);
    if (new_pc->two_friction_values) {
        ivp_real_t s0 = new_cp->obj_a->material.second_friction_x_enabled
            ? new_cp->obj_a->material.second_friction_factor : IVP_REAL(1.0);
        ivp_real_t s1 = new_cp->obj_b->material.second_friction_x_enabled
            ? new_cp->obj_b->material.second_friction_factor : IVP_REAL(1.0);
        new_pc->two_friction_x_scale = s0 * s1;
        if (new_pc->two_friction_x_scale < IVP_REAL_EPS) {
            new_pc->two_friction_x_scale = IVP_REAL_EPS;
        }
    }
    new_pc->psi_scratch.index_in_fs = -1;
    new_pc->psi_scratch.has_negative_pull_since = 0;
    new_pc->psi_scratch.dist_len = new_cp->gap_length;
    new_pc->psi_scratch.friction_is_broken = false;

    /* Find or create system for these cores */
    ivp_friction_system_t *sys_a = ivp_friction_manager_find_system_for_core(mgr, core_a);
    ivp_friction_system_t *sys_b = ivp_friction_manager_find_system_for_core(mgr, core_b);

    ivp_friction_system_t *target_sys = NULL;
    bool created_new_system = false;

    if (sys_a && sys_b && sys_a != sys_b) {
        /* Merge friction systems (unregisters src controllers) */
        friction_system_unregister_controllers(sys_b);
        ivp_friction_system_merge(sys_a, sys_b, mgr);
        target_sys = sys_a;
    } else if (sys_a) {
        target_sys = sys_a;
        system_add_core(target_sys, core_b);
    } else if (sys_b) {
        target_sys = sys_b;
        system_add_core(target_sys, core_a);
    } else {
        target_sys = friction_system_create(mgr, core_a, core_b);
        if (!target_sys) {
            release_new_contact(mgr, new_pc, new_pc_from_pool_tail);
            return NULL;
        }
        created_new_system = true;
    }

    /* Merge sim units if the two cores are in different active units.
     * Reference: friction contact creation triggers sim unit merge
     * (ivp_friction.cxx fusion_friction_systems + sim_unit merge). */
    {
        ivp_sim_unit_t *su_a = core_a->sim_unit;
        ivp_sim_unit_t *su_b = core_b->sim_unit;
        ivp_sim_units_manager_t *sum = mgr->owner
            ? mgr->owner->sim_units_manager : NULL;

        if (sum && su_a && su_b && su_a != su_b
            && su_a != &sum->still_slot && su_b != &sum->still_slot)
        {
            /* Unregister friction controllers before sim unit merge
             * (merge moves controllers, we'll re-register after). */
            friction_system_unregister_controllers(target_sys);
            ivp_sim_unit_merge(sum, su_a, su_b);
            /* su_b is now destroyed; su_a has all cores+controllers.
             * Re-register friction controllers in the merged unit. */
            friction_system_register_controllers(target_sys, su_a);
        } else if (!created_new_system) {
            /* Ensure controllers are registered in the right unit.
             * (They might not be if the system gained a new core.) */
            ivp_sim_unit_t *target_su = friction_system_find_sim_unit(target_sys);
            if (target_su && target_sys->static_cc.sim_unit != target_su) {
                friction_system_unregister_controllers(target_sys);
                friction_system_register_controllers(target_sys, target_su);
            }
        }
    }

    /* Add to doubly-linked list */
    new_pc->next = target_sys->first_contact;
    new_pc->prev = NULL;
    if (target_sys->first_contact) {
        target_sys->first_contact->prev = new_pc;
    }
    target_sys->first_contact = new_pc;
    target_sys->contact_count++;

    /* Add to core pair */
    ivp_friction_core_pair_t *pair = get_or_create_pair(target_sys, core_a, core_b);
    if (!pair || !pair_add_contact(pair, new_pc)) {
        /* Roll back contact insertion on pair allocation failure. */
        if (new_pc->next) {
            new_pc->next->prev = NULL;
        }
        target_sys->first_contact = new_pc->next;
        target_sys->contact_count--;

        release_new_contact(mgr, new_pc, new_pc_from_pool_tail);

        if (created_new_system) {
            for (int i = 0; i < mgr->system_count; i++) {
                if (mgr->systems[i] == target_sys) {
                    for (int j = i; j < mgr->system_count - 1; j++) {
                        mgr->systems[j] = mgr->systems[j + 1];
                    }
                    mgr->system_count--;
                    break;
                }
            }
            friction_system_free_internals(target_sys);
            ivp_free(mgr->alloc, target_sys, sizeof(*target_sys));
        }
        return NULL;
    }

    ivp_synapse_attach_friction_contact(new_pc);

    {
        ivp_friction_event_t event = make_friction_event(&new_pc->cp, new_pc->now_friction_pressure);
        ivp_collision_listeners_fire_friction_created(new_pc->cp.obj_a, new_pc->cp.obj_b, &event);
    }

    return new_pc;
}

/* ── Pruning ────────────────────────────────────────────────────────────── */

void ivp_friction_manager_age_and_prune(ivp_friction_manager_t *mgr) {
    if (!mgr) return;

    for (int i = 0; i < mgr->system_count; ) {
        ivp_friction_system_t *sys = mgr->systems[i];

        ivp_persistent_contact_t *pc = sys->first_contact;
        while (pc) {
            ivp_persistent_contact_t *next = pc->next;

            if (!pc->seen_this_psi || pc->friction_is_broken) {
                ivp_friction_event_t event = make_friction_event(&pc->cp, pc->now_friction_pressure);
                ivp_collision_listeners_fire_friction_deleted(pc->cp.obj_a, pc->cp.obj_b, &event);

                for (int j = 0; j < sys->pair_count; j++) {
                    pair_remove_contact(&sys->pairs[j], pc);
                }

                /* Remove from doubly-linked list */
                if (pc->prev) {
                    pc->prev->next = pc->next;
                } else {
                    sys->first_contact = pc->next;
                }
                if (pc->next) {
                    pc->next->prev = pc->prev;
                }

                ivp_synapse_detach_friction_contact(pc);

                sys->contact_count--;

                pc->next = mgr->free_contacts;
                pc->prev = NULL;
                mgr->free_contacts = pc;
            }

            pc = next;
        }

        system_compact_empty_pairs(sys);

        /* Check for split or removal after pruning contacts.
         * check_split handles the empty-system case and also detects
         * disconnected components (cores no longer linked by contacts). */
        int prev_count = mgr->system_count;
        ivp_friction_system_check_split(sys, mgr);
        if (mgr->system_count != prev_count) {
            /* System was split or removed. Mark involved sim units for
             * union_find so they can detect disconnected components and
             * split at the next PSI.
             * Reference: ivp_friction.cxx line 1212/1219 */
            for (int s = i; s < mgr->system_count; s++) {
                ivp_sim_unit_t *su = friction_system_find_sim_unit(mgr->systems[s]);
                if (su) su->union_find_needed = true;
            }
            if (mgr->system_count < prev_count) {
                /* sys was removed — don't advance i */
                continue;
            }
        }

        i++;
    }
}

/* ── Distance keeper pre-pass ───────────────────────────────────────────── */

/* Independent per-contact pushes targeting keeper_dist.
 * Legacy: do_pushes_distance_keepers runs before calc_solver_PSI.
 * Each contact is pushed independently — no cross-coupling. */

void ivp_friction_system_push_distance_keepers(ivp_friction_system_t *sys,
                                                 const ivp_collision_settings_t *settings,
                                                 ivp_real_t dt) {
    if (!sys || !settings || dt <= IVP_REAL(0.0)) return;
    if (sys->contact_count == 0) return;
#ifdef IVP_DEBUG_DISABLE_COUPLED
    return;
#endif

    ivp_real_t i_dt = IVP_REAL(1.0) / dt;

    ivp_persistent_contact_t *pc = sys->first_contact;
    while (pc) {
        /* Skip contacts beyond keeper distance */
        if (pc->cp.gap_length > settings->keeper_dist) {
            pc = pc->next;
            continue;
        }

        /* Legacy ramp semantics:
         * speedup = (IVP_SLOWLY_TURN_ON_KEEPER - slowly_turn_on_keeper)
         *           / IVP_SLOWLY_TURN_ON_KEEPER * i_delta_time */
        int countdown = pc->slowly_turn_on_keeper;
        if (countdown < 0) countdown = 0;
        if (countdown > IVP_SLOWLY_TURN_ON_KEEPER) {
            countdown = IVP_SLOWLY_TURN_ON_KEEPER;
        }

        ivp_real_t speedup = (ivp_real_t)(IVP_SLOWLY_TURN_ON_KEEPER - countdown)
                           * ((IVP_REAL(1.0) / (ivp_real_t)IVP_SLOWLY_TURN_ON_KEEPER) * i_dt);

        ivp_real_t closing_speed = ivp_contact_closing_speed(&pc->cp);
        ivp_real_t gap_diff = settings->keeper_dist - pc->cp.gap_length;
        ivp_real_t impulse = (closing_speed + gap_diff * speedup) * pc->cp.virtual_mass;

        if (impulse > IVP_REAL(0.0)) {
            pc->now_friction_pressure = impulse * i_dt;
            ivp_contact_apply_impulse(&pc->cp, impulse);
        } else {
            pc->now_friction_pressure = IVP_REAL(0.0);
        }

        if (pc->slowly_turn_on_keeper > 0) {
            pc->slowly_turn_on_keeper--;
        } else {
            pc->slowly_turn_on_keeper = 0;
        }

        pc = pc->next;
    }
}

/* ── Friction solver ────────────────────────────────────────────────────── */

/* Compute the coupling coefficient between two contacts sharing a core.
 *
 * When a unit impulse is applied at contact j along normal_j, this
 * computes how much the closing speed at contact i changes, considering
 * the shared core's translational and rotational response.
 *
 * Returns: speed change at contact i per unit impulse at contact j,
 * through the single core C. */
static ivp_real_t compute_core_coupling(
    const ivp_core_t *core,
    const ivp_vec3r_t *r_j_world,   /* arm from core center to contact j */
    const ivp_vec3r_t *n_j,         /* normal at contact j */
    const ivp_vec3r_t *r_i_world,   /* arm from core center to contact i */
    const ivp_vec3r_t *n_i)         /* normal at contact i */
{
    if (core->physical_unmoveable) return IVP_REAL(0.0);

    /* Linear part: inv_mass * dot(n_j, n_i) */
    ivp_real_t linear = core->inv_mass * ivp_vec3r_dot(n_j, n_i);

    /* Angular part:
     *   1. cross_j_ws = r_j × n_j
     *   2. Transform to core space
     *   3. Apply diagonal inverse inertia
     *   4. Transform back to world space
     *   5. cross_i_ws = r_i × n_i
     *   6. Angular coupling = dot(omega_ws, cross_i_ws)
     */
    ivp_vec3r_t cross_j_ws;
    ivp_vec3r_cross(&cross_j_ws, r_j_world, n_j);

    ivp_vec3r_t cross_j_cs;
    ivp_mat3r_vimult(&cross_j_cs, &core->m_world_f_core_last_psi.rot, &cross_j_ws);

    ivp_vec3r_t omega_cs;
    omega_cs.k[0] = cross_j_cs.k[0] * core->inv_rot_inertia.k[0];
    omega_cs.k[1] = cross_j_cs.k[1] * core->inv_rot_inertia.k[1];
    omega_cs.k[2] = cross_j_cs.k[2] * core->inv_rot_inertia.k[2];

    ivp_vec3r_t omega_ws;
    ivp_mat3r_vmult(&omega_ws, &core->m_world_f_core_last_psi.rot, &omega_cs);

    ivp_vec3r_t cross_i_ws;
    ivp_vec3r_cross(&cross_i_ws, r_i_world, n_i);

    ivp_real_t angular = ivp_vec3r_dot(&omega_ws, &cross_i_ws);

    return linear + angular;
}

/* Compute distance-change matrix element M[i][j]:
 * How much closing speed at contact i changes from unit impulse at contact j.
 *
 * Sign convention: same-side (a-a or b-b) = +1, opposite-side (a-b) = -1.
 * This arises because impulse at j pushes obj_a of j in -normal_j direction
 * and obj_b of j in +normal_j direction, while closing speed at i involves
 * v_b - v_a at contact i. */
static ivp_real_t compute_matrix_element(
    const ivp_persistent_contact_t *ci,
    const ivp_persistent_contact_t *cj)
{
    ivp_real_t m_ij = IVP_REAL(0.0);
    const ivp_core_t *ai = &ci->cp.obj_a->core;
    const ivp_core_t *bi = &ci->cp.obj_b->core;
    const ivp_core_t *aj = &cj->cp.obj_a->core;
    const ivp_core_t *bj = &cj->cp.obj_b->core;

    /* a-a: same side → +1 */
    if (ai == aj) {
        m_ij += compute_core_coupling(ai,
            &cj->cp.r_a, &cj->cp.normal,
            &ci->cp.r_a, &ci->cp.normal);
    }
    /* b-b: same side → +1 */
    if (bi == bj) {
        m_ij += compute_core_coupling(bi,
            &cj->cp.r_b, &cj->cp.normal,
            &ci->cp.r_b, &ci->cp.normal);
    }
    /* a-b: opposite side → -1 */
    if (ai == bj) {
        m_ij -= compute_core_coupling(ai,
            &cj->cp.r_b, &cj->cp.normal,
            &ci->cp.r_a, &ci->cp.normal);
    }
    /* b-a: opposite side → -1 */
    if (bi == aj) {
        m_ij -= compute_core_coupling(bi,
            &cj->cp.r_a, &cj->cp.normal,
            &ci->cp.r_b, &ci->cp.normal);
    }

    return m_ij;
}

/* PGS fallback for oversized systems (> IVP_MAX_FRICTION_MATRIX contacts).
 * Supports SOR (Successive Over-Relaxation) and adaptive iteration count
 * when solver config is active and those features are enabled. */

static void friction_solve_pgs(ivp_persistent_contact_t **contacts, int n,
                               const ivp_real_t *gap_correction,
                               const ivp_collision_settings_t *settings,
                               ivp_real_t dt,
                               const ivp_allocator_t *alloc)
{
    ivp_real_t stack_accumulated[IVP_MAX_FRICTION_MATRIX];
    ivp_real_t *accumulated = stack_accumulated;
    const size_t accum_size = (size_t)n * sizeof(ivp_real_t);

    if (n > IVP_MAX_FRICTION_MATRIX) {
        accumulated = (ivp_real_t *)ivp_alloc(alloc, accum_size);
        if (!accumulated) {
            for (int i = 0; i < n; i++) {
                contacts[i]->now_friction_pressure = IVP_REAL(0.0);
            }
            return;
        }
    }

    for (int i = 0; i < n; i++) accumulated[i] = IVP_REAL(0.0);

    /* Read config for SOR omega and adaptive iteration limits */
    const ivp_solver_config_t *cfg = ivp_solver_config_get_active();
    int base_iters = IVP_PGS_BASE_ITERATIONS;
    int max_iters  = IVP_PGS_MAX_ITERATIONS;
    ivp_real_t sor_omega = IVP_REAL(1.0);
    ivp_real_t residual_eps = IVP_PGS_RESIDUAL_EPS;
    bool adaptive = false;

    if (cfg) {
        base_iters = cfg->pgs_base_iterations;
        max_iters  = cfg->enable_adaptive_pgs ? cfg->pgs_max_iterations : base_iters;
        sor_omega  = cfg->pgs_sor_omega;
        residual_eps = cfg->pgs_residual_eps;
        adaptive   = cfg->enable_adaptive_pgs;
    }

    int iters_used = 0;
    ivp_real_t final_max_residual = IVP_REAL(0.0);

    for (int iter = 0; iter < max_iters; iter++) {
        ivp_real_t max_residual = IVP_REAL(0.0);

        for (int i = 0; i < n; i++) {
            ivp_real_t cs = ivp_contact_closing_speed(&contacts[i]->cp);
            ivp_real_t residual = cs + gap_correction[i];
            ivp_real_t delta_impulse = residual * contacts[i]->cp.virtual_mass;

            /* D1: SOR relaxation */
            delta_impulse *= sor_omega;

            ivp_real_t new_total = accumulated[i] + delta_impulse;
            if (new_total < IVP_REAL(0.0)) new_total = IVP_REAL(0.0);

            ivp_real_t max_impulse = contacts[i]->cp.virtual_mass
                                   * settings->keeper_dist / dt;
            if (new_total > max_impulse) new_total = max_impulse;

            ivp_real_t actual_delta = new_total - accumulated[i];
            accumulated[i] = new_total;

            ivp_real_t abs_residual = ivp_fabs(residual);
            if (abs_residual > max_residual) max_residual = abs_residual;

            if (ivp_fabs(actual_delta) > IVP_REAL_EPS) {
                ivp_contact_apply_impulse(&contacts[i]->cp, actual_delta);
            }
        }

        iters_used = iter + 1;
        final_max_residual = max_residual;

        /* D2: Adaptive early exit after base iterations */
        if (adaptive && iter >= base_iters - 1) {
            if (max_residual < residual_eps) {
                break;
            }
        }
    }

    for (int i = 0; i < n; i++) {
        contacts[i]->now_friction_pressure = accumulated[i] / dt;
    }

    /* B2: Record PGS metrics */
    {
        ivp_solver_metrics_t *met = ivp_solver_metrics_get_active();
        if (met) {
            met->pgs_iterations_used += iters_used;
            if (final_max_residual > met->pgs_final_residual) {
                met->pgs_final_residual = final_max_residual;
            }
        }
    }

    if (accumulated != stack_accumulated) {
        ivp_free(alloc, accumulated, accum_size);
    }
}

static void set_now_pressure_from_impulse(ivp_persistent_contact_t *pc,
                                          ivp_real_t impulse,
                                          ivp_real_t dt)
{
    if (impulse > IVP_REAL_EPS) {
        pc->now_friction_pressure = impulse / dt;
    } else {
        pc->now_friction_pressure = IVP_REAL(0.0);
    }
}

static void unlink_contact_from_system(ivp_friction_system_t *sys,
                                       ivp_persistent_contact_t *pc)
{
    if (!sys || !pc || !sys->manager) return;

    ivp_friction_event_t event = make_friction_event(&pc->cp, pc->now_friction_pressure);
    ivp_collision_listeners_fire_friction_deleted(pc->cp.obj_a, pc->cp.obj_b, &event);

    for (int j = 0; j < sys->pair_count; j++) {
        pair_remove_contact(&sys->pairs[j], pc);
    }

    if (pc->prev) {
        pc->prev->next = pc->next;
    } else {
        sys->first_contact = pc->next;
    }
    if (pc->next) {
        pc->next->prev = pc->prev;
    }

    ivp_synapse_detach_friction_contact(pc);

    sys->contact_count--;
    pc->next = sys->manager->free_contacts;
    pc->prev = NULL;
    sys->manager->free_contacts = pc;
}

static int refresh_and_prune_broken_contacts(ivp_friction_system_t *sys,
                                             const ivp_collision_settings_t *settings)
{
    if (!sys || !settings) return 0;
    int removed = 0;
    ivp_persistent_contact_t *pc = sys->first_contact;
    while (pc) {
        ivp_persistent_contact_t *next = pc->next;
        if (!ivp_mindist_friction_refresh_contact(pc, settings)) {
            unlink_contact_from_system(sys, pc);
            removed++;
        }
        pc = next;
    }
    if (removed > 0) {
        system_compact_empty_pairs(sys);
    }
    return removed;
}

static int collect_eligible_contacts(ivp_friction_system_t *sys,
                                     const ivp_collision_settings_t *settings,
                                     ivp_persistent_contact_t **out_contacts,
                                     int max_contacts)
{
    int n = 0;
    ivp_persistent_contact_t *pc = sys->first_contact;
    while (pc && n < max_contacts) {
        if (!pc->friction_is_broken &&
            pc->cp.age_psi > 1 &&
            pc->cp.gap_length <= settings->keeper_dist)
        {
            pc->psi_scratch.index_in_fs = n;
            out_contacts[n++] = pc;
        } else {
            pc->psi_scratch.index_in_fs = -1;
        }
        pc = pc->next;
    }
    return n;
}

static void static_fr_oversized_matrix_panic(ivp_friction_system_t *sys)
{
    if (!sys) return;
    for (int i = 0; i < sys->core_count; i++) {
        ivp_core_t *core = sys->cores[i];
        if (!core || core->physical_unmoveable) continue;
        core->fast_piling_allowed = true;
    }

    for (int i = 0; i < sys->core_count; i++) {
        ivp_core_t *core = sys->cores[i];
        if (!core || core->physical_unmoveable) continue;

        int movable_partners = 0;
        for (int p = 0; p < sys->pair_count; p++) {
            ivp_friction_core_pair_t *pair = &sys->pairs[p];
            if (pair->contact_count <= 0) continue;
            if (pair->cores[0] != core && pair->cores[1] != core) continue;
            ivp_core_t *other = (pair->cores[0] == core) ? pair->cores[1] : pair->cores[0];
            if (other && !other->physical_unmoveable) {
                movable_partners++;
            }
        }

        if (movable_partners > 1) {
            ivp_vec3r_set_zero(&core->speed);
            core->rot_speed.k[0] = 0.0f;
            core->rot_speed.k[1] = 0.0f;
            core->rot_speed.k[2] = 0.0f;
            ivp_vec3r_set_zero(&core->speed_change);
            ivp_vec3r_set_zero(&core->rot_speed_change);
        }
    }
}

static bool validate_gauss_proposal(const ivp_great_matrix_t *gm,
                                    ivp_persistent_contact_t **contacts,
                                    const int *active_mask,
                                    const ivp_real_t *proposal,
                                    int n,
                                    const ivp_collision_settings_t *settings)
{
    ivp_real_t adhesion_guard = -IVP_MAX_ADHESION_GAUSS * settings->gravity_magnitude;

    for (int i = 0; i < n; i++) {
        if (active_mask[i]) {
            ivp_real_t lhs = proposal[i] * contacts[i]->cp.inv_virtual_mass;
            if (lhs < adhesion_guard) {
                return false;
            }
        }
    }

    for (int i = 0; i < n; i++) {
        if (active_mask[i]) continue;
        ivp_real_t residual = ivp_great_matrix_get_rhs(gm, i);
        for (int j = 0; j < n; j++) {
            residual -= ivp_great_matrix_get(gm, i, j) * proposal[j];
        }
        if (residual > IVP_SOLVER_TEST_EPS) {
            return false;
        }
    }
    return true;
}

static void async_apply_contact_impulse(ivp_persistent_contact_t *pc,
                                        ivp_real_t impulse)
{
    if (impulse <= IVP_REAL_EPS) return;
    ivp_vec3r_t n = pc->cp.normal;
    ivp_vec3r_t push_a = ivp_vec3r(-n.k[0] * impulse, -n.k[1] * impulse, -n.k[2] * impulse);
    ivp_vec3r_t push_b = ivp_vec3r( n.k[0] * impulse,  n.k[1] * impulse,  n.k[2] * impulse);
    ivp_core_async_push_ws(&pc->cp.obj_a->core, &pc->cp.point_a, &push_a);
    ivp_core_async_push_ws(&pc->cp.obj_b->core, &pc->cp.point_b, &push_b);
}

static ivp_result_t apply_normal_impulses_async(ivp_friction_system_t *sys,
                                                ivp_persistent_contact_t **contacts,
                                                ivp_real_t *impulses,
                                                int n,
                                                const ivp_collision_settings_t *settings,
                                                ivp_real_t dt)
{
    if (sys->core_count > IVP_FRICTION_MAX_CORES) return IVP_ERR_INVALID_ARG;

    ivp_vec3r_t saved_speed[IVP_FRICTION_MAX_CORES];
    ivp_vec3f_t saved_rot[IVP_FRICTION_MAX_CORES];
    ivp_vec3r_t saved_speed_change[IVP_FRICTION_MAX_CORES];
    ivp_vec3r_t saved_rot_change[IVP_FRICTION_MAX_CORES];
    for (int c = 0; c < sys->core_count; c++) {
        ivp_core_t *core = sys->cores[c];
        saved_speed[c] = core->speed;
        saved_rot[c] = core->rot_speed;
        saved_speed_change[c] = core->speed_change;
        saved_rot_change[c] = core->rot_speed_change;
    }

    ivp_real_t e_before = IVP_REAL(0.0);
    for (int c = 0; c < sys->core_count; c++) {
        e_before += ivp_core_get_energy(sys->cores[c]);
    }

    for (int i = 0; i < n; i++) {
        ivp_real_t impulse = impulses[i];
        if (impulse < IVP_REAL(0.0)) impulse = IVP_REAL(0.0);
        ivp_real_t max_impulse = contacts[i]->cp.virtual_mass * settings->keeper_dist / dt;
        if (impulse > max_impulse) impulse = max_impulse;
        impulses[i] = impulse;

        contacts[i]->last_normal_impulse = impulse;
        set_now_pressure_from_impulse(contacts[i], impulse, dt);
        async_apply_contact_impulse(contacts[i], impulse);
    }

    for (int c = 0; c < sys->core_count; c++) {
        ivp_core_commit_async_pushes(sys->cores[c]);
    }

    ivp_real_t e_after = IVP_REAL(0.0);
    ivp_real_t e_max_gain = IVP_REAL(0.0);
    for (int c = 0; c < sys->core_count; c++) {
        ivp_core_t *core = sys->cores[c];
        e_after += ivp_core_get_energy(core);
        if (!core->physical_unmoveable) {
            e_max_gain += core->mass * settings->gravity_magnitude * IVP_REAL(0.1);
        }
    }

    if (e_after > e_before + e_max_gain) {
        for (int c = 0; c < sys->core_count; c++) {
            ivp_core_t *core = sys->cores[c];
            core->speed = saved_speed[c];
            core->rot_speed = saved_rot[c];
            core->speed_change = saved_speed_change[c];
            core->rot_speed_change = saved_rot_change[c];
        }
        for (int i = 0; i < n; i++) {
            impulses[i] = IVP_REAL(0.0);
            contacts[i]->last_normal_impulse = IVP_REAL(0.0);
            contacts[i]->now_friction_pressure = IVP_REAL(0.0);
        }
    }

    return IVP_OK;
}

static ivp_result_t friction_system_solve_normal(ivp_friction_system_t *sys,
                                                 const ivp_collision_settings_t *settings,
                                                 ivp_real_t dt)
{
    int eligible_count = 0;
    ivp_persistent_contact_t *pc = sys->first_contact;
    while (pc) {
        if (pc->cp.age_psi > 1 && pc->cp.gap_length <= settings->keeper_dist && !pc->friction_is_broken) {
            eligible_count++;
        }
        pc = pc->next;
    }
    if (eligible_count <= 0) return IVP_OK;

    if (eligible_count > IVP_MAX_FRICTION_MATRIX) {
        static_fr_oversized_matrix_panic(sys);
        for (ivp_persistent_contact_t *c = sys->first_contact; c; c = c->next) {
            c->now_friction_pressure = IVP_REAL(0.0);
        }
        return IVP_OK;
    }

    int n = eligible_count;
    ivp_persistent_contact_t *contacts[IVP_MAX_FRICTION_MATRIX];
    ivp_real_t gap_correction[IVP_MAX_FRICTION_MATRIX];
    ivp_real_t result[IVP_MAX_FRICTION_MATRIX];
    int active_initial[IVP_MAX_FRICTION_MATRIX];
    memset(result, 0, sizeof(result));

    n = collect_eligible_contacts(sys, settings, contacts, IVP_MAX_FRICTION_MATRIX);
    if (n <= 0) return IVP_OK;

    {
        const ivp_solver_config_t *cfg = ivp_solver_config_get_active();
        if (cfg && cfg->enable_shock_propagation) {
            ivp_contact_order_shock_propagation(contacts, n, &cfg->gravity_dir);
        }
    }

    for (int i = 0; i < n; i++) {
        ivp_real_t gap_diff = settings->friction_dist - contacts[i]->cp.gap_length;
        ivp_real_t speedup = IVP_REAL(1.0);
        if (gap_diff < IVP_REAL(0.0)) speedup *= IVP_DESCENDING_SPEEDUP;
        gap_correction[i] = gap_diff * speedup;
        active_initial[i] = (contacts[i]->has_negative_pull_since != 0) ? 1 : 0;
    }

    ivp_great_matrix_t gm;
    ivp_result_t gm_res = ivp_great_matrix_init(&gm, n, sys->alloc);
    if (gm_res != IVP_OK) {
        friction_solve_pgs(contacts, n, gap_correction, settings, dt, sys->alloc);
        return IVP_OK;
    }

    for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) {
            ivp_real_t v = compute_matrix_element(contacts[i], contacts[j]);
            ivp_great_matrix_set(&gm, i, j, v);
            if (i != j) ivp_great_matrix_set(&gm, j, i, v);
        }
        ivp_real_t desired = ivp_contact_closing_speed(&contacts[i]->cp) + gap_correction[i];
        ivp_great_matrix_set_rhs(&gm, i, desired);
    }

    bool gauss_valid = false;
    if (ivp_great_matrix_solve_active_subset(&gm, active_initial, result, sys->alloc) == IVP_OK) {
        gauss_valid = validate_gauss_proposal(&gm, contacts, active_initial, result, n, settings);
    }
    if (!gauss_valid) {
        ivp_result_t uni_res = ivp_great_matrix_solve_unilateral(&gm, result, active_initial, sys->alloc);
        if (uni_res != IVP_OK) {
            friction_solve_pgs(contacts, n, gap_correction, settings, dt, sys->alloc);
            ivp_great_matrix_destroy(&gm, sys->alloc);
            return IVP_OK;
        }
    }
    ivp_great_matrix_destroy(&gm, sys->alloc);

    for (int i = 0; i < n; i++) {
        if (result[i] > IVP_REAL_EPS) {
            if (contacts[i]->has_negative_pull_since >= 0) contacts[i]->has_negative_pull_since = -1;
            else contacts[i]->has_negative_pull_since--;
        } else if (result[i] < -IVP_REAL_EPS) {
            contacts[i]->has_negative_pull_since++;
            if (contacts[i]->has_negative_pull_since > 9) contacts[i]->has_negative_pull_since = 0;
            result[i] = IVP_REAL(0.0);
        } else {
            contacts[i]->has_negative_pull_since = 0;
        }
    }

    return apply_normal_impulses_async(sys, contacts, result, n, settings, dt);
}

ivp_result_t ivp_friction_system_phase_static_normal(ivp_friction_system_t *sys,
                                                     const ivp_collision_settings_t *settings,
                                                     ivp_real_t dt)
{
    if (!sys || !settings || dt <= IVP_REAL(0.0)) return IVP_ERR_INVALID_ARG;
    if (sys->contact_count == 0) return IVP_OK;
#ifdef IVP_DEBUG_DISABLE_COUPLED
    return IVP_OK;
#endif
    for (ivp_persistent_contact_t *pc = sys->first_contact; pc; pc = pc->next) {
        pc->cp.age_psi++;
    }
    ivp_friction_system_push_distance_keepers(sys, settings, dt);
    return friction_system_solve_normal(sys, settings, dt);
}

ivp_result_t ivp_friction_system_solve_coupled(ivp_friction_system_t *sys,
                                                const ivp_collision_settings_t *settings,
                                                ivp_real_t dt)
{
    if (!sys || !settings || dt <= IVP_REAL(0.0)) return IVP_ERR_INVALID_ARG;
    ivp_friction_system_phase_energy(sys, settings, dt);
    ivp_result_t r = ivp_friction_system_phase_static_normal(sys, settings, dt);
    if (r != IVP_OK) return r;
    ivp_friction_system_phase_dynamic_tangential(sys, settings, dt);
    return IVP_OK;
}

static ivp_real_t calc_possible_friction_slide_way(const ivp_persistent_contact_t *pc)
{
    return pc->now_friction_pressure * pc->cp.friction * pc->inv_virt_mass_mindist_no_dir;
}

static void calc_pretension(ivp_persistent_contact_t *pc,
                            ivp_real_t max_local_spring_len)
{
    ivp_real_t qlen = pc->span_friction_s[0] * pc->span_friction_s[0] +
                      pc->span_friction_s[1] * pc->span_friction_s[1];
    ivp_real_t max_q = max_local_spring_len * max_local_spring_len;
    if (qlen <= max_q + IVP_REAL_EPS) return;

    ivp_real_t inv_len = (ivp_real_t)ivp_isqrt_float((float)qlen);
    ivp_real_t len = qlen * inv_len;
    ivp_real_t scale = max_local_spring_len * inv_len;
    if (scale < IVP_REAL(0.0)) scale = IVP_REAL(0.0);
    pc->span_friction_s[0] *= scale;
    pc->span_friction_s[1] *= scale;
    pc->integrated_destroyed_energy +=
        (len - max_local_spring_len) * pc->cp.friction * pc->now_friction_pressure;
    pc->cp_status = IVP_CP_STATUS_NEEDS_RECHECK;
}

static void two_values_friction_clip(ivp_persistent_contact_t *pc)
{
    if (!pc->two_friction_values) return;
    ivp_real_t scale = pc->two_friction_x_scale;
    if (scale > IVP_REAL(1.0)) scale = IVP_REAL(1.0);
    if (scale < IVP_REAL_EPS) scale = IVP_REAL_EPS;
    ivp_real_t max_x_len = IVP_MINIMAL_REAL_FRICTION_LEN * scale;
    if (pc->span_friction_s[0] > max_x_len) pc->span_friction_s[0] = max_x_len;
    if (pc->span_friction_s[0] < -max_x_len) pc->span_friction_s[0] = -max_x_len;
}

static bool should_use_wheel_branch(const ivp_friction_core_pair_t *pair)
{
    if (!pair || pair->contact_count != 1) return false;
    ivp_persistent_contact_t *pc = pair->contacts[0];
    if (!pc) return false;
    return pc->cp.obj_b->core.physical_unmoveable;
}

static void reproject_wheel_span(ivp_persistent_contact_t *pc)
{
    ivp_vec3r_t axis_ws;
    axis_ws.k[0] = pc->cp.obj_a->core.m_world_f_core_last_psi.rot.rows[0].k[0];
    axis_ws.k[1] = pc->cp.obj_a->core.m_world_f_core_last_psi.rot.rows[1].k[0];
    axis_ws.k[2] = pc->cp.obj_a->core.m_world_f_core_last_psi.rot.rows[2].k[0];
    ivp_real_t axis_dot_n = ivp_vec3r_dot(&axis_ws, &pc->cp.normal);
    ivp_vec3r_add_scaled(&axis_ws, &pc->cp.normal, -axis_dot_n);
    ivp_real_t axis_len = ivp_vec3r_length(&axis_ws);
    if (axis_len < IVP_REAL_EPS) return;
    ivp_vec3r_scale(&axis_ws, &axis_ws, IVP_REAL(1.0) / axis_len);
    pc->cp.span_dirs[1] = axis_ws;
    ivp_vec3r_cross(&pc->cp.span_dirs[0], &pc->cp.span_dirs[1], &pc->cp.normal);
    ivp_vec3r_normalize(&pc->cp.span_dirs[0]);
}

static ivp_real_t friction_force_local_constraint_2d(ivp_persistent_contact_t *pc,
                                                      ivp_real_t dt)
{
    if (pc->cp.friction < IVP_REAL_EPS || pc->now_friction_pressure < IVP_REAL_EPS) return IVP_REAL(0.0);

    ivp_vec3r_t v_rel;
    ivp_contact_relative_velocity(&pc->cp, &v_rel);

    ivp_real_t impulse_s[2];
    for (int axis = 0; axis < 2; axis++) {
        ivp_real_t v_tangent = ivp_vec3r_dot(&v_rel, &pc->cp.span_dirs[axis]);
        impulse_s[axis] = pc->span_friction_s[axis] + (-v_tangent * pc->cp.virtual_mass);
    }

    ivp_real_t maximum_impulse = pc->now_friction_pressure * pc->cp.friction * dt;
    ivp_real_t sq_impulse = impulse_s[0] * impulse_s[0] + impulse_s[1] * impulse_s[1];
    if (sq_impulse > maximum_impulse * maximum_impulse) {
        ivp_real_t scale = maximum_impulse * (ivp_real_t)ivp_isqrt_float((float)sq_impulse);
        impulse_s[0] *= scale;
        impulse_s[1] *= scale;
    }

    for (int axis = 0; axis < 2; axis++) {
        ivp_real_t delta = impulse_s[axis] - pc->span_friction_s[axis];
        pc->span_friction_s[axis] = impulse_s[axis];
        if (ivp_fabs(delta) > IVP_REAL_EPS) {
            ivp_contact_apply_impulse_dir(&pc->cp, &pc->cp.span_dirs[axis], delta);
        }
    }

    ivp_real_t spring_q = pc->span_friction_s[0] * pc->span_friction_s[0] +
                          pc->span_friction_s[1] * pc->span_friction_s[1];
    ivp_real_t impulse_q = impulse_s[0] * impulse_s[0] + impulse_s[1] * impulse_s[1];
    ivp_real_t e = IVP_REAL(0.5) * ivp_sqrt(impulse_q * dt * dt * spring_q);
    ivp_real_t delta_e = e - pc->old_energy_dynamic_fr;
    pc->old_energy_dynamic_fr = e;
    return delta_e;
}

static void friction_force_local_constraint_1d(ivp_persistent_contact_t *pc,
                                               ivp_real_t dt)
{
    ivp_vec3r_t spring_ws;
    spring_ws.k[0] = pc->span_friction_s[0] * pc->cp.span_dirs[0].k[0] +
                     pc->span_friction_s[1] * pc->cp.span_dirs[1].k[0];
    spring_ws.k[1] = pc->span_friction_s[0] * pc->cp.span_dirs[0].k[1] +
                     pc->span_friction_s[1] * pc->cp.span_dirs[1].k[1];
    spring_ws.k[2] = pc->span_friction_s[0] * pc->cp.span_dirs[0].k[2] +
                     pc->span_friction_s[1] * pc->cp.span_dirs[1].k[2];

    ivp_real_t spring_len = ivp_vec3r_length(&spring_ws);
    if (spring_len < IVP_REAL_EPS) return;
    ivp_vec3r_scale(&spring_ws, &spring_ws, IVP_REAL(1.0) / spring_len);

    ivp_vec3r_t v_rel;
    ivp_contact_relative_velocity(&pc->cp, &v_rel);
    ivp_real_t now_speed = ivp_vec3r_dot(&v_rel, &spring_ws);
    ivp_real_t desired_speed = spring_len / dt;
    ivp_real_t diff_speed = desired_speed - now_speed;
    ivp_real_t push_val = diff_speed * pc->cp.virtual_mass;
    ivp_real_t maximum_impulse = pc->now_friction_pressure * pc->cp.friction * dt;
    if (push_val > maximum_impulse) push_val = maximum_impulse;
    if (push_val < -maximum_impulse) push_val = -maximum_impulse;
    ivp_contact_apply_impulse_dir(&pc->cp, &spring_ws, push_val);
}

void ivp_friction_system_phase_dynamic_tangential(ivp_friction_system_t *sys,
                                                  const ivp_collision_settings_t *settings,
                                                  ivp_real_t dt)
{
    if (!sys || !settings || dt <= IVP_REAL(0.0)) return;
    if (sys->contact_count == 0) return;

    for (int p = 0; p < sys->pair_count; p++) {
        ivp_friction_core_pair_t *pair = &sys->pairs[p];
        if (pair->contact_count <= 0) continue;

        ivp_real_t max_local_spring_len = IVP_REAL(0.0);
        for (int j = 0; j < pair->contact_count; j++) {
            ivp_persistent_contact_t *pc = pair->contacts[j];
            max_local_spring_len += calc_possible_friction_slide_way(pc);
        }
        max_local_spring_len *= dt * dt;

        if (should_use_wheel_branch(pair)) {
            reproject_wheel_span(pair->contacts[0]);
        }

        ivp_real_t sum_delta_energy = IVP_REAL(0.0);
        for (int j = 0; j < pair->contact_count; j++) {
            ivp_persistent_contact_t *pc = pair->contacts[j];
            calc_pretension(pc, max_local_spring_len);
            two_values_friction_clip(pc);
            if (pc->two_friction_values) {
                friction_force_local_constraint_1d(pc, dt);
            } else {
                sum_delta_energy += friction_force_local_constraint_2d(pc, dt);
            }
        }
        if (sum_delta_energy > IVP_REAL(0.0)) {
            pair->integrated_anti_energy += sum_delta_energy;
        }
    }

    ivp_friction_system_ease(sys);

    {
        const ivp_solver_config_t *ps_cfg = ivp_solver_config_get_active();
        if (ps_cfg && ps_cfg->enable_post_stabilization) {
            ivp_friction_system_post_stabilize(sys, settings, dt);
        }
    }
}

void ivp_friction_system_phase_energy(ivp_friction_system_t *sys,
                                      const ivp_collision_settings_t *settings,
                                      ivp_real_t dt)
{
    (void)dt;
    if (!sys || !settings) return;
    refresh_and_prune_broken_contacts(sys, settings);
    if (sys->contact_count > 0) {
        ivp_friction_system_remove_gained_energy(sys);
    }
}

/* ── Post-stabilization (Baumgarte position correction) ────────────────── */

void ivp_friction_system_post_stabilize(ivp_friction_system_t *sys,
                                         const ivp_collision_settings_t *settings,
                                         ivp_real_t dt)
{
    if (!sys || !settings || dt <= IVP_REAL(0.0)) return;

    const ivp_solver_config_t *cfg = ivp_solver_config_get_active();
    if (!cfg) return;

    ivp_real_t factor = cfg->post_stabilize_factor;
    ivp_real_t max_impulse = cfg->post_stabilize_max_impulse;

    /* One PGS-style pass targeting penetration depth directly.
     * For each contact with penetration (gap < 0), apply a corrective
     * impulse proportional to the penetration depth. */
    ivp_persistent_contact_t *pc = sys->first_contact;
    while (pc) {
        if (pc->cp.age_psi > 1 && pc->cp.gap_length < IVP_REAL(0.0)) {
            /* Penetration depth (positive when penetrating) */
            ivp_real_t depth = -pc->cp.gap_length;

            /* Baumgarte correction impulse:
             * J = factor * depth * virtual_mass / dt
             * This pushes objects apart proportional to overlap. */
            ivp_real_t correction = factor * depth * pc->cp.virtual_mass / dt;

            /* Clamp */
            if (correction > max_impulse) correction = max_impulse;

            if (correction > IVP_REAL_EPS) {
                ivp_contact_apply_impulse(&pc->cp, correction);
            }
        }
        pc = pc->next;
    }
}

/* ── Easing ─────────────────────────────────────────────────────────────── */

static void ease_friction_pair(ivp_friction_core_pair_t *pair)
{
    if (!pair || pair->contact_count < 2) return;
    const int n = pair->contact_count;
    ivp_vec3r_t ease_diff[IVP_MAX_FRICTION_MATRIX];
    if (n > IVP_MAX_FRICTION_MATRIX) return;
    for (int i = 0; i < n; i++) ivp_vec3r_set_zero(&ease_diff[i]);

    ivp_real_t easing_factor = IVP_REAL(1.0) / ((ivp_real_t)n + IVP_REAL_EPS);
    for (int i = 0; i < n; i++) {
        ivp_persistent_contact_t *a = pair->contacts[i];
        for (int j = i + 1; j < n; j++) {
            ivp_persistent_contact_t *b = pair->contacts[j];
            ivp_real_t n_dot = ivp_vec3r_dot(&a->cp.normal, &b->cp.normal);
            if (ivp_fabs(ivp_fabs(n_dot) - IVP_REAL(1.0)) > IVP_REAL(1e-3)) continue;

            ivp_vec3r_t conn;
            ivp_vec3r_sub(&conn, &b->cp.point_a, &a->cp.point_a);
            ivp_real_t conn_len = ivp_vec3r_length(&conn);
            if (conn_len < IVP_REAL_EPS) continue;
            ivp_vec3r_scale(&conn, &conn, IVP_REAL(1.0) / conn_len);

            ivp_vec3r_t spring_a = ivp_vec3r(
                a->span_friction_s[0] * a->cp.span_dirs[0].k[0] + a->span_friction_s[1] * a->cp.span_dirs[1].k[0],
                a->span_friction_s[0] * a->cp.span_dirs[0].k[1] + a->span_friction_s[1] * a->cp.span_dirs[1].k[1],
                a->span_friction_s[0] * a->cp.span_dirs[0].k[2] + a->span_friction_s[1] * a->cp.span_dirs[1].k[2]);
            ivp_vec3r_t spring_b = ivp_vec3r(
                b->span_friction_s[0] * b->cp.span_dirs[0].k[0] + b->span_friction_s[1] * b->cp.span_dirs[1].k[0],
                b->span_friction_s[0] * b->cp.span_dirs[0].k[1] + b->span_friction_s[1] * b->cp.span_dirs[1].k[1],
                b->span_friction_s[0] * b->cp.span_dirs[0].k[2] + b->span_friction_s[1] * b->cp.span_dirs[1].k[2]);

            ivp_real_t proj_a = ivp_vec3r_dot(&spring_a, &conn);
            ivp_real_t proj_b = ivp_vec3r_dot(&spring_b, &conn);
            ivp_real_t optimal = (proj_a + proj_b) * IVP_REAL(0.5);

            ivp_vec3r_t diff_a = conn;
            ivp_vec3r_t diff_b = conn;
            ivp_vec3r_scale(&diff_a, &diff_a, (optimal - proj_a) * easing_factor);
            ivp_vec3r_scale(&diff_b, &diff_b, (optimal - proj_b) * easing_factor);
            ivp_vec3r_add(&ease_diff[i], &ease_diff[i], &diff_a);
            ivp_vec3r_add(&ease_diff[j], &ease_diff[j], &diff_b);
        }
    }

    for (int i = 0; i < n; i++) {
        ivp_persistent_contact_t *pc = pair->contacts[i];
        pc->span_friction_s[0] += ivp_vec3r_dot(&ease_diff[i], &pc->cp.span_dirs[0]);
        pc->span_friction_s[1] += ivp_vec3r_dot(&ease_diff[i], &pc->cp.span_dirs[1]);
    }
}

void ivp_friction_system_ease(ivp_friction_system_t *sys)
{
    if (!sys) return;
    for (int i = 0; i < sys->pair_count; i++) {
        ivp_friction_core_pair_t *pair = &sys->pairs[i];
        pair->next_ease_nr_psi--;
        if (pair->next_ease_nr_psi <= 0) {
            ease_friction_pair(pair);
            pair->next_ease_nr_psi = IVP_EASE_EVERY_NTH_PSI;
        }
    }
}

/* ── Energy removal ─────────────────────────────────────────────────────── */

static ivp_real_t average_inertia(const ivp_core_t *core)
{
    return (core->rot_inertia.k[0] + core->rot_inertia.k[1] + core->rot_inertia.k[2]) * IVP_REAL(0.33333334);
}

static void destroy_mutual_energy(ivp_friction_core_pair_t *pair, ivp_real_t d_e)
{
    if (!pair || d_e <= IVP_REAL_EPS) return;
    ivp_core_t *a = pair->cores[0];
    ivp_core_t *b = pair->cores[1];
    if (!a || !b || a->physical_unmoveable || b->physical_unmoveable) return;

    ivp_vec3r_t v_rel;
    ivp_vec3r_sub(&v_rel, &b->speed, &a->speed);
    ivp_real_t v2 = ivp_vec3r_quad_length(&v_rel);
    ivp_real_t mu = (a->mass * b->mass) / (a->mass + b->mass + IVP_REAL_EPS);
    ivp_real_t e_trans = IVP_REAL(0.5) * mu * v2;

    ivp_vec3r_t wa_cs = ivp_vec3r((ivp_real_t)a->rot_speed.k[0], (ivp_real_t)a->rot_speed.k[1], (ivp_real_t)a->rot_speed.k[2]);
    ivp_vec3r_t wb_cs = ivp_vec3r((ivp_real_t)b->rot_speed.k[0], (ivp_real_t)b->rot_speed.k[1], (ivp_real_t)b->rot_speed.k[2]);
    ivp_vec3r_t wa_ws, wb_ws, w_rel;
    ivp_mat3r_vmult(&wa_ws, &a->m_world_f_core_last_psi.rot, &wa_cs);
    ivp_mat3r_vmult(&wb_ws, &b->m_world_f_core_last_psi.rot, &wb_cs);
    ivp_vec3r_sub(&w_rel, &wb_ws, &wa_ws);
    ivp_real_t w2 = ivp_vec3r_quad_length(&w_rel);
    ivp_real_t ia = average_inertia(a);
    ivp_real_t ib = average_inertia(b);
    ivp_real_t i_mu = (ia * ib) / (ia + ib + IVP_REAL_EPS);
    ivp_real_t e_rot = IVP_REAL(0.5) * i_mu * w2;

    ivp_real_t whole = e_trans + e_rot;
    if (whole <= IVP_REAL_EPS) return;
    ivp_real_t capped = d_e;
    ivp_real_t max_d = IVP_MAX_ENERGY_DESTROY * whole;
    if (capped > max_d) capped = max_d;
    ivp_real_t percent = capped / whole;

    if (v2 > IVP_REAL_EPS) {
        ivp_vec3r_t dv = v_rel;
        ivp_vec3r_scale(&dv, &dv, percent);
        ivp_vec3r_add_scaled(&a->speed, &dv,  b->mass / (a->mass + b->mass + IVP_REAL_EPS));
        ivp_vec3r_add_scaled(&b->speed, &dv, -a->mass / (a->mass + b->mass + IVP_REAL_EPS));
    }
    if (w2 > IVP_REAL_EPS) {
        ivp_vec3r_t dw = w_rel;
        ivp_vec3r_scale(&dw, &dw, percent);
        ivp_vec3r_t da_ws = dw, db_ws = dw;
        ivp_vec3r_scale(&da_ws, &da_ws,  ib / (ia + ib + IVP_REAL_EPS));
        ivp_vec3r_scale(&db_ws, &db_ws, -ia / (ia + ib + IVP_REAL_EPS));
        ivp_vec3r_t da_cs, db_cs;
        ivp_mat3r_vimult(&da_cs, &a->m_world_f_core_last_psi.rot, &da_ws);
        ivp_mat3r_vimult(&db_cs, &b->m_world_f_core_last_psi.rot, &db_ws);
        a->rot_speed.k[0] += (float)da_cs.k[0];
        a->rot_speed.k[1] += (float)da_cs.k[1];
        a->rot_speed.k[2] += (float)da_cs.k[2];
        b->rot_speed.k[0] += (float)db_cs.k[0];
        b->rot_speed.k[1] += (float)db_cs.k[1];
        b->rot_speed.k[2] += (float)db_cs.k[2];
    }
}

void ivp_friction_system_remove_gained_energy(ivp_friction_system_t *sys)
{
    if (!sys) return;
    for (int i = 0; i < sys->pair_count; i++) {
        ivp_friction_core_pair_t *pair = &sys->pairs[i];
        if (pair->integrated_anti_energy > IVP_REAL_EPS) {
            destroy_mutual_energy(pair, pair->integrated_anti_energy);
            pair->integrated_anti_energy = IVP_REAL(0.0);
        }
    }
}

/* ── System merge ───────────────────────────────────────────────────────── */

void ivp_friction_system_merge(ivp_friction_system_t *dst,
                                 ivp_friction_system_t *src,
                                 ivp_friction_manager_t *mgr) {
    if (!dst || !src || !mgr || dst == src) return;
    dst->manager = mgr;

    /* Move all contacts from src to dst */
    if (src->first_contact) {
        ivp_persistent_contact_t *src_last = src->first_contact;
        while (src_last->next) {
            src_last = src_last->next;
        }

        src_last->next = dst->first_contact;
        if (dst->first_contact) {
            dst->first_contact->prev = src_last;
        }
        dst->first_contact = src->first_contact;
        dst->contact_count += src->contact_count;

        src->first_contact = NULL;
        src->contact_count = 0;
    }

    /* Move cores */
    for (int i = 0; i < src->core_count; i++) {
        system_add_core(dst, src->cores[i]);
    }

    /* Move pair membership and preserve pair-level state. */
    for (int i = 0; i < src->pair_count; i++) {
        ivp_friction_core_pair_t *src_pair = &src->pairs[i];
        ivp_friction_core_pair_t *dst_pair =
            get_or_create_pair(dst, src_pair->cores[0], src_pair->cores[1]);

        if (dst_pair) {
            bool dst_pair_was_empty = (dst_pair->contact_count == 0);
            for (int j = 0; j < src_pair->contact_count; j++) {
                pair_add_contact(dst_pair, src_pair->contacts[j]);
            }
            dst_pair->integrated_anti_energy += src_pair->integrated_anti_energy;
            if (dst_pair_was_empty) {
                dst_pair->next_ease_nr_psi = src_pair->next_ease_nr_psi;
            } else if (src_pair->next_ease_nr_psi < dst_pair->next_ease_nr_psi) {
                dst_pair->next_ease_nr_psi = src_pair->next_ease_nr_psi;
            }
        }
    }

    /* Remove src from manager */
    for (int i = 0; i < mgr->system_count; i++) {
        if (mgr->systems[i] == src) {
            /* Shift remaining systems down */
            for (int j = i; j < mgr->system_count - 1; j++) {
                mgr->systems[j] = mgr->systems[j + 1];
            }
            mgr->system_count--;
            break;
        }
    }

    /* Free src */
    friction_system_free_internals(src);
    ivp_free(mgr->alloc, src, sizeof(*src));
}

/* ── System split ───────────────────────────────────────────────────────── */

/* Remove a system from the manager's system array and free it. */
static void manager_remove_system(ivp_friction_manager_t *mgr,
                                    ivp_friction_system_t *sys) {
    /* Unregister per-unit controllers before destruction */
    friction_system_unregister_controllers(sys);

    for (int i = 0; i < mgr->system_count; i++) {
        if (mgr->systems[i] == sys) {
            for (int j = i; j < mgr->system_count - 1; j++) {
                mgr->systems[j] = mgr->systems[j + 1];
            }
            mgr->system_count--;
            friction_system_free_internals(sys);
            ivp_free(mgr->alloc, sys, sizeof(*sys));
            return;
        }
    }
}

/* Find the component label for a core using union-find path compression.
 * labels[i] stores the parent index; roots have labels[i] == i. */
static int uf_find(int *labels, int i) {
    while (labels[i] != i) {
        labels[i] = labels[labels[i]]; /* path halving */
        i = labels[i];
    }
    return i;
}

static void uf_union(int *labels, int a, int b) {
    a = uf_find(labels, a);
    b = uf_find(labels, b);
    if (a != b) {
        /* Always merge higher into lower for determinism */
        if (a > b) { int t = a; a = b; b = t; }
        labels[b] = a;
    }
}

void ivp_friction_system_check_split(ivp_friction_system_t *sys,
                                       ivp_friction_manager_t *mgr) {
    if (!sys || !mgr) return;

    /* Empty system: just remove it */
    if (sys->contact_count == 0) {
        manager_remove_system(mgr, sys);
        return;
    }

    /* Fewer than 2 cores: nothing to split */
    if (sys->core_count < 2) return;

    /* Union-find over cores to discover connected components.
     * Two cores are connected if they share at least one live contact. */
    int nc = sys->core_count;

    /* Stack-allocate for typical sizes, heap for large */
    int stack_labels[64];
    int *labels = (nc <= 64) ? stack_labels
                              : (int *)ivp_alloc(sys->alloc, (size_t)nc * sizeof(int));
    if (!labels) return;

    for (int i = 0; i < nc; i++) labels[i] = i;

    /* Build connectivity from live contacts in pairs */
    for (int p = 0; p < sys->pair_count; p++) {
        ivp_friction_core_pair_t *pair = &sys->pairs[p];
        if (pair->contact_count == 0) continue;

        /* Find indices of both cores */
        int idx0 = -1, idx1 = -1;
        for (int k = 0; k < nc; k++) {
            if (sys->cores[k] == pair->cores[0]) idx0 = k;
            if (sys->cores[k] == pair->cores[1]) idx1 = k;
        }
        if (idx0 >= 0 && idx1 >= 0) {
            uf_union(labels, idx0, idx1);
        }
    }

    /* Check if all cores belong to the same component */
    int root0 = uf_find(labels, 0);
    bool all_connected = true;
    for (int i = 1; i < nc; i++) {
        if (uf_find(labels, i) != root0) {
            all_connected = false;
            break;
        }
    }

    if (all_connected) {
        /* Remove orphaned cores that have no contacts at all */
        int write = 0;
        for (int i = 0; i < nc; i++) {
            bool has_contact = false;
            for (int p = 0; p < sys->pair_count && !has_contact; p++) {
                ivp_friction_core_pair_t *pair = &sys->pairs[p];
                if (pair->contact_count > 0 &&
                    (pair->cores[0] == sys->cores[i] || pair->cores[1] == sys->cores[i])) {
                    has_contact = true;
                }
            }
            if (has_contact) {
                sys->cores[write++] = sys->cores[i];
            }
        }
        sys->core_count = write;

        if (labels != stack_labels) ivp_free(sys->alloc, labels, (size_t)nc * sizeof(int));
        return;
    }

    /* Need to split: count distinct components.
     * Stack-allocate for typical sizes (≤64 cores), heap for larger. */
    int stack_comp_roots[64];
    int *comp_roots = (nc <= 64) ? stack_comp_roots
                                 : (int *)ivp_alloc(sys->alloc, (size_t)nc * sizeof(int));
    if (!comp_roots) {
        if (labels != stack_labels) ivp_free(sys->alloc, labels, (size_t)nc * sizeof(int));
        return;
    }

    int comp_count = 0;
    for (int i = 0; i < nc; i++) {
        int r = uf_find(labels, i);
        bool found = false;
        for (int c = 0; c < comp_count; c++) {
            if (comp_roots[c] == r) { found = true; break; }
        }
        if (!found) {
            comp_roots[comp_count++] = r;
        }
    }

    /* Keep the first component in the existing system;
     * create new systems for the rest. */
    int keep_root = comp_roots[0];

    for (int c = 1; c < comp_count; c++) {
        int split_root = comp_roots[c];

        /* Collect cores for this component */
        ivp_core_t *split_cores[2] = {NULL, NULL};
        int split_core_n = 0;
        for (int i = 0; i < nc && split_core_n < 2; i++) {
            if (uf_find(labels, i) == split_root) {
                split_cores[split_core_n++] = sys->cores[i];
            }
        }
        if (split_core_n < 1) continue;

        /* Need at least 2 cores for friction_system_create; use the first
         * twice if only one (second add will be a no-op duplicate check). */
        ivp_core_t *ca = split_cores[0];
        ivp_core_t *cb = split_core_n > 1 ? split_cores[1] : split_cores[0];

        ivp_friction_system_t *new_sys = friction_system_create(mgr, ca, cb);
        if (!new_sys) continue;

        /* Add any remaining cores beyond the first two */
        for (int i = 0; i < nc; i++) {
            if (uf_find(labels, i) == split_root) {
                system_add_core(new_sys, sys->cores[i]);
            }
        }

        /* Move contacts belonging to this component's core pairs */
        for (int p = 0; p < sys->pair_count; p++) {
            ivp_friction_core_pair_t *pair = &sys->pairs[p];
            if (pair->contact_count == 0) continue;

            /* Check if both cores belong to split_root */
            int idx0 = -1, idx1 = -1;
            for (int k = 0; k < nc; k++) {
                if (sys->cores[k] == pair->cores[0]) idx0 = k;
                if (sys->cores[k] == pair->cores[1]) idx1 = k;
            }
            if (idx0 < 0 || idx1 < 0) continue;
            if (uf_find(labels, idx0) != split_root) continue;

            /* Create pair in new system and move contacts */
            ivp_friction_core_pair_t *new_pair =
                get_or_create_pair(new_sys, pair->cores[0], pair->cores[1]);
            if (!new_pair) continue;

            for (int j = 0; j < pair->contact_count; j++) {
                ivp_persistent_contact_t *pc = pair->contacts[j];

                /* Remove from old system's linked list */
                if (pc->prev) pc->prev->next = pc->next;
                else sys->first_contact = pc->next;
                if (pc->next) pc->next->prev = pc->prev;
                sys->contact_count--;

                /* Add to new system's linked list */
                pc->prev = NULL;
                pc->next = new_sys->first_contact;
                if (new_sys->first_contact) new_sys->first_contact->prev = pc;
                new_sys->first_contact = pc;
                new_sys->contact_count++;

                pair_add_contact(new_pair, pc);
            }
            pair->contact_count = 0; /* emptied */
        }
    }

    /* Remove split-out cores from original system */
    {
        int write = 0;
        for (int i = 0; i < nc; i++) {
            if (uf_find(labels, i) == keep_root) {
                sys->cores[write++] = sys->cores[i];
            }
        }
        sys->core_count = write;
    }

    /* Compact empty pairs from original system */
    system_compact_empty_pairs(sys);

    /* If original system is now empty, remove it too */
    if (sys->contact_count == 0) {
        if (comp_roots != stack_comp_roots) ivp_free(sys->alloc, comp_roots, (size_t)nc * sizeof(int));
        if (labels != stack_labels) ivp_free(sys->alloc, labels, (size_t)nc * sizeof(int));
        manager_remove_system(mgr, sys);
        return;
    }

    if (comp_roots != stack_comp_roots) ivp_free(sys->alloc, comp_roots, (size_t)nc * sizeof(int));
    if (labels != stack_labels) ivp_free(sys->alloc, labels, (size_t)nc * sizeof(int));
}
