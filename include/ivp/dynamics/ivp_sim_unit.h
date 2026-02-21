/* ivp_sim_unit.h — Dynamic simulation island architecture
 *
 * A simulation unit groups objects that interact via controllers or friction
 * contacts. Objects merge into the same unit when connections form and split
 * when connections break. Each unit maintains its own sorted controller list
 * and runs an independent PSI sub-loop.
 *
 * Aligned with original IVP_Simulation_Unit / IVP_Sim_Units_Manager
 * (ivp_sim_unit.hxx / ivp_sim_unit.cxx).
 */
#ifndef IVP_SIM_UNIT_H
#define IVP_SIM_UNIT_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Forward declarations ──────────────────────────────────────────────── */

typedef struct ivp_sim_unit              ivp_sim_unit_t;
typedef struct ivp_sim_unit_core_entry   ivp_sim_unit_core_entry_t;
typedef struct ivp_controller_core       ivp_controller_core_t;
typedef struct ivp_sim_units_manager     ivp_sim_units_manager_t;

/* ── Controller core (per-unit controller entry) ───────────────────────── */

/* Callback for controller simulation within a sim unit PSI.
 * cores/core_count: the cores this controller acts on within the unit.
 * ctx: user data (typically the controller struct). */
typedef void (*ivp_controller_core_fn_t)(void *ctx,
                                          ivp_environment_t *env,
                                          ivp_core_t **cores,
                                          int core_count,
                                          ivp_real_t delta_time);

/* A controller entry registered in a simulation unit's controller list.
 * Sorted by priority (descending) within the unit. */
struct ivp_controller_core {
    ivp_controller_core_fn_t fn;
    void *ctx;
    int priority;

    /* Intrusive doubly-linked list within ivp_sim_unit_t::controller_list */
    ivp_controller_core_t *next_in_unit;
    ivp_controller_core_t *prev_in_unit;

    /* Back-pointer to owning sim unit (NULL if not registered) */
    ivp_sim_unit_t *sim_unit;
};

/* Initialize a controller core entry. */
static inline void ivp_controller_core_init(ivp_controller_core_t *cc,
                                             ivp_controller_core_fn_t fn,
                                             void *ctx,
                                             int priority) {
    cc->fn = fn;
    cc->ctx = ctx;
    cc->priority = priority;
    cc->next_in_unit = NULL;
    cc->prev_in_unit = NULL;
    cc->sim_unit = NULL;
}

/* ── Simulation unit movement state ────────────────────────────────────── */

typedef enum ivp_sim_unit_movement {
    IVP_SUM_MOVING = 0,   /* island has active objects */
    IVP_SUM_CALM   = 1,   /* all objects slow/calm */
    IVP_SUM_STILL  = 2    /* all objects frozen */
} ivp_sim_unit_movement_t;

/* ── Simulation unit ───────────────────────────────────────────────────── */

/* A core entry in the sim unit's core list (intrusive linked list). */
struct ivp_sim_unit_core_entry {
    ivp_core_t *core;
    ivp_sim_unit_core_entry_t *next;
    ivp_sim_unit_core_entry_t *prev;
};

struct ivp_sim_unit {
    /* Cores belonging to this island */
    ivp_sim_unit_core_entry_t *core_list;   /* head of intrusive DLL */
    int core_count;

    /* Controllers sorted by priority (descending) */
    ivp_controller_core_t *controller_list; /* head of sorted intrusive DLL */
    int controller_count;

    ivp_sim_unit_movement_t movement_type;
    bool union_find_needed;

    /* Intrusive doubly-linked list within manager */
    ivp_sim_unit_t *next_unit;
    ivp_sim_unit_t *prev_unit;

    /* Owning manager */
    ivp_sim_units_manager_t *manager;
};

/* ── Simulation units manager ──────────────────────────────────────────── */

struct ivp_sim_units_manager {
    ivp_environment_t *env;

    /* Active (non-frozen) sim units — doubly-linked list */
    ivp_sim_unit_t *active_units;  /* head */
    int active_count;

    /* Frozen/still slot: cores that are NOT_SIM go here. */
    ivp_sim_unit_t still_slot;

    /* Pool of core_entry nodes for allocation */
    const ivp_allocator_t *allocator;
};

/* ── Manager lifecycle ─────────────────────────────────────────────────── */

ivp_sim_units_manager_t *ivp_sim_units_manager_create(ivp_environment_t *env,
                                                       const ivp_allocator_t *alloc);
void ivp_sim_units_manager_destroy(ivp_sim_units_manager_t *mgr);

/* ── Sim unit lifecycle ────────────────────────────────────────────────── */

/* Create a new empty sim unit and add it to the active list. */
ivp_sim_unit_t *ivp_sim_unit_create(ivp_sim_units_manager_t *mgr);

/* Destroy a sim unit (must be empty — no cores, no controllers). */
void ivp_sim_unit_destroy(ivp_sim_units_manager_t *mgr, ivp_sim_unit_t *su);

/* ── Core management ───────────────────────────────────────────────────── */

/* Add a core to a sim unit. Allocates a core_entry internally.
 * If the core already belongs to another unit, it is removed first. */
void ivp_sim_unit_add_core(ivp_sim_unit_t *su, ivp_core_t *core);

/* Remove a core from its current sim unit. Frees the core_entry.
 * If the unit becomes empty, it is destroyed. */
void ivp_sim_unit_remove_core(ivp_sim_units_manager_t *mgr, ivp_core_t *core);

/* ── Sim unit merge ────────────────────────────────────────────────────── */

/* Merge su_b into su_a: all cores and controllers of su_b move to su_a.
 * su_b is destroyed after merge. Returns su_a. */
ivp_sim_unit_t *ivp_sim_unit_merge(ivp_sim_units_manager_t *mgr,
                                    ivp_sim_unit_t *su_a,
                                    ivp_sim_unit_t *su_b);

/* ── Controller registration ───────────────────────────────────────────── */

/* Insert a controller_core into the sim unit's sorted controller list. */
void ivp_sim_unit_add_controller(ivp_sim_unit_t *su, ivp_controller_core_t *cc);

/* Remove a controller_core from its sim unit's controller list. */
void ivp_sim_unit_remove_controller(ivp_controller_core_t *cc);

/* ── PSI execution ─────────────────────────────────────────────────────── */

/* Execute one PSI step for a single sim unit. Follows reference ordering:
 * 1. init_moving_core_for_psi + reset temporarily_unmovable/impacts
 * 2. commit_all_async_pushes
 * 3. run controllers from highest priority to lowest
 * 4. calc_next_psi_matrix (integration)
 * 5. calc movement state (freeze check)
 * 6. union_find if needed */
void ivp_sim_unit_simulate_psi(ivp_sim_unit_t *su,
                                ivp_environment_t *env,
                                ivp_real_t delta_time,
                                ivp_time_t step_end_time);

/* Execute PSI for all active sim units. */
void ivp_sim_units_manager_simulate_all_psi(ivp_sim_units_manager_t *mgr,
                                             ivp_real_t delta_time,
                                             ivp_time_t step_end_time);

/* ── Freeze / wake ─────────────────────────────────────────────────────── */

/* Calculate movement state and potentially freeze the unit. */
void ivp_sim_unit_calc_movement_state(ivp_sim_unit_t *su,
                                       ivp_environment_t *env);

/* Revive a frozen core (move it from still_slot into a new active unit). */
void ivp_sim_unit_revive_core(ivp_sim_units_manager_t *mgr, ivp_core_t *core);

/* ── Queries ───────────────────────────────────────────────────────────── */

/* Check if a core is in the still (frozen) slot. */
bool ivp_sim_unit_core_is_frozen(const ivp_sim_units_manager_t *mgr,
                                   const ivp_core_t *core);

#ifdef __cplusplus
}
#endif

#endif /* IVP_SIM_UNIT_H */
