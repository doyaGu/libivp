/* ivp_friction_system.h — Persistent friction system
 *
 * Contacts persist across PSIs in linked lists. Each friction system
 * groups cores connected by friction contacts. Features:
 *   - 2D surface-coordinate friction forces (span_friction_s[2])
 *   - Gauss-elimination coupled solver
 *   - Energy tracking and gain removal
 *   - Easing every N PSIs
 *   - Distance keeper with ramp-up
 *   - System merge/split when contacts bridge or disconnect friction islands
 *
 * Ported from IVP_Friction_System.
 */
#ifndef IVP_FRICTION_SYSTEM_H
#define IVP_FRICTION_SYSTEM_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/collision/ivp_collision_tolerance.h>
#include <ivp/collision/ivp_synapse.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

/* IVP_EASE_EVERY_NTH_PSI is defined in ivp_solver_constants.h (included via ivp_contact.h). */
#define IVP_FRICTION_MAX_CONTACTS    256
#define IVP_FRICTION_MAX_CORES        64
#define IVP_FRICTION_MAX_PAIRS        64
#define IVP_FRICTION_MATCH_DIST_SQ   (IVP_REAL(0.01) * IVP_REAL(0.01))
#define IVP_FRICTION_MATCH_NORMAL_OS_DOT IVP_REAL(0.85)

typedef enum ivp_contact_point_break_status {
    IVP_CP_STATUS_NORMAL = 0,
    IVP_CP_STATUS_NEEDS_RECHECK = 1,
    IVP_CP_STATUS_BROKEN = 2
} ivp_contact_point_break_status_t;

typedef struct ivp_friction_psi_scratch {
    ivp_vec3r_t surf_normal;
    ivp_vec3r_t contact_point_ws;
    ivp_vec3r_t span_friction_v[2];
    ivp_core_t *contact_core[2];
    ivp_vec3r_t contact_point_cs[2];
    ivp_vec3r_t contact_cross_normal_cs[2];
    ivp_real_t virtual_mass;
    ivp_real_t inv_virtual_mass;
    int index_in_fs;
    int has_negative_pull_since;
    ivp_real_t dist_len;
    bool friction_is_broken;
} ivp_friction_psi_scratch_t;

/* ── Persistent contact ────────────────────────────────────────────────── */

typedef struct ivp_persistent_contact {
    ivp_contact_point_t cp;

    /* 2D surface-coordinate friction forces (in span_dirs basis) */
    ivp_real_t span_friction_s[2];

    /* Energy tracking */
    ivp_real_t integrated_destroyed_energy;
    ivp_real_t now_friction_pressure;
    ivp_real_t last_gap_len;
    ivp_real_t old_energy_dynamic_fr;
    ivp_real_t inv_virt_mass_mindist_no_dir;

    /* Persistence state */
    int  has_negative_pull_since;
    int  slowly_turn_on_keeper;    /* legacy keeper ramp countdown */
    int  impacts_while_system;    /* per-PSI impact-system counter */
    bool two_friction_values;
    ivp_real_t two_friction_x_scale;
    ivp_contact_point_break_status_t cp_status;
    bool friction_is_broken;
    bool seen_this_psi;
    bool impact_resolved;          /* true after first impact has been applied */

    /* Object-space contact points (for frame-to-frame matching) */
    ivp_vec3r_t contact_point_os[2];
    ivp_vec3r_t contact_normal_os[2];

    /* Legacy-style friction synapse linkage on both objects. */
    ivp_synapse_link_t friction_synapse[2];

    /* Warm-start: last normal impulse from coupled solver (Phase D4) */
    ivp_real_t last_normal_impulse;

    /* Per-PSI communication scratch */
    ivp_friction_psi_scratch_t psi_scratch;

    /* Linked list */
    struct ivp_persistent_contact *next;
    struct ivp_persistent_contact *prev;
} ivp_persistent_contact_t;

/* ── Friction core pair ────────────────────────────────────────────────── */

typedef struct ivp_friction_core_pair {
    ivp_core_t *cores[2];
    ivp_persistent_contact_t **contacts;
    int contact_count;
    int contact_capacity;
    int next_ease_nr_psi;
    ivp_real_t integrated_anti_energy;
    const ivp_allocator_t *alloc;
} ivp_friction_core_pair_t;

/* ── Friction system ───────────────────────────────────────────────────── */

typedef struct ivp_friction_system {
    ivp_persistent_contact_t *first_contact;
    int contact_count;

    ivp_core_t **cores;
    int core_count;
    int core_capacity;

    ivp_friction_core_pair_t *pairs;
    int pair_count;
    int pair_capacity;

    struct ivp_friction_manager *manager;
    const ivp_allocator_t *alloc;
} ivp_friction_system_t;

/* ── Friction manager (environment-level) ──────────────────────────────── */

/* Maximum number of pool blocks (each 256 contacts).  Provides up to
 * IVP_FRICTION_POOL_MAX_BLOCKS * 256 = 4096 simultaneous contacts. */
#define IVP_FRICTION_POOL_BLOCK_SIZE  256
#define IVP_FRICTION_POOL_MAX_BLOCKS   16

typedef struct ivp_friction_manager {
    ivp_friction_system_t **systems;
    int system_count;
    int system_capacity;

    /* Block-based contact pool: blocks are never reallocated once created,
     * so pointers into them remain stable.  New blocks are appended when
     * the current block is exhausted. */
    ivp_persistent_contact_t *pool_blocks[IVP_FRICTION_POOL_MAX_BLOCKS];
    int block_count;           /* number of allocated blocks         */
    int current_block_used;    /* slots used in the newest block     */

    ivp_persistent_contact_t *free_contacts;  /* singly-linked free list */

    int psi_counter;
    const ivp_allocator_t *alloc;
} ivp_friction_manager_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

ivp_result_t ivp_friction_manager_init(ivp_friction_manager_t *mgr,
                                         const ivp_allocator_t *alloc);
void ivp_friction_manager_destroy(ivp_friction_manager_t *mgr);

/* ── System lookup ─────────────────────────────────────────────────────── */

/* Find the friction system containing a given core, or NULL. */
ivp_friction_system_t *ivp_friction_manager_find_system_for_core(
    ivp_friction_manager_t *mgr, ivp_core_t *core);

/* ── Contact matching ──────────────────────────────────────────────────── */

/* Match broadphase contact to existing persistent contact,
 * or create new one. Uses object-space position for frame-to-frame matching. */
ivp_persistent_contact_t *ivp_friction_manager_match_or_create(
    ivp_friction_manager_t *mgr,
    const ivp_contact_point_t *new_cp);

/* Remove stale contacts not seen this PSI */
void ivp_friction_manager_age_and_prune(ivp_friction_manager_t *mgr);

/* Mark all contacts as unseen before a new PSI */
void ivp_friction_manager_begin_psi(ivp_friction_manager_t *mgr);

/* ── Distance keeper pre-pass ──────────────────────────────────────────── */

/* Apply independent per-contact distance keeper pushes before the coupled
 * friction solve.  Targets keeper_dist.  Legacy: do_pushes_distance_keepers. */
void ivp_friction_system_push_distance_keepers(ivp_friction_system_t *sys,
                                                 const ivp_collision_settings_t *settings,
                                                 ivp_real_t dt);

/* ── Coupled friction solver ───────────────────────────────────────────── */

/* Solve coupled friction for all contacts in a system */
ivp_result_t ivp_friction_system_solve_coupled(ivp_friction_system_t *sys,
                                                 const ivp_collision_settings_t *settings,
                                                 ivp_real_t dt);

/* Legacy phase split entrypoints. */
void ivp_friction_system_phase_energy(ivp_friction_system_t *sys,
                                      const ivp_collision_settings_t *settings,
                                      ivp_real_t dt);
ivp_result_t ivp_friction_system_phase_static_normal(ivp_friction_system_t *sys,
                                                     const ivp_collision_settings_t *settings,
                                                     ivp_real_t dt);
void ivp_friction_system_phase_dynamic_tangential(ivp_friction_system_t *sys,
                                                  const ivp_collision_settings_t *settings,
                                                  ivp_real_t dt);

/* Ease friction every IVP_EASE_EVERY_NTH_PSI */
void ivp_friction_system_ease(ivp_friction_system_t *sys);

/* Energy removal */
void ivp_friction_system_remove_gained_energy(ivp_friction_system_t *sys);

/* Post-stabilization pass (Baumgarte position correction).
 * Only active when solver config has enable_post_stabilization = true. */
void ivp_friction_system_post_stabilize(ivp_friction_system_t *sys,
                                         const ivp_collision_settings_t *settings,
                                         ivp_real_t dt);

/* System merge/split */
void ivp_friction_system_merge(ivp_friction_system_t *dst,
                                 ivp_friction_system_t *src,
                                 ivp_friction_manager_t *mgr);
void ivp_friction_system_check_split(ivp_friction_system_t *sys,
                                       ivp_friction_manager_t *mgr);

#ifdef __cplusplus
}
#endif

#endif /* IVP_FRICTION_SYSTEM_H */
