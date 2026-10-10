/* ivp_friction_internal.h — port of the reference friction / impact data model
 *
 * Reference: ivp_intern/ivp_friction.hxx, ivp_intern/ivp_friction_solver.hxx,
 * ivp_intern/ivp_impact.hxx, ivp_physics/ivp_contact_situation.hxx.
 *
 * Compile-time options of the reference build (all as in ivp_friction.hxx defaults):
 *   IVP_DISTANCE_KEEPERS, IVP_PANELTY_SOLVER, NOEASING, NO_MUTUAL_ENERGYDESTROY,
 *   IVP_USE_S_VALS_FOR_PRETENSION : not defined
 *   IVP_FAST_WHEELS_ENABLED       : defined (ivu_types.hxx)
 */
#ifndef IVP_FRICTION_INTERNAL_H
#define IVP_FRICTION_INTERNAL_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/containers/ivp_pvec.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include "dynamics/ivp_sim_internal.h"

#define IVP_FR_SLOWLY_TURN_ON_KEEPER   20
#define IVP_FR_MAX_FRICTION_MATRIX     150
#define IVP_FR_MAX_ADHESION_GAUSS      0.01f
#define IVP_FR_MINIMAL_REAL_FRICTION_LEN 0.005f
#define IVP_FR_MAX_ENERGY_DESTROY      0.1f
#define IVP_FR_EASE_EVERY_NTH_PSI      5
#define IVP_FR_DESCENDING_SPEEDUP      20.0f

struct ivp_fr_system;
struct ivp_fr_info_for_core;
struct ivp_contact_pt;
struct ivp_mindist;

/* IVP_Contact_Situation + IVP_Impact_Solver_Long_Term */
typedef struct ivp_impact_solver_lt {
    /* IVP_Contact_Situation */
    ivp_vec3f_t               surf_normal;      /* normized, world space, pointing to second obj */
    ivp_vec3f_t               speed;            /* speed of surface1 relative to surface0 */
    ivp_vec3r_t               contact_point_ws; /* IVP_U_Point */
    struct ivp_object        *objects[2];
    const ivp_compact_edge_t *compact_edges[2];
    const ivp_material_t     *materials[2];

    /* IVP_Impact_Solver_Long_Term */
    short index_in_fs;
    short impacts_while_system;
    bool  coll_time_is_valid;
    bool  friction_is_broken;
    union {
        struct {
            float rescue_speed_addon;
            float distance_reached_in_time;
            float percent_energy_conservation;
        } impact;
        struct {
            struct ivp_fr_info_for_core *friction_infos[2];
            int   has_negative_pull_since;
            float dist_len;
        } friction;
    };
    float virtual_mass;
    float inv_virtual_mass;
    ivp_core_t *contact_core[2];                /* NULL for unmoveable cores */
    ivp_vec3f_t span_friction_v[2];
    ivp_vec3f_t contact_point_cs[2];
    ivp_vec3f_t contact_cross_nomal_cs[2];
} ivp_impact_solver_lt_t;

static inline void ivp_impact_solver_lt_init_tmp(ivp_impact_solver_lt_t *i) {
    i->impacts_while_system = 0;
    i->coll_time_is_valid = false;
    i->friction_is_broken = false;
}

/* IVP_Impact_Solver_Long_Term::get_closing_speed */
double ivp_impact_lt_get_closing_speed(const ivp_impact_solver_lt_t *info);

/* IVP_Synapse_Friction */
typedef struct ivp_fr_synapse {
    struct ivp_fr_synapse    *next, *prev;   /* per object list */
    struct ivp_object        *l_obj;
    struct ivp_contact_pt    *contact_point; /* reference: contact_point_offset */
    short                     status;        /* ivp_synapse_polygon_status_t */
    const ivp_compact_edge_t *edge;
} ivp_fr_synapse_t;

typedef enum ivp_cp_break_status {
    IVP_CPBS_NORMAL,
    IVP_CPBS_NEEDS_RECHECK,
    IVP_CPBS_BROKEN
} ivp_cp_break_status_t;

/* IVP_Contact_Point */
typedef struct ivp_contact_pt {
    /* IVP_Contact_Point_Fast_Static */
    struct ivp_contact_pt *next_dist_in_friction;
    struct ivp_contact_pt *prev_dist_in_friction;
    uint64_t                creation_sequence;
#if IVP_CANONICAL_INTERNAL_STATE
    /* Logical list position, independent of the physical link order. */
    uint64_t                friction_order;
#endif
    ivp_fr_synapse_t       synapse[2];
    float                  inv_virt_mass_mindist_no_dir;
    bool                   two_friction_values;
    /* IVP_Contact_Point_Fast */
    float                  span_friction_s[2];
    ivp_impact_solver_lt_t *tmp_contact_info;
    float                  real_friction_factor;
    float                  integrated_destroyed_energy;
    float                  inv_triangle_det;
    float                  old_energy_dynamic_fr;
    float                  now_friction_pressure;
    float                  last_gap_len;
    short                  slowly_turn_on_keeper;
    uint8_t                cp_status;          /* ivp_cp_break_status_t */
    int                    has_negative_pull_since;
    ivp_time_t             last_time_of_recalc_friction_s_vals;
    ivp_vec3f_t            last_contact_point_ws;
    ivp_vec3f_t            surf_normal_ws;
    /* IVP_Contact_Point */
    struct ivp_fr_system  *l_friction_system;
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    uint64_t               diagnostic_generation;
    uint64_t               diagnostic_recalc_calls;
    uint64_t               diagnostic_geometry_fingerprint[2];
    uint64_t               diagnostic_kinematic_fingerprint[2];
    uint64_t               diagnostic_full_input_fingerprint[2];
    ivp_time_t             diagnostic_last_recalc_time;
    uint64_t               diagnostic_calls_at_time;
#endif
} ivp_contact_pt_t;

static inline ivp_fr_synapse_t *ivp_cp_get_synapse(ivp_contact_pt_t *cp, int i) { return &cp->synapse[i]; }
static inline float ivp_cp_get_gap_length(const ivp_contact_pt_t *cp) { return cp->last_gap_len; }

/* IVP_Friction_Core_Pair */
typedef struct ivp_fr_core_pair {
    const ivp_allocator_t *allocator;
    ivp_pvec_t  fr_dists;            /* ivp_contact_pt_t* */
#if IVP_CANONICAL_INTERNAL_STATE
    /* Logical append position in ivp_fr_system::fr_pairs_of_objs. */
    uint64_t    pair_order;
#endif
    int         next_ease_nr_psi;
    ivp_time_t  last_impact_time_pair;
    float       integrated_anti_energy;
    ivp_core_t *objs[2];
    ivp_vec3f_t span_vector_sum;     /* only for debug (debug_store_vector_before_ease) */
} ivp_fr_core_pair_t;

/* IVP_Friction_Info_For_Core */
typedef struct ivp_fr_info_for_core {
    const ivp_allocator_t *allocator;
    ivp_pvec_t            friction_springs;   /* ivp_contact_pt_t* */
    struct ivp_fr_system *l_friction_system;
} ivp_fr_info_for_core_t;

/* IVP_Friction_System (+ its Sys_Static / Sys_Energy handles) */
typedef struct ivp_fr_system {
    ivp_sim_ctrl_t         dyn_ctrl;      /* IVP_Friction_System (IVP_CP_DYNAMIC_FRICTION) */
    ivp_sim_ctrl_t         static_ctrl;   /* static_fs_handle    (IVP_CP_STATIC_FRICTION)  */
    ivp_sim_ctrl_t         energy_ctrl;   /* energy_fs_handle    (IVP_CP_ENERGY_FRICTION)  */
    struct ivp_environment *l_environment;
    ivp_contact_pt_t      *first_friction_dist;
    ivp_pvec_t             cores_of_friction_system;
    ivp_pvec_t             moveable_cores_of_friction_system;
    ivp_pvec_t             fr_pairs_of_objs;
    short                  friction_obj_number;
    short                  friction_dist_number;
    short                  complex_not_necessary_number;
#if IVP_CANONICAL_INTERNAL_STATE
    /* Sources for new logical list/vector positions. */
    uint64_t               next_friction_order;
    uint64_t               next_pair_order;
#endif
    bool                   union_find_necessary;
    bool                   fr_sys_simulated;
    double                 sum_energy_destroyed;
} ivp_fr_system_t;

/* ── Friction system API ──────────────────────────────────────────────── */

ivp_fr_system_t *ivp_fr_system_create(struct ivp_environment *env);
void ivp_fr_system_destroy(ivp_fr_system_t *fs);
bool ivp_fr_system_add_core_to_system(ivp_fr_system_t *fs, ivp_core_t *core);
void ivp_fr_system_remove_core_from_system(ivp_fr_system_t *fs, ivp_core_t *core);
void ivp_fr_system_add_dist_to_system(ivp_fr_system_t *fs, ivp_contact_pt_t *cp);
void ivp_fr_system_remove_dist_from_system(ivp_fr_system_t *fs, ivp_contact_pt_t *cp);
bool ivp_fr_system_dist_added_update_pair_info(ivp_fr_system_t *fs, ivp_contact_pt_t *cp);
bool ivp_fr_system_dist_removed_update_pair_info(ivp_fr_system_t *fs, ivp_contact_pt_t *cp);
void ivp_fr_system_delete_friction_distance(ivp_fr_system_t *fs, ivp_contact_pt_t *cp);
bool ivp_fr_system_fusion_friction_systems(ivp_fr_system_t *fs, ivp_fr_system_t *second);
ivp_fr_core_pair_t *ivp_fr_system_find_pair_of_cores(ivp_fr_system_t *fs, ivp_core_t *c0, ivp_core_t *c1);
ivp_fr_core_pair_t *ivp_fr_system_get_pair_info_for_objs(ivp_fr_system_t *fs, ivp_core_t *c0, ivp_core_t *c1);
void ivp_fr_system_reset_time(ivp_fr_system_t *fs, double offset);

int  ivp_fr_core_pair_check_all_fr_mindists_to_be_valid(ivp_fr_core_pair_t *pair, ivp_fr_system_t *fs);

/* ── Friction system debug helpers (reference debug methods; used from
 *    IVP_IF blocks, they print through ivp_debug_printf) ─────────────── */

/* IVP_Friction_System::test_hole_fr_system_data (ivp_friction_gaps.cxx):
 * consistency check of dists / friction infos / pairs (CORE on error) */
void ivp_fr_system_test_hole_fr_system_data(ivp_fr_system_t *fs);
/* IVP_Friction_System::debug_clean_tmp_info (ivp_impact.cxx):
 * tmp_contact_info = NULL for every dist of the system */
void ivp_fr_system_debug_clean_tmp_info(ivp_fr_system_t *fs);
/* IVP_Friction_System::debug_check_system_consistency (ivp_friction.cxx) */
void ivp_fr_system_debug_check_system_consistency(ivp_fr_system_t *fs);
/* IVP_Friction_System::print_all_dists / debug_fs_out_ascii (ivp_friction.cxx),
 * ivp_debug_fs_pointers (ivp_friction_gaps.cxx) */
void ivp_fr_system_print_all_dists(ivp_fr_system_t *fs);
void ivp_fr_system_debug_fs_out_ascii(ivp_fr_system_t *fs);
void ivp_fr_system_ivp_debug_fs_pointers(ivp_fr_system_t *fs);
/* IVP_Friction_Core_Pair::debug_printf_pair (ivp_friction.cxx) */
void ivp_fr_core_pair_debug_printf_pair(ivp_fr_core_pair_t *pair);
/* out_friction_info_obj (ivp_friction_gaps.cxx, returns at once) */
void ivp_fr_out_friction_info_obj(ivp_core_t *obj);

ivp_fr_info_for_core_t *ivp_fr_info_create(const ivp_allocator_t *allocator);
void ivp_fr_info_destroy(ivp_fr_info_for_core_t *info);
bool ivp_fr_info_insert_friction_dist(ivp_fr_info_for_core_t *info, ivp_contact_pt_t *cp);
void ivp_fr_info_delete_friction_dist(ivp_fr_info_for_core_t *info, ivp_contact_pt_t *cp);

/* ── Contact point API ────────────────────────────────────────────────── */

ivp_contact_pt_t *ivp_contact_pt_create(struct ivp_mindist *md);
void ivp_contact_pt_destroy(ivp_contact_pt_t *cp);       /* ~IVP_Contact_Point + free */
void ivp_contact_pt_recalc_friction_s_vals(ivp_contact_pt_t *cp);
void ivp_contact_pt_read_materials_for_contact_situation(ivp_contact_pt_t *cp, ivp_impact_solver_lt_t *info);
void ivp_contact_pt_get_material_info(ivp_contact_pt_t *cp, const ivp_material_t *mtl[2]);
void ivp_contact_pt_calc_virtual_mass_of_mindist(ivp_contact_pt_t *cp);
void ivp_contact_pt_calc_coll_distance(ivp_contact_pt_t *cp);
float ivp_contact_pt_get_rescue_speed_impact(ivp_contact_pt_t *cp, struct ivp_environment *env);
void ivp_contact_pt_reset_time(ivp_contact_pt_t *cp, double offset);
void ivp_contact_pt_recompute_friction(ivp_contact_pt_t *cp);
bool ivp_contact_pt_is_same_as(const ivp_contact_pt_t *cp, struct ivp_mindist *md);

/* IVP_Friction_Manager */
ivp_contact_pt_t *ivp_friction_mgr_generate_contact_point(struct ivp_mindist *md, bool *was_success);
ivp_contact_pt_t *ivp_friction_mgr_get_associated_contact_point(struct ivp_mindist *md);
void ivp_friction_mgr_delete_all_contact_points_of_object(struct ivp_object *obj);
void ivp_friction_mgr_restore_object_contact_order(struct ivp_object *obj);

/* ── Friction solver statics (ivp_friction_solver.hxx) ───────────────── */

void ivp_frs_apply_impulse(const ivp_impact_solver_lt_t *info, double impulse);
void ivp_frs_async_apply_impulse(const ivp_impact_solver_lt_t *info, double impulse);
double ivp_frs_calc_desired_gap_speed(double closing_speed, double gap_diff, float speedup_factor);

/* ── Impact (ivp_impact.cxx / ivp_mindist_impact.cxx) ────────────────── */

void ivp_impact_lt_do_impact_long_term(ivp_impact_solver_lt_t *info, ivp_core_t *pushed_cores[2],
                                       float rescue_speed_val, ivp_contact_pt_t *cp);
void ivp_impact_lt_do_impact_of_two_objects(struct ivp_mindist *md, struct ivp_object *o0, struct ivp_object *o1);

#endif /* IVP_FRICTION_INTERNAL_H */
