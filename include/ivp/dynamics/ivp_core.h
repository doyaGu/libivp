/* ivp_core.h — Rigid body core state for IVP C17
 *
 * Contains linear/angular velocity, pose (position + quaternion),
 * damping parameters, inertia tensor, and movement state.
 *
 * Split into static (rarely changing) and dynamic (per-PSI) sections,
 * matching the original IVP_Core layout.
 */
#ifndef IVP_CORE_H
#define IVP_CORE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Movement state machine ─────────────────────────────────────────────── */

typedef enum ivp_movement_type {
    IVP_MT_UNDEFINED       = 0x00, /* sentinel / uninitialized              */
    IVP_MT_MOVING          = 0x01, /* fast movement                         */
    IVP_MT_SLOW            = 0x02, /* below threshold for short time        */
    IVP_MT_CALM            = 0x03, /* slow for > freeze_check_dtime         */
    IVP_MT_NOT_SIM         = 0x08, /* frozen (can be revived)               */
    IVP_MT_STATIC_PHANTOM  = 0x09, /* like static but externally positioned */
    IVP_MT_STATIC          = 0x10, /* static/world object (never moves)     */
    IVP_MT_GET_MINDIST     = 0x21  /* query-only: used to fetch all mindists*/
} ivp_movement_type_t;

/* Movement type classification macros (matching reference) */
#define IVP_MTIS_SIMULATED(mt)    ((mt) < IVP_MT_NOT_SIM)
#define IVP_MTIS_CAN_COLLIDE(mt)  ((mt) & 0x07)
#define IVP_MTIS_IS_MOVING(mt)    IVP_MTIS_CAN_COLLIDE(mt)  /* alias: identical check */
#define IVP_MTIS_IS_STATIC(mt)    ((mt) & 0x10)

/* ── Freeze parameters ──────────────────────────────────────────────────── */

typedef struct ivp_freeze_params {
    ivp_real_t trans_threshold_short;  /* default: 0.01 m                        */
    ivp_real_t rot_threshold_short;    /* default: 0.005 rad (scaled by radius)  */
    ivp_real_t trans_threshold_long;   /* default: 0.1 m                         */
    ivp_real_t rot_threshold_long;     /* default: 0.2 rad                       */
    ivp_real_t freeze_check_dtime;     /* default: 0.3 s  (short window)         */
    ivp_real_t has_moved_long_time;    /* default: 4.0 s  (long window)          */
} ivp_freeze_params_t;

/* Default freeze parameters (matching original IVP) */
static inline ivp_freeze_params_t ivp_freeze_params_default(void) {
    ivp_freeze_params_t p;
    p.trans_threshold_short = IVP_REAL(0.01);
    p.rot_threshold_short   = IVP_REAL(0.005);
    p.trans_threshold_long  = IVP_REAL(0.1);
    p.rot_threshold_long    = IVP_REAL(0.2);
    p.freeze_check_dtime    = IVP_REAL(0.3);
    p.has_moved_long_time   = IVP_REAL(4.0);
    return p;
}

/* ── Movement checkpoint (for freeze detection) ─────────────────────────── */

typedef struct ivp_movement_checkpoint {
    ivp_vec3r_t  pos;         /* position at checkpoint */
    ivp_quatr_t  quat;        /* orientation at checkpoint */
    ivp_time_t   time;        /* timestamp of checkpoint */
} ivp_movement_checkpoint_t;

/* ── Old sync rot state (ported from IVP_Old_Sync_Rot_Z) ───────────────── */
/*
 * Stores pre-push angular velocity and orientation so the impact system
 * can detect double-pushes during a cascade. Also serves as a flag:
 * when non-NULL on ivp_core_t::old_sync_rot, the core has already been
 * pushed in this cascade.
 */
typedef struct ivp_old_sync_rot {
    ivp_vec3r_t  old_sync_rot_speed;              /* pre-push angular velocity */
    ivp_quatr_t  old_sync_q_world_f_core_next_psi; /* pre-push predicted orientation */
    bool         was_pushed_during_i_s;            /* pushed during impact system solve */
} ivp_old_sync_rot_t;

/* ── Core — Rigid Body State ────────────────────────────────────────────── */

/* ivp_core_t holds all physics state for a rigid body. */

typedef struct ivp_core {
    /* ─── Static section (rarely changing) ─────────────────────────────── */
    bool             physical_unmoveable;    /* never integrates              */
    bool             pinned;                 /* no gravity / linear push      */
    bool             rot_inertias_are_equal; /* skip gyroscopic correction    */

    ivp_real_t       upper_limit_radius;     /* bounding radius from CoM     */
    ivp_real_t       max_surface_deviation;  /* max point-to-surface distance */

    /* Inertia tensor (diagonal in body-frame) */
    ivp_vec3r_t      rot_inertia;            /* I_xx, I_yy, I_zz            */
    ivp_real_t       mass;                   /* total mass                   */
    ivp_vec3r_t      inv_rot_inertia;        /* 1/I_xx, 1/I_yy, 1/I_zz     */
    ivp_real_t       inv_mass;               /* 1/mass                       */

    /* Damping parameters */
    ivp_vec3r_t      rot_speed_damp_factor;  /* per-axis angular damping     */
    ivp_real_t       speed_damp_factor;      /* linear damping constant      */

    /* Spin clipping (per-object angular velocity limit) */
    bool             spin_clipping_enabled;
    ivp_vec3f_t      spin_clipping;         /* max angular velocity per axis [rad/s]; 0 = no limit */

    /* ─── Dynamic section (updated every PSI) ──────────────────────────── */
    ivp_movement_type_t movement_state;

    ivp_time_t       time_of_last_psi;       /* timestamp of last PSI        */
    ivp_real_t       i_delta_time;           /* 1.0 / delta_time             */

    /* Accumulated async impulses (merged before integration) */
    ivp_vec3r_t      rot_speed_change;       /* async angular impulse        */
    ivp_vec3r_t      speed_change;           /* async linear impulse         */

    /* Velocities */
    ivp_vec3f_t      rot_speed;              /* angular velocity [body frame, rad/s] */
    ivp_vec3r_t      speed;                  /* linear velocity  [world frame, m/s]  */

    /* Pose */
    ivp_vec3r_t      pos_world_f_core_last_psi;    /* position at last PSI     */
    ivp_vec3r_t      delta_world_f_core_psis;       /* = speed (for AT interp)  */
    ivp_quatr_t      q_world_f_core_last_psi;       /* orientation at last PSI  */
    ivp_quatr_t      q_world_f_core_next_psi;       /* predicted orientation    */
    ivp_transform_r_t m_world_f_core_last_psi;      /* full matrix (redundant)  */

    /* Speed magnitude (cached after integration) */
    ivp_real_t       current_speed;
    ivp_real_t       max_surface_rot_speed;  /* |ω| * upper_limit_radius  */

    /* Freeze detection */
    ivp_movement_checkpoint_t checkpoint_short;
    ivp_movement_checkpoint_t checkpoint_long;
    int                       movement_check_counter;

    /* Multi-body cascade protection (reference: IVP_Core).
     * temporarily_unmovable is set true by impact solver during cascade;
     * for the remainder of this PSI the core is treated as immovable.
     * impacts_since_last_PSI is reset at sim unit PSI start and
     * incremented by impact solver; threshold activates multi-body system. */
    bool                      temporarily_unmovable;
    int                       impacts_since_last_PSI;
    bool                      fast_piling_allowed;

    /* ─── Per-core friction info (ported from IVP_Core_Friction_Info) ──── */
    /*
     * For unmoveable cores: l_friction_info_hash is a vhash mapping each
     * moveable partner core to friction data (O(1) lookup).
     * For moveable cores: moveable_core_friction_info points to the
     * friction info struct owned by the friction system.
     */
    union {
        struct { void *l_friction_info_hash; } for_unmoveables;
        struct { void *moveable_core_friction_info; } for_moveables;
    } core_friction_info;

    /* ─── Old sync rot (ported from IVP_Core::tmp_null) ────────────────── */
    /*
     * During an impact cascade, this pointer is used as a flag:
     *   - NULL: core has not been pushed yet in this cascade
     *   - non-NULL: core was already pushed (stores pre-push state)
     * This replaces the linear scan of pushed_cores in the impact system.
     * Must be reset to NULL when not in use.
     */
    struct ivp_old_sync_rot *old_sync_rot;

    /* ─── Simulation unit linkage ────────────────────────────────────── */
    struct ivp_sim_unit            *sim_unit;       /* owning dynamic island (not owned) */
    struct ivp_sim_unit_core_entry *sim_unit_entry; /* entry node in sim_unit->core_list  */

    /* ─── Object list (ported from IVP_Core_Fast_Static::objects) ──────── */
    /*
     * For merged cores, multiple objects share one core. This linked list
     * tracks all objects associated with this core. For single-object cores,
     * object_list points to the owning object's next_in_core_list (count=1).
     */
    struct ivp_object *object_list;
    int                object_count;
} ivp_core_t;

/* ── Movement check interval ───────────────────────────────────────────── */

#define IVP_MOVEMENT_CHECK_COUNT  10

/* ── Dynamics constants ─────────────────────────────────────────────────── */

#define IVP_MAX_DELTA_PSI_TIME   (IVP_REAL(1.0) / IVP_REAL(10.0))    /* 100ms max */
#define IVP_MIN_DELTA_PSI_TIME   (IVP_REAL(1.0) / IVP_REAL(200.0))   /* 5ms min   */
#define IVP_SLOW_ADDON_DAMPING   IVP_REAL(0.1)
#define IVP_GYRO_SUBSTEP_MAX_ANGLE  (IVP_REAL(5.0) * IVP_REAL(6.0) / IVP_REAL(360.0))  /* ≈ 0.0833 rad */

/* ── Core API ───────────────────────────────────────────────────────────── */

/* Initialize a core with given mass and diagonal inertia.
 * Sets identity pose at origin, zero velocity, MOVING state. */
void ivp_core_init(ivp_core_t *core, ivp_real_t mass,
                   const ivp_vec3r_t *rot_inertia);

/* Set pose (position + orientation quaternion). Recomputes matrix. */
void ivp_core_set_pose(ivp_core_t *core, const ivp_vec3r_t *pos,
                       const ivp_quatr_t *quat);

/* Apply damping: v *= exp(-damp * dt).
 * Adds IVP_SLOW_ADDON_DAMPING if movement_state >= SLOW. */
void ivp_core_damp(ivp_core_t *core, ivp_real_t delta_time);

/* Commit accumulated async pushes into velocity. */
void ivp_core_commit_async_pushes(ivp_core_t *core);

/* Initialize core for next PSI step:
 *   1. Bake q_next_psi → matrix
 *   2. Optionally forward-project position by delta_world_f_core_psis * dt */
void ivp_core_init_for_psi(ivp_core_t *core, ivp_real_t delta_time);

/* Initialize core for next PSI step with explicit forward-projection control.
 * When forward_project is true, position is advanced by
 * delta_world_f_core_psis * dt, matching legacy init_moving_core_for_psi. */
void ivp_core_init_for_psi_ex(ivp_core_t *core, ivp_real_t delta_time,
                               bool forward_project);

/* Compute next PSI matrix (rotation integration + position update):
 *   1. Gyroscopic correction (if unequal inertias)
 *   2. Quaternion step (set_fast_multiple_with_clip / gyro substeps)
 *   3. Update pos, delta, q_last, q_next
 *   4. fast_normize_quat */
void ivp_core_calc_next_psi_matrix(ivp_core_t *core, ivp_real_t delta_time);

/* Evaluate movement state (MOVING/SLOW/CALM).
 * Called periodically (every IVP_MOVEMENT_CHECK_COUNT PSIs).
 * freeze_params may be NULL to use defaults. */
void ivp_core_calc_movement_state(ivp_core_t *core,
                                   const ivp_freeze_params_t *freeze_params);

/* Beam (teleport) to new position/orientation. Resets velocity to zero. */
void ivp_core_beam_to(ivp_core_t *core, const ivp_vec3r_t *pos,
                      const ivp_quatr_t *quat);

/* Set per-object spin clipping. Non-zero components define max angular velocity.
 * The clipping uses the minimum of non-zero components as max magnitude. */
void ivp_core_set_spin_clipping(ivp_core_t *core, const ivp_vec3f_t *max_spin);

/* Disable spin clipping for this core. */
void ivp_core_clear_spin_clipping(ivp_core_t *core);

/* Get AT (any-time) interpolated position.
 * pos(t) = pos_last + delta * (t - time_of_last_psi) */
static inline void ivp_core_get_at_position(const ivp_core_t *core,
                                             ivp_time_t t,
                                             ivp_vec3r_t *out) {
    ivp_real_t dt = (ivp_real_t)(t - core->time_of_last_psi);
    out->k[0] = core->pos_world_f_core_last_psi.k[0] + core->delta_world_f_core_psis.k[0] * dt;
    out->k[1] = core->pos_world_f_core_last_psi.k[1] + core->delta_world_f_core_psis.k[1] * dt;
    out->k[2] = core->pos_world_f_core_last_psi.k[2] + core->delta_world_f_core_psis.k[2] * dt;
}

/* Get AT interpolated orientation.
 * Nlerp between q_last and q_next using fractional PSI time. */
static inline void ivp_core_get_at_quat(const ivp_core_t *core,
                                         ivp_time_t t,
                                         ivp_quatr_t *out) {
    ivp_real_t frac = (ivp_real_t)(t - core->time_of_last_psi) * core->i_delta_time;
    if (frac < IVP_REAL(0.0)) frac = IVP_REAL(0.0);
    if (frac > IVP_REAL(1.0)) frac = IVP_REAL(1.0);
    ivp_quat_nlerp(out, &core->q_world_f_core_last_psi,
                   &core->q_world_f_core_next_psi, frac);
}

/* ── Push / Impulse API ─────────────────────────────────────────────── */

/* Apply a linear push (impulse) at a world-space point.
 * Modifies speed and rot_speed directly (synchronous). */
void ivp_core_push_ws(ivp_core_t *core,
                       const ivp_vec3r_t *pos_ws,
                       const ivp_vec3r_t *impulse_ws);

/* Queue a linear push at a world-space point for later commit.
 * Modifies speed_change and rot_speed_change (async). */
void ivp_core_async_push_ws(ivp_core_t *core,
                              const ivp_vec3r_t *pos_ws,
                              const ivp_vec3r_t *impulse_ws);

/* Apply a center-of-mass push (no torque). Synchronous. */
void ivp_core_center_push(ivp_core_t *core,
                            const ivp_vec3r_t *impulse_ws);

/* Queue a center-of-mass push (no torque). Async. */
void ivp_core_async_center_push(ivp_core_t *core,
                                  const ivp_vec3r_t *impulse_ws);

/* Apply a rotational push in core space. Synchronous. */
void ivp_core_rot_push_cs(ivp_core_t *core,
                            const ivp_vec3r_t *torque_impulse_cs);

/* Queue a rotational push in core space. Async. */
void ivp_core_async_rot_push_cs(ivp_core_t *core,
                                  const ivp_vec3r_t *torque_impulse_cs);

/* ── Property mutation ─────────────────────────────────────────────── */

/* Set mass. Recomputes inv_mass. */
void ivp_core_set_mass(ivp_core_t *core, ivp_real_t mass);

/* Set rotation inertia. Recomputes inv_rot_inertia. */
void ivp_core_set_rotation_inertia(ivp_core_t *core,
                                     const ivp_vec3r_t *rot_inertia);

/* Set pinned state (no gravity, no linear push). */
void ivp_core_set_pinned(ivp_core_t *core, bool pinned);

/* ── Extended queries ──────────────────────────────────────────────── */

/* Get world-space position at last PSI. */
static inline void ivp_core_get_position(const ivp_core_t *core,
                                          ivp_vec3r_t *out) {
    *out = core->pos_world_f_core_last_psi;
}

/* Get linear velocity. */
static inline void ivp_core_get_speed(const ivp_core_t *core,
                                       ivp_vec3r_t *out) {
    *out = core->speed;
}

/* Get angular velocity in core space. */
static inline void ivp_core_get_rot_speed(const ivp_core_t *core,
                                            ivp_vec3f_t *out) {
    *out = core->rot_speed;
}

/* Get surface speed at a world-space point: v_point = v_cm + ω × r. */
void ivp_core_get_surface_speed_ws(const ivp_core_t *core,
                                     const ivp_vec3r_t *point_ws,
                                     ivp_vec3r_t *out_speed_ws);

/* Get total kinetic energy: 0.5*m*v² + 0.5*ω·(I*ω). */
ivp_real_t ivp_core_get_energy(const ivp_core_t *core);

/* Get orientation as quaternion. */
static inline void ivp_core_get_orientation(const ivp_core_t *core,
                                              ivp_quatr_t *out) {
    *out = core->q_world_f_core_last_psi;
}

/* Get AT (any-time) interpolated full transform.
 * Combines AT position and AT orientation into a transform matrix. */
void ivp_core_calc_at_matrix(const ivp_core_t *core,
                              ivp_time_t t,
                              ivp_transform_r_t *out);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CORE_H */
