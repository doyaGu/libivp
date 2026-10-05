/* ivp_core.h — Rigid body core state for IVP C17
 *
 * Port of IVP_Core (ivp_physics/ivp_core.hxx).  The core holds the
 * simulation state of one rigid body: mass properties, velocities,
 * the pose at the last PSI and the predicted orientation at the next PSI.
 *
 * Reference field precision:
 *   speed / rot_speed / speed_change / rot_speed_change : IVP_U_Float_Point
 *   pos_world_f_core_last_psi, quaternions, matrix    : double
 */
#ifndef IVP_CORE_H
#define IVP_CORE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/containers/ivp_pvec.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Movement state machine (IVP_Movement_Type) ─────────────────────────── */

typedef enum ivp_movement_type {
    IVP_MT_UNDEFINED       = 0x00,
    IVP_MT_MOVING          = 0x01, /* fast movement                         */
    IVP_MT_SLOW            = 0x02, /* below threshold for short time        */
    IVP_MT_CALM            = 0x03, /* slow for > freeze_check_dtime         */
    IVP_MT_NOT_SIM         = 0x08, /* frozen (can be revived)               */
    IVP_MT_STATIC_PHANTOM  = 0x09, /* like static but externally positioned */
    IVP_MT_STATIC          = 0x10, /* static/world object (never moves)     */
    IVP_MT_GET_MINDIST     = 0x21  /* query-only: used to fetch all mindists*/
} ivp_movement_type_t;

#define IVP_MTIS_SIMULATED(mt)    ((mt) < IVP_MT_NOT_SIM)
#define IVP_MTIS_CAN_COLLIDE(mt)  ((mt) & 0x07)
#define IVP_MTIS_IS_MOVING(mt)    IVP_MTIS_CAN_COLLIDE(mt)
#define IVP_MTIS_IS_STATIC(mt)    ((mt) & 0x10)

/* ── Freeze parameters (reference constants of ivp_core.cxx / IVP_Freeze_Manager) ── */

typedef struct ivp_freeze_params {
    ivp_real_t trans_threshold_short;  /* IVP_MINIMAL_TRANS_SPEED_PER_PSI_SHORT 0.01 */
    ivp_real_t rot_threshold_short;    /* IVP_MINIMAL_ROT_SPEED_PER_PSI_SHORT   0.005 */
    ivp_real_t trans_threshold_long;   /* IVP_MINIMAL_TRANS_SPEED_PER_PSI_LONG  0.1  */
    ivp_real_t rot_threshold_long;     /* IVP_MINIMAL_ROT_SPEED_PER_PSI_LONG    0.2  */
    ivp_real_t freeze_check_dtime;     /* IVP_Freeze_Manager::freeze_check_dtime 0.3 */
    ivp_real_t has_moved_long_time;    /* IVP_HAS_MOVED_LONG_TIME               4.0  */
} ivp_freeze_params_t;

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

/* ── IVP_Old_Sync_Rot_Z ─────────────────────────────────────────────────── */

typedef struct ivp_old_sync_rot {
    ivp_vec3f_t  old_sync_rot_speed;
    ivp_quatr_t  old_sync_q_world_f_core_next_psi;
    bool         was_pushed_during_i_s;
} ivp_old_sync_rot_t;

struct ivp_sim_unit;
struct ivp_object;

/* ── Core ───────────────────────────────────────────────────────────────── */

struct ivp_environment;
struct ivp_core_merged;

typedef struct ivp_core {
    /* IVP_Core_Fast_Static */
    bool             fast_piling_allowed;
    bool             physical_unmoveable;
    bool             is_in_wakeup_vec;
    bool             rot_inertias_are_equal;
    bool             pinned;
    ivp_real_t       upper_limit_radius;
    ivp_real_t       max_surface_deviation;
    struct ivp_environment *environment;     /* backlink (NULL for a standalone ivp_core_init core) */
    size_t           heap_allocation_size;   /* zero for caller-owned standalone cores */
    void            *car_wheel;              /* IVP_Constraint_Car_Object (opaque) */

    ivp_vec3r_t      rot_inertia;            /* rot_inertia (hesse_val = mass) */
    ivp_real_t       mass;
    ivp_vec3r_t      rot_speed_damp_factor;
    ivp_vec3r_t      inv_rot_inertia;        /* inv_rot_inertia (hesse_val = inv_mass) */
    ivp_real_t       inv_mass;
    ivp_real_t       speed_damp_factor;
    float            inv_object_diameter;    /* 0.5f / upper_limit_radius */

    bool             spin_clipping_enabled;  /* reference: spin_clipping != NULL */
    ivp_vec3f_t      spin_clipping;

    /* IVP_Core::objects (IVP_Vector_of_Objects): all objects of this core
     * (ivp_object_t*); several objects share a core after
     * ivp_environment_merge_objects / ivp_object_attach_object */
    ivp_pvec_t       objects;

    /* IVP_Core_Friction_Info */
    union {
        struct { void *l_friction_info_hash; } for_unmoveables;
        struct { void *moveable_core_friction_info; } for_moveables;
    } core_friction_info;

    /* IVP_Core_Fast_PSI */
    ivp_movement_type_t movement_state;
    bool             temporarily_unmovable;
    short            impacts_since_last_PSI;
    ivp_time_t       time_of_last_psi;
    float            i_delta_time;
    ivp_vec3r_t      rot_speed_change;
    ivp_vec3r_t      speed_change;
    ivp_vec3f_t      rot_speed;              /* core space [rad/s] */
    ivp_vec3r_t      speed;                  /* world space [m/s]  */
    ivp_vec3r_t      pos_world_f_core_last_psi;
    ivp_vec3r_t      delta_world_f_core_psis;
    ivp_quatr_t      q_world_f_core_last_psi;
    ivp_quatr_t      q_world_f_core_next_psi;
    ivp_transform_r_t m_world_f_core_last_psi;

    /* IVP_Core_Fast */
    ivp_vec3f_t      rotation_axis_world_space;
    float            current_speed;
    float            abs_omega;
    float            max_surface_rot_speed;

    /* IVP_Core */
    ivp_pvec_t       controllers_of_core;    /* ivp_sim_ctrl_t* */
    struct ivp_core_merged *merged_core_which_replace_this_core; /* NULL when the core is terminal */
    struct ivp_sim_unit *sim_unit;           /* sim_unit_of_core */
    ivp_time_t       time_of_calm_reference[2];
    ivp_quatf_t      q_world_f_core_calm_reference[2];
    ivp_vec3f_t      position_world_f_core_calm_reference[2];
    struct ivp_core *union_find_father;      /* tmp.union_find_father */
    struct ivp_old_sync_rot *old_sync_rot;   /* tmp_null.old_sync_info */
    unsigned int     mindist_event_already_done;
} ivp_core_t;

/* IVP_Core_Merged (ivp_intern/ivp_merge_core.hxx): the core that replaces the
 * cores of merged objects (ivp_environment_merge_objects). */
typedef struct ivp_core_merged {
    ivp_core_t        core;                         /* IVP_Core base */
    ivp_transform_r_t m_world_f_core_when_created;  /* matrix when the core was created */
} ivp_core_merged_t;

/* ── Dynamics constants ─────────────────────────────────────────────────── */

#define IVP_MAX_DELTA_PSI_TIME   (1.0f / 10.0f)    /* reference ivp_physics.hxx: float */
#define IVP_MIN_DELTA_PSI_TIME   (1.0f / 200.0f)
#define IVP_OBJECT_MOVING_FAST   1.0f       /* [m/sec] */
#define IVP_MAX_OBJECT_ROT_SPEED (3.14159265358979323846 * 0.5f)   /* IVP_PI * 0.5f (double) */

/* ── Core API ───────────────────────────────────────────────────────────── */

/* Initialize a standalone core (no environment, no objects) with given mass
 * and diagonal inertia (identity pose, zero velocity, IVP_MT_NOT_SIM).  Mass
 * properties go through calc_calc.  Cores of objects are created and owned by
 * the environment (ivp_object_get_core). */
void ivp_core_init(ivp_core_t *core, ivp_real_t mass, const ivp_vec3r_t *rot_inertia);

/* Release the containers of a standalone core (controller list, object list,
 * friction info hash). */
void ivp_core_release(ivp_core_t *core);

/* Set pose at last PSI (position + orientation); next-PSI orientation = same. */
void ivp_core_set_pose(ivp_core_t *core, const ivp_vec3r_t *pos, const ivp_quatr_t *quat);

/* Teleport: resets velocity to zero and sets the pose. */
void ivp_core_beam_to(ivp_core_t *core, const ivp_vec3r_t *pos, const ivp_quatr_t *quat);

/* IVP_Core::damp_object via global_damp_core (slow objects get +0.1). */
void ivp_core_damp(ivp_core_t *core, ivp_real_t delta_time);

/* IVP_Core::commit_all_async_pushes */
void ivp_core_commit_async_pushes(ivp_core_t *core);

/* Per-axis angular velocity clipping (reference spin_clipping). */
void ivp_core_set_spin_clipping(ivp_core_t *core, const ivp_vec3f_t *max_spin);
void ivp_core_clear_spin_clipping(ivp_core_t *core);

/* AT position/orientation (inline_calc_at_position / set_interpolate_smoothly). */
void ivp_core_get_at_position(const ivp_core_t *core, ivp_time_t t, ivp_vec3r_t *out);
void ivp_core_get_at_quat(const ivp_core_t *core, ivp_time_t t, ivp_quatr_t *out);
void ivp_core_calc_at_matrix(const ivp_core_t *core, ivp_time_t t, ivp_transform_r_t *out);

/* ── Pushes (reference push_core_ws / async_push_core_ws / center / rot) ── */

void ivp_core_push_ws(ivp_core_t *core, const ivp_vec3r_t *pos_ws, const ivp_vec3r_t *impulse_ws);
void ivp_core_async_push_ws(ivp_core_t *core, const ivp_vec3r_t *pos_ws, const ivp_vec3r_t *impulse_ws);
void ivp_core_center_push(ivp_core_t *core, const ivp_vec3r_t *impulse_ws);
void ivp_core_async_center_push(ivp_core_t *core, const ivp_vec3r_t *impulse_ws);
void ivp_core_rot_push_cs(ivp_core_t *core, const ivp_vec3r_t *torque_impulse_cs);
void ivp_core_async_rot_push_cs(ivp_core_t *core, const ivp_vec3r_t *torque_impulse_cs);

/* ── Property mutation ──────────────────────────────────────────────────── */

void ivp_core_set_mass(ivp_core_t *core, ivp_real_t mass);
void ivp_core_set_rotation_inertia(ivp_core_t *core, const ivp_vec3r_t *rot_inertia);
void ivp_core_set_pinned(ivp_core_t *core, bool pinned);

/* ── Queries ────────────────────────────────────────────────────────────── */

static inline void ivp_core_get_position(const ivp_core_t *core, ivp_vec3r_t *out) {
    *out = core->pos_world_f_core_last_psi;
}
static inline void ivp_core_get_speed(const ivp_core_t *core, ivp_vec3r_t *out) { *out = core->speed; }
static inline void ivp_core_get_rot_speed(const ivp_core_t *core, ivp_vec3f_t *out) { *out = core->rot_speed; }
static inline void ivp_core_get_orientation(const ivp_core_t *core, ivp_quatr_t *out) {
    *out = core->q_world_f_core_last_psi;
}

/* IVP_Core::get_surface_speed_ws */
void ivp_core_get_surface_speed_ws(const ivp_core_t *core, const ivp_vec3r_t *point_ws, ivp_vec3r_t *out_speed_ws);

/* Kinetic energy (get_energy_on_test with current speeds). */
ivp_real_t ivp_core_get_energy(const ivp_core_t *core);

/* IVP_Core::apply_velocity_limit: clip speed, speed_change, rot_speed and
 * rot_speed_change to the environment's anomaly limits (max velocity,
 * max angular velocity per PSI). */
void ivp_core_apply_velocity_limit(ivp_core_t *core);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CORE_H */
