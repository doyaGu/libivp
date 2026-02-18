/* ivp_constraint_local.h — General 6-DOF local constraint
 *
 * A fully configurable bilateral constraint between two objects (or one
 * object and world). Each of the 6 degrees of freedom (3 translation +
 * 3 rotation) can independently be:
 *   - FIXED:   locked to zero relative motion
 *   - LIMITED: free within min/max borders, hard-stops at limits
 *   - FREE:    unconstrained
 *
 * The constraint operates in local reference frames attached to each
 * object. Force limits (max impulse per step) can be set per axis,
 * with configurable behavior on exceed (clip, break, or bend).
 *
 * Ported from ivp_constraint_local.hxx.
 */
#ifndef IVP_CONSTRAINT_LOCAL_H
#define IVP_CONSTRAINT_LOCAL_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Per-axis constraint mode ──────────────────────────────────────────── */

typedef enum ivp_constraint_axis_mode {
    IVP_CAXIS_FREE    = 0,  /* unconstrained                    */
    IVP_CAXIS_FIXED   = 1,  /* locked to reference value        */
    IVP_CAXIS_LIMITED = 2   /* free within [min, max] borders   */
} ivp_constraint_axis_mode_t;

/* ── Force exceed behavior ─────────────────────────────────────────────── */

typedef enum ivp_constraint_force_exceed {
    IVP_CFORCE_CLIP  = 0,  /* clamp impulse to limit            */
    IVP_CFORCE_BREAK = 1,  /* deactivate constraint if exceeded */
    IVP_CFORCE_NONE  = 2,  /* no limit                          */
    IVP_CFORCE_BEND  = 3   /* soft clamp: squared ratio falloff  */
} ivp_constraint_force_exceed_t;

/* ── Per-axis configuration ────────────────────────────────────────────── */

typedef struct ivp_constraint_axis_config {
    ivp_constraint_axis_mode_t     mode;
    ivp_real_t                     border_min;     /* for LIMITED mode */
    ivp_real_t                     border_max;     /* for LIMITED mode */
    ivp_real_t                     max_impulse;    /* per-step impulse limit */
    ivp_constraint_force_exceed_t  exceed_mode;
} ivp_constraint_axis_config_t;

/* ── Local constraint state ────────────────────────────────────────────── */

typedef struct ivp_constraint_local_state {
    /* Objects */
    ivp_object_t *obj_a;            /* reference object (NULL = world) */
    ivp_object_t *obj_b;            /* attached object (required) */

    /* Anchor points in respective body spaces */
    ivp_vec3r_t   anchor_a_os;      /* anchor in A's body space */
    ivp_vec3r_t   anchor_b_os;      /* anchor in B's body space */

    /* Reference orientations (recorded at init for relative frame) */
    ivp_quatr_t   ref_q_a;
    ivp_quatr_t   ref_q_b;

    /* Per-axis config: indices 0-2 = translation X/Y/Z, 3-5 = rotation X/Y/Z */
    ivp_constraint_axis_config_t axes[6];

    /* Solver parameters */
    ivp_real_t    tau;              /* Baumgarte stabilization (0.1-0.3) */
    ivp_real_t    damping;          /* velocity damping (0-1) */
    int           solver_iterations;

    /* Runtime state */
    bool          active;
    bool          broken;           /* set when BREAK threshold exceeded */
    ivp_real_t    accumulated_impulse[6]; /* debug: per-axis impulse this step */
} ivp_constraint_local_state_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Initialize a local constraint with all axes FIXED (equivalent to weld).
 * Call ivp_constraint_local_set_axis_*() to configure individual DOFs. */
void ivp_constraint_local_init(ivp_constraint_local_state_t *state,
                               ivp_object_t *obj_a,
                               const ivp_vec3r_t *anchor_a_os,
                               ivp_object_t *obj_b,
                               const ivp_vec3r_t *anchor_b_os);

/* ── Per-axis configuration ────────────────────────────────────────────── */

/* Fix a translation axis (0=X, 1=Y, 2=Z). */
void ivp_constraint_local_fix_translation(ivp_constraint_local_state_t *state,
                                          int axis);

/* Free a translation axis. */
void ivp_constraint_local_free_translation(ivp_constraint_local_state_t *state,
                                           int axis);

/* Limit a translation axis to [min, max] range. */
void ivp_constraint_local_limit_translation(ivp_constraint_local_state_t *state,
                                            int axis,
                                            ivp_real_t border_min,
                                            ivp_real_t border_max);

/* Fix a rotation axis (0=X, 1=Y, 2=Z). */
void ivp_constraint_local_fix_rotation(ivp_constraint_local_state_t *state,
                                       int axis);

/* Free a rotation axis. */
void ivp_constraint_local_free_rotation(ivp_constraint_local_state_t *state,
                                        int axis);

/* Limit a rotation axis to [min, max] radians. */
void ivp_constraint_local_limit_rotation(ivp_constraint_local_state_t *state,
                                         int axis,
                                         ivp_real_t min_rad,
                                         ivp_real_t max_rad);

/* Set impulse limit for an axis (0-2 = translation, 3-5 = rotation). */
void ivp_constraint_local_set_max_impulse(ivp_constraint_local_state_t *state,
                                          int axis_index,
                                          ivp_real_t max_impulse,
                                          ivp_constraint_force_exceed_t mode);

/* ── Enable / Disable / Broken ─────────────────────────────────────────── */

/* Activate or deactivate the constraint. Inactive constraints are skipped. */
void ivp_constraint_local_set_active(ivp_constraint_local_state_t *state,
                                      bool active);

/* Query whether the constraint is currently active. */
bool ivp_constraint_local_is_active(const ivp_constraint_local_state_t *state);

/* Query whether the constraint has been broken (BREAK threshold exceeded). */
bool ivp_constraint_local_is_broken(const ivp_constraint_local_state_t *state);

/* Reset the broken flag. If reactivate is true, also sets active = true. */
void ivp_constraint_local_reset_broken(ivp_constraint_local_state_t *state,
                                        bool reactivate);

/* ── Runtime rebinding ─────────────────────────────────────────────────── */

/* Update anchor positions in body space. */
void ivp_constraint_local_rebind_anchors(ivp_constraint_local_state_t *state,
                                          const ivp_vec3r_t *anchor_a_os,
                                          const ivp_vec3r_t *anchor_b_os);

/* Re-record current object orientations as the reference (zero-error) pose. */
void ivp_constraint_local_rebind_reference(ivp_constraint_local_state_t *state);

/* ── Controller callback ───────────────────────────────────────────────── */

/* Pass ivp_constraint_local_state_t* as ctx. */
void ivp_controller_constraint_local(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONSTRAINT_LOCAL_H */
