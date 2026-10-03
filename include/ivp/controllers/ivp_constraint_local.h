/* ivp_constraint_local.h — General 6-DOF local constraint
 *
 * A fully configurable bilateral constraint between two objects (or one
 * object and world). Each of the 6 degrees of freedom (3 translation +
 * 3 rotation) can independently be:
 *   - FIXED:   locked to zero relative motion
 *   - LIMITED: free within min/max borders, hard-stops at limits
 *   - FREE:    unconstrained
 *
 * Solved by the reference IVP_Constraint_Local algorithm
 * (ivp_constraint_types.h): one dense Gauss solve per PSI, force_factor
 * (tau, default 1.0) and damp_factor (damping, default 1.0), per-axis
 * MaxImpulse with the reference CLIP / BREAK / BEND semantics.
 *
 * Ported from ivp_constraint_local.hxx.
 */
#ifndef IVP_CONSTRAINT_LOCAL_H
#define IVP_CONSTRAINT_LOCAL_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/controllers/ivp_constraint_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Per-axis constraint mode (reference IVP_CONSTRAINT_AXIS_TYPE values) ─ */

typedef enum ivp_constraint_axis_mode {
    IVP_CAXIS_FREE    = IVP_CONSTRAINT_AXIS_FREE,     /* unconstrained */
    IVP_CAXIS_FIXED   = IVP_CONSTRAINT_AXIS_FIXED,    /* locked to reference value */
    IVP_CAXIS_LIMITED = IVP_CONSTRAINT_AXIS_LIMITED   /* free within [min, max] */
} ivp_constraint_axis_mode_t;

/* Force exceed behaviour: aliases of the unified reference enum
 * ivp_constraint_force_exceed_t (NONE, CLIP, BREAK, BEND). */
#define IVP_CFORCE_NONE  IVP_CFE_NONE
#define IVP_CFORCE_CLIP  IVP_CFE_CLIP
#define IVP_CFORCE_BREAK IVP_CFE_BREAK
#define IVP_CFORCE_BEND  IVP_CFE_BEND

/* ── Per-axis configuration ────────────────────────────────────────────── */

typedef struct ivp_constraint_axis_config {
    ivp_constraint_axis_mode_t     mode;
    ivp_real_t                     border_min;     /* for LIMITED mode */
    ivp_real_t                     border_max;     /* for LIMITED mode */
    ivp_real_t                     max_impulse;    /* reference maximpulse (impulse per PSI) */
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

    /* Per-axis config: indices 0-2 = translation X/Y/Z, 3-5 = rotation X/Y/Z
     * (axes of the reference object's frame) */
    ivp_constraint_axis_config_t axes[6];

    /* Solver parameters */
    ivp_real_t    tau;              /* reference force_factor (default 1.0) */
    ivp_real_t    damping;          /* reference damp_factor (default 1.0) */
    ivp_real_t    limited_axis_stiffness; /* reference default 0.3 */
    int           solver_iterations; /* unused (one dense solve per PSI) */

    /* Runtime state */
    bool          active;
    bool          broken;           /* set when BREAK threshold exceeded */
    ivp_real_t    accumulated_impulse[6]; /* per-axis |impulse| of the last PSI */

    /* Reference solver */
    ivp_constraint_template_t    tmpl;
    ivp_constraint_local_core_t  core;
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

/* Reference change_Aos_to_relaxe_constraint: the attached frame (anchor and
 * axes) is moved to coincide with the reference frame at the current pose. */
void ivp_constraint_local_relax_attached(ivp_constraint_local_state_t *state);

/* Discovery function for ivp_environment_announce_dependent_controller(). */
int ivp_constraint_local_discovery(void *ctx, ivp_environment_t *env,
                                   ivp_object_handle_t *out_handles, int max_handles);

/* ── Controller callback ───────────────────────────────────────────────── */

/* Pass ivp_constraint_local_state_t* as ctx. */
void ivp_controller_constraint_local(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONSTRAINT_LOCAL_H */
