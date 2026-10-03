/* ivp_constraint.h -- Constraint system for IVP C17
 *
 * A constraint limits relative motion between a reference object R (obj_a,
 * NULL = world) and an attached object A (obj_b).  All constraint types are
 * solved by the reference IVP_Constraint_Local algorithm (see
 * ivp_constraint_types.h): one dense Gauss solve per PSI over all fixed
 * axes, synchronous impulses on both cores, MaxImpulse CLIP/BREAK/BEND.
 *
 * Supported constraint types (reference IVP_Template_Constraint setters):
 *   BALL_SOCKET       - 3 translation axes fixed
 *   HINGE             - 3 translation + 2 rotation axes fixed
 *   FIXED             - 3 translation + 3 rotation axes fixed
 *   CARDAN            - 3 translation + 1 rotation axis fixed
 *   BALL_SOCKET_TENSE - ball socket with a pre-tension displacement
 *   GENERAL           - arbitrary fixed translation/rotation dimensions
 *
 * Solver parameters: tau = reference force_factor (default 1.0),
 * damping = reference damp_factor (default 1.0).
 *
 * Ported from ivp_constraint.hxx / ivp_constraint_local.hxx /
 * ivp_template_constraint.hxx.
 */
#ifndef IVP_CONSTRAINT_H
#define IVP_CONSTRAINT_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/controllers/ivp_constraint_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constraint type ────────────────────────────────────────────────────── */

typedef enum ivp_constraint_type {
    IVP_CONSTRAINT_BALL_SOCKET       = 0,  /* position only (3 DOF removed) */
    IVP_CONSTRAINT_HINGE             = 1,  /* position + 2 rotation axes (5 DOF removed) */
    IVP_CONSTRAINT_FIXED             = 2,  /* position + orientation (6 DOF removed) */
    IVP_CONSTRAINT_CARDAN            = 3,  /* position + 1 rotation axis (4 DOF removed) */
    IVP_CONSTRAINT_BALL_SOCKET_TENSE = 4,  /* ball socket with pre-tension */
    IVP_CONSTRAINT_GENERAL           = 5   /* arbitrary fixed dims / template */
} ivp_constraint_type_t;

/* ── Constraint limits ──────────────────────────────────────────────────── */

typedef struct ivp_constraint_limits {
    bool       limited;          /* are angular limits active? */
    ivp_real_t min_angle;        /* minimum angle (rad), hinge only */
    ivp_real_t max_angle;        /* maximum angle (rad), hinge only */
    ivp_real_t limit_stiffness;  /* reference limited_axis_stiffness (0..1) */
    ivp_real_t limit_damping;    /* unused (the reference has no limit damping) */
} ivp_constraint_limits_t;

/* ── Constraint state ───────────────────────────────────────────────────── */

typedef struct ivp_constraint_state {
    ivp_constraint_type_t type;

    /* Attachment (informational; the solver frames live in core) */
    ivp_object_t *obj_a;            /* reference object R (NULL = world) */
    ivp_object_t *obj_b;            /* attached object A */
    ivp_vec3r_t   anchor_a_os;      /* anchor in A's body space */
    ivp_vec3r_t   anchor_b_os;      /* anchor in B's body space */
    ivp_vec3r_t   axis_a_os;        /* hinge axis in A's body space */
    ivp_vec3r_t   axis_b_os;        /* hinge axis in B's body space */
    ivp_quatr_t   ref_q_a;          /* orientation of A at init */
    ivp_quatr_t   ref_q_b;          /* orientation of B at init */

    /* Solver parameters */
    ivp_real_t    tau;              /* reference force_factor (default 1.0) */
    ivp_real_t    damping;          /* reference damp_factor (default 1.0) */
    int           solver_iterations; /* unused (one dense solve per PSI) */

    /* Angular limits (hinge free axis) */
    ivp_constraint_limits_t limits;

    /* Runtime state */
    bool          active;
    ivp_real_t    accumulated_impulse; /* |linear| + |angular| impulse of the last PSI */

    /* MaxImpulse (reference IVP_Constraint_Local_MaxImpulse); a threshold of
     * 0 disables the limit of that group. */
    ivp_real_t                    break_force_threshold;   /* translation impulse limit */
    ivp_real_t                    break_torque_threshold;  /* rotation impulse limit */
    ivp_constraint_force_exceed_t force_exceed_action;     /* CLIP / BREAK / BEND */
    bool                          is_broken;

    /* Reference solver */
    ivp_constraint_local_core_t   core;
    bool                          core_ready;
} ivp_constraint_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

/* Ball socket with explicit anchors in both object spaces
 * (reference set_ballsocket_Ros + set_attached_fixing_point_Aos). */
void ivp_constraint_init_ball_socket(ivp_constraint_state_t *state,
                                      ivp_object_t *obj_a,
                                      const ivp_vec3r_t *anchor_a_os,
                                      ivp_object_t *obj_b,
                                      const ivp_vec3r_t *anchor_b_os);

/* Reference set_ballsocket_ws: anchor given in world space, the attached
 * object is relaxed at its current pose. */
void ivp_constraint_init_ball_socket_ws(ivp_constraint_state_t *state,
                                        ivp_object_t *obj_r,
                                        const ivp_vec3r_t *anchor_ws,
                                        ivp_object_t *obj_a);

/* Reference set_ballsocket_tense_Ros: distanceAR_Ros is the vector from the
 * anchor to where the anchor should be on the attached object (R space). */
void ivp_constraint_init_ball_socket_tense_Ros(ivp_constraint_state_t *state,
                                               ivp_object_t *obj_r,
                                               const ivp_vec3r_t *anchor_Ros,
                                               ivp_object_t *obj_a,
                                               const ivp_vec3r_t *distanceAR_Ros);

/* Hinge with explicit anchors/axes in both object spaces. */
void ivp_constraint_init_hinge(ivp_constraint_state_t *state,
                                 ivp_object_t *obj_a,
                                 const ivp_vec3r_t *anchor_a_os,
                                 const ivp_vec3r_t *axis_a_os,
                                 ivp_object_t *obj_b,
                                 const ivp_vec3r_t *anchor_b_os,
                                 const ivp_vec3r_t *axis_b_os);

/* Reference set_hinge_ws (relaxed attached object). */
void ivp_constraint_init_hinge_ws(ivp_constraint_state_t *state,
                                  ivp_object_t *obj_r,
                                  const ivp_vec3r_t *anchor_ws,
                                  const ivp_vec3r_t *free_axis_ws,
                                  ivp_object_t *obj_a);

/* Reference set_cardanjoint_ws: rotation about fixed_axis_ws is blocked. */
void ivp_constraint_init_cardan_ws(ivp_constraint_state_t *state,
                                   ivp_object_t *obj_r,
                                   const ivp_vec3r_t *anchor_ws,
                                   const ivp_vec3r_t *fixed_axis_ws,
                                   ivp_object_t *obj_a);

/* Fixed (weld) with explicit anchors; the current relative orientation is
 * the rest orientation. */
void ivp_constraint_init_fixed(ivp_constraint_state_t *state,
                                 ivp_object_t *obj_a,
                                 const ivp_vec3r_t *anchor_a_os,
                                 ivp_object_t *obj_b,
                                 const ivp_vec3r_t *anchor_b_os);

/* Reference set_fixed (relaxed, anchored at R's origin). */
void ivp_constraint_init_fixed_ws(ivp_constraint_state_t *state,
                                  ivp_object_t *obj_r,
                                  ivp_object_t *obj_a);

/* Reference set_constraint_ws with arbitrary fixed dimensions. */
void ivp_constraint_init_general_ws(ivp_constraint_state_t *state,
                                    ivp_object_t *obj_r,
                                    const ivp_vec3r_t *anchor_ws,
                                    const ivp_vec3r_t *known_axis_ws,
                                    unsigned fixed_trans_dim,
                                    unsigned fixed_rot_dim,
                                    ivp_object_t *obj_a);

/* Build from a full reference template (axis types, borders, maximpulse,
 * factors are taken from the template). */
void ivp_constraint_init_from_template(ivp_constraint_state_t *state,
                                       const ivp_constraint_template_t *tmpl);

/* Limit the hinge free axis to [min_angle, max_angle] (reference
 * limit_rotation_axis + limited_axis_stiffness = stiffness). */
void ivp_constraint_set_limits(ivp_constraint_state_t *state,
                                 ivp_real_t min_angle,
                                 ivp_real_t max_angle,
                                 ivp_real_t stiffness,
                                 ivp_real_t damping);

/* Discovery function for ivp_environment_announce_dependent_controller()
 * (reference IVP_Constraint is an IVP_Controller_Dependent). */
int ivp_constraint_discovery(void *ctx, ivp_environment_t *env,
                             ivp_object_handle_t *out_handles, int max_handles);

/* ── Controller callback ────────────────────────────────────────────────── */

/* Constraint solver callback. Pass ivp_constraint_state_t* as ctx.
 * Runs once per PSI on the first simulated movable endpoint. */
void ivp_controller_constraint(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONSTRAINT_H */
