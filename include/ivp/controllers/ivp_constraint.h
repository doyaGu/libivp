/* ivp_constraint.h -- Constraint system for IVP C17
 *
 * A constraint limits relative motion between two objects (or one object
 * and world space). The constraint is solved iteratively using impulses.
 *
 * Supported constraint types:
 *   BALL_SOCKET - 3-DOF position constraint (ball joint)
 *   HINGE       - 5-DOF: ball_socket + axis alignment
 *   FIXED       - 6-DOF: lock position and orientation
 *
 * Each constraint stores attachment points in body space and is solved
 * by applying corrective impulses each PSI step.
 *
 * Ported from ivp_constraint.hxx / ivp_constraint_local.hxx.
 */
#ifndef IVP_CONSTRAINT_H
#define IVP_CONSTRAINT_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constraint type ────────────────────────────────────────────────────── */

typedef enum ivp_constraint_type {
    IVP_CONSTRAINT_BALL_SOCKET = 0,  /* position only (3 DOF removed) */
    IVP_CONSTRAINT_HINGE      = 1,  /* position + 1 axis (5 DOF removed) */
    IVP_CONSTRAINT_FIXED      = 2   /* position + orientation (6 DOF removed) */
} ivp_constraint_type_t;

/* ── Constraint limits ──────────────────────────────────────────────────── */

typedef struct ivp_constraint_limits {
    bool       limited;          /* are angular limits active? */
    ivp_real_t min_angle;        /* minimum angle (rad), hinge only */
    ivp_real_t max_angle;        /* maximum angle (rad), hinge only */
    ivp_real_t limit_stiffness;  /* spring constant at limits */
    ivp_real_t limit_damping;    /* damping at limits */
} ivp_constraint_limits_t;

/* ── Constraint state ───────────────────────────────────────────────────── */

typedef struct ivp_constraint_state {
    ivp_constraint_type_t type;

    /* Attachment */
    ivp_object_t *obj_a;            /* first object (NULL = world) */
    ivp_object_t *obj_b;            /* second object (required) */
    ivp_vec3r_t   anchor_a_os;      /* anchor in A's body space */
    ivp_vec3r_t   anchor_b_os;      /* anchor in B's body space */
    ivp_vec3r_t   axis_a_os;        /* hinge axis in A's body space */
    ivp_vec3r_t   axis_b_os;        /* hinge axis in B's body space */
    ivp_quatr_t   ref_q_a;          /* reference orientation for A (fixed) */
    ivp_quatr_t   ref_q_b;          /* reference orientation for B (fixed) */

    /* Solver parameters */
    ivp_real_t    tau;              /* Baumgarte stabilization factor (0.1-0.3) */
    ivp_real_t    damping;          /* velocity damping (0-1) */
    int           solver_iterations; /* impulse iterations per step */

    /* Angular limits (hinge only) */
    ivp_constraint_limits_t limits;

    /* Runtime state */
    bool          active;
    ivp_real_t    accumulated_impulse; /* debug: total impulse this step */
} ivp_constraint_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

/* Initialize a ball-socket constraint between two objects. */
void ivp_constraint_init_ball_socket(ivp_constraint_state_t *state,
                                      ivp_object_t *obj_a,
                                      const ivp_vec3r_t *anchor_a_os,
                                      ivp_object_t *obj_b,
                                      const ivp_vec3r_t *anchor_b_os);

/* Initialize a hinge constraint between two objects.
 * The axis vectors define the hinge rotation axis in each body's space. */
void ivp_constraint_init_hinge(ivp_constraint_state_t *state,
                                 ivp_object_t *obj_a,
                                 const ivp_vec3r_t *anchor_a_os,
                                 const ivp_vec3r_t *axis_a_os,
                                 ivp_object_t *obj_b,
                                 const ivp_vec3r_t *anchor_b_os,
                                 const ivp_vec3r_t *axis_b_os);

/* Initialize a fixed constraint (welds two objects together).
 * Records current relative orientation as reference. */
void ivp_constraint_init_fixed(ivp_constraint_state_t *state,
                                 ivp_object_t *obj_a,
                                 const ivp_vec3r_t *anchor_a_os,
                                 ivp_object_t *obj_b,
                                 const ivp_vec3r_t *anchor_b_os);

/* Set angular limits for a hinge constraint. */
void ivp_constraint_set_limits(ivp_constraint_state_t *state,
                                 ivp_real_t min_angle,
                                 ivp_real_t max_angle,
                                 ivp_real_t stiffness,
                                 ivp_real_t damping);

/* ── Controller callback ────────────────────────────────────────────────── */

/* Constraint solver callback. Pass ivp_constraint_state_t* as ctx.
 * Only acts when obj matches one of the constraint's objects. */
void ivp_controller_constraint(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONSTRAINT_H */
