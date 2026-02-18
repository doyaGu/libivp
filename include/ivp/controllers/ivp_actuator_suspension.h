/* ivp_actuator_suspension.h -- Suspension spring actuator
 *
 * Spring with asymmetric damping and body force clamping for vehicle
 * suspension. Connects a body (obj_a) to a wheel (obj_b).
 * Extension and compression can have different damping rates,
 * and force applied to the body can be independently clamped.
 *
 * Ported from ivp_actuator_spring.cxx (IVP_Actuator_Suspension).
 */
#ifndef IVP_ACTUATOR_SUSPENSION_H
#define IVP_ACTUATOR_SUSPENSION_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Suspension state ──────────────────────────────────────────────────── */

typedef struct ivp_suspension_state {
    /* Attachment */
    ivp_object_t *obj_a;              /* body */
    ivp_object_t *obj_b;              /* wheel */
    ivp_vec3r_t   anchor_a_os;
    ivp_vec3r_t   anchor_b_os;

    /* Spring parameters */
    ivp_real_t    stiffness;          /* k [N/m] */
    ivp_real_t    damping;            /* extension/rebound damping [Ns/m] */
    ivp_real_t    damping_compression;/* compression damping [Ns/m] */
    ivp_real_t    rest_length;
    ivp_real_t    max_body_force;     /* 0 = unlimited */

    /* State (computed per step) */
    ivp_real_t    current_length;
    ivp_real_t    current_force;      /* total force magnitude */
    ivp_real_t    current_body_force; /* clamped force on body */
} ivp_suspension_state_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Initialize with default parameters:
 * stiffness=100, damping=1, damping_compression=1, rest_length=1. */
void ivp_suspension_state_init(ivp_suspension_state_t *state);

/* Configure the suspension between a body and wheel. */
void ivp_suspension_state_setup(ivp_suspension_state_t *state,
                                 ivp_object_t *body,
                                 ivp_object_t *wheel,
                                 const ivp_vec3r_t *anchor_body_os,
                                 const ivp_vec3r_t *anchor_wheel_os,
                                 ivp_real_t stiffness,
                                 ivp_real_t damping,
                                 ivp_real_t damping_compression,
                                 ivp_real_t rest_length);

/* ── Runtime adjustment ────────────────────────────────────────────────── */

void ivp_suspension_set_damping_compression(ivp_suspension_state_t *state,
                                             ivp_real_t value);

void ivp_suspension_set_max_body_force(ivp_suspension_state_t *state,
                                        ivp_real_t value);

/* ── Controller callback ───────────────────────────────────────────────── */

/* Pass ivp_suspension_state_t* as ctx. */
void ivp_controller_suspension(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_SUSPENSION_H */
