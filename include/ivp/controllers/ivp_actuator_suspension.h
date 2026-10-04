/* ivp_actuator_suspension.h -- Suspension spring actuator
 *
 * Spring with asymmetric damping and body force clamping for vehicle
 * suspension. Connects a body (obj_a, anchor 0) to a wheel (obj_b, anchor 1).
 *
 * Port of IVP_Actuator_Suspension (ivp_actuator_spring.cxx:296-379):
 *   force    = (dlen - rest_length) * k - c * damp_speed
 *              (c = damping when extending, damping_compression when closing)
 *   wheel   <- dir * force * dt
 *   body    <- dir * clip(-force, +-max_body_force) * dt
 * Reference semantics: max_body_force is a hard clip; the reference default
 * is 0, which means the body receives NO spring force.
 */
#ifndef IVP_ACTUATOR_SUSPENSION_H
#define IVP_ACTUATOR_SUSPENSION_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/dynamics/ivp_anchor.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Suspension state ──────────────────────────────────────────────────── */

typedef struct ivp_suspension_state {
    /* Attachment */
    ivp_object_t *obj_a;              /* body (anchor 0) */
    ivp_object_t *obj_b;              /* wheel (anchor 1) */
    /* Reference IVP_Anchor::core_pos (float-exact core-space anchors); the
     * setup takes object-space coordinates (set_anchor_position_os). */
    ivp_vec3r_t   anchor_a_os;
    ivp_vec3r_t   anchor_b_os;
    ivp_anchor_t  anchors[2];         /* IVP_Actuator_Two_Point::anchors (object_pos, object list) */

    /* Spring parameters (already multiplied with spring_values_factor) */
    ivp_real_t    stiffness;          /* k [N/m] */
    ivp_real_t    damping;            /* extension/rebound damping [Ns/m] */
    ivp_real_t    damping_compression;/* compression damping [Ns/m] */
    ivp_real_t    rest_length;
    ivp_real_t    max_body_force;     /* hard clip of body force; 0 = no body force */
    ivp_real_t    spring_values_factor; /* reference relative factor, default 1 */

    /* State (computed per step) */
    ivp_real_t    current_length;
    ivp_real_t    current_force;      /* total force magnitude */
    ivp_real_t    current_body_force; /* clamped force on body */
} ivp_suspension_state_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Initialize with the reference template defaults (IVP_Template_Suspension):
 * all constants 0, max_body_force 0. */
void ivp_suspension_state_init(ivp_suspension_state_t *state);

/* Configure the suspension between a body and wheel.  max_body_force is not
 * touched (set it with ivp_suspension_set_max_body_force). */
void ivp_suspension_state_setup(ivp_suspension_state_t *state,
                                 ivp_object_t *body,
                                 ivp_object_t *wheel,
                                 const ivp_vec3r_t *anchor_body_os,
                                 const ivp_vec3r_t *anchor_wheel_os,
                                 ivp_real_t stiffness,
                                 ivp_real_t damping,
                                 ivp_real_t damping_compression,
                                 ivp_real_t rest_length);

/* ── Runtime adjustment (reference setters wake the actuator) ─────────── */

/* Reference IVP_Actuator_Suspension::set_spring_damp_compression:
 * damping_compression = value * spring_values_factor (the early return
 * compares the stored, already multiplied value with value, as the
 * reference does). */
void ivp_suspension_set_damping_compression(ivp_suspension_state_t *state,
                                             ivp_real_t value);

/* Setters inherited from IVP_Actuator_Spring (ivp_actuator_spring.cxx:80-114):
 * set_constant / set_damp multiply with spring_values_factor and always wake;
 * set_len returns early on an unchanged length. */
void ivp_suspension_set_constant(ivp_suspension_state_t *state, ivp_real_t value);
void ivp_suspension_set_damp(ivp_suspension_state_t *state, ivp_real_t value);
void ivp_suspension_set_len(ivp_suspension_state_t *state, ivp_real_t value);

void ivp_suspension_set_max_body_force(ivp_suspension_state_t *state,
                                        ivp_real_t value);

/* Discovery function for ivp_environment_announce_dependent_controller():
 * reports the movable objects of the actuator so that their simulation
 * units are merged (reference IVP_Controller_Dependent semantics). */
int ivp_suspension_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles);

/* ── Controller callback ───────────────────────────────────────────────── */

/* Pass ivp_suspension_state_t* as ctx. */
void ivp_controller_suspension(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_SUSPENSION_H */
