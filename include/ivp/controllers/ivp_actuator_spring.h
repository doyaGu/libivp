/* ivp_actuator_spring.h -- Spring force actuator
 *
 * Connects two objects (or one object to a world anchor) with a spring.
 * Applies Hooke's law force: F = -k * (distance - rest_length)
 * With optional damping: F_damp = -c * closing_velocity
 *
 * Ported from ivp_actuator_spring.hxx.
 */
#ifndef IVP_ACTUATOR_SPRING_H
#define IVP_ACTUATOR_SPRING_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Spring state ───────────────────────────────────────────────────────── */

typedef struct ivp_spring_state {
    /* Attachment: object pointers and local anchor points */
    ivp_object_t *obj_a;            /* first object (NULL = world) */
    ivp_object_t *obj_b;            /* second object (NULL = world) */
    ivp_vec3r_t   anchor_a_os;      /* anchor in object A's core space */
    ivp_vec3r_t   anchor_b_os;      /* anchor in object B's core space */

    /* Spring parameters */
    ivp_real_t    stiffness;        /* spring constant k (N/m) */
    ivp_real_t    damping;          /* damping constant c (N·s/m) */
    ivp_real_t    rest_length;      /* rest length (m) */
    ivp_real_t    max_force;        /* max force clamp (0 = unlimited) */

    /* State (computed per step) */
    ivp_real_t    current_length;   /* current spring length */
    ivp_real_t    current_force;    /* current force magnitude */
} ivp_spring_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

/* Initialize a spring with default parameters.
 * Both objects and anchors must be set separately. */
void ivp_spring_state_init(ivp_spring_state_t *state);

/* Configure a spring between two objects at given anchor points.
 * anchor_a_os / anchor_b_os are in each object's core (body) space.
 * If obj_a or obj_b is NULL, the corresponding anchor is in world space. */
void ivp_spring_state_setup(ivp_spring_state_t *state,
                              ivp_object_t *obj_a,
                              const ivp_vec3r_t *anchor_a_os,
                              ivp_object_t *obj_b,
                              const ivp_vec3r_t *anchor_b_os,
                              ivp_real_t stiffness,
                              ivp_real_t damping,
                              ivp_real_t rest_length);

/* ── Controller callback ────────────────────────────────────────────────── */

/* The spring controller callback. Pass an ivp_spring_state_t* as ctx.
 * NOTE: This controller only affects the two objects in the spring,
 * not the object argument. It ignores the object argument. */
void ivp_controller_spring(const ivp_event_sim_t *event_sim,
                             ivp_object_t *obj,
                             void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_SPRING_H */
