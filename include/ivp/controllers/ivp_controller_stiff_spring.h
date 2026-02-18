/* ivp_controller_stiff_spring.h -- Stiff spring controller
 *
 * Like ivp_actuator_spring but uses implicit integration for numerical
 * stability at high stiffness values. Safe for k * dt^2 / m >> 1.
 *
 * Instead of applying F = -k*x as an explicit force, computes the
 * velocity correction via the implicit Euler solution, preventing
 * overshoot and oscillation instability.
 *
 * Ported from ivp_controller_stiff_spring.hxx.
 */
#ifndef IVP_CONTROLLER_STIFF_SPRING_H
#define IVP_CONTROLLER_STIFF_SPRING_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Stiff spring state ────────────────────────────────────────────────── */

typedef struct ivp_stiff_spring_state {
    /* Attachment */
    ivp_object_t *obj_a;            /* first object (NULL = world) */
    ivp_object_t *obj_b;            /* second object (NULL = world) */
    ivp_vec3r_t   anchor_a_os;      /* anchor in A's body space */
    ivp_vec3r_t   anchor_b_os;      /* anchor in B's body space */

    /* Spring parameters */
    ivp_real_t    stiffness;        /* spring constant k (N/m) */
    ivp_real_t    damping;          /* damping constant c (N·s/m) */
    ivp_real_t    rest_length;      /* rest length (m) */
    ivp_real_t    max_dv;           /* max velocity change per step (0 = unlimited) */

    bool          active;

    /* Runtime output */
    ivp_real_t    current_length;
} ivp_stiff_spring_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_stiff_spring_state_init(ivp_stiff_spring_state_t *state);

void ivp_stiff_spring_state_setup(ivp_stiff_spring_state_t *state,
                                    ivp_object_t *obj_a,
                                    const ivp_vec3r_t *anchor_a_os,
                                    ivp_object_t *obj_b,
                                    const ivp_vec3r_t *anchor_b_os,
                                    ivp_real_t stiffness,
                                    ivp_real_t damping,
                                    ivp_real_t rest_length);

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_stiff_spring(const ivp_event_sim_t *event_sim,
                                   ivp_object_t *obj,
                                   void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_STIFF_SPRING_H */
