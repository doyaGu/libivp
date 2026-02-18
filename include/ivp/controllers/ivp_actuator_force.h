/* ivp_actuator_force.h -- Anchor-based force actuator
 *
 * Applies a force along the line between two anchor points.
 * Can push/pull either or both objects (e.g. magnetic attraction).
 * Force direction follows the line connecting the two anchors.
 *
 * Ported from IVP_Actuator_Force (ivp_actuator.hxx).
 */
#ifndef IVP_ACTUATOR_FORCE_H
#define IVP_ACTUATOR_FORCE_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Force actuator state ───────────────────────────────────────────────── */

typedef struct ivp_force_actuator_state {
    /* Objects and anchor points */
    ivp_object_t *obj_a;            /* first object (NULL = world) */
    ivp_object_t *obj_b;            /* second object (NULL = world) */
    ivp_vec3r_t   anchor_a_os;      /* anchor in object A's body space */
    ivp_vec3r_t   anchor_b_os;      /* anchor in object B's body space */

    /* Force parameters */
    ivp_real_t    force;            /* force magnitude (N, positive = attract) */
    bool          push_first_object;  /* apply force to obj_a? */
    bool          push_second_object; /* apply force to obj_b? */

    /* State */
    bool          active;
    ivp_real_t    current_distance;   /* current distance between anchors */
} ivp_force_actuator_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_force_actuator_state_init(ivp_force_actuator_state_t *state);

void ivp_force_actuator_state_setup(ivp_force_actuator_state_t *state,
                                     ivp_object_t *obj_a,
                                     const ivp_vec3r_t *anchor_a_os,
                                     ivp_object_t *obj_b,
                                     const ivp_vec3r_t *anchor_b_os,
                                     ivp_real_t force,
                                     bool push_first,
                                     bool push_second);

/* Runtime force adjustment (e.g. toggle magnet on/off) */
void ivp_force_actuator_set_force(ivp_force_actuator_state_t *state,
                                    ivp_real_t force);

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_force_actuator(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_FORCE_H */
