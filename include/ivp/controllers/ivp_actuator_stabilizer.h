/* ivp_actuator_stabilizer.h -- Anti-roll bar stabilizer actuator
 *
 * Connects 4 objects as 2 pairs and applies the legacy IVP stabilizer law.
 * Despite the name, the reference force direction increases an existing
 * distance difference between the pairs; libivp preserves that behavior.
 *
 * Ported from ivp_actuator.cxx (IVP_Actuator_Stabilizer).
 */
#ifndef IVP_ACTUATOR_STABILIZER_H
#define IVP_ACTUATOR_STABILIZER_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/dynamics/ivp_anchor.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Stabilizer state ──────────────────────────────────────────────────── */

typedef struct ivp_stabilizer_state {
    /* Pair 0: (obj[0], obj[1]), Pair 1: (obj[2], obj[3]) */
    ivp_object_t *obj[4];
    ivp_vec3r_t   anchor_os[4];    /* IVP_Anchor::core_pos (float-exact); setup takes
                                      object-space coordinates */
    ivp_anchor_t  anchors[4];      /* IVP_Actuator_Four_Point::anchors (object_pos, object list) */

    /* Parameters */
    ivp_real_t    stabi_constant;  /* [N/m] */
    bool          active;

    /* State (computed per step) */
    ivp_real_t    current_diff_dist;
} ivp_stabilizer_state_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Initialize a stabilizer with default parameters.
 * All objects and anchors must be set via setup. */
void ivp_stabilizer_state_init(ivp_stabilizer_state_t *state);

/* Configure the stabilizer with 4 objects, 4 anchors, and stiffness.
 * Pair 0 = (obj0, obj1), Pair 1 = (obj2, obj3). */
void ivp_stabilizer_state_setup(ivp_stabilizer_state_t *state,
                                 ivp_object_t *obj0,
                                 ivp_object_t *obj1,
                                 ivp_object_t *obj2,
                                 ivp_object_t *obj3,
                                 const ivp_vec3r_t *anchor0,
                                 const ivp_vec3r_t *anchor1,
                                 const ivp_vec3r_t *anchor2,
                                 const ivp_vec3r_t *anchor3,
                                 ivp_real_t stabi_constant);

/* ── Runtime adjustment ────────────────────────────────────────────────── */

void ivp_stabilizer_set_constant(ivp_stabilizer_state_t *state,
                                  ivp_real_t value);

/* Discovery function for ivp_environment_announce_dependent_controller():
 * reports the movable objects of the actuator so that their simulation
 * units are merged (reference IVP_Controller_Dependent semantics). */
int ivp_stabilizer_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles);

/* ── Controller callback ───────────────────────────────────────────────── */

/* Pass ivp_stabilizer_state_t* as ctx. */
void ivp_controller_stabilizer(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_STABILIZER_H */
