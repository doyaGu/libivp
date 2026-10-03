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
#include <ivp/utility/ivp_active_value.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Force actuator state ───────────────────────────────────────────────── */

typedef struct ivp_force_actuator_state {
    /* Objects and anchor points */
    ivp_object_t *obj_a;            /* first object (NULL = world) */
    ivp_object_t *obj_b;            /* second object (NULL = world) */
    /* Reference IVP_Anchor::core_pos (float-exact core-space anchors); the
     * setup takes object-space coordinates (set_anchor_position_os), world
     * coordinates for a NULL object (environment static object). */
    ivp_vec3r_t   anchor_a_os;
    ivp_vec3r_t   anchor_b_os;

    /* Force parameters */
    ivp_real_t    force;            /* force magnitude (N); positive pushes A away from B (reference IVP_Actuator_Force) */
    bool          push_first_object;  /* apply force to obj_a? */
    bool          push_second_object; /* apply force to obj_b? */

    /* State */
    bool          active;
    ivp_real_t    current_distance;   /* current distance between anchors */

    /* Reference IVP_Actuator_Force_Active: optional override of the force
     * (IVP_Template_Force::active_float_force), see
     * ivp_force_actuator_bind_active. */
    ivp_active_float_t *active_float_force;
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

/* Runtime force adjustment (reference IVP_Actuator_Force::set_force):
 * no-op for an unchanged value, else wakes the actuator. */
void ivp_force_actuator_set_force(ivp_force_actuator_state_t *state,
                                    ivp_real_t force);

/* Reference IVP_Actuator_Force_Active (IVP_Controller_Factory::create_force
 * with a template active_float_force): subscribes the actuator to
 * active_float_force (IVP_U_Active_Float::add_dependency, holds a reference)
 * and applies its current float value through ivp_force_actuator_set_force;
 * every later change of the value calls set_force again (active_float_changed),
 * which wakes the actuator.  Call after setup; a previous binding is
 * released first; NULL just releases it. */
void ivp_force_actuator_bind_active(ivp_force_actuator_state_t *state,
                                    ivp_active_float_t *active_float_force);

/* Reference ~IVP_Actuator_Force_Active: removes the dependency.  Call before
 * the state is discarded (done by the environment when the actuator
 * controller is removed, see the built-in controller table). */
void ivp_force_actuator_unbind_active(ivp_force_actuator_state_t *state);

/* Discovery function for ivp_environment_announce_dependent_controller():
 * reports the movable objects of the actuator so that their simulation
 * units are merged (reference IVP_Controller_Dependent semantics). */
int ivp_force_actuator_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles);

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_force_actuator(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_FORCE_H */
