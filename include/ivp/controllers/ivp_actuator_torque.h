/* ivp_actuator_torque.h -- Pure torque actuator
 *
 * Applies a constant torque around a specified axis in object space.
 * Reference semantics (IVP_Actuator_Torque, ivp_actuator.cxx:780-809): the
 * torque is only applied while |omega . axis| <= max_rotation_speed, in both
 * directions; with the reference default max_rotation_speed = 0 the torque
 * only acts while the object does not rotate about the axis at all.
 *
 * Distinct from the rotation motor (ivp_actuator_rot_mot), which derives
 * torque from a power curve. This actuator applies torque directly.
 *
 * Ported from IVP_Actuator_Torque (legacy SDK).
 */
#ifndef IVP_ACTUATOR_TORQUE_H
#define IVP_ACTUATOR_TORQUE_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/utility/ivp_active_value.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Torque actuator state ─────────────────────────────────────────────── */

typedef struct ivp_torque_actuator_state {
    ivp_object_t *obj;              /* object to torque */
    ivp_vec3r_t   axis_os;          /* axis_in_core_coord_system: the object-space axis
                                       of the setup rotated into core space and
                                       normized once (IVP_U_Float_Point) */
    ivp_real_t    torque;           /* constant torque (N*m) */
    ivp_real_t    max_rotation_speed; /* |omega.axis| limit (rad/s); default 0 */

    /* Runtime state */
    ivp_real_t    current_speed;    /* current omega projected on axis
                                       (reference rot_speed_out) */
    bool          active;

    /* Reference IVP_Template_Torque active values:
     * IVP_Actuator_Torque_Active overrides (ivp_torque_actuator_bind_active)
     * and the optional rotation speed output terminal
     * (active_float_rotation_speed_out, set every simulated PSI). */
    ivp_active_float_t *active_float_torque;
    ivp_active_float_t *active_float_max_rotation_speed;
    ivp_active_float_t *active_float_rotation_speed_out;
} ivp_torque_actuator_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_torque_actuator_state_init(ivp_torque_actuator_state_t *state);

void ivp_torque_actuator_state_setup(ivp_torque_actuator_state_t *state,
                                      ivp_object_t *obj,
                                      const ivp_vec3r_t *axis_os,
                                      ivp_real_t torque,
                                      ivp_real_t max_rotation_speed);

/* Reference IVP_Template_Torque setup: two anchors on obj given in object
 * space (IVP_Template_Anchor::set_anchor_position_os + IVP_Anchor::
 * init_anchor, i.e. through world space into core space); the axis is
 * normized(anchor1.core_pos - anchor0.core_pos) in core space.  Like the
 * reference constructor, a non-zero torque wakes the object. */
void ivp_torque_actuator_state_setup_anchors(ivp_torque_actuator_state_t *state,
                                             ivp_object_t *obj,
                                             const ivp_vec3r_t *anchor0_os,
                                             const ivp_vec3r_t *anchor1_os,
                                             ivp_real_t torque,
                                             ivp_real_t max_rotation_speed);

/* Runtime adjustment */
void ivp_torque_actuator_set_torque(ivp_torque_actuator_state_t *state,
                                     ivp_real_t torque);

void ivp_torque_actuator_set_max_rotation_speed(ivp_torque_actuator_state_t *state,
                                                 ivp_real_t max_speed);

/* Reference IVP_Actuator_Torque_Active (IVP_Controller_Factory::create_torque
 * with a template active_float_torque / active_float_max_rotation_speed):
 * subscribes to the given active floats (either may be NULL) and applies
 * their current float values, max_rotation_speed first, through the setters
 * above; later changes call the setters again (which wake the actuator).
 * Call after setup; previous bindings are released first. */
void ivp_torque_actuator_bind_active(ivp_torque_actuator_state_t *state,
                                     ivp_active_float_t *active_float_torque,
                                     ivp_active_float_t *active_float_max_rotation_speed);

/* Reference IVP_Template_Torque::active_float_rotation_speed_out: a terminal
 * that receives the measured rotation speed (as float) whenever the torque
 * is applied to a simulated core; a reference is held (add_reference). */
void ivp_torque_actuator_set_rotation_speed_out(ivp_torque_actuator_state_t *state,
                                                ivp_active_float_t *terminal);

/* Reference ~IVP_Actuator_Torque_Active / ~IVP_Actuator_Torque: removes the
 * dependencies and releases the output terminal.  Call before the state is
 * discarded (done by the environment when the controller is removed). */
void ivp_torque_actuator_unbind_active(ivp_torque_actuator_state_t *state);

/* Discovery function for ivp_environment_announce_dependent_controller():
 * reports the movable objects of the actuator so that their simulation
 * units are merged (reference IVP_Controller_Dependent semantics). */
int ivp_torque_actuator_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles);

/* ── Controller callback ────────────────────────────────────────────────── */

/* Pure torque controller. Pass ivp_torque_actuator_state_t* as ctx. */
void ivp_controller_torque_actuator(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_TORQUE_H */
