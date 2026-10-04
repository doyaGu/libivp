/* ivp_actuator_rot_mot.h -- Rotation motor actuator
 *
 * A rotation motor applies a power-limited torque around an axis fixed in
 * one object.  Port of IVP_Actuator_Rot_Mot (ivp_actuator.cxx:597-665):
 *
 *   dw = omega . axis_cs   (0 if it opposes the power sign)
 *   if |dw| > max_rotation_speed: no torque
 *   dw = max(|dw|, 0.1)
 *   a  = power / (dw * |I (.) axis_cs|), clipped to +-max_torque unless
 *        max_torque == 0 (0 means "no clipping")
 *   rot_speed_change += (axis_cs (.) inv_I) * a * dt
 *
 * Both anchors belong to the driven object (the reference requires this).
 * The driven object is anchor 0's object (obj_a); when obj_a is NULL the
 * motor drives obj_b and both anchors belong to obj_b.  The setup takes
 * object-space anchors (set_anchor_position_os) and stores their float
 * core_pos (IVP_Anchor::init_anchor).
 */
#ifndef IVP_ACTUATOR_ROT_MOT_H
#define IVP_ACTUATOR_ROT_MOT_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/dynamics/ivp_anchor.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/utility/ivp_active_value.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Rotation motor state ───────────────────────────────────────────────── */

typedef struct ivp_rot_mot_state {
    /* Anchor objects (reference: both anchors on the driven object) */
    ivp_object_t *obj_a;            /* anchor 0 object = driven object */
    ivp_object_t *obj_b;            /* anchor 1 object (driven if obj_a NULL) */

    /* Axis definition via two anchor points */
    ivp_vec3r_t anchor_a_os;        /* anchor 0 core_pos (float-exact, core space) */
    ivp_vec3r_t anchor_b_os;        /* anchor 1 core_pos (float-exact, core space) */
    ivp_anchor_t anchors[2];        /* IVP_Actuator_Two_Point::anchors (object_pos, object list) */
    /* The rotation axis (core space of the driven object) is
     * normalize(anchor_1 - anchor_0). */

    /* Motor parameters (reference template defaults: all 0) */
    ivp_real_t power;               /* motor power [W]; sign gives direction */
    ivp_real_t max_torque;          /* clip of the drive term; 0 = no clip */
    ivp_real_t max_rotation_speed;  /* |omega.axis| above which no torque acts */

    /* Runtime state */
    ivp_real_t current_speed;       /* current angular speed on axis
                                       (reference rot_speed_out) */
    ivp_real_t current_torque;      /* current applied torque */
    bool       active;

    /* Reference IVP_Template_Rot_Mot active values:
     * IVP_Actuator_Rot_Mot_Active overrides (ivp_rot_mot_bind_active) and
     * the optional rotation speed output terminal. */
    ivp_active_float_t *active_float_max_rotation_speed;
    ivp_active_float_t *active_float_power;
    ivp_active_float_t *active_float_max_torque;
    ivp_active_float_t *active_float_rotation_speed_out;
} ivp_rot_mot_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_rot_mot_state_init(ivp_rot_mot_state_t *state);

void ivp_rot_mot_state_setup(ivp_rot_mot_state_t *state,
                              ivp_object_t *obj_a,
                              const ivp_vec3r_t *anchor_a_os,
                              ivp_object_t *obj_b,
                              const ivp_vec3r_t *anchor_b_os,
                              ivp_real_t power,
                              ivp_real_t max_torque,
                              ivp_real_t max_rotation_speed);

/* Runtime adjustment */
void ivp_rot_mot_set_power(ivp_rot_mot_state_t *state, ivp_real_t power);
void ivp_rot_mot_set_max_torque(ivp_rot_mot_state_t *state, ivp_real_t max_torque);
void ivp_rot_mot_set_max_rotation_speed(ivp_rot_mot_state_t *state, ivp_real_t max_speed);

/* Reference IVP_Actuator_Rot_Mot_Active (IVP_Controller_Factory::create_rotmot
 * with template active floats): subscribes to the given active floats (each
 * may be NULL) and applies their current float values in the reference
 * order max_rotation_speed, power, max_torque through the setters above;
 * later changes call the setters again (which wake the actuator).  Call
 * after setup; previous bindings are released first. */
void ivp_rot_mot_bind_active(ivp_rot_mot_state_t *state,
                             ivp_active_float_t *active_float_max_rotation_speed,
                             ivp_active_float_t *active_float_power,
                             ivp_active_float_t *active_float_max_torque);

/* Reference IVP_Template_Rot_Mot::active_float_rotation_speed_out: terminal
 * receiving the (float) rotation speed every PSI the motor runs with a
 * non-zero power; a reference is held. */
void ivp_rot_mot_set_rotation_speed_out(ivp_rot_mot_state_t *state,
                                        ivp_active_float_t *terminal);

/* Reference ~IVP_Actuator_Rot_Mot_Active / ~IVP_Actuator_Rot_Mot: removes
 * the dependencies and releases the output terminal. */
void ivp_rot_mot_unbind_active(ivp_rot_mot_state_t *state);

/* Discovery function for ivp_environment_announce_dependent_controller():
 * reports the movable objects of the actuator so that their simulation
 * units are merged (reference IVP_Controller_Dependent semantics). */
int ivp_rot_mot_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles);

/* ── Controller callback ────────────────────────────────────────────────── */

/* Rotation motor controller. Pass ivp_rot_mot_state_t* as ctx. */
void ivp_controller_rot_mot(const ivp_event_sim_t *event_sim,
                              ivp_object_t *obj,
                              void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_ROT_MOT_H */
