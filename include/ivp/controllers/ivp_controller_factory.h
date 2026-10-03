/* ivp_controller_factory.h -- IVP_Controller_Factory
 *
 * Port of ivp_controller/ivp_controller_factory.hxx +
 * ivp_environment_factories.cxx (and IVP_Controller_Factory::create_constraint,
 * ivp_template_constraint.cxx): create a controller from a reference style
 * template and register it exactly as the reference constructor does.
 *
 * Templates (IVP_Template_*): actuator end points are template anchors
 * (IVP_Template_Anchor: an object and a world space position, set with
 * ivp_template_anchor_set_position_os/ws/cs).  At creation every anchor is
 * converted like IVP_Anchor::init_anchor (world -> object space, rounded to
 * float -> core space).  A NULL anchor object, or the environment's static
 * object, anchors the end point in the world.
 *
 * The factory allocates the controller state (the libivp state struct of
 * the module, e.g. ivp_force_actuator_state_t) with the environment's
 * allocator and returns it; use the module's functions to change it.  The
 * environment owns the state:
 *   - ivp_controller_factory_delete is the reference `delete controller`
 *     (silent removal, active values released);
 *   - like the reference, an actuator, check dist or constraint is deleted
 *     together with one of its anchor objects, a motion controller together
 *     with its object;
 *   - everything left is deleted with the environment.
 * Registration follows the constructors: two point actuators and
 * constraints are announced to the environment (dependent controllers), a
 * motion controller is added to the core of its object, a check dist
 * inserts its anchors into the hull managers (no controller), a stabilizer
 * (IVP_Actuator_Four_Point) is NOT announced -- announce it with
 * ivp_controller_factory_announce_stabilizer like the reference runner.
 */
#ifndef IVP_CONTROLLER_FACTORY_H
#define IVP_CONTROLLER_FACTORY_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/controllers/ivp_actuator_force.h>
#include <ivp/controllers/ivp_actuator_torque.h>
#include <ivp/controllers/ivp_actuator_rot_mot.h>
#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_actuator_suspension.h>
#include <ivp/controllers/ivp_actuator_stabilizer.h>
#include <ivp/controllers/ivp_actuator_check_dist.h>
#include <ivp/controllers/ivp_controller_motion.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/utility/ivp_active_value.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── IVP_Template_Anchor ───────────────────────────────────────────────── */

typedef struct ivp_template_anchor {
    ivp_object_t *object;       /* NULL: the environment's static object */
    ivp_vec3r_t   coords_world; /* IVP_U_Point coords_world */
} ivp_template_anchor_t;

/* set_anchor_position_ws: the world space position as given. */
void ivp_template_anchor_set_position_ws(ivp_template_anchor_t *anchor, ivp_object_t *obj,
                                         const ivp_vec3r_t *coords_ws);
/* set_anchor_position_os: the object space position (rounded to an
 * IVP_U_Float_Point) is transformed to world space with the object's
 * current matrix (transform_position_to_world_coords). */
void ivp_template_anchor_set_position_os(ivp_template_anchor_t *anchor, ivp_object_t *obj,
                                         const ivp_vec3r_t *coords_os);
/* set_anchor_position_cs: core space position (float), converted to object
 * space with calc_m_core_f_object, then set_anchor_position_os. */
void ivp_template_anchor_set_position_cs(ivp_template_anchor_t *anchor, ivp_object_t *obj,
                                         const ivp_vec3r_t *coords_cs);

/* ── Templates (constructors give the reference defaults) ─────────────── */

/* IVP_Template_Force (IVP_Template_Two_Point) */
typedef struct ivp_template_force {
    void                  *client_data;
    ivp_template_anchor_t *anchors[2];
    float                  force;               /* N */
    ivp_active_float_t    *active_float_force;  /* optional override */
    bool                   push_first_object;   /* default true */
    bool                   push_second_object;  /* default false */
} ivp_template_force_t;
void ivp_template_force_init(ivp_template_force_t *t);

/* IVP_Template_Torque (both anchors on one object: axis anchor0 -> anchor1) */
typedef struct ivp_template_torque {
    void                  *client_data;
    ivp_template_anchor_t *anchors[2];
    float                  torque;
    ivp_active_float_t    *active_float_torque;
    float                  max_rotation_speed;
    ivp_active_float_t    *active_float_max_rotation_speed;
    ivp_active_float_t    *active_float_rotation_speed_out; /* output terminal */
} ivp_template_torque_t;
void ivp_template_torque_init(ivp_template_torque_t *t);

/* IVP_Template_Rot_Mot */
typedef struct ivp_template_rot_mot {
    void                  *client_data;
    ivp_template_anchor_t *anchors[2];
    float                  max_rotation_speed;
    float                  power;
    float                  max_torque;
    ivp_active_float_t    *active_float_max_rotation_speed;
    ivp_active_float_t    *active_float_power;
    ivp_active_float_t    *active_float_max_torque;
    ivp_active_float_t    *active_float_rotation_speed_out; /* output terminal */
} ivp_template_rot_mot_t;
void ivp_template_rot_mot_init(ivp_template_rot_mot_t *t);

/* IVP_Template_Spring */
typedef struct ivp_template_spring {
    void                     *client_data;
    ivp_template_anchor_t    *anchors[2];
    float                     spring_len;
    bool                      spring_values_are_relative;
    bool                      spring_force_only_on_stretch;
    float                     spring_constant;
    float                     spring_damp;
    float                     rel_pos_damp;
    ivp_spring_force_exceed_t max_len_exceed_type;
    float                     break_max_len;   /* default P_FLOAT_MAX */
    ivp_active_float_t       *active_float_spring_len;
    ivp_active_float_t       *active_float_spring_constant;
    ivp_active_float_t       *active_float_spring_damp;
    ivp_active_float_t       *active_float_spring_rel_pos_damp;
} ivp_template_spring_t;
void ivp_template_spring_init(ivp_template_spring_t *t);

/* IVP_Template_Suspension (anchor 0: body, anchor 1: wheel) */
typedef struct ivp_template_suspension {
    ivp_template_spring_t spring;            /* break_max_len default 10e8 */
    float                 spring_dampening_compression;
    float                 max_body_force;
} ivp_template_suspension_t;
void ivp_template_suspension_init(ivp_template_suspension_t *t);

/* IVP_Template_Stabilizer (IVP_Template_Four_Point) */
typedef struct ivp_template_stabilizer {
    void                  *client_data;
    ivp_template_anchor_t *anchors[4];
    float                  stabi_constant;
    ivp_active_float_t    *active_float_stabi_constant; /* unused by the reference actuator */
} ivp_template_stabilizer_t;
void ivp_template_stabilizer_init(ivp_template_stabilizer_t *t);

/* IVP_Template_Check_Dist */
typedef struct ivp_template_check_dist {
    void                   *client_data;
    ivp_object_t           *objects[2];
    ivp_vec3r_t             position_world_space[2];
    float                   range;
    struct ivp_active_int  *mod_is_outside;  /* optional result terminal */
} ivp_template_check_dist_t;
void ivp_template_check_dist_init(ivp_template_check_dist_t *t);

/* IVP_Template_Controller_Motion */
typedef struct ivp_template_controller_motion {
    float       force_factor;          /* default 0.8 */
    float       damp_factor;           /* default 1.0 */
    float       torque_factor;         /* default 0.8 */
    float       angular_damp_factor;   /* default 1.0 */
    ivp_vec3f_t max_translation_force; /* default 1e6 per axis */
    float       max_torque;            /* default 1e6 */
} ivp_template_controller_motion_t;
void ivp_template_controller_motion_init(ivp_template_controller_motion_t *t);

/* ── IVP_Controller_Factory::create_* ──────────────────────────────────── */

/* create_rotmot: IVP_Actuator_Rot_Mot, or _Active when an active float is
 * given (bound in the order max_rotation_speed, power, max_torque). */
ivp_rot_mot_state_t *ivp_controller_factory_create_rotmot(ivp_environment_t *env,
                                                          const ivp_template_rot_mot_t *templ);

/* create_torque: IVP_Actuator_Torque (a non zero torque wakes the
 * actuator), or _Active (max_rotation_speed bound before torque). */
ivp_torque_actuator_state_t *ivp_controller_factory_create_torque(ivp_environment_t *env,
                                                                  const ivp_template_torque_t *templ);

/* create_controller_motion: IVP_Controller_Motion on obj's core (target:
 * origin / identity orientation until set). */
ivp_motion_state_t *ivp_controller_factory_create_controller_motion(ivp_environment_t *env, ivp_object_t *obj,
                                                                    const ivp_template_controller_motion_t *templ);

/* create_force: IVP_Actuator_Force or _Active. */
ivp_force_actuator_state_t *ivp_controller_factory_create_force(ivp_environment_t *env,
                                                                const ivp_template_force_t *templ);

/* create_spring: IVP_Actuator_Spring or _Active (len, constant, damp,
 * rel_pos_damp); relative values are multiplied with the virtual mass
 * factor of the anchors. */
ivp_spring_state_t *ivp_controller_factory_create_spring(ivp_environment_t *env,
                                                         const ivp_template_spring_t *templ);

/* create_suspension: IVP_Actuator_Suspension (no active values). */
ivp_suspension_state_t *ivp_controller_factory_create_suspension(ivp_environment_t *env,
                                                                 const ivp_template_suspension_t *templ);

/* create_stabilizer: IVP_Actuator_Stabilizer, not announced (inert until
 * ivp_controller_factory_announce_stabilizer). */
ivp_stabilizer_state_t *ivp_controller_factory_create_stabilizer(ivp_environment_t *env,
                                                                 const ivp_template_stabilizer_t *templ);

/* create_check_dist: IVP_Actuator_Check_Dist (evaluated immediately, then on
 * hull events; events through ivp_check_dist_set_callback). */
ivp_check_dist_state_t *ivp_controller_factory_create_check_dist(ivp_environment_t *env,
                                                                 const ivp_template_check_dist_t *templ);

/* create_constraint: NULL when neither object is set; force_factor is
 * raised to P_FLOAT_EPS; an IVP_Constraint_Local announced to the
 * environment. */
ivp_constraint_state_t *ivp_controller_factory_create_constraint(ivp_environment_t *env,
                                                                 const ivp_constraint_template_t *templ);

/* ── Factory controllers ───────────────────────────────────────────────── */

/* `delete controller`: removes a factory controller silently, releases its
 * active values / output terminals (check dist: fires its deleted callback
 * and leaves the hull managers) and frees it.  false if `controller` is not
 * a live factory controller of env. */
bool ivp_controller_factory_delete(ivp_environment_t *env, void *controller);

/* IVP_Controller_Manager::announce_controller_to_environment for a factory
 * stabilizer.  Returns its controller handle (also when already announced). */
ivp_controller_handle_t ivp_controller_factory_announce_stabilizer(ivp_environment_t *env,
                                                                   ivp_stabilizer_state_t *stabilizer);

/* The controller handle of a factory controller (invalid for a check dist
 * and an unannounced stabilizer, or when removed by other means). */
ivp_controller_handle_t ivp_controller_factory_get_handle(ivp_environment_t *env, const void *controller);

/* The template's client_data (IVP_Actuator_Two_Point::client_data etc.). */
void *ivp_controller_factory_get_client_data(ivp_environment_t *env, const void *controller);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_FACTORY_H */
