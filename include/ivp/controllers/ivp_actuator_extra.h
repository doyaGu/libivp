/* ivp_actuator_extra.h -- IVP_Actuator_Extra (demo actuator)
 *
 * Port of IVP_Actuator_Extra / IVP_Template_Extra / IVP_Extra_Info
 * (ivp_controller/ivp_actuator.hxx:509-537, ivp_actuator_info.hxx,
 * ivp_actuator.cxx:431-1026).  The reference marks it "for ipion demos
 * only": a two point actuator (priority IVP_CP_ACTUATOR) with two modes,
 *   - puck force (info.is_puck_force): pushes the two anchors apart along
 *     their horizontal (y = 0) direction with a force that is hard-wired to
 *     0 in the reference, i.e. the pushes are zero impulses;
 *   - float cam (info.is_float_cam): keeps its cores awake and moves a
 *     smoothed camera position / look point behind anchor 0 (no force);
 * plus active float subscriptions (force, bomb, float cam parameters) whose
 * change callback does nothing.  The constructor wakes the actuator.
 *
 * Anchors are given in the core space of their objects (like the other
 * libivp actuators).  Registration: ivp_environment_add_controller (or
 * announce_dependent_controller with ivp_actuator_extra_discovery).
 */
#ifndef IVP_ACTUATOR_EXTRA_H
#define IVP_ACTUATOR_EXTRA_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/dynamics/ivp_anchor.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/utility/ivp_active_value.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reference IVP_Extra_Info (all fields cleared by its constructor). */
typedef struct ivp_extra_info {
    void *client_data;

    bool                is_float_cam;
    ivp_active_float_t *active_float_bomb;
    double              range;               /* e.g. for bomb */

    ivp_active_float_t *mod_fc_height;
    ivp_active_float_t *mod_fc_target_height;
    ivp_active_float_t *mod_fc_dist;
    ivp_active_float_t *mod_fc_speed;
    ivp_active_float_t *active_float_force;  /* the force */

    int                 is_physic_cam;

    int                 is_puck_force;
    ivp_active_float_t *mod_pf_forward;
    ivp_active_float_t *mod_pf_sideward;
} ivp_extra_info_t;

/* Reference IVP_Template_Extra (IVP_Template_Two_Point + info). */
typedef struct ivp_template_extra {
    void             *client_data;
    ivp_object_t     *objects[2];      /* anchor objects */
    ivp_vec3r_t       anchors_cs[2];   /* anchor positions (core space) */
    ivp_extra_info_t  info;
} ivp_template_extra_t;

typedef struct ivp_actuator_extra_state {
    void          *client_data;
    ivp_object_t  *obj_a;              /* anchor 0 */
    ivp_object_t  *obj_b;              /* anchor 1 */
    ivp_vec3r_t    anchor_a_os;        /* core space */
    ivp_vec3r_t    anchor_b_os;
    ivp_anchor_t   anchors[2];         /* IVP_Actuator_Two_Point::anchors (object_pos, object list) */
    ivp_extra_info_t info;

    ivp_vec3d_t    current_float_cam_pos;  /* IVP_U_Point */
    ivp_vec3d_t    current_look_point;
    bool           current_vals_are_set;   /* for first time */

    bool           dependencies_installed;
} ivp_actuator_extra_state_t;

/* IVP_Extra_Info::IVP_Extra_Info / IVP_Template_Extra defaults (all zero). */
void ivp_extra_info_init(ivp_extra_info_t *info);
void ivp_template_extra_init(ivp_template_extra_t *templ);

/* IVP_Actuator_Extra constructor: copies the template, subscribes to
 * active_float_force, active_float_bomb and the four mod_fc_* values,
 * places the float cam at (0, -100, 0) looking at the origin and wakes the
 * actuator. */
void ivp_actuator_extra_init(ivp_actuator_extra_state_t *state, const ivp_template_extra_t *templ);

/* ~IVP_Actuator_Extra: removes the active float dependencies.  Call before
 * the state is discarded (done by the environment when the controller is
 * removed, see the built-in controller table). */
void ivp_actuator_extra_destroy(ivp_actuator_extra_state_t *state);

/* get_force (needed by bomb): active_float_force value, else 0. */
double ivp_actuator_extra_get_force(const ivp_actuator_extra_state_t *state);

/* get_float_cam_props: the mod_fc_* values (0 when unset); returns the
 * current float cam position. */
const ivp_vec3d_t *ivp_actuator_extra_get_float_cam_props(ivp_actuator_extra_state_t *state,
                                                          double *fc_height_out,
                                                          double *fc_target_height_out,
                                                          double *fc_dist_out,
                                                          double *fc_speed_out);

/* calc_float_cam_matrix: camera matrix (columns v0, v1, view direction,
 * translation = cam position) from the current float cam values. */
void ivp_actuator_extra_calc_float_cam_matrix(const ivp_actuator_extra_state_t *state,
                                              ivp_transform_r_t *cam_matrix_out);

/* Discovery for ivp_environment_announce_dependent_controller: the movable
 * anchor objects (IVP_Actuator_Two_Point::actuator_controlled_cores). */
int ivp_actuator_extra_discovery(void *ctx, ivp_environment_t *env,
                                 ivp_object_handle_t *out_handles, int max_handles);

/* do_simulation_controller (do_puck_force / do_float_cam).
 * Pass ivp_actuator_extra_state_t* as ctx; priority IVP_CP_ACTUATOR. */
void ivp_controller_actuator_extra(const ivp_event_sim_t *event_sim,
                                   ivp_object_t *obj,
                                   void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_EXTRA_H */
