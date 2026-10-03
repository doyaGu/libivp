/* ivp_object_attach.h -- port of IVP_Object_Attach
 * (ivp_physics/ivp_object_attach.{hxx,cxx})
 *
 * Simple functions that attach an object to another: the attached object
 * moves into the parent's core (physical_core / friction_core /
 * original_core), the merged entity keeps the physical values of the
 * parent (mass, rot_inertia, speed, rot_speed ...).
 *
 * Reference limits (libivp does exactly what the reference does):
 *   - attach_object does not remove the attached object's contact points; if
 *     its old core is deleted (it was the core's only object) while friction
 *     systems still use that core, the reference keeps dangling pointers.
 *     Attach objects without contact points.
 *   - detach_object sets extra_radius = tmpl->extra_radius, so a detached
 *     ball's collision radius becomes the template's extra radius (the ball
 *     radius is not added back as IVP_Ball's constructor does).
 */
#ifndef IVP_OBJECT_ATTACH_H
#define IVP_OBJECT_ATTACH_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/dynamics/ivp_object.h>

#ifdef __cplusplus
extern "C" {
#endif

/* max_distance_attached_object_to_parents_mass_center < 0: use the current
 * distance of the two mass centers (reference default -1.0f) */
#define IVP_OBJECT_ATTACH_CURRENT_DISTANCE (-1.0)

/* IVP_Object_Attach::attach_object: max_distance_attached_object_to_parents_
 * mass_center is used for the collision detection (grows the parent core's
 * radius) and should be set to a reasonable value.  The attached object
 * keeps its current place (reposition_object_Ros in world space); its old
 * core is deleted when it was the core's only object.  parent ==
 * attached_object is ignored. */
void ivp_object_attach_object(ivp_object_t *parent, ivp_object_t *attached_object,
                              double max_distance_attached_object_to_parents_mass_center);

/* IVP_Object_Attach::detach_object: creates a new physical instance (core)
 * for an attached object; may also be used to change the physical
 * parameters of an object that is not attached.  tmpl provides the physical
 * parameters (mass, inertia, damping, mass center override, piling, extra
 * radius, material, client data, nocoll group ident). */
void ivp_object_detach_object(ivp_object_t *attached_object, const ivp_object_template_t *tmpl);

/* IVP_Object_Attach::reposition_object_Ros: q_Ros_f_Aos / shift_Ros_f_Aos
 * define the attached object in the reference object space (parent NULL:
 * world space); check_before_moving is ignored like in the reference. */
ivp_result_t ivp_object_reposition_object_Ros(ivp_object_t *parent, ivp_object_t *attached_object,
                                              const ivp_quatr_t *q_Ros_f_Aos, const ivp_vec3r_t *shift_Ros_f_Aos,
                                              bool check_before_moving);

#ifdef __cplusplus
}
#endif

#endif /* IVP_OBJECT_ATTACH_H */
