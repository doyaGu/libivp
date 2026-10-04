/* ivp_anchor.h -- IVP_Anchor (ivp_physics/ivp_anchor.hxx)
 *
 * An anchor is a position in an object's space, used by the two- and
 * four-point actuators.  Every object keeps a linked list of the anchors of
 * its registered actuators; IVP_Real_Object::set_new_m_object_f_core
 * (detach_object, reposition_object_Ros, merging) recomputes their
 * core_pos from object_pos, so the actuators follow the object into its
 * new core space.
 *
 * The actuator states own their anchors.  object_pos is set by the state
 * setup (IVP_Anchor::init_anchor); the anchor is linked into the object's
 * list (and core_pos points into the state) while the actuator is
 * registered in an environment.
 */
#ifndef IVP_ANCHOR_H
#define IVP_ANCHOR_H

#include <stdbool.h>
#include <ivp/math/ivp_vec3.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_object;

typedef struct ivp_anchor {
    struct ivp_anchor *anchor_next_in_object;   /* linked list of anchors per object */
    struct ivp_anchor *anchor_prev_in_object;
    struct ivp_object *l_anchor_object;         /* NULL: the static object (never moved) */
    ivp_vec3f_t        object_pos;              /* IVP_U_Float_Point, object space */
    ivp_vec3r_t       *core_pos;                /* the actuator's core_pos while linked */
    bool               linked;
} ivp_anchor_t;

/* IVP_Real_Object::insert_anchor: link the anchor into the list of its
 * object (no-op for an anchor without object or already linked);
 * core_pos is the actuator's float-exact core_pos. */
void ivp_anchor_link(ivp_anchor_t *anchor, ivp_vec3r_t *core_pos);

/* IVP_Real_Object::remove_anchor (no-op when not linked). */
void ivp_anchor_unlink(ivp_anchor_t *anchor);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ANCHOR_H */
