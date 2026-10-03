/* ivp_listener_object.h -- object life cycle listeners
 *
 * Port of IVP_Listener_Object / IVP_Event_Object (ivp_listener_object.hxx).
 * A listener is a struct of optional callbacks (NULL = ignored).  It can be
 * installed globally (IVP_Environment::add_listener_object_global, fired
 * for every object) or privately for one object
 * (IVP_Real_Object::add_listener_object, kept by the cluster manager).
 *
 * Firing order follows the reference:
 *   created  - global listeners only (fired at the end of object creation,
 *              before collision detection is enabled)
 *   deleted  - global listeners, then the object's private listeners
 *   revived / frozen - private listeners first, then global listeners
 * Listeners are called from the last added to the first; a private listener
 * may delete the object it listens to (iteration stops).
 */
#ifndef IVP_LISTENER_OBJECT_H
#define IVP_LISTENER_OBJECT_H

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_environment;
struct ivp_object;

typedef struct ivp_event_object {
    struct ivp_environment *environment; /* get current time via ivp_environment_get_current_time */
    struct ivp_object      *real_object;
} ivp_event_object_t;

typedef struct ivp_listener_object ivp_listener_object_t;

struct ivp_listener_object {
    void (*event_object_deleted)(ivp_listener_object_t *self, ivp_event_object_t *event);
    void (*event_object_created)(ivp_listener_object_t *self, ivp_event_object_t *event);
    void (*event_object_revived)(ivp_listener_object_t *self, ivp_event_object_t *event);
    void (*event_object_frozen)(ivp_listener_object_t *self, ivp_event_object_t *event);
    void (*event_environment_deleted)(ivp_listener_object_t *self, struct ivp_environment *env);
    void *user_data;
};

/* IVP_Environment::add/install/remove_listener_object_global */
void ivp_environment_add_listener_object_global(struct ivp_environment *env, ivp_listener_object_t *listener);
void ivp_environment_install_listener_object_global(struct ivp_environment *env, ivp_listener_object_t *listener);
void ivp_environment_remove_listener_object_global(struct ivp_environment *env, ivp_listener_object_t *listener);

/* IVP_Real_Object::add/remove_listener_object (private, one object) */
void ivp_object_add_listener_object(struct ivp_object *obj, ivp_listener_object_t *listener);
void ivp_object_remove_listener_object(struct ivp_object *obj, ivp_listener_object_t *listener);

#ifdef __cplusplus
}
#endif

#endif /* IVP_LISTENER_OBJECT_H */
