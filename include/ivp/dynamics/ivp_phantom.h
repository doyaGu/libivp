/* ivp_phantom.h -- phantom objects (IVP_Controller_Phantom, ivp_phantom.hxx)
 *
 * A phantom is an object that does not collide but keeps track of all
 * objects intruding it.  ivp_object_convert_to_phantom() attaches an
 * ivp_controller_phantom_t to the object (IVP_Real_Object::convert_to_phantom):
 * every mindist of the object becomes a phantom mindist (no impacts, no
 * friction) and the mindist manager reports mindists entering / leaving the
 * phantom volume.  A mindist leaves only when the distance exceeds
 * exit_policy_extra_radius.
 *
 * The controller is owned by the object: it is destroyed when the object is
 * deleted or converted again.  Unlike the reference destructor it is not
 * possible to switch a phantom back to a real object (neither can the
 * reference: the movement state and mindist functions are not restored).
 *
 * The bounding volume trigger controller of ivp_controller_phantom.h
 * (sphere / AABB / object volume, polled once per PSI) is a separate libivp
 * utility and does not use this machinery.
 */
#ifndef IVP_PHANTOM_H
#define IVP_PHANTOM_H

#include <ivp/base/ivp_types.h>
#include <ivp/containers/ivp_pvec.h>
#include <ivp/containers/ivp_set.h>
#include <ivp/containers/ivp_vhash.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_object;
struct ivp_core;
struct ivp_mindist;
struct ivp_environment;
typedef struct ivp_controller_phantom ivp_controller_phantom_t;

/* IVP_Template_Phantom */
typedef struct ivp_template_phantom {
    bool  manage_intruding_objects;   /* default false: maintain get_intruding_objects() */
    bool  manage_intruding_cores;     /* default false: maintain get_intruding_cores()   */
    bool  dont_check_for_unmoveables; /* default false: a static phantom becomes IVP_MT_STATIC_PHANTOM
                                       * (it then sees unmoveable objects as well) */
    float exit_policy_extra_radius;   /* default 0.5 m */
    float exit_policy_extra_time;     /* default 0.5 s (not implemented, as in the reference) */
    bool  manage_sleeping_cores;      /* default false (needs manage_intruding_cores) */
} ivp_template_phantom_t;

void ivp_template_phantom_init(ivp_template_phantom_t *t);

/* IVP_Listener_Phantom (callbacks may be NULL) */
typedef struct ivp_listener_phantom ivp_listener_phantom_t;
struct ivp_listener_phantom {
    void (*mindist_entered_volume)(ivp_listener_phantom_t *self, ivp_controller_phantom_t *c, struct ivp_mindist *md);
    void (*mindist_left_volume)(ivp_listener_phantom_t *self, ivp_controller_phantom_t *c, struct ivp_mindist *md);
    void (*core_entered_volume)(ivp_listener_phantom_t *self, ivp_controller_phantom_t *c, struct ivp_core *core);
    void (*core_left_volume)(ivp_listener_phantom_t *self, ivp_controller_phantom_t *c, struct ivp_core *core);
    /* object deleted or phantom replaced */
    void (*phantom_is_going_to_be_deleted_event)(ivp_listener_phantom_t *self, ivp_controller_phantom_t *c);
    void *user_data;
};

/* IVP_Controller_Phantom (treat as opaque; use the accessors) */
struct ivp_controller_phantom {
    struct ivp_object *object;
    float              exit_policy_extra_radius;
    ivp_pvec_t         listeners;              /* ivp_listener_phantom_t* */
    ivp_set_active_t   set_of_mindists;
    ivp_set_active_t  *set_of_objects;         /* NULL unless manage_intruding_objects */
    ivp_vhash_store_t *mindist_object_counter;
    ivp_vhash_store_t *mindist_core_counter;
    ivp_set_active_t  *set_of_cores;           /* NULL unless manage_intruding_cores */
    ivp_set_t         *set_of_sleeping_cores;  /* NULL unless manage_sleeping_cores */
    void              *object_listener;        /* internal IVP_Listener_Object */
    void              *client_data;            /* for use by game code */
};

/* IVP_Real_Object::convert_to_phantom (tmpl may be NULL = defaults) */
ivp_controller_phantom_t *ivp_object_convert_to_phantom(struct ivp_object *obj, const ivp_template_phantom_t *tmpl);
/* IVP_Real_Object::get_controller_phantom */
ivp_controller_phantom_t *ivp_object_get_controller_phantom(const struct ivp_object *obj);

static inline struct ivp_object *ivp_controller_phantom_get_object(const ivp_controller_phantom_t *c) { return c->object; }
static inline ivp_set_active_t *ivp_controller_phantom_get_intruding_objects(const ivp_controller_phantom_t *c) { return c->set_of_objects; }
static inline ivp_set_active_t *ivp_controller_phantom_get_intruding_cores(const ivp_controller_phantom_t *c) { return c->set_of_cores; }
static inline ivp_set_active_t *ivp_controller_phantom_get_intruding_mindists(ivp_controller_phantom_t *c) { return &c->set_of_mindists; }

void ivp_controller_phantom_add_listener(ivp_controller_phantom_t *c, ivp_listener_phantom_t *l);
void ivp_controller_phantom_remove_listener(ivp_controller_phantom_t *c, ivp_listener_phantom_t *l);
void ivp_controller_phantom_add_sleeping_object(ivp_controller_phantom_t *c, struct ivp_object *obj);
void ivp_controller_phantom_remove_sleeping_object(ivp_controller_phantom_t *c, struct ivp_object *obj);
void ivp_controller_phantom_wake_all_sleeping_objects(ivp_controller_phantom_t *c);

/* the object of a mindist that is not the phantom's own object */
struct ivp_object *ivp_controller_phantom_get_other_object(const ivp_controller_phantom_t *c, const struct ivp_mindist *md);

#ifdef __cplusplus
}
#endif

#endif /* IVP_PHANTOM_H */
