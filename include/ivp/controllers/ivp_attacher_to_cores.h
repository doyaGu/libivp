/* ivp_attacher_to_cores.h -- attach one attachment to every core of a set
 *
 * Port of the template IVP_Attacher_To_Cores<ATTACH_T>
 * (ivp_physics/ivp_attacher_to_cores.hxx).  The attacher listens to an
 * active set of cores (ivp_set_active_t holding ivp_core_t*, like the
 * reference IVP_U_Set_Active<IVP_Core>) and keeps exactly one attachment
 * per core of the set:
 *   - construction: an attachment is created for every core already in the
 *     set (set enumeration order), then the attacher listens to the set;
 *   - element added: ATTACH_T(attacher, core) -> create_attachment;
 *   - element removed: the attachment is deleted (delete_attachment);
 *   - set deleted (ivp_set_active_destroy): the attachments of all cores in
 *     the set are deleted, then the attacher deletes itself
 *     (attacher_deleted hook);
 *   - core deleted (object deletion): the attachment of that core is
 *     deleted (the reference ATTACH_T listens to its core and deletes
 *     itself); the core is NOT removed from the set, as in the reference.
 * The class callbacks play the role of ATTACH_T: create_attachment must
 * attach itself to the core (typically ivp_environment_add_controller_to_core)
 * and delete_attachment must detach it again.  The core -> attachment hash
 * is the reference core_to_attachment_hash (IVP_VHash_Store, initial 16).
 *
 * libivp specifics:
 *   - the environment is passed explicitly; object and environment
 *     deletion are observed through a global object listener;
 *   - ivp_attacher_to_cores_destroy deletes all attachments and stops
 *     listening (the reference destructor is only reachable through the
 *     deletion of the set);
 *   - on environment deletion all attachments are deleted at once (the
 *     environment is still valid while delete_attachment runs);
 *   - optional object filter (ivp_attacher_to_cores_set_object_filter): a
 *     created object accepted by the filter has its core installed into the
 *     set (and is thereby attached).
 */
#ifndef IVP_ATTACHER_TO_CORES_H
#define IVP_ATTACHER_TO_CORES_H

#include <stddef.h>
#include <ivp/base/ivp_types.h>
#include <ivp/containers/ivp_set.h>
#include <ivp/containers/ivp_vhash.h>
#include <ivp/events/ivp_listener_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_environment;
typedef struct ivp_attacher_to_cores ivp_attacher_to_cores_t;

/* The attachment type (reference template argument ATTACH_T). */
typedef struct ivp_attacher_to_cores_class {
    /* ATTACH_T::ATTACH_T(attacher, core): create the attachment of core and
     * attach it to the core.  Returns the attachment (NULL: none). */
    void *(*create_attachment)(ivp_attacher_to_cores_t *attacher, ivp_core_t *core);
    /* ATTACH_T::~ATTACH_T: detach from the core and free.  The hash entry
     * is already removed (attachment_is_going_to_be_deleted). */
    void (*delete_attachment)(ivp_attacher_to_cores_t *attacher, ivp_core_t *core, void *attachment);
    /* Optional: the attacher deleted itself because its set was deleted
     * (reference P_DELETE_THIS); the hook may free the attacher memory. */
    void (*attacher_deleted)(ivp_attacher_to_cores_t *attacher);
} ivp_attacher_to_cores_class_t;

/* libivp extra: decides whether a newly created object joins the set. */
typedef bool (*ivp_attacher_object_filter_fn_t)(void *ctx, ivp_object_t *object);

struct ivp_attacher_to_cores {
    ivp_vhash_store_t                    core_to_attachment_hash;
    ivp_set_active_t                    *set_of_cores;      /* NULL after the set was deleted */
    struct ivp_environment              *environment;       /* NULL after it was deleted */
    const ivp_attacher_to_cores_class_t *cls;
    void                                *user_data;

    /* internal */
    ivp_set_active_listener_t            set_listener;
    ivp_listener_object_t                object_listener;
    ivp_attacher_object_filter_fn_t      object_filter;
    void                                *object_filter_ctx;
    bool                                 initialized;
    bool                                 environment_deleted; /* the environment is gone */
};

/* Reference constructor IVP_Attacher_To_Cores(set_of_cores): attaches to
 * every core already in the set and listens to the set.  The attacher
 * memory must stay valid until ivp_attacher_to_cores_destroy or the
 * attacher_deleted hook.  Returns false on bad arguments / no memory. */
bool ivp_attacher_to_cores_init(ivp_attacher_to_cores_t *attacher,
                                struct ivp_environment *env,
                                ivp_set_active_t *set_of_cores,
                                const ivp_attacher_to_cores_class_t *cls,
                                void *user_data);

/* Delete every attachment and stop listening to the set and the
 * environment (does not call attacher_deleted). */
void ivp_attacher_to_cores_destroy(ivp_attacher_to_cores_t *attacher);

/* core_to_attachment_hash.find_elem(core) */
void *ivp_attacher_to_cores_get_attachment(ivp_attacher_to_cores_t *attacher, const ivp_core_t *core);

static inline int ivp_attacher_to_cores_count(const ivp_attacher_to_cores_t *attacher) {
    return ivp_vhash_store_count(&attacher->core_to_attachment_hash);
}

/* Reference attachment_is_going_to_be_deleted(attachment, core): drops the
 * hash entry of an attachment that deletes itself on its own. */
void ivp_attacher_to_cores_attachment_is_going_to_be_deleted(ivp_attacher_to_cores_t *attacher,
                                                             void *attachment,
                                                             ivp_core_t *attached_core);

/* libivp extra: install the core of every object created from now on for
 * which filter(ctx, object) returns true into the set (NULL disables). */
void ivp_attacher_to_cores_set_object_filter(ivp_attacher_to_cores_t *attacher,
                                             ivp_attacher_object_filter_fn_t filter,
                                             void *ctx);

/* The first object of a core (core->objects.element_at(0)). */
static inline ivp_object_t *ivp_attacher_core_object(ivp_core_t *core) {
    return (core && ivp_pvec_len(&core->objects) > 0) ? (ivp_object_t *)ivp_pvec_at(&core->objects, 0) : NULL;
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_ATTACHER_TO_CORES_H */
