/* ivp_object_attach.h -- Object attachment / detachment
 *
 * Allows multiple objects to share a single core so they move as one
 * rigid body. The "leader" object owns the core; "follower" objects
 * reference it with a fixed offset.
 *
 * On detachment, the follower gets its own independent core initialized
 * from the leader's current state.
 *
 * Ported from ivp_object_attach.hxx.
 */
#ifndef IVP_OBJECT_ATTACH_H
#define IVP_OBJECT_ATTACH_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/base/ivp_forward_decls.h>
#include <ivp/events/ivp_event.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Attachment state ──────────────────────────────────────────────────── */

#define IVP_ATTACH_MAX_FOLLOWERS 16

typedef struct ivp_attachment {
    ivp_object_t *leader;
    struct {
        ivp_object_t *obj;
        ivp_vec3r_t   offset_pos;   /* position offset in leader's core space */
        ivp_quatr_t   offset_rot;   /* rotation offset relative to leader */
    } followers[IVP_ATTACH_MAX_FOLLOWERS];
    int n_followers;
    bool active;
} ivp_attachment_t;

/* ── API ───────────────────────────────────────────────────────────────── */

/* Initialize an attachment group with a leader object. */
void ivp_attachment_init(ivp_attachment_t *att, ivp_object_t *leader);

/* Attach a follower object to the leader.
 * The follower's current position/orientation relative to the leader
 * is computed and stored as a fixed offset.
 * Returns IVP_OK on success. */
ivp_result_t ivp_attachment_add(ivp_attachment_t *att, ivp_object_t *follower);

/* Detach a follower, restoring it as an independent object
 * with velocity inherited from the leader.
 * Returns IVP_OK on success. */
ivp_result_t ivp_attachment_remove(ivp_attachment_t *att, ivp_object_t *follower);

/* Synchronize all follower positions from the leader's current state.
 * Call this once per PSI (typically as a high-priority controller). */
void ivp_attachment_sync(ivp_attachment_t *att);

/* Controller callback that calls ivp_attachment_sync.
 * Pass ivp_attachment_t* as ctx. Only fires once per PSI
 * (when obj == leader). */
void ivp_controller_attachment(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_OBJECT_ATTACH_H */
