/* ivp_controller_phantom.h -- Phantom (sensor/trigger) controller
 *
 * A phantom is a non-physical volume that detects when objects enter or
 * exit its region. It does not apply forces or participate in collision
 * response — it only fires callbacks.
 *
 * Reference semantics (IVP_Controller_Phantom, ivp_phantom.cxx and the
 * phantom mindists of ivp_mindist.cxx:650-700, 1752-1800), approximated
 * without touching the collision core:
 *   - an object enters when the exact convex distance between the phantom
 *     volume and the object's convex geometry (compact ledges / ball) is
 *     <= 0 (GJK on the convex parts, not bounding spheres);
 *   - while inside, the pair is only re-checked after the objects moved by
 *     more than the "hull uncertainty" (penetration + exit_policy_extra_radius,
 *     default 0.5 m); the object leaves at the first re-check that finds it
 *     separated (exit hysteresis);
 *   - sleeping objects are tested too (a moving phantom scans all objects);
 *   - pairs rejected by the environment collision filter or a shared
 *     no-collision group are never reported (reference: no mindist exists).
 *
 * Shapes:
 *   OBJECT - the owner object's own convex geometry (reference phantom)
 *   SPHERE - spherical trigger region (center/radius or attached offset)
 *   AABB   - axis-aligned box trigger region
 */
#ifndef IVP_CONTROLLER_PHANTOM_H
#define IVP_CONTROLLER_PHANTOM_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum objects a single phantom can track */
#define IVP_PHANTOM_MAX_TRACKED 64

/* ── Phantom shape ──────────────────────────────────────────────────────── */

typedef enum ivp_phantom_shape {
    IVP_PHANTOM_SPHERE = 0,
    IVP_PHANTOM_AABB   = 1,
    IVP_PHANTOM_OBJECT = 2
} ivp_phantom_shape_t;

/* ── Phantom callbacks ──────────────────────────────────────────────────── */

/* Called when an object enters or exits the phantom's volume. */
typedef void (*ivp_phantom_event_fn_t)(void *ctx,
                                        ivp_object_t *phantom_owner,
                                        ivp_object_t *other,
                                        bool entered);

/* Reference IVP_Template_Phantom subset. */
typedef struct ivp_phantom_template {
    bool       dont_check_for_unmoveables; /* default false */
    ivp_real_t exit_policy_extra_radius;   /* default 0.5 m */
    bool       respect_collision_filter;   /* default true */
} ivp_phantom_template_t;

void ivp_phantom_template_init(ivp_phantom_template_t *t);

/* Per tracked object exit bookkeeping (reference lazy hull mindist). */
typedef struct ivp_phantom_track {
    ivp_real_t  uncertainty;
    ivp_vec3r_t ref_pos_obj;
    ivp_vec3r_t ref_pos_phantom;
    ivp_quatr_t ref_q_obj;
    ivp_quatr_t ref_q_phantom;
} ivp_phantom_track_t;

/* ── Phantom state ──────────────────────────────────────────────────────── */

typedef struct ivp_phantom_state {
    ivp_phantom_shape_t shape;

    /* Sphere shape: center_ws + radius */
    ivp_vec3r_t center_ws;
    ivp_real_t  radius;

    /* AABB shape: min/max corners in world space */
    ivp_vec3r_t aabb_min_ws;
    ivp_vec3r_t aabb_max_ws;

    /* Attached to an object (moves with it) */
    ivp_object_t *owner;        /* NULL = stationary phantom */
    ivp_vec3r_t   offset_os;    /* offset from owner's core center */

    /* Reference template values */
    ivp_real_t exit_policy_extra_radius;
    bool       dont_check_for_unmoveables;
    bool       respect_collision_filter;

    /* Event callback */
    ivp_phantom_event_fn_t event_fn;
    void *event_ctx;

    /* Tracking state */
    ivp_object_t       *tracked[IVP_PHANTOM_MAX_TRACKED];
    ivp_phantom_track_t track[IVP_PHANTOM_MAX_TRACKED];
    int n_tracked;

    bool active;
} ivp_phantom_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

/* Reference IVP_Real_Object::convert_to_phantom: the owner's convex
 * geometry is the phantom volume.  tmpl may be NULL (defaults). */
void ivp_phantom_init_object(ivp_phantom_state_t *state,
                             ivp_object_t *owner,
                             const ivp_phantom_template_t *tmpl,
                             ivp_phantom_event_fn_t event_fn,
                             void *event_ctx);

/* Initialize a spherical phantom at a world position. */
void ivp_phantom_init_sphere(ivp_phantom_state_t *state,
                               const ivp_vec3r_t *center_ws,
                               ivp_real_t radius,
                               ivp_phantom_event_fn_t event_fn,
                               void *event_ctx);

/* Initialize an AABB phantom in world space. */
void ivp_phantom_init_aabb(ivp_phantom_state_t *state,
                             const ivp_vec3r_t *min_ws,
                             const ivp_vec3r_t *max_ws,
                             ivp_phantom_event_fn_t event_fn,
                             void *event_ctx);

/* Attach phantom to an object (moves with it). */
void ivp_phantom_attach(ivp_phantom_state_t *state,
                          ivp_object_t *owner,
                          const ivp_vec3r_t *offset_os);

/* Exact convex distance between the phantom volume and an object
 * (<= 0 when overlapping; overlap reports 0).  Exposed for tests. */
ivp_real_t ivp_phantom_distance_to_object(const ivp_phantom_state_t *state,
                                          const ivp_object_t *obj);

/* ── Controller callback ────────────────────────────────────────────────── */

/* Phantom controller. Pass ivp_phantom_state_t* as ctx. */
void ivp_controller_phantom(const ivp_event_sim_t *event_sim,
                              ivp_object_t *obj,
                              void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_PHANTOM_H */
