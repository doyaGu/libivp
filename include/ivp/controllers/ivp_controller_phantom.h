/* ivp_controller_phantom.h -- Phantom (sensor/trigger) controller
 *
 * A phantom is a non-physical volume that detects when objects enter or
 * exit its region. It does not apply forces or participate in collision
 * response — it only fires callbacks.
 *
 * Supported shapes:
 *   SPHERE - spherical trigger region
 *   AABB   - axis-aligned box trigger region
 *
 * Ported from ivp_controller_phantom.hxx.
 */
#ifndef IVP_CONTROLLER_PHANTOM_H
#define IVP_CONTROLLER_PHANTOM_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum objects a single phantom can track */
#define IVP_PHANTOM_MAX_TRACKED 64

/* ── Phantom shape ──────────────────────────────────────────────────────── */

typedef enum ivp_phantom_shape {
    IVP_PHANTOM_SPHERE = 0,
    IVP_PHANTOM_AABB   = 1
} ivp_phantom_shape_t;

/* ── Phantom callbacks ──────────────────────────────────────────────────── */

/* Called when an object enters or exits the phantom's volume. */
typedef void (*ivp_phantom_event_fn_t)(void *ctx,
                                        ivp_object_t *phantom_owner,
                                        ivp_object_t *other,
                                        bool entered);

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

    /* Event callback */
    ivp_phantom_event_fn_t event_fn;
    void *event_ctx;

    /* Tracking state */
    ivp_object_t *tracked[IVP_PHANTOM_MAX_TRACKED];
    int n_tracked;

    bool active;
} ivp_phantom_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

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

/* ── Controller callback ────────────────────────────────────────────────── */

/* Phantom controller. Pass ivp_phantom_state_t* as ctx.
 * Tests each object against the phantom volume and fires enter/exit events. */
void ivp_controller_phantom(const ivp_event_sim_t *event_sim,
                              ivp_object_t *obj,
                              void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_PHANTOM_H */
