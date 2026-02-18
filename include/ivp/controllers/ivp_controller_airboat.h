/* ivp_controller_airboat.h — Raycast airboat/hovercraft controller
 *
 * Simulates a watercraft with:
 *   - 4 pontoon/wheel contact points with raycasts
 *   - Spring-damper suspension for ground contacts
 *   - Buoyancy force for water contacts
 *   - Quadratic water drag
 *   - Turbine thrust along forward axis
 *   - Steering via lateral impulses at pontoon points
 *
 * Requires a game-side raycast callback to determine surface contact.
 *
 * Ported from ivp_controller_airboat.h.
 */
#ifndef IVP_CONTROLLER_AIRBOAT_H
#define IVP_CONTROLLER_AIRBOAT_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Pontoon configuration ─────────────────────────────────────────────── */

#define IVP_AIRBOAT_MAX_PONTOONS 4

typedef struct ivp_airboat_pontoon {
    /* Configuration */
    ivp_vec3r_t hardpoint_cs;      /* attachment in chassis core space */
    ivp_vec3r_t direction_cs;      /* ray direction in core space (down) */
    ivp_real_t  suspension_length; /* max suspension travel [m] */
    ivp_real_t  spring_constant;   /* suspension stiffness [N/m] */
    ivp_real_t  damping;           /* suspension damping [Ns/m] */

    /* Runtime state */
    ivp_real_t  compression;       /* current compression [0..1] */
    bool        ground_contact;    /* touching solid ground */
    bool        water_contact;     /* in water */
    ivp_vec3r_t contact_point_ws;
    ivp_vec3r_t contact_normal_ws;
    ivp_real_t  contact_distance;  /* distance from hardpoint to surface */
} ivp_airboat_pontoon_t;

/* ── Raycast callback ──────────────────────────────────────────────────── */

typedef enum ivp_surface_type {
    IVP_SURFACE_NONE  = 0,
    IVP_SURFACE_GROUND = 1,
    IVP_SURFACE_WATER  = 2
} ivp_surface_type_t;

typedef struct ivp_raycast_result {
    bool              hit;
    ivp_surface_type_t surface_type;
    ivp_real_t         distance;
    ivp_vec3r_t        hit_point;
    ivp_vec3r_t        hit_normal;
} ivp_raycast_result_t;

/* User-provided raycast callback.
 * Casts a ray from start_ws in direction dir_ws (normalized).
 * max_dist is the maximum ray length. */
typedef void (*ivp_airboat_raycast_fn_t)(void *ctx,
                                         const ivp_vec3r_t *start_ws,
                                         const ivp_vec3r_t *dir_ws,
                                         ivp_real_t max_dist,
                                         ivp_raycast_result_t *result);

/* ── Airboat state ─────────────────────────────────────────────────────── */

typedef struct ivp_airboat_state {
    ivp_object_t         *chassis;
    ivp_airboat_pontoon_t pontoons[IVP_AIRBOAT_MAX_PONTOONS];
    int                   n_pontoons;

    /* Input controls */
    ivp_real_t  thrust;            /* turbine thrust force [N] */
    ivp_real_t  steering_angle;    /* radians, positive = left */

    /* Physics parameters */
    ivp_real_t  water_gravity;     /* reduced gravity in water (default: 7.0) */
    ivp_real_t  drag_coefficient;  /* quadratic drag (default: 0.003) */
    ivp_real_t  drag_area;         /* frontal area for drag [m^2] (default: 3.5) */
    ivp_real_t  buoyancy_factor;   /* buoyancy force multiplier (default: 0.25) */
    ivp_real_t  steering_gain;     /* steering torque gain (default: 1.0) */

    /* Booster */
    ivp_real_t  booster_force;     /* additional thrust when boosting [N] */
    ivp_real_t  booster_remaining; /* seconds of boost left */
    ivp_real_t  booster_cooldown;  /* seconds until boost available */

    /* Raycast callback */
    ivp_airboat_raycast_fn_t raycast_fn;
    void                    *raycast_ctx;

    bool active;
} ivp_airboat_state_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Initialize a basic 4-pontoon airboat. */
void ivp_airboat_init(ivp_airboat_state_t *state,
                      ivp_object_t *chassis,
                      ivp_real_t length,
                      ivp_real_t width,
                      ivp_real_t suspension_length);

/* Set the raycast callback. Must be called before simulation. */
static inline void ivp_airboat_set_raycast(ivp_airboat_state_t *state,
                                           ivp_airboat_raycast_fn_t fn,
                                           void *ctx) {
    state->raycast_fn = fn;
    state->raycast_ctx = ctx;
}

/* ── Input ─────────────────────────────────────────────────────────────── */

static inline void ivp_airboat_set_thrust(ivp_airboat_state_t *state,
                                          ivp_real_t force) {
    state->thrust = force;
}

static inline void ivp_airboat_set_steering(ivp_airboat_state_t *state,
                                            ivp_real_t angle_rad) {
    state->steering_angle = angle_rad;
}

static inline void ivp_airboat_activate_booster(ivp_airboat_state_t *state,
                                                ivp_real_t force,
                                                ivp_real_t duration) {
    state->booster_force = force;
    state->booster_remaining = duration;
}

/* ── Controller callback ───────────────────────────────────────────────── */

/* Pass ivp_airboat_state_t* as ctx. */
void ivp_controller_airboat(const ivp_event_sim_t *event_sim,
                            ivp_object_t *obj,
                            void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_AIRBOAT_H */
