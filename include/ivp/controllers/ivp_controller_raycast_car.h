/* ivp_controller_raycast_car.h -- Raycast vehicle controller
 *
 * Implements a vehicle with raycast wheels: each wheel casts a ray
 * downward from the chassis. Suspension is modeled as a spring-damper.
 * Tire friction uses a simplified Pacejka-like model.
 *
 * Ported from ivp_controller_raycast_car.hxx.
 */
#ifndef IVP_CONTROLLER_RAYCAST_CAR_H
#define IVP_CONTROLLER_RAYCAST_CAR_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Wheel configuration ───────────────────────────────────────────────── */

#define IVP_CAR_MAX_WHEELS 8

typedef struct ivp_car_wheel {
    /* Configuration */
    ivp_vec3r_t hardpoint_cs;      /* attachment point in chassis core space */
    ivp_vec3r_t direction_cs;      /* ray direction in core space (usually 0,1,0 for Y-down) */
    ivp_real_t  suspension_length; /* max suspension travel [m] */
    ivp_real_t  radius;            /* wheel radius [m] */
    ivp_real_t  spring_constant;   /* suspension stiffness [N/m] */
    ivp_real_t  damping;           /* suspension damping [Ns/m] */

    /* Tire friction */
    ivp_real_t  friction_forward;  /* longitudinal grip (default: 1.0) */
    ivp_real_t  friction_side;     /* lateral grip (default: 1.0) */

    /* Runtime state (updated by controller) */
    ivp_real_t  compression;       /* current spring compression [0..1] */
    ivp_real_t  angular_velocity;  /* wheel spin speed [rad/s] */
    bool        grounded;          /* is this wheel touching ground? */
    ivp_vec3r_t contact_point_ws;  /* world-space ground contact point */
    ivp_vec3r_t contact_normal_ws; /* world-space ground normal */
} ivp_car_wheel_t;

/* ── Car state ─────────────────────────────────────────────────────────── */

typedef struct ivp_car_state {
    ivp_object_t    *chassis;
    ivp_car_wheel_t  wheels[IVP_CAR_MAX_WHEELS];
    int              n_wheels;

    /* Input controls */
    ivp_real_t  steering_angle;    /* radians, positive = left */
    ivp_real_t  engine_force;      /* forward drive force [N] */
    ivp_real_t  brake_force;       /* brake force [N] */

    /* Which wheels steer / drive */
    bool        steer_front;       /* apply steering to wheels [0, 1] */
    bool        drive_rear;        /* apply engine to rear wheels [2, 3] */

    /* Ground plane (for simple flat ground) */
    ivp_vec3r_t ground_normal;
    ivp_real_t  ground_height;

    bool active;
} ivp_car_state_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Initialize a basic 4-wheel car layout.
 * Wheelbase and track are the distances between wheel centers. */
void ivp_car_init_4wheel(ivp_car_state_t *state,
                           ivp_object_t *chassis,
                           ivp_real_t wheelbase,
                           ivp_real_t track_width,
                           ivp_real_t wheel_radius,
                           ivp_real_t suspension_length);

/* Add a custom wheel. Returns wheel index or -1 on failure. */
int ivp_car_add_wheel(ivp_car_state_t *state,
                        const ivp_car_wheel_t *wheel);

/* ── Input ─────────────────────────────────────────────────────────────── */

static inline void ivp_car_set_steering(ivp_car_state_t *state,
                                          ivp_real_t angle_rad) {
    state->steering_angle = angle_rad;
}

static inline void ivp_car_set_engine(ivp_car_state_t *state,
                                        ivp_real_t force) {
    state->engine_force = force;
}

static inline void ivp_car_set_brake(ivp_car_state_t *state,
                                       ivp_real_t force) {
    state->brake_force = force;
}

/* ── Controller callback ───────────────────────────────────────────────── */

/* Pass ivp_car_state_t* as ctx. */
void ivp_controller_raycast_car(const ivp_event_sim_t *event_sim,
                                  ivp_object_t *obj,
                                  void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_RAYCAST_CAR_H */
