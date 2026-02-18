/* ivp_car_system.h — Real-wheel vehicle system
 *
 * A vehicle controller that uses actual rigid body wheel objects
 * connected to a chassis via spring-damper suspension constraints.
 * Unlike the simpler raycast car (which has no wheel objects),
 * this system enables wheel-to-world collisions and complex
 * wheel-chassis interaction.
 *
 * Features:
 *   - Per-wheel spring-damper suspension with compression/release damping
 *   - Per-axle stabilizer bars (anti-roll)
 *   - Steering by rotating front wheel orientations
 *   - Engine torque on driven wheels
 *   - Brake (wheel locking)
 *   - Extra gravity and body counter-torque
 *   - Booster support
 *
 * Ported from ivp_car_system.hxx / ivp_constraint_car.hxx.
 */
#ifndef IVP_CAR_SYSTEM_H
#define IVP_CAR_SYSTEM_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ────────────────────────────────────────────────────────── */

#define IVP_CAR_SYSTEM_MAX_WHEELS  10
#define IVP_CAR_SYSTEM_MAX_AXES     5

/* ── Wheel position enum ──────────────────────────────────────────────── */

typedef enum ivp_car_wheel_pos {
    IVP_WHEEL_FRONT_LEFT  = 0,
    IVP_WHEEL_FRONT_RIGHT = 1,
    IVP_WHEEL_REAR_LEFT   = 2,
    IVP_WHEEL_REAR_RIGHT  = 3
} ivp_car_wheel_pos_t;

typedef enum ivp_car_axis_pos {
    IVP_AXIS_FRONT = 0,
    IVP_AXIS_REAR  = 1
} ivp_car_axis_pos_t;

/* ── Per-wheel configuration ──────────────────────────────────────────── */

typedef struct ivp_car_system_wheel {
    /* Objects */
    ivp_object_t *wheel_obj;           /* the physical wheel body */

    /* Suspension geometry */
    ivp_vec3r_t   hardpoint_cs;        /* chassis-space attachment point */
    ivp_vec3r_t   axis_cs;             /* wheel rotation axis in chassis space */
    int           axis_index;          /* which axle this wheel belongs to */

    /* Suspension parameters */
    ivp_real_t    spring_constant;     /* [N/m] */
    ivp_real_t    spring_damping;      /* release damping [Ns/m] */
    ivp_real_t    spring_damping_compression; /* compression damping [Ns/m] */
    ivp_real_t    spring_pre_tension;  /* pre-load offset [m] */
    ivp_real_t    max_body_force;      /* force clipping [N] */
    ivp_real_t    suspension_travel;   /* max compression distance [m] */

    /* Wheel properties */
    ivp_real_t    radius;              /* wheel radius [m] */
    ivp_real_t    friction;            /* tire friction coefficient */
    ivp_real_t    reversed_sign;       /* 1.0 or -1.0 (left/right) */

    /* Runtime state */
    ivp_real_t    compression;         /* current compression [0..1] */
    ivp_real_t    angular_velocity;    /* wheel spin [rad/s] */
    ivp_real_t    torque;              /* applied engine torque [Nm] */
    ivp_real_t    steering_angle;      /* current steering angle [rad] */
    bool          fixed;               /* wheel is locked (brake on) */
} ivp_car_system_wheel_t;

/* ── Per-axle configuration ───────────────────────────────────────────── */

typedef struct ivp_car_system_axle {
    ivp_real_t    stabilizer_constant; /* anti-roll bar stiffness [N/m] */
    int           left_wheel;          /* index of left wheel */
    int           right_wheel;         /* index of right wheel */
} ivp_car_system_axle_t;

/* ── Car system template (creation parameters) ────────────────────────── */

typedef struct ivp_car_system_template {
    ivp_object_t *car_body;
    int           n_wheels;
    int           n_axes;

    /* Per-wheel */
    ivp_object_t *wheel_objects[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_real_t    wheel_radius[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_real_t    wheel_friction[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_real_t    wheel_reversed_sign[IVP_CAR_SYSTEM_MAX_WHEELS];
    int           wheel_axis_index[IVP_CAR_SYSTEM_MAX_WHEELS];

    /* Suspension */
    ivp_real_t    spring_constant[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_real_t    spring_damping[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_real_t    spring_damping_compression[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_real_t    spring_pre_tension[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_real_t    max_body_force[IVP_CAR_SYSTEM_MAX_WHEELS];

    /* Stabilizers */
    ivp_real_t    stabilizer_constant[IVP_CAR_SYSTEM_MAX_AXES];

    /* Vehicle tuning */
    ivp_real_t    extra_gravity_force;
    ivp_real_t    extra_gravity_height_offset;
    ivp_real_t    body_counter_torque_factor;
    ivp_real_t    fast_turn_factor;
} ivp_car_system_template_t;

/* ── Car system state ─────────────────────────────────────────────────── */

typedef struct ivp_car_system {
    ivp_object_t           *body;
    ivp_car_system_wheel_t  wheels[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_car_system_axle_t   axles[IVP_CAR_SYSTEM_MAX_AXES];
    int                     n_wheels;
    int                     n_axes;

    /* Input controls */
    ivp_real_t  steering_angle;     /* master steering input [rad] */

    /* Vehicle tuning */
    ivp_real_t  extra_gravity_force;
    ivp_real_t  extra_gravity_height_offset;
    ivp_real_t  body_counter_torque_factor;
    ivp_real_t  fast_turn_factor;

    /* Booster */
    ivp_real_t  booster_force;
    ivp_real_t  booster_remaining;
    ivp_real_t  booster_cooldown;
    ivp_real_t  booster_recharge;

    bool active;
} ivp_car_system_t;

/* ── Initialization ───────────────────────────────────────────────────── */

/* Initialize car system from template. */
void ivp_car_system_init(ivp_car_system_t *car,
                           const ivp_car_system_template_t *tmpl);

/* Fill template with sensible defaults for a 4-wheel car. */
void ivp_car_system_template_default(ivp_car_system_template_t *tmpl);

/* ── Steering ─────────────────────────────────────────────────────────── */

/* Set steering for a specific wheel. */
void ivp_car_system_steer_wheel(ivp_car_system_t *car,
                                  int wheel_index,
                                  ivp_real_t angle_rad);

/* Set master steering angle (applied to front wheels via fast_turn_factor). */
void ivp_car_system_set_steering(ivp_car_system_t *car,
                                   ivp_real_t steering_angle);

/* ── Dynamic adjustment ───────────────────────────────────────────────── */

void ivp_car_system_set_spring_constant(ivp_car_system_t *car,
                                          int wheel_index, ivp_real_t k);
void ivp_car_system_set_spring_damping(ivp_car_system_t *car,
                                         int wheel_index, ivp_real_t d);
void ivp_car_system_set_stabilizer(ivp_car_system_t *car,
                                     int axis_index, ivp_real_t k);
void ivp_car_system_set_wheel_torque(ivp_car_system_t *car,
                                       int wheel_index, ivp_real_t torque);
void ivp_car_system_fix_wheel(ivp_car_system_t *car,
                                int wheel_index, bool fixed);

/* ── Booster ──────────────────────────────────────────────────────────── */

void ivp_car_system_activate_booster(ivp_car_system_t *car,
                                       ivp_real_t force,
                                       ivp_real_t duration,
                                       ivp_real_t recharge_time);

/* ── Queries ──────────────────────────────────────────────────────────── */

/* Get body speed along the forward axis. */
ivp_real_t ivp_car_system_get_body_speed(const ivp_car_system_t *car);

/* Get wheel angular velocity. */
ivp_real_t ivp_car_system_get_wheel_angular_velocity(
    const ivp_car_system_t *car, int wheel_index);

/* ── Controller callback ──────────────────────────────────────────────── */

/* Pass ivp_car_system_t* as ctx. */
void ivp_controller_car_system(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CAR_SYSTEM_H */
