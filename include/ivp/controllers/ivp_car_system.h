/* ivp_car_system.h -- Real-wheel vehicle system
 *
 * Port of IVP_Car_System_Real_Wheels / IVP_Template_Car_System
 * (ivp_controller/ivp_car_system.{hxx,cxx}).
 *
 * The car is built from existing objects: one body and n_wheels wheel
 * objects (usually balls).  ivp_car_system_init creates, exactly like the
 * reference constructor:
 *   - an IVP_Constraint_Solver_Car (ivp_constraint_car.h) that keeps every
 *     wheel on its hard point in the body's right/forward plane and aligns
 *     the wheel axes with the body (steering rotates the wheel targets);
 *   - per wheel an IVP_Actuator_Suspension between the body (anchor 500 m
 *     "above" the hard point along index_y) and the wheel center, and an
 *     IVP_Actuator_Torque (drive) about the wheel axis;
 *   - an IVP_Actuator_Torque on the body (counter torque of the wheels);
 *   - per axis with >= 2 wheels an IVP_Actuator_Stabilizer;
 *   - four IVP_Actuator_Force: down force and extra gravity (anchored on the
 *     environment's static object, 1e8 m along the gravity direction) and
 *     two powerslide forces.
 * All actuators are the libivp ports (ivp_actuator_*.h) registered with
 * ivp_environment_add_controller; their states live in ivp_car_system_t,
 * so the struct must not move after ivp_car_system_init.
 *
 * Reference behaviour kept on purpose:
 *   - stabilizers are created but never run (IVP_Actuator_Four_Point never
 *     announces itself to the controller manager); changing their constant
 *     only wakes the body;
 *   - max_body_force defaults to 0 (the body then receives no spring force);
 *   - the coordinate system is index_x = right, index_y = suspension axis
 *     pointing DOWN (with the default y-down gravity), index_z = forward;
 *   - ivp_car_system_get_body_speed is the speed along +index_z [m/s].
 *
 * Differences to the earlier libivp car system (a re-derivation):
 *   - ivp_car_system_init registers everything itself; adding
 *     ivp_controller_car_system to the environment is no longer needed
 *     (it is kept as a no-op for source compatibility);
 *   - ivp_car_system_destroy releases everything; it may be called before
 *     or after the environment is destroyed.  Deleting the body or a wheel
 *     first detaches the car (every other call becomes a no-op; the
 *     reference would keep dangling pointers);
 *   - fix_wheel (handbrake) uses the libivp constraint port
 *     (ivp_constraint.h) with the reference template;
 *   - wheel hard points come from wheel_pos_Bos (body object system);
 *     set wheel_pos_from_objects to use the wheels' current positions
 *     (ivp_car_system_template_default does that);
 *   - the per-wheel / per-axle runtime structs (wheels[], axles[]) and the
 *     velocity-only booster are gone; see the getters below.
 */
#ifndef IVP_CAR_SYSTEM_H
#define IVP_CAR_SYSTEM_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/controllers/ivp_actuator_suspension.h>
#include <ivp/controllers/ivp_actuator_stabilizer.h>
#include <ivp/controllers/ivp_actuator_torque.h>
#include <ivp/controllers/ivp_actuator_force.h>
#include <ivp/controllers/ivp_constraint_car.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_constraint_state;

/* ── Constants ────────────────────────────────────────────────────────── */

#define IVP_CAR_SYSTEM_MAX_WHEELS  10   /* IVP_CAR_SYSTEM_MAX_WHEELS */
#define IVP_CAR_SYSTEM_MAX_AXES     5   /* IVP_CAR_SYSTEM_MAX_AXIS   */
#define IVP_CAR_SYSTEM_MAX_AXIS     IVP_CAR_SYSTEM_MAX_AXES

/* ── Wheel / axis positions (IVP_POS_WHEEL, IVP_POS_AXIS) ─────────────── */

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

/* ── Template (IVP_Template_Car_System) ───────────────────────────────── */

typedef struct ivp_car_system_template {
    ivp_object_t *car_body;
    int           n_wheels;
    int           n_axes;                 /* reference n_axis */

    /* Coordinate system (reference index_x/y/z, is_left_handed) */
    int           index_x;                /* right (default 0) */
    int           index_y;                /* suspension axis, "down" (default 1) */
    int           index_z;                /* forward (default 2) */
    bool          is_left_handed;

    /* Per wheel (index according to ivp_car_wheel_pos_t) */
    ivp_object_t *wheel_objects[IVP_CAR_SYSTEM_MAX_WHEELS];   /* reference car_wheel */
    ivp_real_t    wheel_radius[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_real_t    wheel_friction[IVP_CAR_SYSTEM_MAX_WHEELS];  /* friction_of_wheel (raycast car only) */
    ivp_real_t    wheel_reversed_sign[IVP_CAR_SYSTEM_MAX_WHEELS]; /* 1, or -1 for wheels turned by 180 deg */
    int           wheel_axis_index[IVP_CAR_SYSTEM_MAX_WHEELS];    /* libivp only, unused (axis = (i & 2) ? rear : front) */

    ivp_vec3f_t   wheel_pos_Bos[IVP_CAR_SYSTEM_MAX_WHEELS];   /* wheel centers in the body object system */
    ivp_vec3f_t   trace_pos_Bos[IVP_CAR_SYSTEM_MAX_WHEELS];   /* raycast car only */
    bool          wheel_pos_from_objects; /* libivp: take wheel_pos_Bos from the current wheel positions */

    /* Suspension */
    ivp_real_t    spring_constant[IVP_CAR_SYSTEM_MAX_WHEELS];            /* [N/m] */
    ivp_real_t    spring_damping[IVP_CAR_SYSTEM_MAX_WHEELS];             /* releasing spring */
    ivp_real_t    spring_damping_compression[IVP_CAR_SYSTEM_MAX_WHEELS]; /* compressing spring */
    ivp_real_t    spring_pre_tension[IVP_CAR_SYSTEM_MAX_WHEELS];         /* [m] */
    ivp_real_t    max_body_force[IVP_CAR_SYSTEM_MAX_WHEELS];             /* clip of the force on the body */
    ivp_real_t    raycast_startpoint_height_offset;                      /* raycast car only */

    /* Stabilizers (created, but inert as in the reference) */
    ivp_real_t    stabilizer_constant[IVP_CAR_SYSTEM_MAX_AXES];

    /* Wheel drive */
    ivp_real_t    wheel_max_rotation_speed[IVP_CAR_SYSTEM_MAX_AXES];     /* torque only below this |omega| */

    /* Vehicle tuning */
    ivp_real_t    extra_gravity_force;               /* reference extra_gravity_force_value [N] */
    ivp_real_t    extra_gravity_height_offset;
    ivp_real_t    body_down_force_vertical_offset;
    ivp_real_t    body_counter_torque_factor;        /* e.g. 0.3 */
    ivp_real_t    fast_turn_factor;                  /* default 1 */
} ivp_car_system_template_t;

/* Reference constructor IVP_Template_Car_System(n_wheels, n_axis): all zero,
 * indices x/y/z, right handed, wheel_reversed_sign 1, fast_turn_factor 1. */
void ivp_car_system_template_init(ivp_car_system_template_t *tmpl, int n_wheels, int n_axes);

/* libivp convenience: template_init(4, 2) plus usable default tuning
 * (radius 0.3, springs 50000 N/m, damping 3000, max_body_force 1e6,
 * wheel_max_rotation_speed 100 rad/s) and wheel_pos_from_objects = true. */
void ivp_car_system_template_default(ivp_car_system_template_t *tmpl);

/* ── Car state (IVP_Car_System_Real_Wheels) ───────────────────────────── */

typedef struct ivp_car_system_wheel_skid_info {   /* IVP_Wheel_Skid_Info */
    ivp_vec3f_t last_contact_position_ws;
    float       last_skid_value;       /* 0 = no skidding */
    ivp_time_t  last_skid_time;
} ivp_car_system_wheel_skid_info_t;

typedef struct ivp_car_system {
    ivp_environment_t           *environment;
    ivp_allocator_t              allocator;         /* copied: this state may outlive the environment */
    bool                         allocator_bound;
    int                          n_wheels;
    int                          n_axes;
    bool                         active;           /* init succeeded and not destroyed */

    ivp_object_t                *body;              /* reference car_body */
    ivp_object_t                *car_wheel[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_constraint_solver_car_t *car_constraint_solver;

    ivp_torque_actuator_state_t  car_act_torque_body;
    ivp_torque_actuator_state_t  car_act_torque[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_suspension_state_t       car_spring[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_stabilizer_state_t       car_stabilizer[IVP_CAR_SYSTEM_MAX_AXES];
    bool                         car_stabilizer_exists[IVP_CAR_SYSTEM_MAX_AXES];
    ivp_force_actuator_state_t   car_act_down_force;
    ivp_force_actuator_state_t   car_act_extra_gravity;
    ivp_force_actuator_state_t   car_act_powerslide_back;
    ivp_force_actuator_state_t   car_act_powerslide_front;
    struct ivp_constraint_state *fix_wheel_constraint[IVP_CAR_SYSTEM_MAX_WHEELS];

    float                        wheel_reversed_sign[IVP_CAR_SYSTEM_MAX_WHEELS];
    float                        wheel_radius[IVP_CAR_SYSTEM_MAX_WHEELS];
    float                        body_counter_torque_factor;
    float                        max_speed;
    float                        fast_turn_factor;

    /* booster: two force actuators, created on demand */
    ivp_force_actuator_state_t   booster_actuator[2];
    bool                         booster_active;
    float                        booster_seconds_to_go;
    float                        booster_seconds_until_ready;

    float                        steering_angle;   /* current steering angle [rad] */

    /* controller registrations (libivp) */
    ivp_controller_handle_t      h_torque_body;
    ivp_controller_handle_t      h_torque[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_controller_handle_t      h_spring[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_controller_handle_t      h_down_force;
    ivp_controller_handle_t      h_extra_gravity;
    ivp_controller_handle_t      h_powerslide_back;
    ivp_controller_handle_t      h_powerslide_front;
    ivp_controller_handle_t      h_booster[2];
    ivp_controller_handle_t      h_fix_wheel[IVP_CAR_SYSTEM_MAX_WHEELS];
    ivp_event_listener_handle_t  env_listener;
    bool                         environment_dying;
} ivp_car_system_t;

/* ── Life cycle ───────────────────────────────────────────────────────── */

/* IVP_Car_System_Real_Wheels(env, templ); the environment is the body's.
 * Returns false (car inactive, n_wheels 0) when the constraint system
 * cannot be built (invalid template, singular push matrix, OOM). */
bool ivp_car_system_init(ivp_car_system_t *car, const ivp_car_system_template_t *tmpl);

/* ~IVP_Car_System_Real_Wheels plus the actuators the reference leaves to
 * the objects: removes every controller and constraint, clears
 * core->car_wheel.  Call before the body or a wheel is deleted.  Safe to
 * call twice and after the environment was destroyed. */
void ivp_car_system_destroy(ivp_car_system_t *car);

/* ── Steering ─────────────────────────────────────────────────────────── */

/* do_steering_wheel: rotate the wheel's target about index_y. */
void ivp_car_system_steer_wheel(ivp_car_system_t *car, int wheel_index, ivp_real_t angle_rad);

/* do_steering: front axle with Ackermann angle for the inner/outer wheel,
 * fast_turn_factor spins the body proportional to the steering change. */
void ivp_car_system_set_steering(ivp_car_system_t *car, ivp_real_t steering_angle);

/* IVP_Car_System::calc_ackerman_angle (alpha refers to the innermost wheel). */
ivp_real_t ivp_car_system_calc_ackerman_angle(ivp_real_t alpha, ivp_real_t dx, ivp_real_t dz);

/* ── Car adjustment ───────────────────────────────────────────────────── */

void ivp_car_system_set_spring_constant(ivp_car_system_t *car, int wheel_index, ivp_real_t k);
void ivp_car_system_set_spring_damping(ivp_car_system_t *car, int wheel_index, ivp_real_t d);
void ivp_car_system_set_spring_damping_compression(ivp_car_system_t *car, int wheel_index, ivp_real_t d);
void ivp_car_system_set_max_body_force(ivp_car_system_t *car, int wheel_index, ivp_real_t f);
void ivp_car_system_set_spring_pre_tension(ivp_car_system_t *car, int wheel_index, ivp_real_t pre_tension);
void ivp_car_system_set_spring_length(ivp_car_system_t *car, int wheel_index, ivp_real_t len);
void ivp_car_system_set_stabilizer(ivp_car_system_t *car, int axis_index, ivp_real_t k);
void ivp_car_system_set_fast_turn_factor(ivp_car_system_t *car, ivp_real_t factor);
void ivp_car_system_set_wheel_torque(ivp_car_system_t *car, int wheel_index, ivp_real_t torque);
void ivp_car_system_set_wheel_speed_dampening(ivp_car_system_t *car, int wheel_index, ivp_real_t dampening);
void ivp_car_system_update_body_countertorque(ivp_car_system_t *car);
void ivp_car_system_set_body_downforce(ivp_car_system_t *car, ivp_real_t force);
void ivp_car_system_set_powerslide(ivp_car_system_t *car, ivp_real_t front_accel, ivp_real_t rear_accel);

/* fix_wheel: block (handbrake) or release the wheel rotation. */
void ivp_car_system_fix_wheel(ivp_car_system_t *car, int wheel_index, bool fixed);

/* ── Booster ──────────────────────────────────────────────────────────── */

/* activate_booster(thrust, duration, recharge_time): thrust is a factor of
 * |gravity|; ignored while a booster is running or recharging. */
void ivp_car_system_activate_booster(ivp_car_system_t *car, ivp_real_t thrust, ivp_real_t duration,
                                     ivp_real_t recharge_time);
void ivp_car_system_set_booster_acceleration(ivp_car_system_t *car, ivp_real_t acceleration);
void ivp_car_system_update_booster(ivp_car_system_t *car, ivp_real_t delta_time);
ivp_real_t ivp_car_system_get_booster_delay(const ivp_car_system_t *car);
ivp_real_t ivp_car_system_get_booster_time_to_go(const ivp_car_system_t *car);

/* ── Queries ──────────────────────────────────────────────────────────── */

/* get_body_speed(index_z): body speed along its forward axis [m/s]. */
ivp_real_t ivp_car_system_get_body_speed(const ivp_car_system_t *car);
ivp_real_t ivp_car_system_get_body_speed_axis(const ivp_car_system_t *car, int axis_index);

/* get_wheel_angular_velocity: rotation speed about the wheel axis measured
 * by the drive torque actuator (0 while its torque is 0). */
ivp_real_t ivp_car_system_get_wheel_angular_velocity(const ivp_car_system_t *car, int wheel_index);

ivp_real_t ivp_car_system_get_orig_front_wheel_distance(const ivp_car_system_t *car);
ivp_real_t ivp_car_system_get_orig_axles_distance(const ivp_car_system_t *car);

/* get_skid_info: out must hold n_wheels entries. */
void ivp_car_system_get_skid_info(const ivp_car_system_t *car, ivp_car_system_wheel_skid_info_t *out);

/* ── Controller callback (compatibility) ──────────────────────────────── */

/* No-op.  The car registers its own controllers in ivp_car_system_init;
 * kept so that code adding it to the environment still compiles. */
void ivp_controller_car_system(const ivp_event_sim_t *event_sim, ivp_object_t *obj, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CAR_SYSTEM_H */
