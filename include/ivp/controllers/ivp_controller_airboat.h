/* ivp_controller_airboat.h — Raycast airboat controller
 *
 * Port of IVP_Controller_Raycast_Airboat (ivp_controller/
 * ivp_controller_airboat.{h,cpp}, Valve's 4-pontoon airboat built on the
 * Ipion raycast car system) with the reference float / double evaluation:
 *
 *   - construction (ivp_airboat_init) mirrors the reference constructor:
 *     the body's speed and rotation speed are cleared, the environment's
 *     standard gravity controller is removed from the body's core and a
 *     private IVP_Standard_Gravity_Controller with AIRBOAT_GRAVITY (7 m/s^2)
 *     is attached instead, the controller is announced (priority
 *     IVP_CP_CONSTRAINTS_MAX, dependent on the body's core), the pontoons
 *     are placed from the car system template (wheel_pos_Bos / trace_pos_Bos
 *     through calc_m_core_f_object) and do_steering(0) wakes the body;
 *   - every PSI: one ray per pontoon (start = trace position, direction =
 *     the core's "down" axis, length AIRBOAT_RAYCAST_DIST) is handed to the
 *     game-side ray caster (reference do_raycasts_gameside, here
 *     ivp_airboat_set_raycast); a ground impact pushes the hull with the
 *     pontoon spring (k * compression, scaled by 1.1 / (|n.d| + 0.1) clamped
 *     to [0, 3], relax / compress damping, >= 0), a pontoon in water gets the
 *     buoyancy 1.1 * 0.25 * m * 7 * volume_scalar along the inverted ray;
 *     then water drag 0.5 * 0.003 * 1000 * 3.53 * v^2 (always), the turbine
 *     thrust along the forward axis and the steering / rotational drag
 *     impulses at the pontoons.
 *
 * update_throttle does not wake the body (as in the reference); do_steering
 * and change_wheel_torque do.  Booster values are stored but never applied
 * (reference).
 */
#ifndef IVP_CONTROLLER_AIRBOAT_H
#define IVP_CONTROLLER_AIRBOAT_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/controllers/ivp_car_system.h>
#include <ivp/collision/ivp_ray_solver.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IVP_AIRBOAT_MAX_WHEELS 4

/* Reference constants (ivp_controller_airboat.h) */
#define IVP_AIRBOAT_STEERING_RATE      0.005f
#define IVP_AIRBOAT_THRUST_MAX_REVERSE 1000.0f /* N */
#define IVP_AIRBOAT_THRUST_MAX         2000.0f /* N */
#define IVP_AIRBOAT_DRAG_COEFFICIENT   0.003f
#define IVP_AIRBOAT_WATER_DENSITY      1000.0f /* kg/m3 */
#define IVP_AIRBOAT_DRAG_SURFACE_AREA  3.53f   /* m2 */
#define IVP_AIRBOAT_BUOYANCY_SCALAR    1.1f
#define IVP_AIRBOAT_RAYCAST_DIST       0.35f   /* m */
#define IVP_AIRBOAT_RAYCAST_DIST_WATER 0.1f    /* m (unused by the reference) */
#define IVP_AIRBOAT_GRAVITY            7.0f    /* m/s2 */

/* IVP_Raycast_Airboat_Wheel (IVP_FLOAT / IVP_U_Float_Point members) */
typedef struct ivp_airboat_wheel {
    ivp_vec3f_t hp_cs;               /* hard point, core space */
    ivp_vec3f_t raycast_start_cs;    /* ray cast start position, core space */
    ivp_vec3f_t raycast_dir_cs;
    float       raycast_length;
    ivp_vec3f_t spring_direction_cs;
    float       distance_orig_hp_to_hp;
    float       spring_len;
    float       spring_constant;
    float       spring_damp_relax;
    float       spring_damp_compress;
    float       max_rotation_speed;
    float       wheel_radius;
    float       inv_wheel_radius;
    float       friction_of_wheel;
    float       torque;
    bool        wheel_is_fixed;
    ivp_vec3f_t axis_direction_cs;
    float       angle_wheel;
    float       wheel_angular_velocity;
    ivp_vec3f_t surface_speed_of_wheel_on_ground_ws;
    float       pressure;
    float       raycast_dist;
} ivp_airboat_wheel_t;

/* IVP_Raycast_Airboat_Impact: filled by the game-side ray caster */
typedef struct ivp_airboat_impact {
    bool        impact;            /* bImpact: ray hit the ground */
    bool        impact_water;      /* bImpactWater (unused by the controller) */
    bool        in_water;          /* bInWater: ray start point is in water */
    ivp_vec3r_t impact_point_ws;   /* vecImpactPointWS (IVP_U_Point) */
    ivp_vec3f_t impact_normal_ws;  /* vecImpactNormalWS (IVP_U_Float_Point) */
    float       friction;          /* flFriction */
    float       dampening;         /* flDampening (unused by the controller) */
} ivp_airboat_impact_t;

/* Reference do_raycasts_gameside(nRaycastCount, pRays, pImpacts): fill one
 * impact per ray.  Without a callback no pontoon touches ground or water. */
typedef void (*ivp_airboat_raycast_fn_t)(void *ctx, int n_rays,
                                         const ivp_ray_solver_template_t *rays,
                                         ivp_airboat_impact_t *impacts);

typedef struct ivp_airboat_state {
    ivp_object_t      *body;          /* m_pAirboatBody */
    ivp_environment_t *environment;
    short              n_wheels;
    short              n_axis;
    short              wheels_per_axis;
    ivp_airboat_wheel_t wheels[IVP_AIRBOAT_MAX_WHEELS];
    float              stabilizer_constant[IVP_AIRBOAT_MAX_WHEELS / 2];

    float       gravity_y_direction;  /* +-1 */
    ivp_vec3f_t normized_gravity_ws;
    float       extra_gravity;        /* stored, unused (reference) */
    int         index_x, index_y, index_z;
    bool        is_left_handed;
    float       max_speed;
    float       down_force;
    float       down_force_vertical_offset;
    float       booster_force;
    float       booster_seconds_to_go;
    float       booster_seconds_until_ready;
    float       steering_angle;       /* m_SteeringAngle */
    float       thrust;               /* m_flThrust */

    ivp_airboat_raycast_fn_t raycast_fn;
    void                    *raycast_ctx;

    /* internal: the airboat's own gravity controller and its registration */
    void                    *gravity_controller;
    ivp_controller_handle_t  handle;
    bool                     active;
} ivp_airboat_state_t;

/* Reference constructor IVP_Controller_Raycast_Airboat(env, template):
 * uses car_body, n_wheels / n_axis (clamped to 4 / n_wheels), index_x/y/z,
 * is_left_handed, wheel_pos_Bos, trace_pos_Bos, spring_constant / damping /
 * damping_compression / pre_tension, wheel_radius, wheel_max_rotation_speed,
 * stabilizer_constant, extra_gravity_force and body_down_force_vertical_
 * offset of the template (other fields are ignored), and registers the
 * controller.  Returns false on bad arguments. */
bool ivp_airboat_init(ivp_airboat_state_t *state, const ivp_car_system_template_t *tcs);

/* ~IVP_Controller_Raycast_Airboat: removes the controller.  libivp: the
 * body's core gets the environment's standard gravity controller back (the
 * reference leaks its private gravity controller on the core).  Call while
 * the environment and the body exist. */
void ivp_airboat_destroy(ivp_airboat_state_t *state);

static inline void ivp_airboat_set_raycast(ivp_airboat_state_t *state,
                                           ivp_airboat_raycast_fn_t fn, void *ctx) {
    state->raycast_fn = fn;
    state->raycast_ctx = ctx;
}

/* Reference IVP_Controller_Raycast_Airboat API */
void   ivp_airboat_do_steering_wheel(ivp_airboat_state_t *state, int wheel, float s_angle);
void   ivp_airboat_do_steering(ivp_airboat_state_t *state, float steering_angle);
void   ivp_airboat_update_throttle(ivp_airboat_state_t *state, float throttle);
void   ivp_airboat_change_spring_constant(ivp_airboat_state_t *state, int wheel, float spring_constant);
void   ivp_airboat_change_spring_dampening(ivp_airboat_state_t *state, int wheel, float spring_dampening);
void   ivp_airboat_change_spring_dampening_compression(ivp_airboat_state_t *state, int wheel, float spring_dampening);
void   ivp_airboat_change_spring_pre_tension(ivp_airboat_state_t *state, int wheel, float pre_tension_length);
void   ivp_airboat_change_spring_length(ivp_airboat_state_t *state, int wheel, float spring_length);
void   ivp_airboat_change_wheel_torque(ivp_airboat_state_t *state, int wheel, float torque);
void   ivp_airboat_fix_wheel(ivp_airboat_state_t *state, int wheel, bool stop_wheel);
void   ivp_airboat_change_friction_of_wheel(ivp_airboat_state_t *state, int wheel, float friction);
void   ivp_airboat_change_stabilizer_constant(ivp_airboat_state_t *state, int axis, float stabi_constant);
void   ivp_airboat_change_body_downforce(ivp_airboat_state_t *state, float force);
void   ivp_airboat_set_booster_acceleration(ivp_airboat_state_t *state, float acceleration);
void   ivp_airboat_activate_booster(ivp_airboat_state_t *state, float thrust, float duration, float delay);
float  ivp_airboat_get_booster_delay(const ivp_airboat_state_t *state);
double ivp_airboat_get_body_speed(const ivp_airboat_state_t *state, int index);
double ivp_airboat_get_wheel_angular_velocity(const ivp_airboat_state_t *state, int wheel);
double ivp_airboat_get_orig_front_wheel_distance(const ivp_airboat_state_t *state);
double ivp_airboat_get_orig_axles_distance(const ivp_airboat_state_t *state);

/* get_associated_controlled_cores: the body (for announce_dependent_controller). */
int ivp_airboat_discovery(void *ctx, ivp_environment_t *env,
                          ivp_object_handle_t *out_handles, int max_handles);

/* do_simulation_controller; registered by ivp_airboat_init. */
void ivp_controller_airboat(const ivp_event_sim_t *event_sim,
                            ivp_object_t *obj,
                            void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_AIRBOAT_H */
