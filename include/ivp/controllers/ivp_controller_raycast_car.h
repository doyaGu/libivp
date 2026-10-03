/* ivp_controller_raycast_car.h -- Raycast vehicle controller
 *
 * Port of IVP_Controller_Raycast_Car (ivp_controller/
 * ivp_controller_raycast_car.{hxx,cxx}) with the reference IVP_FLOAT /
 * IVP_U_Float_Point / IVP_DOUBLE evaluation order:
 *
 *   - construction (ivp_car_init) mirrors the reference constructor
 *     (InitRaycastCarBody / Environment / Wheels / Axes): n_wheels and
 *     n_axis are clamped, the controller is announced (priority
 *     IVP_CP_CONSTRAINTS_MAX, dependent on the body's core), the hard points
 *     come from wheel_pos_Bos through calc_m_core_f_object, spring length =
 *     -spring_pre_tension, ray direction = the core's index_y axis signed by
 *     the gravity direction, friction_of_wheel = 1, and do_steering(0) wakes
 *     the body.  Unlike the fake jetski the body speed is kept and the wheels
 *     start free;
 *   - wheels map to axes by (wheel * n_axis) / n_wheels; the first and last
 *     wheel of an axis are its stabilizer pair, axis 0 is steered;
 *   - every PSI: one ray per wheel (from the hard point, length spring_len +
 *     wheel_radius) is handed to the ray caster (reference do_raycasts, here
 *     ivp_car_set_raycast), then the wheel bookkeeping, the per axis
 *     stabilizers (0.5 * constant), extra gravity, the shocks, the booster,
 *     the front/rear axis core-reaction steering solve (relaxation 1.2,
 *     interpolated over the axes in between) and the friction-clipped wheel
 *     forces (fixed wheels and slow cars without torque brake with
 *     -0.5 * v * m / dt).
 *
 * The reference class is abstract (do_raycasts is pure virtual).  Without a
 * ray caster libivp casts the rays against all objects of the environment
 * except the body (libivp extension, hit friction = the hit object's
 * material friction).
 */
#ifndef IVP_CONTROLLER_RAYCAST_CAR_H
#define IVP_CONTROLLER_RAYCAST_CAR_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/controllers/ivp_car_system.h>
#include <ivp/collision/ivp_ray_solver.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IVP_RAYCAST_CAR_MAX_WHEELS 12 /* IVP_RAYCAST_CAR_MAX_WHEELS (n_wheels is also clamped to 10) */
#define IVP_RAYCAST_CAR_MAX_AXIS   (IVP_RAYCAST_CAR_MAX_WHEELS / 2)

/* IVP_Raycast_Car_Wheel (IVP_FLOAT / IVP_U_Float_Point members) */
typedef struct ivp_car_wheel {
    /* static section */
    ivp_vec3f_t hp_cs;               /* hard point, core system */
    ivp_vec3f_t spring_direction_cs;
    float       distance_orig_hp_to_hp;
    float       spring_len;          /* -pre_tension */
    float       spring_constant;
    float       spring_damp_relax;
    float       spring_damp_compress;
    float       max_rotation_speed;
    float       wheel_radius;
    float       inv_wheel_radius;
    float       friction_of_wheel;

    /* dynamic section */
    float       torque;
    bool        wheel_is_fixed;      /* e.g. handbrake */
    ivp_vec3f_t axis_direction_cs;
    float       angle_wheel;
    float       wheel_angular_velocity;

    /* out */
    ivp_vec3f_t surface_speed_of_wheel_on_ground_ws;
    float       pressure;
    float       raycast_dist;

    /* libivp: ray result of the last PSI (not used by the simulation) */
    bool        grounded;
    ivp_vec3r_t contact_point_ws;    /* ray start + direction * raycast_dist */
    ivp_vec3r_t contact_normal_ws;
} ivp_car_wheel_t;

/* Reference do_raycasts(es, n_wheels, t_in, hits_out, friction_of_object_out):
 * for every ray fill hits_out[i] (hit_object NULL = no hit; hit_normal is the
 * reference hit_surface_direction_os in the hit object's object space and
 * hit_distance the IVP_FLOAT hit distance, both taken as float) and
 * friction_out[i] (used even without a hit). */
typedef void (*ivp_car_raycast_fn_t)(void *ctx, const ivp_event_sim_t *es, int n_wheels,
                                     const ivp_ray_solver_template_t *rays,
                                     ivp_ray_hit_t *hits_out, float *friction_out);

typedef struct ivp_car_wheel_skid_info {
    ivp_vec3r_t last_contact_position_ws;
    float       last_skid_value;
    float       last_skid_time;
} ivp_car_wheel_skid_info_t;

typedef struct ivp_car_state {
    ivp_object_t      *car_body;
    ivp_environment_t *environment;

    short           n_wheels;
    short           n_axis;
    short           wheels_per_axis;
    ivp_car_wheel_t wheels[IVP_RAYCAST_CAR_MAX_WHEELS];
    float           stabilizer_constant[IVP_RAYCAST_CAR_MAX_AXIS]; /* axis_of_car[] */

    float       gravity_y_direction; /* +/-1 */
    ivp_vec3f_t normized_gravity_ws;

    float max_speed;
    int   index_x;
    int   index_y;
    int   index_z;
    bool  is_left_handed;
    float extra_gravity;

    float down_force;
    float down_force_vertical_offset;

    float booster_force;
    float booster_seconds_to_go;
    float booster_seconds_until_ready;

    float steering_angle;

    ivp_car_raycast_fn_t raycast_fn;
    void                *raycast_ctx;

    /* libivp convenience inputs (ivp_car_set_engine / ivp_car_set_brake) */
    float engine_force;
    float brake_force;

    ivp_controller_handle_t handle;
    bool active;
} ivp_car_state_t;

/* ── Construction ──────────────────────────────────────────────────────── */

/* Reference constructor IVP_Controller_Raycast_Car(env, template): uses
 * car_body (must be in an environment), n_wheels / n_axes, index_x/y/z,
 * is_left_handed, wheel_pos_Bos, spring_constant / damping /
 * damping_compression / pre_tension, wheel_radius, wheel_max_rotation_speed,
 * stabilizer_constant, extra_gravity_force and body_down_force_vertical_
 * offset (all taken as IVP_FLOAT) and registers the controller.  The state
 * must not move afterwards.  Returns false on bad arguments. */
bool ivp_car_init(ivp_car_state_t *state, const ivp_car_system_template_t *tcs);

/* ~IVP_Controller_Raycast_Car: removes the controller. */
void ivp_car_destroy(ivp_car_state_t *state);

/* libivp convenience: ivp_car_init with a 4 wheel / 2 axis template: front
 * wheels at +wheelbase/2 along index_z (forward), left wheels at
 * -track_width/2 along index_x, hard points at the body's object origin
 * height, spring length suspension_length, springs 50000 N/m, damping 3000. */
bool ivp_car_init_4wheel(ivp_car_state_t *state, ivp_object_t *car_body,
                         ivp_real_t wheelbase, ivp_real_t track_width,
                         ivp_real_t wheel_radius, ivp_real_t suspension_length);

static inline void ivp_car_set_raycast(ivp_car_state_t *state, ivp_car_raycast_fn_t fn, void *ctx)
{
    state->raycast_fn = fn;
    state->raycast_ctx = ctx;
}

/* ── Reference IVP_Controller_Raycast_Car API ──────────────────────────── */

void   ivp_car_do_steering_wheel(ivp_car_state_t *state, int wheel, float s_angle);
/* do_steering: no-op when the angle is unchanged, else wakes the car and
 * steers the wheels of axis 0. */
void   ivp_car_do_steering(ivp_car_state_t *state, float steering_angle);
void   ivp_car_change_spring_constant(ivp_car_state_t *state, int wheel, float spring_constant);
void   ivp_car_change_spring_dampening(ivp_car_state_t *state, int wheel, float spring_dampening);
void   ivp_car_change_spring_dampening_compression(ivp_car_state_t *state, int wheel, float spring_dampening);
void   ivp_car_change_spring_pre_tension(ivp_car_state_t *state, int wheel, float pre_tension_length);
void   ivp_car_change_spring_length(ivp_car_state_t *state, int wheel, float spring_length);
void   ivp_car_change_stabilizer_constant(ivp_car_state_t *state, int axis, float stabi_constant);
void   ivp_car_change_fast_turn_factor(ivp_car_state_t *state, float fast_turn_factor); /* no-op (reference) */
/* change_wheel_torque: also wakes the car */
void   ivp_car_change_wheel_torque(ivp_car_state_t *state, int wheel, float torque);
void   ivp_car_change_body_downforce(ivp_car_state_t *state, float force);
/* fix_wheel: stop the wheel completely (handbrake); does not wake the car */
void   ivp_car_fix_wheel(ivp_car_state_t *state, int wheel, bool stop_wheel);
void   ivp_car_set_booster_acceleration(ivp_car_state_t *state, float acceleration);
void   ivp_car_activate_booster(ivp_car_state_t *state, float thrust, float duration, float delay);
float  ivp_car_get_booster_delay(const ivp_car_state_t *state);
float  ivp_car_get_booster_time_to_go(const ivp_car_state_t *state);
double ivp_car_get_body_speed(const ivp_car_state_t *state, int axis_index);
double ivp_car_get_wheel_angular_velocity(const ivp_car_state_t *state, int wheel);
double ivp_car_get_orig_front_wheel_distance(const ivp_car_state_t *state);
double ivp_car_get_orig_axles_distance(const ivp_car_state_t *state);
void   ivp_car_get_skid_info(const ivp_car_state_t *state, ivp_car_wheel_skid_info_t *array_of_skid_info_out);

/* ── libivp convenience inputs ─────────────────────────────────────────── */

/* = ivp_car_do_steering */
void ivp_car_set_steering(ivp_car_state_t *state, ivp_real_t angle_rad);
/* change_wheel_torque(force * radius / n) on the n wheels of the last axis
 * (0 on the others) */
void ivp_car_set_engine(ivp_car_state_t *state, ivp_real_t force);
/* force > 0: fix_wheel(true) on every wheel, else release them */
void ivp_car_set_brake(ivp_car_state_t *state, ivp_real_t force);

/* get_associated_controlled_cores: the body. */
int ivp_car_discovery(void *ctx, ivp_environment_t *env,
                      ivp_object_handle_t *out_handles, int max_handles);

/* do_simulation_controller; registered by ivp_car_init. */
void ivp_controller_raycast_car(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_RAYCAST_CAR_H */
