/* ivp_controller_fake_jetski.h — Raycast "fake jetski" controller
 *
 * Port of IVP_Controller_Raycast_Fake_Jetski (ivp_controller/
 * ivp_controller_fake_jetski.{h,cpp}, Valve's 4-wheel raycast vehicle used
 * for a jetski) with the reference float / double evaluation:
 *
 *   - construction (ivp_jetski_init) mirrors the reference constructor: the
 *     body's speed and rotation speed are cleared, the controller is
 *     announced (priority IVP_CP_CONSTRAINTS_MAX, dependent on the body's
 *     core), the wheels are placed from the car system template
 *     (wheel_pos_Bos / trace_pos_Bos through calc_m_core_f_object, spring
 *     length = -spring_pre_tension, ray direction = the core's "down" axis),
 *     ALL wheels start fixed (handbrake, as in the reference) and
 *     do_steering(0) wakes the body;
 *   - every PSI: one ray per wheel (length spring_len + wheel_radius +
 *     hp/trace height difference) is handed to the game-side ray caster
 *     (reference do_raycasts, here ivp_jetski_set_raycast), then the
 *     reference wheel bookkeeping, axle stabilizers (0.5 * constant), extra
 *     gravity, shocks, booster, the front/rear core-reaction steering solve
 *     (relaxation 1.2), the friction-clipped wheel forces and the turn-in-
 *     place hack.
 */
#ifndef IVP_CONTROLLER_FAKE_JETSKI_H
#define IVP_CONTROLLER_FAKE_JETSKI_H

#include <ivp/base/ivp_types.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/controllers/ivp_car_system.h>
#include <ivp/collision/ivp_ray_solver.h>
#include <ivp/math/ivp_vec3.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IVP_JETSKI_MAX_WHEELS 4
#define IVP_JETSKI_MAX_AXES   2

typedef enum ivp_jetski_wheel_pos {
    IVP_JETSKI_FRONT_LEFT  = 0,
    IVP_JETSKI_FRONT_RIGHT = 1,
    IVP_JETSKI_REAR_LEFT   = 2,
    IVP_JETSKI_REAR_RIGHT  = 3
} ivp_jetski_wheel_pos_t;

typedef enum ivp_jetski_axis_pos {
    IVP_JETSKI_AXIS_FRONT = 0,
    IVP_JETSKI_AXIS_REAR  = 1
} ivp_jetski_axis_pos_t;

/* IVP_Raycast_Fake_Jetski_Wheel (IVP_FLOAT / IVP_U_Float_Point members) */
typedef struct ivp_jetski_wheel {
    ivp_vec3f_t hp_cs;
    ivp_vec3f_t raycast_start_cs;
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
} ivp_jetski_wheel_t;

typedef struct ivp_jetski_axle {
    float stabilizer_constant;
} ivp_jetski_axle_t;

/* Reference do_raycasts(es, n_wheels, t_in, hits_out, friction_of_object_out):
 * for every ray fill hits_out[i] (hit_object NULL = no hit; hit_normal is the
 * reference hit_surface_direction_os in the hit object's object space and
 * hit_distance the IVP_FLOAT hit distance, both taken as float) and
 * friction_out[i] (used even without a hit).  Without a callback no wheel
 * touches anything (friction 1). */
typedef void (*ivp_jetski_raycast_fn_t)(void *ctx, const ivp_event_sim_t *es, int n_wheels,
                                        const ivp_ray_solver_template_t *rays,
                                        ivp_ray_hit_t *hits_out, float *friction_out);

typedef struct ivp_jetski_wheel_skid_info {
    ivp_vec3r_t last_contact_position_ws;
    float       last_skid_value;
    float       last_skid_time;
} ivp_jetski_wheel_skid_info_t;

typedef struct ivp_jetski_state {
    ivp_object_t      *body;          /* m_pJetskiBody */
    ivp_environment_t *environment;

    short              n_wheels;
    short              n_axis;
    short              wheels_per_axis;
    ivp_jetski_wheel_t wheels[IVP_JETSKI_MAX_WHEELS];
    ivp_jetski_axle_t  axles[IVP_JETSKI_MAX_AXES];

    float              gravity_y_direction;
    ivp_vec3f_t        normized_gravity_ws;
    float              extra_gravity;

    int                index_x;
    int                index_y;
    int                index_z;
    bool               is_left_handed;
    float              max_speed;

    float              down_force;
    float              down_force_vertical_offset;

    float              booster_force;
    float              booster_seconds_to_go;
    float              booster_seconds_until_ready;

    float              steering_angle;   /* m_SteeringAngle */
    float              steering_rate;    /* m_SteeringRate */

    ivp_jetski_raycast_fn_t raycast_fn;
    void                   *raycast_ctx;

    ivp_controller_handle_t handle;
    bool active;
} ivp_jetski_state_t;

/* Reference constructor IVP_Controller_Raycast_Fake_Jetski(env, template):
 * uses car_body, n_wheels / n_axis (clamped to 4 / n_wheels), index_x/y/z,
 * is_left_handed, wheel_pos_Bos, trace_pos_Bos, spring_constant / damping /
 * damping_compression / pre_tension, wheel_radius, wheel_max_rotation_speed,
 * stabilizer_constant, extra_gravity_force and body_down_force_vertical_
 * offset, and registers the controller.  Returns false on bad arguments. */
bool ivp_jetski_init(ivp_jetski_state_t *state, const ivp_car_system_template_t *tcs);

/* ~IVP_Controller_Raycast_Fake_Jetski: removes the controller. */
void ivp_jetski_destroy(ivp_jetski_state_t *state);

static inline void ivp_jetski_set_raycast(ivp_jetski_state_t *state,
                                          ivp_jetski_raycast_fn_t fn, void *ctx) {
    state->raycast_fn = fn;
    state->raycast_ctx = ctx;
}

/* Reference IVP_Controller_Raycast_Fake_Jetski API */
void   ivp_jetski_do_steering_wheel(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, float s_angle);
void   ivp_jetski_do_steering(ivp_jetski_state_t *state, float steering_angle);
void   ivp_jetski_change_spring_constant(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, float spring_constant);
void   ivp_jetski_change_spring_dampening(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, float spring_dampening);
void   ivp_jetski_change_spring_dampening_compression(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos,
                                                      float spring_dampening);
void   ivp_jetski_change_spring_pre_tension(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos,
                                            float pre_tension_length);
void   ivp_jetski_change_spring_length(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, float spring_length);
void   ivp_jetski_change_wheel_torque(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, float torque);
void   ivp_jetski_fix_wheel(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, bool stop_wheel);
void   ivp_jetski_change_friction_of_wheel(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, float friction);
void   ivp_jetski_change_stabilizer_constant(ivp_jetski_state_t *state, ivp_jetski_axis_pos_t pos,
                                             float stabi_constant);
void   ivp_jetski_change_fast_turn_factor(ivp_jetski_state_t *state, float fast_turn_factor); /* no-op (reference) */
void   ivp_jetski_change_body_downforce(ivp_jetski_state_t *state, float force);
void   ivp_jetski_set_booster_acceleration(ivp_jetski_state_t *state, float acceleration);
void   ivp_jetski_activate_booster(ivp_jetski_state_t *state, float thrust, float duration, float delay);
float  ivp_jetski_get_booster_delay(const ivp_jetski_state_t *state);
double ivp_jetski_get_wheel_angular_velocity(const ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos);
double ivp_jetski_get_body_speed(const ivp_jetski_state_t *state, int axis_index);
double ivp_jetski_get_orig_front_wheel_distance(const ivp_jetski_state_t *state);
double ivp_jetski_get_orig_axles_distance(const ivp_jetski_state_t *state);
void   ivp_jetski_get_skid_info(const ivp_jetski_state_t *state, ivp_jetski_wheel_skid_info_t *array_of_skid_info_out);

/* get_associated_controlled_cores: the body. */
int ivp_jetski_discovery(void *ctx, ivp_environment_t *env,
                         ivp_object_handle_t *out_handles, int max_handles);

/* do_simulation_controller; registered by ivp_jetski_init. */
void ivp_controller_fake_jetski(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_FAKE_JETSKI_H */
