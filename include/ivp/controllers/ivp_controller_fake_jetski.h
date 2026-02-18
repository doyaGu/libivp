/* ivp_controller_fake_jetski.h - Legacy-style fake jetski raycast controller */
#ifndef IVP_CONTROLLER_FAKE_JETSKI_H
#define IVP_CONTROLLER_FAKE_JETSKI_H

#include <ivp/base/ivp_types.h>
#include <ivp/controllers/ivp_controller.h>
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

typedef struct ivp_jetski_wheel {
    /* Static/config */
    ivp_vec3r_t hp_cs;
    ivp_vec3r_t raycast_start_cs;
    ivp_vec3r_t spring_direction_cs;
    ivp_real_t  distance_orig_hp_to_hp;
    ivp_real_t  spring_len;
    ivp_real_t  spring_constant;
    ivp_real_t  spring_damp_relax;
    ivp_real_t  spring_damp_compress;
    ivp_real_t  max_rotation_speed;
    ivp_real_t  wheel_radius;
    ivp_real_t  inv_wheel_radius;
    ivp_real_t  friction_of_wheel;

    /* Dynamic */
    ivp_real_t  torque;
    bool        wheel_is_fixed;
    ivp_vec3r_t axis_direction_cs;
    ivp_real_t  angle_wheel;
    ivp_real_t  wheel_angular_velocity;

    /* Runtime output */
    ivp_vec3r_t surface_speed_of_wheel_on_ground_ws;
    ivp_real_t  pressure;
    ivp_real_t  raycast_dist;
} ivp_jetski_wheel_t;

typedef struct ivp_jetski_axle {
    ivp_real_t stabilizer_constant;
} ivp_jetski_axle_t;

typedef struct ivp_jetski_raycast_result {
    bool         hit;
    ivp_real_t   distance;
    ivp_vec3r_t  hit_point;
    ivp_vec3r_t  hit_normal;
    ivp_real_t   friction;
    ivp_object_t *hit_object;
} ivp_jetski_raycast_result_t;

typedef void (*ivp_jetski_raycast_fn_t)(void *ctx,
                                        const ivp_vec3r_t *start_ws,
                                        const ivp_vec3r_t *dir_ws,
                                        ivp_real_t max_dist,
                                        ivp_jetski_raycast_result_t *result);

typedef struct ivp_jetski_wheel_skid_info {
    ivp_vec3r_t last_contact_position_ws;
    ivp_real_t  last_skid_value;
    ivp_real_t  last_skid_time;
} ivp_jetski_wheel_skid_info_t;

typedef struct ivp_jetski_state {
    ivp_object_t      *body;

    short              n_wheels;
    short              n_axis;
    short              wheels_per_axis;
    ivp_jetski_wheel_t wheels[IVP_JETSKI_MAX_WHEELS];
    ivp_jetski_axle_t  axles[IVP_JETSKI_MAX_AXES];

    ivp_real_t         gravity_y_direction;
    ivp_vec3r_t        normized_gravity_ws;
    ivp_real_t         extra_gravity;

    int                index_x;
    int                index_y;
    int                index_z;
    bool               is_left_handed;
    ivp_real_t         max_speed;

    ivp_real_t         down_force;
    ivp_real_t         down_force_vertical_offset;

    ivp_real_t         booster_force;
    ivp_real_t         booster_seconds_to_go;
    ivp_real_t         booster_seconds_until_ready;

    ivp_real_t         steering_angle;
    ivp_real_t         steering_rate;

    ivp_jetski_raycast_fn_t raycast_fn;
    void                   *raycast_ctx;

    bool active;
} ivp_jetski_state_t;

void ivp_jetski_init(ivp_jetski_state_t *state,
                     ivp_object_t *body,
                     ivp_real_t length,
                     ivp_real_t width,
                     ivp_real_t suspension_length);

void ivp_jetski_set_axes(ivp_jetski_state_t *state,
                         int index_x,
                         int index_y,
                         int index_z,
                         bool is_left_handed);

void ivp_jetski_set_gravity(ivp_jetski_state_t *state, const ivp_vec3r_t *gravity_ws);

static inline void ivp_jetski_set_raycast(ivp_jetski_state_t *state,
                                          ivp_jetski_raycast_fn_t fn,
                                          void *ctx) {
    state->raycast_fn = fn;
    state->raycast_ctx = ctx;
}

void ivp_jetski_do_steering_wheel(ivp_jetski_state_t *state,
                                  ivp_jetski_wheel_pos_t wheel_pos,
                                  ivp_real_t steering_angle);

void ivp_jetski_do_steering(ivp_jetski_state_t *state, ivp_real_t steering_angle);

void ivp_jetski_change_spring_constant(ivp_jetski_state_t *state,
                                       ivp_jetski_wheel_pos_t pos,
                                       ivp_real_t spring_constant);

void ivp_jetski_change_spring_dampening(ivp_jetski_state_t *state,
                                        ivp_jetski_wheel_pos_t pos,
                                        ivp_real_t spring_dampening);

void ivp_jetski_change_spring_dampening_compression(ivp_jetski_state_t *state,
                                                    ivp_jetski_wheel_pos_t pos,
                                                    ivp_real_t spring_dampening);

void ivp_jetski_change_spring_pre_tension(ivp_jetski_state_t *state,
                                          ivp_jetski_wheel_pos_t pos,
                                          ivp_real_t pre_tension_length);

void ivp_jetski_change_spring_length(ivp_jetski_state_t *state,
                                     ivp_jetski_wheel_pos_t pos,
                                     ivp_real_t spring_length);

void ivp_jetski_change_wheel_torque(ivp_jetski_state_t *state,
                                    ivp_jetski_wheel_pos_t pos,
                                    ivp_real_t torque);

void ivp_jetski_fix_wheel(ivp_jetski_state_t *state,
                          ivp_jetski_wheel_pos_t pos,
                          bool fixed);

void ivp_jetski_change_friction_of_wheel(ivp_jetski_state_t *state,
                                         ivp_jetski_wheel_pos_t pos,
                                         ivp_real_t friction);

void ivp_jetski_change_stabilizer_constant(ivp_jetski_state_t *state,
                                           ivp_jetski_axis_pos_t pos,
                                           ivp_real_t stabi_constant);

void ivp_jetski_change_fast_turn_factor(ivp_jetski_state_t *state,
                                        ivp_real_t fast_turn_factor);

void ivp_jetski_change_body_downforce(ivp_jetski_state_t *state, ivp_real_t force);

void ivp_jetski_set_booster_acceleration(ivp_jetski_state_t *state, ivp_real_t acceleration);

void ivp_jetski_activate_booster(ivp_jetski_state_t *state,
                                 ivp_real_t thrust,
                                 ivp_real_t duration,
                                 ivp_real_t delay);

ivp_real_t ivp_jetski_get_booster_delay(const ivp_jetski_state_t *state);

ivp_real_t ivp_jetski_get_wheel_angular_velocity(const ivp_jetski_state_t *state,
                                                 ivp_jetski_wheel_pos_t pos);

ivp_real_t ivp_jetski_get_body_speed(const ivp_jetski_state_t *state, int axis_index);

ivp_real_t ivp_jetski_get_orig_front_wheel_distance(const ivp_jetski_state_t *state);

ivp_real_t ivp_jetski_get_orig_axles_distance(const ivp_jetski_state_t *state);

void ivp_jetski_get_skid_info(const ivp_jetski_state_t *state,
                              ivp_jetski_wheel_skid_info_t *array_of_skid_info_out);

void ivp_controller_fake_jetski(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_FAKE_JETSKI_H */
