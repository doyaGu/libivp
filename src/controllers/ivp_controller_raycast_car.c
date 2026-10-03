/* ivp_controller_raycast_car.c -- Raycast vehicle controller
 *
 * Port of IVP_Controller_Raycast_Car (ivp_controller/
 * ivp_controller_raycast_car.cxx) with the reference IVP_FLOAT /
 * IVP_U_Float_Point / IVP_DOUBLE evaluation order.
 */
#include <ivp/controllers/ivp_controller_raycast_car.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include "controllers/ivp_controller_ref.h" /* first: mixed float/double pragma */
#include "solver/ivp_scr_internal.h"
#include "ivp_controller_raycast.h"
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>

#include <math.h>
#include <string.h>

/* ── helpers ───────────────────────────────────────────────────────────── */

static bool car_valid_wheel(const ivp_car_state_t *s, int wheel)
{
    return s && wheel >= 0 && wheel < (int)s->n_wheels;
}

static void car_ensure_in_simulation(ivp_car_state_t *s)
{
    if (s->environment && ivp_controller_handle_is_valid(s->handle)) {
        (void)ivp_environment_ensure_controller_in_simulation(s->environment, s->handle);
    }
}

/* IVP_U_Matrix3::get_col(index, IVP_U_Float_Point *) */
static void car_col_f(const ivp_transform_r_t *m, int col, ivp_vec3f_t *out)
{
    ivp_vec3r_t c;
    lvm_get_col(m, col, &c);
    lvf_set_p(out, &c);
}

/* IVP_U_Float_Point::rotate(axis, angle) */
static void car_rotate_f(ivp_vec3f_t *v, int axis, float angle)
{
    static const int hp[5] = { 0, 1, 2, 0, 1 };
    const int *p = &hp[axis];
    const double c = cos((double)angle);
    const double s = sin((double)angle);
    const double x_new = c * (double)v->k[p[1]] - s * (double)v->k[p[2]];
    v->k[p[2]] = (float)(s * (double)v->k[p[1]] + c * (double)v->k[p[2]]);
    v->k[p[1]] = (float)x_new;
}

/* IVP_U_Point::set_interpolate(const IVP_U_Point *, const IVP_U_Point *, IVP_DOUBLE) */
static void car_interpolate_p(ivp_vec3r_t *out, const ivp_vec3r_t *p0, const ivp_vec3r_t *p1, double s)
{
    const double is = 1.0f - s;
    double a = (double)p0->k[0] * is;
    const double a2 = (double)p1->k[0] * s;
    double b = (double)p0->k[1] * is;
    const double b2 = (double)p1->k[1] * s;
    a += a2;
    const double c = (double)p0->k[2] * is;
    const double c2 = (double)p1->k[2] * s;
    b += b2;
    out->k[0] = (ivp_real_t)a;
    out->k[1] = (ivp_real_t)b;
    out->k[2] = (ivp_real_t)(c + c2);
}

/* GetAxisForWheelIndex */
static int car_axis_for_wheel(const ivp_car_state_t *s, int iWheel)
{
    if (s->n_wheels <= 0 || s->n_axis <= 0) return 0;
    int iAxis = (iWheel * s->n_axis) / s->n_wheels;
    if (iAxis < 0) {
        iAxis = 0;
    } else if (iAxis >= s->n_axis) {
        iAxis = s->n_axis - 1;
    }
    return iAxis;
}

/* BuildAxisWheelBounds */
static void car_axis_wheel_bounds(const ivp_car_state_t *s, int *first, int *last)
{
    IVP_ASSERT(first);
    IVP_ASSERT(last);
    IVP_ASSERT(s->n_axis >= 0);
    IVP_ASSERT(s->n_axis <= IVP_CAR_SYSTEM_MAX_AXIS);

    for (int a = 0; a < s->n_axis; ++a) {
        first[a] = -1;
        last[a] = -1;
    }
    for (int w = 0; w < s->n_wheels; ++w) {
        const int a = car_axis_for_wheel(s, w);
        if (first[a] < 0) first[a] = w;
        last[a] = w;
    }
}

/* GetFrontAndRearAxes */
static bool car_front_and_rear_axes(const ivp_car_state_t *s, int *front, int *rear, int *first, int *last)
{
    IVP_ASSERT(front);
    IVP_ASSERT(rear);
    IVP_ASSERT(first);
    IVP_ASSERT(last);
    *front = -1;
    *rear = -1;
    car_axis_wheel_bounds(s, first, last);
    for (int a = 0; a < s->n_axis; ++a) {
        if (first[a] >= 0) {
            if (*front < 0) *front = a;
            *rear = a;
        }
    }
    return !(*front < 0 || *rear < 0);
}

/* ── construction ──────────────────────────────────────────────────────── */

void ivp_car_do_steering_wheel(ivp_car_state_t *state, int wheel, float s_angle)
{
    if (!car_valid_wheel(state, wheel)) return;
    ivp_car_wheel_t *w = &state->wheels[wheel];
    lvf_zero(&w->axis_direction_cs);
    w->axis_direction_cs.k[state->index_x] = 1.0f;
    car_rotate_f(&w->axis_direction_cs, state->index_y, s_angle);
}

void ivp_car_do_steering(ivp_car_state_t *state, float steering_angle)
{
    if (!state) return;
    /* tell constraint system new steering positions of wheels */
    if (state->steering_angle == steering_angle) return;
    state->steering_angle = steering_angle;
    car_ensure_in_simulation(state);
    for (int i = 0; i < state->n_wheels; i++) {
        if (car_axis_for_wheel(state, i) == 0) ivp_car_do_steering_wheel(state, i, steering_angle);
    }
}

bool ivp_car_init(ivp_car_state_t *state, const ivp_car_system_template_t *tcs)
{
    if (!state) return false;
    memset(state, 0, sizeof(*state));
    state->handle = ivp_controller_handle_invalid();
    if (!tcs || !tcs->car_body || !tcs->car_body->owner) return false;
    ivp_object_t *body = tcs->car_body;
    ivp_environment_t *env = body->owner;

    /* InitRaycastCarBody */
    {
        int template_wheels = tcs->n_wheels;
        if (template_wheels < 0) template_wheels = 0;
        if (template_wheels > IVP_CAR_SYSTEM_MAX_WHEELS) template_wheels = IVP_CAR_SYSTEM_MAX_WHEELS;
        if (template_wheels > IVP_RAYCAST_CAR_MAX_WHEELS) template_wheels = IVP_RAYCAST_CAR_MAX_WHEELS;
        state->n_wheels = (short)template_wheels;

        int template_axis = tcs->n_axes;
        if (template_axis < 1) template_axis = 1;
        const int max_axis_supported = (IVP_CAR_SYSTEM_MAX_AXIS < IVP_RAYCAST_CAR_MAX_AXIS)
                                           ? IVP_CAR_SYSTEM_MAX_AXIS : IVP_RAYCAST_CAR_MAX_AXIS;
        if (template_axis > max_axis_supported) template_axis = max_axis_supported;
        if (template_axis > state->n_wheels) template_axis = (state->n_wheels > 0) ? state->n_wheels : 1;
        state->n_axis = (short)template_axis;
        state->wheels_per_axis = (short)(state->n_wheels / state->n_axis);
    }
    state->car_body = body;
    state->environment = env;
    state->booster_force = 0.0f;
    state->booster_seconds_until_ready = 0.0f;
    state->booster_seconds_to_go = 0.0f;
    state->down_force_vertical_offset = (float)tcs->body_down_force_vertical_offset;
    state->down_force = 0.0f;

    /* InitRaycastCarEnvironment */
    state->index_x = tcs->index_x;
    state->index_y = tcs->index_y;
    state->index_z = tcs->index_z;
    state->is_left_handed = tcs->is_left_handed;
    state->active = true;
    state->handle = ivp_environment_announce_dependent_controller(env, IVP_CP_CONSTRAINTS_MAX,
                                                                  ivp_controller_raycast_car, state,
                                                                  ivp_car_discovery);
    state->extra_gravity = (float)tcs->extra_gravity_force;
    {
        ivp_vec3r_t g;
        ivp_environment_get_gravity(env, &g);
        state->gravity_y_direction = (g.k[state->index_y] > 0) ? 1.0f : -1.0f;
        lvf_set_p(&state->normized_gravity_ws, &g);
        (void)lvf_normize(&state->normized_gravity_ws);
    }

    /* InitRaycastCarWheels */
    {
        ivp_transform_r_t m_core_f_object;
        ivp_ctl_calc_m_core_f_object(body, &m_core_f_object);
        for (int i = 0; i < state->n_wheels; i++) {
            ivp_car_wheel_t *w = &state->wheels[i];
            memset(w, 0, sizeof(*w));
            lvm_vmult4_ff(&m_core_f_object, &tcs->wheel_pos_Bos[i], &w->hp_cs);
            w->spring_len = -(float)tcs->spring_pre_tension[i];
            lvf_zero(&w->spring_direction_cs);
            w->spring_direction_cs.k[state->index_y] = state->gravity_y_direction;
            w->spring_constant = (float)tcs->spring_constant[i];
            w->spring_damp_relax = (float)tcs->spring_damping[i];
            w->spring_damp_compress = (float)tcs->spring_damping_compression[i];
            w->friction_of_wheel = 1.0f; /* pCarSystemTemplate->friction_of_wheel[iWheel]; */
            w->wheel_radius = (float)tcs->wheel_radius[i];
            if (w->wheel_radius < P_FLOAT_EPS) {
                IVP_ASSERT(0);
                w->wheel_radius = P_FLOAT_EPS;
            }
            w->inv_wheel_radius = 1.0f / w->wheel_radius;
            ivp_car_do_steering_wheel(state, i, 0.0f);
            w->wheel_is_fixed = false;
            w->max_rotation_speed = (float)tcs->wheel_max_rotation_speed[i >> 1];
        }
    }

    /* InitRaycastCarAxes */
    state->steering_angle = -1.0f; /* make sure next call is not optimized */
    ivp_car_do_steering(state, 0.0f); /* make sure next call gets through */
    for (int a = 0; a < state->n_axis; a++) {
        state->stabilizer_constant[a] = (float)tcs->stabilizer_constant[a];
    }
    return true;
}

void ivp_car_destroy(ivp_car_state_t *state)
{
    if (!state) return;
    if (state->environment && ivp_controller_handle_is_valid(state->handle)) {
        (void)ivp_environment_remove_dependent_controller(state->environment, state->handle, true);
    }
    state->handle = ivp_controller_handle_invalid();
    state->active = false;
}

bool ivp_car_init_4wheel(ivp_car_state_t *state, ivp_object_t *car_body,
                         ivp_real_t wheelbase, ivp_real_t track_width,
                         ivp_real_t wheel_radius, ivp_real_t suspension_length)
{
    ivp_car_system_template_t t;
    ivp_car_system_template_init(&t, 4, 2);
    t.car_body = car_body;
    const float half_wb = (float)(wheelbase * IVP_REAL(0.5));
    const float half_tw = (float)(track_width * IVP_REAL(0.5));
    for (int i = 0; i < 4; i++) {
        const float x = (i & 1) ? half_tw : -half_tw; /* odd = right */
        const float z = (i & 2) ? -half_wb : half_wb; /* 2, 3 = rear */
        t.wheel_pos_Bos[i] = ivp_vec3f(x, 0.0f, z);
        t.trace_pos_Bos[i] = t.wheel_pos_Bos[i];
        t.wheel_radius[i] = wheel_radius;
        t.spring_constant[i] = IVP_REAL(50000.0);
        t.spring_damping[i] = IVP_REAL(3000.0);
        t.spring_damping_compression[i] = IVP_REAL(3000.0);
        t.spring_pre_tension[i] = -suspension_length;
    }
    return ivp_car_init(state, &t);
}

/* ── setters / getters ─────────────────────────────────────────────────── */

void ivp_car_change_spring_constant(ivp_car_state_t *state, int wheel, float spring_constant)
{
    if (car_valid_wheel(state, wheel)) state->wheels[wheel].spring_constant = spring_constant;
}

void ivp_car_change_spring_dampening(ivp_car_state_t *state, int wheel, float spring_dampening)
{
    if (car_valid_wheel(state, wheel)) state->wheels[wheel].spring_damp_relax = spring_dampening;
}

void ivp_car_change_spring_dampening_compression(ivp_car_state_t *state, int wheel, float spring_dampening)
{
    if (car_valid_wheel(state, wheel)) state->wheels[wheel].spring_damp_compress = spring_dampening;
}

void ivp_car_change_spring_pre_tension(ivp_car_state_t *state, int wheel, float pre_tension_length)
{
    if (car_valid_wheel(state, wheel)) state->wheels[wheel].spring_len = -pre_tension_length;
}

void ivp_car_change_spring_length(ivp_car_state_t *state, int wheel, float spring_length)
{
    if (car_valid_wheel(state, wheel)) state->wheels[wheel].spring_len = spring_length;
}

void ivp_car_change_stabilizer_constant(ivp_car_state_t *state, int axis, float stabi_constant)
{
    if (state && axis >= 0 && axis < IVP_RAYCAST_CAR_MAX_AXIS) state->stabilizer_constant[axis] = stabi_constant;
}

void ivp_car_change_fast_turn_factor(ivp_car_state_t *state, float fast_turn_factor)
{
    (void)state;
    (void)fast_turn_factor; /* reference: fast_turn_factor = fast_turn_factor_ is commented out */
}

void ivp_car_change_wheel_torque(ivp_car_state_t *state, int wheel, float torque)
{
    if (!car_valid_wheel(state, wheel)) return;
    state->wheels[wheel].torque = torque;
    car_ensure_in_simulation(state); /* wake the physics object if need be */
}

void ivp_car_change_body_downforce(ivp_car_state_t *state, float force)
{
    if (state) state->down_force = force;
}

void ivp_car_fix_wheel(ivp_car_state_t *state, int wheel, bool stop_wheel)
{
    if (car_valid_wheel(state, wheel)) state->wheels[wheel].wheel_is_fixed = stop_wheel;
}

void ivp_car_set_booster_acceleration(ivp_car_state_t *state, float acceleration)
{
    if (state) state->booster_force = acceleration;
}

void ivp_car_activate_booster(ivp_car_state_t *state, float thrust, float duration, float delay)
{
    if (!state) return;
    if (state->booster_force != 0.0f) return;
    if (state->booster_seconds_until_ready > 0.0f) return; /* booster not ready yet */
    state->booster_force = thrust;
    state->booster_seconds_to_go = duration;              /* seconds */
    state->booster_seconds_until_ready = duration + delay; /* time when next ignition is possible */
}

float ivp_car_get_booster_delay(const ivp_car_state_t *state)
{
    return state ? state->booster_seconds_until_ready : 0.0f;
}

float ivp_car_get_booster_time_to_go(const ivp_car_state_t *state)
{
    return state ? state->booster_seconds_to_go : 0.0f;
}

double ivp_car_get_body_speed(const ivp_car_state_t *state, int axis_index)
{
    if (!state || !state->car_body) return 0.0;
    const ivp_core_t *core = state->car_body->physical_core;
    ivp_vec3r_t orientation;
    lvm_get_col(&core->m_world_f_core_last_psi, axis_index, &orientation);
    ivp_vec3f_t sp = ivp_core_speed_f(core);
    return lvp_dot_f(&orientation, &sp);
}

double ivp_car_get_wheel_angular_velocity(const ivp_car_state_t *state, int wheel)
{
    return car_valid_wheel(state, wheel) ? (double)state->wheels[wheel].wheel_angular_velocity : 0.0;
}

double ivp_car_get_orig_front_wheel_distance(const ivp_car_state_t *state)
{
    if (!state || state->n_wheels < 2 || state->n_axis < 1) return 0.0f;
    int first[IVP_CAR_SYSTEM_MAX_AXIS], last[IVP_CAR_SYSTEM_MAX_AXIS];
    int front = -1, rear = -1;
    if (!car_front_and_rear_axes(state, &front, &rear, first, last)) return 0.0f;
    if (rear < front) return 0.0f;
    int left = first[front], right = last[front];
    if (left < 0 || right < 0 || left == right) {
        left = 0;
        right = (state->n_wheels > 1) ? 1 : 0;
    }
    const double dist = (double)state->wheels[left].hp_cs.k[state->index_x] -
                        (double)state->wheels[right].hp_cs.k[state->index_x];
    return fabs(dist);
}

double ivp_car_get_orig_axles_distance(const ivp_car_state_t *state)
{
    if (!state || state->n_wheels < 2 || state->n_axis < 2) return 0.0f;
    int first[IVP_CAR_SYSTEM_MAX_AXIS], last[IVP_CAR_SYSTEM_MAX_AXIS];
    int front = -1, rear = -1;
    if (!car_front_and_rear_axes(state, &front, &rear, first, last) || front == rear) return 0.0f;
    const ivp_vec3f_t *f0 = &state->wheels[first[front]].hp_cs, *f1 = &state->wheels[last[front]].hp_cs;
    const ivp_vec3f_t *r0 = &state->wheels[first[rear]].hp_cs, *r1 = &state->wheels[last[rear]].hp_cs;
    const double front_z = (double)(0.5f * (f0->k[state->index_z] + f1->k[state->index_z]));
    const double rear_z = (double)(0.5f * (r0->k[state->index_z] + r1->k[state->index_z]));
    return fabs(front_z - rear_z);
}

void ivp_car_get_skid_info(const ivp_car_state_t *state, ivp_car_wheel_skid_info_t *array_of_skid_info_out)
{
    if (!state || !array_of_skid_info_out) return;
    for (int w = 0; w < state->n_wheels; w++) {
        ivp_car_wheel_skid_info_t *info = &array_of_skid_info_out[w];
        ivp_vec3r_set_zero(&info->last_contact_position_ws);
        info->last_skid_value = 0.0f;
        info->last_skid_time = 0.0f;
    }
}

void ivp_car_set_steering(ivp_car_state_t *state, ivp_real_t angle_rad)
{
    ivp_car_do_steering(state, (float)angle_rad);
}

void ivp_car_set_engine(ivp_car_state_t *state, ivp_real_t force)
{
    if (!state || state->n_wheels <= 0) return;
    state->engine_force = (float)force;
    const int drive_axis = state->n_axis - 1;
    int n_drive = 0;
    for (int i = 0; i < state->n_wheels; i++) {
        if (car_axis_for_wheel(state, i) == drive_axis) n_drive++;
    }
    for (int i = 0; i < state->n_wheels; i++) {
        const float torque = (car_axis_for_wheel(state, i) == drive_axis && n_drive > 0)
                                 ? (float)force * state->wheels[i].wheel_radius / (float)n_drive : 0.0f;
        ivp_car_change_wheel_torque(state, i, torque);
    }
}

void ivp_car_set_brake(ivp_car_state_t *state, ivp_real_t force)
{
    if (!state) return;
    state->brake_force = (float)force;
    for (int i = 0; i < state->n_wheels; i++) ivp_car_fix_wheel(state, i, force > IVP_REAL(0.0));
}

int ivp_car_discovery(void *ctx, ivp_environment_t *env,
                      ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_car_state_t *s = (const ivp_car_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[1] = { s->car_body };
    return ivp_controller_discover_objects(objs, 1, out_handles, max_handles);
}

/* ── simulation (ivp_controller_raycast_car.cxx:107-650) ──────────────── */

/* IVP_Raycast_Car_Wheel_Temp */
typedef struct car_wheel_temp {
    float       friction_value;
    float       stabilizer_force;
    ivp_vec3f_t ground_normal_ws;
    ivp_vec3r_t ground_hit_ws;
    ivp_vec3f_t spring_direction_ws;
    ivp_vec3f_t surface_speed_wheel_ws;
    ivp_vec3f_t projected_surface_speed_wheel_ws;
    ivp_vec3f_t axis_direction_ws;
    ivp_vec3f_t projected_axis_direction_ws;
    float       inv_normal_dot_dir;
} car_wheel_temp_t;

/* libivp default ray caster (the reference do_raycasts is pure virtual):
 * the environment's objects except the body; the hit normal is handed back
 * in the hit object's object space like the reference ray solver. */
static void car_default_raycasts(ivp_car_state_t *s, int n_wheels, const ivp_ray_solver_template_t *rays,
                                 ivp_ray_hit_t *hits, float *frictions)
{
    for (int i = 0; i < n_wheels; i++) {
        ivp_ray_hit_t *hit = &hits[i];
        memset(hit, 0, sizeof(*hit));
        frictions[i] = 1.0f;
        ivp_real_t dist = IVP_REAL(0.0);
        ivp_vec3r_t normal_ws;
        ivp_object_t *hit_obj = NULL;
        if (!s->environment ||
            !ivp_controller_world_raycast(s->environment, s->car_body, &rays[i].ray_start_point,
                                          &rays[i].ray_normized_direction, rays[i].ray_length,
                                          &dist, &normal_ws, &hit_obj) ||
            !hit_obj) {
            continue;
        }
        hit->hit_object = hit_obj;
        hit->hit_distance = (ivp_real_t)(float)dist;
        ivp_transform_r_t tmp;
        lvm_vimult3(ivp_ctl_cache_m_world_f_object(hit_obj, &tmp), &normal_ws, &hit->hit_normal);
        frictions[i] = (float)hit_obj->material.friction;
    }
}

/* SetupWheelRaycasts */
static void car_setup_raycasts(const ivp_car_state_t *s, ivp_ray_solver_template_t *rays,
                               const ivp_transform_r_t *m, car_wheel_temp_t *t)
{
    for (int i = 0; i < s->n_wheels; i++) {
        const ivp_car_wheel_t *w = &s->wheels[i];
        ivp_ray_solver_template_t *r = &rays[i];
        lvm_vmult4_fp(m, &w->hp_cs, &r->ray_start_point);
        lvm_vmult3_f(m, &w->spring_direction_cs, &t[i].spring_direction_ws);
        lvp_set_f(&r->ray_normized_direction, &t[i].spring_direction_ws);
        r->ray_length = (ivp_real_t)(w->spring_len + w->wheel_radius); /* IVP_FLOAT */
        r->ray_flags = IVP_RAY_SOLVER_ALL;
    }
}

/* DoSimulationWheels */
static bool car_wheels(ivp_car_state_t *s, const ivp_ray_solver_template_t *rays, const ivp_transform_r_t *m,
                       car_wheel_temp_t *t, const ivp_ray_hit_t *hits, const float *frictions, ivp_core_t *core)
{
    for (int i = 0; i < s->n_wheels; i++) {
        ivp_car_wheel_t *w = &s->wheels[i];
        car_wheel_temp_t *tw = &t[i];
        const ivp_ray_hit_t *hit = &hits[i];

        if (hit->hit_object) {
            /* impact normal from object space to world space */
            ivp_vec3f_t n_os;
            lvf_set_p(&n_os, &hit->hit_normal);
            ivp_transform_r_t tmp;
            lvm_vmult3_f(ivp_ctl_cache_m_world_f_object(hit->hit_object, &tmp), &n_os, &tw->ground_normal_ws);
            /* distance to move along the ray to reach the impact surface */
            w->raycast_dist = (float)hit->hit_distance;
            tw->inv_normal_dot_dir = (float)((double)1.1f /
                (fabs(lvf_dot(&tw->spring_direction_ws, &tw->ground_normal_ws)) + (double)0.1f));
        } else {
            /* in air: no pressure on the wheel, full extension */
            w->pressure = 0.0f;
            w->raycast_dist = w->spring_len + w->wheel_radius;
            tw->inv_normal_dot_dir = 1.0f;
            lvf_set_multiple(&tw->ground_normal_ws, &tw->spring_direction_ws, (D)-1);
        }

        /* ground friction (if any) * wheel friction */
        tw->friction_value = frictions[i] * w->friction_of_wheel;

        /* the new wheel position (the impact point or the full ray distance) */
        lvp_add_multiple2_pf(&tw->ground_hit_ws, &rays[i].ray_start_point, &tw->spring_direction_ws,
                             (D)w->raycast_dist);

        /* speed at the impact point */
        {
            ivp_vec3r_t sp;
            ivp_core_get_surface_speed_ws(core, &tw->ground_hit_ws, &sp);
            lvf_set_p(&tw->surface_speed_wheel_ws, &sp);
        }
        lvf_set_orthogonal_part(&tw->projected_surface_speed_wheel_ws, &tw->surface_speed_wheel_ws,
                                &tw->ground_normal_ws);

        lvm_vmult3_f(m, &w->axis_direction_cs, &tw->axis_direction_ws);
        lvf_set_orthogonal_part(&tw->projected_axis_direction_ws, &tw->axis_direction_ws, &tw->ground_normal_ws);

        /* libivp: ray result of this PSI */
        w->grounded = hit->hit_object != NULL;
        w->contact_point_ws = tw->ground_hit_ws;
        lvp_set_f(&w->contact_normal_ws, &tw->ground_normal_ws);

        if (!lvf_normize(&tw->projected_axis_direction_ws)) {
            IVP_IF(1) { /* ivp_controller_raycast_car.cxx:387 */
                ivp_debug_printf("IVP_Controller_Raycast_Car::do_simulation_controller projected_axis_direction_ws.normize failed\n");
            }
            return false;
        }
    }
    return true;
}

/* DoSimulationStabilizers */
static void car_stabilizers(ivp_car_state_t *s, car_wheel_temp_t *t)
{
    for (int i = 0; i < s->n_wheels; ++i) t[i].stabilizer_force = 0.0f;
    if (s->n_wheels < 2 || s->n_axis < 1) return;

    int first[IVP_CAR_SYSTEM_MAX_AXIS], last[IVP_CAR_SYSTEM_MAX_AXIS];
    car_axis_wheel_bounds(s, first, last);
    for (int a = 0; a < s->n_axis; ++a) {
        const int i0 = first[a];
        const int i1 = last[a];
        if (i0 < 0 || i1 < 0 || i0 == i1) continue;
        const ivp_car_wheel_t *w0 = &s->wheels[i0];
        const ivp_car_wheel_t *w1 = &s->wheels[i1];
        const double flDiff0 = (double)(w0->raycast_dist - w0->spring_len - w0->wheel_radius);
        const double flDiff1 = (double)(w1->raycast_dist - w1->spring_len - w1->wheel_radius);
        /* raycast vehicles are shakier than real-wheel ones: keep historical scaling */
        const float flStabilizerForce =
            (float)((flDiff1 - flDiff0) * (double)(s->stabilizer_constant[a] * 0.5f));
        t[i0].stabilizer_force += flStabilizerForce;
        t[i1].stabilizer_force -= flStabilizerForce;
    }
}

/* DoSimulationShocks */
static void car_shocks(ivp_car_state_t *s, const car_wheel_temp_t *t, const ivp_ray_hit_t *hits,
                       const ivp_event_sim_t *es, ivp_core_t *core)
{
    for (int i = 0; i < s->n_wheels; i++) {
        ivp_car_wheel_t *w = &s->wheels[i];
        if (!hits[i].hit_object) continue; /* the shocks just go to rest */

        const double flDiff = (double)(w->raycast_dist - w->spring_len - w->wheel_radius);
        if (flDiff >= 0) continue;

        const float flSpringConstant = w->spring_constant;
        const float flSpringRelax = w->spring_damp_relax;
        const float flSpringCompress = w->spring_damp_compress;

        /* static force */
        double flForce = -flDiff * (double)flSpringConstant + (double)t[i].stabilizer_force;
        float flInvNormalDotDir = t[i].inv_normal_dot_dir;
        if (flInvNormalDotDir < 0.0f) flInvNormalDotDir = 0.0f;
        if (flInvNormalDotDir > 3.0f) flInvNormalDotDir = 3.0f;
        flForce *= (double)flInvNormalDotDir;

        /* dynamic force */
        ivp_vec3f_t diff_speed;
        lvf_subtract2(&diff_speed, &t[i].projected_surface_speed_wheel_ws, &t[i].surface_speed_wheel_ws);
        const double flSpeed = lvf_dot(&diff_speed, &t[i].spring_direction_ws);
        if (flSpeed > 0) {
            flForce -= (double)flSpringRelax * flSpeed;
        } else {
            flForce -= (double)flSpringCompress * flSpeed;
        }
        if (flForce < 0) flForce = 0.0f;

        w->pressure = (float)flForce;

        const double flImpulse = flForce * (double)es->delta_time;
        ivp_vec3f_t impulse_ws;
        lvf_set_multiple(&impulse_ws, &t[i].ground_normal_ws, (D)flImpulse);
        ivp_ctl_push_core_ws(core, &t[i].ground_hit_ws, &impulse_ws);
    }
}

/* DoSimulationBooster */
static void car_booster(ivp_car_state_t *s, const ivp_event_sim_t *es, ivp_core_t *core)
{
    const double dt = (double)es->delta_time;
    /* timer for the next boost */
    if (s->booster_seconds_until_ready > 0.0f) {
        s->booster_seconds_until_ready = (float)((double)s->booster_seconds_until_ready - dt);
    }
    /* timer while boosting */
    if (s->booster_seconds_to_go > 0.0f) {
        s->booster_seconds_to_go = (float)((double)s->booster_seconds_to_go - dt);
        if (s->booster_seconds_to_go <= 0.0f) s->booster_force = 0.0f;
    }
    /* boosting: add the boosting force to the speed */
    if (s->booster_force != 0.0f) {
        ivp_vec3f_t booster_dir_ws;
        car_col_f(&core->m_world_f_core_last_psi, s->index_z, &booster_dir_ws);
        ivp_vec3f_t sp = ivp_core_speed_f(core);
        lvf_add_multiple(&sp, &booster_dir_ws, (D)((double)s->booster_force * dt));
        ivp_core_set_speed_f(core, &sp);
    }
}

/* CalcSteeringForces */
static void car_calc_steering_forces(ivp_car_state_t *s, const car_wheel_temp_t *t, ivp_core_t *core,
                                     const ivp_event_sim_t *es, float *forces_straight)
{
    for (int a = 0; a < s->n_axis; ++a) forces_straight[a] = 0.0f;
    if (s->n_axis < 2 || s->n_wheels < 2) return;

    int first[IVP_CAR_SYSTEM_MAX_AXIS], last[IVP_CAR_SYSTEM_MAX_AXIS];
    int iFrontAxis = -1, iRearAxis = -1;
    if (!car_front_and_rear_axes(s, &iFrontAxis, &iRearAxis, first, last) || iFrontAxis == iRearAxis) return;

    const int iFrontWheel0 = first[iFrontAxis];
    const int iFrontWheel1 = last[iFrontAxis];
    const int iRearWheel0 = first[iRearAxis];
    const int iRearWheel1 = last[iRearAxis];
    if (iFrontWheel0 < 0 || iRearWheel0 < 0) return;

    ivp_vec3r_t frontPosWS, backPosWS;
    ivp_vec3f_t frontAxisDirWS, rearAxisDirWS;
    car_interpolate_p(&frontPosWS, &t[iFrontWheel0].ground_hit_ws, &t[iFrontWheel1].ground_hit_ws, (double)0.5f);
    car_interpolate_p(&backPosWS, &t[iRearWheel0].ground_hit_ws, &t[iRearWheel1].ground_hit_ws, (double)0.5f);

    lvf_add2(&frontAxisDirWS, &t[iFrontWheel0].axis_direction_ws, &t[iFrontWheel1].axis_direction_ws);
    if (!lvf_normize(&frontAxisDirWS)) frontAxisDirWS = t[iFrontWheel0].axis_direction_ws;
    lvf_add2(&rearAxisDirWS, &t[iRearWheel0].axis_direction_ws, &t[iRearWheel1].axis_direction_ws);
    if (!lvf_normize(&rearAxisDirWS)) rearAxisDirWS = t[iRearWheel0].axis_direction_ws;

    /* reaction solvers (for the core) at the front and back wheels */
    ivp_scr_f_t crs[2];
    ivp_scr_f_init_translation_ws(&crs[0], core, NULL, &frontPosWS, &frontAxisDirWS, NULL, NULL);
    ivp_scr_f_init_translation_ws(&crs[1], core, NULL, &backPosWS, &rearAxisDirWS, NULL, NULL);

    /* how does a push at the front/back wheel influence the back/front wheel? */
    float front_back = (float)ivp_scr_f_dot3(&crs[0].cr_mult_inv0[0], &crs[1].cross_direction_position_cs0[0]);
    front_back = (float)((double)front_back +
                         (double)ivp_core_get_inv_mass_f(core) * lvf_dot(&frontAxisDirWS, &rearAxisDirWS));

    const double relaxation_rate = 1.2f;
    const double i_dt = (double)es->inverse_delta_time;
    const double a = -(double)crs[0].delta_velocity_ds.k[0] * i_dt * relaxation_rate;
    const double b = -(double)crs[1].delta_velocity_ds.k[0] * i_dt * relaxation_rate;

    /* IVP_Inline_Math::invert_2x2_matrix */
    const double m00 = crs[0].m[0][0];
    const double m01 = (double)front_back;
    const double m10 = (double)front_back;
    const double m11 = crs[1].m[0][0];
    const double D_ = (m00 * m11) - (m01 * m10);
    if (D_ * D_ < P_DOUBLE_EPS * P_DOUBLE_EPS) return; /* IVP_FAULT */
    const double DI = 1.0f / D_;
    const double inv0 = m11 * DI;
    const double inv1 = -m10 * DI;
    const double inv2 = -m01 * DI;
    const double inv3 = m00 * DI;

    const float flFrontForce = (float)(inv0 * a + inv1 * b);
    const float flRearForce = (float)(inv2 * a + inv3 * b);

    const float flInvAxisSpan = 1.0f / (float)(iRearAxis - iFrontAxis);
    for (int iAxis = iFrontAxis; iAxis <= iRearAxis; ++iAxis) {
        if (first[iAxis] < 0) continue;
        const float tt = (float)(iAxis - iFrontAxis) * flInvAxisSpan;
        forces_straight[iAxis] = flFrontForce + (flRearForce - flFrontForce) * tt;
    }
}

/* ApplySteeringForces */
static void car_apply_steering_forces(ivp_car_state_t *s, const car_wheel_temp_t *t, ivp_core_t *core,
                                      const ivp_event_sim_t *es, const float *forces_straight)
{
    if (s->n_wheels <= 0) return;

    float forcesNeededPerAxis[IVP_CAR_SYSTEM_MAX_AXIS];
    for (int a = 0; a < s->n_axis; ++a) forcesNeededPerAxis[a] = forces_straight[a];

    bool bForceFixedWheel = false;
    bool bHasTorque = false;
    for (int i = 0; i < s->n_wheels; ++i) {
        if (s->wheels[i].torque != 0.0f) bHasTorque = true;
    }

    const ivp_vec3f_t speed0 = ivp_core_speed_f(core);
    const float flSpeed = (float)sqrt(lvf_quad_length(&speed0)); /* fast_real_length */
    if (fabsf(flSpeed) < 0.5f && !bHasTorque) bForceFixedWheel = true;

    const float mass = ivp_core_get_mass_f(core);
    const double dt = (double)es->delta_time;
    const double i_dt = (double)es->inverse_delta_time;

    for (int i = 0; i < s->n_wheels; ++i) {
        /* left right */
        ivp_car_wheel_t *w = &s->wheels[i];
        const car_wheel_temp_t *tw = &t[i];

        const float flMaxForce = w->pressure * tw->friction_value;

        ivp_vec3f_t projected_front_dir_ws;
        lvf_cross_and_normize(&projected_front_dir_ws, &tw->projected_axis_direction_ws, &tw->spring_direction_ws);

        float flFrForce = 0.0f;
        const ivp_vec3f_t speed = ivp_core_speed_f(core);
        const float flBodySpeed = (float)lvf_dot(&speed, &projected_front_dir_ws);

        if (w->wheel_is_fixed || bForceFixedWheel) {
            flFrForce = (float)((double)(-.5f * flBodySpeed * mass) * i_dt);
            w->wheel_angular_velocity = 0.0f;
        } else {
            w->wheel_angular_velocity = flBodySpeed * w->inv_wheel_radius;
            const float flSign = flBodySpeed >= 0.0f ? 1.0f : -1.0f;
            const float flFrictionForce = tw->friction_value * (0.25f * mass) * flSign;
            flFrForce = -2.5f * flFrictionForce;
            flFrForce += w->torque * w->inv_wheel_radius;
        }

        const int nAxis = car_axis_for_wheel(s, i);

        double flForceStraight = (double)forcesNeededPerAxis[nAxis];
        const float flQuadSumForce = (float)((double)(flFrForce * flFrForce) + flForceStraight * flForceStraight);
        if (flQuadSumForce > flMaxForce * flMaxForce) {
            const float flFactor = (float)sqrt((double)(flMaxForce * flMaxForce / flQuadSumForce));
            flFrForce *= flFactor;
            flForceStraight *= (double)flFactor;
        }

        forcesNeededPerAxis[nAxis] = (float)((double)forcesNeededPerAxis[nAxis] - flForceStraight);

        w->angle_wheel = (float)((double)w->angle_wheel - (double)w->wheel_angular_velocity * dt);

        /* left right force */
        ivp_vec3f_t impulse;
        lvf_set_multiple(&impulse, &tw->projected_axis_direction_ws, (D)(flForceStraight * dt));
        ivp_ctl_push_core_ws(core, &tw->ground_hit_ws, &impulse);
        /* front back force */
        lvf_set_multiple(&impulse, &projected_front_dir_ws, (D)((double)flFrForce * dt));
        ivp_ctl_push_core_ws(core, &tw->ground_hit_ws, &impulse);
    }
}

void ivp_controller_raycast_car(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx)
{
    ivp_car_state_t *s = (ivp_car_state_t *)ctx;
    if (!s || !s->active || !s->car_body || obj != s->car_body) return;
    ivp_core_t *core = s->car_body->physical_core;
    if (core->physical_unmoveable) return;

    car_wheel_temp_t temp[IVP_RAYCAST_CAR_MAX_WHEELS];
    ivp_ray_solver_template_t rays[IVP_RAYCAST_CAR_MAX_WHEELS];
    ivp_ray_hit_t hits[IVP_RAYCAST_CAR_MAX_WHEELS];
    float frictions[IVP_RAYCAST_CAR_MAX_WHEELS];
    memset(temp, 0, sizeof(temp));
    memset(rays, 0, sizeof(rays));
    memset(hits, 0, sizeof(hits));
    for (int i = 0; i < IVP_RAYCAST_CAR_MAX_WHEELS; i++) frictions[i] = 1.0f;

    const ivp_transform_r_t *m = &core->m_world_f_core_last_psi;

    /* raycasts */
    car_setup_raycasts(s, rays, m, temp);
    if (s->raycast_fn) {
        s->raycast_fn(s->raycast_ctx, event_sim, s->n_wheels, rays, hits, frictions);
    } else {
        car_default_raycasts(s, s->n_wheels, rays, hits, frictions);
    }

    /* wheels */
    if (!car_wheels(s, rays, m, temp, hits, frictions, core)) return;

    /* stabilizers */
    car_stabilizers(s, temp);

    /* extra gravity */
    {
        ivp_vec3f_t sp = ivp_core_speed_f(core);
        lvf_add_multiple(&sp, &s->normized_gravity_ws,
                         (D)((double)(s->extra_gravity * ivp_core_get_inv_mass_f(core)) * (double)event_sim->delta_time));
        ivp_core_set_speed_f(core, &sp);
    }

    /* springs */
    car_shocks(s, temp, hits, event_sim, core);

    /* booster */
    car_booster(s, event_sim, core);

    /* steering */
    {
        float forces_straight[IVP_CAR_SYSTEM_MAX_AXIS];
        car_calc_steering_forces(s, temp, core, event_sim, forces_straight);
        car_apply_steering_forces(s, temp, core, event_sim, forces_straight);
    }
}
