/* ivp_controller_fake_jetski.c — Raycast "fake jetski" controller
 *
 * Port of IVP_Controller_Raycast_Fake_Jetski (ivp_controller/
 * ivp_controller_fake_jetski.cpp) with the reference IVP_FLOAT /
 * IVP_U_Float_Point / IVP_DOUBLE evaluation order.
 */
#include <ivp/controllers/ivp_controller_fake_jetski.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include "controllers/ivp_controller_ref.h" /* first: mixed float/double pragma */
#include "solver/ivp_scr_internal.h"
#include <ivp/base/ivp_message.h>

#include <math.h>
#include <string.h>

/* ── helpers ───────────────────────────────────────────────────────────── */

static bool jetski_valid_wheel(const ivp_jetski_state_t *s, ivp_jetski_wheel_pos_t pos)
{
    return s && (int)pos >= 0 && (int)pos < (int)s->n_wheels;
}

static void jetski_ensure_in_simulation(ivp_jetski_state_t *s)
{
    if (s->environment && ivp_controller_handle_is_valid(s->handle)) {
        (void)ivp_environment_ensure_controller_in_simulation(s->environment, s->handle);
    }
}

/* IVP_U_Matrix3::get_col(index, IVP_U_Float_Point *) */
static void jetski_col_f(const ivp_transform_r_t *m, int col, ivp_vec3f_t *out)
{
    ivp_vec3r_t c;
    lvm_get_col(m, col, &c);
    lvf_set_p(out, &c);
}

/* IVP_U_Float_Point::rotate(axis, angle) */
static void jetski_rotate_f(ivp_vec3f_t *v, int axis, float angle)
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
static void jetski_interpolate_p(ivp_vec3r_t *out, const ivp_vec3r_t *p0, const ivp_vec3r_t *p1, double s)
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

/* ── construction ──────────────────────────────────────────────────────── */

void ivp_jetski_do_steering_wheel(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, float s_angle)
{
    if (!jetski_valid_wheel(state, pos)) return;
    ivp_jetski_wheel_t *w = &state->wheels[pos];
    lvf_zero(&w->axis_direction_cs);
    w->axis_direction_cs.k[state->index_x] = 1.0f;
    jetski_rotate_f(&w->axis_direction_cs, state->index_y, s_angle);
}

void ivp_jetski_do_steering(ivp_jetski_state_t *state, float steering_angle)
{
    if (!state) return;
    if (state->steering_angle == steering_angle) return; /* check for a change */
    state->steering_angle = steering_angle;
    jetski_ensure_in_simulation(state); /* the simulation must be awake: we just got input */
    for (int i = 0; i < state->wheels_per_axis; i++) {
        ivp_jetski_do_steering_wheel(state, (ivp_jetski_wheel_pos_t)i, state->steering_angle);
    }
}

bool ivp_jetski_init(ivp_jetski_state_t *state, const ivp_car_system_template_t *tcs)
{
    if (!state) return false;
    memset(state, 0, sizeof(*state));
    state->handle = ivp_controller_handle_invalid();
    if (!tcs || !tcs->car_body || !tcs->car_body->owner) return false;
    ivp_object_t *body = tcs->car_body;
    ivp_environment_t *env = body->owner;
    ivp_core_t *core = body->physical_core;

    /* InitRaycastCarBody */
    {
        int template_wheels = tcs->n_wheels;
        if (template_wheels < 0) template_wheels = 0;
        if (template_wheels > IVP_CAR_SYSTEM_MAX_WHEELS) template_wheels = IVP_CAR_SYSTEM_MAX_WHEELS;
        if (template_wheels > IVP_JETSKI_MAX_WHEELS) template_wheels = IVP_JETSKI_MAX_WHEELS;
        state->n_wheels = (short)template_wheels;
        int template_axis = tcs->n_axes;
        if (template_axis < 1) template_axis = 1;
        if (template_axis > state->n_wheels) template_axis = (state->n_wheels > 0) ? state->n_wheels : 1;
        state->n_axis = (short)template_axis;
        state->wheels_per_axis = (short)(state->n_wheels / state->n_axis);
    }
    state->body = body;
    state->environment = env;
    state->booster_force = 0.0f;
    state->booster_seconds_until_ready = 0.0f;
    state->booster_seconds_to_go = 0.0f;
    state->down_force_vertical_offset = (float)tcs->body_down_force_vertical_offset;
    state->down_force = 0.0f;
    for (int k = 0; k < 3; k++) {
        core->rot_speed.k[k] = 0.0f;
        core->speed.k[k] = IVP_REAL(0.0);
    }

    /* InitRaycastCarEnvironment */
    state->index_x = tcs->index_x;
    state->index_y = tcs->index_y;
    state->index_z = tcs->index_z;
    state->is_left_handed = tcs->is_left_handed;
    state->active = true;
    state->handle = ivp_environment_announce_dependent_controller(env, IVP_CP_CONSTRAINTS_MAX,
                                                                  ivp_controller_fake_jetski, state,
                                                                  ivp_jetski_discovery);
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
            ivp_jetski_wheel_t *w = &state->wheels[i];
            memset(w, 0, sizeof(*w));
            lvm_vmult4_ff(&m_core_f_object, &tcs->wheel_pos_Bos[i], &w->hp_cs);
            lvm_vmult4_ff(&m_core_f_object, &tcs->trace_pos_Bos[i], &w->raycast_start_cs);
            w->spring_len = -(float)tcs->spring_pre_tension[i];
            lvf_zero(&w->spring_direction_cs);
            w->spring_direction_cs.k[state->index_y] = state->gravity_y_direction;
            w->spring_constant = (float)tcs->spring_constant[i];
            w->spring_damp_relax = (float)tcs->spring_damping[i];
            w->spring_damp_compress = (float)tcs->spring_damping_compression[i];
            w->friction_of_wheel = 1.0f;
            w->wheel_radius = (float)tcs->wheel_radius[i];
            {
                const float wr = (float)tcs->wheel_radius[i];
                w->inv_wheel_radius = (wr > P_FLOAT_EPS) ? (1.0f / wr) : (1.0f / P_FLOAT_EPS);
            }
            ivp_jetski_do_steering_wheel(state, (ivp_jetski_wheel_pos_t)i, 0.0f);
            w->wheel_is_fixed = false;
            w->max_rotation_speed = (float)tcs->wheel_max_rotation_speed[i >> 1];
            w->wheel_is_fixed = true;
        }
    }

    /* InitRaycastCarAxes */
    state->steering_angle = -1.0f; /* make sure the next call is not optimized away */
    state->steering_rate = 0.005f;
    ivp_jetski_do_steering(state, 0.0f);
    for (int a = 0; a < state->n_axis && a < IVP_JETSKI_MAX_AXES; a++) {
        state->axles[a].stabilizer_constant = (float)tcs->stabilizer_constant[a];
    }
    return true;
}

void ivp_jetski_destroy(ivp_jetski_state_t *state)
{
    if (!state) return;
    if (state->environment && ivp_controller_handle_is_valid(state->handle)) {
        (void)ivp_environment_remove_dependent_controller(state->environment, state->handle, true);
    }
    state->handle = ivp_controller_handle_invalid();
    state->active = false;
}

/* ── setters / getters ─────────────────────────────────────────────────── */

void ivp_jetski_change_spring_constant(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, float spring_constant)
{
    if (jetski_valid_wheel(state, pos)) state->wheels[pos].spring_constant = spring_constant;
}

void ivp_jetski_change_spring_dampening(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, float spring_dampening)
{
    if (jetski_valid_wheel(state, pos)) state->wheels[pos].spring_damp_relax = spring_dampening;
}

void ivp_jetski_change_spring_dampening_compression(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos,
                                                    float spring_dampening)
{
    if (jetski_valid_wheel(state, pos)) state->wheels[pos].spring_damp_compress = spring_dampening;
}

void ivp_jetski_change_spring_pre_tension(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos,
                                          float pre_tension_length)
{
    if (jetski_valid_wheel(state, pos)) state->wheels[pos].spring_len = -pre_tension_length;
}

void ivp_jetski_change_spring_length(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, float spring_length)
{
    if (jetski_valid_wheel(state, pos)) state->wheels[pos].spring_len = spring_length;
}

void ivp_jetski_change_wheel_torque(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, float torque)
{
    if (!jetski_valid_wheel(state, pos)) return;
    state->wheels[pos].torque = torque;
    jetski_ensure_in_simulation(state); /* wake the physics object */
}

void ivp_jetski_fix_wheel(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, bool stop_wheel)
{
    if (jetski_valid_wheel(state, pos)) state->wheels[pos].wheel_is_fixed = stop_wheel;
}

void ivp_jetski_change_friction_of_wheel(ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos, float friction)
{
    if (jetski_valid_wheel(state, pos)) state->wheels[pos].friction_of_wheel = friction;
}

void ivp_jetski_change_stabilizer_constant(ivp_jetski_state_t *state, ivp_jetski_axis_pos_t pos,
                                           float stabi_constant)
{
    if (state && (int)pos >= 0 && (int)pos < IVP_JETSKI_MAX_AXES) state->axles[pos].stabilizer_constant = stabi_constant;
}

void ivp_jetski_change_fast_turn_factor(ivp_jetski_state_t *state, float fast_turn_factor)
{
    (void)state;
    (void)fast_turn_factor; /* reference: not implemented for raycasts */
}

void ivp_jetski_change_body_downforce(ivp_jetski_state_t *state, float force)
{
    if (state) state->down_force = force;
}

void ivp_jetski_set_booster_acceleration(ivp_jetski_state_t *state, float acceleration)
{
    if (state) state->booster_force = acceleration;
}

void ivp_jetski_activate_booster(ivp_jetski_state_t *state, float thrust, float duration, float delay)
{
    if (!state) return;
    if (state->booster_force != 0.0f) return;
    if (state->booster_seconds_until_ready > 0.0f) return; /* booster not ready yet */
    state->booster_force = thrust;
    state->booster_seconds_to_go = duration;              /* seconds */
    state->booster_seconds_until_ready = duration + delay; /* next ignition possible */
}

float ivp_jetski_get_booster_delay(const ivp_jetski_state_t *state)
{
    return state ? state->booster_seconds_until_ready : 0.0f;
}

double ivp_jetski_get_wheel_angular_velocity(const ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos)
{
    return jetski_valid_wheel(state, pos) ? (double)state->wheels[pos].wheel_angular_velocity : 0.0;
}

double ivp_jetski_get_body_speed(const ivp_jetski_state_t *state, int axis_index)
{
    if (!state || !state->body) return 0.0;
    const ivp_core_t *core = state->body->physical_core;
    ivp_vec3r_t orientation;
    lvm_get_col(&core->m_world_f_core_last_psi, axis_index, &orientation);
    ivp_vec3f_t sp = ivp_core_speed_f(core);
    return lvp_dot_f(&orientation, &sp);
}

double ivp_jetski_get_orig_front_wheel_distance(const ivp_jetski_state_t *state)
{
    if (!state || state->n_wheels < 2) return 0.0;
    const double dist = (double)(state->wheels[IVP_JETSKI_FRONT_LEFT].hp_cs.k[state->index_x] -
                                 state->wheels[IVP_JETSKI_FRONT_RIGHT].hp_cs.k[state->index_x]);
    return fabs(dist);
}

double ivp_jetski_get_orig_axles_distance(const ivp_jetski_state_t *state)
{
    if (!state || state->n_wheels < 3) return 0.0;
    const double dist = (double)(state->wheels[IVP_JETSKI_FRONT_LEFT].hp_cs.k[state->index_z] -
                                 state->wheels[IVP_JETSKI_REAR_LEFT].hp_cs.k[state->index_z]);
    return fabs(dist);
}

void ivp_jetski_get_skid_info(const ivp_jetski_state_t *state, ivp_jetski_wheel_skid_info_t *array_of_skid_info_out)
{
    if (!state || !array_of_skid_info_out) return;
    for (int w = 0; w < state->n_wheels; w++) {
        ivp_jetski_wheel_skid_info_t *info = &array_of_skid_info_out[w];
        ivp_vec3r_set_zero(&info->last_contact_position_ws);
        info->last_skid_value = 0.0f;
        info->last_skid_time = 0.0f;
    }
}

int ivp_jetski_discovery(void *ctx, ivp_environment_t *env,
                         ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_jetski_state_t *s = (const ivp_jetski_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[1] = { s->body };
    return ivp_controller_discover_objects(objs, 1, out_handles, max_handles);
}

/* ── simulation (ivp_controller_fake_jetski.cpp:30-560) ───────────────── */

/* IVP_Raycast_Fake_Jetski_Wheel_Temp */
typedef struct jetski_wheel_temp {
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
} jetski_wheel_temp_t;

/* SetupWheelRaycasts */
static void jetski_setup_raycasts(const ivp_jetski_state_t *s, ivp_ray_solver_template_t *rays,
                                  const ivp_transform_r_t *m, jetski_wheel_temp_t *t)
{
    for (int i = 0; i < s->n_wheels; i++) {
        const ivp_jetski_wheel_t *w = &s->wheels[i];
        ivp_ray_solver_template_t *r = &rays[i];
        lvm_vmult4_fp(m, &w->raycast_start_cs, &r->ray_start_point);
        lvm_vmult3_f(m, &w->spring_direction_cs, &t[i].spring_direction_ws);
        lvp_set_f(&r->ray_normized_direction, &t[i].spring_direction_ws);
        const float flHeightDelta = w->hp_cs.k[s->index_y] - w->raycast_start_cs.k[s->index_y];
        r->ray_length = (ivp_real_t)(w->spring_len + w->wheel_radius + flHeightDelta);
        r->ray_flags = IVP_RAY_SOLVER_ALL;
    }
}

/* DoSimulationWheels */
static bool jetski_wheels(ivp_jetski_state_t *s, const ivp_transform_r_t *m, jetski_wheel_temp_t *t,
                          const ivp_ray_hit_t *hits, const float *frictions, ivp_core_t *core)
{
    for (int i = 0; i < s->n_wheels; i++) {
        ivp_jetski_wheel_t *w = &s->wheels[i];
        jetski_wheel_temp_t *tw = &t[i];
        const ivp_ray_hit_t *hit = &hits[i];

        if (hit->hit_object) {
            /* impact normal from object space to world space */
            ivp_vec3f_t n_os;
            lvf_set_p(&n_os, &hit->hit_normal);
            ivp_transform_r_t tmp;
            lvm_vmult3_f(ivp_ctl_cache_m_world_f_object(hit->hit_object, &tmp), &n_os, &tw->ground_normal_ws);
            const float flHeightDelta = w->hp_cs.k[s->index_y] - w->raycast_start_cs.k[s->index_y];
            w->raycast_dist = (float)hit->hit_distance - flHeightDelta;
            tw->inv_normal_dot_dir = (float)((double)1.1f /
                (fabs(lvf_dot(&tw->spring_direction_ws, &tw->ground_normal_ws)) + (double)0.1f));
        } else {
            /* wheel in air: no pressure, full extension */
            w->pressure = 0.0f;
            const float flHeightDelta = w->hp_cs.k[s->index_y] - w->raycast_start_cs.k[s->index_y];
            w->raycast_dist = w->spring_len + w->wheel_radius - flHeightDelta;
            tw->inv_normal_dot_dir = 1.0f;
            lvf_set_multiple(&tw->ground_normal_ws, &tw->spring_direction_ws, (D)-1);
        }

        tw->friction_value = frictions[i] * w->friction_of_wheel;

        /* the wheel position (impact point or full ray distance), from the wheel */
        ivp_vec3r_t wheelPosWS;
        lvm_vmult4_fp(m, &w->hp_cs, &wheelPosWS);
        lvp_add_multiple2_pf(&tw->ground_hit_ws, &wheelPosWS, &tw->spring_direction_ws, (D)w->raycast_dist);

        /* "verify - just improves accuracy" */
        ivp_vec3r_t tracePosWS;
        lvm_vmult4_fp(m, &w->raycast_start_cs, &tracePosWS);
        const float flHeightDelta = w->hp_cs.k[s->index_y] - w->raycast_start_cs.k[s->index_y];
        const float flRayDistance = w->raycast_dist + flHeightDelta;
        const float flHeight = (float)((double)tracePosWS.k[s->index_y] +
                                       (double)(tw->spring_direction_ws.k[s->index_y] * flRayDistance));
        tw->ground_hit_ws.k[s->index_y] = (ivp_real_t)flHeight;

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
        if (!lvf_normize(&tw->projected_axis_direction_ws)) {
            ivp_debug_printf("IVP_Controller_Raycast_Fake_Jetski::do_simulation_controller projected_axis_direction_ws.normize failed\n");
            return false;
        }
    }
    return true;
}

/* DoSimulationStabilizers */
static void jetski_stabilizers(ivp_jetski_state_t *s, jetski_wheel_temp_t *t)
{
    if (s->wheels_per_axis == 2) {
        for (unsigned int a = 0; a < (unsigned int)s->n_axis; a++) {
            const ivp_jetski_wheel_t *w0 = &s->wheels[a * 2u];
            const ivp_jetski_wheel_t *w1 = &s->wheels[a * 2u + 1u];
            const double flDiff0 = (double)(w0->raycast_dist - w0->spring_len - w0->wheel_radius);
            const double flDiff1 = (double)(w1->raycast_dist - w1->spring_len - w1->wheel_radius);
            /* raycast vehicles are shakier than real-wheel ones: 0.5 factor */
            t[2u * a].stabilizer_force =
                (float)((flDiff1 - flDiff0) * (double)(s->axles[a].stabilizer_constant * 0.5f));
            t[2u * a + 1u].stabilizer_force = -t[2u * a].stabilizer_force;
        }
    } else {
        for (int i = 0; i < s->n_wheels; i++) t[i].stabilizer_force = 0.0f;
    }
}

/* DoSimulationShocks */
static void jetski_shocks(ivp_jetski_state_t *s, const jetski_wheel_temp_t *t, const ivp_ray_hit_t *hits,
                          const ivp_event_sim_t *es, ivp_core_t *core)
{
    for (int i = 0; i < s->n_wheels; i++) {
        ivp_jetski_wheel_t *w = &s->wheels[i];
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
static void jetski_booster(ivp_jetski_state_t *s, const ivp_event_sim_t *es, ivp_core_t *core)
{
    const double dt = (double)es->delta_time;
    if (s->booster_seconds_until_ready > 0.0f) {
        s->booster_seconds_until_ready = (float)((double)s->booster_seconds_until_ready - dt);
    }
    if (s->booster_seconds_to_go > 0.0f) {
        s->booster_seconds_to_go = (float)((double)s->booster_seconds_to_go - dt);
        if (s->booster_seconds_to_go <= 0.0f) s->booster_force = 0.0f;
    }
    if (s->booster_force != 0.0f) {
        ivp_vec3f_t booster_dir_ws;
        jetski_col_f(&core->m_world_f_core_last_psi, s->index_z, &booster_dir_ws);
        ivp_vec3f_t sp = ivp_core_speed_f(core);
        lvf_add_multiple(&sp, &booster_dir_ws, (D)((double)s->booster_force * dt));
        ivp_core_set_speed_f(core, &sp);
    }
}

/* CalcSteeringForces */
static void jetski_calc_steering_forces(ivp_jetski_state_t *s, const jetski_wheel_temp_t *t,
                                        ivp_core_t *core, const ivp_event_sim_t *es, float *forces_straight)
{
    ivp_vec3r_t frontPosWS, backPosWS;
    int iRearWheel;
    if (s->wheels_per_axis == 2) {
        /* mid points of the tire locations front and back */
        jetski_interpolate_p(&frontPosWS, &t[IVP_JETSKI_FRONT_LEFT].ground_hit_ws,
                             &t[IVP_JETSKI_FRONT_RIGHT].ground_hit_ws, (double)0.5f);
        jetski_interpolate_p(&backPosWS, &t[IVP_JETSKI_REAR_LEFT].ground_hit_ws,
                             &t[IVP_JETSKI_REAR_RIGHT].ground_hit_ws, (double)0.5f);
        iRearWheel = IVP_JETSKI_REAR_LEFT;
    } else {
        frontPosWS = t[0].ground_hit_ws;
        backPosWS = t[1].ground_hit_ws;
        iRearWheel = 1;
    }

    ivp_scr_f_t crs[2];
    ivp_scr_f_init_translation_ws(&crs[0], core, NULL, &frontPosWS, &t[IVP_JETSKI_FRONT_LEFT].axis_direction_ws,
                                  NULL, NULL);
    ivp_scr_f_init_translation_ws(&crs[1], core, NULL, &backPosWS, &t[iRearWheel].axis_direction_ws, NULL, NULL);

    /* how does a push at the front/back wheel influence the back/front wheel? */
    float front_back = (float)ivp_scr_f_dot3(&crs[0].cr_mult_inv0[0], &crs[1].cross_direction_position_cs0[0]);
    front_back = (float)((double)front_back +
                         (double)ivp_core_get_inv_mass_f(core) *
                             lvf_dot(&t[IVP_JETSKI_FRONT_LEFT].axis_direction_ws, &t[iRearWheel].axis_direction_ws));

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
    if (D_ * D_ < P_DOUBLE_EPS * P_DOUBLE_EPS) {
        forces_straight[0] = 0.0f;
        forces_straight[1] = 0.0f;
        return;
    }
    const double DI = 1.0f / D_;
    const double inv0 = m11 * DI;
    const double inv1 = -m10 * DI;
    const double inv2 = -m01 * DI;
    const double inv3 = m00 * DI;
    forces_straight[0] = (float)(inv0 * a + inv1 * b);
    forces_straight[1] = (float)(inv2 * a + inv3 * b);
}

/* TurnInPlaceHack */
static void jetski_turn_in_place_hack(ivp_jetski_state_t *s, const jetski_wheel_temp_t *t, ivp_core_t *core,
                                      const ivp_event_sim_t *es)
{
    /* any wheel fixed or torque applied: no hack */
    for (int i = 0; i < s->n_wheels; i++) {
        const ivp_jetski_wheel_t *w = &s->wheels[i];
        if (w->wheel_is_fixed || (w->torque != 0.0f)) return;
    }

    const ivp_vec3f_t speed0 = ivp_core_speed_f(core);
    const float flSpeed = (float)sqrt(lvf_quad_length(&speed0)); /* fast_real_length */
    const float flSteeringAngle = s->steering_angle;
    const float mass = ivp_core_get_mass_f(core);

    /* slow enough and want to turn: turn in place */
    if (flSpeed < 1.0f) {
        for (int i = 0; i < s->n_wheels; i++) {
            const ivp_jetski_wheel_t *w = &s->wheels[i];
            const jetski_wheel_temp_t *tw = &t[i];

            ivp_vec3f_t projected_front_dir_ws;
            lvf_cross_and_normize(&projected_front_dir_ws, &tw->projected_axis_direction_ws, &tw->spring_direction_ws);

            float flForce = 0.0f;
            const ivp_vec3f_t speed = ivp_core_speed_f(core);
            const float flBodySpeed = (float)lvf_dot(&speed, &projected_front_dir_ws);
            const float flMaxForce = w->pressure * tw->friction_value;

            if (fabsf(flSteeringAngle) > 0.1f) {
                const float flSign = flBodySpeed >= 0.0f ? 1.0f : -1.0f;
                const float flFrictionForce = tw->friction_value * (0.25f * mass) * flSign;
                flForce = -2.5f * flFrictionForce;
                flForce += 750.0f;

                const float flSumForce = flForce * flForce;
                if (flSumForce > flMaxForce * flMaxForce) {
                    const float flFactor = (float)sqrt((double)(flMaxForce * flMaxForce / flSumForce));
                    flForce *= flFactor;
                }
            } else {
                flForce = (float)((double)(-0.5f * flBodySpeed * mass) * (double)es->inverse_delta_time);
            }

            ivp_vec3f_t impulse;
            lvf_set_multiple(&impulse, &projected_front_dir_ws, (D)((double)flForce * (double)es->delta_time));
            ivp_ctl_push_core_ws(core, &tw->ground_hit_ws, &impulse);
        }
    }
}

/* ApplySteeringForces */
static void jetski_apply_steering_forces(ivp_jetski_state_t *s, const jetski_wheel_temp_t *t, ivp_core_t *core,
                                         const ivp_event_sim_t *es, float *forces_straight)
{
    const bool bForceFixedWheel = false;
    const float mass = ivp_core_get_mass_f(core);
    const double dt = (double)es->delta_time;
    const double i_dt = (double)es->inverse_delta_time;

    for (int i = 0; i < s->n_wheels; i++) {
        ivp_jetski_wheel_t *w = &s->wheels[i];
        const jetski_wheel_temp_t *tw = &t[i];

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

        const int nAxis = i / s->wheels_per_axis;
        double flForceRotational = (double)forces_straight[nAxis];

        const float flQuadSumForce = (float)((double)(flFrForce * flFrForce) + flForceRotational * flForceRotational);
        if (flQuadSumForce > flMaxForce * flMaxForce) {
            const float flFactor = (float)sqrt((double)(flMaxForce * flMaxForce / flQuadSumForce));
            flFrForce *= flFactor;
            flForceRotational *= (double)flFactor;
        }

        forces_straight[nAxis] = (float)((double)forces_straight[nAxis] - flForceRotational);

        w->angle_wheel = (float)((double)w->angle_wheel - (double)w->wheel_angular_velocity * dt);

        /* left / right force */
        ivp_vec3f_t impulse;
        lvf_set_multiple(&impulse, &tw->projected_axis_direction_ws, (D)(flForceRotational * dt));
        ivp_ctl_push_core_ws(core, &tw->ground_hit_ws, &impulse);

        /* front / back force */
        lvf_set_multiple(&impulse, &projected_front_dir_ws, (D)((double)flFrForce * dt));
        ivp_ctl_push_core_ws(core, &tw->ground_hit_ws, &impulse);
    }

    /* hack: turn in place */
    jetski_turn_in_place_hack(s, t, core, es);
}

void ivp_controller_fake_jetski(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx)
{
    ivp_jetski_state_t *s = (ivp_jetski_state_t *)ctx;
    if (!s || !s->active || !s->body || obj != s->body) return;
    ivp_core_t *core = s->body->physical_core;
    if (core->physical_unmoveable) return;

    jetski_wheel_temp_t temp[IVP_JETSKI_MAX_WHEELS];
    ivp_ray_solver_template_t rays[IVP_JETSKI_MAX_WHEELS];
    ivp_ray_hit_t hits[IVP_JETSKI_MAX_WHEELS];
    float frictions[IVP_JETSKI_MAX_WHEELS];
    memset(temp, 0, sizeof(temp));
    memset(rays, 0, sizeof(rays));
    memset(hits, 0, sizeof(hits));
    for (int i = 0; i < IVP_JETSKI_MAX_WHEELS; i++) frictions[i] = 1.0f;

    const ivp_transform_r_t *m = &core->m_world_f_core_last_psi;

    /* raycasts */
    jetski_setup_raycasts(s, rays, m, temp);
    if (s->raycast_fn) s->raycast_fn(s->raycast_ctx, event_sim, s->n_wheels, rays, hits, frictions);

    /* wheels */
    if (!jetski_wheels(s, m, temp, hits, frictions, core)) return;

    /* stabilizers */
    jetski_stabilizers(s, temp);

    /* extra gravity */
    {
        ivp_vec3f_t sp = ivp_core_speed_f(core);
        lvf_add_multiple(&sp, &s->normized_gravity_ws,
                         (D)((double)(s->extra_gravity * ivp_core_get_inv_mass_f(core)) * (double)event_sim->delta_time));
        ivp_core_set_speed_f(core, &sp);
    }

    /* springs, booster */
    jetski_shocks(s, temp, hits, event_sim, core);
    jetski_booster(s, event_sim, core);

    /* steering */
    {
        float forces_straight[IVP_JETSKI_MAX_WHEELS] = { 0.0f, 0.0f, 0.0f, 0.0f };
        jetski_calc_steering_forces(s, temp, core, event_sim, forces_straight);
        jetski_apply_steering_forces(s, temp, core, event_sim, forces_straight);
    }
}
