/* ivp_controller_airboat.c — Raycast airboat controller
 *
 * Port of IVP_Controller_Raycast_Airboat (ivp_controller/
 * ivp_controller_airboat.cpp) with the reference IVP_FLOAT /
 * IVP_U_Float_Point / IVP_DOUBLE evaluation order.
 */
#include <ivp/controllers/ivp_controller_airboat.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/base/ivp_message.h>
#include "controllers/ivp_controller_ref.h"

#include <math.h>
#include <string.h>

/* ── the airboat's IVP_Standard_Gravity_Controller ─────────────────────── */

typedef struct airboat_gravity {
    ivp_sim_ctrl_t base;
    ivp_allocator_t allocator;       /* copied: user state may outlive environment */
    ivp_vec3f_t    grav_vec;   /* IVP_U_Float_Point */
} airboat_gravity_t;

static int ag_prio(ivp_sim_ctrl_t *c) { (void)c; return IVP_CP_GRAVITY; }
static ivp_pvec_t *ag_assoc(ivp_sim_ctrl_t *c) { (void)c; return NULL; }
static void ag_core_deleted(ivp_sim_ctrl_t *c, ivp_core_t *core) { (void)c; (void)core; }

/* IVP_Standard_Gravity_Controller::do_simulation_controller */
static void ag_do_simulation(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *core_list)
{
    airboat_gravity_t *g = (airboat_gravity_t *)c;
    for (int i = ivp_pvec_len(core_list) - 1; i >= 0; i--) {
        ivp_core_t *my_core = (ivp_core_t *)ivp_pvec_at(core_list, i);
        if (!my_core->pinned) {
            ivp_core_global_damp_core(my_core, es->delta_time);
            ivp_core_commit_all_async_pushes_ref(my_core);
            ivp_vec3f_t sp = ivp_core_speed_f(my_core);
            lvf_add_multiple(&sp, &g->grav_vec, (D)es->delta_time);
            ivp_core_set_speed_f(my_core, &sp);
        }
    }
}

static const ivp_sim_ctrl_vt_t ag_vt = { ag_prio, ag_do_simulation, ag_assoc, ag_core_deleted, NULL };

/* ── helpers ───────────────────────────────────────────────────────────── */

static bool airboat_valid_wheel(const ivp_airboat_state_t *s, int w)
{
    return s && w >= 0 && w < (int)s->n_wheels;
}

static void airboat_ensure_in_simulation(ivp_airboat_state_t *s)
{
    if (s->environment && ivp_controller_handle_is_valid(s->handle)) {
        (void)ivp_environment_ensure_controller_in_simulation(s->environment, s->handle);
    }
}

/* IVP_U_Matrix3::get_col(index, IVP_U_Float_Point *) */
static void airboat_col_f(const ivp_transform_r_t *m, int col, ivp_vec3f_t *out)
{
    ivp_vec3r_t c;
    lvm_get_col(m, col, &c);
    lvf_set_p(out, &c);
}

/* IVP_U_Float_Point::rotate(axis, angle) */
static void airboat_rotate_f(ivp_vec3f_t *v, int axis, float angle)
{
    static const int hp[5] = { 0, 1, 2, 0, 1 };
    const int *p = &hp[axis];
    const double c = cos((double)angle);
    const double s = sin((double)angle);
    const double x_new = c * (double)v->k[p[1]] - s * (double)v->k[p[2]];
    v->k[p[2]] = (float)(s * (double)v->k[p[1]] + c * (double)v->k[p[2]]);
    v->k[p[1]] = (float)x_new;
}

/* ── construction ──────────────────────────────────────────────────────── */

void ivp_airboat_do_steering_wheel(ivp_airboat_state_t *state, int wheel, float s_angle)
{
    if (!airboat_valid_wheel(state, wheel)) return;
    ivp_airboat_wheel_t *w = &state->wheels[wheel];
    lvf_zero(&w->axis_direction_cs);
    w->axis_direction_cs.k[state->index_x] = 1.0f;
    airboat_rotate_f(&w->axis_direction_cs, state->index_y, s_angle);
}

void ivp_airboat_do_steering(ivp_airboat_state_t *state, float steering_angle)
{
    if (!state) return;
    /* check for a change */
    if (state->steering_angle == steering_angle) return;
    state->steering_angle = steering_angle;
    /* make sure the simulation is awake */
    airboat_ensure_in_simulation(state);
    for (int i = 0; i < state->wheels_per_axis; i++) {
        ivp_airboat_do_steering_wheel(state, i, state->steering_angle);
    }
}

bool ivp_airboat_init(ivp_airboat_state_t *state, const ivp_car_system_template_t *tcs)
{
    if (!state) return false;
    memset(state, 0, sizeof(*state));
    state->handle = ivp_controller_handle_invalid();
    if (!tcs || !tcs->car_body || !tcs->car_body->owner) return false;
    ivp_object_t *body = tcs->car_body;
    ivp_environment_t *env = body->owner;
    ivp_core_t *core = body->physical_core;

    const ivp_allocator_t *allocator = ivp_env_get_allocator(env);
    airboat_gravity_t *gravity = (airboat_gravity_t *)ivp_alloc(allocator, sizeof(*gravity));
    if (!gravity) return false;
    memset(gravity, 0, sizeof(*gravity));
    gravity->allocator = *allocator;
    const ivp_vec3f_t rot_speed_before = core->rot_speed;
    const ivp_vec3r_t speed_before = core->speed;
    ivp_sim_ctrl_t *std_gravity = NULL;

    /* InitRaycastCarBody */
    {
        int template_wheels = tcs->n_wheels;
        if (template_wheels < 0) template_wheels = 0;
        if (template_wheels > IVP_CAR_SYSTEM_MAX_WHEELS) template_wheels = IVP_CAR_SYSTEM_MAX_WHEELS;
        if (template_wheels > IVP_AIRBOAT_MAX_WHEELS) template_wheels = IVP_AIRBOAT_MAX_WHEELS;
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
    state->thrust = 0.0f;
    for (int k = 0; k < 3; k++) {
        core->rot_speed.k[k] = 0.0f;
        core->speed.k[k] = IVP_REAL(0.0);
    }

    /* InitRaycastCarEnvironment */
    state->index_x = tcs->index_x;
    state->index_y = tcs->index_y;
    state->index_z = tcs->index_z;
    state->is_left_handed = tcs->is_left_handed;
    {
        /* remove the old gravity controller and add a different one */
        std_gravity = ivp_env_get_standard_gravity_controller(env);
        if (std_gravity) ivp_core_rem_core_controller(core, std_gravity);
        gravity->base.vt = &ag_vt;
        lvf_set3(&gravity->grav_vec, 0.0f, IVP_AIRBOAT_GRAVITY, 0.0f);
        if (!ivp_core_add_core_controller(core, &gravity->base)) {
            if (std_gravity) (void)ivp_core_add_core_controller(core, std_gravity);
            core->rot_speed = rot_speed_before;
            core->speed = speed_before;
            ivp_allocator_t gravity_allocator = gravity->allocator;
            ivp_free(&gravity_allocator, gravity, sizeof(*gravity));
            return false;
        }
        state->gravity_controller = gravity;
    }
    state->active = true;
    state->handle = ivp_environment_announce_dependent_controller(env, IVP_CP_CONSTRAINTS_MAX,
                                                                  ivp_controller_airboat, state,
                                                                  ivp_airboat_discovery);
    if (!ivp_controller_handle_is_valid(state->handle)) {
        ivp_core_rem_core_controller(core, &gravity->base);
        if (std_gravity) (void)ivp_core_add_core_controller(core, std_gravity);
        state->gravity_controller = NULL;
        core->rot_speed = rot_speed_before;
        core->speed = speed_before;
        state->active = false;
        ivp_allocator_t gravity_allocator = gravity->allocator;
        ivp_free(&gravity_allocator, gravity, sizeof(*gravity));
        return false;
    }
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
            ivp_airboat_wheel_t *w = &state->wheels[i];
            memset(w, 0, sizeof(*w));
            lvm_vmult4_ff(&m_core_f_object, &tcs->wheel_pos_Bos[i], &w->hp_cs);
            lvm_vmult4_ff(&m_core_f_object, &tcs->trace_pos_Bos[i], &w->raycast_start_cs);
            w->raycast_length = IVP_AIRBOAT_RAYCAST_DIST;
            lvf_zero(&w->raycast_dir_cs);
            w->raycast_dir_cs.k[state->index_y] = state->gravity_y_direction;
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
            ivp_airboat_do_steering_wheel(state, i, 0.0f);
            w->wheel_is_fixed = false;
            w->max_rotation_speed = (float)tcs->wheel_max_rotation_speed[i >> 1];
            w->wheel_is_fixed = true;
        }
    }

    /* InitRaycastCarAxes */
    state->steering_angle = -1.0f; /* make sure the next call is not optimized away */
    ivp_airboat_do_steering(state, 0.0f);
    for (int a = 0; a < state->n_axis && a < IVP_AIRBOAT_MAX_WHEELS / 2; a++) {
        state->stabilizer_constant[a] = (float)tcs->stabilizer_constant[a];
    }
    return true;
}

void ivp_airboat_destroy(ivp_airboat_state_t *state)
{
    if (!state) return;
    ivp_environment_t *env = state->environment;
    if (env && ivp_controller_handle_is_valid(state->handle)) {
        (void)ivp_environment_remove_dependent_controller(env, state->handle, true);
    }
    state->handle = ivp_controller_handle_invalid();
    airboat_gravity_t *gravity = (airboat_gravity_t *)state->gravity_controller;
    if (gravity) {
        ivp_object_t *body = state->body;
        if (env && body && body->owner == env &&
            ivp_ctrl_mgr_core_has_controller(body->physical_core, &gravity->base)) {
            ivp_core_rem_core_controller(body->physical_core, &gravity->base);
            ivp_sim_ctrl_t *std_gravity = ivp_env_get_standard_gravity_controller(env);
            if (std_gravity) ivp_core_add_core_controller(body->physical_core, std_gravity);
        }
        ivp_allocator_t gravity_allocator = gravity->allocator;
        ivp_free(&gravity_allocator, gravity, sizeof(*gravity));
        state->gravity_controller = NULL;
    }
    state->active = false;
}

/* ── setters / getters ─────────────────────────────────────────────────── */

void ivp_airboat_update_throttle(ivp_airboat_state_t *state, float throttle)
{
    if (!state) return;
    if (fabs((double)throttle) < (double)0.01f) {
        state->thrust = 0.0f;
    } else if (throttle > 0.0f) {
        state->thrust = IVP_AIRBOAT_THRUST_MAX * throttle;
    } else if (throttle < 0.0f) {
        state->thrust = IVP_AIRBOAT_THRUST_MAX_REVERSE * throttle;
    }
}

void ivp_airboat_change_spring_constant(ivp_airboat_state_t *state, int wheel, float spring_constant)
{
    if (airboat_valid_wheel(state, wheel)) state->wheels[wheel].spring_constant = spring_constant;
}

void ivp_airboat_change_spring_dampening(ivp_airboat_state_t *state, int wheel, float spring_dampening)
{
    if (airboat_valid_wheel(state, wheel)) state->wheels[wheel].spring_damp_relax = spring_dampening;
}

void ivp_airboat_change_spring_dampening_compression(ivp_airboat_state_t *state, int wheel, float spring_dampening)
{
    if (airboat_valid_wheel(state, wheel)) state->wheels[wheel].spring_damp_compress = spring_dampening;
}

void ivp_airboat_change_spring_pre_tension(ivp_airboat_state_t *state, int wheel, float pre_tension_length)
{
    if (airboat_valid_wheel(state, wheel)) state->wheels[wheel].spring_len = -pre_tension_length;
}

void ivp_airboat_change_spring_length(ivp_airboat_state_t *state, int wheel, float spring_length)
{
    if (airboat_valid_wheel(state, wheel)) state->wheels[wheel].spring_len = spring_length;
}

void ivp_airboat_change_wheel_torque(ivp_airboat_state_t *state, int wheel, float torque)
{
    if (!airboat_valid_wheel(state, wheel)) return;
    state->wheels[wheel].torque = torque;
    airboat_ensure_in_simulation(state); /* wake the physics object */
}

void ivp_airboat_fix_wheel(ivp_airboat_state_t *state, int wheel, bool stop_wheel)
{
    if (airboat_valid_wheel(state, wheel)) state->wheels[wheel].wheel_is_fixed = stop_wheel;
}

void ivp_airboat_change_friction_of_wheel(ivp_airboat_state_t *state, int wheel, float friction)
{
    if (airboat_valid_wheel(state, wheel)) state->wheels[wheel].friction_of_wheel = friction;
}

void ivp_airboat_change_stabilizer_constant(ivp_airboat_state_t *state, int axis, float stabi_constant)
{
    if (state && axis >= 0 && axis < IVP_AIRBOAT_MAX_WHEELS / 2) state->stabilizer_constant[axis] = stabi_constant;
}

void ivp_airboat_change_body_downforce(ivp_airboat_state_t *state, float force)
{
    if (state) state->down_force = force;
}

void ivp_airboat_set_booster_acceleration(ivp_airboat_state_t *state, float acceleration)
{
    if (state) state->booster_force = acceleration;
}

void ivp_airboat_activate_booster(ivp_airboat_state_t *state, float thrust, float duration, float delay)
{
    if (!state) return;
    if (state->booster_force != 0.0f) return;
    if (state->booster_seconds_until_ready > 0.0f) return; /* booster not ready yet */
    state->booster_force = thrust;
    state->booster_seconds_to_go = duration;
    state->booster_seconds_until_ready = duration + delay;
}

float ivp_airboat_get_booster_delay(const ivp_airboat_state_t *state)
{
    return state ? state->booster_seconds_until_ready : 0.0f;
}

double ivp_airboat_get_body_speed(const ivp_airboat_state_t *state, int index)
{
    if (!state || !state->body) return 0.0;
    const ivp_core_t *core = state->body->physical_core;
    ivp_vec3r_t orientation;
    lvm_get_col(&core->m_world_f_core_last_psi, index, &orientation);
    ivp_vec3f_t sp = ivp_core_speed_f(core);
    return lvp_dot_f(&orientation, &sp);
}

double ivp_airboat_get_wheel_angular_velocity(const ivp_airboat_state_t *state, int wheel)
{
    return airboat_valid_wheel(state, wheel) ? (double)state->wheels[wheel].wheel_angular_velocity : 0.0;
}

double ivp_airboat_get_orig_front_wheel_distance(const ivp_airboat_state_t *state)
{
    if (!state || state->n_wheels < 2) return 0.0;
    const double dist = (double)(state->wheels[IVP_WHEEL_FRONT_LEFT].hp_cs.k[state->index_x] -
                                 state->wheels[IVP_WHEEL_FRONT_RIGHT].hp_cs.k[state->index_x]);
    return fabs(dist);
}

double ivp_airboat_get_orig_axles_distance(const ivp_airboat_state_t *state)
{
    if (!state || state->n_wheels < 3) return 0.0;
    const double dist = (double)(state->wheels[IVP_WHEEL_FRONT_LEFT].hp_cs.k[state->index_z] -
                                 state->wheels[IVP_WHEEL_REAR_LEFT].hp_cs.k[state->index_z]);
    return fabs(dist);
}

int ivp_airboat_discovery(void *ctx, ivp_environment_t *env,
                          ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_airboat_state_t *s = (const ivp_airboat_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[1] = { s->body };
    return ivp_controller_discover_objects(objs, 1, out_handles, max_handles);
}

/* ── simulation (ivp_controller_airboat.cpp:22-366) ───────────────────── */

/* IVP_Raycast_Airboat_Pontoon_Temp */
typedef struct airboat_pontoon_temp {
    float       friction_value;
    ivp_vec3f_t ground_normal_ws;
    ivp_vec3r_t ground_hit_ws;
    ivp_vec3f_t raycast_dir_ws;
    ivp_vec3f_t surface_speed_wheel_ws;
    ivp_vec3f_t projected_surface_speed_wheel_ws;
    ivp_vec3f_t axis_direction_ws;
    ivp_vec3f_t projected_axis_direction_ws;
    float       inv_normal_dot_dir;
} airboat_pontoon_temp_t;

static void airboat_pre_raycasts(ivp_airboat_state_t *s, ivp_ray_solver_template_t *rays,
                                 const ivp_transform_r_t *m, airboat_pontoon_temp_t *t)
{
    for (int i = 0; i < s->n_wheels; i++) {
        ivp_airboat_wheel_t *w = &s->wheels[i];
        ivp_ray_solver_template_t *r = &rays[i];
        lvm_vmult4_fp(m, &w->raycast_start_cs, &r->ray_start_point);
        lvm_vmult3_f(m, &w->raycast_dir_cs, &t[i].raycast_dir_ws);
        lvp_set_f(&r->ray_normized_direction, &t[i].raycast_dir_ws);
        r->ray_length = (ivp_real_t)IVP_AIRBOAT_RAYCAST_DIST;
        r->ray_flags = IVP_RAY_SOLVER_ALL;
    }
}

static bool airboat_post_raycasts(ivp_airboat_state_t *s, const ivp_ray_solver_template_t *rays,
                                  const ivp_transform_r_t *m, airboat_pontoon_temp_t *t,
                                  const ivp_airboat_impact_t *impacts, ivp_core_t *core)
{
    for (int i = 0; i < s->n_wheels; i++) {
        ivp_airboat_wheel_t *w = &s->wheels[i];
        airboat_pontoon_temp_t *tp = &t[i];
        const ivp_airboat_impact_t *imp = &impacts[i];
        const ivp_ray_solver_template_t *r = &rays[i];

        /* copy the ray length back, it may have changed */
        w->raycast_length = (float)r->ray_length;

        /* test for inverted raycast direction */
        if (imp->in_water) lvf_set_multiple(&tp->raycast_dir_ws, &tp->raycast_dir_ws, (D)-1);

        if (imp->impact) {
            tp->ground_normal_ws = imp->impact_normal_ws;
            ivp_vec3r_t delta;
            lvp_subtract2(&delta, &imp->impact_point_ws, &r->ray_start_point);
            w->raycast_dist = (float)lvp_real_length(&delta);
            tp->inv_normal_dot_dir = (float)((double)1.1f /
                (fabs(lvf_dot(&tp->raycast_dir_ws, &tp->ground_normal_ws)) + (double)0.1f));
            tp->friction_value = imp->friction * w->friction_of_wheel;
        } else {
            w->raycast_dist = w->raycast_length;
            tp->inv_normal_dot_dir = 1.0f;
            lvf_set_multiple(&tp->ground_normal_ws, &tp->raycast_dir_ws, (D)-1);
            tp->friction_value = 1.0f;
        }

        /* the impact point or the full ray distance */
        lvp_add_multiple2_pf(&tp->ground_hit_ws, &r->ray_start_point, &tp->raycast_dir_ws, (D)w->raycast_dist);

        /* speed at the impact point */
        {
            ivp_vec3r_t sp;
            ivp_core_get_surface_speed_ws(core, &tp->ground_hit_ws, &sp);
            lvf_set_p(&tp->surface_speed_wheel_ws, &sp);
        }
        lvf_set_orthogonal_part(&tp->projected_surface_speed_wheel_ws, &tp->surface_speed_wheel_ws,
                                &tp->ground_normal_ws);

        lvm_vmult3_f(m, &w->axis_direction_cs, &tp->axis_direction_ws);
        lvf_set_orthogonal_part(&tp->projected_axis_direction_ws, &tp->axis_direction_ws, &tp->ground_normal_ws);
        if (!lvf_normize(&tp->projected_axis_direction_ws)) {
            ivp_debug_printf("IVP_Controller_Raycast_Airboat::do_simulation_controller projected_axis_direction_ws.normize failed\n");
            return false;
        }
    }
    return true;
}

/* DoSimulationPontoonsGround */
static void airboat_pontoon_ground(const ivp_airboat_wheel_t *w, const airboat_pontoon_temp_t *tp,
                                   const ivp_event_sim_t *es, ivp_core_t *core)
{
    /* check to see if we hit anything, otherwise no force on this point */
    const double flDiff = (double)(w->raycast_dist - w->raycast_length);
    if (flDiff >= 0) return;

    const float flSpringConstant = w->spring_constant;
    const float flSpringRelax = w->spring_damp_relax;
    const float flSpringCompress = w->spring_damp_compress;

    double flForce = -flDiff * (double)flSpringConstant;
    float flInvNormalDotDir = tp->inv_normal_dot_dir;
    if (flInvNormalDotDir < 0.0f) flInvNormalDotDir = 0.0f;
    if (flInvNormalDotDir > 3.0f) flInvNormalDotDir = 3.0f;
    flForce *= (double)flInvNormalDotDir;

    ivp_vec3f_t vecSpeedDelta;
    lvf_subtract2(&vecSpeedDelta, &tp->projected_surface_speed_wheel_ws, &tp->surface_speed_wheel_ws);
    const double flSpeed = lvf_dot(&vecSpeedDelta, &tp->raycast_dir_ws);
    if (flSpeed > 0) {
        flForce -= (double)flSpringRelax * flSpeed;
    } else {
        flForce -= (double)flSpringCompress * flSpeed;
    }
    if (flForce < 0) flForce = 0.0f;

    const double flImpulse = flForce * (double)es->delta_time;
    ivp_vec3f_t vecImpulseWS;
    lvf_set_multiple(&vecImpulseWS, &tp->ground_normal_ws, (D)flImpulse);
    ivp_ctl_push_core_ws(core, &tp->ground_hit_ws, &vecImpulseWS);
}

/* DoSimulationPontoonsWater */
static void airboat_pontoon_water(const ivp_airboat_state_t *s, const airboat_pontoon_temp_t *tp,
                                  const ivp_event_sim_t *es, ivp_core_t *core, bool bFront)
{
    ivp_vec3f_t vecForwardWS;
    airboat_col_f(&core->m_world_f_core_last_psi, s->index_z, &vecForwardWS);

    float flVolumeScalar = 1.0f;
    if (vecForwardWS.k[s->index_y] > 0.0f && !bFront) {
        flVolumeScalar += -vecForwardWS.k[s->index_y];
    } else if (flVolumeScalar > 0.0f && bFront) {
        flVolumeScalar += vecForwardWS.k[s->index_y];
    }

    const float flForce = IVP_AIRBOAT_BUOYANCY_SCALAR * 0.25f * ivp_core_get_mass_f(core) * IVP_AIRBOAT_GRAVITY *
                          flVolumeScalar;
    const double flImpulse = (double)flForce * (double)es->delta_time;
    ivp_vec3f_t vecImpulseWS;
    lvf_set_multiple(&vecImpulseWS, &tp->raycast_dir_ws, (D)flImpulse);
    ivp_ctl_push_core_ws(core, &tp->ground_hit_ws, &vecImpulseWS);
}

/* DoSimulationDrag */
static void airboat_drag(const ivp_event_sim_t *es, ivp_core_t *core)
{
    ivp_vec3f_t speed = ivp_core_speed_f(core);
    ivp_vec3f_t vecAirboatDir = speed;
    (void)lvf_normize(&vecAirboatDir);

    const float flSpeed = (float)lvf_real_length(&speed);
    const float flSpeed2 = flSpeed * flSpeed;

    const float flDragScale = 0.5f * IVP_AIRBOAT_DRAG_COEFFICIENT * IVP_AIRBOAT_WATER_DENSITY *
                              IVP_AIRBOAT_DRAG_SURFACE_AREA;
    const float flDrag = flSpeed2 * flDragScale;

    ivp_vec3f_t vecAirboatNegDir;
    lvf_set_negative(&vecAirboatNegDir, &vecAirboatDir);
    ivp_vec3f_t vecImpulse;
    lvf_set_multiple(&vecImpulse, &vecAirboatNegDir, (D)((double)flDrag * (double)es->delta_time));
    /* center_push_core_multiple_ws(&impulse, 1.0) */
    ivp_vec3r_t imp;
    lvp_set_f(&imp, &vecImpulse);
    ivp_core_center_push(core, &imp);
}

/* DoSimulationTurbine */
static void airboat_turbine(const ivp_airboat_state_t *s, const ivp_event_sim_t *es, ivp_core_t *core)
{
    ivp_vec3f_t vecForwardWS;
    airboat_col_f(&core->m_world_f_core_last_psi, s->index_z, &vecForwardWS);
    ivp_vec3f_t vecImpulse;
    lvf_set_multiple(&vecImpulse, &vecForwardWS, (D)((double)s->thrust * (double)es->delta_time));
    ivp_vec3r_t imp;
    lvp_set_f(&imp, &vecImpulse);
    ivp_core_center_push(core, &imp);
}

/* DoSimulationSteering */
static void airboat_steering(const ivp_airboat_state_t *s, ivp_core_t *core, const ivp_event_sim_t *es)
{
    const ivp_transform_r_t *m = &core->m_world_f_core_last_psi;
    ivp_vec3f_t vecRightWS;
    airboat_col_f(m, s->index_x, &vecRightWS);

    const double i_dt = (double)es->inverse_delta_time;
    const float mass = ivp_core_get_mass_f(core);
    float flForceRotational = 0.0f;

    /* check for rotation */
    if ((double)fabsf(s->steering_angle) > 0.01) {
        const float flSteeringSign = s->steering_angle < 0.0f ? -1.0f : 1.0f;
        flForceRotational = (float)((double)(IVP_AIRBOAT_STEERING_RATE * mass) * i_dt);
        flForceRotational *= flSteeringSign;
    }

    /* general force that helps get back to zero rotation */
    const float flRotSpeedSign = core->rot_speed.k[s->index_y] < 0.0f ? -1.0f : 1.0f;
    float flRotationalDrag = (float)((double)(-0.5f * IVP_AIRBOAT_STEERING_RATE * mass) * i_dt);
    flRotationalDrag *= flRotSpeedSign;
    flForceRotational += flRotationalDrag;

    for (int i = 0; i < s->n_wheels; i++) {
        const ivp_airboat_wheel_t *w = &s->wheels[i];
        const float flPontoonSign = i >= (s->n_wheels / 2) ? -1.0f : 1.0f;
        const float flForceRot = flForceRotational * flPontoonSign;
        ivp_vec3f_t vecImpulse;
        lvf_set_multiple(&vecImpulse, &vecRightWS, (D)((double)flForceRot * (double)es->delta_time));
        ivp_vec3r_t vecPointPositionWS;
        lvm_vmult4_fp(m, &w->raycast_start_cs, &vecPointPositionWS);
        ivp_ctl_push_core_ws(core, &vecPointPositionWS, &vecImpulse);
    }
}

void ivp_controller_airboat(const ivp_event_sim_t *event_sim,
                            ivp_object_t *obj,
                            void *ctx)
{
    ivp_airboat_state_t *s = (ivp_airboat_state_t *)ctx;
    if (!s || !s->active || !s->body || obj != s->body) return;
    ivp_core_t *core = s->body->physical_core;
    if (core->physical_unmoveable) return;

    airboat_pontoon_temp_t temp[IVP_AIRBOAT_MAX_WHEELS];
    ivp_ray_solver_template_t rays[IVP_AIRBOAT_MAX_WHEELS];
    ivp_airboat_impact_t impacts[IVP_AIRBOAT_MAX_WHEELS];
    memset(temp, 0, sizeof(temp));
    memset(rays, 0, sizeof(rays));
    memset(impacts, 0, sizeof(impacts));

    const ivp_transform_r_t *m = &core->m_world_f_core_last_psi;

    /* raycasts */
    airboat_pre_raycasts(s, rays, m, temp);
    if (s->raycast_fn) s->raycast_fn(s->raycast_ctx, s->n_wheels, rays, impacts);
    if (!airboat_post_raycasts(s, rays, m, temp, impacts, core)) return;

    /* pontoons */
    for (int i = 0; i < s->n_wheels; i++) {
        if (impacts[i].impact) {
            airboat_pontoon_ground(&s->wheels[i], &temp[i], event_sim, core);
        } else if (impacts[i].in_water) {
            airboat_pontoon_water(s, &temp[i], event_sim, core, i < 2);
        }
    }

    /* drag (water), turbine (fan), steering */
    airboat_drag(event_sim, core);
    airboat_turbine(s, event_sim, core);
    airboat_steering(s, core, event_sim);
}
