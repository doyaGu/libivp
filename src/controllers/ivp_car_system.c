/* ivp_car_system.c -- port of ivp_controller/ivp_car_system.cxx
 *
 *   IVP_Template_Car_System, IVP_Car_System::calc_ackerman_angle and
 *   IVP_Car_System_Real_Wheels.
 *
 * The constraint solver is ivp_constraint_car.c; suspension, torque, force
 * and stabilizer actuators are the libivp ports (ivp_actuator_*.c) created
 * in the reference order.  Anchors follow IVP_Template_Anchor /
 * IVP_Anchor::init_anchor: object coordinates -> world (cache object) ->
 * object coordinates (IVP_U_Float_Point) -> core coordinates.
 * Reference types and evaluation order: see ivp_constraint_car_internal.h.
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ivp/controllers/ivp_car_system.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include "controllers/ivp_constraint_car_internal.h"
#include "controllers/ivp_controller_ref.h"

/* ── template ───────────────────────────────────────────────────────────── */

void ivp_car_system_template_init(ivp_car_system_template_t *tmpl, int n_wheels, int n_axes)
{
    memset(tmpl, 0, sizeof(*tmpl));
    if (n_wheels < 0) n_wheels = 0;
    if (n_wheels > IVP_CAR_SYSTEM_MAX_WHEELS) n_wheels = IVP_CAR_SYSTEM_MAX_WHEELS;
    tmpl->n_wheels = n_wheels;
    if (n_axes < 1) n_axes = 1;
    if (n_axes > n_wheels) n_axes = (n_wheels > 0) ? n_wheels : 1;
    tmpl->n_axes = n_axes;

    tmpl->index_x = 0;
    tmpl->index_y = 1;
    tmpl->index_z = 2;
    tmpl->is_left_handed = false;
    for (int i = 0; i < n_wheels; i++) {
        tmpl->wheel_reversed_sign[i] = IVP_REAL(1.0);
        tmpl->wheel_axis_index[i] = (i & 2) ? 1 : 0;
    }
    tmpl->fast_turn_factor = IVP_REAL(1.0);
}

void ivp_car_system_template_default(ivp_car_system_template_t *tmpl)
{
    ivp_car_system_template_init(tmpl, 4, 2);
    for (int i = 0; i < IVP_CAR_SYSTEM_MAX_WHEELS; i++) {
        tmpl->wheel_radius[i] = IVP_REAL(0.3);
        tmpl->wheel_friction[i] = IVP_REAL(1.0);
        tmpl->wheel_reversed_sign[i] = IVP_REAL(1.0);
        tmpl->spring_constant[i] = IVP_REAL(50000.0);
        tmpl->spring_damping[i] = IVP_REAL(3000.0);
        tmpl->spring_damping_compression[i] = IVP_REAL(3000.0);
        tmpl->max_body_force[i] = IVP_REAL(1e6);
    }
    for (int i = 0; i < IVP_CAR_SYSTEM_MAX_AXES; i++) {
        tmpl->stabilizer_constant[i] = IVP_REAL(500.0);
        tmpl->wheel_max_rotation_speed[i] = IVP_REAL(100.0);
    }
    tmpl->wheel_pos_from_objects = true;
}

/* ── helpers ────────────────────────────────────────────────────────────── */

/* The car is usable while its constraint solver is announced: deleting the
 * body or a wheel detaches the solver (the reference deletes it together with
 * the actuators of that object, leaving dangling pointers in the car); from
 * then on every call but ivp_car_system_destroy is a no-op. */
static inline bool car_alive(const ivp_car_system_t *car)
{
    return car && car->active && car->car_constraint_solver &&
           ivp_constraint_solver_car_is_active(car->car_constraint_solver) && !car->environment_dying;
}

static inline bool car_wheel_ok(const ivp_car_system_t *car, int w)
{
    return car_alive(car) && w >= 0 && w < car->n_wheels;
}

static inline float car_f(ivp_real_t v) { return (float)v; }

/* An initialised IVP_Anchor: its object, core_pos and object_pos
 * (IVP_U_Float_Point) */
typedef struct car_anchor {
    ivp_object_t *obj;
    ivp_vec3f_t   core_pos;
    ivp_vec3f_t   object_pos;
} car_anchor_t;

/* IVP_Template_Anchor::set_anchor_position_os + IVP_Anchor::init_anchor
 * (shared with the actuator ports, ivp_controller_ref.h) */
static void car_anchor_os(ivp_object_t *obj, const ivp_vec3f_t *coords_os, car_anchor_t *a)
{
    a->obj = obj;
    ivp_ctl_anchor_core_pos_os(obj, coords_os, &a->core_pos, &a->object_pos);
}

/* IVP_Template_Anchor::set_anchor_position_cs */
static void car_anchor_cs(ivp_object_t *obj, const ivp_vec3f_t *coords_cs, car_anchor_t *a)
{
    a->obj = obj;
    ivp_ctl_anchor_core_pos_cs(obj, coords_cs, &a->core_pos, &a->object_pos);
}

/* an anchor of a libivp actuator state from an initialised anchor */
static ivp_vec3r_t car_state_anchor(const car_anchor_t *a, ivp_anchor_t *anchor)
{
    ivp_ctl_anchor_set(anchor, a->obj, &a->object_pos);
    ivp_vec3r_t r;
    lvp_set_f(&r, &a->core_pos);
    return r;
}

static inline ivp_vec3r_t car_vr(const ivp_vec3f_t *v)
{
    ivp_vec3r_t r;
    lvp_set_f(&r, v);
    return r;
}

/* IVP_Actuator_Torque: axis = normized(anchor1.core_pos - anchor0.core_pos) */
static void car_create_torque(ivp_car_system_t *car, ivp_torque_actuator_state_t *st, ivp_controller_handle_t *h,
                              ivp_object_t *obj, const car_anchor_t *anchor0, const car_anchor_t *anchor1,
                              float max_rotation_speed)
{
    ivp_vec3f_t axis;
    lvf_subtract2(&axis, &anchor1->core_pos, &anchor0->core_pos);
    (void)lvf_normize(&axis);
    ivp_vec3r_t axis_r = car_vr(&axis);
    ivp_torque_actuator_state_init(st);
    ivp_torque_actuator_state_setup(st, obj, &axis_r, IVP_REAL(0.0), (ivp_real_t)max_rotation_speed);
    st->axis_os = axis_r; /* already axis_in_core_coord_system */
    *h = ivp_environment_add_controller(car->environment, IVP_CP_ACTUATOR, ivp_controller_torque_actuator, st);
}

/* IVP_Actuator_Force with push_first_object / !push_second_object */
static void car_create_force(ivp_car_system_t *car, ivp_force_actuator_state_t *st, ivp_controller_handle_t *h,
                             ivp_object_t *obj0, const car_anchor_t *anchor0, ivp_object_t *obj1,
                             const car_anchor_t *anchor1, float force)
{
    ivp_vec3r_t a0 = car_vr(&anchor0->core_pos), a1 = car_vr(&anchor1->core_pos);
    ivp_force_actuator_state_init(st);
    ivp_force_actuator_state_setup(st, obj0, &a0, obj1, &a1, (ivp_real_t)force, true, false);
    /* the anchors are already resolved (IVP_Anchor::init_anchor) */
    st->anchor_a_os = car_state_anchor(anchor0, &st->anchors[0]);
    st->anchor_b_os = car_state_anchor(anchor1, &st->anchors[1]);
    *h = ivp_environment_add_controller(car->environment, IVP_CP_ACTUATOR, ivp_controller_force_actuator, st);
}

static void car_remove_controller_ex(ivp_car_system_t *car, ivp_controller_handle_t *h, bool silently)
{
    if (ivp_controller_handle_is_valid(*h) && car->environment && !car->environment_dying) {
        (void)ivp_environment_remove_dependent_controller(car->environment, *h, silently);
    }
    *h = ivp_controller_handle_invalid();
}

static void car_remove_controller(ivp_car_system_t *car, ivp_controller_handle_t *h)
{
    car_remove_controller_ex(car, h, false);
}

/* unregister every actuator / constraint of the car (states stay allocated) */
static void car_unregister_all(ivp_car_system_t *car, bool silently)
{
    car_remove_controller_ex(car, &car->h_booster[0], silently);
    car_remove_controller_ex(car, &car->h_booster[1], silently);
    car->booster_active = false;
    for (int i = 0; i < IVP_CAR_SYSTEM_MAX_WHEELS; i++) {
        car_remove_controller_ex(car, &car->h_fix_wheel[i], silently);
        car_remove_controller_ex(car, &car->h_spring[i], silently);
        car_remove_controller_ex(car, &car->h_torque[i], silently);
    }
    car_remove_controller_ex(car, &car->h_torque_body, silently);
    car_remove_controller_ex(car, &car->h_down_force, silently);
    car_remove_controller_ex(car, &car->h_extra_gravity, silently);
    car_remove_controller_ex(car, &car->h_powerslide_back, silently);
    car_remove_controller_ex(car, &car->h_powerslide_front, silently);
}

static void car_env_event(void *ctx, ivp_environment_t *env, const ivp_event_t *ev)
{
    (void)env;
    ivp_car_system_t *car = (ivp_car_system_t *)ctx;
    if (ev && ev->type == IVP_EVENT_ENVIRONMENT_DELETED) {
        /* leave the environment while it is still intact: the force
         * actuators reference the static object, which is deleted first */
        car_unregister_all(car, true);
        car->environment_dying = true;
        car->env_listener = ivp_event_listener_handle_invalid();
    }
}

static void car_reset_handles(ivp_car_system_t *car)
{
    car->h_torque_body = ivp_controller_handle_invalid();
    car->h_down_force = ivp_controller_handle_invalid();
    car->h_extra_gravity = ivp_controller_handle_invalid();
    car->h_powerslide_back = ivp_controller_handle_invalid();
    car->h_powerslide_front = ivp_controller_handle_invalid();
    car->h_booster[0] = car->h_booster[1] = ivp_controller_handle_invalid();
    for (int i = 0; i < IVP_CAR_SYSTEM_MAX_WHEELS; i++) {
        car->h_torque[i] = ivp_controller_handle_invalid();
        car->h_spring[i] = ivp_controller_handle_invalid();
        car->h_fix_wheel[i] = ivp_controller_handle_invalid();
    }
    car->env_listener = ivp_event_listener_handle_invalid();
}

/* ── IVP_Car_System_Real_Wheels ─────────────────────────────────────────── */

ivp_real_t ivp_car_system_calc_ackerman_angle(ivp_real_t alpha_in, ivp_real_t dx_in, ivp_real_t dz_in)
{
    /* steering angle for the front wheel towards the curve center (alpha) ->
     * Ackermann angle of the outer front wheel; dx = distance of the front
     * wheels, dz = distance of the front and rear axles */
    float alpha = (float)alpha_in, dx = (float)dx_in, dz = (float)dz_in;
    float a = (float)ivp_fabs((D)alpha);
    if (a < 0.001f) return (ivp_real_t)alpha; /* numerical reasons */
    D tan_alpha = (D)tanf(a);
    D h = ((D)dz * tan_alpha) / ((D)dx * tan_alpha + (D)dz);
#ifdef IVP_REAL_DOUBLE
    D beta = atan(h);
#else
    D beta = atanf(h);
#endif
    D signum = (alpha < 0.0f) ? -1.0f : 1.0f;
    return (ivp_real_t)(float)(beta * signum);
}

/* get_body_speed(idx_z) */
ivp_real_t ivp_car_system_get_body_speed_axis(const ivp_car_system_t *car, int idx_z)
{
    if (!car_alive(car) || idx_z < 0 || idx_z > 2) return IVP_REAL(0.0);
    const ivp_core_t *core = car->body->physical_core;
    ivp_vec3f_t vec_ws = ivp_core_speed_f(core);
    ivp_vec3r_t orientation;
    lvm_get_col(&core->m_world_f_core_last_psi, idx_z, &orientation);
    return (ivp_real_t)(float)lvp_dot_f(&orientation, &vec_ws);
}

ivp_real_t ivp_car_system_get_body_speed(const ivp_car_system_t *car)
{
    if (!car_alive(car)) return IVP_REAL(0.0);
    return ivp_car_system_get_body_speed_axis(car, car->car_constraint_solver->z_idx);
}

ivp_real_t ivp_car_system_get_orig_front_wheel_distance(const ivp_car_system_t *car)
{
    if (!car_alive(car) || car->n_wheels < 2) return IVP_REAL(0.0);
    const ivp_constraint_solver_car_t *cs = car->car_constraint_solver;
    const ivp_transform_r_t *m_left_wheel_bs = &cs->wheel_objects[0]->target_position_bs;
    const ivp_transform_r_t *m_right_wheel_bs = &cs->wheel_objects[1]->target_position_bs;
    D dist = m_left_wheel_bs->pos.k[cs->x_idx] - m_right_wheel_bs->pos.k[cs->x_idx];
    return (ivp_real_t)-dist;
}

ivp_real_t ivp_car_system_get_orig_axles_distance(const ivp_car_system_t *car)
{
    if (!car_alive(car) || car->n_wheels < 3) return IVP_REAL(0.0);
    const ivp_constraint_solver_car_t *cs = car->car_constraint_solver;
    const ivp_transform_r_t *m_front_wheel_bs = &cs->wheel_objects[0]->target_position_bs;
    const ivp_transform_r_t *m_rear_wheel_bs = &cs->wheel_objects[2]->target_position_bs;
    D dist = m_front_wheel_bs->pos.k[cs->z_idx] - m_rear_wheel_bs->pos.k[cs->z_idx];
    return (ivp_real_t)ivp_fabs(dist);
}

ivp_real_t ivp_car_system_get_wheel_angular_velocity(const ivp_car_system_t *car, int wheel_index)
{
    if (!car_wheel_ok(car, wheel_index)) return IVP_REAL(0.0);
    return car->car_act_torque[wheel_index].current_speed;
}

/* do_steering_wheel */
void ivp_car_system_steer_wheel(ivp_car_system_t *car, int wheel_nr, ivp_real_t s_angle_in)
{
    if (!car_wheel_ok(car, wheel_nr) || !car->car_constraint_solver->wheel_objects[wheel_nr]) return;
    float s_angle = (float)s_angle_in;
    ivp_constraint_solver_car_t *cs_car = car->car_constraint_solver;

    ivp_transform_r_t *target_mat = &cs_car->wheel_objects[wheel_nr]->target_position_bs;
    ivp_vec3r_t old_translation = target_mat->pos; /* remember */

    ivp_vec3r_t hp;
    hp.k[cs_car->x_idx] = 0.0f;
    hp.k[cs_car->y_idx] = (D)s_angle;
    hp.k[cs_car->z_idx] = 0.0f;
    if (car->wheel_reversed_sign[wheel_nr] < 0.0f) hp.k[cs_car->y_idx] += (D)IVP_PI;

    /* IVP_U_Matrix::init_rot_multiple(&hp, 1.0f) */
    const D factor = 1.0f;
    D x, y, u, w, c, s;
    if (hp.k[0] != 0) {
        x = ivp_cos(hp.k[0] * factor);
        y = ivp_sin(hp.k[0] * factor);
    } else {
        x = 1.0f;
        y = 0.0f;
    }
    if (hp.k[1] != 0) {
        u = ivp_cos(hp.k[1] * factor);
        w = ivp_sin(hp.k[1] * factor);
    } else {
        u = 1.0f;
        w = 0.0f;
    }
    if (hp.k[2] != 0) {
        c = ivp_cos(hp.k[2] * factor);
        s = ivp_sin(hp.k[2] * factor);
    } else {
        c = 1.0f;
        s = 0.0f;
    }
    LVM_E(target_mat, 0, 0) = u * c + w * y * s;
    LVM_E(target_mat, 0, 1) = w * y * c - u * s;
    LVM_E(target_mat, 0, 2) = w * x;
    LVM_E(target_mat, 1, 0) = x * s;
    LVM_E(target_mat, 1, 1) = x * c;
    LVM_E(target_mat, 1, 2) = -y;
    LVM_E(target_mat, 2, 0) = y * u * s - w * c;
    LVM_E(target_mat, 2, 1) = u * y * c + w * s;
    LVM_E(target_mat, 2, 2) = u * x;
    target_mat->pos = old_translation;
}

/* do_steering */
void ivp_car_system_set_steering(ivp_car_system_t *car, ivp_real_t s_angle_in)
{
    if (!car_alive(car)) return;
    float s_angle = (float)s_angle_in;
    /* tell constraint system new steering positions of wheels */
    if (car->steering_angle == s_angle) return;

    ivp_constraint_solver_car_t *cs_car = car->car_constraint_solver;

    /* start to spin the object */
    D d_alpha = (D)(s_angle - car->steering_angle);

    float dx_front_wheels = 1.0f;
    float dz_axles = 1.0f;
    if (car->n_wheels >= 4) {
        dx_front_wheels = (float)ivp_car_system_get_orig_front_wheel_distance(car);
        dz_axles = (float)ivp_car_system_get_orig_axles_distance(car);
    }

    D angular_spin = d_alpha * (D)ivp_car_system_get_body_speed_axis(car, cs_car->z_idx) / (D)dz_axles;
    ivp_core_t *body_core = car->body->physical_core;
    {
        /* rot_speed_change is an IVP_U_Float_Point */
        D rsc = (D)(float)body_core->rot_speed_change.k[cs_car->y_idx];
        body_core->rot_speed_change.k[cs_car->y_idx] =
            (ivp_real_t)(float)(rsc - angular_spin * (D)car->fast_turn_factor);
    }

    car->steering_angle = s_angle;

    ivp_constraint_solver_car_ensure_in_simulation(cs_car);

    int wheels_per_axis = (car->n_axes > 0) ? car->n_wheels / car->n_axes : car->n_wheels;
    for (int i = 0; i < wheels_per_axis; i++) {
        /* turn front wheels */
        if ((i & 1) == (s_angle > 0.0f)) {
            ivp_car_system_steer_wheel(car, i, (ivp_real_t)s_angle);
        } else {
            float s_angle_2;
            if (car->n_wheels >= 4) {
                s_angle_2 = (float)ivp_car_system_calc_ackerman_angle((ivp_real_t)car->steering_angle,
                                                                      (ivp_real_t)dx_front_wheels,
                                                                      (ivp_real_t)dz_axles);
            } else {
                s_angle_2 = car->steering_angle;
            }
            ivp_car_system_steer_wheel(car, i, (ivp_real_t)s_angle_2);
        }
    }

    /* hack for middle wheels */
    if (car->n_wheels > 4) {
        ivp_car_system_steer_wheel(car, 4, (ivp_real_t)(s_angle * .5f));
        ivp_car_system_steer_wheel(car, 5, (ivp_real_t)(s_angle * .5f));
    }
}

bool ivp_car_system_init(ivp_car_system_t *car, const ivp_car_system_template_t *templ)
{
    if (!car) return false;
    memset(car, 0, sizeof(*car));
    car_reset_handles(car);
    if (!templ || !templ->car_body || !templ->car_body->owner) return false;
    if (templ->n_wheels < 1 || templ->n_wheels > IVP_CAR_SYSTEM_MAX_WHEELS) return false;
    if (templ->index_x < 0 || templ->index_x > 2 || templ->index_y < 0 || templ->index_y > 2 ||
        templ->index_z < 0 || templ->index_z > 2 || templ->index_x == templ->index_y ||
        templ->index_x == templ->index_z || templ->index_y == templ->index_z) {
        return false;
    }

    ivp_environment_t *env = templ->car_body->owner;

    /* builds up the car system, using values from the template */
    car->n_wheels = templ->n_wheels;
    car->n_axes = templ->n_axes;

    car->max_speed = 0;
    car->booster_active = false;
    car->booster_seconds_to_go = 0;
    car->booster_seconds_until_ready = 0;
    car->fast_turn_factor = car_f(templ->fast_turn_factor);
    car->environment = env;

    /*** objects ***/
    car->body = templ->car_body;
    for (int i = 0; i < car->n_wheels; i++) {
        car->car_wheel[i] = templ->wheel_objects[i];
        car->wheel_reversed_sign[i] = car_f(templ->wheel_reversed_sign[i]);
        car->wheel_radius[i] = car_f(templ->wheel_radius[i]); /* reference never copies it */
        if (!car->car_wheel[i] || car->car_wheel[i]->owner != env) {
            car->n_wheels = 0;
            return false;
        }
    }

    /*** constraint system ***/
    car->car_constraint_solver =
        ivp_constraint_solver_car_create(templ->index_x, templ->index_y, templ->index_z, templ->is_left_handed);
    if (!car->car_constraint_solver) {
        car->n_wheels = 0;
        return false;
    }

    ivp_vec3f_t hard_points[IVP_CAR_SYSTEM_MAX_WHEELS];
    for (int i = 0; i < car->n_wheels; i++) {
        if (templ->wheel_pos_from_objects) {
            /* libivp: current (AT) wheel center in the body object system */
            ivp_vec3r_t wheel_center_ws = ivp_object_get_cache_object_no_lock(car->car_wheel[i])->core_pos;
            ivp_cache_obj_t *co = ivp_object_get_cache_object_no_lock(car->body);
            lvm_vimult4_pf(&co->m_world_f_object, &wheel_center_ws, &hard_points[i]);
        } else {
            hard_points[i] = templ->wheel_pos_Bos[i];
        }
    }

    ivp_result_t init_res = ivp_constraint_solver_car_init_constraint_system(
        car->car_constraint_solver, env, car->body, car->car_wheel, hard_points, car->n_wheels);
    if (init_res != IVP_OK) {
        ivp_message("IVP_Car_System_Real_Wheels: failed to initialize constraint system\n");
        /* reference IVP_ASSERT(0); libivp's own argument checks (unmoveable or
         * foreign wheels, solver already in use: IVP_ERR_INVALID_ARG /
         * IVP_ERR_INVALID_STATE) have no reference counterpart, the reference
         * fails only for wheel-count and matrix-inversion errors */
        if (init_res != IVP_ERR_INVALID_ARG && init_res != IVP_ERR_INVALID_STATE) IVP_ASSERT(0);
        ivp_constraint_solver_car_destroy(car->car_constraint_solver);
        car->car_constraint_solver = NULL;
        car->n_wheels = 0;
        car->n_axes = 1;
        return false;
    }
    ivp_constraint_solver_car_t *cs_car = car->car_constraint_solver;
    car->env_listener = ivp_environment_add_event_listener(env, car_env_event, car);
    if (!ivp_event_listener_handle_is_valid(car->env_listener)) {
        ivp_constraint_solver_car_destroy(car->car_constraint_solver);
        car->car_constraint_solver = NULL;
        car->n_wheels = 0;
        car->n_axes = 1;
        return false;
    }
    car->active = true;

    /*** springs / shock absorbers, rotation motors ***/
    car_anchor_t anchor_body[IVP_CAR_SYSTEM_MAX_WHEELS];
    car_anchor_t anchor_wheel[IVP_CAR_SYSTEM_MAX_WHEELS];
    for (int wheel_nr = 0; wheel_nr < car->n_wheels; wheel_nr++) {
        float spring_len = 500.0f; /* be careful with car sizes around 500m ;-) */

        ivp_vec3f_t hp = hard_points[wheel_nr];
        hp.k[cs_car->y_idx] -= spring_len;
        car_anchor_os(car->body, &hp, &anchor_body[wheel_nr]);
        ivp_vec3f_t zero;
        lvf_zero(&zero);
        car_anchor_os(car->car_wheel[wheel_nr], &zero, &anchor_wheel[wheel_nr]);

        spring_len -= car_f(templ->spring_pre_tension[wheel_nr]);

        ivp_suspension_state_t *sp = &car->car_spring[wheel_nr];
        ivp_vec3r_t a0 = car_vr(&anchor_body[wheel_nr].core_pos), a1 = car_vr(&anchor_wheel[wheel_nr].core_pos);
        ivp_suspension_state_init(sp);
        ivp_suspension_state_setup(sp, car->body, car->car_wheel[wheel_nr], &a0, &a1,
                                   (ivp_real_t)car_f(templ->spring_constant[wheel_nr]),
                                   (ivp_real_t)car_f(templ->spring_damping[wheel_nr]),
                                   (ivp_real_t)car_f(templ->spring_damping_compression[wheel_nr]),
                                   (ivp_real_t)spring_len);
        /* the anchors are already resolved (IVP_Anchor::init_anchor) */
        sp->anchor_a_os = car_state_anchor(&anchor_body[wheel_nr], &sp->anchors[0]);
        sp->anchor_b_os = car_state_anchor(&anchor_wheel[wheel_nr], &sp->anchors[1]);
        sp->max_body_force = (ivp_real_t)car_f(templ->max_body_force[wheel_nr]);
        car->h_spring[wheel_nr] =
            ivp_environment_add_controller(env, IVP_CP_ACTUATOR, ivp_controller_suspension, sp);

        /* rotation motor (wheel drive) */
        car_anchor_t left, right;
        lvf_zero(&hp);
        hp.k[cs_car->x_idx] = car_f(templ->wheel_reversed_sign[wheel_nr]);
        car_anchor_os(car->car_wheel[wheel_nr], &hp, &left);
        hp.k[cs_car->x_idx] = -car_f(templ->wheel_reversed_sign[wheel_nr]);
        car_anchor_os(car->car_wheel[wheel_nr], &hp, &right);
        car_create_torque(car, &car->car_act_torque[wheel_nr], &car->h_torque[wheel_nr], car->car_wheel[wheel_nr],
                          &left, &right, car_f(templ->wheel_max_rotation_speed[(wheel_nr & 2) ? 1 : 0]));
        car->fix_wheel_constraint[wheel_nr] = NULL;
    }

    /*** wheel rotation counter torque on the body ***/
    {
        ivp_vec3f_t hp;
        car_anchor_t left, right;
        lvf_zero(&hp);
        hp.k[cs_car->x_idx] = 1.0f;
        car_anchor_os(car->body, &hp, &left);
        hp.k[cs_car->x_idx] = -1.0f;
        car_anchor_os(car->body, &hp, &right);
        car_create_torque(car, &car->car_act_torque_body, &car->h_torque_body, car->body, &left, &right,
                          (float)IVP_PI * 100.0f);
        car->body_counter_torque_factor = car_f(templ->body_counter_torque_factor);
    }

    /*** stabilizers: one per axis with at least two wheels (never announced) ***/
    int wheels_per_axis = (car->n_axes > 0) ? (car->n_wheels / car->n_axes) : 0;
    if (wheels_per_axis >= 2) {
        int axis_count = car->n_axes;
        if (axis_count > IVP_CAR_SYSTEM_MAX_AXES) axis_count = IVP_CAR_SYSTEM_MAX_AXES;
        for (int i = 0; i < axis_count; i++) {
            int wheel0 = i * wheels_per_axis;
            int wheel1 = wheel0 + 1;
            if (wheel1 >= car->n_wheels) break;
            ivp_vec3r_t b0 = car_vr(&anchor_body[wheel0].core_pos), w0 = car_vr(&anchor_wheel[wheel0].core_pos);
            ivp_vec3r_t b1 = car_vr(&anchor_body[wheel1].core_pos), w1 = car_vr(&anchor_wheel[wheel1].core_pos);
            ivp_stabilizer_state_init(&car->car_stabilizer[i]);
            ivp_stabilizer_state_setup(&car->car_stabilizer[i], car->body, car->car_wheel[wheel0], car->body,
                                       car->car_wheel[wheel1], &b0, &w0, &b1, &w1,
                                       (ivp_real_t)car_f(templ->stabilizer_constant[i]));
            /* the anchors are already resolved (IVP_Anchor::init_anchor) */
            ivp_stabilizer_state_t *stab = &car->car_stabilizer[i];
            stab->anchor_os[0] = car_state_anchor(&anchor_body[wheel0], &stab->anchors[0]);
            stab->anchor_os[1] = car_state_anchor(&anchor_wheel[wheel0], &stab->anchors[1]);
            stab->anchor_os[2] = car_state_anchor(&anchor_body[wheel1], &stab->anchors[2]);
            stab->anchor_os[3] = car_state_anchor(&anchor_wheel[wheel1], &stab->anchors[3]);
            car->car_stabilizer_exists[i] = true;
        }
    }

    /*** down force ***/
    ivp_object_t *static_object = ivp_environment_get_static_object(env);
    ivp_vec3r_t gravity;
    ivp_environment_get_gravity(env, &gravity);
    {
        ivp_vec3f_t hp;
        car_anchor_t center, down;
        lvf_zero(&hp);
        hp.k[cs_car->y_idx] += car_f(templ->body_down_force_vertical_offset); /* offset for arbitrary rotation */
        car_anchor_cs(car->body, &hp, &center);
        hp.k[cs_car->y_idx] = 1e8f; /* real far downwards so it appears vertical */
        if (gravity.k[cs_car->y_idx] > 0) hp.k[cs_car->y_idx] *= -1;
        car_anchor_os(static_object, &hp, &down);
        car_create_force(car, &car->car_act_down_force, &car->h_down_force, car->body, &center, static_object, &down,
                         0.0f);
    }

    /*** extra gravity ***/
    {
        ivp_vec3f_t center, hp;
        car_anchor_t center_cp, down;
        lvf_zero(&center);
        center.k[cs_car->y_idx] = car_f(templ->extra_gravity_height_offset);
        car_anchor_cs(car->body, &center, &center_cp);
        lvf_zero(&hp);
        hp.k[cs_car->y_idx] = 1e8f;
        if (gravity.k[cs_car->y_idx] > 0) hp.k[cs_car->y_idx] *= -1;
        car_anchor_os(static_object, &hp, &down);
        car_create_force(car, &car->car_act_extra_gravity, &car->h_extra_gravity, car->body, &center_cp, static_object,
                         &down, car_f(templ->extra_gravity_force));
    }

    /*** powerslides ***/
    {
        ivp_vec3f_t hp;
        car_anchor_t left, right;
        lvf_zero(&hp);
        hp.k[cs_car->x_idx] = -1.0f;
        hp.k[cs_car->z_idx] = 2.0f;
        car_anchor_cs(car->body, &hp, &left);
        hp.k[cs_car->x_idx] = 1.0f;
        hp.k[cs_car->z_idx] = 2.0f;
        car_anchor_cs(car->body, &hp, &right);
        car_create_force(car, &car->car_act_powerslide_back, &car->h_powerslide_back, car->body, &left, car->body,
                         &right, 0.0f);

        hp.k[cs_car->x_idx] = -1.0f;
        hp.k[cs_car->z_idx] = -2.0f;
        car_anchor_cs(car->body, &hp, &left);
        hp.k[cs_car->x_idx] = 1.0f;
        hp.k[cs_car->z_idx] = -2.0f;
        car_anchor_cs(car->body, &hp, &right);
        car_create_force(car, &car->car_act_powerslide_front, &car->h_powerslide_front, car->body, &left, car->body,
                         &right, 0.0f);
    }

    for (int w = 0; w < car->n_wheels; w++) ivp_car_system_steer_wheel(car, w, IVP_REAL(0.0));

    car->steering_angle = -1.0f;                   /* make sure next call is not optimized */
    ivp_car_system_set_steering(car, IVP_REAL(0.0)); /* make sure next call gets through */
    return true;
}

void ivp_car_system_destroy(ivp_car_system_t *car)
{
    if (!car) return;
    car_unregister_all(car, false);
    for (int i = 0; i < IVP_CAR_SYSTEM_MAX_WHEELS; i++) {
        free(car->fix_wheel_constraint[i]);
        car->fix_wheel_constraint[i] = NULL;
    }
    if (car->car_constraint_solver) {
        ivp_constraint_solver_car_destroy(car->car_constraint_solver);
        car->car_constraint_solver = NULL;
    }
    if (ivp_event_listener_handle_is_valid(car->env_listener) && car->environment && !car->environment_dying) {
        (void)ivp_environment_remove_event_listener(car->environment, car->env_listener);
    }
    car->env_listener = ivp_event_listener_handle_invalid();
    car->active = false;
    car->n_wheels = 0;
}

/* ── car adjustment ─────────────────────────────────────────────────────── */

void ivp_car_system_set_spring_constant(ivp_car_system_t *car, int w, ivp_real_t constant)
{
    if (!car_wheel_ok(car, w)) return;
    ivp_suspension_set_constant(&car->car_spring[w], (ivp_real_t)(float)constant);
}

void ivp_car_system_set_spring_damping(ivp_car_system_t *car, int w, ivp_real_t damp_factor)
{
    if (!car_wheel_ok(car, w)) return;
    ivp_suspension_set_damp(&car->car_spring[w], (ivp_real_t)(float)damp_factor);
}

void ivp_car_system_set_spring_damping_compression(ivp_car_system_t *car, int w, ivp_real_t damp_factor)
{
    if (!car_wheel_ok(car, w)) return;
    ivp_suspension_set_damping_compression(&car->car_spring[w], (ivp_real_t)(float)damp_factor);
}

void ivp_car_system_set_max_body_force(ivp_car_system_t *car, int w, ivp_real_t mforce)
{
    if (!car_wheel_ok(car, w)) return;
    ivp_suspension_set_max_body_force(&car->car_spring[w], (ivp_real_t)(float)mforce);
}

void ivp_car_system_set_spring_pre_tension(ivp_car_system_t *car, int w, ivp_real_t pre_tension_len)
{
    if (!car_wheel_ok(car, w)) return;
    ivp_suspension_set_len(&car->car_spring[w], (ivp_real_t)(500.0f - (float)pre_tension_len));
}

void ivp_car_system_set_spring_length(ivp_car_system_t *car, int w, ivp_real_t spring_length)
{
    if (!car_wheel_ok(car, w)) return;
    ivp_suspension_set_len(&car->car_spring[w], (ivp_real_t)(float)spring_length);
}

void ivp_car_system_set_stabilizer(ivp_car_system_t *car, int axis_nr, ivp_real_t stabi_constant)
{
    if (!car_alive(car) || axis_nr < 0 || axis_nr >= IVP_CAR_SYSTEM_MAX_AXES) return;
    if (!car->car_stabilizer_exists[axis_nr]) return;
    ivp_stabilizer_set_constant(&car->car_stabilizer[axis_nr], (ivp_real_t)(float)stabi_constant);
}

void ivp_car_system_set_fast_turn_factor(ivp_car_system_t *car, ivp_real_t factor)
{
    if (!car) return;
    car->fast_turn_factor = (float)factor;
}

void ivp_car_system_set_wheel_torque(ivp_car_system_t *car, int w, ivp_real_t torque)
{
    if (!car_wheel_ok(car, w)) return;
    ivp_torque_actuator_set_torque(&car->car_act_torque[w], (ivp_real_t)(float)torque);
}

void ivp_car_system_set_wheel_speed_dampening(ivp_car_system_t *car, int w, ivp_real_t dampening)
{
    if (!car_wheel_ok(car, w)) return;
    car->car_wheel[w]->physical_core->speed_damp_factor = (ivp_real_t)(float)dampening;
}

void ivp_car_system_update_body_countertorque(ivp_car_system_t *car)
{
    if (!car_alive(car)) return;
    float counter_torque = 0.0f;
    /* sum up all (negative) wheel torques */
    for (int i = 0; i < car->n_wheels; i++) counter_torque -= (float)car->car_act_torque[i].torque;
    ivp_torque_actuator_set_torque(&car->car_act_torque_body,
                                   (ivp_real_t)(counter_torque * car->body_counter_torque_factor));
}

void ivp_car_system_set_body_downforce(ivp_car_system_t *car, ivp_real_t force)
{
    if (!car_alive(car)) return;
    ivp_force_actuator_set_force(&car->car_act_down_force, (ivp_real_t)(float)force);
}

void ivp_car_system_set_powerslide(ivp_car_system_t *car, ivp_real_t front_accel, ivp_real_t rear_accel)
{
    if (!car_alive(car)) return;
    float mass = ivp_core_get_mass_f(car->body->physical_core);
    ivp_force_actuator_set_force(&car->car_act_powerslide_back, (ivp_real_t)(mass * (float)front_accel));
    ivp_force_actuator_set_force(&car->car_act_powerslide_front, (ivp_real_t)(mass * (float)rear_accel));
}

/* fix_wheel: stop the wheel completely (e.g. handbrake) */
void ivp_car_system_fix_wheel(ivp_car_system_t *car, int wheel_nr, bool stop_wheel)
{
    if (!car_wheel_ok(car, wheel_nr)) return;
    ivp_constraint_solver_car_t *cs_car = car->car_constraint_solver;
    IVP_ASSERT(cs_car);
    if (!cs_car) return;
    ivp_constraint_car_object_t *wheel_obj = cs_car->wheel_objects[wheel_nr];

    if (!stop_wheel) {
        if (wheel_obj) wheel_obj->fix_wheel_constraint = NULL;
        car_remove_controller(car, &car->h_fix_wheel[wheel_nr]);
        free(car->fix_wheel_constraint[wheel_nr]);
        car->fix_wheel_constraint[wheel_nr] = NULL;
        return;
    }

    ivp_object_t *wheel = car->car_wheel[wheel_nr];

    /* already activated */
    if (car->fix_wheel_constraint[wheel_nr]) return;

    /* constraint blocking the wheel rotation */
    ivp_constraint_template_t constraint;
    ivp_constraint_template_init(&constraint);
    constraint.obj_r = wheel;     /* reference object */
    constraint.obj_a = car->body; /* attached object */
    constraint.axis_type[3 + cs_car->x_idx] = IVP_CONSTRAINT_AXIS_FIXED;
    constraint.axis_type[3 + cs_car->y_idx] = IVP_CONSTRAINT_AXIS_FREE;
    constraint.axis_type[3 + cs_car->z_idx] = IVP_CONSTRAINT_AXIS_FREE;
    constraint.axis_type[cs_car->x_idx] = IVP_CONSTRAINT_AXIS_FREE;
    constraint.axis_type[cs_car->y_idx] = IVP_CONSTRAINT_AXIS_FREE;
    constraint.axis_type[cs_car->z_idx] = IVP_CONSTRAINT_AXIS_FREE;

    /* magic trick: slow down the wheel to avoid effects that would normally
     * result from such an immediate blocking (inertias!) */
    ivp_core_t *wheel_core = wheel->physical_core;
    lvf_add_multiple(&wheel_core->rot_speed, &car->body->physical_core->rot_speed, 0.25f);

    ivp_constraint_state_t *c = (ivp_constraint_state_t *)calloc(1, sizeof(*c));
    if (!c) return;
    ivp_constraint_init_from_template(c, &constraint);
    car->fix_wheel_constraint[wheel_nr] = c;
    car->h_fix_wheel[wheel_nr] = ivp_environment_add_controller(car->environment, IVP_CP_CONSTRAINTS,
                                                                ivp_controller_constraint, c);
    if (wheel_obj) wheel_obj->fix_wheel_constraint = c;
}

/* ── booster ('rocket' propulsion) ──────────────────────────────────────── */

void ivp_car_system_set_booster_acceleration(ivp_car_system_t *car, ivp_real_t acceleration_in)
{
    if (!car_alive(car)) return;
    float acceleration = (float)acceleration_in;
    if (acceleration != 0.0f) {
        ivp_object_t *pCarBody = car->body;
        float mass = ivp_core_get_mass_f(pCarBody->physical_core);
        if (!car->booster_active) {
            ivp_constraint_solver_car_t *pCarConstraint = car->car_constraint_solver;
            ivp_object_t *pStaticObject = ivp_environment_get_static_object(car->environment);

            ivp_vec3f_t frontPoint, backPoint;
            car_anchor_t a0, a1;
            lvf_zero(&frontPoint);
            lvf_zero(&backPoint);
            frontPoint.k[pCarConstraint->z_idx] = 1.0f;
            backPoint.k[pCarConstraint->z_idx] = -1.0f;
            car_anchor_cs(pCarBody, &frontPoint, &a0);
            car_anchor_cs(pCarBody, &backPoint, &a1);
            car_create_force(car, &car->booster_actuator[0], &car->h_booster[0], pCarBody, &a0, pCarBody, &a1,
                             acceleration * mass);

            ivp_vec3f_t centerPoint, downPoint;
            lvf_zero(&centerPoint);
            lvf_zero(&downPoint);
            downPoint.k[pCarConstraint->y_idx] = -1e8f;
            car_anchor_cs(pCarBody, &centerPoint, &a0);
            car_anchor_os(pStaticObject, &downPoint, &a1);
            ivp_vec3r_t gravity;
            ivp_environment_get_gravity(car->environment, &gravity);
            float up_force = (float)((D)(-1.0f * mass) * (D)gravity.k[pCarConstraint->y_idx]);
            car_create_force(car, &car->booster_actuator[1], &car->h_booster[1], pCarBody, &a0, pStaticObject, &a1,
                             up_force);
            car->booster_active = true;
        } else {
            ivp_force_actuator_set_force(&car->booster_actuator[0], (ivp_real_t)(acceleration * mass));
        }
    } else {
        car_remove_controller(car, &car->h_booster[0]);
        car_remove_controller(car, &car->h_booster[1]);
        car->booster_active = false;
    }
}

void ivp_car_system_activate_booster(ivp_car_system_t *car, ivp_real_t thrust, ivp_real_t duration,
                                     ivp_real_t delay)
{
    if (!car_alive(car)) return;
    if (car->booster_active) return;                    /* do we have a booster? */
    if (car->booster_seconds_until_ready > 0.0f) return; /* booster not ready yet */

    /* ignite the booster by adding an actuator force to the car body */
    ivp_vec3r_t g;
    ivp_environment_get_gravity(car->environment, &g);
    float gravity = (float)lvp_real_length(&g);
    ivp_car_system_set_booster_acceleration(car, (ivp_real_t)((float)thrust * gravity));

    car->booster_seconds_to_go = (float)duration;                  /* seconds */
    car->booster_seconds_until_ready = (float)duration + (float)delay; /* next ignition */
}

void ivp_car_system_update_booster(ivp_car_system_t *car, ivp_real_t delta_time)
{
    if (!car_alive(car)) return;
    if (car->booster_seconds_until_ready > 0.0f) car->booster_seconds_until_ready -= (float)delta_time;
    if (car->booster_seconds_to_go > 0.0f) {
        car->booster_seconds_to_go -= (float)delta_time;
        /* booster shut down? */
        if (car->booster_seconds_to_go <= 0.0f) ivp_car_system_set_booster_acceleration(car, IVP_REAL(0.0));
    }
}

ivp_real_t ivp_car_system_get_booster_delay(const ivp_car_system_t *car)
{
    return car ? (ivp_real_t)car->booster_seconds_until_ready : IVP_REAL(0.0);
}

ivp_real_t ivp_car_system_get_booster_time_to_go(const ivp_car_system_t *car)
{
    return car ? (ivp_real_t)car->booster_seconds_to_go : IVP_REAL(0.0);
}

/* ── skid info ──────────────────────────────────────────────────────────── */

void ivp_car_system_get_skid_info(const ivp_car_system_t *car, ivp_car_system_wheel_skid_info_t *out)
{
    if (!car_alive(car) || !out) return;
    for (int w = 0; w < car->n_wheels; w++) {
        const ivp_constraint_car_object_t *wheel = car->car_constraint_solver->wheel_objects[w];
        if (!wheel) {
            memset(&out[w], 0, sizeof(out[w]));
            continue;
        }
        out[w].last_contact_position_ws = wheel->last_contact_position_ws;
        out[w].last_skid_value = wheel->last_skid_value;
        out[w].last_skid_time = wheel->last_skid_time;
    }
}

/* ── compatibility ──────────────────────────────────────────────────────── */

void ivp_controller_car_system(const ivp_event_sim_t *event_sim, ivp_object_t *obj, void *ctx)
{
    (void)event_sim;
    (void)obj;
    (void)ctx;
}
