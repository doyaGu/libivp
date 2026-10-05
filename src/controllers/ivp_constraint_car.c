/* ivp_constraint_car.c -- port of ivp_controller/ivp_constraint_car.cxx
 *
 *   IVP_Constraint_Car_Object, IVP_Constraint_Solver_Car (an
 *   IVP_Controller_Dependent, priority IVP_CP_CONSTRAINTS) and
 *   IVP_Constraint_Solver_Car_Builder, plus the reference-precision
 *   IVP_Solver_Core_Reaction used by the car code (ivp_car_scr_*).
 *
 * The arithmetic follows the reference types and evaluation order
 * (IVP_U_Float_Point -> ivp_vec3f_t, IVP_U_Point / IVP_U_Matrix ->
 * ivp_vec3r_t / ivp_transform_r_t, IVP_DOUBLE -> D).  The IVP_IF blocks of
 * the reference (debug build only: the plan B prints) are ported as IVP_IF.
 *
 * The builder's temporary matrix uses row stride == columns (reference
 * 81357f3).  Deviation: when one of the cores is deleted the solver is
 * detached from the environment instead of deleting itself (the owner frees
 * it).
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/dynamics/ivp_environment.h>
#include "controllers/ivp_constraint_car_internal.h"

/* ══════════════════════════════════════════════════════════════════════════
 * IVP_Solver_Core_Reaction (reference precision)
 * ══════════════════════════════════════════════════════════════════════════ */

/* IVP_U_Point_4::set_line_wise_mult4 */
static inline void p4_line_wise_mult4(ivp_car_point4_t *r, const ivp_car_point4_t *a, const ivp_car_point4_t *b)
{
    D x = (D)(a->v.k[0] * b->v.k[0]);
    D y = (D)(a->v.k[1] * b->v.k[1]);
    D z = (D)(a->v.k[2] * b->v.k[2]);
    D w = (D)(a->hesse_val * b->hesse_val);
    r->v.k[0] = (float)x;
    r->v.k[1] = (float)y;
    r->v.k[2] = (float)z;
    r->hesse_val = (float)w;
}

/* IVP_U_Point_4::dot_product4 (float arithmetic) */
static inline D p4_dot4(const ivp_car_point4_t *a, const ivp_car_point4_t *b)
{
    return (D)(a->v.k[0] * b->v.k[0] + a->v.k[1] * b->v.k[1] + a->v.k[2] * b->v.k[2] + a->hesse_val * b->hesse_val);
}

/* IVP_Core::get_inv_masses() */
static inline void scr_inv_masses(const ivp_core_t *core, ivp_car_point4_t *im)
{
    im->v = ivp_core_inv_rot_inertia_f(core);
    im->hesse_val = ivp_core_get_inv_mass_f(core);
}

/* delta_velocity_ds.k[i] += a * sign  (IVP_FLOAT += IVP_DOUBLE) */
static inline void scr_add_delta(ivp_car_scr_t *s, int i, D a, float sign)
{
    s->delta_velocity_ds.k[i] = (float)((D)s->delta_velocity_ds.k[i] + a * (D)sign);
}

/* IVP_Solver_Core_Reaction::init_trans_ws */
static void scr_init_trans_ws(ivp_car_scr_t *s, const ivp_vec3r_t *pos_ws, ivp_core_t *core,
                              ivp_car_point4_t cdp[3], ivp_car_point4_t cmi[3], float sign)
{
    const ivp_transform_r_t *m_world_f_core = &core->m_world_f_core_last_psi;
    ivp_vec3f_t pos_rel_core;
    lvf_subtract2_p(&pos_rel_core, pos_ws, &m_world_f_core->pos);
    ivp_car_point4_t im;
    scr_inv_masses(core, &im);
    ivp_vec3f_t speed = ivp_core_speed_f(core);

    {
        ivp_vec3f_t cross_ws;
        lvf_cross(&cross_ws, &pos_rel_core, s->direction_ws[0]);
        lvm_vimult3_f(m_world_f_core, &cross_ws, &cdp[0].v);
        cdp[0].hesse_val = 1.0f;
        p4_line_wise_mult4(&cmi[0], &cdp[0], &im);
        s->m[0][0] = s->m[0][0] + p4_dot4(&cmi[0], &cdp[0]);
        D a = lvf_dot(&core->rot_speed, &cdp[0].v) + lvf_dot(&speed, s->direction_ws[0]);
        scr_add_delta(s, 0, a, sign);
    }

    if (!s->direction_ws[1]) return;
    {
        ivp_vec3f_t cross_ws;
        lvf_cross(&cross_ws, &pos_rel_core, s->direction_ws[1]);
        lvm_vimult3_f(m_world_f_core, &cross_ws, &cdp[1].v);
        cdp[1].hesse_val = 1.0f;
        p4_line_wise_mult4(&cmi[1], &cdp[1], &im);
        s->m[1][1] = s->m[1][1] + p4_dot4(&cmi[1], &cdp[1]);
        s->m[0][1] = s->m[0][1] + lvf_dot(&cmi[1].v, &cdp[0].v);
        D a = lvf_dot(&core->rot_speed, &cdp[1].v) + lvf_dot(&speed, s->direction_ws[1]);
        scr_add_delta(s, 1, a, sign);
    }

    if (!s->direction_ws[2]) return;
    {
        ivp_vec3f_t cross_ws;
        lvf_cross(&cross_ws, &pos_rel_core, s->direction_ws[2]);
        lvm_vimult3_f(m_world_f_core, &cross_ws, &cdp[2].v);
        cdp[2].hesse_val = 1.0f;
        p4_line_wise_mult4(&cmi[2], &cdp[2], &im);
        s->m[2][2] = s->m[2][2] + p4_dot4(&cmi[2], &cdp[2]);
        s->m[0][2] = s->m[0][2] + lvf_dot(&cmi[2].v, &cdp[0].v);
        s->m[1][2] = s->m[1][2] + lvf_dot(&cmi[2].v, &cdp[1].v);
        D a = lvf_dot(&core->rot_speed, &cdp[2].v) + lvf_dot(&speed, s->direction_ws[2]);
        scr_add_delta(s, 2, a, sign);
    }
}

/* IVP_Solver_Core_Reaction::init_rot_ws */
static void scr_init_rot_ws(ivp_car_scr_t *s, ivp_core_t *core, ivp_car_point4_t cdp[3], ivp_car_point4_t cmi[3],
                            float sign)
{
    const ivp_transform_r_t *m_world_f_core = &core->m_world_f_core_last_psi;
    ivp_car_point4_t im;
    scr_inv_masses(core, &im);

    {
        lvm_vimult3_f(m_world_f_core, s->direction_ws[0], &cdp[0].v);
        cdp[0].hesse_val = 0.0f;
        p4_line_wise_mult4(&cmi[0], &cdp[0], &im);
        s->m[0][0] = s->m[0][0] + lvf_dot(&cmi[0].v, &cdp[0].v);
        D a = lvf_dot(&core->rot_speed, &cdp[0].v);
        scr_add_delta(s, 0, a, sign);
    }
    if (s->direction_ws[1]) {
        lvm_vimult3_f(m_world_f_core, s->direction_ws[1], &cdp[1].v);
        cdp[1].hesse_val = 0.0f;
        p4_line_wise_mult4(&cmi[1], &cdp[1], &im);
        s->m[1][1] = s->m[1][1] + lvf_dot(&cmi[1].v, &cdp[1].v);
        s->m[0][1] = s->m[0][1] + lvf_dot(&cmi[1].v, &cdp[0].v);
        D a = lvf_dot(&core->rot_speed, &cdp[1].v);
        scr_add_delta(s, 1, a, sign);
    }
    if (s->direction_ws[2]) {
        lvm_vimult3_f(m_world_f_core, s->direction_ws[2], &cdp[2].v);
        cdp[2].hesse_val = 0.0f;
        p4_line_wise_mult4(&cmi[2], &cdp[2], &im);
        s->m[2][2] = s->m[2][2] + lvf_dot(&cmi[2].v, &cdp[2].v);
        s->m[0][2] = s->m[0][2] + lvf_dot(&cmi[2].v, &cdp[0].v);
        s->m[1][2] = s->m[1][2] + lvf_dot(&cmi[2].v, &cdp[1].v);
        D a = lvf_dot(&core->rot_speed, &cdp[2].v);
        scr_add_delta(s, 2, a, sign);
    }
}

void ivp_car_scr_init_translation_ws(ivp_car_scr_t *s, ivp_core_t *core_0, ivp_core_t *core_1,
                                     const ivp_vec3r_t *pos_ws, const ivp_vec3f_t *direction_0_ws,
                                     const ivp_vec3f_t *direction_1_ws, const ivp_vec3f_t *direction_2_ws)
{
    s->direction_ws[0] = direction_0_ws;
    s->direction_ws[1] = direction_1_ws;
    s->direction_ws[2] = direction_2_ws;
    if (direction_1_ws) {
        memset(s->m, 0, sizeof(s->m));
        lvf_zero(&s->delta_velocity_ds);
    } else {
        /* reference: just one element (the rest keeps whatever it was) */
        s->m[0][0] = 0.0f;
        s->delta_velocity_ds.k[0] = 0.0f;
    }
    if (core_0) scr_init_trans_ws(s, pos_ws, core_0, s->cross_direction_position_cs0, s->cr_mult_inv0, 1.0f);
    if (core_1) scr_init_trans_ws(s, pos_ws, core_1, s->cross_direction_position_cs1, s->cr_mult_inv1, -1.0f);
}

void ivp_car_scr_init_rotation_ws(ivp_car_scr_t *s, ivp_core_t *core_0, ivp_core_t *core_1,
                                  const ivp_vec3f_t *direction_0_ws, const ivp_vec3f_t *direction_1_ws,
                                  const ivp_vec3f_t *direction_2_ws)
{
    s->direction_ws[0] = direction_0_ws;
    s->direction_ws[1] = direction_1_ws;
    s->direction_ws[2] = direction_2_ws;
    memset(s->m, 0, sizeof(s->m));
    lvf_zero(&s->delta_velocity_ds);
    if (core_0) scr_init_rot_ws(s, core_0, s->cross_direction_position_cs0, s->cr_mult_inv0, 1.0f);
    if (core_1) scr_init_rot_ws(s, core_1, s->cross_direction_position_cs1, s->cr_mult_inv1, -1.0f);
}

void ivp_car_scr_exert_impulse_dim2(const ivp_car_scr_t *s, ivp_core_t *core_0, ivp_core_t *core_1,
                                    const ivp_vec3f_t *impulse_ds)
{
    if (core_0) {
        D inv_m = (D)ivp_core_get_inv_mass_f(core_0);
        ivp_vec3f_t speed = ivp_core_speed_f(core_0);
        lvf_add_multiple(&speed, s->direction_ws[0], (D)impulse_ds->k[0] * inv_m);
        lvf_add_multiple(&speed, s->direction_ws[1], (D)impulse_ds->k[1] * inv_m);
        ivp_core_set_speed_f(core_0, &speed);
        lvf_add_multiple(&core_0->rot_speed, &s->cr_mult_inv0[0].v, (D)impulse_ds->k[0]);
        lvf_add_multiple(&core_0->rot_speed, &s->cr_mult_inv0[1].v, (D)impulse_ds->k[1]);
    }
    if (core_1) {
        D inv_m = (D)ivp_core_get_inv_mass_f(core_1);
        ivp_vec3f_t speed = ivp_core_speed_f(core_1);
        lvf_add_multiple(&speed, s->direction_ws[0], (D)(-impulse_ds->k[0]) * inv_m);
        lvf_add_multiple(&speed, s->direction_ws[1], (D)(-impulse_ds->k[1]) * inv_m);
        ivp_core_set_speed_f(core_1, &speed);
        lvf_add_multiple(&core_1->rot_speed, &s->cr_mult_inv1[0].v, (D)(-impulse_ds->k[0]));
        lvf_add_multiple(&core_1->rot_speed, &s->cr_mult_inv1[1].v, (D)(-impulse_ds->k[1]));
    }
}

void ivp_car_scr_exert_angular_impulse_dim2(const ivp_car_scr_t *s, ivp_core_t *core_0, ivp_core_t *core_1,
                                            const ivp_vec3f_t *impulse_ds)
{
    if (core_0) {
        lvf_add_multiple(&core_0->rot_speed, &s->cr_mult_inv0[0].v, (D)impulse_ds->k[0]);
        lvf_add_multiple(&core_0->rot_speed, &s->cr_mult_inv0[1].v, (D)impulse_ds->k[1]);
    }
    if (core_1) {
        lvf_add_multiple(&core_1->rot_speed, &s->cr_mult_inv1[0].v, (D)(-impulse_ds->k[0]));
        lvf_add_multiple(&core_1->rot_speed, &s->cr_mult_inv1[1].v, (D)(-impulse_ds->k[1]));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * matrix helpers (IVP_U_Matrix)
 * ══════════════════════════════════════════════════════════════════════════ */

static void car_matrix_init(ivp_transform_r_t *m)
{
    memset(m, 0, sizeof(*m));
    m->rot.rows[0].k[0] = 1.0f;
    m->rot.rows[1].k[1] = 1.0f;
    m->rot.rows[2].k[2] = 1.0f;
}

/* IVP_U_Matrix::inline_mmult4: out = a * mb */
static void car_mmult4(const ivp_transform_r_t *a, const ivp_transform_r_t *mb, ivp_transform_r_t *out)
{
    ivp_vec3r_t v_shift_korr;
    lvm_vmult3(a, &mb->pos, &v_shift_korr);
    ivp_transform_r_t r;
    for (int row = 0; row < 3; row++) {
        ivp_vec3r_t in = a->rot.rows[row];
        lvm_vimult3(mb, &in, &r.rot.rows[row]);
    }
    lvp_add2(&r.pos, &a->pos, &v_shift_korr);
    *out = r;
}

/* IVP_U_Matrix3::get_col into an IVP_U_Float_Point */
static inline void car_get_col_f(const ivp_transform_r_t *m, int col, ivp_vec3f_t *out)
{
    out->k[0] = (float)LVM_E(m, 0, col);
    out->k[1] = (float)LVM_E(m, 1, col);
    out->k[2] = (float)LVM_E(m, 2, col);
}

void ivp_car_calc_m_core_f_object(const ivp_object_t *obj, ivp_transform_r_t *m_core_f_object)
{
    car_matrix_init(m_core_f_object);
    if (obj->q_core_f_object) ivp_quat_to_mat3(obj->q_core_f_object, &m_core_f_object->rot);
    /* reference shift_core_f_object is an IVP_U_Float_Point */
    m_core_f_object->pos.k[0] = (D)(float)obj->shift_core_f_object.k[0];
    m_core_f_object->pos.k[1] = (D)(float)obj->shift_core_f_object.k[1];
    m_core_f_object->pos.k[2] = (D)(float)obj->shift_core_f_object.k[2];
}

/* ══════════════════════════════════════════════════════════════════════════
 * IVP_Constraint_Car_Object
 * ══════════════════════════════════════════════════════════════════════════ */

static ivp_constraint_car_object_t *car_object_create(ivp_constraint_solver_car_t *solver_car,
                                                      ivp_object_t *i_real_obj_app, ivp_object_t *i_real_obj_body,
                                                      const ivp_vec3f_t *target_Bos_override)
{
    ivp_constraint_car_object_t *o = (ivp_constraint_car_object_t *)calloc(1, sizeof(*o));
    if (!o) return NULL;
    o->real_object = i_real_obj_app;
    o->solver_car = solver_car;
    o->fix_wheel_constraint = NULL;
    i_real_obj_app->physical_core->car_wheel = o;
    o->last_skid_value = 0.0f;
    lvf_zero(&o->last_contact_position_ws);
    o->last_skid_time = 0.0;

    if (i_real_obj_body) {
        const ivp_core_t *core_A = i_real_obj_app->physical_core;
        const ivp_core_t *core_B = i_real_obj_body->physical_core;
        const ivp_transform_r_t *m_world_f_B = &core_B->m_world_f_core_last_psi;
        const ivp_transform_r_t *m_world_f_A = &core_A->m_world_f_core_last_psi;
        if (target_Bos_override) {
            car_matrix_init(&o->target_position_bs);
            ivp_transform_r_t m_core_f_object;
            ivp_car_calc_m_core_f_object(i_real_obj_body, &m_core_f_object);
            lvm_vmult4_fp(&m_core_f_object, target_Bos_override, &o->target_position_bs.pos);
        } else {
            lvm_mimult4(m_world_f_B, m_world_f_A, &o->target_position_bs);
        }
    } else {
        car_matrix_init(&o->target_position_bs);
    }
    return o;
}

static void car_object_destroy(ivp_constraint_car_object_t *o, bool touch_core)
{
    if (!o) return;
    if (touch_core && o->real_object && o->real_object->physical_core->car_wheel == o) o->real_object->physical_core->car_wheel = NULL;
    free(o);
}

/* ══════════════════════════════════════════════════════════════════════════
 * IVP_Constraint_Solver_Car
 * ══════════════════════════════════════════════════════════════════════════ */

static inline ivp_constraint_solver_car_t *solver_of(ivp_sim_ctrl_t *c) { return (ivp_constraint_solver_car_t *)c; }

static int solver_get_priority(ivp_sim_ctrl_t *c)
{
    (void)c;
    return IVP_CP_CONSTRAINTS;
}

static ivp_pvec_t *solver_get_cores(ivp_sim_ctrl_t *c) { return &solver_of(c)->cores_of_constraint_system; }

static void solver_do_simulation_controller(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *core_list);
static void solver_core_is_going_to_be_deleted_event(ivp_sim_ctrl_t *c, ivp_core_t *core);

static const ivp_sim_ctrl_vt_t solver_vt = {
    solver_get_priority,
    solver_do_simulation_controller,
    solver_get_cores,
    solver_core_is_going_to_be_deleted_event,
    NULL,
};

ivp_constraint_solver_car_t *ivp_constraint_solver_car_create(int right_idx, int up_idx, int forward_idx,
                                                              bool is_left_hand)
{
    ivp_constraint_solver_car_t *s = (ivp_constraint_solver_car_t *)calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->base.vt = &solver_vt;
    ivp_pvec_init(&s->cores_of_constraint_system);
    memset(s->constraint_is_disabled, 0, sizeof(s->constraint_is_disabled));
    memset(s->c_local_ballsocket, 0, sizeof(s->c_local_ballsocket));
    for (int i = 0; i < IVP_CONSTRAINT_CAR_MAX_WHEELS; i++) s->c_local_ballsocket_handle[i] = ivp_controller_handle_invalid();
    s->body_object = NULL;
    s->x_idx = right_idx;
    s->y_idx = up_idx;
    s->z_idx = forward_idx;
    s->angle_sign = is_left_hand ? -1.0f : 1.0f;
    s->max_delta_speed = 80.0f * 3.0f;
    s->local_translation_in_use = false;
    s->environment = NULL;
    s->psis_left_for_plan_B = 0;
    s->env_listener = ivp_event_listener_handle_invalid();
    return s;
}

/* P_DELETE(this->c_local_ballsocket[k]) for all wheels */
static void solver_delete_local_ballsockets(ivp_constraint_solver_car_t *s, bool silently)
{
    int local_constraint_count = s->n_wheel_objects;
    if (local_constraint_count > IVP_CONSTRAINT_CAR_MAX_WHEELS) {
        IVP_ASSERT(0);
        local_constraint_count = IVP_CONSTRAINT_CAR_MAX_WHEELS;
    }
    for (int k = 0; k < local_constraint_count; k++) {
        if (s->c_local_ballsocket[k]) {
            if (s->environment && !s->environment_dying) {
                (void)ivp_environment_remove_dependent_controller(s->environment, s->c_local_ballsocket_handle[k],
                                                                  silently);
            }
            free(s->c_local_ballsocket[k]);
        }
        s->c_local_ballsocket[k] = NULL;
        s->c_local_ballsocket_handle[k] = ivp_controller_handle_invalid();
    }
}

/* remove from the environment (silently), clear core->car_wheel, delete the
 * car objects and plan-B constraints (the parts of ~IVP_Constraint_Solver_Car
 * that touch the world) */
static void solver_detach(ivp_constraint_solver_car_t *s, bool silently)
{
    if (s->announced) {
        ivp_ctrl_mgr_remove_controller_from_environment(s->environment, &s->base, true);
        s->announced = false;
    }
    car_object_destroy(s->body_object, true);
    s->body_object = NULL;
    for (int i = 0; i < s->n_wheel_objects; i++) {
        car_object_destroy(s->wheel_objects[i], true);
        s->wheel_objects[i] = NULL;
    }
    solver_delete_local_ballsockets(s, silently);
    s->n_wheel_objects = 0;
    s->local_translation_in_use = false;
    ivp_pvec_remove_all(&s->cores_of_constraint_system);
}

static void solver_env_event(void *ctx, ivp_environment_t *env, const ivp_event_t *ev)
{
    (void)env;
    ivp_constraint_solver_car_t *s = (ivp_constraint_solver_car_t *)ctx;
    if (ev && ev->type == IVP_EVENT_ENVIRONMENT_DELETED) {
        /* the environment is still intact: leave it now, before its objects
         * are deleted (reference: core_is_going_to_be_deleted_event deletes
         * the solver during the object teardown) */
        solver_detach(s, true);
        s->environment_dying = true;
        s->env_listener = ivp_event_listener_handle_invalid();
    }
}

void ivp_constraint_solver_car_destroy(ivp_constraint_solver_car_t *s)
{
    if (!s) return;
    if (s->environment && !s->environment_dying) {
        solver_detach(s, true);
        if (ivp_event_listener_handle_is_valid(s->env_listener)) {
            (void)ivp_environment_remove_event_listener(s->environment, s->env_listener);
        }
    } else {
        /* environment gone: only release memory (cores do not exist any more) */
        if (s->announced) s->announced = false;
        car_object_destroy(s->body_object, false);
        for (int i = 0; i < s->n_wheel_objects; i++) car_object_destroy(s->wheel_objects[i], false);
        for (int k = 0; k < IVP_CONSTRAINT_CAR_MAX_WHEELS; k++) free(s->c_local_ballsocket[k]);
    }
    if (s->co_matrix_allocated) ivp_gmmz_destroy(&s->co_matrix, NULL);
    ivp_pvec_clear(&s->cores_of_constraint_system);
    free(s);
}

/* IVP_Constraint_Solver_Car::core_is_going_to_be_deleted_event: the
 * reference deletes the solver (P_DELETE_THIS); libivp detaches it and
 * leaves the memory to its owner. */
static void solver_core_is_going_to_be_deleted_event(ivp_sim_ctrl_t *c, ivp_core_t *core)
{
    (void)core;
    solver_detach(solver_of(c), true);
}

bool ivp_constraint_solver_car_is_active(const ivp_constraint_solver_car_t *s) { return s && s->announced; }

int ivp_constraint_solver_car_get_num_wheels(const ivp_constraint_solver_car_t *s) { return s ? s->n_wheel_objects : 0; }

ivp_constraint_car_object_t *ivp_constraint_solver_car_get_wheel(ivp_constraint_solver_car_t *s, int i)
{
    if (!s || i < 0 || i >= s->n_wheel_objects) return NULL;
    return s->wheel_objects[i];
}

ivp_constraint_car_object_t *ivp_constraint_solver_car_get_body(ivp_constraint_solver_car_t *s)
{
    return s ? s->body_object : NULL;
}

void ivp_constraint_solver_car_get_axes(const ivp_constraint_solver_car_t *s, int *x_idx, int *y_idx, int *z_idx)
{
    if (x_idx) *x_idx = s ? s->x_idx : 0;
    if (y_idx) *y_idx = s ? s->y_idx : 1;
    if (z_idx) *z_idx = s ? s->z_idx : 2;
}

void ivp_constraint_solver_car_ensure_in_simulation(ivp_constraint_solver_car_t *s)
{
    if (!s || !s->announced) return;
    ivp_ctrl_mgr_ensure_controller_in_simulation(s->environment, &s->base);
}

/* ── IVP_Constraint_Solver_Car_Builder ──────────────────────────────────── */

typedef struct car_builder {
    ivp_constraint_solver_car_t *car_solver;
    int        n_appends;      /* e.g. 4 for four wheels, determines matrix size */
    int        n_constraints;  /* decreases with the number of disabled constraints */
    ivp_gmmz_t tmp_matrix;     /* temp, non-inverted matrix */
} car_builder_t;

static void builder_init(car_builder_t *b, ivp_constraint_solver_car_t *car_solver)
{
    memset(b, 0, sizeof(*b));
    b->car_solver = car_solver;
    b->n_appends = car_solver->n_wheel_objects;
    b->n_constraints = 6;
    for (int i = 0; i < 6; i++) car_solver->constraint_is_disabled[i] = false;
}

static void builder_disable_constraint(car_builder_t *b, int idx)
{
    IVP_ASSERT(idx >= 0 && idx < 6);
    if (!b->car_solver->constraint_is_disabled[idx]) {
        b->car_solver->constraint_is_disabled[idx] = true;
        b->n_constraints--;
    }
}

/* Bilateral test pushes along all enabled translation / rotation axes of the
 * body system; the resulting delta speeds are written into tmp_matrix. */
static void builder_calc_pushing_behavior(car_builder_t *b, int A_obj_idx, int push_vec_idx)
{
    ivp_constraint_solver_car_t *cs = b->car_solver;
    IVP_ASSERT(A_obj_idx >= 0 && A_obj_idx < b->n_appends);
    IVP_ASSERT(push_vec_idx >= 0 && push_vec_idx < 6);
    IVP_ASSERT(!cs->constraint_is_disabled[push_vec_idx]);

    ivp_constraint_car_object_t *A_obj = cs->wheel_objects[A_obj_idx];
    ivp_core_t *core_A = A_obj->real_object->physical_core;
    ivp_constraint_car_object_t *B_obj = cs->body_object;
    ivp_core_t *core_B = B_obj->real_object->physical_core;
    const ivp_transform_r_t *m_world_f_B = &core_B->m_world_f_core_last_psi;

    ivp_transform_r_t h_matrix = *m_world_f_B;
    ivp_transform_r_t *m_world_f_A = &h_matrix;
    car_mmult4(m_world_f_B, &A_obj->target_position_bs, m_world_f_A); /* ideal position of A */

    ivp_transform_r_t m_B_from_A;
    lvm_mimult4(m_world_f_B, m_world_f_A, &m_B_from_A);

    ivp_vec3f_t impulse_A, impulse_B, impulse_world;
    lvf_zero(&impulse_B);
    impulse_B.k[push_vec_idx % 3] = 1.0f;
    lvm_vmult3_f(m_world_f_B, &impulse_B, &impulse_world);
    lvm_vimult3_f(m_world_f_A, &impulse_world, &impulse_A);

    ivp_vec3f_t inv_impulse_A, inv_impulse_B, inv_impulse_world;
    lvf_set_negative(&inv_impulse_B, &impulse_B);
    lvm_vmult3_f(m_world_f_B, &inv_impulse_B, &inv_impulse_world);
    lvm_vimult3_f(m_world_f_A, &inv_impulse_world, &inv_impulse_A);
    (void)inv_impulse_A;

    ivp_vec3f_t A_push_pos_in_A, B_push_pos_in_B;
    lvf_zero(&A_push_pos_in_A); /* push in appendix center */
    lvm_vmult4_ff(&m_B_from_A, &A_push_pos_in_A, &B_push_pos_in_B);

    D rot_impulse_strength = 1.0f;
    D inv_rot_impulse_strength = -rot_impulse_strength;

    ivp_vec3f_t speed_change_A_in_B, rot_change_A_in_B;
    ivp_vec3f_t sur_speed_B_in_B, rot_change_B_in_B;
    ivp_vec3f_t sur_speed_B_world;
    ivp_vec3f_t speed_change_B_world;
    lvf_zero(&speed_change_A_in_B);
    lvf_zero(&rot_change_A_in_B);
    lvf_zero(&sur_speed_B_in_B);
    lvf_zero(&rot_change_B_in_B);
    lvf_zero(&speed_change_B_world);

    if (push_vec_idx <= 2) {
        /* push A in its center */
        ivp_vec3f_t speed_change_A_world, rot_change_A_in_A;
        ivp_core_test_push_core(core_A, &A_push_pos_in_A, &impulse_A, &impulse_world, &speed_change_A_world,
                                &rot_change_A_in_A);
        lvm_vimult3_f(m_world_f_B, &speed_change_A_world, &speed_change_A_in_B);
        lvf_zero(&rot_change_A_in_B); /* push in center should not result in rotation */

        /* push B (contrary) */
        ivp_core_test_push_core(core_B, &B_push_pos_in_B, &inv_impulse_B, &inv_impulse_world, &speed_change_B_world,
                                &rot_change_B_in_B);
        ivp_core_get_surface_speed_on_test(core_B, &B_push_pos_in_B, &speed_change_B_world, &rot_change_B_in_B,
                                           &sur_speed_B_world);
        lvm_vimult3_f(m_world_f_B, &sur_speed_B_world, &sur_speed_B_in_B);
    }

    if (push_vec_idx >= 3) {
        /* rot push A (IVP_Core::test_rot_push_core_multiple_cs) */
        ivp_vec3f_t rot_change_A_in_A, h, iri;
        iri = ivp_core_inv_rot_inertia_f(core_A);
        lvf_set_pairwise_mult(&h, &impulse_A, &iri);
        lvf_set_multiple(&rot_change_A_in_A, &h, rot_impulse_strength);
        lvm_vmult3_f(&m_B_from_A, &rot_change_A_in_A, &rot_change_A_in_B);
        lvf_zero(&speed_change_A_in_B);

        /* rot push B (contrary) */
        iri = ivp_core_inv_rot_inertia_f(core_B);
        lvf_set_pairwise_mult(&h, &impulse_B, &iri);
        lvf_set_multiple(&rot_change_B_in_B, &h, inv_rot_impulse_strength);
        lvf_zero(&speed_change_B_world);
        ivp_core_get_surface_speed_on_test(core_B, &B_push_pos_in_B, &speed_change_B_world, &rot_change_B_in_B,
                                           &sur_speed_B_world);
        lvm_vimult3_f(m_world_f_B, &sur_speed_B_world, &sur_speed_B_in_B);
    }

    /* push_vec_idx in the reduced matrix */
    int pv_mat_idx = 0;
    for (int dim = 0; dim < push_vec_idx; dim++) {
        if (!cs->constraint_is_disabled[dim]) pv_mat_idx++;
    }
    IVP_ASSERT(pv_mat_idx >= 0); /* all constraints disabled !? */

    ivp_gmmz_t *g_mat = &b->tmp_matrix;
    int gm_col = b->n_constraints * A_obj_idx + pv_mat_idx;
    int gm_row = 0;

    for (int fill_app_idx = 0; fill_app_idx < cs->n_wheel_objects; fill_app_idx++) {
        ivp_vec3f_t delta_speed_change, delta_rot_change;
        if (fill_app_idx == A_obj_idx) {
            lvf_subtract2(&delta_speed_change, &speed_change_A_in_B, &sur_speed_B_in_B);
            lvf_subtract2(&delta_rot_change, &rot_change_A_in_B, &rot_change_B_in_B);
        } else {
            /* indirect changes (through the motion of B) */
            const ivp_vec3r_t *app_center_world =
                &cs->wheel_objects[fill_app_idx]->real_object->physical_core->m_world_f_core_last_psi.pos;
            ivp_vec3f_t app_center_B, B_at_app_speed_world;
            lvm_vimult4_pf(m_world_f_B, app_center_world, &app_center_B);
            ivp_core_get_surface_speed_on_test(core_B, &app_center_B, &speed_change_B_world, &rot_change_B_in_B,
                                               &B_at_app_speed_world);
            lvm_vimult3_f(m_world_f_B, &B_at_app_speed_world, &delta_speed_change);
            lvf_mult(&delta_speed_change, -1.0f);
            lvf_set_negative(&delta_rot_change, &rot_change_B_in_B);
        }

        if (!cs->constraint_is_disabled[0]) ivp_gmmz_set_value(g_mat, (double)delta_speed_change.k[0], gm_col, gm_row++);
        if (!cs->constraint_is_disabled[1]) ivp_gmmz_set_value(g_mat, (double)delta_speed_change.k[1], gm_col, gm_row++);
        if (!cs->constraint_is_disabled[2]) ivp_gmmz_set_value(g_mat, (double)delta_speed_change.k[2], gm_col, gm_row++);
        if (!cs->constraint_is_disabled[3]) ivp_gmmz_set_value(g_mat, (double)delta_rot_change.k[0], gm_col, gm_row++);
        if (!cs->constraint_is_disabled[4]) ivp_gmmz_set_value(g_mat, (double)delta_rot_change.k[1], gm_col, gm_row++);
        if (!cs->constraint_is_disabled[5]) ivp_gmmz_set_value(g_mat, (double)delta_rot_change.k[2], gm_col, gm_row++);
    }
}

static ivp_result_t builder_calc_constraint_matrix(car_builder_t *b)
{
    ivp_constraint_solver_car_t *cs = b->car_solver;
    int gm_size = b->n_constraints * b->n_appends;
    if (gm_size <= 0) return IVP_ERR_INVALID_ARG;

    ivp_result_t r = ivp_gmmz_init(&b->tmp_matrix, gm_size, NULL);
    if (r != IVP_OK) return r;
    b->tmp_matrix.matrix_eps = (double)P_DOUBLE_EPS;
    b->tmp_matrix.aligned_row_len = gm_size;   /* ivp_constraint_car.cxx:737 */

    for (int app_idx = 0; app_idx < b->n_appends; app_idx++) {
        for (int vec_idx = 0; vec_idx < 6; vec_idx++) {
            if (cs->constraint_is_disabled[vec_idx]) continue;
            builder_calc_pushing_behavior(b, app_idx, vec_idx);
        }
    }

    r = ivp_gmmz_init(&cs->co_matrix, gm_size, NULL);
    if (r != IVP_OK) {
        ivp_gmmz_destroy(&b->tmp_matrix, NULL);
        return r;
    }
    cs->co_matrix_allocated = true;
    ivp_result_t ret_val = ivp_gmmz_invert(&b->tmp_matrix, &cs->co_matrix);
    ivp_gmmz_destroy(&b->tmp_matrix, NULL);
    cs->co_matrix.matrix_eps = (double)P_DOUBLE_EPS;
    return ret_val;
}

ivp_result_t ivp_constraint_solver_car_init_constraint_system(ivp_constraint_solver_car_t *s, ivp_environment_t *env,
                                                              ivp_object_t *body, ivp_object_t *const *wheels,
                                                              const ivp_vec3f_t *wheel_pos_Bos, int n_wheels)
{
    if (!s || !env || !body || (n_wheels > 0 && !wheels)) return IVP_ERR_INVALID_ARG;
    if (s->body_object || s->n_wheel_objects) return IVP_ERR_INVALID_STATE;
    if (n_wheels > IVP_CONSTRAINT_CAR_MAX_WHEELS) {
        ivp_message("IVP_Constraint_Solver_Car::init_constraint_system: wheel count %d exceeds max %d\n",
                    n_wheels, IVP_CONSTRAINT_CAR_MAX_WHEELS);
        return IVP_ERR_INVALID_ARG;
    }
    if (n_wheels < 1) return IVP_ERR_INVALID_ARG;
    for (int i = 0; i < n_wheels; i++) {
        if (!wheels[i] || wheels[i]->owner != env || wheels[i]->physical_core->physical_unmoveable) return IVP_ERR_INVALID_ARG;
    }
    if (body->owner != env || body->physical_core->physical_unmoveable) return IVP_ERR_INVALID_ARG;

    s->environment = env;
    s->environment_dying = false;

    s->body_object = car_object_create(s, body, NULL, NULL);
    if (!s->body_object) {
        s->environment = NULL;
        return IVP_ERR_OUT_OF_MEMORY;
    }
    if (!ivp_pvec_reserve(&s->cores_of_constraint_system, 1)) {
        car_object_destroy(s->body_object, true);
        s->body_object = NULL;
        s->environment = NULL;
        return IVP_ERR_OUT_OF_MEMORY;
    }
    int added = ivp_pvec_add(&s->cores_of_constraint_system, body->physical_core);
    IVP_ASSERT(added >= 0);
    (void)added;

    ivp_result_t res = IVP_OK;
    for (int i = 0; i < n_wheels; i++) {
        ivp_constraint_car_object_t *o =
            car_object_create(s, wheels[i], body, wheel_pos_Bos ? &wheel_pos_Bos[i] : NULL);
        if (!o) {
            res = IVP_ERR_OUT_OF_MEMORY;
            break;
        }
        s->wheel_objects[s->n_wheel_objects++] = o;
        if (!ivp_pvec_reserve(&s->cores_of_constraint_system,
                              ivp_pvec_len(&s->cores_of_constraint_system) + 1)) {
            res = IVP_ERR_OUT_OF_MEMORY;
            break;
        }
        added = ivp_pvec_add(&s->cores_of_constraint_system, wheels[i]->physical_core);
        IVP_ASSERT(added >= 0);
        (void)added;
        s->c_local_ballsocket[i] = NULL;
    }

    if (res == IVP_OK) {
        car_builder_t builder;
        builder_init(&builder, s);
        builder_disable_constraint(&builder, s->y_idx);     /* free Y trans */
        builder_disable_constraint(&builder, s->x_idx + 3); /* free X rot */
        builder_disable_constraint(&builder, s->y_idx + 3); /* free Y rot */
        builder_disable_constraint(&builder, s->z_idx + 3); /* free Z rot */
        res = builder_calc_constraint_matrix(&builder);
    }

    if (res == IVP_OK) {
        if (!ivp_ctrl_mgr_announce_controller_to_environment(env, &s->base)) {
            res = IVP_ERR_OUT_OF_MEMORY;
        } else {
            s->announced = true;
            s->env_listener = ivp_environment_add_event_listener(env, solver_env_event, s);
            if (!ivp_event_listener_handle_is_valid(s->env_listener)) {
                res = IVP_ERR_OUT_OF_MEMORY;
            }
        }
    }
    if (res != IVP_OK) {
        /* This also removes a controller that was announced before its
         * environment-deletion listener failed to register. */
        solver_detach(s, true);
        if (s->co_matrix_allocated) {
            ivp_gmmz_destroy(&s->co_matrix, NULL);
            s->co_matrix_allocated = false;
        }
        s->environment = NULL;
    }
    return res;
}

/* ── simulation ────────────────────────────────────────────────────────── */

/* IVP_Constraint_Solver_Car::do_simulation_controller_rotation */
static void solver_do_simulation_controller_rotation(ivp_constraint_solver_car_t *s, ivp_sim_es_t *es,
                                                     ivp_core_t *core_B, const ivp_transform_r_t *m_world_f_B)
{
    const int x_idx = s->x_idx, y_idx = s->y_idx, z_idx = s->z_idx;
    for (int app_cnt = 0; app_cnt < s->n_wheel_objects; app_cnt++) {
        ivp_constraint_car_object_t *app = s->wheel_objects[app_cnt];
        ivp_core_t *core_A = app->real_object->physical_core;
        const ivp_transform_r_t *m_world_f_A = &core_A->m_world_f_core_last_psi;

        /* target vector in A */
        ivp_vec3r_t target_vec_as, tv_bs, tv_ws;
        lvm_get_col(&app->target_position_bs, x_idx, &tv_bs);
        lvm_vmult3(m_world_f_B, &tv_bs, &tv_ws);
        lvm_vimult3(m_world_f_A, &tv_ws, &target_vec_as);

        D cur_angles_B[2];
        {
            /* IVP_FAST_WHEELS_ENABLED: no turn back of the target vector */
            D ny = target_vec_as.k[z_idx];
            D nx = target_vec_as.k[y_idx];
            cur_angles_B[0] = (D)s->angle_sign * -ivp_atan2(ny, target_vec_as.k[x_idx]);
            cur_angles_B[1] = (D)s->angle_sign * ivp_atan2(nx, target_vec_as.k[x_idx]);
        }

        ivp_vec3f_t y_rot_axis_world2, z_rot_axis_world2;
        car_get_col_f(m_world_f_A, y_idx, &y_rot_axis_world2);
        car_get_col_f(m_world_f_A, z_idx, &z_rot_axis_world2);

        ivp_car_scr_t tcb;
        ivp_car_scr_init_rotation_ws(&tcb, core_B, core_A, &y_rot_axis_world2, &z_rot_axis_world2, NULL);

        D ia1, ia2, ib1, ib2;
        const D tpm00 = tcb.m[0][0];
        const D tpm01 = tcb.m[0][1];
        const D tpm10 = tcb.m[1][0];
        const D tpm11 = tcb.m[1][1];
        /* reference (dimhotepus) argument order */
        if (!ivp_car_invert_2x2_matrix(tpm00, tpm01, tpm10, tpm11, &ia1, &ib1, &ia2, &ib2)) {
            ivp_message("do_constraint_system: Couldn't invert rot matrix!\n");
            return;
        }

        D a = -(D)es->i_delta_time * cur_angles_B[0] - (D)tcb.delta_velocity_ds.k[0];
        D b = -(D)es->i_delta_time * cur_angles_B[1] - (D)tcb.delta_velocity_ds.k[1];

        ivp_vec3f_t rot_impulse_ds;
        rot_impulse_ds.k[0] = (float)(a * ia1 + b * ib1);
        rot_impulse_ds.k[1] = (float)(a * ia2 + b * ib2);
        rot_impulse_ds.k[2] = 0.0f;

        ivp_car_scr_exert_angular_impulse_dim2(&tcb, core_B, core_A, &rot_impulse_ds);
    }
}

/* plan B: one tense ball-and-socket constraint per wheel */
static void solver_start_plan_B(ivp_constraint_solver_car_t *s)
{
    ivp_constraint_car_object_t *body = s->body_object;
    int local_constraint_count = s->n_wheel_objects;
    if (local_constraint_count > IVP_CONSTRAINT_CAR_MAX_WHEELS) {
        IVP_ASSERT(0);
        local_constraint_count = IVP_CONSTRAINT_CAR_MAX_WHEELS;
    }
    ivp_environment_t *env = s->environment;
    for (int app_nr = 0; app_nr < local_constraint_count; app_nr++) {
        ivp_constraint_car_object_t *app = s->wheel_objects[app_nr];

        const ivp_transform_r_t *pm_ws_f_Bcs = &body->real_object->physical_core->m_world_f_core_last_psi;
        ivp_transform_r_t m_ws_f_Aos;
        ivp_object_calc_at_matrix(app->real_object, ivp_env_get_current_time(env), &m_ws_f_Aos);

        ivp_vec3r_t anchorB_Bos, anchorB_ws, anchorB_Aos;
        anchorB_Bos = app->target_position_bs.pos;
        lvm_vmult4(pm_ws_f_Bcs, &anchorB_Bos, &anchorB_ws);
        lvm_vimult4(&m_ws_f_Aos, &anchorB_ws, &anchorB_Aos);
        ivp_vec3r_t anchorA_Aos;
        lvp_zero(&anchorA_Aos);

        ivp_constraint_state_t *c = (ivp_constraint_state_t *)calloc(1, sizeof(*c));
        if (!c) continue;
        ivp_constraint_init_ball_socket_tense_Ros(c, app->real_object, &anchorA_Aos, body->real_object, &anchorB_Aos);
        s->c_local_ballsocket[app_nr] = c;
        s->c_local_ballsocket_handle[app_nr] = ivp_environment_add_controller(env, IVP_CP_CONSTRAINTS,
                                                                              ivp_controller_constraint, c);
    }
    s->local_translation_in_use = true;
}

/* IVP_Constraint_Solver_Car::do_simulation_controller */
static void solver_do_simulation_controller(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *core_list)
{
    (void)core_list;
    ivp_constraint_solver_car_t *s = solver_of(c);
    if (!s->announced || !s->body_object) return;
    const int x_idx = s->x_idx, z_idx = s->z_idx;
    D inv_dtime = (D)es->i_delta_time;

    /* pushes body and appendices so that total impulse is the same and all
     * parts fulfil the specified constraints in the next PSI */
    ivp_constraint_car_object_t *body = s->body_object;
    ivp_core_t *core_B = body->real_object->physical_core;
    const ivp_transform_r_t *m_world_f_B = &core_B->m_world_f_core_last_psi;

    /*********** ROTATION ************/
    solver_do_simulation_controller_rotation(s, es, core_B, m_world_f_B);

    /*********** TRANSLATION ************/
    double *input_vec_ptr = s->co_matrix.desired_vector;
    IVP_ASSERT(s->co_matrix.columns == s->n_wheel_objects * 2);

    int init_local_translation = 0;
    int invalid_count = 0; /* for plan B removal */
    float delta_time = (float)es->delta_time;

    for (int app_cnt = 0; app_cnt < s->n_wheel_objects; app_cnt++) {
        ivp_constraint_car_object_t *app = s->wheel_objects[app_cnt];
        ivp_core_t *core_A = app->real_object->physical_core;
        const ivp_transform_r_t *m_world_f_A = &core_A->m_world_f_core_last_psi;

        /**** translation divergence (after delta time) ***/
        ivp_vec3r_t delta_speed_vec;
        {
            /* current positions in B */
            ivp_vec3f_t cur_pos_A_in_B;
            ivp_vec3f_t cur_pos_B_in_B;
            lvf_set_p(&cur_pos_B_in_B, &app->target_position_bs.pos);
            lvm_vimult4_pf(m_world_f_B, &m_world_f_A->pos, &cur_pos_A_in_B); /* center of app */

            /* speed and rot speed in B */
            ivp_vec3f_t surspeed_A_in_B, surspeed_B_in_world, surspeed_B_in_B;
            ivp_vec3f_t speed_A = ivp_core_speed_f(core_A);
            lvm_vimult3_f(m_world_f_B, &speed_A, &surspeed_A_in_B);
            ivp_core_get_surface_speed(core_B, &cur_pos_B_in_B, &surspeed_B_in_world);
            lvm_vimult3_f(m_world_f_B, &surspeed_B_in_world, &surspeed_B_in_B);

            /* positions at the next PSI in B */
            ivp_vec3f_t next_pos_A_in_B, next_pos_B_in_B;
            lvf_add_multiple2(&next_pos_A_in_B, &cur_pos_A_in_B, &surspeed_A_in_B, (D)delta_time);
            lvf_add_multiple2(&next_pos_B_in_B, &cur_pos_B_in_B, &surspeed_B_in_B, (D)delta_time);

            /* delta speed vec that has to be realized */
            lvp_subtract2_ff(&delta_speed_vec, &next_pos_A_in_B, &next_pos_B_in_B);
            lvp_mult(&delta_speed_vec, inv_dtime * 1.0f);

            /* difference too high? */
            D quad_speed = lvp_quad_length(&delta_speed_vec);
            if (quad_speed > (D)(s->max_delta_speed * s->max_delta_speed)) {
                invalid_count++;
                s->psis_left_for_plan_B = 10;
                if (!s->local_translation_in_use) {
                    /* switch to ball and socket local solution */
                    init_local_translation = 1;
                    break;
                }
            }
        }

        /**** add to input vector for matrix mult ***/
        IVP_ASSERT(s->constraint_is_disabled[s->y_idx]);
        IVP_ASSERT(s->constraint_is_disabled[x_idx + 3]);
        IVP_ASSERT(s->constraint_is_disabled[s->y_idx + 3]);
        IVP_ASSERT(s->constraint_is_disabled[z_idx + 3]);
        *input_vec_ptr++ = (double)delta_speed_vec.k[x_idx];
        *input_vec_ptr++ = (double)delta_speed_vec.k[z_idx];
    }

    if (invalid_count == 0 && s->local_translation_in_use) {
        /* switch back to regular translation */
        s->psis_left_for_plan_B--;
        if (s->psis_left_for_plan_B < 0) {
            solver_delete_local_ballsockets(s, false);
            s->local_translation_in_use = false;
            IVP_IF(1) { /* ivp_constraint_car.cxx:412 */
                ivp_debug_printf("plan B deactivated.\n");
            }
        }
    }

    if (init_local_translation) {
        IVP_IF(1) { /* ivp_constraint_car.cxx:421 */
            ivp_debug_printf("plan B activated.\n");
        }
        /* appendices are too far away -> plan B (local constraints) */
        solver_start_plan_B(s);
    }

    /* pushes that have to be performed */
    ivp_gmmz_mult(&s->co_matrix);

    if (!s->local_translation_in_use) {
        /*** now push objects ***/
        const double *res_vec_ptr = s->co_matrix.result_vector;
        for (int a_cnt = 0; a_cnt < s->n_wheel_objects; a_cnt++) {
            ivp_constraint_car_object_t *app = s->wheel_objects[a_cnt];
            ivp_core_t *core_A = app->real_object->physical_core;
            const ivp_transform_r_t *m_world_f_A = &core_A->m_world_f_core_last_psi;

            /* translation push */
            ivp_vec3f_t t_impulse_ws, t_impulse_bs;
            t_impulse_bs.k[x_idx] = (float)res_vec_ptr[0];
            t_impulse_bs.k[s->y_idx] = 0.0f;
            t_impulse_bs.k[z_idx] = (float)res_vec_ptr[1];
            lvm_vmult3_f(m_world_f_B, &t_impulse_bs, &t_impulse_ws);
            res_vec_ptr += 2;

            ivp_vec3r_t target_position_ws;
            lvm_vmult4(m_world_f_B, &app->target_position_bs.pos, &target_position_ws);

            /* body */
            ivp_vec3r_t imp;
            lvp_set_f(&imp, &t_impulse_ws);
            ivp_core_push_ws(core_B, &target_position_ws, &imp);

            /* appendix (center push) */
            lvf_mult(&t_impulse_ws, -1.0f);
            lvp_set_f(&imp, &t_impulse_ws);
            ivp_core_push_ws(core_A, &m_world_f_A->pos, &imp);
        }
    }
}
