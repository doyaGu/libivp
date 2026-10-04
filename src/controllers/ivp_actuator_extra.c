/* ivp_actuator_extra.c -- IVP_Actuator_Extra port
 * (ivp_controller/ivp_actuator.cxx:67-80, 431-494, 828, 882-1026).
 */
#include <ivp/controllers/ivp_actuator_extra.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>

#include <ivp/base/ivp_assert.h>

#include <string.h>
#include "controllers/ivp_controller_ref.h"

void ivp_extra_info_init(ivp_extra_info_t *info)
{
    /* IVP_Extra_Info::IVP_Extra_Info: every member NULL / 0 / FALSE */
    memset(info, 0, sizeof(*info));
}

void ivp_template_extra_init(ivp_template_extra_t *templ)
{
    memset(templ, 0, sizeof(*templ));
    ivp_extra_info_init(&templ->info);
}

/* IVP_Actuator_Extra::active_float_changed: "for bombs", does nothing */
static void extra_active_float_changed(void *ctx, ivp_active_float_t *af,
                                       double old_value, double new_value)
{
    (void)ctx;
    (void)af;
    (void)old_value;
    (void)new_value;
}

static void extra_add_dependency(ivp_actuator_extra_state_t *state, ivp_active_float_t *af)
{
    if (af) (void)ivp_active_float_add_listener(af, extra_active_float_changed, state);
}

static void extra_remove_dependency(ivp_actuator_extra_state_t *state, ivp_active_float_t *af)
{
    if (af) (void)ivp_active_float_remove_listener(af, extra_active_float_changed, state);
}

static void extra_wake(ivp_actuator_extra_state_t *state)
{
    /* ensure_actuator_in_simulation */
    ivp_controller_wake_actuator(state->obj_a, state->obj_b);
}

void ivp_actuator_extra_init(ivp_actuator_extra_state_t *state, const ivp_template_extra_t *templ)
{
    memset(state, 0, sizeof(*state));
    if (!templ) return;
    state->client_data = templ->client_data;
    state->obj_a = templ->objects[0];
    state->obj_b = templ->objects[1];
    /* IVP_Anchor::core_pos is an IVP_U_Float_Point */
    {
        ivp_vec3f_t c0 = ivp_ctl_f(&templ->anchors_cs[0]), c1 = ivp_ctl_f(&templ->anchors_cs[1]);
        state->anchor_a_os = ivp_ctl_r(&c0);
        state->anchor_b_os = ivp_ctl_r(&c1);
        /* the template gives core coordinates: object_pos = m_object_f_core * core_pos */
        ivp_object_t *objs[2] = { state->obj_a, state->obj_b };
        const ivp_vec3f_t *cs[2] = { &c0, &c1 };
        for (int i = 0; i < 2; i++) {
            ivp_vec3f_t object_pos = *cs[i];
            if (objs[i]) {
                ivp_transform_r_t m_core_f_object;
                ivp_ctl_calc_m_core_f_object(objs[i], &m_core_f_object);
                lvm_vimult4_ff(&m_core_f_object, cs[i], &object_pos);
            }
            ivp_ctl_anchor_set(&state->anchors[i], objs[i], &object_pos);
        }
    }
    state->info = templ->info;

    extra_add_dependency(state, state->info.active_float_force);
    extra_add_dependency(state, state->info.active_float_bomb);
    extra_add_dependency(state, state->info.mod_fc_height);
    extra_add_dependency(state, state->info.mod_fc_target_height);
    extra_add_dependency(state, state->info.mod_fc_dist);
    extra_add_dependency(state, state->info.mod_fc_speed);
    state->dependencies_installed = true;

    /* floating camera */
    state->current_float_cam_pos.k[0] = (double)0.0f; /* start position */
    state->current_float_cam_pos.k[1] = (double)-100.0f;
    state->current_float_cam_pos.k[2] = (double)0.0f;
    memset(&state->current_look_point, 0, sizeof(state->current_look_point));
    state->current_vals_are_set = false; /* flag for first time */
    extra_wake(state);
}

void ivp_actuator_extra_destroy(ivp_actuator_extra_state_t *state)
{
    if (!state || !state->dependencies_installed) return;
    state->dependencies_installed = false;
    extra_remove_dependency(state, state->info.mod_fc_height);
    extra_remove_dependency(state, state->info.mod_fc_target_height);
    extra_remove_dependency(state, state->info.mod_fc_dist);
    extra_remove_dependency(state, state->info.mod_fc_speed);
    extra_remove_dependency(state, state->info.active_float_force);
    extra_remove_dependency(state, state->info.active_float_bomb);
}

double ivp_actuator_extra_get_force(const ivp_actuator_extra_state_t *state)
{
    if (!state || !state->info.active_float_force) return (double)0.0f;
    return ivp_active_float_get(state->info.active_float_force); /* give_double_value */
}

const ivp_vec3d_t *ivp_actuator_extra_get_float_cam_props(ivp_actuator_extra_state_t *state,
                                                          double *fc_height_out,
                                                          double *fc_target_height_out,
                                                          double *fc_dist_out,
                                                          double *fc_speed_out)
{
    const ivp_extra_info_t *info = &state->info;
    *fc_height_out = info->mod_fc_height ? ivp_active_float_get(info->mod_fc_height) : 0.0;
    *fc_target_height_out = info->mod_fc_target_height ? ivp_active_float_get(info->mod_fc_target_height) : 0.0;
    *fc_dist_out = info->mod_fc_dist ? ivp_active_float_get(info->mod_fc_dist) : 0.0;
    *fc_speed_out = info->mod_fc_speed ? ivp_active_float_get(info->mod_fc_speed) : 0.0;
    return &state->current_float_cam_pos;
}

/* ── IVP_U_Point helpers (double) ──────────────────────────────────────── */

/* m_world_f_core_PSI->vmult4(&core_pos, &pos_ws) into a double point */
static void anchor_ws_d(const ivp_object_t *obj, const ivp_vec3r_t *anchor_cs, ivp_vec3d_t *out)
{
    if (!obj) {
        for (int i = 0; i < 3; i++) out->k[i] = (double)anchor_cs->k[i];
        return;
    }
    const ivp_transform_r_t *m = &obj->physical_core->m_world_f_core_last_psi;
    for (int i = 0; i < 3; i++) {
        out->k[i] = (double)m->rot.rows[i].k[0] * (double)anchor_cs->k[0] +
                    (double)m->rot.rows[i].k[1] * (double)anchor_cs->k[1] +
                    (double)m->rot.rows[i].k[2] * (double)anchor_cs->k[2] + (double)m->pos.k[i];
    }
}

/* IVP_U_Point::normize: isqrt(quad_length, 3); IVP_FAULT (untouched) below eps */
static void point_normize(ivp_vec3d_t *v)
{
    double q = v->k[0] * v->k[0] + v->k[1] * v->k[1] + v->k[2] * v->k[2];
    if (q < (double)IVP_DOUBLE_EPS) return;
    double f = ivp_isqrt_double(q, 3);
    v->k[0] *= f;
    v->k[1] *= f;
    v->k[2] *= f;
}

/* IVP_U_Point::set_interpolate */
static void point_set_interpolate(ivp_vec3d_t *res, const ivp_vec3d_t *p0, const ivp_vec3d_t *p1, double s)
{
    double is = (double)1.0f - s;
    double a = p0->k[0] * is;
    double a2 = p1->k[0] * s;
    double b = p0->k[1] * is;
    double b2 = p1->k[1] * s;
    a += a2;
    double c = p0->k[2] * is;
    double c2 = p1->k[2] * s;
    b += b2;
    res->k[0] = a;
    res->k[1] = b;
    res->k[2] = c + c2;
}

static void point_cross(ivp_vec3d_t *out, const ivp_vec3d_t *v1, const ivp_vec3d_t *v2)
{
    double x = v1->k[1] * v2->k[2] - v1->k[2] * v2->k[1];
    double y = v1->k[2] * v2->k[0] - v1->k[0] * v2->k[2];
    double z = v1->k[0] * v2->k[1] - v1->k[1] * v2->k[0];
    out->k[0] = x;
    out->k[1] = y;
    out->k[2] = z;
}

void ivp_actuator_extra_calc_float_cam_matrix(const ivp_actuator_extra_state_t *state,
                                              ivp_transform_r_t *cam_matrix_out)
{
    /* calc camera matrix from current fc values */
    IVP_ASSERT(state->info.is_float_cam);

    ivp_vec3d_t cam_dir;
    for (int i = 0; i < 3; i++) cam_dir.k[i] = state->current_look_point.k[i] - state->current_float_cam_pos.k[i];
    point_normize(&cam_dir);

    ivp_vec3d_t hp;
    hp.k[0] = (double)0.05f; /* to zenit */
    hp.k[1] = (double)-1.0f;
    hp.k[2] = (double)0.05f;
    ivp_vec3d_t v0, v1;
    point_cross(&v0, &cam_dir, &hp);
    point_normize(&v0);
    point_cross(&v1, &cam_dir, &v0);
    point_normize(&v1);

    /* init_columns4(&v0, &v1, &cam_dir, &current_float_cam_pos) */
    for (int i = 0; i < 3; i++) {
        cam_matrix_out->rot.rows[i].k[0] = (ivp_real_t)v0.k[i];
        cam_matrix_out->rot.rows[i].k[1] = (ivp_real_t)v1.k[i];
        cam_matrix_out->rot.rows[i].k[2] = (ivp_real_t)cam_dir.k[i];
        cam_matrix_out->pos.k[i] = (ivp_real_t)state->current_float_cam_pos.k[i];
    }
}

/* ── do_puck_force / do_float_cam ──────────────────────────────────────── */

static void extra_do_puck_force(ivp_actuator_extra_state_t *state, double dtime)
{
    ivp_core_t *pc0 = state->obj_a ? state->obj_a->friction_core : NULL; /* l_anchor_object->friction_core */
    ivp_core_t *pc1 = state->obj_b ? state->obj_b->friction_core : NULL;

    const ivp_vec3f_t core_pos0 = ivp_ctl_f(&state->anchor_a_os);
    const ivp_vec3f_t core_pos1 = ivp_ctl_f(&state->anchor_b_os);
    ivp_vec3r_t pos0_ws, pos1_ws;
    ivp_ctl_anchor_pos_ws(pc0, &core_pos0, &pos0_ws);
    ivp_ctl_anchor_pos_ws(pc1, &core_pos1, &pos1_ws);

    ivp_vec3f_t dir_ws;
    lvf_subtract2_p(&dir_ws, &pos0_ws, &pos1_ws); /* force direction world coords */
    dir_ws.k[1] = 0;
    (void)lvf_fast_normize(&dir_ws);

    {
        double force = 0; /* (dlen - this->info.spring_len) * this->info.spring_constant; */

        ivp_vec3f_t force_vector; /* the force */
        lvf_set_multiple(&force_vector, &dir_ws, (D)(force * dtime));

        if (pc1 && ivp_controller_core_is_simulated(pc1) && !pc1->pinned) {
            ivp_ctl_async_push_core_ws(pc1, &pos1_ws, &force_vector);
        }
        if (pc0 && ivp_controller_core_is_simulated(pc0) && !pc0->pinned) {
            lvf_mult(&force_vector, (D)-1.0);
            ivp_ctl_async_push_core_ws(pc0, &pos0_ws, &force_vector);
        }
    }
}

static void extra_do_float_cam(ivp_actuator_extra_state_t *state, double d_time)
{
    /* could be called per psi: keep the controlled cores awake */
    ivp_controller_wake_object(state->obj_a);
    if (state->obj_b != state->obj_a) ivp_controller_wake_object(state->obj_b);

    IVP_ASSERT(state->info.is_float_cam);

    double fc_height, fc_target_height, fc_dist, fc_speed;
    const ivp_vec3d_t *fc_pos = ivp_actuator_extra_get_float_cam_props(state, &fc_height, &fc_target_height,
                                                                       &fc_dist, &fc_speed);

    /* what direction does actuator currently take ? (physical cores) */
    ivp_vec3d_t pos0_ws, pos1_ws, dir_ws;
    anchor_ws_d(state->obj_a, &state->anchor_a_os, &pos0_ws);
    anchor_ws_d(state->obj_b, &state->anchor_b_os, &pos1_ws);
    for (int i = 0; i < 3; i++) dir_ws.k[i] = pos0_ws.k[i] - pos1_ws.k[i];

    point_normize(&dir_ws);
    dir_ws.k[1] = (double)0.0f; /* proj on y */

    /* where does current camera want to be ? */
    ivp_vec3d_t cur_pos;
    for (int i = 0; i < 3; i++) cur_pos.k[i] = pos0_ws.k[i] + dir_ws.k[i] * fc_dist;
    cur_pos.k[1] -= fc_height;

    /* what position shall camera take ? */
    ivp_vec3d_t from = *fc_pos;
    point_set_interpolate(&state->current_float_cam_pos, &from, &cur_pos, fc_speed * d_time);

    /* what direction shall camera look along ? (smoothing avoids jumps) */
    ivp_vec3d_t look_point;
    for (int i = 0; i < 3; i++) look_point.k[i] = pos0_ws.k[i] + dir_ws.k[i] * (double)0.0f;
    look_point.k[1] -= fc_target_height;
    if (!state->current_vals_are_set) {
        state->current_vals_are_set = true;
        state->current_look_point = look_point;
    } else {
        ivp_vec3d_t old = state->current_look_point;
        point_set_interpolate(&state->current_look_point, &old, &look_point, fc_speed);
    }
}

void ivp_controller_actuator_extra(const ivp_event_sim_t *event_sim,
                                   ivp_object_t *obj,
                                   void *ctx)
{
    ivp_actuator_extra_state_t *state = (ivp_actuator_extra_state_t *)ctx;
    if (!state) return;
    ivp_object_t *ends[2] = { state->obj_a, state->obj_b };
    if (obj != ivp_controller_pick_exec_object(ends, 2)) return;

    if (state->info.is_puck_force) {
        extra_do_puck_force(state, (double)event_sim->delta_time);
        return;
    }
    if (state->info.is_float_cam) {
        extra_do_float_cam(state, (double)event_sim->delta_time);
        return;
    }
}

int ivp_actuator_extra_discovery(void *ctx, ivp_environment_t *env,
                                 ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_actuator_extra_state_t *s = (const ivp_actuator_extra_state_t *)ctx;
    if (!s) return 0;
    /* IVP_Actuator_Two_Point: core0 (if movable), core1 (if movable and != core0) */
    ivp_object_t *objs[2] = { s->obj_a, (s->obj_b != s->obj_a) ? s->obj_b : NULL };
    return ivp_controller_discover_objects(objs, 2, out_handles, max_handles);
}
