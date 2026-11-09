/* ivp_buoyancy_solver.c -- port of IVP_Buoyancy_Solver
 * (ivp_controller/ivp_buoyancy_solver.cxx).
 *
 * Reproduces the reference evaluation types: IVP_FLOAT members/locals and
 * IVP_U_Float_Point / IVP_U_Float_Hesse vectors are float (float op float
 * stays float), IVP_DOUBLE locals and IVP_U_Point / IVP_U_Hesse are D
 * (ivp_real_t), IVP_Inline_Math::ivp_sqrtf / sqrtd / fabsd act on D, and
 * IVP_Inline_Math::fast_asin / save_acosf on float.  Object space
 * transforms use the object's cache matrix like IVP_Real_Object::
 * transform_*_coords.
 */
#include "ivp_buoyancy_solver.h"

#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_compact_triangle.h>
#include <ivp/geometry/ivp_surman.h>

#include <ivp/base/ivp_assert.h>

#include <math.h>
#include <string.h>

#include "controllers/ivp_controller_ref.h"
#include "dynamics/ivp_env_internal.h"

#ifdef IVP_REAL_DOUBLE
#define BS_FABSD(x) fabs(x)  /* IVP_Inline_Math::fabsd */
#define BS_SQRTD(x) sqrt(x)  /* IVP_Inline_Math::ivp_sqrtf / sqrtd */
#else
#define BS_FABSD(x) fabsf(x)
#define BS_SQRTD(x) sqrtf(x)
#endif

#define BS_EPS_PARALLEL 0.000000001f /* P_EPS_PARALLEL (ivu_geometry.hxx) */

/* ── IVP_Inline_Math (float) ──────────────────────────────────────────── */

static inline float bs_fast_asin(float angle)
{
    float quad_angle = angle * angle;
    float triple_angle = angle * quad_angle;
    float quint_angle = triple_angle * quad_angle;
    return angle + 0.12f * triple_angle + 0.29f * quint_angle;
}

/* IVP_Inline_Math::save_acosf: acosd (double acos) since the reference
 * 7b9e404, rounded to float */
static inline float bs_save_acosf(float angle)
{
    return angle >= 1.0f ? 0.0f : (float)acos((double)angle);
}

/* ── IVP_U_Float_Hesse ────────────────────────────────────────────────── */

static inline D fh_get_dist(const ivp_bs_hesse_t *h, const ivp_vec3f_t *p)
{
    return lvf_dot(&h->n, p) + (D)h->hesse_val;
}

static void fh_proj_on_plane(const ivp_bs_hesse_t *h, const ivp_vec3f_t *p, ivp_vec3f_t *result)
{
    D ql = lvf_quad_length(&h->n);
    if (ql < P_DOUBLE_EPS) {
        *result = *p;
        return;
    }
    lvf_add_multiple2(result, p, &h->n, -fh_get_dist(h, p) / ql);
}

static inline void fh_calc_hesse_val(ivp_bs_hesse_t *h, const ivp_vec3f_t *p0)
{
    h->hesse_val = (float)(-lvf_dot(&h->n, p0));
}

static void fh_normize(ivp_bs_hesse_t *h)
{
    D len = lvf_real_length(&h->n);
    if (len < P_DOUBLE_EPS) {
        lvf_zero(&h->n);
        h->hesse_val = 0.0f;
        return;
    }
    D l = 1.0f / len;
    lvf_mult(&h->n, l);
    h->hesse_val = (float)((D)h->hesse_val * l);
}

/* ── IVP_U_Hesse / IVP_U_Plain / IVP_U_Straight (D) ──────────────────── */

static void uh_proj_on_plane(const lv_hesse_t *h, const ivp_vec3r_t *p, ivp_vec3r_t *result)
{
    D ql = lvp_quad_length(&h->n);
    if (ql < P_DOUBLE_EPS) {
        *result = *p;
        return;
    }
    lvp_add_multiple2(result, p, &h->n, -lv_hesse_get_dist(h, p) / ql);
}

/* IVP_U_Point::is_parallel */
static bool up_is_parallel(const ivp_vec3r_t *v1, const ivp_vec3r_t *v2, D eps)
{
    ivp_vec3r_t c;
    lvp_cross(&c, v1, v2);
    return lvp_quad_length(&c) <= eps * eps * lvp_quad_length(v1) * lvp_quad_length(v2);
}

/* IVP_U_Hesse::calc_intersect_with(straight, point_out) */
static bool uh_calc_intersect_with_straight(const lv_hesse_t *h, const ivp_vec3r_t *start_point,
                                            const ivp_vec3r_t *vec, ivp_vec3r_t *point_out)
{
    D dist1 = lv_hesse_get_dist(h, start_point);
    ivp_vec3r_t hp = *start_point;
    lvp_add(&hp, vec);
    D dist2 = lv_hesse_get_dist(h, &hp);
    D delta_dist = dist2 - dist1;
    if (BS_FABSD(delta_dist) < P_DOUBLE_EPS) {
        if (dist1 >= P_DOUBLE_EPS) return false; /* true parallel */
        *point_out = *start_point;
        return true;
    }
    D factor = dist1 / delta_dist;
    lvp_add_multiple2(point_out, start_point, vec, -factor);
    return true;
}

/* IVP_U_Plain::calc_intersect_with(plane2, straight_out); plain_start is
 * the start_point of the IVP_U_Plain(hesse) constructor. */
static bool plain_calc_intersect_with(const lv_hesse_t *plain, const ivp_vec3r_t *plain_start,
                                      const lv_hesse_t *plane2,
                                      ivp_vec3r_t *straight_start_out, ivp_vec3r_t *straight_vec_out)
{
    if (up_is_parallel(&plain->n, &plane2->n, (D)BS_EPS_PARALLEL)) return false;

    lvp_cross(straight_vec_out, &plain->n, &plane2->n);
    (void)lvp_normize(straight_vec_out);

    ivp_vec3r_t hp, hp2;
    uh_proj_on_plane(plane2, plain_start, &hp);
    uh_proj_on_plane(plain, &hp, &hp2);

    ivp_vec3r_t straight_vec;
    lvp_subtract2(&straight_vec, &hp2, plain_start);
    (void)lvp_normize(&straight_vec);

    return uh_calc_intersect_with_straight(plane2, plain_start, &straight_vec, straight_start_out);
}

/* IVP_U_Point::solve_quadratic_equation_accurate */
static void up_solve_quadratic_equation_accurate(ivp_vec3r_t *r, const ivp_vec3r_t *p)
{
    if (BS_FABSD(p->k[0]) < P_DOUBLE_EPS) {
        r->k[0] = 0;
        if (BS_FABSD(p->k[1]) < P_DOUBLE_EPS) {
            r->k[0] = -1.0f;
            return;
        }
        r->k[1] = r->k[2] = -p->k[2] / p->k[1];
        return;
    }
    D b2 = p->k[1] * p->k[1];
    D ac4 = p->k[0] * p->k[2] * 4.0f;
    r->k[0] = b2 - ac4;
    if (r->k[0] >= 0) {
        D i2a = .5f / p->k[0];
        D sr = BS_SQRTD(r->k[0]);
        D l1 = (-p->k[1] - sr) * i2a;
        D l2 = (-p->k[1] + sr) * i2a;
        r->k[1] = l1;
        r->k[2] = l2;
    }
}

/* ── object helpers ───────────────────────────────────────────────────── */

/* object->get_cache_object_no_lock()->m_world_f_object */
static inline const ivp_transform_r_t *bs_m_world_f_object(ivp_object_t *object, ivp_transform_r_t *tmp)
{
    return ivp_ctl_cache_m_world_f_object(object, tmp);
}

/* reference ivp_core_get_surface_speed_os */
static void core_get_surface_speed_os(ivp_core_t *pc, ivp_object_t *object,
                                      const ivp_vec3f_t *point_os, ivp_vec3f_t *speed_out_os)
{
    ivp_transform_r_t tmp;
    const ivp_transform_r_t *m_world_f_object = bs_m_world_f_object(object, &tmp);

    ivp_vec3r_t point_ws;
    lvm_vmult4_fp(m_world_f_object, point_os, &point_ws);   /* transform_position_to_world_coords */

    ivp_vec3f_t point_core;
    lvm_vimult4_pf(&pc->m_world_f_core_last_psi, &point_ws, &point_core);

    ivp_vec3f_t speed_core;
    lvf_cross(&speed_core, &pc->rot_speed, &point_core);

    ivp_vec3f_t speed_out_ws;
    lvm_vmult3_f(&pc->m_world_f_core_last_psi, &speed_core, &speed_out_ws);
    ivp_vec3f_t center_speed = ivp_core_speed_f(pc);
    lvf_add(&speed_out_ws, &center_speed);

    lvm_vimult3_f(m_world_f_object, &speed_out_ws, speed_out_os); /* transform_vector_to_object_coords */

    IVP_IF(0) { /* ivp_buoyancy_solver.cxx:49 */
        if ((BS_FABSD((D)speed_out_os->k[0]) > (D)1E1f) ||
            (BS_FABSD((D)speed_out_os->k[1]) > (D)1E1f) ||
            (BS_FABSD((D)speed_out_os->k[2]) > (D)1E1f)) {
            IVP_CORE();
        }
    }
}

/* IVP_U_Float_Point::set_interpolate (ivu_linear.cxx): this = p1 * s + p0 * (1 - s) */
static void bs_set_interpolate(ivp_vec3f_t *r, const ivp_vec3f_t *p0, const ivp_vec3f_t *p1, D s)
{
    D is = 1.0f - s;
    D a = p0->k[0] * is;
    D a2 = p1->k[0] * s;
    D b = p0->k[1] * is;
    D b2 = p1->k[1] * s;
    a += a2;
    D c = p0->k[2] * is;
    D c2 = p1->k[2] * s;
    b += b2;
    r->k[0] = (float)a;
    r->k[1] = (float)b;
    r->k[2] = (float)(c + c2);
}

/* IVP_IF(1) draw vector of the surface intersection of a triangle
 * (ivp_buoyancy_solver.cxx:1248 / 1319): edge between the two disection
 * points sp1, sp2 (environment->add_draw_vector, color 4) */
static void bs_draw_disection_edge(ivp_buoyancy_solver_t *bs, ivp_object_t *object,
                                   const ivp_vec3f_t *p_start, const ivp_vec3f_t *p_1, const ivp_vec3f_t *p_2,
                                   D factor1, D factor2)
{
    ivp_vec3f_t sp1, sp2;
    ivp_vec3r_t sp1_ws, sp2_ws;
    bs_set_interpolate(&sp1, p_start, p_1, factor1);
    bs_set_interpolate(&sp2, p_start, p_2, factor2);

    ivp_transform_r_t tmp;
    lvm_vmult4_fp(bs_m_world_f_object(object, &tmp), &sp1, &sp1_ws); /* transform_position_to_world_coords */
    lvm_vmult4_fp(bs_m_world_f_object(object, &tmp), &sp2, &sp2_ws);

    ivp_vec3f_t edge12;
    lvf_subtract2_p(&edge12, &sp1_ws, &sp2_ws);
    const double tmp_sp2_ws[3] = { (double)sp2_ws.k[0], (double)sp2_ws.k[1], (double)sp2_ws.k[2] };
    ivp_env_add_draw_vector(bs->core->environment, tmp_sp2_ws, edge12.k, "", 4);
}

/* ── init ─────────────────────────────────────────────────────────────── */

void ivp_buoyancy_solver_init(ivp_buoyancy_solver_t *bs,
                              ivp_core_t *core,
                              const ivp_buoyancy_state_t *t,
                              const ivp_vec3f_t *resulting_speed_of_current_ws)
{
    memset(bs, 0, sizeof(*bs));
    bs->core = core;
    bs->simulate_wing_behavior = t->simulate_wing_behavior;
    bs->buoyancy_eps = (float)t->buoyancy_eps;
    bs->medium_density = (float)t->fluid_density;
    bs->pressure_damp_factor = (float)t->linear_drag * 0.5f * bs->medium_density;
    bs->friction_damp_factor = (float)t->friction_damp_factor * 0.5f * bs->medium_density;
    bs->torque_factor = (float)t->torque_factor;
    bs->ball_rot_dampening_factor = (float)t->angular_drag;
    bs->viscosity_input_factor = (float)t->viscosity_input_factor;
    bs->object_visible_surface_content_under = 0.0f;
    lvf_zero(&bs->sum_impulse_x_point);
    lvf_zero(&bs->sum_impulse_x_movevector);
    lvf_zero(&bs->sum_impulse);
    bs->viscosity_factor = (float)t->viscosity_factor;
    bs->resulting_speed_of_current_ws = *resulting_speed_of_current_ws;
    bs->volume_under = 0;
    lvf_zero(&bs->volume_center_under);
}

/* ── ball ─────────────────────────────────────────────────────────────── */

/* reference compute_disection_points_with_ball */
static int disection_points_with_ball(const ivp_bs_hesse_t *plane1_os, const ivp_bs_hesse_t *plane2_os,
                                      const ivp_vec3f_t *geom_center_os, float radius,
                                      ivp_vec3f_t *p1_os, ivp_vec3f_t *p2_os)
{
    /* convert plane2 from hesse form into point-direction form */
    lv_hesse_t tmp_plane2_os;
    lvp_set_f(&tmp_plane2_os.n, &plane2_os->n);
    tmp_plane2_os.hesse_val = plane2_os->hesse_val;

    /* IVP_U_Plain(&tmp_plane2_os): start point = projection of the origin */
    ivp_vec3r_t plain_start, origin;
    lvp_zero(&origin);
    uh_proj_on_plane(&tmp_plane2_os, &origin, &plain_start);

    lv_hesse_t non_float_plane1_os;
    lvp_set_f(&non_float_plane1_os.n, &plane1_os->n);
    non_float_plane1_os.hesse_val = plane1_os->hesse_val;

    ivp_vec3r_t straight_start, straight_dir;
    lvp_zero(&straight_start);
    lvp_zero(&straight_dir);
    if (!plain_calc_intersect_with(&tmp_plane2_os, &plain_start, &non_float_plane1_os,
                                   &straight_start, &straight_dir)) {
        return 0; /* both planes are parallel */
    }

    /* calc intersection of straight with ball */
    ivp_vec3f_t straight_start_point, straight_vec;
    lvf_set_p(&straight_start_point, &straight_start);
    lvf_set_p(&straight_vec, &straight_dir);
    ivp_vec3f_t T;
    lvf_subtract2(&T, &straight_start_point, geom_center_os);

    ivp_vec3r_t parameters;
    parameters.k[0] = straight_vec.k[0] * straight_vec.k[0] + straight_vec.k[1] * straight_vec.k[1] +
                      straight_vec.k[2] * straight_vec.k[2];
    parameters.k[1] = 2 * T.k[0] * straight_vec.k[0] + 2 * T.k[1] * straight_vec.k[1] +
                      2 * T.k[2] * straight_vec.k[2];
    parameters.k[2] = T.k[0] * T.k[0] + T.k[1] * T.k[1] + T.k[2] * T.k[2] - radius * radius;

    ivp_vec3r_t result;
    lvp_zero(&result);
    up_solve_quadratic_equation_accurate(&result, &parameters);

    if (result.k[0] >= 0.0f) {
        lvf_add_multiple2(p1_os, &straight_start_point, &straight_vec, result.k[1]);
        lvf_add_multiple2(p2_os, &straight_start_point, &straight_vec, result.k[2]);
        return 1;
    }
    lvf_zero(p1_os);
    lvf_zero(p2_os);
    return 0;
}

/* reference compute_buoyancy_values_for_one_ball */
static void ball_buoyancy(ivp_buoyancy_solver_t *bs, int decision, float distance, float radius,
                          const ivp_bs_hesse_t *surface_os, const ivp_vec3f_t *geom_center_os)
{
    bs->volume_under = 0;
    lvf_zero(&bs->volume_center_under);
    switch (decision) {
    case 0:
        bs->volume_under = (float)((4.0f / 3.0f) * IVP_PI * (radius * radius * radius));
        bs->volume_center_under = *geom_center_os;
        break;
    case 1: case 2: case 3: case 4: {
        float height_of_balls_part_under_water = radius + distance;
        float h = height_of_balls_part_under_water;
        bs->volume_under = (float)((IVP_PI * h * h * (3 * radius - h)) / 3.0f);
        float volume_center_under_factor = 0.75f * (4.0f * radius * radius - 4.0f * radius * h + h * h) /
                                           (3.0f * radius - h);
        bs->volume_center_under = *geom_center_os;
        lvf_add_multiple(&bs->volume_center_under, &surface_os->n, (D)volume_center_under_factor);
        break;
    }
    default:
        break;
    }
}

/* reference compute_dampening_values_for_one_ball */
static void ball_dampening(ivp_buoyancy_solver_t *bs, int decision, float distance, float radius,
                           const ivp_vec3f_t *geom_center_os, const ivp_vec3f_t *rel_speed_of_current_os_,
                           const ivp_bs_hesse_t *speed_plane_os_, const ivp_bs_hesse_t *surface_os,
                           const ivp_vec3f_t *p1_os, const ivp_vec3f_t *p2_os)
{
    const float buoyancy_eps = bs->buoyancy_eps;

    ivp_vec3f_t ball_point_of_impulse;
    lvf_zero(&ball_point_of_impulse);
    ivp_vec3f_t ball_impulse_vector;
    lvf_zero(&ball_impulse_vector);

    ivp_vec3f_t rel_speed_of_current_os = *rel_speed_of_current_os_;
    ivp_bs_hesse_t speed_plane_os = *speed_plane_os_;

    D ball_projected_surface_content_under = 0.0f;

    switch (decision) {
    case 0:
        ball_projected_surface_content_under = IVP_PI * radius * radius;
        ball_point_of_impulse = *geom_center_os;
        break;

    case 1: {
        /* center above the surface, speed plane cuts the ball under and above */
        ivp_vec3f_t direction_down;
        fh_proj_on_plane(&speed_plane_os, &surface_os->n, &direction_down);
        if (lvf_quad_length(&direction_down) < buoyancy_eps) goto case_2;
        (void)lvf_normize(&direction_down);

        ivp_vec3f_t p1p2_os;
        lvf_subtract2(&p1p2_os, p2_os, p1_os);
        ivp_vec3f_t Q_os = *p1_os;
        lvf_add_multiple(&Q_os, &p1p2_os, 0.5f);

        D length_p1p2 = lvf_real_length(&p1p2_os);
        if (length_p1p2 < buoyancy_eps) {
            ball_projected_surface_content_under = 0.0f;
            break;
        }
        ivp_bs_hesse_t p1p2_hesse_os;
        p1p2_hesse_os.n = p1p2_os;
        p1p2_hesse_os.hesse_val = 0.0f;
        fh_calc_hesse_val(&p1p2_hesse_os, &Q_os);
        fh_normize(&p1p2_hesse_os);

        ivp_vec3f_t tmp_O1_os, tmp_O2_os;
        lvf_zero(&tmp_O1_os);
        lvf_zero(&tmp_O2_os);
        (void)disection_points_with_ball(surface_os, &p1p2_hesse_os, geom_center_os, radius, &tmp_O1_os, &tmp_O2_os);

        ivp_vec3f_t tmp_O1O2_os;
        lvf_subtract2(&tmp_O1O2_os, &tmp_O2_os, &tmp_O1_os);
        ivp_vec3f_t O_os;
        lvf_zero(&O_os);
        if (lvf_dot(&speed_plane_os.n, &tmp_O1O2_os) >= 0.0f) {
            fh_proj_on_plane(&speed_plane_os, &tmp_O1_os, &O_os);
        } else {
            fh_proj_on_plane(&speed_plane_os, &tmp_O2_os, &O_os);
        }

        D factor = lvf_dot(&surface_os->n, &speed_plane_os.n);
        if (factor <= buoyancy_eps) {
            /* ball falls into the medium: circle segment under the surface */
            ivp_vec3f_t MQ_os;
            lvf_subtract2(&MQ_os, &Q_os, geom_center_os);
            D height = radius - lvf_real_length(&MQ_os);
            D circle_arc = 2 * bs_fast_asin((float)(length_p1p2 / (2.0f * radius))) * radius;
            D surface_content_down = BS_FABSD(0.5f * (circle_arc * radius - length_p1p2 * (radius - height)));

            ivp_vec3f_t center_of_segment_os;
            lvf_zero(&center_of_segment_os);
            if (surface_content_down < buoyancy_eps) {
                lvf_zero(&center_of_segment_os);
            } else {
                if (factor > -buoyancy_eps) {
                    /* speed plane orthogonal on the surface */
                    ball_point_of_impulse = *geom_center_os;
                    lvf_add_multiple(&ball_point_of_impulse, &surface_os->n,
                                     length_p1p2 * length_p1p2 * length_p1p2 / (12 * surface_content_down));
                    ball_projected_surface_content_under = surface_content_down;
                    goto calculate_impulse;
                }
                center_of_segment_os = *geom_center_os;
                lvf_add_multiple(&center_of_segment_os, &direction_down,
                                 length_p1p2 * length_p1p2 * length_p1p2 / (12 * surface_content_down));
            }

            /* part of the ellipse above the surface */
            ivp_vec3f_t projected_center_on_surface_os, center_of_ellipse_os;
            fh_proj_on_plane(surface_os, geom_center_os, &projected_center_on_surface_os);
            fh_proj_on_plane(&speed_plane_os, &projected_center_on_surface_os, &center_of_ellipse_os);

            ivp_vec3f_t edge_O_center_os;
            lvf_subtract2(&edge_O_center_os, &center_of_ellipse_os, &O_os);
            D b = lvf_real_length(&edge_O_center_os);
            ivp_vec3f_t edge_Q_center_os;
            lvf_subtract2(&edge_Q_center_os, &center_of_ellipse_os, &Q_os);
            D part_b = lvf_real_length(&edge_Q_center_os);
            D radius_of_water_disection = BS_SQRTD(BS_FABSD(radius * radius - distance * distance));

            D surface_content_of_cut_off_ellipse_segment;
            ivp_vec3f_t center_of_part_ellipse_os;
            D surface_content_up;
            if (b < buoyancy_eps) {
                surface_content_of_cut_off_ellipse_segment = 0;
                surface_content_up = 0.0f;
                lvf_zero(&center_of_part_ellipse_os);
            } else {
                surface_content_of_cut_off_ellipse_segment =
                    BS_FABSD((radius_of_water_disection * b * bs_save_acosf((float)(part_b / b)) -
                              part_b * (lvf_real_length(&p1p2_os) * 0.5f)));

                D surface_content_of_whole_ellipse = (IVP_PI * radius_of_water_disection * b);
                surface_content_up = BS_FABSD(surface_content_of_whole_ellipse - surface_content_of_cut_off_ellipse_segment);

                D h = b - part_b;
                D s = 2.0f * BS_SQRTD(BS_FABSD(2 * h * b - h * h));
                D arc = BS_FABSD(2 * bs_fast_asin((float)(s / (2.0f * b))) * b);
                D A = BS_FABSD(0.5f * (arc * b - s * (b - h)));

                ivp_vec3f_t center_of_cut_off_part_of_ellipse;
                if (A < buoyancy_eps) {
                    lvf_zero(&center_of_cut_off_part_of_ellipse);
                    surface_content_of_cut_off_ellipse_segment = 0.0f;
                } else {
                    (void)lvf_normize(&edge_O_center_os);
                    center_of_cut_off_part_of_ellipse = center_of_ellipse_os;
                    lvf_add_multiple(&center_of_cut_off_part_of_ellipse, &edge_O_center_os, s * s * s / (12 * A));
                }

                if (surface_content_up < buoyancy_eps) {
                    lvf_zero(&center_of_part_ellipse_os);
                    surface_content_up = 0.0f;
                } else {
                    lvf_set_multiple(&center_of_part_ellipse_os, &center_of_cut_off_part_of_ellipse,
                                     -surface_content_of_cut_off_ellipse_segment);
                    lvf_add_multiple(&center_of_part_ellipse_os, &center_of_ellipse_os, surface_content_of_whole_ellipse);
                    lvf_mult(&center_of_part_ellipse_os, 1.0f / surface_content_up);
                }
            }

            ball_projected_surface_content_under = surface_content_up + surface_content_down;
            if (ball_projected_surface_content_under < buoyancy_eps) {
                ball_projected_surface_content_under = 0.0f;
            } else {
                lvf_set_multiple(&ball_point_of_impulse, &center_of_segment_os, surface_content_down);
                lvf_add_multiple(&ball_point_of_impulse, &center_of_part_ellipse_os, surface_content_up);
                lvf_mult(&ball_point_of_impulse, 1.0f / ball_projected_surface_content_under);
            }
        } else {
            /* ball flies out of the water */
            ivp_vec3f_t projected_center_on_surface_os, center_of_ellipse_os;
            fh_proj_on_plane(surface_os, geom_center_os, &projected_center_on_surface_os);
            fh_proj_on_plane(&speed_plane_os, &projected_center_on_surface_os, &center_of_ellipse_os);

            ivp_vec3f_t edge_O_center_os;
            lvf_subtract2(&edge_O_center_os, &center_of_ellipse_os, &O_os);
            D b = lvf_real_length(&edge_O_center_os);
            ivp_vec3f_t edge_Q_center_os;
            lvf_subtract2(&edge_Q_center_os, &center_of_ellipse_os, &Q_os);
            D radius_of_water_disection = BS_SQRTD(BS_FABSD(radius * radius - distance * distance));
            D height = BS_FABSD(b - lvf_real_length(&edge_Q_center_os));

            ivp_vec3f_t MQ_os;
            lvf_subtract2(&MQ_os, &Q_os, geom_center_os);
            D h = radius - lvf_real_length(&MQ_os);
            D arc = 2 * bs_fast_asin((float)(length_p1p2 / (2.0f * radius))) * radius;
            D surface_content_of_circle_segment = BS_FABSD(0.5f * (arc * radius - length_p1p2 * (radius - h)));

            if (surface_content_of_circle_segment < buoyancy_eps) {
                ball_projected_surface_content_under = 0.0f;
                goto calculate_impulse;
            }

            D surface_content_of_ellipse_segment;
            if (b < buoyancy_eps) {
                surface_content_of_ellipse_segment = 0.0f;
            } else {
                surface_content_of_ellipse_segment =
                    b * radius_of_water_disection * bs_save_acosf((float)(lvf_real_length(&edge_Q_center_os) / b)) -
                    lvf_real_length(&edge_Q_center_os) * (length_p1p2 * 0.5f);
            }

            ball_projected_surface_content_under = surface_content_of_circle_segment - surface_content_of_ellipse_segment;
            if (ball_projected_surface_content_under < buoyancy_eps) {
                ball_projected_surface_content_under = 0.0f;
                goto calculate_impulse;
            }

            ivp_vec3f_t center_of_circle_segment = *geom_center_os;
            lvf_add_multiple(&center_of_circle_segment, &direction_down,
                             length_p1p2 * length_p1p2 * length_p1p2 / (12 * surface_content_of_circle_segment));

            ivp_vec3f_t center_of_ellipse_segment;
            D s = 2.0f * BS_SQRTD(BS_FABSD(2.0f * height * b - height * height));
            D ellipse_arc = 2 * bs_fast_asin((float)(s / (2.0f * b))) * b;
            D A = 0.5f * (ellipse_arc * b - s * (b - height));
            if ((b < buoyancy_eps) || (A < buoyancy_eps)) {
                lvf_zero(&center_of_ellipse_segment);
            } else {
                center_of_ellipse_segment = center_of_ellipse_os;
                lvf_add_multiple(&center_of_ellipse_segment, &direction_down, s * s * s / (12.0f * A));
            }

            lvf_set_multiple(&ball_point_of_impulse, &center_of_ellipse_segment, -surface_content_of_ellipse_segment);
            lvf_add_multiple(&ball_point_of_impulse, &center_of_circle_segment, surface_content_of_circle_segment);
            lvf_mult(&ball_point_of_impulse, 1.0f / ball_projected_surface_content_under);
        }
        break;
    }

    case 2: {
    case_2:;
        /* center above the surface, speed plane cuts the ball only above */
        D factor = lvf_dot(&surface_os->n, &speed_plane_os.n);
        if (factor < 0.0f) {
            ivp_vec3f_t projected_center_on_surface_os;
            fh_proj_on_plane(surface_os, geom_center_os, &projected_center_on_surface_os);
            fh_proj_on_plane(&speed_plane_os, &projected_center_on_surface_os, &ball_point_of_impulse);
            ball_projected_surface_content_under = BS_FABSD(factor * IVP_PI * (radius * radius - distance * distance));
        } else {
            ball_projected_surface_content_under = 0.0f;
        }
        break;
    }

    case 3: {
        /* center under the surface, speed plane cuts the ball under and above */
        ivp_vec3f_t direction_up;
        fh_proj_on_plane(&speed_plane_os, &surface_os->n, &direction_up);
        lvf_mult(&direction_up, -1.0f);
        if (lvf_quad_length(&direction_up) < buoyancy_eps) goto case_4;
        (void)lvf_normize(&direction_up);

        ivp_vec3f_t p1p2_os;
        lvf_subtract2(&p1p2_os, p2_os, p1_os);
        ivp_vec3f_t Q_os = *p1_os;
        lvf_add_multiple(&Q_os, &p1p2_os, 0.5f);

        D length_p1p2 = lvf_real_length(&p1p2_os);
        if (length_p1p2 < buoyancy_eps) {
            ball_projected_surface_content_under = IVP_PI * radius * radius;
            ball_point_of_impulse = *geom_center_os;
            break;
        }
        ivp_bs_hesse_t p1p2_hesse_os;
        p1p2_hesse_os.n = p1p2_os;
        p1p2_hesse_os.hesse_val = 0.0f;
        fh_calc_hesse_val(&p1p2_hesse_os, &Q_os);
        fh_normize(&p1p2_hesse_os);

        ivp_vec3f_t tmp_O1_os, tmp_O2_os;
        lvf_zero(&tmp_O1_os);
        lvf_zero(&tmp_O2_os);
        (void)disection_points_with_ball(surface_os, &p1p2_hesse_os, geom_center_os, radius, &tmp_O1_os, &tmp_O2_os);
        ivp_vec3f_t tmp_O1O2_os;
        lvf_subtract2(&tmp_O1O2_os, &tmp_O2_os, &tmp_O1_os);
        ivp_vec3f_t O_os;
        lvf_zero(&O_os);
        if (lvf_dot(&speed_plane_os.n, &tmp_O1O2_os) >= 0.0f) {
            fh_proj_on_plane(&speed_plane_os, &tmp_O1_os, &O_os);
        } else {
            fh_proj_on_plane(&speed_plane_os, &tmp_O2_os, &O_os);
        }

        ivp_vec3f_t projected_center_on_surface_os, center_of_ellipse_os;
        fh_proj_on_plane(surface_os, geom_center_os, &projected_center_on_surface_os);
        fh_proj_on_plane(&speed_plane_os, &projected_center_on_surface_os, &center_of_ellipse_os);

        D factor = lvf_dot(&surface_os->n, &speed_plane_os.n);
        if (factor <= buoyancy_eps) {
            /* ball falls into the medium */
            ivp_vec3f_t MQ_os;
            lvf_subtract2(&MQ_os, &Q_os, geom_center_os);
            D h = radius - lvf_real_length(&MQ_os);
            D arc = 2 * bs_fast_asin((float)(length_p1p2 / (2.0f * radius))) * radius;

            D cut_off_part_above_surface = BS_FABSD(0.5f * (arc * radius - length_p1p2 * (radius - h)));
            D surface_content_down = BS_FABSD(IVP_PI * radius * radius - cut_off_part_above_surface);

            IVP_ASSERT(surface_content_down > buoyancy_eps);

            ivp_vec3f_t center_of_segment_os;
            lvf_zero(&center_of_segment_os);

            if (cut_off_part_above_surface < buoyancy_eps) {
                ball_projected_surface_content_under = IVP_PI * radius * radius;
                ball_point_of_impulse = *geom_center_os;
            } else {
                if (factor > -buoyancy_eps) {
                    ivp_vec3f_t center_of_cut_off_part_above_surface = *geom_center_os;
                    lvf_add_multiple(&center_of_cut_off_part_above_surface, &surface_os->n,
                                     -(length_p1p2 * length_p1p2 * length_p1p2 / (12 * cut_off_part_above_surface)));
                    lvf_set_multiple(&center_of_segment_os, &center_of_cut_off_part_above_surface, -cut_off_part_above_surface);
                    lvf_add_multiple(&center_of_segment_os, geom_center_os, IVP_PI * radius * radius);
                    lvf_mult(&center_of_segment_os, 1.0f / surface_content_down);

                    ball_projected_surface_content_under = surface_content_down;
                    ball_point_of_impulse = center_of_segment_os;
                    goto calculate_impulse;
                }
                ivp_vec3f_t center_of_cut_off_part_above_surface = *geom_center_os;
                lvf_add_multiple(&center_of_cut_off_part_above_surface, &direction_up,
                                 (length_p1p2 * length_p1p2 * length_p1p2) / (12 * cut_off_part_above_surface));
                lvf_set_multiple(&center_of_segment_os, &center_of_cut_off_part_above_surface, -cut_off_part_above_surface);
                lvf_add_multiple(&center_of_segment_os, geom_center_os, IVP_PI * radius * radius);
                lvf_mult(&center_of_segment_os, 1.0f / surface_content_down);
            }

            /* part of the ellipse above the surface */
            ivp_vec3f_t edge_O_center_os;
            lvf_subtract2(&edge_O_center_os, &center_of_ellipse_os, &O_os);
            D b = lvf_real_length(&edge_O_center_os);
            ivp_vec3f_t edge_Q_center_os;
            lvf_subtract2(&edge_Q_center_os, &center_of_ellipse_os, &Q_os);
            D radius_of_water_disection = BS_SQRTD(BS_FABSD(radius * radius - distance * distance));
            D height = BS_FABSD(b - lvf_real_length(&edge_Q_center_os));

            ivp_vec3f_t center_of_ellipse_segment;
            D surface_content_up;
            if (b < buoyancy_eps) {
                surface_content_up = 0.0f;
                lvf_zero(&center_of_ellipse_segment);
            } else {
                surface_content_up = b * radius_of_water_disection *
                                         bs_save_acosf((float)(lvf_real_length(&edge_Q_center_os) / b)) -
                                     lvf_real_length(&edge_Q_center_os) * (0.5f * length_p1p2);

                D s = 2 * BS_SQRTD(BS_FABSD(2 * height * b - height * height));
                D ellipse_arc = 2 * bs_fast_asin((float)(s / (2 * b))) * b;
                D A = BS_FABSD(0.5f * (ellipse_arc * b - s * (b - height)));
                if (A < buoyancy_eps) {
                    surface_content_up = 0.0f;
                    lvf_zero(&center_of_ellipse_segment);
                } else {
                    center_of_ellipse_segment = center_of_ellipse_os;
                    lvf_add_multiple(&center_of_ellipse_segment, &direction_up, s * s * s / (12 * A));
                }
            }

            ball_projected_surface_content_under = surface_content_up + surface_content_down;
            if (ball_projected_surface_content_under < buoyancy_eps) {
                ball_projected_surface_content_under = 0.0f;
                lvf_zero(&ball_point_of_impulse);
            } else {
                lvf_set_multiple(&ball_point_of_impulse, &center_of_segment_os, surface_content_down);
                lvf_add_multiple(&ball_point_of_impulse, &center_of_ellipse_segment, surface_content_up);
                lvf_mult(&ball_point_of_impulse, 1.0f / ball_projected_surface_content_under);
            }
        } else {
            /* ball flies out of the water */
            ivp_vec3f_t edge_O_center_os;
            lvf_subtract2(&edge_O_center_os, &center_of_ellipse_os, &O_os);
            D b = lvf_real_length(&edge_O_center_os);
            ivp_vec3f_t edge_Q_center_os;
            lvf_subtract2(&edge_Q_center_os, &center_of_ellipse_os, &Q_os);
            D radius_of_water_disection = BS_SQRTD(BS_FABSD(radius * radius - distance * distance));
            D height = BS_FABSD(b - lvf_real_length(&edge_Q_center_os));

            ivp_vec3f_t MQ_os;
            lvf_subtract2(&MQ_os, &Q_os, geom_center_os);
            D h = radius - lvf_real_length(&MQ_os);
            D arc = 2 * bs_fast_asin((float)(length_p1p2 / (2.0f * radius))) * radius;
            D surface_content_of_circle_segment = BS_FABSD(0.5f * (arc * radius - length_p1p2 * (radius - h)));

            if (surface_content_of_circle_segment < buoyancy_eps) goto case_4;

            D surface_content_of_ellipse_segment;
            if (b < buoyancy_eps) {
                surface_content_of_ellipse_segment = 0.0f;
            } else {
                surface_content_of_ellipse_segment =
                    b * radius_of_water_disection * bs_save_acosf((float)(lvf_real_length(&edge_Q_center_os) / b)) -
                    lvf_real_length(&edge_Q_center_os) * (length_p1p2 * 0.5f);
            }

            D part_ring_above_water = BS_FABSD(surface_content_of_circle_segment - surface_content_of_ellipse_segment);

            ball_projected_surface_content_under =
                BS_FABSD(IVP_PI * (radius * radius - radius_of_water_disection * b) - part_ring_above_water);
            if (ball_projected_surface_content_under < buoyancy_eps) {
                ball_projected_surface_content_under = 0.0f;
                goto calculate_impulse;
            }

            ivp_vec3f_t center_of_circle_segment = *geom_center_os;
            lvf_add_multiple(&center_of_circle_segment, &direction_up,
                             length_p1p2 * length_p1p2 * length_p1p2 / (12 * surface_content_of_circle_segment));

            ivp_vec3f_t center_of_circle_segment_under_surface;
            lvf_set_multiple(&center_of_circle_segment_under_surface, &center_of_circle_segment,
                             -surface_content_of_circle_segment);
            lvf_add_multiple(&center_of_circle_segment_under_surface, geom_center_os, IVP_PI * radius * radius);
            D surface_content_of_circle_segment_under_surface =
                BS_FABSD(IVP_PI * radius * radius - surface_content_of_circle_segment);
            if (surface_content_of_circle_segment_under_surface < buoyancy_eps) {
                surface_content_of_circle_segment_under_surface = 0.0f;
                lvf_zero(&center_of_circle_segment_under_surface);
            } else {
                lvf_mult(&center_of_circle_segment_under_surface, 1.0f / surface_content_of_circle_segment_under_surface);
            }

            ivp_vec3f_t center_of_ellipse_segment;
            D s = 2.0f * BS_SQRTD(BS_FABSD(2 * height * b - height * height));
            D ellipse_arc = 2 * bs_fast_asin((float)(s / (2 * b))) * b;
            D A = 0.5f * (ellipse_arc * b - s * (b - height));
            if ((b < buoyancy_eps) || (A < buoyancy_eps)) {
                lvf_zero(&center_of_ellipse_segment);
            } else {
                center_of_ellipse_segment = center_of_ellipse_os;
                lvf_add_multiple(&center_of_ellipse_segment, &direction_up, s * s * s / (12 * A));
            }

            ivp_vec3f_t center_of_ellipse_segment_under_surface;
            lvf_set_multiple(&center_of_ellipse_segment_under_surface, &center_of_ellipse_segment,
                             -surface_content_of_ellipse_segment);
            lvf_add_multiple(&center_of_ellipse_segment_under_surface, &center_of_ellipse_os,
                             IVP_PI * radius_of_water_disection * b);
            D surface_content_of_ellipse_segment_under_surface =
                BS_FABSD(IVP_PI * radius_of_water_disection * b - surface_content_of_ellipse_segment);
            if (surface_content_of_ellipse_segment_under_surface < buoyancy_eps) {
                lvf_zero(&center_of_ellipse_segment_under_surface);
                surface_content_of_ellipse_segment_under_surface = 0.0f;
            } else {
                lvf_mult(&center_of_ellipse_segment_under_surface, 1.0f / surface_content_of_ellipse_segment_under_surface);
            }

            lvf_set_multiple(&ball_point_of_impulse, &center_of_ellipse_segment_under_surface,
                             -surface_content_of_ellipse_segment_under_surface);
            lvf_add_multiple(&ball_point_of_impulse, &center_of_circle_segment_under_surface,
                             surface_content_of_circle_segment_under_surface);
            lvf_mult(&ball_point_of_impulse, 1.0f / ball_projected_surface_content_under);
        }
        break;
    }

    case 4: {
    case_4:;
        /* center under the surface, speed plane cuts the ball completely under */
        D factor = lvf_dot(&surface_os->n, &speed_plane_os.n);
        if (factor <= 0.0f) {
            ball_projected_surface_content_under = IVP_PI * radius * radius;
            ball_point_of_impulse = *geom_center_os;
        } else {
            D surface_content_circle = IVP_PI * radius * radius;
            D surface_content_ellipse = factor * IVP_PI * (radius * radius - distance * distance);
            ball_projected_surface_content_under = BS_FABSD(surface_content_circle - surface_content_ellipse);

            if (ball_projected_surface_content_under < buoyancy_eps) {
                ball_projected_surface_content_under = 0.0f;
                goto calculate_impulse;
            }

            ivp_vec3f_t projected_center_on_surface_os, projected_center_on_ellipse_os;
            fh_proj_on_plane(surface_os, geom_center_os, &projected_center_on_surface_os);
            fh_proj_on_plane(&speed_plane_os, &projected_center_on_surface_os, &projected_center_on_ellipse_os);

            lvf_set_multiple(&ball_point_of_impulse, &projected_center_on_ellipse_os, -surface_content_ellipse);
            lvf_add_multiple(&ball_point_of_impulse, geom_center_os, surface_content_circle);
            lvf_mult(&ball_point_of_impulse, 1.0f / ball_projected_surface_content_under);
        }
        break;
    }

    default:
        break;
    }

calculate_impulse:;
    D quad_length_of_speed = lvf_quad_length(&rel_speed_of_current_os);
    if (quad_length_of_speed < buoyancy_eps) {
        quad_length_of_speed = 0.0f;
        lvf_set3(&rel_speed_of_current_os, 0.0f, 1.0f, 0.0f);
    }
    D pressure_dampening_impulse = ball_projected_surface_content_under * quad_length_of_speed * bs->pressure_damp_factor;
    D friction_dampening_impulse = (3.0f * ball_projected_surface_content_under) * quad_length_of_speed *
                                   bs->friction_damp_factor;

    (void)lvf_normize(&rel_speed_of_current_os);
    lvf_set_multiple(&ball_impulse_vector, &rel_speed_of_current_os, pressure_dampening_impulse + friction_dampening_impulse);

    bs->object_visible_surface_content_under =
        (float)((D)bs->object_visible_surface_content_under + ball_projected_surface_content_under);

    ivp_vec3f_t impulse_x_point;
    lvf_cross(&impulse_x_point, &ball_point_of_impulse, &ball_impulse_vector);
    lvf_add(&bs->sum_impulse_x_point, &impulse_x_point);

    ivp_vec3f_t move_direction_os;
    lvf_zero(&move_direction_os);
    ivp_vec3f_t impulse_x_movevector;
    lvf_cross(&impulse_x_movevector, &move_direction_os, &ball_impulse_vector);
    lvf_add(&bs->sum_impulse_x_movevector, &impulse_x_movevector);

    lvf_add(&bs->sum_impulse, &ball_impulse_vector);
}

/* reference compute_values_for_one_ball */
static void values_for_one_ball(ivp_buoyancy_solver_t *bs, const ivp_object_t *object,
                                const ivp_bs_hesse_t *surface_os, const ivp_vec3f_t *rel_speed_of_current_os)
{
    ivp_vec3f_t geom_center_os;
    lvf_zero(&geom_center_os);
    float radius = object->extra_radius;   /* IVP_Ball::get_radius (get_extra_radius) */

    ivp_bs_hesse_t speed_plane_os;
    lvf_zero(&speed_plane_os.n);
    speed_plane_os.hesse_val = 0.0f;
    float scalar_speed = (float)lvf_quad_length(rel_speed_of_current_os);
    if (scalar_speed < P_DOUBLE_EPS) {
        lvf_set3(&speed_plane_os.n, 0.0f, 1.0f, 0.0f);
    } else {
        speed_plane_os.n = *rel_speed_of_current_os;
    }
    fh_calc_hesse_val(&speed_plane_os, &geom_center_os);
    fh_normize(&speed_plane_os);

    ivp_vec3f_t p1_os, p2_os;
    lvf_zero(&p1_os);
    lvf_zero(&p2_os);

    float distance = (float)fh_get_dist(surface_os, &geom_center_os);
    int decision = -1;
    if (distance <= 0.0f) {
        if (BS_FABSD((D)distance) >= radius) return; /* ball completely above the surface */
        decision = disection_points_with_ball(surface_os, &speed_plane_os, &geom_center_os, radius, &p1_os, &p2_os) ? 1 : 2;
    } else {
        if (distance >= radius) {
            decision = 0;
        } else {
            decision = disection_points_with_ball(surface_os, &speed_plane_os, &geom_center_os, radius, &p1_os, &p2_os) ? 3 : 4;
        }
    }

    ball_buoyancy(bs, decision, distance, radius, surface_os, &geom_center_os);
    ball_dampening(bs, decision, distance, radius, &geom_center_os, rel_speed_of_current_os,
                   &speed_plane_os, surface_os, &p1_os, &p2_os);
}

/* ── polygon ──────────────────────────────────────────────────────────── */

static inline void point_of(const ivp_compact_edge_t *e, const ivp_compact_ledge_t *ledge, ivp_vec3f_t *out)
{
    const ivp_compact_poly_point_t *p = ivp_edge_get_start_point(e, ledge);
    out->k[0] = p->k[0];
    out->k[1] = p->k[1];
    out->k[2] = p->k[2];
}

/* reference compute_pyramid_volume (IVP_FLOAT result) */
static float pyramid_volume(const ivp_vec3f_t *p1, const ivp_vec3f_t *p2, const ivp_vec3f_t *p3, const ivp_vec3f_t *s_point)
{
    ivp_vec3r_t hesse;
    lvp_set_vert_to_area_fff(&hesse, p1, p2, p3);
    ivp_vec3f_t v;
    lvf_subtract2(&v, p1, s_point);
    return (float)(lvp_dot_f(&hesse, &v) * (D)(-1.0f / 6.0f));
}

/* reference compute_volumes_and_centers_for_one_pyramid */
static void pyramid_values(ivp_buoyancy_solver_t *bs, ivp_object_t *object,
                           const ivp_vec3f_t *triangle_points[5], const float distance[6],
                           int decision, int index_positiv, int index_neg, const ivp_vec3f_t *s_point)
{
    switch (decision) {
    case 0: { /* triangle completely under the surface */
        D volume_pyramid_under = pyramid_volume(triangle_points[0], triangle_points[1], triangle_points[2], s_point);
        bs->volume_under = (float)((D)bs->volume_under + volume_pyramid_under);

        ivp_vec3f_t volume_center_pyramid;
        lvf_add2(&volume_center_pyramid, triangle_points[0], triangle_points[1]);
        lvf_add(&volume_center_pyramid, triangle_points[2]);
        lvf_add(&volume_center_pyramid, s_point);
        lvf_mult(&volume_center_pyramid, volume_pyramid_under);
        lvf_add(&bs->volume_center_under, &volume_center_pyramid);
        break;
    }
    case 1: { /* the vertex index_neg is above the surface */
        D factor1 = -distance[index_neg] / (distance[index_neg + 1] - distance[index_neg]);
        D factor2 = -distance[index_neg] / (distance[index_neg + 2] - distance[index_neg]);

        IVP_IF(1) {
            bs_draw_disection_edge(bs, object, triangle_points[index_neg], triangle_points[index_neg + 1],
                                   triangle_points[index_neg + 2], factor1, factor2);
        }

        D volume_pyramid = pyramid_volume(triangle_points[0], triangle_points[1], triangle_points[2], s_point);
        D volume_pyramid_above = volume_pyramid * factor1 * factor2;
        bs->volume_under = (float)((D)bs->volume_under + (volume_pyramid - volume_pyramid_above));

        ivp_vec3f_t area_center_total;
        lvf_add2(&area_center_total, triangle_points[0], triangle_points[1]);
        lvf_add(&area_center_total, triangle_points[2]);

        ivp_vec3f_t area_center_part_above;
        lvf_set_multiple(&area_center_part_above, triangle_points[index_neg], 3.0f - factor1 - factor2);
        lvf_add_multiple(&area_center_part_above, triangle_points[index_neg + 1], factor1);
        lvf_add_multiple(&area_center_part_above, triangle_points[index_neg + 2], factor2);

        D k = factor1 * factor2;
        ivp_vec3f_t area_center_part_under;
        lvf_mult(&area_center_part_above, k);
        area_center_part_under = area_center_total;
        lvf_subtract(&area_center_part_under, &area_center_part_above);
        if ((1.0f - k) < bs->buoyancy_eps) return;
        lvf_mult(&area_center_part_under, 1.0f / (1.0f - k));
        lvf_add(&area_center_part_under, s_point);
        lvf_mult(&area_center_part_under, volume_pyramid - volume_pyramid_above);
        lvf_add(&bs->volume_center_under, &area_center_part_under);
        break;
    }
    case 2: { /* two vertices above, index_positiv under the surface */
        D factor1 = distance[index_positiv] / (distance[index_positiv] - distance[index_positiv + 1]);
        D factor2 = distance[index_positiv] / (distance[index_positiv] - distance[index_positiv + 2]);

        IVP_IF(1) { /* for debugging reasons */
            bs_draw_disection_edge(bs, object, triangle_points[index_positiv], triangle_points[index_positiv + 1],
                                   triangle_points[index_positiv + 2], factor1, factor2);
        }

        D volume_pyramid = pyramid_volume(triangle_points[0], triangle_points[1], triangle_points[2], s_point);
        D volume_pyramid_under = factor1 * factor2 * volume_pyramid;
        bs->volume_under = (float)((D)bs->volume_under + volume_pyramid_under);

        ivp_vec3f_t area_center_part_under;
        lvf_set_multiple(&area_center_part_under, triangle_points[index_positiv], 3.0f - factor1 - factor2);
        lvf_add_multiple(&area_center_part_under, triangle_points[index_positiv + 1], factor1);
        lvf_add_multiple(&area_center_part_under, triangle_points[index_positiv + 2], factor2);
        lvf_add(&area_center_part_under, s_point);
        lvf_mult(&area_center_part_under, volume_pyramid_under);
        lvf_add(&bs->volume_center_under, &area_center_part_under);
        break;
    }
    default:
        break;
    }
}

/* reference compute_rotation_and_translation_values_for_one_triangle */
static void triangle_dampening(ivp_buoyancy_solver_t *bs, ivp_object_t *object,
                               const ivp_compact_triangle_t *current_triangle,
                               const ivp_vec3f_t *triangle_points[5], const ivp_compact_ledge_t *current_ledge,
                               const float distance[6], int decision, int index_positiv, int index_neg)
{
    const float buoyancy_eps = bs->buoyancy_eps;
    ivp_vec3f_t area_center_part_under_os;
    float triangle_surface_under_factor;

    switch (decision) {
    case 0:
        lvf_add2(&area_center_part_under_os, triangle_points[0], triangle_points[1]);
        lvf_add(&area_center_part_under_os, triangle_points[2]);
        lvf_mult(&area_center_part_under_os, 1.0f / 3.0f);
        triangle_surface_under_factor = 1.0f;
        break;
    case 1: {
        D factor1 = -distance[index_neg] / (distance[index_neg + 1] - distance[index_neg]);
        D factor2 = -distance[index_neg] / (distance[index_neg + 2] - distance[index_neg]);

        ivp_vec3f_t area_center_of_triangle_os;
        lvf_add2(&area_center_of_triangle_os, triangle_points[0], triangle_points[1]);
        lvf_add(&area_center_of_triangle_os, triangle_points[2]);
        lvf_mult(&area_center_of_triangle_os, 1.0f / 3.0f);

        ivp_vec3f_t area_center_part_above_os;
        lvf_set_multiple(&area_center_part_above_os, triangle_points[index_neg], (3.0f - factor1 - factor2) * (1.0f / 3.0f));
        lvf_add_multiple(&area_center_part_above_os, triangle_points[index_neg + 1], factor1 * (1.0f / 3.0f));
        lvf_add_multiple(&area_center_part_above_os, triangle_points[index_neg + 2], factor2 * (1.0f / 3.0f));

        D k = factor1 * factor2;
        lvf_mult(&area_center_part_above_os, k);
        area_center_part_under_os = area_center_of_triangle_os;
        lvf_subtract(&area_center_part_under_os, &area_center_part_above_os);

        if ((1.0f - k) < buoyancy_eps) return;
        lvf_mult(&area_center_part_under_os, 1.0f / (1.0f - k));
        triangle_surface_under_factor = (float)(1.0f - k);
        break;
    }
    case 2: {
        D factor1 = distance[index_positiv] / (distance[index_positiv] - distance[index_positiv + 1]);
        D factor2 = distance[index_positiv] / (distance[index_positiv] - distance[index_positiv + 2]);

        lvf_set_multiple(&area_center_part_under_os, triangle_points[index_positiv], (3.0f - factor1 - factor2) * (1.0f / 3.0f));
        lvf_add_multiple(&area_center_part_under_os, triangle_points[index_positiv + 1], factor1 * (1.0f / 3.0f));
        lvf_add_multiple(&area_center_part_under_os, triangle_points[index_positiv + 2], factor2 * (1.0f / 3.0f));

        triangle_surface_under_factor = (float)(factor1 * factor2);
        break;
    }
    default:
        return;
    }

    /* IVP_Compact_Ledge_Solver::calc_hesse_vec_object_not_normized(first edge):
     * inline_set_vert_to_area_defined_by_three_points(p0, prev, next) */
    const ivp_compact_edge_t *edge = ivp_tri_get_first_edge(current_triangle);
    ivp_vec3f_t p0, p1, p2;
    point_of(edge, current_ledge, &p0);
    point_of(ivp_edge_get_next(edge), current_ledge, &p1);
    point_of(ivp_edge_get_prev(edge), current_ledge, &p2);
    ivp_vec3f_t normal_os;
    lvf_set_vert_to_area(&normal_os, &p0, &p2, &p1);

    if (lvf_quad_length(&normal_os) < buoyancy_eps) return;

    ivp_vec3f_t speed_of_object_os;
    core_get_surface_speed_os(bs->core, object, &area_center_part_under_os, &speed_of_object_os);

    ivp_vec3f_t rel_speed_of_current_os;
    lvf_subtract2(&rel_speed_of_current_os, &bs->resulting_speed_of_current_os, &speed_of_object_os);

    float quad_length_of_speed = (float)lvf_quad_length(&rel_speed_of_current_os);
    if (quad_length_of_speed < buoyancy_eps) {
        quad_length_of_speed = 0.0f;
        lvf_set3(&rel_speed_of_current_os, 0.0f, 1.0f, 0.0f);
    }

    float surface_factor_check = (float)lvf_dot(&normal_os, &rel_speed_of_current_os);
    if (surface_factor_check > buoyancy_eps) return; /* turned away from the current */

    (void)lvf_normize(&rel_speed_of_current_os);

    ivp_vec3f_t normized_normal_os = normal_os;
    D len_of_unscaled_normal = lvf_real_length_plus_normize(&normized_normal_os);
    float surface_factor = (float)lvf_dot(&normized_normal_os, &rel_speed_of_current_os);

    float triangle_surface_content = (float)(0.5f * len_of_unscaled_normal);
    float visible_surface_content_under = -surface_factor * triangle_surface_under_factor * triangle_surface_content;

    D impulse = (visible_surface_content_under * quad_length_of_speed * bs->pressure_damp_factor);
    impulse += (triangle_surface_under_factor * triangle_surface_content * quad_length_of_speed * bs->friction_damp_factor);

    ivp_vec3f_t impulse_vector;
    if (bs->simulate_wing_behavior) {
        lvf_set_multiple(&impulse_vector, &normized_normal_os, -impulse);
    } else {
        lvf_set_multiple(&impulse_vector, &rel_speed_of_current_os, impulse);
    }

    IVP_IF(0) { /* ivp_buoyancy_solver.cxx:1523 */
        ivp_vec3r_t center_of_triangle_ws;
        ivp_vec3f_t impulse_vector_ws;
        {
            ivp_transform_r_t tmp;
            lvm_vmult4_fp(bs_m_world_f_object(object, &tmp), &area_center_part_under_os, &center_of_triangle_ws);
            lvm_vmult3_f(bs_m_world_f_object(object, &tmp), &impulse_vector, &impulse_vector_ws);
        }
        const double center_of_triangle[3] = { (double)center_of_triangle_ws.k[0], (double)center_of_triangle_ws.k[1],
                                               (double)center_of_triangle_ws.k[2] };
        ivp_env_add_draw_vector(bs->core->environment, center_of_triangle, impulse_vector_ws.k, "", 2);
    }

    bs->object_visible_surface_content_under =
        (float)((D)bs->object_visible_surface_content_under + BS_FABSD((D)visible_surface_content_under));

    ivp_vec3f_t impulse_x_point;
    lvf_cross(&impulse_x_point, &area_center_part_under_os, &impulse_vector);
    lvf_add(&bs->sum_impulse_x_point, &impulse_x_point);

    ivp_vec3f_t move_direction_os;
    lvf_set_orthogonal_part(&move_direction_os, &rel_speed_of_current_os, &normized_normal_os);

    ivp_vec3f_t impulse_x_movevector;
    lvf_cross(&impulse_x_movevector, &move_direction_os, &impulse_vector);
    lvf_add(&bs->sum_impulse_x_movevector, &impulse_x_movevector);

    lvf_add(&bs->sum_impulse, &impulse_vector);
}

/* reference compute_values_for_one_triangle */
static void values_for_one_triangle(ivp_buoyancy_solver_t *bs, ivp_object_t *object,
                                    const ivp_compact_triangle_t *triangle, const ivp_bs_hesse_t *surface_os,
                                    const ivp_vec3f_t *s_point, const ivp_compact_ledge_t *current_ledge)
{
    ivp_vec3f_t three_points[3];
    point_of(ivp_tri_get_edge(triangle, 0), current_ledge, &three_points[0]);
    point_of(ivp_tri_get_edge(triangle, 1), current_ledge, &three_points[1]);
    point_of(ivp_tri_get_edge(triangle, 2), current_ledge, &three_points[2]);

    const ivp_vec3f_t *triangle_points[5] = { &three_points[0], &three_points[1], &three_points[2],
                                              &three_points[0], &three_points[1] };

    float distance[6];
    int sum_distance = 0;
    int index_positiv = 0, index_neg = 0;
    for (int j = 2; j >= 0; j--) {
        float x = (float)fh_get_dist(surface_os, triangle_points[j]);
        if (x < 0.0f) {
            index_neg = j;
            sum_distance++;
            distance[j] = x - bs->buoyancy_eps;
            distance[j + 3] = distance[j];
        } else {
            index_positiv = j;
            distance[j] = x + bs->buoyancy_eps;
            distance[j + 3] = distance[j];
        }
    }

    pyramid_values(bs, object, triangle_points, distance, sum_distance, index_positiv, index_neg, s_point);
    triangle_dampening(bs, object, triangle, triangle_points, current_ledge, distance, sum_distance,
                       index_positiv, index_neg);
}

/* reference compute_values_for_one_ledge */
static void values_for_one_ledge(ivp_buoyancy_solver_t *bs, ivp_object_t *object, const ivp_compact_ledge_t *current_ledge,
                                 const ivp_bs_hesse_t *surface_os, const ivp_vec3f_t *s_point)
{
    int nr_of_triangles = ivp_ledge_get_n_triangles(current_ledge);
    const ivp_compact_triangle_t *triangle = ivp_ledge_get_first_triangle(current_ledge);
    for (int i = nr_of_triangles; --i >= 0; triangle = ivp_tri_get_next(triangle)) {
        values_for_one_triangle(bs, object, triangle, surface_os, s_point, current_ledge);
    }
}

/* IVP_U_BigVector<IVP_Compact_Ledge> object_ledges(256) */
#define BS_INLINE_LEDGES 256
typedef struct bs_ledge_list {
    const ivp_allocator_t       *allocator;
    const ivp_compact_ledge_t **ledges;
    const ivp_compact_ledge_t *inline_ledges[BS_INLINE_LEDGES];
    int len, capacity;
} bs_ledge_list_t;

static void bs_collect_ledge(void *ctx, const ivp_compact_ledge_t *ledge)
{
    bs_ledge_list_t *l = (bs_ledge_list_t *)ctx;
    if (l->len >= l->capacity) {
        int nc = l->capacity * 2;
        const ivp_compact_ledge_t **n;
        if (l->ledges == l->inline_ledges) {
            n = (const ivp_compact_ledge_t **)ivp_alloc(l->allocator, (size_t)nc * sizeof(*n));
            if (n) memcpy(n, l->inline_ledges, (size_t)l->len * sizeof(*n));
        } else {
            n = (const ivp_compact_ledge_t **)ivp_realloc(l->allocator, (void *)l->ledges,
                                                          (size_t)l->capacity * sizeof(*n), (size_t)nc * sizeof(*n));
        }
        if (!n) return;
        l->ledges = n;
        l->capacity = nc;
    }
    IVP_IF(1) { /* IVP_U_BigVector::add: IVP_ASSERT(index_of(elem) == -1) */
        int index;
        for (index = l->len - 1; index >= 0; index--) {
            if (l->ledges[index] == ledge) break;
        }
        IVP_ASSERT(index == -1);
    }
    l->ledges[l->len++] = ledge;
}

/* reference compute_values_for_one_polygon */
static void values_for_one_polygon(ivp_buoyancy_solver_t *bs, ivp_object_t *object, const ivp_bs_hesse_t *surface_os)
{
    bs->volume_under = 0;
    lvf_zero(&bs->volume_center_under);

    ivp_vec3f_t s_point; /* center projected onto the surface */
    IVP_IF(1) { /* ivp_buoyancy_solver.cxx:1082 */
        D leng = lvf_real_length(&surface_os->n);
        D testval = BS_FABSD(leng - 1.0f);
        IVP_ASSERT(testval < 0.001f);
        IVP_USE(testval);
    }
    lvf_set_multiple(&s_point, &surface_os->n, -surface_os->hesse_val);

    bs_ledge_list_t object_ledges;
    object_ledges.allocator = ivp_env_get_allocator(object->owner);
    object_ledges.ledges = object_ledges.inline_ledges;
    object_ledges.len = 0;
    object_ledges.capacity = BS_INLINE_LEDGES;
    if (object->surman) ivp_surman_get_all_terminal_ledges(object->surman, bs_collect_ledge, &object_ledges);

    for (int index = object_ledges.len - 1; index >= 0; index--) {
        values_for_one_ledge(bs, object, object_ledges.ledges[index], surface_os, &s_point);
    }
    if (object_ledges.ledges != object_ledges.inline_ledges) {
        ivp_free(object_ledges.allocator, (void *)object_ledges.ledges,
                 (size_t)object_ledges.capacity * sizeof(*object_ledges.ledges));
    }

    if (bs->volume_under > bs->buoyancy_eps) {
        lvf_mult(&bs->volume_center_under, 0.25f / bs->volume_under);
    }
}

/* ── entry ────────────────────────────────────────────────────────────── */

bool ivp_buoyancy_solver_compute_forces(ivp_buoyancy_solver_t *bs,
                                        const ivp_vec3f_t *rel_speed_of_current_os,
                                        const ivp_bs_hesse_t *surface_os,
                                        ivp_object_t *object)
{
    /* track object, but don't spend CPU on buoyancy solve */
    if (bs->medium_density <= 0) return false;

    bs->object_visible_surface_content_under = 0.0f;
    lvf_zero(&bs->sum_impulse_x_point);
    lvf_zero(&bs->sum_impulse_x_movevector);
    lvf_zero(&bs->sum_impulse);

    {
        ivp_transform_r_t tmp;
        lvm_vimult3_f(bs_m_world_f_object(object, &tmp), &bs->resulting_speed_of_current_ws,
                      &bs->resulting_speed_of_current_os);
    }

    switch (object->type) {
    case IVP_OBJECT_BALL:
        /* ensure that surface_ws is normized */
        IVP_IF(1) { /* ivp_buoyancy_solver.cxx:114 */
            IVP_ASSERT(BS_FABSD(lvf_real_length(&surface_os->n) - 1.0f) < 0.01f);
        }
        values_for_one_ball(bs, object, surface_os, rel_speed_of_current_os);
        break;
    case IVP_OBJECT_POLYGON:
        values_for_one_polygon(bs, object, surface_os);
        break;
    default:
        break;
    }

    return (bs->volume_under > bs->buoyancy_eps) || (lvf_quad_length(&bs->sum_impulse) > bs->buoyancy_eps);
}
