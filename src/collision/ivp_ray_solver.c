/* ivp_ray_solver.c — Ray casting (IVP_Ray_Solver family)
 *
 * Port of ivp_collision/ivp_ray_solver.cxx (Ipion Software GmbH).  The
 * precision of every member follows the reference: start/end points are
 * IVP_U_Point (double), center and direction IVP_U_Float_Point (float),
 * the length IVP_FLOAT.  Environment queries walk the OV tree
 * (check_ray_against_node / check_ray_group_against_node).
 */
#include <ivp/collision/ivp_ray_solver.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/containers/ivp_min_hash.h>
#include "dynamics/ivp_env_internal.h"
#include "dynamics/ivp_env_hot.h"
#include <ivp/geometry/ivp_compact_ledge_geom.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_cache_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>

#include <float.h>
#include <math.h>
#include <string.h>

#define RS_DOUBLE_RES 1e-12 /* P_DOUBLE_RES */
#define RS_DOUBLE_EPS 1e-10 /* P_DOUBLE_EPS */

/* ── Small helpers (reference vector semantics) ────────────────────────── */

static inline double dot_dd(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static inline double dot_df(const double a[3], const float b[3])
{
    return a[0] * (double)b[0] + a[1] * (double)b[1] + a[2] * (double)b[2];
}

/* IVP_U_Float_Point::subtract: double difference rounded to float */
static inline void fsub(float out[3], const float a[3], const float b[3])
{
    for (int i = 0; i < 3; i++) out[i] = (float)((double)a[i] - (double)b[i]);
}

static inline float fquad(const float v[3])
{
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

/* IVP_U_Float_Point::inline_calc_cross_product */
static inline void fcross(float out[3], const float v1[3], const float v2[3])
{
    double a = (double)v1[1] * (double)v2[2] - (double)v2[1] * (double)v1[2];
    double b = (double)v1[2] * (double)v2[0] - (double)v2[2] * (double)v1[0];
    double c = (double)v1[0] * (double)v2[1] - (double)v2[0] * (double)v1[1];
    out[0] = (float)a;
    out[1] = (float)b;
    out[2] = (float)c;
}

/* IVP_U_Point::normize() */
static inline bool dnormize(double v[3])
{
    double len = dot_dd(v, v);
    if (len < RS_DOUBLE_EPS) return false;
    double f = ivp_isqrt_double(len, 3);
    v[0] *= f;
    v[1] *= f;
    v[2] *= f;
    return true;
}

/* derive center / end like the reference constructors */
static void ray_derive(const double start[3], const float dir[3], float length,
                       float center_out[3], double end_out[3])
{
    double half = (double)(length * 0.5f);
    for (int i = 0; i < 3; i++) {
        /* ray_center_point.set(start); add_multiple(&dir, len * 0.5f) */
        float c = (float)start[i];
        center_out[i] = (float)((double)dir[i] * half + (double)c);
        /* ray_end_point.add_multiple(&start, &dir, len) */
        end_out[i] = start[i] + (double)dir[i] * (double)length;
    }
}

/* ── IVP_Ray_Solver ─────────────────────────────────────────────────────── */

#ifdef IVP_ENABLE_ASSERTS
/* IVP_U_Float_Point::real_length() of the template direction (an
 * IVP_U_Float_Point in the reference): sqrt of the float quad length */
static inline double ray_templ_dir_real_length(const ivp_ray_solver_template_t *templ)
{
    float x = (float)templ->ray_normized_direction.k[0];
    float y = (float)templ->ray_normized_direction.k[1];
    float z = (float)templ->ray_normized_direction.k[2];
    return sqrt((double)(x * x + y * y + z * z));
}
#endif

void ivp_ray_solver_init(ivp_ray_solver_t *rs,
                         const ivp_ray_solver_template_t *templ,
                         ivp_ray_hit_listener_fn add_hit,
                         void *listener)
{
    /* ray dir must be normized (ivp_ray_solver.cxx:33) */
    IVP_ASSERT((ray_templ_dir_real_length(templ) > (double)(1.0f - 1e-4f)) &&
               (ray_templ_dir_real_length(templ) < (double)(1.0f + 1e-4f)));
    memset(rs, 0, sizeof(*rs));
    for (int i = 0; i < 3; i++) {
        rs->ray_direction[i] = (float)templ->ray_normized_direction.k[i];
        rs->ray_start_point[i] = (double)templ->ray_start_point.k[i];
    }
    rs->ray_length = (float)templ->ray_length;
    rs->ray_flags = templ->ray_flags;
    ray_derive(rs->ray_start_point, rs->ray_direction, rs->ray_length,
               rs->ray_center_point, rs->ray_end_point);
    rs->add_hit = add_hit;
    rs->listener = listener;
}

bool ivp_ray_solver_check_sphere(const ivp_ray_solver_t *rs,
                                 const float sphere_center_ws[3],
                                 float sphere_radius)
{
    /* ivp_ray_solver.cxx:79 */
    float delta[3];
    double rad_sum = (double)(rs->ray_length * 0.5f + sphere_radius);
    fsub(delta, sphere_center_ws, rs->ray_center_point);
    if ((double)fquad(delta) >= rad_sum * rad_sum) return false;
    float h[3];
    fcross(h, rs->ray_direction, delta);
    float quad_dist = fquad(h);
    float coll_dist = sphere_radius;
    return quad_dist < coll_dist * coll_dist;
}

static bool check_square(const ivp_ray_solver_t *rs, float pos_dist, float pos_axis_len,
                         const float *min_c, const float *max_c, int c0, int c1)
{
    /* ivp_ray_solver.cxx:151 */
    const int coord_index[2] = { c0, c1 };
    for (int i = 1; i >= 0; --i) {
        int coord = coord_index[i];
        /* IVP_FLOAT comp_val; the limits are (float - double) * float, i.e.
         * IVP_DOUBLE, and the comparison is done in double */
        float comp_val = pos_dist * rs->ray_length * rs->ray_direction[coord];
        double lo = ((double)min_c[coord] - rs->ray_start_point[coord]) * (double)pos_axis_len;
        double hi = ((double)max_c[coord] - rs->ray_start_point[coord]) * (double)pos_axis_len;
        if ((double)comp_val < lo || (double)comp_val > hi) return false;
    }
    return true;
}

#define RS_LEFT 1
#define RS_RIGHT 2
#define RS_ABOVE 4
#define RS_BELOW 8
#define RS_FRONT 16
#define RS_BEHIND 32

static int clip_flags(const double p[3], const float *luf, const float *rlb)
{
    int f = 0;
    if (p[0] < (double)luf[0]) f = RS_LEFT;
    else if (p[0] > (double)rlb[0]) f = RS_RIGHT;
    if (p[1] < (double)luf[1]) f |= RS_ABOVE;
    else if (p[1] > (double)rlb[1]) f |= RS_BELOW;
    if (p[2] < (double)luf[2]) f |= RS_FRONT;
    else if (p[2] > (double)rlb[2]) f |= RS_BEHIND;
    return f;
}

bool ivp_ray_solver_check_cube(const ivp_ray_solver_t *rs,
                               const float luf[3], const float rlb[3])
{
    /* ivp_ray_solver.cxx:183 */
    int sf = clip_flags(rs->ray_start_point, luf, rlb);
    if (sf == 0) return true;
    int ef = clip_flags(rs->ray_end_point, luf, rlb);
    if (ef == 0) return true;
    if (sf & ef) return false;

    const double *s = rs->ray_start_point;
    const double *e = rs->ray_end_point;
    /* ivp_ray_solver.cxx:248-309: IVP_ASSERT(pos_dist >= 0.0f), IVP_ASSERT(pos_axis_len >= 0.0f) per facet */
    if ((sf & RS_FRONT) && !(ef & RS_FRONT)) {
        float pos_dist = (float)((double)luf[2] - s[2]);
        float pos_axis_len = (float)(e[2] - s[2]);
        IVP_ASSERT(pos_dist >= 0.0f);
        IVP_ASSERT(pos_axis_len >= 0.0f);
        float mx[3] = { rlb[0], rlb[1], luf[2] };
        if (check_square(rs, pos_dist, pos_axis_len, luf, mx, 0, 1)) return true;
    }
    if ((sf & RS_RIGHT) && !(ef & RS_RIGHT)) {
        float pos_dist = (float)(s[0] - (double)rlb[0]);
        float pos_axis_len = (float)(s[0] - e[0]);
        IVP_ASSERT(pos_dist >= 0.0f);
        IVP_ASSERT(pos_axis_len >= 0.0f);
        float mn[3] = { rlb[0], luf[1], luf[2] };
        if (check_square(rs, pos_dist, pos_axis_len, mn, rlb, 1, 2)) return true;
    }
    if ((sf & RS_BEHIND) && !(ef & RS_BEHIND)) {
        float pos_dist = (float)(s[2] - (double)rlb[2]);
        float pos_axis_len = (float)(s[2] - e[2]);
        IVP_ASSERT(pos_dist >= 0.0f);
        IVP_ASSERT(pos_axis_len >= 0.0f);
        float mn[3] = { luf[0], luf[1], rlb[2] };
        if (check_square(rs, pos_dist, pos_axis_len, mn, rlb, 0, 1)) return true;
    }
    if ((sf & RS_LEFT) && !(ef & RS_LEFT)) {
        float pos_dist = (float)((double)luf[0] - s[0]);
        float pos_axis_len = (float)(e[0] - s[0]);
        IVP_ASSERT(pos_dist >= 0.0f);
        IVP_ASSERT(pos_axis_len >= 0.0f);
        float mx[3] = { luf[0], rlb[1], rlb[2] };
        if (check_square(rs, pos_dist, pos_axis_len, luf, mx, 1, 2)) return true;
    }
    if ((sf & RS_ABOVE) && !(ef & RS_ABOVE)) {
        float pos_dist = (float)((double)luf[1] - s[1]);
        float pos_axis_len = (float)(e[1] - s[1]);
        IVP_ASSERT(pos_dist >= 0.0f);
        IVP_ASSERT(pos_axis_len >= 0.0f);
        float mx[3] = { rlb[0], luf[1], rlb[2] };
        if (check_square(rs, pos_dist, pos_axis_len, luf, mx, 0, 2)) return true;
    }
    if ((sf & RS_BELOW) && !(ef & RS_BELOW)) {
        float pos_dist = (float)(s[1] - (double)rlb[1]);
        float pos_axis_len = (float)(s[1] - e[1]);
        IVP_ASSERT(pos_dist >= 0.0f);
        IVP_ASSERT(pos_axis_len >= 0.0f);
        float mn[3] = { luf[0], rlb[1], luf[2] };
        if (check_square(rs, pos_dist, pos_axis_len, mn, rlb, 0, 2)) return true;
    }
    return false;
}

/* object transform at the environment's current time (IVP_Cache_Object) */
static void object_world_matrix(const ivp_object_t *obj, ivp_transform_r_t *m)
{
    ivp_time_t t = obj->owner ? ivp_environment_get_time(obj->owner) : obj->physical_core->time_of_last_psi;
    ivp_object_get_at_matrix(obj, t, m);
}

void ivp_ray_solver_check_ball(ivp_ray_solver_t *rs, ivp_object_t *ball)
{
    /* ivp_ray_solver.cxx:552 */
    if (!ball || !rs->add_hit) return;
    ivp_transform_r_t m;
    object_world_matrix(ball, &m);
    float center_ws[3] = { (float)m.pos.k[0], (float)m.pos.k[1], (float)m.pos.k[2] };

    /* IVP_Ball::get_radius() * get_radius() (extra_radius, IVP_FLOAT product) */
    double quad_sphere_radius = (double)(ball->extra_radius * ball->extra_radius);
    double delta[3];
    for (int i = 0; i < 3; i++) delta[i] = rs->ray_start_point[i] - (double)center_ws[i];
    double quad_delta_len = dot_dd(delta, delta);
    if (quad_delta_len < quad_sphere_radius) return; /* ray starts inside */

    double e0 = 1.0;
    double e1 = 2.0 * dot_df(delta, rs->ray_direction);
    double e2 = quad_delta_len - quad_sphere_radius;
    /* solve_quadratic_equation_accurate */
    double b2 = e1 * e1;
    double ac4 = e0 * e2 * (double)4.0f;
    double disc = b2 - ac4;
    if (disc < 0.0) return;
    double i2a = (double)0.5f / e0;
    double sr = sqrt(disc);
    double l1 = (-e1 - sr) * i2a;
    double l2 = (-e1 + sr) * i2a;
    float hit_dist = (float)l1;
    if (hit_dist < 0.0f) {
        hit_dist = (float)l2;
        if (hit_dist < 0.0f) return;
    }
    double n[3];
    for (int i = 0; i < 3; i++) {
        double p = rs->ray_start_point[i] + (double)rs->ray_direction[i] * (double)hit_dist;
        n[i] = p - (double)center_ws[i];
    }
    if (!dnormize(n)) {
        n[0] = 1.0;
        n[1] = 0.0;
        n[2] = 0.0;
    }
    rs->add_hit(rs->listener, ball, NULL, NULL, (double)hit_dist, n);
}

static bool filtered_out(const ivp_ray_solver_t *rs, const ivp_object_t *object)
{
    unsigned flags = rs->ray_flags;
    if (flags & IVP_RAY_SOLVER_IGNORE_PHANTOMS) {
        /* object->get_controller_phantom() */
        if (object->controller_phantom) return true;
        /* libivp extension: optional user classifier */
        if (rs->is_phantom && rs->is_phantom(rs->is_phantom_user, object)) return true;
    }
    if ((flags & IVP_RAY_SOLVER_IGNORE_MOVINGS) && !IVP_MTIS_IS_STATIC(object->object_movement_state)) {
        return true;
    }
    if ((flags & IVP_RAY_SOLVER_IGNORE_STATICS) && IVP_MTIS_IS_STATIC(object->object_movement_state)) {
        return true;
    }
    return false;
}

void ivp_ray_solver_check_object(ivp_ray_solver_t *rs, ivp_object_t *object)
{
    /* ivp_ray_solver.cxx:604 */
    if (!object || filtered_out(rs, object)) return;
    switch (object->type) {
    case IVP_OBJECT_BALL:
        ivp_ray_solver_check_ball(rs, object);
        break;
    case IVP_OBJECT_POLYGON:
        if (object->surman) ivp_surman_insert_all_ledges_hitting_ray(object->surman, rs, object);
        break;
    default:
        break;
    }
}

/* IVP_U_Float_Point object_center(core->get_position_PSI()) */
static inline void core_center_f(const ivp_object_t *obj, float c[3])
{
    const ivp_core_t *core = obj->physical_core;
    c[0] = (float)core->m_world_f_core_last_psi.pos.k[0];
    c[1] = (float)core->m_world_f_core_last_psi.pos.k[1];
    c[2] = (float)core->m_world_f_core_last_psi.pos.k[2];
}

void ivp_ray_solver_check_node(ivp_ray_solver_t *rs, const ivp_ov_node_t *node,
                               const ivp_ov_tree_t *ov_tree)
{
    /* IVP_Ray_Solver::check_ray_against_node (ivp_ray_solver.cxx:717) */
    if (!node) return;

    /* left upper front / right lower back corners of the node cube */
    float luf[3], rlb[3], cubesize;
    ivp_ov_tree_get_luf_coordinates_ws(ov_tree, node, luf, &cubesize);
    rlb[0] = luf[0] + cubesize;
    rlb[1] = luf[1] + cubesize;
    rlb[2] = luf[2] + cubesize;
    if (!ivp_ray_solver_check_cube(rs, luf, rlb)) return; /* not hit */

    /* objects of this node, last to first */
    for (int i = ivp_pvec_len(&node->elements) - 1; i >= 0; --i) {
        const ivp_ov_element_t *elem = (const ivp_ov_element_t *)ivp_pvec_at(&node->elements, i);
        ivp_object_t *obj = elem->real_object;
        float c[3];
        core_center_f(obj, c);
        if (ivp_ray_solver_check_sphere(rs, c, (float)obj->physical_core->upper_limit_radius)) {
            ivp_ray_solver_check_object(rs, obj);
        }
    }

    /* children, last to first */
    for (int i = ivp_pvec_len(&node->children) - 1; i >= 0; --i) {
        ivp_ray_solver_check_node(rs, (const ivp_ov_node_t *)ivp_pvec_at(&node->children, i), ov_tree);
    }
}

void ivp_ray_solver_check_all_objects(ivp_ray_solver_t *rs, const ivp_environment_t *env)
{
    /* IVP_Ray_Solver::check_ray_against_all_objects_in_sim (ivp_ray_solver.cxx:809) */
    if (!rs || !env) return;
    const ivp_ov_tree_t *ov_tree = ivp_env_get_ov_tree((ivp_environment_t *)env);
    if (!ov_tree) return;
    const ivp_ov_node_t *node = ov_tree->root;
    if (!node) return;
    ivp_ray_solver_check_node(rs, node, ov_tree);
}

void ivp_ray_solver_insert_surface_hits(ivp_ray_solver_t *rs, ivp_object_t *object,
                                        const ivp_compact_surface_t *surface)
{
    if (!rs || !surface) return;
    ivp_ray_solver_os_t os;
    ivp_ray_solver_os_init(&os, rs, object);
    ivp_ray_solver_os_check_compact_surface(&os, surface);
}

/* ── IVP_Ray_Solver_Os ──────────────────────────────────────────────────── */

void ivp_ray_solver_os_init(ivp_ray_solver_os_t *os, const ivp_ray_solver_t *rs,
                            ivp_object_t *object)
{
    /* ivp_ray_solver.cxx:536 */
    memset(os, 0, sizeof(*os));
    os->object = object;
    os->add_hit = rs->add_hit;
    os->listener = rs->listener;
    if (object) {
        ivp_transform_r_t m;
        object_world_matrix(object, &m);
        double d[3];
        for (int i = 0; i < 3; i++) d[i] = rs->ray_start_point[i] - (double)m.pos.k[i];
        for (int c = 0; c < 3; c++) {
            double s = 0.0, v = 0.0;
            for (int r = 0; r < 3; r++) {
                s += (double)m.rot.rows[r].k[c] * d[r];
                v += (double)m.rot.rows[r].k[c] * (double)rs->ray_direction[r];
            }
            os->ray_start_point[c] = s;
            os->ray_direction[c] = (float)v;
        }
    } else {
        for (int i = 0; i < 3; i++) {
            os->ray_start_point[i] = rs->ray_start_point[i];
            os->ray_direction[i] = rs->ray_direction[i];
        }
    }
    os->ray_length = rs->ray_length;
    ray_derive(os->ray_start_point, os->ray_direction, os->ray_length,
               os->ray_center_point, os->ray_end_point);
}

void ivp_ray_solver_os_init_local(ivp_ray_solver_os_t *os, const double start_os[3],
                                  const double dir_os[3], double length,
                                  ivp_ray_hit_listener_fn add_hit, void *listener,
                                  ivp_object_t *object)
{
    memset(os, 0, sizeof(*os));
    for (int i = 0; i < 3; i++) {
        os->ray_start_point[i] = start_os[i];
        os->ray_direction[i] = (float)dir_os[i];
    }
    os->ray_length = (float)length;
    ray_derive(os->ray_start_point, os->ray_direction, os->ray_length,
               os->ray_center_point, os->ray_end_point);
    os->add_hit = add_hit;
    os->listener = listener;
    os->object = object;
}

static bool check_sphere_os(const ivp_ray_solver_os_t *os, const float center[3], float radius)
{
    /* ivp_ray_solver.cxx:118 */
    double qrad_sum = (double)(os->ray_length * 0.5f + radius);
    double qsphere_rad = (double)(radius * radius);
    qrad_sum *= qrad_sum;
    float delta[3];
    fsub(delta, center, os->ray_center_point);
    double quad_center_dist = (double)fquad(delta);
    if (quad_center_dist >= qrad_sum) return false;
    if (quad_center_dist < qsphere_rad) return true;
    float h[3];
    fcross(h, os->ray_direction, delta);
    float quad_dist = fquad(h);
    float coll_dist = radius;
    return (coll_dist * coll_dist - quad_dist) >= -0.01f;
}

static void interpolate(const double a[3], const double b[3], double s, double out[3])
{
    /* IVP_U_Point::set_interpolate */
    double is = (double)1.0f - s;
    for (int i = 0; i < 3; i++) out[i] = a[i] * is + b[i] * s;
}

bool ivp_ray_solver_os_check_compact_ledge(ivp_ray_solver_os_t *os,
                                           const ivp_compact_ledge_t *ledge)
{
    /* ivp_ray_solver.cxx:340 */
    int n_triangles = ivp_ledge_get_n_triangles(ledge);
    if (n_triangles <= 0) return false;

    if (n_triangles == 2) {
        /* two sided triangle ledge */
        const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
        const ivp_compact_edge_t *edge = &tri->edges[0];
        double hesse[3];
        ivp_ledge_calc_hesse_vec(ledge, edge, hesse);
        const float *p0 = ivp_edge_get_start_point(edge, ledge)->k;
        double z = dot_df(hesse, p0);
        double a = dot_dd(os->ray_start_point, hesse) - z;
        double b = dot_dd(os->ray_end_point, hesse) - z;
        if (a * b >= 0.0) return false;
        double hit_dist = a / (a - b);
        double ip[3];
        interpolate(os->ray_start_point, os->ray_end_point, hit_dist, ip);
        ivp_ledge_unscaled_qr_t qr;
        ivp_ledge_calc_unscaled_qr_vals(ledge, edge, ip, &qr);
        if (ivp_ledge_unscaled_qr_is_outside(&qr)) return false;
        if (a < 0.0) {
            hesse[0] = -hesse[0];
            hesse[1] = -hesse[1];
            hesse[2] = -hesse[2];
        }
        dnormize(hesse);
        if (os->add_hit) {
            os->add_hit(os->listener, os->object, ledge, tri,
                        hit_dist * (double)os->ray_length, hesse);
        }
        return true;
    }

    /* find a triangle facing the ray start */
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
    const ivp_compact_edge_t *take_edge = NULL;
    for (int i = 0; i < n_triangles; i++, tri++) {
        const ivp_compact_edge_t *edge = &tri->edges[0];
        double hesse[3];
        ivp_ledge_calc_hesse_vec(ledge, edge, hesse);
        double dot_prod = dot_df(hesse, os->ray_direction);
        if (dot_prod > -RS_DOUBLE_RES) continue;
        const float *p0 = ivp_edge_get_start_point(edge, ledge)->k;
        double z = dot_df(hesse, p0);
        double a = dot_dd(os->ray_start_point, hesse) - z;
        if (a > 0.0) {
            double b = dot_dd(os->ray_end_point, hesse) - z;
            if (b > 0.0) return false; /* no way to get a hit */
            take_edge = edge;
            break;
        }
    }
    if (!take_edge) {
        IVP_IF(0) {   /* ivp_ray_solver.cxx:426 */
            ivp_debug_printf("IVP_Ray_Solver::check_ray_against_compact_ledge - no correct face found.\n");
        }
        return false;
    }

    /* navigate to the hit triangle */
    const ivp_compact_edge_t *edge = take_edge;
    for (int tri_cnt = n_triangles; --tri_cnt >= 0;) {
        double hesse[3];
        ivp_ledge_calc_hesse_vec(ledge, edge, hesse);
        double dot_p = dot_df(hesse, os->ray_direction);
        if (dot_p > -RS_DOUBLE_RES) return false; /* walked beyond the terminator */
        const float *p0 = ivp_edge_get_start_point(edge, ledge)->k;
        double z = dot_df(hesse, p0);
        double a = dot_dd(os->ray_start_point, hesse) - z;
        double b = dot_dd(os->ray_end_point, hesse) - z;
        if (a * b >= 0.0) return false;
        double hit_dist = a / (a - b);
        double ip[3];
        interpolate(os->ray_start_point, os->ray_end_point, hit_dist, ip);
        ivp_ledge_unscaled_qr_t qr;
        ivp_ledge_calc_unscaled_qr_vals(ledge, edge, ip, &qr);

        const ivp_compact_edge_t *next_edge = NULL;
        const ivp_compact_edge_t *e = edge;
        for (int j = 0; j < 3; e = ivp_edge_get_next(e), j++) {
            if (qr.checks[j] > 0.0f) continue;
            next_edge = ivp_edge_get_opposite(e);
            break;
        }
        if (next_edge) {
            edge = next_edge;
            continue;
        }
        dnormize(hesse);
        if (os->add_hit) {
            os->add_hit(os->listener, os->object, ledge, ivp_edge_get_triangle(edge),
                        hit_dist * (double)os->ray_length, hesse);
        }
        return true;
    }
    /* now we know that we are in an endless loop */
    IVP_IF(1) {   /* ivp_ray_solver.cxx:494 */
        ivp_debug_printf("check_ray_against_compact_ledge: endl loop.\n");
    }
    return false;
}

void ivp_ray_solver_os_check_ledgetree_node(ivp_ray_solver_os_t *os,
                                            const ivp_compact_ledgetree_node_t *node)
{
    /* ivp_ray_solver.cxx:503 */
    if (!check_sphere_os(os, node->center, node->radius)) return;
    if (ivp_ledgetree_is_terminal(node)) {
        ivp_ray_solver_os_check_compact_ledge(os, ivp_ledgetree_get_compact_ledge(node));
        return;
    }
    ivp_ray_solver_os_check_ledgetree_node(os, ivp_ledgetree_left_son(node));
    ivp_ray_solver_os_check_ledgetree_node(os, ivp_ledgetree_right_son(node));
}

void ivp_ray_solver_os_check_compact_surface(ivp_ray_solver_os_t *os,
                                             const ivp_compact_surface_t *surface)
{
    ivp_ray_solver_os_check_ledgetree_node(os, ivp_surface_get_ledgetree_root(surface));
}

/* ── Collectors ─────────────────────────────────────────────────────────── */

static void fill_hit(ivp_ray_hit_t *h, const ivp_ray_solver_t *rs, ivp_object_t *object,
                     const ivp_compact_ledge_t *ledge, const ivp_compact_triangle_t *tri,
                     double hit_dist, const double n[3])
{
    h->hit_distance = (ivp_real_t)(float)hit_dist;
    for (int i = 0; i < 3; i++) {
        h->hit_point.k[i] = (ivp_real_t)(rs->ray_start_point[i] +
                                         (double)rs->ray_direction[i] * (double)(float)hit_dist);
        h->hit_normal.k[i] = (ivp_real_t)(float)n[i]; /* IVP_U_Float_Point */
    }
    h->hit_object = object;
    h->hit_ledge = ledge;
    h->hit_triangle = tri;
    h->hit_material_index = tri ? ivp_tri_get_material_index(tri) : 0;
}

static void min_add_hit(void *listener, ivp_object_t *object, const ivp_compact_ledge_t *ledge,
                        const ivp_compact_triangle_t *tri, double hit_dist, const double n[3])
{
    /* IVP_Ray_Solver_Min::add_hit_object (ivp_ray_solver.cxx:895) */
    ivp_ray_solver_min_t *rs = (ivp_ray_solver_min_t *)listener;
    if (hit_dist >= (double)rs->min_dist) return;
    rs->min_dist = (float)hit_dist;
    fill_hit(&rs->ray_hit, &rs->base, object, ledge, tri, hit_dist, n);
}

void ivp_ray_solver_min_init(ivp_ray_solver_min_t *rs, const ivp_ray_solver_template_t *templ)
{
    ivp_ray_solver_init(&rs->base, templ, min_add_hit, rs);
    rs->min_dist = FLT_MAX;
    memset(&rs->ray_hit, 0, sizeof(rs->ray_hit));
}

const ivp_ray_hit_t *ivp_ray_solver_min_get_ray_hit(const ivp_ray_solver_min_t *rs)
{
    if (rs->min_dist == FLT_MAX) return NULL;
    return &rs->ray_hit;
}

static void min_hash_add_hit(void *listener, ivp_object_t *object, const ivp_compact_ledge_t *ledge,
                             const ivp_compact_triangle_t *tri, double hit_dist, const double n[3])
{
    /* IVP_Ray_Solver_Min_Hash::add_hit_object (ivp_ray_solver.cxx:49) */
    ivp_ray_solver_min_hash_t *rs = (ivp_ray_solver_min_hash_t *)listener;
    if (rs->n_hits >= IVP_MAX_NUM_RAY_HITS) {
        IVP_IF(1) {   /* ivp_ray_solver.cxx:55 */
            ivp_debug_printf("IVP_Ray_Solver::add_hit_object - max ray hits exceeded -> hit list truncated.\n");
        }
        return;
    }
    IVP_IF(0) {   /* ivp_ray_solver.cxx:68 */
        ivp_debug_printf("object inserted: %s, dist %g\n", object->name, hit_dist);
    }
    fill_hit(&rs->hit_info[rs->n_hits], &rs->base, object, ledge, tri, hit_dist, n);
    rs->hit_key[rs->n_hits] = hit_dist; /* output_min_hash.add(ray_hit, hit_dist) */
    rs->n_hits++;
}

void ivp_ray_solver_min_hash_init(ivp_ray_solver_min_hash_t *rs,
                                  const ivp_ray_solver_template_t *templ)
{
    ivp_ray_solver_init(&rs->base, templ, min_hash_add_hit, rs);
    rs->n_hits = 0;
    memset(rs->hit_info, 0, sizeof(rs->hit_info));
    memset(rs->hit_key, 0, sizeof(rs->hit_key));
}

ivp_result_t ivp_ray_solver_min_hash_fill_min_hash(const ivp_ray_solver_min_hash_t *rs,
                                                   ivp_min_hash_t *out)
{
    /* IVP_Ray_Solver_Min_Hash::output_min_hash: one add per hit, in hit order */
    if (!rs || !out) return IVP_ERR_INVALID_ARG;
    for (int i = 0; i < rs->n_hits; i++) {
        ivp_result_t r = ivp_min_hash_add(out, (void *)&rs->hit_info[i], rs->hit_key[i]);
        if (r != IVP_OK) return r;
    }
    return IVP_OK;
}

int ivp_ray_solver_min_hash_get_sorted(const ivp_ray_solver_min_hash_t *rs,
                                       const ivp_ray_hit_t **out, int capacity)
{
    /* Order in which the reference IVP_U_Min_Hash (SORT_MINDIST_ELEMENTS
     * build) hands out the hits with find_min_elem() / remove_min(): every
     * tree / bucket minimum is the smallest (value, cmp_index) pair and
     * cmp_index grows with every add, so draining the hash is a stable sort
     * on the double hit distance (insertion order for equal values),
     * independent of the bucket placement.  Computed here without the
     * hash (no allocation, no global insertion counter); the container
     * itself is available through ivp_ray_solver_min_hash_fill_min_hash(). */
    int n = rs->n_hits;
    if (!out || capacity <= 0) return n;
    int m = n < capacity ? n : capacity;
    int idx[IVP_MAX_NUM_RAY_HITS];
    for (int i = 0; i < n; i++) {
        double key = rs->hit_key[i];
        int j = i - 1;
        while (j >= 0 && rs->hit_key[idx[j]] > key) {
            idx[j + 1] = idx[j];
            j--;
        }
        idx[j + 1] = i;
    }
    for (int i = 0; i < m; i++) out[i] = &rs->hit_info[idx[i]];
    return n;
}

/* ── IVP_Ray_Solver_Group ──────────────────────────────────────────────── */

void ivp_ray_solver_group_init(ivp_ray_solver_group_t *g, int n_ray_solvers,
                               ivp_ray_solver_t **ray_solvers)
{
    /* ivp_ray_solver.cxx:845 */
    IVP_ASSERT(n_ray_solvers > 0);
    IVP_ASSERT(ray_solvers);
    memset(g, 0, sizeof(*g));
    g->ray_solvers = ray_solvers;
    g->n_ray_solvers = n_ray_solvers;
    if (!ray_solvers || n_ray_solvers <= 0) {
        g->n_ray_solvers = 0;
        return;
    }
    int valid = 0;
    for (int i = 0; i < n_ray_solvers; i++) {
        if (!ray_solvers[i]) continue;
        for (int k = 0; k < 3; k++) {
            g->center_ws[k] = (float)((double)g->center_ws[k] + (double)ray_solvers[i]->ray_center_point[k]);
        }
        valid++;
    }
    if (valid <= 0) {
        g->n_ray_solvers = 0;
        return;
    }
    double f = (double)(1.0f / (float)valid);
    for (int k = 0; k < 3; k++) g->center_ws[k] = (float)((double)g->center_ws[k] * f);
    double qradius = 0.0;
    for (int j = 0; j < n_ray_solvers; j++) {
        const ivp_ray_solver_t *s = ray_solvers[j];
        if (!s) continue;
        double q0 = 0.0, q1 = 0.0;
        for (int k = 0; k < 3; k++) {
            double d0 = s->ray_start_point[k] - (double)g->center_ws[k];
            double d1 = s->ray_end_point[k] - (double)g->center_ws[k];
            q0 += d0 * d0;
            q1 += d1 * d1;
        }
        if (q0 > qradius) qradius = q0;
        if (q1 > qradius) qradius = q1;
    }
    g->radius = (float)sqrt(qradius);
}

static bool group_check_sphere(const ivp_ray_solver_group_t *g, const float c[3], float r)
{
    /* IVP_Ray_Solver_Group::check_ray_group_against_sphere (ivp_ray_solver.cxx:104) */
    float delta[3];
    double rad_sum = (double)(g->radius + r);
    fsub(delta, c, g->center_ws);
    return (double)fquad(delta) < rad_sum * rad_sum;
}

bool ivp_ray_solver_group_check_cube(const ivp_ray_solver_group_t *g,
                                     const float cube_center_ws[3], float cube_size)
{
    /* IVP_Ray_Solver_Group::check_ray_group_against_cube (ivp_ray_solver.cxx:320) */
    float dist[3];
    fsub(dist, cube_center_ws, g->center_ws);
    dist[0] = fabsf(dist[0]);
    dist[1] = fabsf(dist[1]);
    dist[2] = fabsf(dist[2]);
    cube_size = 0.5f * cube_size + g->radius;
    if (dist[0] > cube_size) return false;
    if (dist[1] > cube_size) return false;
    if (dist[2] > cube_size) return false;
    if ((double)fquad(dist) >= (double)(cube_size * cube_size * 3.0f)) return false;
    return true;
}

void ivp_ray_solver_group_check_object(ivp_ray_solver_group_t *g, ivp_object_t *object)
{
    /* ivp_ray_solver.cxx:651 */
    if (!object || !g->ray_solvers || g->n_ray_solvers <= 0 || !g->ray_solvers[0]) return;
    if (filtered_out(g->ray_solvers[0], object)) return;
    switch (object->type) {
    case IVP_OBJECT_BALL:
        for (int i = g->n_ray_solvers - 1; i >= 0; i--) {
            if (g->ray_solvers[i]) ivp_ray_solver_check_ball(g->ray_solvers[i], object);
        }
        break;
    case IVP_OBJECT_POLYGON:
        if (!object->surman) break;
        for (int i = g->n_ray_solvers - 1; i >= 0; i--) {
            if (g->ray_solvers[i]) {
                ivp_surman_insert_all_ledges_hitting_ray(object->surman, g->ray_solvers[i], object);
            }
        }
        break;
    default:
        break;
    }
}

void ivp_ray_solver_group_check_node(ivp_ray_solver_group_t *g, const ivp_ov_node_t *node,
                                     const ivp_ov_tree_t *ov_tree)
{
    /* IVP_Ray_Solver_Group::check_ray_group_against_node (ivp_ray_solver.cxx:762) */
    if (!node) return;

    float luf[3], center[3], cubesize;
    ivp_ov_tree_get_luf_coordinates_ws(ov_tree, node, luf, &cubesize);
    float csh = cubesize * 0.5f;
    center[0] = luf[0] + csh;
    center[1] = luf[1] + csh;
    center[2] = luf[2] + csh;
    if (!ivp_ray_solver_group_check_cube(g, center, cubesize)) return; /* not hit */

    for (int i = ivp_pvec_len(&node->elements) - 1; i >= 0; --i) {
        const ivp_ov_element_t *elem = (const ivp_ov_element_t *)ivp_pvec_at(&node->elements, i);
        ivp_object_t *obj = elem->real_object;
        float c[3];
        core_center_f(obj, c);
        if (group_check_sphere(g, c, (float)obj->physical_core->upper_limit_radius)) {
            ivp_ray_solver_group_check_object(g, obj);
        }
    }

    for (int i = ivp_pvec_len(&node->children) - 1; i >= 0; --i) {
        ivp_ray_solver_group_check_node(g, (const ivp_ov_node_t *)ivp_pvec_at(&node->children, i), ov_tree);
    }
}

void ivp_ray_solver_group_check_all_objects(ivp_ray_solver_group_t *g,
                                            const ivp_environment_t *env)
{
    /* IVP_Ray_Solver_Group::check_ray_group_against_all_objects_in_sim (ivp_ray_solver.cxx:827) */
    if (!g || !env) return;
    const ivp_ov_tree_t *ov_tree = ivp_env_get_ov_tree((ivp_environment_t *)env);
    if (!ov_tree) return;
    const ivp_ov_node_t *node = ov_tree->root;
    if (!node) return;
    ivp_ray_solver_group_check_node(g, node, ov_tree);
}

/* ── Single surface helpers ─────────────────────────────────────────────── */

typedef struct local_min {
    float min_dist;
    ivp_ray_hit_t hit;
    const ivp_ray_input_t *ray;
} local_min_t;

static void local_min_add(void *listener, ivp_object_t *object, const ivp_compact_ledge_t *ledge,
                          const ivp_compact_triangle_t *tri, double hit_dist, const double n[3])
{
    local_min_t *lm = (local_min_t *)listener;
    if (hit_dist >= (double)lm->min_dist) return;
    lm->min_dist = (float)hit_dist;
    lm->hit.hit_distance = (ivp_real_t)(float)hit_dist;
    for (int i = 0; i < 3; i++) {
        lm->hit.hit_point.k[i] = lm->ray->origin.k[i] +
                                 lm->ray->direction.k[i] * (ivp_real_t)(float)hit_dist;
        lm->hit.hit_normal.k[i] = (ivp_real_t)(float)n[i];
    }
    lm->hit.hit_object = object;
    lm->hit.hit_ledge = ledge;
    lm->hit.hit_triangle = tri;
    lm->hit.hit_material_index = tri ? ivp_tri_get_material_index(tri) : 0;
}

static void local_os(ivp_ray_solver_os_t *os, const ivp_ray_input_t *ray, local_min_t *lm)
{
    double s[3], d[3];
    for (int i = 0; i < 3; i++) {
        s[i] = (double)ray->origin.k[i];
        d[i] = (double)ray->direction.k[i];
    }
    lm->min_dist = FLT_MAX;
    lm->ray = ray;
    memset(&lm->hit, 0, sizeof(lm->hit));
    ivp_ray_solver_os_init_local(os, s, d, (double)ray->max_distance, local_min_add, lm, NULL);
}

ivp_result_t ivp_ray_cast_surface(const ivp_compact_surface_t *surface,
                                  const ivp_ray_input_t *ray,
                                  ivp_ray_hit_t *hit_out)
{
    IVP_ASSERT(surface != NULL && ray != NULL && hit_out != NULL);
    ivp_ray_solver_os_t os;
    local_min_t lm;
    local_os(&os, ray, &lm);
    ivp_ray_solver_os_check_compact_surface(&os, surface);
    if (lm.min_dist == FLT_MAX) return IVP_NOT_FOUND;
    *hit_out = lm.hit;
    return IVP_OK;
}

ivp_result_t ivp_ray_cast_ledge(const ivp_compact_ledge_t *ledge,
                                const ivp_ray_input_t *ray,
                                ivp_ray_hit_t *hit_out)
{
    IVP_ASSERT(ledge != NULL && ray != NULL && hit_out != NULL);
    ivp_ray_solver_os_t os;
    local_min_t lm;
    local_os(&os, ray, &lm);
    ivp_ray_solver_os_check_compact_ledge(&os, ledge);
    if (lm.min_dist == FLT_MAX) return IVP_NOT_FOUND;
    *hit_out = lm.hit;
    return IVP_OK;
}

static inline ivp_vec3r_t point_to_vec3r(const ivp_compact_poly_point_t *p)
{
    return ivp_vec3r((ivp_real_t)p->k[0], (ivp_real_t)p->k[1], (ivp_real_t)p->k[2]);
}

bool ivp_ray_test_sphere(const ivp_vec3r_t *origin,
                         const ivp_vec3r_t *direction,
                         const ivp_vec3r_t *center,
                         ivp_real_t radius,
                         ivp_real_t *t_out)
{
    ivp_vec3r_t delta;
    ivp_vec3r_sub(&delta, center, origin);
    ivp_real_t t_closest = ivp_vec3r_dot(&delta, direction);
    ivp_real_t delta_sq = ivp_vec3r_quad_length(&delta);
    ivp_real_t d_sq = delta_sq - t_closest * t_closest;
    ivp_real_t r_sq = radius * radius;
    if (d_sq > r_sq) return false;
    ivp_real_t half_chord = ivp_sqrt(r_sq - d_sq);
    ivp_real_t t0 = t_closest - half_chord;
    ivp_real_t t1 = t_closest + half_chord;
    if (t1 < IVP_REAL(0.0)) return false;
    if (t_out) *t_out = (t0 >= IVP_REAL(0.0)) ? t0 : t1;
    return true;
}

ivp_result_t ivp_ray_test_triangle(const ivp_compact_ledge_t *ledge,
                                   const ivp_compact_triangle_t *tri,
                                   const ivp_ray_input_t *ray,
                                   ivp_real_t *hit_dist)
{
    IVP_ASSERT(ledge != NULL && tri != NULL && ray != NULL && hit_dist != NULL);
    ivp_vec3r_t p0 = point_to_vec3r(ivp_edge_get_start_point(&tri->edges[0], ledge));
    ivp_vec3r_t p1 = point_to_vec3r(ivp_edge_get_start_point(&tri->edges[1], ledge));
    ivp_vec3r_t p2 = point_to_vec3r(ivp_edge_get_start_point(&tri->edges[2], ledge));
    ivp_vec3r_t e01, e02, normal, p0o;
    ivp_vec3r_sub(&e01, &p1, &p0);
    ivp_vec3r_sub(&e02, &p2, &p0);
    ivp_vec3r_cross(&normal, &e01, &e02);
    ivp_real_t denom = ivp_vec3r_dot(&normal, &ray->direction);
    if (ivp_fabs(denom) < IVP_REAL_RES) return IVP_NOT_FOUND;
    ivp_vec3r_sub(&p0o, &p0, &ray->origin);
    ivp_real_t t = ivp_vec3r_dot(&normal, &p0o) / denom;
    if (t < IVP_REAL(0.0) || t > ray->max_distance) return IVP_NOT_FOUND;
    double hp[3];
    for (int i = 0; i < 3; i++) hp[i] = (double)(ray->origin.k[i] + ray->direction.k[i] * t);
    ivp_ledge_unscaled_qr_t qr;
    ivp_ledge_calc_unscaled_qr_vals(ledge, &tri->edges[0], hp, &qr);
    if (ivp_ledge_unscaled_qr_is_outside(&qr)) return IVP_NOT_FOUND;
    *hit_dist = t;
    return IVP_OK;
}
