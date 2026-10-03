/* ivp_controller_phantom.c -- Phantom (sensor/trigger) controller
 *
 * See ivp_controller_phantom.h for the reference semantics emulated here
 * (IVP_Controller_Phantom + phantom mindists of ivp_mindist.cxx).
 */

#include <ivp/controllers/ivp_controller_phantom.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_compact_triangle.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/math/ivp_transform.h>
#include <math.h>
#include <string.h>

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_phantom_template_init(ivp_phantom_template_t *t)
{
    /* Reference IVP_Template_Phantom (ivp_phantom.cxx:9-16) */
    t->dont_check_for_unmoveables = false;
    t->exit_policy_extra_radius = IVP_REAL(0.5);
    t->respect_collision_filter = true;
}

static void phantom_common(ivp_phantom_state_t *state, ivp_phantom_event_fn_t event_fn, void *ctx)
{
    memset(state, 0, sizeof(*state));
    state->exit_policy_extra_radius = IVP_REAL(0.5);
    state->respect_collision_filter = true;
    state->event_fn = event_fn;
    state->event_ctx = ctx;
    state->active = true;
}

void ivp_phantom_init_object(ivp_phantom_state_t *state,
                             ivp_object_t *owner,
                             const ivp_phantom_template_t *tmpl,
                             ivp_phantom_event_fn_t event_fn,
                             void *event_ctx)
{
    phantom_common(state, event_fn, event_ctx);
    state->shape = IVP_PHANTOM_OBJECT;
    state->owner = owner;
    if (tmpl) {
        state->exit_policy_extra_radius = tmpl->exit_policy_extra_radius;
        state->dont_check_for_unmoveables = tmpl->dont_check_for_unmoveables;
        state->respect_collision_filter = tmpl->respect_collision_filter;
    }
}

void ivp_phantom_init_sphere(ivp_phantom_state_t *state,
                               const ivp_vec3r_t *center_ws,
                               ivp_real_t radius,
                               ivp_phantom_event_fn_t event_fn,
                               void *event_ctx)
{
    phantom_common(state, event_fn, event_ctx);
    state->shape     = IVP_PHANTOM_SPHERE;
    state->center_ws = *center_ws;
    state->radius    = radius;
}

void ivp_phantom_init_aabb(ivp_phantom_state_t *state,
                             const ivp_vec3r_t *min_ws,
                             const ivp_vec3r_t *max_ws,
                             ivp_phantom_event_fn_t event_fn,
                             void *event_ctx)
{
    phantom_common(state, event_fn, event_ctx);
    state->shape       = IVP_PHANTOM_AABB;
    state->aabb_min_ws = *min_ws;
    state->aabb_max_ws = *max_ws;
}

void ivp_phantom_attach(ivp_phantom_state_t *state,
                          ivp_object_t *owner,
                          const ivp_vec3r_t *offset_os)
{
    state->owner     = owner;
    state->offset_os = *offset_os;
}

/* ── Convex shapes and GJK distance ─────────────────────────────────────── */

#define PH_MAX_PTS 512

typedef struct ph_shape {
    double pts[PH_MAX_PTS][3];
    int    n;
    double margin;            /* sphere radius around the point set */
} ph_shape_t;

static double d3_dot(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

static void ph_support(const ph_shape_t *s, const double d[3], double out[3])
{
    int best = 0;
    double bd = d3_dot(s->pts[0], d);
    for (int i = 1; i < s->n; i++) {
        double v = d3_dot(s->pts[i], d);
        if (v > bd) { bd = v; best = i; }
    }
    out[0] = s->pts[best][0]; out[1] = s->pts[best][1]; out[2] = s->pts[best][2];
}

/* closest point to the origin on a simplex; reduces the simplex in place */
static void closest_on_simplex(double s[4][3], int *n, double v[3])
{
    if (*n == 1) {
        v[0] = s[0][0]; v[1] = s[0][1]; v[2] = s[0][2];
        return;
    }
    if (*n == 2) {
        double ab[3] = { s[1][0] - s[0][0], s[1][1] - s[0][1], s[1][2] - s[0][2] };
        double t = -d3_dot(s[0], ab) / (d3_dot(ab, ab) > 0 ? d3_dot(ab, ab) : 1.0);
        if (t <= 0.0) { *n = 1; closest_on_simplex(s, n, v); return; }
        if (t >= 1.0) {
            for (int k = 0; k < 3; k++) s[0][k] = s[1][k];
            *n = 1; closest_on_simplex(s, n, v); return;
        }
        for (int k = 0; k < 3; k++) v[k] = s[0][k] + ab[k] * t;
        return;
    }
    if (*n == 3) {
        /* Ericson, closest point on triangle to origin (p = 0) */
        const double *a = s[0], *b = s[1], *c = s[2];
        double ab[3], ac[3], ap[3];
        for (int k = 0; k < 3; k++) { ab[k] = b[k] - a[k]; ac[k] = c[k] - a[k]; ap[k] = -a[k]; }
        double d1 = d3_dot(ab, ap), d2 = d3_dot(ac, ap);
        if (d1 <= 0.0 && d2 <= 0.0) { *n = 1; closest_on_simplex(s, n, v); return; }
        double bp[3] = { -b[0], -b[1], -b[2] };
        double d3 = d3_dot(ab, bp), d4 = d3_dot(ac, bp);
        if (d3 >= 0.0 && d4 <= d3) {
            for (int k = 0; k < 3; k++) s[0][k] = b[k];
            *n = 1; closest_on_simplex(s, n, v); return;
        }
        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            *n = 2; closest_on_simplex(s, n, v); return;           /* edge ab */
        }
        double cp[3] = { -c[0], -c[1], -c[2] };
        double d5 = d3_dot(ab, cp), d6 = d3_dot(ac, cp);
        if (d6 >= 0.0 && d5 <= d6) {
            for (int k = 0; k < 3; k++) s[0][k] = c[k];
            *n = 1; closest_on_simplex(s, n, v); return;
        }
        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            for (int k = 0; k < 3; k++) s[1][k] = c[k];             /* edge ac */
            *n = 2; closest_on_simplex(s, n, v); return;
        }
        double va = d3 * d6 - d5 * d4;
        if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
            for (int k = 0; k < 3; k++) { s[0][k] = b[k]; s[1][k] = c[k]; } /* edge bc */
            *n = 2; closest_on_simplex(s, n, v); return;
        }
        double denom = va + vb + vc;
        if (denom == 0.0) { *n = 2; closest_on_simplex(s, n, v); return; }
        double vv = vb / denom, ww = vc / denom;
        for (int k = 0; k < 3; k++) v[k] = a[k] + ab[k] * vv + ac[k] * ww;
        return;
    }
    /* tetrahedron: check whether the origin is inside, else closest face */
    static const int faces[4][4] = { { 0, 1, 2, 3 }, { 0, 2, 3, 1 }, { 0, 3, 1, 2 }, { 1, 3, 2, 0 } };
    bool inside = true;
    double best_q = -1.0;
    double best_v[3] = { 0, 0, 0 };
    double best_s[4][3];
    int best_n = 0;
    for (int f = 0; f < 4; f++) {
        const double *p0 = s[faces[f][0]], *p1 = s[faces[f][1]], *p2 = s[faces[f][2]], *p3 = s[faces[f][3]];
        double e1[3], e2[3], nrm[3];
        for (int k = 0; k < 3; k++) { e1[k] = p1[k] - p0[k]; e2[k] = p2[k] - p0[k]; }
        nrm[0] = e1[1] * e2[2] - e1[2] * e2[1];
        nrm[1] = e1[2] * e2[0] - e1[0] * e2[2];
        nrm[2] = e1[0] * e2[1] - e1[1] * e2[0];
        double side_o = -d3_dot(nrm, p0);
        double d3p[3] = { p3[0] - p0[0], p3[1] - p0[1], p3[2] - p0[2] };
        double side_p = d3_dot(nrm, d3p);
        if (side_o * side_p < 0.0) {
            inside = false;
            double tri[4][3];
            for (int k = 0; k < 3; k++) { tri[0][k] = p0[k]; tri[1][k] = p1[k]; tri[2][k] = p2[k]; }
            int tn = 3;
            double tv[3];
            closest_on_simplex(tri, &tn, tv);
            double q = d3_dot(tv, tv);
            if (best_q < 0.0 || q < best_q) {
                best_q = q;
                memcpy(best_v, tv, sizeof(tv));
                memcpy(best_s, tri, sizeof(tri));
                best_n = tn;
            }
        }
    }
    if (inside) {
        v[0] = v[1] = v[2] = 0.0;
        return;
    }
    memcpy(s, best_s, sizeof(best_s));
    *n = best_n;
    memcpy(v, best_v, sizeof(best_v));
}

/* GJK distance between the cores of two shapes minus their margins. */
static double ph_distance(const ph_shape_t *A, const ph_shape_t *B)
{
    if (A->n <= 0 || B->n <= 0) return 1e30;
    double s[4][3];
    int n = 0;
    double v[3] = { A->pts[0][0] - B->pts[0][0], A->pts[0][1] - B->pts[0][1], A->pts[0][2] - B->pts[0][2] };
    double dist = sqrt(d3_dot(v, v));
    for (int iter = 0; iter < 64; iter++) {
        double qv = d3_dot(v, v);
        if (qv < 1e-18) { dist = 0.0; break; }
        double nd[3] = { -v[0], -v[1], -v[2] };
        double pa[3], pb[3];
        ph_support(A, nd, pa);
        ph_support(B, v, pb);
        double w[3] = { pa[0] - pb[0], pa[1] - pb[1], pa[2] - pb[2] };
        if (qv - d3_dot(v, w) <= 1e-10 * qv + 1e-14) { dist = sqrt(qv); break; }
        bool dup = false;
        for (int i = 0; i < n; i++) {
            if (s[i][0] == w[0] && s[i][1] == w[1] && s[i][2] == w[2]) dup = true;
        }
        if (dup) { dist = sqrt(qv); break; }
        s[n][0] = w[0]; s[n][1] = w[1]; s[n][2] = w[2];
        n++;
        closest_on_simplex(s, &n, v);
        dist = sqrt(d3_dot(v, v));
        if (n == 4) { dist = 0.0; break; }
    }
    return dist - A->margin - B->margin;
}

static void add_pt(ph_shape_t *s, double x, double y, double z)
{
    if (s->n >= PH_MAX_PTS) return;
    s->pts[s->n][0] = x; s->pts[s->n][1] = y; s->pts[s->n][2] = z;
    s->n++;
}

static void ledge_to_shape(const ivp_compact_ledge_t *ledge, const ivp_transform_r_t *m, ph_shape_t *s)
{
    s->n = 0;
    s->margin = 0.0;
    int nt = ivp_ledge_get_n_triangles(ledge);
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
    for (int i = 0; i < nt; i++, tri = ivp_tri_get_next(tri)) {
        for (int e = 0; e < 3; e++) {
            const ivp_compact_poly_point_t *p = ivp_edge_get_start_point(ivp_tri_get_edge(tri, e), ledge);
            ivp_vec3r_t os, ws;
            ivp_vec3r_set(&os, (ivp_real_t)p->k[0], (ivp_real_t)p->k[1], (ivp_real_t)p->k[2]);
            ivp_transform_point(&ws, m, &os);
            add_pt(s, (double)ws.k[0], (double)ws.k[1], (double)ws.k[2]);
        }
    }
}

typedef struct ph_ledge_list {
    const ivp_compact_ledge_t *ledges[64];
    int n;
} ph_ledge_list_t;

static bool collect_ledge(const ivp_compact_ledge_t *ledge, void *user)
{
    ph_ledge_list_t *l = (ph_ledge_list_t *)user;
    if (l->n < 64) l->ledges[l->n++] = ledge;
    return true;
}

static void object_ledges(const ivp_object_t *obj, ph_ledge_list_t *out)
{
    out->n = 0;
    if (!obj->surman) return;
    const ivp_compact_ledge_t *single = ivp_surman_get_single_convex(obj->surman);
    if (single) {
        out->ledges[out->n++] = single;
        return;
    }
    const ivp_compact_ledgetree_node_t *root = ivp_surman_get_ledgetree_root(obj->surman);
    if (root) ivp_ledgetree_visit_all(root, collect_ledge, out);
}

static bool object_is_ball(const ivp_object_t *obj, double *radius)
{
    if (obj->type != IVP_OBJECT_BALL) return false;
    *radius = (double)obj->physical_core->upper_limit_radius;
    if (obj->surman && ivp_surman_get_type(obj->surman) == IVP_SURMAN_BALL) {
        *radius = (double)((const ivp_surman_ball_t *)obj->surman)->radius;
    }
    return true;
}

/* distance from shape A to the object's convex parts (min over ledges) */
static double distance_shape_object(const ph_shape_t *A, const ivp_object_t *obj)
{
    static ph_shape_t B; /* large; not re-entrant (single-threaded engine) */
    double r;
    if (object_is_ball(obj, &r)) {
        B.n = 0;
        B.margin = r;
        add_pt(&B, (double)obj->physical_core->m_world_f_core_last_psi.pos.k[0],
               (double)obj->physical_core->m_world_f_core_last_psi.pos.k[1],
               (double)obj->physical_core->m_world_f_core_last_psi.pos.k[2]);
        return ph_distance(A, &B);
    }
    ph_ledge_list_t list;
    object_ledges(obj, &list);
    if (list.n == 0) {
        /* no geometry: bounding sphere fallback */
        B.n = 0;
        B.margin = (double)obj->physical_core->upper_limit_radius;
        add_pt(&B, (double)obj->physical_core->m_world_f_core_last_psi.pos.k[0],
               (double)obj->physical_core->m_world_f_core_last_psi.pos.k[1],
               (double)obj->physical_core->m_world_f_core_last_psi.pos.k[2]);
        return ph_distance(A, &B);
    }
    double best = 1e30;
    for (int i = 0; i < list.n; i++) {
        ledge_to_shape(list.ledges[i], &obj->physical_core->m_world_f_core_last_psi, &B);
        double d = ph_distance(A, &B);
        if (d < best) best = d;
    }
    return best;
}

static void phantom_sphere_center(const ivp_phantom_state_t *state, double c[3])
{
    ivp_vec3r_t cw = state->center_ws;
    if (state->owner) {
        ivp_transform_point(&cw, &state->owner->physical_core->m_world_f_core_last_psi, &state->offset_os);
    }
    c[0] = (double)cw.k[0]; c[1] = (double)cw.k[1]; c[2] = (double)cw.k[2];
}

ivp_real_t ivp_phantom_distance_to_object(const ivp_phantom_state_t *state,
                                          const ivp_object_t *obj)
{
    static ph_shape_t A;
    switch (state->shape) {
    case IVP_PHANTOM_SPHERE: {
        double c[3];
        phantom_sphere_center(state, c);
        A.n = 0;
        A.margin = (double)state->radius;
        add_pt(&A, c[0], c[1], c[2]);
        return (ivp_real_t)distance_shape_object(&A, obj);
    }
    case IVP_PHANTOM_AABB: {
        double mn[3], mx[3];
        for (int k = 0; k < 3; k++) { mn[k] = (double)state->aabb_min_ws.k[k]; mx[k] = (double)state->aabb_max_ws.k[k]; }
        if (state->owner) {
            double c[3];
            phantom_sphere_center(state, c);
            for (int k = 0; k < 3; k++) {
                double h = (mx[k] - mn[k]) * 0.5;
                mn[k] = c[k] - h; mx[k] = c[k] + h;
            }
        }
        A.n = 0;
        A.margin = 0.0;
        for (int i = 0; i < 8; i++) {
            add_pt(&A, (i & 1) ? mx[0] : mn[0], (i & 2) ? mx[1] : mn[1], (i & 4) ? mx[2] : mn[2]);
        }
        return (ivp_real_t)distance_shape_object(&A, obj);
    }
    case IVP_PHANTOM_OBJECT:
    default: {
        if (!state->owner) return (ivp_real_t)1e30;
        double r;
        if (object_is_ball(state->owner, &r)) {
            A.n = 0;
            A.margin = r;
            add_pt(&A, (double)state->owner->physical_core->m_world_f_core_last_psi.pos.k[0],
                   (double)state->owner->physical_core->m_world_f_core_last_psi.pos.k[1],
                   (double)state->owner->physical_core->m_world_f_core_last_psi.pos.k[2]);
            return (ivp_real_t)distance_shape_object(&A, obj);
        }
        ph_ledge_list_t list;
        object_ledges(state->owner, &list);
        double best = 1e30;
        for (int i = 0; i < list.n; i++) {
            ledge_to_shape(list.ledges[i], &state->owner->physical_core->m_world_f_core_last_psi, &A);
            double d = distance_shape_object(&A, obj);
            if (d < best) best = d;
        }
        return (ivp_real_t)best;
    }
    }
}

/* ── Tracking helpers ───────────────────────────────────────────────────── */

static int find_tracked(const ivp_phantom_state_t *state, const ivp_object_t *obj)
{
    for (int i = 0; i < state->n_tracked; i++) {
        if (state->tracked[i] == obj) return i;
    }
    return -1;
}

static void store_track_reference(ivp_phantom_state_t *state, int idx, const ivp_object_t *obj,
                                  ivp_real_t uncertainty)
{
    ivp_phantom_track_t *t = &state->track[idx];
    t->uncertainty = uncertainty;
    t->ref_pos_obj = obj->physical_core->pos_world_f_core_last_psi;
    t->ref_q_obj = obj->physical_core->q_world_f_core_last_psi;
    if (state->owner) {
        t->ref_pos_phantom = state->owner->physical_core->pos_world_f_core_last_psi;
        t->ref_q_phantom = state->owner->physical_core->q_world_f_core_last_psi;
    }
}

static void remove_tracked(ivp_phantom_state_t *state, int index)
{
    state->n_tracked--;
    if (index < state->n_tracked) {
        state->tracked[index] = state->tracked[state->n_tracked];
        state->track[index] = state->track[state->n_tracked];
    }
}

/* upper bound of the surface movement of a core since a reference pose
 * (translation + rotation angle * radius), reference hull manager role */
static ivp_real_t core_movement(const ivp_core_t *c, const ivp_vec3r_t *ref_pos, const ivp_quatr_t *ref_q)
{
    ivp_vec3r_t d;
    ivp_vec3r_sub(&d, &c->pos_world_f_core_last_psi, ref_pos);
    ivp_real_t move = ivp_vec3r_length(&d);
    ivp_real_t dot = ivp_fabs(ivp_quat_dot(&c->q_world_f_core_last_psi, ref_q));
    if (dot > IVP_REAL(1.0)) dot = IVP_REAL(1.0);
    ivp_real_t angle = IVP_REAL(2.0) * ivp_acos(dot);
    return move + angle * c->upper_limit_radius;
}

static bool pair_filtered(const ivp_phantom_state_t *state, const ivp_environment_t *env,
                          const ivp_object_t *obj)
{
    if (!state->respect_collision_filter || !state->owner) return false;
    if (ivp_objects_share_no_collision_group(state->owner, obj)) return true;
    if (env && !ivp_collision_filter_check(&env->collision_filter, state->owner, obj)) return true;
    return false;
}

static void phantom_check_object(ivp_phantom_state_t *state, const ivp_environment_t *env,
                                 ivp_object_t *obj)
{
    if (obj == state->owner) return;
    if (state->dont_check_for_unmoveables && obj->physical_core->physical_unmoveable) return;
    if (obj->physical_core->physical_unmoveable && (!state->owner || state->owner->physical_core->physical_unmoveable)) return;
    if (pair_filtered(state, env, obj)) return;

    int idx = find_tracked(state, obj);
    if (idx < 0) {
        ivp_real_t dist = ivp_phantom_distance_to_object(state, obj);
        if (dist <= IVP_REAL(0.0)) {
            if (state->n_tracked >= IVP_PHANTOM_MAX_TRACKED) return;
            idx = state->n_tracked++;
            state->tracked[idx] = obj;
            store_track_reference(state, idx, obj, -dist + state->exit_policy_extra_radius);
            if (state->event_fn) state->event_fn(state->event_ctx, state->owner, obj, true);
        }
        return;
    }

    /* lazy re-check once the pair moved more than the hull uncertainty */
    ivp_phantom_track_t *t = &state->track[idx];
    ivp_real_t moved = core_movement(obj->physical_core, &t->ref_pos_obj, &t->ref_q_obj);
    if (state->owner) {
        moved += core_movement(state->owner->physical_core, &t->ref_pos_phantom, &t->ref_q_phantom);
    }
    if (moved <= t->uncertainty) return;

    ivp_real_t dist = ivp_phantom_distance_to_object(state, obj);
    if (dist > IVP_REAL(0.0)) {
        remove_tracked(state, idx);
        if (state->event_fn) state->event_fn(state->event_ctx, state->owner, obj, false);
    } else {
        store_track_reference(state, idx, obj, -dist + state->exit_policy_extra_radius);
    }
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_phantom(const ivp_event_sim_t *event_sim,
                              ivp_object_t *obj,
                              void *ctx)
{
    ivp_phantom_state_t *state = (ivp_phantom_state_t *)ctx;
    if (!state || !state->active || !obj) return;
    ivp_environment_t *env = event_sim->environment;

    bool owner_moves = state->owner && !state->owner->physical_core->physical_unmoveable;
    if (owner_moves) {
        /* A moving phantom scans every object, sleeping ones included. */
        if (obj != state->owner || !env) return;
        int n = ivp_environment_get_object_count(env);
        for (int i = 0; i < n; i++) {
            ivp_object_t *o = ivp_environment_get_object(env, i);
            if (o) phantom_check_object(state, env, o);
        }
        return;
    }
    /* Static phantom: only moving objects can change their status. */
    phantom_check_object(state, env, obj);
}
