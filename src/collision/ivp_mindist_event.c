/* ivp_mindist_event.c — port of IVP_Mindist_Event_Solver and IVP_3D_Solver
 *
 * Reference: ivp_physics/ivp_mindist_event.{hxx,cxx}, ivp_3d_solver.{hxx,cxx}.
 * Finds the exact time (within the current PSI) at which a mindist reaches
 * its collision distance or changes its Voronoi sector.
 */
#include "collision/ivp_mindist_event_internal.h"
#include <ivp/dynamics/ivp_object.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <math.h>
#include "collision/ivp_lvec.h"
#include "collision/ivp_cls_internal.h"
#include "dynamics/ivp_env_internal.h"
#include "dynamics/ivp_env_hot.h"
#include "collision/ivp_matrix_cache.h"

#ifdef IVP_REAL_DOUBLE
#define NULLSTELLE_EPS 1e-8
#else
#define NULLSTELLE_EPS 3e-3f
#endif

#define P_MAXD(a, b) (((a) > (b)) ? (a) : (b))
#define P_MIND(a, b) (((a) < (b)) ? (a) : (b))

/* ── IVP_3D_Solver ──────────────────────────────────────────────────────── */

typedef struct solver3d solver3d_t;
typedef D (*solver3d_value_fn)(solver3d_t *s, const ivp_transform_r_t *A, const ivp_transform_r_t *B);

struct solver3d {
    solver3d_value_fn get_value;
    D max_deviation;
    D inv_max_deviation;
    /* per-solver data */
    ivp_vec3r_t v[5];
    ivp_vec3f_t f[2];
    D scalar;
};

static void s3d_set_max_deviation(solver3d_t *s, D dev) {
    s->max_deviation = dev;
    s->inv_max_deviation = 1.0f / s->max_deviation;
}

static ivp_time_t s3d_calc_nullstelle(solver3d_t *s, ivp_time_t t0, ivp_time_t t1, D value, D v0, D v1,
                                      struct ivp_object *a, struct ivp_object *b)
{
    IVP_ASSERT(v0 >= value);   /* ivp_3d_solver.cxx:23 */
    IVP_ASSERT(v1 <= value);
    ivp_transform_r_t A, B;
    int counter = 0;
    for (;;) {
        D dv = v1 - v0;
        ivp_time_t tt = t0;
        tt += (double)((value - v0) * (D)ivp_time_diff(t1, t0) / dv);
        if ((counter & 0x3) == 3) {
            if (counter > 64) return t0;
            const D ratio = 0.25f;
            const D inv_ratio = 1.0f - ratio;
            D dt0 = (D)ivp_time_diff(t0, tt);
            D dt1 = (D)ivp_time_diff(t1, tt);
            D mid2 = dt0 + dt1;
            tt += (double)(mid2 * (inv_ratio * 0.5f));
        }
        ivp_object_calc_at_matrix(a, tt, &A);
        ivp_object_calc_at_matrix(b, tt, &B);
        D vv = s->get_value(s, &A, &B);
        if (fabs((double)(vv - value)) < (double)NULLSTELLE_EPS) return tt;
        counter++;
        if (vv >= value) { v0 = vv; t0 = tt; }
        else { v1 = vv; t1 = tt; }
    }
}

static bool s3d_find_first_t_for_value_max_dev(solver3d_t *s, D value, ivp_time_t t_now, ivp_time_t t_max,
                                               int t_now_cache_index, matrix_cache_t *mc_A, matrix_cache_t *mc_B,
                                               const D *opt_val_at_t_now, ivp_time_t *t_out)
{
    D v0;
    if (opt_val_at_t_now) {
        v0 = *opt_val_at_t_now;
    } else {
        IVP_ASSERT(ivp_time_diff(mc_A->base_time, t_now) == 0.0);   /* ivp_3d_solver.cxx:100 */
        IVP_ASSERT(ivp_time_diff(mc_B->base_time, t_now) == 0.0);
        IVP_ASSERT(t_now_cache_index == 0);
        ivp_transform_r_t *A = mc_calc_matrix_at_now(mc_A, t_now, 0);
        ivp_transform_r_t *B = mc_calc_matrix_at_now(mc_B, t_now, 0);
        v0 = s->get_value(s, A, B);
    }
    if (v0 <= value) {
        *t_out = t_now;
        return true;
    }
    ivp_time_t t0 = t_now;
    int cache1_index = t_now_cache_index;
    IVP_ASSERT(ivp_time_diff(t_now, t_max) <= 0.0);   /* ivp_3d_solver.cxx:119 */
    for (;;) {
        D t_step = (v0 - value) * s->inv_max_deviation;
        if ((D)ivp_time_diff(t0, t_max) + t_step > 0.0f) return false;
        t_step += t_step;
        if ((D)ivp_time_diff(t0, t_max) + t_step > 0.0f) {
            t_step = (D)ivp_time_diff(t_max, t0) + P_RES_EPS;
        }
        int int_t_step = (int)(t_step * IVP_3D_SOLVER_STEPS_PER_SECOND);
        if (int_t_step < 1) int_t_step = 1;
        cache1_index += int_t_step;
        IVP_ASSERT(cache1_index <= IVP_3D_SOLVER_MAX_STEPS_PER_PSI);   /* ivp_3d_solver.cxx:148 */
        if (cache1_index > IVP_3D_SOLVER_MAX_STEPS_PER_PSI) cache1_index = IVP_3D_SOLVER_MAX_STEPS_PER_PSI;
        /* int * (1.0f / 200) is a float product in the reference */
        D d_tstep = (D)((float)int_t_step * (1.0f / (float)IVP_3D_SOLVER_STEPS_PER_SECOND));
        ivp_time_t t1 = t0;
        t1 += (double)d_tstep;
        ivp_transform_r_t *A = mc_calc_matrix_at(mc_A, t1, cache1_index);
        ivp_transform_r_t *B = mc_calc_matrix_at(mc_B, t1, cache1_index);
        D v1 = s->get_value(s, A, B);
        if (v1 <= value) {
            ivp_time_t t = s3d_calc_nullstelle(s, t0, t1, value, v0, v1, mc_A->object, mc_B->object);
            if (ivp_time_diff(t, t_max) > 0.0) return false;
            *t_out = t;
            return true;
        }
        if (cache1_index == IVP_3D_SOLVER_MAX_STEPS_PER_PSI) return false;
        t0 = t1;
        v0 = v1;
    }
}

static bool s3d_find_first_t_for_value_coll(solver3d_t *s, D value, D value2, ivp_time_t t_now, ivp_time_t t_max,
                                            matrix_cache_t *mc_A, matrix_cache_t *mc_B,
                                            const D *opt_val_at_t_now, ivp_time_t *t_out)
{
    D tnow_val;
    {
        IVP_ASSERT(ivp_time_diff(mc_A->base_time, t_now) == 0.0);   /* ivp_3d_solver.cxx:265 */
        IVP_ASSERT(ivp_time_diff(mc_B->base_time, t_now) == 0.0);
        ivp_transform_r_t *A = mc_calc_matrix_at_now(mc_A, t_now, 0);
        ivp_transform_r_t *B = mc_calc_matrix_at_now(mc_B, t_now, 0);
        if (!opt_val_at_t_now) tnow_val = s->get_value(s, A, B);
        else tnow_val = *opt_val_at_t_now;
    }
    if (tnow_val > value) {
        return s3d_find_first_t_for_value_max_dev(s, value, t_now, t_max, 0, mc_A, mc_B, &tnow_val, t_out);
    }
    D inv_max_dev = 1.0f / s->max_deviation;
    D v0 = tnow_val;
    ivp_time_t t0 = t_now;
    int cache1_index = 0;
    while (ivp_time_diff(t0, t_max) < 0.0) {
        D t_step = (v0 - value2) * inv_max_dev;
        if ((D)ivp_time_diff(t0, t_max) + t_step > 0.0f) {
            t_step = (D)ivp_time_diff(t_max, t0) + P_RES_EPS;
        } else if (t_step < 0.0f) {
            t_step = 0.0f;
        }
        int int_t_step = (int)(t_step * IVP_3D_SOLVER_STEPS_PER_SECOND);
        if (int_t_step < 1) int_t_step = 1;
        D real_t_step = (D)((float)int_t_step * (1.0f / (float)IVP_3D_SOLVER_STEPS_PER_SECOND));
        ivp_time_t t1 = t0 + (double)real_t_step;
        cache1_index += int_t_step;
        if (cache1_index > IVP_3D_SOLVER_MAX_STEPS_PER_PSI) cache1_index = IVP_3D_SOLVER_MAX_STEPS_PER_PSI;
        ivp_transform_r_t *A = mc_calc_matrix_at(mc_A, t1, cache1_index);
        ivp_transform_r_t *B = mc_calc_matrix_at(mc_B, t1, cache1_index);
        D v1 = s->get_value(s, A, B);
        if (v1 > value) {
            if (ivp_time_diff(t1, t_max) > 0.0) return false;
            D t1_val = v1;
            return s3d_find_first_t_for_value_max_dev(s, value, t1, t_max, cache1_index, mc_A, mc_B, &t1_val, t_out);
        }
        if (v1 <= tnow_val) {
            *t_out = t0;
            return true;
        }
        t0 = t1;
        v0 = v1;
    }
    return false;
}

/* ── solver value functions ─────────────────────────────────────────────── */

/* PF_COLL: v[0]=point_object, v[1]=hesse_of_area_object, v[2]=point_of_area_object */
static D sv_pf_coll(solver3d_t *t, const ivp_transform_r_t *A, const ivp_transform_r_t *B)
{
    ivp_vec3r_t point_world, hesse_world, point_of_area_world, d;
    lvm_vmult4(A, &t->v[0], &point_world);
    lvm_vmult4(B, &t->v[2], &point_of_area_world);
    lvm_vmult3(B, &t->v[1], &hesse_world);
    lvp_subtract2(&d, &point_world, &point_of_area_world);
    return lvp_dot(&d, &hesse_world);
}

/* VEC_PARALLEL_AREA: v[0]=vec_object, v[1]=hesse_of_area_object */
static D sv_vec_parallel_area(solver3d_t *t, const ivp_transform_r_t *A, const ivp_transform_r_t *B)
{
    ivp_vec3r_t vec_world, hesse_world;
    lvm_vmult3(A, &t->v[0], &vec_world);
    lvm_vmult3(B, &t->v[1], &hesse_world);
    return lvp_dot(&vec_world, &hesse_world);
}

/* DISTANCE_OF_TWO_POINTS: v[0]=A_object, v[1]=B_object, v[2]=normized_direction_world_at_t0 */
static D sv_distance_of_two_points(solver3d_t *t, const ivp_transform_r_t *A, const ivp_transform_r_t *B)
{
    ivp_vec3r_t A_world, B_world, dir_world;
    lvm_vmult4(A, &t->v[0], &A_world);
    lvm_vmult4(B, &t->v[1], &B_world);
    lvp_subtract2(&dir_world, &B_world, &A_world);
    D res = lvp_quad_length(&dir_world);
    const D linear_factor = 1.2f;
    D linear_dist = lvp_dot(&dir_world, &t->v[2]) * linear_factor;
    if (linear_dist * (D)fabs((double)linear_dist) < res) return linear_dist;
    return LV_SQRT(res);
}

/* S_VALS: v[0]=P_object, v[1]=K_object, v[2]=K_vec_object */
static D sv_s_vals(solver3d_t *t, const ivp_transform_r_t *A, const ivp_transform_r_t *B)
{
    ivp_vec3r_t P_world, K_world, K_vec_world, PK;
    lvm_vmult4(A, &t->v[0], &P_world);
    lvm_vmult4(B, &t->v[1], &K_world);
    lvm_vmult3(B, &t->v[2], &K_vec_world);
    lvp_subtract2(&PK, &K_world, &P_world);
    return lvp_dot(&PK, &K_vec_world);
}

/* PK_KK: v[0]=P_object, v[1]=P_ne_vec_object, v[2]=K_object, v[3]=K_vec_object */
static D sv_pk_kk(solver3d_t *t, const ivp_transform_r_t *A, const ivp_transform_r_t *B)
{
    ivp_vec3r_t P_world, P_ne_vec_world, K_world, K_vec_world, KP, Lot, Lot2;
    lvm_vmult4(A, &t->v[0], &P_world);
    lvm_vmult3(A, &t->v[1], &P_ne_vec_world);
    lvm_vmult4(B, &t->v[2], &K_world);
    lvm_vmult3(B, &t->v[3], &K_vec_world);
    lvp_subtract2(&KP, &P_world, &K_world);
    lvp_cross(&Lot, &K_vec_world, &KP);
    /* retail: double cross product, normize(), then a z, y, x dot */
    lvp_cross(&Lot2, &Lot, &K_vec_world);
    (void)lvp_normize(&Lot2);
    return P_ne_vec_world.k[2] * Lot2.k[2] + P_ne_vec_world.k[1] * Lot2.k[1] + P_ne_vec_world.k[0] * Lot2.k[0];
}

/* PK_COLL: f[0]=P_object, f[1]=K_object, v[0]=K_vec_object, v[1]=K_Lot_object, scalar=plane_offset */
static D sv_pk_coll(solver3d_t *t, const ivp_transform_r_t *A, const ivp_transform_r_t *B)
{
    ivp_vec3r_t P_world, K_world, K_vec_world, K_Lot_world, PK, H;
    lvm_vmult4_fp(A, &t->f[0], &P_world);
    lvm_vmult4_fp(B, &t->f[1], &K_world);
    lvm_vmult3(B, &t->v[0], &K_vec_world);
    lvm_vmult3(B, &t->v[1], &K_Lot_world);
    lvp_subtract2(&PK, &K_world, &P_world);
    lvp_cross(&H, &PK, &K_vec_world);
    D res = lvp_real_length(&H);
    D res2 = lvp_dot(&H, &K_Lot_world) + t->scalar;
    if (res2 < res) return res2;
    return res;
}

/* PF_NPF: v[0]=vec_object, v[1]=hesse_of_area_object */
static D sv_pf_npf(solver3d_t *t, const ivp_transform_r_t *A, const ivp_transform_r_t *B)
{
    ivp_vec3r_t hesse_world, hesse_other_object;
    lvm_vmult3(B, &t->v[1], &hesse_world);
    lvm_vimult3(A, &hesse_world, &hesse_other_object);
    return lvp_dot(&hesse_other_object, &t->v[0]);
}

/* KK_COLL: v[0]=k_point_object, v[1]=k_vec_object, v[2]=l_point_object, v[3]=l_vec_object, scalar=side */
static D sv_kk_coll(solver3d_t *t, const ivp_transform_r_t *K, const ivp_transform_r_t *L)
{
    ivp_vec3r_t k_vec_world, l_vec_world, k_point_world, l_point_world, H;
    lvm_vmult3(K, &t->v[1], &k_vec_world);
    lvm_vmult4(K, &t->v[0], &k_point_world);
    lvm_vmult3(L, &t->v[3], &l_vec_world);
    lvm_vmult4(L, &t->v[2], &l_point_world);
    lvp_cross(&H, &k_vec_world, &l_vec_world);
    D res = lvp_dot(&H, &k_point_world) - lvp_dot(&H, &l_point_world);
    D iH_len = (D)lv_isqrt_float((float)lvp_quad_length(&H));
    res *= iH_len;
    return res * t->scalar;
}

/* KK_PARALLEL: v[1]=k_vec_object, v[3]=l_vec_object */
static D sv_kk_parallel(solver3d_t *t, const ivp_transform_r_t *K, const ivp_transform_r_t *L)
{
    ivp_vec3r_t k_vec_world, l_vec_world, H;
    lvm_vmult3(K, &t->v[1], &k_vec_world);
    lvm_vmult3(L, &t->v[3], &l_vec_world);
    lvp_cross(&H, &k_vec_world, &l_vec_world);
    return lvp_quad_length(&H);
}

static void s3d_init(solver3d_t *s, solver3d_value_fn fn)
{
    s->get_value = fn;
    s->max_deviation = 0;
    s->inv_max_deviation = 0;
    for (int i = 0; i < 5; i++) lvp_zero(&s->v[i]);
    lvf_zero(&s->f[0]);
    lvf_zero(&s->f[1]);
    s->scalar = 0;
}

/* ── helpers ────────────────────────────────────────────────────────────── */

/* IVP_CLS.give_object_coords(e, m_cache) (cache ledge point overload, alignment check) */
static inline ivp_vec3f_t clp_point(const ivp_compact_edge_t *e, const ivp_cache_ledge_point_t *clp) {
    const ivp_compact_poly_point_t *p = give_object_coords_clp(e, clp);
    ivp_vec3f_t r; r.k[0] = p->k[0]; r.k[1] = p->k[1]; r.k[2] = p->k[2]; return r;
}

/* IVP_CLS.calc_hesse_vec_object_not_normized(edge, m_cache->get_compact_ledge(), out): the ledge
 * overload of give_object_coords (no alignment check) */
static void hesse_vec_not_normized(const ivp_compact_edge_t *edge, const ivp_cache_ledge_point_t *clp, ivp_vec3r_t *out)
{
    cls_calc_hesse_vec_object_not_normized(edge, clp->compact_ledge, out);
}

static inline D lvf_fast_real_length(const ivp_vec3f_t *a) { return LV_SQRT(lvf_quad_length(a)); }

/* ── PF ─────────────────────────────────────────────────────────────────── */

static void calc_next_event_PF(ivp_mindist_event_solver_t *mim, const ivp_compact_edge_t *P, const ivp_compact_edge_t *F,
                               ivp_cache_ledge_point_t *c_P, ivp_cache_ledge_point_t *c_F)
{
    ivp_mindist_t *md = mim->mindist;
    matrix_cache_t cache_P, cache_F;
    mc_init(&cache_P, c_P->clp_cache_object);
    mc_init(&cache_F, c_F->clp_cache_object);
    D max_rot_speed = (D)(cache_P.core->abs_omega + cache_F.core->abs_omega); /* IVP_FLOAT sum */
    mim->event_time_out = mim->t_max;
    solver3d_t pf;
    s3d_init(&pf, sv_pf_coll);
    {
        ivp_vec3f_t pp = clp_point(P, c_P), pa = clp_point(F, c_F);
        lvp_set_f(&pf.v[0], &pp);
        lvp_set_f(&pf.v[2], &pa);
        hesse_vec_not_normized(F, c_F, &pf.v[1]);
        (void)lvp_fast_normize(&pf.v[1]);
        s3d_set_max_deviation(&pf, mim->max_coll_speed);
        float radius = md->sum_extra_radius;
        D sec_dist = ivp_mindist_get_coll_dist(md) + (D)radius;
        D real_coll = (D)(ivp_env_get_mindist_settings(md->env)->real_coll_dist + 0.5f * radius); /* IVP_FLOAT expression */
        D now_len = (D)(md->len_numerator + radius); /* IVP_FLOAT expression */
        if (s3d_find_first_t_for_value_coll(&pf, sec_dist, real_coll, mim->t_now, mim->event_time_out, &cache_P,
                                            &cache_F, &now_len, &mim->event_time_out)) {
            mim->event_type_out = IVP_COLL_PF_COLL;
        }
    }
    solver3d_t npf;
    s3d_init(&npf, sv_pf_npf);
    npf.v[1] = pf.v[1];
    D sec_dist;
    {
        D real_dist = P_MIND(ivp_mindist_get_coll_dist(md), (D)md->len_numerator);
        real_dist += (D)(md->sum_extra_radius * 0.1f);
        /* IVP_FLOAT * IVP_FLOAT */
        sec_dist = (D)(-ivp_env_get_mindist_settings(md->env)->mindist_change_force_dist *
                       c_F->clp_object->physical_core->inv_object_diameter);
        sec_dist *= real_dist;
        sec_dist /= ivp_mindist_get_coll_dist(md);
    }
    ivp_vec3r_t wHesseF, hesse_in_P_object;
    lvm_vmult3(&c_F->clp_cache_object->m_world_f_object, &npf.v[1], &wHesseF);
    lvm_vimult3(&c_P->clp_cache_object->m_world_f_object, &wHesseF, &hesse_in_P_object);
    s3d_set_max_deviation(&npf, max_rot_speed + P_DOUBLE_EPS);
    D max_grad_change = npf.max_deviation * (D)ivp_time_diff(mim->event_time_out, mim->t_now);
    ivp_vec3f_t p_object = clp_point(P, c_P);
    for (const ivp_compact_edge_t *e = ivp_edge_get_opposite(ivp_edge_get_prev(P));;
         e = ivp_edge_get_opposite(ivp_edge_get_prev(e))) {
        ivp_vec3f_t p_next_object = clp_point(ivp_edge_get_next(e), c_P);
        ivp_vec3r_t vec_object;
        lvp_subtract2_ff(&vec_object, &p_next_object, &p_object);
        D grad = lvp_dot(&hesse_in_P_object, &vec_object);
        D qlen = lvp_quad_length(&vec_object);
        D ilen = (D)lv_isqrt_float((float)qlen);
        grad *= ilen;
        if (grad < max_grad_change) {
            lvp_set_multiple(&npf.v[0], &vec_object, ilen);
            if (s3d_find_first_t_for_value_max_dev(&npf, sec_dist, mim->t_now, mim->event_time_out, 0, &cache_P,
                                                   &cache_F, &grad, &mim->event_time_out)) {
                mim->event_type_out = IVP_COLL_PF_NPF;
            }
        }
        if (e == P) break;
    }
}

static void calc_next_event_BF(ivp_mindist_event_solver_t *mim, const ivp_compact_edge_t *F, ivp_cache_obj_t *c_B,
                               ivp_cache_ledge_point_t *c_F)
{
    ivp_mindist_t *md = mim->mindist;
    matrix_cache_t cache_P, cache_F;
    mc_init(&cache_P, c_B);
    mc_init(&cache_F, c_F->clp_cache_object);
    mim->event_time_out = mim->t_max;
    solver3d_t pf;
    s3d_init(&pf, sv_pf_coll);
    lvp_zero(&pf.v[0]);
    ivp_vec3f_t pa = clp_point(F, c_F);
    lvp_set_f(&pf.v[2], &pa);
    hesse_vec_not_normized(F, c_F, &pf.v[1]);
    (void)lvp_fast_normize(&pf.v[1]);
    s3d_set_max_deviation(&pf, mim->max_coll_speed);
    float radius = md->sum_extra_radius;
    D sec_dist = ivp_mindist_get_coll_dist(md) + (D)radius;
    D real_coll = (D)(ivp_env_get_mindist_settings(md->env)->real_coll_dist + 0.5f * radius); /* IVP_FLOAT expression */
    D now_len = (D)(md->len_numerator + radius); /* IVP_FLOAT expression */
    if (s3d_find_first_t_for_value_coll(&pf, sec_dist, real_coll, mim->t_now, mim->t_max, &cache_P, &cache_F,
                                        &now_len, &mim->event_time_out)) {
        mim->event_type_out = IVP_COLL_PF_COLL;
    }
}

/* ── KK ─────────────────────────────────────────────────────────────────── */

static void calc_next_event_KK(ivp_mindist_event_solver_t *mim, const ivp_compact_edge_t *K, const ivp_compact_edge_t *L,
                               ivp_cache_ledge_point_t *c_K, ivp_cache_ledge_point_t *c_L)
{
    ivp_mindist_t *md = mim->mindist;
    matrix_cache_t cache_K, cache_L;
    mc_init(&cache_K, c_K->clp_cache_object);
    mc_init(&cache_L, c_L->clp_cache_object);
    D sum_abs_omega = (D)(c_K->clp_object->physical_core->abs_omega + c_L->clp_object->physical_core->abs_omega); /* IVP_FLOAT sum */
    mim->event_time_out = mim->t_max;
    int side;
    solver3d_t kk;
    s3d_init(&kk, sv_kk_coll);
    {
        ivp_vec3f_t k = clp_point(K, c_K), kn = clp_point(ivp_edge_get_next(K), c_K);
        ivp_vec3f_t l = clp_point(L, c_L), ln = clp_point(ivp_edge_get_next(L), c_L);
        lvp_subtract2_ff(&kk.v[1], &kn, &k);
        (void)lvp_fast_normize(&kk.v[1]);
        lvp_subtract2_ff(&kk.v[3], &ln, &l);
        (void)lvp_fast_normize(&kk.v[3]);
        lvp_set_f(&kk.v[0], &k);
        lvp_set_f(&kk.v[2], &l);
        s3d_set_max_deviation(&kk, mim->max_coll_speed);
        {
            ivp_vec3r_t k_vec_world_now, l_vec_world_now, H;
            lvm_vmult3(&c_L->clp_cache_object->m_world_f_object, &kk.v[3], &l_vec_world_now);
            lvm_vmult3(&c_K->clp_cache_object->m_world_f_object, &kk.v[1], &k_vec_world_now);
            lvp_cross(&H, &k_vec_world_now, &l_vec_world_now);
            D val = -lvp_dot_f(&H, &md->contact_plane);
            /* ivp_mindist_event.cxx:480 "#if 0 IVP_IF(1) { side check assert }": not compiled by the reference */
            if (val > 0.0f) { side = 0; kk.scalar = -1.0f; }
            else { side = 1; kk.scalar = 1.0f; }
        }
        if (s3d_find_first_t_for_value_coll(&kk, ivp_mindist_get_coll_dist(md),
                                            (D)ivp_env_get_mindist_settings(md->env)->real_coll_dist,
                                            mim->t_now, mim->event_time_out, &cache_K, &cache_L, NULL,
                                            &mim->event_time_out)) {
            mim->event_type_out = IVP_COLL_KK_COLL;
        }
    }
    {
        solver3d_t par;
        s3d_init(&par, sv_kk_parallel);
        par.v[1] = kk.v[1];
        par.v[3] = kk.v[3];
        D dev = sum_abs_omega + sum_abs_omega;
        s3d_set_max_deviation(&par, dev + P_DOUBLE_EPS);
        if (s3d_find_first_t_for_value_max_dev(&par, P_DOUBLE_EPS, mim->t_now, mim->event_time_out, 0, &cache_K,
                                               &cache_L, NULL, &mim->event_time_out)) {
            mim->event_type_out = IVP_COLL_KK_PARALLEL;
        }
    }
    const ivp_compact_edge_t *F[4] = { ivp_edge_get_opposite(K), K, ivp_edge_get_opposite(L), L };
    ivp_vec3r_t *edge_object[4] = { &kk.v[3], &kk.v[3], &kk.v[1], &kk.v[1] };
    int reversed[4] = { side, 1 - side, side, 1 - side };
    matrix_cache_t *cache_tab[4] = { &cache_L, &cache_L, &cache_K, &cache_K };
    ivp_cache_ledge_point_t *m_cache_tab[4] = { c_L, c_L, c_K, c_K };
    solver3d_t kkpf;
    s3d_init(&kkpf, sv_vec_parallel_area);
    s3d_set_max_deviation(&kkpf, sum_abs_omega + P_DOUBLE_EPS);
    for (int i = 0; i <= 3; i++) {
        if (!reversed[i]) lvp_set_multiple(&kkpf.v[0], edge_object[i], -1.0f);
        else kkpf.v[0] = *edge_object[i];
        hesse_vec_not_normized(F[i], m_cache_tab[3 - i], &kkpf.v[1]);
        (void)lvp_fast_normize(&kkpf.v[1]);
        IVP_ASSERT(m_cache_tab[3 - i]->compact_ledge == ivp_edge_get_compact_ledge(F[i]));   /* ivp_mindist_event.cxx:592 */
        matrix_cache_t *cache_e = cache_tab[i];
        matrix_cache_t *cache_Fc = cache_tab[3 - i];
        D sec_dist = (D)(-ivp_env_get_mindist_settings(md->env)->mindist_change_force_dist *
                         cache_e->core->inv_object_diameter);
        if (s3d_find_first_t_for_value_max_dev(&kkpf, sec_dist, mim->t_now, mim->event_time_out, 0, cache_e, cache_Fc,
                                               NULL, &mim->event_time_out)) {
            mim->event_type_out = IVP_COLL_KK_PF;
        }
    }
}

/* ── PP / BP / BB ───────────────────────────────────────────────────────── */

static void calc_next_event_PP(ivp_mindist_event_solver_t *mim, const ivp_compact_edge_t *P, const ivp_compact_edge_t *P2,
                               ivp_cache_ledge_point_t *c_P, ivp_cache_ledge_point_t *c_P2)
{
    ivp_mindist_t *md = mim->mindist;
    matrix_cache_t cache_P, cache_P2;
    mc_init(&cache_P, c_P->clp_cache_object);
    mc_init(&cache_P2, c_P2->clp_cache_object);
    struct ivp_core *solver_P = c_P->clp_object->physical_core;
    struct ivp_core *solver_P2 = c_P2->clp_object->physical_core;
    mim->event_time_out = mim->t_max;
    ivp_vec3f_t points_object[2];
    points_object[0] = clp_point(P, c_P);
    points_object[1] = clp_point(P2, c_P2);
    {
        solver3d_t s;
        s3d_init(&s, sv_distance_of_two_points);
        lvp_set_f(&s.v[0], &points_object[0]);
        lvp_set_f(&s.v[1], &points_object[1]);
        lvp_set_multiple_f(&s.v[2], &md->contact_plane, -1);
        /* ivp_mindist_event.cxx:646 / 772 "#if 0 IVP_IF(1) { direction check assert }": not compiled by the reference */
        s3d_set_max_deviation(&s, mim->max_coll_speed);
        float radius = md->sum_extra_radius;
        if (s3d_find_first_t_for_value_coll(&s, ivp_mindist_get_coll_dist(md) + (D)radius,
                                            (D)(ivp_env_get_mindist_settings(md->env)->real_coll_dist + radius * 0.5f), mim->t_now,
                                            mim->t_max, &cache_P, &cache_P2, NULL, &mim->event_time_out)) {
            mim->event_type_out = IVP_COLL_PP_COLL;
        }
    }
    {
        D deviation[2];
        D neg_maximal_safe_s_value;
        {
            D max_dist_PP = (D)ivp_mindist_get_length(md);
            max_dist_PP += mim->worst_case_speed * (D)ivp_time_diff(mim->event_time_out, mim->t_now);
            D speed_P = lvf_fast_real_length(&points_object[0]) * (D)solver_P->abs_omega + (D)solver_P->current_speed;
            D speed_P2 = lvf_fast_real_length(&points_object[1]) * (D)solver_P2->abs_omega + (D)solver_P2->current_speed;
            deviation[0] = speed_P + speed_P2 + max_dist_PP * (D)solver_P2->abs_omega;
            deviation[1] = speed_P + speed_P2 + max_dist_PP * (D)solver_P->abs_omega;
            D object_radius = P_MAXD((D)solver_P->upper_limit_radius, (D)solver_P2->upper_limit_radius);
            D quad_coll_dist = ivp_mindist_get_coll_dist(md);
            quad_coll_dist *= quad_coll_dist;
            neg_maximal_safe_s_value = -P_MIND(max_dist_PP * max_dist_PP, quad_coll_dist) * 0.5f / object_radius;
        }
        matrix_cache_t *cache[2] = { &cache_P, &cache_P2 };
        ivp_cache_ledge_point_t *mcache[2] = { c_P, c_P2 };
        const ivp_compact_edge_t *Point[2] = { P, P2 };
        for (int j = 0; j < 2; j++) {
            const ivp_compact_edge_t *point2 = Point[1 - j];
            const ivp_vec3f_t *p0 = &points_object[j];
            const ivp_vec3f_t *p1 = &points_object[1 - j];
            solver3d_t s;
            s3d_init(&s, sv_s_vals);
            lvp_set_f(&s.v[0], p0);
            lvp_set_f(&s.v[1], p1);
            s3d_set_max_deviation(&s, deviation[j]);
            for (const ivp_compact_edge_t *e = ivp_edge_get_opposite(ivp_edge_get_prev(point2));;
                 e = ivp_edge_get_opposite(ivp_edge_get_prev(e))) {
                ivp_vec3f_t epn = clp_point(ivp_edge_get_next(e), mcache[1 - j]);
                lvp_subtract2_ff(&s.v[2], &epn, p1);
                D qlen = lvp_quad_length(&s.v[2]);
                D inv_len = (D)lv_isqrt_float((float)qlen);
                lvp_mult(&s.v[2], inv_len);
                D sec_dist = neg_maximal_safe_s_value * qlen * inv_len;
                if (s3d_find_first_t_for_value_max_dev(&s, sec_dist, mim->t_now, mim->event_time_out, 0, cache[j],
                                                       cache[1 - j], NULL, &mim->event_time_out)) {
                    mim->event_type_out = IVP_COLL_PP_PK;
                }
                if (e == point2) break;
            }
        }
    }
}

static void calc_next_event_BP(ivp_mindist_event_solver_t *mim, struct ivp_object *ball, const ivp_compact_edge_t *P2,
                               ivp_cache_obj_t *c_B, ivp_cache_ledge_point_t *c_P2)
{
    ivp_mindist_t *md = mim->mindist;
    matrix_cache_t cache_P, cache_P2;
    mc_init(&cache_P, c_B);
    mc_init(&cache_P2, c_P2->clp_cache_object);
    struct ivp_core *solver_P = ball->physical_core;
    struct ivp_core *solver_P2 = c_P2->clp_object->physical_core;
    mim->event_time_out = mim->t_max;
    ivp_vec3f_t p2_object = clp_point(P2, c_P2);
    {
        solver3d_t s;
        s3d_init(&s, sv_distance_of_two_points);
        lvp_zero(&s.v[0]);
        lvp_set_f(&s.v[1], &p2_object);
        lvp_set_multiple_f(&s.v[2], &md->contact_plane, -1);
        /* ivp_mindist_event.cxx:646 / 772 "#if 0 IVP_IF(1) { direction check assert }": not compiled by the reference */
        s3d_set_max_deviation(&s, mim->max_coll_speed);
        float radius = md->sum_extra_radius;
        if (s3d_find_first_t_for_value_coll(&s, ivp_mindist_get_coll_dist(md) + (D)radius,
                                            (D)(ivp_env_get_mindist_settings(md->env)->real_coll_dist + radius * 0.5f), mim->t_now,
                                            mim->t_max, &cache_P, &cache_P2, NULL, &mim->event_time_out)) {
            mim->event_type_out = IVP_COLL_PP_COLL;
        }
    }
    {
        D deviation;
        D neg_maximal_safe_s_value;
        {
            D max_dist_PP = (D)ivp_mindist_get_length(md);
            max_dist_PP += mim->worst_case_speed * (D)ivp_time_diff(mim->event_time_out, mim->t_now);
            D speed_P = (D)solver_P->current_speed;
            D speed_P2 = lvf_fast_real_length(&p2_object) * (D)solver_P2->abs_omega + (D)solver_P2->current_speed;
            deviation = speed_P + speed_P2 + max_dist_PP * (D)solver_P2->abs_omega;
            D inv_object_diameter = (D)solver_P2->inv_object_diameter;
            D quad_coll_dist = ivp_mindist_get_coll_dist(md);
            quad_coll_dist *= quad_coll_dist;
            neg_maximal_safe_s_value = -P_MIND(max_dist_PP * max_dist_PP, quad_coll_dist) * inv_object_diameter;
        }
        solver3d_t s;
        s3d_init(&s, sv_s_vals);
        lvp_zero(&s.v[0]);
        lvp_set_f(&s.v[1], &p2_object);
        s3d_set_max_deviation(&s, deviation);
        for (const ivp_compact_edge_t *e = ivp_edge_get_opposite(ivp_edge_get_prev(P2));;
             e = ivp_edge_get_opposite(ivp_edge_get_prev(e))) {
            ivp_vec3f_t epn = clp_point(ivp_edge_get_next(e), c_P2);
            lvp_subtract2_ff(&s.v[2], &epn, &p2_object);
            D qlen = lvp_quad_length(&s.v[2]);
            D inv_len = (D)lv_isqrt_float((float)qlen);
            lvp_mult(&s.v[2], inv_len);
            D sec_dist = neg_maximal_safe_s_value * qlen * inv_len;
            if (s3d_find_first_t_for_value_max_dev(&s, sec_dist, mim->t_now, mim->event_time_out, 0, &cache_P,
                                                   &cache_P2, NULL, &mim->event_time_out)) {
                mim->event_type_out = IVP_COLL_PP_PK;
            }
            if (e == P2) break;
        }
    }
}

static void calc_next_event_BB(ivp_mindist_event_solver_t *mim, ivp_cache_obj_t *c_A, ivp_cache_obj_t *c_B)
{
    ivp_mindist_t *md = mim->mindist;
    matrix_cache_t cache_A, cache_B;
    mc_init(&cache_A, c_A);
    mc_init(&cache_B, c_B);
    mim->event_time_out = mim->t_max;
    solver3d_t s;
    s3d_init(&s, sv_distance_of_two_points);
    lvp_subtract2(&s.v[2], &c_B->m_world_f_object.pos, &c_A->m_world_f_object.pos);
    (void)lvp_normize(&s.v[2]);
    s3d_set_max_deviation(&s, mim->max_coll_speed);
    float radius = md->sum_extra_radius;
    if (s3d_find_first_t_for_value_coll(&s, ivp_mindist_get_coll_dist(md) + (D)radius,
                                        (D)(ivp_env_get_mindist_settings(md->env)->real_coll_dist + radius * 0.5f), mim->t_now,
                                        mim->t_max, &cache_A, &cache_B, NULL, &mim->event_time_out)) {
        mim->event_type_out = IVP_COLL_PP_COLL;
    }
}

/* ── PK / BK ────────────────────────────────────────────────────────────── */

static void pk_lot_setup(solver3d_t *s, const ivp_vec3r_t *P_world, ivp_cache_ledge_point_t *c_K)
{
    ivp_vec3r_t K_world, K_vec_world, PK, H;
    lvm_vmult4_fp(&c_K->clp_cache_object->m_world_f_object, &s->f[1], &K_world);
    lvm_vmult3(&c_K->clp_cache_object->m_world_f_object, &s->v[0], &K_vec_world);
    lvp_subtract2(&PK, &K_world, P_world);
    lvp_cross(&H, &PK, &K_vec_world);
    lvm_vimult3(&c_K->clp_cache_object->m_world_f_object, &H, &s->v[1]);
    (void)lvp_normize(&s->v[1]);
}

static void pk_pf_sector(ivp_mindist_event_solver_t *mim, const ivp_vec3r_t *p_object, const ivp_compact_edge_t *K,
                         ivp_cache_ledge_point_t *c_K, matrix_cache_t *cache_P, matrix_cache_t *cache_K)
{
    solver3d_t pf;
    s3d_init(&pf, sv_pf_coll);
    s3d_set_max_deviation(&pf, mim->worst_case_speed);
    pf.v[0] = *p_object;
    const ivp_compact_edge_t *FF[2] = { K, ivp_edge_get_opposite(K) };
    for (int i = 0; i <= 1; i++) {
        const ivp_compact_edge_t *F = FF[i];
        ivp_vec3f_t f_object = clp_point(F, c_K);
        ivp_vec3f_t fn_object = clp_point(ivp_edge_get_next(F), c_K);
        ivp_vec3f_t fp_object = clp_point(ivp_edge_get_prev(F), c_K);
        ivp_vec3r_t v0, fhesse_vec;
        lvp_subtract2_ff(&v0, &fn_object, &f_object);
        lvp_set_vert_to_area_fff(&fhesse_vec, &f_object, &fp_object, &fn_object);
        lvp_cross(&pf.v[1], &v0, &fhesse_vec);
        (void)lvp_normize(&pf.v[1]);
        lvp_set_f(&pf.v[2], &f_object);
        D sec_dist = -(D)ivp_env_get_mindist_settings(mim->mindist->env)->mindist_change_force_dist;
        if (s3d_find_first_t_for_value_max_dev(&pf, sec_dist, mim->t_now, mim->event_time_out, 0, cache_P, cache_K,
                                               NULL, &mim->event_time_out)) {
            mim->event_type_out = IVP_COLL_PK_PF;
        }
    }
}

static void calc_next_event_PK(ivp_mindist_event_solver_t *mim, const ivp_compact_edge_t *P, const ivp_compact_edge_t *K,
                               ivp_cache_ledge_point_t *c_P, ivp_cache_ledge_point_t *c_K)
{
    ivp_mindist_t *md = mim->mindist;
    matrix_cache_t cache_P, cache_K;
    mc_init(&cache_P, c_P->clp_cache_object);
    mc_init(&cache_K, c_K->clp_cache_object);
    struct ivp_core *solver_P = c_P->clp_object->physical_core;
    struct ivp_core *solver_K = c_K->clp_object->physical_core;
    D sum_abs_omega = (D)(solver_P->abs_omega + solver_K->abs_omega); /* IVP_FLOAT sum */
    mim->event_time_out = mim->t_max;
    ivp_vec3f_t p_object = clp_point(P, c_P);
    {
        solver3d_t s;
        s3d_init(&s, sv_pk_coll);
        ivp_vec3f_t k_object = clp_point(K, c_K), kn_object = clp_point(ivp_edge_get_next(K), c_K);
        s.f[0] = p_object;
        s.f[1] = k_object;
        lvp_subtract2_ff(&s.v[0], &kn_object, &k_object);
        (void)lvp_fast_normize(&s.v[0]);
        s3d_set_max_deviation(&s, mim->max_coll_speed);
        float radius = md->sum_extra_radius;
        s.scalar = 0.5f * (ivp_mindist_get_coll_dist(md) + (D)radius);
        ivp_vec3r_t P_world;
        lvm_vmult4_fp(&c_P->clp_cache_object->m_world_f_object, &s.f[0], &P_world);
        pk_lot_setup(&s, &P_world, c_K);
        if (s3d_find_first_t_for_value_coll(&s, ivp_mindist_get_coll_dist(md) + (D)radius,
                                            (D)(ivp_env_get_mindist_settings(md->env)->real_coll_dist + radius * 0.9f), mim->t_now,
                                            mim->t_max, &cache_P, &cache_K, NULL, &mim->event_time_out)) {
            mim->event_type_out = IVP_COLL_PK_COLL;
        }
    }
    {
        ivp_vec3r_t pp; lvp_set_f(&pp, &p_object);
        pk_pf_sector(mim, &pp, K, c_K, &cache_P, &cache_K);
    }
    D dist_PK_now = (D)ivp_mindist_get_length(md);
    D max_dev;
    {
        D mindist_PK = dist_PK_now - mim->worst_case_speed * (D)ivp_time_diff(mim->event_time_out, mim->t_now);
        if (mindist_PK < P_RES_EPS) mindist_PK = P_RES_EPS;
        D speed_P = lvf_fast_real_length(&p_object) * (D)solver_P->abs_omega + (D)solver_P->current_speed;
        /* IVP_FLOAT expression */
        D speed_K = (D)(solver_K->max_surface_rot_speed * solver_K->abs_omega + solver_K->current_speed);
        max_dev = (speed_P + speed_K) / mindist_PK + sum_abs_omega;
    }
    {
        solver3d_t s;
        s3d_init(&s, sv_pk_kk);
        ivp_vec3f_t k_object = clp_point(K, c_K), kn_object = clp_point(ivp_edge_get_next(K), c_K);
        lvp_set_f(&s.v[0], &p_object);
        lvp_set_f(&s.v[2], &k_object);
        lvp_subtract2_ff(&s.v[3], &kn_object, &k_object);
        (void)lvp_fast_normize(&s.v[3]);
        s3d_set_max_deviation(&s, max_dev);
        D coll_dist = ivp_mindist_get_coll_dist(md);
        D dist = dist_PK_now < coll_dist ? dist_PK_now : coll_dist;
        D sec_dist = -0.3f * dist * ((D)solver_K->inv_object_diameter * 2.0f);
        for (const ivp_compact_edge_t *e = ivp_edge_get_opposite(ivp_edge_get_prev(P));;
             e = ivp_edge_get_opposite(ivp_edge_get_prev(e))) {
            ivp_vec3f_t en_object = clp_point(ivp_edge_get_next(e), c_P);
            lvp_subtract2_ff(&s.v[1], &en_object, &p_object);
            (void)lvp_fast_normize(&s.v[1]);
            if (s3d_find_first_t_for_value_max_dev(&s, sec_dist, mim->t_now, mim->event_time_out, 0, &cache_P,
                                                   &cache_K, NULL, &mim->event_time_out)) {
                mim->event_type_out = IVP_COLL_PK_KK;
            }
            if (e == P) break;
        }
    }
}

static void calc_next_event_BK(ivp_mindist_event_solver_t *mim, struct ivp_object *ball, const ivp_compact_edge_t *K,
                               ivp_cache_obj_t *c_B, ivp_cache_ledge_point_t *c_K)
{
    ivp_mindist_t *md = mim->mindist;
    (void)ball;
    matrix_cache_t cache_P, cache_K;
    mc_init(&cache_P, c_B);
    mc_init(&cache_K, c_K->clp_cache_object);
    mim->event_time_out = mim->t_max;
    {
        solver3d_t s;
        s3d_init(&s, sv_pk_coll);
        ivp_vec3f_t k_object = clp_point(K, c_K), kn_object = clp_point(ivp_edge_get_next(K), c_K);
        lvf_zero(&s.f[0]);
        s.f[1] = k_object;
        lvp_subtract2_ff(&s.v[0], &kn_object, &k_object);
        (void)lvp_fast_normize(&s.v[0]);
        s3d_set_max_deviation(&s, mim->max_coll_speed);
        float radius = md->sum_extra_radius;
        s.scalar = 0.5f * (ivp_mindist_get_coll_dist(md) + (D)radius);
        pk_lot_setup(&s, &c_B->m_world_f_object.pos, c_K);
        if (s3d_find_first_t_for_value_coll(&s, ivp_mindist_get_coll_dist(md) + (D)radius,
                                            (D)(ivp_env_get_mindist_settings(md->env)->real_coll_dist + radius * 0.9f), mim->t_now,
                                            mim->t_max, &cache_P, &cache_K, NULL, &mim->event_time_out)) {
            mim->event_type_out = IVP_COLL_PK_COLL;
        }
    }
    {
        IVP_ASSERT(mim->worst_case_speed > mim->max_coll_speed);   /* ivp_mindist_event.cxx:1143 */
        ivp_vec3r_t pp; lvp_zero(&pp);
        pk_pf_sector(mim, &pp, K, c_K, &cache_P, &cache_K);
    }
}

/* ── dispatch ───────────────────────────────────────────────────────────── */

static void next_event_B_POLY(ivp_mindist_event_solver_t *mim)
{
    ivp_md_synapse_t *syn0 = ivp_mindist_get_synapse(mim->mindist, 0);
    ivp_md_synapse_t *syn1 = ivp_mindist_get_synapse(mim->mindist, 1);
    const ivp_compact_edge_t *e1 = syn1->edge;
    ivp_cache_obj_t *c0 = ivp_object_get_cache_object(syn0->l_obj);
    ivp_cache_ledge_point_t c1;
    ivp_cache_ledge_point_init(&c1, syn1->l_obj, ivp_edge_get_compact_ledge(e1));
    mim->event_type_out = IVP_COLL_NONE;
#ifdef IVP_DEBUG_BUILD
    c1.tmp.synapse = NULL;   /* ivp_mindist_event.cxx:1193 */
#endif
    switch (ivp_md_synapse_get_status(syn1)) {
    case IVP_ST_POINT:    calc_next_event_BP(mim, syn0->l_obj, e1, c0, &c1); break;
    case IVP_ST_EDGE:     calc_next_event_BK(mim, syn0->l_obj, e1, c0, &c1); break;
    case IVP_ST_TRIANGLE: calc_next_event_BF(mim, e1, c0, &c1); break;
    default:
        IVP_CORE();   /* ivp_mindist_event.cxx:1209 */
        break;
    }
    ivp_cache_obj_remove_reference(c0);
    ivp_cache_ledge_point_remove_reference(&c1);
}

static void next_event_BB(ivp_mindist_event_solver_t *mim)
{
    ivp_cache_obj_t *c0 = ivp_object_get_cache_object(ivp_mindist_get_synapse(mim->mindist, 0)->l_obj);
    ivp_cache_obj_t *c1 = ivp_object_get_cache_object(ivp_mindist_get_synapse(mim->mindist, 1)->l_obj);
    mim->event_type_out = IVP_COLL_NONE;
    calc_next_event_BB(mim, c0, c1);
    ivp_cache_obj_remove_reference(c0);
    ivp_cache_obj_remove_reference(c1);
}

static void next_event_default_poly_poly(ivp_mindist_event_solver_t *mim)
{
    ivp_md_synapse_t *syn0 = ivp_mindist_get_sorted_synapse(mim->mindist, 0);
    ivp_md_synapse_t *syn1 = ivp_mindist_get_sorted_synapse(mim->mindist, 1);
    const ivp_compact_edge_t *e0 = syn0->edge;
    const ivp_compact_edge_t *e1 = syn1->edge;
    ivp_cache_ledge_point_t c0, c1;
    ivp_cache_ledge_point_init(&c0, syn0->l_obj, ivp_edge_get_compact_ledge(e0));
    ivp_cache_ledge_point_init(&c1, syn1->l_obj, ivp_edge_get_compact_ledge(e1));
#ifdef IVP_DEBUG_BUILD
    c0.tmp.synapse = NULL;   /* ivp_mindist_event.cxx:1248: remember order of synapses */
    c1.tmp.synapse = NULL;
#endif
    mim->event_type_out = IVP_COLL_NONE;
    switch (ivp_md_synapse_get_status(syn0)) {
    case IVP_ST_POINT:
        switch (ivp_md_synapse_get_status(syn1)) {
        case IVP_ST_POINT:    calc_next_event_PP(mim, e0, e1, &c0, &c1); break;
        case IVP_ST_EDGE:     calc_next_event_PK(mim, e0, e1, &c0, &c1); break;
        case IVP_ST_TRIANGLE: calc_next_event_PF(mim, e0, e1, &c0, &c1); break;
        default:
            IVP_CORE();   /* ivp_mindist_event.cxx:1277 */
            break;
        }
        break;
    case IVP_ST_EDGE:
        switch (ivp_md_synapse_get_status(syn1)) {
        case IVP_ST_EDGE: calc_next_event_KK(mim, e0, e1, &c0, &c1); break;
        default:
            IVP_CORE();   /* ivp_mindist_event.cxx:1291 */
            break;
        }
        break;
    default:
        IVP_CORE();   /* ivp_mindist_event.cxx:1296: unknown synapse status */
        break;
    }
    ivp_cache_ledge_point_remove_reference(&c0);
    ivp_cache_ledge_point_remove_reference(&c1);
}

/* error: inconsistency between the event solver and the minimize solver */
static void next_event_illegal(ivp_mindist_event_solver_t *mim)
{
    mim->event_type_out = IVP_COLL_NONE;
    IVP_CORE();   /* ivp_mindist_event.cxx:1306 */
}

typedef void (*mim_fn_t)(ivp_mindist_event_solver_t *);

static const mim_fn_t mim_function_table[IVP_ST_MAX_LEGAL][IVP_ST_MAX_LEGAL] = {
    /* POINT    */ { next_event_default_poly_poly, next_event_default_poly_poly, next_event_default_poly_poly, next_event_illegal },
    /* EDGE     */ { next_event_illegal, next_event_default_poly_poly, next_event_illegal, next_event_illegal },
    /* TRIANGLE */ { next_event_illegal, next_event_illegal, next_event_illegal, next_event_illegal },
    /* BALL     */ { next_event_B_POLY, next_event_B_POLY, next_event_B_POLY, next_event_BB },
};

void ivp_mindist_event_solver_calc_time_of_next_event(ivp_mindist_event_solver_t *mim)
{
    int s0 = (int)ivp_md_synapse_get_status(ivp_mindist_get_sorted_synapse(mim->mindist, 0));
    int s1 = (int)ivp_md_synapse_get_status(ivp_mindist_get_sorted_synapse(mim->mindist, 1));
    IVP_ASSERT(s0 >= IVP_ST_POINT && s0 < IVP_ST_MAX_LEGAL);   /* ivp_mindist_event.cxx:1345 */
    IVP_ASSERT(s1 >= IVP_ST_POINT && s1 < IVP_ST_MAX_LEGAL);
    if (s0 < 0 || s0 >= IVP_ST_MAX_LEGAL || s1 < 0 || s1 >= IVP_ST_MAX_LEGAL) {
        mim->event_type_out = IVP_COLL_NONE;
        return;
    }
    mim_function_table[s0][s1](mim);
}
