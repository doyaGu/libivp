/* ivp_scr_internal.h -- float-exact IVP_Solver_Core_Reaction (internal)
 *
 * Port of ivp_physics/ivp_solver_core_reaction.{hxx,cxx} with the reference
 * types and evaluation order:
 *   - direction_ws, delta_velocity_ds and the impulses are IVP_U_Float_Point;
 *   - cross_direction_position_cs* / cr_mult_inv* are IVP_U_Point_4
 *     (= IVP_U_Float_Hesse: three floats + a float hesse_val);
 *   - m_velocity_ds_f_impulse_ds is an IVP_U_Matrix3 (double) accumulated
 *     from float dot products (IVP_U_Point_4::dot_product4 and
 *     IVP_U_Float_Point::dot_product are float expressions);
 *   - the exert_* functions push the float core speeds with
 *     IVP_U_Float_Point::add_multiple (double product, float store); the
 *     dim1 linear push multiplies IVP_FLOAT impulse * IVP_FLOAT inv_mass in
 *     float, dim2/dim3 go through an IVP_DOUBLE inv_m;
 *   - the lever arm is measured from core->get_position_PSI()
 *     (m_world_f_core_last_psi's position).
 *
 * Shared by the public ivp_scr_* API (ivp_solver_core_reaction.c), the
 * friction solver and the controller ports (stiff spring, fixed keyed
 * constraint, floating, raycast car).
 */
#ifndef IVP_SCR_INTERNAL_H
#define IVP_SCR_INTERNAL_H

#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <ivp/dynamics/ivp_core.h>
#include "collision/ivp_lvec.h"
#include "dynamics/ivp_sim_internal.h"

/* IVP_U_Point_4 (IVP_U_Float_Hesse) */
typedef struct ivp_scr_p4 {
    float k[3];
    float hesse_val;
} ivp_scr_p4_t;

typedef struct ivp_scr_f {
    const ivp_vec3f_t *direction_ws[3];
    ivp_scr_p4_t       cross_direction_position_cs0[3];
    ivp_scr_p4_t       cross_direction_position_cs1[3];
    ivp_scr_p4_t       cr_mult_inv0[3];
    ivp_scr_p4_t       cr_mult_inv1[3];
    double             m[3][3];             /* m_velocity_ds_f_impulse_ds (IVP_U_Matrix3) */
    ivp_vec3f_t        delta_velocity_ds;   /* based on current rot_speeds and speeds */
} ivp_scr_f_t;

/* core->get_inv_masses(): inv_rot_inertia with hesse_val = inv_mass */
static inline ivp_scr_p4_t ivp_scr_f_inv_masses(const ivp_core_t *core)
{
    ivp_scr_p4_t im;
#if IVP_ARM64_NEON && defined(IVP_REAL_DOUBLE)
    vst1_f32(im.k, vcvt_f32_f64(vld1q_f64(core->inv_rot_inertia.k)));
    im.k[2] = (float)core->inv_rot_inertia.k[2];
#else
    im.k[0] = (float)core->inv_rot_inertia.k[0];
    im.k[1] = (float)core->inv_rot_inertia.k[1];
    im.k[2] = (float)core->inv_rot_inertia.k[2];
#endif
    im.hesse_val = (float)core->inv_mass;
    return im;
}

/* IVP_U_Point_4::set_line_wise_mult4 (float products) */
static inline void ivp_scr_f_line_wise_mult4(ivp_scr_p4_t *out, const ivp_scr_p4_t *a, const ivp_scr_p4_t *b)
{
#if IVP_ARM64_NEON
    vst1q_f32(out->k, vmulq_f32(vld1q_f32(a->k), vld1q_f32(b->k)));
#else
    double x = (double)(a->k[0] * b->k[0]);
    double y = (double)(a->k[1] * b->k[1]);
    double z = (double)(a->k[2] * b->k[2]);
    double w = (double)(a->hesse_val * b->hesse_val);
    out->k[0] = (float)x;
    out->k[1] = (float)y;
    out->k[2] = (float)z;
    out->hesse_val = (float)w;
#endif
}

/* IVP_U_Point_4::dot_product4 (float expression) */
static inline double ivp_scr_f_dot4(const ivp_scr_p4_t *a, const ivp_scr_p4_t *o)
{
#if IVP_ARM64_NEON
    float32x2_t products = vmul_f32(vld1_f32(a->k), vld1_f32(o->k));
    float sum = vget_lane_f32(products, 0) + vget_lane_f32(products, 1);
    sum += a->k[2] * o->k[2];
    sum += a->hesse_val * o->hesse_val;
    return (double)sum;
#else
    return (double)(a->k[0] * o->k[0] + a->k[1] * o->k[1] + a->k[2] * o->k[2] + a->hesse_val * o->hesse_val);
#endif
}

/* IVP_U_Float_Point::dot_product on the xyz part (float expression) */
static inline double ivp_scr_f_dot3(const ivp_scr_p4_t *a, const ivp_scr_p4_t *o)
{
#if IVP_ARM64_NEON
    float32x2_t products = vmul_f32(vld1_f32(a->k), vld1_f32(o->k));
    float sum = vget_lane_f32(products, 0) + vget_lane_f32(products, 1);
    sum += a->k[2] * o->k[2];
    return (double)sum;
#else
    return (double)(a->k[0] * o->k[0] + a->k[1] * o->k[1] + a->k[2] * o->k[2]);
#endif
}

static inline double ivp_scr_f_dot_vp(const ivp_vec3f_t *a, const ivp_scr_p4_t *o)
{
#if IVP_ARM64_NEON
    float32x2_t products = vmul_f32(vld1_f32(a->k), vld1_f32(o->k));
    float sum = vget_lane_f32(products, 0) + vget_lane_f32(products, 1);
    sum += a->k[2] * o->k[2];
    return (double)sum;
#else
    return (double)(a->k[0] * o->k[0] + a->k[1] * o->k[1] + a->k[2] * o->k[2]);
#endif
}

static inline ivp_vec3f_t ivp_scr_f_xyz(const ivp_scr_p4_t *p)
{
    ivp_vec3f_t r;
    r.k[0] = p->k[0];
    r.k[1] = p->k[1];
    r.k[2] = p->k[2];
    return r;
}

/* IVP_Solver_Core_Reaction::init_trans_ws */
static inline void ivp_scr_f_init_trans_ws(ivp_scr_f_t *s, const ivp_vec3r_t *pos_ws, const ivp_core_t *core,
                                           ivp_scr_p4_t cdp[3], ivp_scr_p4_t cmi[3], float sign)
{
    const ivp_transform_r_t *m_world_f_core = &core->m_world_f_core_last_psi;
    ivp_vec3f_t pos_rel_core;
    lvf_subtract2_p(&pos_rel_core, pos_ws, &m_world_f_core->pos);
    const ivp_scr_p4_t im = ivp_scr_f_inv_masses(core);
    const ivp_vec3f_t speed = ivp_core_speed_f(core);

    for (int i = 0; i < 3; i++) {
        if (i > 0 && !s->direction_ws[i]) return;
        ivp_vec3f_t cross_ws, cs;
        lvf_cross(&cross_ws, &pos_rel_core, s->direction_ws[i]);
        lvm_vimult3_f(m_world_f_core, &cross_ws, &cs);
        cdp[i].k[0] = cs.k[0];
        cdp[i].k[1] = cs.k[1];
        cdp[i].k[2] = cs.k[2];
        cdp[i].hesse_val = 1.0f;
        ivp_scr_f_line_wise_mult4(&cmi[i], &cdp[i], &im);

        s->m[i][i] = s->m[i][i] + ivp_scr_f_dot4(&cmi[i], &cdp[i]);
        for (int j = 0; j < i; j++) {
            s->m[j][i] = s->m[j][i] + ivp_scr_f_dot3(&cmi[i], &cdp[j]);
        }
        double a = ivp_scr_f_dot_vp(&core->rot_speed, &cdp[i]) + lvf_dot(&speed, s->direction_ws[i]);
        /* IVP_FLOAT += IVP_DOUBLE */
        s->delta_velocity_ds.k[i] = (float)((double)s->delta_velocity_ds.k[i] + a * (double)sign);
    }
}

/* IVP_Solver_Core_Reaction::init_rot_ws */
static inline void ivp_scr_f_init_rot_ws(ivp_scr_f_t *s, const ivp_core_t *core, ivp_scr_p4_t cdp[3],
                                         ivp_scr_p4_t cmi[3], float sign)
{
    const ivp_transform_r_t *m_world_f_core = &core->m_world_f_core_last_psi;
    const ivp_scr_p4_t im = ivp_scr_f_inv_masses(core);
    for (int i = 0; i < 3; i++) {
        if (i > 0 && !s->direction_ws[i]) continue;
        ivp_vec3f_t cs;
        lvm_vimult3_f(m_world_f_core, s->direction_ws[i], &cs);
        cdp[i].k[0] = cs.k[0];
        cdp[i].k[1] = cs.k[1];
        cdp[i].k[2] = cs.k[2];
        cdp[i].hesse_val = 0.0f;
        ivp_scr_f_line_wise_mult4(&cmi[i], &cdp[i], &im);

        s->m[i][i] = s->m[i][i] + ivp_scr_f_dot3(&cmi[i], &cdp[i]);
        for (int j = 0; j < i; j++) {
            /* the reference reads an unset cdp[j] when direction j is missing */
            if (!s->direction_ws[j]) continue;
            s->m[j][i] = s->m[j][i] + ivp_scr_f_dot3(&cmi[i], &cdp[j]);
        }
        double a = ivp_scr_f_dot_vp(&core->rot_speed, &cdp[i]);
        s->delta_velocity_ds.k[i] = (float)((double)s->delta_velocity_ds.k[i] + a * (double)sign);
    }
}

/* init_reaction_solver_translation_ws: directions normized, d1 / d2 may be
 * NULL; core NULL = static.  The direction pointers are kept until the
 * exert_* call. */
static inline void ivp_scr_f_init_translation_ws(ivp_scr_f_t *s, const ivp_core_t *core_0, const ivp_core_t *core_1,
                                                 const ivp_vec3r_t *pos_ws, const ivp_vec3f_t *d0,
                                                 const ivp_vec3f_t *d1, const ivp_vec3f_t *d2)
{
    s->direction_ws[0] = d0;
    s->direction_ws[1] = d1;
    s->direction_ws[2] = d2;
    if (d1) {
        memset(s->m, 0, sizeof(s->m));
        lvf_zero(&s->delta_velocity_ds);
    } else {
        s->m[0][0] = 0.0; /* just one element */
        s->delta_velocity_ds.k[0] = 0.0f;
    }
    if (core_0) ivp_scr_f_init_trans_ws(s, pos_ws, core_0, s->cross_direction_position_cs0, s->cr_mult_inv0, 1.0f);
    if (core_1) ivp_scr_f_init_trans_ws(s, pos_ws, core_1, s->cross_direction_position_cs1, s->cr_mult_inv1, -1.0f);
}

/* init_reaction_solver_rotation_ws */
static inline void ivp_scr_f_init_rotation_ws(ivp_scr_f_t *s, const ivp_core_t *core_0, const ivp_core_t *core_1,
                                              const ivp_vec3f_t *d0, const ivp_vec3f_t *d1, const ivp_vec3f_t *d2)
{
    s->direction_ws[0] = d0;
    s->direction_ws[1] = d1;
    s->direction_ws[2] = d2;
    memset(s->m, 0, sizeof(s->m));
    lvf_zero(&s->delta_velocity_ds);
    if (core_0) ivp_scr_f_init_rot_ws(s, core_0, s->cross_direction_position_cs0, s->cr_mult_inv0, 1.0f);
    if (core_1) ivp_scr_f_init_rot_ws(s, core_1, s->cross_direction_position_cs1, s->cr_mult_inv1, -1.0f);
}

/* IVP_U_Matrix3::real_invert (determinant algorithm, fails for |D| < epsilon) */
static inline bool ivp_scr_f_real_invert(double m[3][3], double epsilon)
{
    double r0[3], r1[3], r2[3];
    /* r0 = rows[1] x rows[2], r1 = rows[2] x rows[0], r2 = rows[0] x rows[1] */
    const double *a[3][2] = { { m[1], m[2] }, { m[2], m[0] }, { m[0], m[1] } };
    double *r[3] = { r0, r1, r2 };
    for (int i = 0; i < 3; i++) {
        const double *v1 = a[i][0], *v2 = a[i][1];
        double x = v1[1] * v2[2];
        double x2 = v2[1] * v1[2];
        double y = v1[2] * v2[0];
        double y2 = v2[2] * v1[0];
        x -= x2;
        double z = v1[0] * v2[1];
        double z2 = v2[0] * v1[1];
        y -= y2;
        z -= z2;
        r[i][0] = x;
        r[i][1] = y;
        r[i][2] = z;
    }
    double det = m[0][0] * r0[0] + m[0][1] * r0[1] + m[0][2] * r0[2];
    if (fabs(det) < epsilon) return false;
    double di = 1.0f / det;
    for (int i = 0; i < 3; i++) {
        r0[i] *= di;
        r1[i] *= di;
        r2[i] *= di;
    }
    for (int i = 0; i < 3; i++) {
        m[i][0] = r0[i];
        m[i][1] = r1[i];
        m[i][2] = r2[i];
    }
    return true;
}

/* invert_3x3_matrix: mirror the upper triangle, IVP_U_Matrix3::real_invert */
static inline bool ivp_scr_f_invert_3x3(ivp_scr_f_t *s)
{
    double (*m)[3] = s->m;
    m[1][0] = m[0][1];
    m[2][0] = m[0][2];
    m[2][1] = m[1][2];
    return ivp_scr_f_real_invert(m, P_DOUBLE_EPS);
}

/* exert_impulse_dimN / exert_angular_impulse_dimN */
static inline void ivp_scr_f_exert(const ivp_scr_f_t *s, ivp_core_t *core_0, ivp_core_t *core_1,
                                   const ivp_vec3f_t *impulse_ds, int dims, bool linear)
{
    if (core_0) {
        if (linear) {
            const float inv_mf = ivp_core_get_inv_mass_f(core_0);
            const double inv_m = (double)inv_mf;
            ivp_vec3f_t speed = ivp_core_speed_f(core_0);
            for (int i = 0; i < dims; i++) {
                const double f = (dims == 1) ? (double)(impulse_ds->k[i] * inv_mf) : (double)impulse_ds->k[i] * inv_m;
                lvf_add_multiple(&speed, s->direction_ws[i], (D)f);
            }
            ivp_core_set_speed_f(core_0, &speed);
        }
        for (int i = 0; i < dims; i++) {
            ivp_vec3f_t cmi = ivp_scr_f_xyz(&s->cr_mult_inv0[i]);
            lvf_add_multiple(&core_0->rot_speed, &cmi, (D)impulse_ds->k[i]);
        }
    }
    if (core_1) {
        if (linear) {
            const float inv_mf = ivp_core_get_inv_mass_f(core_1);
            const double inv_m = (double)inv_mf;
            ivp_vec3f_t speed = ivp_core_speed_f(core_1);
            for (int i = 0; i < dims; i++) {
                const double f = (dims == 1) ? (double)(-impulse_ds->k[i] * inv_mf) : (double)-impulse_ds->k[i] * inv_m;
                lvf_add_multiple(&speed, s->direction_ws[i], (D)f);
            }
            ivp_core_set_speed_f(core_1, &speed);
        }
        for (int i = 0; i < dims; i++) {
            ivp_vec3f_t cmi = ivp_scr_f_xyz(&s->cr_mult_inv1[i]);
            lvf_add_multiple(&core_1->rot_speed, &cmi, (D)-impulse_ds->k[i]);
        }
    }
}

/* IVP_U_Matrix3::vmult3 (IVP_U_Float_Point in / out) */
static inline void ivp_scr_f_vmult3(const ivp_scr_f_t *s, const ivp_vec3f_t *in, ivp_vec3f_t *out)
{
    double a = s->m[0][0] * (double)in->k[0] + s->m[0][1] * (double)in->k[1] + s->m[0][2] * (double)in->k[2];
    double b = s->m[1][0] * (double)in->k[0] + s->m[1][1] * (double)in->k[1] + s->m[1][2] * (double)in->k[2];
    double c = s->m[2][0] * (double)in->k[0] + s->m[2][1] * (double)in->k[1] + s->m[2][2] * (double)in->k[2];
    out->k[0] = (float)a;
    out->k[1] = (float)b;
    out->k[2] = (float)c;
}

static inline void ivp_scr_f_exert_impulse_dim1(const ivp_scr_f_t *s, ivp_core_t *c0, ivp_core_t *c1,
                                                const ivp_vec3f_t *imp)
{
    ivp_scr_f_exert(s, c0, c1, imp, 1, true);
}

static inline void ivp_scr_f_exert_impulse_dim2(const ivp_scr_f_t *s, ivp_core_t *c0, ivp_core_t *c1,
                                                const ivp_vec3f_t *imp)
{
    ivp_scr_f_exert(s, c0, c1, imp, 2, true);
}

static inline void ivp_scr_f_exert_impulse_dim3(const ivp_scr_f_t *s, ivp_core_t *c0, ivp_core_t *c1,
                                                const ivp_vec3f_t *imp)
{
    ivp_scr_f_exert(s, c0, c1, imp, 3, true);
}

static inline void ivp_scr_f_exert_angular_impulse_dim3(const ivp_scr_f_t *s, ivp_core_t *c0, ivp_core_t *c1,
                                                        const ivp_vec3f_t *imp)
{
    ivp_scr_f_exert(s, c0, c1, imp, 3, false);
}

#endif /* IVP_SCR_INTERNAL_H */
