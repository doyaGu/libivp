/* ivp_solver_core_reaction.c — port of IVP_Solver_Core_Reaction
 *
 * The public double-typed API wraps the float-exact implementation in
 * solver/ivp_scr_internal.h: the directions and impulses are rounded to
 * IVP_U_Float_Point, cross_direction_position_cs* / cr_mult_inv* hold the
 * reference's float values (IVP_U_Point_4), delta_velocity_ds holds the
 * reference's float IVP_U_Float_Point values and m is the reference's double
 * IVP_U_Matrix3, so the results are bit-identical to the reference.
 */
#include <ivp/solver/ivp_solver_core_reaction.h>
#include <ivp/dynamics/ivp_core.h>
#include <string.h>
#include <math.h>
#include "solver/ivp_scr_internal.h"

static inline void dir_to_f(ivp_vec3f_t *out, const ivp_vec3r_t *d)
{
    out->k[0] = (float)d->k[0];
    out->k[1] = (float)d->k[1];
    out->k[2] = (float)d->k[2];
}

static inline void p4_to_public(ivp_scr_point4_t *out, const ivp_scr_p4_t *p)
{
    out->k[0] = (double)p->k[0];
    out->k[1] = (double)p->k[1];
    out->k[2] = (double)p->k[2];
    out->w = (double)p->hesse_val;
}

static inline void p4_from_public(ivp_scr_p4_t *out, const ivp_scr_point4_t *p)
{
    out->k[0] = (float)p->k[0];
    out->k[1] = (float)p->k[1];
    out->k[2] = (float)p->k[2];
    out->hesse_val = (float)p->w;
}

/* float copies of the public (double) directions; NULL stays NULL */
static void directions_to_f(const ivp_solver_core_reaction_t *s, ivp_vec3f_t df[3], const ivp_vec3f_t *dp[3])
{
    for (int i = 0; i < 3; i++) {
        if (s->direction_ws[i]) {
            dir_to_f(&df[i], s->direction_ws[i]);
            dp[i] = &df[i];
        } else {
            dp[i] = NULL;
        }
    }
}

static void store_public(ivp_solver_core_reaction_t *s, const ivp_scr_f_t *f)
{
    for (int i = 0; i < 3; i++) {
        p4_to_public(&s->cross_direction_position_cs0[i], &f->cross_direction_position_cs0[i]);
        p4_to_public(&s->cross_direction_position_cs1[i], &f->cross_direction_position_cs1[i]);
        p4_to_public(&s->cr_mult_inv0[i], &f->cr_mult_inv0[i]);
        p4_to_public(&s->cr_mult_inv1[i], &f->cr_mult_inv1[i]);
        s->delta_velocity_ds[i] = (double)f->delta_velocity_ds.k[i];
    }
    memcpy(s->m, f->m, sizeof(s->m));
}

void ivp_scr_init_translation_ws(ivp_solver_core_reaction_t *s,
                                 ivp_core_t *core_0, ivp_core_t *core_1,
                                 const ivp_vec3r_t *pos_ws,
                                 const ivp_vec3r_t *d0, const ivp_vec3r_t *d1,
                                 const ivp_vec3r_t *d2)
{
    s->direction_ws[0] = d0;
    s->direction_ws[1] = d1;
    s->direction_ws[2] = d1 ? d2 : NULL;
    ivp_vec3f_t df[3];
    const ivp_vec3f_t *dp[3];
    directions_to_f(s, df, dp);
    ivp_scr_f_t f;
    memset(&f, 0, sizeof(f));
    ivp_scr_f_init_translation_ws(&f, core_0, core_1, pos_ws, dp[0], dp[1], dp[2]);
    store_public(s, &f);
}

void ivp_scr_init_rotation_ws(ivp_solver_core_reaction_t *s,
                              ivp_core_t *core_0, ivp_core_t *core_1,
                              const ivp_vec3r_t *d0, const ivp_vec3r_t *d1,
                              const ivp_vec3r_t *d2)
{
    s->direction_ws[0] = d0;
    s->direction_ws[1] = d1;
    s->direction_ws[2] = d2;
    ivp_vec3f_t df[3];
    const ivp_vec3f_t *dp[3];
    directions_to_f(s, df, dp);
    ivp_scr_f_t f;
    memset(&f, 0, sizeof(f));
    ivp_scr_f_init_rotation_ws(&f, core_0, core_1, dp[0], dp[1], dp[2]);
    store_public(s, &f);
}

ivp_result_t ivp_scr_invert_3x3(ivp_solver_core_reaction_t *s)
{
    double (*m)[3] = s->m;
    m[1][0] = m[0][1];
    m[2][0] = m[0][2];
    m[2][1] = m[1][2];
    /* IVP_U_Matrix3::real_invert(P_DOUBLE_EPS) */
    if (!ivp_scr_f_real_invert(m, P_DOUBLE_EPS)) return IVP_ERR_DEGENERATE_GEOMETRY;
    return IVP_OK;
}

ivp_result_t ivp_scr_invert_2x2(ivp_solver_core_reaction_t *s)
{
    /* IVP_U_Matrix3::real_invert on the symmetric 2x2 block with a unit
     * third row/column */
    double m[3][3] = { { s->m[0][0], s->m[0][1], 0.0 },
                       { s->m[0][1], s->m[1][1], 0.0 },
                       { 0.0, 0.0, 1.0 } };
    if (!ivp_scr_f_real_invert(m, P_DOUBLE_EPS)) return IVP_ERR_DEGENERATE_GEOMETRY;
    s->m[0][0] = m[0][0];
    s->m[0][1] = m[0][1];
    s->m[1][0] = m[1][0];
    s->m[1][1] = m[1][1];
    return IVP_OK;
}

static void exert(const ivp_solver_core_reaction_t *s, ivp_core_t *c0, ivp_core_t *c1,
                  const double imp[3], int dims, bool linear)
{
    ivp_scr_f_t f;
    memset(&f, 0, sizeof(f));
    ivp_vec3f_t df[3];
    const ivp_vec3f_t *dp[3];
    directions_to_f(s, df, dp);
    ivp_vec3f_t impulse;   /* IVP_U_Float_Point &impulse_ds */
    lvf_zero(&impulse);
    for (int i = 0; i < dims; i++) {
        f.direction_ws[i] = dp[i];
        p4_from_public(&f.cr_mult_inv0[i], &s->cr_mult_inv0[i]);
        p4_from_public(&f.cr_mult_inv1[i], &s->cr_mult_inv1[i]);
        impulse.k[i] = (float)imp[i];
    }
    ivp_scr_f_exert(&f, c0, c1, &impulse, dims, linear);
}

void ivp_scr_exert_impulse_dim1(const ivp_solver_core_reaction_t *s, ivp_core_t *c0, ivp_core_t *c1, const double imp[3]) { exert(s, c0, c1, imp, 1, true); }
void ivp_scr_exert_impulse_dim2(const ivp_solver_core_reaction_t *s, ivp_core_t *c0, ivp_core_t *c1, const double imp[3]) { exert(s, c0, c1, imp, 2, true); }
void ivp_scr_exert_impulse_dim3(const ivp_solver_core_reaction_t *s, ivp_core_t *c0, ivp_core_t *c1, const double imp[3]) { exert(s, c0, c1, imp, 3, true); }
void ivp_scr_exert_angular_impulse_dim1(const ivp_solver_core_reaction_t *s, ivp_core_t *c0, ivp_core_t *c1, const double imp[3]) { exert(s, c0, c1, imp, 1, false); }
void ivp_scr_exert_angular_impulse_dim2(const ivp_solver_core_reaction_t *s, ivp_core_t *c0, ivp_core_t *c1, const double imp[3]) { exert(s, c0, c1, imp, 2, false); }
void ivp_scr_exert_angular_impulse_dim3(const ivp_solver_core_reaction_t *s, ivp_core_t *c0, ivp_core_t *c1, const double imp[3]) { exert(s, c0, c1, imp, 3, false); }
