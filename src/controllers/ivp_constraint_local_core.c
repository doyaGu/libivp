/* ivp_constraint_local_core.c -- Reference IVP_Constraint_Local solver
 *
 * Port of ivp_controller/ivp_constraint_local.cxx (init + PSI solver) and
 * ivp_controller/ivp_template_constraint.cxx (template setters).
 *
 * Precision mirrors the reference default build: matrices and IVP_U_Point
 * quantities are double, IVP_U_Float_Point / IVP_FLOAT quantities are
 * rounded to float (FP()) in every build.
 */
#include <ivp/controllers/ivp_constraint_types.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_quat.h>
#include <math.h>
#include <string.h>
#include <ivp/solver/ivp_great_matrix.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include "dynamics/ivp_env_internal.h"

/* reference IVP_FLOAT / IVP_U_Float_Point values: float in every build */
#define FP(x) ((double)(float)(x))

#define CL_PI 3.14159265358979323846
#define CL_DOUBLE_EPS 1e-10      /* reference P_DOUBLE_EPS (double build) */
#define CL_MATRIX_EPS ((double)1e-9f) /* mg_impulse_f_dvRA.MATRIX_EPS = 1E-9f (IVP_DOUBLE member) */

/* ── small double matrix helpers (reference IVP_U_Matrix semantics) ───── */

typedef double m3_t[3][3];

void ivp_cmatrix_identity(ivp_cmatrix_t *m)
{
    memset(m, 0, sizeof(*m));
    m->rows[0][0] = m->rows[1][1] = m->rows[2][2] = 1.0;
}

static void m3_copy_impl(m3_t out, const m3_t a)
{
    memcpy(out, a, sizeof(m3_t));
}

/* out = a * b */
static void m3_mmult_impl(const m3_t a, const m3_t b, m3_t out)
{
    m3_t r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
    m3_copy_impl(out, (const double (*)[3])r);
}

/* out = a^T * b (reference mimult3) */
static void m3_mimult_impl(const m3_t a, const m3_t b, m3_t out)
{
    m3_t r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r[i][j] = a[0][i] * b[0][j] + a[1][i] * b[1][j] + a[2][i] * b[2][j];
    m3_copy_impl(out, (const double (*)[3])r);
}

/* out = a * b^T (reference mi2mult3) */
static void m3_mi2mult_impl(const m3_t a, const m3_t b, m3_t out)
{
    m3_t r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r[i][j] = a[i][0] * b[j][0] + a[i][1] * b[j][1] + a[i][2] * b[j][2];
    m3_copy_impl(out, (const double (*)[3])r);
}

static void m3_vmult_impl(const m3_t a, const double v[3], double out[3])
{
    double r[3];
    for (int i = 0; i < 3; i++)
        r[i] = a[i][0] * v[0] + a[i][1] * v[1] + a[i][2] * v[2];
    out[0] = r[0]; out[1] = r[1]; out[2] = r[2];
}

static void m3_vimult_impl(const m3_t a, const double v[3], double out[3])
{
    double r[3];
    for (int i = 0; i < 3; i++)
        r[i] = a[0][i] * v[0] + a[1][i] * v[1] + a[2][i] * v[2];
    out[0] = r[0]; out[1] = r[1]; out[2] = r[2];
}

static void m3_transpose(m3_t a)
{
    for (int i = 1; i < 3; i++)
        for (int j = 0; j < i; j++) {
            double v = a[i][j];
            a[i][j] = a[j][i];
            a[j][i] = v;
        }
}

static void m3_get_row_impl(const m3_t a, int r, double out[3])
{
    out[0] = a[r][0]; out[1] = a[r][1]; out[2] = a[r][2];
}

static void m3_get_col_impl(const m3_t a, int c, double out[3])
{
    out[0] = a[0][c]; out[1] = a[1][c]; out[2] = a[2][c];
}

/* C17 does not implicitly add const to the element type behind a pointer
 * to an array; C23 does.  Make that read-only conversion explicit while
 * retaining the matrix type and the reference expression order. */
#define M3_READ(m) ((const double (*)[3])(m))
#define m3_copy(out, a) m3_copy_impl((out), M3_READ(a))
#define m3_mmult(a, b, out) m3_mmult_impl(M3_READ(a), M3_READ(b), (out))
#define m3_mimult(a, b, out) m3_mimult_impl(M3_READ(a), M3_READ(b), (out))
#define m3_mi2mult(a, b, out) m3_mi2mult_impl(M3_READ(a), M3_READ(b), (out))
#define m3_vmult(a, v, out) m3_vmult_impl(M3_READ(a), (v), (out))
#define m3_vimult(a, v, out) m3_vimult_impl(M3_READ(a), (v), (out))
#define m3_get_row(a, r, out) m3_get_row_impl(M3_READ(a), (r), (out))
#define m3_get_col(a, c, out) m3_get_col_impl(M3_READ(a), (c), (out))

static void m3_set_col(m3_t a, int c, const double v[3])
{
    a[0][c] = v[0]; a[1][c] = v[1]; a[2][c] = v[2];
}

static void v3_cross(const double a[3], const double b[3], double out[3])
{
    double r0 = a[1] * b[2] - a[2] * b[1];
    double r1 = a[2] * b[0] - a[0] * b[2];
    double r2 = a[0] * b[1] - a[1] * b[0];
    out[0] = r0; out[1] = r1; out[2] = r2;
}

/* IVP_U_Float_Point::inline_calc_cross_product: float products, double
 * differences, float result (inputs are float-exact) */
static void v3_cross_f(const double v1[3], const double v2[3], double out[3])
{
    double a = (double)((float)v1[1] * (float)v2[2]);
    double a2 = (double)((float)v2[1] * (float)v1[2]);
    double b = (double)((float)v1[2] * (float)v2[0]);
    double b2 = (double)((float)v2[2] * (float)v1[0]);
    a -= a2;
    double c = (double)((float)v1[0] * (float)v2[1]);
    double c2 = (double)((float)v2[0] * (float)v1[1]);
    b -= b2;
    c -= c2;
    out[0] = FP(a);
    out[1] = FP(b);
    out[2] = FP(c);
}

/* IVP_U_Point::subtract(const IVP_U_Float_Point *, const IVP_U_Float_Point *):
 * float - float evaluated in float */
static inline double fsub(double a, double b)
{
    return (double)((float)a - (float)b);
}

static double v3_dot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/* reference IVP_U_Point::normize (double build: IVP_Fast_Math::isqrt(length, 3)) */
static bool v3_normize(double v[3])
{
    double q = v3_dot(v, v);
    if (q < CL_DOUBLE_EPS) return false;
    double f = ivp_isqrt_double(q, 3);
    v[0] *= f; v[1] *= f; v[2] *= f;
    return true;
}

/* out = m * p + vv */
static void m4_vmult4(const ivp_cmatrix_t *m, const double p[3], double out[3])
{
    double r[3];
    m3_vmult(m->rows, p, r);
    out[0] = r[0] + m->vv[0]; out[1] = r[1] + m->vv[1]; out[2] = r[2] + m->vv[2];
}

/* out = m^T * (p - vv) */
static void m4_vimult4(const ivp_cmatrix_t *m, const double p[3], double out[3])
{
    double d[3] = { p[0] - m->vv[0], p[1] - m->vv[1], p[2] - m->vv[2] };
    m3_vimult(m->rows, d, out);
}

/* out = a * b */
static void m4_mmult4(const ivp_cmatrix_t *a, const ivp_cmatrix_t *b, ivp_cmatrix_t *out)
{
    ivp_cmatrix_t r;
    m3_mmult(a->rows, b->rows, r.rows);
    m4_vmult4(a, b->vv, r.vv);
    *out = r;
}

/* out = a^-1 * b (orthonormal a) */
static void m4_mimult4(const ivp_cmatrix_t *a, const ivp_cmatrix_t *b, ivp_cmatrix_t *out)
{
    ivp_cmatrix_t r;
    m3_mimult(a->rows, b->rows, r.rows);
    m4_vimult4(a, b->vv, r.vv);
    *out = r;
}

/* out = a * b^-1 (orthonormal b, reference mi2mult4) */
static void m4_mi2mult4(const ivp_cmatrix_t *a, const ivp_cmatrix_t *b, ivp_cmatrix_t *out)
{
    ivp_cmatrix_t r;
    m3_mi2mult(a->rows, b->rows, r.rows);
    double k[3];
    m3_vmult(r.rows, b->vv, k);
    r.vv[0] = a->vv[0] - k[0]; r.vv[1] = a->vv[1] - k[1]; r.vv[2] = a->vv[2] - k[2];
    *out = r;
}

/* reference IVP_U_Matrix::real_invert (determinant algorithm) */
static bool m4_real_invert(ivp_cmatrix_t *m)
{
    double r0[3], r1[3], r2[3];
    v3_cross(m->rows[1], m->rows[2], r0);
    v3_cross(m->rows[2], m->rows[0], r1);
    v3_cross(m->rows[0], m->rows[1], r2);
    double D = v3_dot(m->rows[0], r0);
    if (fabs(D) < CL_DOUBLE_EPS) return false;
    double DI = 1.0 / D;
    for (int i = 0; i < 3; i++) { r0[i] *= DI; r1[i] *= DI; r2[i] *= DI; }
    m3_set_col(m->rows, 0, r0);
    m3_set_col(m->rows, 1, r1);
    m3_set_col(m->rows, 2, r2);
    double p[3];
    m3_vmult(m->rows, m->vv, p);
    m->vv[0] = -p[0]; m->vv[1] = -p[1]; m->vv[2] = -p[2];
    return true;
}

/* reference IVP_U_Matrix3::init_normized3_col */
static void m3_init_normized3_col(m3_t m, const double vp[3], int coordinate)
{
    double v0[3] = { vp[0], vp[1], vp[2] };
    v3_normize(v0);
    double v2[3] = { v0[1], v0[2] - v0[0], -v0[1] };
    if (!v3_normize(v2)) {
        v2[0] = v0[2]; v2[1] = -v0[2]; v2[2] = v0[1] - v0[0];
        v3_normize(v2);
    }
    double v1[3];
    v3_cross(v2, v0, v1);
    static const int p[5] = { 0, 1, 2, 0, 1 };
    m3_set_col(m, p[coordinate], v0);
    m3_set_col(m, p[coordinate + 1], v1);
    m3_set_col(m, p[coordinate + 2], v2);
}

static void m3_identity(m3_t m)
{
    memset(m, 0, sizeof(m3_t));
    m[0][0] = m[1][1] = m[2][2] = 1.0;
}

static void transform_to_cmatrix(const ivp_transform_r_t *t, ivp_cmatrix_t *out)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) out->rows[i][j] = (double)t->rot.rows[i].k[j];
        out->vv[i] = (double)t->pos.k[i];
    }
}

/* core->get_m_world_f_core_PSI() (identity for NULL/world) */
static void core_psi_matrix(const ivp_object_t *obj, ivp_cmatrix_t *out)
{
    if (!obj) {
        ivp_cmatrix_identity(out);
        return;
    }
    transform_to_cmatrix(&obj->physical_core->m_world_f_core_last_psi, out);
}

/* IVP_Real_Object::get_m_world_f_object_AT: the object frame at the current
 * environment time (includes the object<->core shift); identity for NULL. */
static void object_world_matrix(const ivp_object_t *obj, ivp_cmatrix_t *out)
{
    if (!obj) {
        ivp_cmatrix_identity(out);
        return;
    }
    if (!obj->owner) {
        core_psi_matrix(obj, out);
        return;
    }
    ivp_transform_r_t t;
    ivp_object_get_m_world_f_object_AT(obj, &t);
    transform_to_cmatrix(&t, out);
}

/* IVP_Real_Object::calc_m_core_f_object (identity for NULL) */
static void object_core_f_object(const ivp_object_t *obj, ivp_cmatrix_t *out)
{
    ivp_cmatrix_identity(out);
    if (!obj) return;
    if (obj->q_core_f_object) {
        ivp_mat3r_t r;
        ivp_quat_to_mat3(obj->q_core_f_object, &r);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) out->rows[i][j] = (double)r.rows[i].k[j];
    }
    /* shift_core_f_object is an IVP_U_Float_Point in the reference */
    for (int i = 0; i < 3; i++) out->vv[i] = FP(obj->shift_core_f_object.k[i]);
}

static void vec3r_to_d(const ivp_vec3r_t *v, double out[3])
{
    out[0] = (double)v->k[0]; out[1] = (double)v->k[1]; out[2] = (double)v->k[2];
}

/* ── template (reference IVP_Template_Constraint) ──────────────────────── */

void ivp_constraint_template_init(ivp_constraint_template_t *t)
{
    memset(t, 0, sizeof(*t));
    t->force_factor = IVP_REAL(1.0);
    t->damp_factor = IVP_REAL(1.0);
    ivp_cmatrix_identity(&t->m_Ros_f_Rfs);
    ivp_cmatrix_identity(&t->m_Ros_f_Rrs);
    ivp_cmatrix_identity(&t->m_Aos_f_Afs);
    for (int i = 0; i < 3; i++) {
        t->axis_type[i] = IVP_CONSTRAINT_AXIS_FIXED;
        t->axis_type[i + 3] = IVP_CONSTRAINT_AXIS_FREE;
    }
    for (int i = 0; i < 6; i++) t->maximpulse_type[i] = IVP_CFE_NONE;
    t->limited_axis_stiffness = IVP_REAL(0.3);
}

void ivp_constraint_template_set_constraint_Ros(ivp_constraint_template_t *t,
                                                ivp_object_t *obj_r,
                                                const ivp_vec3r_t *anchor_Ros,
                                                const ivp_vec3r_t *known_axis_Ros,
                                                unsigned transdim, unsigned rotdim,
                                                ivp_object_t *obj_a,
                                                const ivp_cmatrix_t *m_Rfs_f_Afs)
{
    ivp_cmatrix_t m_ws_f_Ros, m_ws_f_Aos, m_Aos_f_Ros;
    object_world_matrix(obj_r, &m_ws_f_Ros);
    object_world_matrix(obj_a, &m_ws_f_Aos);
    m4_mimult4(&m_ws_f_Aos, &m_ws_f_Ros, &m_Aos_f_Ros);

    if (known_axis_Ros || anchor_Ros) {
        t->has_m_Ros_f_Rfs = true;
        double axis[3];
        if (!known_axis_Ros) {
            m3_identity(t->m_Ros_f_Rfs.rows);
        } else if (rotdim == 2 || transdim == 2) {
            vec3r_to_d(known_axis_Ros, axis);
            m3_init_normized3_col(t->m_Ros_f_Rfs.rows, axis, 2);
        } else if (rotdim == 1 || transdim == 1) {
            vec3r_to_d(known_axis_Ros, axis);
            m3_init_normized3_col(t->m_Ros_f_Rfs.rows, axis, 0);
        } else {
            m3_identity(t->m_Ros_f_Rfs.rows);
        }
        if (anchor_Ros) {
            vec3r_to_d(anchor_Ros, t->m_Ros_f_Rfs.vv);
        } else {
            t->m_Ros_f_Rfs.vv[0] = t->m_Ros_f_Rfs.vv[1] = t->m_Ros_f_Rfs.vv[2] = 0.0;
        }
    } else {
        t->has_m_Ros_f_Rfs = false;
    }
    t->has_m_Ros_f_Rrs = false;

    if (m_Rfs_f_Afs) {
        t->has_m_Aos_f_Afs = true;
        ivp_cmatrix_t m_Aos_f_Rfs;
        if (t->has_m_Ros_f_Rfs) {
            m4_mmult4(&m_Aos_f_Ros, &t->m_Ros_f_Rfs, &m_Aos_f_Rfs);
        } else {
            m_Aos_f_Rfs = m_Aos_f_Ros;
        }
        m4_mmult4(&m_Aos_f_Rfs, m_Rfs_f_Afs, &t->m_Aos_f_Afs);
    } else {
        t->has_m_Aos_f_Afs = false;
    }

    t->obj_r = obj_r;
    t->obj_a = obj_a;
    int td = (int)transdim, rd = (int)rotdim;
    for (int i = 0; i < 3; i++) {
        t->axis_type[i] = (td-- > 0) ? IVP_CONSTRAINT_AXIS_FIXED : IVP_CONSTRAINT_AXIS_FREE;
    }
    for (int i = 3; i < 6; i++) {
        t->axis_type[i] = (rd-- > 0) ? IVP_CONSTRAINT_AXIS_FIXED : IVP_CONSTRAINT_AXIS_FREE;
    }
}

void ivp_constraint_template_set_constraint_ws(ivp_constraint_template_t *t,
                                               ivp_object_t *obj_r,
                                               const ivp_vec3r_t *anchor_ws,
                                               const ivp_vec3r_t *known_axis_ws,
                                               unsigned transdim, unsigned rotdim,
                                               ivp_object_t *obj_a,
                                               const ivp_cmatrix_t *m_Rfs_f_Afs)
{
    ivp_cmatrix_t m_ws_f_Ros;
    object_world_matrix(obj_r, &m_ws_f_Ros);

    ivp_vec3r_t anchor_Ros, axis_Ros;
    const ivp_vec3r_t *anchor_ptr = NULL, *axis_ptr = NULL;
    if (anchor_ws) {
        double a[3], r[3];
        vec3r_to_d(anchor_ws, a);
        m4_vimult4(&m_ws_f_Ros, a, r);
        ivp_vec3r_set(&anchor_Ros, (ivp_real_t)r[0], (ivp_real_t)r[1], (ivp_real_t)r[2]);
        anchor_ptr = &anchor_Ros;
    }
    if (known_axis_ws) {
        double a[3], r[3];
        vec3r_to_d(known_axis_ws, a);
        m3_vimult(m_ws_f_Ros.rows, a, r);
        if (v3_dot(r, r) > CL_DOUBLE_EPS * CL_DOUBLE_EPS) {
            ivp_vec3r_set(&axis_Ros, (ivp_real_t)r[0], (ivp_real_t)r[1], (ivp_real_t)r[2]);
            axis_ptr = &axis_Ros;
        }
    }
    ivp_constraint_template_set_constraint_Ros(t, obj_r, anchor_ptr, axis_ptr, transdim, rotdim,
                                               obj_a, m_Rfs_f_Afs);
}

void ivp_constraint_template_set_ballsocket_ws(ivp_constraint_template_t *t, ivp_object_t *obj_r,
                                               const ivp_vec3r_t *anchor_ws, ivp_object_t *obj_a)
{
    ivp_constraint_template_set_constraint_ws(t, obj_r, anchor_ws, NULL, 3, 0, obj_a, NULL);
}

void ivp_constraint_template_set_ballsocket_tense_Ros(ivp_constraint_template_t *t, ivp_object_t *obj_r,
                                                      const ivp_vec3r_t *anchor_Ros, ivp_object_t *obj_a,
                                                      const ivp_vec3r_t *distanceAR_Ros)
{
    ivp_cmatrix_t disp;
    ivp_cmatrix_identity(&disp);
    vec3r_to_d(distanceAR_Ros, disp.vv);
    ivp_constraint_template_set_constraint_Ros(t, obj_r, anchor_Ros, NULL, 3, 0, obj_a, &disp);
}

void ivp_constraint_template_set_cardanjoint_ws(ivp_constraint_template_t *t, ivp_object_t *obj_r,
                                                const ivp_vec3r_t *anchor_ws,
                                                const ivp_vec3r_t *fixed_axis_ws, ivp_object_t *obj_a)
{
    ivp_constraint_template_set_constraint_ws(t, obj_r, anchor_ws, fixed_axis_ws, 3, 1, obj_a, NULL);
}

void ivp_constraint_template_set_hinge_ws(ivp_constraint_template_t *t, ivp_object_t *obj_r,
                                          const ivp_vec3r_t *anchor_ws,
                                          const ivp_vec3r_t *free_axis_ws, ivp_object_t *obj_a)
{
    ivp_constraint_template_set_constraint_ws(t, obj_r, anchor_ws, free_axis_ws, 3, 2, obj_a, NULL);
}

void ivp_constraint_template_set_fixed(ivp_constraint_template_t *t, ivp_object_t *obj_r,
                                       ivp_object_t *obj_a)
{
    ivp_constraint_template_set_constraint_ws(t, obj_r, NULL, NULL, 3, 3, obj_a, NULL);
}

void ivp_constraint_template_set_orientation(ivp_constraint_template_t *t, ivp_object_t *obj_r,
                                             ivp_object_t *obj_a)
{
    ivp_constraint_template_set_constraint_ws(t, obj_r, NULL, NULL, 0, 3, obj_a, NULL);
}

void ivp_constraint_template_set_attached_Aos(ivp_constraint_template_t *t,
                                              const ivp_vec3r_t *anchor_Aos,
                                              const ivp_vec3r_t *axis_Aos,
                                              int axis_index)
{
    ivp_cmatrix_t m_ws_f_Ros, m_ws_f_Aos, m_Aos_f_Ros;
    object_world_matrix(t->obj_r, &m_ws_f_Ros);
    object_world_matrix(t->obj_a, &m_ws_f_Aos);
    m4_mimult4(&m_ws_f_Aos, &m_ws_f_Ros, &m_Aos_f_Ros);
    m3_t rel;
    if (t->has_m_Ros_f_Rfs) m3_mmult(m_Aos_f_Ros.rows, t->m_Ros_f_Rfs.rows, rel);
    else m3_copy(rel, m_Aos_f_Ros.rows);

    if (axis_Aos && axis_index >= 0 && axis_index < 3) {
        double ax[3];
        vec3r_to_d(axis_Aos, ax);
        double col[3];
        m3_get_col(rel, axis_index, col);
        double n = sqrt(v3_dot(ax, ax));
        if (n > 0.0 && v3_dot(col, ax) / n < 0.9999) {
            m3_init_normized3_col(rel, ax, axis_index);
        }
    }
    m3_copy(t->m_Aos_f_Afs.rows, rel);
    if (anchor_Aos) vec3r_to_d(anchor_Aos, t->m_Aos_f_Afs.vv);
    else t->m_Aos_f_Afs.vv[0] = t->m_Aos_f_Afs.vv[1] = t->m_Aos_f_Afs.vv[2] = 0.0;
    t->has_m_Aos_f_Afs = true;
}

void ivp_constraint_template_limit_axis(ivp_constraint_template_t *t, int axis6,
                                        ivp_real_t border_left, ivp_real_t border_right)
{
    if (axis6 < 0 || axis6 >= 6) return;
    t->axis_type[axis6] = IVP_CONSTRAINT_AXIS_LIMITED;
    t->borderleft_Rfs[axis6] = border_left;
    t->borderright_Rfs[axis6] = border_right;
}

/* ── core init (reference IVP_Constraint_Local::init) ──────────────────── */

void ivp_constraint_local_core_sort_mappings(ivp_constraint_local_core_t *c)
{
    /* sort_translation_mapping */
    int pos = 0;
    for (int i = 0; i < 3; i++) if (c->fixed[i] == IVP_CONSTRAINT_AXIS_FIXED) c->mapping_uRfs_f_Rfs[pos++] = i;
    for (int i = 0; i < 3; i++) if (c->fixed[i] == IVP_CONSTRAINT_AXIS_LIMITED) c->mapping_uRfs_f_Rfs[pos++] = i;
    for (int i = 0; i < 3; i++) if (c->fixed[i] == IVP_CONSTRAINT_AXIS_FREE) c->mapping_uRfs_f_Rfs[pos++] = i;
    c->fixedtrans_dim = c->limitedtrans_dim = 0;
    for (int i = 0; i < 3; i++) {
        if (c->fixed[i] == IVP_CONSTRAINT_AXIS_FIXED) c->fixedtrans_dim++;
        if (c->fixed[i] & IVP_CONSTRAINT_AXIS_LIMITED) c->limitedtrans_dim++;
    }
    /* sort_rotation_mapping */
    pos = 0;
    for (int i = 0; i < 3; i++) if (c->fixed[i + 3] == IVP_CONSTRAINT_AXIS_FIXED) c->mapping_uRrs_f_Rrs[pos++] = i;
    for (int i = 0; i < 3; i++) if (c->fixed[i + 3] == IVP_CONSTRAINT_AXIS_LIMITED) c->mapping_uRrs_f_Rrs[pos++] = i;
    for (int i = 0; i < 3; i++) if (c->fixed[i + 3] == IVP_CONSTRAINT_AXIS_FREE) c->mapping_uRrs_f_Rrs[pos++] = i;
    c->fixedrot_dim = c->limitedrot_dim = 0;
    for (int i = 3; i < 6; i++) {
        if (c->fixed[i] == IVP_CONSTRAINT_AXIS_FIXED) c->fixedrot_dim++;
        if (c->fixed[i] & IVP_CONSTRAINT_AXIS_LIMITED) c->limitedrot_dim++;
    }
    c->matrix_size = c->fixedtrans_dim + c->fixedrot_dim + c->limitedtrans_dim + c->limitedrot_dim;
}

void ivp_constraint_local_core_set_factors(ivp_constraint_local_core_t *core,
                                           ivp_real_t force_factor,
                                           ivp_real_t damp_factor)
{
    /* IVP_Controller_Factory::create_constraint clamps force_factor >= P_FLOAT_EPS;
     * IVP_Constraint_Local::init: IVP_FLOAT force_factor and
     * damp_factor_div_force = tmpl.damp_factor / force_factor (float / float) */
    float ff = (float)force_factor;
    if (ff <= IVP_FLOAT_EPS) ff = IVP_FLOAT_EPS;
    core->force_factor = (ivp_real_t)ff;
    core->damp_factor_div_force = (ivp_real_t)((float)damp_factor / ff);
}

void ivp_constraint_local_core_init(ivp_constraint_local_core_t *c,
                                    const ivp_constraint_template_t *tmpl)
{
    memset(c, 0, sizeof(*c));
    c->norm = IVP_NORM_MAXIMUM;

    ivp_object_t *obj_r = tmpl->obj_r;
    ivp_object_t *obj_a = tmpl->obj_a;

    if (tmpl->has_m_Aos_f_Afs) {
        ivp_controller_wake_object(obj_r ? obj_r : obj_a);
    }

    ivp_cmatrix_t m_ws_f_Ros, m_ws_f_Aos, m_Ros_f_Aos;
    object_world_matrix(obj_r, &m_ws_f_Ros);
    object_world_matrix(obj_a, &m_ws_f_Aos);
    m4_mimult4(&m_ws_f_Ros, &m_ws_f_Aos, &m_Ros_f_Aos);

    ivp_cmatrix_t m_Rcs_f_Ros, m_Acs_f_Aos, m_Rcs_f_Aos, m_Rcs_f_Acs;
    object_core_f_object(obj_r, &m_Rcs_f_Ros);
    object_core_f_object(obj_a, &m_Acs_f_Aos);
    m4_mmult4(&m_Rcs_f_Ros, &m_Ros_f_Aos, &m_Rcs_f_Aos);
    m4_mi2mult4(&m_Rcs_f_Aos, &m_Acs_f_Aos, &m_Rcs_f_Acs);

    if (tmpl->has_m_Ros_f_Rfs) {
        m4_mmult4(&m_Rcs_f_Ros, &tmpl->m_Ros_f_Rfs, &c->m_Rfs_f_Rcs);
        m4_real_invert(&c->m_Rfs_f_Rcs);
    } else {
        c->m_Rfs_f_Rcs = m_Rcs_f_Ros;
        m4_real_invert(&c->m_Rfs_f_Rcs);
    }
    if (tmpl->has_m_Aos_f_Afs) {
        m4_mmult4(&m_Acs_f_Aos, &tmpl->m_Aos_f_Afs, &c->m_Afs_f_Acs);
        m4_real_invert(&c->m_Afs_f_Acs);
    } else {
        /* Afs is Rfs (relaxed) */
        m4_mmult4(&c->m_Rfs_f_Rcs, &m_Rcs_f_Acs, &c->m_Afs_f_Acs);
    }
    if (tmpl->has_m_Ros_f_Rrs) {
        c->has_rot = true;
        m3_mmult(m_Rcs_f_Ros.rows, tmpl->m_Ros_f_Rrs.rows, c->m_Rrs_f_Rcs);
        m3_transpose(c->m_Rrs_f_Rcs);
        m3_t m_fs_f_rs;
        m3_mi2mult(c->m_Rfs_f_Rcs.rows, c->m_Rrs_f_Rcs, m_fs_f_rs);
        m3_mimult(m_fs_f_rs, c->m_Afs_f_Acs.rows, c->m_Ars_f_Acs);
    } else {
        c->has_rot = false;
    }

    c->obj_r = obj_r;
    c->obj_a = obj_a;
    for (int i = 0; i < 6; i++) c->fixed[i] = (int)tmpl->axis_type[i];
    ivp_constraint_local_core_sort_mappings(c);

    ivp_constraint_local_core_set_factors(c, tmpl->force_factor, tmpl->damp_factor);
    /* IVP_FLOAT members */
    c->limited_axis_stiffness = (ivp_real_t)(float)tmpl->limited_axis_stiffness;
    c->use_maxforce = false;
    for (int i = 0; i < 6; i++) {
        c->borderright_Rfs[i] = (ivp_real_t)(float)tmpl->borderright_Rfs[i];
        c->borderleft_Rfs[i] = (ivp_real_t)(float)tmpl->borderleft_Rfs[i];
        c->halfimpulse[i] = (ivp_real_t)(float)tmpl->maximpulse[i];
        c->maxforce_type[i] = tmpl->maximpulse_type[i];
        if (tmpl->maximpulse_type[i] != IVP_CFE_NONE) c->use_maxforce = true;
    }
    c->enabled = true;   /* reference: init(); activate(); */
    c->broken = false;
}

void ivp_constraint_local_core_wake(ivp_constraint_local_core_t *core)
{
    /* reference: objectR ? objectR->ensure_in_simulation() : objectA->... */
    if (core->obj_r) ivp_controller_wake_object(core->obj_r);
    else ivp_controller_wake_object(core->obj_a);
}

void ivp_constraint_local_core_relax_attached(ivp_constraint_local_core_t *c)
{
    ivp_constraint_local_core_wake(c);
    ivp_cmatrix_t m_ws_f_Rcs, m_ws_f_Acs, m_Rcs_f_Acs;
    core_psi_matrix(c->obj_r, &m_ws_f_Rcs);
    core_psi_matrix(c->obj_a, &m_ws_f_Acs);
    m4_mimult4(&m_ws_f_Rcs, &m_ws_f_Acs, &m_Rcs_f_Acs);
    m4_mmult4(&c->m_Rfs_f_Rcs, &m_Rcs_f_Acs, &c->m_Afs_f_Acs);
    if (c->has_rot) m3_mmult(c->m_Rrs_f_Rcs, m_Rcs_f_Acs.rows, c->m_Ars_f_Acs);
}

void ivp_constraint_local_core_relax_reference(ivp_constraint_local_core_t *c)
{
    ivp_constraint_local_core_wake(c);
    ivp_cmatrix_t m_ws_f_Rcs, m_ws_f_Acs, m_Acs_f_Rcs;
    core_psi_matrix(c->obj_r, &m_ws_f_Rcs);
    core_psi_matrix(c->obj_a, &m_ws_f_Acs);
    m4_mimult4(&m_ws_f_Acs, &m_ws_f_Rcs, &m_Acs_f_Rcs);
    m4_mmult4(&c->m_Afs_f_Acs, &m_Acs_f_Rcs, &c->m_Rfs_f_Rcs);
    if (c->has_rot) m3_mmult(c->m_Ars_f_Acs, m_Acs_f_Rcs.rows, c->m_Rrs_f_Rcs);
}

/* ── reference IVP_Great_Matrix_Many_Zero::solve_great_matrix_many_zero ─ */

/* Reference: an IVP_Great_Matrix_Many_Zero on alloca'd arrays (columns =
 * matrix_size, aligned_row_len, MATRIX_EPS 1E-9f) solved with
 * solve_great_matrix_many_zero.  ivp_gmmz_solve reproduces the reference
 * Gauss elimination including the 64-bit IVP_VecFPU row helpers: an
 * elimination step starting at an odd column processes whole blocks of 4
 * doubles from the previous column, which for a 4x4 system (cardan joint:
 * 3 translation + 1 rotation axis) also adds the source's next row to the
 * target's next row.  The arrays carry zeroed slack for those overruns (the
 * reference's uninitialised alloca memory there is never read back into
 * the solution). */
static bool solve_great_matrix_many_zero(int n, double a[6][6], double b[6], double x[6])
{
    enum { CL_GM_SLACK = 16 };
    double values[6 * 8 + CL_GM_SLACK];
    double desired[8 + CL_GM_SLACK];
    double result[8 + CL_GM_SLACK];
    memset(values, 0, sizeof(values));
    memset(desired, 0, sizeof(desired));
    memset(result, 0, sizeof(result));

    ivp_gmmz_t m;
    memset(&m, 0, sizeof(m));
    m.columns = n;
    m.aligned_row_len = ivp_gm_aligned_len(n);
    m.matrix_eps = CL_MATRIX_EPS;
    IVP_ASSERT(m.aligned_row_len <= 8);
    m.matrix_values = values;
    m.desired_vector = desired;
    m.result_vector = result;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) values[i * m.aligned_row_len + j] = a[i][j];
        desired[i] = b[i];
    }
    if (ivp_gmmz_solve(&m) != IVP_OK) {
        for (int k = 0; k < n; k++) x[k] = 0.0;
        return false;
    }
    for (int i = 0; i < n; i++) x[i] = result[i];
    return true;
}

/* ── core helpers mirroring IVP_Core test pushes ───────────────────────── */

static bool core_static(const ivp_core_t *c)
{
    return !c || c->physical_unmoveable;
}

/* get_surface_speed_on_test: R * (w x p) + v */
static void surface_speed_on_test(const ivp_cmatrix_t *m_ws_f_cs, const double p_cs[3],
                                  const double v_ws[3], const double w_cs[3], double out[3])
{
    double s_cs[3];
    v3_cross_f(w_cs, p_cs, s_cs);
    m3_vmult(m_ws_f_cs->rows, s_cs, out);
    /* inline_vmult3 (float result), IVP_U_Float_Point::add (float) */
    out[0] = (double)((float)out[0] + (float)v_ws[0]);
    out[1] = (double)((float)out[1] + (float)v_ws[1]);
    out[2] = (double)((float)out[2] + (float)v_ws[2]);
}

static void core_speeds(const ivp_core_t *c, double v[3], double w[3])
{
    vec3r_to_d(&c->speed, v);
    w[0] = (double)c->rot_speed.k[0];
    w[1] = (double)c->rot_speed.k[1];
    w[2] = (double)c->rot_speed.k[2];
}

/* test_push_core: rot = (p x imp_cs) (.) inv_I, speed = imp_ws * inv_m */
static void test_push_core(const ivp_core_t *c, const double p_cs[3], const double imp_cs[3],
                           const double imp_ws[3], double dv_ws[3], double dw_cs[3])
{
    double d[3];
    v3_cross_f(p_cs, imp_cs, d);
    for (int k = 0; k < 3; k++) {
        dw_cs[k] = (double)((float)d[k] * (float)c->inv_rot_inertia.k[k]);
        dv_ws[k] = FP(imp_ws[k] * (double)(float)c->inv_mass);
    }
}

static void push_core(ivp_core_t *c, const double p_cs[3], const double imp_cs[3],
                      const double imp_ws[3])
{
    double dv[3], dw[3];
    test_push_core(c, p_cs, imp_cs, imp_ws, dv, dw);
    for (int k = 0; k < 3; k++) { /* rot_speed.add / speed.add (IVP_U_Float_Point) */
        c->rot_speed.k[k] += (float)dw[k];
        c->speed.k[k] = (ivp_real_t)((float)c->speed.k[k] + (float)dv[k]);
    }
}

/* IVP_Core::rot_push_core_cs: retail RVA 0xCB70 keeps only the X inertia
 * product through the addition */
static void rot_push_core_cs(ivp_core_t *c, const double imp_cs[3])
{
    c->rot_speed.k[0] = (float)(FP(imp_cs[0]) * (double)(float)c->inv_rot_inertia.k[0] + (double)c->rot_speed.k[0]);
    for (int k = 1; k < 3; k++) {
        c->rot_speed.k[k] += (float)FP(imp_cs[k] * (double)c->inv_rot_inertia.k[k]);
    }
}

/* IVP_Core::test_rot_push_core_multiple_cs (debug check of the reference
 * solver): h = axis (.) inv_rot_inertia (float), out = h * rot_impulse */
static void test_rot_push_core_multiple_cs(const ivp_core_t *c, const double axis_cs[3], double rot_impulse,
                                           double out[3])
{
    for (int k = 0; k < 3; k++) {
        const float h = (float)axis_cs[k] * (float)c->inv_rot_inertia.k[k];
        out[k] = FP((double)h * rot_impulse);
    }
}

/* IVP_U_Float_Point::real_length (float quad_length, double sqrt) */
static double float_real_length(const double v[3])
{
    const float x = (float)v[0], y = (float)v[1], z = (float)v[2];
    return sqrt((double)(x * x + y * y + z * z));
}

/* reference IVP_Inline_Math::fast_asin (float polynomial) */
static double fast_asin(double a)
{
    float angle = (float)a;
    float q = angle * angle;
    float t = angle * q;
    float qi = t * q;
    return (double)(angle + 0.12f * t + 0.29f * qi);
}

/* reference IVP_U_Quat::set_quaternion(const IVP_U_Matrix3 *) + normize_quat */
static void matrix_to_quat(const m3_t m, double q_out[4] /* x y z w */)
{
    double tr = m[0][0] + m[1][1] + m[2][2];
    double x, y, z, w;
    if (tr > 0.0) {
        double s = sqrt(tr + 1.0);
        w = 0.5 * s;
        s = 0.5 / s;
        x = (m[2][1] - m[1][2]) * s;
        y = (m[0][2] - m[2][0]) * s;
        z = (m[1][0] - m[0][1]) * s;
    } else {
        double q[4];
        static const int nxt[3] = { 1, 2, 0 };
        int i = 0;
        if (m[1][1] > m[0][0]) i = 1;
        if (m[2][2] > m[i][i]) i = 2;
        int j = nxt[i], k = nxt[j];
        double s = sqrt(m[i][i] - (m[j][j] + m[k][k]) + 1.0);
        q[i] = s * 0.5;
        if (s != 0.0) s = 0.5 / s;
        q[3] = (m[k][j] - m[j][k]) * s;
        q[j] = (m[j][i] + m[i][j]) * s;
        q[k] = (m[k][i] + m[i][k]) * s;
        x = q[0]; y = q[1]; z = q[2]; w = q[3];
    }
    double len = sqrt(x * x + y * y + z * z + w * w);
    if (len > 0.0) {
        double il = 1.0 / len;
        x *= il; y *= il; z *= il; w *= il;
    }
    q_out[0] = x; q_out[1] = y; q_out[2] = z; q_out[3] = w;
}

/* ── PSI solver (reference IVP_Constraint_Local::do_simulation_controller) */

bool ivp_constraint_local_core_simulate(ivp_constraint_local_core_t *c,
                                        const ivp_event_sim_t *es)
{
    if (!c->enabled) return false;

    const double d_time = FP(es->delta_time);
    const double inv_dtime = (double)es->inverse_delta_time;

    ivp_core_t *coreR = c->obj_r ? c->obj_r->physical_core : NULL;
    ivp_core_t *coreA = c->obj_a ? c->obj_a->physical_core : NULL;
    IVP_ASSERT(!coreR || coreR->physical_unmoveable || coreR->movement_state != IVP_MT_NOT_SIM);
    IVP_ASSERT(!coreA || coreA->physical_unmoveable || coreA->movement_state != IVP_MT_NOT_SIM);

    const bool fast_ball_socket =
        coreR && coreA && !core_static(coreR) && !core_static(coreA) &&
        !c->force_generic_path && !c->has_rot && !c->use_maxforce &&
        c->fixedtrans_dim == 3 && c->fixedrot_dim == 0 &&
        c->limitedtrans_dim == 0 && c->limitedrot_dim == 0 &&
        c->fixed[0] == IVP_CONSTRAINT_AXIS_FIXED &&
        c->fixed[1] == IVP_CONSTRAINT_AXIS_FIXED &&
        c->fixed[2] == IVP_CONSTRAINT_AXIS_FIXED &&
        c->fixed[3] == IVP_CONSTRAINT_AXIS_FREE &&
        c->fixed[4] == IVP_CONSTRAINT_AXIS_FREE &&
        c->fixed[5] == IVP_CONSTRAINT_AXIS_FREE;

    ivp_cmatrix_t m_ws_f_Rcs, m_ws_f_Acs;
    core_psi_matrix(c->obj_r, &m_ws_f_Rcs);
    core_psi_matrix(c->obj_a, &m_ws_f_Acs);

    /* translation system */
    const double (*m_fs_f_Rcs)[3] = M3_READ(c->m_Rfs_f_Rcs.rows);
    m3_t m_fs_f_ws, m_fs_f_Acs;
    m3_mi2mult(m_fs_f_Rcs, m_ws_f_Rcs.rows, m_fs_f_ws);
    m3_mmult(m_fs_f_ws, m_ws_f_Acs.rows, m_fs_f_Acs);

    /* rotation system and pretension */
    const double (*m_Rrs_f_Rcs)[3] =
        M3_READ(c->has_rot ? c->m_Rrs_f_Rcs : c->m_Rfs_f_Rcs.rows);
    const double (*m_Ars_f_Acs)[3] =
        M3_READ(c->has_rot ? c->m_Ars_f_Acs : c->m_Afs_f_Acs.rows);
    m3_t m_rs_f_Rcs_buffer;
    const double (*pm_rs_f_Rcs)[3] = m_Rrs_f_Rcs;
    m3_t m_rs_f_ws, m_rs_f_Acs;

    const int *mapR = c->mapping_uRrs_f_Rrs;
    double drRA_now_rs[3] = { 0.0, 0.0, 0.0 };
    const int rotdims = c->fixedrot_dim + c->limitedrot_dim;

    if (fast_ball_socket) {
        /* Rotation axes are all free.  The attached rotation frame is still
         * built because the generic path transforms its exact zero impulse
         * through it before adding it to the core. */
        m3_mi2mult(pm_rs_f_Rcs, m_ws_f_Rcs.rows, m_rs_f_ws);
        m3_mmult(m_rs_f_ws, m_ws_f_Acs.rows, m_rs_f_Acs);
    } else if (rotdims == 1) {
        /* cardan joint */
        pm_rs_f_Rcs = M3_READ(m_rs_f_Rcs_buffer);
        double fixed_axisArs_Acs[3], fixed_axisArs_ws[3], fixed_axisArs_Rcs[3], fixed_axisRrs_Rcs[3];
        m3_get_row(m_Ars_f_Acs, mapR[0], fixed_axisArs_Acs);
        m3_vmult(m_ws_f_Acs.rows, fixed_axisArs_Acs, fixed_axisArs_ws);
        m3_vimult(m_ws_f_Rcs.rows, fixed_axisArs_ws, fixed_axisArs_Rcs);
        m3_get_row(m_Rrs_f_Rcs, mapR[0], fixed_axisRrs_Rcs);
        double vertical_axisRA_Rcs[3];
        v3_cross(fixed_axisArs_Rcs, fixed_axisRrs_Rcs, vertical_axisRA_Rcs);

        double rsx_Rcs[3] = { fixed_axisArs_Rcs[0] + fixed_axisRrs_Rcs[0],
                              fixed_axisArs_Rcs[1] + fixed_axisRrs_Rcs[1],
                              fixed_axisArs_Rcs[2] + fixed_axisRrs_Rcs[2] };
        m3_init_normized3_col(m_rs_f_Rcs_buffer, rsx_Rcs, mapR[0]);
        m3_transpose(m_rs_f_Rcs_buffer);
        m3_mi2mult(pm_rs_f_Rcs, m_ws_f_Rcs.rows, m_rs_f_ws);
        m3_mmult(m_rs_f_ws, m_ws_f_Acs.rows, m_rs_f_Acs);

        double vertical_axisRA_Rrs[3];
        m3_vmult(m_Rrs_f_Rcs, vertical_axisRA_Rcs, vertical_axisRA_Rrs);
        double angle_Rrs = atan2(vertical_axisRA_Rrs[mapR[1]], vertical_axisRA_Rrs[mapR[2]]);

        double vertical_axisRA_ws[3], vertical_axisRA_Acs[3], vertical_axisRA_Ars[3];
        m3_vmult(m_ws_f_Rcs.rows, vertical_axisRA_Rcs, vertical_axisRA_ws);
        m3_vimult(m_ws_f_Acs.rows, vertical_axisRA_ws, vertical_axisRA_Acs);
        m3_vmult(m_Ars_f_Acs, vertical_axisRA_Acs, vertical_axisRA_Ars);
        double angle_Ars = atan2(vertical_axisRA_Ars[mapR[1]], vertical_axisRA_Ars[mapR[2]]);

        double diff_axis = angle_Ars - angle_Rrs;
        if (diff_axis > CL_PI) diff_axis -= 2.0 * CL_PI;
        if (diff_axis < -CL_PI) diff_axis += 2.0 * CL_PI;
        drRA_now_rs[mapR[0]] = diff_axis;
        drRA_now_rs[mapR[1]] = 0.0;
        drRA_now_rs[mapR[2]] = 0.0;
    } else {
        m3_mi2mult(pm_rs_f_Rcs, m_ws_f_Rcs.rows, m_rs_f_ws);
        m3_mmult(m_rs_f_ws, m_ws_f_Acs.rows, m_rs_f_Acs);
    }
    const double (*m_rs_f_Rcs)[3] = pm_rs_f_Rcs;

    if (rotdims == 3) {
        m3_t m_rs_f_Ars;
        m3_mi2mult(m_rs_f_Acs, m_Ars_f_Acs, m_rs_f_Ars);
        double q[4];
        matrix_to_quat(M3_READ(m_rs_f_Ars), q);
        drRA_now_rs[0] = 2.0 * fast_asin(q[0]);
        drRA_now_rs[1] = 2.0 * fast_asin(q[1]);
        drRA_now_rs[2] = 2.0 * fast_asin(q[2]);
        if (q[3] < 0.0) {
            drRA_now_rs[0] = -drRA_now_rs[0];
            drRA_now_rs[1] = -drRA_now_rs[1];
            drRA_now_rs[2] = -drRA_now_rs[2];
        }
    } else if (rotdims == 2) {
        /* hinge */
        m3_t m_rs_f_Ars;
        m3_mi2mult(m_rs_f_Acs, m_Ars_f_Acs, m_rs_f_Ars);
        double free_axisArs_Rrs[3];
        m3_get_col(m_rs_f_Ars, mapR[2], free_axisArs_Rrs);
        drRA_now_rs[0] = drRA_now_rs[1] = drRA_now_rs[2] = 0.0;
        drRA_now_rs[mapR[0]] = -atan2(free_axisArs_Rrs[mapR[1]], free_axisArs_Rrs[mapR[2]]);
        drRA_now_rs[mapR[1]] = atan2(free_axisArs_Rrs[mapR[0]], free_axisArs_Rrs[mapR[2]]);
    } else if (rotdims == 0) {
        drRA_now_rs[0] = drRA_now_rs[1] = drRA_now_rs[2] = 0.0;
    }

    /* translation differences */
    /* At this point drRA_rs is virtually the same as rRA_rs
     * (ivp_constraint_local.cxx:487) */
    IVP_IF(0) {
        const float world_vec[3] = { 0.0f, 0.01f, 0.0f };
        if (coreR)
            ivp_env_add_draw_vector(coreR->environment, m_ws_f_Rcs.vv, world_vec, c->obj_r->name, 3);
        if (coreA)
            ivp_env_add_draw_vector(coreA->environment, m_ws_f_Acs.vv, world_vec, c->obj_a->name, 3);
    }

    const double nullvec[3] = { 0.0, 0.0, 0.0 };
    double pointR_Rcs[3], pointA_Acs[3];
    m4_vimult4(&c->m_Rfs_f_Rcs, nullvec, pointR_Rcs);
    m4_vimult4(&c->m_Afs_f_Acs, nullvec, pointA_Acs);
    for (int k = 0; k < 3; k++) { pointR_Rcs[k] = FP(pointR_Rcs[k]); pointA_Acs[k] = FP(pointA_Acs[k]); }
    double pointR_ws[3], pointA_ws[3];
    m4_vmult4(&m_ws_f_Rcs, pointR_Rcs, pointR_ws);
    m4_vmult4(&m_ws_f_Acs, pointA_Acs, pointA_ws);

    double dvRA_fs[3], drRA_rs[3];
    {
        double v_pointR_ws[3] = { 0.0, 0.0, 0.0 }, v_pointA_ws[3] = { 0.0, 0.0, 0.0 };
        if (coreR) {
            double v[3], w[3];
            core_speeds(coreR, v, w);
            surface_speed_on_test(&m_ws_f_Rcs, pointR_Rcs, v, w, v_pointR_ws);
        }
        if (coreA) {
            double v[3], w[3];
            core_speeds(coreA, v, w);
            surface_speed_on_test(&m_ws_f_Acs, pointA_Acs, v, w, v_pointA_ws);
        }
        double v_pointR_fs[3], v_pointA_fs[3];
        m3_vmult(m_fs_f_ws, v_pointR_ws, v_pointR_fs);
        m3_vmult(m_fs_f_ws, v_pointA_ws, v_pointA_fs);
        for (int k = 0; k < 3; k++) { v_pointR_fs[k] = FP(v_pointR_fs[k]); v_pointA_fs[k] = FP(v_pointA_fs[k]); }

        double dvRA_ws[3] = { pointA_ws[0] - pointR_ws[0], pointA_ws[1] - pointR_ws[1],
                              pointA_ws[2] - pointR_ws[2] };
        double dRA_now_fs[3];
        m3_vmult(m_fs_f_ws, dvRA_ws, dRA_now_fs);
        double dv_fs[3];
        for (int k = 0; k < 3; k++) dv_fs[k] = fsub(v_pointA_fs[k], v_pointR_fs[k]);
        /* d_time * damp_factor_div_force: IVP_FLOAT * IVP_FLOAT */
        const double ddf = (double)((float)d_time * (float)c->damp_factor_div_force);
        for (int k = 0; k < 3; k++) dvRA_fs[k] = dRA_now_fs[k] + dv_fs[k] * ddf;

        for (int i = 0; i < 3; i++) {
            if (c->fixed[i] & IVP_CONSTRAINT_AXIS_LIMITED) {
                if (dvRA_fs[i] < (double)(float)c->borderleft_Rfs[i]) {
                    dvRA_fs[i] = (dRA_now_fs[i] - (double)(float)c->borderleft_Rfs[i]) * (double)(float)c->limited_axis_stiffness + dv_fs[i] * d_time;
                    c->fixed[i] |= IVP_CONSTRAINT_AXIS_FIXED;
                } else if (dvRA_fs[i] > (double)(float)c->borderright_Rfs[i]) {
                    dvRA_fs[i] = (dRA_now_fs[i] - (double)(float)c->borderright_Rfs[i]) * (double)(float)c->limited_axis_stiffness + dv_fs[i] * d_time;
                    c->fixed[i] |= IVP_CONSTRAINT_AXIS_FIXED;
                } else {
                    c->fixed[i] &= ~IVP_CONSTRAINT_AXIS_FIXED;
                }
            }
        }
        const double mf = (double)(-(float)c->force_factor) * inv_dtime;
        for (int k = 0; k < 3; k++) dvRA_fs[k] *= mf;

        /* rotation */
        if (fast_ball_socket) {
            drRA_rs[0] = drRA_rs[1] = drRA_rs[2] = 0.0;
        } else {
            double rR_rs[3] = { 0.0, 0.0, 0.0 }, rA_rs[3] = { 0.0, 0.0, 0.0 };
            if (coreR) {
                double v[3], w[3];
                core_speeds(coreR, v, w);
                m3_vmult(m_rs_f_Rcs, w, rR_rs);
            }
            if (coreA) {
                double v[3], w[3];
                core_speeds(coreA, v, w);
                m3_vmult(m_rs_f_Acs, w, rA_rs);
            }
            for (int k = 0; k < 3; k++) { rR_rs[k] = FP(rR_rs[k]); rA_rs[k] = FP(rA_rs[k]); }
            double drs_rs[3];
            for (int k = 0; k < 3; k++) {
                drs_rs[k] = fsub(rA_rs[k], rR_rs[k]);
                drRA_rs[k] = drRA_now_rs[k] + drs_rs[k] * ddf;
            }
            for (int i = 0; i < 3; i++) {
                if (c->fixed[i + 3] & IVP_CONSTRAINT_AXIS_LIMITED) {
                    if (drRA_rs[i] < (double)(float)c->borderleft_Rfs[i + 3]) {
                        drRA_rs[i] = (drRA_now_rs[i] - (double)(float)c->borderleft_Rfs[i + 3]) * (double)(float)c->limited_axis_stiffness + drs_rs[i] * d_time;
                        c->fixed[i + 3] |= IVP_CONSTRAINT_AXIS_FIXED;
                    } else if (drRA_rs[i] > (double)(float)c->borderright_Rfs[i + 3]) {
                        drRA_rs[i] = (drRA_now_rs[i] - (double)(float)c->borderright_Rfs[i + 3]) * (double)(float)c->limited_axis_stiffness + drs_rs[i] * d_time;
                        c->fixed[i + 3] |= IVP_CONSTRAINT_AXIS_FIXED;
                    } else {
                        c->fixed[i + 3] &= ~IVP_CONSTRAINT_AXIS_FIXED;
                    }
                }
            }
            for (int k = 0; k < 3; k++) drRA_rs[k] *= mf;
        }
    }

    if (fast_ball_socket) {
        c->matrix_size = 3;
    } else {
        c->matrix_size = 0;
        for (int i = 0; i < 6; i++) c->matrix_size += c->fixed[i] & IVP_CONSTRAINT_AXIS_FIXED;
    }

    if (c->fixedrot_dim == 1) {
        /* cardan joints: reduce forces when both R-spaces form angles [90..180] */
        double fixed_axisArs_Acs[3], fixed_axisArs_ws[3], fixed_axisArs_Rcs[3], fixed_axisRrs_Rcs[3];
        m3_get_row(m_Ars_f_Acs, mapR[0], fixed_axisArs_Acs);
        m3_vmult(m_ws_f_Acs.rows, fixed_axisArs_Acs, fixed_axisArs_ws);
        m3_vimult(m_ws_f_Rcs.rows, fixed_axisArs_ws, fixed_axisArs_Rcs);
        m3_get_row(m_Rrs_f_Rcs, mapR[0], fixed_axisRrs_Rcs);
        float cosv = (float)v3_dot(fixed_axisArs_Rcs, fixed_axisRrs_Rcs);
        double angleRA = (cosv >= 1.0f) ? 0.0 : (double)(float)acos((double)cosv); /* save_acosf */
        double slowrange = CL_PI - fabs(angleRA);
        if (slowrange < 1.0) {
            for (int k = 0; k < 3; k++) drRA_rs[k] *= slowrange;
        }
    }

    /* test pushes */
    double testnext_dvRA_fs[6][3], testnext_drRA_rs[6][3];
    memset(testnext_dvRA_fs, 0, sizeof(testnext_dvRA_fs));
    memset(testnext_drRA_rs, 0, sizeof(testnext_drRA_rs));

    for (int i = 0; i < 3; i++) {
        if (!(c->fixed[i] & IVP_CONSTRAINT_AXIS_FIXED)) continue;
        double tR_fs[3] = { 0.0, 0.0, 0.0 }, tA_fs[3];
        tR_fs[i] = 1.0;
        for (int k = 0; k < 3; k++) tA_fs[k] = -tR_fs[k];
        double tR_Rcs[3], tA_Acs[3], tR_ws[3], tA_ws[3];
        m3_vimult(m_fs_f_Rcs, tR_fs, tR_Rcs);
        m3_vimult(m_fs_f_Acs, tA_fs, tA_Acs);
        m3_vimult(m_fs_f_ws, tR_fs, tR_ws);
        m3_vimult(m_fs_f_ws, tA_fs, tA_ws);
        for (int k = 0; k < 3; k++) {
            tR_Rcs[k] = FP(tR_Rcs[k]); tA_Acs[k] = FP(tA_Acs[k]);
            tR_ws[k] = FP(tR_ws[k]); tA_ws[k] = FP(tA_ws[k]);
        }

        double dvR_ws[3] = { 0, 0, 0 }, drR_Rcs[3] = { 0, 0, 0 };
        double dvA_ws[3] = { 0, 0, 0 }, drA_Acs[3] = { 0, 0, 0 };
        if (!core_static(coreR)) test_push_core(coreR, pointR_Rcs, tR_Rcs, tR_ws, dvR_ws, drR_Rcs);
        if (!core_static(coreA)) test_push_core(coreA, pointA_Acs, tA_Acs, tA_ws, dvA_ws, drA_Acs);

        double dv_pointR_ws[3] = { 0, 0, 0 }, dv_pointA_ws[3] = { 0, 0, 0 };
        if (coreR) surface_speed_on_test(&m_ws_f_Rcs, pointR_Rcs, dvR_ws, drR_Rcs, dv_pointR_ws);
        if (coreA) surface_speed_on_test(&m_ws_f_Acs, pointA_Acs, dvA_ws, drA_Acs, dv_pointA_ws);

        double dv_pointR_fs[3], dv_pointA_fs[3];
        m3_vmult(m_fs_f_ws, dv_pointR_ws, dv_pointR_fs);
        m3_vmult(m_fs_f_ws, dv_pointA_ws, dv_pointA_fs);
        for (int k = 0; k < 3; k++) {
            testnext_dvRA_fs[i][k] = fsub(FP(dv_pointA_fs[k]), FP(dv_pointR_fs[k]));
        }
        if (!fast_ball_socket) {
            double drR_rs[3], drA_rs[3];
            m3_vmult(m_rs_f_Rcs, drR_Rcs, drR_rs);
            m3_vmult(m_rs_f_Acs, drA_Acs, drA_rs);
            for (int k = 0; k < 3; k++) {
                testnext_drRA_rs[i][k] = fsub(FP(drA_rs[k]), FP(drR_rs[k]));
            }
        }
    }

    for (int i = fast_ball_socket ? 6 : 3; i < 6; i++) {
        if (!(c->fixed[i] & IVP_CONSTRAINT_AXIS_FIXED)) continue;
        double tR_rs[3] = { 0.0, 0.0, 0.0 }, tA_rs[3];
        tR_rs[i - 3] = 1.0;
        for (int k = 0; k < 3; k++) tA_rs[k] = -tR_rs[k];
        double tR_Rcs[3], tA_Acs[3];
        m3_vimult(m_rs_f_Rcs, tR_rs, tR_Rcs);
        m3_vimult(m_rs_f_Acs, tA_rs, tA_Acs);
        for (int k = 0; k < 3; k++) { tR_Rcs[k] = FP(tR_Rcs[k]); tA_Acs[k] = FP(tA_Acs[k]); }

        double drR_rot_Rcs[3] = { 0, 0, 0 }, drA_rot_Acs[3] = { 0, 0, 0 };
        if (!core_static(coreR)) {
            for (int k = 0; k < 3; k++) drR_rot_Rcs[k] = FP(tR_Rcs[k] * (double)coreR->inv_rot_inertia.k[k]);
        }
        if (!core_static(coreA)) {
            for (int k = 0; k < 3; k++) drA_rot_Acs[k] = FP(tA_Acs[k] * (double)coreA->inv_rot_inertia.k[k]);
        }
        double dv_pointR_Rcs[3] = { 0, 0, 0 }, dv_pointA_Acs[3] = { 0, 0, 0 };
        if (coreR) v3_cross_f(drR_rot_Rcs, pointR_Rcs, dv_pointR_Rcs);
        if (coreA) v3_cross_f(drA_rot_Acs, pointA_Acs, dv_pointA_Acs);

        double dv_pointR_fs[3], dv_pointA_fs[3];
        m3_vmult(m_fs_f_Rcs, dv_pointR_Rcs, dv_pointR_fs);
        m3_vmult(m_fs_f_Acs, dv_pointA_Acs, dv_pointA_fs);
        double drR_rs[3], drA_rs[3];
        m3_vmult(m_rs_f_Rcs, drR_rot_Rcs, drR_rs);
        m3_vmult(m_rs_f_Acs, drA_rot_Acs, drA_rs);
        for (int k = 0; k < 3; k++) {
            testnext_dvRA_fs[i][k] = fsub(FP(dv_pointA_fs[k]), FP(dv_pointR_fs[k]));
            testnext_drRA_rs[i][k] = fsub(FP(drA_rs[k]), FP(drR_rs[k]));
        }
    }

    /* build and solve the dense system */
    const int n = c->matrix_size;
    double A[6][6], b[6], x[6];
    memset(A, 0, sizeof(A));
    memset(b, 0, sizeof(b));
    memset(x, 0, sizeof(x));
    if (fast_ball_socket) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) A[j][i] = testnext_dvRA_fs[i][j];
            b[i] = dvRA_fs[i];
        }
    } else {
        int pos_i = 0;
        for (int i = 0; i < 6; i++) {
            if (!(c->fixed[i] & IVP_CONSTRAINT_AXIS_FIXED)) continue;
            int pos_j = 0;
            int j;
            for (j = 0; j < 3; j++) {
                if (c->fixed[j] & IVP_CONSTRAINT_AXIS_FIXED) {
                    A[pos_j][pos_i] = testnext_dvRA_fs[i][j];
                    pos_j++;
                }
            }
            for (; j < 6; j++) {
                if (c->fixed[j] & IVP_CONSTRAINT_AXIS_FIXED) {
                    A[pos_j][pos_i] = testnext_drRA_rs[i][j - 3];
                    pos_j++;
                }
            }
            b[pos_i] = (i < 3) ? dvRA_fs[i] : drRA_rs[i - 3];
            pos_i++;
        }
    }
    if (!solve_great_matrix_many_zero(n, A, b, x)) {
        return false;
    }

    double impulseR_fs[3], impulserotR_rs[3];
    {
        int pos_i = 0;
        for (int i = 0; i < 3; i++) {
            if (c->fixed[i] & IVP_CONSTRAINT_AXIS_FIXED) impulseR_fs[i] = FP(x[pos_i++]);
            else impulseR_fs[i] = 0.0;
        }
        if (fast_ball_socket) {
            impulserotR_rs[0] = impulserotR_rs[1] = impulserotR_rs[2] = 0.0;
        } else {
            for (int j = 3; j < 6; j++) {
                if (c->fixed[j] & IVP_CONSTRAINT_AXIS_FIXED) impulserotR_rs[j - 3] = FP(x[pos_i++]);
                else impulserotR_rs[j - 3] = 0.0;
            }
        }
    }

    if (c->use_maxforce) {
        const ivp_real_t *half = c->halfimpulse;
        bool in_range = false;
        switch (c->norm) {
        case IVP_NORM_MINIMUM:
            in_range = (fabs(impulseR_fs[0]) / (double)half[0] +
                        fabs(impulseR_fs[1]) / (double)half[1] +
                        fabs(impulseR_fs[2]) / (double)half[2]) < 3.0;
            break;
        case IVP_NORM_EUCLIDIC: {
            double ex = fabs(impulseR_fs[0]) / (double)half[0];
            double ey = fabs(impulseR_fs[1]) / (double)half[1];
            double ez = fabs(impulseR_fs[2]) / (double)half[2];
            double ql = ex * ex + ey * ey + ez * ez;
            in_range = ql * inv_dtime * inv_dtime < 1.0;
            break;
        }
        case IVP_NORM_MAXIMUM:
        default:
            in_range = fabs(impulseR_fs[0]) < (double)half[0] && fabs(impulseR_fs[1]) < (double)half[1] &&
                       fabs(impulseR_fs[2]) < (double)half[2] && fabs(impulserotR_rs[0]) < (double)half[3] &&
                       fabs(impulserotR_rs[1]) < (double)half[4] && fabs(impulserotR_rs[2]) < (double)half[5];
            break;
        }
        IVP_IF(0) { /* ivp_constraint_local.cxx:944 */
            ivp_debug_printf("%f  %f  %f     ", fabs(impulseR_fs[0]), fabs(impulseR_fs[1]), fabs(impulseR_fs[2]));
            ivp_debug_printf("%f  %f  %f\n", fabs(impulserotR_rs[0]), fabs(impulserotR_rs[1]), fabs(impulserotR_rs[2]));
        }
        if (!in_range) {
            double maxforce_factor = 1.0;
            static const int map[5] = { 0, 1, 2, 0, 1 };
            for (int i = 0; i < 3; i++) {
                double simple_trans = 0.0, simple_rot = 0.0;
                switch (c->norm) {
                case IVP_NORM_MINIMUM:
                    simple_trans = fabs(impulseR_fs[0]) + fabs(impulseR_fs[1]) + fabs(impulseR_fs[2]);
                    break;
                case IVP_NORM_EUCLIDIC:
                    simple_trans = fabs(impulseR_fs[map[i]]) + fabs(impulseR_fs[map[i + 1]]) * (1.0 / 2.41) +
                                   fabs(impulseR_fs[map[i + 2]]) * (1.0 / 2.41);
                    break;
                case IVP_NORM_MAXIMUM:
                default:
                    simple_trans = fabs(impulseR_fs[i]);
                    simple_rot = fabs(impulserotR_rs[i]);
                    break;
                }
                if ((double)half[i] < simple_trans) {
                    switch (c->maxforce_type[i]) {
                    case IVP_CFE_CLIP: {
                        /* IVP_FLOAT newfactor = d_time * halfimpulse / simplenorm */
                        double newfactor = (double)(float)((double)((float)d_time * (float)half[i]) / simple_trans);
                        if (newfactor < maxforce_factor) maxforce_factor = newfactor;
                        break;
                    }
                    case IVP_CFE_BREAK:
                        c->enabled = false;
                        c->broken = true;
                        return true;
                    case IVP_CFE_BEND: /* reference: not implemented */
                    case IVP_CFE_NONE:
                    default:
                        break;
                    }
                }
                if (simple_rot > CL_DOUBLE_EPS && (double)half[i + 3] / simple_rot < 1.0) {
                    switch (c->maxforce_type[i + 3]) {
                    case IVP_CFE_BREAK:
                        c->enabled = false;
                        c->broken = true;
                        return true;
                    case IVP_CFE_CLIP: /* reference: no effect on rotation */
                    case IVP_CFE_BEND:
                    case IVP_CFE_NONE:
                    default:
                        break;
                    }
                }
            }
            for (int k = 0; k < 3; k++) {
                impulseR_fs[k] = FP(impulseR_fs[k] * maxforce_factor);
                impulserotR_rs[k] = FP(impulserotR_rs[k] * maxforce_factor);
            }
        }
    }

    for (int k = 0; k < 3; k++) {
        c->last_impulse[k] = (ivp_real_t)impulseR_fs[k];
        c->last_impulse[k + 3] = (ivp_real_t)impulserotR_rs[k];
    }

    double impulseA_fs[3], impulserotA_rs[3];
    for (int k = 0; k < 3; k++) {
        impulseA_fs[k] = -impulseR_fs[k];
        impulserotA_rs[k] = -impulserotR_rs[k];
    }

    double impulseR_Rcs[3], impulseA_Acs[3], impulseR_ws[3], impulseA_ws[3];
    double impulserotR_Rcs[3], impulserotA_Acs[3];
    m3_vimult(m_fs_f_Rcs, impulseR_fs, impulseR_Rcs);
    m3_vimult(m_fs_f_Acs, impulseA_fs, impulseA_Acs);
    for (int k = 0; k < 3; k++) { impulseR_Rcs[k] = FP(impulseR_Rcs[k]); impulseA_Acs[k] = FP(impulseA_Acs[k]); }
    m3_vmult(m_ws_f_Rcs.rows, impulseR_Rcs, impulseR_ws);
    m3_vmult(m_ws_f_Acs.rows, impulseA_Acs, impulseA_ws);
    m3_vimult(m_rs_f_Rcs, impulserotR_rs, impulserotR_Rcs);
    m3_vimult(m_rs_f_Acs, impulserotA_rs, impulserotA_Acs);
    for (int k = 0; k < 3; k++) {
        impulseR_ws[k] = FP(impulseR_ws[k]); impulseA_ws[k] = FP(impulseA_ws[k]);
        impulserotR_Rcs[k] = FP(impulserotR_Rcs[k]); impulserotA_Acs[k] = FP(impulserotA_Acs[k]);
    }

    IVP_IF(0) { /* check for plausible values (ivp_constraint_local.cxx:1048) */
        double result_dvRA_fs[3], result_drRA_rs[3];
        double dvR_ws[3], dvA_ws[3], drR_Rcs[3], drA_Acs[3], drR_rot_Rcs[3], drA_rot_Acs[3];
        if (coreR && coreR->physical_unmoveable) {
            for (int k = 0; k < 3; k++) dvR_ws[k] = drR_Rcs[k] = drR_rot_Rcs[k] = 0.0;
        } else {
            test_push_core(coreR, pointR_Rcs, impulseR_Rcs, impulseR_ws, dvR_ws, drR_Rcs);
            test_rot_push_core_multiple_cs(coreR, impulserotR_Rcs, (double)1.0f, drR_rot_Rcs);
        }
        if (coreA && coreA->physical_unmoveable) {
            for (int k = 0; k < 3; k++) dvA_ws[k] = drA_Acs[k] = drA_rot_Acs[k] = 0.0;
        } else {
            test_push_core(coreA, pointA_Acs, impulseA_Acs, impulseA_ws, dvA_ws, drA_Acs);
            test_rot_push_core_multiple_cs(coreA, impulserotA_Acs, (double)1.0f, drA_rot_Acs);
        }
        for (int k = 0; k < 3; k++) result_dvRA_fs[k] = FP(dvA_ws[k] - dvR_ws[k]);
        m3_vmult(m_fs_f_ws, result_dvRA_fs, result_dvRA_fs);
        for (int k = 0; k < 3; k++) {
            result_dvRA_fs[k] = FP(result_dvRA_fs[k]);
            drR_Rcs[k] = (double)((float)drR_Rcs[k] + (float)drR_rot_Rcs[k]);
            drA_Acs[k] = (double)((float)drA_Acs[k] + (float)drA_rot_Acs[k]);
        }
        m3_vmult(m_rs_f_Rcs, drR_Rcs, drR_Rcs);
        m3_vmult(m_rs_f_Acs, drA_Acs, drA_Acs);
        for (int k = 0; k < 3; k++) result_drRA_rs[k] = (double)((float)drA_Acs[k] - (float)drR_Rcs[k]);
        (void)result_dvRA_fs;
        (void)result_drRA_rs;
    }
    IVP_IF(0) { /* ivp_constraint_local.cxx:1081 */
        const char *name_r = c->obj_r->name ? c->obj_r->name : "(null)";
        const char *name_a = c->obj_a->name ? c->obj_a->name : "(null)";
        ivp_debug_printf("%f [%s]-[%s]: IT=%f  IR=%f\n", (double)ivp_env_get_current_time(coreR->environment),
                         name_r, name_a, float_real_length(impulseR_Rcs), float_real_length(impulserotR_Rcs));
    }
    const double debugfactor = (double)0.01f;
    if (!core_static(coreR)) {
        push_core(coreR, pointR_Rcs, impulseR_Rcs, impulseR_ws);
        rot_push_core_cs(coreR, impulserotR_Rcs);
        IVP_IF(1) { /* show lines (ivp_constraint_local.cxx:1091) */
            /* IVP_U_Float_Point::mult(IVP_DOUBLE) */
            const double f = debugfactor * 2.0 * inv_dtime;
            float dir[3];
            for (int k = 0; k < 3; k++) {
                impulseR_ws[k] = FP(impulseR_ws[k] * f);
                dir[k] = (float)impulseR_ws[k];
            }
            ivp_env_add_draw_vector(coreR->environment, pointR_ws, dir, "", 2); /* green */
            double impulserotR_ws[3];
            m3_vmult(m_ws_f_Rcs.rows, impulserotR_Rcs, impulserotR_ws);
            for (int k = 0; k < 3; k++) {
                impulserotR_ws[k] = FP(impulserotR_ws[k]);
                impulserotR_ws[k] = FP(impulserotR_ws[k] * f);
                dir[k] = (float)impulserotR_ws[k];
            }
            ivp_env_add_draw_vector(coreR->environment, pointR_ws, dir, "", 5); /* yellow */
        }
    }
    if (!core_static(coreA)) {
        push_core(coreA, pointA_Acs, impulseA_Acs, impulseA_ws);
        rot_push_core_cs(coreA, impulserotA_Acs);
    }
    return false;
}
