/* ivp_cls_internal.h — IVP_CLS (compact ledge solver) and IVP_Cache_Object helpers
 * shared by the mindist minimize port and the friction port.
 * Reference: ivp_collision/ivp_compact_ledge_solver.cxx, ivp_intern/ivp_cache_object.hxx
 */
#ifndef IVP_CLS_INTERNAL_H
#define IVP_CLS_INTERNAL_H

#include <ivp/collision/ivp_collision_core.h>
#include <ivp/base/ivp_assert.h>
#include <math.h>
#include <stdint.h>
#include "collision/ivp_lvec.h"

#if defined(__GNUC__) || defined(__clang__)
#define IVP_CLS_MAYBE_UNUSED __attribute__((unused))
#else
#define IVP_CLS_MAYBE_UNUSED
#endif

/* ── compact ledge helpers ──────────────────────────────────────────────── */

static inline ivp_vec3f_t pp_f(const ivp_compact_poly_point_t *p) {
    ivp_vec3f_t r; r.k[0] = p->k[0]; r.k[1] = p->k[1]; r.k[2] = p->k[2]; return r;
}

static inline const ivp_compact_poly_point_t *give_object_coords(const ivp_compact_edge_t *edge,
                                                                 const ivp_compact_ledge_t *ledge) {
    IVP_ASSERT(ledge == ivp_edge_get_compact_ledge(edge));   /* ivp_compact_ledge_solver.hxx:226 */
    return &ivp_ledge_get_point_array(ledge)[ivp_edge_get_start_point_index(edge)];
}

static inline ivp_vec3f_t obj_f(const ivp_compact_edge_t *edge, const ivp_compact_ledge_t *ledge) {
    return pp_f(give_object_coords(edge, ledge));
}

/* IVP_Compact_Ledge_Solver::give_object_coords(const IVP_Compact_Edge *, IVP_Cache_Ledge_Point *)
 * (ivp_collision/ivp_compact_ledge_solver.cxx:24).  Only this overload checks the 16-byte
 * alignment of the point; the one taking the ledge (give_object_coords above,
 * ivp_compact_ledge_solver.hxx:224) does not.  Grid ledges have 4-byte aligned points, so the
 * check fails on valid data in the reference debug build too: reported, never trapped.
 * Route exactly the reference calls of the cache ledge point overload through here. */
static inline const ivp_compact_poly_point_t *give_object_coords_clp(const ivp_compact_edge_t *edge,
                                                                     const ivp_cache_ledge_point_t *clp) {
    IVP_ASSERT(clp->compact_ledge == ivp_edge_get_compact_ledge(edge));
    int point_index = ivp_edge_get_start_point_index(edge);
    const ivp_compact_poly_point_t *res = &clp->compact_poly_points[point_index];
#define intp(p) ((intptr_t)(p))
    IVP_ASSERT_NONFATAL((intp(res) & 15) == 0);
#undef intp
    return res;
}

static inline ivp_vec3f_t clp_f(const ivp_compact_edge_t *edge, const ivp_cache_ledge_point_t *clp) {
    return pp_f(give_object_coords_clp(edge, clp));
}

static inline const ivp_compact_ledge_t *edge_ledge(const ivp_compact_edge_t *e) {
    return ivp_edge_get_compact_ledge(e);
}

static inline const ivp_compact_triangle_t *edge_tri(const ivp_compact_edge_t *e) {
    return ivp_edge_get_triangle(e);
}

/* IVP_Unscaled_*_Result sign tests: reference ORs the raw sign bits */
typedef struct { float checks[3]; float scale; } qr_result_t;
typedef struct { float checks[2]; } s_result_t;
typedef struct { float checks_K[2]; float checks_L[2]; } kk_result_t;

static inline bool qr_is_outside(const qr_result_t *r) {
    return signbit(r->checks[0]) || signbit(r->checks[1]) || signbit(r->checks[2]);
}
static inline bool s_is_outside(const s_result_t *r) {
    return signbit(r->checks[0]) || signbit(r->checks[1]);
}
static inline bool kk_is_outside_K(const kk_result_t *r) {
    return signbit(r->checks_K[0]) || signbit(r->checks_K[1]);
}
static inline bool kk_is_outside_L(const kk_result_t *r) {
    return signbit(r->checks_L[0]) || signbit(r->checks_L[1]);
}

/* cache object transforms (IVP_Cache_Object::transform_*) */
static inline void co_pos_to_world_f(const ivp_cache_obj_t *co, const ivp_vec3f_t *p_os, ivp_vec3r_t *out) {
    lvm_vmult4_fp(&co->m_world_f_object, p_os, out);
}
static inline void co_pos_to_object(const ivp_cache_obj_t *co, const ivp_vec3r_t *p_ws, ivp_vec3r_t *out) {
    lvm_vimult4(&co->m_world_f_object, p_ws, out);
}
static inline void co_vec_to_world(const ivp_cache_obj_t *co, const ivp_vec3r_t *v_os, ivp_vec3r_t *out) {
    lvm_vmult3(&co->m_world_f_object, v_os, out);
}
static inline void co_vec_to_object(const ivp_cache_obj_t *co, const ivp_vec3r_t *v_ws, ivp_vec3r_t *out) {
    lvm_vimult3(&co->m_world_f_object, v_ws, out);
}

static inline void cls_give_world_coords_AT(const ivp_compact_edge_t *edge, ivp_cache_ledge_point_t *clp, ivp_vec3r_t *out)
{
    IVP_ASSERT(clp->compact_ledge == ivp_edge_get_compact_ledge(edge));   /* ivp_compact_ledge_solver.cxx:712 */
    ivp_vec3f_t p = clp_f(edge, clp);
    co_pos_to_world_f(clp->clp_cache_object, &p, out);
}

static inline void cls_calc_pos_other_space(const ivp_compact_edge_t *P, ivp_cache_ledge_point_t *c_P,
                                     ivp_cache_ledge_point_t *c_other, ivp_vec3r_t *res)
{
    ivp_vec3f_t p_os = clp_f(P, c_P);
    ivp_vec3r_t p_ws;
    lvm_vmult4_fp(&c_P->clp_cache_object->m_world_f_object, &p_os, &p_ws);
    lvm_vimult4(&c_other->clp_cache_object->m_world_f_object, &p_ws, res);
}

static inline void cls_transform_vec_other_space(const ivp_vec3r_t *dir_os, ivp_cache_ledge_point_t *c_dir,
                                          ivp_cache_ledge_point_t *c_other, ivp_vec3r_t *res)
{
    ivp_vec3r_t dir_ws;
    lvm_vmult3(&c_dir->clp_cache_object->m_world_f_object, dir_os, &dir_ws);
    lvm_vimult3(&c_other->clp_cache_object->m_world_f_object, &dir_ws, res);
}

static inline void cls_transform_pos_other_space(const ivp_vec3f_t *pos_in_os, ivp_cache_ledge_point_t *c_dir,
                                          ivp_cache_ledge_point_t *c_other, ivp_vec3r_t *res)
{
    ivp_vec3r_t dir_ws, pos_os;
    lvp_set_f(&pos_os, pos_in_os);
    lvm_vmult4(&c_dir->clp_cache_object->m_world_f_object, &pos_os, &dir_ws);
    lvm_vimult4(&c_other->clp_cache_object->m_world_f_object, &dir_ws, res);
}

static inline void cls_calc_unscaled_s_val_K_space(const ivp_compact_ledge_t *c_ledge, const ivp_compact_edge_t *edge,
                                            const ivp_vec3r_t *p_object, s_result_t *result)
{
    ivp_vec3f_t p0 = obj_f(edge, c_ledge);
    ivp_vec3f_t p1 = obj_f(ivp_edge_get_next(edge), c_ledge);
    ivp_vec3r_t d, d0, d1;
    lvp_subtract2_ff(&d, &p1, &p0);
    lvp_subtract2_fp(&d0, &p0, p_object);
    lvp_subtract2_fp(&d1, &p1, p_object);
    D a = -lvp_dot(&d, &d0);
    D b = lvp_dot(&d, &d1);
    result->checks[0] = (float)a;
    result->checks[1] = (float)b;
}

static const int ivp_uqr_mod_table[] IVP_CLS_MAYBE_UNUSED = { 0, 2, 1, 0, 2 };

static inline void cls_calc_unscaled_qr_vals_F_space(const ivp_compact_ledge_t *c_ledge, const ivp_compact_edge_t *tri,
                                              const ivp_vec3r_t *p_object, qr_result_t *result)
{
    const int this_edge_index = ivp_edge_get_edge_index(tri);
    const ivp_compact_triangle_t *triangle = edge_tri(tri);
    ivp_vec3f_t tp = obj_f(ivp_tri_get_edge(triangle, 0), c_ledge);
    ivp_vec3f_t tp_next = obj_f(ivp_tri_get_edge(triangle, 1), c_ledge);
    ivp_vec3f_t tp_prev = obj_f(ivp_tri_get_edge(triangle, 2), c_ledge);
    ivp_vec3r_t R, Q, Pvec;
    lvp_subtract2_ff(&Q, &tp, &tp_next);
    lvp_subtract2_ff(&R, &tp_prev, &tp_next);
    D QQ = lvp_quad_length(&Q);
    D RR = lvp_quad_length(&R);
    lvp_subtract2_pf(&Pvec, p_object, &tp_next);
    D QR = lvp_dot(&R, &Q);
    D QQRR = QQ * RR;
    D QRQR = QR * QR;
    D Det = (QQRR - QRQR);
    D sq = lvp_dot(&Pvec, &Q);
    D sr = lvp_dot(&Pvec, &R);
    D q = (RR * sq - sr * QR);
    D r = (QQ * sr - sq * QR);
    result->scale = (float)Det;
    IVP_ASSERT(sizeof(result->checks[0]) == sizeof(float));   /* ivp_compact_ledge_solver.cxx:439 (IVP_FLOAT) */
    result->checks[ivp_uqr_mod_table[this_edge_index]] = (float)r;
    result->checks[ivp_uqr_mod_table[this_edge_index + 1]] = (float)(Det - q - r);
    result->checks[ivp_uqr_mod_table[this_edge_index + 2]] = (float)q;
}

static inline void cls_calc_hesse_vec_object_not_normized(const ivp_compact_edge_t *edge, const ivp_compact_ledge_t *ledge,
                                                   ivp_vec3r_t *out_vec)
{
    ivp_vec3f_t p0 = obj_f(edge, ledge);
    ivp_vec3f_t p1 = obj_f(ivp_edge_get_next(edge), ledge);
    ivp_vec3f_t p2 = obj_f(ivp_edge_get_prev(edge), ledge);
    lvp_set_vert_to_area_fff(out_vec, &p0, &p2, &p1);
}

static inline void cls_calc_hesse_object(const ivp_compact_edge_t *edge, const ivp_compact_ledge_t *ledge, lv_hesse_t *h)
{
    ivp_vec3f_t p0 = obj_f(edge, ledge);
    ivp_vec3f_t p1 = obj_f(ivp_edge_get_next(edge), ledge);
    ivp_vec3f_t p2 = obj_f(ivp_edge_get_prev(edge), ledge);
    lv_hesse_calc_fff(h, &p0, &p2, &p1);
    lv_hesse_normize(h);
}

#endif /* IVP_CLS_INTERNAL_H */
