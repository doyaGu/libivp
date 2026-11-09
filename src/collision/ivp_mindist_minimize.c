/* ivp_mindist_minimize.c — port of IVP_Mindist_Minimize_Solver
 *
 * Reference: ivp_physics/ivp_mindist_minimize.{hxx,cxx}, ivp_mindist_mcases.cxx,
 * ivp_collision/ivp_compact_ledge_solver.cxx (the IVP_CLS functions used by the
 * Voronoi walk).  The walk continues from the synapses' current features
 * (persistent between PSIs), exactly like the reference.
 */
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/diagnostics/ivp_debug_manager.h>
#include <ivp/utility/ivp_string.h>
#include <string.h>
#include <math.h>
#include "collision/ivp_lvec.h"
#include "collision/ivp_mindist_debug.h"
#include "dynamics/ivp_env_internal.h"
#include "dynamics/ivp_env_hot.h"

#define IVP_MAX_MINIMIZE_BEFORE_HASH_CHECK 20
#define IVP_MAX_PIERCINGS 2
#define IVP_LOOP_LIST_SIZE 256

void ivp_mindist_mindist_rescue_push(ivp_mindist_t *md);   /* ivp_mindist_impact.c */

#include "collision/ivp_cls_internal.h"


static D cls_quad_dist_edge_to_point_K_space(const ivp_compact_ledge_t *ledge_K, const ivp_compact_edge_t *K,
                                             const ivp_vec3r_t *object_pos)
{
    ivp_vec3f_t p0 = obj_f(K, ledge_K);
    ivp_vec3f_t p1 = obj_f(ivp_edge_get_next(K), ledge_K);
    ivp_vec3r_t vec_world;
    lvp_set_vert_to_area_ffp(&vec_world, &p0, &p1, object_pos);
    D edge_ql = lvf_quad_distance(&p0, &p1);
    if (edge_ql < P_DOUBLE_EPS) return 0.0;
    return lvp_quad_length(&vec_world) / edge_ql;
}

static D cls_calc_qlen_PP_P_space(const ivp_compact_ledge_t *P_ledge, const ivp_compact_edge_t *P, const ivp_vec3r_t *P_Pos)
{
    ivp_vec3f_t P_os = obj_f(P, P_ledge);
    return lvp_quad_distance_f(P_Pos, &P_os);
}

static D cls_calc_qlen_PK_K_space(const ivp_vec3r_t *P_in_K_space, const ivp_compact_ledge_t *K_ledge,
                                  const ivp_compact_edge_t *K)
{
    /* ivp_compact_ledge_solver.cxx:688 IVP_IF (PS2 alignment check): inside "#if 0" in the reference */
    s_result_t sr;
    cls_calc_unscaled_s_val_K_space(K_ledge, K, P_in_K_space, &sr);
    if (!s_is_outside(&sr)) {
        return cls_quad_dist_edge_to_point_K_space(K_ledge, K, P_in_K_space);
    }
    if (sr.checks[0] < 0.0f) {
        return cls_calc_qlen_PP_P_space(K_ledge, K, P_in_K_space);
    }
    return cls_calc_qlen_PP_P_space(K_ledge, ivp_edge_get_next(K), P_in_K_space);
}

D ivp_lcls_calc_qlen_PF_F_space(const ivp_compact_ledge_t *ledge, const ivp_compact_triangle_t *tri,
                                const ivp_vec3r_t *object_pos)
{
    const ivp_compact_edge_t *e = ivp_tri_get_first_edge(tri);
    qr_result_t qr;
    cls_calc_unscaled_qr_vals_F_space(ledge, e, object_pos, &qr);
    if (!qr_is_outside(&qr)) {
        ivp_vec3r_t normal;
        ivp_vec3f_t start_point = obj_f(e, ledge);
        cls_calc_hesse_vec_object_not_normized(e, ledge, &normal);
        D n_ql = lvp_quad_length(&normal);
        if (n_ql < P_DOUBLE_EPS) return P_DOUBLE_MAX;
        D res = lvp_dot(&normal, object_pos) - lvp_dot_f(&normal, &start_point);
        return res * res / n_ql;
    }
    D qres = P_DOUBLE_MAX;
    for (int i = 2; i >= 0; i--) {
        e = ivp_tri_get_edge(tri, i);
        s_result_t sr;
        cls_calc_unscaled_s_val_K_space(ledge, e, object_pos, &sr);
        if (sr.checks[0] < 0) {
            ivp_vec3f_t e_object = obj_f(e, ledge);
            D qdist = lvp_quad_distance_f(object_pos, &e_object);
            if (qdist < qres) qres = qdist;
            continue;
        }
        if (sr.checks[1] < 0) continue;
        D qdist = cls_quad_dist_edge_to_point_K_space(ledge, e, object_pos);
        if (qdist < qres) qres = qdist;
    }
    return qres;
}

/* IVP_KK_Input */
typedef struct kk_input {
    ivp_vec3f_t L_Los[2];   /* reference stores pointers to the float object coords */
    ivp_vec3r_t K_Los[2];
    ivp_vec3r_t Kvec_Los;
    ivp_vec3r_t Lvec_Los;
    const ivp_compact_edge_t *K;
    const ivp_compact_edge_t *L;
    ivp_cache_ledge_point_t *cache_K;
    ivp_cache_ledge_point_t *cache_L;
    ivp_vec3r_t cross_KL_Los;
} kk_input_t;

static void kk_input_init(kk_input_t *in, const ivp_compact_edge_t *K_in, const ivp_compact_edge_t *L_in,
                          ivp_cache_ledge_point_t *c_K, ivp_cache_ledge_point_t *c_L)
{
    in->K = K_in;
    in->L = L_in;
    ivp_vec3f_t K_os = clp_f(K_in, c_K);
    ivp_vec3f_t K_next_os = clp_f(ivp_edge_get_next(K_in), c_K);
    ivp_vec3r_t Kvec_Kos;
    lvp_subtract2_ff(&Kvec_Kos, &K_next_os, &K_os);
    cls_transform_vec_other_space(&Kvec_Kos, c_K, c_L, &in->Kvec_Los);
    in->cache_K = c_K;
    in->cache_L = c_L;
    cls_calc_pos_other_space(K_in, c_K, c_L, &in->K_Los[0]);
    cls_calc_pos_other_space(ivp_edge_get_next(K_in), c_K, c_L, &in->K_Los[1]);
    in->L_Los[0] = clp_f(L_in, c_L);
    in->L_Los[1] = clp_f(ivp_edge_get_next(L_in), c_L);
    lvp_subtract2_ff(&in->Lvec_Los, &in->L_Los[1], &in->L_Los[0]);
    lvp_cross(&in->cross_KL_Los, &in->Kvec_Los, &in->Lvec_Los);
}

static D kk_calc_quad_distance_edge_edge(const kk_input_t *in)
{
    D quad_dist = lvp_quad_length(&in->cross_KL_Los);
    if (quad_dist > (P_DOUBLE_RES * P_DOUBLE_RES)) {
        D a = lvp_dot(&in->cross_KL_Los, &in->K_Los[0]);
        D b = lvp_dot_f(&in->cross_KL_Los, &in->L_Los[0]);
        return (a - b) * (a - b) / quad_dist;
    }
    return cls_calc_qlen_PK_K_space(&in->K_Los[0], edge_ledge(in->L), in->L);
}

static void lvp_set_interpolate(ivp_vec3r_t *r, const ivp_vec3r_t *p0, const ivp_vec3r_t *p1, D s)
{
    D is = 1.0f - s;
    D a = p0->k[0] * is + p1->k[0] * s;
    D b = p0->k[1] * is + p1->k[1] * s;
    D c = p0->k[2] * is + p1->k[2] * s;
    r->k[0] = a; r->k[1] = b; r->k[2] = c;
}


static bool cls_calc_unscaled_KK_vals(const kk_input_t *in, kk_result_t *result)
{
    const ivp_vec3r_t *norm = &in->cross_KL_Los;
    D quad_dist = lvp_quad_length(norm);
    if (quad_dist > (P_DOUBLE_RES * P_DOUBLE_RES * P_MAX_OBJECT_SIZE * P_MAX_OBJECT_SIZE)) {
        ivp_vec3r_t K_area, L_area;
        lvp_cross(&K_area, &in->Kvec_Los, norm);
        lvp_cross(&L_area, &in->Lvec_Los, norm);
        D a, b, x, ab;
        a = lvp_dot_f(&K_area, &in->L_Los[0]);
        b = lvp_dot_f(&K_area, &in->L_Los[1]);
        x = lvp_dot(&K_area, &in->K_Los[0]);
        ab = a - b;
        result->checks_L[0] = (float)((a - x) * ab);
        result->checks_L[1] = (float)((x - b) * ab);
        a = lvp_dot(&L_area, &in->K_Los[0]);
        b = lvp_dot(&L_area, &in->K_Los[1]);
        x = lvp_dot_f(&L_area, &in->L_Los[0]);
        ab = a - b;
        result->checks_K[0] = (float)((a - x) * ab);
        result->checks_K[1] = (float)((x - b) * ab);
        return true;
    }
    IVP_IF(1) {   /* ivp_compact_ledge_solver.cxx:518 */
        ivp_debug_printf("calc_unscaled_KK_vals: parallel edges\n");
    }
    D min_qdist = P_DOUBLE_MAX;
    static const float check_pos[] = { -1.0f, 0.5f, 2.0f, 0.0f, 1.0f,
                                       -0.001f, 0.001f, 0.999f, 1.001f,
                                       -0.000001f, 0.000001f,
                                       0.999999f, 1.000001f };
    {
        const ivp_compact_ledge_t *ledge_L = in->cache_L->compact_ledge;
        for (int i = 0; i < 11; i++) {
            float pos = check_pos[i];
            ivp_vec3r_t pK_Los;
            lvp_set_interpolate(&pK_Los, &in->K_Los[0], &in->K_Los[1], (D)pos);
            D qdist = cls_quad_dist_edge_to_point_K_space(ledge_L, in->L, &pK_Los);
            if (qdist < min_qdist) {
                min_qdist = qdist;
                result->checks_K[0] = pos;
                result->checks_K[1] = 1.0f - pos;
                s_result_t sr;
                cls_calc_unscaled_s_val_K_space(ledge_L, in->L, &pK_Los, &sr);
                result->checks_L[0] = sr.checks[0];
                result->checks_L[1] = sr.checks[1];
            }
        }
    }
    {
        ivp_vec3r_t L_Kos[2];
        cls_transform_pos_other_space(&in->L_Los[0], in->cache_L, in->cache_K, &L_Kos[0]);
        cls_transform_pos_other_space(&in->L_Los[1], in->cache_L, in->cache_K, &L_Kos[1]);
        const ivp_compact_ledge_t *ledge_K = in->cache_K->compact_ledge;
        for (int i = 0; i < 9; i++) {
            float pos = check_pos[i];
            ivp_vec3r_t pL_Kos;
            lvp_set_interpolate(&pL_Kos, &L_Kos[0], &L_Kos[1], (D)pos);
            D qdist = cls_quad_dist_edge_to_point_K_space(ledge_K, in->K, &pL_Kos);
            if (qdist < min_qdist) {
                min_qdist = qdist;
                result->checks_L[0] = pos;
                result->checks_L[1] = 1.0f - pos;
                s_result_t sr;
                cls_calc_unscaled_s_val_K_space(ledge_K, in->K, &pL_Kos, &sr);
                result->checks_K[0] = sr.checks[0];
                result->checks_K[1] = sr.checks[1];
            }
        }
    }
    return false;
}

static D cls_calc_qlen_KK(const ivp_compact_edge_t *K, const ivp_compact_edge_t *L,
                          ivp_cache_ledge_point_t *c_K, ivp_cache_ledge_point_t *c_L)
{
    kk_input_t kkin;
    kk_input_init(&kkin, K, L, c_K, c_L);
    kk_result_t kkr;
    cls_calc_unscaled_KK_vals(&kkin, &kkr);
    if (!kk_is_outside_L(&kkr)) {
        if (kkr.checks_K[0] <= 0.0f) return cls_calc_qlen_PK_K_space(&kkin.K_Los[0], c_L->compact_ledge, L);
        if (kkr.checks_K[1] <= 0.0f) return cls_calc_qlen_PK_K_space(&kkin.K_Los[1], c_L->compact_ledge, L);
        return kk_calc_quad_distance_edge_edge(&kkin);
    }
    if (!kk_is_outside_K(&kkr)) {
        if (kkr.checks_L[0] <= 0.0f) {
            ivp_vec3r_t L_Kos;
            cls_calc_pos_other_space(L, c_L, c_K, &L_Kos);
            return cls_calc_qlen_PK_K_space(&L_Kos, c_K->compact_ledge, K);
        }
        if (kkr.checks_L[1] <= 0.0f) {
            ivp_vec3r_t L_Kos;
            cls_calc_pos_other_space(ivp_edge_get_next(L), c_L, c_K, &L_Kos);
            return cls_calc_qlen_PK_K_space(&L_Kos, c_K->compact_ledge, K);
        }
        IVP_CORE();   /* ivp_compact_ledge_solver.cxx:749 (falls through) */
    }
    const ivp_compact_edge_t *pK[2] = { K, ivp_edge_get_next(K) };
    const ivp_compact_edge_t *pL[2] = { L, ivp_edge_get_next(L) };
    D min_len = P_DOUBLE_MAX;
    const ivp_compact_edge_t *min_Kp = NULL;
    for (int i = 0; i <= 1; i++) {
        ivp_vec3r_t L_Kos;
        cls_calc_pos_other_space(pL[i], c_L, c_K, &L_Kos);
        D h_len = cls_calc_qlen_PK_K_space(&L_Kos, c_K->compact_ledge, K);
        if (h_len < min_len) {
            min_len = h_len;
            min_Kp = pL[i];
        }
        D h_len_2 = cls_calc_qlen_PK_K_space(&kkin.K_Los[i], c_L->compact_ledge, L);
        if (h_len_2 < min_len) {
            min_len = h_len_2;
            min_Kp = pK[i];
        }
    }
    IVP_ASSERT(min_Kp);   /* ivp_compact_ledge_solver.cxx:783 */
    IVP_USE(min_Kp);
    return min_len;
}

static void lvp_calc_an_orthogonal(ivp_vec3r_t *r, const ivp_vec3r_t *ip)
{
    *r = *ip;
    int max_pos = 0;
    D max = 0.0f;
    for (int i = 2; i >= 0; i--) {
        D h = fabs((double)r->k[i]) > 0 ? (D)fabs((double)r->k[i]) : 0;
        if (h > max) { max = h; max_pos = i; }
    }
    r->k[max_pos] *= -1;
    int other_pos = max_pos - 1;
    if (other_pos < 0) other_pos = 2;
    D h = r->k[max_pos];
    r->k[max_pos] = r->k[other_pos];
    r->k[other_pos] = h;
    ivp_vec3r_t t = *r;
    lvp_cross(r, &t, ip);
}

/* ── IVP_CLS.minimize_on_other_side ─────────────────────────────────────── */

static const ivp_compact_edge_t *cls_minimize_on_other_side(const ivp_compact_edge_t *edge, const ivp_vec3r_t *partner_os)
{
    const ivp_compact_ledge_t *c_ledge = edge_ledge(edge);
    int n_triangles = ivp_ledge_get_n_triangles(c_ledge);
    if (n_triangles <= 0 || n_triangles > IVP_MAX_TRIANGLES_PER_LEDGE) {
        IVP_ASSERT(n_triangles > 0 && n_triangles <= IVP_MAX_TRIANGLES_PER_LEDGE);   /* ivp_mindist_minimize.cxx:725 */
        return edge;
    }
    unsigned char pierce_visited_array[IVP_MAX_TRIANGLES_PER_LEDGE];  /* reference: alloca(n_triangles) */
    memset(pierce_visited_array, 0, (size_t)n_triangles);
    int pierce_idx = ivp_tri_get_pierce_index(edge_tri(edge));
    const ivp_compact_triangle_t *pierced_tri = &ivp_ledge_get_first_triangle(c_ledge)[pierce_idx];
    const ivp_compact_edge_t *F = ivp_tri_get_first_edge(pierced_tri);
    for (;;) {
        pierce_visited_array[ivp_tri_get_tri_index(edge_tri(F))] = 1;
        qr_result_t qr;
        cls_calc_unscaled_qr_vals_F_space(c_ledge, F, partner_os, &qr);
        const ivp_compact_edge_t *e;
        int j;
        int moved = 0;
        for (e = F, j = 0; j < 3; e = ivp_edge_get_next(e), j++) {
            if (qr.checks[j] > 0.0f) continue;
            int tri_idx = ivp_tri_get_tri_index(edge_tri(ivp_edge_get_opposite(e)));
            if (pierce_visited_array[tri_idx]) continue;
            F = ivp_edge_get_opposite(e);
            moved = 1;
            break;
        }
        if (!moved) break;
    }
    return F;
}

/* ── solver state ───────────────────────────────────────────────────────── */

typedef struct mm_loop_hash { intptr_t a, b; } mm_loop_hash_t;

typedef struct mms {
    ivp_mindist_t *mindist;
    int P_Finish_Counter;
#ifdef IVP_DEBUG_BUILD
    D termination_len;   /* ivp_mindist_minimize.hxx:94: used for termination function test */
#endif
    ivp_vec3r_t pos_opposite_BacksideOs;
    mm_loop_hash_t loop_hash[IVP_LOOP_LIST_SIZE];
    int loop_hash_size;
} mms_t;

static void mms_init(mms_t *m, ivp_mindist_t *md)
{
    m->mindist = md;
    lvp_zero(&m->pos_opposite_BacksideOs);
    m->loop_hash_size = 0;
    m->P_Finish_Counter = IVP_MAX_MINIMIZE_BEFORE_HASH_CHECK;
#ifdef IVP_DEBUG_BUILD
    m->termination_len = P_DOUBLE_MAX;
#endif
}

static inline void mms_swap_synapses(mms_t *m) { m->mindist->synapse_sort_flag ^= 1; }

static inline void mms_sort_synapses(mms_t *m, ivp_md_synapse_t *a, ivp_md_synapse_t *b)
{
    (void)b;
    if (a != ivp_mindist_get_sorted_synapse(m->mindist, 0)) mms_swap_synapses(m);
}

/* IVP_LOOP_LIST_SIZE variant of check_loop_hash (the IVP_Hash variant with
 * its IVP_MINDIST_BEHAVIOUR_DEBUG "start loop check" trace,
 * ivp_mindist_minimize.cxx:53, is not compiled by the reference). */
static intptr_t mms_feature_order_key(
    int status, const ivp_compact_edge_t *edge,
    const ivp_compact_ledge_t *ledge)
{
    if (!edge) return status;
    IVP_ASSERT(ledge);
    const ptrdiff_t offset =
        (const char *)edge - (const char *)ledge;
    IVP_ASSERT(offset >= 0);
    IVP_ASSERT((size_t)offset < (size_t)ivp_ledge_get_size(ledge));
    IVP_ASSERT(((uintptr_t)offset & (uintptr_t)3u) == 0u);
    return (intptr_t)offset | status;
}

static bool mms_check_loop_hash(mms_t *m,
                                int s0, const ivp_compact_edge_t *e0,
                                const ivp_compact_ledge_t *ledge0,
                                const ivp_object_t *object0,
                                int s1, const ivp_compact_edge_t *e1,
                                const ivp_compact_ledge_t *ledge1,
                                const ivp_object_t *object1)
{
    IVP_ASSERT(s0 < 4);   /* ivp_mindist_minimize.cxx:94-95: i_s0 / i_s1 */
    IVP_ASSERT(s1 < 4);
    if (m->loop_hash_size >= IVP_LOOP_LIST_SIZE) return true;
    const bool use_reference_order =
        ivp_build_uses_reference_internal_state();
    intptr_t x0 = (intptr_t)e0 | s0;
    intptr_t x1 = (intptr_t)e1 | s1;
    if (!use_reference_order) {
        x0 = mms_feature_order_key(s0, e0, ledge0);
        x1 = mms_feature_order_key(s1, e1, ledge1);
    }
    bool swap;
    if (!use_reference_order) {
        const uint64_t object_key0 =
            ivp_object_order_key(object0);
        const uint64_t object_key1 =
            ivp_object_order_key(object1);
        swap = object_key0 < object_key1;
    } else {
        swap = x0 < x1;
    }
    if (swap) {
        intptr_t h = x0;
        x0 = x1;
        x1 = h;
    }
    for (int i = m->loop_hash_size - 1; i >= 0; i--) {
        if (m->loop_hash[i].a == x0 && m->loop_hash[i].b == x1) return true;
    }
    m->loop_hash[m->loop_hash_size].a = x0;
    m->loop_hash[m->loop_hash_size].b = x1;
    m->loop_hash_size++;
    return false;
}

static inline void syn_update(ivp_cache_ledge_point_t *c, const ivp_compact_edge_t *e, ivp_synapse_polygon_status_t st)
{
    ivp_md_synapse_update(c->tmp.synapse, e, st);
}

/* IVP_Mindist_Minimize_Solver::calc_contact_center_projection: the double
 * core position differences projected on the float plane, summed y, z, x,
 * then stored as float (retail) */
static void set_diff_center(ivp_mindist_t *md, const ivp_cache_obj_t *a, const ivp_cache_obj_t *b)
{
    const ivp_vec3f_t *n = &md->contact_plane;
    md->contact_dot_diff_center =
        (float)(((double)a->core_pos.k[1] - (double)b->core_pos.k[1]) * (double)n->k[1] +
                ((double)a->core_pos.k[2] - (double)b->core_pos.k[2]) * (double)n->k[2] +
                ((double)a->core_pos.k[0] - (double)b->core_pos.k[0]) * (double)n->k[0]);
}

static ivp_mrc_type_t p_minimize_PF(mms_t *m, const ivp_compact_edge_t *P, const ivp_compact_edge_t *F,
                                    ivp_cache_ledge_point_t *c_P, ivp_cache_ledge_point_t *c_F);
static ivp_mrc_type_t p_minimize_Leave_PF(mms_t *m, const ivp_compact_edge_t *P, const ivp_vec3r_t *P_Fos,
                                          const ivp_compact_edge_t *F, ivp_cache_ledge_point_t *c_P,
                                          ivp_cache_ledge_point_t *c_F);
static ivp_mrc_type_t p_minimize_PK(mms_t *m, const ivp_compact_edge_t *P, const ivp_compact_edge_t *K,
                                    ivp_cache_ledge_point_t *c_P, ivp_cache_ledge_point_t *c_K);
static ivp_mrc_type_t p_minimize_Leave_PK(mms_t *m, const ivp_compact_edge_t *P, const ivp_compact_edge_t *K,
                                          ivp_cache_ledge_point_t *c_P, ivp_cache_ledge_point_t *c_K);
static ivp_mrc_type_t p_minimize_PP(mms_t *m, const ivp_compact_edge_t *A, const ivp_compact_edge_t *B,
                                    ivp_cache_ledge_point_t *c_A, ivp_cache_ledge_point_t *c_B);
static ivp_mrc_type_t p_minimize_KK(mms_t *m, const ivp_compact_edge_t *K, const ivp_compact_edge_t *L,
                                    ivp_cache_ledge_point_t *c_K, ivp_cache_ledge_point_t *c_L);
static ivp_mrc_type_t p_minimize_Leave_KK(mms_t *m, const ivp_compact_edge_t *K, const ivp_compact_edge_t *L,
                                          const kk_input_t *kkin, const kk_result_t *kkr,
                                          ivp_cache_ledge_point_t *c_K, ivp_cache_ledge_point_t *c_L);
static ivp_mrc_type_t p_minimize_BP(mms_t *m, ivp_cache_ball_t *c_ball, const ivp_compact_edge_t *P,
                                    ivp_cache_ledge_point_t *c_P);
static ivp_mrc_type_t p_minimize_BK(mms_t *m, ivp_cache_ball_t *c_B, const ivp_compact_edge_t *K,
                                    ivp_cache_ledge_point_t *c_K);
static ivp_mrc_type_t p_minimize_Leave_BK(mms_t *m, ivp_cache_ball_t *c_ball, const ivp_compact_edge_t *K,
                                          ivp_cache_ledge_point_t *c_K);
static ivp_mrc_type_t p_minimize_BF(mms_t *m, ivp_cache_ball_t *c_B, const ivp_compact_edge_t *F,
                                    ivp_cache_ledge_point_t *c_F);

/* ── DEBUG_CHECK_LEN (ivp_mindist_debug.cxx:515-583) ─────────────────────── *
 * Opt-in (IVP_DEBUG_CHECK_LEN = debug build + IVP_MINDIST_BEHAVIOUR_DEBUG):
 * checks that the quad distance of the features the walk visits never grows
 * (CORE) and never falls below real_coll_dist^2 (CORE).  The reference code
 * does not compile (it sits inside "#if 0" and, taken out, defines the
 * functions as IVP_Mindist members although IVP_Mindist_Minimize_Solver
 * declares them, reads a static IVP_Mindist::termination_len, assigns an
 * undeclared "diff" and calls IVP_Mindist::calc_qlen_PF/PK/KK/PP of which
 * only a "#if 0" calc_qlen_PF exists).  Closest compiling equivalent: the
 * solver member termination_len ("#ifdef DEBUG" in the solver class, reset
 * to P_DOUBLE_MAX per solver), "diff" as a local, and the IVP_CLS distance
 * functions evaluated in the second feature's object space:
 *   calc_qlen_PF -> IVP_CLS.calc_qlen_PF_F_space (P in F space; inside the
 *                   triangle the plane distance, else the nearest edge /
 *                   point, as the "#if 0" calc_qlen_PF),
 *   calc_qlen_PK -> IVP_CLS.calc_qlen_PK_K_space (P in K space),
 *   calc_qlen_KK -> IVP_CLS.calc_qlen_KK,
 *   calc_qlen_PP -> IVP_CLS.calc_qlen_PP_P_space (A in B space),
 *   check_len_FF -> p_optimize_FF(F, G, .., P_DOUBLE_MAX) as written. */
#ifdef IVP_DEBUG_CHECK_LEN
static bool mms_validate_termination_len(mms_t *m, D now_len)
{
    D t_len = m->termination_len;

    /* for debugger */
    D real_now_len = LV_SQRT(now_len);
    D real_term_len = LV_SQRT(t_len);
    D diff = real_now_len - real_term_len;
    (void)diff;
    if (now_len - (D)IVP_MIN_TERMINATION_QDLEN > t_len) {   /* @@@ test for '==' */
        IVP_CORE();
    }
    if (now_len - (D)IVP_MIN_TERMINATION_QDLEN_EPS > t_len) {   /* @@@ test for '==' */
        ivp_debug_printf("Termination Len Violation: Epsilon Problem?\n");
        return false;
    }
    const ivp_mindist_settings_t *settings = ivp_env_get_mindist_settings(m->mindist->env);
    if (now_len < (D)(settings->real_coll_dist * settings->real_coll_dist)) {
        IVP_CORE();
        return false;
    }
    m->termination_len = now_len;
    return true;
}

static bool mms_check_len_PP(mms_t *m, const ivp_compact_edge_t *A, const ivp_compact_edge_t *B,
                             ivp_cache_ledge_point_t *c_A, ivp_cache_ledge_point_t *c_B)
{
    ivp_vec3r_t A_Bos;
    cls_calc_pos_other_space(A, c_A, c_B, &A_Bos);
    D now_len = cls_calc_qlen_PP_P_space(c_B->compact_ledge, B, &A_Bos);
    return mms_validate_termination_len(m, now_len);
}

static bool mms_check_len_PF(mms_t *m, const ivp_compact_edge_t *P, const ivp_compact_edge_t *F,
                             ivp_cache_ledge_point_t *c_P, ivp_cache_ledge_point_t *c_F)
{
    /* termination function test */
    ivp_vec3r_t P_Fos;
    cls_calc_pos_other_space(P, c_P, c_F, &P_Fos);
    D now_len = ivp_lcls_calc_qlen_PF_F_space(c_F->compact_ledge, edge_tri(F), &P_Fos);
    return mms_validate_termination_len(m, now_len);
}

static bool mms_check_len_PK(mms_t *m, const ivp_compact_edge_t *P, const ivp_compact_edge_t *K,
                             ivp_cache_ledge_point_t *c_P, ivp_cache_ledge_point_t *c_K)
{
    /* termination function test */
    ivp_vec3r_t P_Kos;
    cls_calc_pos_other_space(P, c_P, c_K, &P_Kos);
    D now_len = cls_calc_qlen_PK_K_space(&P_Kos, c_K->compact_ledge, K);
    return mms_validate_termination_len(m, now_len);
}

static bool mms_check_len_KK(mms_t *m, const ivp_compact_edge_t *K, const ivp_compact_edge_t *L,
                             ivp_cache_ledge_point_t *c_K, ivp_cache_ledge_point_t *c_L)
{
    /* termination function test */
    D now_len = cls_calc_qlen_KK(K, L, c_K, c_L);
    return mms_validate_termination_len(m, now_len);
}

/* declared by the reference, never called */
static IVP_CLS_MAYBE_UNUSED bool mms_check_len_FF(mms_t *m, const ivp_compact_edge_t *F, const ivp_compact_edge_t *G,
                                                  ivp_cache_ledge_point_t *c_F, ivp_cache_ledge_point_t *c_G)
{
    /* termination function test */
    D now_len = ivp_mindist_debug_p_optimize_FF(F, G, c_F, c_G, P_DOUBLE_MAX);
    return mms_validate_termination_len(m, now_len);
}
#endif /* IVP_DEBUG_CHECK_LEN */

/* ── PF ─────────────────────────────────────────────────────────────────── */

static ivp_mrc_type_t p_minimize_PF(mms_t *m, const ivp_compact_edge_t *P, const ivp_compact_edge_t *F,
                                    ivp_cache_ledge_point_t *c_P, ivp_cache_ledge_point_t *c_F)
{
    IVP_ASSERT(c_P->tmp.synapse == ivp_mindist_get_sorted_synapse(m->mindist, 0));   /* ivp_mindist_mcases.cxx:32 */
#ifdef IVP_DEBUG_CHECK_LEN
    (void)mms_check_len_PF(m, P, F, c_P, c_F);   /* ivp_mindist_mcases.cxx:33 */
#endif
    ivp_vec3r_t p_Fos;
    cls_calc_pos_other_space(P, c_P, c_F, &p_Fos);
    qr_result_t qr;
    cls_calc_unscaled_qr_vals_F_space(c_F->compact_ledge, F, &p_Fos, &qr);
    if (!qr_is_outside(&qr)) {
        return p_minimize_Leave_PF(m, P, &p_Fos, F, c_P, c_F);
    }
    const ivp_compact_edge_t *e;
    int j;
    D found_minimal_dist = P_DOUBLE_MAX;
    const ivp_compact_edge_t *min_edge = NULL;
    for (e = F, j = 0; j < 3; e = ivp_edge_get_next(e), j++) {
        D dist = cls_calc_qlen_PK_K_space(&p_Fos, c_F->compact_ledge, e);
        if (dist < found_minimal_dist) { found_minimal_dist = dist; min_edge = e; }
    }
    IVP_ASSERT(min_edge);
    return p_minimize_PK(m, P, min_edge, c_P, c_F);
}

static ivp_mrc_type_t p_minimize_BF(mms_t *m, ivp_cache_ball_t *c_B, const ivp_compact_edge_t *F,
                                    ivp_cache_ledge_point_t *c_F)
{
    ivp_mindist_t *md = m->mindist;
    ivp_vec3r_t p_Fos;
    const ivp_vec3r_t *center_B = &c_B->cache_object->m_world_f_object.pos;
    co_pos_to_object(c_F->clp_cache_object, center_B, &p_Fos);
    qr_result_t qr;
    cls_calc_unscaled_qr_vals_F_space(c_F->compact_ledge, F, &p_Fos, &qr);
    if (!qr_is_outside(&qr)) {
        ivp_vec3f_t F_O_os = clp_f(F, c_F);
        ivp_vec3r_t wHesse_vecF_os;
        cls_calc_hesse_vec_object_not_normized(F, c_F->compact_ledge, &wHesse_vecF_os);
        (void)lvp_normize(&wHesse_vecF_os);
        ivp_vec3r_t contact_plane;
        co_vec_to_world(c_F->clp_cache_object, &wHesse_vecF_os, &contact_plane);
        lvf_set_p(&md->contact_plane, &contact_plane);
        D dist_plus_radius = lvp_dot(&p_Fos, &wHesse_vecF_os) - lvp_dot_f(&wHesse_vecF_os, &F_O_os);
        syn_update(c_F, F, IVP_ST_TRIANGLE);
        if (dist_plus_radius < 0.0f) {
            syn_update(c_F, F, IVP_ST_BACKSIDE);
            m->pos_opposite_BacksideOs = p_Fos;
            return IVP_MRC_BACKSIDE;
        }
        md->len_numerator = (float)(dist_plus_radius - (D)md->sum_extra_radius);
        set_diff_center(md, c_B->cache_object, c_F->clp_cache_object);
        return IVP_MRC_OK;
    }
    const ivp_compact_edge_t *e;
    int j;
    D found_minimal_dist = P_DOUBLE_MAX;
    const ivp_compact_edge_t *min_edge = NULL;
    for (e = F, j = 0; j < 3; e = ivp_edge_get_next(e), j++) {
        D dist = cls_calc_qlen_PK_K_space(&p_Fos, c_F->compact_ledge, e);
        if (dist < found_minimal_dist) { found_minimal_dist = dist; min_edge = e; }
    }
    IVP_ASSERT(min_edge);
    return p_minimize_BK(m, c_B, min_edge, c_F);
}

static ivp_mrc_type_t p_minimize_Leave_PF(mms_t *m, const ivp_compact_edge_t *P, const ivp_vec3r_t *P_Fos,
                                          const ivp_compact_edge_t *F, ivp_cache_ledge_point_t *c_P,
                                          ivp_cache_ledge_point_t *c_F)
{
    ivp_mindist_t *md = m->mindist;
    if (--m->P_Finish_Counter < 0) {
        if (mms_check_loop_hash(m, IVP_ST_POINT, P,
                                c_P->compact_ledge, c_P->clp_object,
                                IVP_ST_TRIANGLE, F,
                                c_F->compact_ledge, c_F->clp_object)) {
            return IVP_MRC_ENDLESS_LOOP;
        }
    }
    ivp_vec3r_t hesse_vec_Pos;
    D back_side_check_distance = 0.0f;
    {
        ivp_vec3r_t wHesse_vecF_Fos;
        cls_calc_hesse_vec_object_not_normized(F, c_F->compact_ledge, &wHesse_vecF_Fos);
        (void)lvp_fast_normize(&wHesse_vecF_Fos);
        ivp_vec3r_t hesse_vec_ws;
        co_vec_to_world(c_F->clp_cache_object, &wHesse_vecF_Fos, &hesse_vec_ws);
        co_vec_to_object(c_P->clp_cache_object, &hesse_vec_ws, &hesse_vec_Pos);
        ivp_vec3f_t F_O_Fos = clp_f(F, c_F);
        md->len_numerator = (float)(lvp_dot(&wHesse_vecF_Fos, P_Fos) - lvp_dot_f(&wHesse_vecF_Fos, &F_O_Fos));
        lvf_set_p(&md->contact_plane, &hesse_vec_ws);
        if ((D)md->len_numerator < back_side_check_distance) {
            lvp_mult(&hesse_vec_Pos, -1.0f);
        }
        md->len_numerator -= md->sum_extra_radius;
        set_diff_center(md, c_P->clp_cache_object, c_F->clp_cache_object);
    }
#ifdef IVP_DEBUG_CHECK_LEN
    (void)mms_check_len_PF(m, P, F, c_P, c_F);   /* ivp_mindist_mcases.cxx:178: check wether world_pos_hash is afflicted */
#endif
    D min_d_pos = 0.0f;
    const ivp_compact_edge_t *min_edge = NULL;
    {
        ivp_vec3f_t we = clp_f(P, c_P);
        const ivp_compact_edge_t *Pm = ivp_edge_get_prev(P);
        const ivp_compact_edge_t *e = ivp_edge_get_prev(ivp_edge_get_opposite(Pm));
        for (;; e = ivp_edge_get_prev(ivp_edge_get_opposite(e))) {
            ivp_vec3f_t wenext = clp_f(e, c_P);
            ivp_vec3r_t vec_Pos;
            lvp_subtract2_ff(&vec_Pos, &wenext, &we);
            D dist = lvp_dot(&hesse_vec_Pos, &vec_Pos);
            if (dist < 0) {
                D e_qlen = lvp_quad_length(&vec_Pos);
                dist *= (D)lv_isqrt_float((float)e_qlen);
                if (dist < min_d_pos) { min_edge = e; min_d_pos = dist; }
            }
            if (e == Pm) break;
        }
    }
    if (min_edge == NULL) {
        syn_update(c_P, P, IVP_ST_POINT);
        syn_update(c_F, F, IVP_ST_TRIANGLE);
        if ((D)(md->len_numerator + md->sum_extra_radius) < back_side_check_distance) {
            m->pos_opposite_BacksideOs = *P_Fos;
            syn_update(c_F, F, IVP_ST_BACKSIDE);
            return IVP_MRC_BACKSIDE;
        }
        return IVP_MRC_OK;
    }
    const ivp_compact_edge_t *P2 = min_edge;
    qr_result_t qr;
    ivp_vec3r_t P2_Fos;
    cls_calc_pos_other_space(P2, c_P, c_F, &P2_Fos);
    cls_calc_unscaled_qr_vals_F_space(edge_ledge(F), F, &P2_Fos, &qr);
    if (!qr_is_outside(&qr)) {
        return p_minimize_Leave_PF(m, P2, &P2_Fos, F, c_P, c_F);
    }
    int violated_q_rs = 0;
    for (int j = 0; j < 3; j++) {
        if (qr.checks[j] >= 0) continue;
        violated_q_rs++;
    }
    if (violated_q_rs == 1) {
        int j;
        const ivp_compact_edge_t *e;
        for (e = F, j = 0; j < 3; e = ivp_edge_get_next(e), j++) {
            if (qr.checks[j] >= 0) continue;
            return p_minimize_KK(m, min_edge, e, c_P, c_F);
        }
    }
    IVP_ASSERT(violated_q_rs == 2);   /* ivp_mindist_mcases.cxx:273 */
    const ivp_compact_edge_t *best_edge = NULL;
    D best_len = P_DOUBLE_MAX;
    {
        int j;
        const ivp_compact_edge_t *e;
        for (e = F, j = 0; j < 3; e = ivp_edge_get_next(e), j++) {
            if (qr.checks[j] > 0) continue;
            D len = cls_calc_qlen_KK(min_edge, e, c_P, c_F);
            if (len < best_len) { best_len = len; best_edge = e; }
        }
    }
    if (!best_edge) best_edge = F;   /* reference asserts violated_q_rs == 2 */
    return p_minimize_KK(m, min_edge, best_edge, c_P, c_F);
}

/* ── KK ─────────────────────────────────────────────────────────────────── */

static ivp_mrc_type_t p_minimize_KK(mms_t *m, const ivp_compact_edge_t *K, const ivp_compact_edge_t *L,
                                    ivp_cache_ledge_point_t *c_K, ivp_cache_ledge_point_t *c_L)
{
    kk_input_t kkin;
    kk_input_init(&kkin, K, L, c_K, c_L);
    kk_result_t kkr;
    cls_calc_unscaled_KK_vals(&kkin, &kkr);
    IVP_ASSERT(c_K->tmp.synapse == ivp_mindist_get_sorted_synapse(m->mindist, 0));   /* ivp_mindist_mcases.cxx:316 */
#ifdef IVP_DEBUG_CHECK_LEN
    (void)mms_check_len_KK(m, K, L, c_K, c_L);   /* ivp_mindist_mcases.cxx:317 */
#endif
    if (!kk_is_outside_L(&kkr)) {
        if (kkr.checks_K[0] < 0.0f) return p_minimize_PK(m, K, L, c_K, c_L);
        if (kkr.checks_K[1] < 0.0f) return p_minimize_PK(m, ivp_edge_get_next(K), L, c_K, c_L);
        return p_minimize_Leave_KK(m, K, L, &kkin, &kkr, c_K, c_L);
    }
    if (!kk_is_outside_K(&kkr)) {
        mms_sort_synapses(m, c_L->tmp.synapse, c_K->tmp.synapse);
        if (kkr.checks_L[0] < 0.0f) return p_minimize_PK(m, L, K, c_L, c_K);
        return p_minimize_PK(m, ivp_edge_get_next(L), K, c_L, c_K);
    }
    const ivp_compact_edge_t *pkp, *plp, *plm, *pkm;
    if (kkr.checks_K[0] < kkr.checks_K[1]) { pkp = K; pkm = ivp_edge_get_next(K); }
    else { pkp = ivp_edge_get_next(K); pkm = K; }
    if (kkr.checks_L[0] < kkr.checks_L[1]) { plp = L; plm = ivp_edge_get_next(L); }
    else { plp = ivp_edge_get_next(L); plm = L; }

    ivp_vec3r_t pkp_Los;
    cls_calc_pos_other_space(pkp, c_K, c_L, &pkp_Los);
    s_result_t sr_kL;
    cls_calc_unscaled_s_val_K_space(c_L->compact_ledge, L, &pkp_Los, &sr_kL);
    if (!s_is_outside(&sr_kL)) return p_minimize_PK(m, pkp, L, c_K, c_L);

    ivp_vec3r_t plp_Kos;
    cls_calc_pos_other_space(plp, c_L, c_K, &plp_Kos);
    s_result_t sr_lK;
    cls_calc_unscaled_s_val_K_space(c_K->compact_ledge, K, &plp_Kos, &sr_lK);
    if (!s_is_outside(&sr_lK)) {
        mms_sort_synapses(m, c_L->tmp.synapse, c_K->tmp.synapse);
        return p_minimize_PK(m, plp, K, c_L, c_K);
    }
    if (sr_kL.checks[0] * kkr.checks_L[0] < 0.0f) {
        return p_minimize_PP(m, pkp, plm, c_K, c_L);
    }
    if (sr_lK.checks[0] * kkr.checks_K[0] < 0.0f) {
        mms_sort_synapses(m, c_L->tmp.synapse, c_K->tmp.synapse);
        return p_minimize_PP(m, plp, pkm, c_L, c_K);
    }
    return p_minimize_PP(m, pkp, plp, c_K, c_L);
}

typedef struct leave_kk_case {
    const ivp_compact_edge_t *F;
    const ivp_compact_edge_t *P;
    const ivp_vec3r_t *P_Fos;
    ivp_cache_ledge_point_t *m_cache_P;
    ivp_cache_ledge_point_t *m_cache_F;
    const ivp_vec3r_t *H_Fos;
    ivp_vec3r_t hesse_Fos;
} leave_kk_case_t;

static ivp_mrc_type_t p_minimize_Leave_KK(mms_t *m, const ivp_compact_edge_t *K, const ivp_compact_edge_t *L,
                                          const kk_input_t *kkin, const kk_result_t *kkr,
                                          ivp_cache_ledge_point_t *c_K, ivp_cache_ledge_point_t *c_L)
{
    ivp_mindist_t *md = m->mindist;
    if (--m->P_Finish_Counter < 0) {
        if (mms_check_loop_hash(m, IVP_ST_EDGE, K,
                                c_K->compact_ledge, c_K->clp_object,
                                IVP_ST_EDGE, L,
                                c_L->compact_ledge, c_L->clp_object)) {
            return IVP_MRC_ENDLESS_LOOP;
        }
    }
    ivp_vec3r_t H_Kos, H_Los, H_ws;
    int side;
    {
        ivp_vec3r_t diff_l_k;
        lvp_subtract2_fp(&diff_l_k, &kkin->L_Los[0], &kkin->K_Los[0]);
        float val = (float)lvp_dot(&diff_l_k, &kkin->cross_KL_Los);
        D HH = lvp_quad_length(&kkin->cross_KL_Los);
        side = signbit(val) ? 1 : 0;
        D iHQ = lv_isqrt(HH, 3);
        H_Los = kkin->cross_KL_Los;
        co_vec_to_world(c_L->clp_cache_object, &H_Los, &H_ws);
        co_vec_to_object(c_K->clp_cache_object, &H_ws, &H_Kos);
        md->len_numerator = (float)fabs((double)((D)val * iHQ));
        md->len_numerator -= md->sum_extra_radius;
        lvf_set_multiple_p(&md->contact_plane, &H_ws, (D)(((float)side - 0.5f) * 2.0f) * iHQ);
        set_diff_center(md, c_K->clp_cache_object, c_L->clp_cache_object);
    }
    bool reverse_side_check[4];
    leave_kk_case_t cases[4];
    ivp_vec3r_t L_Kos[2];
    ivp_vec3r_t K_Los0 = kkin->K_Los[0], K_Los1 = kkin->K_Los[1];
    cls_transform_pos_other_space(&kkin->L_Los[0], c_L, c_K, &L_Kos[0]);
    cls_transform_pos_other_space(&kkin->L_Los[1], c_L, c_K, &L_Kos[1]);
    cases[0].F = ivp_edge_get_opposite(K); cases[0].P = kkin->L; cases[0].P_Fos = &L_Kos[0];
    cases[0].m_cache_F = c_K; cases[0].m_cache_P = c_L; cases[0].H_Fos = &H_Kos;
    cases[1].F = K; cases[1].P = ivp_edge_get_opposite(kkin->L); cases[1].P_Fos = &L_Kos[1];
    cases[1].m_cache_F = c_K; cases[1].m_cache_P = c_L; cases[1].H_Fos = &H_Kos;
    cases[2].F = ivp_edge_get_opposite(L); cases[2].P = kkin->K; cases[2].P_Fos = &K_Los0;
    cases[2].m_cache_F = c_L; cases[2].m_cache_P = c_K; cases[2].H_Fos = &H_Los;
    cases[3].F = L; cases[3].P = ivp_edge_get_opposite(kkin->K); cases[3].P_Fos = &K_Los1;
    cases[3].m_cache_F = c_L; cases[3].m_cache_P = c_K; cases[3].H_Fos = &H_Los;
    for (int i = 0; i <= 3; i++) {
        leave_kk_case_t *c = &cases[i];
        cls_calc_hesse_vec_object_not_normized(c->F, edge_ledge(c->F), &c->hesse_Fos);
        float side_check_dist = (float)lvp_dot(c->H_Fos, &c->hesse_Fos);
        int reverse = signbit(side_check_dist) ? 1 : 0;
        reverse ^= (i >> 1) ^ side;
        reverse_side_check[i] = reverse != 0;
    }
    D min_grad_pos = -P_DOUBLE_RES * 4;
    const ivp_compact_edge_t *min_edge = NULL;
    const ivp_compact_edge_t *min_plane = NULL;
    ivp_cache_ledge_point_t *min_edge_m_cache = NULL;
    ivp_cache_ledge_point_t *min_plane_m_cache = NULL;
    qr_result_t min_qr;
    memset(&min_qr, 0, sizeof(min_qr));
    for (int i = 0; i <= 3; i++) {
        leave_kk_case_t *cF = &cases[i];
        leave_kk_case_t *ce = &cases[(i ^ side) ^ (int)reverse_side_check[i]];
        D delta_h2;
        ivp_vec3r_t e_dir_Fos;
        {
            leave_kk_case_t *ce2 = &cases[i ^ side];
            leave_kk_case_t *ce2_next = &cases[1 ^ (i ^ side)];
            lvp_subtract2(&e_dir_Fos, ce2->P_Fos, ce2_next->P_Fos);
            delta_h2 = lvp_dot(&e_dir_Fos, &cF->hesse_Fos);
        }
        if (delta_h2 >= 0.0f) continue;
        D inv_hesse_real_length = (D)lv_isqrt_float((float)lvp_quad_length(&cF->hesse_Fos));
        D grad = delta_h2 * (D)lv_isqrt_float((float)lvp_quad_length(&e_dir_Fos));
        grad *= inv_hesse_real_length;
        if (grad < min_grad_pos) {
            qr_result_t qr;
            cls_calc_unscaled_qr_vals_F_space(cF->m_cache_F->compact_ledge, cF->F, ce->P_Fos, &qr);
            if (qr.checks[0] > 0.0f) {
                min_grad_pos = grad;
                min_edge = ce->P;
                min_edge_m_cache = ce->m_cache_P;
                min_plane = cF->F;
                min_plane_m_cache = cF->m_cache_F;
                min_qr = qr;
            } else {
                IVP_IF(1) {   /* ivp_mindist_mcases.cxx:565 */
                    ivp_debug_printf("negative r value in KK: %g\n", (double)qr.checks[0]);
                }
            }
        }
    }
    if (min_edge == NULL) {
        syn_update(c_K, K, IVP_ST_EDGE);
        syn_update(c_L, L, IVP_ST_EDGE);
        if ((int)reverse_side_check[0] + (int)reverse_side_check[1] == 2) {
            syn_update(c_K, K, IVP_ST_BACKSIDE);
            syn_update(c_L, L, IVP_ST_TRIANGLE);
            D sl = (D)(kkr->checks_L[0] / (kkr->checks_L[0] + kkr->checks_L[1])); /* IVP_FLOAT quotient */
            lvp_set_interpolate(&m->pos_opposite_BacksideOs, &L_Kos[0], &L_Kos[1], sl);
            return IVP_MRC_BACKSIDE;
        }
        if ((int)reverse_side_check[2] + (int)reverse_side_check[3] == 2) {
            syn_update(c_K, K, IVP_ST_TRIANGLE);
            D sk = (D)(kkr->checks_K[0] / (kkr->checks_K[0] + kkr->checks_K[1])); /* IVP_FLOAT quotient */
            lvp_set_interpolate(&m->pos_opposite_BacksideOs, &K_Los0, &K_Los1, sk);
            syn_update(c_L, L, IVP_ST_BACKSIDE);
            return IVP_MRC_BACKSIDE;
        }
        return IVP_MRC_OK;
    }
    mms_sort_synapses(m, min_edge_m_cache->tmp.synapse, min_plane_m_cache->tmp.synapse);
    if (!qr_is_outside(&min_qr)) {
        ivp_vec3r_t edge_Fos;
        cls_calc_pos_other_space(min_edge, min_edge_m_cache, min_plane_m_cache, &edge_Fos);
        return p_minimize_Leave_PF(m, min_edge, &edge_Fos, min_plane, min_edge_m_cache, min_plane_m_cache);
    }
    if (min_qr.checks[2] >= 0.0f) {
        return p_minimize_KK(m, min_edge, ivp_edge_get_next(min_plane), min_edge_m_cache, min_plane_m_cache);
    }
    if (min_qr.checks[1] >= 0.0f) {
        return p_minimize_KK(m, min_edge, ivp_edge_get_prev(min_plane), min_edge_m_cache, min_plane_m_cache);
    }
    D dist_next = cls_calc_qlen_KK(min_edge, ivp_edge_get_next(min_plane), min_edge_m_cache, min_plane_m_cache);
    D dist_prev = cls_calc_qlen_KK(min_edge, ivp_edge_get_opposite(ivp_edge_get_prev(min_plane)), min_edge_m_cache,
                                   min_plane_m_cache);
#ifdef IVP_MINDIST_BEHAVIOUR_DEBUG
    {   /* ivp_mindist_mcases.cxx:629 (the reference calls the undeclared
         * member calc_qlen_KK: IVP_CLS.calc_qlen_KK is meant) */
        D dist_plane = cls_calc_qlen_KK(min_edge, min_plane, min_edge_m_cache, min_plane_m_cache);
        if ((dist_plane < dist_next) && (dist_plane < dist_prev)) {
            IVP_CORE();
            /* How could this have happened?  No improvement seems possible,
             * though grad says so...  End KL */
            syn_update(c_K, K, IVP_ST_EDGE);
            syn_update(c_L, L, IVP_ST_EDGE);
            return IVP_MRC_OK;
        }
    }
#endif
    if (dist_next > dist_prev) {
        return p_minimize_KK(m, min_edge, ivp_edge_get_prev(min_plane), min_edge_m_cache, min_plane_m_cache);
    }
    return p_minimize_KK(m, min_edge, ivp_edge_get_next(min_plane), min_edge_m_cache, min_plane_m_cache);
}

/* ── BP ─────────────────────────────────────────────────────────────────── */

static ivp_mrc_type_t p_minimize_BP(mms_t *m, ivp_cache_ball_t *c_ball, const ivp_compact_edge_t *P,
                                    ivp_cache_ledge_point_t *c_P)
{
    ivp_mindist_t *md = m->mindist;
    if (--m->P_Finish_Counter < 0) {
        if (mms_check_loop_hash(m, IVP_ST_BALL, NULL, NULL,
                                c_ball->object,
                                IVP_ST_POINT, P, c_P->compact_ledge,
                                c_P->clp_object)) {
            return IVP_MRC_ENDLESS_LOOP;
        }
    }
    ivp_vec3r_t center_ball_Pmos, p_ws;
    cls_give_world_coords_AT(P, c_P, &p_ws);
    const ivp_vec3r_t *center_b_ws = &c_ball->cache_object->m_world_f_object.pos;
    co_pos_to_object(c_P->clp_cache_object, center_b_ws, &center_ball_Pmos);
    ivp_vec3r_t contact_plane;
    lvp_subtract2(&contact_plane, center_b_ws, &p_ws);
    D dist_plus_radius = lvp_real_length_plus_normize(&contact_plane);
    lvf_set_p(&md->contact_plane, &contact_plane);
    md->len_numerator = (float)(dist_plus_radius - (D)md->sum_extra_radius);
    set_diff_center(md, c_ball->cache_object, c_P->clp_cache_object);

    D s_grad_max = 0;
    const ivp_compact_edge_t *Kmax = NULL;
    {
        const ivp_compact_edge_t *Pm = P;
        ivp_vec3f_t Pm_Pmos = clp_f(Pm, c_P);
        ivp_vec3r_t Pm_P_Pmos;
        lvp_subtract2_pf(&Pm_P_Pmos, &center_ball_Pmos, &Pm_Pmos);
        D dist = lvp_dot_f(&Pm_P_Pmos, &Pm_Pmos);
        Pm = ivp_edge_get_prev(Pm);
        const ivp_compact_edge_t *e = ivp_edge_get_prev(ivp_edge_get_opposite(Pm));
        for (;; e = ivp_edge_get_prev(ivp_edge_get_opposite(e))) {
            ivp_vec3f_t tp_next_os = clp_f(e, c_P);
            D grad = lvp_dot_f(&Pm_P_Pmos, &tp_next_os) - dist;
            if (grad > 0) {
                D i_len = (D)lv_isqrt_float((float)lvf_quad_distance(&tp_next_os, &Pm_Pmos));
                grad *= i_len;
                if (grad > s_grad_max) { s_grad_max = grad; Kmax = e; }
            }
            if (e == Pm) break;
        }
    }
    if (Kmax == NULL) {
        syn_update(c_P, P, IVP_ST_POINT);
        return IVP_MRC_OK;
    }
    s_result_t sr;
    cls_calc_unscaled_s_val_K_space(c_P->compact_ledge, Kmax, &center_ball_Pmos, &sr);
    if (sr.checks[1] <= 0) {
        IVP_IF(1) {   /* ivp_mindist_mcases.cxx:741 */
            ivp_debug_printf("BP epsilon problem\n");
        }
        syn_update(c_P, P, IVP_ST_POINT);
        return IVP_MRC_OK;
    }
    if (sr.checks[0] < 0) return p_minimize_BP(m, c_ball, Kmax, c_P);
    return p_minimize_BK(m, c_ball, Kmax, c_P);
}

/* ── PP ─────────────────────────────────────────────────────────────────── */

static ivp_mrc_type_t p_minimize_PP(mms_t *m, const ivp_compact_edge_t *A, const ivp_compact_edge_t *B,
                                    ivp_cache_ledge_point_t *c_A, ivp_cache_ledge_point_t *c_B)
{
    ivp_mindist_t *md = m->mindist;
    if (--m->P_Finish_Counter < 0) {
        if (mms_check_loop_hash(m, IVP_ST_POINT, A,
                                c_A->compact_ledge, c_A->clp_object,
                                IVP_ST_POINT, B,
                                c_B->compact_ledge, c_B->clp_object)) {
            return IVP_MRC_ENDLESS_LOOP;
        }
    }
    IVP_ASSERT(c_A->tmp.synapse == ivp_mindist_get_sorted_synapse(m->mindist, 0));   /* ivp_mindist_mcases.cxx:774 */
#ifdef IVP_DEBUG_CHECK_LEN
    (void)mms_check_len_PP(m, A, B, c_A, c_B);   /* ivp_mindist_mcases.cxx:775 */
#endif
    const ivp_compact_edge_t *Point[2] = { A, B };
    ivp_vec3r_t points_ws[2];
    cls_give_world_coords_AT(A, c_A, &points_ws[0]);
    cls_give_world_coords_AT(B, c_B, &points_ws[1]);
    ivp_vec3r_t point_other_os[2];
    co_pos_to_object(c_B->clp_cache_object, &points_ws[0], &point_other_os[0]);
    co_pos_to_object(c_A->clp_cache_object, &points_ws[1], &point_other_os[1]);
    {
        D qlen = lvp_quad_distance(&points_ws[0], &points_ws[1]);
        if (qlen > P_DOUBLE_RES) {
            D inv_len = lv_isqrt(qlen, 3);
            md->len_numerator = (float)(qlen * inv_len - (D)md->sum_extra_radius);
            /* IVP_U_Float_Point::inline_subtract_and_mult(IVP_U_Point, IVP_U_Point, f) */
            md->contact_plane.k[0] = (float)((points_ws[0].k[0] - points_ws[1].k[0]) * inv_len);
            md->contact_plane.k[1] = (float)((points_ws[0].k[1] - points_ws[1].k[1]) * inv_len);
            md->contact_plane.k[2] = (float)((points_ws[0].k[2] - points_ws[1].k[2]) * inv_len);
        } else {
            return IVP_MRC_ENDLESS_LOOP;
        }
        set_diff_center(md, c_A->clp_cache_object, c_B->clp_cache_object);
    }
    ivp_cache_ledge_point_t *m_cache[2] = { c_A, c_B };
    D s_grad_max = 0.0f;
    const ivp_vec3r_t *P_Pmos_max = NULL;
    const ivp_compact_edge_t *Kmax = NULL;
    const ivp_compact_edge_t *Pmax = NULL;
    ivp_cache_ledge_point_t *m_cache_K_max = NULL;
    ivp_cache_ledge_point_t *m_cache_P_max = NULL;
    for (int ip = 1; ip >= 0; ip--) {
        ivp_cache_ledge_point_t *m_cache_Pm = m_cache[1 - ip];
        const ivp_vec3r_t *P_Pmos = &point_other_os[ip];
        const ivp_compact_edge_t *Pm = Point[1 - ip];
        ivp_vec3f_t Pm_Pmos = clp_f(Pm, m_cache_Pm);
        ivp_vec3r_t Pm_P_Pmos;
        lvp_subtract2_pf(&Pm_P_Pmos, P_Pmos, &Pm_Pmos);
        D dist = lvp_dot_f(&Pm_P_Pmos, &Pm_Pmos);
        Pm = ivp_edge_get_prev(Pm);
        const ivp_compact_edge_t *e = ivp_edge_get_prev(ivp_edge_get_opposite(Pm));
        for (;; e = ivp_edge_get_prev(ivp_edge_get_opposite(e))) {
            ivp_vec3f_t tp_next_os = clp_f(e, m_cache_Pm);
            D grad = lvp_dot_f(&Pm_P_Pmos, &tp_next_os) - dist;
            if (grad > 0) {
                D i_len = (D)lv_isqrt_float((float)lvf_quad_distance(&tp_next_os, &Pm_Pmos));
                grad *= i_len;
                if (grad > s_grad_max) {
                    P_Pmos_max = P_Pmos;
                    s_grad_max = grad;
                    Kmax = e;
                    Pmax = Point[ip];
                    m_cache_K_max = m_cache_Pm;
                    m_cache_P_max = m_cache[ip];
                }
            }
            if (e == Pm) break;
        }
    }
    if (Kmax == NULL) {
    end_PP:
        syn_update(c_A, A, IVP_ST_POINT);
        syn_update(c_B, B, IVP_ST_POINT);
        return IVP_MRC_OK;
    }
    s_result_t sr;
    cls_calc_unscaled_s_val_K_space(m_cache_K_max->compact_ledge, Kmax, P_Pmos_max, &sr);
    if (sr.checks[1] <= 0) {
        IVP_IF(1) {   /* ivp_mindist_mcases.cxx:879 */
            ivp_debug_printf("PP epsilon problem\n");
        }
        goto end_PP;
    }
    if (sr.checks[0] < 0.0f) {
        mms_sort_synapses(m, m_cache_K_max->tmp.synapse, m_cache_P_max->tmp.synapse);
        return p_minimize_PP(m, Kmax, Pmax, m_cache_K_max, m_cache_P_max);
    }
    mms_sort_synapses(m, m_cache_P_max->tmp.synapse, m_cache_K_max->tmp.synapse);
    return p_minimize_Leave_PK(m, Pmax, Kmax, m_cache_P_max, m_cache_K_max);
}

/* ── BK ─────────────────────────────────────────────────────────────────── */

static ivp_mrc_type_t p_minimize_BK(mms_t *m, ivp_cache_ball_t *c_B, const ivp_compact_edge_t *K,
                                    ivp_cache_ledge_point_t *c_K)
{
    ivp_vec3r_t P_Kos;
    co_pos_to_object(c_K->clp_cache_object, &c_B->cache_object->m_world_f_object.pos, &P_Kos);
    s_result_t sr;
    cls_calc_unscaled_s_val_K_space(c_K->compact_ledge, K, &P_Kos, &sr);
    if (sr.checks[0] < 0.0f) return p_minimize_BP(m, c_B, K, c_K);
    if (sr.checks[1] < 0.0f) return p_minimize_BP(m, c_B, ivp_edge_get_next(K), c_K);
    return p_minimize_Leave_BK(m, c_B, K, c_K);
}

static ivp_mrc_type_t p_minimize_Leave_BK(mms_t *m, ivp_cache_ball_t *c_ball, const ivp_compact_edge_t *K,
                                          ivp_cache_ledge_point_t *c_K)
{
    ivp_mindist_t *md = m->mindist;
    if (--m->P_Finish_Counter < 0) {
        if (mms_check_loop_hash(m, IVP_ST_BALL, NULL, NULL,
                                c_ball->object,
                                IVP_ST_EDGE, K, c_K->compact_ledge,
                                c_K->clp_object)) {
            return IVP_MRC_ENDLESS_LOOP;
        }
    }
    ivp_vec3r_t p_ks;
    co_pos_to_object(c_K->clp_cache_object, &c_ball->cache_object->m_world_f_object.pos, &p_ks);
    ivp_vec3f_t k_start_os = clp_f(K, c_K);
    ivp_vec3f_t k_next_os = clp_f(ivp_edge_get_next(K), c_K);
    ivp_vec3f_t k_tri_os = clp_f(ivp_edge_get_prev(K), c_K);
    ivp_vec3f_t k_oppo_tri_os = clp_f(ivp_edge_get_prev(ivp_edge_get_opposite(K)), c_K);
    ivp_vec3r_t vec_K_ks, vec_K_P, vec_K_tri, vec_K_oppo_tri;
    lvp_subtract2_ff(&vec_K_ks, &k_next_os, &k_start_os);
    lvp_subtract2_pf(&vec_K_P, &p_ks, &k_start_os);
    lvp_subtract2_ff(&vec_K_tri, &k_tri_os, &k_start_os);
    lvp_subtract2_ff(&vec_K_oppo_tri, &k_oppo_tri_os, &k_start_os);
    ivp_vec3r_t hesse_tri_os, hesse_oppo_tri_os;
    lvp_cross(&hesse_tri_os, &vec_K_ks, &vec_K_tri);
    lvp_cross(&hesse_oppo_tri_os, &vec_K_oppo_tri, &vec_K_ks);
    D vert[2];
    vert[0] = lvp_dot(&vec_K_P, &hesse_tri_os);
    vert[1] = lvp_dot(&vec_K_P, &hesse_oppo_tri_os);
    qr_result_t qr0, qr1;
    cls_calc_unscaled_qr_vals_F_space(c_K->compact_ledge, K, &p_ks, &qr0);
    cls_calc_unscaled_qr_vals_F_space(c_K->compact_ledge, ivp_edge_get_opposite(K), &p_ks, &qr1);
    if (qr0.checks[0] > 0.0f) {
        if (qr1.checks[0] > 0.0f) {
            if (vert[1] > 0) return p_minimize_BF(m, c_ball, ivp_edge_get_opposite(K), c_K);
        }
        return p_minimize_BF(m, c_ball, K, c_K);
    }
    if (qr1.checks[0] > 0.0f) return p_minimize_BF(m, c_ball, ivp_edge_get_opposite(K), c_K);
    if (vert[0] < -P_DOUBLE_EPS && vert[1] < -P_DOUBLE_EPS) {
        m->pos_opposite_BacksideOs = p_ks;
        syn_update(c_K, K, IVP_ST_BACKSIDE);
        return IVP_MRC_BACKSIDE;
    }
    ivp_vec3r_t Lot2;
    lvp_cross(&Lot2, &vec_K_ks, &vec_K_P);
    D K_ql = lvp_quad_length(&vec_K_ks);
    if (K_ql < P_DOUBLE_EPS) return IVP_MRC_BACKSIDE;
    D iqK_len = 1.0f / K_ql;
    D qdist = lvp_quad_length(&Lot2) * iqK_len;
    D inv_len = lv_isqrt(qdist, 3);
    md->len_numerator = (float)(qdist * inv_len - (D)md->sum_extra_radius);
    { ivp_vec3r_t t = Lot2; lvp_cross(&Lot2, &vec_K_ks, &t); }
    { ivp_vec3r_t t = Lot2; co_vec_to_world(c_K->clp_cache_object, &t, &Lot2); }
    lvf_set_multiple_p(&md->contact_plane, &Lot2, -inv_len * iqK_len);
    set_diff_center(md, c_ball->cache_object, c_K->clp_cache_object);
    syn_update(c_K, K, IVP_ST_EDGE);
    return IVP_MRC_OK;
}

/* ── PK ─────────────────────────────────────────────────────────────────── */

static ivp_mrc_type_t p_minimize_PK(mms_t *m, const ivp_compact_edge_t *P, const ivp_compact_edge_t *K,
                                    ivp_cache_ledge_point_t *c_P, ivp_cache_ledge_point_t *c_K)
{
    IVP_ASSERT(c_P->tmp.synapse == ivp_mindist_get_sorted_synapse(m->mindist, 0));   /* ivp_mindist_mcases.cxx:1038 */
#ifdef IVP_DEBUG_CHECK_LEN
    (void)mms_check_len_PK(m, P, K, c_P, c_K);   /* ivp_mindist_mcases.cxx:1039 */
#endif
    ivp_vec3r_t P_Kos;
    cls_calc_pos_other_space(P, c_P, c_K, &P_Kos);
    s_result_t sr;
    cls_calc_unscaled_s_val_K_space(c_K->compact_ledge, K, &P_Kos, &sr);
    if (sr.checks[0] < 0.0f) return p_minimize_PP(m, P, K, c_P, c_K);
    if (sr.checks[1] < 0.0f) return p_minimize_PP(m, P, ivp_edge_get_next(K), c_P, c_K);
    return p_minimize_Leave_PK(m, P, K, c_P, c_K);
}

static ivp_mrc_type_t p_minimize_Leave_PK(mms_t *m, const ivp_compact_edge_t *P, const ivp_compact_edge_t *K,
                                          ivp_cache_ledge_point_t *c_P, ivp_cache_ledge_point_t *c_K)
{
    ivp_mindist_t *md = m->mindist;
    if (--m->P_Finish_Counter < 0) {
        if (mms_check_loop_hash(m, IVP_ST_POINT, P,
                                c_P->compact_ledge, c_P->clp_object,
                                IVP_ST_EDGE, K,
                                c_K->compact_ledge, c_K->clp_object)) {
            return IVP_MRC_ENDLESS_LOOP;
        }
    }
#ifdef IVP_DEBUG_CHECK_LEN
    (void)mms_check_len_PK(m, P, K, c_P, c_K);   /* ivp_mindist_mcases.cxx:1070 */
#endif
    ivp_vec3r_t p_ks;
    cls_calc_pos_other_space(P, c_P, c_K, &p_ks);
    ivp_vec3f_t k_start_os = clp_f(K, c_K);
    ivp_vec3f_t k_next_os = clp_f(ivp_edge_get_next(K), c_K);
    ivp_vec3f_t k_tri_os = clp_f(ivp_edge_get_prev(K), c_K);
    ivp_vec3f_t k_oppo_tri_os = clp_f(ivp_edge_get_prev(ivp_edge_get_opposite(K)), c_K);
    ivp_vec3r_t vec_K_ks, vec_K_P, vec_K_tri, vec_K_oppo_tri;
    lvp_subtract2_ff(&vec_K_ks, &k_next_os, &k_start_os);
    lvp_subtract2_pf(&vec_K_P, &p_ks, &k_start_os);
    lvp_subtract2_ff(&vec_K_tri, &k_tri_os, &k_start_os);
    lvp_subtract2_ff(&vec_K_oppo_tri, &k_oppo_tri_os, &k_start_os);
    ivp_vec3r_t hesse_tri_os, hesse_oppo_tri_os;
    lvp_cross(&hesse_tri_os, &vec_K_ks, &vec_K_tri);
    lvp_cross(&hesse_oppo_tri_os, &vec_K_oppo_tri, &vec_K_ks);
    D vert[2];
    vert[0] = lvp_dot(&vec_K_P, &hesse_tri_os);
    vert[1] = lvp_dot(&vec_K_P, &hesse_oppo_tri_os);
    qr_result_t qr0, qr1;
    cls_calc_unscaled_qr_vals_F_space(c_K->compact_ledge, K, &p_ks, &qr0);
    cls_calc_unscaled_qr_vals_F_space(c_K->compact_ledge, ivp_edge_get_opposite(K), &p_ks, &qr1);
    if (qr0.checks[0] > 0.0f) {
        if (qr1.checks[0] > 0.0f) {
            if (vert[1] > 0) return p_minimize_PF(m, P, ivp_edge_get_opposite(K), c_P, c_K);
        }
        return p_minimize_PF(m, P, K, c_P, c_K);
    }
    if (qr1.checks[0] > 0.0f) return p_minimize_PF(m, P, ivp_edge_get_opposite(K), c_P, c_K);
    if (vert[0] < 0 && vert[1] < 0) {
        syn_update(c_P, P, IVP_ST_POINT);
        syn_update(c_K, K, IVP_ST_BACKSIDE);
        m->pos_opposite_BacksideOs = p_ks;
        return IVP_MRC_BACKSIDE;
    }
    ivp_vec3r_t Lot2;
    D qlen;
    {
        lvp_cross(&Lot2, &vec_K_ks, &vec_K_P);
        D K_ql2 = lvp_quad_length(&vec_K_ks);
        if (K_ql2 < P_DOUBLE_EPS) return IVP_MRC_BACKSIDE;
        D iqK_len = 1.0f / K_ql2;
        qlen = lvp_quad_length(&Lot2) * iqK_len;
        if (qlen > P_DOUBLE_EPS) {
            D inv_len = lv_isqrt(qlen, 3);
            md->len_numerator = (float)(qlen * inv_len - (D)md->sum_extra_radius);
            { ivp_vec3r_t t = Lot2; lvp_cross(&Lot2, &vec_K_ks, &t); }
            { ivp_vec3r_t t = Lot2; co_vec_to_world(c_K->clp_cache_object, &t, &Lot2); }
            lvf_set_multiple_p(&md->contact_plane, &Lot2, -inv_len * iqK_len);
        } else {
            md->len_numerator = -md->sum_extra_radius;
            ivp_vec3r_t orth;
            lvp_calc_an_orthogonal(&orth, &vec_K_ks);
            (void)lvp_normize(&orth);
            lvf_set_p(&md->contact_plane, &orth);
        }
        set_diff_center(md, c_P->clp_cache_object, c_K->clp_cache_object);
    }
    { ivp_vec3r_t t = Lot2; co_vec_to_object(c_P->clp_cache_object, &t, &Lot2); }
    const ivp_compact_edge_t *NKmax = NULL;
    D max_s_val = P_DOUBLE_RES * qlen;
    ivp_vec3f_t p_os = clp_f(P, c_P);
    const ivp_compact_edge_t *Pm = ivp_edge_get_prev(P);
    const ivp_compact_edge_t *e = ivp_edge_get_prev(ivp_edge_get_opposite(Pm));
    for (;; e = ivp_edge_get_prev(ivp_edge_get_opposite(e))) {
        ivp_vec3r_t e_vec_Pos;
        ivp_vec3f_t ep = clp_f(e, c_P);
        lvp_subtract2_ff(&e_vec_Pos, &ep, &p_os);
        D angle = lvp_dot(&e_vec_Pos, &Lot2);
        if (angle > 0) {
            angle *= (D)lv_isqrt_float((float)lvp_quad_length(&e_vec_Pos));
            if (angle > max_s_val) { max_s_val = angle; NKmax = e; }
        }
        if (e == Pm) break;
    }
    while (NKmax != NULL) {
        if (max_s_val < P_RES_EPS) {
            kk_input_t kkin;
            kk_input_init(&kkin, K, NKmax, c_K, c_P);
            kk_result_t kkr;
            bool check = cls_calc_unscaled_KK_vals(&kkin, &kkr);
            if (!check || kkr.checks_L[0] < 0.0f) {
                IVP_IF(0) {   /* ivp_mindist_mcases.cxx:1221 */
                    ivp_debug_printf("PK_KK epsilon problem\n");
                }
                break;
            }
        }
        return p_minimize_KK(m, NKmax, K, c_P, c_K);
    }
    syn_update(c_P, P, IVP_ST_POINT);
    syn_update(c_K, K, IVP_ST_EDGE);
    return IVP_MRC_OK;
}

/* ── FF (initial / degenerate) ──────────────────────────────────────────── */

static ivp_mrc_type_t p_minimize_FF(mms_t *m, const ivp_compact_edge_t *A, const ivp_compact_edge_t *B,
                                    ivp_cache_ledge_point_t *c_A, ivp_cache_ledge_point_t *c_B)
{
    D min_qdist = P_DOUBLE_MAX;
    {
        const ivp_compact_edge_t *pA, *pB;
        int a, b;
        for (pA = A, a = 0; a < 3; pA = ivp_edge_get_next(pA), a++) {
            for (pB = B, b = 0; b < 3; pB = ivp_edge_get_next(pB), b++) {
                ivp_vec3r_t pos_B_ws, pos_A_ws;
                cls_give_world_coords_AT(pB, c_B, &pos_B_ws);
                cls_give_world_coords_AT(pA, c_A, &pos_A_ws);
                D qdist = lvp_quad_distance(&pos_A_ws, &pos_B_ws);
                if (qdist < min_qdist) {
                    syn_update(c_A, pA, IVP_ST_POINT);
                    syn_update(c_B, pB, IVP_ST_POINT);
                    min_qdist = qdist;
                }
            }
        }
    }
    ivp_cache_ledge_point_t *cc_A[2] = { c_A, c_B };
    ivp_cache_ledge_point_t *cc_B[2] = { c_B, c_A };
    const ivp_compact_edge_t *tabA[2] = { A, B };
    const ivp_compact_edge_t *tabB[2] = { B, A };
    for (int i = 0; i < 2; i++) {
        const ivp_compact_edge_t *pA;
        const ivp_compact_edge_t *pB = tabB[i];
        int a;
        lv_hesse_t hesse_B;
        cls_calc_hesse_object(pB, cc_B[i]->compact_ledge, &hesse_B);
        for (pA = tabA[i], a = 0; a < 3; pA = ivp_edge_get_next(pA), a++) {
            ivp_vec3r_t pA_Bos;
            cls_calc_pos_other_space(pA, cc_A[i], cc_B[i], &pA_Bos);
            qr_result_t qr;
            cls_calc_unscaled_qr_vals_F_space(cc_B[i]->compact_ledge, pB, &pA_Bos, &qr);
            if (!qr_is_outside(&qr)) {
                D qdist = lv_hesse_get_dist(&hesse_B, &pA_Bos);
                qdist *= qdist;
                if (qdist * (1 + P_DOUBLE_RES) < min_qdist) {
                    syn_update(cc_A[i], pA, IVP_ST_POINT);
                    syn_update(cc_B[i], pB, IVP_ST_TRIANGLE);
                    min_qdist = qdist;
                }
            }
        }
    }
    for (int i = 0; i < 2; i++) {
        const ivp_compact_edge_t *pA, *pB;
        int a, b;
        for (pB = tabB[i], b = 0; b < 3; pB = ivp_edge_get_next(pB), b++) {
            ivp_vec3r_t pB_Aos;
            cls_calc_pos_other_space(pB, cc_B[i], cc_A[i], &pB_Aos);
            for (pA = tabA[i], a = 0; a < 3; pA = ivp_edge_get_next(pA), a++) {
                s_result_t sr;
                cls_calc_unscaled_s_val_K_space(cc_A[i]->compact_ledge, pA, &pB_Aos, &sr);
                if (s_is_outside(&sr)) continue;
                D qdist = cls_quad_dist_edge_to_point_K_space(cc_A[i]->compact_ledge, pA, &pB_Aos);
                if (qdist * (1 + P_DOUBLE_RES) < min_qdist) {
                    syn_update(cc_A[i], pA, IVP_ST_EDGE);
                    syn_update(cc_B[i], pB, IVP_ST_POINT);
                    min_qdist = qdist;
                }
            }
        }
    }
    {
        const ivp_compact_edge_t *pA, *pB;
        int a, b;
        for (pA = A, a = 0; a < 3; pA = ivp_edge_get_next(pA), a++) {
            for (pB = B, b = 0; b < 3; pB = ivp_edge_get_next(pB), b++) {
                kk_input_t kkin;
                kk_input_init(&kkin, pA, pB, c_A, c_B);
                kk_result_t kkr;
                cls_calc_unscaled_KK_vals(&kkin, &kkr);
                if (kk_is_outside_K(&kkr)) continue;
                if (kk_is_outside_L(&kkr)) continue;
                D qdist = kk_calc_quad_distance_edge_edge(&kkin);
                if (qdist * (1 + P_DOUBLE_RES) < min_qdist) {
                    syn_update(c_A, pA, IVP_ST_EDGE);
                    syn_update(c_B, pB, IVP_ST_EDGE);
                    min_qdist = qdist;
                }
            }
        }
    }
    ivp_md_synapse_t *syn0 = c_A->tmp.synapse;
    ivp_md_synapse_t *syn1 = c_B->tmp.synapse;
    ivp_cache_ledge_point_t *m_cache_0 = c_A, *m_cache_1 = c_B;
    if ((ivp_md_synapse_get_status(syn1) == IVP_ST_POINT) && (ivp_md_synapse_get_status(syn0) != IVP_ST_POINT)) {
        ivp_md_synapse_t *h = syn0; syn0 = syn1; syn1 = h;
        m_cache_0 = c_B;
        m_cache_1 = c_A;
    }
    mms_sort_synapses(m, m_cache_0->tmp.synapse, m_cache_1->tmp.synapse);
    const ivp_compact_edge_t *e0 = syn0->edge;
    const ivp_compact_edge_t *e1 = syn1->edge;
    IVP_ASSERT(m_cache_0->tmp.synapse == syn0);   /* ivp_mindist_minimize.cxx:477 */
    IVP_ASSERT(m_cache_1->tmp.synapse == syn1);
    ivp_mrc_type_t ret_val = IVP_MRC_UNINITIALIZED;
    switch (ivp_md_synapse_get_status(syn0)) {
    case IVP_ST_POINT:
        switch (ivp_md_synapse_get_status(syn1)) {
        case IVP_ST_POINT:    ret_val = p_minimize_PP(m, e0, e1, m_cache_0, m_cache_1); break;
        case IVP_ST_EDGE:     ret_val = p_minimize_PK(m, e0, e1, m_cache_0, m_cache_1); break;
        case IVP_ST_TRIANGLE: ret_val = p_minimize_PF(m, e0, e1, m_cache_0, m_cache_1); break;
        default:   /* ivp_mindist_minimize.cxx:504 */
            ivp_debug_printf("IVP_Mindist_Minimize_Solver::p_minimize_FF failed(%s:%d)\n", __FILE__, __LINE__);
            break;
        }
        break;
    case IVP_ST_EDGE:
        switch (ivp_md_synapse_get_status(syn1)) {
        case IVP_ST_EDGE: ret_val = p_minimize_KK(m, e0, e1, m_cache_0, m_cache_1); break;
        default:   /* ivp_mindist_minimize.cxx:518 */
            ivp_debug_printf("IVP_Mindist_Minimize_Solver::p_minimize_FF failed(%s:%d)\n", __FILE__, __LINE__);
            break;
        }
        break;
    default:   /* ivp_mindist_minimize.cxx:524 */
        ivp_debug_printf("IVP_Mindist_Minimize_Solver::p_minimize_FF failed(%s:%d)\n", __FILE__, __LINE__);
        break;
    }
    return ret_val;
}

/* ── dispatch (mms_function_table) ──────────────────────────────────────── */

static ivp_mrc_type_t minimize_default_poly_poly(mms_t *m)
{
    ivp_md_synapse_t *syn0 = ivp_mindist_get_sorted_synapse(m->mindist, 0);
    ivp_md_synapse_t *syn1 = ivp_mindist_get_sorted_synapse(m->mindist, 1);
    const ivp_compact_edge_t *e0 = syn0->edge;
    const ivp_compact_edge_t *e1 = syn1->edge;
    ivp_cache_ledge_point_t c0, c1;
    ivp_cache_ledge_point_init(&c0, syn0->l_obj, edge_ledge(e0));
    ivp_cache_ledge_point_init(&c1, syn1->l_obj, edge_ledge(e1));
    c0.tmp.synapse = syn0;
    c1.tmp.synapse = syn1;
    ivp_mrc_type_t ret;
    IVP_IF(ivp_check_debug_mindist(m->mindist)) {   /* ivp_mindist_minimize.cxx:552 */
        ivp_debug_printf("%32s statii: %i:%i \n", "minimize_default_poly_poly", (int)ivp_md_synapse_get_status(syn0),
                         (int)ivp_md_synapse_get_status(syn1));
    }
#define SYN_COMBINE(a, b) ((a) * IVP_ST_MAX_LEGAL + (b))
    switch (SYN_COMBINE(ivp_md_synapse_get_status(syn0), ivp_md_synapse_get_status(syn1))) {
    case SYN_COMBINE(IVP_ST_POINT, IVP_ST_POINT):    ret = p_minimize_PP(m, e0, e1, &c0, &c1); break;
    case SYN_COMBINE(IVP_ST_POINT, IVP_ST_EDGE):     ret = p_minimize_PK(m, e0, e1, &c0, &c1); break;
    case SYN_COMBINE(IVP_ST_POINT, IVP_ST_TRIANGLE): ret = p_minimize_PF(m, e0, e1, &c0, &c1); break;
    case SYN_COMBINE(IVP_ST_EDGE, IVP_ST_EDGE):      ret = p_minimize_KK(m, e0, e1, &c0, &c1); break;
    default:                                         ret = p_minimize_FF(m, e0, e1, &c0, &c1); break;
    }
    ivp_cache_ledge_point_remove_reference(&c0);
    ivp_cache_ledge_point_remove_reference(&c1);
    /* mms->proove_polypoly(); (commented out in the reference) */
    IVP_IF(ivp_check_debug_mindist(m->mindist)) {   /* ivp_mindist_minimize.cxx:591 */
        ivp_debug_printf("%32s statii: %i:%i len %f\n", " '' ", (int)ivp_md_synapse_get_status(syn0),
                         (int)ivp_md_synapse_get_status(syn1), (double)m->mindist->len_numerator);
    }
    return ret;
}

static ivp_mrc_type_t minimize_B_POLY(mms_t *m)
{
    ivp_md_synapse_t *syn_B = ivp_mindist_get_synapse(m->mindist, 0);
    ivp_cache_ball_t cB;
    cB.object = syn_B->l_obj;
    cB.cache_object = ivp_object_get_cache_object(syn_B->l_obj);
    cB.tmp.synapse = syn_B;
    ivp_md_synapse_t *syn_P = ivp_mindist_get_synapse(m->mindist, 1);
    const ivp_compact_edge_t *P = syn_P->edge;
    ivp_cache_ledge_point_t cP;
    ivp_cache_ledge_point_init(&cP, syn_P->l_obj, edge_ledge(P));
    cP.tmp.synapse = syn_P;
    IVP_IF(ivp_check_debug_mindist(m->mindist)) {   /* ivp_mindist_minimize.cxx:622 */
        ivp_debug_printf("%32s statii: %i \n", "minimize_default_ball_poly", (int)ivp_md_synapse_get_status(syn_P));
    }
    ivp_mrc_type_t ret;
    switch (ivp_md_synapse_get_status(syn_P)) {
    case IVP_ST_POINT:    ret = p_minimize_BP(m, &cB, P, &cP); break;
    case IVP_ST_EDGE:     ret = p_minimize_BK(m, &cB, P, &cP); break;
    case IVP_ST_TRIANGLE: ret = p_minimize_BF(m, &cB, P, &cP); break;
    default:
        ret = IVP_MRC_UNINITIALIZED;
        IVP_CORE();   /* ivp_mindist_minimize.cxx:642 */
        break;
    }
    ivp_cache_ledge_point_remove_reference(&cP);
    ivp_cache_obj_remove_reference(cB.cache_object);
    /* mms->proove_ballpoly(); (commented out in the reference) */
    return ret;
}

/* minimize_PB / minimize_KB / minimize_FB (ivp_mindist_minimize.cxx:599-662):
 * a ball synapse sorted second is illegal, the reference COREs (every build) */
static ivp_mrc_type_t minimize_PB(mms_t *m)
{
    IVP_CORE();   /* ivp_mindist_minimize.cxx:601 */
    mms_swap_synapses(m);
    return minimize_B_POLY(m);
}

static ivp_mrc_type_t minimize_KB(mms_t *m)
{
    IVP_CORE();   /* ivp_mindist_minimize.cxx:652 */
    mms_swap_synapses(m);
    return minimize_B_POLY(m);
}

static ivp_mrc_type_t minimize_FB(mms_t *m)
{
    IVP_CORE();   /* ivp_mindist_minimize.cxx:659 */
    mms_swap_synapses(m);
    return minimize_B_POLY(m);
}

static ivp_mrc_type_t minimize_swapped_poly_poly(mms_t *m)
{
    mms_swap_synapses(m);
    return minimize_default_poly_poly(m);
}

static ivp_mrc_type_t minimize_BB(mms_t *m)
{
    ivp_mindist_t *md = m->mindist;
    ivp_md_synapse_t *syn_A = ivp_mindist_get_synapse(md, 0);
    ivp_md_synapse_t *syn_B = ivp_mindist_get_synapse(md, 1);
    ivp_cache_obj_t *cA = ivp_object_get_cache_object(syn_A->l_obj);
    ivp_cache_obj_t *cB = ivp_object_get_cache_object(syn_B->l_obj);
    const ivp_vec3r_t *A = &cA->m_world_f_object.pos;
    const ivp_vec3r_t *B = &cB->m_world_f_object.pos;
    lvf_subtract2_p(&md->contact_plane, A, B);
    D qlen = lvf_quad_length(&md->contact_plane);
    D inv_len;
    if (fabs((double)qlen) > P_DOUBLE_EPS) inv_len = lv_isqrt(qlen, 3);
    else inv_len = 1.0f;
    md->len_numerator = (float)(qlen * inv_len - (D)md->sum_extra_radius);
    lvf_mult(&md->contact_plane, inv_len);
    set_diff_center(md, cA, cB);
    ivp_cache_obj_remove_reference(cA);
    ivp_cache_obj_remove_reference(cB);
    return IVP_MRC_OK;
}

static ivp_mrc_type_t minimize_illegal(mms_t *m)
{
    (void)m;
    IVP_CORE();   /* ivp_mindist_minimize.cxx:775 */
    return IVP_MRC_ILLEGAL;
}

typedef ivp_mrc_type_t (*mms_fn_t)(mms_t *);

static const mms_fn_t mms_function_table[IVP_ST_MAX_LEGAL][IVP_ST_MAX_LEGAL] = {
    /* POINT    */ { minimize_default_poly_poly, minimize_default_poly_poly, minimize_default_poly_poly, minimize_PB },
    /* EDGE     */ { minimize_swapped_poly_poly, minimize_default_poly_poly, minimize_illegal, minimize_KB },
    /* TRIANGLE */ { minimize_swapped_poly_poly, minimize_illegal, minimize_default_poly_poly, minimize_FB },
    /* BALL     */ { minimize_B_POLY, minimize_B_POLY, minimize_B_POLY, minimize_BB },
};

static ivp_mrc_type_t mms_recalc_mindist_sub(mms_t *m)
{
    int s0 = (int)ivp_md_synapse_get_status(ivp_mindist_get_sorted_synapse(m->mindist, 0));
    int s1 = (int)ivp_md_synapse_get_status(ivp_mindist_get_sorted_synapse(m->mindist, 1));
    IVP_ASSERT(s0 >= IVP_ST_POINT && s0 < IVP_ST_MAX_LEGAL);   /* ivp_mindist_minimize.hxx:146 */
    IVP_ASSERT(s1 >= IVP_ST_POINT && s1 < IVP_ST_MAX_LEGAL);
    if (s0 < 0 || s0 >= IVP_ST_MAX_LEGAL || s1 < 0 || s1 >= IVP_ST_MAX_LEGAL) return IVP_MRC_ILLEGAL;
    return mms_function_table[s0][s1](m);
}

static void mms_pierce_mindist(mms_t *m)
{
    ivp_md_synapse_t *syn_pierce;
    ivp_md_synapse_t *syn_other = ivp_mindist_get_sorted_synapse(m->mindist, 0);
    if (ivp_md_synapse_get_status(syn_other) != IVP_ST_BACKSIDE) {
        syn_pierce = ivp_mindist_get_sorted_synapse(m->mindist, 1);
    } else {
        syn_pierce = syn_other;
    }
    IVP_ASSERT(ivp_md_synapse_get_status(syn_pierce) == IVP_ST_BACKSIDE);   /* ivp_mindist_minimize.cxx:165 */
    const ivp_compact_edge_t *F = syn_pierce->edge;
    const ivp_compact_edge_t *pierced_tri = cls_minimize_on_other_side(F, &m->pos_opposite_BacksideOs);
    ivp_md_synapse_update(syn_pierce, pierced_tri, IVP_ST_TRIANGLE);
}

/* ── IVP_Mindist::recalc_invalid_mindist / recalc_mindist ───────────────── */

ivp_mrc_type_t ivp_mindist_recalc_invalid_mindist(ivp_mindist_t *md)
{
    ivp_time_code_t tc = ivp_env_get_current_time_code(md->env);
    if (md->recalc_time_stamp == tc) return IVP_MRC_ALREADY_CALCULATED;
    md->recalc_time_stamp = tc;
    mms_t mms;
    mms_init(&mms, md);
    mms.P_Finish_Counter = 0;
    ivp_mrc_type_t res = mms_recalc_mindist_sub(&mms);
    if (res == IVP_MRC_OK) {
        md->recalc_result = IVP_MDRR_OK;
        return res;
    }
    md->recalc_result = IVP_MDRR_INTRUSION;
    if (res == IVP_MRC_BACKSIDE) {
        mms_pierce_mindist(&mms);
    }
    if (res == IVP_MRC_BACKSIDE || res == IVP_MRC_ENDLESS_LOOP) {
        if (md->mindist_function == IVP_MF_PHANTOM || md->mindist_status == IVP_MD_HULL_RECURSIVE) return res;
        ivp_mindist_mindist_rescue_push(md);
        return res;
    }
    /* reference "default: CORE; break;" then "CORE;" and loops again
     * (ivp_mindist_minimize.cxx:226 / 229); libivp returns the result */
    IVP_CORE();
    IVP_CORE();
    return res;
}

ivp_mrc_type_t ivp_mindist_recalc_mindist(ivp_mindist_t *md)
{
    ivp_time_code_t tc = ivp_env_get_current_time_code(md->env);
    if (md->recalc_time_stamp == tc) return IVP_MRC_ALREADY_CALCULATED;
    md->recalc_time_stamp = tc;
    mms_t mms;
    mms_init(&mms, md);
    int pierce_counter = 0;
    for (;;) {
        ivp_mrc_type_t res = mms_recalc_mindist_sub(&mms);
        if (res == IVP_MRC_OK) {
            md->recalc_result = IVP_MDRR_OK;
            return res;
        }
        md->recalc_result = IVP_MDRR_INTRUSION;
        if (res == IVP_MRC_BACKSIDE) {
            /* ivp_mindist_minimize.cxx:260 "#ifdef IVP_MINDIST_BEHAVIOUR_DEBUG
             * IVP_ASSERT(detect_collision(<IVP_Polygon tetras>) == IVP_FALSE)":
             * not ported, needs the IVP_Polygon tetra representation (not in
             * libivp) and a detect_collision that the reference does not have. */
            mms_pierce_mindist(&mms);
            if (++pierce_counter < IVP_MAX_PIERCINGS) continue;
        }
        if (res == IVP_MRC_BACKSIDE || res == IVP_MRC_ENDLESS_LOOP) {
            if (md->mindist_function == IVP_MF_PHANTOM || md->mindist_status == IVP_MD_HULL_RECURSIVE) return res;
            IVP_IF(1) {   /* ivp_mindist_minimize.cxx:277 */
                const char *name0 = ivp_mindist_get_synapse(md, 0)->l_obj->name;
                if (!name0) name0 = "(null)";
                const char *name1 = ivp_mindist_get_synapse(md, 1)->l_obj->name;
                if (!name1) name1 = "(null)";
                ivp_debug_printf("recalc_mindist: Endless Loop without collision or termination problem.%s %s\n", name0,
                                 name1);
            }
            ivp_mindist_mindist_rescue_push(md);
            /* ivp_mindist_minimize.cxx:292 "#ifdef IVP_MINDIST_BEHAVIOUR_DEBUG"
             * (detect_collision on the IVP_Polygon tetras, then CORE or
             * IVP_IF(1) termination_len reset + trace + "continue"): not
             * ported, same reason as above (and it does not compile in the
             * reference: psyn_0 / psyn_1 / P_Finish_Counter are undeclared). */
            return res;
        }
        /* reference: default: break; -> loops again */
        if (res == IVP_MRC_ILLEGAL || res == IVP_MRC_UNINITIALIZED) return res;
    }
}

/* ── ivp_mindist_debug.cxx ──────────────────────────────────────────────── */

bool ivp_check_debug_mindist(ivp_mindist_t *md)
{
    if (!IVP_DEBUG_OBJECT0) return false;
    if (!IVP_DEBUG_OBJECT1) return false;
    const char *search0 = IVP_DEBUG_OBJECT0;
    const char *search1 = IVP_DEBUG_OBJECT1;

    const char *name0 = ivp_mindist_get_synapse(md, 0)->l_obj->name;
    const char *name1 = ivp_mindist_get_synapse(md, 1)->l_obj->name;
    if ((!ivp_string_cmp(name0, search0, false) && !ivp_string_cmp(name1, search1, false)) ||
        (!ivp_string_cmp(name0, search1, false) && !ivp_string_cmp(name1, search0, false))) {
        ivp_time_t time = ivp_env_get_current_time(ivp_mindist_get_synapse(md, 0)->l_obj->owner);
        if (time - (double)IVP_DEBUG_TIME >= 0.0) {
            ivp_debug_printf("%10s %10s %8f-%X: ", name0, name1, (double)time, (unsigned)((uintptr_t)md & 0xffffu));
            return true;
        }
    }
    return false;
}

int ivp_mindist_debug_p_sqrt(double val)   /* for debugger */
{
    ivp_debug_printf("%g\n", sqrt(val));
    return 0;
}

#ifdef IVP_DEBUG_BUILD

/* ── IVP_Compact_Ledge_Solver "#ifdef DEBUG" helpers ───────────────────── */

ivp_real_t ivp_cls_debug_calc_s_val(const ivp_compact_edge_t *edge, const ivp_vec3r_t *p_world,
                                    ivp_cache_ledge_point_t *m_cache_edge)
{
    /* ivp_compact_ledge_solver.cxx:288 */
    ivp_vec3r_t tp, tp_next;
    cls_give_world_coords_AT(edge, m_cache_edge, &tp);
    cls_give_world_coords_AT(ivp_edge_get_next(edge), m_cache_edge, &tp_next);
    ivp_vec3r_t vec1, vec2;
    lvp_subtract2(&vec1, &tp_next, &tp);
    lvp_subtract2(&vec2, p_world, &tp);
    D v1_ql = lvp_quad_length(&vec1);
    if (v1_ql < P_DOUBLE_EPS) return 0.0;
    D sv = lvp_dot(&vec1, &vec2);
    sv /= v1_ql;
    return sv;
}

void ivp_cls_debug_calc_qr_vals(const ivp_compact_edge_t *e_tri, const ivp_vec3r_t *p_world, ivp_real_t *out_q,
                                ivp_real_t *out_r, ivp_cache_ledge_point_t *m_cache_e_tri)
{
    /* ivp_compact_ledge_solver.cxx:341 */
    *out_q = 0.0;
    *out_r = 0.0;
    ivp_vec3r_t tp, tp_next, tp_prev;
    cls_give_world_coords_AT(e_tri, m_cache_e_tri, &tp);
    cls_give_world_coords_AT(ivp_edge_get_next(e_tri), m_cache_e_tri, &tp_next);
    cls_give_world_coords_AT(ivp_edge_get_prev(e_tri), m_cache_e_tri, &tp_prev);
    ivp_vec3r_t R, Q, Pvec;
    lvp_subtract2(&Q, &tp, &tp_next);   /* other dir */
    lvp_subtract2(&R, &tp_prev, &tp_next);
    D QQ = lvp_quad_length(&Q);
    D RR = lvp_quad_length(&R);
    D QR = lvp_dot(&R, &Q);
    D QQRR = QQ * RR;
    D QRQR = QR * QR;
    D det = QQRR - QRQR;
    if (fabs((double)det) < P_DOUBLE_EPS) return;
    D i_det = 1.0f / det;
    lvp_subtract2(&Pvec, p_world, &tp_next);
    D sq = lvp_dot(&Pvec, &Q);
    D sr = lvp_dot(&Pvec, &R);
    *out_q = i_det * (RR * sq - sr * QR);
    *out_r = i_det * (QQ * sr - sq * QR);
}

void ivp_cls_debug_calc_qr_vals_edge(const ivp_compact_edge_t *e_tri, const ivp_compact_edge_t *p,
                                     ivp_real_t *out_q, ivp_real_t *out_r, ivp_cache_ledge_point_t *m_cache_e_tri,
                                     ivp_cache_ledge_point_t *m_cache_p)
{
    /* ivp_compact_ledge_solver.cxx:382 */
    ivp_vec3r_t pp;
    cls_give_world_coords_AT(p, m_cache_p, &pp);
    ivp_cls_debug_calc_qr_vals(e_tri, &pp, out_q, out_r, m_cache_e_tri);
}

void ivp_cls_debug_calc_hesse_normized_AT(const ivp_compact_edge_t *edge, ivp_cache_ledge_point_t *clp,
                                          ivp_vec3r_t *normal_ws, ivp_real_t *hesse_val_ws)
{
    /* ivp_compact_ledge_solver.cxx:585 (normized) */
    ivp_vec3r_t p0, p1, p2;
    cls_give_world_coords_AT(edge, clp, &p0);
    cls_give_world_coords_AT(ivp_edge_get_next(edge), clp, &p1);
    cls_give_world_coords_AT(ivp_edge_get_prev(edge), clp, &p2);
    lv_hesse_t h;
    lv_hesse_calc_ppp(&h, &p0, &p2, &p1);
    lv_hesse_normize(&h);
    *normal_ws = h.n;
    *hesse_val_ws = h.hesse_val;
}

/* ── ivp_mindist_debug.cxx "#ifdef DEBUG" ──────────────────────────────── */

/* copy-paste from p_minimize_FF for proove_polypoly() */
ivp_real_t ivp_mindist_debug_p_optimize_FF(const ivp_compact_edge_t *A, const ivp_compact_edge_t *B,
                                           ivp_cache_ledge_point_t *m_cache_A, ivp_cache_ledge_point_t *m_cache_B,
                                           ivp_real_t min_qdist)
{
    /* Case: Surface - Surface */

    /* check all 9 point point combinations first */
    {
        const ivp_compact_edge_t *pA, *pB;
        int a, b;
        for (pA = A, a = 0; a < 3; pA = ivp_edge_get_next(pA), a++) {
            for (pB = B, b = 0; b < 3; pB = ivp_edge_get_next(pB), b++) {
                ivp_vec3r_t pos_B_ws, pos_A_ws;
                cls_give_world_coords_AT(pB, m_cache_B, &pos_B_ws);
                cls_give_world_coords_AT(pA, m_cache_A, &pos_A_ws);
                D qdist = lvp_quad_distance(&pos_A_ws, &pos_B_ws);
                if (qdist < min_qdist) min_qdist = qdist;
            }
        }
    }
    /* check all 6 point area combinations */
    ivp_cache_ledge_point_t *cc_A[2] = { m_cache_A, m_cache_B };
    ivp_cache_ledge_point_t *cc_B[2] = { m_cache_B, m_cache_A };
    const ivp_compact_edge_t *tabA[2] = { A, B };
    const ivp_compact_edge_t *tabB[2] = { B, A };
    for (int i = 0; i < 2; i++) {
        const ivp_compact_edge_t *pA;
        const ivp_compact_edge_t *pB = tabB[i];
        int a;
        lv_hesse_t hesse_B;
        cls_calc_hesse_object(pB, cc_B[i]->compact_ledge, &hesse_B);
        for (pA = tabA[i], a = 0; a < 3; pA = ivp_edge_get_next(pA), a++) {
            ivp_vec3r_t pA_Bos;
            cls_calc_pos_other_space(pA, cc_A[i], cc_B[i], &pA_Bos);
            qr_result_t qr;
            cls_calc_unscaled_qr_vals_F_space(cc_B[i]->compact_ledge, pB, &pA_Bos, &qr);
            if (!qr_is_outside(&qr)) {
                D qdist = lv_hesse_get_dist(&hesse_B, &pA_Bos);
                qdist *= qdist;
                if (qdist * (1 + P_DOUBLE_RES) < min_qdist) min_qdist = qdist;
            }
        }
    }
    /* check all 18 point edge combinations */
    for (int i = 0; i < 2; i++) {
        const ivp_compact_edge_t *pA, *pB;
        int a, b;
        for (pB = tabB[i], b = 0; b < 3; pB = ivp_edge_get_next(pB), b++) {
            ivp_vec3r_t pB_Aos;
            cls_calc_pos_other_space(pB, cc_B[i], cc_A[i], &pB_Aos);
            for (pA = tabA[i], a = 0; a < 3; pA = ivp_edge_get_next(pA), a++) {
                s_result_t sr;
                cls_calc_unscaled_s_val_K_space(cc_A[i]->compact_ledge, pA, &pB_Aos, &sr);
                if (s_is_outside(&sr)) continue;
                D qdist = cls_quad_dist_edge_to_point_K_space(cc_A[i]->compact_ledge, pA, &pB_Aos);
                if (qdist * (1 + P_DOUBLE_RES) < min_qdist) min_qdist = qdist;
            }
        }
    }
    {
        /* check all 9 edge edge combinations */
        const ivp_compact_edge_t *pA, *pB;
        int a, b;
        for (pA = A, a = 0; a < 3; pA = ivp_edge_get_next(pA), a++) {
            for (pB = B, b = 0; b < 3; pB = ivp_edge_get_next(pB), b++) {
                kk_input_t kkin;
                kk_input_init(&kkin, pA, pB, m_cache_A, m_cache_B);
                D quad_dist = lvp_quad_length(&kkin.cross_KL_Los);
                if (quad_dist <= (P_DOUBLE_RES * P_DOUBLE_RES * P_MAX_OBJECT_SIZE * P_MAX_OBJECT_SIZE)) {
                    continue;   /* not parallel */
                }
                kk_result_t kkr;
                cls_calc_unscaled_KK_vals(&kkin, &kkr);
                if (kk_is_outside_K(&kkr)) continue;
                if (kk_is_outside_L(&kkr)) continue;
                D qdist = kk_calc_quad_distance_edge_edge(&kkin);
                if (qdist * (1 + P_DOUBLE_RES) < min_qdist) min_qdist = qdist;
            }
        }
    }
    return min_qdist;
}

bool ivp_mindist_debug_proove_polypoly(ivp_mindist_t *mindist)
{
    /* does a REAL SLOW (!!!) brute force recalculation and check.
     * requires collision free situation */
    ivp_md_synapse_t *syn0 = ivp_mindist_get_synapse(mindist, 0);
    ivp_md_synapse_t *syn1 = ivp_mindist_get_synapse(mindist, 1);

    const ivp_compact_edge_t *e0 = syn0->edge;
    const ivp_compact_edge_t *e1 = syn1->edge;
    const ivp_compact_ledge_t *ledge0 = edge_ledge(e0);
    const ivp_compact_ledge_t *ledge1 = edge_ledge(e1);

    ivp_cache_ledge_point_t m_cache_0, m_cache_1;
    ivp_cache_ledge_point_init(&m_cache_0, syn0->l_obj, ledge0);
    ivp_cache_ledge_point_init(&m_cache_1, syn1->l_obj, ledge1);
    m_cache_0.tmp.synapse = syn0;   /* remember order of synapses */
    m_cache_1.tmp.synapse = syn1;

    D min_qdist = P_DOUBLE_MAX;
    const ivp_compact_triangle_t *best_t0 = NULL;
    const ivp_compact_triangle_t *best_t1 = NULL;

    /* each area of the first object */
    for (int l0 = ivp_ledge_get_n_triangles(ledge0) - 1; l0 >= 0; l0--) {
        for (int l1 = ivp_ledge_get_n_triangles(ledge1) - 1; l1 >= 0; l1--) {
            const ivp_compact_triangle_t *t0 = &ivp_ledge_get_first_triangle(ledge0)[l0];
            const ivp_compact_triangle_t *t1 = &ivp_ledge_get_first_triangle(ledge1)[l1];
            D qd = ivp_mindist_debug_p_optimize_FF(ivp_tri_get_first_edge(t0), ivp_tri_get_first_edge(t1), &m_cache_0,
                                                   &m_cache_1, min_qdist);
            if (qd < min_qdist) {
                /* found better situation, synapses are already updated */
                best_t0 = t0;
                best_t1 = t1;
                min_qdist = qd;
            }
        }
    }
    (void)best_t0;
    (void)best_t1;
    ivp_cache_ledge_point_remove_reference(&m_cache_0);
    ivp_cache_ledge_point_remove_reference(&m_cache_1);

    /* compare brute force result with original mindist state */
    D dist_0 = LV_SQRT(min_qdist);
    D dist_1 = (D)mindist->len_numerator;
    D diff = (D)fabs((double)(dist_0 - dist_1)) + P_DOUBLE_EPS;

    D perc = diff * 100.0f / dist_1;
    /* check if difference is neglectible */
    if (perc < 50.0f) return true;

    /* somewhat greater differences ... */
    ivp_debug_printf("\nBrute Force Recalculation of Mindist shows differences:\n");
    ivp_debug_printf("Dsts: md %f, bf %f, diff: %f percent: %f\n", (double)dist_0, (double)dist_1, (double)diff,
                     (double)perc);
    ivp_debug_printf("Coords of synapses (original 0/1, brute force 0/1):\n");
    IVP_CORE();
    return false;
}

bool ivp_mindist_debug_proove_ballpoly(ivp_mindist_t *mindist)
{
    /* does a REAL SLOW (!!!) brute force recalculation and check.
     * requires collision free situation */
    ivp_md_synapse_t *syn0 = ivp_mindist_get_synapse(mindist, 0);
    ivp_md_synapse_t *syn1 = ivp_mindist_get_synapse(mindist, 1);

    const ivp_compact_edge_t *e1 = syn1->edge;
    const ivp_compact_ledge_t *ledge1 = edge_ledge(e1);

    ivp_cache_obj_t *m_cache_0 = ivp_object_get_cache_object(syn0->l_obj);
    ivp_cache_obj_t *m_cache_1 = ivp_object_get_cache_object(syn1->l_obj);

    D min_qdist = P_DOUBLE_MAX;
    const ivp_compact_triangle_t *best_t1 = NULL;
    const ivp_vec3r_t *ball_ws = &m_cache_0->m_world_f_object.pos;
    ivp_vec3r_t ball_Fos;
    co_pos_to_object(m_cache_1, ball_ws, &ball_Fos);

    for (int l1 = ivp_ledge_get_n_triangles(ledge1) - 1; l1 >= 0; l1--) {
        const ivp_compact_triangle_t *t1 = &ivp_ledge_get_first_triangle(ledge1)[l1];
        D qd = ivp_lcls_calc_qlen_PF_F_space(ledge1, t1, &ball_Fos);
        if (qd < min_qdist) {
            /* found better situation, synapses are already updated */
            best_t1 = t1;
            min_qdist = qd;
        }
    }
    (void)best_t1;
    ivp_cache_obj_remove_reference(m_cache_0);
    ivp_cache_obj_remove_reference(m_cache_1);

    /* compare brute force result with original mindist state */
    D dist_0 = LV_SQRT(min_qdist) - (D)mindist->sum_extra_radius;
    D dist_1 = (D)mindist->len_numerator;
    D diff = (D)fabs((double)(dist_0 - dist_1)) + P_DOUBLE_EPS;

    D perc = diff * 100.0f / dist_1;
    /* check if difference is neglectible */
    if (perc < 50.0f) return true;

    /* somewhat greater differences ... */
    ivp_debug_printf("\nBrute Force Recalculation of Mindist shows differences:\n");
    ivp_debug_printf("Dsts: md %f, bf %f, diff: %f percent: %f\n", (double)dist_0, (double)dist_1, (double)diff,
                     (double)perc);
    ivp_debug_printf("Coords of synapses (original 0/1, brute force 0/1):\n");
    IVP_CORE();
    return false;
}

int ivp_mindist_debug_situation(ivp_cache_ledge_point_t *tri_cache, ivp_cache_ledge_point_t *e_cache,
                                const ivp_compact_edge_t *triangle_e0, const ivp_compact_edge_t *edge)
{
    /* may be called from debugger (with print)
     * transforms triangle to flat (paper!), also transforms edge,
     * prints coords and labels */

    /* get world coords */
    ivp_vec3r_t t_e_0, t_e_1, t_e_2;
    cls_give_world_coords_AT(triangle_e0, tri_cache, &t_e_0);
    cls_give_world_coords_AT(ivp_edge_get_next(triangle_e0), tri_cache, &t_e_1);
    cls_give_world_coords_AT(ivp_edge_get_prev(triangle_e0), tri_cache, &t_e_2);

    ivp_vec3r_t e_0, e_1;
    cls_give_world_coords_AT(edge, e_cache, &e_0);
    cls_give_world_coords_AT(ivp_edge_get_next(edge), e_cache, &e_1);

    /*** search matrix flat on triangle with x-axis = triangle_e0 ***/

    /* vector 0 (x) points in direction of triangle_e0 */
    ivp_vec3r_t vec_0;
    lvp_subtract2(&vec_0, &t_e_1, &t_e_0);
    (void)lvp_normize(&vec_0);

    /* vector 2 (z) is orthogonal to triangle, i.e. emerging out of the paper */
    ivp_vec3r_t vec_2;   /* world, normized (IVP_U_Hesse) */
    ivp_real_t vec_2_hesse_val;
    ivp_cls_debug_calc_hesse_normized_AT(triangle_e0, tri_cache, &vec_2, &vec_2_hesse_val);
    (void)vec_2_hesse_val;
    lvp_mult(&vec_2, -1.0f);

    /* vector 1 (y) orthogonal to it */
    ivp_vec3r_t vec_1;
    lvp_cross(&vec_1, &vec_0, &vec_2);
    (void)lvp_normize(&vec_1);

    /* t_e_0 as origin: m_world_from_flat.init_columns4(vec_0, vec_1, vec_2, trans) */
    ivp_transform_r_t m_world_from_flat;
    for (int r = 0; r < 3; r++) {
        LVM_E(&m_world_from_flat, r, 0) = vec_0.k[r];
        LVM_E(&m_world_from_flat, r, 1) = vec_1.k[r];
        LVM_E(&m_world_from_flat, r, 2) = vec_2.k[r];
    }
    m_world_from_flat.pos = t_e_0;

    /*** transform whole situation ***/

    /* transform triangle */
    ivp_vec3r_t tf_0, tf_1, tf_2;
    lvm_vimult4(&m_world_from_flat, &t_e_0, &tf_0);
    lvm_vimult4(&m_world_from_flat, &t_e_1, &tf_1);
    lvm_vimult4(&m_world_from_flat, &t_e_2, &tf_2);

    /* transform edge */
    ivp_vec3r_t ef_0, ef_1;
    lvm_vimult4(&m_world_from_flat, &e_0, &ef_0);
    lvm_vimult4(&m_world_from_flat, &e_1, &ef_1);

    /* calc pierce position (if any) */
    bool pierces = false;
    bool pierce_inside = false;
    ivp_vec3r_t pierce_point_f;
    lvp_zero(&pierce_point_f);

    /* IVP_U_Straight edge_straight.set(&e_0, &edge_vec) (vec normized) */
    ivp_vec3r_t edge_vec;
    lvp_subtract2(&edge_vec, &e_1, &e_0);
    ivp_vec3r_t straight_start = e_0;
    ivp_vec3r_t straight_vec = edge_vec;
    (void)lvp_normize(&straight_vec);
    lv_hesse_t tri_hesse;
    ivp_cls_debug_calc_hesse_normized_AT(triangle_e0, tri_cache, &tri_hesse.n, &tri_hesse.hesse_val);

    ivp_vec3r_t pierce_point_w;
    lvp_zero(&pierce_point_w);
    ivp_real_t q = 0, r = 0;
    bool intersect_ok;
    {   /* IVP_U_Hesse::calc_intersect_with(straight, point_out) */
        D dist1 = lv_hesse_get_dist(&tri_hesse, &straight_start);
        ivp_vec3r_t hp;
        lvp_add2(&hp, &straight_start, &straight_vec);
        D dist2 = lv_hesse_get_dist(&tri_hesse, &hp);
        D delta_dist = dist2 - dist1;
        if (fabs((double)delta_dist) < P_DOUBLE_EPS) {
            if (dist1 >= P_DOUBLE_EPS) {
                intersect_ok = false;   /* true parallel */
            } else {
                pierce_point_w = straight_start;
                intersect_ok = true;
            }
        } else {
            D factor = dist1 / delta_dist;
            lvp_add_multiple2(&pierce_point_w, &straight_start, &straight_vec, -factor);
            intersect_ok = true;
        }
    }
    if (intersect_ok) {
        pierces = true;
        lvm_vimult4(&m_world_from_flat, &pierce_point_w, &pierce_point_f);
        ivp_cls_debug_calc_qr_vals(triangle_e0, &pierce_point_w, &q, &r, tri_cache);
        if (q >= 0 && r >= 0 && r + q <= 1) pierce_inside = true;
    }

    ivp_real_t q_edge_start, q_edge_end, r_edge_start, r_edge_end;
    ivp_cls_debug_calc_qr_vals(triangle_e0, &e_0, &q_edge_start, &r_edge_start, tri_cache);
    ivp_cls_debug_calc_qr_vals(triangle_e0, &e_1, &q_edge_end, &r_edge_end, tri_cache);

    kk_input_t kkin;
    kk_input_init(&kkin, triangle_e0, edge, tri_cache, e_cache);
    kk_result_t kkr;
    cls_calc_unscaled_KK_vals(&kkin, &kkr);

    /** print everything ***/
    ivp_debug_printf("triangle edge situation:");
    ivp_debug_printf(" (coords so that triangle appears flat):\n\n");
    ivp_debug_printf("Triangle: %f %f %f\n", (double)tf_0.k[0], (double)tf_0.k[1], (double)tf_0.k[2]);
    ivp_debug_printf("          %f %f %f\n", (double)tf_1.k[0], (double)tf_1.k[1], (double)tf_1.k[2]);
    ivp_debug_printf("          %f %f %f\n\n", (double)tf_2.k[0], (double)tf_2.k[1], (double)tf_2.k[2]);
    ivp_debug_printf("Edge:     %f %f %f\n", (double)ef_0.k[0], (double)ef_0.k[1], (double)ef_0.k[2]);
    ivp_debug_printf("          %f %f %f\n\n", (double)ef_1.k[0], (double)ef_1.k[1], (double)ef_1.k[2]);

    if (pierces) {
        ivp_debug_printf("pierces %s triangle:  %f %f    q=%f r=%f \n", (pierce_inside) ? "inside" : "outside",
                         (double)pierce_point_f.k[0], (double)pierce_point_f.k[1], (double)q, (double)r);
    } else {
        ivp_debug_printf("Edge is parallel.\n");
    }
    ivp_debug_printf("\n");
    ivp_debug_printf("edge startpoint q=%f r=%f\n", (double)q_edge_start, (double)r_edge_start);
    ivp_debug_printf("                s=%f\n", (double)ivp_cls_debug_calc_s_val(triangle_e0, &e_0, tri_cache));
    ivp_debug_printf("     endpoint   q=%f r=%f\n", (double)q_edge_end, (double)r_edge_end);
    ivp_debug_printf("                s=%f\n\n", (double)ivp_cls_debug_calc_s_val(triangle_e0, &e_1, tri_cache));

    ivp_debug_printf("tri_edge(K) to edge(L): slK=%f skL=%f\n",
                     (double)(kkr.checks_K[0] / (kkr.checks_K[0] + kkr.checks_K[1])),
                     (double)(kkr.checks_L[0] / (kkr.checks_L[0] + kkr.checks_L[1])));
    return 0;   /* for debugger */
}

#endif /* IVP_DEBUG_BUILD */
