/* ivp_mindist.c -- Minimum distance between convex ledges
 *
 * Iterative hill-climbing algorithm for finding the closest pair of
 * geometric features between two convex compact ledges.
 *
 * Algorithm: Voronoi-region walking (matching legacy IVP_Mindist_Minimize_Solver)
 *   State machine transitions between feature pair types:
 *     PF (Point-Face) -> PK (Point-Edge) -> PP (Point-Point) -> KK (Edge-Edge)
 *   Each step checks the Voronoi region and walks to a neighboring feature
 *   that is closer to the current opposing feature.
 *
 * Ported from ivp_mindist_mcases.cxx / ivp_mindist_minimize.cxx.
 *
 * German abbreviations used in original IVP code:
 *   P = Punkt (Point/Vertex)
 *   K = Kante (Edge)
 *   F = Flaeche (Face/Triangle)
 */
#include <ivp/collision/ivp_mindist.h>
#include <ivp/collision/ivp_compact_ledge_solver.h>
#include <ivp/collision/ivp_narrowphase.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/math/ivp_fast_math.h>

#include <string.h>

/* ── Helper: vertex position as ivp_vec3r_t ────────────────────────────── */

static inline ivp_vec3r_t md_point(const ivp_compact_poly_point_t *p)
{
    return ivp_vec3r((ivp_real_t)p->k[0],
                     (ivp_real_t)p->k[1],
                     (ivp_real_t)p->k[2]);
}

static ivp_vec3r_t md_ledge_centroid(const ivp_compact_ledge_t *ledge)
{
    ivp_vec3r_t centroid = {{IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0)}};
    int n_pts = ivp_ledge_get_n_points(ledge);
    const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(ledge);

    for (int i = 0; i < n_pts; ++i) {
        centroid.k[0] += (ivp_real_t)pts[i].k[0];
        centroid.k[1] += (ivp_real_t)pts[i].k[1];
        centroid.k[2] += (ivp_real_t)pts[i].k[2];
    }

    if (n_pts > 0) {
        ivp_real_t inv = IVP_REAL(1.0) / (ivp_real_t)n_pts;
        centroid.k[0] *= inv;
        centroid.k[1] *= inv;
        centroid.k[2] *= inv;
    }

    return centroid;
}

/* ── Minimize state machine ────────────────────────────────────────────── */

/* Maximum iterations before declaring convergence */
#define MD_MAX_ITERATIONS 256

/* Loop detection: simple visited-hash to detect cycling */
#define MD_LOOP_HASH_SIZE 64

typedef struct md_loop_entry {
    uintptr_t a;
    uintptr_t b;
    uint8_t state;
} md_loop_entry_t;

typedef struct md_loop_hash {
    md_loop_entry_t entries[MD_LOOP_HASH_SIZE];
    int count;
} md_loop_hash_t;

static void md_loop_init(md_loop_hash_t *h) {
    h->count = 0;
}

static bool md_loop_check(md_loop_hash_t *h,
                          int state,
                          const ivp_compact_edge_t *a,
                          const ivp_compact_edge_t *b)
{
    uintptr_t xa = (uintptr_t)a;
    uintptr_t xb = (uintptr_t)b;
    if (xa > xb) { uintptr_t t = xa; xa = xb; xb = t; }

    for (int i = 0; i < h->count; i++) {
        if (h->entries[i].a == xa && h->entries[i].b == xb &&
            h->entries[i].state == (uint8_t)state)
            return true;  /* loop detected */
    }
    if (h->count < MD_LOOP_HASH_SIZE) {
        h->entries[h->count].a = xa;
        h->entries[h->count].b = xb;
        h->entries[h->count].state = (uint8_t)state;
        h->count++;
    }
    return false;
}

/* ── Feature state ─────────────────────────────────────────────────────── */

typedef enum {
    MD_STATE_PF,   /* Point(synapse 0) vs Face(synapse 1) */
    MD_STATE_PK,   /* Point(synapse 0) vs Edge(synapse 1) */
    MD_STATE_PP,   /* Point(synapse 0) vs Point(synapse 1) */
    MD_STATE_KK,   /* Edge(synapse 0) vs Edge(synapse 1) */
    MD_STATE_DONE, /* Converged */
    MD_STATE_LOOP  /* Loop detected, use current best */
} md_state_t;

typedef struct md_ctx {
    const ivp_compact_ledge_t *ledge_a;
    const ivp_compact_ledge_t *ledge_b;
    const ivp_compact_edge_t  *feat_a;  /* current feature on ledge_a */
    const ivp_compact_edge_t  *feat_b;  /* current feature on ledge_b */
    md_state_t state;
    ivp_feature_type_t type_a;
    ivp_feature_type_t type_b;
    int iterations;
    md_loop_hash_t loop_hash;
    bool flipped; /* true if (ledge_a, ledge_b) is swapped vs original input order */
} md_ctx_t;

/* ── Point(A) vs Face(B): PF step ──────────────────────────────────────── */

static void md_step_PF(md_ctx_t *ctx)
{
    const ivp_compact_edge_t *P = ctx->feat_a;
    const ivp_compact_edge_t *F = ctx->feat_b; /* first edge of face */

    ivp_vec3r_t p_os = md_point(ivp_edge_get_start_point(P, ctx->ledge_a));

    /* Compute QR values: is P inside face F? */
    ivp_qr_result_t qr;
    ivp_calc_qr_vals(ctx->ledge_b, F, &p_os, &qr);

    if (!ivp_qr_is_outside(&qr)) {
        /* Point projects inside face. Leave_PF:
         * Walk P's neighboring vertices to find one closer to F's plane. */
        ivp_vec3r_t hesse;
        ivp_calc_hesse_vec(ctx->ledge_b, F, &hesse);
        ivp_real_t hesse_sq = ivp_vec3r_quad_length(&hesse);
        if (hesse_sq < IVP_REAL_RES * IVP_REAL_RES) {
            ctx->state = MD_STATE_DONE;
            return;
        }

        ivp_vec3r_t P_pos = md_point(ivp_edge_get_start_point(P, ctx->ledge_a));
        ivp_real_t dist_P = ivp_vec3r_dot(&hesse, &P_pos);
        ivp_real_t best_delta = IVP_REAL(0.0);
        const ivp_compact_edge_t *best_neighbor = NULL;

        /* Walk vertex fan around P: prev(opposite(prev)) repeated */
        const ivp_compact_edge_t *Pm = ivp_edge_get_prev(P);
        const ivp_compact_edge_t *e  = ivp_edge_get_prev(ivp_edge_get_opposite(Pm));
        const ivp_compact_edge_t *start_e = e;
        do {
            ivp_vec3r_t e_pos = md_point(ivp_edge_get_start_point(e, ctx->ledge_a));
            ivp_real_t delta = ivp_vec3r_dot(&hesse, &e_pos) - dist_P;
            /* Normalize by edge length for fair gradient comparison */
            if (delta < best_delta) {
                ivp_real_t edge_qlen = ivp_vec3r_quad_distance(&P_pos, &e_pos);
                if (edge_qlen > IVP_REAL_RES) {
#ifdef IVP_REAL_DOUBLE
                    ivp_real_t grad = delta * (ivp_real_t)ivp_isqrt_double((double)edge_qlen, 0);
#else
                    ivp_real_t grad = delta * ivp_isqrt_float(edge_qlen);
#endif
                    if (grad < best_delta) {
                        best_delta = grad;
                        best_neighbor = e;
                    }
                }
            }
            e = ivp_edge_get_prev(ivp_edge_get_opposite(e));
        } while (e != start_e);

        if (!best_neighbor) {
            /* No improvement: P-F is the local minimum */
            ctx->type_a = IVP_FEATURE_POINT;
            ctx->type_b = IVP_FEATURE_TRIANGLE;
            ctx->state = MD_STATE_DONE;
            return;
        }

        /* Check if new vertex also projects inside F */
        ivp_vec3r_t new_p_os = md_point(ivp_edge_get_start_point(best_neighbor, ctx->ledge_a));
        ivp_qr_result_t qr2;
        ivp_calc_qr_vals(ctx->ledge_b, F, &new_p_os, &qr2);

        if (!ivp_qr_is_outside(&qr2)) {
            /* New vertex also inside face -> repeat PF with new P */
            ctx->feat_a = best_neighbor;
            return;
        }

        /* New vertex is outside face -> find closest edge of F, go KK */
        ivp_real_t best_qdist = IVP_REAL_MAX;
        const ivp_compact_edge_t *best_f_edge = F;
        const ivp_compact_edge_t *fe = F;
        for (int j = 0; j < 3; j++, fe = ivp_edge_get_next(fe)) {
            if (qr2.checks[j] >= IVP_REAL(0.0)) continue;
            ivp_real_t qd = ivp_qdist_pk(ctx->ledge_b, fe, &new_p_os);
            if (qd < best_qdist) {
                best_qdist = qd;
                best_f_edge = fe;
            }
        }
        ctx->feat_a = best_neighbor;
        ctx->feat_b = best_f_edge;
        ctx->state = MD_STATE_KK;
        return;
    }

    /* Point is outside face -> find closest edge of F, go PK */
    ivp_real_t best_qdist = IVP_REAL_MAX;
    const ivp_compact_edge_t *best_edge = F;
    const ivp_compact_edge_t *fe = F;
    for (int j = 0; j < 3; j++, fe = ivp_edge_get_next(fe)) {
        ivp_real_t qd = ivp_qdist_pk(ctx->ledge_b, fe, &p_os);
        if (qd < best_qdist) {
            best_qdist = qd;
            best_edge = fe;
        }
    }

    ctx->feat_b = best_edge;
    ctx->state = MD_STATE_PK;
}

/* ── Point(A) vs Edge(B): PK step ──────────────────────────────────────── */

static void md_step_PK(md_ctx_t *ctx)
{
    const ivp_compact_edge_t *P = ctx->feat_a;
    const ivp_compact_edge_t *K = ctx->feat_b;

    ivp_vec3r_t p_os = md_point(ivp_edge_get_start_point(P, ctx->ledge_a));

    ivp_s_result_t sv;
    ivp_calc_s_val(ctx->ledge_b, K, &p_os, &sv);

    if (sv.checks[0] < IVP_REAL(0.0)) {
        /* P is before K's start -> PP(P, K_start) */
        ctx->state = MD_STATE_PP;
        return;
    }

    if (sv.checks[1] < IVP_REAL(0.0)) {
        /* P is past K's end -> PP(P, K_end) */
        ctx->feat_b = ivp_edge_get_next(K);
        ctx->state = MD_STATE_PP;
        return;
    }

    /* P projects inside K. Leave_PK:
     * Check the two faces adjacent to K. If P is inside either face, go PF. */
    const ivp_compact_edge_t *K_opp = ivp_edge_get_opposite(K);

    /* Check face of K */
    const ivp_compact_triangle_t *tri_K = ivp_edge_get_triangle(K);
    const ivp_compact_edge_t *F_K = &tri_K->edges[0];
    ivp_qr_result_t qr;
    ivp_calc_qr_vals(ctx->ledge_b, F_K, &p_os, &qr);
    if (!ivp_qr_is_outside(&qr)) {
        ctx->feat_b = F_K;
        ctx->state = MD_STATE_PF;
        return;
    }

    /* Check opposite face */
    const ivp_compact_triangle_t *tri_O = ivp_edge_get_triangle(K_opp);
    const ivp_compact_edge_t *F_O = &tri_O->edges[0];
    ivp_qr_result_t qr2;
    ivp_calc_qr_vals(ctx->ledge_b, F_O, &p_os, &qr2);
    if (!ivp_qr_is_outside(&qr2)) {
        ctx->feat_b = F_O;
        ctx->state = MD_STATE_PF;
        return;
    }

    /* P is in edge K's Voronoi region. Walk P's neighbors for improvement. */
    ivp_vec3r_t k0 = md_point(ivp_edge_get_start_point(K, ctx->ledge_b));
    ivp_vec3r_t k1 = md_point(ivp_edge_get_start_point(ivp_edge_get_next(K), ctx->ledge_b));

    ivp_real_t qd_P = ivp_qdist_pk(ctx->ledge_b, K, &p_os);
    ivp_real_t best_improvement = IVP_REAL(0.0);
    const ivp_compact_edge_t *best_neighbor = NULL;

    const ivp_compact_edge_t *Pm = ivp_edge_get_prev(P);
    const ivp_compact_edge_t *e = ivp_edge_get_prev(ivp_edge_get_opposite(Pm));
    const ivp_compact_edge_t *start_e = e;
    do {
        ivp_vec3r_t ne_pos = md_point(ivp_edge_get_start_point(e, ctx->ledge_a));
        ivp_s_result_t sv2;
        ivp_calc_s_val(ctx->ledge_b, K, &ne_pos, &sv2);

        ivp_real_t qd_ne;
        if (sv2.checks[0] >= IVP_REAL(0.0) && sv2.checks[1] >= IVP_REAL(0.0)) {
            qd_ne = ivp_qdist_point_to_line(ctx->ledge_b, K, &ne_pos);
        } else if (sv2.checks[0] < IVP_REAL(0.0)) {
            qd_ne = ivp_vec3r_quad_distance(&ne_pos, &k0);
        } else {
            qd_ne = ivp_vec3r_quad_distance(&ne_pos, &k1);
        }

        ivp_real_t improvement = qd_P - qd_ne;
        if (improvement > best_improvement) {
            best_improvement = improvement;
            best_neighbor = e;
        }
        e = ivp_edge_get_prev(ivp_edge_get_opposite(e));
    } while (e != start_e);

    if (best_neighbor) {
        /* Check if the neighbor projects onto K */
        ivp_vec3r_t ne_pos = md_point(ivp_edge_get_start_point(best_neighbor, ctx->ledge_a));
        ivp_s_result_t sv3;
        ivp_calc_s_val(ctx->ledge_b, K, &ne_pos, &sv3);

        if (sv3.checks[0] < IVP_REAL(0.0)) {
            ctx->feat_a = best_neighbor;
            ctx->state = MD_STATE_PP;
            return;
        }
        if (sv3.checks[1] < IVP_REAL(0.0)) {
            ctx->feat_a = best_neighbor;
            ctx->feat_b = ivp_edge_get_next(K);
            ctx->state = MD_STATE_PP;
            return;
        }
        /* Neighbor also projects onto K -> KK */
        ctx->feat_a = best_neighbor;
        ctx->state = MD_STATE_KK;
        return;
    }

    /* No improvement: P-K is the local minimum */
    ctx->type_a = IVP_FEATURE_POINT;
    ctx->type_b = IVP_FEATURE_EDGE;
    ctx->state = MD_STATE_DONE;
}

/* ── Point(A) vs Point(B): PP step ─────────────────────────────────────── */

static void md_step_PP(md_ctx_t *ctx)
{
    const ivp_compact_edge_t *A = ctx->feat_a;
    const ivp_compact_edge_t *B = ctx->feat_b;

    ivp_vec3r_t pos_A = md_point(ivp_edge_get_start_point(A, ctx->ledge_a));
    ivp_vec3r_t pos_B = md_point(ivp_edge_get_start_point(B, ctx->ledge_b));

    /* Check loop */
    if (md_loop_check(&ctx->loop_hash, MD_STATE_PP, A, B)) {
        ctx->type_a = IVP_FEATURE_POINT;
        ctx->type_b = IVP_FEATURE_POINT;
        ctx->state = MD_STATE_LOOP;
        return;
    }

    ivp_real_t cur_qdist = ivp_vec3r_quad_distance(&pos_A, &pos_B);

    ivp_real_t best_improvement = IVP_REAL(0.0);
    const ivp_compact_edge_t *best_edge = NULL;
    int best_side = -1; /* 0 = neighbor of B, 1 = neighbor of A */

    /* Check neighbors of B (distance to pos_A) */
    {
        const ivp_compact_edge_t *Bm = ivp_edge_get_prev(B);
        const ivp_compact_edge_t *e = ivp_edge_get_prev(ivp_edge_get_opposite(Bm));
        const ivp_compact_edge_t *start_e = e;
        do {
            ivp_vec3r_t ne_pos = md_point(ivp_edge_get_start_point(e, ctx->ledge_b));
            ivp_real_t qd = ivp_vec3r_quad_distance(&pos_A, &ne_pos);
            ivp_real_t improvement = cur_qdist - qd;
            if (improvement > best_improvement) {
                best_improvement = improvement;
                best_edge = e;
                best_side = 0;
            }
            e = ivp_edge_get_prev(ivp_edge_get_opposite(e));
        } while (e != start_e);
    }

    /* Check neighbors of A (distance to pos_B) */
    {
        const ivp_compact_edge_t *Am = ivp_edge_get_prev(A);
        const ivp_compact_edge_t *e = ivp_edge_get_prev(ivp_edge_get_opposite(Am));
        const ivp_compact_edge_t *start_e = e;
        do {
            ivp_vec3r_t ne_pos = md_point(ivp_edge_get_start_point(e, ctx->ledge_a));
            ivp_real_t qd = ivp_vec3r_quad_distance(&ne_pos, &pos_B);
            ivp_real_t improvement = cur_qdist - qd;
            if (improvement > best_improvement) {
                best_improvement = improvement;
                best_edge = e;
                best_side = 1;
            }
            e = ivp_edge_get_prev(ivp_edge_get_opposite(e));
        } while (e != start_e);
    }

    if (!best_edge) {
        /* No improvement: PP is the local minimum */
        ctx->type_a = IVP_FEATURE_POINT;
        ctx->type_b = IVP_FEATURE_POINT;
        ctx->state = MD_STATE_DONE;
        return;
    }

    if (best_side == 0) {
        /* Improved on B side. Check if A projects onto a connecting edge
         * from B toward best_edge -> transition to PK. */
        const ivp_compact_edge_t *connecting = NULL;
        {
            const ivp_compact_edge_t *Bm = ivp_edge_get_prev(B);
            const ivp_compact_edge_t *fe = ivp_edge_get_prev(ivp_edge_get_opposite(Bm));
            const ivp_compact_edge_t *start_fe = fe;
            do {
                if (ivp_edge_get_start_point_index(fe) ==
                    ivp_edge_get_start_point_index(best_edge)) {
                    connecting = ivp_edge_get_prev(fe);
                    break;
                }
                fe = ivp_edge_get_prev(ivp_edge_get_opposite(fe));
            } while (fe != start_fe);
        }

        if (connecting) {
            ivp_s_result_t sv;
            ivp_calc_s_val(ctx->ledge_b, connecting, &pos_A, &sv);
            if (sv.checks[0] >= IVP_REAL(0.0) && sv.checks[1] >= IVP_REAL(0.0)) {
                /* A projects inside the connecting edge -> PK */
                ctx->feat_b = connecting;
                ctx->state = MD_STATE_PK;
                return;
            }
        }
        /* Just move B to new vertex, stay PP */
        ctx->feat_b = best_edge;
    } else {
        /* Improved on A side. Symmetric. */
        const ivp_compact_edge_t *connecting = NULL;
        {
            const ivp_compact_edge_t *Am = ivp_edge_get_prev(A);
            const ivp_compact_edge_t *fe = ivp_edge_get_prev(ivp_edge_get_opposite(Am));
            const ivp_compact_edge_t *start_fe = fe;
            do {
                if (ivp_edge_get_start_point_index(fe) ==
                    ivp_edge_get_start_point_index(best_edge)) {
                    connecting = ivp_edge_get_prev(fe);
                    break;
                }
                fe = ivp_edge_get_prev(ivp_edge_get_opposite(fe));
            } while (fe != start_fe);
        }

        if (connecting) {
            ivp_s_result_t sv;
            ivp_calc_s_val(ctx->ledge_a, connecting, &pos_B, &sv);
            if (sv.checks[0] >= IVP_REAL(0.0) && sv.checks[1] >= IVP_REAL(0.0)) {
                /* B projects inside connecting edge on A -> swap and PK */
                const ivp_compact_ledge_t *tmp_l = ctx->ledge_a;
                ctx->ledge_a = ctx->ledge_b;
                ctx->ledge_b = tmp_l;
                ctx->flipped = !ctx->flipped;
                ctx->feat_a = B;
                ctx->feat_b = connecting;
                ctx->state = MD_STATE_PK;
                return;
            }
        }
        /* Just move A to new vertex, stay PP */
        ctx->feat_a = best_edge;
    }
}

/* ── Edge(A) vs Edge(B): KK step ──────────────────────────────────────── */

static void md_step_KK(md_ctx_t *ctx)
{
    const ivp_compact_edge_t *K = ctx->feat_a;
    const ivp_compact_edge_t *L = ctx->feat_b;

    /* Check loop */
    if (md_loop_check(&ctx->loop_hash, MD_STATE_KK, K, L)) {
        ctx->type_a = IVP_FEATURE_EDGE;
        ctx->type_b = IVP_FEATURE_EDGE;
        ctx->state = MD_STATE_LOOP;
        return;
    }

    /* Compute KK checks */
    ivp_kk_result_t kk;
    ivp_calc_kk_vals(ctx->ledge_a, K, ctx->ledge_b, L, &kk);

    bool outside_K = (kk.checks_k[0] < IVP_REAL(0.0)) || (kk.checks_k[1] < IVP_REAL(0.0));
    bool outside_L = (kk.checks_l[0] < IVP_REAL(0.0)) || (kk.checks_l[1] < IVP_REAL(0.0));

    if (!outside_L) {
        if (kk.checks_k[0] < IVP_REAL(0.0)) {
            /* Closest at K's start -> PK(K_start, L) */
            ctx->state = MD_STATE_PK;
            return;
        }
        if (kk.checks_k[1] < IVP_REAL(0.0)) {
            /* Closest at K's end -> PK(K_end, L) */
            ctx->feat_a = ivp_edge_get_next(K);
            ctx->state = MD_STATE_PK;
            return;
        }
    }

    if (!outside_K) {
        if (kk.checks_l[0] < IVP_REAL(0.0)) {
            /* Closest at L's start -> swap, PK(L_start, K) */
            const ivp_compact_ledge_t *tmp_l = ctx->ledge_a;
            ctx->ledge_a = ctx->ledge_b;
            ctx->ledge_b = tmp_l;
            ctx->flipped = !ctx->flipped;
            ctx->feat_a = L;
            ctx->feat_b = K;
            ctx->state = MD_STATE_PK;
            return;
        }
        if (kk.checks_l[1] < IVP_REAL(0.0)) {
            const ivp_compact_ledge_t *tmp_l = ctx->ledge_a;
            ctx->ledge_a = ctx->ledge_b;
            ctx->ledge_b = tmp_l;
            ctx->flipped = !ctx->flipped;
            ctx->feat_a = ivp_edge_get_next(L);
            ctx->feat_b = K;
            ctx->state = MD_STATE_PK;
            return;
        }
    }

    if (!outside_K && !outside_L) {
        /* Both inside: Leave_KK. Check 4 adjacent faces for gradient. */
        const ivp_compact_edge_t *K_opp = ivp_edge_get_opposite(K);
        const ivp_compact_edge_t *L_opp = ivp_edge_get_opposite(L);

        const ivp_compact_edge_t *faces[4];
        int face_on_a[4]; /* 1 = face on ledge_a, 0 = face on ledge_b */
        faces[0] = &ivp_edge_get_triangle(K)->edges[0];     face_on_a[0] = 1;
        faces[1] = &ivp_edge_get_triangle(K_opp)->edges[0]; face_on_a[1] = 1;
        faces[2] = &ivp_edge_get_triangle(L)->edges[0];     face_on_a[2] = 0;
        faces[3] = &ivp_edge_get_triangle(L_opp)->edges[0]; face_on_a[3] = 0;

        /* Compute midpoints of each edge for gradient evaluation */
        ivp_vec3r_t k0 = md_point(ivp_edge_get_start_point(K, ctx->ledge_a));
        ivp_vec3r_t k1 = md_point(ivp_edge_get_start_point(ivp_edge_get_next(K), ctx->ledge_a));
        ivp_vec3r_t l0 = md_point(ivp_edge_get_start_point(L, ctx->ledge_b));
        ivp_vec3r_t l1 = md_point(ivp_edge_get_start_point(ivp_edge_get_next(L), ctx->ledge_b));

        ivp_vec3r_t k_mid, l_mid;
        ivp_vec3r_lerp(&k_mid, &k0, &k1, IVP_REAL(0.5));
        ivp_vec3r_lerp(&l_mid, &l0, &l1, IVP_REAL(0.5));

        ivp_real_t best_grad = IVP_REAL(0.0);
        const ivp_compact_edge_t *best_face = NULL;
        int best_face_on_a = 0;

        for (int fi = 0; fi < 4; fi++) {
            const ivp_compact_ledge_t *face_ledge = face_on_a[fi]
                ? ctx->ledge_a : ctx->ledge_b;

            ivp_vec3r_t hesse;
            ivp_calc_hesse_vec(face_ledge, faces[fi], &hesse);
            ivp_real_t hesse_sq = ivp_vec3r_quad_length(&hesse);
            if (hesse_sq < IVP_REAL_RES * IVP_REAL_RES) continue;

            /* Test point: midpoint of the OTHER edge */
            const ivp_vec3r_t *test_pt = face_on_a[fi] ? &l_mid : &k_mid;

            ivp_qr_result_t qr;
            ivp_calc_qr_vals(face_ledge, faces[fi], test_pt, &qr);
            if (!ivp_qr_is_outside(&qr)) {
                /* Test point is inside this face. Check plane distance
                 * gradient: signed_dist < 0 means improvement. */
                ivp_real_t sdist = ivp_signed_dist_to_plane(face_ledge, faces[fi], test_pt);
                if (sdist < best_grad) {
                    best_grad = sdist;
                    best_face = faces[fi];
                    best_face_on_a = face_on_a[fi];
                }
            }
        }

        if (!best_face) {
            /* KK is the local minimum */
            ctx->type_a = IVP_FEATURE_EDGE;
            ctx->type_b = IVP_FEATURE_EDGE;
            ctx->state = MD_STATE_DONE;
            return;
        }

        /* Transition to PF with the found face.
         * The "point" is an endpoint of the other edge. */
        if (best_face_on_a) {
            /* Face on A, point is from L (on B). Swap so A=point, B=face. */
            const ivp_compact_ledge_t *tmp_l = ctx->ledge_a;
            ctx->ledge_a = ctx->ledge_b;
            ctx->ledge_b = tmp_l;
            ctx->flipped = !ctx->flipped;
            ctx->feat_a = L;      /* point on old ledge_b */
            ctx->feat_b = best_face; /* face on old ledge_a */
        } else {
            /* Face on B, point is from K (on A). Normal orientation. */
            ctx->feat_a = K;
            ctx->feat_b = best_face;
        }
        ctx->state = MD_STATE_PF;
        return;
    }

    /* Both outside -> transition via nearest endpoints */
    const ivp_compact_edge_t *pk, *pl;
    if (kk.checks_k[0] < kk.checks_k[1]) {
        pk = K;
    } else {
        pk = ivp_edge_get_next(K);
    }
    if (kk.checks_l[0] < kk.checks_l[1]) {
        pl = L;
    } else {
        pl = ivp_edge_get_next(L);
    }

    /* Check if pk projects onto L -> PK */
    ivp_vec3r_t pk_pos = md_point(ivp_edge_get_start_point(pk, ctx->ledge_a));
    ivp_s_result_t sv;
    ivp_calc_s_val(ctx->ledge_b, L, &pk_pos, &sv);
    if (sv.checks[0] >= IVP_REAL(0.0) && sv.checks[1] >= IVP_REAL(0.0)) {
        ctx->feat_a = pk;
        ctx->feat_b = L;
        ctx->state = MD_STATE_PK;
        return;
    }

    /* Check if pl projects onto K -> swap and PK */
    ivp_vec3r_t pl_pos = md_point(ivp_edge_get_start_point(pl, ctx->ledge_b));
    ivp_s_result_t sv2;
    ivp_calc_s_val(ctx->ledge_a, K, &pl_pos, &sv2);
    if (sv2.checks[0] >= IVP_REAL(0.0) && sv2.checks[1] >= IVP_REAL(0.0)) {
        const ivp_compact_ledge_t *tmp_l = ctx->ledge_a;
        ctx->ledge_a = ctx->ledge_b;
        ctx->ledge_b = tmp_l;
        ctx->flipped = !ctx->flipped;
        ctx->feat_a = pl;
        ctx->feat_b = K;
        ctx->state = MD_STATE_PK;
        return;
    }

    /* PP with nearest endpoints */
    ctx->feat_a = pk;
    ctx->feat_b = pl;
    ctx->state = MD_STATE_PP;
}

/* ── Compute final result from converged features ──────────────────────── */

static void md_compute_result(const md_ctx_t *ctx, ivp_closest_pair_t *result)
{
    result->type_a = ctx->type_a;
    result->type_b = ctx->type_b;
    result->edge_a = ctx->feat_a;
    result->edge_b = ctx->feat_b;

    if (ctx->type_a == IVP_FEATURE_POINT && ctx->type_b == IVP_FEATURE_POINT) {
        result->point_a = md_point(ivp_edge_get_start_point(ctx->feat_a, ctx->ledge_a));
        result->point_b = md_point(ivp_edge_get_start_point(ctx->feat_b, ctx->ledge_b));
    }
    else if (ctx->type_a == IVP_FEATURE_POINT && ctx->type_b == IVP_FEATURE_EDGE) {
        ivp_vec3r_t p = md_point(ivp_edge_get_start_point(ctx->feat_a, ctx->ledge_a));
        ivp_vec3r_t e0 = md_point(ivp_edge_get_start_point(ctx->feat_b, ctx->ledge_b));
        ivp_vec3r_t e1 = md_point(ivp_edge_get_start_point(
            ivp_edge_get_next(ctx->feat_b), ctx->ledge_b));

        result->point_a = p;

        ivp_vec3r_t d, dp;
        ivp_vec3r_sub(&d, &e1, &e0);
        ivp_vec3r_sub(&dp, &p, &e0);
        ivp_real_t denom = ivp_vec3r_dot(&d, &d);
        ivp_real_t t = IVP_REAL(0.0);
        if (denom > IVP_REAL_RES * IVP_REAL_RES) {
            t = ivp_vec3r_dot(&dp, &d) / denom;
            if (t < IVP_REAL(0.0)) t = IVP_REAL(0.0);
            if (t > IVP_REAL(1.0)) t = IVP_REAL(1.0);
        }
        ivp_vec3r_lerp(&result->point_b, &e0, &e1, t);
    }
    else if (ctx->type_a == IVP_FEATURE_POINT && ctx->type_b == IVP_FEATURE_TRIANGLE) {
        ivp_vec3r_t p = md_point(ivp_edge_get_start_point(ctx->feat_a, ctx->ledge_a));
        result->point_a = p;

        const ivp_compact_triangle_t *tri = ivp_edge_get_triangle(ctx->feat_b);
        ivp_vec3r_t v0 = md_point(ivp_edge_get_start_point(&tri->edges[0], ctx->ledge_b));
        ivp_vec3r_t v1 = md_point(ivp_edge_get_start_point(&tri->edges[1], ctx->ledge_b));
        ivp_vec3r_t v2 = md_point(ivp_edge_get_start_point(&tri->edges[2], ctx->ledge_b));

        ivp_vec3r_t e0, e1, v;
        ivp_vec3r_sub(&e0, &v1, &v0);
        ivp_vec3r_sub(&e1, &v2, &v0);
        ivp_vec3r_sub(&v, &p, &v0);

        ivp_real_t d00 = ivp_vec3r_dot(&e0, &e0);
        ivp_real_t d01 = ivp_vec3r_dot(&e0, &e1);
        ivp_real_t d11 = ivp_vec3r_dot(&e1, &e1);
        ivp_real_t d20 = ivp_vec3r_dot(&v, &e0);
        ivp_real_t d21 = ivp_vec3r_dot(&v, &e1);

        ivp_real_t det = d00 * d11 - d01 * d01;
        if (det > IVP_REAL_RES * IVP_REAL_RES) {
            ivp_real_t inv = IVP_REAL(1.0) / det;
            ivp_real_t u = (d11 * d20 - d01 * d21) * inv;
            ivp_real_t w = (d00 * d21 - d01 * d20) * inv;
            result->point_b.k[0] = v0.k[0] + u * e0.k[0] + w * e1.k[0];
            result->point_b.k[1] = v0.k[1] + u * e0.k[1] + w * e1.k[1];
            result->point_b.k[2] = v0.k[2] + u * e0.k[2] + w * e1.k[2];
        } else {
            result->point_b = v0;
        }
    }
    else if (ctx->type_a == IVP_FEATURE_EDGE && ctx->type_b == IVP_FEATURE_EDGE) {
        ivp_vec3r_t a0 = md_point(ivp_edge_get_start_point(ctx->feat_a, ctx->ledge_a));
        ivp_vec3r_t a1 = md_point(ivp_edge_get_start_point(
            ivp_edge_get_next(ctx->feat_a), ctx->ledge_a));
        ivp_vec3r_t b0 = md_point(ivp_edge_get_start_point(ctx->feat_b, ctx->ledge_b));
        ivp_vec3r_t b1 = md_point(ivp_edge_get_start_point(
            ivp_edge_get_next(ctx->feat_b), ctx->ledge_b));

        ivp_vec3r_t d_a, d_b, d_ab;
        ivp_vec3r_sub(&d_a, &a1, &a0);
        ivp_vec3r_sub(&d_b, &b1, &b0);
        ivp_vec3r_sub(&d_ab, &a0, &b0);

        ivp_real_t a = ivp_vec3r_dot(&d_a, &d_a);
        ivp_real_t e = ivp_vec3r_dot(&d_b, &d_b);
        ivp_real_t f = ivp_vec3r_dot(&d_b, &d_ab);
        ivp_real_t b = ivp_vec3r_dot(&d_a, &d_b);
        ivp_real_t c = ivp_vec3r_dot(&d_a, &d_ab);

        ivp_real_t denom = a * e - b * b;
        ivp_real_t s, t;

        if (denom > IVP_REAL_RES * IVP_REAL_RES) {
            s = ivp_clamp((b * f - c * e) / denom, IVP_REAL(0.0), IVP_REAL(1.0));
        } else {
            s = IVP_REAL(0.0);
        }
        t = (e > IVP_REAL_RES * IVP_REAL_RES) ? (b * s + f) / e : IVP_REAL(0.0);

        if (t < IVP_REAL(0.0)) {
            t = IVP_REAL(0.0);
            s = (a > IVP_REAL_RES * IVP_REAL_RES)
              ? ivp_clamp(-c / a, IVP_REAL(0.0), IVP_REAL(1.0))
              : IVP_REAL(0.0);
        } else if (t > IVP_REAL(1.0)) {
            t = IVP_REAL(1.0);
            s = (a > IVP_REAL_RES * IVP_REAL_RES)
              ? ivp_clamp((b - c) / a, IVP_REAL(0.0), IVP_REAL(1.0))
              : IVP_REAL(0.0);
        }

        ivp_vec3r_lerp(&result->point_a, &a0, &a1, s);
        ivp_vec3r_lerp(&result->point_b, &b0, &b1, t);
    }
    else {
        /* Fallback: feature types not fully identified */
        result->point_a = md_point(ivp_edge_get_start_point(ctx->feat_a, ctx->ledge_a));
        result->point_b = md_point(ivp_edge_get_start_point(ctx->feat_b, ctx->ledge_b));
    }

    if (ctx->flipped) {
        ivp_feature_type_t tmp_type = result->type_a;
        const ivp_compact_edge_t *tmp_edge = result->edge_a;
        ivp_vec3r_t tmp_point = result->point_a;

        result->type_a = result->type_b;
        result->type_b = tmp_type;
        result->edge_a = result->edge_b;
        result->edge_b = tmp_edge;
        result->point_a = result->point_b;
        result->point_b = tmp_point;
    }

    /* Compute signed distance and stable A→B normal. */
    ivp_vec3r_t center_a = md_ledge_centroid(ctx->ledge_a);
    ivp_vec3r_t center_b = md_ledge_centroid(ctx->ledge_b);
    ivp_vec3r_t center_diff;
    ivp_vec3r_sub(&center_diff, &center_b, &center_a);

    ivp_vec3r_t surf_diff;
    ivp_vec3r_sub(&surf_diff, &result->point_b, &result->point_a);

    result->qdistance = ivp_vec3r_quad_distance(&result->point_a, &result->point_b);
    if (result->qdistance > IVP_REAL_RES * IVP_REAL_RES) {
        ivp_real_t unsigned_dist = ivp_sqrt(result->qdistance);
        ivp_real_t inv_dist = IVP_REAL(1.0) / unsigned_dist;
        result->normal.k[0] = surf_diff.k[0] * inv_dist;
        result->normal.k[1] = surf_diff.k[1] * inv_dist;
        result->normal.k[2] = surf_diff.k[2] * inv_dist;

        ivp_real_t agreement = ivp_vec3r_dot(&surf_diff, &center_diff);
        if (agreement < IVP_REAL(0.0)) {
            result->normal.k[0] = -result->normal.k[0];
            result->normal.k[1] = -result->normal.k[1];
            result->normal.k[2] = -result->normal.k[2];
            result->distance = -unsigned_dist;
        } else {
            result->distance = unsigned_dist;
        }
    } else {
        result->distance = IVP_REAL(0.0);

        ivp_real_t center_len = ivp_vec3r_length(&center_diff);
        if (center_len > IVP_REAL_RES) {
            ivp_real_t inv_len = IVP_REAL(1.0) / center_len;
            result->normal.k[0] = center_diff.k[0] * inv_len;
            result->normal.k[1] = center_diff.k[1] * inv_len;
            result->normal.k[2] = center_diff.k[2] * inv_len;
        } else {
            ivp_vec3r_set(&result->normal, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        }
    }
}

static bool md_is_backside_result(const md_ctx_t *ctx, const ivp_closest_pair_t *result)
{
    const ivp_real_t eps = IVP_REAL(1e-6);

    if (result->type_a == IVP_FEATURE_POINT && result->type_b == IVP_FEATURE_TRIANGLE) {
        ivp_real_t signed_dist = ivp_signed_dist_to_plane(ctx->ledge_b,
                                                           result->edge_b,
                                                           &result->point_a);
        return signed_dist < -eps;
    }

    if (result->type_a == IVP_FEATURE_TRIANGLE && result->type_b == IVP_FEATURE_POINT) {
        ivp_real_t signed_dist = ivp_signed_dist_to_plane(ctx->ledge_a,
                                                           result->edge_a,
                                                           &result->point_b);
        return signed_dist < -eps;
    }

    return false;
}

/* ── Pick initial feature pair ─────────────────────────────────────────── */

/* Find the vertex in `ledge` that is closest to `target`.
 * Returns the edge whose start point is that vertex. */
static const ivp_compact_edge_t *md_find_nearest_vertex(
    const ivp_compact_ledge_t *ledge,
    const ivp_vec3r_t *target)
{
    const ivp_compact_triangle_t *tri0 = ivp_ledge_get_first_triangle(ledge);
    const ivp_compact_edge_t *best_edge = &tri0->edges[0];
    ivp_real_t best_qdist = IVP_REAL_MAX;

    int n_pts = ivp_ledge_get_n_points(ledge);
    const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(ledge);

    /* Track which point indices we've already checked to avoid redundant work.
     * We scan all triangles' edges and pick the best vertex. */
    int n_tri = ivp_ledge_get_n_triangles(ledge);
    for (int t = 0; t < n_tri; t++) {
        for (int e = 0; e < 3; e++) {
            const ivp_compact_edge_t *edge = &tri0[t].edges[e];
            int pidx = ivp_edge_get_start_point_index(edge);
            if (pidx < 0 || pidx >= n_pts) continue;
            ivp_vec3r_t pos = md_point(&pts[pidx]);
            ivp_real_t qd = ivp_vec3r_quad_distance(&pos, target);
            if (qd < best_qdist) {
                best_qdist = qd;
                best_edge = edge;
            }
        }
    }
    return best_edge;
}

static void md_pick_initial(md_ctx_t *ctx)
{
    /* Compute centroids for both ledges to use as proximity targets.
     * Starting hill-climbing from vertices nearest to the opposing centroid
     * prevents convergence to local minima for mismatched-size objects. */
    ivp_vec3r_t centroid_a = {{IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0)}};
    ivp_vec3r_t centroid_b = {{IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0)}};

    int n_pts_a = ivp_ledge_get_n_points(ctx->ledge_a);
    const ivp_compact_poly_point_t *pts_a = ivp_ledge_get_point_array(ctx->ledge_a);
    for (int i = 0; i < n_pts_a; i++) {
        centroid_a.k[0] += (ivp_real_t)pts_a[i].k[0];
        centroid_a.k[1] += (ivp_real_t)pts_a[i].k[1];
        centroid_a.k[2] += (ivp_real_t)pts_a[i].k[2];
    }
    if (n_pts_a > 0) {
        ivp_real_t inv = IVP_REAL(1.0) / (ivp_real_t)n_pts_a;
        centroid_a.k[0] *= inv;
        centroid_a.k[1] *= inv;
        centroid_a.k[2] *= inv;
    }

    int n_pts_b = ivp_ledge_get_n_points(ctx->ledge_b);
    const ivp_compact_poly_point_t *pts_b = ivp_ledge_get_point_array(ctx->ledge_b);
    for (int i = 0; i < n_pts_b; i++) {
        centroid_b.k[0] += (ivp_real_t)pts_b[i].k[0];
        centroid_b.k[1] += (ivp_real_t)pts_b[i].k[1];
        centroid_b.k[2] += (ivp_real_t)pts_b[i].k[2];
    }
    if (n_pts_b > 0) {
        ivp_real_t inv = IVP_REAL(1.0) / (ivp_real_t)n_pts_b;
        centroid_b.k[0] *= inv;
        centroid_b.k[1] *= inv;
        centroid_b.k[2] *= inv;
    }

    /* Start vertex A = vertex of ledge_a closest to centroid_b.
     * Start vertex B = vertex of ledge_b closest to centroid_a. */
    ctx->feat_a = md_find_nearest_vertex(ctx->ledge_a, &centroid_b);
    ctx->feat_b = md_find_nearest_vertex(ctx->ledge_b, &centroid_a);
    ctx->state = MD_STATE_PP;
    ctx->type_a = IVP_FEATURE_POINT;
    ctx->type_b = IVP_FEATURE_POINT;
}

/* ── Main implementation ───────────────────────────────────────────────── */

ivp_result_t ivp_find_closest_pair(
    const ivp_compact_ledge_t *ledge_a,
    const ivp_compact_ledge_t *ledge_b,
    ivp_closest_pair_t *result)
{
    if (!ledge_a || !ledge_b || !result) return IVP_ERR_INVALID_ARG;

    md_ctx_t ctx;
    ctx.ledge_a = ledge_a;
    ctx.ledge_b = ledge_b;
    ctx.iterations = 0;
    ctx.flipped = false;
    md_loop_init(&ctx.loop_hash);

    md_pick_initial(&ctx);

    /* Run the state machine */
    while (ctx.state != MD_STATE_DONE && ctx.state != MD_STATE_LOOP
           && ctx.iterations < MD_MAX_ITERATIONS)
    {
        ctx.iterations++;
        switch (ctx.state) {
        case MD_STATE_PF: md_step_PF(&ctx); break;
        case MD_STATE_PK: md_step_PK(&ctx); break;
        case MD_STATE_PP: md_step_PP(&ctx); break;
        case MD_STATE_KK: md_step_KK(&ctx); break;
        default: ctx.state = MD_STATE_DONE; break;
        }
    }

    /* Compute final closest points from the converged features */
    md_compute_result(&ctx, result);

    if (md_is_backside_result(&ctx, result)) {
        result->status = IVP_MINDIST_STATUS_BACKSIDE;
    } else if (ctx.state == MD_STATE_LOOP) {
        result->status = IVP_MINDIST_STATUS_LOOP;
    } else if (ctx.iterations >= MD_MAX_ITERATIONS && ctx.state != MD_STATE_DONE) {
        result->status = IVP_MINDIST_STATUS_MAX_ITER;
    } else {
        result->status = IVP_MINDIST_STATUS_OK;
    }

    return IVP_OK;
}

/* ── Convenience: squared distance ─────────────────────────────────────── */

ivp_real_t ivp_qdist_ledge_ledge(
    const ivp_compact_ledge_t *ledge_a,
    const ivp_compact_ledge_t *ledge_b)
{
    ivp_closest_pair_t pair;
    if (ivp_find_closest_pair(ledge_a, ledge_b, &pair) != IVP_OK) {
        return IVP_REAL_MAX;
    }
    return pair.qdistance;
}
