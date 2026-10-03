/* lcs_parity_gen.h — deterministic LCS / Gauss problem generator
 *
 * Shared by tests/test_lcs_parity.c and the C++ reference harness
 * tests/data/lcs_ref_harness.cxx that produced tests/data/lcs_cases.h (it
 * links the reference IVP library and
 * runs IVP_Linear_Constraint_Solver::init_and_solve_lc,
 * IVP_Great_Matrix_Many_Zero::solve_great_matrix_many_zero and ::invert on
 * the same inputs).  Inputs are regenerated from (family, seed, n) instead
 * of being stored; every product goes through a volatile temporary so no
 * compiler can contract it into an FMA, and only correctly rounded
 * operations (+ - * / sqrt, int->double) are used, so both sides produce
 * identical bits.  An FNV-1a hash of the generated inputs is stored with
 * each case and checked before the outputs are compared.
 *
 * Must compile as C17 and C++17.
 */
#ifndef LCS_PARITY_GEN_H
#define LCS_PARITY_GEN_H

#include <stdint.h>
#include <string.h>
#include <math.h>

#define LCSGEN_MAX_N 40

enum {
    LCSGEN_SPD_DENSE = 0,   /* B*B^T + delta*I, full rank                */
    LCSGEN_SPD_LOWRANK,     /* B*B^T with rank < n (singular PSD)        */
    LCSGEN_SPD_SPARSE,      /* sparse B: many exact zeros                */
    LCSGEN_BOX_STACK,       /* contact matrix of a stack of boxes        */
    LCSGEN_BOX_STACK_NORM,  /* same, normalised like normize_constraint_equ */
    LCSGEN_DUP_ROWS,        /* duplicated contacts: exactly singular     */
    LCSGEN_ILL,             /* eigen-spread over ~2^40                   */
    LCSGEN_DIAG_DOM,        /* well conditioned                          */
    LCSGEN_NONSYM,          /* non-symmetric                             */
    LCSGEN_SCALED,          /* SPD scaled by 2^s, s in [-30, 30]         */
    LCSGEN_N_FAMILIES
};

typedef struct lcs_parity_case {
    int      family;
    uint32_t seed;
    int      n;
    int      actives_at_begin;
    int      lcs_ok;        /* reference init_and_solve_lc returned IVP_OK  */
    int      gauss_ok;      /* reference solve_great_matrix_many_zero OK    */
    uint64_t input_hash;
    int      bits_offset;   /* n LCS x values, then n Gauss result values   */
    uint64_t misc_hash;     /* lcsgen_misc_hash of the auxiliary outputs    */
} lcs_parity_case_t;

typedef struct lcs_parity_inv_case {
    int      family;
    uint32_t seed;
    int      n;
    int      inv_ok;        /* reference invert() returned IVP_OK           */
    uint64_t input_hash;
    int      bits_offset;   /* n*n inverse values (stride n)                */
} lcs_parity_inv_case_t;

typedef struct lcsgen_rng {
    uint64_t s;
} lcsgen_rng_t;

static uint64_t lcsgen_next(lcsgen_rng_t *r)
{
    uint64_t z = (r->s += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

/* products through a volatile temporary: never fused into an FMA */
static double lcsgen_mul(double a, double b)
{
    volatile double t = a * b;
    return t;
}

/* exact: 53-bit integer times 2^-53, in [0, 1) */
static double lcsgen_unit(lcsgen_rng_t *r)
{
    return (double)(lcsgen_next(r) >> 11) * (1.0 / 9007199254740992.0);
}

/* exact, in [-1, 1) */
static double lcsgen_sym(lcsgen_rng_t *r)
{
    return lcsgen_mul(2.0, lcsgen_unit(r)) - 1.0;
}

static int lcsgen_int(lcsgen_rng_t *r, int lo, int hi)
{
    return lo + (int)(lcsgen_next(r) % (uint64_t)(hi - lo + 1));
}

static double lcsgen_pow2(int e)
{
    double v = 1.0;
    while (e > 0) { v = lcsgen_mul(v, 2.0); e--; }
    while (e < 0) { v = lcsgen_mul(v, 0.5); e++; }
    return v;
}

static uint64_t lcsgen_hash_bits(uint64_t h, const double *v, int count)
{
    for (int i = 0; i < count; i++) {
        uint64_t bits;
        memcpy(&bits, &v[i], sizeof(bits));
        for (int k = 0; k < 8; k++) {
            h ^= (bits >> (8 * k)) & 0xffu;
            h *= 0x100000001b3ULL;
        }
    }
    return h;
}

/* Auxiliary outputs folded into lcs_parity_case_t.misc_hash, in order:
 *   1. transform_to_lower_null_triangle + out-of-line solve_lower_null_matrix
 *      on (A, b): (double)ok, then result[0..n-1]
 *   2. mult_aligned (stride aligned_row_len) of A * b: result[0..n-1]
 *   3. mult (stride n) of A * b: result[0..n-1]
 *   4. matrix_check_unequation_line(i), i < n, with result_vector = the Gauss
 *      solution and desired_vector = b: (double)ok per line            */
static uint64_t lcsgen_misc_hash_add(uint64_t h, double v)
{
    return lcsgen_hash_bits(h, &v, 1);
}

static uint64_t lcsgen_input_hash(const double *A, const double *b, int n)
{
    uint64_t h = 0xcbf29ce484222325ULL;
    h = lcsgen_hash_bits(h, A, n * n);
    h = lcsgen_hash_bits(h, b, n);
    return h;
}

/* A = sum_t lambda_t * B[i][t] * B[j][t] (+ delta on the diagonal),
 * computed for j >= i and mirrored, so A is exactly symmetric. */
static void lcsgen_gram(lcsgen_rng_t *r, int n, int k, int sparse_pct,
                        int lambda_spread, double delta, double *A)
{
    double B[LCSGEN_MAX_N * LCSGEN_MAX_N];
    double lam[LCSGEN_MAX_N];
    for (int i = 0; i < n; i++) {
        for (int t = 0; t < k; t++) {
            double v = lcsgen_sym(r);
            if (sparse_pct > 0 && lcsgen_int(r, 0, 99) < sparse_pct) v = 0.0;
            B[i * LCSGEN_MAX_N + t] = v;
        }
    }
    for (int t = 0; t < k; t++) {
        lam[t] = lambda_spread > 0
                     ? lcsgen_mul(1.0 + lcsgen_unit(r), lcsgen_pow2(-lcsgen_int(r, 0, lambda_spread)))
                     : 1.0;
    }
    for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) {
            double sum = 0.0;
            for (int t = 0; t < k; t++) {
                double p = lcsgen_mul(lcsgen_mul(B[i * LCSGEN_MAX_N + t], lam[t]),
                                      B[j * LCSGEN_MAX_N + t]);
                sum = sum + p;
            }
            if (i == j) sum = sum + delta;
            A[i * n + j] = sum;
            A[j * n + i] = sum;
        }
    }
}

/* b: entries in [lo, hi) with probability pos_pct% positive */
static void lcsgen_rhs(lcsgen_rng_t *r, int n, int pos_pct, double scale, double *b)
{
    for (int i = 0; i < n; i++) {
        double u = lcsgen_unit(r);
        double v = (lcsgen_int(r, 0, 99) < pos_pct) ? (0.01 + u) : -(0.01 + u);
        b[i] = lcsgen_mul(v, scale);
    }
}

static void lcsgen_cross(const double *a, const double *c, double *out)
{
    out[0] = lcsgen_mul(a[1], c[2]) - lcsgen_mul(a[2], c[1]);
    out[1] = lcsgen_mul(a[2], c[0]) - lcsgen_mul(a[0], c[2]);
    out[2] = lcsgen_mul(a[0], c[1]) - lcsgen_mul(a[1], c[0]);
}

/* Stack of ceil(n/4) boxes, four corner contacts under each box (the
 * bottom box rests on static ground); A = J W J^T for the normal rows. */
static void lcsgen_box_stack(lcsgen_rng_t *r, int n, int normalize, double *A, double *b)
{
    enum { MAXB = (LCSGEN_MAX_N + 3) / 4 };
    double inv_m[MAXB], inv_i[MAXB][3], cen[MAXB][3], hx[MAXB], hy[MAXB], hz[MAXB];
    double J[LCSGEN_MAX_N][2][6];
    int body[LCSGEN_MAX_N][2];
    int nb = (n + 3) / 4;
    int exact_flat = lcsgen_int(r, 0, 2) == 0; /* exactly coplanar contacts */
    double y = 0.0;

    for (int k = 0; k < nb; k++) {
        hx[k] = 0.25 + lcsgen_mul(0.75, lcsgen_unit(r));
        hy[k] = 0.25 + lcsgen_mul(0.75, lcsgen_unit(r));
        hz[k] = 0.25 + lcsgen_mul(0.75, lcsgen_unit(r));
        double m = lcsgen_mul(1.0 + lcsgen_unit(r), lcsgen_pow2(lcsgen_int(r, -4, 4)));
        inv_m[k] = 1.0 / m;
        double m3 = m / 3.0;
        double xx = lcsgen_mul(hx[k], hx[k]);
        double yy = lcsgen_mul(hy[k], hy[k]);
        double zz = lcsgen_mul(hz[k], hz[k]);
        inv_i[k][0] = 1.0 / lcsgen_mul(m3, yy + zz);
        inv_i[k][1] = 1.0 / lcsgen_mul(m3, xx + zz);
        inv_i[k][2] = 1.0 / lcsgen_mul(m3, xx + yy);
        cen[k][0] = lcsgen_mul(0.1, lcsgen_sym(r));
        cen[k][1] = y + hy[k];
        cen[k][2] = lcsgen_mul(0.1, lcsgen_sym(r));
        y = y + lcsgen_mul(2.0, hy[k]);
    }

    for (int c = 0; c < n; c++) {
        int k = c / 4;
        int corner = c % 4;
        double sx = (corner & 1) ? 0.9 : -0.9;
        double sz = (corner & 2) ? 0.9 : -0.9;
        double p[3];
        p[0] = cen[k][0] + lcsgen_mul(sx, hx[k]);
        p[1] = cen[k][1] - hy[k];
        p[2] = cen[k][2] + lcsgen_mul(sz, hz[k]);
        double nrm[3];
        if (exact_flat) {
            nrm[0] = 0.0; nrm[1] = 1.0; nrm[2] = 0.0;
        } else {
            double tx = lcsgen_mul(0.05, lcsgen_sym(r));
            double tz = lcsgen_mul(0.05, lcsgen_sym(r));
            double len = sqrt(lcsgen_mul(tx, tx) + 1.0 + lcsgen_mul(tz, tz));
            nrm[0] = tx / len; nrm[1] = 1.0 / len; nrm[2] = tz / len;
        }
        double ru[3], ang[3];
        for (int a = 0; a < 3; a++) ru[a] = p[a] - cen[k][a];
        lcsgen_cross(ru, nrm, ang);
        body[c][0] = k;
        for (int a = 0; a < 3; a++) { J[c][0][a] = nrm[a]; J[c][0][3 + a] = ang[a]; }
        if (k > 0) {
            double rl[3];
            for (int a = 0; a < 3; a++) rl[a] = p[a] - cen[k - 1][a];
            lcsgen_cross(rl, nrm, ang);
            body[c][1] = k - 1;
            for (int a = 0; a < 3; a++) { J[c][1][a] = -nrm[a]; J[c][1][3 + a] = -ang[a]; }
        } else {
            body[c][1] = -1;
        }
    }

    for (int c = 0; c < n; c++) {
        for (int d = c; d < n; d++) {
            double sum = 0.0;
            for (int bc = 0; bc < 2; bc++) {
                int kb = body[c][bc];
                if (kb < 0) continue;
                for (int bd = 0; bd < 2; bd++) {
                    if (body[d][bd] != kb) continue;
                    for (int a = 0; a < 3; a++) {
                        sum = sum + lcsgen_mul(lcsgen_mul(J[c][bc][a], inv_m[kb]), J[d][bd][a]);
                    }
                    for (int a = 0; a < 3; a++) {
                        sum = sum + lcsgen_mul(lcsgen_mul(J[c][bc][3 + a], inv_i[kb][a]),
                                               J[d][bd][3 + a]);
                    }
                }
            }
            A[c * n + d] = sum;
            A[d * n + c] = sum;
        }
    }

    lcsgen_rhs(r, n, 80, 1.0, b);

    if (normalize) {
        double max_val = 0.0;
        for (int i = n - 1; i >= 0; i--) {
            if (A[i * n + i] > max_val) max_val = A[i * n + i];
        }
        double a_norm = max_val > 1e-10 ? 1.0 / max_val : 1.0;
        max_val = 0.0;
        for (int i = n - 1; i >= 0; i--) {
            if (fabs(b[i]) > max_val) max_val = fabs(b[i]);
        }
        double b_norm = max_val > 1e-10 ? 1.0 / max_val : 1.0;
        for (int i = 0; i < n * n; i++) A[i] = lcsgen_mul(A[i], a_norm);
        for (int i = 0; i < n; i++) b[i] = lcsgen_mul(b[i], b_norm);
    }
}

/* Fill A (n x n, stride n) and b (n) for (family, seed, n). */
static void lcsgen_make(int family, uint32_t seed, int n, double *A, double *b)
{
    lcsgen_rng_t r;
    r.s = (uint64_t)seed * 0x2545F4914F6CDD1DULL + (uint64_t)family * 1000003ULL + (uint64_t)n;
    (void)lcsgen_next(&r);

    switch (family) {
    case LCSGEN_SPD_DENSE: {
        int dsel = lcsgen_int(&r, 0, 2);
        double delta = dsel == 0 ? 0.0 : (dsel == 1 ? 0.0009765625 : 1.0);
        lcsgen_gram(&r, n, n, 0, 0, delta, A);
        lcsgen_rhs(&r, n, lcsgen_int(&r, 10, 90), 1.0, b);
        break;
    }
    case LCSGEN_SPD_LOWRANK: {
        int k = n > 1 ? lcsgen_int(&r, 1, n - 1) : 1;
        lcsgen_gram(&r, n, k, 0, 0, 0.0, A);
        lcsgen_rhs(&r, n, lcsgen_int(&r, 30, 95), 1.0, b);
        break;
    }
    case LCSGEN_SPD_SPARSE:
        lcsgen_gram(&r, n, n, 75, 0, 0.0001220703125, A);
        lcsgen_rhs(&r, n, lcsgen_int(&r, 5, 60), 1.0, b);
        break;
    case LCSGEN_BOX_STACK:
        lcsgen_box_stack(&r, n, 0, A, b);
        break;
    case LCSGEN_BOX_STACK_NORM:
        lcsgen_box_stack(&r, n, 1, A, b);
        break;
    case LCSGEN_DUP_ROWS: {
        lcsgen_gram(&r, n, n, 0, 0, 0.0625, A);
        lcsgen_rhs(&r, n, 70, 1.0, b);
        int dups = n / 3;
        for (int d = 0; d < dups; d++) {
            int src = lcsgen_int(&r, 0, n - 1);
            int dst = lcsgen_int(&r, 0, n - 1);
            if (src == dst) continue;
            for (int j = 0; j < n; j++) A[dst * n + j] = A[src * n + j];
            for (int i = 0; i < n; i++) A[i * n + dst] = A[i * n + src];
            A[dst * n + dst] = A[src * n + src];
            b[dst] = b[src];
        }
        break;
    }
    case LCSGEN_ILL:
        lcsgen_gram(&r, n, n, 0, 40, 0.0, A);
        lcsgen_rhs(&r, n, lcsgen_int(&r, 20, 90), 1.0, b);
        break;
    case LCSGEN_DIAG_DOM:
        for (int i = 0; i < n; i++) {
            for (int j = i; j < n; j++) {
                double v = i == j ? (double)n + lcsgen_unit(&r) : lcsgen_sym(&r);
                A[i * n + j] = v;
                A[j * n + i] = v;
            }
        }
        lcsgen_rhs(&r, n, lcsgen_int(&r, 10, 90), 1.0, b);
        break;
    case LCSGEN_NONSYM:
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                A[i * n + j] = (i == j) ? 1.0 + lcsgen_unit(&r) : lcsgen_mul(0.5, lcsgen_sym(&r));
            }
        }
        lcsgen_rhs(&r, n, lcsgen_int(&r, 10, 90), 1.0, b);
        break;
    case LCSGEN_SCALED:
    default: {
        double sc = lcsgen_pow2(lcsgen_int(&r, -30, 30));
        lcsgen_gram(&r, n, n, 0, 0, 0.001, A);
        for (int i = 0; i < n * n; i++) A[i] = lcsgen_mul(A[i], sc);
        lcsgen_rhs(&r, n, lcsgen_int(&r, 10, 90), lcsgen_pow2(lcsgen_int(&r, -20, 20)), b);
        break;
    }
    }
}

#endif /* LCS_PARITY_GEN_H */
