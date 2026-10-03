// lcs_ref_harness.cxx — generator of tests/data/lcs_cases.h (NOT part of the
// CMake build).  Links the reference C++ IVP engine (its own CMake, release
// build: cmake -DCMAKE_BUILD_TYPE=Release -DIVP_BUILD_EXAMPLES=OFF
// -DIVP_BUILD_TESTS=OFF, which yields six static libs) and runs its dense
// solvers on the problems of tests/lcs_parity_gen.h.  Build recipe:
//
//   L=<reference build dir>; R=<reference source dir>
//   LIBS="$L/libivp_physics.a $L/libivp_utility.a $L/libivp_collision.a \
//         $L/libivp_controller.a $L/libivp_surface_manager.a $L/libivp_compact_builder.a"
//   c++ -std=c++17 -O2 -ffp-contract=off -fno-fast-math \
//       -DAPPLE -DIVP_VERSION_SDK -DUNIX -DNDEBUG -w \
//       -I$L/include/ivp -I$R/ivp_physics -I$R/ivp_intern -I$R/ivp_utility -Itests \
//       tests/data/lcs_ref_harness.cxx $LIBS $LIBS -o lcs_ref
//   ./lcs_ref tests/data/lcs_cases.h
//
// The stored data was produced on macOS arm64 (Apple clang 21) where the
// reference uses IVP_VECFPU_SIZE 4 and FMA contraction.  lcs_cases_strict.h
// (IVP_GM_USE_FMA 0) is produced the same way from a reference build
// configured with CMAKE_CXX_FLAGS=-ffp-contract=off
// (./lcs_ref tests/data/lcs_cases_strict.h, then rename the include guard to
// LCS_CASES_STRICT_H).
//
#include <ivp_physics.hxx>
#include <ivp_great_matrix.hxx>
#include <ivu_memory.hxx>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <vector>

#include "lcs_parity_gen.h"

static const size_t EXT_SIZE = 65535;           // size_of_external_mem is 16 bit
alignas(64) static char ext_mem[EXT_SIZE + 256];

struct Arena {
    IVP_U_Memory mem;
    Arena(unsigned char fill) {
        memset(ext_mem, fill, sizeof(ext_mem));
        mem.init_mem_transaction_usage(ext_mem, EXT_SIZE);
    }
    void *get(size_t bytes) { return mem.get_mem(bytes); }
    bool still_in_external_block() {
        char *p = (char *)mem.get_mem(8);
        return p >= ext_mem && p < ext_mem + sizeof(ext_mem);
    }
};

static int g_block_overflow = 0;

static uint64_t bits_of(double d) { uint64_t u; memcpy(&u, &d, 8); return u; }

// init_and_solve_lc exactly as IVP_Friction_Solver calls it: A with row
// stride aligned_size, b and x with aligned_size entries.
static int ref_lcs(const double *A, const double *b, int n, int actives, double *x, unsigned char fill)
{
    int as = (n + IVP_VECFPU_SIZE - 1) & IVP_VECFPU_MASK;
    std::vector<double> Aal((size_t)n * as + 4, 0.0), bal(as + 4, 0.0), xal(as + 4, 0.0);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) Aal[(size_t)i * as + j] = A[i * n + j];
        bal[i] = b[i];
    }
    Arena ar(fill);
    IVP_Linear_Constraint_Solver lcs;
    IVP_RETURN_TYPE r = lcs.init_and_solve_lc(Aal.data(), bal.data(), xal.data(), n, actives, &ar.mem);
    if (!ar.still_in_external_block()) g_block_overflow++;
    for (int i = 0; i < n; i++) x[i] = xal[i];
    return r == IVP_OK ? 1 : 0;
}

// solve_great_matrix_many_zero with the allocation pattern of
// IVP_Friction_Solver::solve_linear_equation_and_push.
static int ref_gauss(const double *A, const double *b, int n, double *x, unsigned char fill)
{
    Arena ar(fill);
    IVP_Great_Matrix_Many_Zero m;
    m.columns = n;
    m.calc_aligned_row_len();
    m.desired_vector = (IVP_DOUBLE *)ar.get(m.aligned_row_len * sizeof(IVP_DOUBLE));
    m.result_vector = (IVP_DOUBLE *)ar.get(m.aligned_row_len * sizeof(IVP_DOUBLE));
    m.matrix_values = (IVP_DOUBLE *)ar.get((n * m.aligned_row_len + IVP_VECFPU_SIZE - 1) * sizeof(IVP_DOUBLE));
    m.align_matrix_values();
    // the release build does not clear padding: in the libivp port padding
    // starts at zero, so start the reference from zeroed memory as well
    // (fill == 0); other fills probe the dependence on uninitialised memory.
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) m.matrix_values[i * m.aligned_row_len + j] = A[i * n + j];
        m.desired_vector[i] = b[i];
    }
    IVP_RETURN_TYPE r = m.solve_great_matrix_many_zero();
    if (!ar.still_in_external_block()) g_block_overflow++;
    for (int i = 0; i < n; i++) x[i] = m.result_vector[i];
    m.matrix_values = m.desired_vector = m.result_vector = NULL;
    return r == IVP_OK ? 1 : 0;
}

struct Mat {
    IVP_Great_Matrix_Many_Zero m;
    Mat(Arena &ar, int n) {
        m.columns = n;
        m.calc_aligned_row_len();
        m.matrix_values = (IVP_DOUBLE *)ar.get(n * m.aligned_row_len * sizeof(IVP_DOUBLE));
        m.desired_vector = (IVP_DOUBLE *)ar.get(m.aligned_row_len * sizeof(IVP_DOUBLE));
        m.result_vector = (IVP_DOUBLE *)ar.get(m.aligned_row_len * sizeof(IVP_DOUBLE));
    }
    void fill(const double *A, const double *b, int stride) {
        int n = m.columns;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) m.matrix_values[i * stride + j] = A[i * n + j];
            m.desired_vector[i] = b[i];
        }
    }
    ~Mat() { m.matrix_values = m.desired_vector = m.result_vector = NULL; }
};

// see lcsgen_misc_hash_add in lcs_parity_gen.h
static uint64_t ref_misc(const double *A, const double *b, int n, const double *x_gauss)
{
    uint64_t h = 0xcbf29ce484222325ULL;
    {
        Arena ar(0);
        Mat t(ar, n);
        t.fill(A, b, t.m.aligned_row_len);
        t.m.transform_to_lower_null_triangle();
        int ok = t.m.solve_lower_null_matrix() == IVP_OK;
        h = lcsgen_misc_hash_add(h, (double)ok);
        for (int i = 0; i < n; i++) h = lcsgen_misc_hash_add(h, t.m.result_vector[i]);
    }
    {
        Arena ar(0);
        Mat t(ar, n);
        t.fill(A, b, t.m.aligned_row_len);
        t.m.mult_aligned();
        for (int i = 0; i < n; i++) h = lcsgen_misc_hash_add(h, t.m.result_vector[i]);
    }
    {
        Arena ar(0);
        Mat t(ar, n);
        t.fill(A, b, n);
        t.m.mult();
        for (int i = 0; i < n; i++) h = lcsgen_misc_hash_add(h, t.m.result_vector[i]);
    }
    {
        Arena ar(0);
        Mat t(ar, n);
        t.fill(A, b, t.m.aligned_row_len);
        for (int i = 0; i < n; i++) t.m.result_vector[i] = x_gauss[i];
        for (int i = 0; i < n; i++) {
            int ok = t.m.matrix_check_unequation_line(i) == IVP_OK;
            h = lcsgen_misc_hash_add(h, (double)ok);
        }
    }
    return h;
}

static int ref_invert(const double *A, int n, double *inv)
{
    Arena ar(0);
    IVP_Great_Matrix_Many_Zero m, out;
    m.columns = n;
    m.calc_aligned_row_len();
    out.columns = n;
    out.calc_aligned_row_len();
    m.matrix_values = (IVP_DOUBLE *)ar.get(n * m.aligned_row_len * sizeof(IVP_DOUBLE));
    m.desired_vector = (IVP_DOUBLE *)ar.get(m.aligned_row_len * sizeof(IVP_DOUBLE));
    m.result_vector = (IVP_DOUBLE *)ar.get(m.aligned_row_len * sizeof(IVP_DOUBLE));
    out.matrix_values = (IVP_DOUBLE *)ar.get(n * out.aligned_row_len * sizeof(IVP_DOUBLE));
    out.desired_vector = (IVP_DOUBLE *)ar.get(out.aligned_row_len * sizeof(IVP_DOUBLE));
    out.result_vector = (IVP_DOUBLE *)ar.get(out.aligned_row_len * sizeof(IVP_DOUBLE));
    for (int i = 0; i < n * n; i++) m.matrix_values[i] = A[i]; // stride = columns
    IVP_RETURN_TYPE r = m.invert(&out);
    for (int i = 0; i < n * n; i++) inv[i] = out.matrix_values[i];
    m.matrix_values = m.desired_vector = m.result_vector = NULL;
    out.matrix_values = out.desired_vector = out.result_vector = NULL;
    return r == IVP_OK ? 1 : 0;
}

static void put_bits(FILE *f, const std::vector<uint64_t> &v)
{
    for (size_t i = 0; i < v.size(); i++) {
        fprintf(f, "0x%016llxULL,%s", (unsigned long long)v[i], (i % 4 == 3) ? "\n" : " ");
    }
    if (v.size() % 4) fprintf(f, "\n");
    if (v.empty()) fprintf(f, "0\n");
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <out_header>\n", argv[0]);
        return 2;
    }
    const int N_LCS = 600;
    const int N_INV = 80;

    lcsgen_rng_t pick;
    pick.s = 0x1234567890abcdefULL;

    std::vector<lcs_parity_case_t> cases;
    std::vector<uint64_t> bits;
    std::vector<lcs_parity_inv_case_t> inv_cases;
    std::vector<uint64_t> inv_bits;

    int lcs_ok = 0, gauss_ok = 0, garbage_dep_lcs = 0, garbage_dep_gauss = 0;
    int lcp_valid = 0, gauss_wrong = 0;
    int fam_count[LCSGEN_N_FAMILIES] = {0};
    double A[LCSGEN_MAX_N * LCSGEN_MAX_N], b[LCSGEN_MAX_N], x[LCSGEN_MAX_N], xg[LCSGEN_MAX_N];
    double x2[LCSGEN_MAX_N];

    // cases found by searching the port for rare solver paths
    // (Gauss fallback in full_setup, LU increment/decrement failures,
    // step-limit faults, zero-step heuristic, illegal variables, ...)
    static const unsigned targeted[][4] = {
    {6, 657262802u, 9, 0}, /* limiting_neg */
    {9, 4048934927u, 24, 13}, /* limiting_neg */
    {6, 3974078719u, 2, 0}, /* limiting_neg */
    {6, 3737401873u, 27, 0}, /* setup_step_fault */
    {9, 2008164357u, 2, 1}, /* setup_step_fault */
    {9, 2103437329u, 39, 0}, /* main_step_fault */
    {6, 4071155488u, 31, 31}, /* illegal_vars */
    {6, 1173945102u, 27, 11}, /* illegal_vars */
    {8, 2265116648u, 3, 3}, /* illegal_vars */
    {6, 3489558824u, 27, 27}, /* setup_step_fault */
    {9, 2684684942u, 27, 0}, /* main_step_fault */
    {9, 1021423110u, 11, 11}, /* main_step_fault */
    {9, 1696867672u, 37, 37}, /* accel0_to_active */
    {9, 2765120487u, 10, 3}, /* incr_fail */
    {9, 1838101141u, 25, 18}, /* full_setup_lu_fail */
    {9, 835699339u, 8, 8}, /* fast_setup_fail */
    {9, 4001801905u, 36, 34}, /* fast_setup_fail */
    {9, 1981783723u, 28, 0}, /* incr_fail */
    {9, 2761306671u, 17, 9}, /* zero_steps */
    {9, 3794470906u, 37, 18}, /* full_setup_lu_fail */
    {4, 2023100207u, 7, 7}, /* full_setup_lu_fail */
    {9, 2611073071u, 12, 2}, /* accel0_to_active */
    {9, 4083885297u, 7, 7}, /* zero_steps */
    {3, 1142967788u, 7, 7}, /* setup_gauss_fail */
    {9, 751238676u, 36, 27}, /* zero_steps */
    {9, 2966851916u, 33, 0}, /* accel0_to_active */
    };
    const int N_TARGETED = (int)(sizeof(targeted) / sizeof(targeted[0]));
    const int N_ALL = N_LCS + N_TARGETED;

    for (int c = 0; c < N_ALL; c++) {
        lcs_parity_case_t pc;
        memset(&pc, 0, sizeof(pc));
        if (c < N_LCS) {
            pc.family = c % LCSGEN_N_FAMILIES;
            int band = lcsgen_int(&pick, 0, 99);
            pc.n = band < 50 ? lcsgen_int(&pick, 1, 10) : (band < 80 ? lcsgen_int(&pick, 11, 24) : lcsgen_int(&pick, 25, 40));
            pc.seed = (uint32_t)(lcsgen_next(&pick) & 0xffffffffu);
            switch (lcsgen_int(&pick, 0, 4)) {
            case 0: pc.actives_at_begin = 0; break;
            case 1: pc.actives_at_begin = pc.n; break;
            case 2: pc.actives_at_begin = pc.n / 2; break;
            default: pc.actives_at_begin = lcsgen_int(&pick, 0, pc.n); break;
            }
        } else {
            const unsigned *t = targeted[c - N_LCS];
            pc.family = (int)t[0];
            pc.seed = (uint32_t)t[1];
            pc.n = (int)t[2];
            pc.actives_at_begin = (int)t[3];
        }
        int n = pc.n;
        lcsgen_make(pc.family, pc.seed, n, A, b);
        pc.input_hash = lcsgen_input_hash(A, b, n);

        pc.lcs_ok = ref_lcs(A, b, n, pc.actives_at_begin, x, 0);
        pc.gauss_ok = ref_gauss(A, b, n, xg, 0);
        pc.misc_hash = ref_misc(A, b, n, xg);

        // dependence on uninitialised arena memory (informational)
        static const unsigned char fills[2] = {0xA5, 0x7F};
        for (int fi = 0; fi < 2; fi++) {
            int r2 = ref_lcs(A, b, n, pc.actives_at_begin, x2, fills[fi]);
            if (r2 != pc.lcs_ok || memcmp(x, x2, sizeof(double) * n) != 0) { garbage_dep_lcs++; break; }
        }
        for (int fi = 0; fi < 2; fi++) {
            int r2 = ref_gauss(A, b, n, x2, fills[fi]);
            if (r2 != pc.gauss_ok || memcmp(xg, x2, sizeof(double) * n) != 0) { garbage_dep_gauss++; break; }
        }

        lcs_ok += pc.lcs_ok;
        gauss_ok += pc.gauss_ok;
        fam_count[pc.family]++;
        if (pc.lcs_ok) {
            int valid = 1;
            for (int i = 0; i < n; i++) {
                double acc = -b[i];
                for (int j = 0; j < n; j++) acc += A[i * n + j] * x[j];
                if (x[i] < 0 || acc < -1e-4 || fabs(x[i] * acc) > 1e-4) valid = 0;
            }
            lcp_valid += valid;
        }
        if (pc.gauss_ok) {
            double worst = 0;
            for (int i = 0; i < n; i++) {
                double acc = -b[i];
                for (int j = 0; j < n; j++) acc += A[i * n + j] * xg[j];
                if (fabs(acc) > worst) worst = fabs(acc);
            }
            if (worst > 1e-6) gauss_wrong++;
        }

        pc.bits_offset = (int)bits.size();
        for (int i = 0; i < n; i++) bits.push_back(bits_of(x[i]));
        for (int i = 0; i < n; i++) bits.push_back(bits_of(xg[i]));
        cases.push_back(pc);
    }

    static const int inv_fams[] = {LCSGEN_NONSYM, LCSGEN_SPD_DENSE, LCSGEN_DIAG_DOM, LCSGEN_ILL,
                                   LCSGEN_DUP_ROWS, LCSGEN_SPD_LOWRANK, LCSGEN_BOX_STACK_NORM, LCSGEN_SCALED};
    int inv_ok = 0;
    std::vector<double> inv(LCSGEN_MAX_N * LCSGEN_MAX_N);
    for (int c = 0; c < N_INV; c++) {
        lcs_parity_inv_case_t ic;
        memset(&ic, 0, sizeof(ic));
        ic.family = inv_fams[c % 8];
        ic.n = lcsgen_int(&pick, 1, 16);
        ic.seed = (uint32_t)(lcsgen_next(&pick) & 0xffffffffu);
        lcsgen_make(ic.family, ic.seed, ic.n, A, b);
        ic.input_hash = lcsgen_input_hash(A, b, ic.n);
        ic.inv_ok = ref_invert(A, ic.n, inv.data());
        inv_ok += ic.inv_ok;
        ic.bits_offset = (int)inv_bits.size();
        for (int i = 0; i < ic.n * ic.n; i++) inv_bits.push_back(bits_of(inv[i]));
        inv_cases.push_back(ic);
    }

    FILE *f = fopen(argv[1], "w");
    if (!f) { perror("fopen"); return 1; }
    fprintf(f, "/* lcs_cases.h — reference outputs for test_lcs_parity.c (generated, do not edit)\n"
               " *\n"
               " * Produced by a C++ harness linked against the reference IVP library\n"
               " * (64-bit macOS arm64 release build: IVP_VECFPU_SIZE 4, clang -O3,\n"
               " * default FP contraction), started on zero-filled IVP_U_Memory.\n"
               " * Inputs are regenerated by lcs_parity_gen.h; values are IEEE-754 bits.\n"
               " * LCS cases: %d (%d random, %d targeted; %d IVP_OK), Gauss: %d IVP_OK,\n"
               " * invert cases: %d (%d IVP_OK).\n"
               " */\n", N_ALL, N_LCS, N_TARGETED, lcs_ok, gauss_ok, N_INV, inv_ok);
    fprintf(f, "#ifndef LCS_CASES_H\n#define LCS_CASES_H\n\n#include \"lcs_parity_gen.h\"\n\n");
    fprintf(f, "#define LCS_PARITY_N_CASES %d\n#define LCS_PARITY_N_INV_CASES %d\n\n", N_ALL, N_INV);
    fprintf(f, "static const lcs_parity_case_t lcs_parity_cases[LCS_PARITY_N_CASES] = {\n"
               "    /* family, seed, n, actives_at_begin, lcs_ok, gauss_ok, input_hash, bits_offset, misc_hash */\n");
    for (size_t i = 0; i < cases.size(); i++) {
        const lcs_parity_case_t &pc = cases[i];
        fprintf(f, "    {%d, %uu, %d, %d, %d, %d, 0x%016llxULL, %d, 0x%016llxULL},\n", pc.family, pc.seed, pc.n,
                pc.actives_at_begin, pc.lcs_ok, pc.gauss_ok, (unsigned long long)pc.input_hash, pc.bits_offset,
                (unsigned long long)pc.misc_hash);
    }
    fprintf(f, "};\n\nstatic const uint64_t lcs_parity_bits[] = {\n");
    put_bits(f, bits);
    fprintf(f, "};\n\nstatic const lcs_parity_inv_case_t lcs_parity_inv_cases[LCS_PARITY_N_INV_CASES] = {\n"
               "    /* family, seed, n, inv_ok, input_hash, bits_offset */\n");
    for (size_t i = 0; i < inv_cases.size(); i++) {
        const lcs_parity_inv_case_t &ic = inv_cases[i];
        fprintf(f, "    {%d, %uu, %d, %d, 0x%016llxULL, %d},\n", ic.family, ic.seed, ic.n, ic.inv_ok,
                (unsigned long long)ic.input_hash, ic.bits_offset);
    }
    fprintf(f, "};\n\nstatic const uint64_t lcs_parity_inv_bits[] = {\n");
    put_bits(f, inv_bits);
    fprintf(f, "};\n\n#endif /* LCS_CASES_H */\n");
    fclose(f);

    printf("LCS cases %d: ok %d, valid LCP among ok %d, garbage-dependent %d\n", N_ALL, lcs_ok, lcp_valid, garbage_dep_lcs);
    printf("Gauss: ok %d, wrong (resid>1e-6) among ok %d, garbage-dependent %d\n", gauss_ok, gauss_wrong, garbage_dep_gauss);
    printf("invert cases %d: ok %d\n", N_INV, inv_ok);
    printf("families:");
    for (int i = 0; i < LCSGEN_N_FAMILIES; i++) printf(" %d", fam_count[i]);
    printf("\nexternal block overflow: %d\nbits: %zu + %zu\n", g_block_overflow, bits.size(), inv_bits.size());
    return 0;
}
