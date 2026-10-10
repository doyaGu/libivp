/* test_active_value_ref.c — active values vs. the original IVP engine
 *
 * Expected values/logs were produced by running the original
 * ivu_active_value.cxx (IVP_U_Active_*, IVP_U_Active_Value_Manager) with
 * the same scenarios (scratch generator gen_active.cxx).
 */
#include "ivp_test.h"
#include <ivp/utility/ivp_active_value.h>
#include <stdio.h>
#include <string.h>

static char g_log[512];

static void log_float(void *ctx, ivp_active_float_t *src, double old_v, double new_v) {
    (void)old_v;
    (void)new_v;
    char buf[64];
    snprintf(buf, sizeof buf, "%s=%.17g;", (const char *)ctx, ivp_active_float_get(src));
    strncat(g_log, buf, sizeof(g_log) - strlen(g_log) - 1);
}

static void log_int(void *ctx, ivp_active_int_t *src, int old_v, int new_v) {
    (void)old_v;
    (void)new_v;
    char buf[64];
    snprintf(buf, sizeof buf, "%s=%d;", (const char *)ctx, ivp_active_int_get(src));
    strncat(g_log, buf, sizeof(g_log) - strlen(g_log) - 1);
}

#define LOG_CLEAR() (g_log[0] = 0)
#define ASSERT_LOG(expected) do { \
    if (strcmp(g_log, (expected)) != 0) { \
        fprintf(stderr, "  log mismatch: got '%s' expected '%s' (line %d)\n", g_log, (expected), __LINE__); \
        (*_fail_count)++; return; } } while (0)

/* Must run first in a reference-internals build: that path reproduces the
 * process-wide meter-zero construction quirk.  The normal build evaluates
 * the new node independently of earlier graphs. */
IVP_TEST(construction_update_policy) {
#if IVP_REFERENCE_INTERNAL_STATE
    if (ivp_active_value_change_meter() != -1) {
        fprintf(stderr, "  (skipped: change meter already used)\n");
        return;
    }
    ivp_active_float_t *a = ivp_active_float_terminal_create(NULL, "qa", 1.0);
    ivp_active_float_t *b = ivp_active_float_terminal_create(NULL, "qb", 2.0);
    ivp_active_float_set(a, 3.0);
    IVP_ASSERT_EQ(ivp_active_value_change_meter(), 0);
    /* node constructed while change_meter == last_update == 0: not evaluated */
    ivp_active_float_t *s = ivp_active_float_add_create(NULL, "qs", a, b);
    IVP_ASSERT_TRUE(ivp_active_float_get(s) == 0.0);
    ivp_active_float_set(b, 2.0); /* same value: meter++ but no propagation */
    IVP_ASSERT_TRUE(ivp_active_float_get(s) == 0.0);
    IVP_ASSERT_EQ(ivp_active_value_change_meter(), 1);
#else
    IVP_ASSERT_EQ(ivp_active_value_change_meter(), -1);
    ivp_active_float_t *a = ivp_active_float_terminal_create(NULL, "qa", 1.0);
    ivp_active_float_t *b = ivp_active_float_terminal_create(NULL, "qb", 2.0);
    ivp_active_float_set(a, 3.0);
    IVP_ASSERT_EQ(ivp_active_value_change_meter(), -1);
    ivp_active_float_t *s = ivp_active_float_add_create(NULL, "qs", a, b);
    IVP_ASSERT_TRUE(ivp_active_float_get(s) == 5.0);
    ivp_active_float_set(b, 2.0);
    IVP_ASSERT_TRUE(ivp_active_float_get(s) == 5.0);
#endif
    ivp_active_float_set(b, 4.0);
    IVP_ASSERT_TRUE(ivp_active_float_get(s) == 7.0);
    ivp_active_float_remove_ref(s);
    ivp_active_float_remove_ref(a);
    ivp_active_float_remove_ref(b);
}

IVP_TEST(ref_oscillators) {
    ivp_active_float_t *t = ivp_active_float_terminal_create(NULL, "t", 0.0);
    ivp_active_float_t *sine = ivp_active_float_sine_create(NULL, "sine", t, 2.0, 1.5, 0.25, 0.3);
    ivp_active_float_t *sq = ivp_active_float_square_create(NULL, "sq", t, 3.0, -1.0, 4.0);
    ivp_active_float_t *pu = ivp_active_float_pulse_create(NULL, "pu", t, 2.0, 1, 4, 0.0, 10.0);
    /* square is not evaluated at construction (reference) */
    IVP_ASSERT_TRUE(ivp_active_float_get(sine) == 0x1.62f5a304c0ba6p-1);
    IVP_ASSERT_TRUE(ivp_active_float_get(sq) == 0.0);
    IVP_ASSERT_TRUE(ivp_active_float_get(pu) == 10.0);
    static const double ts[8] = { 0.1, 0.2, 0.4, 0.6, 0.8, 2.5, -0.2, -0.5 };
    static const double ref[8][3] = {
        { 0x1.f032e57384474p-1, -1.0, 10.0 },
        { 0x1.37612cef4934ap+0, -1.0, 0.0 },
        { 0x1.96393f9220a6ap+0, 4.0, 0.0 },
        { 0x1.bf09bf4615228p+0, 4.0, 10.0 },
        { 0x1.ab6115753aa41p+0, -1.0, 0.0 },
        { -0x1.ff2e6ff1a1b58p-1, 4.0, 10.0 },
        { 0x1.9a9f9ceae4f3cp-4, -1.0, 10.0 },
        { -0x1.6ec259de92694p-1, 4.0, 10.0 },
    };
    for (int i = 0; i < 8; i++) {
        ivp_active_float_set(t, ts[i]);
        IVP_ASSERT_TRUE(ivp_active_float_get(sine) == ref[i][0]);
        IVP_ASSERT_TRUE(ivp_active_float_get(sq) == ref[i][1]);
        IVP_ASSERT_TRUE(ivp_active_float_get(pu) == ref[i][2]);
    }
    ivp_active_float_remove_ref(sine);
    ivp_active_float_remove_ref(sq);
    ivp_active_float_remove_ref(pu);
    ivp_active_float_remove_ref(t);
}

IVP_TEST(ref_diamond_change_meter_dedup) {
    ivp_active_float_t *a = ivp_active_float_terminal_create(NULL, "a", 1.0);
    ivp_active_float_t *two = ivp_active_float_terminal_create(NULL, "two", 2.0);
    ivp_active_float_t *three = ivp_active_float_terminal_create(NULL, "three", 3.0);
    ivp_active_float_t *b = ivp_active_float_mult_create(NULL, "b", a, two);
    ivp_active_float_t *c = ivp_active_float_add_create(NULL, "c", a, three);
    ivp_active_float_t *d = ivp_active_float_add_create(NULL, "d", b, c);
    ivp_active_float_add_listener(b, log_float, "B");
    ivp_active_float_add_listener(c, log_float, "C");
    ivp_active_float_add_listener(d, log_float, "D");
    IVP_ASSERT_TRUE(ivp_active_float_get(d) == 6.0);

    /* D fires once, computed from the new C and the not-yet-updated B
     * (b = 10, c = 8, but d = 2 + 8 = 10) — reference behaviour */
    LOG_CLEAR();
    ivp_active_float_set(a, 5.0);
    IVP_ASSERT_TRUE(ivp_active_float_get(d) == 10.0);
    ASSERT_LOG("C=8;D=10;B=10;");

    LOG_CLEAR();
    ivp_active_float_set(three, 4.0);
    IVP_ASSERT_TRUE(ivp_active_float_get(d) == 19.0);
    ASSERT_LOG("C=9;D=19;");

    LOG_CLEAR();
    ivp_active_float_set(a, 6.0);
    IVP_ASSERT_TRUE(ivp_active_float_get(d) == 20.0);
    ASSERT_LOG("C=10;D=20;B=12;");

    ivp_active_float_remove_listener(b, log_float, "B");
    ivp_active_float_remove_listener(c, log_float, "C");
    ivp_active_float_remove_listener(d, log_float, "D");
    ivp_active_float_remove_ref(d);
    ivp_active_float_remove_ref(c);
    ivp_active_float_remove_ref(b);
    ivp_active_float_remove_ref(three);
    ivp_active_float_remove_ref(two);
    ivp_active_float_remove_ref(a);
}

IVP_TEST(ref_listener_and_dependent_order) {
    ivp_active_float_t *t = ivp_active_float_terminal_create(NULL, "lt", 1.0);
    ivp_active_float_t *k = ivp_active_float_terminal_create(NULL, "lk", 10.0);
    ivp_active_float_add_listener(t, log_float, "L1");
    ivp_active_float_t *n1 = ivp_active_float_add_create(NULL, "ln1", t, k);
    ivp_active_float_add_listener(t, log_float, "L2");
    ivp_active_float_t *n2 = ivp_active_float_mult_create(NULL, "ln2", t, k);
    ivp_active_float_add_listener(n1, log_float, "N1");
    ivp_active_float_add_listener(n2, log_float, "N2");
    LOG_CLEAR();
    ivp_active_float_set(t, 2.0);
    ASSERT_LOG("N2=20;L2=2;N1=12;L1=2;");
    IVP_ASSERT_TRUE(ivp_active_float_remove_listener(t, log_float, "L2"));
    LOG_CLEAR();
    ivp_active_float_set(t, 3.0);
    ASSERT_LOG("N2=30;N1=13;L1=3;");

    ivp_active_float_remove_listener(t, log_float, "L1");
    ivp_active_float_remove_listener(n1, log_float, "N1");
    ivp_active_float_remove_listener(n2, log_float, "N2");
    ivp_active_float_remove_ref(n2);
    ivp_active_float_remove_ref(n1);
    IVP_ASSERT_EQ(t->ref_count, 1);
    IVP_ASSERT_EQ(t->derived_count, 0);
    ivp_active_float_remove_ref(k);
    ivp_active_float_remove_ref(t);
}

IVP_TEST(ref_test_range_switch_limit) {
    ivp_active_float_t *x = ivp_active_float_terminal_create(NULL, "x", -1.0);
    ivp_active_float_t *lo = ivp_active_float_terminal_create(NULL, "lo", 0.0);
    ivp_active_float_t *hi = ivp_active_float_terminal_create(NULL, "hi", 1.0);
    ivp_active_int_t *tr = ivp_active_int_test_range_create(NULL, "tr", x, lo, hi);
    ivp_active_float_t *va = ivp_active_float_terminal_create(NULL, "va", 7.0);
    ivp_active_float_t *vb = ivp_active_float_terminal_create(NULL, "vb", -3.0);
    ivp_active_float_t *sw = ivp_active_float_switch_create(NULL, "sw", tr, vb, va);
    ivp_active_float_t *lim = ivp_active_float_limit_create(NULL, "lim", x, -0.5, 0.75);
    ivp_active_float_t *am = ivp_active_float_add_multiple_create(NULL, "am", x, va, 0.5);
    ivp_active_float_t *sub = ivp_active_float_sub_create(NULL, "sub", va, x);
    ivp_active_int_add_listener(tr, log_int, "TR");
    ivp_active_float_add_listener(sw, log_float, "SW");

    IVP_ASSERT_EQ(ivp_active_int_get(tr), 0);
    IVP_ASSERT_TRUE(ivp_active_float_get(sw) == -3.0);
    IVP_ASSERT_TRUE(ivp_active_float_get(lim) == -0.5);
    IVP_ASSERT_TRUE(ivp_active_float_get(am) == 2.5);
    IVP_ASSERT_TRUE(ivp_active_float_get(sub) == 8.0);

    static const double xs[5] = { 0.5, 1.0, 2.0, 0.0, -0.25 };
    static const int r_tr[5] = { 1, 1, 0, 1, 0 };
    static const double r_v[5][4] = {
        { 7, 0.5, 4, 6.5 }, { 7, 0.75, 4.5, 6 }, { -3, 0.75, 5.5, 5 },
        { 7, 0, 3.5, 7 }, { -3, -0.25, 3.25, 7.25 } };
    static const char *r_log[5] = { "TR=1;SW=7;", "", "TR=0;SW=-3;", "TR=1;SW=7;", "TR=0;SW=-3;" };
    for (int i = 0; i < 5; i++) {
        LOG_CLEAR();
        ivp_active_float_set(x, xs[i]);
        IVP_ASSERT_EQ(ivp_active_int_get(tr), r_tr[i]);
        IVP_ASSERT_TRUE(ivp_active_float_get(sw) == r_v[i][0]);
        IVP_ASSERT_TRUE(ivp_active_float_get(lim) == r_v[i][1]);
        IVP_ASSERT_TRUE(ivp_active_float_get(am) == r_v[i][2]);
        IVP_ASSERT_TRUE(ivp_active_float_get(sub) == r_v[i][3]);
        ASSERT_LOG(r_log[i]);
    }
    LOG_CLEAR();
    ivp_active_float_set(hi, -1.0);
    IVP_ASSERT_EQ(ivp_active_int_get(tr), 0);
    ASSERT_LOG("");

    ivp_active_int_t *ci = ivp_active_int_terminal_create(NULL, "ci", 0);
    ivp_active_float_t *sw2 = ivp_active_float_switch_create(NULL, "sw2", ci, vb, va);
    IVP_ASSERT_TRUE(ivp_active_float_get(sw2) == -3.0);
    ivp_active_int_set(ci, 3);
    IVP_ASSERT_TRUE(ivp_active_float_get(sw2) == 7.0);

    /* compatibility alias */
    ivp_active_float_t *cl = ivp_active_float_clamp_create(NULL, "cl", x, -0.1, 0.1);
    IVP_ASSERT_EQ(cl->type, IVP_ACTIVE_FLOAT_CLAMP);
    IVP_ASSERT_TRUE(ivp_active_float_get(cl) == -0.1);

    ivp_active_int_remove_listener(tr, log_int, "TR");
    ivp_active_float_remove_listener(sw, log_float, "SW");
    ivp_active_float_remove_ref(cl);
    ivp_active_float_remove_ref(sw2);
    ivp_active_int_remove_ref(ci);
    ivp_active_float_remove_ref(sub);
    ivp_active_float_remove_ref(am);
    ivp_active_float_remove_ref(lim);
    ivp_active_float_remove_ref(sw);
    ivp_active_int_remove_ref(tr);
    IVP_ASSERT_EQ(x->ref_count, 1);
    IVP_ASSERT_EQ(lo->derived_count, 0);
    ivp_active_float_remove_ref(vb);
    ivp_active_float_remove_ref(va);
    ivp_active_float_remove_ref(hi);
    ivp_active_float_remove_ref(lo);
    ivp_active_float_remove_ref(x);
}

IVP_TEST(ref_manager) {
    ivp_active_value_manager_t *m = ivp_active_value_manager_create(NULL);
    IVP_ASSERT_NOT_NULL(m);
    ivp_active_float_t *ct = ivp_active_value_manager_get_float(m, "current_time");
    ivp_active_float_t *dn = ivp_active_value_manager_get_float(m, "double_null");
    IVP_ASSERT_NOT_NULL(ct);
    IVP_ASSERT_NOT_NULL(dn);
    IVP_ASSERT_TRUE(ct == ivp_active_value_manager_current_time(m));
    IVP_ASSERT_TRUE(ivp_active_float_get(ct) == 0.0);

    ivp_active_float_t *foo = ivp_active_value_manager_install_float(m, "foo", 2.5);
    IVP_ASSERT_TRUE(ivp_active_value_manager_install_float(m, "foo", 9.0) == foo);
    IVP_ASSERT_NULL(ivp_active_value_manager_create_float(m, "foo", 1.0));
    IVP_ASSERT_TRUE(ivp_active_float_get(foo) == 2.5);

    ivp_active_float_t *num = ivp_active_value_manager_get_float(m, "-.5");
    ivp_active_float_t *num2 = ivp_active_value_manager_get_float(m, "12.75");
    IVP_ASSERT_NOT_NULL(num);
    IVP_ASSERT_NOT_NULL(num2);
    IVP_ASSERT_TRUE(ivp_active_float_get(num) == -0.5);
    IVP_ASSERT_TRUE(ivp_active_float_get(num2) == 12.75);
    IVP_ASSERT_NULL(ivp_active_value_manager_get_float(m, "abc"));
    IVP_ASSERT_NULL(ivp_active_value_manager_get_float(m, "-x"));
    IVP_ASSERT_TRUE(ivp_active_value_manager_get_float(m, "-.5") == num);

    ivp_active_int_t *in = ivp_active_value_manager_get_int(m, "42");
    IVP_ASSERT_NOT_NULL(in);
    IVP_ASSERT_EQ(ivp_active_int_get(in), 42);
    ivp_active_int_t *in2 = ivp_active_value_manager_install_int(m, "cnt", 3);
    IVP_ASSERT_EQ(ivp_active_int_get(in2), 3);
    IVP_ASSERT_NULL(ivp_active_value_manager_create_int(m, "cnt", 1));

    /* user node registered in the manager (no name → refused) */
    ivp_active_float_t *anon = ivp_active_float_terminal_create(NULL, NULL, 1.0);
    IVP_ASSERT_FALSE(ivp_active_value_manager_insert_float(m, anon));
    ivp_active_float_remove_ref(anon);

    ivp_active_float_t *s = ivp_active_float_sine_create(NULL, "msine", ct, 1.0, 1.0, 0.0, 0.0);
    ivp_active_value_manager_refresh_psi(m, 1.25);
    IVP_ASSERT_TRUE(ivp_active_float_get(ct) == 1.25);
    IVP_ASSERT_TRUE(ivp_active_float_get(s) == 0.9489846193555862);

    ivp_active_float_t *fm = ivp_active_float_mult_create(NULL, "fm", foo, ct);
    IVP_ASSERT_TRUE(ivp_active_float_get(fm) == 3.125);
    ivp_active_float_set_delayed(foo, 4.0, true);
    IVP_ASSERT_TRUE(ivp_active_float_get(foo) == 4.0);
    IVP_ASSERT_TRUE(ivp_active_float_get(fm) == 3.125);
    ivp_active_value_manager_update_delayed(m);
    IVP_ASSERT_TRUE(ivp_active_float_get(fm) == 5.0);
    ivp_active_float_set_delayed(foo, 5.0, true);
    ivp_active_value_manager_refresh_psi(m, 2.0);
    IVP_ASSERT_TRUE(ivp_active_float_get(fm) == 10.0);
    IVP_ASSERT_TRUE(ivp_active_float_get(s) == 0.90929742682568171);

    ivp_active_float_remove_ref(fm);
    ivp_active_float_remove_ref(s);
    ivp_active_value_manager_destroy(m);
}

int main(void) {
    IVP_REGISTER(construction_update_policy);
    IVP_REGISTER(ref_oscillators);
    IVP_REGISTER(ref_diamond_change_meter_dedup);
    IVP_REGISTER(ref_listener_and_dependent_order);
    IVP_REGISTER(ref_test_range_switch_limit);
    IVP_REGISTER(ref_manager);
    IVP_RUN_ALL();
}
