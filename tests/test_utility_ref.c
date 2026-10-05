/* test_utility_ref.c — min hash / string / string hash / set / diff hash
 *
 * Min hash, string and string-hash expectations were produced by running
 * the original ivu_min_hash.cxx, ivu_string.cxx and ivu_hash.cxx with the
 * same inputs (elements are fake pointer values so that the pointer hash
 * and therefore the tie order is identical).
 */
#include "ivp_test.h"
#include <ivp/containers/ivp_min_hash.h>
#include <ivp/containers/ivp_set.h>
#include <ivp/containers/ivp_diff_hash.h>
#include <ivp/utility/ivp_string.h>
#include <ivp/base/ivp_assert.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>

static void *fake(int i) { return (void *)(uintptr_t)(0x10000 + 48 * i); }
static int unfake(const void *p) { return (int)(((uintptr_t)p - 0x10000) / 48); }

static int g_assert_count = 0;
static void count_assert(const char *expr, const char *file, int line,
                         const char *func, const char *msg, void *userdata) {
    (void)expr; (void)file; (void)line; (void)func; (void)msg; (void)userdata;
    g_assert_count++;
}

IVP_TEST(ref_min_hash_tie_order) {
    ivp_min_hash_t h;
    IVP_ASSERT_EQ(ivp_min_hash_init(&h, NULL, 16), IVP_OK);
    static const double vals[40] = {
        3, 1, 4, 1, 5, 9, 2, 6, 5, 3, 5, 8, 9, 7, 9, 3, 2, 3, 8, 4,
        6, 2, 6, 4, 3, 3, 8, 3, 2, 7, 9, 5, 0, 2, 8, 8, 4, 1, 9, 7 };
    for (int i = 0; i < 40; i++) ivp_min_hash_add(&h, fake(i), vals[i]);
    IVP_ASSERT_EQ(ivp_min_hash_count(&h), 40);
    ivp_min_hash_change_value(&h, fake(5), 1.0);
    ivp_min_hash_change_value(&h, fake(32), 1.0);
    ivp_min_hash_remove(&h, fake(3));
    IVP_ASSERT_FALSE(ivp_min_hash_is_elem(&h, fake(3)));
    IVP_ASSERT_TRUE(ivp_min_hash_is_elem(&h, fake(4)));

    static const int r_enum[39] = { 33, 22, 11, 0, 17, 6, 39, 28, 1, 34, 23, 12, 18, 7, 29, 2,
                                    35, 24, 13, 19, 8, 30, 36, 25, 14, 20, 9, 31, 4, 37, 26,
                                    15, 21, 10, 32, 5, 38, 27, 16 };
    ivp_min_hash_enumerator_t en;
    ivp_min_hash_enumerator_init(&en, &h);
    void *e;
    int n = 0;
    while ((e = ivp_min_hash_enumerator_next(&en))) {
        IVP_ASSERT_TRUE(n < 39);
        IVP_ASSERT_EQ(unfake(e), r_enum[n]);
        n++;
    }
    IVP_ASSERT_EQ(n, 39);

    static const int r_lt3[9] = { 33, 6, 28, 1, 37, 21, 32, 5, 16 };
    ivp_min_hash_enumerator_init(&en, &h);
    n = 0;
    while ((e = ivp_min_hash_enumerator_next_lt(&en, 3.0))) {
        IVP_ASSERT_TRUE(n < 9);
        IVP_ASSERT_EQ(unfake(e), r_lt3[n]);
        n++;
    }
    IVP_ASSERT_EQ(n, 9);

    static const int r_pop[39][2] = {
        {1,1},{37,1},{5,1},{32,1},{6,2},{16,2},{21,2},{28,2},{33,2},{0,3},{9,3},{15,3},{17,3},
        {24,3},{25,3},{27,3},{2,4},{19,4},{23,4},{36,4},{4,5},{8,5},{10,5},{31,5},{7,6},{20,6},
        {22,6},{13,7},{29,7},{39,7},{11,8},{18,8},{26,8},{34,8},{35,8},{12,9},{14,9},{30,9},{38,9} };
    n = 0;
    while (ivp_min_hash_find_min_elem(&h)) {
        IVP_ASSERT_TRUE(n < 39);
        IVP_ASSERT_EQ(unfake(ivp_min_hash_find_min_elem(&h)), r_pop[n][0]);
        IVP_ASSERT_TRUE(ivp_min_hash_find_min_value(&h) == (double)r_pop[n][1]);
        ivp_min_hash_remove_min(&h);
        n++;
    }
    IVP_ASSERT_EQ(n, 39);
    IVP_ASSERT_EQ(ivp_min_hash_count(&h), 0);
    IVP_ASSERT_TRUE(ivp_min_hash_find_min_value(&h) == 0.0);
    ivp_min_hash_destroy(&h);

    /* not 2^x: the reference debug build asserts (ivu_min_hash.cxx:42) */
    ivp_set_assert_handler(count_assert, NULL);
    g_assert_count = 0;
    IVP_ASSERT_TRUE(ivp_min_hash_init(&h, NULL, 12) != IVP_OK);
    ivp_set_assert_handler(NULL, NULL);
#ifdef IVP_ENABLE_ASSERTS
    IVP_ASSERT_EQ(g_assert_count, 1);
#endif
}

IVP_TEST(ref_string_find_and_cmp) {
    const char *s = "Hello World of Physics";
    static const char *keys[7] = { "World", "world", "W?rld", "w?RLD", "xyz", "", "Physics" };
    static const int r_find[7][4] = {
        { 6, 6, 6, 6 }, { -1, 6, -1, 6 }, { -1, -1, 6, 6 }, { -1, -1, -1, 6 },
        { -1, -1, -1, -1 }, { 0, 0, 0, 0 }, { 15, 15, 15, 15 } };
    for (int k = 0; k < 7; k++) {
        for (int mode = 0; mode < 4; mode++) {
            const char *r = ivp_string_find(s, keys[k], mode);
            IVP_ASSERT_EQ(r ? (int)(r - s) : -1, r_find[k][mode]);
        }
    }
    static const char *pats[12] = { "Hello*", "*Physics", "*World*", "H?llo W*", "hello*",
                                    "Hello World of Physics", "Hello", "Hello World of Physics!",
                                    "*of*ics", "*xyz*", "abc*Physics", "*" };
    static const int r_cmp[12][2] = { {0,0},{0,0},{0,0},{0,0},{1,0},{0,0},{32,32},{33,33},
                                      {0,0},{-1,-1},{1,1},{0,0} };
    for (int k = 0; k < 12; k++) {
        IVP_ASSERT_EQ(ivp_string_cmp(s, pats[k], false), r_cmp[k][0]);
        IVP_ASSERT_EQ(ivp_string_cmp(s, pats[k], true), r_cmp[k][1]);
    }
    char up[] = "MiXed case 123 zz";
    ivp_string_uppercase(up);
    IVP_ASSERT_TRUE(strcmp(up, "MIXED CASE 123 ZZ") == 0);

    IVP_ASSERT_EQ(ivp_p_strcmp(NULL, NULL), 0);
    IVP_ASSERT_EQ(ivp_p_strcmp(NULL, "a"), 1);
    IVP_ASSERT_EQ(ivp_p_strcmp("a", NULL), -1);
    IVP_ASSERT_EQ((int)ivp_p_strlen(NULL), 0);
    IVP_ASSERT_EQ(ivp_p_atoi("42"), 42);
    IVP_ASSERT_TRUE(ivp_p_atof("-.5") == -0.5);
    char tok[] = "a, b\r;c";
    IVP_ASSERT_TRUE(strcmp(ivp_p_str_tok(tok, IVP_WHITESPACE), "a") == 0);
    IVP_ASSERT_TRUE(strcmp(ivp_p_str_tok(NULL, IVP_WHITESPACE), "b") == 0); /* '\r' stripped */
    char *m = ivp_string_make(NULL, "%s-%d", "obj", 7);
    IVP_ASSERT_TRUE(strcmp(m, "obj-7") == 0);
    ivp_string_free(NULL, m);
}

IVP_TEST(ref_string_hash) {
    ivp_string_hash_t sh;
    IVP_ASSERT_EQ(ivp_string_hash_init(&sh, NULL, 13, (void *)(uintptr_t)0x7), IVP_OK);
    static const char *names[7] = { "alpha", "beta", "gamma", "delta", "current_time", "double_null", "x" };
    static const int r_idx[7] = { 4, 10, 5, 1, 10, 8, 10 };
    for (int i = 0; i < 7; i++) {
        IVP_ASSERT_EQ(ivp_string_hash_index(&sh, names[i]), r_idx[i]);
        ivp_string_hash_add(&sh, names[i], (void *)(uintptr_t)(100 + i));
    }
    ivp_string_hash_add(&sh, "beta", (void *)(uintptr_t)200);
    IVP_ASSERT_EQ((int)(uintptr_t)ivp_string_hash_find(&sh, "beta"), 200);
    IVP_ASSERT_EQ((int)(uintptr_t)ivp_string_hash_find(&sh, "x"), 106);
    IVP_ASSERT_EQ((int)(uintptr_t)ivp_string_hash_find(&sh, "nope"), 7);
    ivp_string_hash_remove(&sh, "beta");
    IVP_ASSERT_EQ((int)(uintptr_t)ivp_string_hash_find(&sh, "beta"), 101);
    ivp_string_hash_destroy(&sh);
}

static int g_added, g_removed, g_deleted;
static void on_added(void *ctx, ivp_set_active_t *s, void *e) { (void)ctx; (void)s; (void)e; g_added++; }
static void on_removed(void *ctx, ivp_set_active_t *s, void *e) { (void)ctx; (void)s; (void)e; g_removed++; }
static void on_deleted(void *ctx, ivp_set_active_t *s) { (void)ctx; (void)s; g_deleted++; }

IVP_TEST(set_and_active_set) {
    ivp_set_t s;
    ivp_set_init(&s, NULL, 4);
    for (int i = 0; i < 20; i++) ivp_set_add(&s, fake(i));
    ivp_set_install(&s, fake(3));
    IVP_ASSERT_EQ(ivp_set_count(&s), 20);
    IVP_ASSERT_TRUE(ivp_set_find(&s, fake(7)) == fake(7));
    ivp_set_remove(&s, fake(7));
    IVP_ASSERT_NULL(ivp_set_find(&s, fake(7)));
    ivp_set_enumerator_t en;
    ivp_set_enumerator_init(&en, &s);
    int n = 0;
    while (ivp_set_enumerator_next(&en, &s)) n++;
    IVP_ASSERT_EQ(n, 19);
    ivp_set_destroy(&s);

    ivp_set_active_t as;
    ivp_set_active_init(&as, NULL, 8);
    ivp_set_active_listener_t l = { on_added, on_removed, on_deleted, NULL };
    ivp_set_active_add_listener(&as, &l);
    g_added = g_removed = g_deleted = 0;
    ivp_set_active_add(&as, fake(1));
    ivp_set_active_install(&as, fake(1)); /* already present: no event */
    ivp_set_active_install(&as, fake(2));
    ivp_set_active_remove(&as, fake(1));
    IVP_ASSERT_EQ(g_added, 2);
    IVP_ASSERT_EQ(g_removed, 1);
    ivp_set_active_destroy(&as);
    IVP_ASSERT_EQ(g_deleted, 1);
}

static unsigned int dh_base_index(const void *b, const void *ref) {
    (void)ref;
    return (unsigned int)((uintptr_t)b >> 4) * 7u;
}
static unsigned int dh_search_index(const void *s) {
    return (unsigned int)((uintptr_t)s >> 4) * 7u;
}
static bool dh_equal(const void *b, const void *s) { return b == s; }

IVP_TEST(diff_hash_resorts_found_to_front) {
    void *base[8];
    for (int i = 0; i < 8; i++) base[i] = fake(i);
    short buffer[32];
    ivp_diff_hash_t dh;
    IVP_ASSERT_EQ(ivp_diff_hash_init(&dh, NULL, base, 8, buffer, 32, NULL,
                                     dh_base_index, dh_search_index, dh_equal), IVP_OK);
    IVP_ASSERT_TRUE(ivp_diff_hash_check(&dh, fake(5), NULL) == fake(5));
    IVP_ASSERT_TRUE(ivp_diff_hash_check(&dh, fake(2), NULL) == fake(2));
    IVP_ASSERT_NULL(ivp_diff_hash_check(&dh, fake(42), NULL));
    IVP_ASSERT_TRUE(ivp_diff_hash_check(&dh, fake(7), NULL) == fake(7));
    IVP_ASSERT_EQ(dh.n_found_objects, 3);
    /* found elements moved to the front in check order, the displaced
     * ones took their places */
    IVP_ASSERT_TRUE(base[0] == fake(5));
    IVP_ASSERT_TRUE(base[1] == fake(2));
    IVP_ASSERT_TRUE(base[2] == fake(7));
    IVP_ASSERT_TRUE(base[5] == fake(0));
    IVP_ASSERT_TRUE(base[7] == fake(1)); /* moved twice: slot 1 -> 2 -> 7 */
    /* every element still findable through the hash */
    for (int i = 0; i < 8; i++) {
        if (i == 5 || i == 2 || i == 7) continue;
        IVP_ASSERT_TRUE(ivp_diff_hash_check(&dh, fake(i), NULL) == fake(i));
    }
    IVP_ASSERT_EQ(dh.n_found_objects, 8);
    ivp_diff_hash_destroy(&dh);

    /* small buffer → private buffer */
    for (int i = 0; i < 8; i++) base[i] = fake(i);
    IVP_ASSERT_EQ(ivp_diff_hash_init(&dh, NULL, base, 8, buffer, 8, NULL,
                                     dh_base_index, dh_search_index, dh_equal), IVP_OK);
    IVP_ASSERT_TRUE(dh.private_buffer);
    IVP_ASSERT_EQ(dh.buffersize_minus_one, 31);
    IVP_ASSERT_TRUE(ivp_diff_hash_check(&dh, fake(3), NULL) == fake(3));
    ivp_diff_hash_destroy(&dh);
}

IVP_TEST(diff_hash_rejects_invalid_capacity_and_configuration) {
    void *base[1] = { fake(0) };
    short buffer[4];
    ivp_diff_hash_t dh;

    IVP_ASSERT_EQ(ivp_diff_hash_init(NULL, NULL, base, 1, buffer, 4, NULL,
                                     dh_base_index, dh_search_index, dh_equal),
                  IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_diff_hash_init(&dh, NULL, base, -1, buffer, 4, NULL,
                                     dh_base_index, dh_search_index, dh_equal),
                  IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_diff_hash_init(&dh, NULL, base, SHRT_MAX + 2,
                                     NULL, 0, NULL, dh_base_index,
                                     dh_search_index, dh_equal),
                  IVP_ERR_CAPACITY_EXCEEDED);
    IVP_ASSERT_EQ(ivp_diff_hash_init(&dh, NULL, base, 1, buffer, 3, NULL,
                                     dh_base_index, dh_search_index, dh_equal),
                  IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_diff_hash_init(&dh, NULL, base, 1, buffer, 4, NULL,
                                     NULL, dh_search_index, dh_equal),
                  IVP_ERR_INVALID_ARG);

    IVP_ASSERT_EQ(ivp_diff_hash_init(&dh, NULL, base, 1, buffer, 4, NULL,
                                     dh_base_index, dh_search_index, dh_equal),
                  IVP_OK);
    IVP_ASSERT_TRUE(ivp_diff_hash_check(&dh, fake(0), NULL) == fake(0));
    IVP_ASSERT_NULL(ivp_diff_hash_check(&dh, fake(0), NULL));
    ivp_diff_hash_destroy(&dh);
    ivp_diff_hash_destroy(&dh);
    ivp_diff_hash_destroy(NULL);
}

int main(void) {
    IVP_REGISTER(ref_min_hash_tie_order);
    IVP_REGISTER(ref_string_find_and_cmp);
    IVP_REGISTER(ref_string_hash);
    IVP_REGISTER(set_and_active_set);
    IVP_REGISTER(diff_hash_resorts_found_to_front);
    IVP_REGISTER(diff_hash_rejects_invalid_capacity_and_configuration);
    IVP_RUN_ALL();
}
