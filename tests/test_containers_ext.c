#include "ivp_test.h"
#include <ivp/containers/ivp_minlist.h>
#include <ivp/containers/ivp_vhash.h>
#include <ivp/containers/ivp_hashmap.h>
#include <ivp/utility/ivp_hash.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Vhash comparison function for pointer equality */
static bool ptr_compare(const void *a, const void *b) {
    return a == b;
}

/* Test data */
static int dummy_a = 1;
static int dummy_b = 2;
static int dummy_c = 3;
static int dummy_d = 4;
static int dummy_e = 5;

/* ========== Minlist Tests ========== */

IVP_TEST(minlist_init_destroy) {
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 16);

    IVP_ASSERT_FALSE(ivp_minlist_has_elements(&ml));
    IVP_ASSERT_EQ(ivp_minlist_count(&ml), 0);

    ivp_minlist_destroy(&ml);
}

IVP_TEST(minlist_add_single) {
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 16);

    ivp_minlist_add(&ml, &dummy_a, 5.0f);

    IVP_ASSERT_TRUE(ivp_minlist_has_elements(&ml));
    IVP_ASSERT_EQ(ivp_minlist_count(&ml), 1);
    IVP_ASSERT_NEAR(ivp_minlist_find_min_value(&ml), 5.0f, 0.001f);

    ivp_minlist_destroy(&ml);
}

IVP_TEST(minlist_add_sorted_order) {
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 16);

    ivp_minlist_add(&ml, &dummy_a, 5.0f);
    ivp_minlist_add(&ml, &dummy_b, 1.0f);
    ivp_minlist_add(&ml, &dummy_c, 3.0f);

    IVP_ASSERT_EQ(ivp_minlist_count(&ml), 3);
    IVP_ASSERT_NEAR(ivp_minlist_find_min_value(&ml), 1.0f, 0.001f);

    ivp_minlist_destroy(&ml);
}

IVP_TEST(minlist_remove) {
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 16);

    ivp_minlist_add(&ml, &dummy_a, 5.0f);
    ivp_minlist_add(&ml, &dummy_b, 1.0f);
    ivp_minlist_index_t idx_c = ivp_minlist_add(&ml, &dummy_c, 3.0f);

    /* Remove middle element (3.0) */
    ivp_minlist_remove(&ml, idx_c);

    IVP_ASSERT_EQ(ivp_minlist_count(&ml), 2);
    IVP_ASSERT_NEAR(ivp_minlist_find_min_value(&ml), 1.0f, 0.001f);

    ivp_minlist_destroy(&ml);
}

IVP_TEST(minlist_remove_min) {
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 16);

    ivp_minlist_add(&ml, &dummy_a, 5.0f);
    ivp_minlist_index_t idx_min = ivp_minlist_add(&ml, &dummy_b, 1.0f);
    ivp_minlist_add(&ml, &dummy_c, 3.0f);

    /* Remove minimum element (1.0) */
    ivp_minlist_remove(&ml, idx_min);

    IVP_ASSERT_EQ(ivp_minlist_count(&ml), 2);
    IVP_ASSERT_NEAR(ivp_minlist_find_min_value(&ml), 3.0f, 0.001f);

    ivp_minlist_destroy(&ml);
}

IVP_TEST(minlist_find_min_elem) {
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 16);

    ivp_minlist_add(&ml, &dummy_a, 5.0f);
    ivp_minlist_add(&ml, &dummy_b, 1.0f);
    ivp_minlist_add(&ml, &dummy_c, 3.0f);

    void *min_elem = ivp_minlist_find_min_elem(&ml);
    IVP_ASSERT_NOT_NULL(min_elem);
    IVP_ASSERT_TRUE(min_elem == &dummy_b);

    ivp_minlist_destroy(&ml);
}

IVP_TEST(minlist_enumerator) {
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 16);

    /* Add 5 elements in unsorted order */
    ivp_minlist_add(&ml, &dummy_a, 5.0f);
    ivp_minlist_add(&ml, &dummy_b, 1.0f);
    ivp_minlist_add(&ml, &dummy_c, 4.0f);
    ivp_minlist_add(&ml, &dummy_d, 2.0f);
    ivp_minlist_add(&ml, &dummy_e, 3.0f);

    /* Enumerate all elements - should be in ascending order */
    ivp_minlist_enumerator_t en;
    ivp_minlist_enum_init(&en, &ml);

    void *elem1 = ivp_minlist_enum_next(&en);
    IVP_ASSERT_TRUE(elem1 == &dummy_b); /* 1.0 */

    void *elem2 = ivp_minlist_enum_next(&en);
    IVP_ASSERT_TRUE(elem2 == &dummy_d); /* 2.0 */

    void *elem3 = ivp_minlist_enum_next(&en);
    IVP_ASSERT_TRUE(elem3 == &dummy_e); /* 3.0 */

    void *elem4 = ivp_minlist_enum_next(&en);
    IVP_ASSERT_TRUE(elem4 == &dummy_c); /* 4.0 */

    void *elem5 = ivp_minlist_enum_next(&en);
    IVP_ASSERT_TRUE(elem5 == &dummy_a); /* 5.0 */

    void *elem_null = ivp_minlist_enum_next(&en);
    IVP_ASSERT_NULL(elem_null);

    ivp_minlist_destroy(&ml);
}

IVP_TEST(minlist_enum_next_lt) {
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 16);

    /* Add 5 elements */
    ivp_minlist_add(&ml, &dummy_a, 5.0f);
    ivp_minlist_add(&ml, &dummy_b, 1.0f);
    ivp_minlist_add(&ml, &dummy_c, 4.0f);
    ivp_minlist_add(&ml, &dummy_d, 2.0f);
    ivp_minlist_add(&ml, &dummy_e, 3.0f);

    /* Enumerate elements with max_limit=3.5 */
    ivp_minlist_enumerator_t en;
    ivp_minlist_enum_init(&en, &ml);

    void *elem1 = ivp_minlist_enum_next_lt(&en, 3.5f);
    IVP_ASSERT_TRUE(elem1 == &dummy_b); /* 1.0 < 3.5 */

    void *elem2 = ivp_minlist_enum_next_lt(&en, 3.5f);
    IVP_ASSERT_TRUE(elem2 == &dummy_d); /* 2.0 < 3.5 */

    void *elem3 = ivp_minlist_enum_next_lt(&en, 3.5f);
    IVP_ASSERT_TRUE(elem3 == &dummy_e); /* 3.0 < 3.5 */

    void *elem_null = ivp_minlist_enum_next_lt(&en, 3.5f);
    IVP_ASSERT_NULL(elem_null); /* 4.0 >= 3.5, stops */

    ivp_minlist_destroy(&ml);
}

IVP_TEST(minlist_grow_many) {
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 4);

    int values[40];
    for (int i = 0; i < 40; i++) {
        values[i] = i;
        ivp_minlist_index_t idx = ivp_minlist_add(&ml, &values[i], (float)(40 - i));
        IVP_ASSERT_TRUE(idx != IVP_MINLIST_UNUSED);
    }

    IVP_ASSERT_EQ(ivp_minlist_count(&ml), 40);
    IVP_ASSERT_NEAR(ivp_minlist_find_min_value(&ml), 1.0f, 0.001f);

    ivp_minlist_enumerator_t en;
    ivp_minlist_enum_init(&en, &ml);
    for (int i = 39; i >= 0; i--) {
        void *elem = ivp_minlist_enum_next(&en);
        IVP_ASSERT_TRUE(elem == &values[i]);
    }
    IVP_ASSERT_NULL(ivp_minlist_enum_next(&en));

    ivp_minlist_destroy(&ml);
}

IVP_TEST(minlist_supports_32bit_indices) {
    enum { COUNT = 100001 };
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 4);

    ivp_minlist_index_t high = IVP_MINLIST_UNUSED;
    for (uint32_t i = 0; i < COUNT; i++) {
        ivp_minlist_index_t index = ivp_minlist_add(
            &ml, (void *)(uintptr_t)(i + 1u), (float)(COUNT - i));
        IVP_ASSERT_TRUE(index != IVP_MINLIST_UNUSED);
        if (index > UINT16_MAX) high = index;
    }
    IVP_ASSERT_TRUE(high > UINT16_MAX);
    IVP_ASSERT_EQ(ivp_minlist_count(&ml), (uint32_t)COUNT);

    for (uint32_t i = 0; i < COUNT; i++) ivp_minlist_remove(&ml, i);
    IVP_ASSERT_FALSE(ivp_minlist_has_elements(&ml));
    ivp_minlist_destroy(&ml);
}

IVP_TEST(minlist_equal_zero_and_index_reuse) {
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 4);
    ivp_minlist_index_t ia = ivp_minlist_add(&ml, &dummy_a, -0.0f);
    ivp_minlist_index_t ib = ivp_minlist_add(&ml, &dummy_b, 0.0f);
    ivp_minlist_index_t ic = ivp_minlist_add(&ml, &dummy_c, -0.0f);
    ivp_minlist_enumerator_t en;
    ivp_minlist_enum_init(&en, &ml);
    IVP_ASSERT_TRUE(ivp_minlist_enum_next(&en) == &dummy_c);
    IVP_ASSERT_TRUE(ivp_minlist_enum_next(&en) == &dummy_b);
    IVP_ASSERT_TRUE(ivp_minlist_enum_next(&en) == &dummy_a);
    IVP_ASSERT_TRUE(signbit(ivp_minlist_find_min_value(&ml)) != 0);

    ivp_minlist_remove(&ml, ib);
    ivp_minlist_index_t reused = ivp_minlist_add(&ml, &dummy_d, 1.0f);
    IVP_ASSERT_EQ(reused, ib);
    ivp_minlist_remove(&ml, ia);
    ivp_minlist_remove(&ml, ic);
    ivp_minlist_remove(&ml, reused);
    ivp_minlist_destroy(&ml);
}

IVP_TEST(minlist_nonfinite_values_use_reference_fallback) {
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 4);
    ivp_minlist_index_t finite_a = ivp_minlist_add(&ml, &dummy_a, 1.0f);
    ivp_minlist_index_t nan_value = ivp_minlist_add(&ml, &dummy_b, NAN);
    ivp_minlist_index_t finite_b = ivp_minlist_add(&ml, &dummy_c, 2.0f);
    ivp_minlist_index_t pos_inf = ivp_minlist_add(&ml, &dummy_d, INFINITY);
    ivp_minlist_index_t neg_inf = ivp_minlist_add(&ml, &dummy_e, -INFINITY);
    ivp_minlist_enumerator_t en;
    ivp_minlist_enum_init(&en, &ml);
    IVP_ASSERT_TRUE(ivp_minlist_enum_next(&en) == &dummy_e);
    IVP_ASSERT_TRUE(ivp_minlist_enum_next(&en) == &dummy_a);
    IVP_ASSERT_TRUE(ivp_minlist_enum_next(&en) == &dummy_c);
    IVP_ASSERT_TRUE(ivp_minlist_enum_next(&en) == &dummy_d);
    IVP_ASSERT_TRUE(ivp_minlist_enum_next(&en) == &dummy_b);
    IVP_ASSERT_NULL(ivp_minlist_enum_next(&en));
    ivp_minlist_remove(&ml, nan_value);
    ivp_minlist_remove(&ml, pos_inf);
    ivp_minlist_remove(&ml, neg_inf);
    ivp_minlist_remove(&ml, finite_b);
    ivp_minlist_remove(&ml, finite_a);
    ivp_minlist_destroy(&ml);
}

IVP_TEST(minlist_index_activates_for_finite_values) {
    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 4);
    for (int i = 0; i < 2048; ++i)
        ivp_minlist_add(&ml, (void *)(uintptr_t)(i + 1), (float)(i % 71));
    IVP_ASSERT_EQ(ml.nonfinite_count, 0);
    IVP_ASSERT_TRUE(ml.tree_active);
    ivp_minlist_destroy(&ml);

    ivp_minlist_init(&ml, ivp_default_allocator(), 4);
    ivp_minlist_index_t nan_index = ivp_minlist_add(&ml, &dummy_a, NAN);
    for (int i = 0; i < 2048; ++i)
        ivp_minlist_add(&ml, (void *)(uintptr_t)(i + 1), (float)i);
    IVP_ASSERT_EQ(ml.nonfinite_count, 1);
    IVP_ASSERT_FALSE(ml.tree_active);
    ivp_minlist_remove(&ml, nan_index);
    IVP_ASSERT_EQ(ml.nonfinite_count, 0);
    IVP_ASSERT_TRUE(ml.tree_active);
    ivp_minlist_destroy(&ml);
}

typedef struct minlist_model_entry {
    float value;
    uint32_t sequence;
    ivp_minlist_index_t handle;
    bool active;
} minlist_model_entry_t;

static uint32_t minlist_test_rand(uint32_t *state)
{
    *state = *state * UINT32_C(1664525) + UINT32_C(1013904223);
    return *state;
}

static void minlist_verify_model(ivp_minlist_t *ml, const minlist_model_entry_t *entries,
                                 const uint32_t *order, uint32_t order_count, int *_fail_count)
{
    IVP_ASSERT_EQ(ivp_minlist_count(ml), order_count);
    ivp_minlist_enumerator_t en;
    ivp_minlist_enum_init(&en, ml);
    for (uint32_t i = 0; i < order_count; i++) {
        void *actual = ivp_minlist_enum_next(&en);
        IVP_ASSERT_TRUE(actual == (void *)(uintptr_t)(order[i] + 1u));
        if (i > 0) {
            const minlist_model_entry_t *prev = &entries[order[i - 1u]];
            const minlist_model_entry_t *current = &entries[order[i]];
            IVP_ASSERT_TRUE(prev->value < current->value ||
                            (prev->value == current->value &&
                             prev->sequence > current->sequence));
        }
    }
    IVP_ASSERT_NULL(ivp_minlist_enum_next(&en));
}

IVP_TEST(minlist_randomized_differential) {
    enum { operations = 12000, max_entries = 7000 };
    minlist_model_entry_t *entries =
        (minlist_model_entry_t *)calloc(max_entries, sizeof(*entries));
    uint32_t *order = (uint32_t *)malloc(max_entries * sizeof(*order));
    IVP_ASSERT_NOT_NULL(entries);
    IVP_ASSERT_NOT_NULL(order);
    if (!entries || !order) {
        free(entries);
        free(order);
        return;
    }

    ivp_minlist_t ml;
    ivp_minlist_init(&ml, ivp_default_allocator(), 4);
    uint32_t random_state = UINT32_C(0x51a7b00b);
    uint32_t entry_count = 0;
    uint32_t order_count = 0;
    for (uint32_t op = 0; op < operations; op++) {
        bool add = order_count < 1400u ||
                   (entry_count < max_entries && (minlist_test_rand(&random_state) & 3u) != 0u);
        if (add) {
            uint32_t id = entry_count++;
            float value = (float)((int)(minlist_test_rand(&random_state) % 257u) - 128);
            entries[id].value = value;
            entries[id].sequence = id;
            entries[id].active = true;
            entries[id].handle = ivp_minlist_add(
                &ml, (void *)(uintptr_t)(id + 1u), value);
            IVP_ASSERT_TRUE(entries[id].handle != IVP_MINLIST_UNUSED);
            uint32_t pos = 0;
            while (pos < order_count && entries[order[pos]].value < value) pos++;
            memmove(&order[pos + 1u], &order[pos],
                    (size_t)(order_count - pos) * sizeof(*order));
            order[pos] = id;
            order_count++;
        } else {
            uint32_t pos = minlist_test_rand(&random_state) % order_count;
            uint32_t id = order[pos];
            ivp_minlist_remove(&ml, entries[id].handle);
            entries[id].active = false;
            memmove(&order[pos], &order[pos + 1u],
                    (size_t)(order_count - pos - 1u) * sizeof(*order));
            order_count--;
        }
        if ((op & 31u) == 0u) {
            minlist_verify_model(&ml, entries, order, order_count, _fail_count);
            if (*_fail_count != 0) return;
        }
    }
    minlist_verify_model(&ml, entries, order, order_count, _fail_count);
    if (*_fail_count != 0) return;
    while (order_count > 0) {
        uint32_t id = order[--order_count];
        ivp_minlist_remove(&ml, entries[id].handle);
    }
    ivp_minlist_destroy(&ml);
    free(order);
    free(entries);
}

/* ========== Vhash Tests ========== */

IVP_TEST(vhash_init_destroy) {
    ivp_vhash_t h;
    ivp_vhash_init(&h, ivp_default_allocator(), 16, ptr_compare);

    IVP_ASSERT_EQ(ivp_vhash_count(&h), 0);
    IVP_ASSERT_EQ(ivp_vhash_capacity(&h), 16);

    ivp_vhash_destroy(&h);
}

IVP_TEST(vhash_add_find) {
    ivp_vhash_t h;
    ivp_vhash_init(&h, ivp_default_allocator(), 16, ptr_compare);

    uint32_t hash = ivp_vhash_ptr_index(&dummy_a);
    ivp_vhash_add(&h, &dummy_a, hash);

    void *found = ivp_vhash_find(&h, &dummy_a, hash);
    IVP_ASSERT_NOT_NULL(found);
    IVP_ASSERT_TRUE(found == &dummy_a);

    ivp_vhash_destroy(&h);
}

IVP_TEST(vhash_find_not_found) {
    ivp_vhash_t h;
    ivp_vhash_init(&h, ivp_default_allocator(), 16, ptr_compare);

    uint32_t hash = ivp_vhash_ptr_index(&dummy_a);
    ivp_vhash_add(&h, &dummy_a, hash);

    /* Try to find a different element */
    uint32_t hash_b = ivp_vhash_ptr_index(&dummy_b);
    void *found = ivp_vhash_find(&h, &dummy_b, hash_b);
    IVP_ASSERT_NULL(found);

    ivp_vhash_destroy(&h);
}

IVP_TEST(vhash_remove) {
    ivp_vhash_t h;
    ivp_vhash_init(&h, ivp_default_allocator(), 16, ptr_compare);

    uint32_t hash = ivp_vhash_ptr_index(&dummy_a);
    ivp_vhash_add(&h, &dummy_a, hash);

    void *removed = ivp_vhash_remove(&h, &dummy_a, hash);
    IVP_ASSERT_NOT_NULL(removed);
    IVP_ASSERT_TRUE(removed == &dummy_a);

    /* Verify element is no longer found */
    void *found = ivp_vhash_find(&h, &dummy_a, hash);
    IVP_ASSERT_NULL(found);

    ivp_vhash_destroy(&h);
}

IVP_TEST(vhash_touch_untouch) {
    ivp_vhash_t h;
    ivp_vhash_init(&h, ivp_default_allocator(), 16, ptr_compare);

    uint32_t hash_a = ivp_vhash_ptr_index(&dummy_a);
    uint32_t hash_b = ivp_vhash_ptr_index(&dummy_b);

    ivp_vhash_add(&h, &dummy_a, hash_a);
    ivp_vhash_add(&h, &dummy_b, hash_b);

    /* Touch first element */
    void *touched = ivp_vhash_touch(&h, &dummy_a, hash_a);
    IVP_ASSERT_NOT_NULL(touched);
    IVP_ASSERT_TRUE(touched == &dummy_a);

    /* Untouch all elements */
    ivp_vhash_untouch_all(&h);

    /* After untouch, we can't directly verify touched state from outside,
       but the function should execute without error */
    IVP_ASSERT_EQ(ivp_vhash_count(&h), 2);

    ivp_vhash_destroy(&h);
}

IVP_TEST(vhash_store_add_find_remove) {
    ivp_vhash_store_t store;
    ivp_vhash_store_init(&store, ivp_default_allocator(), 16);

    /* Add key-value pair */
    ivp_vhash_store_add(&store, &dummy_a, &dummy_b);

    /* Find value by key */
    void *value = ivp_vhash_store_find(&store, &dummy_a);
    IVP_ASSERT_NOT_NULL(value);
    IVP_ASSERT_TRUE(value == &dummy_b);

    /* Remove key-value pair */
    void *removed_value = ivp_vhash_store_remove(&store, &dummy_a);
    IVP_ASSERT_NOT_NULL(removed_value);
    IVP_ASSERT_TRUE(removed_value == &dummy_b);

    /* Verify key no longer exists */
    void *not_found = ivp_vhash_store_find(&store, &dummy_a);
    IVP_ASSERT_NULL(not_found);

    ivp_vhash_store_destroy(&store);
}

static int g_store_assert_count = 0;
static void store_count_assert(const char *expr, const char *file, int line,
                               const char *func, const char *msg, void *userdata) {
    (void)expr; (void)file; (void)line; (void)func; (void)msg; (void)userdata;
    g_store_assert_count++;
}

IVP_TEST(vhash_store_null_value_collision_path) {
    ivp_vhash_store_t store;
    ivp_vhash_store_init(&store, ivp_default_allocator(), 16);

    uint32_t shared_hash = ivp_vhash_ptr_index(&dummy_a);
    ivp_vhash_store_add_hash(&store, &dummy_a, NULL, shared_hash);
    ivp_vhash_store_add_hash(&store, &dummy_b, &dummy_c, shared_hash);

    void *found_b = ivp_vhash_store_find_hash(&store, &dummy_b, shared_hash);
    IVP_ASSERT_NOT_NULL(found_b);
    IVP_ASSERT_TRUE(found_b == &dummy_c);

    /* IVP_VHash_Store::remove_elem asserts e->elem (a NULL value) */
    g_store_assert_count = 0;
    ivp_set_assert_handler(store_count_assert, NULL);
    void *removed_null = ivp_vhash_store_remove_hash(&store, &dummy_a, shared_hash);
    ivp_set_assert_handler(NULL, NULL);
#ifdef IVP_ENABLE_ASSERTS
    IVP_ASSERT_EQ(g_store_assert_count, 1);
#endif
    IVP_ASSERT_NULL(removed_null);
    IVP_ASSERT_EQ(ivp_vhash_store_count(&store), 1);

    found_b = ivp_vhash_store_find_hash(&store, &dummy_b, shared_hash);
    IVP_ASSERT_NOT_NULL(found_b);
    IVP_ASSERT_TRUE(found_b == &dummy_c);

    void *removed_b = ivp_vhash_store_remove_hash(&store, &dummy_b, shared_hash);
    IVP_ASSERT_NOT_NULL(removed_b);
    IVP_ASSERT_TRUE(removed_b == &dummy_c);
    IVP_ASSERT_EQ(ivp_vhash_store_count(&store), 0);

    ivp_vhash_store_destroy(&store);
}

/* ========== OOM Allocator for Resilience Tests ========== */

typedef struct oom_ctx {
    bool fail;  /* when true, alloc returns NULL */
} oom_ctx_t;

static void *oom_alloc(size_t size, void *ctx) {
    oom_ctx_t *oc = (oom_ctx_t *)ctx;
    if (oc->fail) return NULL;
    return malloc(size);
}

static void *oom_realloc(void *ptr, size_t old_size, size_t new_size, void *ctx) {
    (void)old_size;
    oom_ctx_t *oc = (oom_ctx_t *)ctx;
    if (oc->fail) return NULL;
    return realloc(ptr, new_size);
}

static void oom_free(void *ptr, size_t size, void *ctx) {
    (void)size;
    (void)ctx;
    free(ptr);
}

/* ========== OOM Resilience Tests ========== */

IVP_TEST(vhash_add_survives_rehash_oom) {
    oom_ctx_t oc = { .fail = false };
    ivp_allocator_t alloc = { oom_alloc, oom_realloc, oom_free, &oc };

    ivp_vhash_t h;
    ivp_vhash_init(&h, &alloc, 4, ptr_compare);

    /* Fill to 50% load (2 out of 4 slots) — next add triggers rehash */
    uint32_t hash_a = ivp_vhash_ptr_index(&dummy_a);
    uint32_t hash_b = ivp_vhash_ptr_index(&dummy_b);
    ivp_vhash_add(&h, &dummy_a, hash_a);
    ivp_vhash_add(&h, &dummy_b, hash_b);
    IVP_ASSERT_EQ(ivp_vhash_count(&h), 2);

    /* Make allocator fail — rehash will silently fail, add should not hang */
    oc.fail = true;
    uint32_t hash_c = ivp_vhash_ptr_index(&dummy_c);
    ivp_vhash_add(&h, &dummy_c, hash_c);

    /* Existing entries should remain intact */
    oc.fail = false;
    void *found_a = ivp_vhash_find(&h, &dummy_a, hash_a);
    IVP_ASSERT_NOT_NULL(found_a);
    IVP_ASSERT_TRUE(found_a == &dummy_a);
    void *found_b = ivp_vhash_find(&h, &dummy_b, hash_b);
    IVP_ASSERT_NOT_NULL(found_b);
    IVP_ASSERT_TRUE(found_b == &dummy_b);

    ivp_vhash_destroy(&h);
}

IVP_TEST(vhash_store_add_survives_rehash_oom) {
    oom_ctx_t oc = { .fail = false };
    ivp_allocator_t alloc = { oom_alloc, oom_realloc, oom_free, &oc };

    ivp_vhash_store_t store;
    ivp_vhash_store_init(&store, &alloc, 4);

    /* Fill to 50% load (2 out of 4 slots) — next add triggers rehash */
    ivp_vhash_store_add(&store, &dummy_a, &dummy_b);
    ivp_vhash_store_add(&store, &dummy_c, &dummy_d);
    IVP_ASSERT_EQ(ivp_vhash_store_count(&store), 2);

    /* Make allocator fail — rehash will silently fail, add should not hang */
    oc.fail = true;
    ivp_vhash_store_add(&store, &dummy_e, &dummy_a);

    /* Existing entries should remain intact */
    oc.fail = false;
    void *found_a = ivp_vhash_store_find(&store, &dummy_a);
    IVP_ASSERT_NOT_NULL(found_a);
    IVP_ASSERT_TRUE(found_a == &dummy_b);
    void *found_c = ivp_vhash_store_find(&store, &dummy_c);
    IVP_ASSERT_NOT_NULL(found_c);
    IVP_ASSERT_TRUE(found_c == &dummy_d);

    ivp_vhash_store_destroy(&store);
}

IVP_TEST(hashmap_put_returns_error_on_rehash_oom) {
    oom_ctx_t oc = { .fail = false };
    ivp_allocator_t alloc = { oom_alloc, oom_realloc, oom_free, &oc };

    ivp_hashmap_t map;
    ivp_result_t r = ivp_hashmap_init(&map, &alloc, 16,
                                       ivp_hash_u32, ivp_eq_memcmp, sizeof(uint32_t));
    IVP_ASSERT_TRUE(ivp_ok(r));

    /* Fill to exactly 50% load (8 out of 16) — next put triggers rehash */
    uint32_t keys[8];
    for (int i = 0; i < 8; i++) {
        keys[i] = (uint32_t)(i + 100);
        r = ivp_hashmap_put(&map, &keys[i], &dummy_a);
        IVP_ASSERT_TRUE(ivp_ok(r));
    }
    IVP_ASSERT_EQ(ivp_hashmap_count(&map), 8);

    /* Next put triggers rehash (count >= capacity/2). Make allocator fail. */
    oc.fail = true;
    uint32_t key_oom = 999;
    r = ivp_hashmap_put(&map, &key_oom, &dummy_b);
    IVP_ASSERT_TRUE(r == IVP_ERR_OUT_OF_MEMORY);

    /* Existing entries should remain intact */
    oc.fail = false;
    IVP_ASSERT_EQ(ivp_hashmap_count(&map), 8);
    for (int i = 0; i < 8; i++) {
        void *val = ivp_hashmap_get(&map, &keys[i]);
        IVP_ASSERT_NOT_NULL(val);
        IVP_ASSERT_TRUE(val == &dummy_a);
    }

    ivp_hashmap_destroy(&map);
}

/* ========== Main Test Runner ========== */

int main(void) {
    /* Minlist tests */
    IVP_REGISTER(minlist_init_destroy);
    IVP_REGISTER(minlist_add_single);
    IVP_REGISTER(minlist_add_sorted_order);
    IVP_REGISTER(minlist_remove);
    IVP_REGISTER(minlist_remove_min);
    IVP_REGISTER(minlist_find_min_elem);
    IVP_REGISTER(minlist_enumerator);
    IVP_REGISTER(minlist_enum_next_lt);
    IVP_REGISTER(minlist_grow_many);
    IVP_REGISTER(minlist_supports_32bit_indices);
    IVP_REGISTER(minlist_equal_zero_and_index_reuse);
    IVP_REGISTER(minlist_nonfinite_values_use_reference_fallback);
    IVP_REGISTER(minlist_index_activates_for_finite_values);
    IVP_REGISTER(minlist_randomized_differential);

    /* Vhash tests */
    IVP_REGISTER(vhash_init_destroy);
    IVP_REGISTER(vhash_add_find);
    IVP_REGISTER(vhash_find_not_found);
    IVP_REGISTER(vhash_remove);
    IVP_REGISTER(vhash_touch_untouch);
    IVP_REGISTER(vhash_store_add_find_remove);
    IVP_REGISTER(vhash_store_null_value_collision_path);

    /* OOM resilience tests */
    IVP_REGISTER(vhash_add_survives_rehash_oom);
    IVP_REGISTER(vhash_store_add_survives_rehash_oom);
    IVP_REGISTER(hashmap_put_returns_error_on_rehash_oom);

    IVP_RUN_ALL();
}
