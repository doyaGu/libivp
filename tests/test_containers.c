/* test_containers.c — Unit tests for IVP C17 container module */
#include "ivp_test.h"
#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/containers/ivp_hashmap.h>
#include <ivp/containers/ivp_minheap.h>
#include <ivp/containers/ivp_pvec.h>
#include <limits.h>

/* Dummy values for pointer tests */
static int dummy_vals[64];

/* ═══════════════════════════════════════════════════════════════════════════
 * Pointer vector tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(pvec_growth_and_capacity_failure_are_atomic) {
    ivp_pvec_t vec;
    ivp_pvec_init(&vec);
    IVP_ASSERT_TRUE(ivp_pvec_reserve(&vec, 9));
    IVP_ASSERT_TRUE(vec.cap >= 9);
    IVP_ASSERT_EQ(ivp_pvec_add(&vec, &dummy_vals[0]), 0);
    IVP_ASSERT_EQ(ivp_pvec_add(&vec, &dummy_vals[1]), 1);
    IVP_ASSERT_EQ(ivp_pvec_len(&vec), 2);
    ivp_pvec_clear(&vec);

    vec.elems = NULL;
    vec.n = INT_MAX;
    vec.cap = INT_MAX;
    IVP_ASSERT_EQ(ivp_pvec_add(&vec, &dummy_vals[0]), -1);
    IVP_ASSERT_EQ(vec.n, INT_MAX);
    IVP_ASSERT_EQ(vec.cap, INT_MAX);
    IVP_ASSERT_NULL(vec.elems);
    vec.n = 0;
    vec.cap = 0;
}

IVP_TEST(pvec_uses_supplied_allocator) {
    ivp_allocator_t counting = ivp_counting_allocator_create(NULL);
    ivp_pvec_t vec;
    ivp_pvec_init_with_allocator(&vec, &counting);

    IVP_ASSERT_EQ(ivp_pvec_add(&vec, &dummy_vals[0]), 0);
    IVP_ASSERT_EQ(ivp_pvec_add(&vec, &dummy_vals[1]), 1);

    ivp_alloc_stats_t live;
    ivp_counting_allocator_get_stats(&counting, &live);
    IVP_ASSERT_TRUE(live.current_bytes > 0);

    ivp_pvec_clear(&vec);
    ivp_alloc_stats_t cleared;
    ivp_counting_allocator_get_stats(&counting, &cleared);
    IVP_ASSERT_EQ(cleared.current_bytes, 0);
    ivp_counting_allocator_destroy(&counting);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Hashmap tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(hashmap_put_get) {
    ivp_hashmap_t map;
    ivp_result_t r = ivp_hashmap_init(&map, NULL, 16,
                                       ivp_hash_u32, ivp_eq_memcmp, sizeof(uint32_t));
    IVP_ASSERT_TRUE(ivp_ok(r));

    uint32_t keys[8];
    int values[8];
    for (int i = 0; i < 8; i++) {
        keys[i] = (uint32_t)(i * 7 + 1);
        values[i] = i * 100;
        r = ivp_hashmap_put(&map, &keys[i], &values[i]);
        IVP_ASSERT_TRUE(ivp_ok(r));
    }

    IVP_ASSERT_EQ(ivp_hashmap_count(&map), 8);

    for (int i = 0; i < 8; i++) {
        void *v = ivp_hashmap_get(&map, &keys[i]);
        IVP_ASSERT_NOT_NULL(v);
        IVP_ASSERT_EQ(*(int *)v, i * 100);
    }

    /* Non-existent key */
    uint32_t missing_key = 999;
    IVP_ASSERT_NULL(ivp_hashmap_get(&map, &missing_key));

    ivp_hashmap_destroy(&map);
}

IVP_TEST(hashmap_update) {
    ivp_hashmap_t map;
    ivp_hashmap_init(&map, NULL, 16, ivp_hash_u32, ivp_eq_memcmp, sizeof(uint32_t));

    uint32_t key = 42;
    int val1 = 100;
    int val2 = 200;

    ivp_hashmap_put(&map, &key, &val1);
    IVP_ASSERT_EQ(*(int *)ivp_hashmap_get(&map, &key), 100);

    /* Update same key */
    ivp_hashmap_put(&map, &key, &val2);
    IVP_ASSERT_EQ(*(int *)ivp_hashmap_get(&map, &key), 200);
    IVP_ASSERT_EQ(ivp_hashmap_count(&map), 1);

    ivp_hashmap_destroy(&map);
}

IVP_TEST(hashmap_remove) {
    ivp_hashmap_t map;
    ivp_hashmap_init(&map, NULL, 16, ivp_hash_u32, ivp_eq_memcmp, sizeof(uint32_t));

    uint32_t keys[4] = {10, 20, 30, 40};
    int values[4] = {100, 200, 300, 400};
    for (int i = 0; i < 4; i++) {
        ivp_hashmap_put(&map, &keys[i], &values[i]);
    }

    void *removed = ivp_hashmap_remove(&map, &keys[1]);
    IVP_ASSERT_NOT_NULL(removed);
    IVP_ASSERT_EQ(*(int *)removed, 200);
    IVP_ASSERT_EQ(ivp_hashmap_count(&map), 3);

    /* Should not find removed key anymore */
    IVP_ASSERT_NULL(ivp_hashmap_get(&map, &keys[1]));

    /* Other keys should still be findable */
    IVP_ASSERT_NOT_NULL(ivp_hashmap_get(&map, &keys[0]));
    IVP_ASSERT_NOT_NULL(ivp_hashmap_get(&map, &keys[2]));
    IVP_ASSERT_NOT_NULL(ivp_hashmap_get(&map, &keys[3]));

    ivp_hashmap_destroy(&map);
}

IVP_TEST(hashmap_contains) {
    ivp_hashmap_t map;
    ivp_hashmap_init(&map, NULL, 16, ivp_hash_u32, ivp_eq_memcmp, sizeof(uint32_t));

    uint32_t key = 55;
    int val = 1;
    ivp_hashmap_put(&map, &key, &val);

    IVP_ASSERT_TRUE(ivp_hashmap_contains(&map, &key));

    uint32_t miss = 99;
    IVP_ASSERT_FALSE(ivp_hashmap_contains(&map, &miss));

    ivp_hashmap_destroy(&map);
}

IVP_TEST(hashmap_rehash) {
    ivp_hashmap_t map;
    ivp_hashmap_init(&map, NULL, 16, ivp_hash_u32, ivp_eq_memcmp, sizeof(uint32_t));

    /* Insert enough entries to trigger rehash (>50% of 16 = 8 entries) */
    uint32_t keys[20];
    int values[20];
    for (int i = 0; i < 20; i++) {
        keys[i] = (uint32_t)i;
        values[i] = i * 10;
        ivp_result_t r = ivp_hashmap_put(&map, &keys[i], &values[i]);
        IVP_ASSERT_TRUE(ivp_ok(r));
    }

    IVP_ASSERT_EQ(ivp_hashmap_count(&map), 20);

    /* All entries should still be findable after rehash */
    for (int i = 0; i < 20; i++) {
        void *v = ivp_hashmap_get(&map, &keys[i]);
        IVP_ASSERT_NOT_NULL(v);
        IVP_ASSERT_EQ(*(int *)v, i * 10);
    }

    ivp_hashmap_destroy(&map);
}

IVP_TEST(hashmap_clear) {
    ivp_hashmap_t map;
    ivp_hashmap_init(&map, NULL, 16, ivp_hash_u32, ivp_eq_memcmp, sizeof(uint32_t));

    uint32_t key = 1;
    int val = 42;
    ivp_hashmap_put(&map, &key, &val);
    IVP_ASSERT_EQ(ivp_hashmap_count(&map), 1);

    ivp_hashmap_clear(&map);
    IVP_ASSERT_EQ(ivp_hashmap_count(&map), 0);
    IVP_ASSERT_NULL(ivp_hashmap_get(&map, &key));

    ivp_hashmap_destroy(&map);
}

IVP_TEST(hashmap_pair_key) {
    ivp_hashmap_t map;
    ivp_hashmap_init(&map, NULL, 16, ivp_hash_pair, ivp_eq_pair,
                     sizeof(ivp_pair_key_t));

    ivp_pair_key_t pk1 = ivp_pair_key(5, 10);
    ivp_pair_key_t pk2 = ivp_pair_key(10, 5);  /* should normalize to same */
    int val = 42;

    ivp_hashmap_put(&map, &pk1, &val);
    IVP_ASSERT_TRUE(ivp_hashmap_contains(&map, &pk2));

    void *v = ivp_hashmap_get(&map, &pk2);
    IVP_ASSERT_NOT_NULL(v);
    IVP_ASSERT_EQ(*(int *)v, 42);

    ivp_hashmap_destroy(&map);
}

IVP_TEST(hashmap_reserve) {
    ivp_hashmap_t map;
    ivp_result_t r = ivp_hashmap_init(&map, NULL, 4, ivp_hash_u32, ivp_eq_memcmp, sizeof(uint32_t));
    IVP_ASSERT_TRUE(ivp_ok(r));
    r = ivp_hashmap_reserve(&map, 100);
    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_TRUE(ivp_hashmap_cap(&map) >= 200);
    ivp_hashmap_destroy(&map);
}

IVP_TEST(hashmap_init_invalid_callbacks) {
    ivp_hashmap_t map;

    ivp_result_t r = ivp_hashmap_init(&map, NULL, 16,
                                      NULL, ivp_eq_memcmp,
                                      sizeof(uint32_t));
    IVP_ASSERT_FALSE(ivp_ok(r));

    r = ivp_hashmap_init(&map, NULL, 16,
                         ivp_hash_u32, NULL,
                         sizeof(uint32_t));
    IVP_ASSERT_FALSE(ivp_ok(r));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MinHeap tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(minheap_push_pop) {
    ivp_minheap_t heap;
    ivp_result_t r = ivp_minheap_init(&heap, NULL, 16, NULL);
    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_TRUE(ivp_minheap_empty(&heap));

    /* Push in random order */
    ivp_minheap_push(&heap, 5.0, &dummy_vals[5]);
    ivp_minheap_push(&heap, 2.0, &dummy_vals[2]);
    ivp_minheap_push(&heap, 8.0, &dummy_vals[8]);
    ivp_minheap_push(&heap, 1.0, &dummy_vals[1]);
    ivp_minheap_push(&heap, 3.0, &dummy_vals[3]);

    IVP_ASSERT_EQ(ivp_minheap_count(&heap), 5);

    /* Pop should come out in ascending order */
    ivp_heap_entry_t entry;
    r = ivp_minheap_pop(&heap, &entry);
    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_NEAR(entry.key, 1.0, 1e-10);
    IVP_ASSERT_EQ(entry.value, &dummy_vals[1]);

    ivp_minheap_pop(&heap, &entry);
    IVP_ASSERT_NEAR(entry.key, 2.0, 1e-10);

    ivp_minheap_pop(&heap, &entry);
    IVP_ASSERT_NEAR(entry.key, 3.0, 1e-10);

    ivp_minheap_pop(&heap, &entry);
    IVP_ASSERT_NEAR(entry.key, 5.0, 1e-10);

    ivp_minheap_pop(&heap, &entry);
    IVP_ASSERT_NEAR(entry.key, 8.0, 1e-10);

    IVP_ASSERT_TRUE(ivp_minheap_empty(&heap));

    ivp_minheap_destroy(&heap);
}

IVP_TEST(minheap_peek) {
    ivp_minheap_t heap;
    ivp_minheap_init(&heap, NULL, 16, NULL);

    ivp_minheap_push(&heap, 10.0, &dummy_vals[0]);
    ivp_minheap_push(&heap, 3.0, &dummy_vals[1]);
    ivp_minheap_push(&heap, 7.0, &dummy_vals[2]);

    ivp_heap_entry_t *top = ivp_minheap_peek(&heap);
    IVP_ASSERT_NOT_NULL(top);
    IVP_ASSERT_NEAR(top->key, 3.0, 1e-10);

    ivp_minheap_destroy(&heap);
}

IVP_TEST(minheap_update_key) {
    ivp_minheap_t heap;
    ivp_minheap_init(&heap, NULL, 16, NULL);

    ivp_minheap_push(&heap, 5.0, &dummy_vals[0]);
    ivp_minheap_push(&heap, 3.0, &dummy_vals[1]);
    ivp_minheap_push(&heap, 7.0, &dummy_vals[2]);

    /* The min is 3.0 at some index. Let's update it to 10.0 */
    /* Find the entry with key=3 */
    ivp_heap_entry_t *top = ivp_minheap_peek(&heap);
    IVP_ASSERT_NEAR(top->key, 3.0, 1e-10);

    /* Update index 0 (the min) to a large value */
    ivp_minheap_update_key(&heap, 0, 10.0);

    /* New min should be 5.0 */
    top = ivp_minheap_peek(&heap);
    IVP_ASSERT_NEAR(top->key, 5.0, 1e-10);

    ivp_minheap_destroy(&heap);
}

IVP_TEST(minheap_remove) {
    ivp_minheap_t heap;
    ivp_minheap_init(&heap, NULL, 16, NULL);

    ivp_minheap_push(&heap, 5.0, &dummy_vals[0]);
    ivp_minheap_push(&heap, 3.0, &dummy_vals[1]);
    ivp_minheap_push(&heap, 7.0, &dummy_vals[2]);
    ivp_minheap_push(&heap, 1.0, &dummy_vals[3]);

    /* Remove the root (1.0) */
    ivp_minheap_remove(&heap, 0);
    IVP_ASSERT_EQ(ivp_minheap_count(&heap), 3);

    /* New min should be 3.0 */
    ivp_heap_entry_t *top = ivp_minheap_peek(&heap);
    IVP_ASSERT_NEAR(top->key, 3.0, 1e-10);

    ivp_minheap_destroy(&heap);
}

IVP_TEST(minheap_empty_pop) {
    ivp_minheap_t heap;
    ivp_minheap_init(&heap, NULL, 16, NULL);

    ivp_heap_entry_t entry;
    ivp_result_t r = ivp_minheap_pop(&heap, &entry);
    IVP_ASSERT_FALSE(ivp_ok(r));

    ivp_minheap_destroy(&heap);
}

static int s_heap_indices[64];

static void test_heap_index_cb(void *value, int new_index) {
    /* value = &dummy_vals[i], compute i */
    int idx = (int)((int *)value - dummy_vals);
    if (idx >= 0 && idx < 64) {
        s_heap_indices[idx] = new_index;
    }
}

IVP_TEST(minheap_index_callback) {
    memset(s_heap_indices, -1, sizeof(s_heap_indices));

    ivp_minheap_t heap;
    ivp_minheap_init(&heap, NULL, 16, test_heap_index_cb);

    ivp_minheap_push(&heap, 5.0, &dummy_vals[0]);
    ivp_minheap_push(&heap, 3.0, &dummy_vals[1]);
    ivp_minheap_push(&heap, 1.0, &dummy_vals[2]);

    /* After pushes, the minimum (dummy_vals[2]) should be at index 0 */
    IVP_ASSERT_EQ(s_heap_indices[2], 0);

    ivp_minheap_destroy(&heap);
}

IVP_TEST(minheap_many_elements) {
    ivp_minheap_t heap;
    ivp_minheap_init(&heap, NULL, 4, NULL);

    /* Push 50 elements in reverse order */
    for (int i = 49; i >= 0; i--) {
        ivp_minheap_push(&heap, (ivp_time_t)i, NULL);
    }
    IVP_ASSERT_EQ(ivp_minheap_count(&heap), 50);

    /* Pop all — should come out in order 0, 1, 2, ... */
    for (int i = 0; i < 50; i++) {
        ivp_heap_entry_t entry;
        ivp_minheap_pop(&heap, &entry);
        IVP_ASSERT_NEAR(entry.key, (double)i, 1e-10);
    }
    IVP_ASSERT_TRUE(ivp_minheap_empty(&heap));

    ivp_minheap_destroy(&heap);
}

IVP_TEST(minheap_reserve) {
    ivp_minheap_t heap;
    ivp_result_t r = ivp_minheap_init(&heap, NULL, 0, NULL);
    IVP_ASSERT_TRUE(ivp_ok(r));
    r = ivp_minheap_reserve(&heap, 128);
    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_TRUE(ivp_minheap_cap(&heap) >= 128);
    ivp_minheap_destroy(&heap);
}

/* ── Main ───────────────────────────────────────────────────────────────── */

int main(void) {
    /* Pointer vector */
    IVP_REGISTER(pvec_growth_and_capacity_failure_are_atomic);
    IVP_REGISTER(pvec_uses_supplied_allocator);

    /* Hashmap */
    IVP_REGISTER(hashmap_put_get);
    IVP_REGISTER(hashmap_update);
    IVP_REGISTER(hashmap_remove);
    IVP_REGISTER(hashmap_contains);
    IVP_REGISTER(hashmap_rehash);
    IVP_REGISTER(hashmap_clear);
    IVP_REGISTER(hashmap_pair_key);
    IVP_REGISTER(hashmap_reserve);
    IVP_REGISTER(hashmap_init_invalid_callbacks);

    /* MinHeap */
    IVP_REGISTER(minheap_push_pop);
    IVP_REGISTER(minheap_peek);
    IVP_REGISTER(minheap_update_key);
    IVP_REGISTER(minheap_remove);
    IVP_REGISTER(minheap_empty_pop);
    IVP_REGISTER(minheap_index_callback);
    IVP_REGISTER(minheap_many_elements);
    IVP_REGISTER(minheap_reserve);

    IVP_RUN_ALL();
}
