/* test_containers.c — Unit tests for IVP C17 container module */
#include "ivp_test.h"
#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/containers/ivp_array.h>
#include <ivp/containers/ivp_hashmap.h>
#include <ivp/containers/ivp_minheap.h>

/* Dummy values for pointer tests */
static int dummy_vals[64];

static int s_lifecycle_init_count;
static int s_lifecycle_reset_count;
static int s_lifecycle_copy_count;
static int s_lifecycle_move_count;
static int s_lifecycle_dispose_count;

typedef struct registry_case {
    ivp_elem_type_id_t id;
    const char *name;
    size_t size;
    size_t align;
    uint32_t required_flags;
    const ivp_elem_typeinfo_t *expected;
} registry_case_t;

#define ASSERT_REGISTRY_CASE(c) do { \
    const ivp_elem_typeinfo_t *by_id__ = ivp_elem_typeinfo_from_id((c).id); \
    IVP_ASSERT_NOT_NULL(by_id__); \
    IVP_ASSERT_TRUE(by_id__ == (c).expected); \
    IVP_ASSERT_EQ(by_id__->type_id, (uint32_t)(c).id); \
    IVP_ASSERT_EQ(by_id__->size, (c).size); \
    IVP_ASSERT_EQ(by_id__->align, (c).align); \
    IVP_ASSERT_TRUE((by_id__->flags & (c).required_flags) == (c).required_flags); \
    const ivp_elem_typeinfo_t *by_name__ = ivp_elem_typeinfo_from_name((c).name); \
    IVP_ASSERT_TRUE(by_name__ == by_id__); \
    IVP_ASSERT_TRUE(ivp_elem_typeinfo_is_registered(by_id__)); \
} while (0)

static void lifecycle_on_init(void *elem, void *userdata) {
    IVP_UNUSED(elem);
    IVP_UNUSED(userdata);
    s_lifecycle_init_count++;
}

static void lifecycle_on_reset(void *elem, void *userdata) {
    IVP_UNUSED(elem);
    IVP_UNUSED(userdata);
    s_lifecycle_reset_count++;
}

static void lifecycle_on_copy(void *elem, int src_index, int dst_index, void *userdata) {
    IVP_UNUSED(elem);
    IVP_UNUSED(src_index);
    IVP_UNUSED(dst_index);
    IVP_UNUSED(userdata);
    s_lifecycle_copy_count++;
}

static void lifecycle_on_move(void *elem, int src_index, int dst_index, void *userdata) {
    IVP_UNUSED(elem);
    IVP_UNUSED(src_index);
    IVP_UNUSED(dst_index);
    IVP_UNUSED(userdata);
    s_lifecycle_move_count++;
}

static void lifecycle_on_dispose(void *elem, void *userdata) {
    IVP_UNUSED(elem);
    IVP_UNUSED(userdata);
    s_lifecycle_dispose_count++;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Array tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(array_push_pop) {
    ivp_array_t arr;
    ivp_result_t r = ivp_array_init(&arr, NULL, 4);
    IVP_ASSERT_TRUE(ivp_ok(r));

    IVP_ASSERT_EQ(ivp_array_len(&arr), 0);

    for (int i = 0; i < 10; i++) {
        r = ivp_array_push(&arr, &dummy_vals[i]);
        IVP_ASSERT_TRUE(ivp_ok(r));
    }
    IVP_ASSERT_EQ(ivp_array_len(&arr), 10);

    void *v = ivp_array_pop(&arr);
    IVP_ASSERT_EQ(v, &dummy_vals[9]);
    IVP_ASSERT_EQ(ivp_array_len(&arr), 9);

    ivp_array_destroy(&arr);
}

IVP_TEST(array_get_set) {
    ivp_array_t arr;
    ivp_array_init(&arr, NULL, 4);

    ivp_array_push(&arr, &dummy_vals[0]);
    ivp_array_push(&arr, &dummy_vals[1]);
    ivp_array_push(&arr, &dummy_vals[2]);

    IVP_ASSERT_EQ(ivp_array_get(&arr, 0), &dummy_vals[0]);
    IVP_ASSERT_EQ(ivp_array_get(&arr, 1), &dummy_vals[1]);
    IVP_ASSERT_EQ(ivp_array_get(&arr, 2), &dummy_vals[2]);

    ivp_array_set(&arr, 1, &dummy_vals[10]);
    IVP_ASSERT_EQ(ivp_array_get(&arr, 1), &dummy_vals[10]);

    ivp_array_destroy(&arr);
}

IVP_TEST(array_remove_ordered) {
    ivp_array_t arr;
    ivp_array_init(&arr, NULL, 8);

    for (int i = 0; i < 5; i++) {
        ivp_array_push(&arr, &dummy_vals[i]);
    }

    /* Remove middle element, order should be preserved */
    void *removed = ivp_array_remove_ordered(&arr, 2);
    IVP_ASSERT_EQ(removed, &dummy_vals[2]);
    IVP_ASSERT_EQ(ivp_array_len(&arr), 4);

    /* Verify order: 0, 1, 3, 4 */
    IVP_ASSERT_EQ(ivp_array_get(&arr, 0), &dummy_vals[0]);
    IVP_ASSERT_EQ(ivp_array_get(&arr, 1), &dummy_vals[1]);
    IVP_ASSERT_EQ(ivp_array_get(&arr, 2), &dummy_vals[3]);
    IVP_ASSERT_EQ(ivp_array_get(&arr, 3), &dummy_vals[4]);

    ivp_array_destroy(&arr);
}

IVP_TEST(array_remove_swap) {
    ivp_array_t arr;
    ivp_array_init(&arr, NULL, 8);

    for (int i = 0; i < 5; i++) {
        ivp_array_push(&arr, &dummy_vals[i]);
    }

    /* Remove index 1: last element (4) should be swapped in */
    void *removed = ivp_array_remove_swap(&arr, 1);
    IVP_ASSERT_EQ(removed, &dummy_vals[1]);
    IVP_ASSERT_EQ(ivp_array_len(&arr), 4);
    IVP_ASSERT_EQ(ivp_array_get(&arr, 1), &dummy_vals[4]);

    ivp_array_destroy(&arr);
}

IVP_TEST(array_insert) {
    ivp_array_t arr;
    ivp_array_init(&arr, NULL, 8);

    ivp_array_push(&arr, &dummy_vals[0]);
    ivp_array_push(&arr, &dummy_vals[2]);

    /* Insert at index 1 */
    ivp_result_t r = ivp_array_insert(&arr, 1, &dummy_vals[1]);
    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_EQ(ivp_array_len(&arr), 3);
    IVP_ASSERT_EQ(ivp_array_get(&arr, 0), &dummy_vals[0]);
    IVP_ASSERT_EQ(ivp_array_get(&arr, 1), &dummy_vals[1]);
    IVP_ASSERT_EQ(ivp_array_get(&arr, 2), &dummy_vals[2]);

    ivp_array_destroy(&arr);
}

IVP_TEST(array_find) {
    ivp_array_t arr;
    ivp_array_init(&arr, NULL, 8);

    for (int i = 0; i < 5; i++) {
        ivp_array_push(&arr, &dummy_vals[i]);
    }

    IVP_ASSERT_EQ(ivp_array_find(&arr, &dummy_vals[3]), 3);
    IVP_ASSERT_EQ(ivp_array_find(&arr, &dummy_vals[0]), 0);
    IVP_ASSERT_EQ(ivp_array_find(&arr, &dummy_vals[10]), -1); /* not found */

    ivp_array_destroy(&arr);
}

IVP_TEST(array_clear) {
    ivp_array_t arr;
    ivp_array_init(&arr, NULL, 8);

    for (int i = 0; i < 5; i++) {
        ivp_array_push(&arr, &dummy_vals[i]);
    }
    IVP_ASSERT_EQ(ivp_array_len(&arr), 5);

    ivp_array_clear(&arr);
    IVP_ASSERT_EQ(ivp_array_len(&arr), 0);

    /* Should still be able to push after clear */
    ivp_array_push(&arr, &dummy_vals[0]);
    IVP_ASSERT_EQ(ivp_array_len(&arr), 1);

    ivp_array_destroy(&arr);
}

IVP_TEST(array_grow) {
    ivp_array_t arr;
    ivp_array_init(&arr, NULL, 2);

    /* Push more elements than initial capacity to trigger growth */
    for (int i = 0; i < 50; i++) {
        ivp_result_t r = ivp_array_push(&arr, &dummy_vals[i % 64]);
        IVP_ASSERT_TRUE(ivp_ok(r));
    }
    IVP_ASSERT_EQ(ivp_array_len(&arr), 50);

    ivp_array_destroy(&arr);
}

IVP_TEST(elem_typeinfo_registry) {
    static const registry_case_t k_cases[] = {
        {
            IVP_ELEM_TYPE_ID_PTR,
            "void*",
            sizeof(void *),
            sizeof(void *),
            IVP_ELEM_FLAG_POINTER | IVP_ELEM_FLAG_TRIVIAL_MOVE |
                IVP_ELEM_FLAG_TRIVIAL_COPY | IVP_ELEM_FLAG_TRIVIAL_DROP,
            &IVP_ELEM_TYPEINFO_PTR
        },
        {
            IVP_ELEM_TYPE_ID_U32,
            "u32",
            sizeof(uint32_t),
            sizeof(uint32_t),
            IVP_ELEM_FLAG_TRIVIAL_MOVE | IVP_ELEM_FLAG_TRIVIAL_COPY | IVP_ELEM_FLAG_TRIVIAL_DROP,
            &IVP_ELEM_TYPEINFO_U32
        },
        {
            IVP_ELEM_TYPE_ID_F32,
            "f32",
            sizeof(float),
            sizeof(float),
            IVP_ELEM_FLAG_TRIVIAL_MOVE | IVP_ELEM_FLAG_TRIVIAL_COPY | IVP_ELEM_FLAG_TRIVIAL_DROP,
            &IVP_ELEM_TYPEINFO_F32
        },
        {
            IVP_ELEM_TYPE_ID_VEC3F,
            "vec3f",
            sizeof(ivp_vec3f_t),
            _Alignof(ivp_vec3f_t),
            IVP_ELEM_FLAG_TRIVIAL_MOVE | IVP_ELEM_FLAG_TRIVIAL_COPY | IVP_ELEM_FLAG_TRIVIAL_DROP,
            &IVP_ELEM_TYPEINFO_VEC3F
        }
    };

    for (size_t i = 0; i < sizeof(k_cases) / sizeof(k_cases[0]); i++) {
        ASSERT_REGISTRY_CASE(k_cases[i]);
    }

    IVP_ASSERT_EQ(ivp_elem_typeinfo_registry_count(), (uint32_t)(sizeof(k_cases) / sizeof(k_cases[0])));

    IVP_ASSERT_NULL(ivp_elem_typeinfo_from_id((ivp_elem_type_id_t)999));
    IVP_ASSERT_NULL(ivp_elem_typeinfo_from_name("unknown-type"));
}

IVP_TEST(elem_typeinfo_registry_runtime_register) {
    const uint32_t base_count = ivp_elem_typeinfo_registry_count();

    static const ivp_elem_typeinfo_t custom_vec3u = {
        1001u,
        "user.vec3u",
        sizeof(uint32_t) * 3u,
        _Alignof(uint32_t),
        IVP_ELEM_FLAG_TRIVIAL_MOVE | IVP_ELEM_FLAG_TRIVIAL_COPY | IVP_ELEM_FLAG_TRIVIAL_DROP,
        NULL
    };

    IVP_ASSERT_TRUE(ivp_elem_typeinfo_register(&custom_vec3u));
    IVP_ASSERT_EQ(ivp_elem_typeinfo_registry_count(), base_count + 1u);

    const ivp_elem_typeinfo_t *by_id = ivp_elem_typeinfo_from_id((ivp_elem_type_id_t)1001u);
    const ivp_elem_typeinfo_t *by_name = ivp_elem_typeinfo_from_name("user.vec3u");
    IVP_ASSERT_TRUE(by_id == &custom_vec3u);
    IVP_ASSERT_TRUE(by_name == &custom_vec3u);
    IVP_ASSERT_TRUE(ivp_elem_typeinfo_is_registered(&custom_vec3u));

    bool found = false;
    for (uint32_t i = 0; i < ivp_elem_typeinfo_registry_count(); i++) {
        const ivp_elem_typeinfo_t *at = ivp_elem_typeinfo_registry_at(i);
        if (at == &custom_vec3u) {
            found = true;
            break;
        }
    }
    IVP_ASSERT_TRUE(found);

    IVP_ASSERT_TRUE(ivp_elem_typeinfo_register(&custom_vec3u));
    IVP_ASSERT_EQ(ivp_elem_typeinfo_registry_count(), base_count + 1u);

    static const ivp_elem_typeinfo_t conflict_id = {
        1001u,
        "user.other",
        sizeof(uint32_t),
        _Alignof(uint32_t),
        IVP_ELEM_FLAG_TRIVIAL_MOVE | IVP_ELEM_FLAG_TRIVIAL_COPY | IVP_ELEM_FLAG_TRIVIAL_DROP,
        NULL
    };
    static const ivp_elem_typeinfo_t conflict_name = {
        1002u,
        "user.vec3u",
        sizeof(uint32_t),
        _Alignof(uint32_t),
        IVP_ELEM_FLAG_TRIVIAL_MOVE | IVP_ELEM_FLAG_TRIVIAL_COPY | IVP_ELEM_FLAG_TRIVIAL_DROP,
        NULL
    };

    IVP_ASSERT_FALSE(ivp_elem_typeinfo_register(&conflict_id));
    IVP_ASSERT_FALSE(ivp_elem_typeinfo_register(&conflict_name));

    IVP_ASSERT_FALSE(ivp_elem_typeinfo_register(NULL));
    IVP_ASSERT_NULL(ivp_elem_typeinfo_registry_at(ivp_elem_typeinfo_registry_count()));
}

IVP_TEST(array_lifecycle_callbacks) {
    s_lifecycle_init_count = 0;
    s_lifecycle_reset_count = 0;
    s_lifecycle_copy_count = 0;
    s_lifecycle_move_count = 0;
    s_lifecycle_dispose_count = 0;

    ivp_elem_lifecycle_t lc;
    lc.init = lifecycle_on_init;
    lc.reset = lifecycle_on_reset;
    lc.copy = lifecycle_on_copy;
    lc.move = lifecycle_on_move;
    lc.dispose = lifecycle_on_dispose;

    ivp_elem_typeinfo_t ti;
    ti.type_id = IVP_ELEM_TYPE_ID_INVALID;
    ti.name = "test-int*";
    ti.size = sizeof(int *);
    ti.align = sizeof(void *);
    ti.flags = IVP_ELEM_FLAG_POINTER;
    ti.lifecycle = &lc;

    ivp_array_t arr;
    ivp_result_t r = ivp_array_init_ex(&arr, NULL, 2, &ti, NULL);
    IVP_ASSERT_TRUE(ivp_ok(r));

    /* init=3 */
    ivp_array_push(&arr, &dummy_vals[0]);
    ivp_array_push(&arr, &dummy_vals[1]);
    ivp_array_insert(&arr, 1, &dummy_vals[2]);
    IVP_ASSERT_EQ(s_lifecycle_init_count, 3);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 0);
    IVP_ASSERT_EQ(s_lifecycle_copy_count, 0);
    IVP_ASSERT_EQ(s_lifecycle_move_count, 1);

    /* set: dispose old + init new */
    ivp_array_set(&arr, 0, &dummy_vals[3]);
    IVP_ASSERT_EQ(s_lifecycle_init_count, 4);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 1);

    /* pop/remove_ordered/remove_swap transfer ownership to caller (no dispose) */
    ivp_array_pop(&arr);
    ivp_array_remove_ordered(&arr, 0);
    ivp_array_remove_swap(&arr, 0);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 1);
    IVP_ASSERT_EQ(s_lifecycle_copy_count, 0);
    IVP_ASSERT_EQ(s_lifecycle_move_count, 2);

    /* re-add and clear should call reset+dispose on each live element */
    ivp_array_push(&arr, &dummy_vals[4]);
    ivp_array_push(&arr, &dummy_vals[5]);
    IVP_ASSERT_EQ(s_lifecycle_init_count, 6);
    ivp_array_clear(&arr);
    IVP_ASSERT_EQ(s_lifecycle_reset_count, 0);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 3);

    /* destroy with empty array should not change counts */
    ivp_array_destroy(&arr);
    IVP_ASSERT_EQ(s_lifecycle_init_count, 6);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 3);
}

IVP_TEST(array_lifecycle_destroy_nonempty) {
    s_lifecycle_init_count = 0;
    s_lifecycle_reset_count = 0;
    s_lifecycle_copy_count = 0;
    s_lifecycle_move_count = 0;
    s_lifecycle_dispose_count = 0;

    ivp_elem_lifecycle_t lc;
    lc.init = lifecycle_on_init;
    lc.reset = lifecycle_on_reset;
    lc.copy = lifecycle_on_copy;
    lc.move = lifecycle_on_move;
    lc.dispose = lifecycle_on_dispose;

    ivp_elem_typeinfo_t ti;
    ti.type_id = IVP_ELEM_TYPE_ID_INVALID;
    ti.name = "test-int*";
    ti.size = sizeof(int *);
    ti.align = sizeof(void *);
    ti.flags = IVP_ELEM_FLAG_POINTER;
    ti.lifecycle = &lc;

    ivp_array_t arr;
    ivp_result_t r = ivp_array_init_ex(&arr, NULL, 2, &ti, NULL);
    IVP_ASSERT_TRUE(ivp_ok(r));

    ivp_array_push(&arr, &dummy_vals[0]);
    ivp_array_push(&arr, &dummy_vals[1]);
    ivp_array_push(&arr, &dummy_vals[2]);
    IVP_ASSERT_EQ(s_lifecycle_init_count, 3);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 0);

    ivp_array_destroy(&arr);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 3);
}

IVP_TEST(array_reserve) {
    ivp_array_t arr;
    ivp_result_t r = ivp_array_init(&arr, NULL, 0);
    IVP_ASSERT_TRUE(ivp_ok(r));
    r = ivp_array_reserve(&arr, 64);
    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_TRUE(ivp_array_cap(&arr) >= 64);
    ivp_array_destroy(&arr);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FArray tests
 * ═══════════════════════════════════════════════════════════════════════════ */

static int s_index_cb_count;
static int s_last_cb_index;

static void test_index_cb(void *elem, int new_index, void *userdata) {
    (void)elem;
    (void)userdata;
    s_index_cb_count++;
    s_last_cb_index = new_index;
}

IVP_TEST(farray_basic) {
    ivp_array_t arr;
    ivp_result_t r = ivp_array_init_indexed(&arr, NULL, 4, NULL, NULL);
    IVP_ASSERT_TRUE(ivp_ok(r));

    for (int i = 0; i < 5; i++) {
        ivp_array_push(&arr, &dummy_vals[i]);
    }
    IVP_ASSERT_EQ(ivp_array_len(&arr), 5);

    void *removed = ivp_array_remove_swap(&arr, 1);
    IVP_ASSERT_EQ(removed, &dummy_vals[1]);
    IVP_ASSERT_EQ(ivp_array_len(&arr), 4);

    ivp_array_destroy(&arr);
}

IVP_TEST(farray_index_callback) {
    s_index_cb_count = 0;
    s_last_cb_index  = -1;

    ivp_array_t arr;
    ivp_array_init_indexed(&arr, NULL, 8, test_index_cb, NULL);

    /* Push 5 elements: callback should fire for each push with its index */
    for (int i = 0; i < 5; i++) {
        ivp_array_push(&arr, &dummy_vals[i]);
    }

    /* Remove index 0: last element (index 4) should be swapped to 0 */
    s_index_cb_count = 0;
    ivp_array_remove_swap(&arr, 0);

    /* The callback should have been called for the swapped element */
    IVP_ASSERT_TRUE(s_index_cb_count > 0);
    IVP_ASSERT_EQ(s_last_cb_index, 0);  /* swapped to position 0 */

    ivp_array_destroy(&arr);
}

IVP_TEST(farray_find) {
    ivp_array_t arr;
    ivp_array_init_indexed(&arr, NULL, 8, NULL, NULL);

    for (int i = 0; i < 5; i++) {
        ivp_array_push(&arr, &dummy_vals[i]);
    }

    IVP_ASSERT_EQ(ivp_array_find(&arr, &dummy_vals[3]), 3);
    IVP_ASSERT_EQ(ivp_array_find(&arr, &dummy_vals[42]), -1);

    ivp_array_destroy(&arr);
}

IVP_TEST(farray_lifecycle_callbacks) {
    s_lifecycle_init_count = 0;
    s_lifecycle_reset_count = 0;
    s_lifecycle_copy_count = 0;
    s_lifecycle_move_count = 0;
    s_lifecycle_dispose_count = 0;

    ivp_elem_lifecycle_t lc;
    lc.init = lifecycle_on_init;
    lc.reset = lifecycle_on_reset;
    lc.copy = lifecycle_on_copy;
    lc.move = lifecycle_on_move;
    lc.dispose = lifecycle_on_dispose;

    ivp_elem_typeinfo_t ti;
    ti.type_id = IVP_ELEM_TYPE_ID_INVALID;
    ti.name = "test-int*";
    ti.size = sizeof(int *);
    ti.align = sizeof(void *);
    ti.flags = IVP_ELEM_FLAG_POINTER;
    ti.lifecycle = &lc;

    ivp_array_t arr;
    ivp_result_t r = ivp_array_init_indexed_ex(&arr, NULL, 4,
                                                NULL,
                                                NULL,
                                                &ti,
                                                NULL);
    IVP_ASSERT_TRUE(ivp_ok(r));

    ivp_array_push(&arr, &dummy_vals[0]);
    ivp_array_push(&arr, &dummy_vals[1]);
    ivp_array_push(&arr, &dummy_vals[2]);
    IVP_ASSERT_EQ(s_lifecycle_init_count, 3);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 0);

    ivp_array_remove_swap(&arr, 1);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 0);
    IVP_ASSERT_EQ(s_lifecycle_move_count, 1);

    ivp_array_clear(&arr);
    IVP_ASSERT_EQ(s_lifecycle_reset_count, 0);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 2);

    ivp_array_push(&arr, &dummy_vals[3]);
    IVP_ASSERT_EQ(s_lifecycle_init_count, 4);
    ivp_array_destroy(&arr);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 3);
    IVP_ASSERT_EQ(s_lifecycle_copy_count, 0);
}

IVP_TEST(farray_reserve) {
    ivp_array_t arr;
    ivp_result_t r = ivp_array_init_indexed(&arr, NULL, 0, NULL, NULL);
    IVP_ASSERT_TRUE(ivp_ok(r));
    r = ivp_array_reserve(&arr, 64);
    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_TRUE(ivp_array_cap(&arr) >= 64);
    ivp_array_destroy(&arr);
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

IVP_TEST(hashmap_lifecycle_callbacks) {
    s_lifecycle_init_count = 0;
    s_lifecycle_reset_count = 0;
    s_lifecycle_copy_count = 0;
    s_lifecycle_move_count = 0;
    s_lifecycle_dispose_count = 0;

    ivp_elem_lifecycle_t lc;
    lc.init = lifecycle_on_init;
    lc.reset = lifecycle_on_reset;
    lc.copy = lifecycle_on_copy;
    lc.move = lifecycle_on_move;
    lc.dispose = lifecycle_on_dispose;

    ivp_elem_typeinfo_t ti;
    ti.type_id = IVP_ELEM_TYPE_ID_INVALID;
    ti.name = "test-int*";
    ti.size = sizeof(int *);
    ti.align = sizeof(void *);
    ti.flags = IVP_ELEM_FLAG_POINTER;
    ti.lifecycle = &lc;

    ivp_hashmap_t map;
    ivp_result_t r = ivp_hashmap_init_ex(&map, NULL, 4,
                                         ivp_hash_u32, ivp_eq_memcmp,
                                         sizeof(uint32_t), &ti, NULL);
    IVP_ASSERT_TRUE(ivp_ok(r));

    uint32_t keys[10];
    int values[10];
    for (int i = 0; i < 10; i++) {
        keys[i] = (uint32_t)(i + 1);
        values[i] = i * 10;
        ivp_hashmap_put(&map, &keys[i], &values[i]);
    }
    IVP_ASSERT_EQ(s_lifecycle_init_count, 10);

    int replacement = 999;
    ivp_hashmap_put(&map, &keys[0], &replacement);
    IVP_ASSERT_EQ(s_lifecycle_init_count, 11);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 1);

    ivp_hashmap_remove(&map, &keys[1]);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 1);
    IVP_ASSERT_TRUE(s_lifecycle_move_count > 0 || s_lifecycle_copy_count > 0);

    ivp_hashmap_clear(&map);
    IVP_ASSERT_TRUE(s_lifecycle_reset_count > 0);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 10);

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

    ivp_result_t r = ivp_hashmap_init_ex(&map, NULL, 16,
                                         NULL, ivp_eq_memcmp,
                                         sizeof(uint32_t), &IVP_ELEM_TYPEINFO_PTR, NULL);
    IVP_ASSERT_FALSE(ivp_ok(r));

    r = ivp_hashmap_init_ex(&map, NULL, 16,
                            ivp_hash_u32, NULL,
                            sizeof(uint32_t), &IVP_ELEM_TYPEINFO_PTR, NULL);
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

IVP_TEST(minheap_lifecycle_callbacks) {
    s_lifecycle_init_count = 0;
    s_lifecycle_reset_count = 0;
    s_lifecycle_copy_count = 0;
    s_lifecycle_move_count = 0;
    s_lifecycle_dispose_count = 0;

    ivp_elem_lifecycle_t lc;
    lc.init = lifecycle_on_init;
    lc.reset = lifecycle_on_reset;
    lc.copy = lifecycle_on_copy;
    lc.move = lifecycle_on_move;
    lc.dispose = lifecycle_on_dispose;

    ivp_elem_typeinfo_t ti;
    ti.type_id = IVP_ELEM_TYPE_ID_INVALID;
    ti.name = "test-int*";
    ti.size = sizeof(int *);
    ti.align = sizeof(void *);
    ti.flags = IVP_ELEM_FLAG_POINTER;
    ti.lifecycle = &lc;

    ivp_minheap_t heap;
    ivp_result_t r = ivp_minheap_init_ex(&heap, NULL, 2, NULL, &ti, NULL);
    IVP_ASSERT_TRUE(ivp_ok(r));

    ivp_minheap_push(&heap, 5.0, &dummy_vals[0]);
    ivp_minheap_push(&heap, 2.0, &dummy_vals[1]);
    ivp_minheap_push(&heap, 8.0, &dummy_vals[2]);
    ivp_minheap_push(&heap, 1.0, &dummy_vals[3]);
    IVP_ASSERT_EQ(s_lifecycle_init_count, 4);

    ivp_heap_entry_t out;
    ivp_minheap_pop(&heap, &out);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 0);
    IVP_ASSERT_TRUE(s_lifecycle_move_count > 0);

    ivp_minheap_remove(&heap, 0);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 1);

    ivp_minheap_clear(&heap);
    IVP_ASSERT_TRUE(s_lifecycle_reset_count > 0);
    IVP_ASSERT_EQ(s_lifecycle_dispose_count, 3);

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
    /* Array */
    IVP_REGISTER(array_push_pop);
    IVP_REGISTER(array_get_set);
    IVP_REGISTER(array_remove_ordered);
    IVP_REGISTER(array_remove_swap);
    IVP_REGISTER(array_insert);
    IVP_REGISTER(array_find);
    IVP_REGISTER(array_clear);
    IVP_REGISTER(array_grow);
    IVP_REGISTER(elem_typeinfo_registry);
    IVP_REGISTER(elem_typeinfo_registry_runtime_register);
    IVP_REGISTER(array_reserve);
    IVP_REGISTER(array_lifecycle_callbacks);
    IVP_REGISTER(array_lifecycle_destroy_nonempty);

    /* FArray */
    IVP_REGISTER(farray_basic);
    IVP_REGISTER(farray_index_callback);
    IVP_REGISTER(farray_find);
    IVP_REGISTER(farray_reserve);
    IVP_REGISTER(farray_lifecycle_callbacks);

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
    IVP_REGISTER(hashmap_lifecycle_callbacks);

    /* MinHeap */
    IVP_REGISTER(minheap_push_pop);
    IVP_REGISTER(minheap_peek);
    IVP_REGISTER(minheap_update_key);
    IVP_REGISTER(minheap_remove);
    IVP_REGISTER(minheap_empty_pop);
    IVP_REGISTER(minheap_index_callback);
    IVP_REGISTER(minheap_many_elements);
    IVP_REGISTER(minheap_reserve);
    IVP_REGISTER(minheap_lifecycle_callbacks);

    IVP_RUN_ALL();
}
