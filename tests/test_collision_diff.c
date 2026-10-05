/* Regression tests for the actual OV collision partition and backlinks. */
#include "ivp_test.h"
#include "collision/ivp_collision_diff.h"
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/diagnostics/ivp_statistic_manager.h>
#include "dynamics/ivp_env_internal.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef struct test_collision {
    ivp_collision_t base;
    struct ivp_object *objects[2];
    int id;
} test_collision_t;

static int callbacks;
static void get_objects(ivp_collision_t *base, struct ivp_object *out[2])
{
    test_collision_t *c = (test_collision_t *)base;
    callbacks++;
    out[0] = c->objects[0];
    out[1] = c->objects[1];
}
static const ivp_collision_vt_t pure_vt = {get_objects, NULL, NULL, NULL};
static const ivp_collision_vt_t custom_vt = {get_objects, NULL, NULL, NULL};
static struct ivp_object *key(const ivp_collision_t *base, const struct ivp_object *owner)
{
    const test_collision_t *c = (const test_collision_t *)base;
    if (base->cvt != &pure_vt) return NULL;
    if (c->objects[0] == owner) return c->objects[1];
    if (c->objects[1] == owner) return c->objects[0];
    return NULL;
}

static void fill(ivp_coll_fvector_t *v, test_collision_t *c, struct ivp_object *objects, int n)
{
    ivp_coll_fvector_init(v);
    for (int i = 0; i < n; ++i) {
        c[i] = (test_collision_t){0};
        c[i].base.cvt = &pure_vt;
        c[i].base.fvector_index[0] = c[i].base.fvector_index[1] = -1;
        c[i].id = i;
        c[i].objects[i & 1] = &objects[0];
        c[i].objects[!(i & 1)] = &objects[i + 1];
        ivp_coll_fvector_add(v, &c[i].base);
        /* A different second endpoint index must remain unchanged on swaps. */
        c[i].base.fvector_index[1] = n + i;
    }
}

static bool linear(ivp_coll_fvector_t *v, int *found, struct ivp_object *obj)
{
    for (int i = *found; i < v->n; ++i) {
        struct ivp_object *o[2];
        v->elems[i]->cvt->get_objects(v->elems[i], o);
        if (o[0] == obj || o[1] == obj) {
            if (i > *found) ivp_coll_fvector_swap(v, *found, i);
            ++*found;
            return true;
        }
    }
    return false;
}

static uint32_t random_next(uint32_t *seed)
{
    *seed = *seed * UINT32_C(1664525) + UINT32_C(1013904223);
    return *seed;
}

IVP_TEST(indexed_partition_matches_linear_and_backlinks) {
    ivp_collision_diff_storage_t *storage = NULL;
    ivp_allocator_t counting = ivp_counting_allocator_create(NULL);
    struct ivp_object *objects = calloc(390, sizeof(*objects));
    test_collision_t *actual = calloc(384, sizeof(*actual));
    test_collision_t *expected = calloc(384, sizeof(*expected));
    IVP_ASSERT_NOT_NULL(objects); IVP_ASSERT_NOT_NULL(actual); IVP_ASSERT_NOT_NULL(expected);
    uint32_t seed = 20261004;
    for (int round = 0; round < 80; ++round) {
        int n = round == 0 ? 384 : (int)(random_next(&seed) % 385);
        ivp_coll_fvector_t a, b;
        fill(&a, actual, objects, n); fill(&b, expected, objects, n);
        ivp_collision_diff_t diff;
        ivp_collision_diff_begin(&diff, &a, &objects[0], n + 4, &storage, &counting, key);
        if (n >= 64) IVP_ASSERT_NOT_NULL(diff.index);
        int found = 0;
        for (int query = 0; query < n * 2 + 10; ++query) {
            /* Reverse enumeration, repeated queries, misses, and a query for
             * the owner (matches every entry, so must use the linear path). */
            unsigned q = query < n ? (unsigned)(n - query) : random_next(&seed) % (unsigned)(n + 4);
            if (query == 0 && (round & 1)) q = 0;
            IVP_ASSERT_EQ(ivp_collision_diff_check(&diff, &objects[q]), linear(&b, &found, &objects[q]));
            IVP_ASSERT_EQ(diff.found, found);
            for (int i = 0; i < n; ++i) {
                test_collision_t *ac = (test_collision_t *)a.elems[i];
                test_collision_t *bc = (test_collision_t *)b.elems[i];
                IVP_ASSERT_EQ(ac->id, bc->id);
                IVP_ASSERT_EQ(ac->base.fvector_index[0], i);
                IVP_ASSERT_EQ(ac->base.fvector_index[1], n + ac->id);
            }
        }
        ivp_coll_fvector_clear(&a); ivp_coll_fvector_clear(&b);
    }
    ivp_collision_diff_storage_free(storage, &counting);
    ivp_alloc_stats_t stats; ivp_counting_allocator_get_stats(&counting, &stats);
    IVP_ASSERT_EQ(stats.current_bytes, 0);
    ivp_counting_allocator_destroy(&counting);
    free(expected); free(actual); free(objects);
}

IVP_TEST(duplicates_and_user_callbacks_keep_linear_semantics) {
    struct ivp_object objects[100] = {0}; test_collision_t c[96];
    ivp_coll_fvector_t v; fill(&v, c, objects, 96);
    ivp_collision_diff_storage_t *storage = NULL; ivp_collision_diff_t diff;
    c[95].objects[0] = c[94].objects[1];  /* both have key objects[95] */
    ivp_collision_diff_begin(&diff, &v, &objects[0], 96, &storage, ivp_default_allocator(), key);
    IVP_ASSERT_NULL(diff.index);
    IVP_ASSERT_TRUE(ivp_collision_diff_check(&diff, &objects[95]));
    IVP_ASSERT_TRUE(v.elems[0] == &c[94].base);
    IVP_ASSERT_TRUE(ivp_collision_diff_check(&diff, &objects[95]));
    IVP_ASSERT_TRUE(v.elems[1] == &c[95].base);
    ivp_coll_fvector_clear(&v); fill(&v, c, objects, 96);
    c[10].base.cvt = &custom_vt; callbacks = 0;
    ivp_collision_diff_begin(&diff, &v, &objects[0], 96, &storage, ivp_default_allocator(), key);
    IVP_ASSERT_NULL(diff.index); IVP_ASSERT_EQ(callbacks, 0);
    IVP_ASSERT_TRUE(ivp_collision_diff_check(&diff, &objects[96]));
    IVP_ASSERT_EQ(callbacks, 96);
    ivp_coll_fvector_clear(&v);
    ivp_collision_diff_storage_free(storage, ivp_default_allocator());
}

typedef struct failing_allocator { ivp_allocator_t base; bool fail; size_t live; int allocations; } failing_allocator_t;
static void *allocate(size_t bytes, void *context) {
    failing_allocator_t *a = context;
    if (a->fail) return NULL;
    void *p = malloc(bytes); if (p) {a->live += bytes; a->allocations++;} return p;
}
static void release(void *p, size_t bytes, void *context) {
    failing_allocator_t *a = context; if (p) {a->live -= bytes; free(p);}
}

IVP_TEST(scratch_reuse_recursion_and_oom_fallback) {
    struct ivp_object *objects = calloc(520, sizeof(*objects));
    test_collision_t *c = calloc(512, sizeof(*c));
    IVP_ASSERT_NOT_NULL(objects); IVP_ASSERT_NOT_NULL(c);
    ivp_coll_fvector_t v; fill(&v, c, objects, 128);
    failing_allocator_t a = {{allocate, NULL, release, NULL}, false, 0, 0}; a.base.ctx = &a;
    ivp_collision_diff_storage_t *outer = NULL, *inner = NULL;
    ivp_collision_diff_t d, recursive;
    a.fail = true;
    ivp_collision_diff_begin(&d, &v, objects, 128, &outer, &a.base, key);
    IVP_ASSERT_NULL(d.index); IVP_ASSERT_NULL(outer);
    IVP_ASSERT_TRUE(ivp_collision_diff_check(&d, &objects[128]));
    a.fail = false;
    ivp_collision_diff_begin(&d, &v, objects, 128, &outer, &a.base, key);
    IVP_ASSERT_NOT_NULL(d.index);
    int allocations = a.allocations;
    ivp_collision_diff_begin(&d, &v, objects, 128, &outer, &a.base, key);
    IVP_ASSERT_EQ(a.allocations, allocations);
    test_collision_t other[128]; ivp_coll_fvector_t nested;
    fill(&nested, other, objects, 128);
    ivp_collision_diff_begin(&recursive, &nested, objects, 128, &inner, &a.base, key);
    IVP_ASSERT_TRUE(outer != inner);
    IVP_ASSERT_TRUE(ivp_collision_diff_check(&recursive, &objects[127]));
    /* An inner recheck must not corrupt the suspended outer partition. */
    IVP_ASSERT_TRUE(ivp_collision_diff_check(&d, &objects[128]));
    IVP_ASSERT_TRUE(v.elems[0] == &c[127].base);
    ivp_coll_fvector_clear(&nested);
    ivp_coll_fvector_clear(&v); fill(&v, c, objects, 512);
    a.fail = true;
    ivp_collision_diff_storage_t *saved = outer;
    ivp_collision_diff_begin(&d, &v, objects, 512, &outer, &a.base, key);
    IVP_ASSERT_NULL(d.index); IVP_ASSERT_TRUE(outer == saved);
    IVP_ASSERT_TRUE(ivp_collision_diff_check(&d, &objects[512]));
    a.fail = false;
    ivp_collision_diff_begin(&d, &v, objects, 512, &outer, &a.base, key);
    IVP_ASSERT_NOT_NULL(d.index);
    ivp_coll_fvector_remove_allow_resort(&v, &c[1].base);
    IVP_ASSERT_TRUE(ivp_collision_diff_check(&d, &objects[511]));
    IVP_ASSERT_NULL(d.index);
    ivp_collision_diff_storage_free(outer, &a.base); ivp_collision_diff_storage_free(inner, &a.base);
    IVP_ASSERT_EQ(a.live, 0);
    ivp_coll_fvector_clear(&v); free(c); free(objects);
}

IVP_TEST(indices_above_65535_and_small_world_no_allocation) {
    const int n = 70000;
    struct ivp_object *objects = calloc((size_t)n + 2, sizeof(*objects));
    test_collision_t *c = calloc((size_t)n, sizeof(*c));
    IVP_ASSERT_NOT_NULL(objects); IVP_ASSERT_NOT_NULL(c);
    ivp_coll_fvector_t v; fill(&v, c, objects, 16);
    ivp_collision_diff_storage_t *storage = NULL; ivp_collision_diff_t d;
    ivp_collision_diff_begin(&d, &v, objects, 16, &storage, ivp_default_allocator(), key);
    IVP_ASSERT_NULL(storage); IVP_ASSERT_NULL(d.index);
    ivp_coll_fvector_clear(&v); fill(&v, c, objects, n);
    ivp_collision_diff_begin(&d, &v, objects, n, &storage, ivp_default_allocator(), key);
    IVP_ASSERT_NOT_NULL(d.index);
    IVP_ASSERT_TRUE(ivp_collision_diff_check(&d, &objects[n]));
    IVP_ASSERT_TRUE(v.elems[0] == &c[n - 1].base);
    IVP_ASSERT_EQ(c[0].base.fvector_index[0], n - 1);
    IVP_ASSERT_FALSE(ivp_collision_diff_check(&d, &objects[n]));
    IVP_ASSERT_TRUE(ivp_collision_diff_check(&d, &objects[n - 1]));
    IVP_ASSERT_TRUE(v.elems[1] == &c[n - 2].base);
    ivp_collision_diff_storage_free(storage, ivp_default_allocator());
    ivp_coll_fvector_clear(&v); free(c); free(objects);
}

static bool allow_with_callback(const struct ivp_object *a, const struct ivp_object *b, void *ctx)
{
    (void)a; (void)b;
    ++*(int *)ctx;
    return true;
}

IVP_TEST(empty_manager_does_not_require_allocator_owner) {
    ivp_mindist_mgr_t manager;
    ivp_mindist_mgr_init(&manager, NULL);
    ivp_mindist_mgr_destroy(&manager);
    IVP_ASSERT_NULL(manager.recheck_diff_storage);
}

IVP_TEST(dense_world_matches_forced_linear_each_psi) {
    enum { N = 256 };
    ivp_allocator_t counting = ivp_counting_allocator_create(NULL);
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.allocator = &counting;
    cfg.disable_performance_counter = true;
    cfg.delta_psi_time = IVP_REAL(1.0) / IVP_REAL(180.0);
    ivp_vec3r_set_zero(&cfg.gravity);
    ivp_environment_t *indexed = ivp_environment_create(&cfg);
    int calls = 0;
    cfg.collision_filter = ivp_collision_filter_callback(allow_with_callback, &calls);
    ivp_environment_t *legacy = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(indexed); IVP_ASSERT_NOT_NULL(legacy);
    ivp_object_t *objects[2][N];
    for (int world = 0; world < 2; ++world) {
        for (int i = 0; i < N; ++i) {
            ivp_object_template_t t; ivp_object_template_init(&t);
            t.type = IVP_OBJECT_BALL; t.ball_radius = IVP_REAL(0.4); t.mass = IVP_REAL(1.0);
            ivp_vec3r_set(&t.position, (i & 1) ? IVP_REAL(4.0) : IVP_REAL(-4.0),
                          (ivp_real_t)((i / 2 % 16) * 2), (ivp_real_t)((i / 32) * 2));
            ivp_object_t *obj = ivp_environment_create_object(world ? legacy : indexed, &t, NULL);
            IVP_ASSERT_NOT_NULL(obj);
            obj->material.elasticity = IVP_REAL(0.9);
            ivp_vec3r_set(&obj->physical_core->speed, (i & 1) ? IVP_REAL(-80.0) : IVP_REAL(80.0),
                          IVP_REAL(0.0), IVP_REAL(0.0));
            obj->physical_core->delta_world_f_core_psis = obj->physical_core->speed;
            objects[world][i] = obj;
        }
    }
    for (int step = 0; step < 180; ++step) {
        ivp_environment_simulate_one_psi(indexed);
        ivp_environment_simulate_one_psi(legacy);
        for (int i = 0; i < N; ++i) {
            if (!objects[0][i]) continue;
            const ivp_core_t *a = objects[0][i]->physical_core, *b = objects[1][i]->physical_core;
            IVP_ASSERT_EQ(memcmp(a->pos_world_f_core_last_psi.k, b->pos_world_f_core_last_psi.k,
                                 sizeof(a->pos_world_f_core_last_psi.k)), 0);
            IVP_ASSERT_EQ(memcmp(&a->q_world_f_core_last_psi, &b->q_world_f_core_last_psi,
                                 sizeof(a->q_world_f_core_last_psi)), 0);
            IVP_ASSERT_EQ(memcmp(a->speed.k, b->speed.k, sizeof(a->speed.k)), 0);
            IVP_ASSERT_EQ(memcmp(a->rot_speed.k, b->rot_speed.k, sizeof(a->rot_speed.k)), 0);
            IVP_ASSERT_EQ(objects[0][i]->object_movement_state, objects[1][i]->object_movement_state);
            IVP_ASSERT_EQ(objects[0][i]->ov_element->collision_fvector.n,
                           objects[1][i]->ov_element->collision_fvector.n);
        }
        const ivp_statistic_manager_t *a = ivp_environment_get_statistic_manager(indexed);
        const ivp_statistic_manager_t *b = ivp_environment_get_statistic_manager(legacy);
        IVP_ASSERT_EQ(a->mindists_generated, b->mindists_generated);
        IVP_ASSERT_EQ(a->mindists_deleted, b->mindists_deleted);
        IVP_ASSERT_EQ(a->impact_counter, b->impact_counter);
        if (step == 60) {
            ivp_environment_destroy_object(indexed, objects[0][0]);
            ivp_environment_destroy_object(legacy, objects[1][0]);
            objects[0][0] = objects[1][0] = NULL;
        }
    }
    IVP_ASSERT_NOT_NULL(ivp_env_get_mindist_mgr(indexed)->recheck_diff_storage);
    IVP_ASSERT_NULL(ivp_env_get_mindist_mgr(legacy)->recheck_diff_storage);
    IVP_ASSERT_TRUE(calls > 0);
    ivp_environment_destroy(indexed); ivp_environment_destroy(legacy);
    ivp_alloc_stats_t stats; ivp_counting_allocator_get_stats(&counting, &stats);
    IVP_ASSERT_EQ(stats.current_bytes, 0);
    ivp_counting_allocator_destroy(&counting);
}

int main(void) {
    IVP_REGISTER(indexed_partition_matches_linear_and_backlinks);
    IVP_REGISTER(duplicates_and_user_callbacks_keep_linear_semantics);
    IVP_REGISTER(scratch_reuse_recursion_and_oom_fallback);
    IVP_REGISTER(indices_above_65535_and_small_world_no_allocation);
    IVP_REGISTER(dense_world_matches_forced_linear_each_psi);
    IVP_REGISTER(empty_manager_does_not_require_allocator_owner);
    IVP_RUN_ALL();
}
