/* test_base.c — Unit tests for IVP C17 base module */
#include "ivp_test.h"
#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_arena.h>
#include <ivp/base/ivp_message.h>
#include <ivp/base/ivp_u_memory.h>
#include <ivp/base/ivp_time.h>
#include <stdint.h>
#include <limits.h>

typedef struct message_capture {
    char text[10000];
    size_t length;
    int calls;
} message_capture_t;

static void capture_message(const char *text, void *userdata) {
    message_capture_t *capture = (message_capture_t *)userdata;
    capture->length = strlen(text);
    size_t copy_len = capture->length;
    if (copy_len >= sizeof(capture->text)) {
        copy_len = sizeof(capture->text) - 1;
    }
    memcpy(capture->text, text, copy_len);
    capture->text[copy_len] = '\0';
    capture->calls++;
}

/* ── Result tests ───────────────────────────────────────────────────────── */

IVP_TEST(result_ok) {
    IVP_ASSERT_TRUE(ivp_ok(IVP_OK));
    IVP_ASSERT_FALSE(ivp_ok(IVP_ERR_INVALID_ARG));
    IVP_ASSERT_FALSE(ivp_ok(IVP_ERR_OUT_OF_MEMORY));
}

IVP_TEST(result_str) {
    IVP_ASSERT_TRUE(strcmp(ivp_result_str(IVP_OK), "OK") == 0);
    IVP_ASSERT_TRUE(strcmp(ivp_result_str(IVP_ERR_INVALID_ARG), "INVALID_ARG") == 0);
    IVP_ASSERT_TRUE(strcmp(ivp_result_str(IVP_ERR_SINGULAR_MATRIX), "SINGULAR_MATRIX") == 0);
}

IVP_TEST(message_prefix_preserves_bounded_output) {
    char input[10000];
    memset(input, 'x', sizeof(input) - 1);
    input[sizeof(input) - 1] = '\0';
    message_capture_t capture = {0};

    ivp_set_message_output(capture_message, &capture);
    ivp_message("%s", input);
    ivp_set_message_output(NULL, NULL);

    IVP_ASSERT_EQ(capture.calls, 1);
    IVP_ASSERT_EQ(capture.length, sizeof(capture.text) - 1);
    IVP_ASSERT_TRUE(memcmp(capture.text, "[havok] ", 8) == 0);
    IVP_ASSERT_EQ(capture.text[8], 'x');
    IVP_ASSERT_EQ(capture.text[sizeof(capture.text) - 2], 'x');
    IVP_ASSERT_EQ(capture.text[sizeof(capture.text) - 1], '\0');
}

/* ── Default allocator tests ────────────────────────────────────────────── */

IVP_TEST(default_alloc_basic) {
    const ivp_allocator_t *a = ivp_default_allocator();
    IVP_ASSERT_NOT_NULL(a);

    void *p = ivp_alloc(a, 128);
    IVP_ASSERT_NOT_NULL(p);

    /* Write and read back to verify memory is usable */
    memset(p, 0xAB, 128);
    IVP_ASSERT_EQ(((unsigned char*)p)[0], 0xAB);
    IVP_ASSERT_EQ(((unsigned char*)p)[127], 0xAB);

    ivp_free(a, p, 128);
}

IVP_TEST(default_alloc_realloc) {
    const ivp_allocator_t *a = ivp_default_allocator();
    void *p = ivp_alloc(a, 64);
    IVP_ASSERT_NOT_NULL(p);
    memset(p, 0xCD, 64);

    void *p2 = ivp_realloc(a, p, 64, 256);
    IVP_ASSERT_NOT_NULL(p2);
    /* First 64 bytes should be preserved */
    IVP_ASSERT_EQ(((unsigned char*)p2)[0], 0xCD);
    IVP_ASSERT_EQ(((unsigned char*)p2)[63], 0xCD);

    ivp_free(a, p2, 256);
}

/* ── Aligned allocation tests ───────────────────────────────────────────── */

IVP_TEST(aligned_alloc_16) {
    const ivp_allocator_t *a = ivp_default_allocator();
    void *p = ivp_alloc_aligned(a, 100, 16);
    IVP_ASSERT_NOT_NULL(p);
    IVP_ASSERT_EQ(((uintptr_t)p) % 16, (uintptr_t)0);
    memset(p, 0x55, 100);
    ivp_free_aligned(a, p);
}

IVP_TEST(aligned_alloc_64) {
    const ivp_allocator_t *a = ivp_default_allocator();
    void *p = ivp_alloc_aligned(a, 200, 64);
    IVP_ASSERT_NOT_NULL(p);
    IVP_ASSERT_EQ(((uintptr_t)p) % 64, (uintptr_t)0);
    memset(p, 0xAA, 200);
    ivp_free_aligned(a, p);
}

typedef struct reject_alloc_ctx {
    int calls;
} reject_alloc_ctx_t;

static void *reject_alloc(size_t size, void *ctx) {
    reject_alloc_ctx_t *state = (reject_alloc_ctx_t *)ctx;
    IVP_UNUSED(size);
    state->calls++;
    return (void *)(uintptr_t)1;
}

static void reject_free(void *ptr, size_t size, void *ctx) {
    IVP_UNUSED(ptr);
    IVP_UNUSED(size);
    IVP_UNUSED(ctx);
}

IVP_TEST(aligned_alloc_rejects_overflow) {
    reject_alloc_ctx_t state = {0};
    ivp_allocator_t a = {
        .alloc = reject_alloc,
        .realloc = NULL,
        .free = reject_free,
        .ctx = &state
    };

    IVP_ASSERT_NULL(ivp_alloc_aligned(&a, SIZE_MAX, 8));
    IVP_ASSERT_NULL(ivp_alloc_aligned(&a, 1, SIZE_MAX));
    IVP_ASSERT_EQ(state.calls, 0);
}

IVP_TEST(aligned_alloc_rejects_invalid_allocator) {
    ivp_allocator_t a = {0};
    IVP_ASSERT_NULL(ivp_alloc_aligned(NULL, 16, 16));
    IVP_ASSERT_NULL(ivp_alloc_aligned(&a, 16, 16));
    ivp_free_aligned(&a, (void *)(uintptr_t)1);
}

/* ── Counting allocator tests ───────────────────────────────────────────── */

IVP_TEST(counting_alloc_stats) {
    ivp_allocator_t ca = ivp_counting_allocator_create(NULL);
    ivp_alloc_stats_t stats;

    void *p1 = ivp_alloc(&ca, 100);
    IVP_ASSERT_NOT_NULL(p1);

    void *p2 = ivp_alloc(&ca, 200);
    IVP_ASSERT_NOT_NULL(p2);

    ivp_counting_allocator_get_stats(&ca, &stats);
    IVP_ASSERT_EQ(stats.alloc_count, 2);
    IVP_ASSERT_EQ(stats.free_count, 0);
    IVP_ASSERT_EQ(stats.bytes_allocated, 300);
    IVP_ASSERT_EQ(stats.current_bytes, 300);
    IVP_ASSERT_EQ(stats.peak_bytes, 300);

    ivp_free(&ca, p1, 100);
    ivp_counting_allocator_get_stats(&ca, &stats);
    IVP_ASSERT_EQ(stats.free_count, 1);
    IVP_ASSERT_EQ(stats.current_bytes, 200);
    IVP_ASSERT_EQ(stats.peak_bytes, 300);

    ivp_free(&ca, p2, 200);
    ivp_counting_allocator_get_stats(&ca, &stats);
    IVP_ASSERT_EQ(stats.current_bytes, 0);

    ivp_counting_allocator_destroy(&ca);
}

IVP_TEST(counting_alloc_destroy) {
    ivp_allocator_t ca = ivp_counting_allocator_create(NULL);

    void *p = ivp_alloc(&ca, 32);
    IVP_ASSERT_NOT_NULL(p);
    ivp_free(&ca, p, 32);

    ivp_counting_allocator_destroy(&ca);

    /* After destroy, allocator falls back to passthrough behavior. */
    ivp_alloc_stats_t stats;
    ivp_counting_allocator_get_stats(&ca, &stats);
    IVP_ASSERT_EQ(stats.alloc_count, 0);
    IVP_ASSERT_EQ(stats.free_count, 0);

    void *q = ivp_alloc(&ca, 16);
    IVP_ASSERT_NOT_NULL(q);
    ivp_free(&ca, q, 16);
}

/* ── Arena tests ────────────────────────────────────────────────────────── */

IVP_TEST(arena_basic) {
    const ivp_allocator_t *a = ivp_default_allocator();
    ivp_arena_t *arena = ivp_arena_create(a);
    IVP_ASSERT_NOT_NULL(arena);

    void *p1 = ivp_arena_alloc(arena, 64);
    IVP_ASSERT_NOT_NULL(p1);
    memset(p1, 0x11, 64);

    void *p2 = ivp_arena_alloc(arena, 128);
    IVP_ASSERT_NOT_NULL(p2);
    memset(p2, 0x22, 128);

    /* p1 and p2 should not overlap */
    IVP_ASSERT_NE(p1, p2);

    ivp_arena_destroy(arena);
}

IVP_TEST(arena_save_restore) {
    const ivp_allocator_t *a = ivp_default_allocator();
    ivp_arena_t *arena = ivp_arena_create(a);

    void *p1 = ivp_arena_alloc(arena, 32);
    IVP_ASSERT_NOT_NULL(p1);
    memset(p1, 0xAA, 32);

    ivp_arena_checkpoint_t cp = ivp_arena_save(arena);

    void *p2 = ivp_arena_alloc(arena, 64);
    IVP_ASSERT_NOT_NULL(p2);
    memset(p2, 0xBB, 64);

    /* Restore to checkpoint: p2 region is now available for reuse */
    ivp_arena_restore(arena, cp);

    /* Allocate again — should succeed (reusing space) */
    void *p3 = ivp_arena_alloc(arena, 64);
    IVP_ASSERT_NOT_NULL(p3);

    ivp_arena_destroy(arena);
}

IVP_TEST(arena_reset) {
    const ivp_allocator_t *a = ivp_default_allocator();
    ivp_arena_t *arena = ivp_arena_create(a);

    /* Make several allocations */
    for (int i = 0; i < 50; i++) {
        void *p = ivp_arena_alloc(arena, 256);
        IVP_ASSERT_NOT_NULL(p);
    }

    /* Reset should put us back to the beginning */
    ivp_arena_reset(arena);

    void *p = ivp_arena_alloc(arena, 32);
    IVP_ASSERT_NOT_NULL(p);

    ivp_arena_destroy(arena);
}

IVP_TEST(arena_stats_smoke) {
    const ivp_allocator_t *a = ivp_default_allocator();
    ivp_arena_t *arena = ivp_arena_create(a);
    IVP_ASSERT_NOT_NULL(arena);

    void *p1 = ivp_arena_alloc(arena, 64);
    void *p2 = ivp_arena_alloc(arena, 96);
    IVP_ASSERT_NOT_NULL(p1);
    IVP_ASSERT_NOT_NULL(p2);

    ivp_arena_stats_t stats;
    memset(&stats, 0, sizeof(stats));
    ivp_arena_get_stats(arena, &stats);

#ifdef IVP_ENABLE_STATS
    IVP_ASSERT_TRUE(stats.alloc_count >= 2u);
    IVP_ASSERT_TRUE(stats.total_allocated >= 160u);
    IVP_ASSERT_TRUE(stats.high_water_mark > 0u);
#else
    IVP_ASSERT_EQ(stats.alloc_count, 0u);
    IVP_ASSERT_EQ(stats.total_allocated, 0u);
    IVP_ASSERT_EQ(stats.high_water_mark, 0u);
#endif

    ivp_arena_destroy(arena);
}

/* ── IVP_U_Memory port (ivp_u_memory) ───────────────────────────────────── */

static ptrdiff_t u_off(const ivp_u_memory_t *m, const void *p) {
    return (const char *)p - m->block_start;
}

IVP_TEST(u_memory_layout) {
    ivp_u_memory_t m;
    IVP_ASSERT_EQ(ivp_u_memory_init(&m, NULL), IVP_OK);
    /* neuer_sp_block(0): the pointer starts 64 bytes into the first block */
    char *p0 = (char *)ivp_u_memory_get_mem(&m, 8);
    IVP_ASSERT_EQ(u_off(&m, p0), 64);
    IVP_ASSERT_EQ(((uintptr_t)p0 & 31u), 0u);
    /* next = (start + size + 0x4f) & ~0x1f */
    char *p1 = (char *)ivp_u_memory_get_mem(&m, 40);
    IVP_ASSERT_EQ(p1 - p0, 64);
    char *p2 = (char *)ivp_u_memory_get_mem(&m, 0);
    IVP_ASSERT_EQ(p2 - p1, 96);
    char *p3 = (char *)ivp_u_memory_get_mem(&m, 33);
    IVP_ASSERT_EQ(p3 - p2, 64);
    char *p4 = (char *)ivp_u_memory_get_mem(&m, 1);
    IVP_ASSERT_EQ(p4 - p3, 96);
    /* end of transaction: back to the block start, contents kept */
    memset(p0, 0x5a, 8);
    ivp_u_memory_start_transaction(&m);
    ivp_u_memory_end_transaction(&m);
    char *q0 = (char *)ivp_u_memory_get_mem(&m, 8);
    IVP_ASSERT_EQ(u_off(&m, q0), 0);
    IVP_ASSERT_EQ(p0 - q0, 64);
    IVP_ASSERT_EQ((int)(unsigned char)p0[7], 0x5a);
    IVP_ASSERT_EQ((int)(unsigned char)q0[0], 0); /* fresh first block is zero */
    ivp_u_memory_destroy(&m);
}

IVP_TEST(u_memory_overflow_blocks) {
    ivp_u_memory_t m;
    IVP_ASSERT_EQ(ivp_u_memory_init(&m, NULL), IVP_OK);
    ivp_u_memory_start_transaction(&m);
    ivp_u_memory_end_transaction(&m);
    /* after the first reset the block holds IVU_MEMORY_BLOCK_SIZE bytes:
     * an allocation whose rounded end is still below 0x7fe0 fits */
    char *a = (char *)ivp_u_memory_get_mem(&m, 0x7fe0 - 0x50 - 0x20);
    IVP_ASSERT_EQ(u_off(&m, a), 0);
    char *first_start = m.block_start;
    /* the next one does not fit: a new block, request at its start */
    char *b = (char *)ivp_u_memory_get_mem(&m, 100);
    IVP_ASSERT_TRUE(m.block_start != first_start);
    IVP_ASSERT_EQ(u_off(&m, b), 0);
    IVP_ASSERT_EQ(((uintptr_t)b & 31u), 0u);
    char *c = (char *)ivp_u_memory_get_mem(&m, 8);
    IVP_ASSERT_EQ(c - b, 160);
    memset(b, 0x77, 100);
    /* a request larger than a block gets a block of its own size */
    char *big = (char *)ivp_u_memory_get_mem(&m, 100000);
    IVP_ASSERT_NOT_NULL(big);
    IVP_ASSERT_EQ(u_off(&m, big), 0);
    big[99999] = 1;
    ivp_u_memory_end_transaction(&m);
    IVP_ASSERT_TRUE(m.block_start == first_start);
    /* the freed standard block comes back with its old contents (the
     * reference allocator reuses the slot without clearing it) */
    ivp_u_memory_start_transaction(&m);
    (void)ivp_u_memory_get_mem(&m, 0x7fe0 - 0x50 - 0x20);
    char *b2 = (char *)ivp_u_memory_get_mem(&m, 100);
    IVP_ASSERT_EQ((int)(unsigned char)b2[0], 0x77);
    IVP_ASSERT_EQ((int)(unsigned char)b2[99], 0x77);
    ivp_u_memory_end_transaction(&m);
    ivp_u_memory_destroy(&m);
}

IVP_TEST(u_memory_counting_allocator) {
    ivp_allocator_t ca = ivp_counting_allocator_create(NULL);
    ivp_u_memory_t m;
    IVP_ASSERT_EQ(ivp_u_memory_init(&m, &ca), IVP_OK);
    for (int t = 0; t < 3; t++) {
        ivp_u_memory_start_transaction(&m);
        for (int i = 0; i < 10; i++) IVP_ASSERT_NOT_NULL(ivp_u_memory_get_mem(&m, 20000));
        ivp_u_memory_end_transaction(&m);
    }
    ivp_u_memory_destroy(&m);
    ivp_alloc_stats_t st;
    ivp_counting_allocator_get_stats(&ca, &st);
    IVP_ASSERT_EQ(st.current_bytes, 0);
    IVP_ASSERT_EQ(st.alloc_count, st.free_count);
    ivp_counting_allocator_destroy(&ca);
}

IVP_TEST(u_memory_custom_first_block) {
    ivp_allocator_t ca = ivp_counting_allocator_create(NULL);
    ivp_u_memory_t m;
    const size_t first_block_size = 0x1800u;
    IVP_ASSERT_EQ(ivp_u_memory_init_first_block(&m, &ca,
                                                first_block_size), IVP_OK);
    IVP_ASSERT_EQ(m.first_block_size, first_block_size);

    ivp_alloc_stats_t st;
    ivp_counting_allocator_get_stats(&ca, &st);
    IVP_ASSERT_TRUE(st.current_bytes < (int64_t)IVP_U_MEM_BLOCK_SIZE);

    ivp_u_memory_start_transaction(&m);
    ivp_u_memory_end_transaction(&m);
    IVP_ASSERT_EQ(m.end, first_block_size);
    char *first_start = m.block_start;
    IVP_ASSERT_NOT_NULL(ivp_u_memory_get_mem(&m, first_block_size / 2u));
    IVP_ASSERT_NOT_NULL(ivp_u_memory_get_mem(&m, first_block_size / 2u));
    IVP_ASSERT_TRUE(m.block_start != first_start);
    IVP_ASSERT_EQ(m.end, (size_t)IVP_U_MEM_BLOCK_SIZE - IVP_U_MEM_ELEM_SIZE);

    ivp_u_memory_destroy(&m);
    ivp_counting_allocator_get_stats(&ca, &st);
    IVP_ASSERT_EQ(st.current_bytes, 0);
    IVP_ASSERT_EQ(st.alloc_count, st.free_count);
    ivp_counting_allocator_destroy(&ca);
}

/* ── Time tests ─────────────────────────────────────────────────────────── */

IVP_TEST(time_calc_steps_basic) {
    ivp_time_t residual = 0.0;
    int steps = ivp_time_calc_steps(0.1, IVP_DEFAULT_DELTA_TIME, &residual);
    IVP_ASSERT_TRUE(steps >= 6);
    IVP_ASSERT_TRUE(residual >= 0.0);
    IVP_ASSERT_TRUE(residual < IVP_DEFAULT_DELTA_TIME);
}

IVP_TEST(time_calc_steps_zero_dt) {
    ivp_time_t residual = 0.0;
    int steps = ivp_time_calc_steps(0.5, 0.0, &residual);
    IVP_ASSERT_EQ(steps, 0);
    IVP_ASSERT_NEAR(residual, 0.5, 1e-12);
}

IVP_TEST(time_calc_steps_exact) {
    ivp_time_t residual = 0.0;
    int steps = ivp_time_calc_steps(1.0, 0.25, &residual);
    IVP_ASSERT_EQ(steps, 4);
    IVP_ASSERT_NEAR(residual, 0.0, 1e-12);
}

IVP_TEST(time_calc_steps_clamp_int_max) {
    ivp_time_t dt = 0.25;
    ivp_time_t residual = 0.0;
    ivp_time_t elapsed = ((ivp_time_t)INT_MAX + 1024.0) * dt;

    int steps = ivp_time_calc_steps(elapsed, dt, &residual);
    IVP_ASSERT_EQ(steps, INT_MAX);
    IVP_ASSERT_TRUE(residual >= 0.0);
}

/* ── Types constants tests ──────────────────────────────────────────────── */

IVP_TEST(types_precision_constants) {
    IVP_ASSERT_TRUE(IVP_REAL_EPS > 0);
    IVP_ASSERT_TRUE(IVP_REAL_RES > 0);
    IVP_ASSERT_TRUE(IVP_REAL_MAX > 0);
    IVP_ASSERT_TRUE((double)IVP_PI > 3.14);
    IVP_ASSERT_TRUE((double)IVP_PI < 3.15);
    IVP_ASSERT_TRUE((double)IVP_PI_2 > 1.57);
    IVP_ASSERT_TRUE((double)IVP_PI_2 < 1.58);
}

IVP_TEST(types_coord_index) {
    IVP_ASSERT_EQ(IVP_INDEX_X, 0);
    IVP_ASSERT_EQ(IVP_INDEX_Y, 1);
    IVP_ASSERT_EQ(IVP_INDEX_Z, 2);
}

/* ── Main ───────────────────────────────────────────────────────────────── */

int main(void) {
    IVP_REGISTER(result_ok);
    IVP_REGISTER(result_str);
    IVP_REGISTER(message_prefix_preserves_bounded_output);
    IVP_REGISTER(default_alloc_basic);
    IVP_REGISTER(default_alloc_realloc);
    IVP_REGISTER(aligned_alloc_16);
    IVP_REGISTER(aligned_alloc_64);
    IVP_REGISTER(aligned_alloc_rejects_overflow);
    IVP_REGISTER(aligned_alloc_rejects_invalid_allocator);
    IVP_REGISTER(counting_alloc_stats);
    IVP_REGISTER(counting_alloc_destroy);
    IVP_REGISTER(arena_basic);
    IVP_REGISTER(arena_save_restore);
    IVP_REGISTER(arena_reset);
    IVP_REGISTER(arena_stats_smoke);
    IVP_REGISTER(u_memory_layout);
    IVP_REGISTER(u_memory_overflow_blocks);
    IVP_REGISTER(u_memory_counting_allocator);
    IVP_REGISTER(u_memory_custom_first_block);
    IVP_REGISTER(time_calc_steps_basic);
    IVP_REGISTER(time_calc_steps_zero_dt);
    IVP_REGISTER(time_calc_steps_exact);
    IVP_REGISTER(time_calc_steps_clamp_int_max);
    IVP_REGISTER(types_precision_constants);
    IVP_REGISTER(types_coord_index);
    IVP_RUN_ALL();
}
