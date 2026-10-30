/* test_memory_region.c -- the memory region allocator and its in-place
 * checkpoints (ivp/base/ivp_memory_region.h). */
#include "ivp_test.h"

#include <ivp/base/ivp_memory_region.h>

#include <stdint.h>
#include <string.h>

#define MR_BLOCKS 64

static bool mr_all_zero(const void *ptr, size_t size)
{
    const unsigned char *bytes = (const unsigned char *)ptr;
    for (size_t i = 0; i < size; i++) {
        if (bytes[i]) return false;
    }
    return true;
}

/* sizes across both kinds of block */
static size_t mr_size(int i)
{
    static const size_t sizes[] = { 1, 8, 16, 24, 100, 1000, 4096, 4097,
                                    9000, 70000 };
    return sizes[i % (int)(sizeof(sizes) / sizeof(sizes[0]))];
}

IVP_TEST(blocks_are_aligned_zeroed_and_inside_the_region)
{
    ivp_memory_region_t *region = ivp_memory_region_create(0);
    IVP_ASSERT_TRUE(region != NULL);
    const ivp_allocator_t *a = ivp_memory_region_allocator(region);
    IVP_ASSERT_TRUE(ivp_memory_region_is_allocator(a));
    IVP_ASSERT_TRUE(!ivp_memory_region_is_allocator(ivp_default_allocator()));
    void *blocks[MR_BLOCKS];
    for (int i = 0; i < MR_BLOCKS; i++) {
        blocks[i] = ivp_alloc(a, mr_size(i));
        IVP_ASSERT_TRUE(blocks[i] != NULL);
        IVP_ASSERT_EQ((uintptr_t)blocks[i] % 16u, (uintptr_t)0);
        IVP_ASSERT_TRUE(mr_all_zero(blocks[i], mr_size(i)));
        memset(blocks[i], 0xA5, mr_size(i));
    }
    /* freed blocks come back zeroed */
    for (int i = 0; i < MR_BLOCKS; i += 2) ivp_free(a, blocks[i], mr_size(i));
    for (int i = 0; i < MR_BLOCKS; i += 2) {
        blocks[i] = ivp_alloc(a, mr_size(i));
        IVP_ASSERT_TRUE(blocks[i] != NULL);
        IVP_ASSERT_TRUE(mr_all_zero(blocks[i], mr_size(i)));
    }
    /* no two live blocks overlap */
    for (int i = 0; i < MR_BLOCKS; i++) {
        for (int j = i + 1; j < MR_BLOCKS; j++) {
            uintptr_t ai = (uintptr_t)blocks[i], aj = (uintptr_t)blocks[j];
            IVP_ASSERT_TRUE(ai + mr_size(i) <= aj || aj + mr_size(j) <= ai);
        }
    }
    ivp_memory_region_destroy(region);
}

IVP_TEST(realloc_keeps_contents_and_zeroes_growth)
{
    ivp_memory_region_t *region = ivp_memory_region_create(0);
    const ivp_allocator_t *a = ivp_memory_region_allocator(region);
    unsigned char *p = (unsigned char *)ivp_alloc(a, 10);
    for (int i = 0; i < 10; i++) p[i] = (unsigned char)(i + 1);
    p = (unsigned char *)ivp_realloc(a, p, 10, 12);   /* same class */
    IVP_ASSERT_TRUE(p[9] == 10 && p[10] == 0 && p[11] == 0);
    p = (unsigned char *)ivp_realloc(a, p, 12, 20000);   /* moves */
    IVP_ASSERT_TRUE(p != NULL);
    for (int i = 0; i < 10; i++) IVP_ASSERT_EQ(p[i], (unsigned char)(i + 1));
    IVP_ASSERT_TRUE(mr_all_zero(p + 12, 20000 - 12));
    p = (unsigned char *)ivp_realloc(a, p, 20000, 5);
    for (int i = 0; i < 5; i++) IVP_ASSERT_EQ(p[i], (unsigned char)(i + 1));
    ivp_free(a, p, 5);
    ivp_memory_region_destroy(region);
}

/* live blocks and the sizes they were allocated with */
typedef struct mr_table {
    void *blocks[MR_BLOCKS];
    size_t sizes[MR_BLOCKS];
} mr_table_t;

/* allocate, write and free in a fixed pattern; returns a fingerprint of
 * the addresses handed out and the bytes found */
static uint64_t mr_churn(const ivp_allocator_t *a, mr_table_t *t, int rounds)
{
    uint64_t hash = 0xcbf29ce484222325ull;
    for (int r = 0; r < rounds; r++) {
        for (int i = r % 3; i < MR_BLOCKS; i += 3) {
            if (t->blocks[i]) {
                hash = (hash ^ ((unsigned char *)t->blocks[i])[0]) * 0x100000001b3ull;
                ivp_free(a, t->blocks[i], t->sizes[i]);
                t->blocks[i] = NULL;
            } else {
                t->sizes[i] = mr_size(i + r);
                t->blocks[i] = ivp_alloc(a, t->sizes[i]);
                if (!t->blocks[i]) return 0;
                memset(t->blocks[i], (int)(i + r), t->sizes[i]);
                hash = (hash ^ (uint64_t)(uintptr_t)t->blocks[i]) * 0x100000001b3ull;
            }
        }
    }
    return hash;
}

IVP_TEST(restore_brings_back_blocks_bytes_and_allocator_state)
{
    ivp_memory_region_t *region = ivp_memory_region_create(0);
    const ivp_allocator_t *a = ivp_memory_region_allocator(region);
    /* the block table lives in the region, like a world's own pointers */
    mr_table_t *table = (mr_table_t *)ivp_alloc(a, sizeof(*table));
    IVP_ASSERT_TRUE(mr_churn(a, table, 7) != 0);
    size_t used = ivp_memory_region_used(region);
    ivp_memory_checkpoint_t *checkpoint =
        ivp_memory_region_checkpoint(region, NULL);
    IVP_ASSERT_TRUE(checkpoint != NULL);
    IVP_ASSERT_EQ(ivp_memory_checkpoint_size(checkpoint), used);

    uint64_t first = mr_churn(a, table, 11);
    IVP_ASSERT_TRUE(first != 0);
    IVP_ASSERT_TRUE(ivp_memory_region_used(region) > used);

    /* after a restore the same operations see the same addresses and
     * bytes, however often it is repeated */
    for (int again = 0; again < 3; again++) {
        IVP_ASSERT_EQ(ivp_memory_region_restore(region, checkpoint), IVP_OK);
        IVP_ASSERT_EQ(ivp_memory_region_used(region), used);
        IVP_ASSERT_TRUE(mr_churn(a, table, 11) == first);
    }
    ivp_memory_checkpoint_destroy(checkpoint);
    ivp_memory_region_destroy(region);
}

IVP_TEST(restore_rejects_another_regions_checkpoint)
{
    ivp_memory_region_t *one = ivp_memory_region_create(1u << 20);
    ivp_memory_region_t *two = ivp_memory_region_create(1u << 20);
    ivp_memory_checkpoint_t *checkpoint = ivp_memory_region_checkpoint(one, NULL);
    IVP_ASSERT_EQ(ivp_memory_region_restore(two, checkpoint), IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_memory_region_restore(two, NULL), IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_memory_region_restore(one, checkpoint), IVP_OK);
    ivp_memory_checkpoint_destroy(checkpoint);
    ivp_memory_region_destroy(one);
    ivp_memory_region_destroy(two);
}

IVP_TEST(a_full_region_fails_allocations_until_restored)
{
    ivp_memory_region_t *region = ivp_memory_region_create(1u << 20);
    IVP_ASSERT_TRUE(region != NULL);
    const ivp_allocator_t *a = ivp_memory_region_allocator(region);
    ivp_memory_checkpoint_t *empty = ivp_memory_region_checkpoint(region, NULL);
    int count = 0;
    while (ivp_alloc(a, 40000)) count++;
    IVP_ASSERT_TRUE(count > 10 && count < 30);
    IVP_ASSERT_TRUE(ivp_alloc(a, 40000) == NULL);
    IVP_ASSERT_EQ(ivp_memory_region_restore(region, empty), IVP_OK);
    IVP_ASSERT_TRUE(ivp_alloc(a, 40000) != NULL);
    ivp_memory_checkpoint_destroy(empty);
    ivp_memory_region_destroy(region);
}

IVP_TEST(oversized_requests_fail_and_are_counted)
{
    ivp_memory_region_t *region = ivp_memory_region_create(1u << 20);
    const ivp_allocator_t *a = ivp_memory_region_allocator(region);
    IVP_ASSERT_EQ(ivp_memory_region_failures(region), (size_t)0);
    /* rounding SIZE_MAX up to a page must not wrap to a tiny block */
    IVP_ASSERT_TRUE(ivp_alloc(a, SIZE_MAX) == NULL);
    IVP_ASSERT_TRUE(ivp_alloc(a, SIZE_MAX - 100) == NULL);
    void *small = ivp_alloc(a, 64);
    IVP_ASSERT_TRUE(small != NULL);
    IVP_ASSERT_TRUE(ivp_realloc(a, small, 64, SIZE_MAX) == NULL);
    IVP_ASSERT_EQ(ivp_memory_region_failures(region), (size_t)3);
    /* the count lives outside the region: a restore keeps it */
    ivp_memory_checkpoint_t *checkpoint = ivp_memory_region_checkpoint(region, NULL);
    IVP_ASSERT_TRUE(ivp_alloc(a, 2u << 20) == NULL);
    IVP_ASSERT_EQ(ivp_memory_region_restore(region, checkpoint), IVP_OK);
    IVP_ASSERT_EQ(ivp_memory_region_failures(region), (size_t)4);
    ivp_memory_checkpoint_destroy(checkpoint);
    ivp_memory_region_destroy(region);
}

IVP_TEST(a_checkpoint_is_not_kept_in_its_own_region)
{
    ivp_memory_region_t *region = ivp_memory_region_create(1u << 20);
    IVP_ASSERT_TRUE(ivp_memory_region_checkpoint(
                        region, ivp_memory_region_allocator(region)) == NULL);
    ivp_memory_region_t *other = ivp_memory_region_create(1u << 20);
    ivp_memory_checkpoint_t *checkpoint = ivp_memory_region_checkpoint(
        region, ivp_memory_region_allocator(other));
    IVP_ASSERT_TRUE(checkpoint != NULL);
    IVP_ASSERT_EQ(ivp_memory_region_restore(region, checkpoint), IVP_OK);
    ivp_memory_checkpoint_destroy(checkpoint);
    ivp_memory_region_destroy(other);
    ivp_memory_region_destroy(region);
}

/* A region created after another was destroyed may get its address; the
 * old region's checkpoints must still be refused. */
IVP_TEST(a_new_region_refuses_an_old_regions_checkpoint)
{
    for (int attempt = 0; attempt < 8; attempt++) {
        ivp_memory_region_t *old = ivp_memory_region_create(1u << 20);
        ivp_memory_checkpoint_t *checkpoint = ivp_memory_region_checkpoint(old, NULL);
        ivp_memory_region_destroy(old);
        ivp_memory_region_t *fresh = ivp_memory_region_create(1u << 20);
        IVP_ASSERT_EQ(ivp_memory_region_restore(fresh, checkpoint),
                      IVP_ERR_INVALID_ARG);
        ivp_memory_checkpoint_destroy(checkpoint);
        ivp_memory_region_destroy(fresh);
    }
}

int main(void)
{
    IVP_REGISTER(blocks_are_aligned_zeroed_and_inside_the_region);
    IVP_REGISTER(realloc_keeps_contents_and_zeroes_growth);
    IVP_REGISTER(restore_brings_back_blocks_bytes_and_allocator_state);
    IVP_REGISTER(restore_rejects_another_regions_checkpoint);
    IVP_REGISTER(a_full_region_fails_allocations_until_restored);
    IVP_REGISTER(oversized_requests_fail_and_are_counted);
    IVP_REGISTER(a_checkpoint_is_not_kept_in_its_own_region);
    IVP_REGISTER(a_new_region_refuses_an_old_regions_checkpoint);
    IVP_RUN_ALL();
}
