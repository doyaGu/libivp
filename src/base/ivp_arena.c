/* ivp_arena.c — unified block and transaction arena
 *
 * The block layout and allocation rounding reproduce the reference
 * IVP_U_Memory implementation.  Checkpoints and heap-owned lifecycle add the
 * ordinary scratch-arena use case without maintaining a second allocator.
 */
#include "base/ivp_arena_internal.h"
#include <ivp/base/ivp_assert.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>

#if defined(__has_feature)
#if __has_feature(address_sanitizer)
#define IVP_ARENA_ASAN 1
#endif
#endif
#if !defined(IVP_ARENA_ASAN) && defined(__SANITIZE_ADDRESS__)
#define IVP_ARENA_ASAN 1
#endif
#ifdef IVP_ARENA_ASAN
#include <sanitizer/asan_interface.h>
#endif

typedef struct arena_block_header {
    char  *next;
    size_t total;
} arena_block_header_t;

_Static_assert(sizeof(arena_block_header_t) == IVP_ARENA_BLOCK_HEADER_SIZE,
               "reference arena block header layout changed");
#if UINTPTR_MAX == UINT64_MAX && IVP_ARENA_RECYCLE_BLOCKS == 4 && \
    !defined(IVP_ENABLE_STATS)
_Static_assert(sizeof(ivp_arena_t) == 104u,
               "reference arena owner layout changed");
#endif

/* Reference constants: IVU_MEM_ALIGN is the reservation added before the
 * 32-byte mask is applied; p_Memory_Elem::data starts eight bytes in. */
#define ARENA_RESERVE           0x50u
#define ARENA_ALIGNMENT_MASK    0x1fu
#define ARENA_BLOCK_DATA_OFFSET 8u
#define ARENA_BLOCK_EXTRA \
    ((size_t)IVP_ARENA_BLOCK_HEADER_SIZE + (size_t)ARENA_RESERVE + 32u)

static inline size_t arena_round(size_t n)
{
    return (n + ((size_t)ARENA_RESERVE - 1u)) &
           ~(size_t)ARENA_ALIGNMENT_MASK;
}

static inline arena_block_header_t *arena_header(char *raw)
{
    return (arena_block_header_t *)(void *)raw;
}

static inline char *arena_data_start(char *raw)
{
    uintptr_t address = (uintptr_t)raw + (uintptr_t)ARENA_BLOCK_DATA_OFFSET;
    address = (address + ((uintptr_t)ARENA_RESERVE - 1u)) &
              ~(uintptr_t)ARENA_ALIGNMENT_MASK;
    return raw + (address - (uintptr_t)raw);
}

static inline void arena_guard(char *raw, size_t usable)
{
#ifdef IVP_ARENA_ASAN
    size_t total = arena_header(raw)->total;
    char *data = arena_data_start(raw);
    char *header_end = raw + sizeof(arena_block_header_t);
    ASAN_UNPOISON_MEMORY_REGION(raw, total);
    ASAN_POISON_MEMORY_REGION(header_end,
                              (size_t)(data - header_end));
    ASAN_POISON_MEMORY_REGION(data + usable,
                              total - (size_t)(data - raw) - usable);
#else
    (void)raw;
    (void)usable;
#endif
}

static inline void arena_unguard(char *raw)
{
#ifdef IVP_ARENA_ASAN
    ASAN_UNPOISON_MEMORY_REGION(raw, arena_header(raw)->total);
#else
    (void)raw;
#endif
}

static void arena_release_block(ivp_arena_t *arena, char *raw)
{
#if IVP_ARENA_RECYCLE_BLOCKS > 0
    if (arena->recycled_count == IVP_ARENA_RECYCLE_BLOCKS) {
        char *oldest = arena->recycled[0];
        arena_unguard(oldest);
        ivp_free(arena->alloc, oldest, arena_header(oldest)->total);
        for (int i = 1; i < arena->recycled_count; i++) {
            arena->recycled[i - 1] = arena->recycled[i];
        }
        arena->recycled_count--;
    }
    arena->recycled[arena->recycled_count++] = raw;
#else
    arena_unguard(raw);
    ivp_free(arena->alloc, raw, arena_header(raw)->total);
#endif
#ifdef IVP_ENABLE_STATS
    if (arena->stats.block_count > 0u) arena->stats.block_count--;
#endif
}

static char *arena_acquire_block(ivp_arena_t *arena, size_t total)
{
#if IVP_ARENA_RECYCLE_BLOCKS > 0
    for (int i = arena->recycled_count - 1; i >= 0; i--) {
        char *raw = arena->recycled[i];
        if (arena_header(raw)->total == total) {
            for (int k = i + 1; k < arena->recycled_count; k++) {
                arena->recycled[k - 1] = arena->recycled[k];
            }
            arena->recycled_count--;
            return raw;
        }
    }
#endif
    char *raw = (char *)ivp_alloc(arena->alloc, total);
    if (!raw) return NULL;
    memset(raw, 0, total);
    arena_header(raw)->total = total;
    return raw;
}

static void *arena_new_block(ivp_arena_t *arena, size_t size)
{
    size_t block_size = arena->first_block
        ? (size_t)IVP_ARENA_BLOCK_SIZE
        : arena->first_block_size;
    size_t usable = block_size - (size_t)IVP_ARENA_BLOCK_HEADER_SIZE;
    if (size > SIZE_MAX / 2u) return NULL;
    size = arena_round(size);
    if (size > usable) usable = size;
    if (usable > SIZE_MAX - ARENA_BLOCK_EXTRA) return NULL;

    char *raw = arena_acquire_block(arena, usable + ARENA_BLOCK_EXTRA);
    if (!raw) return NULL;
    arena_header(raw)->next = arena->current_block;
    arena->current_block = raw;
    arena->block_data = arena_data_start(raw);
    arena->position = size;
    arena->limit = usable;
    arena_guard(raw, usable);
    if (arena->clear_on_allocate && size != 0u) {
        memset(arena->block_data, 0, size);
    }
#ifdef IVP_ENABLE_STATS
    arena->stats.block_count++;
#endif
    return arena->block_data;
}

static void arena_record_allocation(ivp_arena_t *arena, size_t size,
                                    size_t reserved)
{
#ifdef IVP_ENABLE_STATS
    arena->stats.total_allocated += size;
    arena->stats.alloc_count++;
    arena->live_allocated += reserved;
    if (arena->live_allocated > arena->stats.high_water_mark) {
        arena->stats.high_water_mark = arena->live_allocated;
    }
#else
    (void)arena;
    (void)size;
    (void)reserved;
#endif
}

ivp_result_t ivp_arena_init_sized(ivp_arena_t *arena,
                                  const ivp_allocator_t *alloc,
                                  size_t first_block_size)
{
    if (!arena || first_block_size <= IVP_ARENA_BLOCK_HEADER_SIZE ||
        first_block_size > IVP_ARENA_BLOCK_SIZE) {
        return IVP_ERR_INVALID_ARG;
    }
    memset(arena, 0, sizeof(*arena));
    arena->alloc = alloc ? alloc : ivp_default_allocator();
    arena->first_block_size = first_block_size;
    if (!arena_new_block(arena, 0u)) return IVP_ERR_OUT_OF_MEMORY;
    arena->first_block = arena->current_block;
    return IVP_OK;
}

ivp_result_t ivp_arena_init(ivp_arena_t *arena,
                            const ivp_allocator_t *alloc)
{
    return ivp_arena_init_sized(arena, alloc, IVP_ARENA_BLOCK_SIZE);
}

ivp_arena_t *ivp_arena_create(const ivp_allocator_t *alloc)
{
    const ivp_allocator_t *actual = alloc ? alloc : ivp_default_allocator();
    ivp_arena_t *arena = (ivp_arena_t *)ivp_alloc(actual, sizeof(*arena));
    if (!arena) return NULL;
    if (ivp_arena_init(arena, actual) != IVP_OK) {
        ivp_free(actual, arena, sizeof(*arena));
        return NULL;
    }
    arena->owns_self = true;
    return arena;
}

void ivp_arena_destroy(ivp_arena_t *arena)
{
    if (!arena || !arena->alloc) return;
    const ivp_allocator_t *alloc = arena->alloc;
    bool owns_self = arena->owns_self;
    for (char *block = arena->current_block; block;) {
        char *next = arena_header(block)->next;
        arena_unguard(block);
        ivp_free(arena->alloc, block, arena_header(block)->total);
        block = next;
    }
#if IVP_ARENA_RECYCLE_BLOCKS > 0
    for (int i = 0; i < arena->recycled_count; i++) {
        arena_unguard(arena->recycled[i]);
        ivp_free(arena->alloc, arena->recycled[i],
                 arena_header(arena->recycled[i])->total);
    }
#endif
    memset(arena, 0, sizeof(*arena));
    if (owns_self) ivp_free(alloc, arena, sizeof(*arena));
}

void *ivp_arena_alloc(ivp_arena_t *arena, size_t size)
{
    if (!arena || !arena->current_block) return NULL;
    size_t old_position = arena->position;
    if (size > SIZE_MAX - old_position - (size_t)ARENA_RESERVE) return NULL;
    size_t next_position = arena_round(old_position + size);
    void *result;
    size_t reserved;
    if (next_position >= arena->limit) {
        result = arena_new_block(arena, size);
        reserved = result ? arena->position : 0u;
    } else {
        arena->position = next_position;
        result = arena->block_data + old_position;
        reserved = next_position - old_position;
        if (arena->clear_on_allocate) {
            memset(result, 0, next_position - old_position);
        }
    }
    if (result) arena_record_allocation(arena, size, reserved);
    IVP_IF(result && ((intptr_t)result > 0x780000) &&
           ((intptr_t)result < 0x792f48)) /* ivu_memory.hxx:109, no-op */
    {
        char *probe = (char *)result;
        probe++;
        probe--;
        (void)probe;
    }
    return result;
}

void ivp_arena_reset(ivp_arena_t *arena)
{
    if (!arena || !arena->first_block) return;
    for (char *block = arena->current_block; block;) {
        char *next = arena_header(block)->next;
        if (block == arena->first_block) break;
        arena_release_block(arena, block);
        block = next;
    }
    arena->current_block = arena->first_block;
    arena->block_data = arena_data_start(arena->first_block);
    arena->position = 0u;
    arena->limit = arena->first_block_size;
    arena_guard(arena->first_block, arena->limit);
#ifdef IVP_ENABLE_STATS
    arena->live_allocated = 0u;
    arena->stats.block_count = 1u;
#endif
}

ivp_arena_checkpoint_t ivp_arena_save(const ivp_arena_t *arena)
{
    ivp_arena_checkpoint_t checkpoint = {0};
    if (!arena) return checkpoint;
    checkpoint.block = arena->current_block;
    checkpoint.position = arena->position;
    checkpoint.limit = arena->limit;
#ifdef IVP_ENABLE_STATS
    checkpoint.live_allocated = arena->live_allocated;
#endif
    return checkpoint;
}

void ivp_arena_restore(ivp_arena_t *arena,
                       ivp_arena_checkpoint_t checkpoint)
{
    if (!arena || !checkpoint.block) return;
    char *saved = (char *)checkpoint.block;
    char *block = arena->current_block;
    while (block && block != saved) {
        char *next = arena_header(block)->next;
        arena_release_block(arena, block);
        block = next;
    }
    if (!block) return;
    arena->current_block = saved;
    arena->block_data = arena_data_start(saved);
    arena->position = checkpoint.position;
    arena->limit = checkpoint.limit;
    arena_guard(saved, arena->limit);
#ifdef IVP_ENABLE_STATS
    arena->live_allocated = checkpoint.live_allocated;
#endif
}

void ivp_arena_begin_transaction(ivp_arena_t *arena)
{
    if (!arena) return;
    IVP_ASSERT(arena->transaction_depth >= 0);
    if (arena->transaction_depth == INT_MAX) return;
    arena->transaction_depth++;
}

void ivp_arena_end_transaction(ivp_arena_t *arena)
{
    if (!arena) return;
    IVP_ASSERT_MSG(arena->transaction_depth > 0,
                   "arena transaction underflow");
    if (arena->transaction_depth <= 0) return;
    arena->transaction_depth--;
    if (arena->transaction_depth == 0) ivp_arena_reset(arena);
}

void ivp_arena_set_clear_on_allocate(ivp_arena_t *arena, bool enable)
{
    if (arena) arena->clear_on_allocate = enable;
}

void ivp_arena_get_stats(const ivp_arena_t *arena, ivp_arena_stats_t *out)
{
    if (!arena || !out) return;
#ifdef IVP_ENABLE_STATS
    *out = arena->stats;
#else
    memset(out, 0, sizeof(*out));
#endif
}
