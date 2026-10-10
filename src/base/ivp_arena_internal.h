/* ivp_arena_internal.h — inline lifecycle and state for engine-owned arenas */
#ifndef IVP_ARENA_INTERNAL_H
#define IVP_ARENA_INTERNAL_H

#include <ivp/base/ivp_arena.h>
#include <ivp/base/ivp_types.h>

#define IVP_ARENA_BLOCK_HEADER_SIZE 16u

#ifndef IVP_ARENA_RECYCLE_BLOCKS
#define IVP_ARENA_RECYCLE_BLOCKS 4
#endif

/* Kept inline in each environment so arena setup adds no allocation whose
 * address or history could affect the reference-internals build. */
struct ivp_arena {
    const ivp_allocator_t *alloc;
    char   *first_block;
    char   *current_block;
    char   *block_data;
    size_t  position;
    size_t  limit;
    size_t  first_block_size;
    int     transaction_depth;
    bool    clear_on_allocate;
    bool    owns_self;
#if IVP_ARENA_RECYCLE_BLOCKS > 0
    char   *recycled[IVP_ARENA_RECYCLE_BLOCKS];
    int     recycled_count;
#endif
#ifdef IVP_ENABLE_STATS
    ivp_arena_stats_t stats;
    size_t live_allocated;
#endif
};

ivp_result_t ivp_arena_init(ivp_arena_t *arena,
                            const ivp_allocator_t *alloc);
ivp_result_t ivp_arena_init_sized(ivp_arena_t *arena,
                                  const ivp_allocator_t *alloc,
                                  size_t first_block_size);

/* Transactions retain the first block and its byte history. */
void ivp_arena_begin_transaction(ivp_arena_t *arena);
void ivp_arena_end_transaction(ivp_arena_t *arena);

/* Clear every future requested span and its reserved trailing bytes. */
void ivp_arena_set_clear_on_allocate(ivp_arena_t *arena, bool enable);

#endif /* IVP_ARENA_INTERNAL_H */
