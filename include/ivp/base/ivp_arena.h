/* ivp_arena.h — block arena for temporary engine storage
 *
 * One arena implementation serves both ordinary builder scratch storage and
 * the environment's transaction storage.  Its allocation layout is the
 * layout of the reference IVP_U_Memory: 32-byte aligned starts, 48--79 bytes
 * of reserved trailing space, and a retained first block.  Stable-internals
 * builds clear every reserved span; the default reference policy may retain
 * bytes across transactions.  Environment-owned instances select that
 * content policy; ordinary scratch arenas use the same allocation engine.
 */
#ifndef IVP_ARENA_H
#define IVP_ARENA_H

#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reference IVU_MEMORY_BLOCK_SIZE. */
#define IVP_ARENA_BLOCK_SIZE 0x7fe0u

typedef struct ivp_arena_stats {
    size_t block_count;
    size_t total_allocated;
    size_t high_water_mark;
    size_t alloc_count;
} ivp_arena_stats_t;

typedef struct ivp_arena ivp_arena_t;

typedef struct ivp_arena_checkpoint {
    void   *block;
    size_t  position;
    size_t  limit;
    size_t  live_allocated;
} ivp_arena_checkpoint_t;

/* Heap-owned arena for short-lived local work. */
ivp_arena_t *ivp_arena_create(const ivp_allocator_t *alloc);
/* Release every block; also release the arena object when create allocated it. */
void ivp_arena_destroy(ivp_arena_t *arena);

/* Allocate one reserved span.  The returned address is 32-byte aligned.
 * Returns NULL only for an invalid arena, overflow, or allocator failure. */
void *ivp_arena_alloc(ivp_arena_t *arena, size_t size);

/* Release overflow blocks and rewind the retained first block. */
void ivp_arena_reset(ivp_arena_t *arena);

/* Nested callers may save and restore ordinary scratch state. */
ivp_arena_checkpoint_t ivp_arena_save(const ivp_arena_t *arena);
void ivp_arena_restore(ivp_arena_t *arena, ivp_arena_checkpoint_t checkpoint);

void ivp_arena_get_stats(const ivp_arena_t *arena, ivp_arena_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ARENA_H */
