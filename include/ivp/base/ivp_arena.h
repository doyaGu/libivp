/* ivp_arena.h — Bump-pointer arena allocator for IVP C17
 *
 * Linked-list of blocks (default ~32KB each).
 * Supports save/restore (stack-based checkpoint) for per-frame temporaries.
 */
#ifndef IVP_ARENA_H
#define IVP_ARENA_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants (matching original IVP) ──────────────────────────────────── */

/* Block size ~32KB (original: 0x7FE0) */
#define IVP_ARENA_BLOCK_SIZE  0x7FE0u

/* Default alignment (32 bytes on 32-bit, 80 bytes on 64-bit) */
#if defined(__LP64__) || defined(_WIN64) || defined(__x86_64__)
    #define IVP_ARENA_ALIGN  80u
#else
    #define IVP_ARENA_ALIGN  32u
#endif

/* ── Arena types ────────────────────────────────────────────────────────── */

typedef struct ivp_arena ivp_arena_t;

/* Checkpoint for save/restore */
typedef struct ivp_arena_checkpoint {
    void *block;    /* saved block pointer */
    char *cursor;   /* saved cursor position */
} ivp_arena_checkpoint_t;

/* ── API ────────────────────────────────────────────────────────────────── */

/* Create an arena. Uses the given allocator for block allocation. */
ivp_arena_t *ivp_arena_create(const ivp_allocator_t *alloc);

/* Destroy the arena and free all blocks. */
void ivp_arena_destroy(ivp_arena_t *arena);

/* Allocate from the arena (bump pointer, aligned). Returns NULL on OOM. */
void *ivp_arena_alloc(ivp_arena_t *arena, size_t size);

/* Reset the arena: free all blocks except the first one, reset cursor. */
void ivp_arena_reset(ivp_arena_t *arena);

/* Save current state (checkpoint). Can be nested. */
ivp_arena_checkpoint_t ivp_arena_save(const ivp_arena_t *arena);

/* Restore to a previously saved checkpoint.
 * Frees all blocks allocated after the checkpoint (except the checkpoint block). */
void ivp_arena_restore(ivp_arena_t *arena, ivp_arena_checkpoint_t cp);

/* ── Statistics (optional, enabled by IVP_ENABLE_STATS) ─────────────────── */

typedef struct ivp_arena_stats {
    size_t block_count;
    size_t total_allocated;
    size_t high_water_mark;
    size_t alloc_count;
} ivp_arena_stats_t;

void ivp_arena_get_stats(const ivp_arena_t *arena, ivp_arena_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ARENA_H */
