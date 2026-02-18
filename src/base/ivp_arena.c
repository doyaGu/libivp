/* ivp_arena.c - Bump-pointer arena allocator implementation */
#include <ivp/base/ivp_arena.h>
#include <ivp/base/ivp_assert.h>
#include <string.h>

/* Internal types */
typedef struct ivp_arena_block {
    struct ivp_arena_block *next;   /* linked list (newest->oldest) */
    size_t                  capacity;
#ifdef IVP_ENABLE_STATS
    size_t                  used;
#endif
    /* data follows inline: block + 1 */
} ivp_arena_block_t;

struct ivp_arena {
    const ivp_allocator_t *alloc;
    ivp_arena_block_t     *first;   /* oldest block (never freed in reset) */
    ivp_arena_block_t     *current; /* most recent block */
    char                  *cursor;  /* next free byte (aligned) */
    char                  *end;     /* end of current block data */

#ifdef IVP_ENABLE_STATS
    ivp_arena_stats_t      stats;
#endif
};

/* Helpers */
static inline uintptr_t align_up(uintptr_t p, size_t align) {
    return (p + align - 1) & ~(align - 1);
}

static ivp_arena_block_t *arena_new_block(const ivp_allocator_t *a,
                                          size_t min_data_size) {
    size_t data_size = IVP_ARENA_BLOCK_SIZE;
    if (min_data_size > data_size) data_size = min_data_size;

    size_t total = sizeof(ivp_arena_block_t) + data_size + IVP_ARENA_ALIGN;
    ivp_arena_block_t *b = (ivp_arena_block_t *)ivp_alloc(a, total);
    if (!b) return NULL;

    b->next = NULL;
    b->capacity = data_size + IVP_ARENA_ALIGN;
#ifdef IVP_ENABLE_STATS
    b->used = 0;
#endif
    return b;
}

static inline char *block_data_start(ivp_arena_block_t *b) {
    uintptr_t base = (uintptr_t)(b + 1);
    return (char *)align_up(base, IVP_ARENA_ALIGN);
}

static inline char *block_data_end(ivp_arena_block_t *b) {
    return (char *)(b + 1) + b->capacity;
}

/* API implementation */
ivp_arena_t *ivp_arena_create(const ivp_allocator_t *alloc) {
    if (!alloc) alloc = ivp_default_allocator();

    ivp_arena_t *arena = (ivp_arena_t *)ivp_alloc(alloc, sizeof(ivp_arena_t));
    if (!arena) return NULL;
    memset(arena, 0, sizeof(*arena));

    arena->alloc = alloc;

    ivp_arena_block_t *b = arena_new_block(alloc, 0);
    if (!b) {
        ivp_free(alloc, arena, sizeof(ivp_arena_t));
        return NULL;
    }
    arena->first = b;
    arena->current = b;
    arena->cursor = block_data_start(b);
    arena->end = block_data_end(b);

#ifdef IVP_ENABLE_STATS
    arena->stats.block_count = 1;
#endif

    return arena;
}

void ivp_arena_destroy(ivp_arena_t *arena) {
    if (!arena) return;
    const ivp_allocator_t *a = arena->alloc;

    /* Free all blocks */
    ivp_arena_block_t *b = arena->current;
    while (b) {
        ivp_arena_block_t *prev = b->next;
        size_t total = sizeof(ivp_arena_block_t) + b->capacity;
        ivp_free(a, b, total);
        b = prev;
    }

    ivp_free(a, arena, sizeof(ivp_arena_t));
}

void *ivp_arena_alloc(ivp_arena_t *arena, size_t size) {
    if (!arena || size == 0) return NULL;

    /* Align the allocation */
    uintptr_t cur = (uintptr_t)arena->cursor;
    uintptr_t aligned = align_up(cur, IVP_ARENA_ALIGN);
    char *p = (char *)aligned;
    char *new_cursor = p + size;

    /* Align the new cursor too for next allocation */
    new_cursor = (char *)align_up((uintptr_t)new_cursor, IVP_ARENA_ALIGN);

    if (new_cursor <= arena->end) {
        arena->cursor = new_cursor;
#ifdef IVP_ENABLE_STATS
        arena->stats.total_allocated += size;
        arena->stats.alloc_count++;
        arena->current->used = (size_t)(arena->cursor - block_data_start(arena->current));

        size_t live_used = 0;
        for (const ivp_arena_block_t *b = arena->current; b; b = b->next) {
            live_used += b->used;
        }
        if (live_used > arena->stats.high_water_mark) {
            arena->stats.high_water_mark = live_used;
        }
#endif
        return p;
    }

    /* Need a new block */
    ivp_arena_block_t *b = arena_new_block(arena->alloc, size + IVP_ARENA_ALIGN);
    if (!b) return NULL;

    b->next = arena->current;
    arena->current = b;
    arena->cursor = block_data_start(b);
    arena->end = block_data_end(b);

#ifdef IVP_ENABLE_STATS
    arena->stats.block_count++;
#endif

    /* Recurse (will succeed now) */
    return ivp_arena_alloc(arena, size);
}

void ivp_arena_reset(ivp_arena_t *arena) {
    if (!arena) return;

    /* Free all blocks except first */
    ivp_arena_block_t *b = arena->current;
    while (b && b != arena->first) {
        ivp_arena_block_t *prev = b->next;
        size_t total = sizeof(ivp_arena_block_t) + b->capacity;
        ivp_free(arena->alloc, b, total);
        b = prev;
    }

    arena->current = arena->first;
    arena->first->next = NULL;
    arena->cursor = block_data_start(arena->first);
    arena->end = block_data_end(arena->first);

#ifdef IVP_ENABLE_STATS
    arena->first->used = 0;
    arena->stats.block_count = 1;
#endif
}

ivp_arena_checkpoint_t ivp_arena_save(const ivp_arena_t *arena) {
    ivp_arena_checkpoint_t cp;
    cp.block = arena->current;
    cp.cursor = arena->cursor;
    return cp;
}

void ivp_arena_restore(ivp_arena_t *arena, ivp_arena_checkpoint_t cp) {
    if (!arena) return;

    /* Free blocks allocated after the checkpoint */
    ivp_arena_block_t *saved = (ivp_arena_block_t *)cp.block;
    ivp_arena_block_t *b = arena->current;
    while (b && b != saved) {
        ivp_arena_block_t *prev = b->next;
        size_t total = sizeof(ivp_arena_block_t) + b->capacity;
        ivp_free(arena->alloc, b, total);
#ifdef IVP_ENABLE_STATS
        if (arena->stats.block_count > 0) {
            arena->stats.block_count--;
        }
#endif
        b = prev;
    }

    arena->current = saved;
    arena->cursor = cp.cursor;
    arena->end = block_data_end(saved);
#ifdef IVP_ENABLE_STATS
    if (saved) {
        saved->used = (size_t)(arena->cursor - block_data_start(saved));
    }
#endif
}

void ivp_arena_get_stats(const ivp_arena_t *arena, ivp_arena_stats_t *out) {
    if (!arena || !out) return;
#ifdef IVP_ENABLE_STATS
    *out = arena->stats;
#else
    memset(out, 0, sizeof(*out));
#endif
}

