/* ivp_allocator.h — Pluggable memory allocator for IVP C17
 *
 * Provides a vtable-based allocator. Default implementation uses malloc/free.
 * Debug mode supports magic-number validation and aligned allocation.
 */
#ifndef IVP_ALLOCATOR_H
#define IVP_ALLOCATOR_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Allocator vtable ───────────────────────────────────────────────────── */

typedef struct ivp_allocator {
    /* alloc/realloc return NULL without changing existing storage on
     * failure.  free must accept NULL.  old_size/size are the exact sizes
     * supplied by the owning module.  The vtable context must outlive every
     * allocation made through it, including state that may outlive an
     * environment. */
    void *(*alloc)(size_t size, void *ctx);
    void *(*realloc)(void *ptr, size_t old_size, size_t new_size, void *ctx);
    void  (*free)(void *ptr, size_t size, void *ctx);
    void *ctx;  /* user context (e.g. debug counters, pool handle) */
} ivp_allocator_t;

/* Default allocator (malloc/free). Thread-safe (uses libc). */
const ivp_allocator_t *ivp_default_allocator(void);

/* ── Convenience wrappers ───────────────────────────────────────────────── */

static inline void *ivp_alloc(const ivp_allocator_t *a, size_t size) {
    return a->alloc(size, a->ctx);
}

static inline void *ivp_realloc(const ivp_allocator_t *a, void *ptr,
                                 size_t old_size, size_t new_size) {
    return a->realloc(ptr, old_size, new_size, a->ctx);
}

static inline void ivp_free(const ivp_allocator_t *a, void *ptr, size_t size) {
    a->free(ptr, size, a->ctx);
}

/* ── Aligned allocation (matching original IVP magic 0x65981234) ────────── */

/* Allocate memory with specified alignment.
 * In debug mode, a magic header (0x65981234) is prepended for validation.
 * Returns NULL on failure. */
void *ivp_alloc_aligned(const ivp_allocator_t *a, size_t size, size_t alignment);

/* Free aligned memory previously allocated with ivp_alloc_aligned. */
void  ivp_free_aligned(const ivp_allocator_t *a, void *ptr);

/* ── Debug counting allocator ───────────────────────────────────────────── */

typedef struct ivp_alloc_stats {
    int64_t alloc_count;
    int64_t free_count;
    int64_t bytes_allocated;
    int64_t bytes_freed;
    int64_t peak_bytes;
    int64_t current_bytes;
} ivp_alloc_stats_t;

/* Create a counting allocator wrapping another allocator (or default).
 * Destroy it only after every allocation made through it has been freed. */
ivp_allocator_t ivp_counting_allocator_create(const ivp_allocator_t *inner);
/* Destroy a counting allocator and restore passthrough behavior. */
void            ivp_counting_allocator_destroy(ivp_allocator_t *a);
void            ivp_counting_allocator_get_stats(const ivp_allocator_t *a,
                                                  ivp_alloc_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ALLOCATOR_H */
