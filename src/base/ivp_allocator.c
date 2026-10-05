/* ivp_allocator.c — Default and counting allocator implementations */
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_assert.h>
#include <stdlib.h>
#include <string.h>

/* ── Default allocator (malloc/free) ────────────────────────────────────── */

static void *default_alloc(size_t size, void *ctx) {
    IVP_UNUSED(ctx);
    return malloc(size);
}

static void *default_realloc(void *ptr, size_t old_size, size_t new_size, void *ctx) {
    IVP_UNUSED(ctx);
    IVP_UNUSED(old_size);
    return realloc(ptr, new_size);
}

static void default_free(void *ptr, size_t size, void *ctx) {
    IVP_UNUSED(ctx);
    IVP_UNUSED(size);
    free(ptr);
}

static const ivp_allocator_t s_default_allocator = {
    .alloc   = default_alloc,
    .realloc = default_realloc,
    .free    = default_free,
    .ctx     = NULL
};

const ivp_allocator_t *ivp_default_allocator(void) {
    return &s_default_allocator;
}

/* ── Aligned allocation ─────────────────────────────────────────────────── */

/* Debug magic number matching original IVP (0x65981234) */
#define IVP_ALIGNED_MAGIC  0x65981234u

typedef struct {
    uint32_t magic;
    size_t   total_size;
    size_t   offset;  /* offset from raw allocation to aligned pointer */
} ivp_aligned_header_t;

void *ivp_alloc_aligned(const ivp_allocator_t *a, size_t size, size_t alignment) {
    if (!a || !a->alloc || size == 0 || alignment == 0) return NULL;

    /* Ensure alignment is power of 2 and at least sizeof(void*) */
    if (alignment < sizeof(void*)) alignment = sizeof(void*);
    /* Round up alignment to power-of-2 */
    size_t align = 1;
    while (align < alignment) {
        if (align > SIZE_MAX / 2u) return NULL;
        align <<= 1;
    }

    size_t header_size = sizeof(ivp_aligned_header_t);
    if (size > SIZE_MAX - header_size) return NULL;
    if (align > SIZE_MAX - size - header_size) return NULL;
    size_t total = size + align + header_size;
    char *raw = (char *)a->alloc(total, a->ctx);
    if (!raw) return NULL;

    /* Align pointer past the header */
    uintptr_t raw_addr = (uintptr_t)(raw + header_size);
    uintptr_t aligned_addr = (raw_addr + align - 1) & ~(align - 1);
    char *aligned = (char *)aligned_addr;

    /* Store header just before the aligned pointer */
    ivp_aligned_header_t *hdr = (ivp_aligned_header_t *)(aligned) - 1;
    hdr->magic      = IVP_ALIGNED_MAGIC;
    hdr->total_size = total;
    hdr->offset     = (size_t)(aligned - raw);

    return aligned;
}

void ivp_free_aligned(const ivp_allocator_t *a, void *ptr) {
    if (!a || !a->free || !ptr) return;

    ivp_aligned_header_t *hdr = ((ivp_aligned_header_t *)ptr) - 1;
    IVP_ASSERT(hdr->magic == IVP_ALIGNED_MAGIC);
    IVP_IF(1) /* ivu_memory.cxx:186 */
    {
        hdr->magic = 0;
    }

    char *raw = (char *)ptr - hdr->offset;
    a->free(raw, hdr->total_size, a->ctx);
}

/* ── Counting allocator ─────────────────────────────────────────────────── */

typedef struct {
    const ivp_allocator_t *inner;
    ivp_alloc_stats_t      stats;
} ivp_counting_ctx_t;

static void *counting_alloc(size_t size, void *ctx) {
    ivp_counting_ctx_t *c = (ivp_counting_ctx_t *)ctx;
    void *ptr = c->inner->alloc(size, c->inner->ctx);
    if (ptr) {
        c->stats.alloc_count++;
        c->stats.bytes_allocated += (int64_t)size;
        c->stats.current_bytes   += (int64_t)size;
        if (c->stats.current_bytes > c->stats.peak_bytes) {
            c->stats.peak_bytes = c->stats.current_bytes;
        }
    }
    return ptr;
}

static void *counting_realloc(void *ptr, size_t old_size, size_t new_size, void *ctx) {
    ivp_counting_ctx_t *c = (ivp_counting_ctx_t *)ctx;
    void *new_ptr = c->inner->realloc(ptr, old_size, new_size, c->inner->ctx);
    if (new_ptr) {
        int64_t diff = (int64_t)new_size - (int64_t)old_size;
        c->stats.current_bytes += diff;
        if (diff > 0) c->stats.bytes_allocated += diff;
        else          c->stats.bytes_freed     -= diff;
        if (c->stats.current_bytes > c->stats.peak_bytes) {
            c->stats.peak_bytes = c->stats.current_bytes;
        }
    }
    return new_ptr;
}

static void counting_free(void *ptr, size_t size, void *ctx) {
    ivp_counting_ctx_t *c = (ivp_counting_ctx_t *)ctx;
    c->inner->free(ptr, size, c->inner->ctx);
    c->stats.free_count++;
    c->stats.bytes_freed   += (int64_t)size;
    c->stats.current_bytes -= (int64_t)size;
}

ivp_allocator_t ivp_counting_allocator_create(const ivp_allocator_t *inner) {
    if (!inner) inner = ivp_default_allocator();

    /* Allocate the counting context using the inner allocator itself */
    ivp_counting_ctx_t *c = (ivp_counting_ctx_t *)inner->alloc(
        sizeof(ivp_counting_ctx_t), inner->ctx);
    if (!c) {
        /* Fallback to passthrough allocator if context allocation fails. */
        ivp_allocator_t passthrough = {
            .alloc   = inner->alloc,
            .realloc = inner->realloc,
            .free    = inner->free,
            .ctx     = inner->ctx
        };
        return passthrough;
    }
    memset(&c->stats, 0, sizeof(c->stats));
    c->inner = inner;

    ivp_allocator_t a = {
        .alloc   = counting_alloc,
        .realloc = counting_realloc,
        .free    = counting_free,
        .ctx     = c
    };
    return a;
}

void ivp_counting_allocator_destroy(ivp_allocator_t *a) {
    if (!a || !a->ctx) return;
    if (a->alloc != counting_alloc ||
        a->realloc != counting_realloc ||
        a->free != counting_free) {
        return;
    }

    ivp_counting_ctx_t *c = (ivp_counting_ctx_t *)a->ctx;
    const ivp_allocator_t *inner = c->inner ? c->inner : ivp_default_allocator();
    inner->free(c, sizeof(*c), inner->ctx);

    a->alloc   = inner->alloc;
    a->realloc = inner->realloc;
    a->free    = inner->free;
    a->ctx     = inner->ctx;
}

void ivp_counting_allocator_get_stats(const ivp_allocator_t *a,
                                       ivp_alloc_stats_t *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!a || !a->ctx) return;
    if (a->alloc != counting_alloc ||
        a->realloc != counting_realloc ||
        a->free != counting_free) {
        return;
    }
    ivp_counting_ctx_t *c = (ivp_counting_ctx_t *)a->ctx;
    *out = c->stats;
}
