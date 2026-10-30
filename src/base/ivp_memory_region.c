/* ivp_memory_region.c -- memory regions with in-place checkpoints.
 *
 * The region reserves its address range once and commits pages as the
 * allocator moves its top up.  The allocator's bookkeeping (the top and
 * the free lists, whose links lie in the free blocks) sits at the start of
 * the region, so copying the bytes in use captures it with the blocks, and
 * writing them back restores both.
 *
 * Blocks up to REGION_SMALL_MAX bytes come from free lists per 16-byte
 * size class; larger blocks are rounded to REGION_LARGE_GRANULE and reused
 * only at exactly their size.  Freed memory is never given back to the
 * top, so the top only grows between restores. */
#include <ivp/base/ivp_memory_region.h>

#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#if defined(_WIN32)
#include <windows.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif

#if defined(__has_feature)
#if __has_feature(address_sanitizer)
#define IVP_REGION_ASAN 1
#endif
#endif
#if !defined(IVP_REGION_ASAN) && defined(__SANITIZE_ADDRESS__)
#define IVP_REGION_ASAN 1
#endif
#ifdef IVP_REGION_ASAN
#include <sanitizer/asan_interface.h>
#endif

#define REGION_ALIGN 16u
#define REGION_SMALL_MAX 4096u
#define REGION_SMALL_CLASSES (REGION_SMALL_MAX / REGION_ALIGN)
#define REGION_LARGE_GRANULE 4096u
#define REGION_COMMIT_STEP ((size_t)1 << 20)
#define REGION_DEFAULT_CAPACITY ((size_t)1 << 30)

/* a free block larger than REGION_SMALL_MAX */
typedef struct region_large {
    size_t size;
    struct region_large *next;
} region_large_t;

/* bookkeeping at the start of the region */
typedef struct region_state {
    size_t top;   /* offset of the first byte never handed out */
    void *small[REGION_SMALL_CLASSES + 1];   /* index: size / REGION_ALIGN */
    region_large_t *large;
} region_state_t;

struct ivp_memory_region {
    ivp_allocator_t allocator;   /* ctx is the region */
    uint64_t id;                 /* unique in the process */
    unsigned char *base;
    size_t capacity;
    size_t committed;
    size_t page;
    atomic_flag lock;
    atomic_size_t failures;      /* outside the region: not restored */
};

struct ivp_memory_checkpoint {
    const ivp_memory_region_t *region;
    uint64_t region_id;          /* a later region may reuse the address */
    ivp_allocator_t storage;
    size_t size;
    unsigned char *bytes;
};

static atomic_uint_least64_t region_next_id = 1;

static size_t region_round(size_t value, size_t granule)
{
    return (value + granule - 1) / granule * granule;
}

static region_state_t *region_state(const ivp_memory_region_t *region)
{
    return (region_state_t *)region->base;
}

/* ── copying ─────────────────────────────────────────────────────────── */

/* A checkpoint copies the region whole, including bytes AddressSanitizer
 * marks as poisoned (the arena poisons the slack of its blocks), so the
 * copy must not be instrumented; volatile keeps the compiler from turning
 * it into an intercepted memcpy. */
#ifdef IVP_REGION_ASAN
__attribute__((no_sanitize("address")))
static void region_copy_out(unsigned char *dst, const unsigned char *src,
                            size_t size)
{
    const volatile unsigned char *from = src;
    for (size_t i = 0; i < size; i++) dst[i] = from[i];
}
#else
static void region_copy_out(unsigned char *dst, const unsigned char *src,
                            size_t size)
{
    memcpy(dst, src, size);
}
#endif

/* ── address space ───────────────────────────────────────────────────── */

static size_t region_page_size(void)
{
#if defined(_WIN32)
    SYSTEM_INFO info;
    GetSystemInfo(&info);
    return (size_t)info.dwPageSize;
#else
    long page = sysconf(_SC_PAGESIZE);
    return page > 0 ? (size_t)page : 4096u;
#endif
}

static unsigned char *region_reserve(size_t capacity)
{
#if defined(_WIN32)
    return (unsigned char *)VirtualAlloc(NULL, capacity, MEM_RESERVE,
                                         PAGE_NOACCESS);
#else
    int flags = MAP_PRIVATE | MAP_ANON;
#ifdef MAP_NORESERVE
    flags |= MAP_NORESERVE;
#endif
    void *base = mmap(NULL, capacity, PROT_NONE, flags, -1, 0);
    return base == MAP_FAILED ? NULL : (unsigned char *)base;
#endif
}

static bool region_commit_pages(unsigned char *start, size_t length)
{
#if defined(_WIN32)
    return VirtualAlloc(start, length, MEM_COMMIT, PAGE_READWRITE) != NULL;
#else
    return mprotect(start, length, PROT_READ | PROT_WRITE) == 0;
#endif
}

static void region_release(unsigned char *base, size_t capacity)
{
#if defined(_WIN32)
    (void)capacity;
    VirtualFree(base, 0, MEM_RELEASE);
#else
    munmap(base, capacity);
#endif
}

/* make [0, end) usable */
static bool region_commit(ivp_memory_region_t *region, size_t end)
{
    if (end <= region->committed) return true;
    if (end > region->capacity) return false;
    size_t target = region->committed + REGION_COMMIT_STEP;
    if (target < end) target = end;
    target = region_round(target, region->page);
    if (target > region->capacity) target = region->capacity;
    if (!region_commit_pages(region->base + region->committed,
                             target - region->committed)) {
        return false;
    }
    region->committed = target;
    return true;
}

/* ── allocator ───────────────────────────────────────────────────────── */

static void region_lock(ivp_memory_region_t *region)
{
    while (atomic_flag_test_and_set_explicit(&region->lock,
                                             memory_order_acquire)) {
    }
}

static void region_unlock(ivp_memory_region_t *region)
{
    atomic_flag_clear_explicit(&region->lock, memory_order_release);
}

/* the size a request occupies */
static size_t region_block_size(size_t size)
{
    if (size == 0) size = 1;
    return size <= REGION_SMALL_MAX ? region_round(size, REGION_ALIGN)
                                    : region_round(size, REGION_LARGE_GRANULE);
}

static void *region_take(ivp_memory_region_t *region, size_t block)
{
    region_state_t *state = region_state(region);
    if (block <= REGION_SMALL_MAX) {
        void **head = &state->small[block / REGION_ALIGN];
        if (*head) {
            void *found = *head;
            *head = *(void **)found;
            return found;
        }
    } else {
        for (region_large_t **link = &state->large; *link;
             link = &(*link)->next) {
            if ((*link)->size == block) {
                region_large_t *found = *link;
                *link = found->next;
                return found;
            }
        }
    }
    if (block > region->capacity - state->top ||
        !region_commit(region, state->top + block)) {
        return NULL;
    }
    void *fresh = region->base + state->top;
    state->top += block;
    return fresh;
}

static void region_give(ivp_memory_region_t *region, void *ptr, size_t block)
{
    region_state_t *state = region_state(region);
    if (block <= REGION_SMALL_MAX) {
        void **head = &state->small[block / REGION_ALIGN];
        *(void **)ptr = *head;
        *head = ptr;
    } else {
        region_large_t *free_block = (region_large_t *)ptr;
        free_block->size = block;
        free_block->next = state->large;
        state->large = free_block;
    }
}

static void *region_refuse(ivp_memory_region_t *region)
{
    atomic_fetch_add_explicit(&region->failures, 1, memory_order_relaxed);
    return NULL;
}

static void *region_alloc(size_t size, void *ctx)
{
    ivp_memory_region_t *region = (ivp_memory_region_t *)ctx;
    /* larger requests cannot fit, and rounding them could wrap */
    if (size > region->capacity) return region_refuse(region);
    size_t block = region_block_size(size);
    region_lock(region);
    void *ptr = region_take(region, block);
    region_unlock(region);
    if (!ptr) return region_refuse(region);
#ifdef IVP_REGION_ASAN
    ASAN_UNPOISON_MEMORY_REGION(ptr, block);
#endif
    memset(ptr, 0, block);
    return ptr;
}

static void region_free(void *ptr, size_t size, void *ctx)
{
    if (!ptr) return;
    ivp_memory_region_t *region = (ivp_memory_region_t *)ctx;
    region_lock(region);
    region_give(region, ptr, region_block_size(size));
    region_unlock(region);
}

static void *region_realloc(void *ptr, size_t old_size, size_t new_size,
                            void *ctx)
{
    if (!ptr) return region_alloc(new_size, ctx);
    ivp_memory_region_t *region = (ivp_memory_region_t *)ctx;
    if (new_size > region->capacity) return region_refuse(region);
    if (region_block_size(old_size) == region_block_size(new_size)) {
        if (new_size > old_size) {
            memset((unsigned char *)ptr + old_size, 0, new_size - old_size);
        }
        return ptr;
    }
    void *grown = region_alloc(new_size, ctx);
    if (!grown) return NULL;
    memcpy(grown, ptr, old_size < new_size ? old_size : new_size);
    region_free(ptr, old_size, ctx);
    return grown;
}

/* ── region ──────────────────────────────────────────────────────────── */

ivp_memory_region_t *ivp_memory_region_create(size_t capacity,
                                              ivp_result_t *result)
{
    const ivp_allocator_t *host = ivp_default_allocator();
    ivp_memory_region_t *region =
        (ivp_memory_region_t *)ivp_alloc(host, sizeof(*region));
    if (!region) {
        ivp_result_set(result, IVP_ERR_OUT_OF_MEMORY);
        return NULL;
    }
    memset(region, 0, sizeof(*region));
    region->id = atomic_fetch_add_explicit(&region_next_id, 1, memory_order_relaxed);
    atomic_init(&region->failures, 0);
    region->page = region_page_size();
    region->capacity = region_round(
        capacity ? capacity : REGION_DEFAULT_CAPACITY, region->page);
    region->base = region_reserve(region->capacity);
    atomic_flag_clear(&region->lock);
    size_t header = region_round(sizeof(region_state_t), REGION_ALIGN);
    if (!region->base || header > region->capacity ||
        !region_commit(region, header)) {
        if (region->base) region_release(region->base, region->capacity);
        ivp_free(host, region, sizeof(*region));
        ivp_result_set(result, IVP_ERR_OUT_OF_MEMORY);
        return NULL;
    }
    memset(region->base, 0, header);
    region_state(region)->top = header;
    region->allocator.alloc = region_alloc;
    region->allocator.realloc = region_realloc;
    region->allocator.free = region_free;
    region->allocator.ctx = region;
    ivp_result_set(result, IVP_OK);
    return region;
}

void ivp_memory_region_destroy(ivp_memory_region_t *region)
{
    if (!region) return;
    region_release(region->base, region->capacity);
    ivp_free(ivp_default_allocator(), region, sizeof(*region));
}

const ivp_allocator_t *ivp_memory_region_allocator(ivp_memory_region_t *region)
{
    return region ? &region->allocator : NULL;
}

bool ivp_memory_region_is_allocator(const ivp_allocator_t *allocator)
{
    return allocator && allocator->alloc == region_alloc;
}

size_t ivp_memory_region_used(const ivp_memory_region_t *region)
{
    return region ? region_state(region)->top : 0u;
}

size_t ivp_memory_region_failures(const ivp_memory_region_t *region)
{
    return region ? atomic_load_explicit(&region->failures, memory_order_relaxed)
                  : 0u;
}

/* ── checkpoints ─────────────────────────────────────────────────────── */

ivp_memory_checkpoint_t *ivp_memory_region_checkpoint(
    ivp_memory_region_t *region, const ivp_allocator_t *storage,
    ivp_result_t *result)
{
    if (!storage) storage = ivp_default_allocator();
    /* a copy kept in the region itself would be overwritten by a restore */
    if (!region || storage == &region->allocator) {
        ivp_result_set(result, IVP_ERR_INVALID_ARG);
        return NULL;
    }
    ivp_memory_checkpoint_t *checkpoint =
        (ivp_memory_checkpoint_t *)ivp_alloc(storage, sizeof(*checkpoint));
    if (!checkpoint) {
        ivp_result_set(result, IVP_ERR_OUT_OF_MEMORY);
        return NULL;
    }
    checkpoint->region = region;
    checkpoint->region_id = region->id;
    checkpoint->storage = *storage;
    checkpoint->size = region_state(region)->top;
    checkpoint->bytes = (unsigned char *)ivp_alloc(storage, checkpoint->size);
    if (!checkpoint->bytes) {
        ivp_free(storage, checkpoint, sizeof(*checkpoint));
        ivp_result_set(result, IVP_ERR_OUT_OF_MEMORY);
        return NULL;
    }
    region_copy_out(checkpoint->bytes, region->base, checkpoint->size);
    ivp_result_set(result, IVP_OK);
    return checkpoint;
}

ivp_result_t ivp_memory_region_restore(ivp_memory_region_t *region,
                                       const ivp_memory_checkpoint_t *checkpoint)
{
    if (!region || !checkpoint || checkpoint->region != region ||
        checkpoint->region_id != region->id) {
        return IVP_ERR_INVALID_ARG;
    }
#ifdef IVP_REGION_ASAN
    /* poisoning lives outside the region and does not go back with it: the
     * bytes in use may hold different blocks now, so clear it */
    size_t top = region_state(region)->top;
    ASAN_UNPOISON_MEMORY_REGION(region->base,
                                top > checkpoint->size ? top : checkpoint->size);
#endif
    /* the region was at least this large when the checkpoint was taken,
     * and committed pages are never given back */
    memcpy(region->base, checkpoint->bytes, checkpoint->size);
    return IVP_OK;
}

size_t ivp_memory_checkpoint_size(const ivp_memory_checkpoint_t *checkpoint)
{
    return checkpoint ? checkpoint->size : 0u;
}

void ivp_memory_checkpoint_destroy(ivp_memory_checkpoint_t *checkpoint)
{
    if (!checkpoint) return;
    ivp_allocator_t storage = checkpoint->storage;
    ivp_free(&storage, checkpoint->bytes, checkpoint->size);
    ivp_free(&storage, checkpoint, sizeof(*checkpoint));
}
