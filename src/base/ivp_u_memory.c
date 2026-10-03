/* ivp_u_memory.c — port of the reference IVP_U_Memory (ivu_memory.cxx)
 *
 * See ivp_u_memory.h for the semantics.  Block layout (host side):
 *
 *   raw:  [ header | ... | data (32-byte aligned) ......... | slack ]
 *
 * The header (next link, allocation size) lives in the bytes in front of the
 * aligned data start, like p_Memory_Elem::next in the reference; the data
 * start is align(raw + offsetof(p_Memory_Elem, data)), which is always at
 * least 56 bytes into the block.  All bookkeeping is done with offsets from
 * the data start, so no pointer ever points outside its block. */
#include <ivp/base/ivp_u_memory.h>
#include <ivp/base/ivp_assert.h>
#include <stdint.h>
#include <string.h>

/* Under AddressSanitizer everything outside [data start, data start + usable
 * size) of a block is poisoned, so a reference-faithful overrun that left
 * the arena area would be reported. */
#if defined(__has_feature)
#if __has_feature(address_sanitizer)
#define IVP_U_MEM_ASAN 1
#endif
#endif
#if !defined(IVP_U_MEM_ASAN) && defined(__SANITIZE_ADDRESS__)
#define IVP_U_MEM_ASAN 1
#endif
#ifdef IVP_U_MEM_ASAN
#include <sanitizer/asan_interface.h>
#endif

typedef struct u_mem_block_hdr {
    char  *next;     /* p_Memory_Elem::next */
    size_t total;    /* bytes allocated for this block */
} u_mem_block_hdr_t;

/* bytes in front of the first usable byte that the block reserves on top of
 * its data size: the reference mallocs sizeof(p_Memory_Elem) + ng +
 * IVU_MEM_ALIGN; 32 more bytes cover hosts whose malloc is only 8-byte
 * aligned and the 16 bytes the first block grows by after its first reset. */
#define U_MEM_BLOCK_EXTRA ((size_t)IVP_U_MEM_ELEM_SIZE + (size_t)IVP_U_MEM_ALIGN + 32u)

static inline size_t u_mem_round(size_t n)
{
    /* align_to_next_adress on an offset from a 32-byte aligned start */
    return (n + ((size_t)IVP_U_MEM_ALIGN - 1u)) & ~(size_t)IVP_U_MEM_MASK;
}

static inline u_mem_block_hdr_t *u_mem_hdr(char *raw)
{
    return (u_mem_block_hdr_t *)(void *)raw;
}

static inline char *u_mem_data_start(char *raw)
{
    uintptr_t a = (uintptr_t)raw + (uintptr_t)IVP_U_MEM_ELEM_DATA;
    a = (a + ((uintptr_t)IVP_U_MEM_ALIGN - 1u)) & ~(uintptr_t)IVP_U_MEM_MASK;
    return raw + (a - (uintptr_t)raw);
}

/* usable area of a block: [data start, data start + usable) */
static inline void u_mem_guard(char *raw, size_t usable)
{
#ifdef IVP_U_MEM_ASAN
    size_t total = u_mem_hdr(raw)->total;
    char *data = u_mem_data_start(raw);
    char *hdr_end = raw + sizeof(u_mem_block_hdr_t);
    ASAN_UNPOISON_MEMORY_REGION(raw, total);
    ASAN_POISON_MEMORY_REGION(hdr_end, (size_t)(data - hdr_end));
    ASAN_POISON_MEMORY_REGION(data + usable, total - (size_t)(data - raw) - usable);
#else
    (void)raw;
    (void)usable;
#endif
}

static inline void u_mem_unguard(char *raw)
{
#ifdef IVP_U_MEM_ASAN
    ASAN_UNPOISON_MEMORY_REGION(raw, u_mem_hdr(raw)->total);
#else
    (void)raw;
#endif
}

static void u_mem_release_block(ivp_u_memory_t *mem, char *raw)
{
#if IVP_U_MEMORY_RECYCLE_BLOCKS > 0
    if (mem->n_recycled == IVP_U_MEMORY_RECYCLE_BLOCKS) {
        char *oldest = mem->recycled[0];
        u_mem_unguard(oldest);
        ivp_free(mem->alloc, oldest, u_mem_hdr(oldest)->total);
        for (int i = 1; i < mem->n_recycled; i++) mem->recycled[i - 1] = mem->recycled[i];
        mem->n_recycled--;
    }
    mem->recycled[mem->n_recycled++] = raw;
#else
    u_mem_unguard(raw);
    ivp_free(mem->alloc, raw, u_mem_hdr(raw)->total);
#endif
}

static char *u_mem_acquire_block(ivp_u_memory_t *mem, size_t total)
{
#if IVP_U_MEMORY_RECYCLE_BLOCKS > 0
    for (int i = mem->n_recycled - 1; i >= 0; i--) {
        char *raw = mem->recycled[i];
        if (u_mem_hdr(raw)->total == total) {
            for (int k = i + 1; k < mem->n_recycled; k++) mem->recycled[k - 1] = mem->recycled[k];
            mem->n_recycled--;
            return raw; /* old contents kept, like the reference allocator */
        }
    }
#endif
    char *raw = (char *)ivp_alloc(mem->alloc, total);
    if (!raw) return NULL;
    memset(raw, 0, total);
    u_mem_hdr(raw)->total = total;
    return raw;
}

/* IVP_U_Memory::neuer_sp_block */
static void *u_mem_new_block(ivp_u_memory_t *mem, size_t groesse)
{
    size_t ng = (size_t)IVP_U_MEM_BLOCK_SIZE - (size_t)IVP_U_MEM_ELEM_SIZE;
    if (groesse > SIZE_MAX / 2) return NULL;
    groesse = u_mem_round(groesse);
    if (groesse > ng) ng = groesse;
    if (ng > SIZE_MAX - U_MEM_BLOCK_EXTRA) return NULL;
    char *raw = u_mem_acquire_block(mem, ng + U_MEM_BLOCK_EXTRA);
    if (!raw) return NULL;

    u_mem_hdr(raw)->next = mem->last_elem;
    mem->last_elem = raw;
    mem->block_start = u_mem_data_start(raw);
    mem->pos = groesse;
    mem->end = ng;
    u_mem_guard(raw, ng);
    return mem->block_start;
}

ivp_result_t ivp_u_memory_init(ivp_u_memory_t *mem, const ivp_allocator_t *alloc)
{
    if (!mem) return IVP_ERR_INVALID_ARG;
    memset(mem, 0, sizeof(*mem));
    mem->alloc = alloc ? alloc : ivp_default_allocator();
    /* init_mem_transaction_usage(): transaction_in_use = 0; neuer_sp_block(0) */
    mem->transaction_in_use = 0;
    if (!u_mem_new_block(mem, 0)) return IVP_ERR_OUT_OF_MEMORY;
    mem->first_elem = mem->last_elem;
    return IVP_OK;
}

void ivp_u_memory_destroy(ivp_u_memory_t *mem)
{
    if (!mem || !mem->alloc) return;
    for (char *f = mem->last_elem; f;) {
        char *n = u_mem_hdr(f)->next;
        u_mem_unguard(f);
        ivp_free(mem->alloc, f, u_mem_hdr(f)->total);
        f = n;
    }
#if IVP_U_MEMORY_RECYCLE_BLOCKS > 0
    for (int i = 0; i < mem->n_recycled; i++) {
        u_mem_unguard(mem->recycled[i]);
        ivp_free(mem->alloc, mem->recycled[i], u_mem_hdr(mem->recycled[i])->total);
    }
    mem->n_recycled = 0;
#endif
    mem->first_elem = NULL;
    mem->last_elem = NULL;
    mem->block_start = NULL;
    mem->pos = 0;
    mem->end = 0;
}

void *ivp_u_memory_get_mem(ivp_u_memory_t *mem, size_t size)
{
    size_t op = mem->pos;
    if (size > SIZE_MAX - op - (size_t)IVP_U_MEM_ALIGN) return NULL;
    size_t p = u_mem_round(op + size);
    if (p >= mem->end) {
        return u_mem_new_block(mem, size);
    }
    mem->pos = p;
    char *op_ptr = mem->block_start + op;
    IVP_IF(((intptr_t)op_ptr > 0x780000) && ((intptr_t)op_ptr < 0x792f48)) /* ivu_memory.hxx:109, no-op */
    {
        op_ptr++;
        op_ptr--;
    }
    return op_ptr;
}

void ivp_u_memory_start_transaction(ivp_u_memory_t *mem)
{
    mem->transaction_in_use++;
}

void ivp_u_memory_end_transaction(ivp_u_memory_t *mem)
{
    mem->transaction_in_use--;
    /* free_mem_transaction(): free all blocks except the first one */
    for (char *f = mem->last_elem; f;) {
        char *n = u_mem_hdr(f)->next;
        if (f == mem->first_elem) break;
        u_mem_release_block(mem, f);
        f = n;
    }
    mem->block_start = u_mem_data_start(mem->first_elem);
    mem->pos = 0;
    mem->end = (size_t)IVP_U_MEM_BLOCK_SIZE;
    mem->last_elem = mem->first_elem;
    u_mem_guard(mem->first_elem, mem->end);
}
