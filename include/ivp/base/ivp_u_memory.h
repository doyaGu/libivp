/* ivp_u_memory.h — port of the reference IVP_U_Memory transaction arena
 *
 * Reference: ivp_utility/ivu_memory.{hxx,cxx} (non-MEMTEST build, no
 * external scratchpad), 64-bit layout (PLATFORM_64BITS).
 *
 * The reference environment owns one such arena (IVP_Environment::
 * short_term_mem) that the friction solver and its linear constraint solver
 * take all their scratch arrays from.  The reference policy never clears the
 * arena: a new transaction hands out the same bytes again, and the reference
 * solvers read some of them before writing them (row padding, IVP_VecFPU
 * block overruns).  The mainline policy clears each complete reserved span.
 * This module supports both policies while preserving the reference layout:
 *
 *  - get_mem(size): the next allocation starts at
 *        align(previous_start + size),  align(p) = (p + 0x4f) & ~0x1f
 *    (IVU_MEM_ALIGN 0x50 with a 32-byte IVU_MEM_MASK), i.e. every allocation
 *    is 32-byte aligned and followed by a gap of 48..79 bytes.  Offsets are
 *    computed relative to the 32-byte aligned start of the block, so the
 *    layout does not depend on where the host allocator puts the block.
 *  - Blocks: the first block has room for IVU_MEMORY_BLOCK_SIZE -
 *    sizeof(p_Memory_Elem) = 0x7fe0 - 16 bytes; right after creation the
 *    pointer starts 64 bytes in (neuer_sp_block(0) rounds 0 up to 64).  The
 *    first end_transaction resets it to the block start and from then on
 *    the usable size is IVU_MEMORY_BLOCK_SIZE (0x7fe0) bytes.  A request that
 *    does not fit opens a new block of max(0x7fe0 - 16, rounded size) bytes
 *    (the rest of the current block is abandoned); the request is placed at
 *    its start.
 *  - start_transaction does not move the pointer; end_transaction frees
 *    every block except the first and moves the pointer back to the start of
 *    the first block.  Allocations made outside a transaction simply advance
 *    the pointer (the reference friction easing does that).
 *  - A standalone arena defaults to reference-style reuse; an environment
 *    selects its build-wide state policy after initialization.  The first
 *    block starts zero-filled: in the reference it is a fresh 32816-byte
 *    malloc that
 *    lands on fresh (zero)
 *    pages on macOS/arm64 (measured).  Blocks opened on overflow are freed at
 *    the end of the transaction; the reference allocator hands the same slot
 *    back on the next overflow with its old contents (measured: LIFO reuse,
 *    no zeroing), which is emulated by recycling freed blocks of the same
 *    size (IVP_U_MEMORY_RECYCLE_BLOCKS).  The first use of a slot in the
 *    reference holds whatever an unrelated earlier allocation left there,
 *    which cannot be reproduced; libivp uses zero-filled memory instead.
 *
 * Every allocation is followed by at least 48 bytes that still belong to the
 * same block, so the reference's IVP_VecFPU overruns (at most 32 bytes past
 * the end of an array) never leave the block.  `clear_on_allocate` is the
 * mainline history-independent policy: it clears both the requested bytes
 * and this reserved slack on every allocation. */
#ifndef IVP_U_MEMORY_H
#define IVP_U_MEMORY_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Constants of the 64-bit reference build (ivu_memory.hxx). */
#define IVP_U_MEM_ALIGN       0x50u   /* IVU_MEM_ALIGN                        */
#define IVP_U_MEM_MASK        0x1fu   /* IVU_MEM_MASK = ~0x1f                 */
#define IVP_U_MEM_BLOCK_SIZE  0x7fe0u /* IVU_MEMORY_BLOCK_SIZE                */
#define IVP_U_MEM_ELEM_SIZE   16u     /* sizeof(p_Memory_Elem) on LP64        */
#define IVP_U_MEM_ELEM_DATA   8u      /* offsetof(p_Memory_Elem, data) on LP64 */

#ifndef IVP_U_MEMORY_RECYCLE_BLOCKS
#define IVP_U_MEMORY_RECYCLE_BLOCKS 4 /* freed overflow blocks kept for reuse (0: none) */
#endif

typedef struct ivp_u_memory {
    const ivp_allocator_t *alloc;
    char   *first_elem;          /* block never freed by end_transaction       */
    char   *last_elem;           /* newest block (blocks are linked newest->oldest) */
    char   *block_start;         /* 32-byte aligned data start of last_elem    */
    size_t  pos;                 /* speicherbeginn - block_start               */
    size_t  end;                 /* speicherende - block_start                 */
    size_t  first_block_size;    /* usable bytes retained across transactions  */
    int     transaction_in_use;
    bool    clear_on_allocate;   /* zero requested bytes and alignment slack   */
#if IVP_U_MEMORY_RECYCLE_BLOCKS > 0
    char   *recycled[IVP_U_MEMORY_RECYCLE_BLOCKS]; /* freed blocks, LIFO      */
    int     n_recycled;
#endif
} ivp_u_memory_t;

/* IVP_U_Memory() + init_mem_transaction_usage(): allocates the first block.
 * alloc may be NULL (default allocator). */
ivp_result_t ivp_u_memory_init(ivp_u_memory_t *mem, const ivp_allocator_t *alloc);

/* As above, but retain a smaller first block for scratch arenas that do not
 * depend on the reference short-term-memory layout.  Overflow blocks keep
 * the normal IVP_U_MEM_BLOCK_SIZE capacity. */
ivp_result_t ivp_u_memory_init_first_block(ivp_u_memory_t *mem,
                                            const ivp_allocator_t *alloc,
                                            size_t first_block_size);

/* free_mem(): releases every block. */
void ivp_u_memory_destroy(ivp_u_memory_t *mem);

/* get_mem(size): 32-byte aligned and uninitialised unless clear-on-allocate
 * is enabled; NULL only if the host allocator fails. */
void *ivp_u_memory_get_mem(ivp_u_memory_t *mem, size_t size);

/* Make future allocations independent of prior transaction contents.  The
 * complete reserved span is cleared, including the 48..79 byte alignment
 * gap that reference VecFPU operations may access.  Disabled by default to
 * retain strict reference behaviour. */
void ivp_u_memory_set_clear_on_allocate(ivp_u_memory_t *mem, bool enable);

/* start_memory_transaction / end_memory_transaction. */
void ivp_u_memory_start_transaction(ivp_u_memory_t *mem);
void ivp_u_memory_end_transaction(ivp_u_memory_t *mem);

#ifdef __cplusplus
}
#endif

#endif /* IVP_U_MEMORY_H */
