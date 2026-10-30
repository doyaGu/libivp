/* ivp_memory_region.h — Memory regions with in-place checkpoints
 * (libivp extension)
 *
 * A region reserves one range of address space and serves an allocator
 * whose blocks all lie inside it; the allocator's own bookkeeping lies
 * inside it too.  A checkpoint copies every byte in use, and restoring
 * writes them back in place: blocks allocated since the checkpoint are free
 * again, blocks freed since are allocated again with the bytes they had,
 * and every pointer into the region stays valid.
 *
 * A world built entirely from a region's allocator (its environment, and
 * the controller states, sets and active values the caller gives it) can be
 * put back to any checkpoint this way, whatever it contains; ivp_rewind.h
 * builds on that.  Memory outside the region is not touched: geometry,
 * which does not change once built, may live anywhere.
 *
 * Every block comes zero-filled, so what an allocation holds never depends
 * on the history of the region before it.
 *
 * The allocator is safe to call from several threads; checkpoint and
 * restore must not run while anything else uses the region.
 *
 * Built only with IVP_ENABLE_CHECKPOINTS (CMake option, default off).
 */
#ifndef IVP_MEMORY_REGION_H
#define IVP_MEMORY_REGION_H

/* the library defines IVP_ENABLE_CHECKPOINTS for its users (CMake option) */
#if !IVP_ENABLE_CHECKPOINTS
#error "ivp_memory_region.h needs libivp built with IVP_ENABLE_CHECKPOINTS=ON"
#endif

#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_memory_region ivp_memory_region_t;
typedef struct ivp_memory_checkpoint ivp_memory_checkpoint_t;

/* Reserve `capacity` bytes of address space (0: 1 GiB); memory is committed
 * as the allocator needs it.  Returns NULL and sets *result (if non-NULL)
 * on failure: IVP_ERR_OUT_OF_MEMORY when the address space cannot be
 * reserved. */
ivp_memory_region_t *ivp_memory_region_create(size_t capacity,
                                              ivp_result_t *result);

/* Release the region and everything allocated from it. */
void ivp_memory_region_destroy(ivp_memory_region_t *region);

/* The region's allocator, valid as long as the region.  Every block is
 * 16-byte aligned; allocation fails once the region is full. */
const ivp_allocator_t *ivp_memory_region_allocator(ivp_memory_region_t *region);

/* Whether allocator is the allocator of a memory region. */
bool ivp_memory_region_is_allocator(const ivp_allocator_t *allocator);

/* Bytes a checkpoint copies: the extent of the region in use. */
size_t ivp_memory_region_used(const ivp_memory_region_t *region);

/* Allocations the region has refused since it was created.  The count
 * lives outside the region: a restore does not take it back. */
size_t ivp_memory_region_failures(const ivp_memory_region_t *region);

/* Copy the region in use, outside the region (storage NULL: the default
 * allocator).  Returns NULL and sets *result (if non-NULL) on failure:
 * IVP_ERR_INVALID_ARG without a region or when storage is the region's own
 * allocator, whose copy a restore would overwrite; IVP_ERR_OUT_OF_MEMORY
 * when storage cannot hold the copy. */
ivp_memory_checkpoint_t *ivp_memory_region_checkpoint(
    ivp_memory_region_t *region, const ivp_allocator_t *storage,
    ivp_result_t *result);

/* Put the region back to checkpoint, which must have been taken of this
 * region (not of an earlier one at the same address); IVP_ERR_INVALID_ARG
 * otherwise. */
ivp_result_t ivp_memory_region_restore(ivp_memory_region_t *region,
                                       const ivp_memory_checkpoint_t *checkpoint);

/* Bytes the checkpoint holds. */
size_t ivp_memory_checkpoint_size(const ivp_memory_checkpoint_t *checkpoint);

void ivp_memory_checkpoint_destroy(ivp_memory_checkpoint_t *checkpoint);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MEMORY_REGION_H */
