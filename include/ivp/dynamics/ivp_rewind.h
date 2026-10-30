/* ivp_rewind.h — Rewinding a world to any recorded PSI (libivp extension)
 *
 * A rewinder records a world PSI by PSI: before every PSI it applies the
 * caller's input for that PSI through a callback and logs a copy of the
 * input; every keyframe_interval PSIs it keeps a checkpoint of the world's
 * memory region (ivp_memory_region.h) in a ring of keyframe_capacity
 * keyframes.  Seeking to an earlier PSI restores the nearest keyframe at or
 * before it in place and replays the logged inputs; seeking forward replays
 * from the current world.  The world after seeking to PSI n is the one the
 * recording had after n PSIs, bit for bit.
 *
 * Stepping after seeking back branches: the inputs and keyframes after the
 * current PSI are discarded and new ones are recorded.  When the ring drops
 * its oldest keyframe, the inputs before the next keyframe go with it, so
 * the oldest reachable PSI moves on.
 *
 * The world must live entirely in the region: the environment is created
 * with the region's allocator, and so must be every controller state, set
 * or active value the caller hands to the world, or a restore puts the
 * world's memory back but not theirs.  Everything else the region does not
 * hold stays as it is.  A restore keeps the environment and every object
 * and controller at its address: pointers stay valid for whatever exists at
 * the PSI sought, and things created after it are gone.
 *
 * The world may only change through ivp_rewind_step and the input callback,
 * and the callback must depend only on the PSI, its input and memory in the
 * region; otherwise a replay cannot reproduce the recording.  The rewinder
 * keeps its keyframes and log outside the region (config.storage).
 *
 * Built only with IVP_ENABLE_CHECKPOINTS (CMake option, default off).
 */
#ifndef IVP_REWIND_H
#define IVP_REWIND_H

#include <ivp/base/ivp_memory_region.h>
#include <ivp/base/ivp_result.h>
#include <ivp/dynamics/ivp_environment.h>

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_rewind ivp_rewind_t;

/* Apply the input recorded for the PSI that follows `psi` PSIs; called
 * while recording and again for every replayed PSI.  input is NULL when
 * size is 0. */
typedef void (*ivp_rewind_apply_fn_t)(void *ctx, ivp_environment_t *env,
                                      uint64_t psi, const void *input,
                                      size_t size);

typedef struct ivp_rewind_config {
    int keyframe_interval;   /* PSIs between keyframes, at least 1 */
    int keyframe_capacity;   /* keyframes kept, at least 1 */
    ivp_rewind_apply_fn_t apply;   /* may be NULL */
    void *apply_ctx;
    /* keyframes and the input log, outside the region; NULL: the default
     * allocator */
    const ivp_allocator_t *storage;
} ivp_rewind_config_t;

/* Record env, which was created with the allocator of region and is not
 * inside a simulate call, starting with the keyframe of PSI 0.  env and
 * region stay the caller's and must outlive the rewinder.  Returns NULL and
 * sets *result on failure: IVP_ERR_INVALID_ARG when env does not live in
 * region or config.storage is the region's own allocator. */
ivp_rewind_t *ivp_rewind_create(ivp_environment_t *env,
                                ivp_memory_region_t *region,
                                const ivp_rewind_config_t *config,
                                ivp_result_t *result);

/* Destroy the rewinder; the world stays as it is. */
void ivp_rewind_destroy(ivp_rewind_t *rewind);

/* Apply input (copied into the log) and simulate one PSI, keeping a
 * keyframe first when the PSI count is a multiple of the interval.  After a
 * seek back this discards the recorded future.  When the keyframe or the
 * input cannot be stored nothing changes.  IVP_ERR_OUT_OF_MEMORY after the
 * PSI ran means the region refused an allocation during it: the world holds
 * what the engine made of that, and the PSI is recorded (a replay meets the
 * same refusal). */
ivp_result_t ivp_rewind_step(ivp_rewind_t *rewind, const void *input,
                             size_t size);

/* Move to the world after `psi` PSIs, which must lie between
 * ivp_rewind_oldest_psi and ivp_rewind_recorded_psi; IVP_ERR_INVALID_ARG
 * otherwise.  IVP_ERR_OUT_OF_MEMORY means a replayed PSI met a refused
 * allocation, as recorded; the world has still reached `psi`. */
ivp_result_t ivp_rewind_seek(ivp_rewind_t *rewind, uint64_t psi);

ivp_environment_t *ivp_rewind_environment(const ivp_rewind_t *rewind);
uint64_t ivp_rewind_current_psi(const ivp_rewind_t *rewind);
uint64_t ivp_rewind_oldest_psi(const ivp_rewind_t *rewind);
/* the last PSI the recorded inputs reach */
uint64_t ivp_rewind_recorded_psi(const ivp_rewind_t *rewind);

#ifdef __cplusplus
}
#endif

#endif /* IVP_REWIND_H */
