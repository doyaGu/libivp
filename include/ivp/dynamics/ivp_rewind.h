/* ivp_rewind.h — Rewinding a world to any recorded PSI (libivp extension)
 *
 * A rewinder records a world PSI by PSI: before every PSI it applies the
 * caller's input for that PSI through a callback and logs a copy of the
 * input; every keyframe_interval PSIs it keeps a world snapshot
 * (ivp_snapshot.h) in a ring of keyframe_capacity keyframes.  Seeking to an
 * earlier PSI restores the nearest keyframe at or before it and replays the
 * logged inputs; seeking forward replays from the current world.  Because a
 * restored world continues bit for bit, the world after seeking to PSI n is
 * the one the recording had after n PSIs.
 *
 * Stepping after seeking back branches: the inputs and keyframes after the
 * current PSI are discarded and new ones are recorded.  When the ring drops
 * its oldest keyframe, the inputs before the next keyframe go with it, so
 * the oldest reachable PSI moves on.
 *
 * The rewinder owns the world it is created with and replaces it on every
 * seek that restores a keyframe, so object and controller pointers change:
 * find objects again by their order key (ivp_rewind_find_object) and
 * controller states by registration order (ivp_rewind_controller_state).
 * The rewinder owns the controller states of restored worlds; those of the
 * world it was created with stay the caller's and must outlive that world.
 *
 * The world may only change through ivp_rewind_step and the input callback,
 * and the callback must depend only on the PSI and its input; otherwise a
 * replay cannot reproduce the recording.  Snapshot coverage applies: a
 * world that holds state a snapshot does not support cannot be recorded.
 * Built only with IVP_ENABLE_SNAPSHOTS (CMake option, default off).
 */
#ifndef IVP_REWIND_H
#define IVP_REWIND_H

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
    /* the configuration the world was created with; restored worlds use it */
    ivp_environment_config_t environment_config;
    int keyframe_interval;   /* PSIs between keyframes, at least 1 */
    int keyframe_capacity;   /* keyframes kept, at least 1 */
    ivp_rewind_apply_fn_t apply;   /* may be NULL */
    void *apply_ctx;
} ivp_rewind_config_t;

/* Take over env (not inside a simulate call) and keep it as the keyframe of
 * PSI 0.  Returns NULL and sets *result on failure; env is then still the
 * caller's. */
ivp_rewind_t *ivp_rewind_create(ivp_environment_t *env,
                                const ivp_rewind_config_t *config,
                                ivp_result_t *result);

/* Destroy the rewinder with its current world. */
void ivp_rewind_destroy(ivp_rewind_t *rewind);

/* Apply input (copied into the log) and simulate one PSI, keeping a
 * keyframe first when the PSI count is a multiple of the interval.  After a
 * seek back this discards the recorded future.  On failure nothing
 * changes. */
ivp_result_t ivp_rewind_step(ivp_rewind_t *rewind, const void *input,
                             size_t size);

/* Move to the world after `psi` PSIs, which must lie between
 * ivp_rewind_oldest_psi and ivp_rewind_recorded_psi; IVP_ERR_INVALID_ARG
 * otherwise.  On failure the current world is kept. */
ivp_result_t ivp_rewind_seek(ivp_rewind_t *rewind, uint64_t psi);

ivp_environment_t *ivp_rewind_environment(const ivp_rewind_t *rewind);
uint64_t ivp_rewind_current_psi(const ivp_rewind_t *rewind);
uint64_t ivp_rewind_oldest_psi(const ivp_rewind_t *rewind);
/* the last PSI the recorded inputs reach */
uint64_t ivp_rewind_recorded_psi(const ivp_rewind_t *rewind);

/* The current world's object with the given ivp_object_order_key, or NULL. */
ivp_object_t *ivp_rewind_find_object(const ivp_rewind_t *rewind,
                                     uint64_t order_key);

/* The state of the current world's registered controller `index`, in
 * registration order, or NULL. */
void *ivp_rewind_controller_state(const ivp_rewind_t *rewind, int index);

#ifdef __cplusplus
}
#endif

#endif /* IVP_REWIND_H */
