/* ivp_merge_core.h -- Merged core for breakable object groups
 *
 * A merge core joins multiple objects into a single rigid body by
 * summing their masses and inertias. When a break threshold is exceeded
 * (impulse or energy), the merge splits and objects regain independence.
 *
 * Unlike ivp_object_attach (which keeps follower cores pinned),
 * merge_core replaces the individual cores with a single shared
 * aggregate core. On split, new individual cores are created from
 * the aggregate state.
 *
 * Ported from ivp_merge_core.hxx.
 */
#ifndef IVP_MERGE_CORE_H
#define IVP_MERGE_CORE_H

#include <ivp/base/ivp_types.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Merge group ───────────────────────────────────────────────────────── */

#define IVP_MERGE_MAX_OBJECTS 32

/* Forward declare for break callback typedef */
typedef struct ivp_merge_group ivp_merge_group_t;

typedef struct ivp_merge_member {
    ivp_object_t *obj;
    ivp_vec3r_t   offset_pos;   /* offset from merge center in merge frame */
    ivp_quatr_t   offset_rot;   /* rotation offset from merge frame */
    ivp_core_t    saved_core;   /* original core state (restored on split) */
} ivp_merge_member_t;

/* Callback when the merge group breaks. */
typedef void (*ivp_merge_break_fn_t)(void *ctx,
                                       ivp_merge_group_t *group);

struct ivp_merge_group {
    ivp_core_t         merged_core;     /* aggregate core */
    ivp_merge_member_t members[IVP_MERGE_MAX_OBJECTS];
    int                n_members;
    ivp_real_t         break_impulse;   /* break threshold (impulse magnitude) */
    ivp_merge_break_fn_t break_fn;
    void              *break_ctx;
    bool               merged;          /* true if currently merged */
};

/* ── API ───────────────────────────────────────────────────────────────── */

/* Initialize a merge group. break_impulse <= 0 means unbreakable. */
void ivp_merge_group_init(ivp_merge_group_t *group,
                            ivp_real_t break_impulse);

/* Add an object to the merge group. Must call ivp_merge_group_merge()
 * after adding all objects. Returns IVP_OK on success. */
ivp_result_t ivp_merge_group_add(ivp_merge_group_t *group,
                                   ivp_object_t *obj);

/* Compute the merged core from all members.
 * After this call, all member objects share the merged core state. */
void ivp_merge_group_merge(ivp_merge_group_t *group);

/* Split the merged group: restore individual cores from saved state
 * with velocities inherited from the merged core.
 * Typically called from the break callback or externally. */
void ivp_merge_group_split(ivp_merge_group_t *group);

/* Check if accumulated impulse exceeds break threshold.
 * Call after collision resolution. If exceeded, calls break_fn
 * and splits automatically. */
void ivp_merge_group_check_break(ivp_merge_group_t *group,
                                   ivp_real_t impulse_magnitude);

/* Set break callback. */
static inline void ivp_merge_group_set_break_callback(
    ivp_merge_group_t *group,
    ivp_merge_break_fn_t fn,
    void *ctx)
{
    group->break_fn = fn;
    group->break_ctx = ctx;
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_MERGE_CORE_H */
