/* ivp_matrix_cache.h — IVP_U_Matrix_Cache (internal)
 *
 * Reference: ivp_physics/ivp_3d_solver.hxx.  The object matrices of one
 * PSI at IVP_3D_SOLVER_MAX_STEPS_PER_PSI + 1 sample times, shared by the
 * 3D solvers of ivp_mindist_event.c.
 */
#ifndef IVP_COLLISION_MATRIX_CACHE_H
#define IVP_COLLISION_MATRIX_CACHE_H

#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/dynamics/ivp_object.h>
#include "collision/ivp_cls_internal.h"
#include "dynamics/ivp_env_internal.h"

void ivp_object_calc_at_matrix(const struct ivp_object *obj, ivp_time_t t, ivp_transform_r_t *out);

#define IVP_3D_SOLVER_MAX_STEPS_PER_PSI 20
#define IVP_3D_SOLVER_PSIS_PER_SECOND   10   /* (int)(1.0f / IVP_MAX_DELTA_PSI_TIME) */
#define IVP_3D_SOLVER_STEPS_PER_SECOND  (IVP_3D_SOLVER_MAX_STEPS_PER_PSI * IVP_3D_SOLVER_PSIS_PER_SECOND)

/* ── IVP_U_Matrix_Cache ─────────────────────────────────────────────────── */

/* static int IVP_U_Matrix_Cache::hits / misses (ivp_3d_solver.hxx:14), only
 * used by the IVP_IF(0) blocks of calc_matrix_at */
static int mc_hits;
static int mc_misses;

typedef struct matrix_cache {
    struct ivp_object  *object;
    struct ivp_core    *core;
    ivp_time_t          base_time;
    ivp_time_code_t     base_time_code;
    ivp_transform_r_t  *m_world_f_object[IVP_3D_SOLVER_MAX_STEPS_PER_PSI + 1];
    ivp_transform_r_t   matrizes[IVP_3D_SOLVER_MAX_STEPS_PER_PSI + 1];
} matrix_cache_t;

static inline void mc_init(matrix_cache_t *mc, ivp_cache_obj_t *co)
{
    mc->object = co->object;
    mc->core = co->object->physical_core;
    if (!IVP_MTIS_SIMULATED(co->object->object_movement_state)) {
        for (int i = IVP_3D_SOLVER_MAX_STEPS_PER_PSI; i >= 0; i--) mc->m_world_f_object[i] = &co->m_world_f_object;
    } else {
        for (int i = IVP_3D_SOLVER_MAX_STEPS_PER_PSI; i > 0; i--) mc->m_world_f_object[i] = NULL;
        mc->m_world_f_object[0] = &co->m_world_f_object;
    }
    mc->base_time_code = co->valid_until_time_code;
    mc->base_time = ivp_env_get_current_time(co->object->owner);
#ifdef IVP_DEBUG_BUILD
    if (IVP_MTIS_SIMULATED(co->object->object_movement_state)) {   /* ivp_3d_solver.hxx:99 */
        IVP_ASSERT(mc->base_time_code == co->valid_until_time_code);
    }
#endif
}

static inline ivp_transform_r_t *mc_calc_matrix_at(matrix_cache_t *mc, ivp_time_t t, int index)
{
    /* ivp_3d_solver.hxx:26-41: the index is clamped to
     * [0, IVP_3D_SOLVER_MAX_STEPS_PER_PSI]; only a request that was in range
     * is checked against its time */
    const int requested_index = index;
    if (index < 0) {
        index = 0;
    } else if (index > IVP_3D_SOLVER_MAX_STEPS_PER_PSI) {
        index = IVP_3D_SOLVER_MAX_STEPS_PER_PSI;
    }
    if (requested_index == index) {
        IVP_ASSERT(((int)(ivp_time_diff(t, mc->base_time) * (IVP_3D_SOLVER_MAX_STEPS_PER_PSI * IVP_3D_SOLVER_PSIS_PER_SECOND) + (double).5f)) == index);
    }
    if (!mc->m_world_f_object[index]) {
        mc->m_world_f_object[index] = &mc->matrizes[index];
        ivp_object_calc_at_matrix(mc->object, t, mc->m_world_f_object[index]);
        IVP_IF(0) {   /* ivp_3d_solver.hxx:46 */
            mc_misses++;
            mc_hits--;
        }
    }
    IVP_IF(0) {   /* ivp_3d_solver.hxx:52 */
        mc_hits++;
        if (mc_misses % 1000 == 0) {
            ivp_debug_printf("cache hit ratio %f\n", (double)mc_hits / (double)(mc_misses + mc_hits));
        }
    }
    return mc->m_world_f_object[index];
}

static inline ivp_transform_r_t *mc_calc_matrix_at_now(matrix_cache_t *mc, ivp_time_t t, int index)
{
    /* ivp_3d_solver.hxx:65-66 */
    IVP_ASSERT(index == 0);
    IVP_ASSERT(ivp_time_diff(t, mc->base_time) == 0.0);
    IVP_USE(t);
    IVP_USE(index);
    return mc->m_world_f_object[0];
}

#endif /* IVP_COLLISION_MATRIX_CACHE_H */
