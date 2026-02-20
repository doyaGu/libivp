/* ivp_island_manager.c — Simulation island detection via union-find
 *
 * Partitions dynamic objects into independent islands.  Two objects
 * are in the same island if they share a controller (dependent mode)
 * or are connected by friction contacts.
 *
 * Static objects are excluded from islands — they can't move, so they
 * don't need simulation stepping and shouldn't prevent freeze.
 */
#include <ivp/dynamics/ivp_island_manager.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/solver/ivp_friction_system.h>
#include <ivp/utility/ivp_union_find.h>
#include <string.h>

/* Internal controller system header for target inspection */
#include "controllers/ivp_controller_system.h"
#include "ivp_object_system.h"

/* ── Lifecycle ──────────────────────────────────────────────────────────── */

void ivp_island_manager_init(ivp_island_manager_t *mgr,
                              const ivp_allocator_t *alloc)
{
    memset(mgr, 0, sizeof(*mgr));
    mgr->alloc = alloc ? alloc : ivp_default_allocator();
}

static void free_islands(ivp_island_manager_t *mgr)
{
    for (int i = 0; i < mgr->island_count; i++) {
        ivp_island_t *isl = &mgr->islands[i];
        if (isl->object_indices && isl->object_capacity > 0) {
            ivp_free(mgr->alloc, isl->object_indices,
                     (size_t)isl->object_capacity * sizeof(int));
        }
    }
    mgr->island_count = 0;
}

void ivp_island_manager_destroy(ivp_island_manager_t *mgr)
{
    if (mgr->parent) {
        ivp_free(mgr->alloc, mgr->parent,
                 (size_t)mgr->uf_capacity * sizeof(int));
        mgr->parent = NULL;
    }
    if (mgr->rank) {
        ivp_free(mgr->alloc, mgr->rank,
                 (size_t)mgr->uf_capacity * sizeof(int));
        mgr->rank = NULL;
    }

    free_islands(mgr);
    if (mgr->islands) {
        ivp_free(mgr->alloc, mgr->islands,
                 (size_t)mgr->island_capacity * sizeof(ivp_island_t));
        mgr->islands = NULL;
    }
    mgr->uf_capacity = 0;
    mgr->island_capacity = 0;
}

/* ── Ensure capacity ───────────────────────────────────────────────────── */

static bool ensure_uf_capacity(ivp_island_manager_t *mgr, int n)
{
    if (n <= mgr->uf_capacity) return true;

    int new_cap = n + 64;

    int *new_parent = (int *)ivp_alloc(mgr->alloc,
                                        (size_t)new_cap * sizeof(int));
    int *new_rank   = (int *)ivp_alloc(mgr->alloc,
                                        (size_t)new_cap * sizeof(int));
    if (!new_parent || !new_rank) {
        if (new_parent) ivp_free(mgr->alloc, new_parent,
                                  (size_t)new_cap * sizeof(int));
        if (new_rank)   ivp_free(mgr->alloc, new_rank,
                                  (size_t)new_cap * sizeof(int));
        return false;
    }

    if (mgr->parent) {
        ivp_free(mgr->alloc, mgr->parent,
                 (size_t)mgr->uf_capacity * sizeof(int));
    }
    if (mgr->rank) {
        ivp_free(mgr->alloc, mgr->rank,
                 (size_t)mgr->uf_capacity * sizeof(int));
    }

    mgr->parent = new_parent;
    mgr->rank = new_rank;
    mgr->uf_capacity = new_cap;
    return true;
}

static bool ensure_island_capacity(ivp_island_manager_t *mgr, int n)
{
    if (n <= mgr->island_capacity) return true;

    int new_cap = n + 32;
    ivp_island_t *new_islands = (ivp_island_t *)ivp_alloc(
        mgr->alloc, (size_t)new_cap * sizeof(ivp_island_t));
    if (!new_islands) return false;

    memset(new_islands, 0, (size_t)new_cap * sizeof(ivp_island_t));

    /* Move existing islands (pointers, not deep copy) */
    if (mgr->islands) {
        memcpy(new_islands, mgr->islands,
               (size_t)mgr->island_count * sizeof(ivp_island_t));
        ivp_free(mgr->alloc, mgr->islands,
                 (size_t)mgr->island_capacity * sizeof(ivp_island_t));
    }

    mgr->islands = new_islands;
    mgr->island_capacity = new_cap;
    return true;
}

/* ── Add object to island ──────────────────────────────────────────────── */

static bool island_add_object(ivp_island_manager_t *mgr,
                               ivp_island_t *isl, int obj_idx)
{
    if (isl->object_count >= isl->object_capacity) {
        int new_cap = isl->object_capacity ? isl->object_capacity * 2 : 8;
        int *new_indices = (int *)ivp_alloc(
            mgr->alloc, (size_t)new_cap * sizeof(int));
        if (!new_indices) return false;

        if (isl->object_indices) {
            memcpy(new_indices, isl->object_indices,
                   (size_t)isl->object_count * sizeof(int));
            ivp_free(mgr->alloc, isl->object_indices,
                     (size_t)isl->object_capacity * sizeof(int));
        }
        isl->object_indices = new_indices;
        isl->object_capacity = new_cap;
    }

    isl->object_indices[isl->object_count++] = obj_idx;
    return true;
}

static void union_dependent_controller_connectivity(ivp_island_manager_t *mgr,
                                                    ivp_environment_t *env,
                                                    int object_count)
{
    ivp_controller_system_t *csys = (ivp_controller_system_t *)env->controller_system;
    if (!csys) return;

    int nc = ivp_controller_system_count(csys);
    for (int ci = 0; ci < nc; ci++) {
        ivp_controller_slot_t *slot = ivp_controller_system_get_at(csys, ci);
        if (!slot || !slot->enabled) continue;
        if (slot->mode != IVP_CONTROLLER_DEPENDENT) continue;

        ivp_object_handle_t associated[IVP_CONTROLLER_DISCOVERY_STACK_MAX];
        int associated_count = ivp_controller_slot_collect_associated_handles(
            slot, env, associated, IVP_CONTROLLER_DISCOVERY_STACK_MAX);
        if (associated_count < 2) continue;

        int first = -1;
        for (int t = 0; t < associated_count; t++) {
            ivp_object_handle_t h = associated[t];
            if (!ivp_object_handle_is_valid(h)) continue;
            ivp_object_t *obj = ivp_object_system_get_by_handle(
                (const ivp_object_system_t *)env->object_system, h);
            if (!obj) continue;
            if (obj->core.physical_unmoveable) continue;

            int idx = obj->dense_index;
            if (idx < 0 || idx >= object_count) continue;

            if (first < 0) {
                first = idx;
            } else {
                ivp_union_find_union_by_rank(mgr->parent, mgr->rank, first, idx);
            }
        }
    }
}

static void union_friction_connectivity(ivp_island_manager_t *mgr,
                                        ivp_environment_t *env,
                                        int object_count)
{
    ivp_friction_manager_t *fm = &env->friction_manager;
    for (int si = 0; si < fm->system_count; si++) {
        ivp_friction_system_t *fs = fm->systems[si];
        if (!fs || fs->core_count < 2) continue;

        int first_idx = -1;
        for (int ci = 0; ci < fs->core_count; ci++) {
            ivp_core_t *core = fs->cores[ci];
            if (!core) continue;

            ivp_object_t *obj = (ivp_object_t *)core;
            if (obj->core.physical_unmoveable) continue;

            int idx = obj->dense_index;
            if (idx < 0 || idx >= object_count) continue;

            if (first_idx < 0) {
                first_idx = idx;
            } else {
                ivp_union_find_union_by_rank(mgr->parent, mgr->rank, first_idx, idx);
            }
        }
    }
}

static void island_init_defaults(ivp_island_t *isl, const ivp_allocator_t *alloc)
{
    memset(isl, 0, sizeof(*isl));
    isl->alloc = alloc;
    isl->all_frozen = true;
    isl->has_moving = false;
}

/* ── Rebuild ───────────────────────────────────────────────────────────── */

void ivp_island_manager_rebuild(ivp_island_manager_t *mgr,
                                 struct ivp_environment *env)
{
    int n = ivp_environment_get_object_count(env);
    if (n == 0) {
        free_islands(mgr);
        mgr->island_count = 0;
        return;
    }

    /* Ensure union-find buffers */
    if (!ensure_uf_capacity(mgr, n)) return;

    /* Initialize: each object is its own parent */
    ivp_union_find_init_with_rank(mgr->parent, mgr->rank, n);

    union_dependent_controller_connectivity(mgr, env, n);
    union_friction_connectivity(mgr, env, n);

    /* --- Build islands from union-find roots --- */
    free_islands(mgr);
    mgr->island_count = 0;

    /* Map: root → island index. Use rank[] as temporary (will be overwritten). */
    int *root_to_island = mgr->rank; /* reuse buffer as temp map */
    for (int i = 0; i < n; i++) {
        root_to_island[i] = -1;
    }

    for (int i = 0; i < n; i++) {
        ivp_object_t *obj = ivp_environment_get_object(env, i);
        if (!obj) continue;
        if (obj->core.physical_unmoveable) continue;

        int root = ivp_union_find_find(mgr->parent, i);
        int isl_idx = root_to_island[root];

        if (isl_idx < 0) {
            /* New island */
            if (!ensure_island_capacity(mgr, mgr->island_count + 1)) return;
            isl_idx = mgr->island_count++;
            root_to_island[root] = isl_idx;

            ivp_island_t *isl = &mgr->islands[isl_idx];
            island_init_defaults(isl, mgr->alloc);
        }

        ivp_island_t *isl = &mgr->islands[isl_idx];
        island_add_object(mgr, isl, i);

        /* Track movement state */
        ivp_movement_type_t ms = obj->core.movement_state;
        if (ms != IVP_MT_NOT_SIM) {
            isl->all_frozen = false;
        }
        if (ms == IVP_MT_MOVING || ms == IVP_MT_SLOW) {
            isl->has_moving = true;
        }
    }
}

/* ── Queries ───────────────────────────────────────────────────────────── */

int ivp_island_manager_find_island(const ivp_island_manager_t *mgr,
                                    int object_dense_index)
{
    for (int i = 0; i < mgr->island_count; i++) {
        const ivp_island_t *isl = &mgr->islands[i];
        for (int j = 0; j < isl->object_count; j++) {
            if (isl->object_indices[j] == object_dense_index) {
                return i;
            }
        }
    }
    return -1;
}
