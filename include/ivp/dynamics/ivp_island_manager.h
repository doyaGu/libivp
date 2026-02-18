/* ivp_island_manager.h — Automatic simulation island detection
 *
 * Partitions objects into independent simulation islands using
 * union-find over cores connected by controllers or friction contacts.
 * Each island can be frozen/woken independently.
 *
 * Ported from legacy IVP_Sim_Unit / IVP_Clustering.
 */
#ifndef IVP_ISLAND_MANAGER_H
#define IVP_ISLAND_MANAGER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define IVP_ISLAND_MAX_OBJECTS   1024
#define IVP_ISLAND_MAX_ISLANDS    256

/* ── Island ────────────────────────────────────────────────────────────── */

typedef struct ivp_island {
    int   *object_indices;       /* indices into environment object array */
    int    object_count;
    int    object_capacity;
    bool   all_frozen;           /* true if all cores are NOT_SIM        */
    bool   has_moving;           /* true if any core is MOVING/SLOW      */
    const ivp_allocator_t *alloc;
} ivp_island_t;

/* ── Island manager ────────────────────────────────────────────────────── */

typedef struct ivp_island_manager {
    /* Union-find parent array (indexed by object dense_index) */
    int   *parent;
    int   *rank;
    int    uf_capacity;

    /* Resulting islands after build */
    ivp_island_t *islands;
    int    island_count;
    int    island_capacity;

    const ivp_allocator_t *alloc;
} ivp_island_manager_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

void ivp_island_manager_init(ivp_island_manager_t *mgr,
                              const ivp_allocator_t *alloc);

void ivp_island_manager_destroy(ivp_island_manager_t *mgr);

/* ── Build islands ─────────────────────────────────────────────────────── */

/* Rebuild all islands from scratch.  Scans controllers and friction
 * contacts to determine connectivity.  Static objects are excluded
 * from islands (they don't need simulation). */
void ivp_island_manager_rebuild(ivp_island_manager_t *mgr,
                                 struct ivp_environment *env);

/* ── Queries ───────────────────────────────────────────────────────────── */

/* Number of islands after last rebuild. */
static inline int ivp_island_manager_count(const ivp_island_manager_t *mgr) {
    return mgr->island_count;
}

/* Get island by index [0, count). */
static inline const ivp_island_t *ivp_island_manager_get(
    const ivp_island_manager_t *mgr, int index)
{
    if (index < 0 || index >= mgr->island_count) return NULL;
    return &mgr->islands[index];
}

/* Find which island an object belongs to (by dense_index).
 * Returns island index, or -1 if not in any island (e.g. static). */
int ivp_island_manager_find_island(const ivp_island_manager_t *mgr,
                                    int object_dense_index);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ISLAND_MANAGER_H */
