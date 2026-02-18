/* ivp_broadphase.h — Broadphase collision detection interface
 *
 * Provides a sweep-and-prune (SAP) broadphase as an alternative to
 * the O(n²) brute-force approach.  The SAP broadphase sorts objects
 * along a primary axis and sweeps for overlapping AABB intervals,
 * giving O(n log n) average-case for well-distributed scenes.
 *
 * Falls back to O(n²) for small scenes (< IVP_SAP_THRESHOLD objects).
 */
#ifndef IVP_BROADPHASE_H
#define IVP_BROADPHASE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/collision/ivp_collision_tolerance.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

/* Objects below this count use brute-force (O(n²) is faster for tiny N) */
#define IVP_SAP_THRESHOLD  32

/* Maximum broadphase pair candidates before narrowphase */
#define IVP_SAP_MAX_PAIRS  4096

/* ── SAP entry ─────────────────────────────────────────────────────────── */

typedef struct ivp_sap_entry {
    int      obj_index;    /* index into environment object array       */
    ivp_real_t min_val;    /* AABB min on sweep axis                    */
    ivp_real_t max_val;    /* AABB max on sweep axis                    */
} ivp_sap_entry_t;

/* ── SAP broadphase state ──────────────────────────────────────────────── */

typedef struct ivp_broadphase_sap {
    ivp_sap_entry_t *entries;       /* sorted axis entries              */
    int              entry_count;
    int              entry_capacity;
    int              sweep_axis;    /* 0=X, 1=Y, 2=Z (auto-selected)   */
    const ivp_allocator_t *alloc;
} ivp_broadphase_sap_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

void ivp_broadphase_sap_init(ivp_broadphase_sap_t *sap,
                              const ivp_allocator_t *alloc);

void ivp_broadphase_sap_destroy(ivp_broadphase_sap_t *sap);

/* ── Detection ─────────────────────────────────────────────────────────── */

/* Sweep-and-prune broadphase that can replace ivp_broadphase_detect().
 * Same interface: fills contacts array, returns count. */
int ivp_broadphase_sap_detect(ivp_broadphase_sap_t *sap,
                               struct ivp_environment *env,
                               const ivp_collision_settings_t *coll_settings,
                               ivp_contact_point_t *contacts,
                               int max_contacts);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BROADPHASE_H */
