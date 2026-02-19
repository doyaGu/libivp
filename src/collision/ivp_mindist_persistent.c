/* ivp_mindist_persistent.c — Persistent minimum distance tracking
 *
 * Caches closest feature pairs between object pairs across PSI steps.
 * Entries age out and are periodically refreshed with full distance
 * recomputation.
 */
#include <ivp/collision/ivp_mindist_persistent.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/collision/ivp_mindist.h>
#include <ivp/collision/ivp_synapse.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/math/ivp_vec3.h>
#include <string.h>

/* ── Lifecycle ──────────────────────────────────────────────────────────── */

void ivp_mindist_manager_init(ivp_mindist_manager_t *mgr,
                                const ivp_allocator_t *alloc)
{
    memset(mgr, 0, sizeof(*mgr));
    mgr->alloc = alloc ? alloc : ivp_default_allocator();
}

void ivp_mindist_manager_destroy(ivp_mindist_manager_t *mgr)
{
    if (mgr->entries) {
        for (int i = 0; i < mgr->entry_count; i++) {
            ivp_mindist_entry_t *e = &mgr->entries[i];
            if (e->active) {
                ivp_synapse_detach_mindist_entry(e);
            }
        }
        ivp_free(mgr->alloc, mgr->entries,
                 (size_t)mgr->entry_capacity * sizeof(ivp_mindist_entry_t));
        mgr->entries = NULL;
    }
    mgr->entry_count = 0;
    mgr->entry_capacity = 0;
}

/* ── Ensure capacity ───────────────────────────────────────────────────── */

static bool ensure_capacity(ivp_mindist_manager_t *mgr, int needed)
{
    if (needed <= mgr->entry_capacity) return true;

    int new_cap = mgr->entry_capacity ? mgr->entry_capacity * 2 : 256;
    if (new_cap < needed) new_cap = needed;

    ivp_mindist_entry_t *new_entries = (ivp_mindist_entry_t *)ivp_alloc(
        mgr->alloc, (size_t)new_cap * sizeof(ivp_mindist_entry_t));
    if (!new_entries) return false;

    memset(new_entries, 0, (size_t)new_cap * sizeof(ivp_mindist_entry_t));

    if (mgr->entries) {
        memcpy(new_entries, mgr->entries,
               (size_t)mgr->entry_count * sizeof(ivp_mindist_entry_t));
        ivp_free(mgr->alloc, mgr->entries,
                 (size_t)mgr->entry_capacity * sizeof(ivp_mindist_entry_t));
    }

    mgr->entries = new_entries;
    mgr->entry_capacity = new_cap;
    return true;
}

/* ── Operations ────────────────────────────────────────────────────────── */

ivp_mindist_entry_t *ivp_mindist_manager_get_or_create(
    ivp_mindist_manager_t *mgr,
    ivp_object_t *obj_a,
    ivp_object_t *obj_b)
{
    /* Canonical ordering: smaller pointer first */
    if ((uintptr_t)obj_a > (uintptr_t)obj_b) {
        ivp_object_t *tmp = obj_a;
        obj_a = obj_b;
        obj_b = tmp;
    }

    /* Search for existing entry */
    for (int i = 0; i < mgr->entry_count; i++) {
        ivp_mindist_entry_t *e = &mgr->entries[i];
        if (!e->active) continue;
        if (e->obj_a == obj_a && e->obj_b == obj_b) {
            return e;
        }
    }

    /* Try to reuse an inactive slot */
    for (int i = 0; i < mgr->entry_count; i++) {
        ivp_mindist_entry_t *e = &mgr->entries[i];
        if (!e->active) {
            ivp_synapse_detach_mindist_entry(e);
            memset(e, 0, sizeof(*e));
            e->obj_a = obj_a;
            e->obj_b = obj_b;
            e->state = IVP_MDIST_UNINITIALIZED;
            e->active = true;
            e->feature_a.vertex_index = -1;
            e->feature_a.edge_index = -1;
            e->feature_a.face_index = -1;
            e->feature_b.vertex_index = -1;
            e->feature_b.edge_index = -1;
            e->feature_b.face_index = -1;
            ivp_synapse_attach_mindist_entry(e);
            return e;
        }
    }

    /* Need new slot */
    if (!ensure_capacity(mgr, mgr->entry_count + 1)) return NULL;

    ivp_mindist_entry_t *e = &mgr->entries[mgr->entry_count++];
    memset(e, 0, sizeof(*e));
    e->obj_a = obj_a;
    e->obj_b = obj_b;
    e->state = IVP_MDIST_UNINITIALIZED;
    e->active = true;
    e->feature_a.vertex_index = -1;
    e->feature_a.edge_index = -1;
    e->feature_a.face_index = -1;
    e->feature_b.vertex_index = -1;
    e->feature_b.edge_index = -1;
    e->feature_b.face_index = -1;
    ivp_synapse_attach_mindist_entry(e);
    return e;
}

void ivp_mindist_manager_remove_object(ivp_mindist_manager_t *mgr,
                                         ivp_object_t *obj)
{
    for (int i = 0; i < mgr->entry_count; i++) {
        ivp_mindist_entry_t *e = &mgr->entries[i];
        if (!e->active) continue;
        if (e->obj_a == obj || e->obj_b == obj) {
            ivp_synapse_detach_mindist_entry(e);
            e->active = false;
        }
    }
}

void ivp_mindist_manager_age_entries(ivp_mindist_manager_t *mgr)
{
    for (int i = 0; i < mgr->entry_count; i++) {
        ivp_mindist_entry_t *e = &mgr->entries[i];
        if (!e->active) continue;

        e->age++;

        /* Invalidate entries that haven't been refreshed in a while */
        if (e->age >= IVP_MINDIST_RECALC_AGE) {
            e->state = IVP_MDIST_INVALID;
        }
    }
}

void ivp_mindist_entry_recompute(ivp_mindist_entry_t *entry)
{
    if (!entry || !entry->active) return;
    if (!entry->obj_a || !entry->obj_b) return;

    /* Use the generic mindist computation.  This calls the full
     * narrowphase solver which handles sphere-sphere, sphere-polygon,
     * and polygon-polygon cases. */
    ivp_contact_point_t cp;
    int produced = ivp_contact_compute_generic_multi(
        entry->obj_a, entry->obj_b, &cp, 1);

    if (produced > 0) {
        entry->last_distance = cp.gap_length;
        entry->last_normal = cp.normal;
        entry->closest_point_a = cp.point_a;
        entry->closest_point_b = cp.point_b;

        /* Detect backside intrusion: if the gap is severely negative,
         * the geometry is penetrating from the back face. Transition
         * the synapses to ST_BACKSIDE and the entry to INVALID,
         * triggering a full re-initialization next time.
         * (Reference alignment: IVP_ST_BACKSIDE triggers INVALID mindist) */
        if (cp.gap_length < -cp.obj_a->core.upper_limit_radius * IVP_REAL(0.5)) {
            entry->synapse[0].status = IVP_ST_BACKSIDE;
            entry->synapse[1].status = IVP_ST_BACKSIDE;
            entry->state = IVP_MDIST_INVALID;
        } else {
            entry->state = IVP_MDIST_EXACT;
        }
        entry->age = 0;
    } else {
        /* No contact found — objects are far apart */
        entry->state = IVP_MDIST_HULL;
    }
}

bool ivp_mindist_entry_update_cached(ivp_mindist_entry_t *entry)
{
    if (!entry || !entry->active) return false;

    /* If state is invalid, need full recomputation */
    if (entry->state == IVP_MDIST_UNINITIALIZED ||
        entry->state == IVP_MDIST_INVALID ||
        entry->state == IVP_MDIST_HULL_RECURSIVE) {
        ivp_mindist_entry_recompute(entry);
        return entry->state == IVP_MDIST_EXACT;
    }

    /* For cached states, estimate new distance from object motion.
     * This is a conservative approximation — if objects haven't moved
     * much, the cached distance is still approximately correct. */
    const ivp_core_t *ca = &entry->obj_a->core;
    const ivp_core_t *cb = &entry->obj_b->core;

    /* Estimate maximum distance change from velocities */
    ivp_real_t speed_a = ca->current_speed + ca->max_surface_rot_speed;
    ivp_real_t speed_b = cb->current_speed + cb->max_surface_rot_speed;
    ivp_real_t max_movement = speed_a + speed_b; /* per second */

    /* If objects are slow enough, cached result is valid */
    if (max_movement < IVP_REAL(0.1)) {
        return true; /* cache still valid */
    }

    /* Otherwise, need full recomputation */
    ivp_mindist_entry_recompute(entry);
    return entry->state == IVP_MDIST_EXACT;
}

void ivp_mindist_manager_invalidate_all(ivp_mindist_manager_t *mgr)
{
    if (!mgr) return;
    for (int i = 0; i < mgr->entry_count; i++) {
        ivp_mindist_entry_t *e = &mgr->entries[i];
        if (!e->active) continue;
        e->state = IVP_MDIST_INVALID;
    }
}
