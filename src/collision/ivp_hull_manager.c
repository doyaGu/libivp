/* ivp_hull_manager.c — Hull expansion tracking implementation
 *
 * Tracks accumulated travel distance per object.  When an object has
 * moved more than a threshold fraction of its bounding radius, its
 * hull is considered "expanded" and nearby mindist entries should be
 * re-evaluated.
 */
#include <ivp/collision/ivp_hull_manager.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_fast_math.h>
#include <string.h>

/* Hull expansion threshold: fraction of base radius before invalidation */
#define IVP_HULL_EXPANSION_FRACTION  IVP_REAL(0.3)

/* ── Lifecycle ──────────────────────────────────────────────────────────── */

void ivp_hull_manager_init(ivp_hull_manager_t *mgr,
                             const ivp_allocator_t *alloc)
{
    memset(mgr, 0, sizeof(*mgr));
    mgr->alloc = alloc ? alloc : ivp_default_allocator();
}

void ivp_hull_manager_destroy(ivp_hull_manager_t *mgr)
{
    if (mgr->entries) {
        ivp_free(mgr->alloc, mgr->entries,
                 (size_t)mgr->entry_capacity * sizeof(ivp_hull_entry_t));
        mgr->entries = NULL;
    }
    mgr->entry_count = 0;
    mgr->entry_capacity = 0;
}

/* ── Ensure capacity ───────────────────────────────────────────────────── */

static bool ensure_capacity(ivp_hull_manager_t *mgr, int needed)
{
    if (needed <= mgr->entry_capacity) return true;

    int new_cap = mgr->entry_capacity ? mgr->entry_capacity * 2 : 64;
    if (new_cap < needed) new_cap = needed;

    ivp_hull_entry_t *new_entries = (ivp_hull_entry_t *)ivp_alloc(
        mgr->alloc, (size_t)new_cap * sizeof(ivp_hull_entry_t));
    if (!new_entries) return false;

    memset(new_entries, 0, (size_t)new_cap * sizeof(ivp_hull_entry_t));
    if (mgr->entries) {
        memcpy(new_entries, mgr->entries,
               (size_t)mgr->entry_count * sizeof(ivp_hull_entry_t));
        ivp_free(mgr->alloc, mgr->entries,
                 (size_t)mgr->entry_capacity * sizeof(ivp_hull_entry_t));
    }

    mgr->entries = new_entries;
    mgr->entry_capacity = new_cap;
    return true;
}

/* ── Operations ────────────────────────────────────────────────────────── */

void ivp_hull_manager_add_object(ivp_hull_manager_t *mgr,
                                   ivp_object_t *obj)
{
    if (!obj) return;

    /* Check if already tracked */
    for (int i = 0; i < mgr->entry_count; i++) {
        if (mgr->entries[i].active && mgr->entries[i].obj == obj) {
            return; /* already present */
        }
    }

    /* Try reuse inactive slot */
    for (int i = 0; i < mgr->entry_count; i++) {
        if (!mgr->entries[i].active) {
            ivp_hull_entry_t *e = &mgr->entries[i];
            e->obj = obj;
            e->last_checked_pos = obj->core.pos_world_f_core_last_psi;
            e->accumulated_travel = IVP_REAL(0.0);
            e->base_radius = obj->core.upper_limit_radius;
            e->hull_radius = e->base_radius;
            e->active = true;
            return;
        }
    }

    /* Append */
    if (!ensure_capacity(mgr, mgr->entry_count + 1)) return;

    ivp_hull_entry_t *e = &mgr->entries[mgr->entry_count++];
    e->obj = obj;
    e->last_checked_pos = obj->core.pos_world_f_core_last_psi;
    e->accumulated_travel = IVP_REAL(0.0);
    e->base_radius = obj->core.upper_limit_radius;
    e->hull_radius = e->base_radius;
    e->active = true;
}

void ivp_hull_manager_remove_object(ivp_hull_manager_t *mgr,
                                      ivp_object_t *obj)
{
    for (int i = 0; i < mgr->entry_count; i++) {
        if (mgr->entries[i].active && mgr->entries[i].obj == obj) {
            mgr->entries[i].active = false;
            return;
        }
    }
}

int ivp_hull_manager_update(ivp_hull_manager_t *mgr, ivp_real_t dt)
{
    (void)dt;
    int expanded = 0;

    for (int i = 0; i < mgr->entry_count; i++) {
        ivp_hull_entry_t *e = &mgr->entries[i];
        if (!e->active || !e->obj) continue;

        /* Compute distance traveled since last check */
        ivp_vec3r_t diff;
        ivp_vec3r_sub(&diff, &e->obj->core.pos_world_f_core_last_psi,
                      &e->last_checked_pos);
        ivp_real_t travel = ivp_vec3r_length(&diff);

        /* Also account for rotational surface movement */
        travel += e->obj->core.max_surface_rot_speed * dt;

        e->accumulated_travel += travel;
        e->last_checked_pos = e->obj->core.pos_world_f_core_last_psi;

        /* Update hull radius */
        e->hull_radius = e->base_radius + e->accumulated_travel;

        /* Check if expansion exceeds threshold */
        if (e->accumulated_travel > e->base_radius * IVP_HULL_EXPANSION_FRACTION) {
            expanded++;
        }
    }

    return expanded;
}

ivp_real_t ivp_hull_manager_get_radius(const ivp_hull_manager_t *mgr,
                                         const ivp_object_t *obj)
{
    for (int i = 0; i < mgr->entry_count; i++) {
        const ivp_hull_entry_t *e = &mgr->entries[i];
        if (e->active && e->obj == obj) {
            return e->hull_radius;
        }
    }
    return obj->core.upper_limit_radius;
}

void ivp_hull_manager_reset(ivp_hull_manager_t *mgr,
                              ivp_object_t *obj)
{
    for (int i = 0; i < mgr->entry_count; i++) {
        ivp_hull_entry_t *e = &mgr->entries[i];
        if (e->active && e->obj == obj) {
            e->accumulated_travel = IVP_REAL(0.0);
            e->hull_radius = e->base_radius;
            e->last_checked_pos = obj->core.pos_world_f_core_last_psi;
            return;
        }
    }
}
