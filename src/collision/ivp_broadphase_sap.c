/* ivp_broadphase_sap.c — Sweep-and-prune broadphase
 *
 * Sorts objects along the axis of greatest variance, then sweeps for
 * overlapping AABB intervals.  Average case O(n log n) for well-
 * distributed scenes.  Falls back to brute-force for small N.
 *
 * After finding candidate pairs, applies the same filter chain as the
 * original O(n²) broadphase and calls narrowphase for geometry tests.
 */
#include <ivp/collision/ivp_broadphase.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/collision/ivp_collision_filter.h>
#include <ivp/collision/ivp_range_manager.h>
#include <ivp/collision/ivp_mindist.h>
#include <ivp/collision/ivp_compact_ledge_solver.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/math/ivp_vec3.h>
#include <string.h>
#include <math.h>

/* ── Lifecycle ──────────────────────────────────────────────────────────── */

void ivp_broadphase_sap_init(ivp_broadphase_sap_t *sap,
                              const ivp_allocator_t *alloc)
{
    memset(sap, 0, sizeof(*sap));
    sap->alloc = alloc ? alloc : ivp_default_allocator();
    sap->sweep_axis = 0;
}

void ivp_broadphase_sap_destroy(ivp_broadphase_sap_t *sap)
{
    if (sap->entries) {
        ivp_free(sap->alloc, sap->entries,
                 (size_t)sap->entry_capacity * sizeof(ivp_sap_entry_t));
        sap->entries = NULL;
    }
    sap->entry_count = 0;
    sap->entry_capacity = 0;
}

/* ── Axis variance selection ───────────────────────────────────────────── */

/* Pick the axis with greatest positional variance for best SAP performance */
static int choose_sweep_axis(struct ivp_environment *env, int n)
{
    if (n < 2) return 0;

    ivp_real_t mean[3] = {0, 0, 0};
    for (int i = 0; i < n; i++) {
        ivp_object_t *obj = ivp_environment_get_object(env, i);
        if (!obj) continue;
        const ivp_vec3r_t *p = &obj->core.pos_world_f_core_last_psi;
        mean[0] += p->k[0];
        mean[1] += p->k[1];
        mean[2] += p->k[2];
    }
    ivp_real_t inv_n = IVP_REAL(1.0) / (ivp_real_t)n;
    mean[0] *= inv_n;
    mean[1] *= inv_n;
    mean[2] *= inv_n;

    ivp_real_t var[3] = {0, 0, 0};
    for (int i = 0; i < n; i++) {
        ivp_object_t *obj = ivp_environment_get_object(env, i);
        if (!obj) continue;
        const ivp_vec3r_t *p = &obj->core.pos_world_f_core_last_psi;
        for (int a = 0; a < 3; a++) {
            ivp_real_t d = p->k[a] - mean[a];
            var[a] += d * d;
        }
    }

    int best = 0;
    if (var[1] > var[best]) best = 1;
    if (var[2] > var[best]) best = 2;
    return best;
}

/* ── Insertion sort (nearly-sorted from previous frame) ────────────────── */

static void insertion_sort_entries(ivp_sap_entry_t *e, int n)
{
    for (int i = 1; i < n; i++) {
        ivp_sap_entry_t key = e[i];
        int j = i - 1;
        while (j >= 0 && e[j].min_val > key.min_val) {
            e[j + 1] = e[j];
            j--;
        }
        e[j + 1] = key;
    }
}

/* ── Pair filter (same logic as original broadphase) ───────────────────── */

static bool pair_passes_filters(struct ivp_environment *env,
                                 ivp_object_t *obj_a,
                                 ivp_object_t *obj_b)
{
    /* Skip static-static pairs */
    if (obj_a->core.physical_unmoveable && obj_b->core.physical_unmoveable) {
        return false;
    }

    /* Skip pairs in the same no-collision group (legacy) */
    if (ivp_objects_share_no_collision_group(obj_a, obj_b)) {
        return false;
    }

    /* Collision filter check (new system) */
    if (!ivp_collision_filter_check(&env->collision_filter, obj_a, obj_b)) {
        return false;
    }

    return true;
}

/* ── Narrowphase call + contact validation ─────────────────────────────── */

static int run_narrowphase(ivp_object_t *obj_a, ivp_object_t *obj_b,
                            ivp_real_t margin,
                            ivp_contact_point_t *out, int max_out)
{
    int produced = ivp_contact_compute_generic_multi(
        obj_a, obj_b, out, max_out);

    int valid = 0;
    for (int k = 0; k < produced; k++) {
        ivp_contact_point_t *cp = &out[k];

        /* Ignore invalid/non-contact results from narrowphase */
        if (!isfinite(cp->gap_length)) {
            continue;
        }

        if (obj_a->type == IVP_OBJECT_POLYGON &&
            obj_b->type == IVP_OBJECT_POLYGON &&
            cp->gap_length > margin) {
            continue;
        }

        if (valid != k) {
            out[valid] = *cp;
        }
        valid++;
    }
    return valid;
}

/* ── SAP broadphase ────────────────────────────────────────────────────── */

int ivp_broadphase_sap_detect(ivp_broadphase_sap_t *sap,
                               struct ivp_environment *env,
                               const ivp_collision_settings_t *coll_settings,
                               ivp_contact_point_t *contacts,
                               int max_contacts)
{
    int n = ivp_environment_get_object_count(env);
    if (n < 2) return 0;

    /* Initialize range manager */
    ivp_range_manager_t rm;
    ivp_real_t broadphase_dt = ivp_environment_get_delta_psi_time(env);
    if (coll_settings && coll_settings->inv_delta_psi_time > IVP_REAL_EPS) {
        broadphase_dt = IVP_REAL(1.0) / coll_settings->inv_delta_psi_time;
    }
    ivp_range_manager_init(&rm, broadphase_dt);

    /* Choose sweep axis based on positional variance */
    sap->sweep_axis = choose_sweep_axis(env, n);
    int axis = sap->sweep_axis;

    /* Ensure entry buffer is large enough */
    if (n > sap->entry_capacity) {
        if (sap->entries) {
            ivp_free(sap->alloc, sap->entries,
                     (size_t)sap->entry_capacity * sizeof(ivp_sap_entry_t));
        }
        sap->entry_capacity = n + 32; /* some headroom */
        sap->entries = (ivp_sap_entry_t *)ivp_alloc(
            sap->alloc,
            (size_t)sap->entry_capacity * sizeof(ivp_sap_entry_t));
        if (!sap->entries) {
            sap->entry_capacity = 0;
            return 0;
        }
    }

    /* Build AABB entries: position ± (radius + per-object range) */
    sap->entry_count = 0;
    for (int i = 0; i < n; i++) {
        ivp_object_t *obj = ivp_environment_get_object(env, i);
        if (!obj) continue;

        ivp_real_t pos = obj->core.pos_world_f_core_last_psi.k[axis];
        ivp_real_t r = obj->core.upper_limit_radius;

        /* Per-object range from range manager (speed-based expansion) */
        ivp_real_t obj_range = ivp_range_manager_get_coll_range_world(&rm, &obj->core);

        /* Use larger of range-manager and collision tolerance */
        ivp_real_t expand = obj_range > coll_settings->max_distance_for_impact
                          ? obj_range
                          : coll_settings->max_distance_for_impact;

        ivp_sap_entry_t *e = &sap->entries[sap->entry_count++];
        e->obj_index = i;
        e->min_val = pos - r - expand;
        e->max_val = pos + r + expand;
    }

    /* Sort by min_val (insertion sort — nearly sorted between frames) */
    insertion_sort_entries(sap->entries, sap->entry_count);

    /* Sweep: for each entry, check against all subsequent overlapping entries */
    int count = 0;
    for (int i = 0; i < sap->entry_count && count < max_contacts; i++) {
        ivp_sap_entry_t *ei = &sap->entries[i];
        ivp_object_t *obj_a = ivp_environment_get_object(env, ei->obj_index);
        if (!obj_a) continue;

        for (int j = i + 1; j < sap->entry_count && count < max_contacts; j++) {
            ivp_sap_entry_t *ej = &sap->entries[j];

            /* If ej->min > ei->max, no more overlaps for this entry */
            if (ej->min_val > ei->max_val) break;

            ivp_object_t *obj_b = ivp_environment_get_object(env, ej->obj_index);
            if (!obj_b) continue;

            /* Apply filter chain */
            if (!pair_passes_filters(env, obj_a, obj_b)) continue;

            /* Full 3D bounding sphere check (SAP only pruned on one axis) */
            ivp_real_t pair_margin = ivp_range_manager_get_pair_margin(
                &rm, &obj_a->core, &obj_b->core);
            ivp_real_t margin = pair_margin > coll_settings->max_distance_for_impact
                              ? pair_margin
                              : coll_settings->max_distance_for_impact;

            const ivp_vec3r_t *pa = &obj_a->core.pos_world_f_core_last_psi;
            const ivp_vec3r_t *pb = &obj_b->core.pos_world_f_core_last_psi;
            ivp_real_t ra = obj_a->core.upper_limit_radius;
            ivp_real_t rb = obj_b->core.upper_limit_radius;

            ivp_vec3r_t diff;
            ivp_vec3r_sub(&diff, pb, pa);
            ivp_real_t qdist = ivp_vec3r_quad_length(&diff);
            ivp_real_t threshold = ra + rb + margin;

            if (qdist >= threshold * threshold) continue;

            /* Narrowphase */
            int free_slots = max_contacts - count;
            if (free_slots <= 0) break;

            int valid = run_narrowphase(obj_a, obj_b, margin,
                                         &contacts[count], free_slots);
            count += valid;
        }
    }

    return count;
}
