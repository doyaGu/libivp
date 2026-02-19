/* ivp_impact_system.c — Multi-body impact cascade solver
 *
 * Implements iterative cascade: resolves impacts, checks for new
 * collisions caused by velocity changes, and expands the system
 * to include affected pairs until stable.
 */

#include <ivp/solver/ivp_impact_system.h>
#include <ivp/solver/ivp_impact.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/solver/ivp_solver_metrics.h>
#include <ivp/solver/ivp_solver_constants.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_vec3.h>
#include <string.h>

/* ── Helpers ────────────────────────────────────────────────────────────── */

/* Returns false if the element could not be added (OOM). */
static inline bool add_unique_core(ivp_array_t *arr, ivp_core_t *c) {
    /* Use old_sync_rot as O(1) "already tracked" flag (reference-aligned). */
    if (c->old_sync_rot != NULL) return true; /* already in the system */
    /* Mark with a non-NULL sentinel (actual state stored if needed later). */
    static ivp_old_sync_rot_t s_sentinel = {0};
    c->old_sync_rot = &s_sentinel;
    return ivp_ok(ivp_array_push(arr, c));
}

static inline bool add_unique_pair(ivp_array_t *arr, ivp_friction_core_pair_t *p) {
    if (ivp_array_find(arr, p) >= 0) return true;
    return ivp_ok(ivp_array_push(arr, p));
}

/* Legacy parity: refresh sibling contacts in the start pair before entering
 * the cascade loop. Legacy recalculates friction/material state and may drop
 * broken contacts here; c17 currently has no direct "friction_is_broken"
 * delete path, so we refresh runtime coefficients only. */
static void precheck_start_pair(ivp_friction_core_pair_t *start_pair,
                                ivp_persistent_contact_t *start_contact)
{
    if (!start_pair) return;

    for (int ci = 0; ci < start_pair->contact_count; ci++) {
        ivp_persistent_contact_t *pc = start_pair->contacts[ci];
        if (!pc || pc == start_contact) continue;
        if (!pc->seen_this_psi) continue;
        if (!pc->cp.obj_a || !pc->cp.obj_b) continue;

        ivp_contact_combine_materials(&pc->cp);
        ivp_contact_compute_virtual_mass(&pc->cp);
    }
}

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

void ivp_impact_system_init(ivp_impact_system_t *sys,
                              const ivp_allocator_t *alloc)
{
    if (!sys) return;
    if (!alloc) alloc = ivp_default_allocator();
    memset(sys, 0, sizeof(*sys));
    sys->alloc = alloc;
    ivp_array_init(&sys->pushed_cores, alloc, 16);
    ivp_array_init(&sys->known_cores, alloc, 16);
    ivp_array_init(&sys->pairs, alloc, 16);
}

void ivp_impact_system_destroy(ivp_impact_system_t *sys)
{
    if (!sys) return;
    ivp_array_destroy(&sys->pushed_cores);
    ivp_array_destroy(&sys->known_cores);
    ivp_array_destroy(&sys->pairs);
    memset(sys, 0, sizeof(*sys));
}

/* ── Cascade solver ────────────────────────────────────────────────────── */

void ivp_impact_system_solve(ivp_impact_system_t *sys,
                               ivp_persistent_contact_t *start_contact,
                               ivp_friction_system_t *fs,
                               ivp_friction_core_pair_t *start_pair,
                               const ivp_collision_settings_t *settings)
{
    if (!sys || !start_contact || !settings) return;

    sys->sum_of_pushes = 0;
    sys->start_contact_impulse_sum = IVP_REAL(0.0);

    /* Reset arrays for this solve pass */
    ivp_array_clear(&sys->pushed_cores);
    ivp_array_clear(&sys->known_cores);
    ivp_array_clear(&sys->pairs);

    ivp_core_t *start_a = start_contact->cp.obj_a ? &start_contact->cp.obj_a->core : NULL;
    ivp_core_t *start_b = start_contact->cp.obj_b ? &start_contact->cp.obj_b->core : NULL;
    if (!start_a || !start_b) return;

    /* Seed cascade from the start contact cores even if start_pair is missing. */
    if (!add_unique_core(&sys->pushed_cores, start_a)) return;
    if (!add_unique_core(&sys->pushed_cores, start_b)) return;
    if (!add_unique_core(&sys->known_cores, start_a)) return;
    if (!add_unique_core(&sys->known_cores, start_b)) return;

    /* If caller did not provide the pair, discover it from friction system. */
    if (!start_pair && fs) {
        for (int fpi = 0; fpi < fs->pair_count; fpi++) {
            ivp_friction_core_pair_t *fp = &fs->pairs[fpi];
            if ((fp->cores[0] == start_a && fp->cores[1] == start_b) ||
                (fp->cores[0] == start_b && fp->cores[1] == start_a)) {
                start_pair = fp;
                break;
            }
        }
    }

    if (start_pair) {
        precheck_start_pair(start_pair, start_contact);
        if (!add_unique_pair(&sys->pairs, start_pair)) return;
    }

    /* Resolve the start contact first (standalone, pushes=0) */
    ivp_real_t imp = ivp_impact_resolve(&start_contact->cp, settings);
    if (imp > IVP_IMPACT_EPS) {
        sys->sum_of_pushes++;
        sys->start_contact_impulse_sum += imp;
    }

    /* Cascade loop */
    for (int iter = 0; iter < IVP_IMPACT_CASCADE_MAX_ITERS; iter++) {
        if (sys->sum_of_pushes >= IVP_MAXIMUM_SYSTEM_PUSH_NUMBER) break;
        int prev_pushed = ivp_array_len(&sys->pushed_cores);

        ivp_persistent_contact_t *worst_contact = NULL;
        ivp_real_t worst_closing_speed = IVP_SPEED_ADDON_SYSTEM_IMPACT;

        /* Legacy parity: select one worst impact candidate per iteration. */
        for (int pi = 0; pi < ivp_array_len(&sys->pairs); pi++) {
            ivp_friction_core_pair_t *pair =
                (ivp_friction_core_pair_t *)ivp_array_get(&sys->pairs, pi);

            for (int ci = 0; ci < pair->contact_count; ci++) {
                ivp_persistent_contact_t *pc = pair->contacts[ci];
                if (!pc) continue;
                if (!pc->seen_this_psi) continue;

                /* Recompute closing speed with updated velocities */
                ivp_real_t cs = ivp_contact_closing_speed(&pc->cp);
                if (cs > worst_closing_speed) {
                    worst_closing_speed = cs;
                    worst_contact = pc;
                }
            }
        }

        if (!worst_contact) break;

        /* Resolve selected impact with system-aware conservation.
         * More cascading pushes → higher effective elasticity
         * to prevent accumulated energy drain. */
        worst_contact->impacts_while_system++;
        ivp_real_t imp2 = ivp_impact_resolve_system(
            &worst_contact->cp, settings, worst_contact->impacts_while_system);
        if (imp2 > IVP_IMPACT_EPS) {
            sys->sum_of_pushes++;
            if (worst_contact == start_contact) {
                sys->start_contact_impulse_sum += imp2;
            }

            ivp_core_t *ca = worst_contact->cp.obj_a ? &worst_contact->cp.obj_a->core : NULL;
            ivp_core_t *cb = worst_contact->cp.obj_b ? &worst_contact->cp.obj_b->core : NULL;
            if (ca && cb) {
                if (!add_unique_core(&sys->pushed_cores, ca)) goto oom;
                if (!add_unique_core(&sys->pushed_cores, cb)) goto oom;
            }
        } else {
            break;
        }

        /* Expand: for each newly pushed core, add its friction pairs */
        if (fs) {
            for (int pi = prev_pushed; pi < ivp_array_len(&sys->pushed_cores); pi++) {
                ivp_core_t *core =
                    (ivp_core_t *)ivp_array_get(&sys->pushed_cores, pi);

                /* Find all pairs in the friction system involving this core */
                for (int fpi = 0; fpi < fs->pair_count; fpi++) {
                    ivp_friction_core_pair_t *fp = &fs->pairs[fpi];
                    if (fp->cores[0] == core || fp->cores[1] == core) {
                        if (!add_unique_pair(&sys->pairs, fp)) goto oom;

                        /* Add partner core to known list */
                        ivp_core_t *partner = (fp->cores[0] == core)
                            ? fp->cores[1] : fp->cores[0];
                        if (!add_unique_core(&sys->known_cores, partner)) goto oom;
                    }
                }
            }
        }
        continue;
oom:
        /* OOM during cascade expansion: stop iterating with current
         * (incomplete) tracking state rather than silently mis-accounting. */
        break;
    }

    /* B4: Record impact system pushes */
    {
        ivp_solver_metrics_t *met = ivp_solver_metrics_get_active();
        if (met) {
            met->impact_system_pushes += sys->sum_of_pushes;
        }
    }

    /* Clean up: reset old_sync_rot sentinel on all tracked cores */
    for (int i = 0; i < ivp_array_len(&sys->pushed_cores); i++) {
        ivp_core_t *c = (ivp_core_t *)ivp_array_get(&sys->pushed_cores, i);
        c->old_sync_rot = NULL;
    }
    for (int i = 0; i < ivp_array_len(&sys->known_cores); i++) {
        ivp_core_t *c = (ivp_core_t *)ivp_array_get(&sys->known_cores, i);
        c->old_sync_rot = NULL;
    }
}
