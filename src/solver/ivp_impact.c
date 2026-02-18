/* ivp_impact.c — Impact and friction resolution
 *
 * Iterative impulse-based collision response matching original IVP behavior:
 *   - Iterative push loop with friction-angle clamping (max IVP_IMPACT_MAX_PUSHES)
 *   - Turnaround detection using integral push direction
 *   - Energy conservation via fine-tuning phase (test push → interpolate)
 *   - Rescue push for safety gap
 *   - Coulomb friction (static/dynamic)
 *   - Distance keeper gap maintenance
 */
#include <ivp/solver/ivp_impact.h>
#include <ivp/solver/ivp_solver_metrics.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_vec3.h>
#include <string.h>

/* ── Impact resolution: effective elasticity ──────────────────────────── */

/* Compute effective elasticity from raw elasticity and
 * pushes_while_system count.
 *
 * Legacy formula (ivp_impact.cxx ~line 697):
 *   used_conservation = 1 - (1/(pushes*0.5+1)) * (1-e)
 *
 * For pushes_while_system = 0 (standalone):
 *   used_conservation = e
 *   give_back = sqrt(e)
 *
 * For pushes_while_system > 0 (cascade):
 *   used_conservation converges toward 1.0
 *   give_back = sqrt(used_conservation)
 */
static ivp_real_t ivp_impact_effective_elasticity(ivp_real_t raw_elasticity,
                                                    int pushes_while_system)
{
    ivp_real_t used_conservation;
    if (pushes_while_system <= 0) {
        used_conservation = raw_elasticity;
    } else {
        ivp_real_t scale = IVP_REAL(1.0) /
            ((ivp_real_t)pushes_while_system * IVP_INV_HALF_CONSERVATION_STEPS
             + IVP_REAL(1.0));
        used_conservation = IVP_REAL(1.0)
                          - scale * (IVP_REAL(1.0) - raw_elasticity);
    }
    return ivp_sqrt(used_conservation);
}

/* ── Iterative impact solver state ────────────────────────────────────── */

typedef struct {
    ivp_contact_point_t *cp;

    ivp_vec3r_t push_dir;         /* current normalized push direction       */
    ivp_vec3r_t integral_pushes;  /* accumulated push vector (turnaround)    */

    ivp_real_t cos_friction;      /* cos(arctan(friction_factor))            */
    ivp_real_t sin_friction;      /* sin(arctan(friction_factor))            */

    /* Saved core speeds for test-push undo */
    ivp_vec3r_t saved_speed_a;
    ivp_vec3r_t saved_speed_b;
    ivp_vec3f_t saved_rot_a;
    ivp_vec3f_t saved_rot_b;
    ivp_vec3r_t saved_integral;
} impact_work_t;

/* ── Helpers ──────────────────────────────────────────────────────────── */

/* Save core speeds (for test-push undo). */
static void save_speeds(impact_work_t *w)
{
    w->saved_speed_a  = w->cp->obj_a->core.speed;
    w->saved_rot_a    = w->cp->obj_a->core.rot_speed;
    w->saved_speed_b  = w->cp->obj_b->core.speed;
    w->saved_rot_b    = w->cp->obj_b->core.rot_speed;
    w->saved_integral = w->integral_pushes;
}

/* Restore saved core speeds (undo test push). */
static void restore_speeds(impact_work_t *w)
{
    w->cp->obj_a->core.speed     = w->saved_speed_a;
    w->cp->obj_a->core.rot_speed = w->saved_rot_a;
    w->cp->obj_b->core.speed     = w->saved_speed_b;
    w->cp->obj_b->core.rot_speed = w->saved_rot_b;
    w->integral_pushes            = w->saved_integral;
}

/* Initialise friction-cone parameters from material properties.
 *
 * Legacy (ivp_impact.cxx ~line 1269):
 *   impact_fri_fact = friction * (1 + sqrt(elasticity))
 *   fri_angle       = atan(impact_fri_fact)
 *   cos_friction    = cos(fri_angle)
 *   sin_friction    = sin(fri_angle)
 *
 * We compute cos/sin without trig:
 *   cos(atan(x)) = 1/sqrt(1+x^2)
 *   sin(atan(x)) = x/sqrt(1+x^2)
 */
static void init_friction_cone(impact_work_t *w, ivp_real_t elasticity)
{
    ivp_real_t fri = w->cp->friction
                   * (IVP_REAL(1.0) + ivp_sqrt(elasticity));
    ivp_real_t inv_hyp = IVP_REAL(1.0) / ivp_sqrt(IVP_REAL(1.0) + fri * fri);
    w->cos_friction = inv_hyp;
    w->sin_friction = fri * inv_hyp;
}

/* Update push direction from current relative velocity.
 *
 * Implements turnaround detection and friction-cone clipping
 * (legacy get_world_push_direction, ivp_impact.cxx lines 335-406).
 *
 * C17 convention:
 *   normal   = A→B direction
 *   push_dir · normal > 0  ⇒  approaching (correct)
 *   push_dir · normal < 0  ⇒  turnaround (use integral direction)
 */
static void update_push_direction(impact_work_t *w)
{
    ivp_vec3r_t v_rel;
    ivp_contact_relative_velocity(w->cp, &v_rel);

    ivp_real_t v_len = ivp_vec3r_length(&v_rel);
    if (v_len < IVP_REAL_EPS) return; /* keep current direction */

    /* push_dir = −normalize(v_rel)
     * v_rel = v_b − v_a; for approaching objects v_rel · normal < 0
     * so −v_rel · normal > 0 → push_dir ≈ normal */
    ivp_real_t inv_len = IVP_REAL(1.0) / v_len;
    w->push_dir.k[0] = -v_rel.k[0] * inv_len;
    w->push_dir.k[1] = -v_rel.k[1] * inv_len;
    w->push_dir.k[2] = -v_rel.k[2] * inv_len;

    ivp_real_t angle = ivp_vec3r_dot(&w->push_dir, &w->cp->normal);

    /* Turnaround: objects now separating → use integral push direction */
    if (angle < IVP_REAL(0.0)) {
        ivp_real_t int_len = ivp_vec3r_length(&w->integral_pushes);
        if (int_len > IVP_REAL_EPS) {
            ivp_real_t inv_il = IVP_REAL(1.0) / int_len;
            w->push_dir.k[0] = w->integral_pushes.k[0] * inv_il;
            w->push_dir.k[1] = w->integral_pushes.k[1] * inv_il;
            w->push_dir.k[2] = w->integral_pushes.k[2] * inv_il;
        } else {
            w->push_dir = w->cp->normal;
        }
        return;
    }

    /* Friction-cone clipping: if push_dir is too tangential,
     * project onto the cone boundary.
     *
     * The cone is centred on +normal with half-angle α where
     * cos(α) = cos_friction.  Outside the cone: angle < cos_friction.
     */
    if (w->cos_friction > IVP_REAL_EPS && angle < w->cos_friction) {
        /* Tangential component of push_dir */
        ivp_vec3r_t tangent;
        tangent.k[0] = w->push_dir.k[0] - angle * w->cp->normal.k[0];
        tangent.k[1] = w->push_dir.k[1] - angle * w->cp->normal.k[1];
        tangent.k[2] = w->push_dir.k[2] - angle * w->cp->normal.k[2];

        ivp_real_t tan_len = ivp_vec3r_length(&tangent);
        if (tan_len > IVP_REAL_EPS) {
            ivp_real_t inv_tl = IVP_REAL(1.0) / tan_len;
            /* Clamp to cone boundary:
             * push_dir = cos_friction·normal + sin_friction·tangent_hat */
            w->push_dir.k[0] = w->cos_friction * w->cp->normal.k[0]
                              + w->sin_friction * tangent.k[0] * inv_tl;
            w->push_dir.k[1] = w->cos_friction * w->cp->normal.k[1]
                              + w->sin_friction * tangent.k[1] * inv_tl;
            w->push_dir.k[2] = w->cos_friction * w->cp->normal.k[2]
                              + w->sin_friction * tangent.k[2] * inv_tl;
        }
        /* else: purely normal — already inside cone */
    }
}

/* Apply impulse of given magnitude along current push_dir. */
static void do_push(impact_work_t *w, ivp_real_t impulse)
{
    ivp_contact_apply_impulse_dir(w->cp, &w->push_dir, impulse);

    /* Accumulate integral pushes for turnaround detection */
    w->integral_pushes.k[0] += w->push_dir.k[0] * impulse;
    w->integral_pushes.k[1] += w->push_dir.k[1] * impulse;
    w->integral_pushes.k[2] += w->push_dir.k[2] * impulse;
}

/* ── Main iterative impact resolution ─────────────────────────────────── */

/* Matches legacy do_impact() (ivp_impact.cxx lines 680-913):
 *
 * 1. Estimate initial impulse (10 % of full elastic exchange)
 * 2. Iterative loop: push → recompute relative speed → update push
 *    direction (friction-cone clip / turnaround) until virtual_speed ≤ 0
 * 3. Fine-tuning phase: test push of 1.0, measure effect, interpolate
 *    to exact give_back_speed for energy conservation
 * 4. Rescue push if gap < real_coll_dist
 */
static ivp_real_t ivp_impact_resolve_iterative(ivp_contact_point_t *cp,
                                                 const ivp_collision_settings_t *settings,
                                                 int pushes_while_system)
{
    /* Only resolve if objects are actually approaching */
    ivp_real_t closing_speed = ivp_contact_closing_speed(cp);
    if (closing_speed <= IVP_IMPACT_EPS) {
        return IVP_REAL(0.0);
    }

    /* ── Setup ─────────────────────────────────────────────────────── */

    impact_work_t w;
    memset(&w, 0, sizeof(w));
    w.cp       = cp;
    w.push_dir = cp->normal; /* start pushing along contact normal */

    /* Effective elasticity (already sqrt'd) */
    ivp_real_t effective_e =
        ivp_impact_effective_elasticity(cp->elasticity, pushes_while_system);

    /* Friction-cone parameters */
    init_friction_cone(&w, cp->elasticity);

    /* Give-back speed = how fast the objects should be separating after
     * the impact.  effective_e = sqrt(used_conservation), so
     * give_back_speed = closing_speed * sqrt(used_conservation). */
    ivp_real_t virtual_speed   = closing_speed;
    ivp_real_t give_back_speed = virtual_speed * effective_e;

    /* System addon for cascaded impacts */
    if (pushes_while_system > 0) {
        give_back_speed += IVP_SPEED_ADDON_SYSTEM_IMPACT
                         * ivp_sqrt((ivp_real_t)pushes_while_system);
    }

    /* Rescue speed target for minimum separation velocity.
     * Only active when objects are deeply penetrating (gap < real_coll_dist). */
    ivp_real_t rescue_speed = IVP_REAL(0.0);
    if (cp->gap_length < settings->real_coll_dist) {
        ivp_real_t rescue_depth = settings->real_coll_dist - cp->gap_length;
        rescue_speed = rescue_depth * IVP_SAFETY_FACTOR_FOR_DELAY;
    }

    /* ── Initial impulse estimate ──────────────────────────────────── */

    /* Full elastic exchange impulse = 2 · vm_reduced · cs.
     * We apply IVP_IMPACT_INITIAL_FRACTION (10 %) per iteration. */
    ivp_real_t impulse = IVP_REAL(2.0) * cp->virtual_mass * closing_speed
                       * IVP_IMPACT_INITIAL_FRACTION;

    if (impulse < IVP_REAL_EPS) {
        /* Negligible impulse — bail out with rescue only */
        if (rescue_speed > IVP_REAL(0.0)) {
            ivp_real_t ri = rescue_speed * cp->virtual_mass;
            ivp_contact_apply_impulse(cp, ri);
            return ri;
        }
        return IVP_REAL(0.0);
    }

    /* ── Phase 1: iterative zeroing loop ───────────────────────────── */

    int push_count = 0;
    while (virtual_speed > IVP_REAL(0.0) && push_count < IVP_IMPACT_MAX_PUSHES) {
        push_count++;
        do_push(&w, impulse);

        /* Recompute closing speed along contact normal */
        virtual_speed = ivp_contact_closing_speed(cp);

        /* Update push direction (friction-cone clip + turnaround) */
        update_push_direction(&w);
    }

    /* ── Phase 2: fine-tuning for energy conservation ──────────────── */

    /* After zeroing, virtual_speed is zero or slightly negative (overshoot).
     * We still need to push objects apart by give_back_speed. */
    give_back_speed += virtual_speed; /* subtract overshoot */

    if (give_back_speed > IVP_REAL(0.0)
        && push_count < IVP_IMPACT_MAX_PUSHES) {
        /* Legacy parity: fine-tuning "give-back" impulse is along the
         * contact normal, not the last friction-clipped direction.
         * Using a tangential direction here injects energy in stacks. */
        w.push_dir = cp->normal;

        /* Save state for test-push undo */
        save_speeds(&w);

        /* Test push with unit impulse to measure the effect */
        do_push(&w, IVP_REAL(1.0));
        ivp_real_t vs_after_test = ivp_contact_closing_speed(cp);

        /* speed_diff = how much closing speed changes per unit impulse
         * (negative, since push reduces closing speed) */
        ivp_real_t speed_diff = virtual_speed - vs_after_test;

        /* Undo test push */
        restore_speeds(&w);

        if (ivp_fabs(speed_diff) > IVP_IMPACT_EPS) {
            /* Scale to achieve exactly give_back_speed of separation */
            ivp_real_t target_push = give_back_speed / speed_diff;
            if (target_push > IVP_REAL(0.0)) {
                do_push(&w, target_push);
            }
        }
    }

    /* ── Phase 3: rescue push for safety gap ───────────────────────── */

    if (rescue_speed > IVP_REAL(0.0)) {
        /* Compute current separation speed */
        ivp_real_t current_sep = -ivp_contact_closing_speed(cp);
        /* current_sep > 0 means separating */

        ivp_real_t needed = rescue_speed - current_sep;
        if (needed > IVP_REAL(0.0)) {
            ivp_real_t ri = needed * cp->virtual_mass;
            ivp_contact_apply_impulse(cp, ri);
            w.integral_pushes.k[0] += cp->normal.k[0] * ri;
            w.integral_pushes.k[1] += cp->normal.k[1] * ri;
            w.integral_pushes.k[2] += cp->normal.k[2] * ri;
        }
    }

    /* B3: Record impact push count */
    {
        ivp_solver_metrics_t *met = ivp_solver_metrics_get_active();
        if (met) {
            met->impact_push_count += push_count;
        }
    }

    return ivp_vec3r_length(&w.integral_pushes);
}

/* ── Public API ────────────────────────────────────────────────────────── */

ivp_real_t ivp_impact_resolve(ivp_contact_point_t *cp,
                               const ivp_collision_settings_t *settings)
{
    return ivp_impact_resolve_iterative(cp, settings, 0);
}

ivp_real_t ivp_impact_resolve_system(ivp_contact_point_t *cp,
                                       const ivp_collision_settings_t *settings,
                                       int pushes_while_system)
{
    return ivp_impact_resolve_iterative(cp, settings, pushes_while_system);
}

/* ── Friction resolution ──────────────────────────────────────────────── */

ivp_real_t ivp_friction_resolve(ivp_contact_point_t *cp,
                                 const ivp_collision_settings_t *settings,
                                 ivp_real_t delta_time)
{
    if (cp->friction < IVP_REAL_EPS) {
        return IVP_REAL(0.0);
    }

    /* Compute relative velocity at contact */
    ivp_vec3r_t v_rel;
    ivp_contact_relative_velocity(cp, &v_rel);

    /* Tangential velocity = v_rel - (v_rel · n) * n */
    ivp_real_t vn = ivp_vec3r_dot(&v_rel, &cp->normal);
    ivp_vec3r_t v_tangential;
    v_tangential.k[0] = v_rel.k[0] - vn * cp->normal.k[0];
    v_tangential.k[1] = v_rel.k[1] - vn * cp->normal.k[1];
    v_tangential.k[2] = v_rel.k[2] - vn * cp->normal.k[2];

    ivp_real_t v_tan_len = ivp_vec3r_length(&v_tangential);
    if (v_tan_len < IVP_MINIMAL_REAL_FRICTION_LEN) {
        return IVP_REAL(0.0);
    }

    /* Friction direction = -normalize(v_tangential) */
    ivp_vec3r_t fri_dir;
    ivp_real_t inv_v_tan = IVP_REAL(1.0) / v_tan_len;
    fri_dir.k[0] = -v_tangential.k[0] * inv_v_tan;
    fri_dir.k[1] = -v_tangential.k[1] * inv_v_tan;
    fri_dir.k[2] = -v_tangential.k[2] * inv_v_tan;

    /* Compute virtual mass along tangent direction */
    ivp_real_t vm_a = ivp_calc_virtual_mass(&cp->obj_a->core, &cp->r_a, &fri_dir);
    ivp_real_t vm_b = ivp_calc_virtual_mass(&cp->obj_b->core, &cp->r_b, &fri_dir);
    ivp_real_t vm_tan;

    if (cp->obj_a->core.physical_unmoveable) {
        vm_tan = vm_b;
    } else if (cp->obj_b->core.physical_unmoveable) {
        vm_tan = vm_a;
    } else {
        vm_tan = (vm_a * vm_b) / (vm_a + vm_b);
    }

    /* Impulse to zero tangential velocity */
    ivp_real_t desired_impulse = v_tan_len * vm_tan;

    /* Coulomb friction limit: J_friction ≤ μ * J_normal */
    ivp_real_t normal_impulse_estimate;
    ivp_real_t cs = ivp_fabs(ivp_contact_closing_speed(cp));
    if (cs > IVP_REAL_EPS) {
        normal_impulse_estimate = cs * cp->virtual_mass;
    } else {
        normal_impulse_estimate = cp->virtual_mass *
            settings->gravity_magnitude * delta_time;
    }

    ivp_real_t max_friction_impulse = cp->friction * normal_impulse_estimate;

    ivp_real_t actual_impulse;
    if (desired_impulse <= max_friction_impulse) {
        actual_impulse = desired_impulse; /* static friction */
    } else {
        actual_impulse = max_friction_impulse; /* dynamic friction */
    }

    ivp_contact_apply_impulse_dir(cp, &fri_dir, actual_impulse);
    return actual_impulse;
}

/* ── Distance keeper ──────────────────────────────────────────────────── */

void ivp_distance_keeper_push(ivp_contact_point_t *cp,
                               const ivp_collision_settings_t *settings,
                               ivp_real_t delta_time)
{
    /* Only active for nearby contacts */
    if (cp->gap_length > settings->keeper_dist) {
        return;
    }

    ivp_real_t closing_speed = ivp_contact_closing_speed(cp);
    ivp_real_t gap_diff = settings->keeper_dist - cp->gap_length;

    /* Speedup ramps up over IVP_SLOWLY_TURN_ON_KEEPER PSIs.
     * Legacy formula: speedup = age_frac * (1.0 / dt).
     * For dt=1/60 this gives ~60 at full ramp-up. */
    ivp_real_t age_frac = (ivp_real_t)cp->age_psi
                        / (ivp_real_t)IVP_SLOWLY_TURN_ON_KEEPER;
    if (age_frac > IVP_REAL(1.0)) age_frac = IVP_REAL(1.0);

    ivp_real_t speedup = age_frac / delta_time;

    /* For overlapping contacts use descending speedup */
    if (gap_diff > settings->keeper_dist) {
        speedup = IVP_DESCENDING_SPEEDUP;
    }

    ivp_real_t desired_push_speed = closing_speed + gap_diff * speedup;

    if (desired_push_speed > IVP_REAL(0.0)) {
        ivp_real_t impulse = desired_push_speed * cp->virtual_mass;
        ivp_contact_apply_impulse(cp, impulse);
    }
}
