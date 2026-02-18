/* ivp_actuator_stabilizer.c -- Anti-roll bar stabilizer implementation
 *
 * 4 anchor points as 2 pairs. Applies force proportional to
 * (dist[1] - dist[0]) * stabi_constant to equalize pair distances.
 */

#include <ivp/controllers/ivp_actuator_stabilizer.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_stabilizer_state_init(ivp_stabilizer_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->active         = true;
    state->stabi_constant = IVP_REAL(0.0);
}

void ivp_stabilizer_state_setup(ivp_stabilizer_state_t *state,
                                 ivp_object_t *obj0,
                                 ivp_object_t *obj1,
                                 ivp_object_t *obj2,
                                 ivp_object_t *obj3,
                                 const ivp_vec3r_t *anchor0,
                                 const ivp_vec3r_t *anchor1,
                                 const ivp_vec3r_t *anchor2,
                                 const ivp_vec3r_t *anchor3,
                                 ivp_real_t stabi_constant)
{
    state->obj[0] = obj0;
    state->obj[1] = obj1;
    state->obj[2] = obj2;
    state->obj[3] = obj3;
    state->anchor_os[0] = *anchor0;
    state->anchor_os[1] = *anchor1;
    state->anchor_os[2] = *anchor2;
    state->anchor_os[3] = *anchor3;
    state->stabi_constant = stabi_constant;
}

/* ── Runtime adjustment ────────────────────────────────────────────────── */

void ivp_stabilizer_set_constant(ivp_stabilizer_state_t *state,
                                  ivp_real_t value)
{
    state->stabi_constant = value;
}

/* ── Controller callback ───────────────────────────────────────────────── */

void ivp_controller_stabilizer(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;

    ivp_stabilizer_state_t *state = (ivp_stabilizer_state_t *)ctx;
    if (!state || !state->active) return;

    /* Execute-once guard: select first dynamic object among all 4 */
    ivp_object_t *exec_obj = NULL;
    for (int i = 0; i < 4; i++) {
        if (state->obj[i] && !state->obj[i]->core.physical_unmoveable) {
            exec_obj = state->obj[i];
            break;
        }
    }
    if (!exec_obj || obj != exec_obj) return;

    /* Get all 4 anchor world positions */
    ivp_vec3r_t pos_ws[4];
    for (int i = 0; i < 4; i++) {
        ivp_controller_get_anchor_ws(state->obj[i], &state->anchor_os[i], &pos_ws[i]);
    }

    /* Compute direction and distance for each pair */
    ivp_vec3r_t dir[2];
    ivp_real_t dist[2];

    for (int p = 0; p < 2; p++) {
        int i0 = p * 2;
        int i1 = p * 2 + 1;
        ivp_vec3r_sub(&dir[p], &pos_ws[i0], &pos_ws[i1]);
        dist[p] = ivp_vec3r_length(&dir[p]);
        if (dist[p] > IVP_REAL_EPS) {
            ivp_real_t inv_len = IVP_REAL(1.0) / dist[p];
            ivp_vec3r_scale(&dir[p], &dir[p], inv_len);
        }
    }

    ivp_real_t diff = dist[1] - dist[0];
    state->current_diff_dist = diff;

    ivp_real_t force_mag = diff * state->stabi_constant;
    if (ivp_fabs(force_mag) < IVP_REAL_EPS) return;

    /* Apply force to pair 0: push apart/together along dir[0]
     * to increase/decrease dist[0] toward equilibrium.
     * Positive diff means dist[1] > dist[0], so increase dist[0]:
     *   obj[0] gets force along +dir[0], obj[1] gets force along -dir[0].
     * Apply opposite to pair 1: decrease dist[1]:
     *   obj[2] gets force along -dir[1], obj[3] gets force along +dir[1]. */
    {
        ivp_vec3r_t f0, f1;
        ivp_vec3r_scale(&f0, &dir[0], force_mag);
        ivp_vec3r_scale(&f1, &dir[0], -force_mag);
        ivp_controller_apply_force_at_anchor(state->obj[0], &pos_ws[0], &f0, dt);
        ivp_controller_apply_force_at_anchor(state->obj[1], &pos_ws[1], &f1, dt);
    }
    {
        ivp_vec3r_t f2, f3;
        ivp_vec3r_scale(&f2, &dir[1], -force_mag);
        ivp_vec3r_scale(&f3, &dir[1], force_mag);
        ivp_controller_apply_force_at_anchor(state->obj[2], &pos_ws[2], &f2, dt);
        ivp_controller_apply_force_at_anchor(state->obj[3], &pos_ws[3], &f3, dt);
    }
}
