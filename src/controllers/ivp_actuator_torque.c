/* ivp_actuator_torque.c -- Pure torque actuator implementation
 *
 * Port of IVP_Actuator_Torque (ivp_controller/ivp_actuator.cxx:667-809).
 *
 *   rot_speed_out = omega_cs . axis_cs
 *   if |rot_speed_out| > max_rotation_speed: skip (both directions)
 *   rot_speed_change += (axis_cs (.) inv_I) * torque * dt
 */
#include <ivp/controllers/ivp_actuator_torque.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_mat3.h>

#include <string.h>
#include "controllers/ivp_controller_ref.h"

/* IVP_Actuator_Two_Point::ensure_actuator_in_simulation (both anchors of a
 * torque actuator are attached to one object) */
static void torque_ensure_actuator_in_simulation(ivp_object_t *obj)
{
    if (obj && obj->owner) ivp_environment_ensure_actuator_in_simulation(obj->owner, &obj, 1);
}

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_torque_actuator_state_init(ivp_torque_actuator_state_t *state)
{
    /* Reference IVP_Template_Torque: torque 0, max_rotation_speed 0. */
    memset(state, 0, sizeof(*state));
    state->active = true;
}

void ivp_torque_actuator_state_setup(ivp_torque_actuator_state_t *state,
                                      ivp_object_t *obj,
                                      const ivp_vec3r_t *axis_os,
                                      ivp_real_t torque,
                                      ivp_real_t max_rotation_speed)
{
    memset(state, 0, sizeof(*state));
    state->obj = obj;
    /* Reference constructor: axis_in_core_coord_system (IVP_U_Float_Point) =
     * anchor1.core_pos - anchor0.core_pos, normized once.  The object space
     * axis is rotated into core space (calc_m_core_f_object). */
    {
        ivp_transform_r_t m_core_f_object;
        ivp_ctl_calc_m_core_f_object(obj, &m_core_f_object);
        ivp_vec3f_t a = ivp_ctl_f(axis_os), axis_cs;
        lvm_vmult3_f(&m_core_f_object, &a, &axis_cs);
        (void)lvf_normize(&axis_cs);
        state->axis_os = ivp_ctl_r(&axis_cs);
    }
    state->torque = (ivp_real_t)(float)torque;                         /* IVP_FLOAT */
    state->max_rotation_speed = (ivp_real_t)(float)max_rotation_speed; /* IVP_FLOAT */
    state->active = true;
    /* Reference constructor: if (torque) ensure_actuator_in_simulation(). */
    if ((float)state->torque != 0.0f) torque_ensure_actuator_in_simulation(obj);
}

void ivp_torque_actuator_state_setup_anchors(ivp_torque_actuator_state_t *state,
                                             ivp_object_t *obj,
                                             const ivp_vec3r_t *anchor0_os,
                                             const ivp_vec3r_t *anchor1_os,
                                             ivp_real_t torque,
                                             ivp_real_t max_rotation_speed)
{
    memset(state, 0, sizeof(*state));
    state->obj = obj;
    /* IVP_Actuator_Torque constructor: axis_in_core_coord_system.subtract(
     * &anchor1->core_pos, &anchor0->core_pos); normize() */
    {
        ivp_environment_t *env = obj ? obj->owner : NULL;
        ivp_vec3r_t cp0, cp1;
        ivp_ctl_anchor_setup_os(obj, env, anchor0_os, &cp0, NULL);
        ivp_ctl_anchor_setup_os(obj, env, anchor1_os, &cp1, NULL);
        ivp_vec3f_t c0 = ivp_ctl_f(&cp0), c1 = ivp_ctl_f(&cp1), axis_cs;
        lvf_subtract2(&axis_cs, &c1, &c0);
        (void)lvf_normize(&axis_cs);
        state->axis_os = ivp_ctl_r(&axis_cs);
    }
    state->torque = (ivp_real_t)(float)torque;                         /* IVP_FLOAT */
    state->max_rotation_speed = (ivp_real_t)(float)max_rotation_speed; /* IVP_FLOAT */
    state->active = true;
    if ((float)state->torque != 0.0f) torque_ensure_actuator_in_simulation(obj);
}

void ivp_torque_actuator_set_torque(ivp_torque_actuator_state_t *state,
                                     ivp_real_t torque)
{
    /* set_torque(IVP_DOUBLE): IVP_FLOAT member */
    if ((double)(float)state->torque == (double)torque) return;
    state->torque = (ivp_real_t)(float)torque;
    torque_ensure_actuator_in_simulation(state->obj);
}

void ivp_torque_actuator_set_max_rotation_speed(ivp_torque_actuator_state_t *state,
                                                 ivp_real_t max_speed)
{
    /* set_max_rotation_speed(IVP_DOUBLE): IVP_FLOAT member */
    if ((double)(float)state->max_rotation_speed == (double)max_speed) return;
    state->max_rotation_speed = (ivp_real_t)(float)max_speed;
    torque_ensure_actuator_in_simulation(state->obj);
}

/* ── IVP_Actuator_Torque_Active ─────────────────────────────────────────── */

/* Reference IVP_Actuator_Torque_Active::active_float_changed. */
static void torque_active_float_changed(void *ctx, ivp_active_float_t *af,
                                        double old_value, double new_value)
{
    (void)old_value;
    (void)new_value;
    ivp_torque_actuator_state_t *state = (ivp_torque_actuator_state_t *)ctx;
    if (af == state->active_float_max_rotation_speed) {
        ivp_torque_actuator_set_max_rotation_speed(state, (ivp_real_t)ivp_active_float_get_float(af));
    }
    if (af == state->active_float_torque) {
        ivp_torque_actuator_set_torque(state, (ivp_real_t)ivp_active_float_get_float(af));
    }
}

static void torque_release(ivp_torque_actuator_state_t *state, ivp_active_float_t **slot)
{
    ivp_active_float_t *af = *slot;
    if (!af) return;
    *slot = NULL;
    (void)ivp_active_float_remove_listener(af, torque_active_float_changed, state);
}

static void torque_bind_one(ivp_torque_actuator_state_t *state, ivp_active_float_t **slot,
                            ivp_active_float_t *af)
{
    if (!af) return;
    if (!ivp_active_float_add_listener(af, torque_active_float_changed, state)) return;
    *slot = af;
    if (slot == &state->active_float_max_rotation_speed) {
        ivp_torque_actuator_set_max_rotation_speed(state, (ivp_real_t)ivp_active_float_get_float(af));
    } else {
        ivp_torque_actuator_set_torque(state, (ivp_real_t)ivp_active_float_get_float(af));
    }
}

void ivp_torque_actuator_bind_active(ivp_torque_actuator_state_t *state,
                                     ivp_active_float_t *active_float_torque,
                                     ivp_active_float_t *active_float_max_rotation_speed)
{
    if (!state) return;
    torque_release(state, &state->active_float_max_rotation_speed);
    torque_release(state, &state->active_float_torque);
    /* constructor order: max_rotation_speed, then torque */
    torque_bind_one(state, &state->active_float_max_rotation_speed, active_float_max_rotation_speed);
    torque_bind_one(state, &state->active_float_torque, active_float_torque);
}

void ivp_torque_actuator_set_rotation_speed_out(ivp_torque_actuator_state_t *state,
                                                ivp_active_float_t *terminal)
{
    if (!state || state->active_float_rotation_speed_out == terminal) return;
    if (terminal) ivp_active_float_add_ref(terminal);
    ivp_active_float_t *old = state->active_float_rotation_speed_out;
    state->active_float_rotation_speed_out = terminal;
    if (old) ivp_active_float_remove_ref(old);
}

void ivp_torque_actuator_unbind_active(ivp_torque_actuator_state_t *state)
{
    if (!state) return;
    /* ~IVP_Actuator_Torque_Active, then ~IVP_Actuator_Torque */
    torque_release(state, &state->active_float_max_rotation_speed);
    torque_release(state, &state->active_float_torque);
    ivp_torque_actuator_set_rotation_speed_out(state, NULL);
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_torque_actuator(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx)
{
    ivp_torque_actuator_state_t *state = (ivp_torque_actuator_state_t *)ctx;
    if (!state->active || !state->obj) return;
    if (obj != state->obj) return;

    /* Reference IVP_Actuator_Torque::do_simulation_controller
     * (ivp_actuator.cxx:762-801). */
    /* clear if not updated */
    state->current_speed = IVP_REAL(0.0);
    if ((float)state->torque == 0.0f) return;

    ivp_core_t *core = state->obj->physical_core;
    if (core->physical_unmoveable) return;
    if (!ivp_controller_core_is_simulated(core) || core->pinned) return; /* only rotate simulated cores */

    const ivp_vec3f_t axis = ivp_ctl_f(&state->axis_os); /* axis_in_core_coord_system */

    const double domega = ivp_ctl_get_rot_speed_cs(core, &axis);
    const float rot_speed_out = (float)domega; /* memorize for sound e.g. */
    state->current_speed = (ivp_real_t)rot_speed_out;
    if (state->active_float_rotation_speed_out) {
        ivp_active_float_set(state->active_float_rotation_speed_out, (double)rot_speed_out);
    }

    if (fabs(domega) > (double)(float)state->max_rotation_speed) return; /* too fast */

    const double rot_imp = (double)(float)state->torque * (double)event_sim->delta_time;
    ivp_ctl_async_rot_push_core_multiple_cs(core, &axis, rot_imp);
}

int ivp_torque_actuator_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_torque_actuator_state_t *s = (const ivp_torque_actuator_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[1] = { s->obj };
    return ivp_controller_discover_objects(objs, 1, out_handles, max_handles);
}
