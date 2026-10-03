/* ivp_actuator_rot_mot.c -- Rotation motor actuator implementation
 *
 * Port of IVP_Actuator_Rot_Mot::do_simulation_controller
 * (ivp_controller/ivp_actuator.cxx:597-665).
 */
#include <ivp/controllers/ivp_actuator_rot_mot.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_mat3.h>

#include <string.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include "controllers/ivp_controller_ref.h"

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_rot_mot_state_init(ivp_rot_mot_state_t *state)
{
    /* Reference IVP_Template_Rot_Mot: max_rotation_speed = power = max_torque = 0 */
    memset(state, 0, sizeof(*state));
    state->active = true;
}

void ivp_rot_mot_state_setup(ivp_rot_mot_state_t *state,
                              ivp_object_t *obj_a,
                              const ivp_vec3r_t *anchor_a_os,
                              ivp_object_t *obj_b,
                              const ivp_vec3r_t *anchor_b_os,
                              ivp_real_t power,
                              ivp_real_t max_torque,
                              ivp_real_t max_rotation_speed)
{
    memset(state, 0, sizeof(*state));
    state->obj_a = obj_a;
    state->obj_b = obj_b;
    /* IVP_Template_Anchor::set_anchor_position_os + IVP_Anchor::init_anchor;
     * a NULL anchor object stands for the driven object. */
    ivp_object_t *o0 = obj_a ? obj_a : obj_b;
    ivp_object_t *o1 = obj_b ? obj_b : obj_a;
    ivp_environment_t *env = ivp_ctl_env_of2(o0, o1);
    ivp_ctl_anchor_setup_os(o0, env, anchor_a_os, &state->anchor_a_os);
    ivp_ctl_anchor_setup_os(o1, env, anchor_b_os, &state->anchor_b_os);
    /* IVP_Actuator_Rot_Mot constructor (ivp_actuator.cxx:514) */
    if (o0 && o1 && o0->original_core != o1->original_core) {
        ivp_debug_printf("Rot Mot Actuation has to be fixed on one object\n");
    }
    /* IVP_FLOAT members */
    state->power = (ivp_real_t)(float)power;
    state->max_torque = (ivp_real_t)(float)max_torque;
    state->max_rotation_speed = (ivp_real_t)(float)max_rotation_speed;
    state->active = true;
}

static ivp_object_t *rot_mot_driven(const ivp_rot_mot_state_t *state)
{
    return state->obj_a ? state->obj_a : state->obj_b;
}

static void rot_mot_wake(ivp_rot_mot_state_t *state)
{
    ivp_controller_wake_actuator(state->obj_a, state->obj_b);
}

/* Reference setters (IVP_DOUBLE argument, IVP_FLOAT member) return early on
 * equal values, else wake the actuator. */
void ivp_rot_mot_set_power(ivp_rot_mot_state_t *state, ivp_real_t power)
{
    if ((double)(float)state->power == (double)power) return;
    state->power = (ivp_real_t)(float)power;
    rot_mot_wake(state);
}

void ivp_rot_mot_set_max_torque(ivp_rot_mot_state_t *state, ivp_real_t max_torque)
{
    if ((double)(float)state->max_torque == (double)max_torque) return;
    state->max_torque = (ivp_real_t)(float)max_torque;
    rot_mot_wake(state);
}

void ivp_rot_mot_set_max_rotation_speed(ivp_rot_mot_state_t *state, ivp_real_t max_speed)
{
    if ((double)(float)state->max_rotation_speed == (double)max_speed) return;
    state->max_rotation_speed = (ivp_real_t)(float)max_speed;
    rot_mot_wake(state);
}

/* ── IVP_Actuator_Rot_Mot_Active ────────────────────────────────────────── */

/* Reference IVP_Actuator_Rot_Mot_Active::active_float_changed. */
static void rot_mot_active_float_changed(void *ctx, ivp_active_float_t *af,
                                         double old_value, double new_value)
{
    (void)old_value;
    (void)new_value;
    ivp_rot_mot_state_t *state = (ivp_rot_mot_state_t *)ctx;
    if (af == state->active_float_max_rotation_speed)
        ivp_rot_mot_set_max_rotation_speed(state, (ivp_real_t)ivp_active_float_get_float(af));
    if (af == state->active_float_power)
        ivp_rot_mot_set_power(state, (ivp_real_t)ivp_active_float_get_float(af));
    if (af == state->active_float_max_torque)
        ivp_rot_mot_set_max_torque(state, (ivp_real_t)ivp_active_float_get_float(af));
}

static void rot_mot_release(ivp_rot_mot_state_t *state, ivp_active_float_t **slot)
{
    ivp_active_float_t *af = *slot;
    if (!af) return;
    *slot = NULL;
    (void)ivp_active_float_remove_listener(af, rot_mot_active_float_changed, state);
}

static bool rot_mot_add(ivp_rot_mot_state_t *state, ivp_active_float_t **slot, ivp_active_float_t *af)
{
    if (!af) return false;
    if (!ivp_active_float_add_listener(af, rot_mot_active_float_changed, state)) return false;
    *slot = af;
    return true;
}

void ivp_rot_mot_bind_active(ivp_rot_mot_state_t *state,
                             ivp_active_float_t *active_float_max_rotation_speed,
                             ivp_active_float_t *active_float_power,
                             ivp_active_float_t *active_float_max_torque)
{
    if (!state) return;
    rot_mot_release(state, &state->active_float_max_rotation_speed);
    rot_mot_release(state, &state->active_float_power);
    rot_mot_release(state, &state->active_float_max_torque);
    /* IVP_Actuator_Rot_Mot_Active constructor order */
    if (rot_mot_add(state, &state->active_float_max_rotation_speed, active_float_max_rotation_speed)) {
        ivp_rot_mot_set_max_rotation_speed(state,
            (ivp_real_t)ivp_active_float_get_float(active_float_max_rotation_speed));
    }
    if (rot_mot_add(state, &state->active_float_power, active_float_power)) {
        ivp_rot_mot_set_power(state, (ivp_real_t)ivp_active_float_get_float(active_float_power));
    }
    if (rot_mot_add(state, &state->active_float_max_torque, active_float_max_torque)) {
        ivp_rot_mot_set_max_torque(state, (ivp_real_t)ivp_active_float_get_float(active_float_max_torque));
    }
}

void ivp_rot_mot_set_rotation_speed_out(ivp_rot_mot_state_t *state, ivp_active_float_t *terminal)
{
    if (!state || state->active_float_rotation_speed_out == terminal) return;
    if (terminal) ivp_active_float_add_ref(terminal);
    ivp_active_float_t *old = state->active_float_rotation_speed_out;
    state->active_float_rotation_speed_out = terminal;
    if (old) ivp_active_float_remove_ref(old);
}

void ivp_rot_mot_unbind_active(ivp_rot_mot_state_t *state)
{
    if (!state) return;
    rot_mot_release(state, &state->active_float_max_rotation_speed);
    rot_mot_release(state, &state->active_float_power);
    rot_mot_release(state, &state->active_float_max_torque);
    ivp_rot_mot_set_rotation_speed_out(state, NULL);
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_rot_mot(const ivp_event_sim_t *event_sim,
                               ivp_object_t *obj,
                               void *ctx)
{
    ivp_rot_mot_state_t *state = (ivp_rot_mot_state_t *)ctx;
    if (!state->active) return;
    const double power = (double)(float)state->power;
    if (power == 0) return; /* don't wake up object */

    ivp_object_t *driven = rot_mot_driven(state);
    if (!driven || obj != driven) return;
    ivp_core_t *core = driven->physical_core;           /* obj->get_core() */
    if (core->physical_unmoveable) return;

    /* Reference IVP_Actuator_Rot_Mot::do_simulation_controller
     * (ivp_actuator.cxx:599-665).  Axis in the driven core's coordinate
     * system: axis_in_core_coord_system = normized(core_pos1 - core_pos0)
     * (IVP_U_Float_Point) while the object still has its original core,
     * else anchor 1 is transformed into the space of anchor 0's core. */
    const ivp_vec3f_t core_pos0 = ivp_ctl_f(&state->anchor_a_os);
    const ivp_vec3f_t core_pos1 = ivp_ctl_f(&state->anchor_b_os);
    ivp_vec3f_t axis;
    if (core == driven->original_core) {
        lvf_subtract2(&axis, &core_pos1, &core_pos0);
    } else {
        ivp_object_t *obj1 = state->obj_b ? state->obj_b : state->obj_a;   /* anchor 1 object */
        ivp_vec3r_t pos1_ws;
        ivp_ctl_anchor_pos_ws(obj1->physical_core, &core_pos1, &pos1_ws);
        ivp_vec3f_t pos1_cs0;
        lvm_vimult4_pf(&core->m_world_f_core_last_psi, &pos1_ws, &pos1_cs0);
        lvf_subtract2(&axis, &pos1_cs0, &core_pos0);
    }
    if (!lvf_normize(&axis)) return;

    double domega = ivp_ctl_get_rot_speed_cs(core, &axis);

    if (power * domega < 0) { /* power in opposite direction */
        domega = 0.0f;
    }
    const float rot_speed_out = (float)domega; /* memorize for sound e.g. */
    state->current_speed = (ivp_real_t)rot_speed_out;
    if (state->active_float_rotation_speed_out) {
        ivp_active_float_set(state->active_float_rotation_speed_out, (double)rot_speed_out);
    }

    if (domega < 0.0f) domega = -domega;
    if (domega > (double)(float)state->max_rotation_speed) {
        IVP_IF(0) { /* ivp_actuator.cxx:639 */
            ivp_debug_printf("Rot Mot rot speed threshold reached!\n");
        }
        state->current_torque = IVP_REAL(0.0);
        return; /* too fast */
    }
    if (domega < 0.1f) domega = 0.1f;

    const double rot_inertia_i = ivp_ctl_get_rot_inertia_cs(core, &axis);
    double rot_force = power / (domega * rot_inertia_i);

    /* clip force (max_torque == 0 means no clipping) */
    const double max_torque = (double)(float)state->max_torque;
    if (max_torque != 0.0f) {
        if (rot_force > max_torque) {
            IVP_IF(0) { /* ivp_actuator.cxx:657 */
                ivp_debug_printf("RM Force clipped to %g, was %g\n", max_torque, rot_force);
            }
            rot_force = max_torque;
        } else if (rot_force < -max_torque) {
            IVP_IF(0) { /* ivp_actuator.cxx:665 */
                ivp_debug_printf("RM Force clipped to %g, was %g\n", -max_torque, rot_force);
            }
            rot_force = -max_torque;
        }
    }
    state->current_torque = (ivp_real_t)rot_force;

    const double rot_imp = rot_force * (double)event_sim->delta_time;
    ivp_ctl_async_rot_push_core_multiple_cs(core, &axis, rot_imp);
}

int ivp_rot_mot_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles)
{
    (void)env;
    const ivp_rot_mot_state_t *s = (const ivp_rot_mot_state_t *)ctx;
    if (!s) return 0;
    ivp_object_t *objs[2] = { s->obj_a, s->obj_b };
    return ivp_controller_discover_objects(objs, 2, out_handles, max_handles);
}
