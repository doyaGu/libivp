/* ivp_forcefield.c -- Force field controller implementation */

#include <ivp/controllers/ivp_forcefield.h>
#include <ivp/dynamics/ivp_object.h>
#include <string.h>

void ivp_forcefield_state_init(ivp_forcefield_state_t *state,
                                ivp_forcefield_mode_t mode,
                                const ivp_vec3r_t *value)
{
    state->mode   = mode;
    state->value  = *value;
    state->active = true;
}

void ivp_controller_forcefield(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    ivp_forcefield_state_t *state = (ivp_forcefield_state_t *)ctx;
    if (!state || !state->active) return;

    ivp_core_t *core = &obj->core;
    if (core->physical_unmoveable || core->pinned) return;

    switch (state->mode) {
    case IVP_FORCEFIELD_CONSTANT: {
        /* Constant acceleration: dv = accel * dt */
        ivp_vec3r_add_scaled(&core->speed_change, &state->value, dt);
        break;
    }
    case IVP_FORCEFIELD_FORCE: {
        /* Constant force: dv = F/m * dt */
        ivp_vec3r_t dv;
        ivp_vec3r_scale(&dv, &state->value, core->inv_mass * dt);
        ivp_vec3r_add(&core->speed_change, &core->speed_change, &dv);
        break;
    }
    case IVP_FORCEFIELD_DRAG: {
        /* Velocity-proportional drag: F = -drag * v
         * dv = -drag * v / m * dt */
        ivp_vec3r_t drag;
        drag.k[0] = -state->value.k[0] * core->speed.k[0] * core->inv_mass * dt;
        drag.k[1] = -state->value.k[1] * core->speed.k[1] * core->inv_mass * dt;
        drag.k[2] = -state->value.k[2] * core->speed.k[2] * core->inv_mass * dt;
        ivp_vec3r_add(&core->speed_change, &core->speed_change, &drag);
        break;
    }
    }
}
