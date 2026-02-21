/* ivp_controller_buoyancy.c -- Buoyancy controller entry points */

#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/dynamics/ivp_object.h>

#include <string.h>

#include "ivp_buoyancy_solver.h"

void ivp_buoyancy_init(ivp_buoyancy_state_t *state,
                       ivp_real_t surface_y,
                       ivp_real_t fluid_density)
{
    memset(state, 0, sizeof(*state));
    ivp_vec3r_set(&state->surface_normal,
                  IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    state->surface_height = -surface_y;
    state->fluid_density = fluid_density;
    state->linear_drag = IVP_REAL(0.5);
    state->angular_drag = IVP_REAL(0.3);
    ivp_vec3r_set(&state->flow_velocity,
                  IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    state->active = true;
}

void ivp_buoyancy_init_plane(ivp_buoyancy_state_t *state,
                             const ivp_vec3r_t *normal,
                             ivp_real_t height,
                             ivp_real_t fluid_density)
{
    memset(state, 0, sizeof(*state));
    state->surface_normal = *normal;
    if (ivp_vec3r_normalize(&state->surface_normal) != IVP_OK) {
        ivp_vec3r_set(&state->surface_normal,
                      IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    }
    state->surface_height = height;
    state->fluid_density = fluid_density;
    state->linear_drag = IVP_REAL(0.5);
    state->angular_drag = IVP_REAL(0.3);
    ivp_vec3r_set(&state->flow_velocity,
                  IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    state->active = true;
}

void ivp_controller_buoyancy(const ivp_event_sim_t *event_sim,
                             ivp_object_t *obj,
                             void *ctx)
{
    ivp_buoyancy_state_t *state = (ivp_buoyancy_state_t *)ctx;
    if (!state || !state->active) {
        return;
    }

    ivp_core_t *core = &obj->core;
    if (core->physical_unmoveable || core->pinned) {
        return;
    }

    ivp_buoyancy_solver_step(event_sim, obj, state);
}
