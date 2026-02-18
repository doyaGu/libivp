/* ivp_controller_phantom.c -- Phantom (sensor/trigger) controller */

#include <ivp/controllers/ivp_controller_phantom.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_phantom_init_sphere(ivp_phantom_state_t *state,
                               const ivp_vec3r_t *center_ws,
                               ivp_real_t radius,
                               ivp_phantom_event_fn_t event_fn,
                               void *event_ctx)
{
    memset(state, 0, sizeof(*state));
    state->shape     = IVP_PHANTOM_SPHERE;
    state->center_ws = *center_ws;
    state->radius    = radius;
    state->event_fn  = event_fn;
    state->event_ctx = event_ctx;
    state->active    = true;
}

void ivp_phantom_init_aabb(ivp_phantom_state_t *state,
                             const ivp_vec3r_t *min_ws,
                             const ivp_vec3r_t *max_ws,
                             ivp_phantom_event_fn_t event_fn,
                             void *event_ctx)
{
    memset(state, 0, sizeof(*state));
    state->shape       = IVP_PHANTOM_AABB;
    state->aabb_min_ws = *min_ws;
    state->aabb_max_ws = *max_ws;
    state->event_fn    = event_fn;
    state->event_ctx   = event_ctx;
    state->active      = true;
}

void ivp_phantom_attach(ivp_phantom_state_t *state,
                          ivp_object_t *owner,
                          const ivp_vec3r_t *offset_os)
{
    state->owner     = owner;
    state->offset_os = *offset_os;
}

/* ── Test if object is inside phantom ───────────────────────────────────── */

static void update_phantom_position(ivp_phantom_state_t *state)
{
    if (!state->owner) return;

    /* Move phantom center to owner's world position + offset */
    ivp_vec3r_t offset_ws;
    ivp_transform_dir(&offset_ws,
                       &state->owner->core.m_world_f_core_last_psi,
                       &state->offset_os);

    if (state->shape == IVP_PHANTOM_SPHERE) {
        ivp_vec3r_add(&state->center_ws,
                      &state->owner->core.pos_world_f_core_last_psi,
                      &offset_ws);
    } else {
        /* For AABB attached to an object, compute center then expand */
        ivp_vec3r_t center;
        ivp_vec3r_add(&center,
                      &state->owner->core.pos_world_f_core_last_psi,
                      &offset_ws);
        ivp_vec3r_t half;
        half.k[0] = (state->aabb_max_ws.k[0] - state->aabb_min_ws.k[0]) * IVP_REAL(0.5);
        half.k[1] = (state->aabb_max_ws.k[1] - state->aabb_min_ws.k[1]) * IVP_REAL(0.5);
        half.k[2] = (state->aabb_max_ws.k[2] - state->aabb_min_ws.k[2]) * IVP_REAL(0.5);
        ivp_vec3r_sub(&state->aabb_min_ws, &center, &half);
        ivp_vec3r_add(&state->aabb_max_ws, &center, &half);
    }
}

static bool is_object_inside(const ivp_phantom_state_t *state,
                               const ivp_object_t *obj)
{
    const ivp_vec3r_t *pos = &obj->core.pos_world_f_core_last_psi;

    switch (state->shape) {
    case IVP_PHANTOM_SPHERE: {
        ivp_vec3r_t diff;
        ivp_vec3r_sub(&diff, pos, &state->center_ws);
        ivp_real_t dist2 = ivp_vec3r_dot(&diff, &diff);
        ivp_real_t r = state->radius + obj->core.upper_limit_radius;
        return dist2 <= r * r;
    }
    case IVP_PHANTOM_AABB: {
        ivp_real_t r = obj->core.upper_limit_radius;
        return pos->k[0] + r >= state->aabb_min_ws.k[0] &&
               pos->k[0] - r <= state->aabb_max_ws.k[0] &&
               pos->k[1] + r >= state->aabb_min_ws.k[1] &&
               pos->k[1] - r <= state->aabb_max_ws.k[1] &&
               pos->k[2] + r >= state->aabb_min_ws.k[2] &&
               pos->k[2] - r <= state->aabb_max_ws.k[2];
    }
    }
    return false;
}

/* ── Tracking helpers ───────────────────────────────────────────────────── */

static int find_tracked(const ivp_phantom_state_t *state, const ivp_object_t *obj)
{
    for (int i = 0; i < state->n_tracked; i++) {
        if (state->tracked[i] == obj) return i;
    }
    return -1;
}

static bool add_tracked(ivp_phantom_state_t *state, ivp_object_t *obj)
{
    if (state->n_tracked >= IVP_PHANTOM_MAX_TRACKED) return false;
    state->tracked[state->n_tracked++] = obj;
    return true;
}

static void remove_tracked(ivp_phantom_state_t *state, int index)
{
    state->n_tracked--;
    if (index < state->n_tracked) {
        state->tracked[index] = state->tracked[state->n_tracked];
    }
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_phantom(const ivp_event_sim_t *event_sim,
                              ivp_object_t *obj,
                              void *ctx)
{
    ivp_phantom_state_t *state = (ivp_phantom_state_t *)ctx;
    if (!state || !state->active) return;

    /* Skip the phantom's own owner */
    if (obj == state->owner) return;

    /* Update phantom position if attached */
    update_phantom_position(state);

    bool inside = is_object_inside(state, obj);
    int tracked_idx = find_tracked(state, obj);

    if (inside && tracked_idx < 0) {
        /* Object just entered */
        if (add_tracked(state, obj) && state->event_fn) {
            state->event_fn(state->event_ctx, state->owner, obj, true);
        }
    } else if (!inside && tracked_idx >= 0) {
        /* Object just exited */
        remove_tracked(state, tracked_idx);
        if (state->event_fn) {
            state->event_fn(state->event_ctx, state->owner, obj, false);
        }
    }
}
