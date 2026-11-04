/* demo_drive.h -- the feel of the driving examples
 *
 * A key is on or off, a car is not: the throttle and the steering follow the
 * keys through a smoothing, so holding a key winds the steering on, releasing it
 * brings it back to the centre (faster than it was wound on), and a quick tap is a
 * small correction. A gamepad (the first one found) gives the same inputs as
 * analog values: left stick x steers, the right trigger is the throttle and the
 * left one the brake / reverse. At speed the steering lock shrinks, and the
 * follow camera looks ahead of the vehicle and pulls back a little with speed.
 *
 *   demo_drive_t d;  demo_drive_defaults(&d);
 *   viewer_init:     demo_drive_panel(&v->panel, &d);
 *   update (window): demo_drive_read(&d, &in);  demo_drive_step(&d, &in, v->frame_dt);
 *                    use d.throttle, d.steer (+1 = right), in.brake
 *   update_view:     demo_drive_follow_object(v, &d, vehicle);
 */
#ifndef IVP_EXAMPLE_DEMO_DRIVE_H
#define IVP_EXAMPLE_DEMO_DRIVE_H

#include "demo_app.h"
#include "demo_drive_core.h"

static void demo_drive_noop(void *ctx) { (void)ctx; }

/* F5 sliders for the response (call from viewer_init). */
static inline void demo_drive_panel(demo_panel_t *p, demo_drive_t *d)
{
    demo_panel_add_float(p, "throttle time", &d->throttle_time, 0.05f, 1.0f, demo_drive_noop, d);
    demo_panel_add_float(p, "steer time", &d->steer_time, 0.1f, 1.5f, demo_drive_noop, d);
    demo_panel_add_float(p, "centring time", &d->center_time, 0.05f, 1.0f, demo_drive_noop, d);
}

static inline float demo_drive_axis(SDL_Gamepad *pad, SDL_GamepadAxis axis)
{
    float a = (float)SDL_GetGamepadAxis(pad, axis) / 32767.0f;
    const float dead = 0.12f;
    if (fabsf(a) < dead) return 0.0f;
    a = (fabsf(a) - dead) / (1.0f - dead) * (a < 0.0f ? -1.0f : 1.0f);
    return a > 1.0f ? 1.0f : (a < -1.0f ? -1.0f : a);
}

/* The keys (arrows; B brakes) and the first gamepad. */
static inline void demo_drive_read(demo_drive_t *d, demo_drive_input_t *in)
{
    memset(in, 0, sizeof(*in));
    const bool *keys = SDL_GetKeyboardState(NULL);
    in->throttle = (keys[SDL_SCANCODE_UP] ? 1.0f : 0.0f) - (keys[SDL_SCANCODE_DOWN] ? 1.0f : 0.0f);
    in->steer = (keys[SDL_SCANCODE_RIGHT] ? 1.0f : 0.0f) - (keys[SDL_SCANCODE_LEFT] ? 1.0f : 0.0f);
    in->brake = keys[SDL_SCANCODE_B];

    const double now = (double)SDL_GetTicks() * 0.001;
    if ((!d->pad || !SDL_GamepadConnected(d->pad)) && now >= d->next_pad_search) {
        if (d->pad) SDL_CloseGamepad(d->pad);
        d->pad = NULL;
        d->next_pad_search = now + 2.0;
        if (SDL_InitSubSystem(SDL_INIT_GAMEPAD)) {
            int n = 0;
            SDL_JoystickID *ids = SDL_GetGamepads(&n);
            if (ids && n > 0) d->pad = SDL_OpenGamepad(ids[0]);
            SDL_free(ids);
        }
    }
    if (d->pad && SDL_GamepadConnected(d->pad)) {
        const float sx = demo_drive_axis(d->pad, SDL_GAMEPAD_AXIS_LEFTX);
        const float rt = (float)SDL_GetGamepadAxis(d->pad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER) / 32767.0f;
        const float lt = (float)SDL_GetGamepadAxis(d->pad, SDL_GAMEPAD_AXIS_LEFT_TRIGGER) / 32767.0f;
        if (sx != 0.0f || rt > 0.05f || lt > 0.05f || SDL_GetGamepadButton(d->pad, SDL_GAMEPAD_BUTTON_SOUTH)) {
            in->used_pad = true;
            in->steer = sx;
            in->throttle = (rt > 0.05f ? rt : 0.0f) - (lt > 0.05f ? lt : 0.0f);
            in->brake = SDL_GetGamepadButton(d->pad, SDL_GAMEPAD_BUTTON_SOUTH);
        }
    }
}

/* A follow camera: the target eases to the vehicle plus a look-ahead (the velocity times 0.4 s),
 * and the orbit distance grows with the speed. Frame-rate independent. */
static inline void demo_drive_follow(demo_viewer_t *v, demo_drive_t *d, const float centre[3], const float vel[3])
{
    const float dt = v->frame_dt > 0.1f ? 0.1f : v->frame_dt;
    const float k = 1.0f - expf(-dt * 5.0f);
    for (int i = 0; i < 3; i++) {
        const float want = centre[i] + (i == 1 ? 0.0f : 0.4f * vel[i]);
        v->cam.target[i] += (want - v->cam.target[i]) * k;
    }
    const float speed = sqrtf(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]);
    /* the distance the user chose (scroll) is what is left after taking our extra off */
    const float base = v->cam.orbit_distance - d->cam_dist;
    const float want_extra = fminf(0.2f * speed, 0.5f * base);
    d->cam_dist += (want_extra - d->cam_dist) * (1.0f - expf(-dt * 1.5f));
    v->cam.orbit_distance = base + d->cam_dist;
}

/* Call from update_view, after the physics step, to follow the display pose. */
static inline void demo_drive_follow_object(demo_viewer_t *v, demo_drive_t *d,
                                            const ivp_object_t *object)
{
    float centre[3];
    demo_object_center(object, centre);
    const ivp_vec3r_t *speed = &object->physical_core->speed;
    const float vel[3] = {(float)speed->k[0], (float)speed->k[1], (float)speed->k[2]};
    demo_drive_follow(v, d, centre, vel);
}

#endif /* IVP_EXAMPLE_DEMO_DRIVE_H */
