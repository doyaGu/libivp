/* demo_drive_core.h -- the smoothing behind demo_drive.h, without SDL or the renderer
 * (so a unit test can drive it). */
#ifndef IVP_EXAMPLE_DEMO_DRIVE_CORE_H
#define IVP_EXAMPLE_DEMO_DRIVE_CORE_H

#include <math.h>
#include <stdbool.h>
#include <string.h>

typedef struct SDL_Gamepad SDL_Gamepad;

typedef struct demo_drive_input {
    float throttle;        /* -1 (reverse) .. 1 */
    float steer;           /* -1 (left) .. 1 (right) */
    bool  brake;
    bool  used_pad;        /* the values come from a gamepad */
} demo_drive_input_t;

typedef struct demo_drive {
    float throttle, steer;     /* smoothed, the values to use */
    float throttle_time;       /* s to go from 0 to full throttle with the key held */
    float steer_time;          /* s to wind the steering from the centre to the lock */
    float center_time;         /* s to bring it back from the lock */
    SDL_Gamepad *pad;
    double next_pad_search;
    float cam_dist;            /* smoothed extra distance of the follow camera */
} demo_drive_t;

static inline void demo_drive_defaults(demo_drive_t *d)
{
    memset(d, 0, sizeof(*d));
    d->throttle_time = 0.25f;
    d->steer_time = 0.45f;
    d->center_time = 0.20f;
}

static inline float demo_drive_approach(float cur, float target, float rate, float dt)
{
    const float step = rate * dt;
    if (target > cur + step) return cur + step;
    if (target < cur - step) return cur - step;
    return target;
}

/* One step of the smoothing with the real frame time. */
static inline void demo_drive_step(demo_drive_t *d, const demo_drive_input_t *in, float dt)
{
    if (dt > 0.1f) dt = 0.1f;
    /* the throttle winds on over throttle_time and falls twice as fast */
    const bool winding = fabsf(in->throttle) > fabsf(d->throttle) && in->throttle * d->throttle >= 0.0f;
    d->throttle = demo_drive_approach(d->throttle, in->throttle, (winding ? 1.0f : 2.0f) / d->throttle_time, dt);
    /* the steering: a new direction is wound on at 1 / steer_time, a reversal twice as fast, and
     * letting go brings it back at 1 / center_time */
    float rate;
    if (in->steer == 0.0f) rate = 1.0f / d->center_time;
    else if (in->steer * d->steer < 0.0f) rate = 2.0f / d->steer_time;
    else rate = 1.0f / d->steer_time;
    d->steer = demo_drive_approach(d->steer, in->steer, rate, dt);
}

/* The fraction of the full steering lock allowed at this speed (1 at a stand, 0.5 at ref_speed). */
static inline float demo_drive_lock(float speed, float ref_speed)
{
    return 1.0f / (1.0f + fabsf(speed) / ref_speed);
}

#endif /* IVP_EXAMPLE_DEMO_DRIVE_CORE_H */
