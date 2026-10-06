/* example_boat.c -- A motor boat in a pool, pushing floating crates
 *
 * The hull is a light box, so the buoyancy controller lets it float; a
 * user controller adds thrust along the hull axis, a rudder torque and a
 * keel that kills sideways slip, but only while the hull is in the water.
 * Six crates and a ball float in the pool and can be pushed around.
 * Arrow keys: up/down throttle, left/right steer. Space resets the scene.
 *
 * The water is a translucent slab; the cabin and the funnel are visuals
 * drawn on the hull pose.
 *
 * Demonstrates: ivp_controller_buoyancy for floating bodies, a user
 *               controller attached to one core with
 *               ivp_environment_add_controller_to_core, solid cylinder and
 *               translucent solids, a follow camera.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_controller.h>
#include <ivp/controllers/ivp_controller_buoyancy.h>

#define WATER_Y        (-2.0f)     /* surface; up is -Y */
#define FLOOR_Y        4.5f        /* top of the pool floor */
#define POOL_X         16.0f
#define POOL_Z         10.0f
#define WATER_DENSITY  1.0f        /* masses are 1/1000 of real ones, see example_buoyancy */
#define N_CRATES       6

#define THRUST_ACCEL   3.2f        /* m/s^2 at full throttle */
#define REVERSE_ACCEL  1.6f
#define YAW_ACCEL      1.1f        /* rad/s^2 at full rudder */
#define KEEL           4.0f        /* 1/s: sideways slip damping */

static const double HULL_HALF[3] = { 1.6, 0.3, 0.65 };

typedef struct boat_demo {
    ivp_object_t        *floor, *walls[4];
    ivp_object_t        *hull;
    ivp_object_t        *crates[N_CRATES];
    ivp_object_t        *ball;
    ivp_buoyancy_state_t buoy;
    ivp_controller_handle_t drive_handle;
    float                throttle, steer;     /* -1 .. 1, set by update() */
    /* headless measurements */
    float                heading0, heading1;
    bool                 frozen_before;       /* the boat slept before it was driven */
    float                speed_cruise;
    float                max_tilt;
    float                crate_moved;
} boat_demo_t;

/* Heading of the hull's forward (+X) axis in the water plane, degrees. */
static float heading_deg(const ivp_object_t *hull)
{
    float pos[3], rot[9];
    demo_object_pose(hull, pos, rot);
    return atan2f(rot[2], rot[0]) * (180.0f / 3.14159265f);
}

/* The "engine": runs once per PSI for the hull's core. Acting through
 * speed_change / rot_speed_change like the reference async_add_* calls. */
static void boat_drive(const ivp_event_sim_t *es, ivp_object_t *obj, void *ctx)
{
    boat_demo_t *s = (boat_demo_t *)ctx;
    ivp_core_t *core = obj->physical_core;
    if (core->physical_unmoveable) return;
    const float dt = (float)es->delta_time;

    float pos[3], rot[9];
    demo_object_pose(obj, pos, rot);
    const float fwd[3] = { rot[0], rot[1], rot[2] };          /* hull +X */
    const float side[3] = { rot[6], rot[7], rot[8] };         /* hull +Z */
    if (pos[1] < WATER_Y - 0.25f) return;                     /* propeller out of the water */

    float accel = s->throttle > 0.0f ? s->throttle * THRUST_ACCEL : s->throttle * REVERSE_ACCEL;
    float lateral = (float)(core->speed.k[0]) * side[0] + (float)(core->speed.k[1]) * side[1]
                  + (float)(core->speed.k[2]) * side[2];
    for (int i = 0; i < 3; i++)
        core->speed_change.k[i] += (ivp_real_t)((fwd[i] * accel - side[i] * lateral * KEEL) * dt);

    /* rudder: yaw about the world vertical, expressed in the core system
     * (the hull's own Y axis is vertical for a level boat) */
    core->rot_speed_change.k[1] += (ivp_real_t)(s->steer * YAW_ACCEL * dt);
}

static void boat_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    boat_demo_t *s = (boat_demo_t *)app->user;
    float tilt = demo_object_tilt_deg(s->hull);
    if (tilt > s->max_tilt) s->max_tilt = tilt;

    if (v) {
        const bool *keys = SDL_GetKeyboardState(NULL);
        s->throttle = (keys[SDL_SCANCODE_UP] ? 1.0f : 0.0f) - (keys[SDL_SCANCODE_DOWN] ? 1.0f : 0.0f);
        s->steer = (keys[SDL_SCANCODE_LEFT] ? 1.0f : 0.0f) - (keys[SDL_SCANCODE_RIGHT] ? 1.0f : 0.0f);
    } else {
        /* scripted: rest 6 s (the boat freezes), full motor 1.5 s, then
         * half motor and rudder 2 s */
        s->throttle = frame < 360 ? 0.0f : (frame < 450 ? 1.0f : 0.6f);
        s->steer = (frame >= 450 && frame < 570) ? 1.0f : 0.0f;
        if (frame == 359) s->frozen_before = ivp_object_is_frozen(s->hull);
        if (frame == 449) {
            s->speed_cruise = demo_object_speed(s->hull);
            s->heading0 = heading_deg(s->hull);
        }
        if (frame == 570) s->heading1 = heading_deg(s->hull);
    }
    /* a boat resting on the water freezes, and a frozen core runs no controllers */
    if (s->throttle != 0.0f || s->steer != 0.0f)
        ivp_environment_ensure_core_in_simulation(app->world.env, s->hull);
    if (v) {
        float target[3];
        demo_object_center(s->hull, target);
        for (int k = 0; k < 3; k++) v->cam.target[k] += (target[k] - v->cam.target[k]) * 0.04f;
    }
}

static void boat_reset(demo_app_t *app)
{
    boat_demo_t *s = (boat_demo_t *)app->user;
    s->throttle = s->steer = 0.0f;
    s->max_tilt = 0.0f;
}

static void boat_draw(demo_app_t *app, demo_viewer_t *v)
{
    boat_demo_t *s = (boat_demo_t *)app->user;
    const float c_floor[3] = { 0.42f, 0.38f, 0.32f };
    const float c_wall[3]  = { 0.55f, 0.58f, 0.62f };
    const float c_hull[3]  = { 0.90f, 0.92f, 0.96f };
    const float c_cabin[3] = { 0.85f, 0.25f, 0.22f };
    const float c_water[3] = { 0.12f, 0.38f, 0.72f };
    const float c_crate[3] = { 0.85f, 0.62f, 0.30f };
    const float c_ball[3]  = { 0.95f, 0.85f, 0.25f };

    demo_draw_object(v, s->floor, c_floor);
    for (int i = 0; i < 4; i++) demo_draw_object(v, s->walls[i], c_wall);
    demo_draw_object(v, s->hull, c_hull);
    for (int i = 0; i < N_CRATES; i++) demo_draw_object(v, s->crates[i], c_crate);
    demo_draw_object(v, s->ball, c_ball);

    /* cabin and funnel ride on the hull; up is -Y in the hull frame too */
    float pos[3], rot[9];
    demo_object_pose(s->hull, pos, rot);
    if (v->solid) {
        const float cab_half[3] = { 0.55f, 0.3f, 0.5f };
        const float cab[3] = { pos[0] - rot[3] * 0.6f - rot[0] * 0.35f, pos[1] - rot[4] * 0.6f - rot[1] * 0.35f,
                               pos[2] - rot[5] * 0.6f - rot[2] * 0.35f };
        demo_solid_box(v, cab, cab_half, rot, c_cabin);
        if (v->renderer) {
            const float a[3] = { cab[0] - rot[3] * 0.3f, cab[1] - rot[4] * 0.3f, cab[2] - rot[5] * 0.3f };
            const float b[3] = { cab[0] - rot[3] * 0.75f, cab[1] - rot[4] * 0.75f, cab[2] - rot[5] * 0.75f };
            ivp_render_draw_solid_cylinder(v->renderer, a, b, 0.14f, c_hull);
        }
    }

    /* the water body: a translucent slab down to the pool floor */
    if (v->solid) {
        /* a hair inside the pool: coplanar faces would z-fight with it */
        const float inset = 0.03f;
        const float depth = FLOOR_Y - WATER_Y - inset;
        const float centre[3] = { 0.0f, WATER_Y + depth * 0.5f, 0.0f };
        const float half[3] = { POOL_X - inset, depth * 0.5f, POOL_Z - inset };
        demo_set_alpha(v, 0.30f);
        demo_solid_box(v, centre, half, NULL, c_water);
        demo_set_alpha(v, 1.0f);
    } else {
        for (float x = -POOL_X; x <= POOL_X; x += 2.0f) {
            float a[3] = { x, WATER_Y, -POOL_Z }, b[3] = { x, WATER_Y, POOL_Z };
            demo_line(v, a, b, c_water);
        }
        for (float z = -POOL_Z; z <= POOL_Z; z += 2.0f) {
            float a[3] = { -POOL_X, WATER_Y, z }, b[3] = { POOL_X, WATER_Y, z };
            demo_line(v, a, b, c_water);
        }
    }

    float c[3];
    demo_object_center(s->hull, c);
    demo_hud(v, DEMO_COL_INFO, "Speed %.1f m/s  heading %.0f deg  draft %.2f m  tilt %.1f deg",
             (double)demo_object_speed(s->hull), (double)heading_deg(s->hull),
             (double)(c[1] - WATER_Y), (double)demo_object_tilt_deg(s->hull));
    demo_hud(v, DEMO_COL_INFO, "Throttle %+.0f%%  rudder %+.0f%%", (double)(s->throttle * 100.0f),
             (double)(s->steer * 100.0f));
}

static void boat_stats(demo_app_t *app, char *buf, size_t size)
{
    boat_demo_t *s = (boat_demo_t *)app->user;
    float c[3];
    demo_object_center(s->hull, c);
    snprintf(buf, size, "hull (%.2f, %.2f, %.2f) speed %.2f heading %.0f tilt %.1f", (double)c[0],
             (double)c[1], (double)c[2], (double)demo_object_speed(s->hull),
             (double)heading_deg(s->hull), (double)demo_object_tilt_deg(s->hull));
}

static bool boat_check(demo_app_t *app)
{
    boat_demo_t *s = (boat_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    float c[3];
    demo_object_center(s->hull, c);
    ok &= demo_check(tag, s->frozen_before, "the resting boat had frozen before it was driven");
    ok &= demo_check(tag, s->max_tilt < 30.0f, "the hull stays upright (max tilt %.1f deg)",
                     (double)s->max_tilt);
    ok &= demo_check(tag, fabsf(c[0]) < POOL_X && fabsf(c[2]) < POOL_Z,
                     "the boat stays inside the pool (%.1f, %.1f)", (double)c[0], (double)c[2]);
    ok &= demo_check(tag, s->speed_cruise > 1.0f, "the motor drives the boat: %.2f m/s after 1.5 s",
                     (double)s->speed_cruise);
    float dh = fabsf(s->heading1 - s->heading0);
    if (dh > 180.0f) dh = 360.0f - dh;
    ok &= demo_check(tag, dh > 30.0f, "the rudder turns the boat: %.0f deg", (double)dh);
    int floating = 0;
    for (int i = 0; i < N_CRATES; i++) {
        float p[3];
        demo_object_center(s->crates[i], p);
        if (fabsf(p[1] - WATER_Y) < 0.5f) floating++;
    }
    ok &= demo_check(tag, floating == N_CRATES, "%d/%d crates float at the surface", floating, N_CRATES);
    float bp[3];
    demo_object_center(s->ball, bp);
    ok &= demo_check(tag, fabsf(bp[1] - WATER_Y) < 0.6f, "the ball floats (y %.2f, surface %.2f)",
                     (double)bp[1], (double)WATER_Y);
    return ok;
}

static ivp_object_t *static_box(demo_world_t *w, double hx, double hy, double hz, double x, double y, double z)
{
    return demo_create_box(w, NULL, hx, hy, hz, 0.0, NULL, x, y, z);
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Boat -- Buoyancy and a Motor";
    app.cfg.tag         = "boat";
    app.cfg.friction    = 0.4;
    app.cfg.elasticity  = 0.2;
    app.cfg.orbit_dist  = 16.0f;
    app.cfg.orbit_yaw   = 25.0f;
    app.cfg.target[1]   = WATER_Y;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    boat_demo_t *s = (boat_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* pool: floor and four walls reaching 3 m above the water (up is -Y) */
    s->floor    = static_box(w, POOL_X + 0.5, 0.5, POOL_Z + 0.5, 0.0, FLOOR_Y + 0.5, 0.0);
    s->walls[0] = static_box(w, POOL_X + 0.5, 4.0, 0.4, 0.0, 0.5, -(POOL_Z + 0.4));
    s->walls[1] = static_box(w, POOL_X + 0.5, 4.0, 0.4, 0.0, 0.5, POOL_Z + 0.4);
    s->walls[2] = static_box(w, 0.4, 4.0, POOL_Z + 0.5, -(POOL_X + 0.4), 0.5, 0.0);
    s->walls[3] = static_box(w, 0.4, 4.0, POOL_Z + 0.5, POOL_X + 0.4, 0.5, 0.0);

    {   /* hull: relative density 0.45, floats a little more than half out */
        const double vol = 8.0 * HULL_HALF[0] * HULL_HALF[1] * HULL_HALF[2];
        const double mass = 0.45 * vol * WATER_DENSITY;
        ivp_object_template_t t;
        demo_template_dynamic(&t, &w->mat, mass);
        demo_set_box_inertia(&t, mass, HULL_HALF[0], HULL_HALF[1] * 3.0, HULL_HALF[2]);
        demo_template_pose(&t, NULL, -12.0, WATER_Y - 0.05, 0.0);
        s->hull = demo_create_box_templ(w, &t, HULL_HALF[0], HULL_HALF[1], HULL_HALF[2]);
        if (!s->hull) return 1;
    }

    static const double crate_pos[N_CRATES][3] = {
        { -2.0, 0.0, -3.0 }, { 1.0, 0.0, 2.5 }, { 3.5, 0.0, -1.0 },
        { 6.0, 0.0, 3.0 }, { 8.0, 0.0, -3.5 }, { 10.5, 0.0, 0.5 } };
    for (int i = 0; i < N_CRATES; i++) {
        const double h = 0.45;
        const double mass = 0.55 * 8.0 * h * h * h * WATER_DENSITY;
        ivp_object_template_t t;
        demo_template_dynamic(&t, &w->mat, mass);
        demo_set_box_inertia(&t, mass, h, h, h);
        demo_template_pose(&t, NULL, crate_pos[i][0], WATER_Y - 0.1, crate_pos[i][2]);
        s->crates[i] = demo_create_box_templ(w, &t, h, h, h);
        if (!s->crates[i]) return 1;
    }
    {
        const double r = 0.5;
        const double mass = 0.45 * (4.0 / 3.0) * 3.14159265 * r * r * r * WATER_DENSITY;
        ivp_object_template_t t;
        demo_template_dynamic(&t, &w->mat, mass);
        demo_set_sphere_inertia(&t, mass, r);
        demo_template_pose(&t, NULL, -4.0, WATER_Y - 0.1, 4.5);
        s->ball = demo_create_ball_templ(w, &t, r);
        if (!s->ball) return 1;
    }

    ivp_buoyancy_init(&s->buoy, (ivp_real_t)WATER_Y, (ivp_real_t)WATER_DENSITY);
    s->buoy.linear_drag  = IVP_REAL(3.0);
    s->buoy.angular_drag = IVP_REAL(2.0);
    ivp_environment_add_controller(w->env, 200, ivp_controller_buoyancy, &s->buoy);

    s->drive_handle = ivp_environment_add_controller(w->env, IVP_CP_ACTUATOR, boat_drive, s);
    ivp_environment_add_controller_to_core(w->env, s->drive_handle, s->hull);

    app.update   = boat_update;
    app.on_reset = boat_reset;
    app.draw     = boat_draw;
    app.stats    = boat_stats;
    app.check    = boat_check;
    app.controls = "Arrows: throttle and rudder";

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    ivp_buoyancy_state_destroy(&s->buoy);
    free(s);
    return rc;
}
