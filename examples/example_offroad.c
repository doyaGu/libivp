/* example_offroad.c -- The raycast car on a heightfield terrain
 *
 * example_vehicle's four-wheel raycast car, driven over rolling hills made
 * of a compact grid. The wheels cast their rays against the environment, so
 * the grid terrain is just another object to them. Crates on the hills can
 * be bumped. Arrow keys steer and drive, B brakes, Space resets the scene;
 * the camera follows the car.
 *
 * Demonstrates: ivp_car_init_4wheel on non-flat static geometry,
 *               the compact grid terrain, a follow camera.
 */
#include "demo_app.h"
#include "demo_terrain.h"

#include <ivp/controllers/ivp_controller_raycast_car.h>
#include "demo_car.h"

#define N          61             /* terrain points per side */
#define FIELD      2.0f           /* metres between points: 120 m square */
#define N_CRATES   14

#define ENGINE_FWD   220.0
#define ENGINE_REV  -110.0
#define BRAKE_FORCE  200.0
#define STEER_ANGLE  0.4

typedef struct offroad_demo {
    demo_terrain_t   terrain;
    ivp_object_t    *chassis;
    ivp_object_t    *crates[N_CRATES];
    ivp_car_state_t  car;
    /* headless measurements */
    float            start[3];
    float            travelled;
    float            max_tilt;
    float            lowest_clearance;      /* chassis centre above the terrain, min over the run */
    int              grounded_min_late;
} offroad_demo_t;

/* Gentle rolling hills; the start area around the origin is nearly flat. */
static float hill_height(float x, float z)
{
    float d = sqrtf(x * x + z * z);
    float flat = d < 10.0f ? d / 10.0f : 1.0f;       /* ramps the hills in */
    float h = 2.2f * sinf(x * 0.11f) * cosf(z * 0.09f)
            + 1.1f * sinf(x * 0.23f + z * 0.17f)
            + 0.5f * cosf(z * 0.41f);
    return 4.0f + flat * h;
}

static int wheels_grounded(const offroad_demo_t *s)
{
    int n = 0;
    for (int i = 0; i < s->car.n_wheels; i++)
        if (s->car.wheels[i].grounded) n++;
    return n;
}

static void apply_input(offroad_demo_t *s, float steer, float engine, float brake)
{
    ivp_car_set_steering(&s->car, (ivp_real_t)steer);
    ivp_car_set_engine(&s->car, (ivp_real_t)engine);
    ivp_car_set_brake(&s->car, (ivp_real_t)brake);
}

static void offroad_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    offroad_demo_t *s = (offroad_demo_t *)app->user;
    demo_terrain_make_mesh(&s->terrain, v->renderer);
}

static void offroad_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    offroad_demo_t *s = (offroad_demo_t *)app->user;
    float c[3];
    demo_object_center(s->chassis, c);
    float tilt = demo_object_tilt_deg(s->chassis);
    if (tilt > s->max_tilt) s->max_tilt = tilt;
    float surface_y;
    if (frame > 90 && demo_terrain_surface_y(&s->terrain, c[0], c[2], &surface_y)) {
        float clear = surface_y - c[1];                     /* up is -Y */
        if (clear < s->lowest_clearance) s->lowest_clearance = clear;
    }

    if (v) {
        const bool *keys = SDL_GetKeyboardState(NULL);
        float steer = 0.0f, engine = 0.0f, brake = 0.0f;
        if (keys[SDL_SCANCODE_LEFT])  steer += (float)STEER_ANGLE;
        if (keys[SDL_SCANCODE_RIGHT]) steer -= (float)STEER_ANGLE;
        if (keys[SDL_SCANCODE_UP])    engine = (float)ENGINE_FWD;
        if (keys[SDL_SCANCODE_DOWN])  engine = (float)ENGINE_REV;
        if (keys[SDL_SCANCODE_B])     brake = (float)BRAKE_FORCE;
        apply_input(s, steer, engine, brake);
        for (int k = 0; k < 3; k++) v->cam.target[k] += (c[k] - v->cam.target[k]) * 0.05f;
        return;
    }
    /* scripted: settle 1.5 s, drive 4 s with a gentle left bend, brake */
    float steer = 0.0f, engine = 0.0f, brake = 0.0f;
    if (frame >= 90 && frame < 330) engine = (float)ENGINE_FWD;
    if (frame >= 210 && frame < 330) steer = 0.15f;
    if (frame >= 330) brake = (float)BRAKE_FORCE;
    if (frame == 90) memcpy(s->start, c, sizeof(s->start));
    if (frame == 570) {
        s->travelled = demo_distance(c, s->start);
        s->grounded_min_late = wheels_grounded(s);
    }
    apply_input(s, steer, engine, brake);
}

static void offroad_reset(demo_app_t *app)
{
    offroad_demo_t *s = (offroad_demo_t *)app->user;
    s->max_tilt = 0.0f;
    s->lowest_clearance = 1e9f;
}

static void offroad_draw(demo_app_t *app, demo_viewer_t *v)
{
    offroad_demo_t *s = (offroad_demo_t *)app->user;
    const float c_land[3]    = { 0.40f, 0.55f, 0.32f };
    const float c_wire[3]    = { 0.30f, 0.62f, 0.30f };
    const float c_chassis[3] = { 0.85f, 0.18f, 0.16f };
    const float c_crate[3]   = { 0.90f, 0.55f, 0.20f };
    const float c_wheel[3]   = { 0.12f, 0.12f, 0.14f };

    demo_terrain_draw(v, &s->terrain, v->solid ? c_land : c_wire);
    for (int i = 0; i < N_CRATES; i++) demo_draw_object(v, s->crates[i], c_crate);
    demo_draw_object(v, s->chassis, c_chassis);

    demo_draw_car(v, s->chassis, &s->car, c_chassis);
    if (!v->solid) {
        for (int i = 0; i < s->car.n_wheels; i++) {
            const ivp_car_wheel_t *wh = &s->car.wheels[i];
            ivp_vec3r_t hp_cs = ivp_vec3r(wh->hp_cs.k[0], wh->hp_cs.k[1], wh->hp_cs.k[2]);
            float hp[3];
            demo_core_point_ws(s->chassis, &hp_cs, hp);
            demo_sphere(v, hp, wh->wheel_radius, c_wheel);
        }
    }

    float p[3];
    demo_object_center(s->chassis, p);
    demo_hud(v, DEMO_COL_INFO, "Speed %.1f m/s  grounded %d/4  tilt %.0f deg  height %.1f m",
             (double)demo_object_speed(s->chassis), wheels_grounded(s),
             (double)demo_object_tilt_deg(s->chassis), (double)(-p[1]));
}

static void offroad_stats(demo_app_t *app, char *buf, size_t size)
{
    offroad_demo_t *s = (offroad_demo_t *)app->user;
    float p[3];
    demo_object_center(s->chassis, p);
    snprintf(buf, size, "car (%.1f, %.1f, %.1f) speed %.2f grounded %d tilt %.1f", (double)p[0], (double)p[1],
             (double)p[2], (double)demo_object_speed(s->chassis), wheels_grounded(s),
             (double)demo_object_tilt_deg(s->chassis));
}

static bool offroad_check(demo_app_t *app)
{
    offroad_demo_t *s = (offroad_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->travelled > 20.0f, "the car drove %.1f m over the hills",
                     (double)s->travelled);
    ok &= demo_check(tag, s->lowest_clearance > 0.0f,
                     "the chassis stayed above the terrain (lowest clearance %.2f m)", (double)s->lowest_clearance);
    ok &= demo_check(tag, s->max_tilt < 55.0f, "the car stayed on its wheels (max tilt %.1f deg)",
                     (double)s->max_tilt);
    ok &= demo_check(tag, s->grounded_min_late >= 1, "wheels touch the ground at the end (%d/4)",
                     s->grounded_min_late);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Offroad -- Raycast Car on a Heightfield";
    app.cfg.tag         = "offroad";
    app.cfg.friction    = 0.9;
    app.cfg.elasticity  = 0.1;
    app.cfg.orbit_dist  = 18.0f;
    app.cfg.orbit_yaw   = 30.0f;
    app.cfg.target[1]   = -4.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    offroad_demo_t *s = (offroad_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->lowest_clearance = 1e9f;

    float *heights = (float *)malloc(sizeof(float) * N * N);
    if (!heights) return 1;
    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++)
            heights[r * N + c] = hill_height(((float)r - 0.5f * (N - 1)) * FIELD,
                                             ((float)c - 0.5f * (N - 1)) * FIELD);
    bool built = demo_terrain_create(w, &s->terrain, N, FIELD, heights, app.cfg.friction, app.cfg.elasticity);
    free(heights);
    if (!built) {
        fprintf(stderr, "[offroad] terrain build failed\n");
        return 1;
    }

    /* crates scattered on the hills, dropped from 1 m above the surface */
    unsigned seed = 77u;
    for (int i = 0; i < N_CRATES; i++) {
        seed = seed * 1664525u + 1013904223u;
        float x = 8.0f + (float)((seed >> 8) & 1023) / 1023.0f * 24.0f;
        seed = seed * 1664525u + 1013904223u;
        float z = -14.0f + (float)((seed >> 8) & 1023) / 1023.0f * 28.0f;
        float y = -(demo_terrain_height_at(&s->terrain, x, z) + 1.2f);
        s->crates[i] = demo_create_box(w, NULL, 0.5, 0.5, 0.5, 2.0, NULL, x, y, z);
        if (!s->crates[i]) return 1;
    }

    /* chassis: same car as example_vehicle, dropped onto the start area */
    ivp_object_template_t cht;
    demo_template_dynamic(&cht, &w->mat, 20.0);
    demo_set_box_inertia(&cht, 20.0, 1.0, 0.4, 2.0);
    cht.material.friction = IVP_REAL(0.3);
    cht.material.elasticity = IVP_REAL(0.1);
    demo_template_pose(&cht, NULL, 0.0, -(demo_terrain_height_at(&s->terrain, 0.0f, 0.0f) + 1.6), 0.0);
    s->chassis = demo_create_box_templ(w, &cht, 1.0, 0.4, 2.0);
    if (!s->chassis) return 1;

    if (!ivp_car_init_4wheel(&s->car, s->chassis, IVP_REAL(3.0), IVP_REAL(1.8), IVP_REAL(0.35), IVP_REAL(0.6))) {
        fprintf(stderr, "ivp_car_init_4wheel failed\n");
        return 1;
    }
    for (int i = 0; i < s->car.n_wheels; i++) {
        s->car.wheels[i].spring_constant      = 800.0f;
        s->car.wheels[i].spring_damp_relax    = 60.0f;
        s->car.wheels[i].spring_damp_compress = 60.0f;
        s->car.wheels[i].friction_of_wheel    = 1.2f;
    }

    app.controls    = "Arrows: steer / drive   B: brake";
    app.viewer_init = offroad_viewer_init;
    app.update      = offroad_update;
    app.on_reset    = offroad_reset;
    app.draw        = offroad_draw;
    app.stats       = offroad_stats;
    app.check       = offroad_check;

    int rc = demo_run(&app, argc, argv);
    ivp_car_destroy(&s->car);
    demo_world_destroy(&app.world);
    demo_terrain_destroy(&s->terrain);
    free(s);
    return rc;
}
