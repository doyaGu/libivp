/* example_jetski.c -- The fake jetski on a wavy sea
 *
 * ivp_jetski is a four-wheel raycast vehicle that is meant to skim over water:
 * four rays go down from the hull, the game-side callback says where the water
 * is, and the controller springs the hull off it, drives it, steers it and
 * pushes it forward with a booster. It starts with its wheels fixed (the
 * reference handbrake).
 *
 * Here the water is a travelling wave: the ray callback intersects every ray
 * with the surface and returns the slope as the hit normal, so the jetski
 * pitches and rolls over the swell. A static floor 6 m below catches it if it
 * sinks, and walls keep it in the pool.
 *
 *   Up / Down      throttle forward / reverse
 *   Left / Right   steer
 *   B              booster (8 for half a second, then it recharges)
 *   F5 panel       wave height, spring stiffness, throttle torque
 *
 * Demonstrates: ivp_jetski_init, ivp_jetski_set_raycast, ivp_jetski_fix_wheel,
 *               ivp_jetski_change_wheel_torque, ivp_jetski_do_steering,
 *               ivp_jetski_activate_booster, ivp_jetski_change_spring_*.
 */
#include "demo_app.h"
#include "demo_water.h"

#include <ivp/controllers/ivp_controller_fake_jetski.h>

#define SEA_X       40.0
#define SEA_Z       50.0
#define FLOOR_Y     6.0
#define WAVE_LEN    10.0
#define BODY_HX     0.5
#define BODY_HY     0.3
#define BODY_HZ     1.2
#define BODY_MASS   250.0

typedef struct jetski_demo {
    ivp_environment_t *env;
    ivp_object_t      *floor, *wall[4], *body;
    ivp_jetski_state_t ski;
    bool               ski_ok;
    /* panel */
    float              amp, spring_k, torque;
    /* controls */
    float              throttle, steer;
    /* measurements */
    int                rays_hit, rays_total;
    float              speed_before_boost, speed_peak_boost;
    float              start_heading, max_tilt;
    float              dist_max, tilt_max;
    double             z0;
    bool               boosted;
} jetski_demo_t;

/* ── the sea ─────────────────────────────────────────────────────────────── */

static double sea_y(const jetski_demo_t *s, double x, double t)
{
    const double k = 2.0 * M_PI / WAVE_LEN;
    return -(double)s->amp * sin(k * x - k * 2.5 * t);          /* mean level y = 0, up is -Y */
}

static double sea_slope(const jetski_demo_t *s, double x, double t)
{
    const double k = 2.0 * M_PI / WAVE_LEN;
    return -(double)s->amp * k * cos(k * x - k * 2.5 * t);
}

/* Reference do_raycasts: every ray against the wave. The surface is a height field of x, so a short fixed point
 * iteration finds where the ray meets it. */
static void sea_rays(void *ctx, const ivp_event_sim_t *es, int n_wheels, const ivp_ray_solver_template_t *rays,
                     ivp_ray_hit_t *hits_out, float *friction_out)
{
    jetski_demo_t *s = (jetski_demo_t *)ctx;
    const double t = (double)ivp_environment_get_time(es->environment);
    for (int i = 0; i < n_wheels; i++) {
        const ivp_vec3r_t *st = &rays[i].ray_start_point, *dir = &rays[i].ray_normized_direction;
        ivp_ray_hit_t *hit = &hits_out[i];
        memset(hit, 0, sizeof(*hit));
        friction_out[i] = 1.0f;
        s->rays_total++;
        if ((float)dir->k[1] <= 1e-6f) continue;
        if (fabs((double)st->k[0]) > SEA_X || fabs((double)st->k[2]) > SEA_Z) continue;
        double len = ((double)sea_y(s, (double)st->k[0], t) - (double)st->k[1]) / (double)dir->k[1];
        for (int it = 0; it < 4; it++) {
            const double x = (double)st->k[0] + len * (double)dir->k[0];
            len = (sea_y(s, x, t) - (double)st->k[1]) / (double)dir->k[1];
        }
        if (len < 0.0 || len > (double)rays[i].ray_length) continue;
        const double x = (double)st->k[0] + len * (double)dir->k[0], sl = sea_slope(s, x, t), n = sqrt(sl * sl + 1.0);
        hit->hit_object = s->floor;
        hit->hit_distance = (ivp_real_t)(float)len;
        hit->hit_normal = ivp_vec3r((ivp_real_t)(sl / n), (ivp_real_t)(-1.0 / n), IVP_REAL(0.0));      /* out of the water */
        friction_out[i] = 0.8f;
        s->rays_hit++;
    }
}

/* ── controls ────────────────────────────────────────────────────────────── */

#define TOP_SPEED 6.0f

static void drive(jetski_demo_t *s)
{
    /* the torque fades out towards the top speed: the traction is not limited by the wheels' spin */
    float fade = 1.0f - demo_object_speed(s->body) / TOP_SPEED;
    if (fade < 0.0f) fade = 0.0f;
    ivp_jetski_change_wheel_torque(&s->ski, IVP_JETSKI_REAR_LEFT, s->throttle * s->torque * fade);
    ivp_jetski_change_wheel_torque(&s->ski, IVP_JETSKI_REAR_RIGHT, s->throttle * s->torque * fade);
    ivp_jetski_do_steering(&s->ski, s->steer * 0.06f);
}

static void apply_springs(void *ctx)
{
    jetski_demo_t *s = (jetski_demo_t *)ctx;
    for (int i = 0; i < 4; i++) ivp_jetski_change_spring_constant(&s->ski, (ivp_jetski_wheel_pos_t)i, s->spring_k);
    ivp_environment_ensure_core_in_simulation(s->env, s->body);
}

static void apply_torque(void *ctx)
{
    jetski_demo_t *s = (jetski_demo_t *)ctx;
    drive(s);
    ivp_environment_ensure_core_in_simulation(s->env, s->body);
}

static void jetski_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    jetski_demo_t *s = (jetski_demo_t *)app->user;
    demo_panel_t *p = &v->panel;
    demo_panel_add_float(p, "wave height", &s->amp, 0.0f, 1.0f, NULL, s);
    demo_panel_add_float(p, "spring k", &s->spring_k, 4000.0f, 40000.0f, apply_springs, s);
    demo_panel_add_float(p, "throttle torque", &s->torque, 100.0f, 1200.0f, apply_torque, s);
}

static float heading_deg(const jetski_demo_t *s)
{
    float pos[3], rot[9];
    demo_object_pose(s->body, pos, rot);
    return atan2f(rot[6], rot[8]) * (180.0f / 3.14159265f);
}

static float tilt_of(const jetski_demo_t *s)
{
    return demo_object_tilt_deg(s->body);
}

static void jetski_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    jetski_demo_t *s = (jetski_demo_t *)app->user;
    if (!s->ski_ok) return;
    const double t = (double)ivp_environment_get_time(s->env);

    drive(s);
    if (v) {
        const bool *keys = SDL_GetKeyboardState(NULL);
        s->throttle = (keys[SDL_SCANCODE_UP] ? 1.0f : 0.0f) - (keys[SDL_SCANCODE_DOWN] ? 0.5f : 0.0f);
        s->steer = (keys[SDL_SCANCODE_RIGHT] ? 1.0f : 0.0f) - (keys[SDL_SCANCODE_LEFT] ? 1.0f : 0.0f);
        if (demo_key_just_pressed(v, SDL_SCANCODE_B)) ivp_jetski_activate_booster(&s->ski, 8.0f, 0.5f, 1.0f);
        for (int k = 0; k < 3; k++) {
            float c[3];
            demo_object_center(s->body, c);
            v->cam.target[k] += (c[k] - v->cam.target[k]) * 0.05f;
        }
    } else {
        /* the script of the scenario: handbrake off and throttle, steer, booster, steer back */
        if (frame == 60) {
            for (int i = 0; i < 4; i++) ivp_jetski_fix_wheel(&s->ski, (ivp_jetski_wheel_pos_t)i, false);
            s->throttle = 1.0f;
            drive(s);
        }
        if (frame == 200) s->start_heading = heading_deg(s);
        if (frame == 201) {
            s->steer = 1.0f;
            drive(s);
        }
        if (frame == 300) {
            s->steer = 0.0f;
            drive(s);
        }
        if (frame == 340) {
            s->speed_before_boost = demo_object_speed(s->body);
            ivp_jetski_activate_booster(&s->ski, 8.0f, 0.5f, 1.0f);
            s->boosted = true;
        }
        if (s->boosted && frame < 440 && demo_object_speed(s->body) > s->speed_peak_boost) s->speed_peak_boost = demo_object_speed(s->body);
    }
    if (frame > 120) {
        float c[3];
        demo_object_center(s->body, c);
        const double surf = sea_y(s, (double)c[0], t);
        const float d = (float)fabs((double)c[1] - surf);
        if (d > s->dist_max) s->dist_max = d;
        if (tilt_of(s) > s->tilt_max) s->tilt_max = tilt_of(s);
    }
}

static void jetski_reset(demo_app_t *app)
{
    jetski_demo_t *s = (jetski_demo_t *)app->user;
    s->dist_max = s->tilt_max = 0.0f;
}

/* ── drawing ─────────────────────────────────────────────────────────────── */

static float sea_height(void *ctx, float x, float z)
{
    const jetski_demo_t *s = (const jetski_demo_t *)ctx;
    (void)z;
    return (float)sea_y(s, (double)x, (double)ivp_environment_get_time(s->env));
}

static void draw_sea(demo_viewer_t *v, const jetski_demo_t *s, const float centre[3])
{
    if (v->solid) {
        const float deep[3] = { 0.08f, 0.30f, 0.70f }, crest[3] = { 0.60f, 0.85f, 1.0f };
        const float x0 = fmaxf(centre[0] - 24.0f, (float)-SEA_X), x1 = fminf(centre[0] + 24.0f, (float)SEA_X);
        const float z0 = fmaxf(centre[2] - 24.0f, (float)-SEA_Z), z1 = fminf(centre[2] + 24.0f, (float)SEA_Z);
        demo_draw_water_plates(v, x0, x1, z0, z1, 1.5f, sea_height, (void *)s, deep, crest, 0.58f);
        return;
    }
    const double t = (double)ivp_environment_get_time(s->env);
    const float c1[3] = { 0.30f, 0.55f, 0.90f };
    const double x0 = floor((double)centre[0] / 2.0) * 2.0, z0 = floor((double)centre[2] / 2.0) * 2.0;
    for (int zi = -8; zi <= 8; zi++) {
        float prev[3] = { 0 };
        for (int xi = -16; xi <= 16; xi++) {
            const double x = x0 + xi * 1.0, z = z0 + zi * 2.0;
            const float p[3] = { (float)x, (float)sea_y(s, x, t), (float)z };
            if (xi > -16 && fabs(x) <= SEA_X && fabs(z) <= SEA_Z) demo_line(v, prev, p, c1);
            memcpy(prev, p, sizeof(prev));
        }
    }
    for (int xi = -8; xi <= 8; xi++) {
        float prev[3] = { 0 };
        for (int zi = -8; zi <= 8; zi++) {
            const double x = x0 + xi * 2.0, z = z0 + zi * 2.0;
            const float p[3] = { (float)x, (float)sea_y(s, x, t), (float)z };
            if (zi > -8 && fabs(x) <= SEA_X && fabs(z) <= SEA_Z) demo_line(v, prev, p, c1);
            memcpy(prev, p, sizeof(prev));
        }
    }
}

static void jetski_draw(demo_app_t *app, demo_viewer_t *v)
{
    jetski_demo_t *s = (jetski_demo_t *)app->user;
    const float c_floor[3] = { 0.62f, 0.55f, 0.38f }, c_wall[3] = { 0.40f, 0.43f, 0.48f }, c_ski[3] = { 0.95f, 0.65f, 0.20f };
    demo_draw_object(v, s->floor, c_floor);
    for (int i = 0; i < 4; i++) demo_draw_object(v, s->wall[i], c_wall);
    demo_draw_object(v, s->body, c_ski);
    float c[3];
    demo_object_center(s->body, c);
    draw_sea(v, s, c);
    /* the four rays */
    if (s->ski_ok)
        for (int i = 0; i < 4; i++) {
            const ivp_jetski_wheel_t *w = &s->ski.wheels[i];
            ivp_vec3r_t p = ivp_vec3r(w->hp_cs.k[0], w->hp_cs.k[1], w->hp_cs.k[2]);
            float a[3];
            demo_core_point_ws(s->body, &p, a);
            const float b[3] = { a[0], a[1] + w->raycast_dist, a[2] };
            /* the ray is green when the spring is relaxed and red as it is squeezed */
            const float full = w->spring_len + w->wheel_radius, k = fminf(1.0f, fmaxf(0.0f, (full - w->raycast_dist) / 0.4f));
            const float col[3] = { 0.3f + 0.7f * k, 0.9f - 0.6f * k, 0.3f };
            demo_thick_line(v, a, b, col, 0.04f);
            demo_sphere(v, b, 0.06f + 0.1f * k, col);
        }
    demo_hud(v, DEMO_COL_INFO, "speed %.1f m/s  heading %.0f deg  tilt %.0f deg  booster %s", (double)demo_object_speed(s->body),
             (double)heading_deg(s), (double)tilt_of(s),
             s->ski_ok && ivp_jetski_get_booster_delay(&s->ski) > 0.0f ? "recharging" : "ready");
    demo_hud(v, DEMO_COL_INFO, "throttle %.1f  steer %+.1f  rays %d of %d hit the water", (double)s->throttle, (double)s->steer,
             s->rays_hit, s->rays_total);
}

static void jetski_stats(demo_app_t *app, char *buf, size_t size)
{
    jetski_demo_t *s = (jetski_demo_t *)app->user;
    float c[3];
    demo_object_center(s->body, c);
    snprintf(buf, size, "pos (%.1f %.1f %.1f) speed %.2f heading %.0f tilt max %.0f", (double)c[0], (double)c[1], (double)c[2],
             (double)demo_object_speed(s->body), (double)heading_deg(s), (double)s->tilt_max);
}

static bool jetski_check(demo_app_t *app)
{
    jetski_demo_t *s = (jetski_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    float c[3];
    demo_object_center(s->body, c);
    ok &= demo_check(tag, s->ski_ok, "the jetski controller started");
    ok &= demo_check(tag, s->rays_hit > s->rays_total / 2, "the rays hit the water (%d of %d)", s->rays_hit, s->rays_total);
    ok &= demo_check(tag, c[2] - (float)s->z0 > 8.0f, "it drove forward %.1f m", (double)(c[2] - (float)s->z0));
    ok &= demo_check(tag, s->dist_max < 1.8f, "it skims the surface (farthest %.2f m from the water)", (double)s->dist_max);
    ok &= demo_check(tag, s->tilt_max < 40.0f, "it never rolls over (tilt up to %.0f deg)", (double)s->tilt_max);
    ok &= demo_check(tag, fabsf(heading_deg(s) - s->start_heading) > 8.0f, "steering turned it (%.0f deg to %.0f deg)",
                     (double)s->start_heading, (double)heading_deg(s));
    ok &= demo_check(tag, s->speed_peak_boost > s->speed_before_boost + 1.0f, "the booster adds speed (%.1f to %.1f m/s)",
                     (double)s->speed_before_boost, (double)s->speed_peak_boost);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Jetski -- Fake Jetski on Waves";
    app.cfg.tag         = "jetski";
    app.cfg.friction    = 0.5;
    app.cfg.elasticity  = 0.1;
    app.cfg.orbit_dist  = 14.0f;
    app.cfg.orbit_yaw   = 160.0f;
    app.cfg.orbit_pitch = -22.0f;
    app.cfg.target[2]   = -28.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    jetski_demo_t *s = (jetski_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;
    s->amp = 0.25f;
    s->spring_k = 15000.0f;
    s->torque = 600.0f;

    s->floor = demo_create_box(w, NULL, SEA_X + 2.0, 0.5, SEA_Z + 2.0, 0.0, NULL, 0.0, FLOOR_Y + 0.5, 0.0);
    s->wall[0] = demo_create_box(w, NULL, 1.0, 6.0, SEA_Z + 2.0, 0.0, NULL, SEA_X + 3.0, 0.0, 0.0);
    s->wall[1] = demo_create_box(w, NULL, 1.0, 6.0, SEA_Z + 2.0, 0.0, NULL, -SEA_X - 3.0, 0.0, 0.0);
    s->wall[2] = demo_create_box(w, NULL, SEA_X + 4.0, 6.0, 1.0, 0.0, NULL, 0.0, 0.0, SEA_Z + 3.0);
    s->wall[3] = demo_create_box(w, NULL, SEA_X + 4.0, 6.0, 1.0, 0.0, NULL, 0.0, 0.0, -SEA_Z - 3.0);

    s->z0 = -28.0;
    ivp_object_template_t bt;
    const ivp_material_t m = demo_material(0.5, 0.1);
    demo_template_dynamic(&bt, &m, BODY_MASS);
    bt.rot_inertia_is_factor = false;
    bt.rot_inertia = ivp_vec3r(IVP_REAL(132.0), IVP_REAL(141.0), IVP_REAL(28.0));
    bt.speed_damp_factor = IVP_REAL(0.0);
    bt.rot_speed_damp_factor = ivp_vec3r_zero();
    demo_template_pose(&bt, NULL, 0.0, -0.9, s->z0);
    s->body = demo_create_box_templ(w, &bt, BODY_HX, BODY_HY, BODY_HZ);
    if (!s->body) return 1;

    ivp_car_system_template_t tcs;
    ivp_car_system_template_init(&tcs, 4, 2);
    tcs.car_body = s->body;
    for (int i = 0; i < 4; i++) {
        const double wx = (i & 1) ? 0.45 : -0.45, wz = (i & 2) ? -0.9 : 0.9;
        tcs.wheel_pos_Bos[i] = ivp_vec3f((float)wx, 0.3f, (float)wz);
        tcs.trace_pos_Bos[i] = ivp_vec3f((float)wx, 0.3f, (float)wz);
        tcs.wheel_radius[i] = (ivp_real_t)0.3f;
        tcs.spring_constant[i] = (ivp_real_t)s->spring_k;
        tcs.spring_damping[i] = IVP_REAL(1500.0);
        tcs.spring_damping_compression[i] = IVP_REAL(1500.0);
        tcs.spring_pre_tension[i] = (ivp_real_t)-0.35f;
    }
    tcs.stabilizer_constant[0] = IVP_REAL(2000.0);
    tcs.stabilizer_constant[1] = IVP_REAL(2000.0);
    tcs.wheel_max_rotation_speed[0] = IVP_REAL(25.0);
    tcs.wheel_max_rotation_speed[1] = IVP_REAL(25.0);
    tcs.extra_gravity_force = IVP_REAL(500.0);
    tcs.body_down_force_vertical_offset = (ivp_real_t)0.2f;
    s->ski_ok = ivp_jetski_init(&s->ski, &tcs);
    if (!s->ski_ok) {
        fprintf(stderr, "[jetski] ivp_jetski_init failed\n");
        return 1;
    }
    ivp_jetski_set_raycast(&s->ski, sea_rays, s);
    /* the reference handbrake is on: let the wheels turn */
    for (int i = 0; i < 4; i++) ivp_jetski_fix_wheel(&s->ski, (ivp_jetski_wheel_pos_t)i, false);

    app.controls    = "Up/Down: throttle   Left/Right: steer   B: booster   F5: wave, springs, torque";
    app.viewer_init = jetski_viewer_init;
    app.update      = jetski_update;
    app.on_reset    = jetski_reset;
    app.draw        = jetski_draw;
    app.stats       = jetski_stats;
    app.check       = jetski_check;

    int rc = demo_run(&app, argc, argv);
    ivp_jetski_destroy(&s->ski);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
