/* example_friction.c -- Six lanes, six ways to slow a crate down
 *
 * Every lane starts a 2 kg crate at 8 m/s. What differs is the friction:
 *
 *   1 rubber    contact friction 1.0
 *   2 concrete  contact friction 0.5
 *   3 ice       contact friction 0.03: the crate slides to the end wall
 *   4 ground    ice, plus a ground friction controller (Coulomb friction while
 *               the crate is near the ground plane); it should stop like 2,
 *               and it does not act on a crate that is lifted off the ground
 *   5 global    ice, plus a global friction controller: it also acts in the air
 *   6 belt      ice, plus a reference world friction controller with a desired
 *               speed against the crate: a conveyor belt that stops the crate
 *               and carries it back
 *
 * Space resets the scene. K gives the crates a push of 4 m/s again, L throws
 * them into the air. The F5 panel changes the two friction coefficients and
 * the belt.
 *
 * Demonstrates: ivp_world_friction_init_ground / init_global / init (the three
 *               modes), ivp_environment_add_controller_to_core, contact
 *               friction in the object material.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_controller_world_friction.h>

#define N_LANES    6
#define LANE_W     3.0f           /* lane pitch */
#define LANE_X0    (-10.0f)       /* lane start */
#define LANE_X1    42.0f
#define START_X    (-8.0f)
#define LAUNCH     8.0            /* m/s */
#define CRATE_HS   0.5

typedef struct lane_desc {
    const char *name;
    double      surface_mu;       /* contact friction of the lane */
    float       col[3];           /* the lane */
    float       text[3];          /* its line in the HUD */
} lane_desc_t;

static const lane_desc_t lanes[N_LANES] = {
    { "rubber",   1.00, { 0.20, 0.20, 0.22 }, { 0.80, 0.80, 0.86 } },
    { "concrete", 0.50, { 0.55, 0.55, 0.58 }, { 0.92, 0.92, 0.92 } },
    { "ice",      0.03, { 0.62, 0.84, 0.95 }, { 0.62, 0.84, 0.95 } },
    { "ground",   0.03, { 0.62, 0.84, 0.95 }, { 0.55, 0.92, 0.70 } },
    { "global",   0.03, { 0.62, 0.84, 0.95 }, { 0.98, 0.80, 0.50 } },
    { "belt",     0.03, { 0.30, 0.30, 0.34 }, { 0.98, 0.88, 0.30 } },
};

typedef struct friction_demo {
    ivp_object_t               *strip[N_LANES], *wall[N_LANES][2], *crate[N_LANES];
    ivp_world_friction_state_t  ground, global, belt;
    ivp_controller_handle_t     h_ground, h_global, h_belt;
    ivp_environment_t          *env;
    /* panel */
    float                       ground_mu, global_mu, belt_speed, belt_grip;
    /* measurements */
    float                       stop_dist[N_LANES];     /* distance when the crate came to rest, -1 = still moving */
    float                       min_vx_belt;
    float                       max_x_belt;
    float                       air_vx[2][2];           /* vx of the thrown crates 5 and 25 frames later, ground / global */
    bool                        thrown;
    float                       ice_vx;                 /* the ice crate after 6 s */
} friction_demo_t;

static float lane_z(int i) { return ((float)i - 0.5f * (float)(N_LANES - 1)) * LANE_W; }

static float crate_vx(const friction_demo_t *s, int i)
{
    return (float)s->crate[i]->physical_core->speed.k[0];
}

/* The panel values to the controllers. */
static void friction_apply(void *ctx)
{
    friction_demo_t *s = (friction_demo_t *)ctx;
    s->ground.friction_coeff = (ivp_real_t)s->ground_mu;
    s->global.friction_coeff = (ivp_real_t)s->global_mu;
    const ivp_vec3r_t want = ivp_vec3r((ivp_real_t)s->belt_speed, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_world_friction_set_desired_speed_ws(&s->belt, &want);
    /* grip along the belt; none vertically (gravity is not the belt's business), some sideways */
    const ivp_vec3r_t grip = ivp_vec3r((ivp_real_t)s->belt_grip, IVP_REAL(0.0), IVP_REAL(2.0));
    ivp_world_friction_set_friction_value_translation(&s->belt, &grip);
    for (int i = 0; i < N_LANES; i++) ivp_environment_ensure_core_in_simulation(s->env, s->crate[i]);
}

static void friction_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    friction_demo_t *s = (friction_demo_t *)app->user;
    demo_panel_t *p = &v->panel;
    demo_panel_add_float(p, "ground mu",   &s->ground_mu,  0.0f, 1.5f, friction_apply, s);
    demo_panel_add_float(p, "global mu",   &s->global_mu,  0.0f, 1.5f, friction_apply, s);
    demo_panel_add_float(p, "belt speed",  &s->belt_speed, -8.0f, 8.0f, friction_apply, s);
    demo_panel_add_float(p, "belt grip",   &s->belt_grip,  0.5f, 10.0f, friction_apply, s);
}

static void push_all(friction_demo_t *s, double dv)
{
    for (int i = 0; i < N_LANES; i++) {
        ivp_environment_ensure_core_in_simulation(s->env, s->crate[i]);
        demo_async_add_speed_ws(s->env, s->crate[i], dv, 0.0, 0.0);
    }
}

/* Throw the crates up and ahead: in the air only the global controller (and the
 * belt) still brake them. */
static void throw_crates(friction_demo_t *s, int first, int last, double ahead, double up)
{
    for (int i = first; i <= last; i++) {
        ivp_environment_ensure_core_in_simulation(s->env, s->crate[i]);
        demo_async_add_speed_ws(s->env, s->crate[i], ahead, -up, 0.0);
    }
}

static void friction_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    friction_demo_t *s = (friction_demo_t *)app->user;
    if (v && demo_key_just_pressed(v, SDL_SCANCODE_K)) {
        push_all(s, 4.0);
        for (int i = 0; i < N_LANES; i++) s->stop_dist[i] = -1.0f;
    }
    if (v && demo_key_just_pressed(v, SDL_SCANCODE_L)) throw_crates(s, 0, N_LANES - 1, 0.0, 6.0);
    if (!v) {
        /* at rest after 6 s: throw the ground and the global crate ahead and up, and
         * look at what the two controllers do to their speed in the air */
        if (frame == 360) throw_crates(s, 3, 4, 6.0, 6.0);
        if (frame == 378 || frame == 398) {
            s->air_vx[0][frame == 398] = crate_vx(s, 3);
            s->air_vx[1][frame == 398] = crate_vx(s, 4);
            if (frame == 398) s->thrown = true;
        }
        if (frame == 359) s->ice_vx = crate_vx(s, 2);
    }
    /* the belt crate would otherwise freeze at the turning point, and a frozen core runs no controller */
    ivp_environment_ensure_core_in_simulation(app->world.env, s->crate[5]);

    for (int i = 0; i < N_LANES; i++) {
        float p[3];
        demo_object_center(s->crate[i], p);
        if (s->stop_dist[i] < 0.0f && frame > 30 && demo_object_speed(s->crate[i]) < 0.05f)
            s->stop_dist[i] = p[0] - START_X;
        if (i == 5) {
            if (crate_vx(s, 5) < s->min_vx_belt) s->min_vx_belt = crate_vx(s, 5);
            if (p[0] > s->max_x_belt) s->max_x_belt = p[0];
        }
    }
}

static void friction_reset(demo_app_t *app)
{
    friction_demo_t *s = (friction_demo_t *)app->user;
    for (int i = 0; i < N_LANES; i++) s->stop_dist[i] = -1.0f;
    s->min_vx_belt = 0.0f;
    s->max_x_belt = START_X;
    friction_apply(s);
}

static void friction_draw(demo_app_t *app, demo_viewer_t *v)
{
    friction_demo_t *s = (friction_demo_t *)app->user;
    const float c_crate[3] = { 0.90f, 0.55f, 0.20f }, c_wall[3] = { 0.40f, 0.42f, 0.48f };
    const float c_mark[3] = { 0.95f, 0.95f, 0.95f }, c_stripe[3] = { 0.85f, 0.75f, 0.15f };

    for (int i = 0; i < N_LANES; i++) {
        demo_draw_object(v, s->strip[i], lanes[i].col);
        demo_draw_object(v, s->wall[i][0], c_wall);
        demo_draw_object(v, s->wall[i][1], c_wall);
        demo_draw_object(v, s->crate[i], c_crate);
    }
    /* distance marks every 5 m from the start line */
    for (int k = 0; k <= 8; k++) {
        float x = START_X + 5.0f * (float)k;
        float a[3] = { x, -0.02f, lane_z(0) - 1.4f }, b[3] = { x, -0.02f, lane_z(N_LANES - 1) + 1.4f };
        demo_line(v, a, b, c_mark);
    }
    /* the belt: stripes that move with the desired speed */
    if (v->solid) {
        const double t = ivp_environment_get_time(app->world.env);
        const float pitch = 2.0f;
        float off = fmodf((float)(t * (double)s->belt_speed), pitch);
        if (off < 0.0f) off += pitch;
        const float half[3] = { 0.12f, 0.01f, 1.3f };
        for (float x = LANE_X0 + 0.6f + off; x < LANE_X1 - 0.6f; x += pitch) {
            const float c[3] = { x, -0.01f, lane_z(5) };
            demo_solid_box(v, c, half, NULL, c_stripe);
        }
    }

    for (int i = 0; i < N_LANES; i++) {
        float p[3];
        demo_object_center(s->crate[i], p);
        float mu = (float)lanes[i].surface_mu;
        if (i == 3) mu = s->ground_mu;
        if (i == 4) mu = s->global_mu;
        char tail[48];
        if (s->stop_dist[i] >= 0.0f) snprintf(tail, sizeof(tail), "stopped after %.1f m", (double)s->stop_dist[i]);
        else snprintf(tail, sizeof(tail), "%.1f m, %.1f m/s", (double)(p[0] - START_X), (double)crate_vx(s, i));
        demo_hud(v, lanes[i].text, "%d %-8s mu %.2f  %s", i + 1, lanes[i].name, (double)mu, tail);
    }
}

static void friction_stats(demo_app_t *app, char *buf, size_t size)
{
    friction_demo_t *s = (friction_demo_t *)app->user;
    snprintf(buf, size, "vx %.2f %.2f %.2f %.2f %.2f %.2f", (double)crate_vx(s, 0), (double)crate_vx(s, 1),
             (double)crate_vx(s, 2), (double)crate_vx(s, 3), (double)crate_vx(s, 4), (double)crate_vx(s, 5));
}

static bool friction_check(demo_app_t *app)
{
    friction_demo_t *s = (friction_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    const float d0 = s->stop_dist[0], d1 = s->stop_dist[1], d3 = s->stop_dist[3];
    ok &= demo_check(tag, d0 > 0.0f && d1 > 0.0f, "rubber stopped after %.1f m, concrete after %.1f m", (double)d0,
                     (double)d1);
    ok &= demo_check(tag, d0 > 0.0f && d1 > 1.5f * d0 && d1 < 2.5f * d0,
                     "half the friction takes about twice the distance (%.1f m / %.1f m)", (double)d1, (double)d0);
    ok &= demo_check(tag, s->stop_dist[2] < 0.0f && s->ice_vx > 4.0f,
                     "the crate on ice is still sliding after 6 s (%.1f m/s)", (double)s->ice_vx);
    ok &= demo_check(tag, d3 > 0.0f && d3 > 0.7f * d1 && d3 < 1.4f * d1,
                     "the ground friction controller stops the crate like concrete (%.1f m against %.1f m)",
                     (double)d3, (double)d1);
    ok &= demo_check(tag, s->stop_dist[4] > 0.0f, "the global friction controller stops the crate (%.1f m)",
                     (double)s->stop_dist[4]);
    ok &= demo_check(tag, s->min_vx_belt < -1.5f, "the belt carries the crate back (%.1f m/s)", (double)s->min_vx_belt);
    ok &= demo_check(tag, s->max_x_belt - START_X > 8.0f && s->max_x_belt - START_X < 25.0f,
                     "the belt turned the crate around after %.1f m", (double)(s->max_x_belt - START_X));
    const float lost_ground = s->air_vx[0][0] - s->air_vx[0][1], lost_global = s->air_vx[1][0] - s->air_vx[1][1];
    ok &= demo_check(tag, s->thrown && lost_ground < 0.2f && lost_global > 1.0f,
                     "in the air the ground friction does nothing (lost %.2f m/s in 0.33 s), the global friction still brakes (%.2f m/s)",
                     (double)lost_ground, (double)lost_global);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Friction -- Contact and World Friction";
    app.cfg.tag         = "friction";
    app.cfg.friction    = 0.5;
    app.cfg.elasticity  = 0.1;
    app.cfg.orbit_dist  = 38.0f;
    app.cfg.orbit_yaw   = 20.0f;
    app.cfg.orbit_pitch = -38.0f;
    app.cfg.target[0]   = 16.0f;
    app.cfg.target[1]   = 0.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    friction_demo_t *s = (friction_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;

    const double len = 0.5 * (double)(LANE_X1 - LANE_X0), cx = 0.5 * (double)(LANE_X1 + LANE_X0);
    for (int i = 0; i < N_LANES; i++) {
        const double z = (double)lane_z(i);
        ivp_material_t lane_mat = demo_material(lanes[i].surface_mu, 0.1);
        s->strip[i] = demo_create_box(w, &lane_mat, len, 0.25, 1.35, 0.0, NULL, cx, 0.25, z);
        ivp_material_t wall_mat = demo_material(0.2, 0.2);
        s->wall[i][0] = demo_create_box(w, &wall_mat, 0.3, 0.6, 1.35, 0.0, NULL, (double)LANE_X0 - 0.3, -0.35, z);
        s->wall[i][1] = demo_create_box(w, &wall_mat, 0.3, 0.6, 1.35, 0.0, NULL, (double)LANE_X1 + 0.3, -0.35, z);
        ivp_material_t crate_mat = demo_material(1.0, 0.05);
        s->crate[i] = demo_create_box(w, &crate_mat, CRATE_HS, CRATE_HS, CRATE_HS, 2.0, NULL,
                                      (double)START_X, -CRATE_HS - 0.02, z);
        if (!s->strip[i] || !s->crate[i]) return 1;
        s->crate[i]->physical_core->speed = ivp_vec3r((ivp_real_t)LAUNCH, IVP_REAL(0.0), IVP_REAL(0.0));
        s->stop_dist[i] = -1.0f;
    }

    /* the ground plane is y = 0, "out of the ground" is -Y (IVP worlds are Y down) */
    const ivp_vec3r_t up = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    s->ground_mu = 0.5f;
    s->global_mu = 0.5f;
    s->belt_speed = -3.0f;
    s->belt_grip = 2.0f;
    ivp_world_friction_init_ground(&s->ground, &up, IVP_REAL(0.0), (ivp_real_t)s->ground_mu);
    ivp_world_friction_init_global(&s->global, (ivp_real_t)s->global_mu);
    ivp_world_friction_init(&s->belt, s->crate[5]);
    s->h_ground = ivp_environment_add_controller(w->env, IVP_CP_ACTUATOR, ivp_controller_world_friction, &s->ground);
    s->h_global = ivp_environment_add_controller(w->env, IVP_CP_ACTUATOR, ivp_controller_world_friction, &s->global);
    s->h_belt   = ivp_environment_add_controller(w->env, IVP_CP_ACTUATOR, ivp_controller_world_friction, &s->belt);
    ivp_environment_add_controller_to_core(w->env, s->h_ground, s->crate[3]);
    ivp_environment_add_controller_to_core(w->env, s->h_global, s->crate[4]);
    ivp_environment_add_controller_to_core(w->env, s->h_belt, s->crate[5]);
    friction_apply(s);
    s->max_x_belt = START_X;

    app.controls    = "K: push the crates (+4 m/s)   L: throw them up   F5: friction and belt";
    app.viewer_init = friction_viewer_init;
    app.update      = friction_update;
    app.on_reset    = friction_reset;
    app.draw        = friction_draw;
    app.stats       = friction_stats;
    app.check       = friction_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
