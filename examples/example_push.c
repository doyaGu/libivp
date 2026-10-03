/* example_push.c -- Applying speed changes and impulses to an object
 *
 * Port of ivp_examples/quickstart/ive_quickstart_force.cxx
 * (IVP manual section 5.5, Applying forces to an object).
 *
 * A 1 m cube rests on the ground.
 *   J  async_add_speed_object_ws: add 3 m/s upward speed (uniform, no
 *      rotation; the reference uses S, a camera key in the libivp examples)
 *   U  async_push_object_ws: an upward impulse of 100 N*s at the corner
 *      (0.5, 0.5, 0.5) of the cube: the cube jumps and spins.  (This is the
 *      reference value: on a 1 kg cube it is a 100 m/s kick, the cube flies
 *      a few hundred metres high and needs ~20 s to come back -- the same
 *      happens with the legacy engine.)
 *
 * Demonstrates: asynchronous speed changes / impulses (applied at the next
 *               PSI), impulses at a point creating torque, waking up a
 *               resting (frozen) object (ivp_environment_ensure_core_in_simulation),
 *               object -> world coordinates (reference
 *               transform_position_to_world_coords).
 */
#include "demo_app.h"

static const double CUBE_HS = 0.5;

typedef struct push_demo {
    ivp_object_t *ground;
    ivp_object_t *cube;
    const char   *last_action;
    float         peak_height;    /* since the last action */
    float         peak_rot_speed;
    float         peak_tilt;
    float         speed_jump_height;  /* headless: result of the J action */
    float         speed_jump_spin;
} push_demo_t;

static void begin_action(push_demo_t *s, const char *what)
{
    s->last_action = what;
    s->peak_height = 0.0f;
    s->peak_rot_speed = 0.0f;
    s->peak_tilt = 0.0f;
    printf("[push] %s\n", what);
}

static void add_speed(demo_app_t *app)
{
    push_demo_t *s = (push_demo_t *)app->user;
    begin_action(s, "add speed (0, -3, 0)");
    demo_async_add_speed_ws(app->world.env, s->cube, 0.0, -3.0, 0.0);
}

static void push_corner(demo_app_t *app)
{
    push_demo_t *s = (push_demo_t *)app->user;
    begin_action(s, "push corner with (0, -100, 0) N*s");
    ivp_vec3r_t corner_os = ivp_vec3r((ivp_real_t)CUBE_HS, (ivp_real_t)CUBE_HS, (ivp_real_t)CUBE_HS);
    float corner_ws[3];
    demo_core_point_ws(s->cube, &corner_os, corner_ws);
    ivp_vec3r_t point = ivp_vec3r(corner_ws[0], corner_ws[1], corner_ws[2]);
    ivp_vec3r_t impulse = ivp_vec3r(0, IVP_REAL(-100.0), 0);
    demo_async_push_ws(app->world.env, s->cube, &point, &impulse);
}

static void track(push_demo_t *s)
{
    float h = (float)(-CUBE_HS - s->cube->physical_core->pos_world_f_core_last_psi.k[1]);
    if (h > s->peak_height) s->peak_height = h;
    const ivp_vec3f_t *w = &s->cube->physical_core->rot_speed;
    float rs = sqrtf(w->k[0] * w->k[0] + w->k[1] * w->k[1] + w->k[2] * w->k[2]);
    if (rs > s->peak_rot_speed) s->peak_rot_speed = rs;
    float tilt = demo_object_tilt_deg(s->cube);
    if (tilt > s->peak_tilt) s->peak_tilt = tilt;
}

static void push_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    push_demo_t *s = (push_demo_t *)app->user;
    track(s);
    if (v) {
        if (demo_key_just_pressed(v, SDL_SCANCODE_J)) add_speed(app);
        if (demo_key_just_pressed(v, SDL_SCANCODE_U)) push_corner(app);
        return;
    }
    if (frame == 90) add_speed(app);
    if (frame == 100) {
        /* in the air: a centre push does not rotate the cube */
        s->speed_jump_spin = s->peak_rot_speed;
    }
    if (frame == 160) {
        printf("[push] after add speed: peak height %.3f m (expected v^2/2g = %.3f m), spin in the air %.4f rad/s\n",
               (double)s->peak_height, 9.0 / (2.0 * DEMO_GRAVITY), (double)s->speed_jump_spin);
        s->speed_jump_height = s->peak_height;
    }
    if (frame == 300) push_corner(app);
}

static void push_draw(demo_app_t *app, demo_viewer_t *v)
{
    push_demo_t *s = (push_demo_t *)app->user;
    demo_draw_object(v, s->ground, DEMO_COL_STATIC);
    demo_draw_object(v, s->cube, DEMO_COL_OBJECT_B);
    demo_draw_velocity(v, s->cube, DEMO_COL_VELOCITY);
    /* the corner that U pushes */
    ivp_vec3r_t corner_os = ivp_vec3r((ivp_real_t)CUBE_HS, (ivp_real_t)CUBE_HS, (ivp_real_t)CUBE_HS);
    float corner_ws[3];
    demo_core_point_ws(s->cube, &corner_os, corner_ws);
    demo_point(v, corner_ws, 8.0f, DEMO_COL_HIGHLIGHT);

    demo_hud_object(v, "Cube", s->cube);
    if (s->last_action) {
        demo_hud(v, DEMO_COL_HIGHLIGHT, "Last: %s", s->last_action);
        demo_hud(v, DEMO_COL_INFO, "peak height %.2f m   peak spin %.2f rad/s",
                 (double)s->peak_height, (double)s->peak_rot_speed);
    }
}

static void push_stats(demo_app_t *app, char *buf, size_t size)
{
    push_demo_t *s = (push_demo_t *)app->user;
    snprintf(buf, size, "y=%.3f speed=%.3f peak_h=%.3f peak_spin=%.3f peak_tilt=%.1f%s",
             (double)s->cube->physical_core->pos_world_f_core_last_psi.k[1], (double)demo_object_speed(s->cube),
             (double)s->peak_height, (double)s->peak_rot_speed, (double)s->peak_tilt,
             ivp_object_is_frozen(s->cube) ? " frozen" : "");
}

static bool push_check(demo_app_t *app)
{
    push_demo_t *s = (push_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    /* v = 3 m/s -> h = 0.46 m (minus a little damping) */
    ok &= demo_check(tag, s->speed_jump_height > 0.35f && s->speed_jump_height < 0.5f,
                     "add speed: jump height %.3f m (v^2/2g = 0.459 m)", (double)s->speed_jump_height);
    ok &= demo_check(tag, s->speed_jump_spin < 0.05f,
                     "add speed: no rotation in the air (%.4f rad/s)", (double)s->speed_jump_spin);
    ok &= demo_check(tag, s->peak_rot_speed > 10.0f && s->peak_height > 50.0f,
                     "corner push: cube was launched (%.1f m so far) and spins (%.1f rad/s)",
                     (double)s->peak_height, (double)s->peak_rot_speed);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title      = "IVP Quickstart: Force";
    app.cfg.tag        = "push";
    app.cfg.orbit_dist = 15.0f;
    app.cfg.target[1]  = -3.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    push_demo_t *s = (push_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;

    s->ground = demo_create_box(&app.world, NULL, 10.0, 0.5, 10.0, 0.0, NULL, 0.0, 0.5, 0.0);
    s->cube = demo_create_box(&app.world, NULL, CUBE_HS, CUBE_HS, CUBE_HS, 1.0, NULL, 0.0, -CUBE_HS, 0.0);

    app.controls = "J: add speed upward   U: push at the corner (torque)";
    app.update   = push_update;
    app.draw     = push_draw;
    app.stats    = push_stats;
    app.check    = push_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
