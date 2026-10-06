/* example_black_hole.c -- Custom controller: radial attractor
 *
 * Port of ivp_examples/examples/ive_example_black_hole.cxx.
 *
 * 120 small boxes and balls are scattered on a spiral around a "black
 * hole" at the origin.  A user controller (the reference subclasses
 * IVP_Controller_Independent) pulls every object towards the centre with
 * an inverse-square acceleration strength / d^2 (capped at 50 m/s^2, no
 * pull inside 0.5 m).  There is no ground: the objects collapse into a
 * tumbling clump around the attractor.
 *
 * Difference to the reference: the reference keeps the world gravity
 * (9.81) on, but its controller loops over ALL its objects every time it
 * is called, and the engine calls an independent controller once per
 * simulation unit -- with 120 separate objects that is dozens of calls
 * per PSI, so the effective pull is many times stronger than written and
 * beats gravity (verified with the legacy C++ engine: ~2700 calls in the first second).  The libivp
 * callback runs once per attached core, so each object is pulled exactly
 * once per PSI; with gravity on, the 80 / d^2 pull is far weaker than
 * gravity and everything just falls away.  This port therefore starts
 * with the world gravity switched off (G toggles it).
 *
 * Unlike example_forcefield (a uniform built-in force field), this shows
 * how to write an own controller with the public controller API:
 *   ivp_environment_add_controller(env, IVP_CP_ACTUATOR, fn, ctx)
 *   ivp_environment_add_controller_to_core(env, handle, object)
 * The callback runs once per PSI for every simulated core it is attached
 * to and changes the core's speed asynchronously (reference
 * async_add_speed_object_ws).
 *
 * Demonstrates: user controllers, controller priorities, per-core
 *               controller attachment, many-body collisions.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_controller.h>

#define NUM_OBJECTS 120

static const double OBJ_HS = 0.2;

typedef struct black_hole {
    ivp_vec3r_t             center;
    float                   strength;     /* inverse-square pull; also on the F5 panel */
    ivp_controller_handle_t handle;
    ivp_object_t           *objects[NUM_OBJECTS];
    long                    n_calls;       /* controller invocations */
    bool                    gravity_on;
} black_hole_t;

static void set_world_gravity(demo_app_t *app, bool on)
{
    black_hole_t *bh = (black_hole_t *)app->user;
    ivp_vec3r_t g = ivp_vec3r(0, on ? (ivp_real_t)DEMO_GRAVITY : IVP_REAL(0.0), 0);
    ivp_environment_set_gravity(app->world.env, &g);
    bh->gravity_on = on;
    /* wake everything up so the change is felt at once */
    for (int i = 0; i < NUM_OBJECTS; i++)
        ivp_environment_ensure_core_in_simulation(app->world.env, bh->objects[i]);
}

/* do_simulation_controller: applied to each core the controller is
 * attached to (the reference loops over its own object list instead). */
static void black_hole_controller(const ivp_event_sim_t *es, ivp_object_t *obj, void *ctx)
{
    black_hole_t *bh = (black_hole_t *)ctx;
    ivp_core_t *core = obj->physical_core;
    if (core->physical_unmoveable) return;
    bh->n_calls++;

    double dir[3], dist2 = 0.0;
    for (int i = 0; i < 3; i++) {
        dir[i] = (double)bh->center.k[i] - (double)core->pos_world_f_core_last_psi.k[i];
        dist2 += dir[i] * dir[i];
    }
    double dist = sqrt(dist2);
    if (dist < 0.5) return;
    double force_mag = (double)bh->strength / dist2;
    if (force_mag > 50.0) force_mag = 50.0;
    double scale = force_mag * (double)es->delta_time / dist;
    /* async_add_speed_object_ws (the core is in simulation already) */
    for (int i = 0; i < 3; i++)
        core->speed_change.k[i] += (ivp_real_t)(dir[i] * scale);
}

static void hole_distances(const black_hole_t *bh, float *mean, float *max)
{
    double sum = 0.0;
    float worst = 0.0f;
    float c[3];
    vec3r_to_float3(&bh->center, c);
    for (int i = 0; i < NUM_OBJECTS; i++) {
        float p[3];
        demo_object_center(bh->objects[i], p);
        float d = demo_distance(p, c);
        sum += (double)d;
        if (d > worst) worst = d;
    }
    *mean = (float)(sum / NUM_OBJECTS);
    *max = worst;
}

static void black_hole_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    black_hole_t *bh = (black_hole_t *)app->user;
    demo_param_float(v, "strength", &bh->strength, 0.0f, 300.0f);
}

static void black_hole_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    black_hole_t *bh = (black_hole_t *)app->user;
    (void)frame;
    if (v && demo_key_just_pressed(v, SDL_SCANCODE_G)) set_world_gravity(app, !bh->gravity_on);
}

static void black_hole_draw(demo_app_t *app, demo_viewer_t *v)
{
    black_hole_t *bh = (black_hole_t *)app->user;
    float cp[3];
    vec3r_to_float3(&bh->center, cp);
    demo_sphere(v, cp, 0.5f, DEMO_COL_WARNING);

    for (int i = 0; i < NUM_OBJECTS; i++)
        demo_draw_object(v, bh->objects[i], (i % 3 == 0) ? DEMO_COL_OBJECT_B : DEMO_COL_OBJECT_A);

    float mean, max;
    hole_distances(bh, &mean, &max);
    demo_hud(v, DEMO_COL_INFO, "Objects: %d   strength: %.0f   world gravity: %s", NUM_OBJECTS,
             (double)bh->strength, bh->gravity_on ? "ON (9.81)" : "OFF");
    demo_hud(v, DEMO_COL_INFO, "Mean distance to the hole: %.2f m   max: %.2f m",
             (double)mean, (double)max);
}

static void black_hole_stats(demo_app_t *app, char *buf, size_t size)
{
    black_hole_t *bh = (black_hole_t *)app->user;
    float mean, max, max_speed = 0.0f;
    hole_distances(bh, &mean, &max);
    for (int i = 0; i < NUM_OBJECTS; i++) {
        float sp = demo_object_speed(bh->objects[i]);
        if (sp > max_speed) max_speed = sp;
    }
    snprintf(buf, size, "mean_dist=%.3f max_dist=%.3f max_speed=%.3f controller_calls=%ld",
             (double)mean, (double)max, (double)max_speed, bh->n_calls);
}

static bool black_hole_check(demo_app_t *app)
{
    black_hole_t *bh = (black_hole_t *)app->user;
    const char *tag = app->cfg.tag;
    float mean, max;
    hole_distances(bh, &mean, &max);
    bool ok = true;
    ok &= demo_check(tag, bh->n_calls > 0, "controller was called (%ld times)", bh->n_calls);
    /* the objects start 5..12.2 m away (mean ~8.7 m) */
    ok &= demo_check(tag, mean < 5.0f, "objects collapsed towards the hole (mean distance %.2f m)",
                     (double)mean);
    ok &= demo_check(tag, max < 15.0f, "no object escaped (max distance %.2f m)", (double)max);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title      = "IVP Example: Black Hole";
    app.cfg.tag        = "black_hole";
    app.cfg.orbit_dist = 30.0f;
    app.cfg.target[1]  = 0.0f;
    app.cfg.friction   = 0.5;
    app.cfg.elasticity = 0.3;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    black_hole_t *bh = (black_hole_t *)calloc(1, sizeof(*bh));
    if (!bh) return 1;
    app.user = bh;
    bh->center = ivp_vec3r(0, 0, 0);
    bh->strength = 80.0f;

    for (int i = 0; i < NUM_OBJECTS; i++) {
        double angle = i * 2.3998;
        double radius = 5.0 + (i % 10) * 0.8;
        double x = radius * cos(angle);
        double z = radius * sin(angle);
        double y = -3.0 + (i % 7) * 1.0;
        if (i % 3 == 0)
            bh->objects[i] = demo_create_ball(&app.world, NULL, OBJ_HS, 0.5, NULL, x, y, z);
        else
            bh->objects[i] = demo_create_box(&app.world, NULL, OBJ_HS, OBJ_HS, OBJ_HS, 0.5, NULL, x, y, z);
    }

    /* Register the controller and attach it to every core (reference
     * get_controller_manager()->add_controller_to_core) */
    bh->handle = ivp_environment_add_controller(app.world.env, IVP_CP_ACTUATOR,
                                                black_hole_controller, bh);
    for (int i = 0; i < NUM_OBJECTS; i++)
        ivp_environment_add_controller_to_core(app.world.env, bh->handle, bh->objects[i]);

    /* the attractor replaces the world gravity (see the header comment) */
    set_world_gravity(&app, false);

    app.controls = "G: toggle the world gravity (reference: on)";
    app.viewer_init = black_hole_viewer_init;
    app.update = black_hole_update;
    app.draw  = black_hole_draw;
    app.stats = black_hole_stats;
    app.check = black_hole_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(bh);
    return rc;
}
