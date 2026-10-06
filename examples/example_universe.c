/* example_universe.c -- Universe manager: streaming static objects in/out
 *
 * Port of ivp_examples/examples/ive_example_universe.cxx.
 *
 * A 10x10 "world" of static boxes is described in a table, but none of the
 * boxes exists in the environment at start-up.  A universe manager adds
 * the boxes near moving objects on demand and removes boxes nobody checks
 * for collision any more:
 *   ensure_fn   (IVP_Universe_Manager::ensure_objects_in_environment)
 *               creates every table box within the requested sphere,
 *   evict_fn    (object_no_longer_needed) deletes a box,
 *   deleted_fn  (event_object_deleted) keeps the table in sync,
 *   settings    (provide_universe_settings) thresholds 10/5 and 30/20.
 * One ball is dropped at start-up; the HUD shows how many table boxes are
 * currently in / out of the simulation.
 *   N       drop another ball
 *   X / Del delete the focused ball (Tab to focus; the reference uses D,
 *           which is a camera key in the libivp examples)
 *
 * Demonstrates: ivp_universe_manager_t, ivp_environment_set_universe_manager,
 *               creating / destroying objects at runtime (also from inside
 *               simulation callbacks).
 */
#include "demo_app.h"

#include <ivp/dynamics/ivp_universe_manager.h>

#define WORLD_GRID_W 10
#define WORLD_GRID_D 10
#define WORLD_TOTAL  (WORLD_GRID_W * WORLD_GRID_D)
#define MAX_BALLS    16

static const double WORLD_BOX_HS = 0.35;

typedef struct world_box {
    double        wx, wz;   /* world position (y = 0.4) */
    ivp_object_t *obj;      /* NULL while outside the simulation */
} world_box_t;

typedef struct ball_entry {
    ivp_object_t *obj;
    double        radius;
} ball_entry_t;

typedef struct universe {
    demo_world_t *world;
    ivp_material_t mat;
    ivp_object_t *ground;
    world_box_t   boxes[WORLD_TOTAL];
    int           objects_in_sim;
    int           objects_outside;
    int           max_in_sim;
    int           n_created;   /* ensure_fn insertions */
    int           n_evicted;   /* evict_fn removals    */
    ball_entry_t  balls[MAX_BALLS];
    int           n_balls;
    unsigned int  seed;
} universe_t;

/* ── Universe manager callbacks ───────────────────────────────────────── */

/* ensure_objects_in_environment: every non-moving object within the sphere
 * around a moving object must be in the simulation.  A box is within the
 * sphere as soon as any part of it is (its bounding sphere intersects the
 * requested one): testing the box centres only (as the reference example
 * does) let a slowly rolling ball move into the space of a box that was
 * streamed in afterwards; the box then appeared inside the ball and the
 * interpenetration shot the ball away at hundreds of m/s. */
static void um_ensure(void *ctx, ivp_object_t *requester, const float *center, double radius)
{
    universe_t *u = (universe_t *)ctx;
    (void)requester;
    double r = radius + WORLD_BOX_HS * sqrt(3.0);
    double r2 = r * r;
    for (int i = 0; i < WORLD_TOTAL; i++) {
        world_box_t *wb = &u->boxes[i];
        double dx = wb->wx - (double)center[0];
        double dz = wb->wz - (double)center[2];
        if (dx * dx + dz * dz >= r2 || wb->obj) continue;
        wb->obj = demo_create_box(u->world, &u->mat, WORLD_BOX_HS, WORLD_BOX_HS, WORLD_BOX_HS, 0.0,
                                  NULL, wb->wx, 0.4, wb->wz);
        if (!wb->obj) continue;
        u->objects_in_sim++;
        u->objects_outside--;
        u->n_created++;
        if (u->objects_in_sim > u->max_in_sim) u->max_in_sim = u->objects_in_sim;
    }
}

/* object_no_longer_needed: nobody checks this object for collisions. */
static void um_evict(void *ctx, ivp_object_t *obj)
{
    universe_t *u = (universe_t *)ctx;
    for (int i = 0; i < WORLD_TOTAL; i++) {
        if (u->boxes[i].obj == obj) {
            u->n_evicted++;
            /* um_deleted updates the table and the counters */
            demo_destroy_object(u->world, obj);
            return;
        }
    }
}

/* event_object_deleted: drop our reference before the object goes away. */
static void um_deleted(void *ctx, ivp_object_t *obj)
{
    universe_t *u = (universe_t *)ctx;
    for (int i = 0; i < WORLD_TOTAL; i++) {
        if (u->boxes[i].obj == obj) {
            u->boxes[i].obj = NULL;
            u->objects_in_sim--;
            u->objects_outside++;
            return;
        }
    }
}

/* ── Balls ────────────────────────────────────────────────────────────── */

static void spawn_ball(universe_t *u, double x, double y, double z, double radius, double mass)
{
    if (u->n_balls >= MAX_BALLS) return;
    ball_entry_t *e = &u->balls[u->n_balls];
    e->radius = radius;
    e->obj = demo_create_ball(u->world, &u->mat, radius, mass, NULL, x, y, z);
    if (e->obj) u->n_balls++;
}

static void spawn_random_ball(universe_t *u)
{
    u->seed = u->seed * 1103515245u + 12345u;
    double x = ((int)((u->seed >> 16) % 100) - 50) * 0.05;
    u->seed = u->seed * 1103515245u + 12345u;
    double z = ((int)((u->seed >> 16) % 100) - 50) * 0.05;
    double radius = 0.3 + ((u->seed >> 20) % 20) * 0.01;
    spawn_ball(u, x, -6.0, z, radius, 1.5);
}

static void delete_ball(universe_t *u, demo_viewer_t *v, ivp_object_t *obj)
{
    for (int i = 0; i < u->n_balls; i++) {
        if (u->balls[i].obj != obj) continue;
        demo_viewer_forget(v, obj);
        demo_destroy_object(u->world, obj);
        for (int j = i; j < u->n_balls - 1; j++) u->balls[j] = u->balls[j + 1];
        u->n_balls--;
        return;
    }
}

static void universe_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    universe_t *u = (universe_t *)app->user;
    if (v) {
        if (demo_key_just_pressed(v, SDL_SCANCODE_N)) spawn_random_ball(u);
        bool del = demo_key_just_pressed(v, SDL_SCANCODE_X);
        del = demo_key_just_pressed(v, SDL_SCANCODE_DELETE) || del;
        if (del && v->focus.focused) delete_ball(u, v, v->focus.focused);
        return;
    }
    /* headless script: drop five more balls, then delete two of them */
    if (frame >= 60 && frame <= 300 && frame % 60 == 0) spawn_random_ball(u);
    if (frame == 420 && u->n_balls > 1) delete_ball(u, NULL, u->balls[1].obj);
    if (frame == 480 && u->n_balls > 0) delete_ball(u, NULL, u->balls[0].obj);
}

static void universe_draw(demo_app_t *app, demo_viewer_t *v)
{
    universe_t *u = (universe_t *)app->user;
    demo_draw_object(v, u->ground, DEMO_COL_STATIC);

    /* table boxes: in the simulation solid colour, outside as small marks */
    for (int i = 0; i < WORLD_TOTAL; i++) {
        if (u->boxes[i].obj) {
            demo_draw_object(v, u->boxes[i].obj, DEMO_COL_OBJECT_A);
        } else {
            float p[3] = { (float)u->boxes[i].wx, 0.4f, (float)u->boxes[i].wz };
            demo_cross(v, p, 0.15f, DEMO_COL_HELP);
        }
    }
    for (int i = 0; i < u->n_balls; i++)
        demo_draw_object(v, u->balls[i].obj, DEMO_COL_OBJECT_B);

    demo_hud(v, DEMO_COL_INFO, "World boxes total: %d", WORLD_TOTAL);
    demo_hud(v, DEMO_COL_INFO, "  In simulation:  %d", u->objects_in_sim);
    demo_hud(v, DEMO_COL_INFO, "  Outside sim:    %d", u->objects_outside);
    demo_hud(v, DEMO_COL_INFO, "Dynamic balls:    %d", u->n_balls);
    demo_hud(v, DEMO_COL_INFO, "Objects in environment: %d",
             ivp_environment_get_object_count(app->world.env));
    demo_hud(v, DEMO_COL_HELP, "ensure_objects_in_environment() -> %d boxes added", u->n_created);
    demo_hud(v, DEMO_COL_HELP, "object_no_longer_needed()       -> %d boxes removed", u->n_evicted);
}

static void universe_stats(demo_app_t *app, char *buf, size_t size)
{
    universe_t *u = (universe_t *)app->user;
    float max_speed = 0.0f;
    for (int i = 0; i < u->n_balls; i++) {
        float sp = demo_object_speed(u->balls[i].obj);
        if (sp > max_speed) max_speed = sp;
    }
    snprintf(buf, size, "in_sim=%d outside=%d created=%d evicted=%d balls=%d env_objects=%d ball_max_speed=%.2f",
             u->objects_in_sim, u->objects_outside, u->n_created, u->n_evicted, u->n_balls,
             ivp_environment_get_object_count(app->world.env), (double)max_speed);
}

static bool universe_check(demo_app_t *app)
{
    universe_t *u = (universe_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    int in_table = 0;
    for (int i = 0; i < WORLD_TOTAL; i++)
        if (u->boxes[i].obj) in_table++;
    ok &= demo_check(tag, u->n_created > 0, "ensure_fn streamed boxes in (%d insertions)", u->n_created);
    ok &= demo_check(tag, u->max_in_sim < WORLD_TOTAL,
                     "only part of the world was ever in the simulation (max %d/%d)",
                     u->max_in_sim, WORLD_TOTAL);
    ok &= demo_check(tag, in_table == u->objects_in_sim &&
                          u->objects_in_sim + u->objects_outside == WORLD_TOTAL,
                     "table and counters agree (%d in, %d out)", u->objects_in_sim, u->objects_outside);
    ok &= demo_check(tag, ivp_environment_get_object_count(app->world.env) ==
                          1 + u->objects_in_sim + u->n_balls,
                     "environment holds ground + %d boxes + %d balls", u->objects_in_sim, u->n_balls);
    bool balls_ok = true;
    for (int i = 0; i < u->n_balls; i++)
        if (u->balls[i].obj->physical_core->pos_world_f_core_last_psi.k[1] > 0.5) balls_ok = false;
    ok &= demo_check(tag, balls_ok, "balls stay on top of the ground / boxes");
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Example: Universe Manager";
    app.cfg.tag         = "universe";
    app.cfg.orbit_dist  = 17.0f;
    app.cfg.orbit_pitch = -35.0f;
    app.cfg.target[1]   = -1.0f;
    app.cfg.friction    = 0.5;
    app.cfg.elasticity  = 0.6;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    universe_t *u = (universe_t *)calloc(1, sizeof(*u));
    if (!u) return 1;
    app.user = u;
    u->world = &app.world;
    u->mat = app.world.mat;
    u->seed = 54321u;

    /* World table: box positions, none of them in the simulation yet */
    for (int xi = 0; xi < WORLD_GRID_W; xi++) {
        for (int zi = 0; zi < WORLD_GRID_D; zi++) {
            world_box_t *wb = &u->boxes[xi * WORLD_GRID_D + zi];
            wb->wx = (xi - WORLD_GRID_W / 2) * 1.5;
            wb->wz = (zi - WORLD_GRID_D / 2) * 1.5;
            wb->obj = NULL;
        }
    }
    u->objects_in_sim = 0;
    u->objects_outside = WORLD_TOTAL;

    /* Universe manager (the reference installs it with the environment) */
    {
        ivp_universe_manager_t um;
        ivp_universe_manager_init(&um);
        um.ensure_fn  = um_ensure;
        um.evict_fn   = um_evict;
        um.deleted_fn = um_deleted;
        um.ctx        = u;
        um.num_objects_in_environment_threshold_0 = 10;
        um.check_objects_per_second_threshold_0   = 5;
        um.num_objects_in_environment_threshold_1 = 30;
        um.check_objects_per_second_threshold_1   = 20;
        ivp_environment_set_universe_manager(app.world.env, &um);
    }

    /* One large static ground plane, always in the simulation */
    u->ground = demo_create_box(&app.world, &u->mat, 20.0, 0.5, 20.0, 0.0, NULL, 0.0, 1.0, 0.0);

    /* One initial ball */
    spawn_ball(u, 0.0, -4.0, 0.0, 0.45, 2.0);

    app.controls = "N: drop a ball   X/Del: delete the focused ball (Tab: focus)";
    app.update   = universe_update;
    app.draw     = universe_draw;
    app.stats    = universe_stats;
    app.check    = universe_check;

    int rc = demo_run(&app, argc, argv);
    /* the environment's teardown reports deletions to um_deleted (u) */
    demo_world_destroy(&app.world);
    free(u);
    return rc;
}
