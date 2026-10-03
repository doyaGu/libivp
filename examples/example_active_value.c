/* example_active_value.c -- Active value chain
 *
 * Port of ivp_examples/quickstart/ive_quickstart_active_value.cxx
 * (IVP manual section 5.11, Active Values).
 *
 * An active value chain  result = speed * factor  (factor = 1.5) is built
 * from two terminals and a multiplication node.  Pressing P pushes cube 1
 * upwards with 4 m/s, sets the "speed" terminal to 4 and pushes cube 2
 * with the value the chain computes (6 m/s), so cube 2 jumps higher.
 * (The reference uses SPACE, which resets the scene in the libivp
 * examples.)
 *
 * In addition to the reference, the three values are registered by name
 * in the environment's active value manager and a listener on "result"
 * logs every change of the chain.
 *
 * Demonstrates: ivp_active_float_terminal_create / ivp_active_float_mult_create
 *               (IVP_U_Active_Terminal_Double / IVP_U_Active_Mult),
 *               ivp_active_float_set, change propagation, listeners,
 *               ivp_environment_get_active_value_manager.
 */
#include "demo_app.h"

#include <ivp/utility/ivp_active_value.h>

static const double CUBE_HS = 0.4;

typedef struct active_value_demo {
    ivp_object_t       *ground;
    ivp_object_t       *cube1;
    ivp_object_t       *cube2;
    ivp_active_float_t *av_speed;
    ivp_active_float_t *av_factor;
    ivp_active_float_t *av_mult;
    int                 n_changes;     /* listener notifications */
    double              last_result;
    int                 n_pushes;
    float               peak1, peak2;  /* highest point (smallest y) after a push */
} active_value_demo_t;

static void result_changed(void *ctx, ivp_active_float_t *source, double old_value, double new_value)
{
    active_value_demo_t *s = (active_value_demo_t *)ctx;
    s->n_changes++;
    s->last_result = new_value;
    printf("[active_value] '%s' changed %.2f -> %.2f\n", ivp_active_float_name(source),
           old_value, new_value);
}

static void push_cubes(demo_app_t *app)
{
    active_value_demo_t *s = (active_value_demo_t *)app->user;
    /* push cube 1 up (negative Y is up) */
    demo_async_add_speed_ws(app->world.env, s->cube1, 0.0, -4.0, 0.0);
    /* set the active value; cube 2 gets the multiplied speed */
    ivp_active_float_set(s->av_speed, 4.0);
    double result = ivp_active_float_get(s->av_mult);
    demo_async_add_speed_ws(app->world.env, s->cube2, 0.0, -result, 0.0);
    s->n_pushes++;
    s->peak1 = s->peak2 = 0.0f;
}

static void track_peaks(active_value_demo_t *s)
{
    float y1 = (float)s->cube1->physical_core->pos_world_f_core_last_psi.k[1];
    float y2 = (float)s->cube2->physical_core->pos_world_f_core_last_psi.k[1];
    if (y1 < s->peak1) s->peak1 = y1;
    if (y2 < s->peak2) s->peak2 = y2;
}

static void active_value_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    active_value_demo_t *s = (active_value_demo_t *)app->user;
    track_peaks(s);
    if (v) {
        if (demo_key_just_pressed(v, SDL_SCANCODE_P)) push_cubes(app);
        return;
    }
    /* headless: push once the cubes rest */
    if (frame == 60) push_cubes(app);
}

static void active_value_draw(demo_app_t *app, demo_viewer_t *v)
{
    active_value_demo_t *s = (active_value_demo_t *)app->user;
    demo_draw_object(v, s->ground, DEMO_COL_STATIC);
    demo_draw_object(v, s->cube1, DEMO_COL_OBJECT_B);
    demo_draw_object(v, s->cube2, DEMO_COL_OBJECT_A);
    demo_draw_velocity(v, s->cube1, DEMO_COL_VELOCITY);
    demo_draw_velocity(v, s->cube2, DEMO_COL_VELOCITY);

    demo_hud(v, DEMO_COL_INFO, "speed(%.1f) * factor(%.1f) = %.1f",
             ivp_active_float_get(s->av_speed), ivp_active_float_get(s->av_factor),
             ivp_active_float_get(s->av_mult));
    demo_hud(v, DEMO_COL_INFO, "listener notifications: %d", s->n_changes);
    demo_hud_object(v, "Cube1 (orange)", s->cube1);
    demo_hud_object(v, "Cube2 (blue)", s->cube2);
    if (s->n_pushes > 0)
        demo_hud(v, DEMO_COL_INFO, "Jump height: cube1 %.2f m, cube2 %.2f m",
                 (double)(-CUBE_HS - s->peak1), (double)(-CUBE_HS - s->peak2));
}

static void active_value_stats(demo_app_t *app, char *buf, size_t size)
{
    active_value_demo_t *s = (active_value_demo_t *)app->user;
    snprintf(buf, size, "result=%.2f y1=%.3f y2=%.3f peak1=%.3f peak2=%.3f changes=%d",
             ivp_active_float_get(s->av_mult),
             (double)s->cube1->physical_core->pos_world_f_core_last_psi.k[1],
             (double)s->cube2->physical_core->pos_world_f_core_last_psi.k[1],
             (double)s->peak1, (double)s->peak2, s->n_changes);
}

static bool active_value_check(demo_app_t *app)
{
    active_value_demo_t *s = (active_value_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, fabs(ivp_active_float_get(s->av_mult) - 6.0) < 1e-9,
                     "chain computed 4 * 1.5 = %.3f", ivp_active_float_get(s->av_mult));
    ok &= demo_check(tag, s->n_changes == 1 && fabs(s->last_result - 6.0) < 1e-9,
                     "listener saw one change (%d)", s->n_changes);
    {
        ivp_active_value_manager_t *m = ivp_environment_get_active_value_manager(app->world.env);
        ivp_active_float_t *r = m ? ivp_active_value_manager_get_float(m, "result") : NULL;
        ok &= demo_check(tag, r == s->av_mult, "manager finds 'result' by name");
    }
    /* h = v^2 / 2g: 0.82 m for 4 m/s, 1.83 m for 6 m/s */
    float h1 = (float)(-CUBE_HS) - s->peak1, h2 = (float)(-CUBE_HS) - s->peak2;
    ok &= demo_check(tag, h1 > 0.5f && h2 > 1.4f * h1,
                     "cube2 jumped higher (%.2f m vs %.2f m)", (double)h2, (double)h1);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title      = "IVP Quickstart: Active Value";
    app.cfg.tag        = "active_value";
    app.cfg.orbit_dist = 18.0f;
    app.cfg.target[1]  = -3.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    active_value_demo_t *s = (active_value_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;

    s->ground = demo_create_box(&app.world, NULL, 10.0, 0.5, 10.0, 0.0, NULL, 0.0, 0.5, 0.0);
    s->cube1 = demo_create_box(&app.world, NULL, CUBE_HS, CUBE_HS, CUBE_HS, 1.0, NULL, -2.0, -CUBE_HS, 0.0);
    s->cube2 = demo_create_box(&app.world, NULL, CUBE_HS, CUBE_HS, CUBE_HS, 1.0, NULL,  2.0, -CUBE_HS, 0.0);

    /* Active value chain: a = speed terminal, b = factor (1.5), c = a * b */
    s->av_speed  = ivp_active_float_terminal_create(NULL, "speed", 0.0);
    s->av_factor = ivp_active_float_terminal_create(NULL, "factor", 1.5);
    s->av_mult   = ivp_active_float_mult_create(NULL, "result", s->av_speed, s->av_factor);
    ivp_active_float_add_listener(s->av_mult, result_changed, s);

    /* make the chain visible to everybody using the environment */
    {
        ivp_active_value_manager_t *m = ivp_environment_get_active_value_manager(app.world.env);
        if (m) {
            ivp_active_value_manager_insert_float(m, s->av_speed);
            ivp_active_value_manager_insert_float(m, s->av_factor);
            ivp_active_value_manager_insert_float(m, s->av_mult);
        }
    }

    app.controls = "P: push cube1 (cube2 gets speed * factor = 1.5x)";
    app.update   = active_value_update;
    app.draw     = active_value_draw;
    app.stats    = active_value_stats;
    app.check    = active_value_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);   /* drops the manager references */
    ivp_active_float_remove_listener(s->av_mult, result_changed, s);
    ivp_active_float_remove_ref(s->av_mult);
    ivp_active_float_remove_ref(s->av_factor);
    ivp_active_float_remove_ref(s->av_speed);
    free(s);
    return rc;
}
