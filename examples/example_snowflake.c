/* example_snowflake.c -- Spring platform with falling balls
 *
 * Port of ivp_examples/examples/ive_example_snowflake.cxx.
 *
 * A 4x4 grid of flat boxes hangs in the air; neighbouring boxes are
 * connected by springs (k = 8, damping 0.2, rest length = grid spacing)
 * and the four corner boxes are pinned.  Six balls are dropped onto the
 * sagging, trampoline-like platform.
 *
 * Like in the reference (checked against the legacy C++ engine, which
 * produces the same platform motion) the platform is very soft: it sags
 * about 3 m and keeps swinging, the six 0.5 m balls are narrower than the
 * 0.6 m gaps between the boxes and drop through, and there is no ground.
 *
 * Demonstrates: ivp_actuator_spring networks, pinned objects
 *               (ivp_object_set_pinned, reference IVP_Real_Object::
 *               set_pinned), balls bouncing on a spring-supported surface.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_actuator_spring.h>

#define GRID_SIZE    4
#define NUM_PLATFORM (GRID_SIZE * GRID_SIZE)
#define NUM_DROPS    6
#define MAX_SPRINGS  (2 * GRID_SIZE * (GRID_SIZE - 1))

static const double BOX_HS  = 0.3;
static const double SPACING = 1.2;
static const double DROP_R  = 0.25;
static const double SPRING_K = 8.0;
static const double SPRING_DAMP = 0.2;

typedef struct snowflake {
    ivp_object_t      *platform[NUM_PLATFORM];
    ivp_object_t      *drops[NUM_DROPS];
    ivp_spring_state_t springs[MAX_SPRINGS];
    int                n_springs;
    ivp_vec3r_t        corner_start[4];
    float              max_center_y;    /* lowest point of the centre (y down) */
    bool               passed[NUM_DROPS];
    int                n_passed;        /* balls that fell through the platform */
} snowflake_t;

static const int CORNERS[4] = { 0, GRID_SIZE - 1, NUM_PLATFORM - GRID_SIZE, NUM_PLATFORM - 1 };

static void add_spring(snowflake_t *s, ivp_environment_t *env, int a, int b)
{
    ivp_spring_state_t *sp = &s->springs[s->n_springs++];
    ivp_vec3r_t zero = ivp_vec3r(0, 0, 0);
    ivp_spring_state_init(sp);
    ivp_spring_state_setup(sp, s->platform[a], &zero, s->platform[b], &zero,
                           (ivp_real_t)SPRING_K, (ivp_real_t)SPRING_DAMP, (ivp_real_t)SPACING);
    ivp_environment_add_controller(env, IVP_CP_SPRING, ivp_controller_spring, sp);
}

static float platform_center_y(const snowflake_t *s)
{
    /* average height of the four inner boxes */
    const int inner[4] = { 5, 6, 9, 10 };
    float y = 0.0f;
    for (int i = 0; i < 4; i++) y += (float)s->platform[inner[i]]->physical_core->pos_world_f_core_last_psi.k[1];
    return y * 0.25f;
}

static float corner_drift(const snowflake_t *s)
{
    float worst = 0.0f;
    for (int i = 0; i < 4; i++) {
        float a[3], b[3];
        demo_object_center(s->platform[CORNERS[i]], a);
        vec3r_to_float3(&s->corner_start[i], b);
        float d = demo_distance(a, b);
        if (d > worst) worst = d;
    }
    return worst;
}

static int drops_on_platform(const snowflake_t *s)
{
    int n = 0;
    for (int i = 0; i < NUM_DROPS; i++) {
        const ivp_vec3r_t *p = &s->drops[i]->physical_core->pos_world_f_core_last_psi;
        if (fabs((double)p->k[0]) < 2.6 && fabs((double)p->k[2]) < 2.6 && p->k[1] < -1.0 && p->k[1] > -6.0)
            n++;
    }
    return n;
}

/* track the motion of the platform centre (start height -2) */
static void snowflake_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    snowflake_t *s = (snowflake_t *)app->user;
    (void)v;
    (void)frame;
    float cy = platform_center_y(s);
    if (cy > s->max_center_y) s->max_center_y = cy;
    for (int i = 0; i < NUM_DROPS; i++) {
        const ivp_vec3r_t *p = &s->drops[i]->physical_core->pos_world_f_core_last_psi;
        if (p->k[1] > 8.0 && !s->passed[i]) { s->passed[i] = true; s->n_passed++; }
    }
}

static void snowflake_reset(demo_app_t *app)
{
    snowflake_t *s = (snowflake_t *)app->user;
    s->max_center_y = -2.0f;
    s->n_passed = 0;
    memset(s->passed, 0, sizeof(s->passed));
}

static void snowflake_draw(demo_app_t *app, demo_viewer_t *v)
{
    snowflake_t *s = (snowflake_t *)app->user;
    for (int i = 0; i < NUM_PLATFORM; i++) {
        bool pinned = s->platform[i]->physical_core->pinned;
        demo_draw_object(v, s->platform[i], pinned ? DEMO_COL_STATIC : DEMO_COL_SPRING);
    }
    for (int i = 0; i < s->n_springs; i++)
        demo_draw_link(v, s->springs[i].obj_a, s->springs[i].obj_b, DEMO_COL_HIGHLIGHT);
    for (int i = 0; i < NUM_DROPS; i++)
        demo_draw_object(v, s->drops[i], DEMO_COL_WARNING);

    demo_hud(v, DEMO_COL_INFO, "Platform: %dx%d grid, %d springs, 4 pinned corners",
             GRID_SIZE, GRID_SIZE, s->n_springs);
    demo_hud(v, DEMO_COL_INFO, "Spring K: %.1f   damping: %.1f", SPRING_K, SPRING_DAMP);
    demo_hud(v, DEMO_COL_INFO, "Platform centre sag: %.2f m   balls on platform: %d/%d",
             (double)(platform_center_y(s) + 2.0f), drops_on_platform(s), NUM_DROPS);
}

static void snowflake_stats(demo_app_t *app, char *buf, size_t size)
{
    snowflake_t *s = (snowflake_t *)app->user;
    float max_speed = 0.0f;
    for (int i = 0; i < NUM_DROPS; i++) {
        float sp = demo_object_speed(s->drops[i]);
        if (sp > max_speed) max_speed = sp;
    }
    snprintf(buf, size, "center_y=%.3f max_center_y=%.3f corner_drift=%.4f balls_on_platform=%d/%d balls_through=%d ball_max_speed=%.3f",
             (double)platform_center_y(s), (double)s->max_center_y, (double)corner_drift(s),
             drops_on_platform(s), NUM_DROPS, s->n_passed, (double)max_speed);
}

static bool snowflake_check(demo_app_t *app)
{
    snowflake_t *s = (snowflake_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    float cy = platform_center_y(s);
    ok &= demo_check(tag, corner_drift(s) < 0.05f,
                     "pinned corners keep their place (drift %.4f m)", (double)corner_drift(s));
    ok &= demo_check(tag, cy > -1.99f && cy < 4.0f,
                     "platform centre sags below its start height (y = %.3f, start -2)", (double)cy);
    ok &= demo_check(tag, s->max_center_y > -1.0f && s->max_center_y < 5.0f,
                     "platform swings down like a trampoline (lowest centre y = %.3f)",
                     (double)s->max_center_y);
    ok &= demo_check(tag, s->n_passed == NUM_DROPS,
                     "the 0.5 m balls passed the 0.6 m gaps, as in the reference (%d/%d)",
                     s->n_passed, NUM_DROPS);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Example: Snowflake";
    app.cfg.tag         = "snowflake";
    app.cfg.orbit_dist  = 12.0f;
    app.cfg.orbit_pitch = -25.0f;
    app.cfg.target[1]   = -2.0f;
    app.cfg.friction    = 0.5;
    app.cfg.elasticity  = 0.4;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    snowflake_t *s = (snowflake_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* Grid of platform boxes */
    for (int gz = 0; gz < GRID_SIZE; gz++) {
        for (int gx = 0; gx < GRID_SIZE; gx++) {
            double x = (gx - GRID_SIZE / 2.0 + 0.5) * SPACING;
            double z = (gz - GRID_SIZE / 2.0 + 0.5) * SPACING;
            s->platform[gz * GRID_SIZE + gx] =
                demo_create_box(w, NULL, BOX_HS, BOX_HS * 0.3, BOX_HS, 0.5, NULL, x, -2.0, z);
        }
    }

    /* Springs to the right and front neighbours */
    for (int gz = 0; gz < GRID_SIZE; gz++) {
        for (int gx = 0; gx < GRID_SIZE; gx++) {
            int idx = gz * GRID_SIZE + gx;
            if (gx + 1 < GRID_SIZE) add_spring(s, w->env, idx, gz * GRID_SIZE + gx + 1);
            if (gz + 1 < GRID_SIZE) add_spring(s, w->env, idx, (gz + 1) * GRID_SIZE + gx);
        }
    }

    /* Pin the corner boxes */
    for (int i = 0; i < 4; i++) {
        ivp_object_set_pinned(s->platform[CORNERS[i]], true);
        s->corner_start[i] = s->platform[CORNERS[i]]->physical_core->pos_world_f_core_last_psi;
    }

    /* Falling balls */
    for (int i = 0; i < NUM_DROPS; i++) {
        double x = (i - NUM_DROPS / 2.0) * 0.6;
        s->drops[i] = demo_create_ball(w, NULL, DROP_R, 1.0, NULL, x, -6.0 - i * 1.5, 0.0);
    }

    s->max_center_y = -2.0f;
    app.update = snowflake_update;
    app.on_reset = snowflake_reset;
    app.draw   = snowflake_draw;
    app.stats  = snowflake_stats;
    app.check  = snowflake_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
