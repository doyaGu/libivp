/* example_domino.c -- Chain of falling domino tiles
 *
 * Port of ivp_examples/examples/ive_example_domino.cxx.
 *
 * Twenty-five thin, tall boxes stand in a row on a static ground.  The
 * first tile gets a small spin (async_add_rot_speed_object_cs) so it tips
 * over into the second one and the cascade runs down the whole row.
 *
 * Demonstrates: many resting contacts, stacking / toppling, an
 *               asynchronous rotation impulse (reference
 *               IVP_Real_Object::async_add_rot_speed_object_cs),
 *               sleeping objects woken up by collisions.
 */
#include "demo_app.h"

#define NUM_TILES 25

/* tile half extents: thin (0.1) x tall (0.6) x wide (0.3) */
static const double TILE_HX = 0.1, TILE_HY = 0.6, TILE_HZ = 0.3;
static const double TILE_SPACING = 0.55;

typedef struct domino {
    ivp_object_t *ground;
    ivp_object_t *tiles[NUM_TILES];
    bool          tipped;
} domino_t;

static int count_toppled(const domino_t *s)
{
    int n = 0;
    for (int i = 0; i < NUM_TILES; i++)
        if (demo_object_tilt_deg(s->tiles[i]) > 45.0f) n++;
    return n;
}

/* Index of the last tile that is no longer upright (the cascade front). */
static int cascade_front(const domino_t *s)
{
    int front = -1;
    for (int i = 0; i < NUM_TILES; i++)
        if (demo_object_tilt_deg(s->tiles[i]) > 10.0f) front = i;
    return front;
}

static void tip_first_tile(demo_app_t *app)
{
    domino_t *s = (domino_t *)app->user;
    /* IVP_U_Float_Point angular_vel(0, 0, -2): async_add_rot_speed_object_cs */
    demo_async_add_rot_speed_cs(app->world.env, s->tiles[0], 0.0, 0.0, -2.0);
    s->tipped = true;
}

static void domino_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    domino_t *s = (domino_t *)app->user;
    (void)v;
    (void)frame;
    /* the reference tips the tile once at start-up; tip it again after a
     * scene reset */
    if (!s->tipped) tip_first_tile(app);
}

static void domino_reset(demo_app_t *app)
{
    domino_t *s = (domino_t *)app->user;
    s->tipped = false;   /* tip the first tile again on the next frame */
}

static void domino_draw(demo_app_t *app, demo_viewer_t *v)
{
    domino_t *s = (domino_t *)app->user;
    demo_draw_object(v, s->ground, DEMO_COL_STATIC);
    for (int i = 0; i < NUM_TILES; i++) {
        bool down = demo_object_tilt_deg(s->tiles[i]) > 45.0f;
        demo_draw_object(v, s->tiles[i], down ? DEMO_COL_OBJECT_B : DEMO_COL_HIGHLIGHT);
    }
    demo_hud(v, DEMO_COL_INFO, "Tiles: %d   toppled: %d", NUM_TILES, count_toppled(s));
}

static void domino_stats(demo_app_t *app, char *buf, size_t size)
{
    domino_t *s = (domino_t *)app->user;
    int frozen = 0;
    for (int i = 0; i < NUM_TILES; i++)
        if (ivp_object_is_frozen(s->tiles[i])) frozen++;
    snprintf(buf, size, "toppled=%d/%d front=%d frozen=%d first_tilt=%.1f last_tilt=%.1f",
             count_toppled(s), NUM_TILES, cascade_front(s), frozen,
             (double)demo_object_tilt_deg(s->tiles[0]),
             (double)demo_object_tilt_deg(s->tiles[NUM_TILES - 1]));
}

static bool domino_check(demo_app_t *app)
{
    domino_t *s = (domino_t *)app->user;
    const char *tag = app->cfg.tag;
    int toppled = count_toppled(s);
    bool ok = true;
    ok &= demo_check(tag, toppled >= NUM_TILES - 1,
                     "cascade ran through the row (%d/%d tiles toppled)", toppled, NUM_TILES);
    /* every tile stays near the ground line z = 0, x within the row */
    bool in_row = true;
    for (int i = 0; i < NUM_TILES; i++) {
        const ivp_vec3r_t *p = &s->tiles[i]->physical_core->pos_world_f_core_last_psi;
        if (fabs((double)p->k[2]) > 1.0 || p->k[1] > 0.0 || p->k[1] < -1.0) in_row = false;
    }
    ok &= demo_check(tag, in_row, "tiles lie on the ground along the row");
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Example: Domino";
    app.cfg.tag         = "domino";
    app.cfg.orbit_dist  = 25.0f;
    app.cfg.orbit_pitch = -20.0f;
    app.cfg.target[0]   = 5.0f;
    app.cfg.target[1]   = -1.0f;
    app.cfg.friction    = 0.6;
    app.cfg.elasticity  = 0.5;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    domino_t *s = (domino_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;

    s->ground = demo_create_box(&app.world, NULL, 20.0, 0.5, 5.0, 0.0, NULL, 0.0, 0.5, 0.0);
    for (int i = 0; i < NUM_TILES; i++) {
        s->tiles[i] = demo_create_box(&app.world, NULL, TILE_HX, TILE_HY, TILE_HZ, 0.3, NULL,
                                      -5.0 + i * TILE_SPACING, -TILE_HY, 0.0);
    }

    app.update   = domino_update;
    app.on_reset = domino_reset;
    app.draw     = domino_draw;
    app.stats    = domino_stats;
    app.check    = domino_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
