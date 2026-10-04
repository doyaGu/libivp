/* example_collision_filter.c -- Collision filter demonstrations
 *
 * Three zones side by side demonstrate different collision filters:
 *   Zone 1 (left):  Exclusive pair filter — the collision between two cubes
 *                   toggles every 5 seconds (pass through <-> collide); at
 *                   every toggle the upper cube is dropped again onto the
 *                   lower one.
 *   Zone 2 (center): Group filter — same-group objects pass through
 *                   each other, different groups collide normally: of a
 *                   column red, red, blue, blue the second red cube falls
 *                   through the first one, the blue cubes land on the reds
 *                   and fall through each other.
 *   Zone 3 (right): Layer mask filter — objects on disjoint layers
 *                   ignore each other: of a column green, yellow, yellow,
 *                   green each cube falls through the other colour and
 *                   lands on the cube of its own layer.
 *
 * A meta filter combines group + layer mask + the exclusive pair filter as
 * an AND composite.  The filters are consulted when the collision detection
 * creates a new object pair (when two objects come near each other), so:
 *   - a pair filter toggle applies to the next approach of the two cubes
 *     (hence the re-drop);
 *   - the collision_group / layer_mask setters immediately recheck existing
 *     pairs.  The cubes start 4 m apart so the pass-through behavior remains
 *     visually clear.
 * (The pair filter used to be set up but never added to the environment,
 *  and the zone 2 and 3 cubes started 1 m apart, red and blue cubes even
 *  interpenetrating.)
 *
 * Demonstrates: ivp_collision_filter (group, layer_mask, exclusive_pair,
 *               meta), collision_group, layer_mask on objects.
 */
#include "demo_app.h"

#include <ivp/collision/ivp_collision_filter.h>

#define PAIR_TOGGLE_PERIOD 5.0   /* seconds between collision on/off toggle */
#define CUBE_HS 0.8
#define N_OBJ 10

typedef struct filter_demo {
    ivp_object_t               *ground;
    ivp_object_t               *objs[N_OBJ];
    int                         n_obj;
    int                         z1_lower, z1_upper, z2_start, z3_start;
    ivp_vec3r_t                 z1_upper_start;
    ivp_collision_filter_t      sub_filters[3];
    ivp_meta_collision_filter_t meta;
    ivp_exclusive_pair_filter_t pair_filter;
    bool                        pair_collision_off;
    int                         pair_toggle_count;
    double                      next_toggle;
    /* tracking */
    float                       z1_gap_off;    /* upper - lower centre height, end of the first (off) phase */
} filter_demo_t;

static void drop_upper(demo_app_t *app)
{
    filter_demo_t *s = (filter_demo_t *)app->user;
    ivp_object_t *o = s->objs[s->z1_upper];
    ivp_quatr_t q = ivp_quatr_identity();
    ivp_environment_ensure_core_in_simulation(app->world.env, o);
    ivp_core_beam_to(o->physical_core, &s->z1_upper_start, &q);
}

static void set_pair(filter_demo_t *s, bool off)
{
    s->pair_collision_off = off;
    if (off)
        ivp_exclusive_pair_filter_disable(&s->pair_filter, s->objs[s->z1_lower], s->objs[s->z1_upper]);
    else
        ivp_exclusive_pair_filter_enable(&s->pair_filter, s->objs[s->z1_lower], s->objs[s->z1_upper]);
}

static float zone1_gap(const filter_demo_t *s)
{
    return (float)(s->objs[s->z1_lower]->physical_core->pos_world_f_core_last_psi.k[1] -
                   s->objs[s->z1_upper]->physical_core->pos_world_f_core_last_psi.k[1]);
}

/* class of object i: 0 zone 1, 1 zone 2 red, 2 zone 2 blue, 3 zone 3 green,
 * 4 zone 3 yellow */
static int obj_class(const filter_demo_t *s, int i)
{
    if (i < s->z2_start) return 0;
    if (i < s->z3_start) return i < s->z2_start + 2 ? 1 : 2;
    return (i == s->z3_start || i == s->z3_start + 3) ? 3 : 4;
}

static float obj_y(const filter_demo_t *s, int i)
{
    return (float)s->objs[i]->physical_core->pos_world_f_core_last_psi.k[1];
}

static void filter_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    filter_demo_t *s = (filter_demo_t *)app->user;
    (void)v;
    (void)frame;
    /* Toggle the zone 1 pair filter periodically (simulated time) */
    double t = ivp_environment_get_time(app->world.env);
    if (t >= s->next_toggle) {
        if (s->pair_toggle_count == 0) s->z1_gap_off = zone1_gap(s);
        s->next_toggle += PAIR_TOGGLE_PERIOD;
        set_pair(s, !s->pair_collision_off);
        s->pair_toggle_count++;
        drop_upper(app);
        printf("[collision_filter] t=%.2f pair collision %s\n", t, s->pair_collision_off ? "OFF" : "ON");
    }
}

static void filter_reset(demo_app_t *app)
{
    filter_demo_t *s = (filter_demo_t *)app->user;
    set_pair(s, true);
    s->next_toggle = ivp_environment_get_time(app->world.env) + PAIR_TOGGLE_PERIOD;
}

static void filter_draw(demo_app_t *app, demo_viewer_t *v)
{
    filter_demo_t *s = (filter_demo_t *)app->user;
    const float c_ground[] = {0.22f, 0.24f, 0.30f};
    const float c_hud[]    = {0.95f, 0.97f, 1.0f};
    const float c_z1_on[]  = {0.9f, 0.5f, 0.2f};
    const float c_z1_off[] = {0.4f, 0.4f, 0.4f};
    const float c_red[]    = {1.0f, 0.3f, 0.2f};
    const float c_blue[]   = {0.3f, 0.5f, 1.0f};
    const float c_green[]  = {0.2f, 1.0f, 0.3f};
    const float c_yellow[] = {1.0f, 1.0f, 0.2f};
    const float *cols[5] = {NULL, c_red, c_blue, c_green, c_yellow};

    demo_draw_object(v, s->ground, c_ground);
    for (int i = 0; i < s->n_obj; i++) {
        int c = obj_class(s, i);
        const float *col = c == 0 ? (s->pair_collision_off ? c_z1_off : c_z1_on) : cols[c];
        demo_draw_object(v, s->objs[i], col);
    }
    /* Zone separators (vertical lines) */
    {
        float a[3] = {-6.0f, -12.0f, -5.0f};
        float b[3] = {-6.0f, 0.0f, -5.0f};
        demo_line(v, a, b, c_hud);
        a[0] = b[0] = 6.0f;
        demo_line(v, a, b, c_hud);
    }
    {
        float org[3] = {0, 0, 0};
        demo_axes(v, org, NULL, 1.5f);
    }
    demo_hud(v, s->pair_collision_off ? c_z1_off : c_z1_on,
             "Zone1: ExclusivePair %s (toggle %.0fs, %d toggles)",
             s->pair_collision_off ? "OFF (ghost)" : "ON (collide)", PAIR_TOGGLE_PERIOD, s->pair_toggle_count);
    demo_hud(v, c_red, "Zone2: GroupFilter (same group=ghost)");
    demo_hud(v, c_green, "Zone3: LayerMask (disjoint layers=ghost)");
}

static void filter_stats(demo_app_t *app, char *buf, size_t size)
{
    filter_demo_t *s = (filter_demo_t *)app->user;
    int n = snprintf(buf, size, "pair_off=%d toggles=%d zone1 gap=%.3f | y:", s->pair_collision_off ? 1 : 0,
                     s->pair_toggle_count, (double)zone1_gap(s));
    for (int i = s->z2_start; i < s->n_obj && n > 0 && (size_t)n < size; i++)
        n += snprintf(buf + n, size - (size_t)n, " %.2f", (double)obj_y(s, i));
}

static bool filter_check(demo_app_t *app)
{
    filter_demo_t *s = (filter_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    const float stack = 2.0f * (float)CUBE_HS;   /* one cube height */
    ok &= demo_check(tag, s->pair_toggle_count >= 1 && s->z1_gap_off < 0.3f,
                     "zone 1, pair collision off: the upper cube fell through the lower one (height gap %.3f m)",
                     (double)s->z1_gap_off);
    float gap = zone1_gap(s);
    ok &= demo_check(tag, !s->pair_collision_off && fabsf(gap - stack) < 0.1f,
                     "zone 1, pair collision on: the re-dropped cube rests on the lower one (gap %.3f m)", (double)gap);
    int r0 = s->z2_start, r1 = r0 + 1, b0 = r0 + 2, b1 = r0 + 3;
    ok &= demo_check(tag, fabsf(obj_y(s, r1) - obj_y(s, r0)) < 0.2f && fabsf(obj_y(s, b1) - obj_y(s, b0)) < 0.2f,
                     "zone 2, same group passes through: red at %.2f / %.2f, blue at %.2f / %.2f",
                     (double)obj_y(s, r0), (double)obj_y(s, r1), (double)obj_y(s, b0), (double)obj_y(s, b1));
    ok &= demo_check(tag, fabsf(obj_y(s, r0) - obj_y(s, b0) - stack) < 0.15f &&
                          fabsf(obj_y(s, r0) - obj_y(s, b1) - stack) < 0.15f,
                     "zone 2, different groups collide: the blue cubes rest on the red ones");
    int g0 = s->z3_start, y0 = g0 + 1, y1 = g0 + 2, g1 = g0 + 3;
    ok &= demo_check(tag, fabsf(obj_y(s, y0) - obj_y(s, g0)) < 0.2f,
                     "zone 3, disjoint layers: the yellow cube fell through the green one (%.2f / %.2f)",
                     (double)obj_y(s, g0), (double)obj_y(s, y0));
    ok &= demo_check(tag, fabsf(obj_y(s, g0) - obj_y(s, g1) - stack) < 0.15f &&
                          fabsf(obj_y(s, y0) - obj_y(s, y1) - stack) < 0.15f,
                     "zone 3, same layer collides: green on green (%.2f), yellow on yellow (%.2f)",
                     (double)obj_y(s, g1), (double)obj_y(s, y1));
    return ok;
}

static ivp_object_t *add_cube(demo_app_t *app, double friction, double x, double y)
{
    filter_demo_t *s = (filter_demo_t *)app->user;
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.mass = IVP_REAL(2.0);   /* 2kg cubes — moderate mass for visible interaction */
    set_box_inertia(&t, IVP_REAL(2.0), (ivp_real_t)CUBE_HS, (ivp_real_t)CUBE_HS, (ivp_real_t)CUBE_HS);
    t.material.friction   = (ivp_real_t)friction;
    t.material.elasticity = IVP_REAL(0.2);   /* low bounce: focus on filter behavior */
    t.position = ivp_vec3r((ivp_real_t)x, (ivp_real_t)y, IVP_REAL(0.0));
    ivp_object_t *o = demo_create_box_templ(&app->world, &t, CUBE_HS, CUBE_HS, CUBE_HS);
    s->objs[s->n_obj++] = o;
    return o;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Collision Filters -- Group, Layer, Pair";
    app.cfg.tag          = "collision_filter";
    app.cfg.orbit_dist   = 35.0f;
    app.cfg.orbit_yaw    = 15.0f;
    app.cfg.orbit_pitch  = -25.0f;
    app.cfg.target[1]    = -4.0f;
    app.cfg.grid_extent  = 25.0f;
    app.cfg.grid_spacing = 2.0f;

    filter_demo_t *s = (filter_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;

    /* ── Meta collision filter: group AND layer_mask AND exclusive pairs ── */
    if (!ivp_ok(ivp_exclusive_pair_filter_init(&s->pair_filter, NULL))) return 1;
    s->sub_filters[0] = ivp_collision_filter_group();
    s->sub_filters[1] = ivp_collision_filter_layer_mask();
    s->sub_filters[2] = s->pair_filter.base;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ecfg.collision_filter = ivp_collision_filter_meta(&s->meta, s->sub_filters, 3);
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(&app.world, &gt, 25.0, 0.5, 10.0);

    /* ═══ Zone 1 (LEFT, x=-12): exclusive pair filter ═══════════════════ *
     * high friction: the cubes stack reliably */
    s->z1_lower = s->n_obj;
    add_cube(&app, 0.7, -12.0, -1.3);
    s->z1_upper = s->n_obj;
    add_cube(&app, 0.7, -12.0, -6.0);    /* dropped from above */
    s->z1_upper_start = s->objs[s->z1_upper]->physical_core->pos_world_f_core_last_psi;
    set_pair(s, true);                   /* start with collision disabled */
    s->next_toggle = PAIR_TOGGLE_PERIOD;

    /* ═══ Zone 2 (CENTER, x=0): group filter ════════════════════════════ *
     * A column, 4 m apart: red, red (group 1), blue, blue (group 2). */
    s->z2_start = s->n_obj;
    for (int i = 0; i < 4; i++) {
        ivp_object_t *cube = add_cube(&app, 0.6, 0.0, -1.3 - 4.0 * i);
        ivp_object_set_collision_group(cube, i < 2 ? 1 : 2);
    }

    /* ═══ Zone 3 (RIGHT, x=12): layer mask filter ═══════════════════════ *
     * A column, 4 m apart: green, yellow, yellow, green.  Green cubes on
     * layer 0x01, yellow cubes on layer 0x02; both collide with the ground
     * (layer 0xFFFFFFFF).  (In the order green, yellow, green, yellow the
     * last cube bounces on its partner, which shares its place with a
     * cube it ignores, and then sinks through it -- the legacy engine
     * does exactly the same with an IVP_Collision_Filter_Exclusive_Pair.) */
    s->z3_start = s->n_obj;
    for (int i = 0; i < 4; i++) {
        ivp_object_t *cube = add_cube(&app, 0.6, 12.0, -1.3 - 4.0 * i);
        ivp_object_set_layer_mask(cube, (i == 0 || i == 3) ? 0x01 : 0x02);
    }

    app.update   = filter_update;
    app.on_reset = filter_reset;
    app.draw     = filter_draw;
    app.stats    = filter_stats;
    app.check    = filter_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    ivp_exclusive_pair_filter_destroy(&s->pair_filter);
    free(s);
    return rc;
}
