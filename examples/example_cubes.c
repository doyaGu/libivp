/* example_cubes.c -- Cube stacking and collision demo
 *
 * A pyramid of cubes with varied sizes, masses, and materials stacked
 * on a ground platform.  Includes a tilted ramp for rolling, a sphere
 * for variety, and a HUD showing object count / FPS.
 *
 * Demonstrates: box surfaces, sphere surface, collisions, elasticity,
 *               friction, stacking, HUD text, coordinate axes.
 */
#include "demo_app.h"

#define MAX_DYN 24
#define GROUND_TOP (-0.5f)

typedef struct obj_vis {
    ivp_object_t *obj;
    float col[3];
    const char *group;   /* "base", "layer2", "brick", "ball", "drop" */
    float start[3];
} obj_vis_t;

typedef struct cubes_demo {
    ivp_object_t *ground, *ramp;
    obj_vis_t     dyn[MAX_DYN];
    int           n_dyn;
    float         max_speed_late;   /* after 6 s */
} cubes_demo_t;

static const float palette[][3] = {
    {1.0f, 0.4f, 0.2f}, {0.3f, 1.0f, 0.3f}, {0.3f, 0.5f, 1.0f},
    {1.0f, 1.0f, 0.2f}, {1.0f, 0.2f, 0.8f}, {0.2f, 1.0f, 0.9f},
    {0.9f, 0.6f, 0.1f}, {0.6f, 0.3f, 1.0f}
};

static obj_vis_t *add_dyn(cubes_demo_t *s, ivp_object_t *obj, const char *group, const float *col)
{
    obj_vis_t *d = &s->dyn[s->n_dyn];
    d->obj = obj;
    d->group = group;
    const float *c = col ? col : palette[s->n_dyn % 8];
    d->col[0] = c[0]; d->col[1] = c[1]; d->col[2] = c[2];
    demo_object_center(obj, d->start);
    s->n_dyn++;
    return d;
}

static void cubes_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    (void)app;
    v->dragger.settings.max_force = 65.0f;
    v->dragger.settings.ramp_rate = 4.5f;
    v->focus.follow_lerp = 8.0f;
}

static void cubes_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    cubes_demo_t *s = (cubes_demo_t *)app->user;
    (void)v;
    (void)frame;
    if (ivp_environment_get_time(app->world.env) < 6.0) return;
    for (int i = 0; i < s->n_dyn; i++) {
        float sp = demo_object_speed(s->dyn[i].obj);
        if (sp > s->max_speed_late) s->max_speed_late = sp;
    }
}

static void cubes_draw(demo_app_t *app, demo_viewer_t *v)
{
    cubes_demo_t *s = (cubes_demo_t *)app->user;
    const float c_ground[] = {0.22f, 0.24f, 0.30f};
    const float c_ramp[]   = {0.62f, 0.52f, 0.34f};
    const float c_vel[]    = {0.2f, 1.0f, 0.4f};
    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->ramp, c_ramp);
    {
        float origin[3] = {0, 0, 0};
        demo_axes(v, origin, NULL, 2.0f);
    }
    int toppled = 0;
    float max_speed = 0.0f;
    for (int i = 0; i < s->n_dyn; i++) {
        const obj_vis_t *d = &s->dyn[i];
        demo_draw_object(v, d->obj, d->col);
        float speed = demo_object_speed(d->obj);
        if (speed > max_speed) max_speed = speed;
        if (speed > 0.5f) demo_draw_velocity(v, d->obj, c_vel);
        if (demo_body_of(d->obj)->shape == DEMO_SHAPE_BOX && demo_object_tilt_deg(d->obj) > 44.0f) toppled++;
    }
    demo_hud(v, DEMO_COL_INFO, "Objects: %d  Time: %.1fs  max speed %.2f m/s  toppled %d",
             s->n_dyn, (double)ivp_environment_get_time(app->world.env), (double)max_speed, toppled);
}

static const obj_vis_t *find_group(const cubes_demo_t *s, const char *group)
{
    for (int i = 0; i < s->n_dyn; i++)
        if (strcmp(s->dyn[i].group, group) == 0) return &s->dyn[i];
    return NULL;
}

static void cubes_stats(demo_app_t *app, char *buf, size_t size)
{
    cubes_demo_t *s = (cubes_demo_t *)app->user;
    float max_speed = 0.0f, sum_y = 0.0f;
    int fast = 0;
    for (int i = 0; i < s->n_dyn; i++) {
        float sp = demo_object_speed(s->dyn[i].obj);
        if (sp > max_speed) max_speed = sp;
        if (sp > 2.0f) fast++;
        sum_y += (float)s->dyn[i].obj->physical_core->pos_world_f_core_last_psi.k[1];
    }
    float ball[3];
    demo_object_center(find_group(s, "ball")->obj, ball);
    snprintf(buf, size, "max_speed=%.3f fast=%d avg_y=%.3f ball=(%.2f,%.2f,%.2f)",
             (double)max_speed, fast, (double)(sum_y / (float)s->n_dyn),
             (double)ball[0], (double)ball[1], (double)ball[2]);
}

static bool cubes_check(demo_app_t *app)
{
    cubes_demo_t *s = (cubes_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    int base_ok = 0, n_base = 0, layer_ok = 0, n_layer = 0, drop_ok = 0, n_drop = 0;
    float base_shift = 0.0f, max_box_speed = 0.0f;
    for (int i = 0; i < s->n_dyn; i++) {
        const obj_vis_t *d = &s->dyn[i];
        float p[3];
        demo_object_center(d->obj, p);
        float shift = sqrtf((p[0] - d->start[0]) * (p[0] - d->start[0]) +
                            (p[2] - d->start[2]) * (p[2] - d->start[2]));
        float tilt = demo_object_tilt_deg(d->obj);
        if (strcmp(d->group, "ball") != 0) {
            float sp = demo_object_speed(d->obj);
            if (sp > max_box_speed) max_box_speed = sp;
        }
        if (strcmp(d->group, "base") == 0) {
            n_base++;
            if (shift > base_shift) base_shift = shift;
            /* still standing on the ground (top at -0.5, half size 0.8) */
            if (shift < 0.4f && fabsf(p[1] - (GROUND_TOP - 0.8f)) < 0.05f && tilt < 5.0f) base_ok++;
        } else if (strcmp(d->group, "layer2") == 0) {
            n_layer++;
            /* on top of the base layer: -0.5 - 1.6 - 0.5 */
            if (fabsf(p[1] - (GROUND_TOP - 2.1f)) < 0.1f && tilt < 5.0f) layer_ok++;
        } else if (strcmp(d->group, "brick") == 0) {
            ok &= demo_check(tag, p[1] < GROUND_TOP - 3.0f && fabsf(p[0]) < 1.0f && fabsf(p[2]) < 1.0f,
                             "top brick rests on the stack (centre (%.2f, %.2f, %.2f))",
                             (double)p[0], (double)p[1], (double)p[2]);
        } else if (strcmp(d->group, "drop") == 0) {
            n_drop++;
            if (fabsf(p[1] - (GROUND_TOP - 0.8f)) < 0.1f && tilt < 5.0f) drop_ok++;
        } else if (strcmp(d->group, "ball") == 0) {
            /* rolled down the ramp (x 4..12) towards the pyramid */
            ok &= demo_check(tag, p[0] < 4.5f && p[0] > 2.0f && fabsf(p[1] - (GROUND_TOP - 0.7f)) < 0.1f,
                             "ball rolled down the ramp to the pyramid: (%.2f, %.2f, %.2f), speed %.2f",
                             (double)p[0], (double)p[1], (double)p[2], (double)demo_object_speed(d->obj));
        }
    }
    ok &= demo_check(tag, base_ok == n_base, "%d/%d base cubes still standing in place (max shift %.2f m)",
                     base_ok, n_base, (double)base_shift);
    ok &= demo_check(tag, layer_ok == n_layer, "%d/%d second layer cubes rest on the base layer", layer_ok, n_layer);
    ok &= demo_check(tag, drop_ok == n_drop, "%d/%d dropped cubes landed flat on the ground", drop_ok, n_drop);
    ok &= demo_check(tag, max_box_speed < 0.05f && s->max_speed_late < 5.0f,
                     "boxes at rest (max speed %.3f m/s, max after 6 s %.2f m/s)",
                     (double)max_box_speed, (double)s->max_speed_late);
    return ok;
}

static ivp_object_t *make_cube(demo_world_t *w, double hx, double hy, double hz, double mass,
                               double friction, double elasticity, double x, double y, double z,
                               const ivp_quatr_t *q, double speed_damp)
{
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.mass = (ivp_real_t)mass;
    set_box_inertia(&t, t.mass, (ivp_real_t)hx, (ivp_real_t)hy, (ivp_real_t)hz);
    t.material.friction   = (ivp_real_t)friction;
    t.material.elasticity = (ivp_real_t)elasticity;
    if (speed_damp >= 0.0) t.speed_damp_factor = (ivp_real_t)speed_damp;
    t.position = ivp_vec3r((ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    if (q) t.orientation = *q;
    return demo_create_box_templ(w, &t, hx, hy, hz);
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Cubes -- Stacking Demo";
    app.cfg.tag          = "cubes";
    app.cfg.orbit_dist   = 25.0f;
    app.cfg.orbit_yaw    = 30.0f;
    app.cfg.orbit_pitch  = -25.0f;
    app.cfg.target[1]    = -3.0f;
    app.cfg.grid_extent  = 15.0f;
    app.cfg.grid_spacing = 2.0f;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    cubes_demo_t *s = (cubes_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* ── Ground platform ────────────────────────────────────────────────── */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(w, &gt, 15.0, 0.5, 15.0);

    /* ── Ramp (tilted static box, ~15 degrees around Z) ─────────────────── *
     * It slopes down towards the pyramid: the ball rolls into the stack
     * (tilted the other way it rolled off the platform into the void). */
    ivp_object_template_t rt;
    ivp_object_template_init(&rt);
    rt.mass = 0.0f;
    rt.physical_unmoveable = true;
    rt.position = ivp_vec3r(IVP_REAL(8.0), IVP_REAL(-1.5), IVP_REAL(0.0));
    rt.orientation = ivp_quatr_make(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(-0.13), IVP_REAL(0.99));
    ivp_quat_fast_normalize(&rt.orientation);
    s->ramp = demo_create_box_templ(w, &rt, 4.0, 0.2, 2.0);

    /* --- Pyramid base layer (3x3 grid) -------------------------------- */
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            ivp_object_t *o = make_cube(w, 0.8, 0.8, 0.8, 3.0, 0.9, 0.15,
                                        -2.0 + 2.0 * col, -1.3, -2.0 + 2.0 * row, NULL, -1.0);
            add_dyn(s, o, "base", NULL);
        }
    }

    /* --- Second layer (2x2, dropped from above) ----------------------- */
    for (int row = 0; row < 2; row++) {
        for (int col = 0; col < 2; col++) {
            /* Slight random-ish rotation */
            ivp_quatr_t q = ivp_quatr_make((ivp_real_t)(0.05 * (col + 1)), (ivp_real_t)(0.03 * (row + 1)),
                                           IVP_REAL(0.0), IVP_REAL(0.99));
            ivp_quat_fast_normalize(&q);
            ivp_object_t *o = make_cube(w, 0.5, 0.5, 0.5, 2.0, 0.8, 0.2,
                                        -1.0 + 2.0 * col, -4.0, -1.0 + 2.0 * row, &q, -1.0);
            add_dyn(s, o, "layer2", NULL);
        }
    }

    /* --- Top brick (elongated) ---------------------------------------- */
    {
        ivp_quatr_t q = ivp_quatr_make(IVP_REAL(0.0), IVP_REAL(0.1), IVP_REAL(0.0), IVP_REAL(0.99));
        ivp_quat_fast_normalize(&q);
        ivp_object_t *o = make_cube(w, 1.2, 0.6, 0.6, 4.0, 0.7, 0.3, 0.0, -6.0, 0.0, &q, -1.0);
        const float c[3] = {0.9f, 0.9f, 0.2f};
        add_dyn(s, o, "brick", c);
    }

    /* --- Sphere on the ramp ------------------------------------------- */
    {
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.mass = IVP_REAL(2.0);
        ivp_real_t I = IVP_REAL(0.4) * IVP_REAL(2.0) * IVP_REAL(0.49); /* 2/5 * m * r^2 */
        ivp_vec3r_set(&t.rot_inertia, I, I, I);
        t.rot_inertia_is_factor = false; /* absolute inertia */
        t.material.friction   = IVP_REAL(0.4);
        t.material.elasticity = IVP_REAL(0.6);
        t.position = ivp_vec3r(IVP_REAL(9.0), IVP_REAL(-4.0), IVP_REAL(0.0));
        ivp_object_t *o = demo_create_ball_templ(w, &t, 0.7);
        const float c[3] = {0.2f, 0.9f, 1.0f};
        add_dyn(s, o, "ball", c);
    }

    /* --- Extra falling cubes (low elasticity to settle quickly) -------- */
    for (int i = 0; i < 3; i++) {
        ivp_quatr_t q = ivp_quatr_make((ivp_real_t)(0.1 * (i + 1)), IVP_REAL(0.05), IVP_REAL(0.0), IVP_REAL(0.99));
        ivp_quat_fast_normalize(&q);
        /* Cap elasticity at 0.15 for dropped cubes to avoid persistent bouncing */
        ivp_object_t *o = make_cube(w, 0.8, 0.8, 0.8, 1.0 + 2.0 * i, 0.6, 0.15,
                                    -4.0 + 4.0 * i, -8.0 - 3.0 * i, 5.0, &q, 0.03);
        add_dyn(s, o, "drop", NULL);
    }

    app.viewer_init = cubes_viewer_init;
    app.update      = cubes_update;
    app.draw        = cubes_draw;
    app.stats       = cubes_stats;
    app.check       = cubes_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
