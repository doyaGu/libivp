/* example_forcefield.c -- Force field with mode switching
 *
 * Multiple objects of varying masses under a force field.
 * Press 1/2/3 to switch between CONSTANT, FORCE, and DRAG modes.
 *   CONSTANT: uniform acceleration — all objects move equally.
 *   FORCE:    constant force — lighter objects accelerate faster.
 *   DRAG:     velocity-proportional resistance slows moving objects.
 *
 * Demonstrates: ivp_forcefield, three force field modes, mass-dependent
 *               behavior, keyboard input, HUD.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_forcefield.h>

#define N_OBJECTS 24
#define START_Y   (-3.0f)

typedef struct {
    ivp_object_t *obj;
    float col[3];
    float mass;
} ff_obj_t;

typedef struct forcefield_demo {
    ivp_object_t          *ground;
    ff_obj_t               objs[N_OBJECTS];
    int                    n_obj;
    ivp_forcefield_state_t ff;
    int                    current_mode;
    /* headless measurements */
    float                  fall_vy[N_OBJECTS];      /* vertical speed 0.5 s after the start */
    float                  wind_x0[N_OBJECTS];      /* x when the wind started */
    float                  wind_dx[N_OBJECTS];      /* x travel during 0.5 s of wind */
    float                  drag_v0, drag_v1;        /* mean speed at the drag switch / 0.5 s later */
} forcefield_demo_t;

static const char *mode_names[] = {"REDUCED GRAVITY", "HORIZONTAL WIND", "VISCOUS DRAG"};

/* Switch the field law.  Resting objects may be frozen (not simulated, so
 * no controller reaches them): wake every object so that the new field
 * acts at once. */
static void set_mode(demo_app_t *app, int mode)
{
    forcefield_demo_t *s = (forcefield_demo_t *)app->user;
    s->current_mode = mode;
    switch (mode) {
    case 0:
        /* REDUCED GRAVITY: upward acceleration partially cancels gravity.
         * Net effect: ~3.8 m/s^2 down (all objects equally). */
        s->ff.mode = IVP_FORCEFIELD_CONSTANT;
        s->ff.value = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-6.0), IVP_REAL(0.0));
        break;
    case 1:
        /* HORIZONTAL WIND: constant force in +X direction.  Light objects
         * (0.5kg) accelerate at 30 m/s^2, heavy (10kg) at 1.5 m/s^2. */
        s->ff.mode = IVP_FORCEFIELD_FORCE;
        s->ff.value = ivp_vec3r(IVP_REAL(15.0), IVP_REAL(0.0), IVP_REAL(0.0));
        break;
    default:
        /* VISCOUS DRAG: velocity-proportional resistance, coefficient 5. */
        s->ff.mode = IVP_FORCEFIELD_DRAG;
        s->ff.value = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(5.0), IVP_REAL(5.0));
        break;
    }
    for (int i = 0; i < s->n_obj; i++)
        ivp_environment_ensure_core_in_simulation(app->world.env, s->objs[i].obj);
    printf("[forcefield] t=%.2f mode %s\n", ivp_environment_get_time(app->world.env), mode_names[mode]);
}

static float mean_speed(const forcefield_demo_t *s)
{
    float sum = 0.0f;
    for (int i = 0; i < s->n_obj; i++) sum += demo_object_speed(s->objs[i].obj);
    return sum / (float)s->n_obj;
}

static void forcefield_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    forcefield_demo_t *s = (forcefield_demo_t *)app->user;
    if (v) {
        if (demo_key_just_pressed(v, SDL_SCANCODE_1)) set_mode(app, 0);
        if (demo_key_just_pressed(v, SDL_SCANCODE_2)) set_mode(app, 1);
        if (demo_key_just_pressed(v, SDL_SCANCODE_3)) set_mode(app, 2);
        return;
    }
    /* scripted: reduced gravity, then 0.5 s of wind, then drag */
    if (frame == 30) {
        for (int i = 0; i < s->n_obj; i++)
            s->fall_vy[i] = (float)s->objs[i].obj->physical_core->speed.k[1];
    }
    if (frame == 180) {
        set_mode(app, 1);
        for (int i = 0; i < s->n_obj; i++)
            s->wind_x0[i] = (float)s->objs[i].obj->physical_core->pos_world_f_core_last_psi.k[0];
    }
    if (frame == 210) {
        for (int i = 0; i < s->n_obj; i++)
            s->wind_dx[i] = (float)s->objs[i].obj->physical_core->pos_world_f_core_last_psi.k[0] - s->wind_x0[i];
        set_mode(app, 2);
        s->drag_v0 = mean_speed(s);
    }
    if (frame == 240) s->drag_v1 = mean_speed(s);
}

static void forcefield_draw(demo_app_t *app, demo_viewer_t *v)
{
    forcefield_demo_t *s = (forcefield_demo_t *)app->user;
    const float c_ground[] = {0.22f, 0.24f, 0.30f};
    const float c_vel[]    = {0.2f, 1.0f, 0.4f};
    const float c_mode[]   = {1.0f, 1.0f, 0.2f};
    demo_draw_object(v, s->ground, c_ground);
    int airborne = 0;
    for (int i = 0; i < s->n_obj; i++) {
        demo_draw_object(v, s->objs[i].obj, s->objs[i].col);
        if (demo_object_speed(s->objs[i].obj) > 0.5f) demo_draw_velocity(v, s->objs[i].obj, c_vel);
        if ((float)s->objs[i].obj->physical_core->pos_world_f_core_last_psi.k[1] < -1.0f) airborne++;
    }
    {
        float org[3] = {0, 0, 0};
        demo_axes(v, org, NULL, 1.5f);
    }
    static const char *desc[] = {
        "Upward accel -6 m/s^2 (net ~3.8 down, all equal)",
        "Horizontal force +X 15N (light blown, heavy resist)",
        "Drag coeff 5 (moving objects slow down)"
    };
    demo_hud(v, c_mode, "Mode: %s  [1]Gravity [2]Wind [3]Drag", mode_names[s->current_mode]);
    demo_hud(v, DEMO_COL_INFO, "%s", desc[s->current_mode]);
    demo_hud(v, DEMO_COL_INFO, "Red=0.5kg  Green=3kg  Blue=10kg  Airborne: %d", airborne);
}

static void forcefield_stats(demo_app_t *app, char *buf, size_t size)
{
    forcefield_demo_t *s = (forcefield_demo_t *)app->user;
    float max_speed = 0.0f, avg_h = 0.0f, max_x = -1e9f;
    int frozen = 0;
    for (int i = 0; i < s->n_obj; i++) {
        const ivp_object_t *o = s->objs[i].obj;
        float sp = demo_object_speed(o);
        if (sp > max_speed) max_speed = sp;
        avg_h += -(float)o->physical_core->pos_world_f_core_last_psi.k[1];
        float x = (float)o->physical_core->pos_world_f_core_last_psi.k[0];
        if (x > max_x) max_x = x;
        if (ivp_object_is_frozen(o)) frozen++;
    }
    snprintf(buf, size, "mode=%s max_speed=%.3f mean_speed=%.3f avg_height=%.3f max_x=%.2f frozen=%d/%d",
             mode_names[s->current_mode], (double)max_speed, (double)mean_speed(s),
             (double)(avg_h / (float)s->n_obj), (double)max_x, frozen, s->n_obj);
}

static bool forcefield_check(demo_app_t *app)
{
    forcefield_demo_t *s = (forcefield_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    /* reduced gravity: every object falls at g - 6 regardless of its mass */
    float vmin = 1e9f, vmax = -1e9f, vsum = 0.0f;
    for (int i = 0; i < s->n_obj; i++) {
        vsum += s->fall_vy[i];
        if (s->fall_vy[i] < vmin) vmin = s->fall_vy[i];
        if (s->fall_vy[i] > vmax) vmax = s->fall_vy[i];
    }
    float vmean = vsum / (float)s->n_obj;
    float g = (float)app->world.env->gravity.k[1];
    float expect = (g - 6.0f) * 0.5f;
    /* (a few percent of spread are expected: the reference movement check,
     * which visits one simulation unit every ~15 units, can see a core that
     * has not moved 1 cm yet and mark it IVP_MT_SLOW, i.e. extra damping
     * until its next check) */
    ok &= demo_check(tag, fabsf(vmean - expect) < 0.06f * expect && vmax - vmin < 0.05f * vmean,
                     "reduced gravity: all masses fall alike, %.3f m/s after 0.5 s (expected %.3f, spread %.4f)",
                     (double)vmean, (double)expect, (double)(vmax - vmin));
    /* wind: a constant force accelerates the light objects much more */
    float dx[3] = {0, 0, 0};
    int nx[3] = {0, 0, 0};
    bool all_pos = true;
    for (int i = 0; i < s->n_obj; i++) {
        int k = s->objs[i].mass < 1.0f ? 0 : (s->objs[i].mass < 5.0f ? 1 : 2);
        dx[k] += s->wind_dx[i];
        nx[k]++;
        if (!(s->wind_dx[i] > 0.0f)) all_pos = false;
    }
    for (int k = 0; k < 3; k++) dx[k] /= (float)(nx[k] > 0 ? nx[k] : 1);
    ok &= demo_check(tag, all_pos && dx[0] > dx[1] && dx[1] > dx[2] && dx[0] > 5.0f * dx[2],
                     "wind (+X 15 N): 0.5 s travel 0.5 kg %.3f m > 3 kg %.3f m > 10 kg %.3f m",
                     (double)dx[0], (double)dx[1], (double)dx[2]);
    ok &= demo_check(tag, s->drag_v1 < 0.3f * s->drag_v0,
                     "drag: mean speed %.3f -> %.3f m/s within 0.5 s", (double)s->drag_v0, (double)s->drag_v1);
    int on_ground = 0;
    for (int i = 0; i < s->n_obj; i++) {
        const ivp_vec3r_t *p = &s->objs[i].obj->physical_core->pos_world_f_core_last_psi;
        if (fabs((double)p->k[0]) < 18.0 && fabs((double)p->k[2]) < 18.0 && (double)p->k[1] < -0.5) on_ground++;
    }
    ok &= demo_check(tag, on_ground == s->n_obj, "%d/%d objects still on the platform", on_ground, s->n_obj);
    return ok;
}

static void add_obj(forcefield_demo_t *s, ivp_object_t *obj, float mass, float r, float g, float b)
{
    ff_obj_t *o = &s->objs[s->n_obj++];
    o->obj = obj;
    o->mass = mass;
    o->col[0] = r; o->col[1] = g; o->col[2] = b;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Force Field -- Mode Switching";
    app.cfg.tag          = "forcefield";
    app.cfg.orbit_dist   = 30.0f;
    app.cfg.orbit_yaw    = 25.0f;
    app.cfg.orbit_pitch  = -25.0f;
    app.cfg.target[1]    = -5.0f;
    app.cfg.grid_extent  = 18.0f;
    app.cfg.grid_spacing = 2.0f;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    forcefield_demo_t *s = (forcefield_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(w, &gt, 18.0, 0.5, 18.0);

    /* Light cubes (mass 0.5) — red */
    for (int i = 0; i < 8; i++) {
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.mass = IVP_REAL(0.5);
        set_box_inertia(&t, IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(0.5));
        t.material.friction   = IVP_REAL(0.5);
        t.material.elasticity = IVP_REAL(0.3);
        float angle = 6.2832f * (float)i / 8.0f;
        float r = 5.0f;
        t.position = ivp_vec3r((ivp_real_t)(r * cosf(angle)), (ivp_real_t)START_Y, (ivp_real_t)(r * sinf(angle)));
        add_obj(s, demo_create_box_templ(w, &t, 0.5, 0.5, 0.5), 0.5f, 1.0f, 0.3f, 0.2f);
    }

    /* Medium cubes (mass 3.0) — green */
    for (int i = 0; i < 8; i++) {
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.mass = IVP_REAL(3.0);
        set_box_inertia(&t, IVP_REAL(3.0), IVP_REAL(0.7), IVP_REAL(0.7), IVP_REAL(0.7));
        t.material.friction   = IVP_REAL(0.5);
        t.material.elasticity = IVP_REAL(0.3);
        float angle = 6.2832f * (float)i / 8.0f + 0.39f;
        float r = 8.0f;
        t.position = ivp_vec3r((ivp_real_t)(r * cosf(angle)), (ivp_real_t)START_Y, (ivp_real_t)(r * sinf(angle)));
        add_obj(s, demo_create_box_templ(w, &t, 0.7, 0.7, 0.7), 3.0f, 0.3f, 1.0f, 0.3f);
    }

    /* Heavy spheres (mass 10.0) — blue */
    for (int i = 0; i < 8; i++) {
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.mass = IVP_REAL(10.0);
        set_sphere_inertia(&t, IVP_REAL(10.0), IVP_REAL(0.8));
        t.material.friction   = IVP_REAL(0.5);
        t.material.elasticity = IVP_REAL(0.3);
        float angle = 6.2832f * (float)i / 8.0f + 0.78f;
        float r = 11.0f;
        t.position = ivp_vec3r((ivp_real_t)(r * cosf(angle)), (ivp_real_t)START_Y, (ivp_real_t)(r * sinf(angle)));
        add_obj(s, demo_create_ball_templ(w, &t, 0.8), 10.0f, 0.3f, 0.5f, 1.0f);
    }

    /* ── Force field: start in CONSTANT mode, upward (anti-gravity) ─────── */
    ivp_vec3r_t ff_val = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    ivp_forcefield_state_init(&s->ff, IVP_FORCEFIELD_CONSTANT, &ff_val);
    ivp_environment_add_controller(w->env, 100, ivp_controller_forcefield, &s->ff);

    app.controls = "1: reduced gravity   2: horizontal wind   3: viscous drag";
    app.update   = forcefield_update;
    app.draw     = forcefield_draw;
    app.stats    = forcefield_stats;
    app.check    = forcefield_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
