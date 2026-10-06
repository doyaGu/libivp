/* example_buoyancy.c -- Mixed objects in water with flow current
 *
 * Boxes and spheres of varying densities are dropped into a "lake".
 * Light objects float, heavy ones sink.  A flow current pushes objects
 * sideways.  HUD shows depth and buoyancy status for each object.
 *
 * Demonstrates: ivp_controller_buoyancy, fluid density, flow_velocity,
 *               sphere vs box shapes, HUD text, velocity arrows.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_controller_buoyancy.h>

/* Water surface in Y-down coords -- negative Y = up */
#define WATER_SURFACE_Y  (-7.0f)
/* Medium density in the mass scale of the objects below: their masses are
 * 1/1000 of real objects of that size and material (the 1 m^3 "steel" cube
 * weighs 8 kg, i.e. density 8 = 8000/1000), so "water" is 1000/1000 = 1.
 * Buoyancy and drag accelerations only depend on the density ratio and
 * the medium density relative to the object masses, so the scene behaves
 * like real objects in real water.  (With medium_density 1000 every object
 * was 40 to 20000 times lighter than the water it displaces: buoyancy
 * accelerations of 40 g to 20000 g shot the objects out of the water at
 * the 2000 m/s velocity limit, and the explicit quadratic pressure drag
 * (linear_drag 3, 30 times the reference default) overshot and diverged
 * within a few PSIs -- speed changes above 1e5 m/s, which the reference
 * debug build rejects (IVP_Core::core_plausible_check).) */
#define WATER_DENSITY    1.0f
#define FLOW_SPEED_X     1.2f
#define SEA_FLOOR_Y      4.5f     /* top of the ground box */

/* ── Object descriptors ────────────────────────────────────────────────── */
#define N_OBJS 7

typedef struct {
    const char *label;
    float mass;
    float hs[3];        /* half-extents (box) or {radius,0,0} (sphere) */
    float start_pos[3];
    float col[3];
    bool  is_sphere;
} obj_desc_t;

static const obj_desc_t descs[N_OBJS] = {
    /* label          mass   half-sizes                start-pos              color                sphere */
    {"Cork  0.3kg",   0.3f, {0.5f,0.5f,0.5f},  {-6.0f,-6.0f, 0.0f}, {0.95f,0.85f,0.35f}, false},
    {"Wood  1.0kg",   1.0f, {0.6f,0.4f,0.6f},  {-3.0f,-5.0f, 0.0f}, {0.75f,0.55f,0.25f}, false},
    {"Ball  1.5kg",   1.5f, {0.55f,0,0},        { 0.0f,-7.0f, 0.0f}, {0.3f, 0.8f, 1.0f},  true },
    {"Steel 8.0kg",   8.0f, {0.5f,0.5f,0.5f},  { 3.0f,-4.0f, 0.0f}, {0.65f,0.65f,0.70f}, false},
    {"Brick 3.0kg",   3.0f, {0.7f,0.35f,0.5f}, { 6.0f,-6.0f, 0.0f}, {0.85f,0.40f,0.25f}, false},
    {"Foam  0.2kg",   0.2f, {0.8f,0.8f,0.8f},  {-4.5f,-9.0f, 3.0f}, {1.0f, 1.0f, 0.8f},  false},
    {"Lead  12kg",   12.0f, {0.4f,0.4f,0.4f},  { 4.5f,-8.0f,-3.0f}, {0.3f, 0.3f, 0.4f},  false},
};

static float desc_volume(const obj_desc_t *d)
{
    if (d->is_sphere) return (4.0f / 3.0f) * 3.14159265f * d->hs[0] * d->hs[0] * d->hs[0];
    return 8.0f * d->hs[0] * d->hs[1] * d->hs[2];
}

static float desc_density(const obj_desc_t *d)
{
    float vol = desc_volume(d);
    return vol > 1e-6f ? d->mass / vol : 0.0f;
}

/* vertical half extent of the (unrotated) object */
static float desc_half_height(const obj_desc_t *d)
{
    return d->is_sphere ? d->hs[0] : d->hs[1];
}

typedef struct buoyancy_demo {
    ivp_object_t        *ground, *wall_l, *wall_r;
    ivp_object_t        *objs[N_OBJS];
    ivp_buoyancy_state_t buoy;
    float                max_speed;           /* over the whole run */
} buoyancy_demo_t;

static void give_initial_speed(buoyancy_demo_t *s)
{
    /* Objects above water get a small initial downward velocity for natural drop */
    for (int i = 0; i < N_OBJS; i++) {
        if (descs[i].start_pos[1] < WATER_SURFACE_Y)
            s->objs[i]->physical_core->speed = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.5), IVP_REAL(0.0));
    }
}

/* fraction of the object's height below the water surface (0..1) */
static float submerged_fraction(const obj_desc_t *d, float y)
{
    float h = desc_half_height(d);
    float under = (y + h) - WATER_SURFACE_Y;     /* depth of the lowest point */
    float f = under / (2.0f * h);
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    return f;
}

static void buoyancy_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    buoyancy_demo_t *s = (buoyancy_demo_t *)app->user;
    (void)v;
    (void)frame;
    for (int i = 0; i < N_OBJS; i++) {
        float speed = demo_object_speed(s->objs[i]);
        if (speed > s->max_speed) s->max_speed = speed;
    }
}

static void buoyancy_reset(demo_app_t *app)
{
    buoyancy_demo_t *s = (buoyancy_demo_t *)app->user;
    give_initial_speed(s);
}

static void buoyancy_draw(demo_app_t *app, demo_viewer_t *v)
{
    buoyancy_demo_t *s = (buoyancy_demo_t *)app->user;
    const float c_water[]  = {0.10f, 0.30f, 0.65f};
    const float c_ground[] = {0.30f, 0.28f, 0.25f};
    const float c_wall[]   = {0.20f, 0.20f, 0.25f};
    const float c_vel[]    = {0.2f,  1.0f,  0.4f };
    const float c_buoy[]   = {0.3f,  0.6f,  1.0f };
    const float c_flow[]   = {0.3f,  0.7f,  1.0f };

    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->wall_l, c_wall);
    demo_draw_object(v, s->wall_r, c_wall);

    /* Water surface grid (denser, with thick border) */
    {
        float ext = 12.0f;
        float wy = WATER_SURFACE_Y;
        for (float x = -ext; x <= ext; x += 1.5f) {
            float a[3] = {x, wy, -ext}, b[3] = {x, wy, ext};
            demo_line(v, a, b, c_water);
        }
        for (float z = -ext; z <= ext; z += 1.5f) {
            float a[3] = {-ext, wy, z}, b[3] = {ext, wy, z};
            demo_line(v, a, b, c_water);
        }
        float a[3], b[3];
        a[1] = b[1] = wy;
        a[2] = -ext; b[2] = ext; a[0] = b[0] = -ext;
        demo_thick_line(v, a, b, c_water, 3);
        a[0] = b[0] = ext;
        demo_thick_line(v, a, b, c_water, 3);
        a[0] = -ext; b[0] = ext; a[2] = b[2] = -ext;
        demo_thick_line(v, a, b, c_water, 3);
        a[2] = b[2] = ext;
        demo_thick_line(v, a, b, c_water, 3);
    }

    /* The water body: a translucent slab down to the sea floor */
    if (v->solid) {
        const float ext = 12.0f;
        const float depth = SEA_FLOOR_Y - WATER_SURFACE_Y;
        const float centre[3] = { 0.0f, WATER_SURFACE_Y + depth * 0.5f, 0.0f };
        const float half[3] = { ext, depth * 0.5f, ext };
        demo_set_alpha(v, 0.28f);
        demo_solid_box(v, centre, half, NULL, c_water);
        demo_set_alpha(v, 1.0f);
    }

    /* Flow direction arrow in the water */
    {
        float fa[3] = {-8.0f, WATER_SURFACE_Y, 8.0f};
        float fb[3] = {-3.0f, WATER_SURFACE_Y, 8.0f};
        demo_arrow(v, fa, fb, c_flow, 0.5f);
    }

    for (int i = 0; i < N_OBJS; i++) {
        const obj_desc_t *d = &descs[i];
        demo_draw_object(v, s->objs[i], d->col);
        float p[3];
        demo_object_center(s->objs[i], p);
        float speed = demo_object_speed(s->objs[i]);
        if (speed > 0.4f) demo_draw_velocity(v, s->objs[i], c_vel);
        /* upward buoyancy indicator when (partly) submerged */
        if (submerged_fraction(d, p[1]) > 0.01f) {
            float bb[3] = {p[0], p[1] - 1.5f, p[2]};
            demo_arrow(v, p, bb, c_buoy, 0.2f);
        }
    }

    {
        float org[3] = {0, 0, 0};
        demo_axes(v, org, NULL, 1.5f);
    }

    demo_hud(v, DEMO_COL_INFO, "Objects: %d  Water Y=%.1f  (rel. density = object / water)",
             N_OBJS, (double)WATER_SURFACE_Y);
    for (int i = 0; i < N_OBJS; i++) {
        float p[3];
        demo_object_center(s->objs[i], p);
        float depth = p[1] - WATER_SURFACE_Y;
        float sub = submerged_fraction(&descs[i], p[1]);
        const char *status = sub >= 0.999f ? (p[1] > SEA_FLOOR_Y - 2.0f ? "SUNK" : "SINKING")
                           : (sub > 0.0f ? "FLOATING" : "ABOVE");
        demo_hud(v, descs[i].col, "%s  depth:%.1f  %s  rel. density %.2f",
                 descs[i].label, (double)depth, status,
                 (double)(desc_density(&descs[i]) / WATER_DENSITY));
    }
    demo_hud(v, DEMO_COL_INFO, "Current: +X %.1f m/s", (double)FLOW_SPEED_X);
}

static void buoyancy_stats(demo_app_t *app, char *buf, size_t size)
{
    buoyancy_demo_t *s = (buoyancy_demo_t *)app->user;
    int n = snprintf(buf, size, "max_speed=%.2f", (double)s->max_speed);
    for (int i = 0; i < N_OBJS && n > 0 && (size_t)n < size; i++) {
        float p[3];
        demo_object_center(s->objs[i], p);
        n += snprintf(buf + n, size - (size_t)n, " %.5s(%.1f,%.2f)", descs[i].label,
                      (double)p[0], (double)p[1]);
    }
}

static bool buoyancy_check(demo_app_t *app)
{
    buoyancy_demo_t *s = (buoyancy_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    for (int i = 0; i < N_OBJS; i++) {
        const obj_desc_t *d = &descs[i];
        float p[3];
        demo_object_center(s->objs[i], p);
        float density = desc_density(d);
        float speed = demo_object_speed(s->objs[i]);
        if (density < WATER_DENSITY) {
            /* floats: the expected immersion is density / water density */
            float sub = submerged_fraction(d, p[1]);
            float expect = density / WATER_DENSITY;
            ok &= demo_check(tag, sub > 0.0f && sub < 1.0f && fabsf(sub - expect) < 0.25f && speed < 2.0f,
                             "%s (rel. density %.2f) floats: %.0f%% submerged (expected ~%.0f%%), y=%.2f speed %.2f",
                             d->label, (double)(density / WATER_DENSITY), (double)(sub * 100.0f), (double)(expect * 100.0f),
                             (double)p[1], (double)speed);
            ok &= demo_check(tag, p[0] - d->start_pos[0] > 1.0f,
                             "%s drifted with the current (+X %.2f m)", d->label,
                             (double)(p[0] - d->start_pos[0]));
        } else {
            float h = desc_half_height(d);
            ok &= demo_check(tag, p[1] > SEA_FLOOR_Y - 2.0f * h - 0.3f && speed < 0.5f,
                             "%s (rel. density %.2f) sank to the floor: y=%.2f (floor %.1f) speed %.2f",
                             d->label, (double)(density / WATER_DENSITY), (double)p[1], (double)SEA_FLOOR_Y, (double)speed);
        }
    }
    ok &= demo_check(tag, s->max_speed < 30.0f, "plausible speeds (max %.2f m/s)", (double)s->max_speed);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Buoyancy -- Floating & Sinking";
    app.cfg.tag          = "buoyancy";
    app.cfg.orbit_dist   = 28.0f;
    app.cfg.orbit_yaw    = 20.0f;
    app.cfg.orbit_pitch  = -20.0f;
    app.cfg.target[1]    = -2.0f;
    app.cfg.grid_extent  = 15.0f;
    app.cfg.grid_spacing = 2.0f;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    buoyancy_demo_t *s = (buoyancy_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* ── Ground (sea floor) ─────────────────────────────────────────────── */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    gt.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(5.0), IVP_REAL(0.0));
    s->ground = demo_create_box_templ(w, &gt, 15.0, 0.5, 15.0);

    /* ── Side walls (keep objects from drifting away with current) ──── *
     * They reach 1 m above the water surface (y = -8 .. 6): floating
     * objects are stopped by the right wall instead of drifting over it. */
    ivp_object_template_t wt;
    ivp_object_template_init(&wt);
    wt.mass = 0.0f;
    wt.physical_unmoveable = true;
    wt.position = ivp_vec3r(IVP_REAL(-12.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    s->wall_l = demo_create_box_templ(w, &wt, 0.3, 7.0, 15.0);
    wt.position = ivp_vec3r(IVP_REAL(12.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    s->wall_r = demo_create_box_templ(w, &wt, 0.3, 7.0, 15.0);

    /* ── Floatable objects ──────────────────────────────────────────────── */
    for (int i = 0; i < N_OBJS; i++) {
        const obj_desc_t *d = &descs[i];
        ivp_object_template_t ot;
        ivp_object_template_init(&ot);
        ot.mass = (ivp_real_t)d->mass;
        if (d->is_sphere)
            set_sphere_inertia(&ot, ot.mass, (ivp_real_t)d->hs[0]);
        else
            set_box_inertia(&ot, ot.mass,
                (ivp_real_t)d->hs[0], (ivp_real_t)d->hs[1], (ivp_real_t)d->hs[2]);
        ot.material.friction   = IVP_REAL(0.5);
        ot.material.elasticity = IVP_REAL(0.2);
        ot.position = ivp_vec3r((ivp_real_t)d->start_pos[0], (ivp_real_t)d->start_pos[1],
                                (ivp_real_t)d->start_pos[2]);
        if (d->is_sphere)
            s->objs[i] = demo_create_ball_templ(w, &ot, d->hs[0]);
        else
            s->objs[i] = demo_create_box_templ(w, &ot, d->hs[0], d->hs[1], d->hs[2]);
        if (!s->objs[i]) return 1;
    }
    give_initial_speed(s);

    ivp_buoyancy_init(&s->buoy, (ivp_real_t)WATER_SURFACE_Y, (ivp_real_t)WATER_DENSITY);
    s->buoy.linear_drag  = IVP_REAL(3.0);
    s->buoy.angular_drag = IVP_REAL(2.0);
    ivp_vec3r_set(&s->buoy.flow_velocity, (ivp_real_t)FLOW_SPEED_X, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_add_controller(w->env, 200, ivp_controller_buoyancy, &s->buoy);

    app.update   = buoyancy_update;
    app.on_reset = buoyancy_reset;
    app.draw     = buoyancy_draw;
    app.stats    = buoyancy_stats;
    app.check    = buoyancy_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    ivp_buoyancy_state_destroy(&s->buoy);
    free(s);
    return rc;
}
