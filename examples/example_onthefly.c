/* example_onthefly.c -- Changing object properties at runtime
 *
 * Port of ivp_examples/examples/ive_example_onthefly.cxx.
 *
 * Two cubes (low friction material 0.3 / 0.2) rest on a high friction
 * ground (0.9 / 0.8) and are connected by a spring (k = 5, rest length 1,
 * damping 0.1) that pulls them together.  While the simulation runs:
 *   1-5  change the mass of both cubes (0.5, 1, 2, 5, 10 kg)
 *   G    toggle the spring (spring constant 5 <-> 0)
 *   V    cancel the velocities of both cubes (the reference uses R, which
 *        is a camera key in the libivp examples)
 *
 * Demonstrates: ivp_object_change_mass (IVP_Real_Object::change_mass),
 *               ivp_spring_set_constant (IVP_Actuator_Spring::set_constant),
 *               per-object materials, asynchronous speed changes.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_actuator_spring.h>

static const double CUBE_HS = 0.4;
static const double SPRING_K = 5.0;

typedef struct onthefly {
    ivp_material_t     mat_lo;
    ivp_material_t     mat_hi;
    ivp_object_t      *ground;
    ivp_object_t      *cube1;
    ivp_object_t      *cube2;
    ivp_spring_state_t spring;
    bool               spring_on;
    double             current_mass;
    const char        *last_action;
} onthefly_t;

static void set_mass(onthefly_t *s, double mass)
{
    s->current_mass = mass;
    ivp_object_change_mass(s->cube1, (ivp_real_t)mass);
    ivp_object_change_mass(s->cube2, (ivp_real_t)mass);
    s->last_action = "mass changed";
}

static void toggle_spring(onthefly_t *s)
{
    s->spring_on = !s->spring_on;
    ivp_spring_set_constant(&s->spring, (ivp_real_t)(s->spring_on ? SPRING_K : 0.0));
    s->last_action = s->spring_on ? "spring on" : "spring off";
}

/* async_add_speed_object_ws is additive: add the negated current speeds. */
static void zero_velocities(demo_app_t *app, onthefly_t *s)
{
    ivp_object_t *cubes[2] = { s->cube1, s->cube2 };
    for (int i = 0; i < 2; i++) {
        ivp_core_t *c = cubes[i]->physical_core;
        demo_async_add_speed_ws(app->world.env, cubes[i],
                                -(double)c->speed.k[0], -(double)c->speed.k[1], -(double)c->speed.k[2]);
        demo_async_add_rot_speed_cs(app->world.env, cubes[i],
                                    -(double)c->rot_speed.k[0], -(double)c->rot_speed.k[1],
                                    -(double)c->rot_speed.k[2]);
    }
    s->last_action = "velocities cancelled";
}

static void onthefly_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    onthefly_t *s = (onthefly_t *)app->user;
    if (v) {
        static const double masses[5] = { 0.5, 1.0, 2.0, 5.0, 10.0 };
        static const SDL_Scancode keys[5] = { SDL_SCANCODE_1, SDL_SCANCODE_2, SDL_SCANCODE_3,
                                              SDL_SCANCODE_4, SDL_SCANCODE_5 };
        for (int i = 0; i < 5; i++)
            if (demo_key_just_pressed(v, keys[i])) set_mass(s, masses[i]);
        if (demo_key_just_pressed(v, SDL_SCANCODE_G)) toggle_spring(s);
        if (demo_key_just_pressed(v, SDL_SCANCODE_V)) zero_velocities(app, s);
        return;
    }
    /* headless script: heavy cubes, spring off, kick, cancel, spring on */
    switch (frame) {
    case 180: set_mass(s, 5.0); break;
    case 240: toggle_spring(s); break;                      /* off */
    case 250: demo_async_add_speed_ws(app->world.env, s->cube1, -3.0, 0.0, 0.0); break;
    case 300: zero_velocities(app, s); break;
    case 360: set_mass(s, 0.5); toggle_spring(s); break;    /* light + on */
    default: break;
    }
}

static void onthefly_reset(demo_app_t *app)
{
    onthefly_t *s = (onthefly_t *)app->user;
    s->last_action = "scene reset";
}

static void onthefly_draw(demo_app_t *app, demo_viewer_t *v)
{
    onthefly_t *s = (onthefly_t *)app->user;
    demo_draw_object(v, s->ground, DEMO_COL_STATIC);
    demo_draw_object(v, s->cube1, DEMO_COL_OBJECT_B);
    demo_draw_object(v, s->cube2, DEMO_COL_OBJECT_A);
    if (s->spring_on) demo_draw_link(v, s->cube1, s->cube2, DEMO_COL_HIGHLIGHT);
    demo_draw_velocity(v, s->cube1, DEMO_COL_VELOCITY);
    demo_draw_velocity(v, s->cube2, DEMO_COL_VELOCITY);

    demo_hud(v, DEMO_COL_INFO, "Mass: %.1f kg (core mass %.2f)", s->current_mass,
             (double)s->cube1->physical_core->mass);
    demo_hud(v, DEMO_COL_INFO, "Spring: %s (k = %.1f)   distance %.2f m",
             s->spring_on ? "ON" : "OFF", (double)s->spring.stiffness,
             (double)demo_object_distance(s->cube1, s->cube2));
    if (s->last_action) demo_hud(v, DEMO_COL_HIGHLIGHT, "Last action: %s", s->last_action);
}

static void onthefly_stats(demo_app_t *app, char *buf, size_t size)
{
    onthefly_t *s = (onthefly_t *)app->user;
    snprintf(buf, size, "mass=%.2f spring=%s k=%.1f dist=%.3f v1=%.3f v2=%.3f",
             (double)s->cube1->physical_core->mass, s->spring_on ? "on" : "off",
             (double)s->spring.stiffness, (double)demo_object_distance(s->cube1, s->cube2),
             (double)demo_object_speed(s->cube1), (double)demo_object_speed(s->cube2));
}

static bool onthefly_check(demo_app_t *app)
{
    onthefly_t *s = (onthefly_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    float d = demo_object_distance(s->cube1, s->cube2);
    ok &= demo_check(tag, fabsf((float)s->cube1->physical_core->mass - 0.5f) < 1e-4f &&
                          fabsf((float)s->cube2->physical_core->mass - 0.5f) < 1e-4f,
                     "change_mass applied (mass %.3f / %.3f)",
                     (double)s->cube1->physical_core->mass, (double)s->cube2->physical_core->mass);
    ok &= demo_check(tag, s->spring_on && fabsf((float)s->spring.stiffness - (float)SPRING_K) < 1e-4f,
                     "spring re-enabled (k = %.2f)", (double)s->spring.stiffness);
    ok &= demo_check(tag, d > 0.8f && d < 2.5f,
                     "spring pulled the cubes together (distance %.3f m, started at 3 m)", (double)d);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Example: On The Fly";
    app.cfg.tag         = "onthefly";
    app.cfg.orbit_dist  = 18.0f;
    app.cfg.orbit_pitch = -25.0f;
    app.cfg.target[1]   = -2.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    onthefly_t *s = (onthefly_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* Two custom materials (the default material is not used) */
    s->mat_lo = demo_material(0.3, 0.2);
    s->mat_hi = demo_material(0.9, 0.8);

    s->ground = demo_create_box(w, &s->mat_hi, 10.0, 0.5, 10.0, 0.0, NULL, 0.0, 0.5, 0.0);
    s->cube1 = demo_create_box(w, &s->mat_lo, CUBE_HS, CUBE_HS, CUBE_HS, 1.0, NULL, -1.5, -3.0, 0.0);
    s->cube2 = demo_create_box(w, &s->mat_lo, CUBE_HS, CUBE_HS, CUBE_HS, 1.0, NULL,  1.5, -3.0, 0.0);
    s->current_mass = 1.0;

    /* IVP_Template_Spring: k = 5, rest length 1, damping 0.1, anchors at
     * the object origins (absolute values, spring_values_are_relative off) */
    {
        ivp_vec3r_t a = ivp_vec3r(0, 0, 0), b = ivp_vec3r(0, 0, 0);
        ivp_spring_state_init(&s->spring);
        ivp_spring_state_setup(&s->spring, s->cube1, &a, s->cube2, &b,
                               (ivp_real_t)SPRING_K, IVP_REAL(0.1), IVP_REAL(1.0));
        ivp_environment_add_controller(w->env, IVP_CP_SPRING, ivp_controller_spring, &s->spring);
    }
    s->spring_on = true;

    app.controls = "1-5: set mass (0.5/1/2/5/10)   G: toggle spring   V: cancel velocities";
    app.update   = onthefly_update;
    app.on_reset = onthefly_reset;
    app.draw     = onthefly_draw;
    app.stats    = onthefly_stats;
    app.check    = onthefly_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
