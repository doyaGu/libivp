/* example_factory.c -- Controllers made from templates, owned by the environment
 *
 * IVP_Controller_Factory creates a controller from a template and registers it
 * the way the reference constructors do. The environment owns the controller:
 * it can be deleted by hand, it goes when one of its anchor objects goes, and
 * what is left goes with the environment. Five stations, all built from
 * templates:
 *
 *   spring   a bob on a spring from a point in the world (world anchor: a NULL object)
 *   lamp     a check-distance actuator between the bob and a point; its callback
 *            lights the lamp while the bob is within 1 m
 *   push     a force actuator pushes a crate along a slippery floor, away from a
 *            point in the world behind it; the force is
 *            an active float, set from the panel
 *   spin     a torque actuator on a bar on an axle; the torque is an active float,
 *            a second float receives the measured rotation speed
 *   platform a motion controller (made by the factory) follows a circle
 *
 * The HUD lists the controllers and whether the environment still owns them
 * (ivp_controller_factory_get_client_data finds the live ones).
 *
 *   X   delete the bob: the spring and the lamp go with it
 *   Z   delete the platform: its motion controller goes with it
 *   F5 panel   force, torque, spring stiffness, and switches that delete a
 *              controller or make it again
 *
 * Demonstrates: ivp_template_*, ivp_controller_factory_create_spring / _force /
 *               _torque / _check_dist / _controller_motion, _delete, _get_client_data,
 *               active floats bound to templates.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_constraint_local.h>
#include <ivp/controllers/ivp_controller_factory.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/utility/ivp_active_value.h>

#define X_SPRING  (-12.0)
#define X_PUSH    (-6.0)
#define X_SPIN    0.0
#define X_PLAT    12.5

typedef struct factory_demo {
    ivp_environment_t           *env;
    ivp_object_t                *ground, *wall;
    ivp_object_t                *bob, *crate, *bar, *platform;
    /* factory controllers (NULL or a pointer the environment owns) */
    ivp_spring_state_t          *spring;
    ivp_check_dist_state_t      *lamp;
    ivp_force_actuator_state_t  *push;
    ivp_torque_actuator_state_t *spin;
    ivp_motion_state_t          *motion;
    ivp_constraint_local_state_t axle;
    /* active floats */
    ivp_active_float_t          *af_force, *af_torque, *af_spin;
    /* panel */
    float                        force, torque, spring_k;
    bool                         want_spring, want_push;
    /* lamp */
    bool                         lamp_on;
    int                          lamp_events;
    bool                         lamp_deleted_seen;
    /* measurements */
    float                        push_x0, bob_swing_min, bob_swing_max, plat_x_min, plat_x_max;
    bool                         bob_gone, platform_gone;
    double                       axle_speed;
} factory_demo_t;

static bool alive(const factory_demo_t *s, const void *controller)
{
    /* the template's client data finds the live ones; the motion controller has none, but a handle */
    return controller && (ivp_controller_factory_get_client_data(s->env, controller) != NULL ||
                          ivp_controller_handle_is_valid(ivp_controller_factory_get_handle(s->env, controller)));
}

/* ── creation from templates ─────────────────────────────────────────────── */

static ivp_spring_state_t *make_spring(factory_demo_t *s)
{
    ivp_template_anchor_t a0, a1;
    ivp_template_anchor_set_position_ws(&a0, NULL, &(const ivp_vec3r_t){ IVP_REAL(X_SPRING), IVP_REAL(-6.0), IVP_REAL(0.0) });
    ivp_template_anchor_set_position_os(&a1, s->bob, &(const ivp_vec3r_t){ IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0) });
    ivp_template_spring_t t;
    ivp_template_spring_init(&t);
    t.client_data = (void *)"spring";
    t.anchors[0] = &a0;
    t.anchors[1] = &a1;
    t.spring_len = 3.0f;
    t.spring_constant = s->spring_k;
    t.spring_damp = 1.0f;
    return ivp_controller_factory_create_spring(s->env, &t);
}

static void lamp_event(void *ctx, ivp_object_t *a, ivp_object_t *b, ivp_real_t distance, bool inside)
{
    factory_demo_t *s = (factory_demo_t *)ctx;
    (void)a;
    (void)b;
    (void)distance;
    s->lamp_on = inside;
    s->lamp_events++;
}

static void lamp_deleted(void *ctx, struct ivp_check_dist_state *state)
{
    factory_demo_t *s = (factory_demo_t *)ctx;
    (void)state;
    s->lamp_deleted_seen = true;
    s->lamp_on = false;
}

static ivp_check_dist_state_t *make_lamp(factory_demo_t *s)
{
    ivp_template_check_dist_t t;
    ivp_template_check_dist_init(&t);
    t.client_data = (void *)"lamp";
    t.objects[0] = s->bob;
    t.objects[1] = ivp_environment_get_static_object(s->env);       /* the world: the check needs two objects */
    t.position_world_space[0] = ivp_vec3r(IVP_REAL(X_SPRING), IVP_REAL(-3.0), IVP_REAL(0.0));
    t.position_world_space[1] = ivp_vec3r(IVP_REAL(X_SPRING), IVP_REAL(-1.8), IVP_REAL(0.0));
    t.range = 1.0f;
    ivp_check_dist_state_t *c = ivp_controller_factory_create_check_dist(s->env, &t);
    if (c) {
        ivp_check_dist_set_callback(c, lamp_event, s);
    }
    return c;
}

static ivp_force_actuator_state_t *make_push(factory_demo_t *s)
{
    ivp_template_anchor_t a0, a1;
    ivp_template_anchor_set_position_os(&a0, s->crate, &(const ivp_vec3r_t){ IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0) });
    ivp_template_anchor_set_position_ws(&a1, NULL, &(const ivp_vec3r_t){ IVP_REAL(X_PUSH - 30.0), IVP_REAL(-0.5), IVP_REAL(0.0) });
    ivp_template_force_t t;
    ivp_template_force_init(&t);
    t.client_data = (void *)"push";
    t.anchors[0] = &a0;
    t.anchors[1] = &a1;
    t.force = s->force;
    t.active_float_force = s->af_force;
    return ivp_controller_factory_create_force(s->env, &t);
}

static ivp_torque_actuator_state_t *make_spin(factory_demo_t *s)
{
    ivp_template_anchor_t a0, a1;
    ivp_template_anchor_set_position_os(&a0, s->bar, &(const ivp_vec3r_t){ IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0) });
    ivp_template_anchor_set_position_os(&a1, s->bar, &(const ivp_vec3r_t){ IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0) });
    ivp_template_torque_t t;
    ivp_template_torque_init(&t);
    t.client_data = (void *)"spin";
    t.anchors[0] = &a0;
    t.anchors[1] = &a1;
    t.torque = s->torque;
    t.max_rotation_speed = 5.0f;
    t.active_float_torque = s->af_torque;
    t.active_float_rotation_speed_out = s->af_spin;
    return ivp_controller_factory_create_torque(s->env, &t);
}

/* ── panel ───────────────────────────────────────────────────────────────── */

static void apply_force(void *ctx)
{
    factory_demo_t *s = (factory_demo_t *)ctx;
    ivp_active_float_set(s->af_force, (double)s->force);
    if (s->crate) ivp_environment_ensure_core_in_simulation(s->env, s->crate);
}

static void apply_torque(void *ctx)
{
    factory_demo_t *s = (factory_demo_t *)ctx;
    ivp_active_float_set(s->af_torque, (double)s->torque);
    ivp_environment_ensure_core_in_simulation(s->env, s->bar);
}

static void apply_spring_k(void *ctx)
{
    factory_demo_t *s = (factory_demo_t *)ctx;
    if (alive(s, s->spring)) ivp_spring_set_constant(s->spring, (ivp_real_t)s->spring_k);
    if (s->bob && !s->bob_gone) ivp_environment_ensure_core_in_simulation(s->env, s->bob);
}

static void apply_want_spring(void *ctx)
{
    factory_demo_t *s = (factory_demo_t *)ctx;
    if (s->want_spring && !alive(s, s->spring) && !s->bob_gone) s->spring = make_spring(s);
    if (!s->want_spring && alive(s, s->spring)) {
        ivp_controller_factory_delete(s->env, s->spring);
        s->spring = NULL;
    }
    if (!s->bob_gone) ivp_environment_ensure_core_in_simulation(s->env, s->bob);
}

static void apply_want_push(void *ctx)
{
    factory_demo_t *s = (factory_demo_t *)ctx;
    if (s->want_push && !alive(s, s->push)) s->push = make_push(s);
    if (!s->want_push && alive(s, s->push)) {
        ivp_controller_factory_delete(s->env, s->push);
        s->push = NULL;
    }
    ivp_environment_ensure_core_in_simulation(s->env, s->crate);
}

static void factory_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    factory_demo_t *s = (factory_demo_t *)app->user;
    demo_panel_t *p = &v->panel;
    demo_panel_add_float(p, "push force", &s->force, 0.0f, 60.0f, apply_force, s);
    demo_panel_add_float(p, "torque", &s->torque, -6.0f, 6.0f, apply_torque, s);
    demo_panel_add_float(p, "spring k", &s->spring_k, 20.0f, 200.0f, apply_spring_k, s);
    demo_panel_add_bool(p, "spring exists", &s->want_spring, apply_want_spring, s);
    demo_panel_add_bool(p, "push exists", &s->want_push, apply_want_push, s);
}

/* ── per frame ───────────────────────────────────────────────────────────── */

static void factory_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    factory_demo_t *s = (factory_demo_t *)app->user;
    demo_world_t *w = &app->world;
    const double t = (double)ivp_environment_get_time(s->env);

    if ((v && demo_key_just_pressed(v, SDL_SCANCODE_X)) || (!v && frame == 420)) {
        if (!s->bob_gone) {
            s->bob_gone = true;
            demo_destroy_object(w, s->bob);
            s->bob = NULL;
            /* the environment deleted the spring and the lamp with the bob */
        }
    }
    if ((v && demo_key_just_pressed(v, SDL_SCANCODE_Z)) || (!v && frame == 480)) {
        if (!s->platform_gone) {
            s->platform_gone = true;
            demo_destroy_object(w, s->platform);
            s->platform = NULL;
        }
    }
    if (!s->platform_gone && alive(s, s->motion)) {
        const double a = 1.2 * t;
        const ivp_vec3r_t target = ivp_vec3r((ivp_real_t)(X_PLAT + 2.0 * cos(a)), (ivp_real_t)(-2.5 + 0.8 * sin(2.0 * a)),
                                             (ivp_real_t)(2.0 * sin(a)));
        ivp_motion_state_set_target_position(s->motion, &target);
        float pc[3];
        demo_object_center(s->platform, pc);
        if (t > 1.0 && pc[0] < s->plat_x_min) s->plat_x_min = pc[0];
        if (t > 1.0 && pc[0] > s->plat_x_max) s->plat_x_max = pc[0];
    }
    if (!v && frame == 150) {
        s->force = 30.0f;
        apply_force(s);
    }
    if (!v && frame == 200) {
        s->torque = 3.0f;
        apply_torque(s);
    }
    if (!v && frame == 300) {
        s->want_push = false;
        apply_want_push(s);
    }
    if (!v && frame == 360) {
        s->want_push = true;
        apply_want_push(s);
    }
    if (s->bob && !s->bob_gone) {
        float c[3];
        demo_object_center(s->bob, c);
        if (t > 1.0 && c[1] < s->bob_swing_min) s->bob_swing_min = c[1];
        if (t > 1.0 && c[1] > s->bob_swing_max) s->bob_swing_max = c[1];
    }
    s->axle_speed = ivp_active_float_get(s->af_spin);
}

static void factory_reset(demo_app_t *app)
{
    factory_demo_t *s = (factory_demo_t *)app->user;
    s->bob_swing_min = 1e9f;
    s->bob_swing_max = -1e9f;
    s->plat_x_min = 1e9f;
    s->plat_x_max = -1e9f;
}

/* ── drawing ─────────────────────────────────────────────────────────────── */

static const char *status(const factory_demo_t *s, const void *c)
{
    return alive(s, c) ? "owned by the environment" : "deleted";
}

static void factory_draw(demo_app_t *app, demo_viewer_t *v)
{
    factory_demo_t *s = (factory_demo_t *)app->user;
    const float c_ground[3] = { 0.50f, 0.53f, 0.57f }, c_bob[3] = { 0.95f, 0.65f, 0.30f }, c_crate[3] = { 0.40f, 0.78f, 0.95f },
                c_bar[3] = { 0.55f, 0.90f, 0.50f }, c_plat[3] = { 0.95f, 0.45f, 0.35f }, c_lamp_on[3] = { 1.0f, 0.95f, 0.3f },
                c_lamp_off[3] = { 0.35f, 0.35f, 0.30f };
    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->wall, c_ground);
    if (s->bob) demo_draw_object(v, s->bob, c_bob);
    demo_draw_object(v, s->crate, c_crate);
    demo_draw_object(v, s->bar, c_bar);
    if (s->platform) demo_draw_object(v, s->platform, c_plat);

    const float anchor[3] = { (float)X_SPRING, -6.0f, 0.0f }, lamp[3] = { (float)X_SPRING, -1.8f, 0.0f };
    demo_sphere(v, anchor, 0.15f, DEMO_COL_STATIC);
    demo_sphere(v, lamp, 0.25f, s->lamp_on ? c_lamp_on : c_lamp_off);
    if (s->bob && alive(s, s->spring)) {
        float c[3];
        demo_object_center(s->bob, c);
        demo_line(v, anchor, c, DEMO_COL_SPRING);
    }
    if (alive(s, s->push)) {
        float c[3];
        demo_object_center(s->crate, c);
        const float tip[3] = { c[0] + 0.1f * s->force, c[1], c[2] };
        demo_arrow(v, c, tip, DEMO_COL_VELOCITY, 0.3f);
    }

    demo_hud(v, alive(s, s->spring) ? DEMO_COL_INFO : DEMO_COL_WARNING, "spring    %s", status(s, s->spring));
    demo_hud(v, alive(s, s->lamp) ? DEMO_COL_INFO : DEMO_COL_WARNING, "lamp      %s, %s (%d events)", status(s, s->lamp),
             s->lamp_on ? "on" : "off", s->lamp_events);
    demo_hud(v, alive(s, s->push) ? DEMO_COL_INFO : DEMO_COL_WARNING, "push      %s, force %.0f N", status(s, s->push),
             (double)s->force);
    demo_hud(v, alive(s, s->spin) ? DEMO_COL_INFO : DEMO_COL_WARNING, "spin      %s, torque %.1f, bar turns at %.2f rad/s",
             status(s, s->spin), (double)s->torque, s->axle_speed);
    demo_hud(v, alive(s, s->motion) ? DEMO_COL_INFO : DEMO_COL_WARNING, "platform  %s", status(s, s->motion));
}

static void factory_stats(demo_app_t *app, char *buf, size_t size)
{
    factory_demo_t *s = (factory_demo_t *)app->user;
    snprintf(buf, size, "alive spring %d lamp %d push %d spin %d motion %d, lamp events %d", alive(s, s->spring), alive(s, s->lamp),
             alive(s, s->push), alive(s, s->spin), alive(s, s->motion), s->lamp_events);
}

static bool factory_check(demo_app_t *app)
{
    factory_demo_t *s = (factory_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    float c[3];
    ok &= demo_check(tag, s->bob_swing_max - s->bob_swing_min > 1.0f, "the bob swung on its spring (%.2f m)",
                     (double)(s->bob_swing_max - s->bob_swing_min));
    ok &= demo_check(tag, s->lamp_events >= 2, "the lamp switched %d times", s->lamp_events);
    demo_object_center(s->crate, c);
    ok &= demo_check(tag, c[0] > s->push_x0 + 1.0f, "the pushed crate moved %.1f m", (double)(c[0] - s->push_x0));
    ok &= demo_check(tag, fabs(s->axle_speed) > 1.0, "the torque turns the bar (%.2f rad/s)", s->axle_speed);
    ok &= demo_check(tag, alive(s, s->push), "the push was deleted and made again, and lives");
    ok &= demo_check(tag, s->bob_gone && !alive(s, s->spring) && !alive(s, s->lamp),
                     "deleting the bob took the spring and the lamp with it");
    ok &= demo_check(tag, s->plat_x_max - s->plat_x_min > 2.0f, "the motion controller carried the platform around its circle (%.1f m wide)",
                     (double)(s->plat_x_max - s->plat_x_min));
    ok &= demo_check(tag, s->platform_gone && !alive(s, s->motion), "deleting the platform took its motion controller");
    ok &= demo_check(tag, alive(s, s->spin), "the torque actuator is still there");
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Factory -- Controllers from Templates";
    app.cfg.tag         = "factory";
    app.cfg.friction    = 0.5;
    app.cfg.elasticity  = 0.2;
    app.cfg.orbit_dist  = 30.0f;
    app.cfg.orbit_yaw   = 180.0f;
    app.cfg.orbit_pitch = -18.0f;
    app.cfg.target[0]   = -2.0f;
    app.cfg.target[1]   = -3.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    factory_demo_t *s = (factory_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;
    s->force = 0.0f;
    s->torque = 0.0f;
    s->spring_k = 60.0f;
    s->want_spring = true;
    s->want_push = true;
    s->bob_swing_min = 1e9f;
    s->bob_swing_max = -1e9f;
    s->plat_x_min = 1e9f;
    s->plat_x_max = -1e9f;

    s->ground = demo_create_box(w, NULL, 18.0, 0.5, 5.0, 0.0, NULL, 0.0, 0.5, 0.0);
    s->wall = demo_create_box(w, NULL, 0.4, 1.0, 5.0, 0.0, NULL, X_PUSH + 14.0, -1.0, 0.0);

    /* spring and lamp */
    s->bob = demo_create_ball(w, NULL, 0.4, 1.0, NULL, X_SPRING, -4.6, 0.0);

    /* push: a crate on a slippery strip */
    const ivp_material_t ice = demo_material(0.03, 0.1);
    s->crate = demo_create_box(w, &ice, 0.6, 0.6, 0.6, 2.0, NULL, X_PUSH, -0.62, 0.0);
    s->push_x0 = (float)X_PUSH;

    /* spin: a slab on a vertical axle (a local constraint that frees the rotation about y) */
    s->bar = demo_create_box(w, NULL, 1.4, 0.15, 0.4, 2.0, NULL, X_SPIN, -3.0, 0.0);
    {
        const ivp_vec3r_t a = ivp_vec3r(IVP_REAL(X_SPIN), IVP_REAL(-3.0), IVP_REAL(0.0)), b = ivp_vec3r_zero();
        ivp_constraint_local_init(&s->axle, NULL, &a, s->bar, &b);
        ivp_constraint_local_free_rotation(&s->axle, 1);
        ivp_environment_announce_dependent_controller(s->env, IVP_CP_CONSTRAINTS, ivp_controller_constraint_local, &s->axle,
                                                      ivp_constraint_local_discovery);
    }

    /* platform driven by a factory motion controller */
    const ivp_material_t m = demo_material(0.5, 0.1);
    s->platform = demo_create_box(w, &m, 1.2, 0.2, 1.2, 4.0, NULL, X_PLAT, -2.5, 0.0);
    if (!s->bob || !s->crate || !s->bar || !s->platform) return 1;

    s->af_force = ivp_active_float_terminal_create(NULL, "force", 0.0);
    s->af_torque = ivp_active_float_terminal_create(NULL, "torque", 0.0);
    s->af_spin = ivp_active_float_terminal_create(NULL, "spin", 0.0);

    s->spring = make_spring(s);
    s->lamp = make_lamp(s);
    s->push = make_push(s);
    s->spin = make_spin(s);
    ivp_template_controller_motion_t mt;
    ivp_template_controller_motion_init(&mt);
    s->motion = ivp_controller_factory_create_controller_motion(s->env, s->platform, &mt);
    if (!s->spring || !s->lamp || !s->push || !s->spin || !s->motion) {
        fprintf(stderr, "[factory] could not create: spring %p lamp %p push %p spin %p motion %p\n", (void *)s->spring, (void *)s->lamp, (void *)s->push, (void *)s->spin, (void *)s->motion);
        return 1;
    }

    app.controls    = "X: delete the bob   Z: delete the platform   F5: force, torque, switches";
    app.viewer_init = factory_viewer_init;
    app.update      = factory_update;
    app.on_reset    = factory_reset;
    app.draw        = factory_draw;
    app.stats       = factory_stats;
    app.check       = factory_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);                 /* the environment deletes what is left */
    ivp_active_float_remove_ref(s->af_spin);
    ivp_active_float_remove_ref(s->af_torque);
    ivp_active_float_remove_ref(s->af_force);
    free(s);
    return rc;
}
