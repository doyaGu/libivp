/* example_actuators.c -- Torque, suspension and stabilizer actuators
 *
 * Left, three turntables, each a bar on a vertical axle (a local constraint
 * that frees only the rotation about y), each driven by a torque actuator:
 *
 *   A   torque 3 N m, speed limit 6 rad/s: it spins up and holds the limit
 *       (the actuator applies the torque only while |w| <= the limit)
 *   B   torque -3 N m, limit 3 rad/s: the same the other way round, slower
 *   C   limit 0, the reference default: the torque acts only while the bar
 *       does not turn at all, so it gets one kick and then coasts
 *
 * Right, two axle rigs. A chassis stands on two wheels (balls) connected to
 * it by suspension actuators: a spring with separate damping for rebound and
 * compression, and a cap on the force the chassis receives. A spring alone
 * is unstable sideways, like a pogo stick, so each wheel also has a strut, a
 * local constraint that frees only the vertical axis. Every 3 s the right
 * wheel is kicked upwards. The right rig additionally has a stabilizer
 * actuator between its two wheels. The stabilizer follows the reference law,
 * which pushes the two pairs of anchors further apart when their distances
 * differ, so it does not damp the roll: the right chassis rolls more than
 * the left one, and above a stiffness of about 300 the loop is unstable and
 * the chassis is thrown off. Compare the roll angles on the HUD.
 *
 *   F5 panel   turntable A torque and speed limit, suspension stiffness,
 *              stabilizer constant
 *
 * Demonstrates: ivp_torque_actuator_*, ivp_suspension_*, ivp_stabilizer_*,
 *               ivp_environment_add_controller, ivp_constraint_local_*.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_actuator_stabilizer.h>
#include <ivp/controllers/ivp_actuator_suspension.h>
#include <ivp/controllers/ivp_actuator_torque.h>
#include <ivp/controllers/ivp_constraint_local.h>

#define N_TURN   3
#define N_RIGS   2
#define KICK_PERIOD 3.0

typedef struct turntable {
    const char                  *name;
    double                       x, torque, limit;
    ivp_object_t                *bar;
    ivp_constraint_local_state_t axle;
    ivp_torque_actuator_state_t  torque_act;
    float                        peak;           /* highest |w| */
} turntable_t;

typedef struct rig {
    double                       x;
    bool                         stabilised;
    ivp_object_t                *chassis, *wheel[2];
    ivp_suspension_state_t       susp[2];
    ivp_constraint_local_state_t strut[2];     /* guides the wheel on a vertical line */
    ivp_stabilizer_state_t       stab;
    float                        roll, max_roll;   /* chassis roll (rad) */
} rig_t;

typedef struct actuators_demo {
    ivp_environment_t *env;
    ivp_object_t      *ground;
    turntable_t        turn[N_TURN];
    rig_t              rig[N_RIGS];
    long               kicks;
    /* panel */
    float              torque_a, limit_a, susp_k, stab_k;
} actuators_demo_t;

static const float susp_damp = 60.0f;

/* The bar is a slab, not a thin rod: with a thin bar (an inertia ratio of 1:46 between its axes)
 * the dense solve of the axle constraint, whose rotation about y is free, grows an instability
 * within a second. */
/* ── turntables ──────────────────────────────────────────────────────────── */

static void setup_turntable(actuators_demo_t *s, demo_world_t *w, turntable_t *t)
{
    t->bar = demo_create_box(w, NULL, 1.4, 0.15, 0.4, 2.0, NULL, t->x, -3.0, 0.0);
    const ivp_vec3r_t a = ivp_vec3r(IVP_REAL(t->x), IVP_REAL(-3.0), IVP_REAL(0.0)), b = ivp_vec3r_zero();
    ivp_constraint_local_init(&t->axle, NULL, &a, t->bar, &b);
    ivp_constraint_local_free_rotation(&t->axle, 1);
    ivp_environment_announce_dependent_controller(s->env, IVP_CP_CONSTRAINTS, ivp_controller_constraint_local,
                                                  &t->axle, ivp_constraint_local_discovery);
    /* the axis is the line between two object-space anchors: down the y axis */
    const ivp_vec3r_t a0 = ivp_vec3r_zero(), a1 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_torque_actuator_state_init(&t->torque_act);
    ivp_torque_actuator_state_setup_anchors(&t->torque_act, t->bar, &a0, &a1, (ivp_real_t)t->torque,
                                            (ivp_real_t)t->limit);
    ivp_environment_add_controller(s->env, IVP_CP_ACTUATOR, ivp_controller_torque_actuator, &t->torque_act);
}

static float spin_of(const turntable_t *t)
{
    return t->bar->physical_core->rot_speed.k[1];
}

/* ── axle rigs ───────────────────────────────────────────────────────────── */

#define WHEEL_R    0.3
#define SUSP_REST  1.2
#define RIG_HALF   1.5

static void setup_rig(actuators_demo_t *s, demo_world_t *w, rig_t *r)
{
    r->chassis = demo_create_box(w, NULL, RIG_HALF + 0.3, 0.15, 0.5, 10.0, NULL, r->x, -(WHEEL_R * 2.0 + SUSP_REST), 0.0);
    for (int i = 0; i < 2; i++) {
        const double side = i ? RIG_HALF : -RIG_HALF;
        r->wheel[i] = demo_create_ball(w, NULL, WHEEL_R, 1.0, NULL, r->x + side, -WHEEL_R - 0.01, 0.0);
        const ivp_vec3r_t on_body = ivp_vec3r((ivp_real_t)side, IVP_REAL(0.0), IVP_REAL(0.0)),
                          on_wheel = ivp_vec3r_zero();

        /* a spring alone is unstable sideways, like a pogo stick: the strut keeps the wheel under its anchor
         * (x and z fixed, y and the wheel's rotation free) */
        ivp_constraint_local_init(&r->strut[i], r->chassis, &on_body, r->wheel[i], &on_wheel);
        ivp_constraint_local_free_translation(&r->strut[i], 1);
        for (int k = 0; k < 3; k++) ivp_constraint_local_free_rotation(&r->strut[i], k);
        ivp_environment_announce_dependent_controller(s->env, IVP_CP_CONSTRAINTS, ivp_controller_constraint_local,
                                                      &r->strut[i], ivp_constraint_local_discovery);

        ivp_suspension_state_init(&r->susp[i]);
        ivp_suspension_state_setup(&r->susp[i], r->chassis, r->wheel[i], &on_body, &on_wheel, (ivp_real_t)s->susp_k,
                                   (ivp_real_t)susp_damp, (ivp_real_t)susp_damp, (ivp_real_t)SUSP_REST);
        ivp_suspension_set_max_body_force(&r->susp[i], IVP_REAL(1e5));
        ivp_environment_add_controller(s->env, IVP_CP_ACTUATOR, ivp_controller_suspension, &r->susp[i]);
    }
    if (r->stabilised) {
        const ivp_vec3r_t left = ivp_vec3r((ivp_real_t)-RIG_HALF, IVP_REAL(0.0), IVP_REAL(0.0)),
                          right = ivp_vec3r((ivp_real_t)RIG_HALF, IVP_REAL(0.0), IVP_REAL(0.0)),
                          wheel = ivp_vec3r_zero();
        ivp_stabilizer_state_init(&r->stab);
        ivp_stabilizer_state_setup(&r->stab, r->chassis, r->wheel[0], r->chassis, r->wheel[1], &left, &wheel, &right,
                                   &wheel, (ivp_real_t)s->stab_k);
        ivp_environment_add_controller(s->env, IVP_CP_ACTUATOR, ivp_controller_stabilizer, &r->stab);
    }
}

static float roll_of(const ivp_object_t *chassis)
{
    ivp_mat3r_t m;
    ivp_quat_to_mat3(&chassis->physical_core->q_world_f_core_last_psi, &m);
    return (float)atan2(m.rows[1].k[0], m.rows[0].k[0]);          /* x axis in the xy plane */
}

/* ── panel ───────────────────────────────────────────────────────────────── */

static void apply_turntable_a(void *ctx)
{
    actuators_demo_t *s = (actuators_demo_t *)ctx;
    ivp_torque_actuator_set_torque(&s->turn[0].torque_act, (ivp_real_t)s->torque_a);
    ivp_torque_actuator_set_max_rotation_speed(&s->turn[0].torque_act, (ivp_real_t)s->limit_a);
    ivp_environment_ensure_core_in_simulation(s->env, s->turn[0].bar);
}

static void apply_susp(void *ctx)
{
    actuators_demo_t *s = (actuators_demo_t *)ctx;
    for (int i = 0; i < N_RIGS; i++)
        for (int k = 0; k < 2; k++) ivp_suspension_set_constant(&s->rig[i].susp[k], (ivp_real_t)s->susp_k);
}

static void apply_stab(void *ctx)
{
    actuators_demo_t *s = (actuators_demo_t *)ctx;
    ivp_stabilizer_set_constant(&s->rig[1].stab, (ivp_real_t)s->stab_k);
    ivp_environment_ensure_core_in_simulation(s->env, s->rig[1].chassis);
}

static void actuators_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    actuators_demo_t *s = (actuators_demo_t *)app->user;
    demo_panel_t *p = &v->panel;
    demo_panel_add_float(p, "A torque", &s->torque_a, -8.0f, 8.0f, apply_turntable_a, s);
    demo_panel_add_float(p, "A limit", &s->limit_a, 0.0f, 12.0f, apply_turntable_a, s);
    demo_panel_add_float(p, "spring k", &s->susp_k, 100.0f, 1500.0f, apply_susp, s);
    demo_panel_add_float(p, "stabilizer", &s->stab_k, 0.0f, 350.0f, apply_stab, s);
}

/* ── per frame ───────────────────────────────────────────────────────────── */

static void actuators_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    actuators_demo_t *s = (actuators_demo_t *)app->user;
    (void)v;
    (void)frame;
    const double t = (double)ivp_environment_get_time(s->env);

    /* both rigs get the same kick on the right wheel */
    if (t > 1.0 && fmod(t - 1.0, KICK_PERIOD) < DEMO_PSI_TIME) {
        for (int i = 0; i < N_RIGS; i++) demo_async_add_speed_ws(s->env, s->rig[i].wheel[1], 0.0, -4.0, 0.0);
        s->kicks++;
    }
    for (int i = 0; i < N_TURN; i++) {
        const float w = fabsf(spin_of(&s->turn[i]));
        if (w > s->turn[i].peak) s->turn[i].peak = w;
    }
    for (int i = 0; i < N_RIGS; i++) {
        rig_t *r = &s->rig[i];
        r->roll = roll_of(r->chassis);
        if (t > 0.5 && fabsf(r->roll) > r->max_roll) r->max_roll = fabsf(r->roll);
    }
}

static void actuators_reset(demo_app_t *app)
{
    actuators_demo_t *s = (actuators_demo_t *)app->user;
    for (int i = 0; i < N_TURN; i++) s->turn[i].peak = 0.0f;
    for (int i = 0; i < N_RIGS; i++) s->rig[i].max_roll = 0.0f;
    apply_turntable_a(s);
}

/* ── drawing ─────────────────────────────────────────────────────────────── */

/* A ring round the axle with an arrowhead: the direction of the torque, bright when the bar turns at the limit. */
static void draw_torque_ring(demo_viewer_t *v, const turntable_t *t)
{
    const double tq = (double)t->torque_act.torque;
    if (fabs(tq) < 1e-6) return;
    const float w = fabsf(spin_of(t)), lim = (float)t->torque_act.max_rotation_speed;
    const float k = lim > 0.0f ? fminf(1.0f, w / lim) : 0.0f;
    const float col[3] = { 0.4f + 0.6f * k, 0.8f, 0.4f };
    const float r = 1.7f, sgn = tq > 0.0 ? 1.0f : -1.0f;
    float prev[3] = { (float)t->x + r, -3.0f, 0.0f };
    for (int i = 1; i <= 24; i++) {
        const float a = sgn * 4.7f * (float)i / 24.0f;
        const float p[3] = { (float)t->x + r * cosf(a), -3.0f, r * sinf(a) };
        if (i == 24) demo_arrow(v, prev, p, col, 0.35f);
        else demo_thick_line(v, prev, p, col, 0.06f);
        memcpy(prev, p, sizeof(prev));
    }
}

static void actuators_draw(demo_app_t *app, demo_viewer_t *v)
{
    actuators_demo_t *s = (actuators_demo_t *)app->user;
    const float c_ground[3] = { 0.50f, 0.53f, 0.57f }, c_bar[3] = { 0.95f, 0.65f, 0.30f },
                c_chassis[3] = { 0.40f, 0.78f, 0.95f }, c_chassis2[3] = { 0.55f, 0.90f, 0.50f },
                c_wheel[3] = { 0.30f, 0.30f, 0.34f };
    demo_draw_object(v, s->ground, c_ground);
    for (int i = 0; i < N_TURN; i++) {
        demo_draw_object(v, s->turn[i].bar, c_bar);
        const float a[3] = { (float)s->turn[i].x, -3.0f, 0.0f }, b[3] = { (float)s->turn[i].x, 0.0f, 0.0f };
        demo_line(v, a, b, DEMO_COL_STATIC);               /* the axle */
        draw_torque_ring(v, &s->turn[i]);
    }
    for (int i = 0; i < N_RIGS; i++) {
        rig_t *r = &s->rig[i];
        demo_draw_object(v, r->chassis, r->stabilised ? c_chassis2 : c_chassis);
        for (int k = 0; k < 2; k++) {
            demo_draw_object(v, r->wheel[k], c_wheel);
            /* the strut: blue when it is longer than its rest length, green at rest, red when squeezed */
            float cb[3], cw[3];
            demo_object_center(r->chassis, cb);
            demo_object_center(r->wheel[k], cw);
            cb[0] += k ? (float)RIG_HALF : (float)-RIG_HALF;
            const float len = cw[1] - cb[1], squeeze = (float)(SUSP_REST - len) / 0.3f;
            const float t = fminf(1.0f, fabsf(squeeze));
            const float col[3] = { squeeze > 0.0f ? 0.3f + 0.7f * t : 0.3f, 0.9f - 0.5f * t, squeeze < 0.0f ? 0.3f + 0.7f * t : 0.3f };
            demo_thick_line(v, cb, cw, col, 0.08f);
            /* the force on the chassis */
            const float f = (float)r->susp[k].current_body_force / 4000.0f;
            const float tip[3] = { cb[0], cb[1] + f, cb[2] };
            if (fabsf(f) > 0.05f) demo_arrow(v, cb, tip, DEMO_COL_VELOCITY, 0.15f);
        }
    }
    for (int i = 0; i < N_TURN; i++)
        demo_hud(v, DEMO_COL_INFO, "turntable %s  torque %+.1f  limit %.0f  w %+.2f rad/s  peak %.2f", s->turn[i].name,
                 s->turn[i].torque_act.torque, s->turn[i].torque_act.max_rotation_speed, (double)spin_of(&s->turn[i]),
                 (double)s->turn[i].peak);
    demo_hud(v, DEMO_COL_HELP, "plain rig roll %+.1f deg (max %.1f)    stabilizer rig roll %+.1f deg (max %.1f)",
             (double)s->rig[0].roll * 57.2958, (double)s->rig[0].max_roll * 57.2958, (double)s->rig[1].roll * 57.2958,
             (double)s->rig[1].max_roll * 57.2958);
}

static void actuators_stats(demo_app_t *app, char *buf, size_t size)
{
    actuators_demo_t *s = (actuators_demo_t *)app->user;
    snprintf(buf, size, "w %.2f %.2f %.2f, max roll %.1f / %.1f deg", (double)spin_of(&s->turn[0]),
             (double)spin_of(&s->turn[1]), (double)spin_of(&s->turn[2]), (double)s->rig[0].max_roll * 57.2958,
             (double)s->rig[1].max_roll * 57.2958);
}

static bool actuators_check(demo_app_t *app)
{
    actuators_demo_t *s = (actuators_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    const float wa = spin_of(&s->turn[0]), wb = spin_of(&s->turn[1]);
    ok &= demo_check(tag, wa > 5.0f && wa < 7.0f, "turntable A holds its speed limit (%.2f rad/s, limit %.0f)",
                     (double)wa, s->turn[0].limit);
    ok &= demo_check(tag, wb < -2.0f && wb > -4.0f, "turntable B turns the other way at its limit (%.2f rad/s)", (double)wb);
    ok &= demo_check(tag, s->turn[2].peak < 2.0f, "turntable C only gets a kick (peak %.2f rad/s)", (double)s->turn[2].peak);
    float p0[3], p1[3];
    demo_object_center(s->rig[0].chassis, p0);
    demo_object_center(s->rig[1].chassis, p1);
    ok &= demo_check(tag, p0[1] < -0.8f && p1[1] < -0.8f, "both chassis stand on their suspension (y %.2f, %.2f)", (double)p0[1],
                     (double)p1[1]);
    ok &= demo_check(tag, s->rig[0].max_roll > 0.01f, "the kick rolls the plain chassis (%.1f deg)",
                     (double)s->rig[0].max_roll * 57.2958);
    ok &= demo_check(tag, s->rig[1].max_roll > s->rig[0].max_roll + 0.01f,
                     "and the reference stabilizer makes it roll more (%.1f against %.1f deg)",
                     (double)s->rig[1].max_roll * 57.2958, (double)s->rig[0].max_roll * 57.2958);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Actuators -- Torque, Suspension, Stabilizer";
    app.cfg.tag         = "actuators";
    app.cfg.friction    = 0.6;
    app.cfg.elasticity  = 0.1;
    app.cfg.orbit_dist  = 16.0f;
    app.cfg.orbit_yaw   = 180.0f;
    app.cfg.orbit_pitch = -14.0f;
    app.cfg.target[0]   = 0.0f;
    app.cfg.target[1]   = -2.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    actuators_demo_t *s = (actuators_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;
    s->torque_a = 3.0f;
    s->limit_a = 6.0f;
    s->susp_k = 500.0f;
    s->stab_k = 250.0f;

    s->ground = demo_create_box(w, NULL, 16.0, 0.5, 4.0, 0.0, NULL, 0.0, 0.5, 0.0);
    const struct { const char *name; double x, torque, limit; } tt[N_TURN] = {
        { "A", -12.0, 3.0, 6.0 }, { "B", -8.0, -3.0, 3.0 }, { "C", -4.0, 3.0, 0.0 } };
    for (int i = 0; i < N_TURN; i++) {
        s->turn[i].name = tt[i].name;
        s->turn[i].x = tt[i].x;
        s->turn[i].torque = tt[i].torque;
        s->turn[i].limit = tt[i].limit;
        setup_turntable(s, w, &s->turn[i]);
    }
    s->rig[0].x = 3.0;
    s->rig[1].x = 11.0;
    s->rig[1].stabilised = true;
    for (int i = 0; i < N_RIGS; i++) setup_rig(s, w, &s->rig[i]);

    app.controls    = "F5 panel: turntable A, spring and stabilizer constants";
    app.viewer_init = actuators_viewer_init;
    app.update      = actuators_update;
    app.on_reset    = actuators_reset;
    app.draw        = actuators_draw;
    app.stats       = actuators_stats;
    app.check       = actuators_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
