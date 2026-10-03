/* example_motion_controller.c -- Direct motion control demo
 *
 * A flat table with walls holds several objects at rest.  One "cue"
 * sphere is driven by a motion controller.  Press 1/2/3 to switch
 * between VELOCITY, FORCE, and IMPULSE modes.  Arrow keys move the
 * cue sphere in the chosen mode.
 *
 * Demonstrates: ivp_controller_motion, velocity/force/impulse modes,
 *               keyboard-driven physics, mode switching.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_controller_motion.h>

#define N_TABLE_OBJ 8
#define VEL_SPEED     8.0f    /* VELOCITY mode: m/s */
#define FORCE_N      30.0f    /* FORCE mode: N */
#define IMPULSE_NS    5.0f    /* IMPULSE mode: N*s per frame the key is held */
#define CUE_MASS      2.0f

/* Wrapper: only apply motion to the cue object */
typedef struct {
    ivp_motion_state_t state;
    ivp_object_t      *target;
} cue_motion_t;

static void cue_motion_controller(const ivp_event_sim_t *event_sim, ivp_object_t *obj, void *ctx)
{
    cue_motion_t *cm = ctx;
    if (obj == cm->target) ivp_controller_motion(event_sim, obj, &cm->state);
}

typedef struct motion_demo {
    ivp_object_t *ground, *walls[4], *cue;
    ivp_object_t *tbl_objs[N_TABLE_OBJ];
    cue_motion_t  cue_motion;
    int           current_mode;   /* 0=VELOCITY, 1=FORCE, 2=IMPULSE */
    float         input[2];       /* last arrow input (x, z) */
    /* headless measurements */
    float         vel_hold_vx;    /* x speed at the end of the velocity drive */
    float         vel_stop_speed; /* speed 0.25 s after releasing it */
    float         force_v0, force_v1;   /* -x speed when the force drive starts / after 0.5 s */
    float         force_coast;          /* speed 0.25 s after releasing it */
    float         imp_v0, imp_v1;       /* z speed before / after one impulse */
    float         max_table_speed;      /* fastest table object (the cue hit it) */
} motion_demo_t;

static const char *mode_names[] = {"VELOCITY", "FORCE", "IMPULSE"};
static const float palette[][3] = {
    {1.0f, 0.4f, 0.2f}, {0.3f, 1.0f, 0.3f}, {0.3f, 0.5f, 1.0f},
    {1.0f, 1.0f, 0.2f}, {1.0f, 0.2f, 0.8f}, {0.2f, 1.0f, 0.9f},
    {0.9f, 0.6f, 0.1f}, {0.6f, 0.3f, 1.0f}
};

/* Apply the arrow input in the current mode.  A resting cue is frozen (not
 * simulated, no controller reaches it): wake it whenever there is input. */
static void apply_input(demo_app_t *app, float mx, float mz)
{
    motion_demo_t *s = (motion_demo_t *)app->user;
    s->input[0] = mx;
    s->input[1] = mz;
    if (mx != 0.0f || mz != 0.0f) ivp_environment_ensure_core_in_simulation(app->world.env, s->cue);
    switch (s->current_mode) {
    case 0: { /* VELOCITY */
        ivp_vec3r_t v = ivp_vec3r((ivp_real_t)(mx * VEL_SPEED), IVP_REAL(0.0), (ivp_real_t)(mz * VEL_SPEED));
        ivp_motion_state_set_velocity(&s->cue_motion.state, &v);
        break;
    }
    case 1: { /* FORCE */
        ivp_vec3r_t f = ivp_vec3r((ivp_real_t)(mx * FORCE_N), IVP_REAL(0.0), (ivp_real_t)(mz * FORCE_N));
        ivp_motion_state_set_force(&s->cue_motion.state, &f);
        break;
    }
    case 2: { /* IMPULSE */
        if (mx != 0.0f || mz != 0.0f) {
            ivp_vec3r_t imp = ivp_vec3r((ivp_real_t)(mx * IMPULSE_NS), IVP_REAL(0.0), (ivp_real_t)(mz * IMPULSE_NS));
            ivp_motion_state_set_impulse(&s->cue_motion.state, &imp);
        }
        break;
    }
    }
}

static float cue_v(const motion_demo_t *s, int k)
{
    return (float)s->cue->physical_core->speed.k[k];
}

static void motion_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    motion_demo_t *s = (motion_demo_t *)app->user;
    for (int i = 0; i < N_TABLE_OBJ; i++) {
        float sp = demo_object_speed(s->tbl_objs[i]);
        if (sp > s->max_table_speed) s->max_table_speed = sp;
    }
    if (v) {
        if (demo_key_just_pressed(v, SDL_SCANCODE_1)) s->current_mode = 0;
        if (demo_key_just_pressed(v, SDL_SCANCODE_2)) s->current_mode = 1;
        if (demo_key_just_pressed(v, SDL_SCANCODE_3)) s->current_mode = 2;
        const bool *keys = SDL_GetKeyboardState(NULL);
        float mx = 0.0f, mz = 0.0f;
        if (keys[SDL_SCANCODE_LEFT])  mx -= 1.0f;
        if (keys[SDL_SCANCODE_RIGHT]) mx += 1.0f;
        if (keys[SDL_SCANCODE_UP])    mz -= 1.0f;
        if (keys[SDL_SCANCODE_DOWN])  mz += 1.0f;
        apply_input(app, mx, mz);
        return;
    }
    /* scripted: the cue rests (and freezes) for 2 s, then
     *   VELOCITY: right for 0.5 s, release (the target speed 0 stops it)
     *   FORCE:    left for 0.5 s, release (it rolls on)
     *   IMPULSE:  one frame down */
    float mx = 0.0f, mz = 0.0f;
    if (frame >= 120 && frame < 150) mx = 1.0f;
    if (frame == 150) s->vel_hold_vx = cue_v(s, 0);
    if (frame == 165) s->vel_stop_speed = demo_object_speed(s->cue);
    if (frame == 180) { s->current_mode = 1; s->force_v0 = -cue_v(s, 0); }
    if (frame >= 180 && frame < 210) mx = -1.0f;
    if (frame == 210) s->force_v1 = -cue_v(s, 0);
    if (frame == 225) s->force_coast = demo_object_speed(s->cue);
    if (frame == 300) { s->current_mode = 2; s->imp_v0 = cue_v(s, 2); }
    if (frame == 300) mz = 1.0f;
    if (frame == 302) s->imp_v1 = cue_v(s, 2);
    apply_input(app, mx, mz);
}

static void motion_draw(demo_app_t *app, demo_viewer_t *v)
{
    motion_demo_t *s = (motion_demo_t *)app->user;
    const float c_ground[] = {0.25f, 0.28f, 0.32f};
    const float c_wall[]   = {0.45f, 0.40f, 0.35f};
    const float c_cue[]    = {1.0f, 1.0f, 1.0f};
    const float c_vel[]    = {0.2f, 1.0f, 0.4f};
    const float c_mode[]   = {1.0f, 1.0f, 0.2f};
    demo_draw_object(v, s->ground, c_ground);
    for (int i = 0; i < 4; i++) demo_draw_object(v, s->walls[i], c_wall);
    for (int i = 0; i < N_TABLE_OBJ; i++) demo_draw_object(v, s->tbl_objs[i], palette[i % 8]);
    demo_draw_object(v, s->cue, c_cue);
    demo_draw_velocity(v, s->cue, c_vel);
    {
        float org[3] = {0, 0, 0};
        demo_axes(v, org, NULL, 1.5f);
    }
    demo_hud(v, c_mode, "Mode: %s  [1]VEL [2]FORCE [3]IMPULSE", mode_names[s->current_mode]);
    demo_hud(v, c_cue, "Arrow keys: move cue    White sphere = cue");
    demo_hud_object(v, "Cue", s->cue);
}

static void motion_stats(demo_app_t *app, char *buf, size_t size)
{
    motion_demo_t *s = (motion_demo_t *)app->user;
    float p[3];
    demo_object_center(s->cue, p);
    snprintf(buf, size, "mode=%s input=(%.0f,%.0f) cue=(%.2f,%.2f,%.2f) v=(%.2f,%.2f,%.2f) table_max_speed=%.2f%s",
             mode_names[s->current_mode], (double)s->input[0], (double)s->input[1],
             (double)p[0], (double)p[1], (double)p[2],
             (double)cue_v(s, 0), (double)cue_v(s, 1), (double)cue_v(s, 2), (double)s->max_table_speed,
             ivp_object_is_frozen(s->cue) ? " frozen" : "");
}

static bool motion_check(demo_app_t *app)
{
    motion_demo_t *s = (motion_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, fabsf(s->vel_hold_vx - VEL_SPEED) < 0.5f && s->vel_stop_speed < 0.5f,
                     "VELOCITY: cue driven at %.2f m/s (target %.0f), %.2f m/s 0.25 s after release",
                     (double)s->vel_hold_vx, (double)VEL_SPEED, (double)s->vel_stop_speed);
    float acc = (s->force_v1 - s->force_v0) / 0.5f;
    ok &= demo_check(tag, fabsf(acc - FORCE_N / CUE_MASS) < 0.2f * FORCE_N / CUE_MASS && s->force_coast > 0.5f * s->force_v1,
                     "FORCE: cue accelerated at %.1f m/s^2 (F/m = %.1f), rolls on at %.2f m/s after release",
                     (double)acc, (double)(FORCE_N / CUE_MASS), (double)s->force_coast);
    float dv = s->imp_v1 - s->imp_v0;
    ok &= demo_check(tag, fabsf(dv - IMPULSE_NS / CUE_MASS) < 0.3f,
                     "IMPULSE: one %.0f N*s impulse changed the cue speed by %.2f m/s (J/m = %.2f)",
                     (double)IMPULSE_NS, (double)dv, (double)(IMPULSE_NS / CUE_MASS));
    float p[3];
    demo_object_center(s->cue, p);
    ok &= demo_check(tag, fabsf(p[0]) < 10.0f && fabsf(p[2]) < 8.0f && p[1] < 0.0f,
                     "cue stays on the table (%.2f, %.2f, %.2f)", (double)p[0], (double)p[1], (double)p[2]);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Motion Controller -- Velocity/Force/Impulse";
    app.cfg.tag          = "motion_controller";
    app.cfg.orbit_dist   = 25.0f;
    app.cfg.orbit_yaw    = 0.0f;
    app.cfg.orbit_pitch  = -40.0f;
    app.cfg.target[1]    = -2.0f;
    app.cfg.grid_extent  = 10.0f;
    app.cfg.grid_spacing = 2.0f;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    motion_demo_t *s = (motion_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* ── Table (ground) ─────────────────────────────────────────────────── */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(w, &gt, 10.0, 0.5, 8.0);

    /* ── Table walls (4 static boxes) ───────────────────────────────────── */
    {
        static const double wall_pos[4][3] = {
            {-10.0, -1.0, 0.0}, {10.0, -1.0, 0.0}, {0.0, -1.0, -8.0}, {0.0, -1.0, 8.0}
        };
        for (int i = 0; i < 4; i++) {
            ivp_object_template_t wt;
            ivp_object_template_init(&wt);
            wt.mass = 0.0f;
            wt.physical_unmoveable = true;
            wt.position = ivp_vec3r((ivp_real_t)wall_pos[i][0], (ivp_real_t)wall_pos[i][1], (ivp_real_t)wall_pos[i][2]);
            s->walls[i] = i < 2 ? demo_create_box_templ(w, &wt, 0.3, 1.0, 8.0)
                                : demo_create_box_templ(w, &wt, 10.0, 1.0, 0.3);
        }
    }

    /* 4 cubes — low friction simulates pool-table felt, moderate elasticity for bouncing */
    for (int i = 0; i < 4; i++) {
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.mass = IVP_REAL(1.5);
        set_box_inertia(&t, IVP_REAL(1.5), IVP_REAL(0.6), IVP_REAL(0.6), IVP_REAL(0.6));
        t.material.friction   = IVP_REAL(0.15);
        t.material.elasticity = IVP_REAL(0.4);
        t.position = ivp_vec3r(IVP_REAL(-3.0) + IVP_REAL(2.0) * (ivp_real_t)i, IVP_REAL(-1.1), IVP_REAL(3.0));
        s->tbl_objs[i] = demo_create_box_templ(w, &t, 0.6, 0.6, 0.6);
    }

    /* 4 spheres — pool-ball style: low friction, high elasticity */
    for (int i = 0; i < 4; i++) {
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.mass = IVP_REAL(1.0);
        set_sphere_inertia(&t, IVP_REAL(1.0), IVP_REAL(0.5));
        t.material.friction   = IVP_REAL(0.1);
        t.material.elasticity = IVP_REAL(0.85);
        t.position = ivp_vec3r(IVP_REAL(-3.0) + IVP_REAL(2.0) * (ivp_real_t)i, IVP_REAL(-1.0), IVP_REAL(-3.0));
        s->tbl_objs[4 + i] = demo_create_ball_templ(w, &t, 0.5);
    }

    /* ── Cue sphere ─────────────────────────────────────────────────────── */
    ivp_object_template_t cue_t;
    ivp_object_template_init(&cue_t);
    cue_t.mass = (ivp_real_t)CUE_MASS;
    set_sphere_inertia(&cue_t, cue_t.mass, IVP_REAL(0.5));
    cue_t.material.friction   = IVP_REAL(0.1);   /* smooth cue ball */
    cue_t.material.elasticity = IVP_REAL(0.9);   /* near-elastic collisions */
    cue_t.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    s->cue = demo_create_ball_templ(w, &cue_t, 0.5);

    /* ── Motion controller for cue ──────────────────────────────────────── *
     * only the velocity / force / impulse modes: a new state drives toward
     * the origin like the reference IVP_Controller_Motion */
    ivp_motion_state_init(&s->cue_motion.state);
    s->cue_motion.state.linear_mode = IVP_MOTION_NONE;
    s->cue_motion.state.angular_mode = IVP_MOTION_NONE;
    s->cue_motion.state.max_speed = IVP_REAL(12.0);
    s->cue_motion.target = s->cue;
    ivp_environment_add_controller(w->env, 100, cue_motion_controller, &s->cue_motion);

    app.controls = "1/2/3: velocity / force / impulse mode   Arrow keys: move the cue";
    app.update   = motion_update;
    app.draw     = motion_draw;
    app.stats    = motion_stats;
    app.check    = motion_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
