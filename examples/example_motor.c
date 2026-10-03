/* example_motor.c -- Rotation motor with blade smashing objects
 *
 * A motor spins a blade constrained by a hinge to a pedestal.
 * Small cubes are placed on a ground plane where the blade can hit
 * them and send them flying.  HUD shows angular speed.
 *
 * The pedestal stands on the ground and the blade turns around it at the
 * height of the cubes.  (The pedestal and the blade used to float 4.3 m
 * above the ground and the cubes, released at blade height, just fell to
 * the ground below the blade; two of them started inside the blade.)
 * The rotation motor follows the reference IVP_Actuator_Rot_Mot: the
 * torque is power / (omega * I_axis) (ivp_actuator.cxx:650), i.e. the
 * delivered power is power / I_axis.  The blade's I_axis is 6.46 kg m^2,
 * so power 20 delivers ~3 W (power 3 only ~0.5 W: the blade crawled and
 * merely pushed the cubes along).
 *
 * Demonstrates: ivp_actuator_rot_mot, ivp_constraint (hinge),
 *               power / torque / max speed, collision interaction,
 *               angular velocity, HUD.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_actuator_rot_mot.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/math/ivp_transform.h>

#define N_TARGETS 8
#define MAX_SPEED 1.6f        /* rad/s */
#define TARGET_RADIUS 3.5f

static const char *MOTOR_INTERNAL_GROUP = "motor_internal";

/* Geometry: the pedestal stands on the ground (top of the ground box at
 * y = -0.5), the blade turns around it at the height of the target cubes. */
#define PEDESTAL_Y   (-1.3)
#define HINGE_OFS_Y  (0.3)            /* hinge, relative to the pedestal centre */
#define BLADE_Y      (PEDESTAL_Y + HINGE_OFS_Y)
#define TARGET_Y     (-0.9)

typedef struct motor_demo {
    ivp_object_t          *ground, *pedestal, *blade;
    ivp_object_t          *targets[N_TARGETS];
    float                  target_start[N_TARGETS][3];
    ivp_constraint_state_t pivot;
    ivp_rot_mot_state_t    motor;
    float                  max_target_speed[N_TARGETS];
    float                  max_hinge_err;
} motor_demo_t;

static float hinge_error(const motor_demo_t *s)
{
    ivp_vec3r_t a_ws, b_ws;
    ivp_vec3r_t anchor_ped_os = ivp_vec3r(IVP_REAL(0.0), (ivp_real_t)HINGE_OFS_Y, IVP_REAL(0.0));
    ivp_vec3r_t anchor_blade_os = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_transform_point(&a_ws, &s->pedestal->physical_core->m_world_f_core_last_psi, &anchor_ped_os);
    ivp_transform_point(&b_ws, &s->blade->physical_core->m_world_f_core_last_psi, &anchor_blade_os);
    float dx = (float)(b_ws.k[0] - a_ws.k[0]);
    float dy = (float)(b_ws.k[1] - a_ws.k[1]);
    float dz = (float)(b_ws.k[2] - a_ws.k[2]);
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static float blade_omega(const motor_demo_t *s, float *omega_y)
{
    const ivp_vec3f_t *w = &s->blade->physical_core->rot_speed;
    if (omega_y) *omega_y = fabsf(w->k[1]);
    return sqrtf(w->k[0] * w->k[0] + w->k[1] * w->k[1] + w->k[2] * w->k[2]);
}

static void motor_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    (void)app;
    v->dragger.settings.max_force = 60.0f;
}

static void motor_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    motor_demo_t *s = (motor_demo_t *)app->user;
    (void)v;
    (void)frame;
    for (int i = 0; i < N_TARGETS; i++) {
        float sp = demo_object_speed(s->targets[i]);
        if (sp > s->max_target_speed[i]) s->max_target_speed[i] = sp;
    }
    float err = hinge_error(s);
    if (err > s->max_hinge_err) s->max_hinge_err = err;
}

static void motor_reset(demo_app_t *app)
{
    motor_demo_t *s = (motor_demo_t *)app->user;
    for (int i = 0; i < N_TARGETS; i++) s->max_target_speed[i] = 0.0f;
    s->max_hinge_err = 0.0f;
}

static void motor_draw(demo_app_t *app, demo_viewer_t *v)
{
    motor_demo_t *s = (motor_demo_t *)app->user;
    const float c_ped[]    = {0.40f, 0.40f, 0.45f};
    const float c_blade[]  = {0.20f, 0.70f, 1.0f };
    const float c_target[] = {1.0f,  0.45f, 0.15f};
    const float c_ground[] = {0.30f, 0.30f, 0.35f};
    const float c_vel[]    = {0.20f, 1.0f,  0.40f};

    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->pedestal, c_ped);
    demo_draw_object(v, s->blade, c_blade);
    {
        /* local frame of the blade */
        float pos[3], rot[9];
        demo_object_pose(s->blade, pos, rot);
        demo_axes(v, pos, rot, 1.5f);
    }
    for (int i = 0; i < N_TARGETS; i++) {
        demo_draw_object(v, s->targets[i], c_target);
        if (demo_object_speed(s->targets[i]) > 1.0f) demo_draw_velocity(v, s->targets[i], c_vel);
    }
    {
        float org[3] = {0, 0, 0};
        demo_axes(v, org, NULL, 2.0f);
    }

    float omega = blade_omega(s, NULL);
    demo_hud(v, DEMO_COL_INFO, "Targets: %d", N_TARGETS);
    demo_hud(v, c_blade, "Blade: %.2f rad/s  (%.0f RPM)  motor: power %.0f, max torque %.0f N*m, max %.1f rad/s",
             (double)omega, (double)(omega * 60.0f / (2.0f * 3.14159f)), (double)s->motor.power,
             (double)s->motor.max_torque, (double)MAX_SPEED);
    demo_hud(v, DEMO_COL_INFO, "hinge anchor error %.3f m", (double)hinge_error(s));
}

static int targets_hit(const motor_demo_t *s)
{
    int n = 0;
    for (int i = 0; i < N_TARGETS; i++) {
        float p[3];
        demo_object_center(s->targets[i], p);
        if (demo_distance(p, s->target_start[i]) > 1.0f) n++;
    }
    return n;
}

static void motor_stats(demo_app_t *app, char *buf, size_t size)
{
    motor_demo_t *s = (motor_demo_t *)app->user;
    float omega_y;
    float omega = blade_omega(s, &omega_y);
    float max_ts = 0.0f;
    for (int i = 0; i < N_TARGETS; i++)
        if (demo_object_speed(s->targets[i]) > max_ts) max_ts = demo_object_speed(s->targets[i]);
    float bp[3];
    demo_object_center(s->blade, bp);
    snprintf(buf, size, "omega=%.3f omega_y=%.3f motor_speed=%.3f target_max=%.2f hit=%d/%d hinge_err=%.4f blade=(%.2f,%.2f,%.2f)",
             (double)omega, (double)omega_y, (double)s->motor.current_speed, (double)max_ts,
             targets_hit(s), N_TARGETS, (double)hinge_error(s), (double)bp[0], (double)bp[1], (double)bp[2]);
}

static bool motor_check(demo_app_t *app)
{
    motor_demo_t *s = (motor_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    float omega_y;
    float omega = blade_omega(s, &omega_y);
    ok &= demo_check(tag, fabsf(omega_y - MAX_SPEED) < 0.05f * MAX_SPEED && omega - omega_y < 0.01f,
                     "motor spins the blade about the hinge axis at its max speed (%.3f rad/s, limit %.1f)",
                     (double)omega_y, (double)MAX_SPEED);
    ok &= demo_check(tag, s->max_hinge_err < 0.05f, "hinge holds the blade (max anchor error %.4f m)",
                     (double)s->max_hinge_err);
    int n_fast = 0;
    float vmax = 0.0f;
    for (int i = 0; i < N_TARGETS; i++) {
        if (s->max_target_speed[i] > 1.5f) n_fast++;
        if (s->max_target_speed[i] > vmax) vmax = s->max_target_speed[i];
    }
    int hit = targets_hit(s);
    ok &= demo_check(tag, hit >= N_TARGETS - 2 && n_fast >= N_TARGETS / 2,
                     "blade knocked the targets away: %d/%d moved > 1 m, %d hit faster than 1.5 m/s (max %.2f m/s)",
                     hit, N_TARGETS, n_fast, (double)vmax);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Motor -- Spinning Blade";
    app.cfg.tag          = "motor";
    app.cfg.orbit_dist   = 20.0f;
    app.cfg.orbit_pitch  = -25.0f;
    app.cfg.orbit_yaw    = 30.0f;
    app.cfg.target[1]    = -1.5f;
    app.cfg.grid_extent  = 15.0f;
    app.cfg.grid_spacing = 2.0f;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    motor_demo_t *s = (motor_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_object_template_t grt;
    ivp_object_template_init(&grt);
    grt.mass = 0.0f;
    grt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(w, &grt, 15.0, 0.5, 15.0);

    /* ── Pedestal (static post, the blade's axle) ───────────────────────── */
    ivp_object_template_t pt;
    ivp_object_template_init(&pt);
    pt.mass = 0.0f;
    pt.physical_unmoveable = true;
    pt.no_collision_group = MOTOR_INTERNAL_GROUP;
    pt.position = ivp_vec3r(IVP_REAL(0.0), (ivp_real_t)PEDESTAL_Y, IVP_REAL(0.0));
    s->pedestal = demo_create_box_templ(w, &pt, 0.15, 0.8, 0.15);

    /* ── Blade (dynamic, the spinning part) ─────────────────────────────── */
    ivp_object_template_t bt;
    ivp_object_template_init(&bt);
    const ivp_real_t blade_mass = IVP_REAL(1.2);
    bt.mass = blade_mass;
    bt.no_collision_group = MOTOR_INTERNAL_GROUP;
    ivp_vec3r_set(&bt.rot_inertia,
        blade_mass * IVP_REAL(0.2725/3.0),
        blade_mass * IVP_REAL(16.16/3.0),
        blade_mass * IVP_REAL(16.0625/3.0));
    bt.rot_inertia_is_factor = false; /* absolute inertia */
    bt.material.friction   = IVP_REAL(0.5);
    bt.material.elasticity = IVP_REAL(0.35);
    bt.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&bt.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    bt.position = ivp_vec3r(IVP_REAL(0.0), (ivp_real_t)BLADE_Y, IVP_REAL(0.0));
    s->blade = demo_create_box_templ(w, &bt, 4.0, 0.25, 0.4);
    s->blade->physical_core->rot_speed.k[1] = 0.2f;

    /* ── Hinge constraint ───────────────────────────────────────────────── */
    {
        ivp_vec3r_t anchor_ped = ivp_vec3r(IVP_REAL(0.0), (ivp_real_t)HINGE_OFS_Y, IVP_REAL(0.0));
        ivp_vec3r_t anchor_blade = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t axis_ped = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        ivp_vec3r_t axis_blade = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        ivp_constraint_init_hinge(&s->pivot, s->pedestal, &anchor_ped, &axis_ped,
                                  s->blade, &anchor_blade, &axis_blade);
        s->pivot.solver_iterations = 28;
        s->pivot.tau = IVP_REAL(0.25);
        s->pivot.damping = IVP_REAL(0.95);
    }
    ivp_environment_add_controller(w->env, 50, ivp_controller_constraint, &s->pivot);

    /* ── Rotation motor ─────────────────────────────────────────────────── */
    ivp_rot_mot_state_init(&s->motor);
    {
        ivp_vec3r_t a0 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t a1 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        ivp_rot_mot_state_setup(&s->motor, s->blade, &a0, s->blade, &a1,
            IVP_REAL(20.0),   /* power */
            IVP_REAL(4.0),    /* max torque */
            (ivp_real_t)MAX_SPEED);
    }
    ivp_environment_add_controller(w->env, 100, ivp_controller_rot_mot, &s->motor);

    /* ── Target cubes around the blade (on the ground) ──────────────────── */
    for (int i = 0; i < N_TARGETS; i++) {
        ivp_object_template_t tt;
        ivp_object_template_init(&tt);
        tt.mass = IVP_REAL(0.06);
        set_box_inertia(&tt, IVP_REAL(0.06), IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
        tt.material.friction   = IVP_REAL(0.5);
        tt.material.elasticity = IVP_REAL(0.2);
        /* circle at blade height, between the blade's start directions */
        float angle = ((float)i + 0.5f) * 6.2832f / (float)N_TARGETS;
        tt.position = ivp_vec3r((ivp_real_t)(cosf(angle) * TARGET_RADIUS), (ivp_real_t)TARGET_Y,
                                (ivp_real_t)(sinf(angle) * TARGET_RADIUS));
        s->targets[i] = demo_create_box_templ(w, &tt, 0.4, 0.4, 0.4);
        demo_object_center(s->targets[i], s->target_start[i]);
    }

    app.viewer_init = motor_viewer_init;
    app.update      = motor_update;
    app.on_reset    = motor_reset;
    app.draw        = motor_draw;
    app.stats       = motor_stats;
    app.check       = motor_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
