/* example_hinge.c -- Two cubes connected by a hinge
 *
 * Port of ivp_examples/quickstart/ive_quickstart_hinge.cxx
 * (IVP manual section 5.10, Connecting two objects with a hinge).
 *
 * A static reference cube floats 5 m above the ground; a dynamic cube 2 m
 * to its side is attached with a hinge whose pivot is the centre of the
 * reference cube and whose free axis is the reference cube's Z axis.  The
 * attached cube swings like a pendulum in the XY plane.
 *
 * Demonstrates: the reference template path for constraints:
 *               ivp_constraint_template_set_constraint_Ros(..., 3, 2, ...)
 *               == IVP_Template_Constraint::set_hinge_Ros, then
 *               ivp_constraint_init_from_template + ivp_controller_constraint.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_constraint.h>

static const double CUBE_HS = 0.4;

typedef struct hinge_demo {
    ivp_object_t          *ground;
    ivp_object_t          *cube1;   /* static reference */
    ivp_object_t          *cube2;   /* attached */
    ivp_constraint_state_t hinge;
    float                  min_angle, max_angle;  /* swing range (deg) */
    float                  max_radius_err;
    float                  max_z;
} hinge_demo_t;

/* Angle of cube2 around the hinge axis, 0 = start (+X), 90 = below. */
static float swing_angle_deg(const hinge_demo_t *s)
{
    float p1[3], p2[3];
    demo_object_center(s->cube1, p1);
    demo_object_center(s->cube2, p2);
    return atan2f(p2[1] - p1[1], p2[0] - p1[0]) * (180.0f / 3.14159265f);
}

static void hinge_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    hinge_demo_t *s = (hinge_demo_t *)app->user;
    (void)v;
    (void)frame;
    float a = swing_angle_deg(s);
    if (a < s->min_angle) s->min_angle = a;
    if (a > s->max_angle) s->max_angle = a;
    float err = fabsf(demo_object_distance(s->cube1, s->cube2) - 2.0f);
    if (err > s->max_radius_err) s->max_radius_err = err;
    float z = fabsf((float)s->cube2->physical_core->pos_world_f_core_last_psi.k[2]);
    if (z > s->max_z) s->max_z = z;
}

static void hinge_reset(demo_app_t *app)
{
    hinge_demo_t *s = (hinge_demo_t *)app->user;
    s->min_angle = s->max_angle = 0.0f;
    s->max_radius_err = s->max_z = 0.0f;
}

static void hinge_draw(demo_app_t *app, demo_viewer_t *v)
{
    hinge_demo_t *s = (hinge_demo_t *)app->user;
    demo_draw_object(v, s->ground, DEMO_COL_STATIC);
    demo_draw_object(v, s->cube1, DEMO_COL_STATIC);
    demo_draw_object(v, s->cube2, DEMO_COL_OBJECT_B);
    demo_draw_link(v, s->cube1, s->cube2, DEMO_COL_HIGHLIGHT);

    /* hinge axis at cube1 */
    float p1[3];
    demo_object_center(s->cube1, p1);
    float axis_a[3] = { p1[0], p1[1], p1[2] - 1.5f };
    float axis_b[3] = { p1[0], p1[1], p1[2] + 1.5f };
    demo_arrow(v, axis_a, axis_b, DEMO_COL_HIGHLIGHT, 0.15f);

    demo_hud_object(v, "Static cube", s->cube1);
    demo_hud_object(v, "Dynamic cube", s->cube2);
    demo_hud(v, DEMO_COL_INFO, "Hinge axis: Z (shown in yellow)   swing angle %.1f deg",
             (double)swing_angle_deg(s));
}

static void hinge_stats(demo_app_t *app, char *buf, size_t size)
{
    hinge_demo_t *s = (hinge_demo_t *)app->user;
    snprintf(buf, size, "angle=%.2f range=[%.2f, %.2f] radius_err_max=%.4f z_max=%.4f speed=%.3f",
             (double)swing_angle_deg(s), (double)s->min_angle, (double)s->max_angle,
             (double)s->max_radius_err, (double)s->max_z, (double)demo_object_speed(s->cube2));
}

static bool hinge_check(demo_app_t *app)
{
    hinge_demo_t *s = (hinge_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->max_angle > 150.0f,
                     "cube swung through the bottom to the far side (max angle %.1f deg)",
                     (double)s->max_angle);
    ok &= demo_check(tag, s->max_radius_err < 0.05f,
                     "pivot distance stays 2 m (max error %.4f m)", (double)s->max_radius_err);
    ok &= demo_check(tag, s->max_z < 0.05f,
                     "motion stays in the hinge plane (max |z| %.4f m)", (double)s->max_z);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title      = "IVP Quickstart: Hinge";
    app.cfg.tag        = "hinge";
    app.cfg.orbit_dist = 15.0f;
    app.cfg.target[1]  = -3.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    hinge_demo_t *s = (hinge_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;

    s->ground = demo_create_box(&app.world, NULL, 15.0, 0.5, 15.0, 0.0, NULL, 0.0, 0.5, 0.0);
    /* reference (static) cube and the attached (dynamic) cube, offset along X */
    s->cube1 = demo_create_box(&app.world, NULL, CUBE_HS, CUBE_HS, CUBE_HS, 0.0, NULL, 0.0, -5.0, 0.0);
    s->cube2 = demo_create_box(&app.world, NULL, CUBE_HS, CUBE_HS, CUBE_HS, 1.0, NULL, 2.0, -5.0, 0.0);

    /* Hinge: pivot at cube1's centre, free axis Z in cube1's object space
     * (set_hinge_Ros = set_constraint_Ros(R, anchor, axis, 3, 2, A, NULL)) */
    {
        ivp_vec3r_t hinge_coord = ivp_vec3r(0, 0, 0);
        ivp_vec3r_t hinge_axis = ivp_vec3r(0, 0, 1);
        ivp_constraint_template_t ct;
        ivp_constraint_template_init(&ct);
        ivp_constraint_template_set_constraint_Ros(&ct, s->cube1, &hinge_coord, &hinge_axis,
                                                   3, 2, s->cube2, NULL);
        ivp_constraint_init_from_template(&s->hinge, &ct);
        ivp_environment_add_controller(app.world.env, IVP_CP_CONSTRAINTS,
                                       ivp_controller_constraint, &s->hinge);
    }

    app.update   = hinge_update;
    app.on_reset = hinge_reset;
    app.draw     = hinge_draw;
    app.stats    = hinge_stats;
    app.check    = hinge_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
