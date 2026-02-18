/* example_freefall.c -- Balls bouncing under gravity
 *
 * Multiple balls of varying sizes, masses, and elasticities are dropped
 * from different heights onto a ground plane.  The HUD shows per-ball
 * velocity, bounce count (sign changes of vertical speed), and height.
 *
 * Demonstrates: ball surface manager, gravity, elasticity differences,
 *               position/speed readback, HUD text, velocity arrows.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/base/ivp_types.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_surman.h>

#include <stdio.h>

#define N_BALLS 5

typedef struct {
    float radius;
    float mass;
    float elasticity;
    float start_y;     /* negative = above ground in Y-down */
    float start_x;
    float col[3];
    const char *label;
} ball_desc_t;

static const ball_desc_t descs[N_BALLS] = {
    {0.6f,  1.0f, 0.90f, -12.0f, -5.0f, {0.2f, 0.8f, 1.0f},  "Rubber"},
    {0.8f,  2.0f, 0.60f, -10.0f, -2.0f, {1.0f, 0.5f, 0.1f},  "Plastic"},
    {1.0f,  5.0f, 0.30f, -15.0f,  1.0f, {0.6f, 0.6f, 0.7f},  "Steel"},
    {0.5f,  0.5f, 0.80f,  -8.0f,  4.0f, {0.3f, 1.0f, 0.3f},  "Tennis"},
    {1.2f, 10.0f, 0.10f, -18.0f,  7.0f, {0.5f, 0.3f, 0.2f},  "Clay"},
};

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Freefall -- Bouncing Balls", .vsync = true
    };
    ivp_renderer_t *renderer = ivp_renderer_create(&rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -5.0f;
    cam.orbit_distance  = 28.0f;
    cam.orbit_yaw       = 15.0f;
    cam.orbit_pitch     = -15.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    /* Enable CCD: fast-falling objects may tunnel through thin ground. */
    ecfg.ccd_enabled = true;
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "freefall");

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 20.0f, 0.5f, 12.0f);
    ivp_surman_polygon_t sm_ground;
    ivp_surman_polygon_init(&sm_ground, ground_surf);

    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.type = IVP_OBJECT_POLYGON;
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    ivp_environment_create_object(env, &gt, &sm_ground.base);

    /* ── Balls ──────────────────────────────────────────────────────────── */
    ivp_surman_ball_t sm_balls[N_BALLS];
    ivp_object_t *balls[N_BALLS];

    for (int i = 0; i < N_BALLS; i++) {
        const ball_desc_t *d = &descs[i];
        ivp_surman_ball_init(&sm_balls[i], d->radius);

        ivp_object_template_t bt;
        ivp_object_template_init(&bt);
        bt.type = IVP_OBJECT_BALL;
        bt.mass = (ivp_real_t)d->mass;
        set_sphere_inertia(&bt, bt.mass, (ivp_real_t)d->radius);
        bt.material.friction   = IVP_REAL(0.5);
        bt.material.elasticity = (ivp_real_t)d->elasticity;
        bt.position = ivp_vec3r((ivp_real_t)d->start_x, (ivp_real_t)d->start_y, IVP_REAL(0.0));

        balls[i] = ivp_environment_create_object(env, &bt, &sm_balls[i].base);
        /* Small initial downward velocity matching ~0.5m free-fall for natural drop */
        balls[i]->core.speed = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    }

    /* ── Bounce tracking ────────────────────────────────────────────────── */
    int bounces[N_BALLS] = {0};
    float prev_vy[N_BALLS] = {0};
    float peak_down_vy[N_BALLS] = {0};

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_ground[] = {0.35f, 0.35f, 0.40f};
    const float c_grid[]   = {0.12f, 0.12f, 0.18f};
    const float c_hud[]    = {0.80f, 0.80f, 0.80f};
    const float c_vel[]    = {0.20f, 1.0f,  0.40f};
    const float c_drag[]   = {1.0f, 0.85f, 0.25f};
    const float c_focus[]  = {1.0f, 0.65f, 0.25f};

    ivp_example_dragger_t dragger;
    ivp_example_dragger_init(&dragger);
    ivp_example_dragger_attach(&dragger, env, 100);

    ivp_example_focus_controller_t focus;
    ivp_example_focus_init(&focus);

    ivp_example_input_tracker_t input;
    ivp_example_input_tracker_init(&input);

    ivp_camera_defaults_t cam_defaults;
    ivp_camera_store_defaults(&cam_defaults, &cam);

    ivp_scene_snapshot_t scene_snap;
    ivp_scene_snapshot_take(&scene_snap, env);

    sim_loop_t sl;
    sim_loop_init(&sl, renderer);

    while (ivp_renderer_begin_frame(renderer, &cam)) {
        float frame_dt = sim_loop_begin(&sl, renderer);

        ivp_example_input_tracker_update(&input);
        if (input.scene_reset_requested) {
            ivp_scene_snapshot_restore(&scene_snap);
            for (int i = 0; i < N_BALLS; i++) {
                bounces[i] = 0; prev_vy[i] = 0; peak_down_vy[i] = 0;
            }
            sl.sim_accum = 0.0f;
        }
        if (input.view_reset_requested) {
            ivp_camera_restore_defaults(&cam, &cam_defaults);
        }

        int win_w = 0, win_h = 0;
        ivp_renderer_get_size(renderer, &win_w, &win_h);
        ivp_example_dragger_update(&dragger, &cam, env, win_w, win_h, frame_dt);
        ivp_example_focus_update(&focus, &cam, env, frame_dt);

        sim_loop_step(&sl, env, frame_dt);

        /* Grid & ground */
        ivp_draw_ground_grid(renderer, 20.0f, 2.0f, c_grid);
        {
            float gp[3] = {0, 0, 0}, gs[3] = {20.0f, 0.5f, 12.0f};
            ivp_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus, c_drag, c_focus);

        /* Axes */
        {
            float org[3] = {0, 0, 0};
            ivp_draw_axes(renderer, org, NULL, 2.0f);
        }

        /* Balls */
        float max_ball_speed = 0.0f;
        float sum_height = 0.0f;
        int settled_count = 0;
        for (int i = 0; i < N_BALLS; i++) {
            const ball_desc_t *d = &descs[i];
            ivp_vec3r_t p, v;
            ivp_object_get_position(balls[i], &p);
            ivp_object_get_speed(balls[i], &v);

            float fp[3];
            vec3r_to_float3(&p, fp);
            ivp_draw_wire_sphere(renderer, fp, d->radius, d->col);

            /* Bounce detection: sign change of vertical speed (Y component) */
            float cur_vy = (float)v.k[1];
            if (cur_vy > peak_down_vy[i]) peak_down_vy[i] = cur_vy;
            /* Lower threshold catches low-energy bounces near settling.
             * prev_vy > 0.05 ensures ball was moving down; cur_vy < -0.05 = now going up. */
            if (prev_vy[i] > 0.05f && cur_vy < -0.05f) {
                bounces[i]++;
            }
            prev_vy[i] = cur_vy;

            /* Velocity arrow */
            float speed = vec3r_speed(&v);
            if (speed > max_ball_speed) max_ball_speed = speed;
            float height = -(float)p.k[1];
            sum_height += height;
            if (speed < 0.35f && (float)p.k[1] > -0.3f) settled_count++;
            if (speed > 0.5f) {
                float sc = 0.3f;
                float to[3] = {fp[0]+(float)v.k[0]*sc,
                               fp[1]+(float)v.k[1]*sc,
                               fp[2]+(float)v.k[2]*sc};
                ivp_draw_arrow(renderer, fp, to, c_vel, 0.2f);
            }
        }

        if (ivp_example_inspector_tick(&inspector)) {
            int total_bounces = 0;
            int max_bounce_idx = 0;
            float max_impact_v = 0.0f;
            for (int i = 0; i < N_BALLS; i++) total_bounces += bounces[i];
            for (int i = 0; i < N_BALLS; i++) {
                if (bounces[i] > bounces[max_bounce_idx]) max_bounce_idx = i;
                if (peak_down_vy[i] > max_impact_v) max_impact_v = peak_down_vy[i];
            }
            ivp_example_inspector_log(&inspector,
                "max_speed=%.3f avg_height=%.3f settled=%d/%d total_bounces=%d top_bouncer=%s(%d) max_down_vy=%.3f",
                max_ball_speed,
                sum_height / (float)N_BALLS,
                settled_count, N_BALLS,
                total_bounces,
                descs[max_bounce_idx].label,
                bounces[max_bounce_idx],
                max_impact_v);
        }

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  Balls: %d  Time: %.1fs  CCD: ON",
                     sl.fps_display, N_BALLS, (float)sl.t_prev);
            ivp_draw_text_2d(renderer, 10.0f, 10.0f, buf, c_hud);

            for (int i = 0; i < N_BALLS; i++) {
                ivp_vec3r_t p, v;
                ivp_object_get_position(balls[i], &p);
                ivp_object_get_speed(balls[i], &v);
                float speed = vec3r_speed(&v);
                float height = -(float)p.k[1]; /* negate for display (positive = up) */
                snprintf(buf, sizeof(buf), "%s (e=%.2f m=%.0f): h=%.1f v=%.1f bounces=%d",
                         descs[i].label, descs[i].elasticity, descs[i].mass,
                         height, speed, bounces[i]);
                ivp_draw_text_2d(renderer, 10.0f, 30.0f + 18.0f * (float)i, buf, descs[i].col);
            }

            ivp_example_draw_interaction_hud(renderer,
                10.0f, 30.0f + 18.0f * N_BALLS + 10.0f, c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
