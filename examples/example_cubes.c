/* example_cubes.c -- Cube stacking and collision demo
 *
 * A pyramid of cubes with varied sizes, masses, and materials stacked
 * on a ground platform.  Includes a tilted ramp for rolling, a sphere
 * for variety, and a HUD showing object count / FPS.
 *
 * Demonstrates: box surfaces, sphere surface, collisions, elasticity,
 *               friction, stacking, HUD text, coordinate axes.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/base/ivp_types.h>

#include <stdio.h>

/* ── Object tracking ────────────────────────────────────────────────────── */
#define MAX_DYN 24
typedef struct obj_vis {
    ivp_object_t *obj;
    float hs[3];       /* half-size for drawing */
    float col[3];      /* color */
    bool  is_sphere;   /* draw sphere instead of box */
    float radius;      /* sphere radius */
} obj_vis_t;

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Cubes -- Stacking Demo", .vsync = true
    };
    ivp_renderer_t *renderer = NULL;
    ivp_renderer_create(&renderer, &rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }
    ivp_renderer_set_clear_color(renderer, 0.26f, 0.30f, 0.36f, 1.0f);

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -3.0f;
    cam.orbit_distance  = 25.0f;
    cam.orbit_yaw       = 30.0f;
    cam.orbit_pitch     = -25.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "cubes");

    /* ── Ground platform ────────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 15.0f, 0.5f, 15.0f);
    ivp_surman_polygon_t sm_ground;
    ivp_surman_polygon_init(&sm_ground, ground_surf);

    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.type = IVP_OBJECT_POLYGON;
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    ivp_environment_create_object(env, &gt, &sm_ground.base);

    /* ── Ramp (tilted static box) ───────────────────────────────────────── */
    ivp_compact_surface_t *ramp_surf =
        ivp_build_box_surface(NULL, 4.0f, 0.2f, 2.0f);
    ivp_surman_polygon_t sm_ramp;
    ivp_surman_polygon_init(&sm_ramp, ramp_surf);

    ivp_object_template_t rt;
    ivp_object_template_init(&rt);
    rt.type = IVP_OBJECT_POLYGON;
    rt.mass = 0.0f;
    rt.physical_unmoveable = true;
    rt.position = ivp_vec3r(IVP_REAL(8.0), IVP_REAL(-1.5), IVP_REAL(0.0));
    /* Tilt ~15 degrees around Z axis */
    rt.orientation = ivp_quatr_make(IVP_REAL(0.0), IVP_REAL(0.0),
                                     IVP_REAL(0.13), IVP_REAL(0.99));
    ivp_quat_fast_normalize(&rt.orientation);
    ivp_object_t *ramp = ivp_environment_create_object(env, &rt, &sm_ramp.base);

    /* ── Dynamic objects ────────────────────────────────────────────────── */
    obj_vis_t dyn[MAX_DYN];
    int n_dyn = 0;

    /* Different cube surfaces for variety */
    ivp_compact_surface_t *cube1_surf = ivp_build_box_surface(NULL, 0.8f, 0.8f, 0.8f);
    ivp_compact_surface_t *cube2_surf = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    ivp_compact_surface_t *cube3_surf = ivp_build_box_surface(NULL, 1.2f, 0.6f, 0.6f);

    /* We need unique surman per object */
    ivp_surman_polygon_t sm_dyn[MAX_DYN];
    ivp_surman_ball_t    sm_ball_dyn[4];
    int ball_idx = 0;

    /* Colors palette */
    const float palette[][3] = {
        {1.0f, 0.4f, 0.2f}, {0.3f, 1.0f, 0.3f}, {0.3f, 0.5f, 1.0f},
        {1.0f, 1.0f, 0.2f}, {1.0f, 0.2f, 0.8f}, {0.2f, 1.0f, 0.9f},
        {0.9f, 0.6f, 0.1f}, {0.6f, 0.3f, 1.0f}
    };

    /* --- Pyramid base layer (3x3 grid) -------------------------------- */
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            ivp_surman_polygon_init(&sm_dyn[n_dyn], cube1_surf);

            ivp_object_template_t t;
            ivp_object_template_init(&t);
            t.type = IVP_OBJECT_POLYGON;
            t.mass = IVP_REAL(3.0);
            set_box_inertia(&t, IVP_REAL(3.0), IVP_REAL(0.8), IVP_REAL(0.8), IVP_REAL(0.8));
            t.material.friction   = IVP_REAL(0.9);
            t.material.elasticity = IVP_REAL(0.15);
            t.position = ivp_vec3r(
                IVP_REAL(-2.0) + IVP_REAL(2.0) * (ivp_real_t)col,
                IVP_REAL(-1.3),
                IVP_REAL(-2.0) + IVP_REAL(2.0) * (ivp_real_t)row);

            dyn[n_dyn].obj = ivp_environment_create_object(env, &t, &sm_dyn[n_dyn].base);
            dyn[n_dyn].hs[0] = dyn[n_dyn].hs[1] = dyn[n_dyn].hs[2] = 0.8f;
            dyn[n_dyn].col[0] = palette[(n_dyn) % 8][0];
            dyn[n_dyn].col[1] = palette[(n_dyn) % 8][1];
            dyn[n_dyn].col[2] = palette[(n_dyn) % 8][2];
            dyn[n_dyn].is_sphere = false;
            n_dyn++;
        }
    }

    /* --- Second layer (2x2, dropped from above) ----------------------- */
    for (int row = 0; row < 2; row++) {
        for (int col = 0; col < 2; col++) {
            ivp_surman_polygon_init(&sm_dyn[n_dyn], cube2_surf);

            ivp_object_template_t t;
            ivp_object_template_init(&t);
            t.type = IVP_OBJECT_POLYGON;
            t.mass = IVP_REAL(2.0);
            set_box_inertia(&t, IVP_REAL(2.0), IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(0.5));
            t.material.friction   = IVP_REAL(0.8);
            t.material.elasticity = IVP_REAL(0.2);
            t.position = ivp_vec3r(
                IVP_REAL(-1.0) + IVP_REAL(2.0) * (ivp_real_t)col,
                IVP_REAL(-4.0),
                IVP_REAL(-1.0) + IVP_REAL(2.0) * (ivp_real_t)row);
            /* Slight random-ish rotation */
            t.orientation = ivp_quatr_make(
                IVP_REAL(0.05) * (ivp_real_t)(col + 1),
                IVP_REAL(0.03) * (ivp_real_t)(row + 1),
                IVP_REAL(0.0), IVP_REAL(0.99));
            ivp_quat_fast_normalize(&t.orientation);

            dyn[n_dyn].obj = ivp_environment_create_object(env, &t, &sm_dyn[n_dyn].base);
            dyn[n_dyn].hs[0] = dyn[n_dyn].hs[1] = dyn[n_dyn].hs[2] = 0.5f;
            dyn[n_dyn].col[0] = palette[(n_dyn) % 8][0];
            dyn[n_dyn].col[1] = palette[(n_dyn) % 8][1];
            dyn[n_dyn].col[2] = palette[(n_dyn) % 8][2];
            dyn[n_dyn].is_sphere = false;
            n_dyn++;
        }
    }

    /* --- Top brick (elongated) ---------------------------------------- */
    {
        ivp_surman_polygon_init(&sm_dyn[n_dyn], cube3_surf);
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.type = IVP_OBJECT_POLYGON;
        t.mass = IVP_REAL(4.0);
        set_box_inertia(&t, IVP_REAL(4.0), IVP_REAL(1.2), IVP_REAL(0.6), IVP_REAL(0.6));
        t.material.friction   = IVP_REAL(0.7);
        t.material.elasticity = IVP_REAL(0.3);
        t.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-6.0), IVP_REAL(0.0));
        t.orientation = ivp_quatr_make(IVP_REAL(0.0), IVP_REAL(0.1), IVP_REAL(0.0), IVP_REAL(0.99));
        ivp_quat_fast_normalize(&t.orientation);

        dyn[n_dyn].obj = ivp_environment_create_object(env, &t, &sm_dyn[n_dyn].base);
        dyn[n_dyn].hs[0] = 1.2f; dyn[n_dyn].hs[1] = 0.6f; dyn[n_dyn].hs[2] = 0.6f;
        dyn[n_dyn].col[0] = 0.9f; dyn[n_dyn].col[1] = 0.9f; dyn[n_dyn].col[2] = 0.2f;
        dyn[n_dyn].is_sphere = false;
        n_dyn++;
    }

    /* --- Sphere on the ramp ------------------------------------------- */
    {
        ivp_surman_ball_init(&sm_ball_dyn[ball_idx], 0.7f);
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.type = IVP_OBJECT_BALL;
        t.mass = IVP_REAL(2.0);
        ivp_real_t I = IVP_REAL(0.4) * IVP_REAL(2.0) * IVP_REAL(0.49); /* 2/5 * m * r^2 */
        ivp_vec3r_set(&t.rot_inertia, I, I, I);
        t.material.friction   = IVP_REAL(0.4);
        t.material.elasticity = IVP_REAL(0.6);
        t.position = ivp_vec3r(IVP_REAL(9.0), IVP_REAL(-4.0), IVP_REAL(0.0));

        dyn[n_dyn].obj = ivp_environment_create_object(env, &t, &sm_ball_dyn[ball_idx].base);
        dyn[n_dyn].is_sphere = true;
        dyn[n_dyn].radius = 0.7f;
        dyn[n_dyn].col[0] = 0.2f; dyn[n_dyn].col[1] = 0.9f; dyn[n_dyn].col[2] = 1.0f;
        n_dyn++;
        ball_idx++;
    }

    /* --- Extra falling cubes (low elasticity to settle quickly) -------- */
    for (int i = 0; i < 3; i++) {
        ivp_surman_polygon_init(&sm_dyn[n_dyn], cube1_surf);
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.type = IVP_OBJECT_POLYGON;
        t.mass = IVP_REAL(1.0) + IVP_REAL(2.0) * (ivp_real_t)i;
        set_box_inertia(&t, t.mass, IVP_REAL(0.8), IVP_REAL(0.8), IVP_REAL(0.8));
        t.material.friction   = IVP_REAL(0.6);
        /* Cap elasticity at 0.15 for dropped cubes to avoid persistent bouncing */
        t.material.elasticity = IVP_REAL(0.15);
        t.speed_damp_factor   = IVP_REAL(0.03);
        t.position = ivp_vec3r(
            IVP_REAL(-4.0) + IVP_REAL(4.0) * (ivp_real_t)i,
            IVP_REAL(-8.0) - IVP_REAL(3.0) * (ivp_real_t)i,
            IVP_REAL(5.0));
        t.orientation = ivp_quatr_make(
            IVP_REAL(0.1) * (ivp_real_t)(i + 1), IVP_REAL(0.05), IVP_REAL(0.0), IVP_REAL(0.99));
        ivp_quat_fast_normalize(&t.orientation);

        dyn[n_dyn].obj = ivp_environment_create_object(env, &t, &sm_dyn[n_dyn].base);
        dyn[n_dyn].hs[0] = dyn[n_dyn].hs[1] = dyn[n_dyn].hs[2] = 0.8f;
        dyn[n_dyn].col[0] = palette[(n_dyn) % 8][0];
        dyn[n_dyn].col[1] = palette[(n_dyn) % 8][1];
        dyn[n_dyn].col[2] = palette[(n_dyn) % 8][2];
        dyn[n_dyn].is_sphere = false;
        n_dyn++;
    }

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_ground[] = {0.22f, 0.24f, 0.30f};
    const float c_ramp[]   = {0.62f, 0.52f, 0.34f};
    const float c_grid[]   = {0.38f, 0.43f, 0.52f};
    const float c_hud[]    = {0.95f, 0.97f, 1.0f};
    const float c_vel[]    = {0.2f, 1.0f, 0.4f};
    const float c_drag[]   = {1.0f, 0.85f, 0.25f};
    const float c_focus[]  = {1.0f, 0.65f, 0.25f};

    ivp_example_dragger_t dragger;
    ivp_example_dragger_init(&dragger);
    dragger.settings.max_force = 65.0f;
    dragger.settings.ramp_rate = 4.5f;
    ivp_example_dragger_attach(&dragger, env, 100);

    ivp_example_focus_controller_t focus;
    ivp_example_focus_init(&focus);
    focus.follow_lerp = 8.0f;

    ivp_example_input_tracker_t input;
    ivp_example_input_tracker_init(&input);

    ivp_camera_defaults_t cam_defaults;
    ivp_camera_store_defaults(&cam_defaults, &cam);

    ivp_scene_snapshot_t scene_snap;
    ivp_scene_snapshot_take(&scene_snap, env);

    /* ── Main loop ──────────────────────────────────────────────────────── */
    sim_loop_t sl;
    sim_loop_init(&sl, renderer);

    while (ivp_renderer_begin_frame(renderer, &cam)) {
        float frame_dt = sim_loop_begin(&sl, renderer);

        ivp_example_input_tracker_update(&input);
        if (input.scene_reset_requested) {
            ivp_scene_snapshot_restore(&scene_snap);
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

        /* Ground */
        ivp_render_draw_ground_grid(renderer, 15.0f, 2.0f, c_grid);
        {
            float gp[3] = {0, 0, 0}, gs[3] = {15.0f, 0.5f, 15.0f};
            ivp_render_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        /* Ramp */
        {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(ramp, &p);
            ivp_object_get_orientation(ramp, &q);
            float rot[9]; quat_to_col_rot(&q, rot);
            float fp[3]; vec3r_to_float3(&p, fp);
            float hs[3] = {4.0f, 0.2f, 2.0f};
            ivp_render_draw_wire_box(renderer, fp, hs, rot, c_ramp);
        }

        /* Origin axes */
        {
            float origin[3] = {0, 0, 0};
            ivp_render_draw_axes(renderer, origin, NULL, 2.0f);
        }

        /* Dynamic objects */
        float max_dyn_speed = 0.0f;
        int toppled_count = 0;
        int fast_movers = 0;
        float sum_y = 0.0f;
        float sum_abs_xz = 0.0f;
        for (int i = 0; i < n_dyn; i++) {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(dyn[i].obj, &p);
            ivp_object_get_orientation(dyn[i].obj, &q);

            float fp[3]; vec3r_to_float3(&p, fp);

            if (dyn[i].is_sphere) {
                ivp_render_draw_wire_sphere(renderer, fp, dyn[i].radius, dyn[i].col);
            } else {
                float rot[9]; quat_to_col_rot(&q, rot);
                ivp_render_draw_wire_box(renderer, fp, dyn[i].hs, rot, dyn[i].col);

                float up_y = fabsf(rot[4]);
                if (up_y < 0.72f) toppled_count++;
            }

            /* Velocity arrow (only if moving fast enough) */
            ivp_vec3r_t v;
            ivp_object_get_speed(dyn[i].obj, &v);
            float speed = vec3r_speed(&v);
            if (speed > max_dyn_speed) max_dyn_speed = speed;
            if (speed > 2.0f) fast_movers++;
            sum_y += (float)p.k[1];
            sum_abs_xz += sqrtf((float)p.k[0] * (float)p.k[0] + (float)p.k[2] * (float)p.k[2]);
            if (speed > 0.5f) {
                float scale = 0.3f;
                float to[3] = {
                    fp[0] + (float)v.k[0] * scale,
                    fp[1] + (float)v.k[1] * scale,
                    fp[2] + (float)v.k[2] * scale
                };
                ivp_render_draw_arrow(renderer, fp, to, c_vel, 0.3f);
            }
        }

        if (ivp_example_inspector_tick(&inspector)) {
            ivp_example_inspector_log(&inspector,
                "max_speed=%.3f toppled=%d/%d fast=%d avg_y=%.3f avg_radius=%.3f",
                max_dyn_speed,
                toppled_count, n_dyn,
                fast_movers,
                sum_y / (float)n_dyn,
                sum_abs_xz / (float)n_dyn);
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus, c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  Objects: %d  Time: %.1fs",
                     sl.fps_display, n_dyn, (float)sl.t_prev);
            ivp_render_draw_text(renderer, 10.0f, 10.0f, buf, c_hud);
            ivp_example_draw_interaction_hud(renderer, 10.0f, 30.0f, c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, cube1_surf);
    ivp_compact_surface_free(NULL, cube2_surf);
    ivp_compact_surface_free(NULL, cube3_surf);
    ivp_compact_surface_free(NULL, ramp_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
