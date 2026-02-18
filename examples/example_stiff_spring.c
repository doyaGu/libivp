/* example_stiff_spring.c -- Regular vs stiff spring comparison
 *
 * Two side-by-side spring pairs with identical stiffness (k=5000).
 * Left pair uses a regular explicit spring — shows instability.
 * Right pair uses an implicit stiff spring — remains stable.
 * Both start displaced.  A heavy ball drops onto each to stress-test.
 *
 * Demonstrates: ivp_actuator_spring, ivp_controller_stiff_spring,
 *               high-stiffness instability, implicit integration.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_controller_stiff_spring.h>

#include <stdio.h>

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Stiff Spring -- Regular vs Implicit", .vsync = true
    };
    ivp_renderer_t *renderer = ivp_renderer_create(&rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[0]      =  0.0f;
    cam.target[1]      = -5.0f;
    cam.orbit_distance  = 28.0f;
    cam.orbit_yaw       = 0.0f;
    cam.orbit_pitch     = -15.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    /* Enable anomaly detection: the regular spring side will trigger anomalies
     * when the explicit integrator causes energy gain at high stiffness. */
    ecfg.anomaly_config.max_speed = IVP_REAL(50.0);
    ecfg.anomaly_config.auto_correct = true;
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "stiff_spring");

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 20.0f, 0.5f, 10.0f);
    ivp_surman_polygon_t sm_ground;
    create_ground_object(env, ground_surf, &sm_ground, IVP_REAL(0.0));

    /* ── Static anchors (ceiling) ───────────────────────────────────────── */
    ivp_compact_surface_t *anchor_surf =
        ivp_build_box_surface(NULL, 0.3f, 0.3f, 0.3f);

    ivp_surman_polygon_t sm_anch_l, sm_anch_r;
    ivp_surman_polygon_init(&sm_anch_l, anchor_surf);
    ivp_surman_polygon_init(&sm_anch_r, anchor_surf);

    ivp_object_template_t at;
    ivp_object_template_init(&at);
    at.type = IVP_OBJECT_POLYGON;
    at.mass = 0.0f;
    at.physical_unmoveable = true;

    at.position = ivp_vec3r(IVP_REAL(-6.0), IVP_REAL(-12.0), IVP_REAL(0.0));
    ivp_object_t *anchor_l = ivp_environment_create_object(env, &at, &sm_anch_l.base);

    at.position = ivp_vec3r(IVP_REAL(6.0), IVP_REAL(-12.0), IVP_REAL(0.0));
    ivp_object_t *anchor_r = ivp_environment_create_object(env, &at, &sm_anch_r.base);

    /* ── Hanging cubes (displaced downward to stretch springs) ──────────── */
    ivp_compact_surface_t *cube_surf =
        ivp_build_box_surface(NULL, 0.8f, 0.8f, 0.8f);

    ivp_surman_polygon_t sm_cube_l, sm_cube_r;
    ivp_surman_polygon_init(&sm_cube_l, cube_surf);
    ivp_surman_polygon_init(&sm_cube_r, cube_surf);

    ivp_object_template_t ct;
    ivp_object_template_init(&ct);
    ct.type = IVP_OBJECT_POLYGON;
    ct.mass = IVP_REAL(2.0);
    set_box_inertia(&ct, IVP_REAL(2.0), IVP_REAL(0.8), IVP_REAL(0.8), IVP_REAL(0.8));
    ct.material.friction   = IVP_REAL(0.6);
    ct.material.elasticity = IVP_REAL(0.1);

    ct.position = ivp_vec3r(IVP_REAL(-6.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    ivp_object_t *cube_l = ivp_environment_create_object(env, &ct, &sm_cube_l.base);

    ct.position = ivp_vec3r(IVP_REAL(6.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    ivp_object_t *cube_r = ivp_environment_create_object(env, &ct, &sm_cube_r.base);

    /* ── Heavy stress-test balls ────────────────────────────────────────── */
    ivp_surman_ball_t sm_ball_l, sm_ball_r;
    ivp_surman_ball_init(&sm_ball_l, 0.6f);
    ivp_surman_ball_init(&sm_ball_r, 0.6f);

    ivp_object_template_t bt;
    ivp_object_template_init(&bt);
    bt.type = IVP_OBJECT_BALL;
    bt.mass = IVP_REAL(8.0);
    set_sphere_inertia(&bt, IVP_REAL(8.0), IVP_REAL(0.6));
    bt.material.friction   = IVP_REAL(0.4);
    bt.material.elasticity = IVP_REAL(0.3);

    bt.position = ivp_vec3r(IVP_REAL(-6.0), IVP_REAL(-14.0), IVP_REAL(0.0));
    ivp_object_t *ball_l = ivp_environment_create_object(env, &bt, &sm_ball_l.base);

    bt.position = ivp_vec3r(IVP_REAL(6.0), IVP_REAL(-14.0), IVP_REAL(0.0));
    ivp_object_t *ball_r = ivp_environment_create_object(env, &bt, &sm_ball_r.base);

    /* ── Regular spring (LEFT, k=5000) ──────────────────────────────────── */
    ivp_spring_state_t reg_spring;
    ivp_spring_state_init(&reg_spring);
    {
        ivp_vec3r_t a = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.3), IVP_REAL(0.0));
        ivp_vec3r_t b = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.8), IVP_REAL(0.0));
        ivp_spring_state_setup(&reg_spring,
            anchor_l, &a, cube_l, &b,
            IVP_REAL(5000.0), IVP_REAL(20.0), IVP_REAL(3.0));
    }
    ivp_environment_add_controller(env, 100, ivp_controller_spring, &reg_spring);

    /* ── Stiff spring (RIGHT, k=5000) ───────────────────────────────────── */
    ivp_stiff_spring_state_t stiff_spring;
    ivp_stiff_spring_state_init(&stiff_spring);
    {
        ivp_vec3r_t a = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.3), IVP_REAL(0.0));
        ivp_vec3r_t b = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.8), IVP_REAL(0.0));
        ivp_stiff_spring_state_setup(&stiff_spring,
            anchor_r, &a, cube_r, &b,
            IVP_REAL(5000.0), IVP_REAL(20.0), IVP_REAL(3.0));
    }
    ivp_environment_add_controller(env, 100,
        ivp_controller_stiff_spring, &stiff_spring);

    /* ── Tracking ───────────────────────────────────────────────────────── */
    float max_reg_ext   = 0.0f;
    float max_stiff_ext = 0.0f;
    int   prev_anomalies = 0;

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_ground[]  = {0.22f, 0.24f, 0.30f};
    const float c_grid[]    = {0.38f, 0.43f, 0.52f};
    const float c_hud[]     = {0.95f, 0.97f, 1.0f};
    const float c_anchor[]  = {0.5f, 0.5f, 0.5f};
    const float c_reg[]     = {1.0f, 0.4f, 0.2f};  /* red-orange */
    const float c_stiff[]   = {0.3f, 0.8f, 1.0f};  /* cyan */
    const float c_spring_r[] = {1.0f, 0.8f, 0.2f};
    const float c_spring_s[] = {0.2f, 1.0f, 0.8f};
    const float c_ball[]    = {0.6f, 0.6f, 0.7f};
    const float c_vel[]     = {0.2f, 1.0f, 0.4f};
    const float c_drag[]    = {1.0f, 0.85f, 0.25f};
    const float c_focus[]   = {1.0f, 0.65f, 0.25f};

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

        /* Track extension */
        float reg_ext = (float)(reg_spring.current_length - reg_spring.rest_length);
        float stiff_ext = (float)(stiff_spring.current_length - stiff_spring.rest_length);
        if (reg_ext < 0) reg_ext = -reg_ext;
        if (stiff_ext < 0) stiff_ext = -stiff_ext;
        if (reg_ext > max_reg_ext) max_reg_ext = reg_ext;
        if (stiff_ext > max_stiff_ext) max_stiff_ext = stiff_ext;

        /* Grid & ground */
        ivp_draw_ground_grid(renderer, 20.0f, 2.0f, c_grid);
        {
            float gp[3] = {0, 0, 0}, gs[3] = {20.0f, 0.5f, 10.0f};
            ivp_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        /* Draw anchors */
        for (int i = 0; i < 2; i++) {
            ivp_object_t *a = (i == 0) ? anchor_l : anchor_r;
            ivp_vec3r_t p;
            ivp_object_get_position(a, &p);
            float fp[3]; vec3r_to_float3(&p, fp);
            float hs[3] = {0.3f, 0.3f, 0.3f};
            ivp_draw_wire_box(renderer, fp, hs, NULL, c_anchor);
        }

        /* Draw hanging cubes + spring lines */
        float fp_cubes[2][3];
        float fp_anchors[2][3];
        for (int i = 0; i < 2; i++) {
            ivp_object_t *c = (i == 0) ? cube_l : cube_r;
            ivp_object_t *a = (i == 0) ? anchor_l : anchor_r;
            const float *col = (i == 0) ? c_reg : c_stiff;
            const float *spr = (i == 0) ? c_spring_r : c_spring_s;

            ivp_vec3r_t cp, ap; ivp_quatr_t cq;
            ivp_object_get_position(c, &cp);
            ivp_object_get_orientation(c, &cq);
            ivp_object_get_position(a, &ap);

            vec3r_to_float3(&cp, fp_cubes[i]);
            vec3r_to_float3(&ap, fp_anchors[i]);

            float rot[9]; quat_to_col_rot(&cq, rot);
            float hs[3] = {0.8f, 0.8f, 0.8f};
            ivp_draw_wire_box(renderer, fp_cubes[i], hs, rot, col);
            ivp_draw_thick_line(renderer, fp_anchors[i], fp_cubes[i], spr, 2);
        }

        /* Draw stress-test balls */
        for (int i = 0; i < 2; i++) {
            ivp_object_t *b = (i == 0) ? ball_l : ball_r;
            ivp_vec3r_t p;
            ivp_object_get_position(b, &p);
            float fp[3]; vec3r_to_float3(&p, fp);
            ivp_draw_wire_sphere(renderer, fp, 0.6f, c_ball);

            ivp_vec3r_t v;
            ivp_object_get_speed(b, &v);
            float speed = vec3r_speed(&v);
            if (speed > 0.3f) {
                float sc = 0.3f;
                float to[3] = {fp[0]+(float)v.k[0]*sc,
                               fp[1]+(float)v.k[1]*sc,
                               fp[2]+(float)v.k[2]*sc};
                ivp_draw_arrow(renderer, fp, to, c_vel, 0.2f);
            }
        }

        /* Labels over each side */
        {
            float lp[3] = {-6.0f, -14.0f, 0.0f};
            float rp[3] = { 6.0f, -14.0f, 0.0f};
            ivp_draw_text_2d(renderer, (float)(win_w / 4 - 60), (float)(win_h - 30),
                             "REGULAR (explicit)", c_reg);
            ivp_draw_text_2d(renderer, (float)(3 * win_w / 4 - 50), (float)(win_h - 30),
                             "STIFF (implicit)", c_stiff);
        }

        /* Axes */
        {
            float org[3] = {0, 0, 0};
            ivp_draw_axes(renderer, org, NULL, 1.5f);
        }

        if (ivp_example_inspector_tick(&inspector)) {
            ivp_example_inspector_log(&inspector,
                "reg_len=%.3f reg_force=%.3f reg_max_ext=%.3f stiff_len=%.3f stiff_max_ext=%.3f",
                (float)reg_spring.current_length,
                (float)reg_spring.current_force,
                max_reg_ext,
                (float)stiff_spring.current_length,
                max_stiff_ext);
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus,
                                              c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  k=5000 N/m  Time: %.1fs",
                     sl.fps_display, (float)sl.t_prev);
            ivp_draw_text_2d(renderer, 10.0f, 10.0f, buf, c_hud);

            snprintf(buf, sizeof(buf), "Regular: len=%.2f force=%.1f",
                     (float)reg_spring.current_length,
                     (float)reg_spring.current_force);
            ivp_draw_text_2d(renderer, 10.0f, 30.0f, buf, c_reg);

            snprintf(buf, sizeof(buf), "Stiff:   len=%.2f",
                     (float)stiff_spring.current_length);
            ivp_draw_text_2d(renderer, 10.0f, 48.0f, buf, c_stiff);

            /* Anomaly detection status */
            int anom = env->anomaly_manager.anomalies_detected;
            int corr = env->anomaly_manager.anomalies_corrected;
            snprintf(buf, sizeof(buf), "Anomalies: %d detected, %d corrected (max_speed=50)",
                     anom, corr);
            const float c_anom[] = {1.0f, 0.6f, 0.2f};
            ivp_draw_text_2d(renderer, 10.0f, 66.0f, buf,
                             anom > prev_anomalies ? c_anom : c_hud);
            prev_anomalies = anom;

            ivp_example_draw_interaction_hud(renderer, 10.0f, 88.0f,
                                             c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, anchor_surf);
    ivp_compact_surface_free(NULL, cube_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
