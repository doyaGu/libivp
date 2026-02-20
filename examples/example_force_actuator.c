/* example_force_actuator.c -- Magnetic force actuator demo
 *
 * Two cubes connected by a force actuator that pulls them together.
 * Every 5 seconds the magnet toggles on/off.  Force direction arrows
 * and distance readout shown in HUD.
 *
 * Demonstrates: ivp_actuator_force, force toggling, anchor-based
 *               force application, HUD text, velocity arrows.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/controllers/ivp_actuator_force.h>

#include <stdio.h>

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Force Actuator -- Magnet Toggle", .vsync = true
    };
    ivp_renderer_t *renderer = NULL;
    ivp_renderer_create(&renderer, &rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -3.0f;
    cam.orbit_distance  = 22.0f;
    cam.orbit_yaw       = 20.0f;
    cam.orbit_pitch     = -20.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "force_actuator");

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 15.0f, 0.5f, 10.0f);
    ivp_surman_polygon_t sm_ground;
    create_ground_object(env, ground_surf, &sm_ground, IVP_REAL(0.0));

    /* ── Two cubes ~8m apart ────────────────────────────────────────────── */
    ivp_compact_surface_t *cube_surf =
        ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    ivp_surman_polygon_t sm_a, sm_b;
    ivp_surman_polygon_init(&sm_a, cube_surf);
    ivp_surman_polygon_init(&sm_b, cube_surf);

    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.type = IVP_OBJECT_POLYGON;
    ta.mass = IVP_REAL(3.0);
    set_box_inertia(&ta, IVP_REAL(3.0), IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ta.material.friction   = IVP_REAL(0.6);
    ta.material.elasticity = IVP_REAL(0.2);
    ta.position = ivp_vec3r(IVP_REAL(-4.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    ivp_object_t *cube_a = ivp_environment_create_object(env, &ta, &sm_a.base);

    ivp_object_template_t tb;
    ivp_object_template_init(&tb);
    tb.type = IVP_OBJECT_POLYGON;
    tb.mass = IVP_REAL(3.0);
    set_box_inertia(&tb, IVP_REAL(3.0), IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    tb.material.friction   = IVP_REAL(0.6);
    tb.material.elasticity = IVP_REAL(0.2);
    tb.position = ivp_vec3r(IVP_REAL(4.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    ivp_object_t *cube_b = ivp_environment_create_object(env, &tb, &sm_b.base);

    /* ── Force actuator ─────────────────────────────────────────────────── */
    ivp_force_actuator_state_t fa_state;
    ivp_force_actuator_state_init(&fa_state);
    ivp_vec3r_t anchor_a = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t anchor_b = ivp_vec3r(IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    /* 15N attraction on 3kg cubes = 5 m/s² (half gravity).
     * Note: real magnetic force has inverse-square falloff with distance;
     * this constant force is a simplification for demonstration. */
    ivp_force_actuator_state_setup(&fa_state,
        cube_a, &anchor_a, cube_b, &anchor_b,
        IVP_REAL(15.0), true, true);
    ivp_environment_add_controller(env, 100,
        ivp_controller_force_actuator, &fa_state);

    /* ── Toggle state ───────────────────────────────────────────────────── */
    float toggle_timer = 0.0f;
    bool  magnet_on    = true;
    ivp_real_t force_mag = IVP_REAL(15.0);
    int   toggle_count = 0;
    float min_dist = 1000.0f;
    float max_dist = 0.0f;

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_ground[] = {0.22f, 0.24f, 0.30f};
    const float c_grid[]   = {0.38f, 0.43f, 0.52f};
    const float c_hud[]    = {0.95f, 0.97f, 1.0f};
    const float c_cube_a[] = {1.0f, 0.4f, 0.2f};
    const float c_cube_b[] = {0.3f, 0.5f, 1.0f};
    const float c_force[]  = {1.0f, 1.0f, 0.2f};
    const float c_vel[]    = {0.2f, 1.0f, 0.4f};
    const float c_drag[]   = {1.0f, 0.85f, 0.25f};
    const float c_focus[]  = {1.0f, 0.65f, 0.25f};
    const float c_on[]     = {0.2f, 1.0f, 0.3f};
    const float c_off[]    = {1.0f, 0.3f, 0.2f};

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

        /* Toggle magnet every 5 seconds */
        toggle_timer += frame_dt;
        if (toggle_timer >= 5.0f) {
            toggle_timer -= 5.0f;
            magnet_on = !magnet_on;
            ivp_force_actuator_set_force(&fa_state,
                magnet_on ? force_mag : IVP_REAL(0.0));
            toggle_count++;
        }

        sim_loop_step(&sl, env, frame_dt);

        float dist = (float)fa_state.current_distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;

        /* Grid & ground */
        ivp_render_draw_ground_grid(renderer, 15.0f, 2.0f, c_grid);
        {
            float gp[3] = {0, 0, 0}, gs[3] = {15.0f, 0.5f, 10.0f};
            ivp_render_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        /* Cubes */
        ivp_vec3r_t pa, pb;
        float fpa[3], fpb[3];
        {
            ivp_quatr_t qa;
            ivp_object_get_position(cube_a, &pa);
            ivp_object_get_orientation(cube_a, &qa);
            float rot[9]; quat_to_col_rot(&qa, rot);
            vec3r_to_float3(&pa, fpa);
            float hs[3] = {1.0f, 1.0f, 1.0f};
            ivp_render_draw_wire_box(renderer, fpa, hs, rot, c_cube_a);
        }
        {
            ivp_quatr_t qb;
            ivp_object_get_position(cube_b, &pb);
            ivp_object_get_orientation(cube_b, &qb);
            float rot[9]; quat_to_col_rot(&qb, rot);
            vec3r_to_float3(&pb, fpb);
            float hs[3] = {1.0f, 1.0f, 1.0f};
            ivp_render_draw_wire_box(renderer, fpb, hs, rot, c_cube_b);
        }

        /* Force direction arrow (A→B) when magnet is on */
        if (magnet_on)
            ivp_render_draw_arrow(renderer, fpa, fpb, c_force, 0.4f);

        /* Velocity arrows */
        for (int i = 0; i < 2; i++) {
            ivp_object_t *obj = (i == 0) ? cube_a : cube_b;
            float *fp = (i == 0) ? fpa : fpb;
            ivp_vec3r_t v;
            ivp_object_get_speed(obj, &v);
            float speed = vec3r_speed(&v);
            if (speed > 0.3f) {
                float sc = 0.3f;
                float to[3] = {fp[0]+(float)v.k[0]*sc,
                               fp[1]+(float)v.k[1]*sc,
                               fp[2]+(float)v.k[2]*sc};
                ivp_render_draw_arrow(renderer, fp, to, c_vel, 0.2f);
            }
        }

        /* Axes */
        {
            float org[3] = {0, 0, 0};
            ivp_render_draw_axes(renderer, org, NULL, 1.5f);
        }

        /* Inspector */
        {
            ivp_vec3r_t va, vb;
            ivp_object_get_speed(cube_a, &va);
            ivp_object_get_speed(cube_b, &vb);
            if (ivp_example_inspector_tick(&inspector)) {
                ivp_example_inspector_log(&inspector,
                    "dist=%.3f force=%.1f on=%d toggles=%d speed_a=%.3f speed_b=%.3f min_dist=%.3f max_dist=%.3f",
                    dist, (float)fa_state.force,
                    magnet_on ? 1 : 0, toggle_count,
                    vec3r_speed(&va), vec3r_speed(&vb),
                    min_dist, max_dist);
            }
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus,
                                              c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  Time: %.1fs",
                     sl.fps_display, (float)sl.t_prev);
            ivp_render_draw_text(renderer, 10.0f, 10.0f, buf, c_hud);

            snprintf(buf, sizeof(buf), "Force: %.1f N  Distance: %.2f m",
                     (float)fa_state.force, dist);
            ivp_render_draw_text(renderer, 10.0f, 30.0f, buf, c_force);

            snprintf(buf, sizeof(buf), "Magnet: %s  (toggles every 5s)",
                     magnet_on ? "ON" : "OFF");
            ivp_render_draw_text(renderer, 10.0f, 48.0f, buf,
                             magnet_on ? c_on : c_off);

            ivp_example_draw_interaction_hud(renderer, 10.0f, 70.0f,
                                             c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, cube_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
