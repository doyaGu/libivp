/* example_collision_filter.c -- Collision filter demonstrations
 *
 * Three zones side by side demonstrate different collision filters:
 *   Zone 1 (left):  Exclusive pair filter — two cubes toggle collision
 *                   every 5 seconds (pass through ↔ collide).
 *   Zone 2 (center): Group filter — same-group objects pass through
 *                   each other, different groups collide normally.
 *   Zone 3 (right): Layer mask filter — objects on disjoint layers
 *                   ignore each other.
 *
 * A meta filter combines group + layer mask as AND composite.
 *
 * Demonstrates: ivp_collision_filter (group, layer_mask, exclusive_pair,
 *               meta), collision_group, layer_mask on objects.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/collision/ivp_collision_filter.h>

#include <stdio.h>

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Collision Filters -- Group, Layer, Pair", .vsync = true
    };
    ivp_renderer_t *renderer = NULL;
    ivp_renderer_create(&renderer, &rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -4.0f;
    cam.orbit_distance  = 35.0f;
    cam.orbit_yaw       = 15.0f;
    cam.orbit_pitch     = -25.0f;

    /* ── Meta collision filter: group AND layer_mask ─────────────────────── */
    ivp_collision_filter_t sub_filters[2];
    sub_filters[0] = ivp_collision_filter_group();
    sub_filters[1] = ivp_collision_filter_layer_mask();
    ivp_meta_collision_filter_t meta;
    ivp_collision_filter_t meta_filter =
        ivp_collision_filter_meta(&meta, sub_filters, 2);

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ecfg.collision_filter = meta_filter;
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "collision_filter");

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 25.0f, 0.5f, 10.0f);
    ivp_surman_polygon_t sm_ground;
    create_ground_object(env, ground_surf, &sm_ground, IVP_REAL(0.0));

    /* ── Shared surfaces ────────────────────────────────────────────────── */
    ivp_compact_surface_t *cube_surf =
        ivp_build_box_surface(NULL, 0.8f, 0.8f, 0.8f);
    ivp_surman_polygon_t sm[12];
    ivp_object_t *objs[12];
    int n_obj = 0;

    /* ═══════════════════════════════════════════════════════════════════════
     * Zone 1 (LEFT, x=-12): Exclusive pair filter
     * Two cubes stacked — pair filter disabled → they pass through.
     * Toggled every 5 seconds.
     * ══════════════════════════════════════════════════════════════════════ */
    ivp_object_template_t z1t;
    ivp_object_template_init(&z1t);
    z1t.type = IVP_OBJECT_POLYGON;
    z1t.mass = IVP_REAL(2.0);          /* 2kg cubes — moderate mass for visible interaction */
    set_box_inertia(&z1t, IVP_REAL(2.0), IVP_REAL(0.8), IVP_REAL(0.8), IVP_REAL(0.8));
    z1t.material.friction   = IVP_REAL(0.7);   /* high friction: cubes stack reliably */
    z1t.material.elasticity = IVP_REAL(0.2);   /* low bounce: focus on filter behavior */

    /* Lower cube */
    ivp_surman_polygon_init(&sm[n_obj], cube_surf);
    z1t.position = ivp_vec3r(IVP_REAL(-12.0), IVP_REAL(-1.3), IVP_REAL(0.0));
    objs[n_obj] = ivp_environment_create_object(env, &z1t, &sm[n_obj].base);
    int z1_lower = n_obj++;

    /* Upper cube (dropped from above) */
    ivp_surman_polygon_init(&sm[n_obj], cube_surf);
    z1t.position = ivp_vec3r(IVP_REAL(-12.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    objs[n_obj] = ivp_environment_create_object(env, &z1t, &sm[n_obj].base);
    int z1_upper = n_obj++;

    /* Exclusive pair filter */
    ivp_exclusive_pair_filter_t pair_filter;
    ivp_exclusive_pair_filter_init(&pair_filter, NULL);
    /* Start with collision disabled between the pair */
    ivp_exclusive_pair_filter_disable(&pair_filter,
        objs[z1_lower], objs[z1_upper]);

    #define PAIR_TOGGLE_PERIOD 5.0f  /* seconds between collision on/off toggle */
    float pair_toggle_timer = 0.0f;
    bool  pair_collision_off = true;
    int   pair_toggle_count  = 0;

    /* ═══════════════════════════════════════════════════════════════════════
     * Zone 2 (CENTER, x=0): Group filter
     * Two red cubes (group 1) and two blue cubes (group 2).
     * Same-group cubes pass through each other; different groups collide.
     * ══════════════════════════════════════════════════════════════════════ */
    ivp_object_template_t z2t;
    ivp_object_template_init(&z2t);
    z2t.type = IVP_OBJECT_POLYGON;
    z2t.mass = IVP_REAL(2.0);
    set_box_inertia(&z2t, IVP_REAL(2.0), IVP_REAL(0.8), IVP_REAL(0.8), IVP_REAL(0.8));
    z2t.material.friction   = IVP_REAL(0.6);
    z2t.material.elasticity = IVP_REAL(0.2);

    /* Red cubes — group 1 */
    for (int i = 0; i < 2; i++) {
        ivp_surman_polygon_init(&sm[n_obj], cube_surf);
        z2t.position = ivp_vec3r(
            IVP_REAL(-0.5) + IVP_REAL(1.0) * (ivp_real_t)i,
            IVP_REAL(-4.0) - IVP_REAL(2.0) * (ivp_real_t)i,
            IVP_REAL(0.0));
        objs[n_obj] = ivp_environment_create_object(env, &z2t, &sm[n_obj].base);
        objs[n_obj]->collision_group = 1;
        n_obj++;
    }
    int z2_start = n_obj - 2;

    /* Blue cubes — group 2 */
    for (int i = 0; i < 2; i++) {
        ivp_surman_polygon_init(&sm[n_obj], cube_surf);
        z2t.position = ivp_vec3r(
            IVP_REAL(0.5) - IVP_REAL(1.0) * (ivp_real_t)i,
            IVP_REAL(-5.0) - IVP_REAL(2.0) * (ivp_real_t)i,
            IVP_REAL(0.0));
        objs[n_obj] = ivp_environment_create_object(env, &z2t, &sm[n_obj].base);
        objs[n_obj]->collision_group = 2;
        n_obj++;
    }

    /* ═══════════════════════════════════════════════════════════════════════
     * Zone 3 (RIGHT, x=12): Layer mask filter
     * Green cubes on layer 0x01, yellow cubes on layer 0x02.
     * Disjoint layers → they pass through each other.
     * Both collide with ground (layer 0xFFFFFFFF).
     * ══════════════════════════════════════════════════════════════════════ */
    ivp_object_template_t z3t;
    ivp_object_template_init(&z3t);
    z3t.type = IVP_OBJECT_POLYGON;
    z3t.mass = IVP_REAL(2.0);
    set_box_inertia(&z3t, IVP_REAL(2.0), IVP_REAL(0.8), IVP_REAL(0.8), IVP_REAL(0.8));
    z3t.material.friction   = IVP_REAL(0.6);
    z3t.material.elasticity = IVP_REAL(0.2);

    /* Green — layer 0x01 */
    for (int i = 0; i < 2; i++) {
        ivp_surman_polygon_init(&sm[n_obj], cube_surf);
        z3t.position = ivp_vec3r(
            IVP_REAL(11.5) + IVP_REAL(1.0) * (ivp_real_t)i,
            IVP_REAL(-5.0) - IVP_REAL(2.0) * (ivp_real_t)i,
            IVP_REAL(0.0));
        objs[n_obj] = ivp_environment_create_object(env, &z3t, &sm[n_obj].base);
        objs[n_obj]->layer_mask = 0x01;
        n_obj++;
    }
    int z3_start = n_obj - 2;

    /* Yellow — layer 0x02 */
    for (int i = 0; i < 2; i++) {
        ivp_surman_polygon_init(&sm[n_obj], cube_surf);
        z3t.position = ivp_vec3r(
            IVP_REAL(12.5) - IVP_REAL(1.0) * (ivp_real_t)i,
            IVP_REAL(-6.0) - IVP_REAL(2.0) * (ivp_real_t)i,
            IVP_REAL(0.0));
        objs[n_obj] = ivp_environment_create_object(env, &z3t, &sm[n_obj].base);
        objs[n_obj]->layer_mask = 0x02;
        n_obj++;
    }

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_ground[] = {0.22f, 0.24f, 0.30f};
    const float c_grid[]   = {0.38f, 0.43f, 0.52f};
    const float c_hud[]    = {0.95f, 0.97f, 1.0f};
    const float c_z1_on[]  = {0.9f, 0.5f, 0.2f};
    const float c_z1_off[] = {0.4f, 0.4f, 0.4f};
    const float c_red[]    = {1.0f, 0.3f, 0.2f};
    const float c_blue[]   = {0.3f, 0.5f, 1.0f};
    const float c_green[]  = {0.2f, 1.0f, 0.3f};
    const float c_yellow[] = {1.0f, 1.0f, 0.2f};
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

        /* Toggle Zone 1 pair filter periodically */
        pair_toggle_timer += frame_dt;
        if (pair_toggle_timer >= PAIR_TOGGLE_PERIOD) {
            pair_toggle_timer -= PAIR_TOGGLE_PERIOD;
            pair_collision_off = !pair_collision_off;
            pair_toggle_count++;
            if (pair_collision_off)
                ivp_exclusive_pair_filter_disable(&pair_filter,
                    objs[z1_lower], objs[z1_upper]);
            else
                ivp_exclusive_pair_filter_enable(&pair_filter,
                    objs[z1_lower], objs[z1_upper]);
        }

        sim_loop_step(&sl, env, frame_dt);

        /* Grid & ground */
        ivp_render_draw_ground_grid(renderer, 25.0f, 2.0f, c_grid);
        {
            float gp[3] = {0, 0, 0}, gs[3] = {25.0f, 0.5f, 10.0f};
            ivp_render_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        /* Draw all objects per zone */
        for (int i = 0; i < n_obj; i++) {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(objs[i], &p);
            ivp_object_get_orientation(objs[i], &q);
            float fp[3]; vec3r_to_float3(&p, fp);
            float rot[9]; quat_to_col_rot(&q, rot);
            float hs[3] = {0.8f, 0.8f, 0.8f};

            const float *col;
            if (i == z1_lower || i == z1_upper)
                col = pair_collision_off ? c_z1_off : c_z1_on;
            else if (i >= z2_start && i < z2_start + 2)
                col = c_red;
            else if (i >= z2_start + 2 && i < z2_start + 4)
                col = c_blue;
            else if (i >= z3_start && i < z3_start + 2)
                col = c_green;
            else
                col = c_yellow;

            ivp_render_draw_wire_box(renderer, fp, hs, rot, col);
        }

        /* Zone separators (vertical lines) */
        {
            float a[3] = {-6.0f, -12.0f, -5.0f};
            float b[3] = {-6.0f, 0.0f, -5.0f};
            ivp_render_draw_line(renderer, a, b, c_hud);
            a[0] = b[0] = 6.0f;
            ivp_render_draw_line(renderer, a, b, c_hud);
        }

        /* Axes */
        {
            float org[3] = {0, 0, 0};
            ivp_render_draw_axes(renderer, org, NULL, 1.5f);
        }

        if (ivp_example_inspector_tick(&inspector)) {
            ivp_example_inspector_log(&inspector,
                "pair_off=%d pair_toggles=%d",
                pair_collision_off ? 1 : 0,
                pair_toggle_count);
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus,
                                              c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  Objects: %d  Time: %.1fs",
                     sl.fps_display, n_obj, (float)sl.t_prev);
            ivp_render_draw_text(renderer, 10.0f, 10.0f, buf, c_hud);

            snprintf(buf, sizeof(buf),
                "Zone1: ExclusivePair %s (toggle 5s)",
                pair_collision_off ? "OFF (ghost)" : "ON (collide)");
            ivp_render_draw_text(renderer, 10.0f, 30.0f, buf,
                             pair_collision_off ? c_z1_off : c_z1_on);

            ivp_render_draw_text(renderer, 10.0f, 48.0f,
                "Zone2: GroupFilter (same group=ghost)", c_red);

            ivp_render_draw_text(renderer, 10.0f, 66.0f,
                "Zone3: LayerMask (disjoint layers=ghost)", c_green);

            ivp_example_draw_interaction_hud(renderer, 10.0f, 88.0f,
                                             c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_exclusive_pair_filter_destroy(&pair_filter);
    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, cube_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
