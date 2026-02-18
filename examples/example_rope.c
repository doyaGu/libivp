/* example_rope.c -- Rope / pendulum chain with weight
 *
 * A longer chain of links connected by ball-socket constraints,
 * anchored to a static ceiling bar.  A heavy weight hangs at the end.
 * A ground plane catches anything that falls.  Joints are highlighted.
 *
 * Demonstrates: ivp_constraint (ball_socket), chain building,
 *               stacking objects, weight at chain end, HUD text.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/base/ivp_types.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/controllers/ivp_constraint.h>

#include <stdio.h>
#include <string.h>

#define CHAIN_LEN   8
#define LINK_HX     0.20f
#define LINK_HY     0.55f
#define LINK_HZ     0.20f
#define LINK_GAP    0.05f
#define INITIAL_SWING_X 1.8

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Rope -- Chain Pendulum", .vsync = true
    };
    ivp_renderer_t *renderer = ivp_renderer_create(&rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -8.0f;
    cam.orbit_distance  = 22.0f;
    cam.orbit_yaw       = 25.0f;
    cam.orbit_pitch     = -15.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "rope");
    float weight_peak_abs_x = 0.0f;

    /* ── Ground plane ───────────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 12.0f, 0.5f, 12.0f);
    ivp_surman_polygon_t sm_ground;
    ivp_surman_polygon_init(&sm_ground, ground_surf);

    ivp_object_template_t grt;
    ivp_object_template_init(&grt);
    grt.type = IVP_OBJECT_POLYGON;
    grt.mass = 0.0f;
    grt.physical_unmoveable = true;
    grt.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    ivp_environment_create_object(env, &grt, &sm_ground.base);

    /* ── Link surface ───────────────────────────────────────────────────── */
    ivp_compact_surface_t *link_surf =
        ivp_build_box_surface(NULL, LINK_HX, LINK_HY, LINK_HZ);

    /* surman per object */
    ivp_surman_polygon_t sm_links[CHAIN_LEN + 1]; /* +1 for ceiling mount */
    for (int i = 0; i <= CHAIN_LEN; i++)
        ivp_surman_polygon_init(&sm_links[i], link_surf);

    /* ── Ceiling bar (static, wider) ────────────────────────────────────── */
    ivp_compact_surface_t *ceil_surf =
        ivp_build_box_surface(NULL, 1.5f, 0.3f, 0.3f);
    ivp_surman_polygon_t sm_ceil;
    ivp_surman_polygon_init(&sm_ceil, ceil_surf);

    ivp_object_template_t ct;
    ivp_object_template_init(&ct);
    ct.type = IVP_OBJECT_POLYGON;
    ct.mass = 0.0f;
    ct.physical_unmoveable = true;
    ct.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-15.0), IVP_REAL(0.0));
    ivp_object_t *ceiling = ivp_environment_create_object(env, &ct, &sm_ceil.base);

    /* ── Chain links ────────────────────────────────────────────────────── */
    ivp_object_t *links[CHAIN_LEN];
    ivp_real_t step = 2.0f * LINK_HY + LINK_GAP;

    /* Position links in a slight downward arc from the ceiling attachment
     * to approximate the hanging configuration under gravity. This reduces
     * violent initial constraint shock vs starting all links at the same X. */
    for (int i = 0; i < CHAIN_LEN; i++) {
        ivp_object_template_t lt;
        ivp_object_template_init(&lt);
        lt.type = IVP_OBJECT_POLYGON;
        lt.mass = IVP_REAL(0.6);
        set_box_inertia(&lt, IVP_REAL(0.6),
            (ivp_real_t)LINK_HX, (ivp_real_t)LINK_HY, (ivp_real_t)LINK_HZ);
        lt.material.friction   = IVP_REAL(0.6);
        lt.material.elasticity = IVP_REAL(0.05);
        lt.speed_damp_factor     = IVP_REAL(0.03);
        lt.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.03), IVP_REAL(0.03), IVP_REAL(0.03));
        /* Arc: X offset decays from INITIAL_SWING_X at top toward 0 at bottom */
        float t_frac = (float)(i + 1) / (float)(CHAIN_LEN + 1);
        ivp_real_t offset_x = (ivp_real_t)(INITIAL_SWING_X * (1.0f - t_frac));
        lt.position = ivp_vec3r(offset_x,
            IVP_REAL(-15.0) + step * (ivp_real_t)(i + 1),
            IVP_REAL(0.0));
        links[i] = ivp_environment_create_object(env, &lt, &sm_links[i + 1].base);
    }

    /* ── Weight at chain end ────────────────────────────────────────────── */
    ivp_compact_surface_t *weight_surf =
        ivp_build_box_surface(NULL, 0.6f, 0.6f, 0.6f);
    ivp_surman_polygon_t sm_weight;
    ivp_surman_polygon_init(&sm_weight, weight_surf);

    ivp_object_template_t wt;
    ivp_object_template_init(&wt);
    wt.type = IVP_OBJECT_POLYGON;
    wt.mass = IVP_REAL(2.0);
    /* 0.6m half-extent cube, 2kg: I_x = (2/3)(0.36+0.36) = 0.48. Use helper for consistency. */
    set_box_inertia(&wt, IVP_REAL(2.0), IVP_REAL(0.6), IVP_REAL(0.6), IVP_REAL(0.6));
    wt.material.friction   = IVP_REAL(0.8);
    wt.material.elasticity = IVP_REAL(0.1);
    wt.speed_damp_factor     = IVP_REAL(0.02);
    wt.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.02), IVP_REAL(0.02), IVP_REAL(0.02));
    /* Weight at bottom of arc — smallest X offset */
    float w_frac = (float)(CHAIN_LEN + 1) / (float)(CHAIN_LEN + 1);
    ivp_real_t weight_x = (ivp_real_t)(INITIAL_SWING_X * (1.0f - w_frac));
    wt.position = ivp_vec3r(weight_x,
        IVP_REAL(-15.0) + step * (ivp_real_t)(CHAIN_LEN + 1),
        IVP_REAL(0.0));
    ivp_object_t *weight = ivp_environment_create_object(env, &wt, &sm_weight.base);

    /* ── Constraints ────────────────────────────────────────────────────── */
    ivp_constraint_state_t constraints[CHAIN_LEN + 1];
    memset(constraints, 0, sizeof(constraints));

    /* Ceiling → first link */
    {
        ivp_vec3r_t a_pt = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.3), IVP_REAL(0.0));
        ivp_vec3r_t b_pt = ivp_vec3r(IVP_REAL(0.0), -LINK_HY, IVP_REAL(0.0));
        ivp_constraint_init_ball_socket(&constraints[0],
            ceiling, &a_pt, links[0], &b_pt);
        ivp_environment_add_controller(env, 50,
            ivp_controller_constraint, &constraints[0]);
    }

    /* Link-to-link */
    for (int i = 1; i < CHAIN_LEN; i++) {
        ivp_vec3r_t a_bot = ivp_vec3r(IVP_REAL(0.0), LINK_HY, IVP_REAL(0.0));
        ivp_vec3r_t b_top = ivp_vec3r(IVP_REAL(0.0), -LINK_HY, IVP_REAL(0.0));
        ivp_constraint_init_ball_socket(&constraints[i],
            links[i - 1], &a_bot, links[i], &b_top);
        ivp_environment_add_controller(env, 50,
            ivp_controller_constraint, &constraints[i]);
    }

    /* Last link → weight */
    {
        ivp_vec3r_t a_bot = ivp_vec3r(IVP_REAL(0.0), LINK_HY, IVP_REAL(0.0));
        ivp_vec3r_t b_top = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.6), IVP_REAL(0.0));
        ivp_constraint_init_ball_socket(&constraints[CHAIN_LEN],
            links[CHAIN_LEN - 1], &a_bot, weight, &b_top);
        ivp_environment_add_controller(env, 50,
            ivp_controller_constraint, &constraints[CHAIN_LEN]);
    }

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_link[]    = {0.85f, 0.65f, 0.20f};
    const float c_weight[]  = {0.90f, 0.25f, 0.20f};
    const float c_ceil[]    = {0.50f, 0.50f, 0.55f};
    const float c_joint[]   = {1.0f,  0.30f, 0.30f};
    const float c_ground[]  = {0.30f, 0.30f, 0.35f};
    const float c_grid[]    = {0.12f, 0.12f, 0.18f};
    const float c_hud[]     = {0.80f, 0.80f, 0.80f};
    const float c_vel[]     = {0.20f, 1.0f,  0.40f};
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

        float max_speed = 0.0f;
        float max_stretch_ratio = 0.0f;
        int max_stretch_link = -1;
        const float expected_link_dist = 2.0f * LINK_HY + LINK_GAP;
        const float expected_weight_dist = LINK_HY + 0.6f + LINK_GAP;
        float sum_link_x = 0.0f;
        for (int i = 0; i < CHAIN_LEN; i++) {
            ivp_vec3r_t p;
            ivp_object_get_position(links[i], &p);
            sum_link_x += (float)p.k[0];

            ivp_vec3r_t v;
            ivp_object_get_speed(links[i], &v);
            float speed = vec3r_speed(&v);
            if (speed > max_speed) max_speed = speed;

            if (i > 0) {
                ivp_vec3r_t pa, pb;
                ivp_object_get_position(links[i - 1], &pa);
                ivp_object_get_position(links[i], &pb);
                float dx = (float)(pb.k[0] - pa.k[0]);
                float dy = (float)(pb.k[1] - pa.k[1]);
                float dz = (float)(pb.k[2] - pa.k[2]);
                float dist = sqrtf(dx*dx + dy*dy + dz*dz);
                if (isfinite(dist) && expected_link_dist > 1e-5f) {
                    float ratio = dist / expected_link_dist;
                    if (ratio > max_stretch_ratio) {
                        max_stretch_ratio = ratio;
                        max_stretch_link = i;
                    }
                }
            }
        }
        {
            ivp_vec3r_t vw, pa, pb, wp;
            ivp_object_get_speed(weight, &vw);
            float ws = vec3r_speed(&vw);
            if (ws > max_speed) max_speed = ws;
            ivp_object_get_position(weight, &wp);
            float abs_x = fabsf((float)wp.k[0]);
            if (abs_x > weight_peak_abs_x) weight_peak_abs_x = abs_x;

            ivp_object_get_position(links[CHAIN_LEN - 1], &pa);
            ivp_object_get_position(weight, &pb);
            float dx = (float)(pb.k[0] - pa.k[0]);
            float dy = (float)(pb.k[1] - pa.k[1]);
            float dz = (float)(pb.k[2] - pa.k[2]);
            float dist = sqrtf(dx*dx + dy*dy + dz*dz);
            if (isfinite(dist) && expected_weight_dist > 1e-5f) {
                float ratio = dist / expected_weight_dist;
                if (ratio > max_stretch_ratio) {
                    max_stretch_ratio = ratio;
                    max_stretch_link = CHAIN_LEN;
                }
            }
        }

        if (ivp_example_inspector_tick(&inspector)) {
            ivp_vec3r_t wp;
            ivp_object_get_position(weight, &wp);
            ivp_example_inspector_log(&inspector,
                "max_speed=%.3f stretch_ratio=%.3f worst_link=%d weight_x=%.3f swing_x_peak=%.3f chain_mean_x=%.3f",
                max_speed,
                max_stretch_ratio,
                max_stretch_link,
                (float)wp.k[0],
                weight_peak_abs_x,
                sum_link_x / (float)CHAIN_LEN);
        }

        if (max_speed > 35.0f || max_stretch_ratio > 4.0f) {
            ivp_example_inspector_log(&inspector,
                "instability flag (speed=%.3f ratio=%.3f)",
                max_speed, max_stretch_ratio);
        }

        /* Grid & ground */
        ivp_draw_ground_grid(renderer, 12.0f, 2.0f, c_grid);
        {
            float gp[3] = {0, 2.0f, 0}, gs[3] = {12.0f, 0.5f, 12.0f};
            ivp_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        /* Ceiling bar */
        {
            ivp_vec3r_t p;
            ivp_object_get_position(ceiling, &p);
            float fp[3]; vec3r_to_float3(&p, fp);
            float hs[3] = {1.5f, 0.3f, 0.3f};
            ivp_draw_wire_box(renderer, fp, hs, NULL, c_ceil);
        }

        /* Chain links and joint connectors */
        float prev_pos[3];
        {
            ivp_vec3r_t p;
            ivp_object_get_position(ceiling, &p);
            vec3r_to_float3(&p, prev_pos);
        }
        for (int i = 0; i < CHAIN_LEN; i++) {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(links[i], &p);
            ivp_object_get_orientation(links[i], &q);

            float rot[9]; quat_to_col_rot(&q, rot);
            float fp[3]; vec3r_to_float3(&p, fp);
            float hs[3] = {LINK_HX, LINK_HY, LINK_HZ};

            /* Alternate link colors slightly */
            float lc[3] = {c_link[0], c_link[1], c_link[2]};
            if (i % 2) { lc[0] *= 0.8f; lc[1] *= 0.8f; lc[2] *= 0.8f; }

            ivp_draw_wire_box(renderer, fp, hs, rot, lc);

            /* Joint connector line */
            ivp_draw_thick_line(renderer, prev_pos, fp, c_joint, 2);

            /* Small sphere at joint */
            float jp[3] = {
                (prev_pos[0] + fp[0]) * 0.5f,
                (prev_pos[1] + fp[1]) * 0.5f,
                (prev_pos[2] + fp[2]) * 0.5f
            };
            ivp_draw_wire_sphere(renderer, jp, 0.08f, c_joint);

            prev_pos[0] = fp[0]; prev_pos[1] = fp[1]; prev_pos[2] = fp[2];
        }

        /* Weight */
        {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(weight, &p);
            ivp_object_get_orientation(weight, &q);

            float rot[9]; quat_to_col_rot(&q, rot);
            float fp[3]; vec3r_to_float3(&p, fp);
            float hs[3] = {0.6f, 0.6f, 0.6f};

            ivp_draw_thick_line(renderer, prev_pos, fp, c_joint, 2);
            ivp_draw_wire_box(renderer, fp, hs, rot, c_weight);

            /* Velocity arrow on weight */
            ivp_vec3r_t v;
            ivp_object_get_speed(weight, &v);
            float speed = vec3r_speed(&v);
            if (speed > 0.3f) {
                float sc = 0.4f;
                float to[3] = {fp[0]+(float)v.k[0]*sc,
                               fp[1]+(float)v.k[1]*sc,
                               fp[2]+(float)v.k[2]*sc};
                ivp_draw_arrow(renderer, fp, to, c_vel, 0.25f);
            }
        }

        /* Origin axes */
        {
            float org[3] = {0, 0, 0};
            ivp_draw_axes(renderer, org, NULL, 1.5f);
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus, c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  Chain: %d links + weight",
                     sl.fps_display, CHAIN_LEN);
            ivp_draw_text_2d(renderer, 10.0f, 10.0f, buf, c_hud);

            ivp_vec3r_t wp;
            ivp_object_get_position(weight, &wp);
            ivp_vec3r_t wv;
            ivp_object_get_speed(weight, &wv);
            float wspeed = vec3r_speed(&wv);
            snprintf(buf, sizeof(buf), "Weight pos: (%.1f, %.1f, %.1f)  speed: %.2f m/s",
                     (float)wp.k[0], (float)wp.k[1], (float)wp.k[2], wspeed);
            ivp_draw_text_2d(renderer, 10.0f, 30.0f, buf, c_weight);

            ivp_example_draw_interaction_hud(renderer, 10.0f, 50.0f, c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, weight_surf);
    ivp_compact_surface_free(NULL, ceil_surf);
    ivp_compact_surface_free(NULL, link_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
