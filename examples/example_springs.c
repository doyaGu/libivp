/* example_springs.c -- Spring lattice and trampoline
 *
 * Multiple objects connected by springs in a network.  A heavy sphere
 * is dropped onto a spring-bridge structure.  Force arrows show spring
 * tension.  An anchored spring acts as a bungee cord.
 *
 * Demonstrates: ivp_actuator_spring, multiple springs, stiffness/damping,
 *               rest length, sphere, force visualization, HUD.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/base/ivp_types.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/controllers/ivp_actuator_spring.h>

#include <stdio.h>

/* ── Objects ────────────────────────────────────────────────────────────── */
#define N_BRIDGE 4        /* bridge nodes */
#define MAX_SPRINGS 16

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Springs -- Bridge & Bungee", .vsync = true
    };
    ivp_renderer_t *renderer = ivp_renderer_create(&rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -4.0f;
    cam.orbit_distance  = 28.0f;
    cam.orbit_yaw       = 25.0f;
    cam.orbit_pitch     = -20.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    /* Standard Earth gravity (Y-down). Previous value 3.0 was too weak/floaty. */
    ivp_vec3r_set(&ecfg.gravity, IVP_REAL(0.0), IVP_REAL(9.81), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "springs");

    /* ── Ground ─────────────────────────────────────────────────────────── */
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

    /* ── Bridge pillars (static) ────────────────────────────────────────── */
    ivp_compact_surface_t *pillar_surf =
        ivp_build_box_surface(NULL, 0.5f, 2.0f, 0.5f);
    ivp_surman_polygon_t sm_pil_l, sm_pil_r;
    ivp_surman_polygon_init(&sm_pil_l, pillar_surf);
    ivp_surman_polygon_init(&sm_pil_r, pillar_surf);

    ivp_object_template_t pilt;
    ivp_object_template_init(&pilt);
    pilt.type = IVP_OBJECT_POLYGON;
    pilt.mass = 0.0f;
    pilt.physical_unmoveable = true;
    pilt.position = ivp_vec3r(IVP_REAL(-6.0), IVP_REAL(-7.0), IVP_REAL(0.0));
    ivp_object_t *pillar_l = ivp_environment_create_object(env, &pilt, &sm_pil_l.base);
    pilt.position = ivp_vec3r(IVP_REAL(6.0), IVP_REAL(-7.0), IVP_REAL(0.0));
    ivp_object_t *pillar_r = ivp_environment_create_object(env, &pilt, &sm_pil_r.base);

    /* ── Bridge nodes (dynamic cubes hanging on springs) ────────────────── */
    ivp_compact_surface_t *node_surf =
        ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    ivp_surman_polygon_t sm_nodes[N_BRIDGE];
    ivp_object_t *nodes[N_BRIDGE];

    float node_spacing = 12.0f / (float)(N_BRIDGE + 1);
    for (int i = 0; i < N_BRIDGE; i++) {
        ivp_surman_polygon_init(&sm_nodes[i], node_surf);
        ivp_object_template_t nt;
        ivp_object_template_init(&nt);
        nt.type = IVP_OBJECT_POLYGON;
        nt.mass = IVP_REAL(0.6);
        /* 0.5m half-extent cube, 0.6kg: I_x = (m/3)(hy^2+hz^2) = 0.1 */
        set_box_inertia(&nt, IVP_REAL(0.6), IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(0.5));
        nt.material.friction   = IVP_REAL(0.7);
        nt.material.elasticity = IVP_REAL(0.05);
        nt.speed_damp_factor     = IVP_REAL(0.15);
        nt.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        float nx = -6.0f + node_spacing * (float)(i + 1);
        nt.position = ivp_vec3r((ivp_real_t)nx, IVP_REAL(-7.0), IVP_REAL(0.0));
        nodes[i] = ivp_environment_create_object(env, &nt, &sm_nodes[i].base);
    }

    /* ── Heavy sphere (dropped onto bridge) ─────────────────────────────── */
    ivp_surman_ball_t sm_ball;
    ivp_surman_ball_init(&sm_ball, 0.8f);

    ivp_object_template_t bt;
    ivp_object_template_init(&bt);
    bt.type = IVP_OBJECT_BALL;
    bt.mass = IVP_REAL(0.5);
    /* Sphere I = (2/5) m r^2 = 0.4 * 0.5 * 0.64 = 0.128 */
    set_sphere_inertia(&bt, IVP_REAL(0.5), IVP_REAL(0.8));
    bt.material.friction   = IVP_REAL(0.5);
    bt.material.elasticity = IVP_REAL(0.0);
    bt.speed_damp_factor = IVP_REAL(0.0);
    bt.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    bt.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-10.0), IVP_REAL(0.0));
    ivp_object_t *ball = ivp_environment_create_object(env, &bt, &sm_ball.base);
    /* Initial downward velocity (~0.5m free-fall equivalent) for natural drop */
    ball->core.speed = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    /* ── Bungee cube (attached to ceiling spring) ───────────────────────── */
    ivp_compact_surface_t *bungee_surf =
        ivp_build_box_surface(NULL, 0.6f, 0.6f, 0.6f);
    ivp_surman_polygon_t sm_bungee;
    ivp_surman_polygon_init(&sm_bungee, bungee_surf);

    ivp_object_template_t bgt;
    ivp_object_template_init(&bgt);
    bgt.type = IVP_OBJECT_POLYGON;
    bgt.mass = IVP_REAL(3.0);
    /* 0.6m half-extent cube, 3kg: I_x = (m/3)(hy^2+hz^2) = 0.24 */
    set_box_inertia(&bgt, IVP_REAL(3.0), IVP_REAL(0.6), IVP_REAL(0.6), IVP_REAL(0.6));
    bgt.material.friction   = IVP_REAL(0.6);
    bgt.material.elasticity = IVP_REAL(0.0);
    bgt.speed_damp_factor = IVP_REAL(0.0);
    bgt.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    bgt.position = ivp_vec3r(IVP_REAL(10.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    ivp_object_t *bungee = ivp_environment_create_object(env, &bgt, &sm_bungee.base);

    /* ── Static ceiling anchor for bungee ───────────────────────────────── */
    ivp_compact_surface_t *anchor_surf =
        ivp_build_box_surface(NULL, 0.3f, 0.3f, 0.3f);
    ivp_surman_polygon_t sm_anchor;
    ivp_surman_polygon_init(&sm_anchor, anchor_surf);

    ivp_object_template_t at;
    ivp_object_template_init(&at);
    at.type = IVP_OBJECT_POLYGON;
    at.mass = 0.0f;
    at.physical_unmoveable = true;
    at.position = ivp_vec3r(IVP_REAL(10.0), IVP_REAL(-14.0), IVP_REAL(0.0));
    ivp_object_t *anchor = ivp_environment_create_object(env, &at, &sm_anchor.base);

    const float ball_y0 = -10.0f;
    const float bungee_y0 = -6.0f;
    float ball_min_y_seen = ball_y0;
    float ball_max_y_seen = ball_y0;
    float bungee_min_y_seen = bungee_y0;
    float bungee_max_y_seen = bungee_y0;
    float ball_max_up_v = 0.0f;
    float ball_max_down_v = 0.0f;
    float bungee_max_up_v = 0.0f;
    float bungee_max_down_v = 0.0f;
    float max_bridge_sag = 0.0f;
    float max_bungee_extension = 0.0f;

    /* ── Springs ────────────────────────────────────────────────────────── */
    ivp_spring_state_t springs[MAX_SPRINGS];
    int n_springs = 0;

    /* Bridge springs: pillar_l → node[0] → ... → node[N-1] → pillar_r */
    {
        ivp_vec3r_t a_pt = ivp_vec3r(IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t b_pt = ivp_vec3r(IVP_REAL(-0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_real_t bridge_rest_len = (ivp_real_t)node_spacing - IVP_REAL(1.0);
        if (bridge_rest_len < IVP_REAL(0.3)) bridge_rest_len = IVP_REAL(0.3);

        /* Bridge springs: k=400 N/m gives ~2cm sag under gravity for 4x0.6kg nodes.
         * Damping ratio ~0.3 (underdamped, visible oscillation). */

        /* Left pillar → first node */
        ivp_spring_state_init(&springs[n_springs]);
        ivp_spring_state_setup(&springs[n_springs],
            pillar_l, &a_pt, nodes[0], &b_pt,
            IVP_REAL(400.0), IVP_REAL(50.0), bridge_rest_len);
        springs[n_springs].max_force = IVP_REAL(80.0);
        ivp_environment_add_controller(env, 100, ivp_controller_spring, &springs[n_springs]);
        n_springs++;

        /* Inter-node springs */
        ivp_vec3r_t na_pt = ivp_vec3r(IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t nb_pt = ivp_vec3r(IVP_REAL(-0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        for (int i = 1; i < N_BRIDGE; i++) {
            ivp_spring_state_init(&springs[n_springs]);
            ivp_spring_state_setup(&springs[n_springs],
                nodes[i-1], &na_pt, nodes[i], &nb_pt,
                IVP_REAL(400.0), IVP_REAL(50.0), bridge_rest_len);
            springs[n_springs].max_force = IVP_REAL(80.0);
            ivp_environment_add_controller(env, 100, ivp_controller_spring, &springs[n_springs]);
            n_springs++;
        }

        /* Last node → right pillar */
        ivp_spring_state_init(&springs[n_springs]);
        ivp_vec3r_t rp = ivp_vec3r(IVP_REAL(-0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_spring_state_setup(&springs[n_springs],
            nodes[N_BRIDGE-1], &na_pt, pillar_r, &rp,
            IVP_REAL(400.0), IVP_REAL(50.0), bridge_rest_len);
        springs[n_springs].max_force = IVP_REAL(80.0);
        ivp_environment_add_controller(env, 100, ivp_controller_spring, &springs[n_springs]);
        n_springs++;
    }

    /* Bungee spring: anchor → bungee cube */
    {
        ivp_vec3r_t a_pt = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.3), IVP_REAL(0.0));
        ivp_vec3r_t b_pt = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.6), IVP_REAL(0.0));

        /* Bungee: k=26 N/m supports 3kg at ~1.1m extension under Earth gravity.
         * Damping ~12 for moderate energy dissipation. */
        ivp_spring_state_init(&springs[n_springs]);
        ivp_spring_state_setup(&springs[n_springs],
            anchor, &a_pt, bungee, &b_pt,
            IVP_REAL(26.0), IVP_REAL(12.0), IVP_REAL(8.0));
        springs[n_springs].max_force = IVP_REAL(60.0);
        ivp_environment_add_controller(env, 100, ivp_controller_spring, &springs[n_springs]);
        n_springs++;
    }

    ivp_real_t bridge_rest_ref = springs[0].rest_length;

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_node[]    = {1.0f, 0.5f, 0.1f};
    const float c_ball[]    = {0.2f, 0.8f, 1.0f};
    const float c_bungee[]  = {0.9f, 0.3f, 0.9f};
    const float c_anchor[]  = {0.5f, 0.5f, 0.5f};
    const float c_pillar[]  = {0.45f, 0.40f, 0.35f};
    const float c_spring[]  = {1.0f, 1.0f, 0.2f};
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
        for (int i = 0; i < N_BRIDGE; i++) {
            ivp_vec3r_t v;
            ivp_object_get_speed(nodes[i], &v);
            float speed = vec3r_speed(&v);
            if (speed > max_speed) max_speed = speed;
        }
        {
            ivp_vec3r_t vb, vg;
            ivp_object_get_speed(ball, &vb);
            ivp_object_get_speed(bungee, &vg);
            float sb = vec3r_speed(&vb);
            float sg = vec3r_speed(&vg);
            if (sb > max_speed) max_speed = sb;
            if (sg > max_speed) max_speed = sg;

            float ball_vy = (float)vb.k[1];
            float bungee_vy = (float)vg.k[1];
            if (ball_vy < 0.0f && -ball_vy > ball_max_up_v) ball_max_up_v = -ball_vy;
            if (ball_vy > ball_max_down_v) ball_max_down_v = ball_vy;
            if (bungee_vy < 0.0f && -bungee_vy > bungee_max_up_v) bungee_max_up_v = -bungee_vy;
            if (bungee_vy > bungee_max_down_v) bungee_max_down_v = bungee_vy;
        }

        float max_bridge_ratio = 0.0f;
        for (int i = 0; i < N_BRIDGE + 1; i++) {
            float rest = (float)springs[i].rest_length;
            float curr = (float)springs[i].current_length;
            float ratio = (rest > 1e-5f) ? (curr / rest) : 0.0f;
            if (ratio > max_bridge_ratio) max_bridge_ratio = ratio;
        }

        float max_bridge_force_abs = 0.0f;
        for (int i = 0; i < N_BRIDGE + 1; i++) {
            float f = (float)springs[i].current_force;
            float af = (f >= 0.0f) ? f : -f;
            if (af > max_bridge_force_abs) max_bridge_force_abs = af;
        }

        float bridge_mid_y = 0.0f;
        for (int i = 0; i < N_BRIDGE; i++) {
            ivp_vec3r_t pn;
            ivp_object_get_position(nodes[i], &pn);
            bridge_mid_y += (float)pn.k[1];
        }
        bridge_mid_y /= (float)N_BRIDGE;
        float bridge_sag = bridge_mid_y - (-7.0f);
        if (bridge_sag > max_bridge_sag) max_bridge_sag = bridge_sag;

        float bungee_extension = (float)(springs[N_BRIDGE + 1].current_length - springs[N_BRIDGE + 1].rest_length);
        if (bungee_extension > max_bungee_extension) max_bungee_extension = bungee_extension;

        ivp_vec3r_t dbg_ball_pos, dbg_bungee_pos, dbg_ball_vel, dbg_bungee_vel;
        ivp_object_get_position(ball, &dbg_ball_pos);
        ivp_object_get_position(bungee, &dbg_bungee_pos);
        ivp_object_get_speed(ball, &dbg_ball_vel);
        ivp_object_get_speed(bungee, &dbg_bungee_vel);

        float ball_y = (float)dbg_ball_pos.k[1];
        float bungee_y = (float)dbg_bungee_pos.k[1];
        if (ball_y < ball_min_y_seen) ball_min_y_seen = ball_y;
        if (ball_y > ball_max_y_seen) ball_max_y_seen = ball_y;
        if (bungee_y < bungee_min_y_seen) bungee_min_y_seen = bungee_y;
        if (bungee_y > bungee_max_y_seen) bungee_max_y_seen = bungee_y;

        float ball_up_lift = ball_y0 - ball_min_y_seen;
        float bungee_up_lift = bungee_y0 - bungee_min_y_seen;

        if (ivp_example_inspector_tick(&inspector)) {
            ivp_example_inspector_log(&inspector,
                     "max_speed=%.3f bridge_ratio=%.3f bridge_sag=%.3f bridge_sag_max=%.3f bridge|F|max=%.3f bungee_ext=%.3f bungee_ext_max=%.3f ball{y=%.3f vy=%.3f upLift=%.3f upVmax=%.3f downVmax=%.3f} bungee{y=%.3f vy=%.3f upLift=%.3f upVmax=%.3f downVmax=%.3f}",
                   max_speed,
                   max_bridge_ratio,
                     bridge_sag,
                     max_bridge_sag,
                   max_bridge_force_abs,
                     bungee_extension,
                     max_bungee_extension,
                   ball_y,
                   (float)dbg_ball_vel.k[1],
                   ball_up_lift,
                   ball_max_up_v,
                   ball_max_down_v,
                   bungee_y,
                   (float)dbg_bungee_vel.k[1],
                   bungee_up_lift,
                   bungee_max_up_v,
                   bungee_max_down_v);
        }

        /* Diagnostics only; avoid mutating simulation state in debug path. */

        /* Grid & ground */
        ivp_draw_ground_grid(renderer, 15.0f, 2.0f, c_grid);
        {
            float gp[3] = {0,0,0}, gs[3] = {15.0f, 0.5f, 15.0f};
            ivp_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        /* Pillars */
        {
            ivp_vec3r_t p;
            ivp_object_get_position(pillar_l, &p);
            float fp[3]; vec3r_to_float3(&p, fp);
            float hs[3] = {0.5f, 2.0f, 0.5f};
            ivp_draw_wire_box(renderer, fp, hs, NULL, c_pillar);

            ivp_object_get_position(pillar_r, &p);
            vec3r_to_float3(&p, fp);
            ivp_draw_wire_box(renderer, fp, hs, NULL, c_pillar);
        }

        /* Bridge nodes + inter-node spring lines */
        float node_positions[N_BRIDGE][3];
        for (int i = 0; i < N_BRIDGE; i++) {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(nodes[i], &p);
            ivp_object_get_orientation(nodes[i], &q);
            float rot[9]; quat_to_col_rot(&q, rot);

            vec3r_to_float3(&p, node_positions[i]);

            float hs[3] = {0.5f, 0.5f, 0.5f};
            ivp_draw_wire_box(renderer, node_positions[i], hs, rot, c_node);
        }

        /* Spring lines for bridge */
        {
            ivp_vec3r_t pl; ivp_object_get_position(pillar_l, &pl);
            float lp[3]; vec3r_to_float3(&pl, lp);
            ivp_draw_thick_line(renderer, lp, node_positions[0], c_spring, 2);

            for (int i = 1; i < N_BRIDGE; i++)
                ivp_draw_thick_line(renderer, node_positions[i-1], node_positions[i], c_spring, 2);

            ivp_vec3r_t pr; ivp_object_get_position(pillar_r, &pr);
            float rp[3]; vec3r_to_float3(&pr, rp);
            ivp_draw_thick_line(renderer, node_positions[N_BRIDGE-1], rp, c_spring, 2);
        }

        /* Heavy ball */
        {
            ivp_vec3r_t p;
            ivp_object_get_position(ball, &p);
            float fp[3]; vec3r_to_float3(&p, fp);
            ivp_draw_wire_sphere(renderer, fp, 0.8f, c_ball);

            /* Velocity arrow */
            ivp_vec3r_t v;
            ivp_object_get_speed(ball, &v);
            float spd = vec3r_speed(&v);
            if (spd > 0.3f) {
                float sc = 0.3f;
                float to[3] = {fp[0]+(float)v.k[0]*sc, fp[1]+(float)v.k[1]*sc, fp[2]+(float)v.k[2]*sc};
                ivp_draw_arrow(renderer, fp, to, c_vel, 0.2f);
            }
        }

        /* Bungee anchor + cube + spring line */
        {
            ivp_vec3r_t pa; ivp_object_get_position(anchor, &pa);
            float ap[3]; vec3r_to_float3(&pa, ap);
            float ahs[3] = {0.3f, 0.3f, 0.3f};
            ivp_draw_wire_box(renderer, ap, ahs, NULL, c_anchor);

            ivp_vec3r_t pb; ivp_quatr_t qb;
            ivp_object_get_position(bungee, &pb);
            ivp_object_get_orientation(bungee, &qb);
            float rot[9]; quat_to_col_rot(&qb, rot);
            float bp[3]; vec3r_to_float3(&pb, bp);
            float bhs[3] = {0.6f, 0.6f, 0.6f};
            ivp_draw_wire_box(renderer, bp, bhs, rot, c_bungee);

            /* Bungee cord */
            ivp_draw_thick_line(renderer, ap, bp, c_spring, 2);

            /* Velocity arrow */
            ivp_vec3r_t v;
            ivp_object_get_speed(bungee, &v);
            float spd = vec3r_speed(&v);
            if (spd > 0.3f) {
                float sc = 0.3f;
                float to[3] = {bp[0]+(float)v.k[0]*sc, bp[1]+(float)v.k[1]*sc, bp[2]+(float)v.k[2]*sc};
                ivp_draw_arrow(renderer, bp, to, c_vel, 0.2f);
            }
        }

        /* Axes */
        {
            float org[3] = {0, 0, 0};
            ivp_draw_axes(renderer, org, NULL, 1.5f);
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus, c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  Springs: %d  Time: %.1fs",
                     sl.fps_display, n_springs, (float)sl.t_prev);
            ivp_draw_text_2d(renderer, 10.0f, 10.0f, buf, c_hud);

            ivp_vec3r_t bp; ivp_object_get_position(ball, &bp);
            ivp_vec3r_t bv; ivp_object_get_speed(ball, &bv);
            float bspd = vec3r_speed(&bv);
            snprintf(buf, sizeof(buf), "Ball: (%.1f,%.1f,%.1f) speed:%.1f",
                     (float)bp.k[0], (float)bp.k[1], (float)bp.k[2], bspd);
            ivp_draw_text_2d(renderer, 10.0f, 30.0f, buf, c_ball);

            ivp_vec3r_t bgp; ivp_object_get_position(bungee, &bgp);
            snprintf(buf, sizeof(buf), "Bungee: (%.1f,%.1f,%.1f)",
                     (float)bgp.k[0], (float)bgp.k[1], (float)bgp.k[2]);
            ivp_draw_text_2d(renderer, 10.0f, 48.0f, buf, c_bungee);

            ivp_example_draw_interaction_hud(renderer, 10.0f, 70.0f, c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, bungee_surf);
    ivp_compact_surface_free(NULL, anchor_surf);
    ivp_compact_surface_free(NULL, node_surf);
    ivp_compact_surface_free(NULL, pillar_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
