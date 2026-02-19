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
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/controllers/ivp_controller_motion.h>

#include <SDL3/SDL.h>
#include <stdio.h>

/* Wrapper: only apply motion to the cue object */
typedef struct {
    ivp_motion_state_t state;
    ivp_object_t      *target;
} cue_motion_t;

static void cue_motion_controller(const ivp_event_sim_t *event_sim,
                                   ivp_object_t *obj,
                                   void *ctx)
{
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    cue_motion_t *cm = ctx;
    if (obj == cm->target)
        ivp_controller_motion(event_sim, obj, &cm->state);
}

#define N_TABLE_OBJ 8

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Motion Controller -- Velocity/Force/Impulse",
        .vsync = true
    };
    ivp_renderer_t *renderer = ivp_renderer_create(&rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -2.0f;
    cam.orbit_distance  = 25.0f;
    cam.orbit_yaw       = 0.0f;
    cam.orbit_pitch     = -40.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "motion_controller");

    /* ── Table (ground) ─────────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 10.0f, 0.5f, 8.0f);
    ivp_surman_polygon_t sm_ground;
    create_ground_object(env, ground_surf, &sm_ground, IVP_REAL(0.0));

    /* ── Table walls (4 static boxes) ───────────────────────────────────── */
    ivp_compact_surface_t *wall_x_surf =
        ivp_build_box_surface(NULL, 0.3f, 1.0f, 8.0f);
    ivp_compact_surface_t *wall_z_surf =
        ivp_build_box_surface(NULL, 10.0f, 1.0f, 0.3f);

    ivp_surman_polygon_t sm_walls[4];
    ivp_compact_surface_t *wall_surfs[] = {
        wall_x_surf, wall_x_surf, wall_z_surf, wall_z_surf
    };
    ivp_real_t wall_pos[][3] = {
        {IVP_REAL(-10.0), IVP_REAL(-1.0), IVP_REAL(0.0)},
        {IVP_REAL( 10.0), IVP_REAL(-1.0), IVP_REAL(0.0)},
        {IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(-8.0)},
        {IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL( 8.0)}
    };
    for (int i = 0; i < 4; i++) {
        ivp_surman_polygon_init(&sm_walls[i], wall_surfs[i]);
        ivp_object_template_t wt;
        ivp_object_template_init(&wt);
        wt.type = IVP_OBJECT_POLYGON;
        wt.mass = 0.0f;
        wt.physical_unmoveable = true;
        wt.position = ivp_vec3r(wall_pos[i][0], wall_pos[i][1], wall_pos[i][2]);
        ivp_environment_create_object(env, &wt, &sm_walls[i].base);
    }

    /* ── Table objects: cubes and spheres ────────────────────────────────── */
    ivp_compact_surface_t *tbl_cube_surf =
        ivp_build_box_surface(NULL, 0.6f, 0.6f, 0.6f);
    ivp_surman_polygon_t sm_tbl_p[N_TABLE_OBJ];
    ivp_surman_ball_t    sm_tbl_b[N_TABLE_OBJ];
    ivp_object_t *tbl_objs[N_TABLE_OBJ];
    bool tbl_is_sphere[N_TABLE_OBJ];
    int n_tbl = 0;

    const float palette[][3] = {
        {1.0f, 0.4f, 0.2f}, {0.3f, 1.0f, 0.3f}, {0.3f, 0.5f, 1.0f},
        {1.0f, 1.0f, 0.2f}, {1.0f, 0.2f, 0.8f}, {0.2f, 1.0f, 0.9f},
        {0.9f, 0.6f, 0.1f}, {0.6f, 0.3f, 1.0f}
    };

    /* 4 cubes — low friction simulates pool-table felt, moderate elasticity for bouncing */
    for (int i = 0; i < 4; i++) {
        ivp_surman_polygon_init(&sm_tbl_p[n_tbl], tbl_cube_surf);
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.type = IVP_OBJECT_POLYGON;
        t.mass = IVP_REAL(1.5);
        set_box_inertia(&t, IVP_REAL(1.5), IVP_REAL(0.6), IVP_REAL(0.6), IVP_REAL(0.6));
        t.material.friction   = IVP_REAL(0.15);  /* low: pool-table felt */
        t.material.elasticity = IVP_REAL(0.4);   /* moderate bounce off walls */
        t.position = ivp_vec3r(
            IVP_REAL(-3.0) + IVP_REAL(2.0) * (ivp_real_t)i,
            IVP_REAL(-1.1),
            IVP_REAL(3.0));
        tbl_objs[n_tbl] = ivp_environment_create_object(env, &t, &sm_tbl_p[n_tbl].base);
        tbl_is_sphere[n_tbl] = false;
        n_tbl++;
    }

    /* 4 spheres — pool-ball style: low friction, high elasticity */
    for (int i = 0; i < 4; i++) {
        ivp_surman_ball_init(&sm_tbl_b[i], 0.5f);
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.type = IVP_OBJECT_BALL;
        t.mass = IVP_REAL(1.0);
        set_sphere_inertia(&t, IVP_REAL(1.0), IVP_REAL(0.5));
        t.material.friction   = IVP_REAL(0.1);   /* low: smooth pool ball */
        t.material.elasticity = IVP_REAL(0.85);  /* high: crisp ball-to-ball collisions */
        t.position = ivp_vec3r(
            IVP_REAL(-3.0) + IVP_REAL(2.0) * (ivp_real_t)i,
            IVP_REAL(-1.0),
            IVP_REAL(-3.0));
        tbl_objs[n_tbl] = ivp_environment_create_object(env, &t, &sm_tbl_b[i].base);
        tbl_is_sphere[n_tbl] = true;
        n_tbl++;
    }

    /* ── Cue sphere ─────────────────────────────────────────────────────── */
    ivp_surman_ball_t sm_cue;
    ivp_surman_ball_init(&sm_cue, 0.5f);

    ivp_object_template_t cue_t;
    ivp_object_template_init(&cue_t);
    cue_t.type = IVP_OBJECT_BALL;
    cue_t.mass = IVP_REAL(2.0);
    set_sphere_inertia(&cue_t, IVP_REAL(2.0), IVP_REAL(0.5));
    cue_t.material.friction   = IVP_REAL(0.1);   /* smooth cue ball */
    cue_t.material.elasticity = IVP_REAL(0.9);  /* near-elastic collisions */
    cue_t.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    ivp_object_t *cue = ivp_environment_create_object(env, &cue_t, &sm_cue.base);

    /* ── Motion controller for cue ──────────────────────────────────────── */
    cue_motion_t cue_motion;
    ivp_motion_state_init(&cue_motion.state);
    cue_motion.state.max_speed = IVP_REAL(12.0);
    cue_motion.target = cue;
    ivp_environment_add_controller(env, 100,
        cue_motion_controller, &cue_motion);

    int current_mode = 0; /* 0=VELOCITY, 1=FORCE, 2=IMPULSE */
    const char *mode_names[] = {"VELOCITY", "FORCE", "IMPULSE"};
    bool prev_k1 = false, prev_k2 = false, prev_k3 = false;

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_ground[]  = {0.25f, 0.28f, 0.32f};
    const float c_grid[]    = {0.38f, 0.43f, 0.52f};
    const float c_hud[]     = {0.95f, 0.97f, 1.0f};
    const float c_wall[]    = {0.45f, 0.40f, 0.35f};
    const float c_cue[]     = {1.0f, 1.0f, 1.0f};
    const float c_vel[]     = {0.2f, 1.0f, 0.4f};
    const float c_mode[]    = {1.0f, 1.0f, 0.2f};
    const float c_drag[]    = {1.0f, 0.85f, 0.25f};
    const float c_focus[]   = {1.0f, 0.65f, 0.25f};

    ivp_example_dragger_t dragger;
    ivp_example_dragger_init(&dragger);
    ivp_example_dragger_attach(&dragger, env, 100);

    ivp_example_focus_controller_t focus_ctrl;
    ivp_example_focus_init(&focus_ctrl);

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
        ivp_example_focus_update(&focus_ctrl, &cam, env, frame_dt);

        /* Keyboard: mode switching */
        const bool *keys = SDL_GetKeyboardState(NULL);
        bool k1 = keys[SDL_SCANCODE_1];
        bool k2 = keys[SDL_SCANCODE_2];
        bool k3 = keys[SDL_SCANCODE_3];
        if (k1 && !prev_k1) current_mode = 0;
        if (k2 && !prev_k2) current_mode = 1;
        if (k3 && !prev_k3) current_mode = 2;
        prev_k1 = k1; prev_k2 = k2; prev_k3 = k3;

        /* Arrow keys → motion input */
        float mx = 0.0f, mz = 0.0f;
        if (keys[SDL_SCANCODE_LEFT])  mx -= 1.0f;
        if (keys[SDL_SCANCODE_RIGHT]) mx += 1.0f;
        if (keys[SDL_SCANCODE_UP])    mz -= 1.0f;
        if (keys[SDL_SCANCODE_DOWN])  mz += 1.0f;

        /* Apply motion based on mode */
        switch (current_mode) {
        case 0: { /* VELOCITY */
            ivp_vec3r_t v = ivp_vec3r((ivp_real_t)(mx * 8.0f),
                                       IVP_REAL(0.0),
                                       (ivp_real_t)(mz * 8.0f));
            ivp_motion_state_set_velocity(&cue_motion.state, &v);
            break;
        }
        case 1: { /* FORCE */
            ivp_vec3r_t f = ivp_vec3r((ivp_real_t)(mx * 30.0f),
                                       IVP_REAL(0.0),
                                       (ivp_real_t)(mz * 30.0f));
            ivp_motion_state_set_force(&cue_motion.state, &f);
            break;
        }
        case 2: { /* IMPULSE */
            if (mx != 0.0f || mz != 0.0f) {
                ivp_vec3r_t imp = ivp_vec3r((ivp_real_t)(mx * 5.0f),
                                             IVP_REAL(0.0),
                                             (ivp_real_t)(mz * 5.0f));
                ivp_motion_state_set_impulse(&cue_motion.state, &imp);
            }
            break;
        }
        }

        sim_loop_step(&sl, env, frame_dt);

        /* Grid & ground */
        ivp_draw_ground_grid(renderer, 10.0f, 2.0f, c_grid);
        {
            float gp[3] = {0, 0, 0}, gs[3] = {10.0f, 0.5f, 8.0f};
            ivp_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        /* Walls */
        {
            float whs_x[3] = {0.3f, 1.0f, 8.0f};
            float whs_z[3] = {10.0f, 1.0f, 0.3f};
            for (int i = 0; i < 4; i++) {
                float wp[3] = {(float)wall_pos[i][0],
                               (float)wall_pos[i][1],
                               (float)wall_pos[i][2]};
                ivp_draw_wire_box(renderer, wp,
                    (i < 2) ? whs_x : whs_z, NULL, c_wall);
            }
        }

        /* Table objects */
        for (int i = 0; i < n_tbl; i++) {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(tbl_objs[i], &p);
            ivp_object_get_orientation(tbl_objs[i], &q);
            float fp[3]; vec3r_to_float3(&p, fp);
            if (tbl_is_sphere[i]) {
                ivp_draw_wire_sphere(renderer, fp, 0.5f, palette[i % 8]);
            } else {
                float rot[9]; quat_to_col_rot(&q, rot);
                float hs[3] = {0.6f, 0.6f, 0.6f};
                ivp_draw_wire_box(renderer, fp, hs, rot, palette[i % 8]);
            }
        }

        /* Cue sphere */
        {
            ivp_vec3r_t p;
            ivp_object_get_position(cue, &p);
            float fp[3]; vec3r_to_float3(&p, fp);
            ivp_draw_wire_sphere(renderer, fp, 0.5f, c_cue);

            ivp_vec3r_t v;
            ivp_object_get_speed(cue, &v);
            float speed = vec3r_speed(&v);
            if (speed > 0.3f) {
                float sc = 0.3f;
                float to[3] = {fp[0]+(float)v.k[0]*sc,
                               fp[1]+(float)v.k[1]*sc,
                               fp[2]+(float)v.k[2]*sc};
                ivp_draw_arrow(renderer, fp, to, c_vel, 0.2f);
            }
        }

        /* Axes */
        {
            float org[3] = {0, 0, 0};
            ivp_draw_axes(renderer, org, NULL, 1.5f);
        }

        if (ivp_example_inspector_tick(&inspector)) {
            ivp_vec3r_t cv;
            ivp_object_get_speed(cue, &cv);
            ivp_example_inspector_log(&inspector,
                "mode=%s cue_speed=%.3f",
                mode_names[current_mode], vec3r_speed(&cv));
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus_ctrl,
                                              c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  Time: %.1fs",
                     sl.fps_display, (float)sl.t_prev);
            ivp_draw_text_2d(renderer, 10.0f, 10.0f, buf, c_hud);

            snprintf(buf, sizeof(buf), "Mode: %s  [1]VEL [2]FORCE [3]IMPULSE",
                     mode_names[current_mode]);
            ivp_draw_text_2d(renderer, 10.0f, 30.0f, buf, c_mode);

            ivp_draw_text_2d(renderer, 10.0f, 48.0f,
                "Arrow keys: move cue    White sphere = cue", c_cue);

            ivp_example_draw_interaction_hud(renderer, 10.0f, 70.0f,
                                             c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, tbl_cube_surf);
    ivp_compact_surface_free(NULL, wall_z_surf);
    ivp_compact_surface_free(NULL, wall_x_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
