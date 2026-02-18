/* example_forcefield.c -- Force field with mode switching
 *
 * Multiple objects of varying masses under a force field.
 * Press 1/2/3 to switch between CONSTANT, FORCE, and DRAG modes.
 *   CONSTANT: uniform acceleration — all objects move equally.
 *   FORCE:    constant force — lighter objects accelerate faster.
 *   DRAG:     velocity-proportional resistance slows moving objects.
 *
 * Demonstrates: ivp_forcefield, three force field modes, mass-dependent
 *               behavior, keyboard input, HUD.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/controllers/ivp_forcefield.h>

#include <SDL3/SDL.h>
#include <stdio.h>

#define N_OBJECTS 24

typedef struct {
    ivp_object_t *obj;
    float col[3];
    float mass;
    float hs[3];
    bool  is_sphere;
    float radius;
} ff_obj_t;

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Force Field -- Mode Switching", .vsync = true
    };
    ivp_renderer_t *renderer = ivp_renderer_create(&rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -5.0f;
    cam.orbit_distance  = 30.0f;
    cam.orbit_yaw       = 25.0f;
    cam.orbit_pitch     = -25.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "forcefield");

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 18.0f, 0.5f, 18.0f);
    ivp_surman_polygon_t sm_ground;
    create_ground_object(env, ground_surf, &sm_ground, IVP_REAL(0.0));

    /* ── Objects: cubes and spheres of varying mass ──────────────────────── */
    ivp_compact_surface_t *cube_s =
        ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    ivp_compact_surface_t *cube_m =
        ivp_build_box_surface(NULL, 0.7f, 0.7f, 0.7f);
    ivp_compact_surface_t *cube_l =
        ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);

    ivp_surman_polygon_t sm_poly[N_OBJECTS];
    ivp_surman_ball_t    sm_ball[N_OBJECTS];
    ff_obj_t objs[N_OBJECTS];
    int n_obj = 0;

    /* Light cubes (mass 0.5) — red */
    for (int i = 0; i < 8; i++) {
        ivp_surman_polygon_init(&sm_poly[n_obj], cube_s);
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.type = IVP_OBJECT_POLYGON;
        t.mass = IVP_REAL(0.5);
        set_box_inertia(&t, IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(0.5));
        t.material.friction   = IVP_REAL(0.5);
        t.material.elasticity = IVP_REAL(0.3);
        float angle = 6.2832f * (float)i / 8.0f;
        float r = 5.0f;
        t.position = ivp_vec3r(
            (ivp_real_t)(r * cosf(angle)),
            IVP_REAL(-3.0),
            (ivp_real_t)(r * sinf(angle)));
        objs[n_obj].obj = ivp_environment_create_object(env, &t, &sm_poly[n_obj].base);
        objs[n_obj].col[0] = 1.0f; objs[n_obj].col[1] = 0.3f; objs[n_obj].col[2] = 0.2f;
        objs[n_obj].mass = 0.5f;
        objs[n_obj].hs[0] = objs[n_obj].hs[1] = objs[n_obj].hs[2] = 0.5f;
        objs[n_obj].is_sphere = false;
        n_obj++;
    }

    /* Medium cubes (mass 3.0) — green */
    for (int i = 0; i < 8; i++) {
        ivp_surman_polygon_init(&sm_poly[n_obj], cube_m);
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.type = IVP_OBJECT_POLYGON;
        t.mass = IVP_REAL(3.0);
        set_box_inertia(&t, IVP_REAL(3.0), IVP_REAL(0.7), IVP_REAL(0.7), IVP_REAL(0.7));
        t.material.friction   = IVP_REAL(0.5);
        t.material.elasticity = IVP_REAL(0.3);
        float angle = 6.2832f * (float)i / 8.0f + 0.39f;
        float r = 8.0f;
        t.position = ivp_vec3r(
            (ivp_real_t)(r * cosf(angle)),
            IVP_REAL(-3.0),
            (ivp_real_t)(r * sinf(angle)));
        objs[n_obj].obj = ivp_environment_create_object(env, &t, &sm_poly[n_obj].base);
        objs[n_obj].col[0] = 0.3f; objs[n_obj].col[1] = 1.0f; objs[n_obj].col[2] = 0.3f;
        objs[n_obj].mass = 3.0f;
        objs[n_obj].hs[0] = objs[n_obj].hs[1] = objs[n_obj].hs[2] = 0.7f;
        objs[n_obj].is_sphere = false;
        n_obj++;
    }

    /* Heavy spheres (mass 10.0) — blue */
    for (int i = 0; i < 8; i++) {
        ivp_surman_ball_init(&sm_ball[i], 0.8f);
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.type = IVP_OBJECT_BALL;
        t.mass = IVP_REAL(10.0);
        set_sphere_inertia(&t, IVP_REAL(10.0), IVP_REAL(0.8));
        t.material.friction   = IVP_REAL(0.5);
        t.material.elasticity = IVP_REAL(0.3);
        float angle = 6.2832f * (float)i / 8.0f + 0.78f;
        float r = 11.0f;
        t.position = ivp_vec3r(
            (ivp_real_t)(r * cosf(angle)),
            IVP_REAL(-3.0),
            (ivp_real_t)(r * sinf(angle)));
        objs[n_obj].obj = ivp_environment_create_object(env, &t, &sm_ball[i].base);
        objs[n_obj].col[0] = 0.3f; objs[n_obj].col[1] = 0.5f; objs[n_obj].col[2] = 1.0f;
        objs[n_obj].mass = 10.0f;
        objs[n_obj].is_sphere = true;
        objs[n_obj].radius = 0.8f;
        n_obj++;
    }

    /* ── Force field: start in CONSTANT mode, upward (anti-gravity) ─────── */
    ivp_forcefield_state_t ff_state;
    ivp_vec3r_t ff_val = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    ivp_forcefield_state_init(&ff_state, IVP_FORCEFIELD_CONSTANT, &ff_val);
    ivp_environment_add_controller(env, 100, ivp_controller_forcefield, &ff_state);

    int current_mode = 0;
    const char *mode_names[] = {"REDUCED GRAVITY", "HORIZONTAL WIND", "VISCOUS DRAG"};
    bool prev_key1 = false, prev_key2 = false, prev_key3 = false;

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_ground[] = {0.22f, 0.24f, 0.30f};
    const float c_grid[]   = {0.38f, 0.43f, 0.52f};
    const float c_hud[]    = {0.95f, 0.97f, 1.0f};
    const float c_vel[]    = {0.2f, 1.0f, 0.4f};
    const float c_drag[]   = {1.0f, 0.85f, 0.25f};
    const float c_focus[]  = {1.0f, 0.65f, 0.25f};
    const float c_mode[]   = {1.0f, 1.0f, 0.2f};

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

        /* Keyboard: mode switching */
        const bool *keys = SDL_GetKeyboardState(NULL);
        bool k1 = keys[SDL_SCANCODE_1];
        bool k2 = keys[SDL_SCANCODE_2];
        bool k3 = keys[SDL_SCANCODE_3];

        if (k1 && !prev_key1) {
            /* REDUCED GRAVITY: upward acceleration partially cancels gravity.
             * Net effect: ~3.8 m/s² down (all objects equally). */
            current_mode = 0;
            ivp_vec3r_t v = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-6.0), IVP_REAL(0.0));
            ivp_forcefield_state_init(&ff_state, IVP_FORCEFIELD_CONSTANT, &v);
        }
        if (k2 && !prev_key2) {
            /* HORIZONTAL WIND: constant force in +X direction.
             * Light objects (0.5kg) accelerate at 30 m/s², heavy (10kg) at 1.5 m/s².
             * Demonstrates mass-dependent behavior clearly. */
            current_mode = 1;
            ivp_vec3r_t v = ivp_vec3r(IVP_REAL(15.0), IVP_REAL(0.0), IVP_REAL(0.0));
            ivp_forcefield_state_init(&ff_state, IVP_FORCEFIELD_FORCE, &v);
        }
        if (k3 && !prev_key3) {
            /* VISCOUS DRAG: velocity-proportional resistance.
             * Coefficient 5 for clearly visible slowdown effect. */
            current_mode = 2;
            ivp_vec3r_t v = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(5.0), IVP_REAL(5.0));
            ivp_forcefield_state_init(&ff_state, IVP_FORCEFIELD_DRAG, &v);
        }
        prev_key1 = k1; prev_key2 = k2; prev_key3 = k3;

        sim_loop_step(&sl, env, frame_dt);

        /* Grid & ground */
        ivp_draw_ground_grid(renderer, 18.0f, 2.0f, c_grid);
        {
            float gp[3] = {0, 0, 0}, gs[3] = {18.0f, 0.5f, 18.0f};
            ivp_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        /* Objects */
        float max_speed = 0.0f;
        float avg_height = 0.0f;
        int airborne = 0;
        for (int i = 0; i < n_obj; i++) {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(objs[i].obj, &p);
            ivp_object_get_orientation(objs[i].obj, &q);
            float fp[3]; vec3r_to_float3(&p, fp);

            if (objs[i].is_sphere) {
                ivp_draw_wire_sphere(renderer, fp, objs[i].radius, objs[i].col);
            } else {
                float rot[9]; quat_to_col_rot(&q, rot);
                ivp_draw_wire_box(renderer, fp, objs[i].hs, rot, objs[i].col);
            }

            ivp_vec3r_t v;
            ivp_object_get_speed(objs[i].obj, &v);
            float speed = vec3r_speed(&v);
            if (speed > max_speed) max_speed = speed;
            avg_height += -(float)p.k[1];
            if ((float)p.k[1] < -1.0f) airborne++;

            if (speed > 0.5f) {
                float sc = 0.3f;
                float to[3] = {fp[0]+(float)v.k[0]*sc,
                               fp[1]+(float)v.k[1]*sc,
                               fp[2]+(float)v.k[2]*sc};
                ivp_draw_arrow(renderer, fp, to, c_vel, 0.2f);
            }
        }
        avg_height /= (float)n_obj;

        /* Axes */
        {
            float org[3] = {0, 0, 0};
            ivp_draw_axes(renderer, org, NULL, 1.5f);
        }

        if (ivp_example_inspector_tick(&inspector)) {
            ivp_example_inspector_log(&inspector,
                "mode=%s max_speed=%.3f avg_height=%.3f airborne=%d/%d",
                mode_names[current_mode], max_speed, avg_height,
                airborne, n_obj);
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus,
                                              c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  Objects: %d  Time: %.1fs",
                     sl.fps_display, n_obj, (float)sl.t_prev);
            ivp_draw_text_2d(renderer, 10.0f, 10.0f, buf, c_hud);

            snprintf(buf, sizeof(buf), "Mode: %s  [1]Gravity [2]Wind [3]Drag",
                     mode_names[current_mode]);
            ivp_draw_text_2d(renderer, 10.0f, 30.0f, buf, c_mode);

            /* Mode description */
            const char *desc[] = {
                "Upward accel -6 m/s^2 (net ~3.8 down, all equal)",
                "Horizontal force +X 15N (light blown, heavy resist)",
                "Drag coeff 5 (moving objects slow down)"
            };
            ivp_draw_text_2d(renderer, 10.0f, 48.0f, desc[current_mode], c_hud);

            snprintf(buf, sizeof(buf),
                "Red=0.5kg  Green=3kg  Blue=10kg  Airborne: %d", airborne);
            ivp_draw_text_2d(renderer, 10.0f, 66.0f, buf, c_hud);

            ivp_example_draw_interaction_hud(renderer, 10.0f, 88.0f,
                                             c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, cube_l);
    ivp_compact_surface_free(NULL, cube_m);
    ivp_compact_surface_free(NULL, cube_s);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
