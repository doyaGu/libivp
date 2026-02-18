/* example_check_distance.c -- Distance-triggered rotation motor
 *
 * A pendulum swings above a small box.  When the pendulum tip comes
 * within a distance threshold, a callback activates a rotation motor
 * on the box.  When the pendulum swings away, the motor stops.
 * A circle on the ground visualizes the threshold radius.
 *
 * Demonstrates: ivp_actuator_check_dist, distance threshold callbacks,
 *               ivp_constraint (ball_socket), ivp_actuator_rot_mot,
 *               combined controller interaction.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/controllers/ivp_actuator_check_dist.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/controllers/ivp_actuator_rot_mot.h>

#include <stdio.h>

/* ── Distance event callback ────────────────────────────────────────────── */

typedef struct {
    ivp_rot_mot_state_t *motor;
    ivp_real_t           motor_power;
    bool                 motor_active;
    int                  enter_count;
    int                  exit_count;
} dist_event_ctx_t;

static void on_distance_event(void *ctx,
                               ivp_object_t *obj_a,
                               ivp_object_t *obj_b,
                               ivp_real_t distance,
                               bool is_inside_range)
{
    (void)obj_a; (void)obj_b; (void)distance;
    dist_event_ctx_t *dc = ctx;
    if (is_inside_range) {
        ivp_rot_mot_set_power(dc->motor, dc->motor_power);
        dc->motor_active = true;
        dc->enter_count++;
    } else {
        ivp_rot_mot_set_power(dc->motor, IVP_REAL(0.0));
        dc->motor_active = false;
        dc->exit_count++;
    }
}

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Check Distance -- Motor Trigger", .vsync = true
    };
    ivp_renderer_t *renderer = ivp_renderer_create(&rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -5.0f;
    cam.orbit_distance  = 25.0f;
    cam.orbit_yaw       = 30.0f;
    cam.orbit_pitch     = -20.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "check_distance");

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 15.0f, 0.5f, 15.0f);
    ivp_surman_polygon_t sm_ground;
    create_ground_object(env, ground_surf, &sm_ground, IVP_REAL(0.0));

    /* ── Pendulum pivot (static) ────────────────────────────────────────── */
    ivp_compact_surface_t *pivot_surf =
        ivp_build_box_surface(NULL, 0.3f, 0.3f, 0.3f);
    ivp_surman_polygon_t sm_pivot;
    ivp_surman_polygon_init(&sm_pivot, pivot_surf);

    ivp_object_template_t pvt;
    ivp_object_template_init(&pvt);
    pvt.type = IVP_OBJECT_POLYGON;
    pvt.mass = 0.0f;
    pvt.physical_unmoveable = true;
    pvt.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-10.0), IVP_REAL(0.0));
    ivp_object_t *pivot = ivp_environment_create_object(env, &pvt, &sm_pivot.base);

    /* ── Pendulum bob (heavy sphere) ────────────────────────────────────── */
    ivp_surman_ball_t sm_bob;
    ivp_surman_ball_init(&sm_bob, 0.6f);

    ivp_object_template_t bobt;
    ivp_object_template_init(&bobt);
    bobt.type = IVP_OBJECT_BALL;
    bobt.mass = IVP_REAL(5.0);
    set_sphere_inertia(&bobt, IVP_REAL(5.0), IVP_REAL(0.6));
    bobt.material.friction   = IVP_REAL(0.3);
    bobt.material.elasticity = IVP_REAL(0.1);
    /* Start displaced to the side so it swings */
    bobt.position = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(-5.0), IVP_REAL(0.0));
    ivp_object_t *bob = ivp_environment_create_object(env, &bobt, &sm_bob.base);

    /* ── Ball-socket constraint (pendulum joint) ────────────────────────── */
    ivp_constraint_state_t pend_joint;
    {
        ivp_vec3r_t a_anchor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t b_anchor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.6), IVP_REAL(0.0));
        ivp_constraint_init_ball_socket(&pend_joint,
            pivot, &a_anchor, bob, &b_anchor);
    }
    ivp_environment_add_controller(env, 50, ivp_controller_constraint, &pend_joint);

    /* ── Spinning box (target on ground) ────────────────────────────────── */
    ivp_compact_surface_t *box_surf =
        ivp_build_box_surface(NULL, 0.8f, 0.4f, 0.8f);
    ivp_surman_polygon_t sm_box;
    ivp_surman_polygon_init(&sm_box, box_surf);

    ivp_object_template_t bxt;
    ivp_object_template_init(&bxt);
    bxt.type = IVP_OBJECT_POLYGON;
    bxt.mass = IVP_REAL(2.0);
    set_box_inertia(&bxt, IVP_REAL(2.0), IVP_REAL(0.8), IVP_REAL(0.4), IVP_REAL(0.8));
    bxt.material.friction   = IVP_REAL(0.8);
    bxt.material.elasticity = IVP_REAL(0.05);
    bxt.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.9), IVP_REAL(0.0));
    ivp_object_t *spin_box = ivp_environment_create_object(env, &bxt, &sm_box.base);

    /* ── Rotation motor on the box ──────────────────────────────────────── */
    ivp_rot_mot_state_t motor;
    ivp_rot_mot_state_init(&motor);
    {
        ivp_vec3r_t ma = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t mb = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
        /* Moderate torque for a 2kg box (I≈0.43): 5N·m → ~12 rad/s² angular accel */
        ivp_rot_mot_state_setup(&motor,
            NULL, &ma, spin_box, &mb,
            IVP_REAL(0.0), IVP_REAL(5.0), IVP_REAL(6.0));
    }
    ivp_environment_add_controller(env, 100, ivp_controller_rot_mot, &motor);

    /* ── Distance checker (bob ↔ spin_box, threshold 4m) ────────────────── */
    dist_event_ctx_t dist_ctx = {
        .motor = &motor,
        .motor_power = IVP_REAL(8.0),
        .motor_active = false,
        .enter_count = 0,
        .exit_count = 0
    };

    ivp_check_dist_state_t dist_check;
    ivp_check_dist_state_init(&dist_check);
    {
        ivp_vec3r_t pa = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t pb = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_check_dist_state_setup(&dist_check, bob, &pa, spin_box, &pb,
                                     IVP_REAL(4.0));
    }
    dist_check.use_object_centers = true;
    ivp_check_dist_set_callback(&dist_check, on_distance_event, &dist_ctx);
    ivp_environment_add_controller(env, 100,
        ivp_controller_check_dist, &dist_check);

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_ground[]  = {0.22f, 0.24f, 0.30f};
    const float c_grid[]    = {0.38f, 0.43f, 0.52f};
    const float c_hud[]     = {0.95f, 0.97f, 1.0f};
    const float c_pivot[]   = {0.5f, 0.5f, 0.5f};
    const float c_bob[]     = {0.2f, 0.8f, 1.0f};
    const float c_box_on[]  = {0.2f, 1.0f, 0.3f};
    const float c_box_off[] = {1.0f, 0.5f, 0.1f};
    const float c_range[]   = {1.0f, 1.0f, 0.2f};
    const float c_link[]    = {0.6f, 0.6f, 0.6f};
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

        /* Grid & ground */
        ivp_draw_ground_grid(renderer, 15.0f, 2.0f, c_grid);
        {
            float gp[3] = {0, 0, 0}, gs[3] = {15.0f, 0.5f, 15.0f};
            ivp_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        /* Pivot */
        {
            ivp_vec3r_t p;
            ivp_object_get_position(pivot, &p);
            float fp[3]; vec3r_to_float3(&p, fp);
            float hs[3] = {0.3f, 0.3f, 0.3f};
            ivp_draw_wire_box(renderer, fp, hs, NULL, c_pivot);
        }

        /* Bob + pendulum rod */
        float fp_bob[3], fp_pivot[3];
        {
            ivp_vec3r_t pb, pp;
            ivp_object_get_position(bob, &pb);
            ivp_object_get_position(pivot, &pp);
            vec3r_to_float3(&pb, fp_bob);
            vec3r_to_float3(&pp, fp_pivot);
            ivp_draw_wire_sphere(renderer, fp_bob, 0.6f, c_bob);
            ivp_draw_thick_line(renderer, fp_pivot, fp_bob, c_link, 2);
        }

        /* Spin box (color depends on motor state) */
        float fp_box[3];
        {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(spin_box, &p);
            ivp_object_get_orientation(spin_box, &q);
            vec3r_to_float3(&p, fp_box);
            float rot[9]; quat_to_col_rot(&q, rot);
            float hs[3] = {0.8f, 0.4f, 0.8f};
            ivp_draw_wire_box(renderer, fp_box, hs, rot,
                              dist_ctx.motor_active ? c_box_on : c_box_off);
        }

        /* Threshold circle (on ground plane, Y≈0, 16 segments) */
        {
            int segs = 16;
            for (int i = 0; i < segs; i++) {
                float a0 = 6.2832f * (float)i / (float)segs;
                float a1 = 6.2832f * (float)(i + 1) / (float)segs;
                float r = 4.0f; /* match threshold */
                float p0[3] = {fp_box[0] + r * cosf(a0), -0.2f,
                               fp_box[2] + r * sinf(a0)};
                float p1[3] = {fp_box[0] + r * cosf(a1), -0.2f,
                               fp_box[2] + r * sinf(a1)};
                ivp_draw_line(renderer, p0, p1, c_range);
            }
        }

        /* Velocity arrow on bob */
        {
            ivp_vec3r_t v;
            ivp_object_get_speed(bob, &v);
            float speed = vec3r_speed(&v);
            if (speed > 0.3f) {
                float sc = 0.3f;
                float to[3] = {fp_bob[0]+(float)v.k[0]*sc,
                               fp_bob[1]+(float)v.k[1]*sc,
                               fp_bob[2]+(float)v.k[2]*sc};
                ivp_draw_arrow(renderer, fp_bob, to, c_vel, 0.2f);
            }
        }

        /* Axes */
        {
            float org[3] = {0, 0, 0};
            ivp_draw_axes(renderer, org, NULL, 1.5f);
        }

        if (ivp_example_inspector_tick(&inspector)) {
            ivp_example_inspector_log(&inspector,
                "dist=%.3f inside=%d motor=%d enters=%d exits=%d motor_speed=%.3f",
                (float)dist_check.current_distance,
                1 - dist_check.is_outside,
                dist_ctx.motor_active ? 1 : 0,
                dist_ctx.enter_count,
                dist_ctx.exit_count,
                (float)motor.current_speed);
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus,
                                              c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  Time: %.1fs",
                     sl.fps_display, (float)sl.t_prev);
            ivp_draw_text_2d(renderer, 10.0f, 10.0f, buf, c_hud);

            snprintf(buf, sizeof(buf), "Distance: %.2f m  Threshold: 4.0 m",
                     (float)dist_check.current_distance);
            ivp_draw_text_2d(renderer, 10.0f, 30.0f, buf, c_range);

            snprintf(buf, sizeof(buf), "Motor: %s  Speed: %.1f rad/s",
                     dist_ctx.motor_active ? "ON" : "OFF",
                     (float)motor.current_speed);
            ivp_draw_text_2d(renderer, 10.0f, 48.0f, buf,
                             dist_ctx.motor_active ? c_box_on : c_box_off);

            ivp_example_draw_interaction_hud(renderer, 10.0f, 70.0f,
                                             c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, box_surf);
    ivp_compact_surface_free(NULL, pivot_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
