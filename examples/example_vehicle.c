/* example_vehicle.c -- Raycast vehicle demo
 *
 * A four-wheel raycast car drives on a flat ground with ramps.
 * Arrow keys steer and accelerate, Space brakes.  Wheel contact
 * points are drawn as small spheres.  Suspension compression is
 * visualized as coloured lines.
 *
 * Demonstrates: ivp_controller_raycast_car, 4-wheel init, steering,
 *               engine, brake, wheel contact visualization, ramps.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/controllers/ivp_controller_raycast_car.h>
#include <ivp/math/ivp_fast_math.h>

#include <SDL3/SDL.h>
#include <stdio.h>

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Vehicle -- Raycast Car", .vsync = true
    };
    ivp_renderer_t *renderer = NULL;
    ivp_renderer_create(&renderer, &rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -2.0f;
    cam.orbit_distance  = 20.0f;
    cam.orbit_yaw       = 30.0f;
    cam.orbit_pitch     = -25.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "vehicle");

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 40.0f, 0.5f, 40.0f);
    ivp_surman_polygon_t sm_ground;
    create_ground_object(env, ground_surf, &sm_ground, IVP_REAL(0.0));

    /* ── Ramps ──────────────────────────────────────────────────────────── */
    ivp_compact_surface_t *ramp_surf =
        ivp_build_box_surface(NULL, 4.0f, 0.3f, 3.0f);
    ivp_surman_polygon_t sm_ramp1, sm_ramp2;
    ivp_surman_polygon_init(&sm_ramp1, ramp_surf);
    ivp_surman_polygon_init(&sm_ramp2, ramp_surf);

    ivp_object_template_t rpt;
    ivp_object_template_init(&rpt);
    rpt.type = IVP_OBJECT_POLYGON;
    rpt.mass = 0.0f;
    rpt.physical_unmoveable = true;

    /* Ramp tilt: 15 degrees around Z axis, expressed as axis-angle quaternion.
     * q = (0, 0, sin(angle/2), cos(angle/2)) for Z-axis rotation. */
    ivp_real_t ramp_half = IVP_REAL(15.0) * IVP_REAL(3.14159265) / IVP_REAL(360.0);

    /* Ramp 1: tilted forward (+15 deg) */
    rpt.position = ivp_vec3r(IVP_REAL(15.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    rpt.orientation = ivp_quatr_make(IVP_REAL(0.0), IVP_REAL(0.0),
                                       ivp_sin(ramp_half), ivp_cos(ramp_half));
    ivp_quat_fast_normalize(&rpt.orientation);
    ivp_object_t *ramp1 = ivp_environment_create_object(env, &rpt, &sm_ramp1.base);

    /* Ramp 2: tilted opposite side (-15 deg) */
    rpt.position = ivp_vec3r(IVP_REAL(-15.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    rpt.orientation = ivp_quatr_make(IVP_REAL(0.0), IVP_REAL(0.0),
                                       -ivp_sin(ramp_half), ivp_cos(ramp_half));
    ivp_quat_fast_normalize(&rpt.orientation);
    ivp_object_t *ramp2 = ivp_environment_create_object(env, &rpt, &sm_ramp2.base);

    /* ── Obstacle boxes ─────────────────────────────────────────────────── */
    ivp_compact_surface_t *obs_surf =
        ivp_build_box_surface(NULL, 0.8f, 0.8f, 0.8f);
    ivp_surman_polygon_t sm_obs[4];
    ivp_object_t *obs_objs[4];
    for (int i = 0; i < 4; i++) {
        ivp_surman_polygon_init(&sm_obs[i], obs_surf);
        ivp_object_template_t ot;
        ivp_object_template_init(&ot);
        ot.type = IVP_OBJECT_POLYGON;
        ot.mass = IVP_REAL(1.0);
        set_box_inertia(&ot, IVP_REAL(1.0), IVP_REAL(0.8), IVP_REAL(0.8), IVP_REAL(0.8));
        ot.material.friction   = IVP_REAL(0.6);
        ot.material.elasticity = IVP_REAL(0.3);
        float angle = 6.2832f * (float)i / 4.0f;
        ot.position = ivp_vec3r(
            (ivp_real_t)(8.0f * cosf(angle)),
            IVP_REAL(-1.3),
            (ivp_real_t)(8.0f * sinf(angle)));
        obs_objs[i] = ivp_environment_create_object(env, &ot, &sm_obs[i].base);
    }

    /* ── Chassis (box) ──────────────────────────────────────────────────── */
    ivp_compact_surface_t *chassis_surf =
        ivp_build_box_surface(NULL, 1.0f, 0.4f, 2.0f);
    ivp_surman_polygon_t sm_chassis;
    ivp_surman_polygon_init(&sm_chassis, chassis_surf);

    ivp_object_template_t cht;
    ivp_object_template_init(&cht);
    cht.type = IVP_OBJECT_POLYGON;
    cht.mass = IVP_REAL(20.0);
    set_box_inertia(&cht, IVP_REAL(20.0), IVP_REAL(1.0), IVP_REAL(0.4), IVP_REAL(2.0));
    cht.material.friction   = IVP_REAL(0.3);
    cht.material.elasticity = IVP_REAL(0.1);
    cht.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-3.0), IVP_REAL(0.0));
    ivp_object_t *chassis = ivp_environment_create_object(env, &cht, &sm_chassis.base);

    /* ── Raycast car ────────────────────────────────────────────────────── */
    ivp_car_state_t car;
    ivp_car_init_4wheel(&car, chassis,
        IVP_REAL(3.0),   /* wheelbase */
        IVP_REAL(1.8),   /* track width */
        IVP_REAL(0.35),  /* wheel radius */
        IVP_REAL(0.6));  /* suspension length */

    /* Tune suspension */
    for (int i = 0; i < car.n_wheels; i++) {
        car.wheels[i].spring_constant = IVP_REAL(800.0);
        car.wheels[i].damping         = IVP_REAL(60.0);
        car.wheels[i].friction_forward = IVP_REAL(1.2);
        car.wheels[i].friction_side    = IVP_REAL(1.0);
    }
    car.steer_front = true;
    car.drive_rear  = true;
    car.ground_normal = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    car.ground_height = IVP_REAL(0.0);

    ivp_environment_add_controller(env, 100,
        ivp_controller_raycast_car, &car);

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_ground[]   = {0.22f, 0.24f, 0.30f};
    const float c_grid[]     = {0.38f, 0.43f, 0.52f};
    const float c_hud[]      = {0.95f, 0.97f, 1.0f};
    const float c_chassis[]  = {0.2f, 0.6f, 1.0f};
    const float c_ramp[]     = {0.6f, 0.5f, 0.3f};
    const float c_obs[]      = {0.9f, 0.5f, 0.2f};
    const float c_wheel_g[]  = {0.2f, 1.0f, 0.3f};  /* grounded */
    const float c_wheel_a[]  = {1.0f, 0.3f, 0.2f};  /* airborne */
    const float c_susp[]     = {1.0f, 1.0f, 0.2f};
    const float c_vel[]      = {0.2f, 1.0f, 0.4f};
    const float c_drag[]     = {1.0f, 0.85f, 0.25f};
    const float c_focus[]    = {1.0f, 0.65f, 0.25f};

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

        /* Keyboard input */
        const bool *keys = SDL_GetKeyboardState(NULL);

        /* Steering */
        ivp_real_t steer = IVP_REAL(0.0);
        if (keys[SDL_SCANCODE_LEFT])  steer += IVP_REAL(0.4);
        if (keys[SDL_SCANCODE_RIGHT]) steer -= IVP_REAL(0.4);
        ivp_car_set_steering(&car, steer);

        /* Engine: 200N forward, 100N reverse (balanced for 20kg chassis) */
        ivp_real_t engine = IVP_REAL(0.0);
        if (keys[SDL_SCANCODE_UP])   engine = IVP_REAL(200.0);
        if (keys[SDL_SCANCODE_DOWN]) engine = IVP_REAL(-100.0);
        ivp_car_set_engine(&car, engine);

        /* Brake: 200N gives ~1g deceleration on 20kg chassis (realistic max) */
        ivp_real_t brake = IVP_REAL(0.0);
        if (keys[SDL_SCANCODE_SPACE]) brake = IVP_REAL(200.0);
        ivp_car_set_brake(&car, brake);

        sim_loop_step(&sl, env, frame_dt);

        /* Follow camera: track chassis */
        {
            ivp_vec3r_t cp;
            ivp_object_get_position(chassis, &cp);
            float target[3]; vec3r_to_float3(&cp, target);
            cam.target[0] += (target[0] - cam.target[0]) * 0.05f;
            cam.target[1] += (target[1] - cam.target[1]) * 0.05f;
            cam.target[2] += (target[2] - cam.target[2]) * 0.05f;
        }

        /* Grid & ground */
        ivp_render_draw_ground_grid(renderer, 40.0f, 4.0f, c_grid);
        {
            float gp[3] = {0, 0, 0}, gs[3] = {40.0f, 0.5f, 40.0f};
            ivp_render_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        /* Ramps */
        for (int i = 0; i < 2; i++) {
            ivp_object_t *r = (i == 0) ? ramp1 : ramp2;
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(r, &p);
            ivp_object_get_orientation(r, &q);
            float fp[3]; vec3r_to_float3(&p, fp);
            float rot[9]; quat_to_col_rot(&q, rot);
            float hs[3] = {4.0f, 0.3f, 3.0f};
            ivp_render_draw_wire_box(renderer, fp, hs, rot, c_ramp);
        }

        /* Obstacles */
        for (int i = 0; i < 4; i++) {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(obs_objs[i], &p);
            ivp_object_get_orientation(obs_objs[i], &q);
            float fp[3]; vec3r_to_float3(&p, fp);
            float rot[9]; quat_to_col_rot(&q, rot);
            float hs[3] = {0.8f, 0.8f, 0.8f};
            ivp_render_draw_wire_box(renderer, fp, hs, rot, c_obs);
        }

        /* Chassis */
        {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(chassis, &p);
            ivp_object_get_orientation(chassis, &q);
            float fp[3]; vec3r_to_float3(&p, fp);
            float rot[9]; quat_to_col_rot(&q, rot);
            float hs[3] = {1.0f, 0.4f, 2.0f};
            ivp_render_draw_wire_box(renderer, fp, hs, rot, c_chassis);

            /* Velocity arrow */
            ivp_vec3r_t v;
            ivp_object_get_speed(chassis, &v);
            float speed = vec3r_speed(&v);
            if (speed > 0.5f) {
                float sc = 0.3f;
                float to[3] = {fp[0]+(float)v.k[0]*sc,
                               fp[1]+(float)v.k[1]*sc,
                               fp[2]+(float)v.k[2]*sc};
                ivp_render_draw_arrow(renderer, fp, to, c_vel, 0.3f);
            }
        }

        /* Wheel contact points + suspension visualization.
         * Transform hardpoint_cs through chassis rotation (not just translation)
         * so wheels display correctly on ramps and during turns. */
        int n_grounded = 0;
        ivp_vec3r_t chassis_p; ivp_quatr_t chassis_q;
        ivp_object_get_position(chassis, &chassis_p);
        ivp_object_get_orientation(chassis, &chassis_q);
        ivp_mat3r_t chassis_rot;
        ivp_quat_to_mat3(&chassis_q, &chassis_rot);

        for (int i = 0; i < car.n_wheels; i++) {
            const ivp_car_wheel_t *w = &car.wheels[i];

            /* Transform hardpoint from chassis-space to world-space:
             * hp_ws = chassis_rot * hp_cs + chassis_pos */
            float hp[3];
            for (int r = 0; r < 3; r++) {
                hp[r] = (float)(chassis_rot.rows[r].k[0] * w->hardpoint_cs.k[0]
                              + chassis_rot.rows[r].k[1] * w->hardpoint_cs.k[1]
                              + chassis_rot.rows[r].k[2] * w->hardpoint_cs.k[2]
                              + chassis_p.k[r]);
            }

            if (w->grounded) {
                n_grounded++;
                float cp[3] = {(float)w->contact_point_ws.k[0],
                               (float)w->contact_point_ws.k[1],
                               (float)w->contact_point_ws.k[2]};
                /* Draw wheel as circle at wheel center using actual radius (0.35m) */
                ivp_render_draw_wire_sphere(renderer, cp, 0.35f, c_wheel_g);
                ivp_render_draw_line(renderer, hp, cp, c_susp);
            } else {
                /* Draw ungrounded wheel marker at transformed hardpoint */
                ivp_render_draw_wire_sphere(renderer, hp, 0.35f, c_wheel_a);
            }
        }

        /* Axes */
        {
            float org[3] = {0, 0, 0};
            ivp_render_draw_axes(renderer, org, NULL, 2.0f);
        }

        if (ivp_example_inspector_tick(&inspector)) {
            ivp_vec3r_t cv;
            ivp_object_get_speed(chassis, &cv);
            ivp_example_inspector_log(&inspector,
                "speed=%.3f steer=%.3f engine=%.1f brake=%.1f grounded=%d/%d",
                vec3r_speed(&cv),
                (float)car.steering_angle,
                (float)car.engine_force,
                (float)car.brake_force,
                n_grounded, car.n_wheels);
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus,
                                              c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            ivp_vec3r_t cv;
            ivp_object_get_speed(chassis, &cv);
            float speed = vec3r_speed(&cv);

            snprintf(buf, sizeof(buf), "FPS: %.0f  Speed: %.1f m/s  Time: %.1fs",
                     sl.fps_display, speed, (float)sl.t_prev);
            ivp_render_draw_text(renderer, 10.0f, 10.0f, buf, c_hud);

            snprintf(buf, sizeof(buf),
                "Arrows: steer/drive  Space: brake  Grounded: %d/%d",
                n_grounded, car.n_wheels);
            ivp_render_draw_text(renderer, 10.0f, 30.0f, buf, c_hud);

            snprintf(buf, sizeof(buf), "Steer: %.2f  Engine: %.0f N  Brake: %.0f N",
                     (float)car.steering_angle,
                     (float)car.engine_force,
                     (float)car.brake_force);
            ivp_render_draw_text(renderer, 10.0f, 48.0f, buf, c_susp);

            ivp_example_draw_interaction_hud(renderer, 10.0f, 70.0f,
                                             c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, obs_surf);
    ivp_compact_surface_free(NULL, ramp_surf);
    ivp_compact_surface_free(NULL, chassis_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
