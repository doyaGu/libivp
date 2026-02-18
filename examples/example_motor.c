/* example_motor.c -- Rotation motor with blade smashing objects
 *
 * A motor spins a blade constrained by a hinge to a pedestal.
 * Small cubes are placed on a ground plane where the blade can hit
 * them and send them flying.  HUD shows angular speed.
 *
 * Demonstrates: ivp_actuator_rot_mot, ivp_constraint (hinge),
 *               power / torque / max speed, collision interaction,
 *               angular velocity, HUD.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/controllers/ivp_actuator_rot_mot.h>
#include <ivp/controllers/ivp_constraint.h>

#include <stdio.h>

#define N_TARGETS 8
static const char *MOTOR_INTERNAL_GROUP = "motor_internal";

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Motor -- Spinning Blade", .vsync = true
    };
    ivp_renderer_t *renderer = ivp_renderer_create(&rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -4.0f;
    cam.orbit_distance  = 20.0f;
    cam.orbit_pitch     = -25.0f;
    cam.orbit_yaw       = 30.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "motor");

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 15.0f, 0.5f, 15.0f);
    ivp_surman_polygon_t sm_ground;
    ivp_surman_polygon_init(&sm_ground, ground_surf);

    ivp_object_template_t grt;
    ivp_object_template_init(&grt);
    grt.type = IVP_OBJECT_POLYGON;
    grt.mass = 0.0f;
    grt.physical_unmoveable = true;
    ivp_environment_create_object(env, &grt, &sm_ground.base);

    /* ── Pedestal (static, taller) ──────────────────────────────────────── */
    ivp_compact_surface_t *ped_surf =
        ivp_build_box_surface(NULL, 0.15f, 0.8f, 0.15f);
    ivp_surman_polygon_t sm_ped;
    ivp_surman_polygon_init(&sm_ped, ped_surf);

    ivp_object_template_t pt;
    ivp_object_template_init(&pt);
    pt.type = IVP_OBJECT_POLYGON;
    pt.mass = 0.0f;
    pt.physical_unmoveable = true;
    pt.no_collision_group = MOTOR_INTERNAL_GROUP;
    pt.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-5.6), IVP_REAL(0.0));
    ivp_object_t *pedestal = ivp_environment_create_object(env, &pt, &sm_ped.base);

    /* ── Blade (dynamic, the spinning part) ─────────────────────────────── */
    ivp_compact_surface_t *blade_surf =
        ivp_build_box_surface(NULL, 4.0f, 0.25f, 0.4f);
    ivp_surman_polygon_t sm_blade;
    ivp_surman_polygon_init(&sm_blade, blade_surf);

    ivp_object_template_t bt;
    ivp_object_template_init(&bt);
    bt.type = IVP_OBJECT_POLYGON;
    const ivp_real_t blade_mass = IVP_REAL(1.2);
    bt.mass = blade_mass;
    bt.no_collision_group = MOTOR_INTERNAL_GROUP;
    ivp_vec3r_set(&bt.rot_inertia,
        blade_mass * IVP_REAL(0.2725/3.0),
        blade_mass * IVP_REAL(16.16/3.0),
        blade_mass * IVP_REAL(16.0625/3.0));
    bt.material.friction   = IVP_REAL(0.5);
    bt.material.elasticity = IVP_REAL(0.35);
    bt.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&bt.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    bt.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-6.8), IVP_REAL(0.0));
    ivp_object_t *blade = ivp_environment_create_object(env, &bt, &sm_blade.base);
    blade->core.rot_speed.k[1] = 0.2f;

    /* ── Hinge constraint ───────────────────────────────────────────────── */
    ivp_constraint_state_t pivot;
    {
        ivp_vec3r_t anchor_ped = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-1.2), IVP_REAL(0.0));
        ivp_vec3r_t anchor_blade = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t axis_ped = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        ivp_vec3r_t axis_blade = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        ivp_constraint_init_hinge(&pivot,
            pedestal, &anchor_ped, &axis_ped,
            blade, &anchor_blade, &axis_blade);
        pivot.solver_iterations = 28;
        pivot.tau = IVP_REAL(0.25);
        pivot.damping = IVP_REAL(0.95);
    }
    ivp_environment_add_controller(env, 50,
        ivp_controller_constraint, &pivot);

    /* ── Rotation motor ─────────────────────────────────────────────────── */
    ivp_rot_mot_state_t motor;
    ivp_rot_mot_state_init(&motor);
    {
        ivp_vec3r_t a0 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t a1 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        ivp_rot_mot_state_setup(&motor,
            blade,    &a0,
            blade,    &a1,
            IVP_REAL(3.0),    /* power */
            IVP_REAL(4.0),    /* max torque */
            IVP_REAL(1.6));   /* max speed (rad/s) */
    }
    ivp_environment_add_controller(env, 100,
        ivp_controller_rot_mot, &motor);

    /* ── Target cubes around the blade ──────────────────────────────────── */
    ivp_compact_surface_t *target_surf =
        ivp_build_box_surface(NULL, 0.4f, 0.4f, 0.4f);
    ivp_surman_polygon_t sm_targets[N_TARGETS];
    ivp_object_t *targets[N_TARGETS];

    for (int i = 0; i < N_TARGETS; i++) {
        ivp_surman_polygon_init(&sm_targets[i], target_surf);

        ivp_object_template_t tt;
        ivp_object_template_init(&tt);
        tt.type = IVP_OBJECT_POLYGON;
        tt.mass = IVP_REAL(0.06);
        /* 0.4m half-extent cube, 0.06kg: I_x = (0.06/3)(0.16+0.16) = 0.0064 */
        set_box_inertia(&tt, IVP_REAL(0.06), IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
        tt.material.friction   = IVP_REAL(0.5);
        tt.material.elasticity = IVP_REAL(0.2);

        /* Place in a circle at blade height */
        float angle = (float)i * 6.2832f / (float)N_TARGETS;
        float radius = 3.5f;
        tt.position = ivp_vec3r(
            (ivp_real_t)(cosf(angle) * radius),
            IVP_REAL(-6.4),
            (ivp_real_t)(sinf(angle) * radius));

        targets[i] = ivp_environment_create_object(env, &tt, &sm_targets[i].base);
    }

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_ped[]    = {0.40f, 0.40f, 0.45f};
    const float c_blade[]  = {0.20f, 0.70f, 1.0f };
    const float c_target[] = {1.0f,  0.45f, 0.15f};
    const float c_ground[] = {0.30f, 0.30f, 0.35f};
    const float c_grid[]   = {0.12f, 0.12f, 0.18f};
    const float c_hud[]    = {0.80f, 0.80f, 0.80f};
    const float c_vel[]    = {0.20f, 1.0f,  0.40f};
    const float c_drag[]   = {1.0f, 0.85f, 0.25f};
    const float c_focus[]  = {1.0f, 0.65f, 0.25f};

    ivp_example_dragger_t dragger;
    ivp_example_dragger_init(&dragger);
    dragger.settings.max_force = 60.0f;
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

        float max_target_speed = 0.0f;
        int fast_targets = 0;
        for (int i = 0; i < N_TARGETS; i++) {
            ivp_vec3r_t v;
            ivp_object_get_speed(targets[i], &v);
            float speed = vec3r_speed(&v);
            if (speed > max_target_speed) max_target_speed = speed;
            if (speed > 10.0f) fast_targets++;
        }
        ivp_vec3r_t pivot_a_ws, pivot_b_ws;
        ivp_vec3r_t anchor_ped_os = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-1.2), IVP_REAL(0.0));
        ivp_vec3r_t anchor_blade_os = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_transform_point(&pivot_a_ws, &pedestal->core.m_world_f_core_last_psi, &anchor_ped_os);
        ivp_transform_point(&pivot_b_ws, &blade->core.m_world_f_core_last_psi, &anchor_blade_os);
        float dx = (float)(pivot_b_ws.k[0] - pivot_a_ws.k[0]);
        float dy = (float)(pivot_b_ws.k[1] - pivot_a_ws.k[1]);
        float dz = (float)(pivot_b_ws.k[2] - pivot_a_ws.k[2]);
        float hinge_err = sqrtf(dx * dx + dy * dy + dz * dz);
        ivp_vec3f_t ww = blade->core.rot_speed;
        float omega_dbg = sqrtf(ww.k[0]*ww.k[0] + ww.k[1]*ww.k[1] + ww.k[2]*ww.k[2]);
        float omega_y = fabsf(ww.k[1]);
        float cap_ratio = (float)(motor.max_rotation_speed > IVP_REAL(1e-6)
            ? (omega_y / (float)motor.max_rotation_speed)
            : 0.0f);

        if (ivp_example_inspector_tick(&inspector)) {
            ivp_example_inspector_log(&inspector,
                "omega=%.3f omega_y=%.3f cap_ratio=%.3f target_max=%.3f fast_targets=%d/%d hinge_anchor_err=%.3f",
                omega_dbg,
                omega_y,
                cap_ratio,
                max_target_speed,
                fast_targets, N_TARGETS,
                hinge_err);
        }

        if (max_target_speed > 80.0f || cap_ratio > 2.5f || hinge_err > 1.5f) {
            ivp_example_inspector_log(&inspector,
                "instability flag (cap_ratio=%.3f target_max=%.3f hinge_err=%.3f)",
                cap_ratio, max_target_speed, hinge_err);
        }

        /* Grid & ground */
        ivp_draw_ground_grid(renderer, 15.0f, 2.0f, c_grid);
        {
            float gp[3] = {0,0,0}, gs[3] = {15.0f, 0.5f, 15.0f};
            ivp_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        /* Pedestal */
        {
            ivp_vec3r_t p;
            ivp_object_get_position(pedestal, &p);
            float fp[3]; vec3r_to_float3(&p, fp);
            float hs[3] = {0.15f, 0.8f, 0.15f};
            ivp_draw_wire_box(renderer, fp, hs, NULL, c_ped);
        }

        /* Blade */
        {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(blade, &p);
            ivp_object_get_orientation(blade, &q);
            float rot[9]; quat_to_col_rot(&q, rot);
            float fp[3]; vec3r_to_float3(&p, fp);
            float hs[3] = {4.0f, 0.25f, 0.4f};
            ivp_draw_wire_box(renderer, fp, hs, rot, c_blade);

            /* Axes at blade center showing local frame */
            ivp_draw_axes(renderer, fp, rot, 1.5f);
        }

        /* Targets */
        for (int i = 0; i < N_TARGETS; i++) {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(targets[i], &p);
            ivp_object_get_orientation(targets[i], &q);
            float rot[9]; quat_to_col_rot(&q, rot);
            float fp[3]; vec3r_to_float3(&p, fp);
            float hs[3] = {0.4f, 0.4f, 0.4f};
            ivp_draw_wire_box(renderer, fp, hs, rot, c_target);

            /* Velocity arrow if moving */
            ivp_vec3r_t v;
            ivp_object_get_speed(targets[i], &v);
            float spd = vec3r_speed(&v);
            if (spd > 1.0f) {
                float sc = 0.25f;
                float to[3] = {fp[0]+(float)v.k[0]*sc, fp[1]+(float)v.k[1]*sc, fp[2]+(float)v.k[2]*sc};
                ivp_draw_arrow(renderer, fp, to, c_vel, 0.2f);
            }
        }

        /* Origin axes */
        {
            float org[3] = {0, 0, 0};
            ivp_draw_axes(renderer, org, NULL, 2.0f);
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus, c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  Targets: %d",
                     sl.fps_display, N_TARGETS);
            ivp_draw_text_2d(renderer, 10.0f, 10.0f, buf, c_hud);

            /* Angular velocity of blade (from core, float vec3) */
            ivp_vec3f_t w = blade->core.rot_speed;
            float omega = sqrtf(w.k[0]*w.k[0]+w.k[1]*w.k[1]+w.k[2]*w.k[2]);
            float rpm = omega * 60.0f / (2.0f * 3.14159f);
            snprintf(buf, sizeof(buf), "Blade: %.1f rad/s  (%.0f RPM)", omega, rpm);
            ivp_draw_text_2d(renderer, 10.0f, 30.0f, buf, c_blade);

            ivp_example_draw_interaction_hud(renderer, 10.0f, 50.0f, c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, target_surf);
    ivp_compact_surface_free(NULL, blade_surf);
    ivp_compact_surface_free(NULL, ped_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
