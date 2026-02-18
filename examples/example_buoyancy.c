/* example_buoyancy.c -- Mixed objects in water with flow current
 *
 * Boxes and spheres of varying densities are dropped into a "lake".
 * Light objects float, heavy ones sink.  A flow current pushes objects
 * sideways.  HUD shows depth and buoyancy status for each object.
 *
 * Demonstrates: ivp_controller_buoyancy, fluid density, flow_velocity,
 *               sphere vs box shapes, HUD text, velocity arrows.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/base/ivp_types.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/controllers/ivp_controller_buoyancy.h>

#include <stdio.h>

/* Water surface in Y-down coords -- negative Y = up */
#define WATER_SURFACE_Y  (-7.0f)

/* ── Object descriptors ────────────────────────────────────────────────── */
#define N_OBJS 7

typedef struct {
    const char *label;
    float mass;
    float hs[3];        /* half-extents (box) or {radius,0,0} (sphere) */
    float start_pos[3];
    float col[3];
    bool  is_sphere;
} obj_desc_t;

static const obj_desc_t descs[N_OBJS] = {
    /* label          mass   half-sizes                start-pos              color                sphere */
    {"Cork  0.3kg",   0.3f, {0.5f,0.5f,0.5f},  {-6.0f,-6.0f, 0.0f}, {0.95f,0.85f,0.35f}, false},
    {"Wood  1.0kg",   1.0f, {0.6f,0.4f,0.6f},  {-3.0f,-5.0f, 0.0f}, {0.75f,0.55f,0.25f}, false},
    {"Ball  1.5kg",   1.5f, {0.55f,0,0},        { 0.0f,-7.0f, 0.0f}, {0.3f, 0.8f, 1.0f},  true },
    {"Steel 8.0kg",   8.0f, {0.5f,0.5f,0.5f},  { 3.0f,-4.0f, 0.0f}, {0.65f,0.65f,0.70f}, false},
    {"Brick 3.0kg",   3.0f, {0.7f,0.35f,0.5f}, { 6.0f,-6.0f, 0.0f}, {0.85f,0.40f,0.25f}, false},
    {"Foam  0.2kg",   0.2f, {0.8f,0.8f,0.8f},  {-4.5f,-9.0f, 3.0f}, {1.0f, 1.0f, 0.8f},  false},
    {"Lead  12kg",   12.0f, {0.4f,0.4f,0.4f},  { 4.5f,-8.0f,-3.0f}, {0.3f, 0.3f, 0.4f},  false},
};


int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Buoyancy -- Floating & Sinking", .vsync = true
    };
    ivp_renderer_t *renderer = ivp_renderer_create(&rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -2.0f;
    cam.orbit_distance  = 28.0f;
    cam.orbit_yaw       = 20.0f;
    cam.orbit_pitch     = -20.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "buoyancy");

    /* ── Ground (sea floor) ─────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 15.0f, 0.5f, 15.0f);
    ivp_surman_polygon_t sm_ground;
    ivp_surman_polygon_init(&sm_ground, ground_surf);

    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.type = IVP_OBJECT_POLYGON;
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    gt.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(5.0), IVP_REAL(0.0));
    ivp_environment_create_object(env, &gt, &sm_ground.base);

    /* ── Side walls (keep objects from drifting away with current) ──── */
    ivp_compact_surface_t *wall_surf =
        ivp_build_box_surface(NULL, 0.3f, 6.0f, 15.0f);
    ivp_surman_polygon_t sm_wall_l, sm_wall_r;
    ivp_surman_polygon_init(&sm_wall_l, wall_surf);
    ivp_surman_polygon_init(&sm_wall_r, wall_surf);

    ivp_object_template_t wt;
    ivp_object_template_init(&wt);
    wt.type = IVP_OBJECT_POLYGON;
    wt.mass = 0.0f;
    wt.physical_unmoveable = true;
    wt.position = ivp_vec3r(IVP_REAL(-12.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_create_object(env, &wt, &sm_wall_l.base);
    wt.position = ivp_vec3r(IVP_REAL(12.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_create_object(env, &wt, &sm_wall_r.base);

    /* ── Create floatable objects ───────────────────────────────────────── */
    ivp_compact_surface_t *surfs[N_OBJS];
    ivp_surman_polygon_t   sm_poly[N_OBJS];
    ivp_surman_ball_t      sm_ball[N_OBJS];
    ivp_object_t          *objs[N_OBJS];
    float                  start_x[N_OBJS];

    ivp_buoyancy_state_t buoy;
    ivp_buoyancy_init(&buoy,
        (ivp_real_t)WATER_SURFACE_Y,
        IVP_REAL(1000.0));
    buoy.linear_drag  = IVP_REAL(3.0);
    buoy.angular_drag = IVP_REAL(2.0);
    ivp_vec3r_set(&buoy.flow_velocity, IVP_REAL(1.2), IVP_REAL(0.0), IVP_REAL(0.0));

    for (int i = 0; i < N_OBJS; i++) {
        const obj_desc_t *d = &descs[i];

        if (d->is_sphere) {
            ivp_surman_ball_init(&sm_ball[i], d->hs[0]);
        } else {
            surfs[i] = ivp_build_box_surface(NULL, d->hs[0], d->hs[1], d->hs[2]);
            ivp_surman_polygon_init(&sm_poly[i], surfs[i]);
        }

        ivp_object_template_t ot;
        ivp_object_template_init(&ot);
        ot.type = d->is_sphere ? IVP_OBJECT_BALL : IVP_OBJECT_POLYGON;
        ot.mass = (ivp_real_t)d->mass;
        /* Per-shape inertia: sphere uses (2/5)mr^2, box uses (m/3)(hy^2+hz^2) */
        if (d->is_sphere)
            set_sphere_inertia(&ot, ot.mass, (ivp_real_t)d->hs[0]);
        else
            set_box_inertia(&ot, ot.mass,
                (ivp_real_t)d->hs[0], (ivp_real_t)d->hs[1], (ivp_real_t)d->hs[2]);
        ot.material.friction   = IVP_REAL(0.5);
        ot.material.elasticity = IVP_REAL(0.2);
        ot.position = ivp_vec3r(
            (ivp_real_t)d->start_pos[0],
            (ivp_real_t)d->start_pos[1],
            (ivp_real_t)d->start_pos[2]);

        if (d->is_sphere)
            objs[i] = ivp_environment_create_object(env, &ot, &sm_ball[i].base);
        else
            objs[i] = ivp_environment_create_object(env, &ot, &sm_poly[i].base);
        start_x[i] = d->start_pos[0];

        /* Objects above water get a small initial downward velocity for natural drop */
        if (d->start_pos[1] < WATER_SURFACE_Y)
            objs[i]->core.speed = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.5), IVP_REAL(0.0));
    }

    ivp_environment_add_controller(env, 200,
        ivp_controller_buoyancy, &buoy);

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_water[]  = {0.10f, 0.30f, 0.65f};
    const float c_ground[] = {0.30f, 0.28f, 0.25f};
    const float c_grid[]   = {0.10f, 0.10f, 0.15f};
    const float c_wall[]   = {0.20f, 0.20f, 0.25f};
    const float c_hud[]    = {0.8f,  0.8f,  0.8f };
    const float c_vel[]    = {0.2f,  1.0f,  0.4f };
    const float c_buoy[]   = {0.3f,  0.6f,  1.0f };
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

        sim_loop_step(&sl, env, frame_dt);

        /* Grid & ground */
        ivp_draw_ground_grid(renderer, 15.0f, 2.0f, c_grid);
        {
            float gp[3] = {0, 5.0f, 0}, gs[3] = {15.0f, 0.5f, 15.0f};
            ivp_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }
        /* Walls */
        {
            float ws[3] = {0.3f, 6.0f, 15.0f};
            float wl[3] = {-12.0f, 0, 0}; ivp_draw_wire_box(renderer, wl, ws, NULL, c_wall);
            float wr[3] = { 12.0f, 0, 0}; ivp_draw_wire_box(renderer, wr, ws, NULL, c_wall);
        }

        /* Water surface grid (denser, with thick border) */
        {
            float ext = 12.0f;
            float wy = WATER_SURFACE_Y;
            for (float x = -ext; x <= ext; x += 1.5f) {
                float a[3] = {x, wy, -ext}, b[3] = {x, wy, ext};
                ivp_draw_line(renderer, a, b, c_water);
            }
            for (float z = -ext; z <= ext; z += 1.5f) {
                float a[3] = {-ext, wy, z}, b[3] = {ext, wy, z};
                ivp_draw_line(renderer, a, b, c_water);
            }
            /* Thick border lines */
            float a[3], b[3];
            a[1] = b[1] = wy;
            a[2] = -ext; b[2] = ext; a[0] = b[0] = -ext;
            ivp_draw_thick_line(renderer, a, b, c_water, 3);
            a[0] = b[0] = ext;
            ivp_draw_thick_line(renderer, a, b, c_water, 3);
            a[0] = -ext; b[0] = ext; a[2] = b[2] = -ext;
            ivp_draw_thick_line(renderer, a, b, c_water, 3);
            a[2] = b[2] = ext;
            ivp_draw_thick_line(renderer, a, b, c_water, 3);
        }

        /* Flow direction arrow in the water */
        {
            float fa[3] = {-8.0f, WATER_SURFACE_Y, 8.0f};
            float fb[3] = {-3.0f, WATER_SURFACE_Y, 8.0f};
            const float c_flow[] = {0.3f, 0.7f, 1.0f};
            ivp_draw_arrow(renderer, fa, fb, c_flow, 0.5f);
        }

        /* Draw objects */
        float max_obj_speed = 0.0f;
        int submerged_count = 0;
        int mostly_submerged_count = 0;
        int floating_near_surface_count = 0;
        float sum_submerge = 0.0f;
        float sum_drift_x = 0.0f;
        float sum_depth = 0.0f;
        for (int i = 0; i < N_OBJS; i++) {
            const obj_desc_t *d = &descs[i];
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(objs[i], &p);
            ivp_object_get_orientation(objs[i], &q);

            float fp[3]; vec3r_to_float3(&p, fp);

            if (d->is_sphere) {
                ivp_draw_wire_sphere(renderer, fp, d->hs[0], d->col);
            } else {
                float rot[9]; quat_to_col_rot(&q, rot);
                float hs[3] = {d->hs[0], d->hs[1], d->hs[2]};
                ivp_draw_wire_box(renderer, fp, hs, rot, d->col);
            }

            /* Velocity arrow */
            ivp_vec3r_t v;
            ivp_object_get_speed(objs[i], &v);
            float speed = vec3r_speed(&v);
            if (speed > max_obj_speed) max_obj_speed = speed;
            if (speed > 0.4f) {
                float sc = 0.3f;
                float to[3] = {fp[0]+(float)v.k[0]*sc,
                               fp[1]+(float)v.k[1]*sc,
                               fp[2]+(float)v.k[2]*sc};
                ivp_draw_arrow(renderer, fp, to, c_vel, 0.2f);
            }

            /* Upward buoyancy indicator when submerged.
             * For boxes, use vertical half-height (max extent) instead of
             * the diagonal radius for a more meaningful submersion estimate. */
            float radius = d->is_sphere
                ? d->hs[0]
                : fmaxf(d->hs[0], fmaxf(d->hs[1], d->hs[2]));
            float submerge = 0.0f;
            if (radius > 1e-6f) {
                float dist = WATER_SURFACE_Y - fp[1];
                float h = radius - dist;
                if (h > 0.0f) {
                    float max_h = 2.0f * radius;
                    if (h > max_h) h = max_h;
                    submerge = h / max_h;
                }
            }

            if (submerge > 0.01f) {
                submerged_count++;
                float ba[3] = {fp[0], fp[1], fp[2]};
                float bb[3] = {fp[0], fp[1] - 1.5f, fp[2]};
                ivp_draw_arrow(renderer, ba, bb, c_buoy, 0.2f);
            }
            if (submerge > 0.50f) mostly_submerged_count++;
            if (submerge > 0.10f && submerge < 0.90f && fabsf((float)v.k[1]) < 1.0f)
                floating_near_surface_count++;
            sum_submerge += submerge;
            sum_drift_x += (fp[0] - start_x[i]);
            sum_depth += (fp[1] - WATER_SURFACE_Y);
        }

        if (ivp_example_inspector_tick(&inspector)) {
            ivp_example_inspector_log(&inspector,
                "max_speed=%.3f submerged=%.2f/%d mostly_submerged=%d near_surface=%d mean_drift_x=%.3f mean_depth=%.3f",
                max_obj_speed,
                sum_submerge,
                N_OBJS,
                mostly_submerged_count,
                floating_near_surface_count,
                sum_drift_x / (float)N_OBJS,
                sum_depth / (float)N_OBJS);
        }

        /* Axes at origin */
        {
            float org[3] = {0, 0, 0};
            ivp_draw_axes(renderer, org, NULL, 1.5f);
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus, c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  Objects: %d  Water Y=%.1f",
                     sl.fps_display, N_OBJS, WATER_SURFACE_Y);
            ivp_draw_text_2d(renderer, 10.0f, 10.0f, buf, c_hud);

            for (int i = 0; i < N_OBJS; i++) {
                ivp_vec3r_t p;
                ivp_object_get_position(objs[i], &p);
                float depth = (float)p.k[1] - WATER_SURFACE_Y;
                const char *status = depth > 0.1f ? "SUNK" : (depth > -0.5f ? "FLOATING" : "ABOVE");
                /* Compute object density (kg/m^3) for display.
                 * Volume: sphere = (4/3)pi*r^3, box = 8*hx*hy*hz.
                 * Water density = 1000 kg/m^3; object sinks if density > water. */
                float vol;
                if (descs[i].is_sphere)
                    vol = (4.0f/3.0f) * 3.14159f * descs[i].hs[0] * descs[i].hs[0] * descs[i].hs[0];
                else
                    vol = 8.0f * descs[i].hs[0] * descs[i].hs[1] * descs[i].hs[2];
                float density = (vol > 1e-6f) ? descs[i].mass / vol : 0.0f;
                snprintf(buf, sizeof(buf), "%s  depth:%.1f  %s  d:%.0f",
                         descs[i].label, depth, status, density);
                ivp_draw_text_2d(renderer, 10.0f, 30.0f + 18.0f * (float)i, buf, descs[i].col);
            }

            ivp_draw_text_2d(renderer, 10.0f, 30.0f + 18.0f * N_OBJS + 10.0f,
                "Current:+X 1.5m/s", c_hud);
            ivp_example_draw_interaction_hud(renderer,
                10.0f, 30.0f + 18.0f * N_OBJS + 28.0f, c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    for (int i = 0; i < N_OBJS; i++) {
        if (!descs[i].is_sphere)
            ivp_compact_surface_free(NULL, surfs[i]);
    }
    ivp_compact_surface_free(NULL, wall_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
