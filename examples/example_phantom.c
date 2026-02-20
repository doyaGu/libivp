/* example_phantom.c -- Trigger volume (phantom) demo
 *
 * A stationary sphere phantom at the center (drawn as wireframe) and
 * cubes/spheres dropped from above.  Enter/exit events are tracked.
 * Objects inside the phantom are drawn brighter.  HUD shows the
 * tracked count and last event.
 *
 * Demonstrates: ivp_controller_phantom, sphere phantom, enter/exit
 *               callbacks, tracked object list, HUD event log.
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/controllers/ivp_controller_phantom.h>

#include <stdio.h>
#include <string.h>

/* ── Phantom event tracking ─────────────────────────────────────────────── */

#define EVENT_LOG_SIZE 5

typedef struct {
    int  total_enters;
    int  total_exits;
    char last_events[EVENT_LOG_SIZE][64];
    int  log_head;
} phantom_tracker_t;

static void on_phantom_event(void *ctx,
                              ivp_object_t *phantom_owner,
                              ivp_object_t *other,
                              bool entered)
{
    (void)phantom_owner;
    phantom_tracker_t *tr = ctx;
    if (entered) tr->total_enters++;
    else         tr->total_exits++;

    const char *name = other->name ? other->name : "?";
    snprintf(tr->last_events[tr->log_head], 64,
             "%s %s", entered ? "ENTER" : "EXIT", name);
    tr->log_head = (tr->log_head + 1) % EVENT_LOG_SIZE;
}

#define N_DROP 10

int main(void)
{
    /* ── Renderer ───────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP Phantom -- Trigger Volumes", .vsync = true
    };
    ivp_renderer_t *renderer = NULL;
    ivp_renderer_create(&renderer, &rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -4.0f;
    cam.orbit_distance  = 25.0f;
    cam.orbit_yaw       = 20.0f;
    cam.orbit_pitch     = -25.0f;

    /* ── Physics ────────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ivp_environment_t *env = ivp_environment_create(&ecfg);
    ivp_example_inspector_t inspector;
    ivp_example_inspector_init(&inspector, "phantom");

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_compact_surface_t *ground_surf =
        ivp_build_box_surface(NULL, 15.0f, 0.5f, 15.0f);
    ivp_surman_polygon_t sm_ground;
    create_ground_object(env, ground_surf, &sm_ground, IVP_REAL(0.0));

    /* ── Phantom: sphere at center, radius 3 ────────────────────────────── */
    phantom_tracker_t tracker;
    memset(&tracker, 0, sizeof(tracker));

    ivp_phantom_state_t phantom;
    {
        ivp_vec3r_t center = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-3.0), IVP_REAL(0.0));
        ivp_phantom_init_sphere(&phantom, &center, IVP_REAL(3.0),
                                  on_phantom_event, &tracker);
    }
    ivp_environment_add_controller(env, 100,
        ivp_controller_phantom, &phantom);

    /* ── Drop objects: cubes and spheres from varying heights ────────────── */
    ivp_compact_surface_t *drop_cube_surf =
        ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);

    ivp_surman_polygon_t sm_drop_p[N_DROP];
    ivp_surman_ball_t    sm_drop_b[N_DROP];
    ivp_object_t *drop_objs[N_DROP];
    bool drop_is_sphere[N_DROP];
    char drop_names[N_DROP][16];

    const float drop_cols[][3] = {
        {1.0f, 0.4f, 0.2f}, {0.3f, 1.0f, 0.3f}, {0.3f, 0.5f, 1.0f},
        {1.0f, 1.0f, 0.2f}, {1.0f, 0.2f, 0.8f}, {0.2f, 1.0f, 0.9f},
        {0.9f, 0.6f, 0.1f}, {0.6f, 0.3f, 1.0f},
        {0.8f, 0.8f, 0.4f}, {0.4f, 0.8f, 0.8f}
    };

    for (int i = 0; i < N_DROP; i++) {
        snprintf(drop_names[i], sizeof(drop_names[i]), "obj_%d", i);
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.mass = IVP_REAL(1.0) + IVP_REAL(0.5) * (ivp_real_t)i;
        t.material.friction   = IVP_REAL(0.5);
        t.material.elasticity = IVP_REAL(0.4);
        t.name = drop_names[i];

        float angle = 6.2832f * (float)i / (float)N_DROP;
        float r = 2.0f + 3.0f * (float)(i % 3);
        t.position = ivp_vec3r(
            (ivp_real_t)(r * cosf(angle)),
            IVP_REAL(-8.0) - IVP_REAL(2.0) * (ivp_real_t)(i % 4),
            (ivp_real_t)(r * sinf(angle)));

        if (i % 2 == 0) {
            /* Cube */
            t.type = IVP_OBJECT_POLYGON;
            ivp_surman_polygon_init(&sm_drop_p[i], drop_cube_surf);
            set_box_inertia(&t, t.mass, IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(0.5));
            drop_objs[i] = ivp_environment_create_object(env, &t, &sm_drop_p[i].base);
            drop_is_sphere[i] = false;
        } else {
            /* Sphere */
            t.type = IVP_OBJECT_BALL;
            ivp_surman_ball_init(&sm_drop_b[i], 0.5f);
            set_sphere_inertia(&t, t.mass, IVP_REAL(0.5));
            drop_objs[i] = ivp_environment_create_object(env, &t, &sm_drop_b[i].base);
            drop_is_sphere[i] = true;
        }
        /* Initial downward velocity for natural drop appearance */
        drop_objs[i]->core.speed = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.5), IVP_REAL(0.0));
    }

    /* ── Colors ─────────────────────────────────────────────────────────── */
    const float c_ground[]   = {0.22f, 0.24f, 0.30f};
    const float c_grid[]     = {0.38f, 0.43f, 0.52f};
    const float c_hud[]      = {0.95f, 0.97f, 1.0f};
    const float c_phantom[]  = {1.0f, 1.0f, 0.2f};
    const float c_bright[]   = {1.0f, 1.0f, 1.0f};
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

        sim_loop_step(&sl, env, frame_dt);

        /* Grid & ground */
        ivp_render_draw_ground_grid(renderer, 15.0f, 2.0f, c_grid);
        {
            float gp[3] = {0, 0, 0}, gs[3] = {15.0f, 0.5f, 15.0f};
            ivp_render_draw_wire_box(renderer, gp, gs, NULL, c_ground);
        }

        /* Phantom volume (wireframe sphere) */
        {
            float pc[3] = {(float)phantom.center_ws.k[0],
                           (float)phantom.center_ws.k[1],
                           (float)phantom.center_ws.k[2]};
            ivp_render_draw_wire_sphere(renderer, pc,
                (float)phantom.radius, c_phantom);
        }

        /* Drop objects */
        for (int i = 0; i < N_DROP; i++) {
            ivp_vec3r_t p; ivp_quatr_t q;
            ivp_object_get_position(drop_objs[i], &p);
            ivp_object_get_orientation(drop_objs[i], &q);
            float fp[3]; vec3r_to_float3(&p, fp);

            /* Check if inside phantom */
            bool inside = false;
            for (int t = 0; t < phantom.n_tracked; t++) {
                if (phantom.tracked[t] == drop_objs[i]) {
                    inside = true;
                    break;
                }
            }
            const float *col = inside ? c_bright : drop_cols[i];

            if (drop_is_sphere[i]) {
                ivp_render_draw_wire_sphere(renderer, fp, 0.5f, col);
            } else {
                float rot[9]; quat_to_col_rot(&q, rot);
                float hs[3] = {0.5f, 0.5f, 0.5f};
                ivp_render_draw_wire_box(renderer, fp, hs, rot, col);
            }

            /* Velocity arrow */
            ivp_vec3r_t v;
            ivp_object_get_speed(drop_objs[i], &v);
            float speed = vec3r_speed(&v);
            if (speed > 0.5f) {
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

        if (ivp_example_inspector_tick(&inspector)) {
            ivp_example_inspector_log(&inspector,
                "tracked=%d enters=%d exits=%d",
                phantom.n_tracked,
                tracker.total_enters,
                tracker.total_exits);
        }

        ivp_example_interaction_draw_overlays(renderer, &dragger, &focus,
                                              c_drag, c_focus);

        /* HUD */
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "FPS: %.0f  Objects: %d  Time: %.1fs",
                     sl.fps_display, N_DROP, (float)sl.t_prev);
            ivp_render_draw_text(renderer, 10.0f, 10.0f, buf, c_hud);

            snprintf(buf, sizeof(buf), "Phantom: tracked=%d  enters=%d  exits=%d",
                     phantom.n_tracked,
                     tracker.total_enters,
                     tracker.total_exits);
            ivp_render_draw_text(renderer, 10.0f, 30.0f, buf, c_phantom);

            /* Event log */
            for (int i = 0; i < EVENT_LOG_SIZE; i++) {
                int idx = (tracker.log_head - 1 - i + EVENT_LOG_SIZE)
                          % EVENT_LOG_SIZE;
                if (tracker.last_events[idx][0] != '\0') {
                    ivp_render_draw_text(renderer, 10.0f, 50.0f + 18.0f * (float)i,
                                     tracker.last_events[idx], c_hud);
                }
            }

            ivp_example_draw_interaction_hud(renderer, 10.0f,
                50.0f + 18.0f * EVENT_LOG_SIZE + 8.0f, c_hud, c_drag);
        }

        ivp_renderer_end_frame(renderer);
    }

    ivp_example_dragger_detach(&dragger, env);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, drop_cube_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_renderer_destroy(renderer);
    return 0;
}
