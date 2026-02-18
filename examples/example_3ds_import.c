/* example_3ds_import.c -- Load .3ds file → physics simulation
 *
 * Demonstrates the content pipeline: load .3ds → build surface → create
 * environment + ground + dynamic object → simulate with render loop.
 *
 * Usage: example_3ds_import [filename.3ds]
 *        (if no file given, uses a built-in tetrahedron)
 */
#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/builder/ivp_import_3ds.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_surman.h>

#include <stdio.h>
#include <string.h>

/* Inline synthetic tetrahedron .3ds for when no file is provided */
static void w16(unsigned char *buf, int *pos, uint16_t val) {
    buf[*pos] = (unsigned char)(val & 0xFF);
    buf[*pos + 1] = (unsigned char)((val >> 8) & 0xFF);
    *pos += 2;
}
static void w32(unsigned char *buf, int *pos, uint32_t val) {
    buf[*pos] = (unsigned char)(val & 0xFF);
    buf[*pos + 1] = (unsigned char)((val >> 8) & 0xFF);
    buf[*pos + 2] = (unsigned char)((val >> 16) & 0xFF);
    buf[*pos + 3] = (unsigned char)((val >> 24) & 0xFF);
    *pos += 4;
}
static void wf32(unsigned char *buf, int *pos, float val) {
    uint32_t bits;
    memcpy(&bits, &val, sizeof(bits));
    w32(buf, pos, bits);
}

static int build_demo_3ds(unsigned char *buf) {
    int pos = 0;
    float verts[4][3] = {
        {-0.5f, 0.0f, -0.3f}, {0.5f, 0.0f, -0.3f},
        {0.0f, 0.0f, 0.6f}, {0.0f, -0.8f, 0.0f}
    };
    uint16_t faces[4][3] = {{0,2,1},{0,1,3},{1,2,3},{0,3,2}};

    int vertlist_size = 6 + 2 + 4 * 3 * 4;
    int facelist_size = 6 + 2 + 4 * 4 * 2;
    int trimesh_size = 6 + vertlist_size + facelist_size;
    const char *name = "demo";
    int objblock_size = 6 + (int)strlen(name) + 1 + trimesh_size;
    int objmesh_size = 6 + objblock_size;
    int main_size = 6 + objmesh_size;

    w16(buf, &pos, 0x4D4D); w32(buf, &pos, (uint32_t)main_size);
    w16(buf, &pos, 0x3D3D); w32(buf, &pos, (uint32_t)objmesh_size);
    w16(buf, &pos, 0x4000); w32(buf, &pos, (uint32_t)objblock_size);
    for (int i = 0; name[i]; i++) buf[pos++] = (unsigned char)name[i];
    buf[pos++] = '\0';
    w16(buf, &pos, 0x4100); w32(buf, &pos, (uint32_t)trimesh_size);
    w16(buf, &pos, 0x4110); w32(buf, &pos, (uint32_t)vertlist_size);
    w16(buf, &pos, 4);
    for (int i = 0; i < 4; i++) {
        wf32(buf, &pos, verts[i][0]);
        wf32(buf, &pos, verts[i][1]);
        wf32(buf, &pos, verts[i][2]);
    }
    w16(buf, &pos, 0x4120); w32(buf, &pos, (uint32_t)facelist_size);
    w16(buf, &pos, 4);
    for (int i = 0; i < 4; i++) {
        w16(buf, &pos, faces[i][0]);
        w16(buf, &pos, faces[i][1]);
        w16(buf, &pos, faces[i][2]);
        w16(buf, &pos, 0);
    }
    return pos;
}

int main(int argc, char **argv)
{
    /* ── Renderer ──────────────────────────────────────────────────────── */
    ivp_render_config_t rcfg = {
        .width = 1280, .height = 720,
        .title = "IVP 3DS Import Example", .vsync = true
    };
    ivp_renderer_t *renderer = ivp_renderer_create(&rcfg);
    if (!renderer) { fprintf(stderr, "renderer init failed\n"); return 1; }

    ivp_camera_t cam;
    ivp_camera_init(&cam);
    cam.target[1]      = -3.0f;
    cam.orbit_distance  = 15.0f;
    cam.orbit_yaw       = 25.0f;
    cam.orbit_pitch     = -20.0f;

    /* ── Load 3DS ──────────────────────────────────────────────────────── */
    ivp_compact_surface_t *mesh_surface = NULL;
    ivp_3ds_scene_t scene;
    memset(&scene, 0, sizeof(scene));

    if (argc > 1) {
        ivp_result_t r = ivp_3ds_load_file(NULL, argv[1], &scene);
        if (!ivp_ok(r)) {
            fprintf(stderr, "Failed to load %s: %s\n", argv[1], ivp_result_str(r));
            ivp_renderer_destroy(renderer);
            return 1;
        }
        mesh_surface = ivp_3ds_build_surface(NULL, &scene, 0, NULL);
    } else {
        unsigned char buf[1024];
        int sz = build_demo_3ds(buf);
        ivp_result_t r = ivp_3ds_load_memory(NULL, buf, sz, &scene);
        if (ivp_ok(r))
            mesh_surface = ivp_3ds_build_surface(NULL, &scene, 0, NULL);
    }

    if (!mesh_surface) {
        fprintf(stderr, "Failed to build mesh surface\n");
        ivp_3ds_scene_free(&scene);
        ivp_renderer_destroy(renderer);
        return 1;
    }

    /* ── Physics ───────────────────────────────────────────────────────── */
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ivp_environment_t *env = ivp_environment_create(&ecfg);

    /* Ground */
    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 20.0f, 0.5f, 20.0f);
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = IVP_REAL(0.0);
    ivp_vec3r_set(&gt.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ground = ivp_environment_create_object(env, &gt, NULL);

    /* Dynamic imported object */
    ivp_object_template_t ot;
    ivp_object_template_init(&ot);
    ot.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&ot.position, IVP_REAL(0.0), IVP_REAL(-5.0), IVP_REAL(0.0));
    ivp_object_t *dyn = ivp_environment_create_object(env, &ot, NULL);

    /* ── Render visuals (legacy draw helpers) ─────────────────────────── */
    float ground_half[3] = {20.0f, 0.5f, 20.0f};
    float ground_color[3] = {0.4f, 0.5f, 0.4f};
    float dyn_half[3] = {0.5f, 0.5f, 0.5f};
    float dyn_color[3] = {0.9f, 0.3f, 0.2f};

    /* ── Main loop (use ivp_renderer frame loop and sim utilities) ───── */
    sim_loop_t sl;
    sim_loop_init(&sl, renderer);

    while (ivp_renderer_begin_frame(renderer, &cam)) {
        float frame_dt = sim_loop_begin(&sl, renderer);

        /* Step physics at fixed timestep (simple single-step for this example) */
        (void)frame_dt;
        ivp_environment_simulate_dtime(env, IVP_REAL(1.0 / 60.0));

        /* Draw ground */
        {
            const ivp_core_t *c = ivp_object_get_core_const(ground);
            float gp[3] = {(float)c->pos_world_f_core_last_psi.k[0],
                           (float)c->pos_world_f_core_last_psi.k[1],
                           (float)c->pos_world_f_core_last_psi.k[2]};
            float grot[9];
            quat_to_col_rot(&c->q_world_f_core_last_psi, grot);
            ivp_draw_wire_box(renderer, gp, ground_half, grot, ground_color);
        }

        /* Draw imported object */
        {
            const ivp_core_t *c = ivp_object_get_core_const(dyn);
            float dp[3] = {(float)c->pos_world_f_core_last_psi.k[0],
                           (float)c->pos_world_f_core_last_psi.k[1],
                           (float)c->pos_world_f_core_last_psi.k[2]};
            float drot[9];
            quat_to_col_rot(&c->q_world_f_core_last_psi, drot);
            ivp_draw_wire_box(renderer, dp, dyn_half, drot, dyn_color);
        }

        ivp_renderer_end_frame(renderer);
    }

    /* ── Cleanup ───────────────────────────────────────────────────────── */
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, mesh_surface);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_3ds_scene_free(&scene);
    ivp_renderer_destroy(renderer);

    return 0;
}
