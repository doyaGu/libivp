/* ivp_render_core.c -- Renderer lifecycle, frame loop, event dispatch
 *
 * Owns the full lifecycle of ivp_renderer_t: creation of the window, the GPU
 * device (ivp_render_gpu.h), the streams and the solid instanced pipelines,
 * the frame loop (begin/end), and teardown.
 * Stats and projection also live here (trivial, no separate file needed).
 */

#include "ivp_render_internal.h"
#include <ivp/base/ivp_allocator.h>

#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include <math.h>

/* ── Config ────────────────────────────────────────────────────────────── */

void ivp_render_config_init(ivp_render_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->width             = IVP_RENDER_DEFAULT_WIDTH;
    cfg->height            = IVP_RENDER_DEFAULT_HEIGHT;
    cfg->title             = "IVP Example";
    cfg->vsync             = true;
    cfg->msaa_samples      = 4;
    cfg->clear_color[0]    = 0.20f;
    cfg->clear_color[1]    = 0.24f;
    cfg->clear_color[2]    = 0.30f;
    cfg->clear_color[3]    = 1.0f;
    cfg->max_frame_dt      = 0.1f;
    cfg->max_draw_vertices = IVP_RENDER_DEFAULT_MAX_DRAW_VERTS;
    cfg->max_hud_vertices  = IVP_RENDER_DEFAULT_MAX_HUD_VERTS;
    cfg->max_tri_vertices  = IVP_RENDER_DEFAULT_MAX_TRI_VERTS;
    cfg->sphere_segments   = IVP_RENDER_DEFAULT_SPHERE_SEGMENTS;
    cfg->hud_font_scale    = IVP_RENDER_DEFAULT_HUD_FONT_SCALE;
}

/* ── Resolve defaults ──────────────────────────────────────────────────── */

static ivp_render_config_t resolve_config(const ivp_render_config_t *c)
{
    ivp_render_config_t d;
    ivp_render_config_init(&d);

    if (c->width  > 0) d.width  = c->width;
    if (c->height > 0) d.height = c->height;
    if (c->title)      d.title  = c->title;
    d.vsync = c->vsync;

    if (c->msaa_samples < 0)      d.msaa_samples = 0;
    else if (c->msaa_samples > 0) d.msaa_samples = c->msaa_samples;

    if (c->max_frame_dt > 0.0f) d.max_frame_dt = c->max_frame_dt;

    if (c->clear_color[0] != 0.0f || c->clear_color[1] != 0.0f ||
        c->clear_color[2] != 0.0f || c->clear_color[3] != 0.0f) {
        for (int i = 0; i < 4; i++) d.clear_color[i] = c->clear_color[i];
    }

    if (c->max_draw_vertices > 0) d.max_draw_vertices = c->max_draw_vertices;
    if (c->max_hud_vertices  > 0) d.max_hud_vertices  = c->max_hud_vertices;
    if (c->sphere_segments   > 0) d.sphere_segments    = c->sphere_segments;
    if (c->hud_font_scale    > 0) d.hud_font_scale     = c->hud_font_scale;

    return d;
}

/* ── Vertex layout helpers ─────────────────────────────────────────────── */

typedef struct { float pos[3]; float col[3]; } draw_vertex_t;
typedef struct { float pos[2]; float col[3]; } hud_vertex_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

/* Per-instance layout of the solid and shadow pipelines: the proto mesh
 * vertex (pos[3] + nrm[3]) and ivp_solid_instance_t. */
static const ivp_gpu_attrib_desc_t solid_vtx_attrs[2] = {
    { 0, 3, 0                      },  /* aPos */
    { 1, 3, 3 * (int)sizeof(float) },  /* aNrm */
};
static const ivp_gpu_attrib_desc_t solid_inst_attrs[5] = {
    { 2, 4, 0                       },  /* iRow0 */
    { 3, 4, 4  * (int)sizeof(float) },  /* iRow1 */
    { 4, 4, 8  * (int)sizeof(float) },  /* iRow2 */
    { 5, 4, 12 * (int)sizeof(float) },  /* iRow3 */
    { 6, 4, 16 * (int)sizeof(float) },  /* iCol: rgb + alpha */
};

void ivp_render_solid_pipeline_desc(ivp_gpu_pipeline_desc_t *desc,
                                    ivp_gpu_program_t program)
{
    memset(desc, 0, sizeof(*desc));
    desc->program           = program;
    desc->primitive         = IVP_GPU_PRIMITIVE_TRIANGLES;
    desc->stride            = 6 * (int)sizeof(float);   /* proto vertex */
    desc->attribs           = solid_vtx_attrs;
    desc->attrib_count      = 2;
    desc->inst_stride       = (int)sizeof(ivp_solid_instance_t);
    desc->inst_attribs      = solid_inst_attrs;
    desc->inst_attrib_count = 5;
    desc->depth_test        = true;
    desc->depth_write       = true;
}

ivp_result_t ivp_renderer_create(ivp_renderer_t **out,
                                 const ivp_render_config_t *config)
{
    if (!out || !config) return IVP_ERR_INVALID_ARG;

    ivp_render_config_t cfg = resolve_config(config);

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return IVP_ERR_RENDER_SDL_INIT;
    }

    SDL_Window *win = SDL_CreateWindow(cfg.title, cfg.width, cfg.height,
                                       SDL_WINDOW_RESIZABLE |
                                       ivp_gpu_window_flags(cfg.msaa_samples));
    if (!win) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return IVP_ERR_RENDER_WINDOW;
    }

    const ivp_allocator_t *alloc = ivp_default_allocator();
    ivp_renderer_t *r = (ivp_renderer_t *)ivp_alloc(alloc, sizeof(*r));
    if (!r) { SDL_DestroyWindow(win); return IVP_ERR_OUT_OF_MEMORY; }
    memset(r, 0, sizeof(*r));
    r->window = win;
    /* The window may not get the requested size: in the browser a canvas
     * sized by the page's CSS decides it. */
    r->width  = cfg.width;
    r->height = cfg.height;
    SDL_GetWindowSize(win, &r->width, &r->height);

    /* GPU device, with its swapchain and shaders */
    ivp_result_t res = ivp_gpu_device_create(&r->gpu, win, cfg.msaa_samples,
                                             cfg.vsync);
    if (res != IVP_OK) goto fail_win;

    /* 3D stream (depth test + write) */
    {
        ivp_gpu_attrib_desc_t attrs[2] = {
            { 0, 3, (int)offsetof(draw_vertex_t, pos) },
            { 1, 3, (int)offsetof(draw_vertex_t, col) },
        };
        ivp_gpu_pipeline_desc_t desc = {0};
        desc.program      = IVP_GPU_PROGRAM_LINE3D;
        desc.primitive    = IVP_GPU_PRIMITIVE_LINES;
        desc.stride       = (int)sizeof(draw_vertex_t);
        desc.attribs      = attrs;
        desc.attrib_count = 2;
        desc.depth_test   = true;
        desc.depth_write  = true;

        res = ivp_gpu_stream_create(&r->stream_3d, r->gpu, &desc,
                                    cfg.max_draw_vertices);
        if (res != IVP_OK) goto fail_gpu;
    }

    /* 2D HUD stream (no depth) */
    {
        ivp_gpu_attrib_desc_t attrs[2] = {
            { 0, 2, (int)offsetof(hud_vertex_t, pos) },
            { 1, 3, (int)offsetof(hud_vertex_t, col) },
        };
        ivp_gpu_pipeline_desc_t desc = {0};
        desc.program      = IVP_GPU_PROGRAM_LINE2D;
        desc.primitive    = IVP_GPU_PRIMITIVE_LINES;
        desc.stride       = (int)sizeof(hud_vertex_t);
        desc.attribs      = attrs;
        desc.attrib_count = 2;

        res = ivp_gpu_stream_create(&r->stream_2d, r->gpu, &desc,
                                    cfg.max_hud_vertices);
        if (res != IVP_OK) goto fail_3d;
    }

    /* Instanced solid pipeline (for the proto meshes and user meshes) */
    {
        ivp_gpu_pipeline_desc_t desc;
        ivp_render_solid_pipeline_desc(&desc, IVP_GPU_PROGRAM_SOLID);
        desc.shadow_sampler = true;

        r->inst_pipeline = ivp_gpu_pipeline_create(r->gpu, &desc);
        if (!r->inst_pipeline) { res = IVP_ERR_RENDER_SHADER; goto fail_2d; }

        /* Same shaders, alpha blended, depth tested but not written */
        desc.blend       = true;
        desc.depth_write = false;
        r->inst_pipeline_blend = ivp_gpu_pipeline_create(r->gpu, &desc);
        if (!r->inst_pipeline_blend) {
            ivp_gpu_pipeline_destroy(r->gpu, r->inst_pipeline);
            res = IVP_ERR_RENDER_SHADER;
            goto fail_2d;
        }
    }

    /* Proto shapes (icosphere + unit box + instance buffers) */
    res = ivp_proto_shapes_init(&r->protos, r->gpu);
    if (res != IVP_OK) goto fail_inst;

    /* Overlay lines: same layout as the 3D stream, no depth test */
    {
        ivp_gpu_attrib_desc_t attrs[2] = {
            { 0, 3, (int)offsetof(draw_vertex_t, pos) },
            { 1, 3, (int)offsetof(draw_vertex_t, col) },
        };
        ivp_gpu_pipeline_desc_t desc = {0};
        desc.program      = IVP_GPU_PROGRAM_LINE3D;
        desc.primitive    = IVP_GPU_PRIMITIVE_LINES;
        desc.stride       = (int)sizeof(draw_vertex_t);
        desc.attribs      = attrs;
        desc.attrib_count = 2;
        desc.depth_always = true;   /* on top of everything, but in the same pass */
        res = ivp_gpu_stream_create(&r->stream_overlay, r->gpu, &desc,
                                    cfg.max_draw_vertices / 4);
        if (res != IVP_OK) {
            ivp_proto_shapes_destroy(&r->protos, r->gpu);
            goto fail_inst;
        }
    }

    /* 2D filled rectangles in three layers.  Position + rgba, blended. */
    {
        ivp_gpu_attrib_desc_t attrs[2] = {
            { 0, 2, 0 },
            { 1, 4, 2 * (int)sizeof(float) },
        };
        const int caps[3] = { 64, 4096, cfg.max_hud_vertices * 4 };
        for (int i = 0; i < 3; i++) {
            ivp_gpu_pipeline_desc_t desc = {0};
            desc.program      = IVP_GPU_PROGRAM_RECT;
            desc.primitive    = IVP_GPU_PRIMITIVE_TRIANGLES;
            desc.stride       = 6 * (int)sizeof(float);
            desc.attribs      = attrs;
            desc.attrib_count = 2;
            desc.blend        = true;
            desc.depth_always = true;   /* same pass as the scene: keep its depth target */
            res = ivp_gpu_stream_create(&r->stream_rect[i], r->gpu, &desc, caps[i]);
            if (res != IVP_OK) {
                for (int k = 0; k < i; k++)
                    ivp_gpu_stream_destroy(&r->stream_rect[k], r->gpu);
                ivp_gpu_stream_destroy(&r->stream_overlay, r->gpu);
                ivp_proto_shapes_destroy(&r->protos, r->gpu);
                goto fail_inst;
            }
        }
    }

    /* Shadow map and depth pass; the renderer works without them */
    if (!ivp_shadow_init(r))
        fprintf(stderr, "shadows unavailable\n");

    /* Lighting defaults */
    {
        /* IVP worlds are Y down (gravity +Y, camera up -Y): light from above */
        float default_dir[3] = { 0.40f, -0.80f, -0.45f };
        memcpy(r->lighting.light_dir, default_dir, sizeof(default_dir));
        r->lighting.ambient = 0.45f;
    }

    /* Timing */
    r->start_ticks    = SDL_GetPerformanceCounter();
    r->prev_ticks     = 0;
    r->max_frame_dt   = cfg.max_frame_dt;

    /* Config */
    for (int i = 0; i < 4; i++) r->clear_col[i] = cfg.clear_color[i];
    r->sphere_segments = cfg.sphere_segments;
    r->hud_font_scale  = cfg.hud_font_scale;

    ivp_camera_input_init(&r->cam_input);

    *out = r;
    return IVP_OK;

fail_inst:
    ivp_gpu_pipeline_destroy(r->gpu, r->inst_pipeline_blend);
    ivp_gpu_pipeline_destroy(r->gpu, r->inst_pipeline);
fail_2d:
    ivp_gpu_stream_destroy(&r->stream_2d, r->gpu);
fail_3d:
    ivp_gpu_stream_destroy(&r->stream_3d, r->gpu);
fail_gpu:
    ivp_gpu_device_destroy(r->gpu);
fail_win:
    SDL_DestroyWindow(win);
    ivp_free(alloc, r, sizeof(*r));
    return res;
}

void ivp_renderer_destroy(ivp_renderer_t *r)
{
    if (!r) return;
    const ivp_allocator_t *alloc = ivp_default_allocator();

    /* Wait for GPU idle before destroying resources */
    ivp_gpu_wait_idle(r->gpu);

    ivp_shadow_destroy(r);
    for (int i = 0; i < 3; i++) ivp_gpu_stream_destroy(&r->stream_rect[i], r->gpu);
    ivp_gpu_stream_destroy(&r->stream_overlay, r->gpu);
    ivp_solid_destroy_meshes(r);
    ivp_proto_shapes_destroy(&r->protos, r->gpu);
    ivp_gpu_pipeline_destroy(r->gpu, r->inst_pipeline);
    ivp_gpu_pipeline_destroy(r->gpu, r->inst_pipeline_blend);
    r->inst_pipeline = NULL;
    r->inst_pipeline_blend = NULL;
    ivp_gpu_stream_destroy(&r->stream_2d, r->gpu);
    ivp_gpu_stream_destroy(&r->stream_3d, r->gpu);
    ivp_gpu_device_destroy(r->gpu);
    SDL_DestroyWindow(r->window);
    ivp_free(alloc, r, sizeof(*r));
    SDL_Quit();
}

/* ── Frame loop ────────────────────────────────────────────────────────── */

void ivp_renderer_set_camera(ivp_renderer_t *r, const ivp_camera_t *cam)
{
    if (!r || !cam) return;
    ivp_camera_compute_matrices(cam, r->width, r->height,
                                r->view_mat, r->proj_mat, r->vp_mat);
    memcpy(r->cam_target, cam->target, sizeof(r->cam_target));
    r->cam_dist = cam->mode == IVP_CAMERA_MODE_FLY ? 30.0f : cam->orbit_distance;
    ivp_frustum_extract(&r->frustum, r->vp_mat);
}

bool ivp_renderer_begin_frame(ivp_renderer_t *r, ivp_camera_t *cam)
{
    /* Timing */
    Uint64 now = SDL_GetPerformanceCounter();
    if (r->prev_ticks == 0) r->prev_ticks = now;
    r->frame_dt = (float)((double)(now - r->prev_ticks) /
                          (double)SDL_GetPerformanceFrequency());
    r->prev_ticks = now;
    if (r->frame_dt > r->max_frame_dt) r->frame_dt = r->max_frame_dt;

    /* Pre-events */
    if (r->callbacks.pre_events_cb)
        r->callbacks.pre_events_cb(r->callbacks.pre_events_cb_ud);

    /* Events */
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_EVENT_QUIT) return false;
        if (r->callbacks.event_cb &&
            r->callbacks.event_cb(&ev, r->callbacks.event_cb_ud))
            continue;
        ivp_camera_process_event(cam, &r->cam_input, &ev,
                                 &r->width, &r->height);
    }

    /* Post-events */
    if (r->callbacks.post_events_cb)
        r->callbacks.post_events_cb(r->callbacks.post_events_cb_ud);

    /* Camera update */
    ivp_camera_apply_key_pan(cam, &r->cam_input);
    if (cam) {
        ivp_camera_update_orbit(cam);
        if (cam->mode == IVP_CAMERA_MODE_FLY)
            ivp_camera_update_fly(cam);
    }
    ivp_renderer_set_camera(r, cam);

    /* Wait for the GPU and a target to draw into */
    r->frame_active = false;
    ivp_gpu_frame_status_t status = ivp_gpu_frame_begin(r->gpu);
    if (status == IVP_GPU_FRAME_ERROR) return false;
    if (status == IVP_GPU_FRAME_SKIP) return true;
    r->frame_active = true;

    /* Reset streams + instance buffers */
    ivp_gpu_stream_reset(&r->stream_3d);
    ivp_gpu_stream_reset(&r->stream_2d);
    ivp_gpu_stream_reset(&r->stream_overlay);
    for (int i = 0; i < 3; i++) ivp_gpu_stream_reset(&r->stream_rect[i]);
    r->overlay_mode = false;
    ivp_solid_begin_frame(r);
    r->culled_count = 0;
    return true;
}

void ivp_renderer_end_frame(ivp_renderer_t *r)
{
    if (!r->frame_active) return;
    r->frame_active = false;

    /* Sky gradient: a window sized quad in the first rectangle layer */
    if (r->backdrop_on) ivp_hud_push_backdrop(r);

    /* Upload all streams + instance buffers */
    ivp_gpu_stream_upload(&r->stream_3d, r->gpu);
    ivp_gpu_stream_upload(&r->stream_2d, r->gpu);
    ivp_gpu_stream_upload(&r->stream_overlay, r->gpu);
    for (int i = 0; i < 3; i++) ivp_gpu_stream_upload(&r->stream_rect[i], r->gpu);
    ivp_solid_upload(r);

    /* Depth pass from the light, before the main pass reads it */
    ivp_shadow_render(r);

    /* A capture frame is read back after it is submitted */
    bool capturing = ivp_capture_begin(r);

    ivp_gpu_pass_t *pass = ivp_gpu_main_pass_begin(r->gpu, r->clear_col,
                                                   r->width, r->height);
    if (!pass) {
        ivp_gpu_frame_cancel(r->gpu);
        return;
    }

    /* 2D projection shared by the rectangle layers and the HUD lines */
    float ortho[16];
    ivp_render_mat4_ortho(ortho, 0.0f, (float)r->width, (float)r->height, 0.0f, -1.0f, 1.0f);

    /* ── Backdrop first, the scene draws over it ───────────────────────── */
    ivp_gpu_stream_draw(&r->stream_rect[IVP_RECT_BACKDROP], pass, ortho, 64);

    /* ── 3D wireframe draw (uniform = VP matrix, 64 bytes) ────────────── */
    ivp_gpu_stream_draw(&r->stream_3d, pass, r->vp_mat, 64);

    /* ── Instanced solid draws (sphere + box proto meshes) ────────────── */
    if (r->protos.ready && r->inst_pipeline) {
        struct {
            float vp[16];
            float light_dir[4];
            float light_vp[16];
            float shadow[4];
        } inst_uniform;
        memset(&inst_uniform, 0, sizeof(inst_uniform));
        memcpy(inst_uniform.vp, r->vp_mat, 64);
        memcpy(inst_uniform.light_dir, r->lighting.light_dir, 12);
        inst_uniform.light_dir[3] = r->lighting.ambient;
        memcpy(inst_uniform.light_vp, r->light_vp, 64);
        inst_uniform.shadow[0] = r->shadow_valid ? 1.0f : 0.0f;

        ivp_gpu_bind_pipeline(pass, r->inst_pipeline);
        ivp_gpu_push_uniforms(pass, &inst_uniform, sizeof(inst_uniform));
        if (r->shadow_map) ivp_gpu_bind_shadow_map(pass, r->shadow_map);

        ivp_solid_draw(r, pass);

        /* translucent solids over the opaque ones */
        if (r->inst_pipeline_blend) {
            ivp_gpu_bind_pipeline(pass, r->inst_pipeline_blend);
            ivp_gpu_push_uniforms(pass, &inst_uniform, sizeof(inst_uniform));
            if (r->shadow_map) ivp_gpu_bind_shadow_map(pass, r->shadow_map);
            ivp_solid_draw_blend(r, pass);
        }
    }

    /* ── Overlay lines: after the solids, no depth test ──────────────── */
    ivp_gpu_stream_draw(&r->stream_overlay, pass, r->vp_mat, 64);

    /* ── 2D HUD: panels, then text and widgets, then lines ──────────────── */
    ivp_gpu_stream_draw(&r->stream_rect[IVP_RECT_PANEL], pass, ortho, 64);
    ivp_gpu_stream_draw(&r->stream_rect[IVP_RECT_FRONT], pass, ortho, 64);
    ivp_gpu_stream_draw(&r->stream_2d, pass, ortho, 64);

    ivp_gpu_pass_end(pass);

    /* Pre-swap */
    if (r->callbacks.pre_swap_cb)
        r->callbacks.pre_swap_cb(r->callbacks.pre_swap_cb_ud);

    ivp_gpu_frame_end(r->gpu);
    if (capturing) ivp_capture_finish(r);
}

/* ── Accessors ─────────────────────────────────────────────────────────── */

double ivp_renderer_get_time(const ivp_renderer_t *r)
{
    Uint64 now = SDL_GetPerformanceCounter();
    return (double)(now - r->start_ticks) / (double)SDL_GetPerformanceFrequency();
}

float ivp_renderer_get_dt(const ivp_renderer_t *r)
{
    return r->frame_dt;
}

void ivp_renderer_get_size(const ivp_renderer_t *r, int *w, int *h)
{
    if (w) *w = r->width;
    if (h) *h = r->height;
}

const float *ivp_renderer_get_vp_matrix(const ivp_renderer_t *r)
{
    return r->vp_mat;
}

const float *ivp_renderer_get_view_matrix(const ivp_renderer_t *r)
{
    return r->view_mat;
}

SDL_Window *ivp_renderer_get_window(const ivp_renderer_t *r)
{
    return r ? r->window : NULL;
}

void ivp_renderer_set_clear_color(ivp_renderer_t *r,
                                  float red, float green,
                                  float blue, float alpha)
{
    if (!r) return;
    if (red   < 0.0f) red   = 0.0f; if (red   > 1.0f) red   = 1.0f;
    if (green < 0.0f) green = 0.0f; if (green > 1.0f) green = 1.0f;
    if (blue  < 0.0f) blue  = 0.0f; if (blue  > 1.0f) blue  = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f; if (alpha > 1.0f) alpha = 1.0f;
    r->clear_col[0] = red;
    r->clear_col[1] = green;
    r->clear_col[2] = blue;
    r->clear_col[3] = alpha;
}

/* ── Callback setters ──────────────────────────────────────────────────── */

void ivp_renderer_set_event_callback(ivp_renderer_t *r,
                                     ivp_event_callback_t cb, void *userdata)
{
    if (!r) return;
    r->callbacks.event_cb = cb;
    r->callbacks.event_cb_ud = userdata;
}

void ivp_renderer_set_pre_events_callback(ivp_renderer_t *r,
                                          ivp_frame_callback_t cb, void *ud)
{
    if (!r) return;
    r->callbacks.pre_events_cb = cb;
    r->callbacks.pre_events_cb_ud = ud;
}

void ivp_renderer_set_post_events_callback(ivp_renderer_t *r,
                                           ivp_frame_callback_t cb, void *ud)
{
    if (!r) return;
    r->callbacks.post_events_cb = cb;
    r->callbacks.post_events_cb_ud = ud;
}

void ivp_renderer_set_pre_swap_callback(ivp_renderer_t *r,
                                        ivp_frame_callback_t cb, void *ud)
{
    if (!r) return;
    r->callbacks.pre_swap_cb = cb;
    r->callbacks.pre_swap_cb_ud = ud;
}

/* ── Projection ────────────────────────────────────────────────────────── */

bool ivp_renderer_project(const ivp_renderer_t *r,
                          const float world[3], float screen[2])
{
    const float *m = r->vp_mat;
    float x = m[0]*world[0] + m[4]*world[1] + m[8]*world[2]  + m[12];
    float y = m[1]*world[0] + m[5]*world[1] + m[9]*world[2]  + m[13];
    float w = m[3]*world[0] + m[7]*world[1] + m[11]*world[2] + m[15];
    if (w <= 0.0001f) return false;
    float inv_w = 1.0f / w;
    screen[0] = (x * inv_w * 0.5f + 0.5f) * (float)r->width;
    screen[1] = (1.0f - (y * inv_w * 0.5f + 0.5f)) * (float)r->height;
    return true;
}

/* ── Stats ─────────────────────────────────────────────────────────────── */

void ivp_render_stats_get(const ivp_renderer_t *r, ivp_render_stats_t *stats)
{
    if (!r || !stats) return;

    stats->draw_vertex_count    = r->stream_3d.vertex_count;
    stats->draw_vertex_capacity = r->stream_3d.capacity;
    stats->draw_utilization     = (stats->draw_vertex_capacity > 0)
        ? (float)stats->draw_vertex_count / (float)stats->draw_vertex_capacity
        : 0.0f;

    stats->hud_vertex_count    = r->stream_2d.vertex_count;
    stats->hud_vertex_capacity = r->stream_2d.capacity;
    stats->hud_utilization     = (stats->hud_vertex_capacity > 0)
        ? (float)stats->hud_vertex_count / (float)stats->hud_vertex_capacity
        : 0.0f;

    stats->tri_vertex_count    = 0;   /* no triangle stream */
    stats->tri_vertex_capacity = 0;
    stats->tri_utilization     = 0.0f;

    stats->solid_sphere_count = r->protos.ready
        ? r->protos.sphere_instances.count : 0;
    stats->solid_box_count    = r->protos.ready
        ? r->protos.box_instances.count : 0;
    stats->culled_count = r->culled_count;

    stats->frame_dt = r->frame_dt;
    stats->fps = (stats->frame_dt > 1e-6f) ? 1.0f / stats->frame_dt : 0.0f;
}
