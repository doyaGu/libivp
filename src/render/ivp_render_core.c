/* ivp_render_core.c -- Renderer lifecycle, frame loop, event dispatch
 *
 * Owns the full lifecycle of ivp_renderer_t: creation of GPU device,
 * shader cache, unified streams, frame loop (begin/end), and teardown.
 * Stats and projection also live here (trivial, no separate file needed).
 */

#include "ivp_render_internal.h"
#include <ivp/base/ivp_allocator.h>

#include <string.h>
#include <stdio.h>
#include <stddef.h>

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
                                       SDL_WINDOW_RESIZABLE);
    if (!win) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return IVP_ERR_RENDER_WINDOW;
    }

    const ivp_allocator_t *alloc = ivp_default_allocator();
    ivp_renderer_t *r = (ivp_renderer_t *)ivp_alloc(alloc, sizeof(*r));
    if (!r) { SDL_DestroyWindow(win); return IVP_ERR_OUT_OF_MEMORY; }
    memset(r, 0, sizeof(*r));
    r->window = win;
    r->width  = cfg.width;
    r->height = cfg.height;

    /* GPU device */
    ivp_result_t res = ivp_gpu_device_init(&r->gpu, win, cfg.msaa_samples);
    if (res != IVP_OK) goto fail_win;

    /* Vsync */
    SDL_SetGPUSwapchainParameters(r->gpu.device, win,
        SDL_GPU_SWAPCHAINCOMPOSITION_SDR,
        cfg.vsync ? SDL_GPU_PRESENTMODE_VSYNC
                  : SDL_GPU_PRESENTMODE_IMMEDIATE);

    /* Shader cache */
    res = ivp_shader_cache_load(&r->shaders, r->gpu.shader_format);
    if (res != IVP_OK) goto fail_gpu;

    /* 3D stream (depth test + write) */
    {
        ivp_gpu_attrib_desc_t attrs[2] = {
            { 0, 3, (int)offsetof(draw_vertex_t, pos) },
            { 1, 3, (int)offsetof(draw_vertex_t, col) },
        };
        ivp_gpu_stream_desc_t desc = {0};
        desc.capacity     = cfg.max_draw_vertices;
        desc.stride       = (int)sizeof(draw_vertex_t);
        desc.attribs      = attrs;
        desc.attrib_count = 2;
        desc.depth_test   = true;
        desc.depth_write  = true;

        res = ivp_gpu_stream_create(&r->stream_3d, &r->gpu, &desc,
                                    r->shaders.vs_3d, r->shaders.vs_3d_len,
                                    r->shaders.fs, r->shaders.fs_len);
        if (res != IVP_OK) goto fail_shaders;
    }

    /* 2D HUD stream (no depth) */
    {
        ivp_gpu_attrib_desc_t attrs[2] = {
            { 0, 2, (int)offsetof(hud_vertex_t, pos) },
            { 1, 3, (int)offsetof(hud_vertex_t, col) },
        };
        ivp_gpu_stream_desc_t desc = {0};
        desc.capacity     = cfg.max_hud_vertices;
        desc.stride       = (int)sizeof(hud_vertex_t);
        desc.attribs      = attrs;
        desc.attrib_count = 2;
        desc.depth_test   = false;
        desc.depth_write  = false;

        res = ivp_gpu_stream_create(&r->stream_2d, &r->gpu, &desc,
                                    r->shaders.vs_2d, r->shaders.vs_2d_len,
                                    r->shaders.fs, r->shaders.fs_len);
        if (res != IVP_OK) goto fail_3d;
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

fail_3d:
    ivp_gpu_stream_destroy(&r->stream_3d, &r->gpu);
fail_shaders:
    ivp_shader_cache_free(&r->shaders);
fail_gpu:
    ivp_gpu_device_destroy(&r->gpu);
fail_win:
    SDL_DestroyWindow(win);
    ivp_free(alloc, r, sizeof(*r));
    return res;
}

void ivp_renderer_destroy(ivp_renderer_t *r)
{
    if (!r) return;
    const ivp_allocator_t *alloc = ivp_default_allocator();

    ivp_gpu_stream_destroy(&r->stream_2d, &r->gpu);
    ivp_gpu_stream_destroy(&r->stream_3d, &r->gpu);
    ivp_shader_cache_free(&r->shaders);
    ivp_gpu_device_destroy(&r->gpu);
    SDL_DestroyWindow(r->window);
    ivp_free(alloc, r, sizeof(*r));
    SDL_Quit();
}

/* ── Frame loop ────────────────────────────────────────────────────────── */

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
    ivp_camera_compute_matrices(cam, r->width, r->height,
                                r->view_mat, r->proj_mat, r->vp_mat);

    /* Depth resize */
    int pw, ph;
    SDL_GetWindowSizeInPixels(r->window, &pw, &ph);
    ivp_gpu_device_ensure_depth(&r->gpu, pw, ph);
    if (!r->gpu.depth_texture) return true;

    /* Acquire GPU frame */
    r->cmd = SDL_AcquireGPUCommandBuffer(r->gpu.device);
    if (!r->cmd) return false;

    r->swapchain_tex = NULL;
    Uint32 sw, sh;
    if (!SDL_WaitAndAcquireGPUSwapchainTexture(r->cmd, r->window,
                                                &r->swapchain_tex, &sw, &sh)) {
        SDL_CancelGPUCommandBuffer(r->cmd);
        r->cmd = NULL;
        return true;
    }
    (void)sw; (void)sh;
    if (!r->swapchain_tex) {
        SDL_SubmitGPUCommandBuffer(r->cmd);
        r->cmd = NULL;
        return true;
    }

    /* Reset streams */
    ivp_gpu_stream_reset(&r->stream_3d);
    ivp_gpu_stream_reset(&r->stream_2d);
    return true;
}

void ivp_renderer_end_frame(ivp_renderer_t *r)
{
    if (!r->cmd || !r->swapchain_tex) return;

    /* Upload */
    ivp_gpu_stream_upload(&r->stream_3d, &r->gpu, r->cmd);
    ivp_gpu_stream_upload(&r->stream_2d, &r->gpu, r->cmd);

    /* Render pass */
    SDL_GPUColorTargetInfo ct = {0};
    ct.texture     = r->swapchain_tex;
    ct.clear_color = (SDL_FColor){ r->clear_col[0], r->clear_col[1],
                                    r->clear_col[2], r->clear_col[3] };
    ct.load_op     = SDL_GPU_LOADOP_CLEAR;
    ct.store_op    = SDL_GPU_STOREOP_STORE;

    SDL_GPUDepthStencilTargetInfo dt = {0};
    dt.texture           = r->gpu.depth_texture;
    dt.clear_depth       = 1.0f;
    dt.load_op           = SDL_GPU_LOADOP_CLEAR;
    dt.store_op          = SDL_GPU_STOREOP_DONT_CARE;
    dt.stencil_load_op   = SDL_GPU_LOADOP_DONT_CARE;
    dt.stencil_store_op  = SDL_GPU_STOREOP_DONT_CARE;

    SDL_GPURenderPass *pass = SDL_BeginGPURenderPass(r->cmd, &ct, 1, &dt);
    if (!pass) {
        SDL_CancelGPUCommandBuffer(r->cmd);
        r->cmd = NULL;
        r->swapchain_tex = NULL;
        return;
    }

    SDL_GPUViewport vp = { 0, 0, (float)r->width, (float)r->height, 0.0f, 1.0f };
    SDL_SetGPUViewport(pass, &vp);

    /* 3D draw */
    ivp_gpu_stream_draw(&r->stream_3d, pass, r->cmd, r->vp_mat);

    /* 2D HUD overlay (ortho matrix) */
    {
        float ortho[16];
        ivp_render_mat4_ortho(ortho, 0.0f, (float)r->width,
                              (float)r->height, 0.0f, -1.0f, 1.0f);
        ivp_gpu_stream_draw(&r->stream_2d, pass, r->cmd, ortho);
    }

    SDL_EndGPURenderPass(pass);

    /* Pre-swap */
    if (r->callbacks.pre_swap_cb)
        r->callbacks.pre_swap_cb(r->callbacks.pre_swap_cb_ud);

    SDL_SubmitGPUCommandBuffer(r->cmd);
    r->cmd = NULL;
    r->swapchain_tex = NULL;
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

    stats->frame_dt = r->frame_dt;
    stats->fps = (stats->frame_dt > 1e-6f) ? 1.0f / stats->frame_dt : 0.0f;
}