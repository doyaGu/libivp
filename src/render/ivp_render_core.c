/* ivp_render_core.c -- Renderer lifecycle, frame loop, event dispatch (SDL3 GPU) */

#include <ivp/render/ivp_render_core.h>
#include <ivp/render/ivp_render_math.h>
#include <ivp/base/ivp_allocator.h>
#include "ivp_render_internal.h"

#include <SDL3/SDL.h>

#include <string.h>
#include <stdio.h>

/* ── Renderer struct ───────────────────────────────────────────────────── */

struct ivp_renderer {
    SDL_Window              *window;
    ivp_gpu_device_t         gpu;
    ivp_render_draw_ctx_t   *draw_ctx;
    ivp_render_hud_ctx_t    *hud_ctx;
    ivp_camera_input_t       cam_input;
    ivp_render_mat4_t        view_mat;
    ivp_render_mat4_t        proj_mat;
    ivp_render_mat4_t        vp_mat;
    int                      width, height;
    Uint64                   start_ticks;
    Uint64                   prev_ticks;
    float                    frame_dt;
    float                    max_frame_dt;
    float                    clear_col[4];
    ivp_render_callbacks_t   callbacks;
    /* Per-frame GPU command state (valid between begin_frame / end_frame) */
    SDL_GPUCommandBuffer    *cmd;
    SDL_GPUTexture          *swapchain_tex;
};

/* ── Config ────────────────────────────────────────────────────────────── */

void ivp_render_config_init(ivp_render_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->width        = 1280;
    cfg->height       = 720;
    cfg->title        = "IVP Example";
    cfg->vsync        = true;
    cfg->msaa_samples = 4;
    cfg->clear_color[0] = 0.20f;
    cfg->clear_color[1] = 0.24f;
    cfg->clear_color[2] = 0.30f;
    cfg->clear_color[3] = 1.0f;
    cfg->max_frame_dt = 0.1f;
    ivp_render_draw_config_init(&cfg->draw_config);
    ivp_render_hud_config_init(&cfg->hud_config);
}

/* Resolve zero-means-default fields for backward compatibility. */
static ivp_render_config_t resolve_config(const ivp_render_config_t *config)
{
    ivp_render_config_t resolved;
    ivp_render_config_init(&resolved);

    if (config->width > 0)        resolved.width        = config->width;
    if (config->height > 0)       resolved.height       = config->height;
    if (config->title)            resolved.title         = config->title;
    resolved.vsync = config->vsync;

    if (config->msaa_samples < 0) {
        resolved.msaa_samples = 0; /* disabled */
    } else if (config->msaa_samples > 0) {
        resolved.msaa_samples = config->msaa_samples;
    }

    if (config->max_frame_dt > 0.0f) resolved.max_frame_dt = config->max_frame_dt;

    /* clear_color: use if any component is non-zero */
    if (config->clear_color[0] != 0.0f || config->clear_color[1] != 0.0f ||
        config->clear_color[2] != 0.0f || config->clear_color[3] != 0.0f) {
        for (int i = 0; i < 4; i++)
            resolved.clear_color[i] = config->clear_color[i];
    }

    /* Sub-configs: use non-zero fields from caller */
    if (config->draw_config.max_vertices > 0)
        resolved.draw_config.max_vertices = config->draw_config.max_vertices;
    if (config->draw_config.sphere_segments > 0)
        resolved.draw_config.sphere_segments = config->draw_config.sphere_segments;
    if (config->hud_config.max_vertices > 0)
        resolved.hud_config.max_vertices = config->hud_config.max_vertices;
    if (config->hud_config.font_scale > 0)
        resolved.hud_config.font_scale = config->hud_config.font_scale;

    return resolved;
}

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

ivp_result_t ivp_renderer_create_ex(ivp_renderer_t **out,
                                    const ivp_render_config_t *config)
{
    if (!out || !config) return IVP_ERR_INVALID_ARG;

    ivp_render_config_t cfg = resolve_config(config);

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return IVP_ERR_RENDER_SDL_INIT;
    }

    SDL_Window *win = SDL_CreateWindow(
        cfg.title,
        cfg.width, cfg.height,
        SDL_WINDOW_RESIZABLE);
    if (!win) {
        fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return IVP_ERR_RENDER_WINDOW;
    }

    const ivp_allocator_t *alloc = ivp_default_allocator();

    /* Allocate renderer early so we can use its gpu field */
    ivp_renderer_t *r = (ivp_renderer_t *)ivp_alloc(alloc, sizeof(*r));
    if (!r) {
        SDL_DestroyWindow(win);
        return IVP_ERR_OUT_OF_MEMORY;
    }
    memset(r, 0, sizeof(*r));
    r->window = win;

    /* Initialize GPU device */
    ivp_result_t res = ivp_gpu_device_init(&r->gpu, win, cfg.msaa_samples);
    if (res != IVP_OK) {
        SDL_DestroyWindow(win);
        ivp_free(alloc, r, sizeof(*r));
        return res;
    }

    /* Set vsync present mode */
    SDL_SetGPUSwapchainParameters(r->gpu.device, win,
        SDL_GPU_SWAPCHAINCOMPOSITION_SDR,
        cfg.vsync ? SDL_GPU_PRESENTMODE_VSYNC
                  : SDL_GPU_PRESENTMODE_IMMEDIATE);

    /* Create sub-contexts with config */
    res = ivp_render_draw_ctx_create_ex(&r->draw_ctx, &cfg.draw_config, &r->gpu);
    if (res != IVP_OK) {
        ivp_gpu_device_destroy(&r->gpu);
        SDL_DestroyWindow(win);
        ivp_free(alloc, r, sizeof(*r));
        return res;
    }

    res = ivp_render_hud_ctx_create_ex(&r->hud_ctx, &cfg.hud_config, &r->gpu);
    if (res != IVP_OK) {
        ivp_render_draw_ctx_destroy(r->draw_ctx, &r->gpu);
        ivp_gpu_device_destroy(&r->gpu);
        SDL_DestroyWindow(win);
        ivp_free(alloc, r, sizeof(*r));
        return res;
    }

    r->width        = cfg.width;
    r->height       = cfg.height;
    r->start_ticks  = SDL_GetPerformanceCounter();
    r->prev_ticks   = 0;
    r->max_frame_dt = cfg.max_frame_dt;
    r->clear_col[0] = cfg.clear_color[0];
    r->clear_col[1] = cfg.clear_color[1];
    r->clear_col[2] = cfg.clear_color[2];
    r->clear_col[3] = cfg.clear_color[3];

    ivp_camera_input_init(&r->cam_input);

    *out = r;
    return IVP_OK;
}

ivp_renderer_t *ivp_renderer_create(const ivp_render_config_t *config)
{
    ivp_renderer_t *r = NULL;
    ivp_result_t res = ivp_renderer_create_ex(&r, config);
    if (res != IVP_OK) return NULL;
    return r;
}

void ivp_renderer_destroy(ivp_renderer_t *r)
{
    if (!r) return;
    const ivp_allocator_t *alloc = ivp_default_allocator();
    ivp_render_draw_ctx_destroy(r->draw_ctx, &r->gpu);
    ivp_render_hud_ctx_destroy(r->hud_ctx, &r->gpu);
    ivp_gpu_device_destroy(&r->gpu);
    SDL_DestroyWindow(r->window);
    ivp_free(alloc, r, sizeof(*r));
    SDL_Quit();
}

/* ── Frame loop ────────────────────────────────────────────────────────── */

bool ivp_renderer_begin_frame(ivp_renderer_t *r, ivp_camera_t *cam)
{
    Uint64 now_ticks = SDL_GetPerformanceCounter();
    if (r->prev_ticks == 0) r->prev_ticks = now_ticks;
    r->frame_dt = (float)((double)(now_ticks - r->prev_ticks) /
                          (double)SDL_GetPerformanceFrequency());
    r->prev_ticks = now_ticks;
    if (r->frame_dt > r->max_frame_dt) r->frame_dt = r->max_frame_dt;

    /* Pre-events callback */
    if (r->callbacks.pre_events_cb)
        r->callbacks.pre_events_cb(r->callbacks.pre_events_cb_ud);

    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_EVENT_QUIT) return false;

        /* Per-event callback; if it returns true, skip camera handling */
        if (r->callbacks.event_cb &&
            r->callbacks.event_cb(&ev, r->callbacks.event_cb_ud))
            continue;

        ivp_camera_process_event(cam, &r->cam_input, &ev,
                                 &r->width, &r->height);
    }

    /* Post-events callback */
    if (r->callbacks.post_events_cb)
        r->callbacks.post_events_cb(r->callbacks.post_events_cb_ud);

    /* WASD camera pan */
    ivp_camera_apply_key_pan(cam, &r->cam_input);

    /* Update camera matrices */
    if (cam) {
        ivp_camera_update_orbit(cam);
        if (cam->mode == IVP_CAMERA_MODE_FLY)
            ivp_camera_update_fly(cam);
    }
    ivp_camera_compute_matrices(cam, r->width, r->height,
                                r->view_mat, r->proj_mat, r->vp_mat);

    /* Ensure depth texture matches window size */
    int pw, ph;
    SDL_GetWindowSizeInPixels(r->window, &pw, &ph);
    ivp_gpu_device_ensure_depth(&r->gpu, pw, ph);
    if (!r->gpu.depth_texture) {
        return true; /* skip frame on transient allocation failure */
    }

    /* Acquire swapchain texture */
    r->cmd = SDL_AcquireGPUCommandBuffer(r->gpu.device);
    if (!r->cmd) return false;

    r->swapchain_tex = NULL;
    Uint32 sw, sh;
    if (!SDL_WaitAndAcquireGPUSwapchainTexture(r->cmd, r->window,
                                                &r->swapchain_tex, &sw, &sh)) {
        SDL_CancelGPUCommandBuffer(r->cmd);
        r->cmd = NULL;
        return true; /* minimised — skip frame but don't quit */
    }
    (void)sw;
    (void)sh;
    if (!r->swapchain_tex) {
        /* Window minimised or no texture available */
        SDL_SubmitGPUCommandBuffer(r->cmd);
        r->cmd = NULL;
        return true;
    }

    ivp_render_draw_ctx_reset(r->draw_ctx);
    ivp_render_hud_ctx_reset(r->hud_ctx);
    return true;
}

void ivp_renderer_end_frame(ivp_renderer_t *r)
{
    if (!r->cmd || !r->swapchain_tex) return;

    /* Upload draw + HUD vertex data via copy passes (before the render pass) */
    ivp_render_draw_ctx_upload(r->draw_ctx, &r->gpu, r->cmd);
    ivp_render_hud_ctx_upload(r->hud_ctx, &r->gpu, r->cmd);

    /* Begin render pass with clear */
    SDL_GPUColorTargetInfo color_target = {0};
    color_target.texture     = r->swapchain_tex;
    color_target.clear_color.r = r->clear_col[0];
    color_target.clear_color.g = r->clear_col[1];
    color_target.clear_color.b = r->clear_col[2];
    color_target.clear_color.a = r->clear_col[3];
    color_target.load_op     = SDL_GPU_LOADOP_CLEAR;
    color_target.store_op    = SDL_GPU_STOREOP_STORE;

    SDL_GPUDepthStencilTargetInfo depth_target = {0};
    depth_target.texture        = r->gpu.depth_texture;
    depth_target.clear_depth    = 1.0f;
    depth_target.load_op        = SDL_GPU_LOADOP_CLEAR;
    depth_target.store_op       = SDL_GPU_STOREOP_DONT_CARE;
    depth_target.stencil_load_op  = SDL_GPU_LOADOP_DONT_CARE;
    depth_target.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;

    SDL_GPURenderPass *pass = SDL_BeginGPURenderPass(
        r->cmd, &color_target, 1, &depth_target);
    if (!pass) {
        SDL_CancelGPUCommandBuffer(r->cmd);
        r->cmd = NULL;
        r->swapchain_tex = NULL;
        return;
    }

    /* Set viewport */
    SDL_GPUViewport vp = {0};
    vp.x = 0; vp.y = 0;
    vp.w = (float)r->width;
    vp.h = (float)r->height;
    vp.min_depth = 0.0f;
    vp.max_depth = 1.0f;
    SDL_SetGPUViewport(pass, &vp);

    /* Draw 3D lines */
    ivp_render_draw_ctx_draw(r->draw_ctx, &r->gpu, r->cmd, pass, r->vp_mat);

    /* Draw 2D HUD overlay (no depth) */
    ivp_render_hud_ctx_draw(r->hud_ctx, &r->gpu, r->cmd, pass,
                            r->width, r->height);

    SDL_EndGPURenderPass(pass);

    /* Pre-swap callback */
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

ivp_render_draw_ctx_t *ivp_renderer_get_draw_ctx(ivp_renderer_t *r)
{
    return r ? r->draw_ctx : NULL;
}

ivp_render_hud_ctx_t *ivp_renderer_get_hud_ctx(ivp_renderer_t *r)
{
    return r ? r->hud_ctx : NULL;
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
                                          ivp_frame_callback_t cb, void *userdata)
{
    if (!r) return;
    r->callbacks.pre_events_cb = cb;
    r->callbacks.pre_events_cb_ud = userdata;
}

void ivp_renderer_set_post_events_callback(ivp_renderer_t *r,
                                           ivp_frame_callback_t cb, void *userdata)
{
    if (!r) return;
    r->callbacks.post_events_cb = cb;
    r->callbacks.post_events_cb_ud = userdata;
}

void ivp_renderer_set_pre_swap_callback(ivp_renderer_t *r,
                                        ivp_frame_callback_t cb, void *userdata)
{
    if (!r) return;
    r->callbacks.pre_swap_cb = cb;
    r->callbacks.pre_swap_cb_ud = userdata;
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
    float ndc_x = x * inv_w;
    float ndc_y = y * inv_w;
    screen[0] = (ndc_x * 0.5f + 0.5f) * (float)r->width;
    screen[1] = (1.0f - (ndc_y * 0.5f + 0.5f)) * (float)r->height;
    return true;
}
