/* ivp_render_internal.h -- Renderer struct definition (internal)
 *
 * Defines the full layout of ivp_renderer_t, which is opaque outside
 * src/render/.  All render .c files include this to access struct members.
 */
#ifndef IVP_RENDER_INTERNAL_H
#define IVP_RENDER_INTERNAL_H

#include <ivp/render/ivp_render.h>
#include "ivp_render_gpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Callback aggregate ────────────────────────────────────────────────── */

typedef struct ivp_render_callbacks {
    ivp_event_callback_t event_cb;
    void                *event_cb_ud;
    ivp_frame_callback_t pre_events_cb;
    void                *pre_events_cb_ud;
    ivp_frame_callback_t post_events_cb;
    void                *post_events_cb_ud;
    ivp_frame_callback_t pre_swap_cb;
    void                *pre_swap_cb_ud;
} ivp_render_callbacks_t;

/* ── Renderer struct ───────────────────────────────────────────────────── */

struct ivp_renderer {
    /* Platform */
    SDL_Window              *window;
    int                      width, height;

    /* GPU backend */
    ivp_gpu_device_t         gpu;
    ivp_shader_cache_t       shaders;
    ivp_gpu_stream_t         stream_3d;   /* 3D wireframe lines */
    ivp_gpu_stream_t         stream_2d;   /* 2D HUD overlay     */

    /* Camera input state */
    ivp_camera_input_t       cam_input;

    /* Matrices (computed each frame) */
    ivp_render_mat4_t        view_mat;
    ivp_render_mat4_t        proj_mat;
    ivp_render_mat4_t        vp_mat;

    /* Timing */
    Uint64                   start_ticks;
    Uint64                   prev_ticks;
    float                    frame_dt;
    float                    max_frame_dt;

    /* Config */
    float                    clear_col[4];
    int                      sphere_segments;
    int                      hud_font_scale;

    /* Callbacks */
    ivp_render_callbacks_t   callbacks;

    /* Per-frame GPU command state (valid between begin_frame / end_frame) */
    SDL_GPUCommandBuffer    *cmd;
    SDL_GPUTexture          *swapchain_tex;
};

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_INTERNAL_H */
