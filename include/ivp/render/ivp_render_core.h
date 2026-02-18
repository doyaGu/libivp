/* ivp_render_core.h -- Renderer lifecycle, frame loop, event dispatch */
#ifndef IVP_RENDER_CORE_H
#define IVP_RENDER_CORE_H

#include <ivp/render/ivp_render_types.h>
#include <ivp/render/ivp_render_camera.h>
#include <ivp/render/ivp_render_draw.h>
#include <ivp/render/ivp_render_hud.h>
#include <stdbool.h>
#include <SDL3/SDL.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Config ────────────────────────────────────────────────────────────── */

typedef struct ivp_render_config {
    int         width;           /* default: 1280 */
    int         height;          /* default: 720  */
    const char *title;           /* default: "IVP Example" */
    bool        vsync;           /* default: true */
    int         msaa_samples;    /* default: 4 (0=use default, <0=disabled) */
    int         depth_bits;      /* default: 24 */
    float       line_width;      /* default: 1.5 */
    float       clear_color[4];  /* default: {0.20, 0.24, 0.30, 1.0} */
    float       max_frame_dt;    /* default: 0.1 */
    ivp_render_draw_config_t draw_config;
    ivp_render_hud_config_t  hud_config;
} ivp_render_config_t;

/* Initialize a render config with all defaults. */
void ivp_render_config_init(ivp_render_config_t *cfg);

/* ── Callback types ────────────────────────────────────────────────────── */

/* Per-event callback: return true to consume the event (skip camera). */
typedef bool (*ivp_event_callback_t)(const SDL_Event *ev, void *userdata);
/* Frame-phase callback: called at specific points in the frame. */
typedef void (*ivp_frame_callback_t)(void *userdata);

/* Aggregate callback struct (internal) */
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

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

/* Create renderer with proper error propagation. */
ivp_result_t ivp_renderer_create_ex(ivp_renderer_t **out,
                                    const ivp_render_config_t *config);

/* Legacy convenience wrapper: returns NULL on failure. */
ivp_renderer_t *ivp_renderer_create(const ivp_render_config_t *config);

void ivp_renderer_destroy(ivp_renderer_t *r);

/* ── Frame loop ────────────────────────────────────────────────────────── */

/* Returns false when user closes window. */
bool ivp_renderer_begin_frame(ivp_renderer_t *r, ivp_camera_t *cam);
void ivp_renderer_end_frame(ivp_renderer_t *r);

/* ── Accessors ─────────────────────────────────────────────────────────── */

double       ivp_renderer_get_time(const ivp_renderer_t *r);
float        ivp_renderer_get_dt(const ivp_renderer_t *r);
void         ivp_renderer_get_size(const ivp_renderer_t *r, int *w, int *h);
const float *ivp_renderer_get_vp_matrix(const ivp_renderer_t *r);
const float *ivp_renderer_get_view_matrix(const ivp_renderer_t *r);
SDL_Window  *ivp_renderer_get_window(const ivp_renderer_t *r);

void ivp_renderer_set_clear_color(ivp_renderer_t *r,
                                  float red, float green,
                                  float blue, float alpha);

/* Sub-context accessors (for direct use of draw/hud APIs). */
ivp_render_draw_ctx_t *ivp_renderer_get_draw_ctx(ivp_renderer_t *r);
ivp_render_hud_ctx_t  *ivp_renderer_get_hud_ctx(ivp_renderer_t *r);

/* ── Callback setters ──────────────────────────────────────────────────── */

void ivp_renderer_set_event_callback(ivp_renderer_t *r,
                                     ivp_event_callback_t cb, void *userdata);
void ivp_renderer_set_pre_events_callback(ivp_renderer_t *r,
                                          ivp_frame_callback_t cb, void *userdata);
void ivp_renderer_set_post_events_callback(ivp_renderer_t *r,
                                           ivp_frame_callback_t cb, void *userdata);
void ivp_renderer_set_pre_swap_callback(ivp_renderer_t *r,
                                        ivp_frame_callback_t cb, void *userdata);

/* ── Projection ────────────────────────────────────────────────────────── */

/* Project a 3D world point to 2D screen coordinates (pixel space, top-left origin).
 * Returns false if the point is behind the camera. */
bool ivp_renderer_project(const ivp_renderer_t *r,
                          const float world[3], float screen[2]);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_CORE_H */
