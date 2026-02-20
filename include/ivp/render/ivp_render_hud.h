/* ivp_render_hud.h -- 2D text overlay API */
#ifndef IVP_RENDER_HUD_H
#define IVP_RENDER_HUD_H

#include <ivp/render/ivp_render_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define IVP_RENDER_HUD_MAX_VERTICES   (32 * 1024)
#define IVP_RENDER_HUD_FONT_SCALE     2
#define IVP_RENDER_HUD_FONT_W         5
#define IVP_RENDER_HUD_FONT_H         7

/* ── Config ────────────────────────────────────────────────────────────── */

typedef struct ivp_render_hud_config {
    int max_vertices;  /* default: IVP_RENDER_HUD_MAX_VERTICES (32K) */
    int font_scale;    /* default: IVP_RENDER_HUD_FONT_SCALE (2) */
} ivp_render_hud_config_t;

void ivp_render_hud_config_init(ivp_render_hud_config_t *cfg);

/* ── Text alignment ───────────────────────────────────────────────────── */

typedef enum ivp_text_align {
    IVP_TEXT_ALIGN_LEFT   = 0,
    IVP_TEXT_ALIGN_CENTER = 1,
    IVP_TEXT_ALIGN_RIGHT  = 2
} ivp_text_align_t;

/* ── Context lifecycle (managed by renderer; not called directly) ───────── */

/* Reset vertex count to zero (called at start of frame). */
void ivp_render_hud_ctx_reset(ivp_render_hud_ctx_t *ctx);

/* ── Text drawing ──────────────────────────────────────────────────────── */

void ivp_render_draw_text_2d(ivp_render_hud_ctx_t *ctx,
                             float x, float y,
                             const char *text,
                             const float color[3]);

/* Draw text with per-call scale. */
void ivp_render_draw_text_2d_scaled(ivp_render_hud_ctx_t *ctx,
                                    float x, float y,
                                    const char *text,
                                    const float color[3],
                                    int scale);

/* Draw text with alignment (uses context's default font scale). */
void ivp_render_draw_text_2d_aligned(ivp_render_hud_ctx_t *ctx,
                                     float x, float y,
                                     const char *text,
                                     const float color[3],
                                     ivp_text_align_t align);

/* Printf-style formatted text (convenience for HUD). */
void ivp_render_draw_text_2d_fmt(ivp_render_hud_ctx_t *ctx,
                                 float x, float y,
                                 const float color[3],
                                 const char *fmt, ...);

/* ── Utilities ─────────────────────────────────────────────────────────── */

/* Returns the line height in pixels for the built-in font. */
float ivp_render_hud_line_height(void);

/* Returns the pixel width of a text string (single line, no newlines). */
float ivp_render_hud_text_width(const char *text);

/* Scaled utility functions. */
float ivp_render_hud_line_height_scaled(int scale);
float ivp_render_hud_text_width_scaled(const char *text, int scale);

/* Current vertex count. */
int ivp_render_hud_ctx_count(const ivp_render_hud_ctx_t *ctx);

/* Maximum vertex capacity. */
int ivp_render_hud_ctx_capacity(const ivp_render_hud_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_HUD_H */
