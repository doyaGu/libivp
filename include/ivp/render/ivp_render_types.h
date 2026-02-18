/* ivp_render_types.h -- Forward declarations, color type, render error codes */
#ifndef IVP_RENDER_TYPES_H
#define IVP_RENDER_TYPES_H

#include <ivp/base/ivp_result.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Color type ────────────────────────────────────────────────────────── */

typedef struct ivp_color3f {
    float r, g, b;
} ivp_color3f_t;

static inline ivp_color3f_t ivp_color3f(float r, float g, float b)
{
    ivp_color3f_t c = {r, g, b};
    return c;
}

static inline const float *ivp_color3f_ptr(const ivp_color3f_t *c)
{
    return &c->r;
}

/* ── Forward declarations ──────────────────────────────────────────────── */

typedef struct ivp_renderer         ivp_renderer_t;
typedef struct ivp_camera           ivp_camera_t;
typedef struct ivp_render_draw_ctx  ivp_render_draw_ctx_t;
typedef struct ivp_render_hud_ctx   ivp_render_hud_ctx_t;

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_TYPES_H */
