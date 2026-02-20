/* ivp_render_color.h -- Predefined color palette constants for IVP examples */
#ifndef IVP_RENDER_COLOR_H
#define IVP_RENDER_COLOR_H

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

#ifdef __cplusplus
extern "C" {
#endif

/* ── Semantic colors ───────────────────────────────────────────────────── */

#define IVP_COLOR_VELOCITY    ((ivp_color3f_t){0.2f, 1.0f, 0.4f})
#define IVP_COLOR_DRAG        ((ivp_color3f_t){1.0f, 0.85f, 0.25f})
#define IVP_COLOR_FOCUS       ((ivp_color3f_t){1.0f, 0.65f, 0.25f})

/* ── Theme colors (light) ──────────────────────────────────────────────── */

#define IVP_COLOR_GRID_LIGHT  ((ivp_color3f_t){0.38f, 0.43f, 0.52f})
#define IVP_COLOR_HUD_LIGHT   ((ivp_color3f_t){0.95f, 0.97f, 1.0f})
#define IVP_COLOR_GROUND_LIGHT ((ivp_color3f_t){0.22f, 0.24f, 0.30f})

/* ── Theme colors (dark) ──────────────────────────────────────────────── */

#define IVP_COLOR_GRID_DARK   ((ivp_color3f_t){0.12f, 0.12f, 0.18f})
#define IVP_COLOR_HUD_DARK    ((ivp_color3f_t){0.80f, 0.80f, 0.80f})
#define IVP_COLOR_GROUND_DARK ((ivp_color3f_t){0.35f, 0.35f, 0.40f})

/* ── Axis colors ───────────────────────────────────────────────────────── */

#define IVP_COLOR_AXIS_X      ((ivp_color3f_t){1.0f, 0.0f, 0.0f})
#define IVP_COLOR_AXIS_Y      ((ivp_color3f_t){0.0f, 1.0f, 0.0f})
#define IVP_COLOR_AXIS_Z      ((ivp_color3f_t){0.0f, 0.0f, 1.0f})

/* ── Multi-object palette ──────────────────────────────────────────────── */

#define IVP_PALETTE_SIZE 8

static const ivp_color3f_t IVP_PALETTE[IVP_PALETTE_SIZE] = {
    {1.0f, 0.4f, 0.2f},   /* orange */
    {0.3f, 1.0f, 0.3f},   /* green */
    {0.3f, 0.5f, 1.0f},   /* blue */
    {1.0f, 1.0f, 0.2f},   /* yellow */
    {1.0f, 0.2f, 0.8f},   /* magenta */
    {0.2f, 1.0f, 0.9f},   /* cyan */
    {0.9f, 0.6f, 0.1f},   /* amber */
    {0.6f, 0.3f, 1.0f},   /* violet */
};

static inline ivp_color3f_t ivp_palette_color(int index)
{
    return IVP_PALETTE[index % IVP_PALETTE_SIZE];
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_COLOR_H */
