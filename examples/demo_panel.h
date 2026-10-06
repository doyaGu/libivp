/* demo_panel.h -- A small parameter panel (sliders and checkboxes)
 *
 * Included by demo_app.h before the viewer.  The panel owns no window or
 * input code: the caller passes the mouse state and gets back whether the
 * panel wants the mouse, so that clicks on it do not pick or drag bodies.
 * It draws with ivp_render_draw_rect / ivp_render_draw_text.
 */
#ifndef IVP_EXAMPLE_DEMO_PANEL_H
#define IVP_EXAMPLE_DEMO_PANEL_H

#include <ivp/render/ivp_render.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define DEMO_PANEL_MAX_PARAMS 16
#define DEMO_PANEL_WIDTH      270.0f
#define DEMO_PANEL_PAD        10.0f
#define DEMO_PANEL_HEAD_H     26.0f
#define DEMO_PANEL_FLOAT_H    36.0f
#define DEMO_PANEL_BOOL_H     26.0f

typedef void (*demo_param_cb_t)(void *ctx);

typedef struct demo_param {
    const char     *label;
    float          *fval;        /* slider when set */
    bool           *bval;        /* checkbox when set */
    float           lo, hi;
    demo_param_cb_t on_change;   /* called after the value changed; may be NULL */
    void           *ctx;
    int             col;         /* column and offset below the panel top, set by demo_panel_layout */
    float           ry;
} demo_param_t;

typedef struct demo_panel {
    demo_param_t params[DEMO_PANEL_MAX_PARAMS];
    int          n;
    bool         open;
    int          active;         /* slider being dragged, -1 = none */
    float        x, y;           /* top left, set by demo_panel_update */
    float        w, h;
    float        max_h;          /* column height limit; more rows start a new column (0 = none) */
    int          n_cols;
    bool         prev_lmb;
} demo_panel_t;

static inline void demo_panel_init(demo_panel_t *p)
{
    memset(p, 0, sizeof(*p));
    p->active = -1;
}

static inline bool demo_panel_add_float(demo_panel_t *p, const char *label, float *v,
                                        float lo, float hi, demo_param_cb_t cb, void *ctx)
{
    if (p->n >= DEMO_PANEL_MAX_PARAMS) return false;
    demo_param_t *q = &p->params[p->n++];
    q->label = label;  q->fval = v;  q->bval = NULL;
    q->lo = lo;  q->hi = hi;  q->on_change = cb;  q->ctx = ctx;
    return true;
}

static inline bool demo_panel_add_bool(demo_panel_t *p, const char *label, bool *v,
                                       demo_param_cb_t cb, void *ctx)
{
    if (p->n >= DEMO_PANEL_MAX_PARAMS) return false;
    demo_param_t *q = &p->params[p->n++];
    q->label = label;  q->fval = NULL;  q->bval = v;
    q->lo = 0.0f;  q->hi = 1.0f;  q->on_change = cb;  q->ctx = ctx;
    return true;
}

static inline float demo_panel_row_h(const demo_param_t *q)
{
    return q->fval ? DEMO_PANEL_FLOAT_H : DEMO_PANEL_BOOL_H;
}

/* Assign every row to a column (rows flow into a new column when max_h is
 * reached) and set the panel size. */
static inline void demo_panel_layout(demo_panel_t *p)
{
    float y = DEMO_PANEL_HEAD_H, tallest = 0.0f;
    int col = 0;
    for (int i = 0; i < p->n; i++) {
        demo_param_t *q = &p->params[i];
        float rh = demo_panel_row_h(q);
        if (p->max_h > 0.0f && y > DEMO_PANEL_HEAD_H && y + rh + DEMO_PANEL_PAD > p->max_h) {
            col++;
            y = DEMO_PANEL_HEAD_H;
        }
        q->col = col;
        q->ry = y;
        y += rh;
        if (y + DEMO_PANEL_PAD > tallest) tallest = y + DEMO_PANEL_PAD;
    }
    p->n_cols = col + 1;
    p->w = DEMO_PANEL_WIDTH * (float)p->n_cols;
    p->h = p->n ? tallest : DEMO_PANEL_HEAD_H + DEMO_PANEL_PAD;
}

/* Slider track of a float row, in window pixels. */
static inline void demo_panel_track(const demo_panel_t *p, const demo_param_t *q,
                                    float *x0, float *x1, float *y)
{
    const float cx = p->x + DEMO_PANEL_WIDTH * (float)q->col;
    *x0 = cx + DEMO_PANEL_PAD;
    *x1 = cx + DEMO_PANEL_WIDTH - DEMO_PANEL_PAD;
    *y  = p->y + q->ry + 26.0f;
}

/* Place the panel at the top right (below `top`), handle the mouse.
 * Returns true while the panel owns the mouse (hovered or dragging). */
static inline bool demo_panel_update(demo_panel_t *p, int win_w, float top,
                                     float mx, float my, bool lmb)
{
    bool pressed = lmb && !p->prev_lmb;
    p->prev_lmb = lmb;
    if (!p->open) { p->active = -1; return false; }

    demo_panel_layout(p);
    p->x = (float)win_w - p->w - 10.0f;
    p->y = top;

    bool inside = mx >= p->x && mx <= p->x + p->w && my >= p->y && my <= p->y + p->h;
    if (!lmb) p->active = -1;

    for (int i = 0; i < p->n; i++) {
        demo_param_t *q = &p->params[i];
        const float rh = demo_panel_row_h(q), row_y = p->y + q->ry;
        const float cx = p->x + DEMO_PANEL_WIDTH * (float)q->col;
        if (pressed && inside && mx >= cx && mx < cx + DEMO_PANEL_WIDTH && my >= row_y && my < row_y + rh) {
            if (q->bval) {
                *q->bval = !*q->bval;
                if (q->on_change) q->on_change(q->ctx);
            } else {
                p->active = i;
            }
        }
        if (p->active == i && q->fval) {
            float x0, x1, ty;
            demo_panel_track(p, q, &x0, &x1, &ty);
            float t = (mx - x0) / (x1 - x0);
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            float nv = q->lo + (q->hi - q->lo) * t;
            if (nv != *q->fval) {
                *q->fval = nv;
                if (q->on_change) q->on_change(q->ctx);
            }
        }
    }
    return inside || p->active >= 0;
}

static inline void demo_panel_draw(const demo_panel_t *p, ivp_renderer_t *r)
{
    if (!p->open || !r) return;
    const float bg[3]    = { 0.04f, 0.06f, 0.09f };
    const float edge[3]  = { 0.35f, 0.45f, 0.60f };
    const float text[3]  = { 0.85f, 0.90f, 0.95f };
    const float dim[3]   = { 0.60f, 0.68f, 0.78f };
    const float track[3] = { 0.25f, 0.30f, 0.38f };
    const float fill[3]  = { 0.40f, 0.70f, 1.00f };
    const float knob[3]  = { 0.95f, 0.97f, 1.00f };
    const float on[3]    = { 0.45f, 0.85f, 0.55f };

    ivp_render_draw_rect(r, p->x, p->y, p->w, p->h, bg, 0.72f);
    ivp_render_draw_rect(r, p->x, p->y, p->w, 2.0f, edge, 0.9f);
    ivp_render_draw_text(r, p->x + DEMO_PANEL_PAD, p->y + 7.0f, "Parameters", text);
    ivp_render_draw_text_aligned(r, p->x + p->w - DEMO_PANEL_PAD, p->y + 7.0f, "F5",
                                 dim, IVP_TEXT_ALIGN_RIGHT);

    for (int i = 0; i < p->n; i++) {
        const demo_param_t *q = &p->params[i];
        const float cx = p->x + DEMO_PANEL_WIDTH * (float)q->col, row_y = p->y + q->ry;
        char val[32];
        if (q->fval) {
            snprintf(val, sizeof(val), "%.2f", (double)*q->fval);
            ivp_render_draw_text(r, cx + DEMO_PANEL_PAD, row_y + 3.0f, q->label, text);
            ivp_render_draw_text_aligned(r, cx + DEMO_PANEL_WIDTH - DEMO_PANEL_PAD, row_y + 3.0f, val,
                                         dim, IVP_TEXT_ALIGN_RIGHT);
            float x0, x1, ty;
            demo_panel_track(p, q, &x0, &x1, &ty);
            float t = (*q->fval - q->lo) / (q->hi - q->lo);
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            ivp_render_draw_rect(r, x0, ty - 2.0f, x1 - x0, 4.0f, track, 1.0f);
            ivp_render_draw_rect(r, x0, ty - 2.0f, (x1 - x0) * t, 4.0f, fill, 1.0f);
            ivp_render_draw_rect(r, x0 + (x1 - x0) * t - 4.0f, ty - 8.0f, 8.0f, 16.0f,
                                 p->active == i ? fill : knob, 1.0f);
        } else {
            float bx = cx + DEMO_PANEL_PAD, by = row_y + 4.0f;
            ivp_render_draw_rect(r, bx, by, 16.0f, 16.0f, track, 1.0f);
            if (*q->bval) ivp_render_draw_rect(r, bx + 3.0f, by + 3.0f, 10.0f, 10.0f, on, 1.0f);
            ivp_render_draw_text(r, bx + 26.0f, row_y + 5.0f, q->label, text);
        }
    }
}

#endif /* IVP_EXAMPLE_DEMO_PANEL_H */
