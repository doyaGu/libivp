/* render.h -- Backward-compatible facade for IVP render module
 *
 * This header preserves the original API used by all 14 examples.
 * Internally it delegates to the ivp_render module.
 */
#ifndef IVP_EXAMPLE_RENDER_H
#define IVP_EXAMPLE_RENDER_H

#include <ivp/render/ivp_render.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Drawing primitives (legacy signatures, delegate to draw context) ─── */

void ivp_draw_wire_box(ivp_renderer_t *r,
                       const float pos[3],
                       const float half_size[3],
                       const float *rot,
                       const float color[3]);

void ivp_draw_wire_sphere(ivp_renderer_t *r,
                          const float pos[3],
                          float radius,
                          const float color[3]);

/* Higher-quality sphere with explicit segment count (use IVE_SPHERE_SEGS = 32) */
void ivp_draw_wire_sphere_ex(ivp_renderer_t *r,
                              const float pos[3],
                              float radius,
                              const float color[3],
                              int segments);

void ivp_draw_wire_cylinder(ivp_renderer_t *r,
                            const float base[3],
                            const float axis[3],
                            float height,
                            float radius,
                            int segments,
                            const float color[3]);

void ivp_draw_line(ivp_renderer_t *r,
                   const float a[3],
                   const float b[3],
                   const float color[3]);

void ivp_draw_thick_line(ivp_renderer_t *r,
                         const float a[3],
                         const float b[3],
                         const float color[3],
                         float thickness);

void ivp_draw_arrow(ivp_renderer_t *r,
                    const float from[3],
                    const float to[3],
                    const float color[3],
                    float head_size);

void ivp_draw_ground_grid(ivp_renderer_t *r,
                          float extent,
                          float spacing,
                          const float color[3]);

void ivp_draw_wire_quad(ivp_renderer_t *r,
                        const float corners[4][3],
                        const float color[3]);

void ivp_draw_axes(ivp_renderer_t *r,
                   const float pos[3],
                   const float *rot,
                   float scale);

/* ── 2D HUD text (legacy signature, delegates to hud context) ──────────── */

void ivp_draw_text_2d(ivp_renderer_t *r,
                      float x, float y,
                      const char *text,
                      const float color[3]);

#ifdef __cplusplus
}
#endif

#endif /* IVP_EXAMPLE_RENDER_H */
