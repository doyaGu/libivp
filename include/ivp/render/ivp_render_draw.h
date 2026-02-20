/* ivp_render_draw.h -- 3D wireframe drawing primitives API */
#ifndef IVP_RENDER_DRAW_H
#define IVP_RENDER_DRAW_H

#include <ivp/render/ivp_render_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define IVP_RENDER_MAX_VERTICES       (128 * 1024)
#define IVP_RENDER_SPHERE_SEGMENTS    24

/* ── Config ────────────────────────────────────────────────────────────── */

typedef struct ivp_render_draw_config {
    int max_vertices;     /* default: IVP_RENDER_MAX_VERTICES (128K) */
    int sphere_segments;  /* default: IVP_RENDER_SPHERE_SEGMENTS (24) */
} ivp_render_draw_config_t;

void ivp_render_draw_config_init(ivp_render_draw_config_t *cfg);

/* ── Context lifecycle (managed by renderer; not called directly) ───────── */

/* Reset vertex count to zero (called at start of frame). */
void ivp_render_draw_ctx_reset(ivp_render_draw_ctx_t *ctx);

/* Current vertex count. */
int  ivp_render_draw_ctx_count(const ivp_render_draw_ctx_t *ctx);

/* Maximum vertex capacity. */
int  ivp_render_draw_ctx_capacity(const ivp_render_draw_ctx_t *ctx);

/* Check if there is room for n more vertices. */
int  ivp_render_draw_ctx_has_room(const ivp_render_draw_ctx_t *ctx, int n);

/* ── 3D drawing primitives ─────────────────────────────────────────────── */

void ivp_render_draw_line(ivp_render_draw_ctx_t *ctx,
                          const float a[3], const float b[3],
                          const float color[3]);

void ivp_render_draw_thick_line(ivp_render_draw_ctx_t *ctx,
                                const float a[3], const float b[3],
                                const float color[3], float thickness);

void ivp_render_draw_wire_box(ivp_render_draw_ctx_t *ctx,
                              const float pos[3],
                              const float half_size[3],
                              const float *rot,     /* 9 floats col-major, or NULL */
                              const float color[3]);

void ivp_render_draw_wire_sphere(ivp_render_draw_ctx_t *ctx,
                                 const float pos[3],
                                 float radius,
                                 const float color[3]);

void ivp_render_draw_wire_sphere_ex(ivp_render_draw_ctx_t *ctx,
                                    const float pos[3],
                                    float radius,
                                    const float color[3],
                                    int segments);

void ivp_render_draw_wire_cylinder(ivp_render_draw_ctx_t *ctx,
                                   const float base[3],
                                   const float axis[3],
                                   float height,
                                   float radius,
                                   int segments,
                                   const float color[3]);

void ivp_render_draw_wire_quad(ivp_render_draw_ctx_t *ctx,
                               const float corners[4][3],
                               const float color[3]);

void ivp_render_draw_arrow(ivp_render_draw_ctx_t *ctx,
                           const float from[3],
                           const float to[3],
                           const float color[3],
                           float head_size);

void ivp_render_draw_ground_grid(ivp_render_draw_ctx_t *ctx,
                                 float extent,
                                 float spacing,
                                 const float color[3]);

void ivp_render_draw_axes(ivp_render_draw_ctx_t *ctx,
                          const float pos[3],
                          const float *rot,   /* 9 floats col-major, or NULL */
                          float scale);

/* ── Additional drawing primitives ────────────────────────────────────── */

/* Circle -- ring in a plane defined by center, normal, radius. */
void ivp_render_draw_circle(ivp_render_draw_ctx_t *ctx,
                            const float center[3],
                            const float normal[3],
                            float radius,
                            int segments,
                            const float color[3]);

/* Cone wireframe -- apex + base center + base radius. */
void ivp_render_draw_wire_cone(ivp_render_draw_ctx_t *ctx,
                               const float apex[3],
                               const float base_center[3],
                               float base_radius,
                               int segments,
                               const float color[3]);

/* Capsule wireframe -- two spherical caps + connecting cylinder. */
void ivp_render_draw_wire_capsule(ivp_render_draw_ctx_t *ctx,
                                  const float a[3],
                                  const float b[3],
                                  float radius,
                                  int segments,
                                  const float color[3]);

/* Plane -- finite rectangular region with normal indicator. */
void ivp_render_draw_plane(ivp_render_draw_ctx_t *ctx,
                           const float center[3],
                           const float normal[3],
                           float extent,
                           const float color[3]);

/* AABB from min/max corners. */
void ivp_render_draw_aabb(ivp_render_draw_ctx_t *ctx,
                          const float min[3],
                          const float max[3],
                          const float color[3]);

/* Cross marker (3 axis-aligned lines through a point). */
void ivp_render_draw_cross(ivp_render_draw_ctx_t *ctx,
                           const float pos[3],
                           float size,
                           const float color[3]);

/* Point marker (small diamond/octahedron at a point). */
void ivp_render_draw_point(ivp_render_draw_ctx_t *ctx,
                           const float pos[3],
                           float size,
                           const float color[3]);

/* Contact point visualization -- point + normal arrow. */
void ivp_render_draw_contact(ivp_render_draw_ctx_t *ctx,
                             const float pos[3],
                             const float normal[3],
                             float normal_length,
                             const float color[3]);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_DRAW_H */
