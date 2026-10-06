/* ivp_render.h -- IVP Render Module Public API
 *
 * Single public header for the IVP debug-visualization renderer.
 * Provides lifecycle management, 3D wireframe drawing, 2D HUD text,
 * screen-to-world picking utilities, and frame statistics.
 *
 * All drawing functions take an ivp_renderer_t* directly -- there is no
 * separate "draw context" or "hud context" to extract.
 *
 * Architecture layers (internal):
 *   Platform  -- SDL3 window, event pump, timing
 *   Backend   -- SDL3 GPU device, pipelines, vertex streams, shaders
 *   Frontend  -- this public API (lifecycle + draw + query)
 */
#ifndef IVP_RENDER_H
#define IVP_RENDER_H

#include <ivp/base/ivp_result.h>
#include <ivp/render/ivp_render_camera.h>
#include <ivp/render/ivp_render_math.h>
#include <ivp/render/ivp_render_color.h>
#include <stdbool.h>
#include <SDL3/SDL.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Forward declarations ──────────────────────────────────────────────── */

typedef struct ivp_renderer ivp_renderer_t;

/* ── Text alignment ───────────────────────────────────────────────────── */

typedef enum ivp_text_align {
    IVP_TEXT_ALIGN_LEFT   = 0,
    IVP_TEXT_ALIGN_CENTER = 1,
    IVP_TEXT_ALIGN_RIGHT  = 2
} ivp_text_align_t;

/* ── Configuration ─────────────────────────────────────────────────────── */

#define IVP_RENDER_DEFAULT_WIDTH           1280
#define IVP_RENDER_DEFAULT_HEIGHT          720
#define IVP_RENDER_DEFAULT_MAX_DRAW_VERTS  (128 * 1024)
#define IVP_RENDER_DEFAULT_MAX_HUD_VERTS   (32  * 1024)
#define IVP_RENDER_DEFAULT_MAX_TRI_VERTS   (256 * 1024)
#define IVP_RENDER_DEFAULT_SPHERE_SEGMENTS 24
#define IVP_RENDER_DEFAULT_HUD_FONT_SCALE  2
#define IVP_RENDER_HUD_FONT_W             5
#define IVP_RENDER_HUD_FONT_H             7

typedef struct ivp_render_config {
    int         width;              /* default: 1280 */
    int         height;             /* default: 720  */
    const char *title;              /* default: "IVP Example" */
    bool        vsync;              /* default: true */
    int         msaa_samples;       /* default: 4 (0=default, <0=disabled) */
    float       clear_color[4];     /* default: {0.20, 0.24, 0.30, 1.0} */
    float       max_frame_dt;       /* default: 0.1 */
    int         max_draw_vertices;  /* 3D wire stream capacity  (default: 128K) */
    int         max_hud_vertices;   /* 2D HUD stream cap        (default: 32K)  */
    int         max_tri_vertices;   /* 3D solid tri stream cap   (default: 256K) */
    int         sphere_segments;    /* wireframe sphere segs     (default: 24)   */
    int         hud_font_scale;     /* bitmap font scale         (default: 2)    */
} ivp_render_config_t;

void ivp_render_config_init(ivp_render_config_t *cfg);

/* ── Callback types ────────────────────────────────────────────────────── */

/* Per-event callback: return true to consume the event (skip camera). */
typedef bool (*ivp_event_callback_t)(const SDL_Event *ev, void *userdata);
/* Frame-phase callback: called at specific points in the frame. */
typedef void (*ivp_frame_callback_t)(void *userdata);

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

ivp_result_t ivp_renderer_create(ivp_renderer_t **out,
                                 const ivp_render_config_t *config);

void ivp_renderer_destroy(ivp_renderer_t *r);

/* ── Frame loop ────────────────────────────────────────────────────────── */

/* Process events, update camera, acquire GPU frame.
 * Returns false when the user closes the window. */
bool ivp_renderer_begin_frame(ivp_renderer_t *r, ivp_camera_t *cam);

/* Upload vertex data, execute render passes, present. */
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

/* ── Callback setters ──────────────────────────────────────────────────── */

void ivp_renderer_set_event_callback(ivp_renderer_t *r,
                                     ivp_event_callback_t cb, void *userdata);
void ivp_renderer_set_pre_events_callback(ivp_renderer_t *r,
                                          ivp_frame_callback_t cb, void *ud);
void ivp_renderer_set_post_events_callback(ivp_renderer_t *r,
                                           ivp_frame_callback_t cb, void *ud);
void ivp_renderer_set_pre_swap_callback(ivp_renderer_t *r,
                                        ivp_frame_callback_t cb, void *ud);

/* ── 3D Drawing primitives ─────────────────────────────────────────────── */

void ivp_render_draw_line(ivp_renderer_t *r,
                          const float a[3], const float b[3],
                          const float color[3]);

void ivp_render_draw_thick_line(ivp_renderer_t *r,
                                const float a[3], const float b[3],
                                const float color[3], float thickness);

void ivp_render_draw_wire_box(ivp_renderer_t *r,
                              const float pos[3],
                              const float half_size[3],
                              const float *rot,     /* 9 floats col-major, or NULL */
                              const float color[3]);

void ivp_render_draw_wire_sphere(ivp_renderer_t *r,
                                 const float pos[3],
                                 float radius,
                                 const float color[3]);

void ivp_render_draw_wire_sphere_ex(ivp_renderer_t *r,
                                    const float pos[3],
                                    float radius,
                                    const float color[3],
                                    int segments);

void ivp_render_draw_wire_cylinder(ivp_renderer_t *r,
                                   const float base[3],
                                   const float axis[3],
                                   float height, float radius,
                                   int segments,
                                   const float color[3]);

void ivp_render_draw_wire_quad(ivp_renderer_t *r,
                               const float corners[4][3],
                               const float color[3]);

void ivp_render_draw_arrow(ivp_renderer_t *r,
                           const float from[3], const float to[3],
                           const float color[3], float head_size);

void ivp_render_draw_ground_grid(ivp_renderer_t *r,
                                 float extent, float spacing,
                                 const float color[3]);

void ivp_render_draw_axes(ivp_renderer_t *r,
                          const float pos[3],
                          const float *rot,  /* 9 floats col-major, or NULL */
                          float scale);

void ivp_render_draw_circle(ivp_renderer_t *r,
                            const float center[3],
                            const float normal[3],
                            float radius, int segments,
                            const float color[3]);

void ivp_render_draw_wire_cone(ivp_renderer_t *r,
                               const float apex[3],
                               const float base_center[3],
                               float base_radius, int segments,
                               const float color[3]);

void ivp_render_draw_wire_capsule(ivp_renderer_t *r,
                                  const float a[3], const float b[3],
                                  float radius, int segments,
                                  const float color[3]);

void ivp_render_draw_plane(ivp_renderer_t *r,
                           const float center[3],
                           const float normal[3],
                           float extent,
                           const float color[3]);

void ivp_render_draw_aabb(ivp_renderer_t *r,
                          const float min[3], const float max[3],
                          const float color[3]);

void ivp_render_draw_cross(ivp_renderer_t *r,
                           const float pos[3],
                           float size,
                           const float color[3]);

void ivp_render_draw_point(ivp_renderer_t *r,
                           const float pos[3],
                           float size,
                           const float color[3]);

void ivp_render_draw_contact(ivp_renderer_t *r,
                             const float pos[3],
                             const float normal[3],
                             float normal_length,
                             const float color[3]);

/* While on, wire primitives are drawn on top of everything else (no depth
 * test), for debug overlays.  Reset to off at the start of every frame. */
void ivp_render_set_overlay(ivp_renderer_t *r, bool on);

/* ── 3D Solid drawing primitives (instanced, lit) ──────────────────────── */

void ivp_render_draw_solid_sphere(ivp_renderer_t *r,
                                  const float pos[3],
                                  float radius,
                                  const float color[3]);

void ivp_render_draw_solid_box(ivp_renderer_t *r,
                               const float pos[3],
                               const float half_size[3],
                               const float *rot,     /* 9 floats col-major, or NULL */
                               const float color[3]);

void ivp_render_draw_solid_aabb(ivp_renderer_t *r,
                                const float min[3], const float max[3],
                                const float color[3]);

/* Solid cylinder between the axis end points a and b. */
void ivp_render_draw_solid_cylinder(ivp_renderer_t *r,
                                    const float a[3], const float b[3],
                                    float radius,
                                    const float color[3]);

/* Solid cone; same arguments as ivp_render_draw_wire_cone. */
void ivp_render_draw_solid_cone(ivp_renderer_t *r,
                                const float apex[3],
                                const float base_center[3],
                                float base_radius,
                                const float color[3]);

/* Solid capsule: cylinder between a and b plus a hemisphere at each end. */
void ivp_render_draw_solid_capsule(ivp_renderer_t *r,
                                   const float a[3], const float b[3],
                                   float radius,
                                   const float color[3]);

/* ── Static solid meshes ───────────────────────────────────────────────── */

/* A triangle mesh uploaded once and drawn many times, instanced and lit like
 * the primitives above.  Triangles must wind counter-clockwise seen from
 * outside; normals are flat (one per triangle).  Returns NULL if the mesh is
 * empty, an index is out of range, or it needs more than 65535 vertices
 * (3 per triangle, so at most 21845 triangles).  `positions` is
 * 3 floats per vertex, `indices` 3 per triangle. */
typedef struct ivp_render_mesh ivp_render_mesh_t;

ivp_render_mesh_t *ivp_render_mesh_create(ivp_renderer_t *r,
                                          const float *positions,
                                          int vertex_count,
                                          const int *indices,
                                          int triangle_count);

/* Frees the mesh; r may be NULL.  Meshes still alive when the renderer is
 * destroyed are released with it. */
void ivp_render_mesh_destroy(ivp_render_mesh_t *mesh);

/* model: column-major 4x4 (mesh space -> world). */
void ivp_render_draw_solid_mesh(ivp_renderer_t *r,
                                ivp_render_mesh_t *mesh,
                                const float model[16],
                                const float color[3]);

/* ── Lighting control ──────────────────────────────────────────────────── */

/* Directional light shadows for the solid primitives (on by default). */
void ivp_render_set_shadows(ivp_renderer_t *r, bool on);
bool ivp_render_get_shadows(const ivp_renderer_t *r);

void ivp_render_set_light_dir(ivp_renderer_t *r, const float dir[3]);
void ivp_render_set_ambient(ivp_renderer_t *r, float ambient);

/* ── 2D HUD text ───────────────────────────────────────────────────────── */

void ivp_render_draw_text(ivp_renderer_t *r,
                          float x, float y,
                          const char *text,
                          const float color[3]);

void ivp_render_draw_text_scaled(ivp_renderer_t *r,
                                 float x, float y,
                                 const char *text,
                                 const float color[3],
                                 int scale);

void ivp_render_draw_text_aligned(ivp_renderer_t *r,
                                  float x, float y,
                                  const char *text,
                                  const float color[3],
                                  ivp_text_align_t align);

void ivp_render_draw_text_fmt(ivp_renderer_t *r,
                              float x, float y,
                              const float color[3],
                              const char *fmt, ...);

/* Text measurement (independent of renderer instance). */
float ivp_render_text_width(const char *text, int scale);
float ivp_render_line_height(int scale);

/* ── Projection / picking ──────────────────────────────────────────────── */

bool ivp_renderer_project(const ivp_renderer_t *r,
                          const float world[3], float screen[2]);

void ivp_render_unproject_ray(const ivp_renderer_t *r,
                              float screen_x, float screen_y,
                              float ray_origin[3], float ray_dir[3]);

/* Matrix-explicit variant for custom camera setups. */
void ivp_render_unproject_ray_ex(const float vp_mat[16],
                                 int viewport_w, int viewport_h,
                                 float screen_x, float screen_y,
                                 float ray_origin[3], float ray_dir[3]);

bool ivp_render_ray_sphere(const float ray_origin[3],
                           const float ray_dir[3],
                           const float sphere_center[3],
                           float sphere_radius, float *t_out);

bool ivp_render_ray_aabb(const float ray_origin[3],
                         const float ray_dir[3],
                         const float aabb_min[3],
                         const float aabb_max[3],
                         float *t_out);

/* ── Frame statistics ──────────────────────────────────────────────────── */

typedef struct ivp_render_stats {
    int   draw_vertex_count;
    int   draw_vertex_capacity;
    float draw_utilization;
    int   hud_vertex_count;
    int   hud_vertex_capacity;
    float hud_utilization;
    int   tri_vertex_count;
    int   tri_vertex_capacity;
    float tri_utilization;
    int   solid_sphere_count;
    int   solid_box_count;
    int   culled_count;
    float frame_dt;
    float fps;
} ivp_render_stats_t;

/* ── Screenshots ───────────────────────────────────────────────────────── */

/* Save the next completed frame (HUD included) as an RGB PNG.  The file is
 * written when ivp_renderer_end_frame returns; NULL cancels a pending request. */
void ivp_render_request_capture(ivp_renderer_t *r, const char *png_path);
bool ivp_render_capture_pending(const ivp_renderer_t *r);

void ivp_render_stats_get(const ivp_renderer_t *r, ivp_render_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_H */
