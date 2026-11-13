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

/* ── Frustum (6 planes, world-space) ───────────────────────────────────── */

typedef struct ivp_frustum {
    float planes[6][4];   /* [left,right,bottom,top,near,far] × (nx,ny,nz,d) */
} ivp_frustum_t;

void ivp_frustum_extract(ivp_frustum_t *f, const float vp[16]);
bool ivp_frustum_test_sphere(const ivp_frustum_t *f,
                             const float center[3], float radius);
bool ivp_frustum_test_aabb(const ivp_frustum_t *f,
                           const float mn[3], const float mx[3]);

/* ── Proto shape forward declarations ──────────────────────────────────── */

/* (declarations after struct definitions below) */

/* Push the backdrop gradient quad (ivp_render_hud.c). */
void ivp_hud_push_backdrop(ivp_renderer_t *r);

enum { IVP_RECT_BACKDROP = 0, IVP_RECT_PANEL = 1, IVP_RECT_FRONT = 2 };

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

/* ── Lighting ──────────────────────────────────────────────────────────── */

typedef struct ivp_render_lighting {
    float light_dir[3];   /* normalised directional light direction */
    float ambient;        /* ambient factor [0,1] */
} ivp_render_lighting_t;

/* ── Prototype shape meshes for instanced solid draw ───────────────────── */

typedef struct ivp_proto_shapes {
    ivp_gpu_mesh_t         *sphere;
    ivp_gpu_mesh_t         *box;
    ivp_gpu_mesh_t         *cylinder;   /* radius 1, y in [-1,1] */
    ivp_gpu_mesh_t         *cone;       /* base radius 1 at y=-1, apex y=+1 */
    ivp_gpu_instance_buf_t  sphere_instances;
    ivp_gpu_instance_buf_t  box_instances;
    ivp_gpu_instance_buf_t  cylinder_instances;
    ivp_gpu_instance_buf_t  cone_instances;
    /* translucent instances of sphere, box, cylinder, cone (blend pass) */
    ivp_gpu_instance_buf_t  blend[4];
    bool                    ready;
} ivp_proto_shapes_t;

/* User mesh (ivp_render_mesh_create): static geometry drawn instanced with
 * the same pipeline.  Linked into ivp_renderer.meshes. */
struct ivp_render_mesh {
    ivp_renderer_t         *renderer;
    ivp_render_mesh_t      *next;
    ivp_gpu_mesh_t         *mesh;
    ivp_gpu_instance_buf_t  instances;
    ivp_gpu_instance_buf_t  blend_instances;
    float                   center[3];   /* bounding sphere, mesh space */
    float                   radius;
};

/* Instance data layout (must match instanced vertex shader attributes). */
typedef struct ivp_solid_instance {
    float transform[16];  /* column-major 4×4 model matrix */
    float color[3];
    float alpha;          /* 1 = opaque; < 1 is drawn in the blend pass */
} ivp_solid_instance_t;

/* Pipeline description shared by the instanced solid and shadow pipelines:
 * proto mesh vertices plus ivp_solid_instance_t (ivp_render_core.c). */
void ivp_render_solid_pipeline_desc(ivp_gpu_pipeline_desc_t *desc,
                                    ivp_gpu_program_t program);

/* Proto shape lifecycle (defined in ivp_render_solid.c). */
ivp_result_t ivp_proto_shapes_init(ivp_proto_shapes_t *ps, ivp_gpu_device_t *dev);
void         ivp_proto_shapes_destroy(ivp_proto_shapes_t *ps, ivp_gpu_device_t *dev);

/* Per-frame hooks for the built-in shapes and every user mesh
 * (defined in ivp_render_solid.c). */
void ivp_solid_begin_frame(ivp_renderer_t *r);
void ivp_solid_upload(ivp_renderer_t *r);
void ivp_solid_draw(ivp_renderer_t *r, ivp_gpu_pass_t *pass);
void ivp_solid_draw_blend(ivp_renderer_t *r, ivp_gpu_pass_t *pass);
void ivp_solid_destroy_meshes(ivp_renderer_t *r);
bool ivp_solid_has_instances(const ivp_renderer_t *r);

/* ── Shadows (defined in ivp_render_shadow.c) ──────────────────────────── */

void ivp_shadow_light_matrix(float out[16], const float light_dir[3],
                             const float center[3], float extent, int map_size);
bool ivp_shadow_init(ivp_renderer_t *r);
void ivp_shadow_destroy(ivp_renderer_t *r);
void ivp_shadow_render(ivp_renderer_t *r);

/* ── Screenshot hooks (defined in ivp_render_capture.c) ────────────────── */

bool ivp_png_write_rgb(const char *path, const unsigned char *rgb, int w, int h);
/* Before the main pass: start reading back this frame if one is requested.
 * Returns whether the frame is captured. */
bool ivp_capture_begin(ivp_renderer_t *r);
/* After ivp_gpu_frame_end of a captured frame: write the PNG. */
void ivp_capture_finish(ivp_renderer_t *r);

/* ── Renderer struct ───────────────────────────────────────────────────── */

struct ivp_renderer {
    /* Platform */
    SDL_Window              *window;
    int                      width, height;

    /* GPU backend */
    ivp_gpu_device_t        *gpu;
    ivp_gpu_stream_t         stream_3d;   /* 3D wireframe lines */
    ivp_gpu_stream_t         stream_2d;   /* 2D HUD overlay     */
    /* 2D filled rectangles, alpha blended, in three layers drawn in order:
     * backdrop (before the 3D scene), panels, then text and widgets */
    ivp_gpu_stream_t         stream_rect[3];
    bool                     backdrop_on;
    float                    backdrop_top[3], backdrop_bottom[3];
    ivp_gpu_stream_t         stream_overlay; /* 3D lines, no depth test, drawn last */
    bool                     overlay_mode;   /* wire draws go to stream_overlay */

    /* Instanced solid rendering */
    ivp_gpu_pipeline_t      *inst_pipeline;
    ivp_gpu_pipeline_t      *inst_pipeline_blend;   /* alpha blended, no depth write */
    float                    solid_alpha;           /* applies to the next solid draws */
    ivp_proto_shapes_t       protos;
    ivp_render_mesh_t       *meshes;      /* user meshes, singly linked */

    /* Directional light shadow (ivp_render_shadow.c) */
    ivp_gpu_shadow_map_t    *shadow_map;
    int                      shadow_map_size;
    ivp_gpu_pipeline_t      *shadow_pipeline;
    bool                     shadows_enabled;
    bool                     shadow_valid;   /* the map holds this frame's depth */
    float                    light_vp[16];
    float                    cam_target[3];  /* centre of the shadow box */
    float                    cam_dist;       /* sizes the shadow box */

    /* Camera input state */
    ivp_camera_input_t       cam_input;

    /* Matrices (computed each frame) */
    ivp_render_mat4_t        view_mat;
    ivp_render_mat4_t        proj_mat;
    ivp_render_mat4_t        vp_mat;

    /* Frustum (computed each frame from VP matrix) */
    ivp_frustum_t            frustum;

    /* Lighting */
    ivp_render_lighting_t    lighting;

    /* Timing */
    Uint64                   start_ticks;
    Uint64                   prev_ticks;
    float                    frame_dt;
    float                    max_frame_dt;

    /* Stats */
    ivp_render_stats_t       last_stats;
    float                    ema_fps;
    int                      culled_count;   /* frustum-culled prims this frame */

    /* Config */
    float                    clear_col[4];
    int                      sphere_segments;
    int                      hud_font_scale;

    /* Callbacks */
    ivp_render_callbacks_t   callbacks;

    /* Screenshot (ivp_render_capture.c) */
    char                     capture_path[512];

    /* A frame is being recorded (between begin_frame and end_frame) */
    bool                     frame_active;
};

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_INTERNAL_H */
