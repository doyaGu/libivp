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

/* ── HUD atlas ─────────────────────────────────────────────────────────── */

SDL_GPUTexture *ivp_hud_atlas_create(ivp_gpu_device_t *dev);

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
    ivp_gpu_proto_mesh_t    sphere;
    ivp_gpu_proto_mesh_t    box;
    ivp_gpu_proto_mesh_t    cylinder;   /* radius 1, y in [-1,1] */
    ivp_gpu_proto_mesh_t    cone;       /* base radius 1 at y=-1, apex y=+1 */
    ivp_gpu_instance_buf_t  sphere_instances;
    ivp_gpu_instance_buf_t  box_instances;
    ivp_gpu_instance_buf_t  cylinder_instances;
    ivp_gpu_instance_buf_t  cone_instances;
    bool                    ready;
} ivp_proto_shapes_t;

/* User mesh (ivp_render_mesh_create): static geometry drawn instanced with
 * the same pipeline.  Linked into ivp_renderer.meshes. */
struct ivp_render_mesh {
    ivp_renderer_t         *renderer;
    ivp_render_mesh_t      *next;
    ivp_gpu_proto_mesh_t    mesh;
    ivp_gpu_instance_buf_t  instances;
    float                   center[3];   /* bounding sphere, mesh space */
    float                   radius;
};

/* Instance data layout (must match instanced vertex shader attributes). */
typedef struct ivp_solid_instance {
    float transform[16];  /* column-major 4×4 model matrix */
    float color[3];
    float _pad;
} ivp_solid_instance_t;

/* Proto shape lifecycle (defined in ivp_render_solid.c). */
ivp_result_t ivp_proto_shapes_init(ivp_proto_shapes_t *ps, ivp_gpu_device_t *dev);
void         ivp_proto_shapes_destroy(ivp_proto_shapes_t *ps, ivp_gpu_device_t *dev);

/* Per-frame hooks for the built-in shapes and every user mesh
 * (defined in ivp_render_solid.c). */
void ivp_solid_begin_frame(ivp_renderer_t *r);
void ivp_solid_upload(ivp_renderer_t *r);
void ivp_solid_draw(ivp_renderer_t *r, SDL_GPURenderPass *pass);
void ivp_solid_advance(ivp_renderer_t *r);
void ivp_solid_destroy_meshes(ivp_renderer_t *r);

/* ── Screenshot hooks (defined in ivp_render_capture.c) ────────────────── */

bool ivp_png_write_rgb(const char *path, const unsigned char *rgb, int w, int h);
SDL_GPUTexture *ivp_capture_target(ivp_renderer_t *r);   /* NULL: not capturing */
void ivp_capture_record(ivp_renderer_t *r);              /* blit + download */
void ivp_capture_finish(ivp_renderer_t *r, SDL_GPUFence *fence);
void ivp_capture_destroy(ivp_renderer_t *r);

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
    ivp_gpu_stream_t         stream_tri;  /* 3D solid triangles  */
    ivp_gpu_stream_t         stream_overlay; /* 3D lines, no depth test, drawn last */
    bool                     overlay_mode;   /* wire draws go to stream_overlay */

    /* Instanced solid rendering */
    SDL_GPUGraphicsPipeline *inst_pipeline;
    ivp_proto_shapes_t       protos;
    ivp_render_mesh_t       *meshes;      /* user meshes, singly linked */

    /* MSAA */
    ivp_gpu_msaa_t           msaa;

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
    SDL_GPUTexture          *capture_tex;
    SDL_GPUTransferBuffer   *capture_tb;
    int                      capture_w, capture_h;

    /* Per-frame GPU command state (valid between begin_frame / end_frame) */
    SDL_GPUCommandBuffer    *cmd;
    SDL_GPUTexture          *swapchain_tex;

    /* Frame-level fence ring (replaces per-stream fence tracking) */
    SDL_GPUFence            *frame_fences[IVP_GPU_RING_FRAMES];
    int                      frame_ring;  /* current ring index [0..2] */
};

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_INTERNAL_H */
