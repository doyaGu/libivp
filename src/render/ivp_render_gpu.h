/* ivp_render_gpu.h -- Internal: the render module's GPU backend interface
 *
 * This header is INTERNAL to the render module. It is not part of the public
 * API and should not be included by code outside src/render/.
 *
 * The renderer draws through this interface only.  One backend implements
 * it, chosen at configure time (IVP_RENDER_BACKEND):
 *   ivp_render_sdlgpu.c  SDL3 GPU (Metal, Vulkan, D3D12): the native default
 *   ivp_render_gles.c    OpenGL ES 3.0 / WebGL2: the Emscripten build
 *
 * The device, pipelines, buffers, meshes, shadow map and passes are opaque
 * backend objects.  Streams and instance buffers (ivp_render_stream.c) add
 * CPU staging on top of backend buffers.
 *
 * A frame:
 *   ivp_gpu_frame_begin
 *   ivp_gpu_buffer_upload ...            (before the first pass)
 *   ivp_gpu_shadow_pass_begin ... ivp_gpu_pass_end   (optional)
 *   ivp_gpu_capture_begin                (optional, before the main pass)
 *   ivp_gpu_main_pass_begin ... ivp_gpu_pass_end
 *   ivp_gpu_frame_end                    (or ivp_gpu_frame_cancel)
 */
#ifndef IVP_RENDER_GPU_H
#define IVP_RENDER_GPU_H

#include <ivp/base/ivp_result.h>
#include <SDL3/SDL.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_gpu_device     ivp_gpu_device_t;
typedef struct ivp_gpu_pipeline   ivp_gpu_pipeline_t;
typedef struct ivp_gpu_buffer     ivp_gpu_buffer_t;
typedef struct ivp_gpu_mesh       ivp_gpu_mesh_t;
typedef struct ivp_gpu_shadow_map ivp_gpu_shadow_map_t;
typedef struct ivp_gpu_pass       ivp_gpu_pass_t;

/* ── Device ────────────────────────────────────────────────────────────── */

/* Window flags the backend needs, to be added to SDL_CreateWindow's.  Also
 * sets any attributes that must be in place before the window exists. */
SDL_WindowFlags ivp_gpu_window_flags(int msaa_samples);

ivp_result_t ivp_gpu_device_create(ivp_gpu_device_t **out, SDL_Window *window,
                                   int msaa_samples, bool vsync);
void         ivp_gpu_device_destroy(ivp_gpu_device_t *dev);

/* Wait until the GPU no longer reads any resource (before destroying one
 * that frames in flight may use). */
void ivp_gpu_wait_idle(ivp_gpu_device_t *dev);

/* ── Frame ─────────────────────────────────────────────────────────────── */

typedef enum ivp_gpu_frame_status {
    IVP_GPU_FRAME_READY,   /* record and draw this frame */
    IVP_GPU_FRAME_SKIP,    /* no target this frame (minimised, resizing) */
    IVP_GPU_FRAME_ERROR    /* the device failed */
} ivp_gpu_frame_status_t;

ivp_gpu_frame_status_t ivp_gpu_frame_begin(ivp_gpu_device_t *dev);
void ivp_gpu_frame_end(ivp_gpu_device_t *dev);      /* submit and present */
void ivp_gpu_frame_cancel(ivp_gpu_device_t *dev);   /* drop the frame */

/* ── Pipelines ─────────────────────────────────────────────────────────── */

/* Shader programs; each backend supplies its own code for them. */
typedef enum ivp_gpu_program {
    IVP_GPU_PROGRAM_LINE3D,   /* 3D lines: pos3 + col3, uniform mat4 vp */
    IVP_GPU_PROGRAM_LINE2D,   /* 2D lines: pos2 + col3, uniform mat4 ortho */
    IVP_GPU_PROGRAM_SOLID,    /* instanced lit solids with shadows */
    IVP_GPU_PROGRAM_SHADOW,   /* instanced depth only, uniform mat4 light_vp */
    IVP_GPU_PROGRAM_RECT,     /* 2D rectangles: pos2 + rgba, uniform mat4 ortho */
    IVP_GPU_PROGRAM_COUNT
} ivp_gpu_program_t;

typedef enum ivp_gpu_primitive {
    IVP_GPU_PRIMITIVE_LINES,
    IVP_GPU_PRIMITIVE_TRIANGLES
} ivp_gpu_primitive_t;

typedef struct ivp_gpu_attrib_desc {
    int location;
    int components;  /* 2, 3, or 4 floats */
    int offset;      /* byte offset within the vertex struct */
} ivp_gpu_attrib_desc_t;

typedef struct ivp_gpu_pipeline_desc {
    ivp_gpu_program_t            program;
    ivp_gpu_primitive_t          primitive;
    /* Per-vertex attributes (vertex buffer, or the mesh for instanced draws) */
    int                          stride;
    const ivp_gpu_attrib_desc_t *attribs;
    int                          attrib_count;
    /* Per-instance attributes (instanced pipelines only) */
    int                          inst_stride;
    const ivp_gpu_attrib_desc_t *inst_attribs;
    int                          inst_attrib_count;
    bool depth_test;
    bool depth_write;
    bool depth_always;    /* keep the depth target, never reject (overlays) */
    bool blend;           /* straight alpha blending */
    bool depth_only;      /* shadow pass: no colour target, depth bias */
    bool shadow_sampler;  /* the fragment shader samples the shadow map */
} ivp_gpu_pipeline_desc_t;

ivp_gpu_pipeline_t *ivp_gpu_pipeline_create(ivp_gpu_device_t *dev,
                                            const ivp_gpu_pipeline_desc_t *desc);
void ivp_gpu_pipeline_destroy(ivp_gpu_device_t *dev, ivp_gpu_pipeline_t *pipe);

/* ── Buffers and meshes ────────────────────────────────────────────────── */

/* Vertex data rewritten every frame; `size` is the capacity in bytes.
 * Upload between ivp_gpu_frame_begin and the first pass. */
ivp_gpu_buffer_t *ivp_gpu_buffer_create(ivp_gpu_device_t *dev, size_t size);
void ivp_gpu_buffer_destroy(ivp_gpu_device_t *dev, ivp_gpu_buffer_t *buf);
void ivp_gpu_buffer_upload(ivp_gpu_device_t *dev, ivp_gpu_buffer_t *buf,
                           const void *data, size_t size);

/* Static vertices with 16-bit indices, drawn as instanced triangles. */
ivp_gpu_mesh_t *ivp_gpu_mesh_create(ivp_gpu_device_t *dev,
                                    const void *vtx_data, int vtx_count,
                                    int vtx_stride,
                                    const Uint16 *idx_data, int idx_count);
void ivp_gpu_mesh_destroy(ivp_gpu_device_t *dev, ivp_gpu_mesh_t *mesh);

/* ── Shadow map (square depth texture, compared when sampled) ──────────── */

ivp_gpu_shadow_map_t *ivp_gpu_shadow_map_create(ivp_gpu_device_t *dev, int size);
void ivp_gpu_shadow_map_destroy(ivp_gpu_device_t *dev, ivp_gpu_shadow_map_t *map);

/* ── Passes and draws ──────────────────────────────────────────────────── */

/* Depth-only pass into the shadow map, cleared to 1.  NULL on failure. */
ivp_gpu_pass_t *ivp_gpu_shadow_pass_begin(ivp_gpu_device_t *dev,
                                          ivp_gpu_shadow_map_t *map);
/* The window pass (or the capture target), colour cleared to `clear` and
 * depth to 1, with a width x height viewport.  NULL on failure: cancel the
 * frame. */
ivp_gpu_pass_t *ivp_gpu_main_pass_begin(ivp_gpu_device_t *dev,
                                        const float clear[4],
                                        int width, int height);
void ivp_gpu_pass_end(ivp_gpu_pass_t *pass);

void ivp_gpu_bind_pipeline(ivp_gpu_pass_t *pass, ivp_gpu_pipeline_t *pipe);
/* Vertex stage uniform block 0; stays bound for the following draws. */
void ivp_gpu_push_uniforms(ivp_gpu_pass_t *pass, const void *data, size_t size);
void ivp_gpu_bind_shadow_map(ivp_gpu_pass_t *pass, ivp_gpu_shadow_map_t *map);

void ivp_gpu_draw(ivp_gpu_pass_t *pass, const ivp_gpu_buffer_t *buf,
                  int vertex_count);
void ivp_gpu_draw_mesh_instanced(ivp_gpu_pass_t *pass, const ivp_gpu_mesh_t *mesh,
                                 const ivp_gpu_buffer_t *instances,
                                 int instance_count);

/* ── Screenshot ────────────────────────────────────────────────────────── */

/* Read back this frame: call before ivp_gpu_main_pass_begin.  False when the
 * backend cannot capture. */
bool ivp_gpu_capture_begin(ivp_gpu_device_t *dev);
/* After ivp_gpu_frame_end: the captured frame as tightly packed RGB8 rows,
 * top row first, owned by the caller (free()).  NULL if capturing failed. */
unsigned char *ivp_gpu_capture_take(ivp_gpu_device_t *dev, int *width, int *height);

/* ── Streams and instance buffers (ivp_render_stream.c) ────────────────── */

/* A pipeline plus a per-frame vertex buffer with CPU staging. */
typedef struct ivp_gpu_stream {
    ivp_gpu_pipeline_t *pipeline;
    ivp_gpu_buffer_t   *buf;
    void               *vertices;      /* CPU staging */
    int                 vertex_count;
    int                 capacity;      /* in vertices */
    int                 stride;        /* bytes per vertex */
} ivp_gpu_stream_t;

ivp_result_t ivp_gpu_stream_create(ivp_gpu_stream_t *out, ivp_gpu_device_t *dev,
                                   const ivp_gpu_pipeline_desc_t *desc,
                                   int capacity);
void ivp_gpu_stream_destroy(ivp_gpu_stream_t *s, ivp_gpu_device_t *dev);

static inline void ivp_gpu_stream_reset(ivp_gpu_stream_t *s)
{
    s->vertex_count = 0;
}

/* Room for `count` vertices in the staging area, or NULL when full. */
void *ivp_gpu_stream_push(ivp_gpu_stream_t *s, int count);
void  ivp_gpu_stream_upload(ivp_gpu_stream_t *s, ivp_gpu_device_t *dev);
/* Bind the pipeline, push the uniforms and draw the staged vertices. */
void  ivp_gpu_stream_draw(ivp_gpu_stream_t *s, ivp_gpu_pass_t *pass,
                          const void *uniform_data, size_t uniform_size);

/* Per-instance data for instanced mesh draws, with CPU staging. */
typedef struct ivp_gpu_instance_buf {
    ivp_gpu_buffer_t *buf;
    void             *data;       /* CPU staging */
    int               count;      /* current instance count */
    int               capacity;   /* max instances */
    int               stride;     /* bytes per instance */
} ivp_gpu_instance_buf_t;

ivp_result_t ivp_gpu_instance_buf_create(ivp_gpu_instance_buf_t *out,
                                         ivp_gpu_device_t *dev,
                                         int capacity, int stride);
void ivp_gpu_instance_buf_destroy(ivp_gpu_instance_buf_t *b, ivp_gpu_device_t *dev);

static inline void ivp_gpu_instance_buf_reset(ivp_gpu_instance_buf_t *b)
{
    b->count = 0;
}

void *ivp_gpu_instance_buf_push(ivp_gpu_instance_buf_t *b, int count);
void  ivp_gpu_instance_buf_upload(ivp_gpu_instance_buf_t *b, ivp_gpu_device_t *dev);
/* Draw `mesh` once per staged instance; the pipeline is already bound. */
void  ivp_gpu_instance_buf_draw(const ivp_gpu_instance_buf_t *b,
                                const ivp_gpu_mesh_t *mesh, ivp_gpu_pass_t *pass);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_GPU_H */
