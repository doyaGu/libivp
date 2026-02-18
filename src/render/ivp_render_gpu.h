/* ivp_render_gpu.h -- Internal: shader compile, VAO/VBO management
 *
 * This header is INTERNAL to the render module. It is not part of the public
 * API and should not be included by code outside src/render/.
 */
#ifndef IVP_RENDER_GPU_H
#define IVP_RENDER_GPU_H

#include <ivp/render/ivp_render_types.h>
#include <glad/glad.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Shader handle ─────────────────────────────────────────────────────── */

typedef struct ivp_render_shader {
    GLuint program;
    GLint  u_mvp;   /* uniform location for MVP / projection matrix */
} ivp_render_shader_t;

/* ── Vertex buffer handle ──────────────────────────────────────────────── */

typedef struct ivp_render_vbuf {
    GLuint vao;
    GLuint vbo;
    int    capacity;
    int    stride;
} ivp_render_vbuf_t;

/* ── Vertex attribute descriptor ───────────────────────────────────────── */

typedef struct ivp_render_attrib_desc {
    GLuint index;
    GLint  size;        /* number of components (2 or 3) */
    int    offset;      /* byte offset within vertex struct */
} ivp_render_attrib_desc_t;

/* ── Shader sources ────────────────────────────────────────────────────── */

/* 3D line shader (position + color, MVP transform) */
extern const char *ivp_render_vs_3d_src;
extern const char *ivp_render_fs_3d_src;

/* 2D HUD shader (position + color, orthographic projection) */
extern const char *ivp_render_vs_2d_src;
extern const char *ivp_render_fs_2d_src;

/* ── Functions ─────────────────────────────────────────────────────────── */

/* Compile and link a shader program. Returns IVP_OK on success. */
ivp_result_t ivp_render_shader_compile(ivp_render_shader_t *out,
                                       const char *vs_src,
                                       const char *fs_src,
                                       const char *uniform_name);

void ivp_render_shader_destroy(ivp_render_shader_t *s);

/* Create a vertex buffer with the given capacity and vertex layout. */
ivp_result_t ivp_render_vbuf_create(ivp_render_vbuf_t *out,
                                    int capacity,
                                    int stride,
                                    const ivp_render_attrib_desc_t *attribs,
                                    int attrib_count);

void ivp_render_vbuf_destroy(ivp_render_vbuf_t *buf);

/* Upload vertex data and draw as GL_LINES. */
void ivp_render_vbuf_draw_lines(const ivp_render_vbuf_t *buf,
                                const ivp_render_shader_t *shader,
                                const float *matrix,
                                const void *data,
                                int vertex_count);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_GPU_H */
