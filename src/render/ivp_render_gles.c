/* ivp_render_gles.c -- GPU backend on OpenGL ES 3.0 (WebGL2 in the browser)
 *
 * Implements ivp_render_gpu.h for the Emscripten build, where SDL3's GPU
 * API has no backend.  GL keeps its resources alive while they are in use,
 * so per-frame buffers are a single buffer object rewritten each frame and
 * there are no fences.  Pipelines are a program plus fixed-function state;
 * pushed uniforms go to one std140 uniform buffer at binding 0.
 *
 * Conventions that differ from SDL GPU are handled in the shaders
 * (src/render/shaders/es3): the light matrix keeps SDL GPU's depth range of
 * [0,1], which GL stores as 0.5 * z + 0.5, and GL textures start at the
 * bottom row.  Multisampling is the context's (WebGL antialias).
 */

#include "ivp_render_gpu.h"
#include "ivp_render_gles_shaders.h"

#include <GLES3/gl3.h>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ATTRIBS 8   /* locations 0..7 */

struct ivp_gpu_pass {
    ivp_gpu_device_t *dev;
};

struct ivp_gpu_pipeline {
    GLuint                program;
    GLenum                primitive;
    int                   stride;
    ivp_gpu_attrib_desc_t attribs[MAX_ATTRIBS];
    int                   attrib_count;
    int                   inst_stride;
    ivp_gpu_attrib_desc_t inst_attribs[MAX_ATTRIBS];
    int                   inst_attrib_count;
    bool                  depth_test, depth_write, depth_always;
    bool                  blend, depth_only;
};

struct ivp_gpu_buffer {
    GLuint vbo;
};

struct ivp_gpu_mesh {
    GLuint vbo;
    GLuint ibo;
    int    index_count;
};

struct ivp_gpu_shadow_map {
    GLuint tex;
    GLuint fbo;
    int    size;
};

struct ivp_gpu_device {
    SDL_Window               *window;
    SDL_GLContext             context;
    GLuint                    programs[IVP_GPU_PROGRAM_COUNT];
    GLuint                    ubo;
    GLuint                    vao;
    unsigned                  enabled_attribs;   /* bit per attribute location */
    const ivp_gpu_pipeline_t *bound;
    ivp_gpu_pass_t            pass;
    int                       frame_w, frame_h;   /* drawable, device pixels */
};

/* Size of the largest uniform block (tri3d_inst: two mat4 and two vec4). */
#define UNIFORM_BUFFER_SIZE 160

/* ═══════════════════════════════════════════════════════════════════════════
 * Programs
 * ═══════════════════════════════════════════════════════════════════════════ */

static const char *const program_sources[IVP_GPU_PROGRAM_COUNT][2] = {
    [IVP_GPU_PROGRAM_LINE3D] = { ivp_gles_line3d_vert,     ivp_gles_line_frag       },
    [IVP_GPU_PROGRAM_LINE2D] = { ivp_gles_line2d_vert,     ivp_gles_line_frag       },
    [IVP_GPU_PROGRAM_SOLID]  = { ivp_gles_tri3d_inst_vert, ivp_gles_tri3d_inst_frag },
    [IVP_GPU_PROGRAM_SHADOW] = { ivp_gles_shadow_vert,     ivp_gles_shadow_frag     },
    [IVP_GPU_PROGRAM_RECT]   = { ivp_gles_hud_rect_vert,   ivp_gles_hud_rect_frag   },
};

static GLuint compile_shader(GLenum stage, const char *source)
{
    GLuint shader = glCreateShader(stage);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(shader, (GLsizei)sizeof(log), NULL, log);
        fprintf(stderr, "%s shader compilation failed: %s\n",
                stage == GL_VERTEX_SHADER ? "vertex" : "fragment", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

static GLuint link_program(const char *vs_source, const char *fs_source)
{
    GLuint vs = compile_shader(GL_VERTEX_SHADER, vs_source);
    GLuint fs = vs ? compile_shader(GL_FRAGMENT_SHADER, fs_source) : 0;
    if (!fs) {
        if (vs) glDeleteShader(vs);
        return 0;
    }
    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(program, (GLsizei)sizeof(log), NULL, log);
        fprintf(stderr, "shader program link failed: %s\n", log);
        glDeleteProgram(program);
        return 0;
    }

    /* Uniform block U at binding 0, the shadow map on texture unit 0 */
    GLuint block = glGetUniformBlockIndex(program, "U");
    if (block != GL_INVALID_INDEX) glUniformBlockBinding(program, block, 0);
    GLint sampler = glGetUniformLocation(program, "shadow_map");
    if (sampler >= 0) {
        glUseProgram(program);
        glUniform1i(sampler, 0);
        glUseProgram(0);
    }
    return program;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Device
 * ═══════════════════════════════════════════════════════════════════════════ */

SDL_WindowFlags ivp_gpu_window_flags(int msaa_samples)
{
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 0);   /* opaque canvas */
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, msaa_samples >= 2 ? 1 : 0);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, msaa_samples >= 4 ? 4 :
                                                   msaa_samples >= 2 ? 2 : 0);
    return SDL_WINDOW_OPENGL;
}

ivp_result_t ivp_gpu_device_create(ivp_gpu_device_t **out, SDL_Window *window,
                                   int msaa_samples, bool vsync)
{
    (void)msaa_samples;   /* set with the context attributes */
    *out = NULL;
    ivp_gpu_device_t *dev = (ivp_gpu_device_t *)calloc(1, sizeof(*dev));
    if (!dev) return IVP_ERR_OUT_OF_MEMORY;
    dev->window = window;

    dev->context = SDL_GL_CreateContext(window);
    if (!dev->context) {
        fprintf(stderr, "SDL_GL_CreateContext failed: %s\n", SDL_GetError());
        free(dev);
        return IVP_ERR_RENDER_GPU_DEVICE;
    }
#ifndef __EMSCRIPTEN__
    /* In the browser frames follow requestAnimationFrame (ivp_gpu_frame_end) */
    SDL_GL_SetSwapInterval(vsync ? 1 : 0);
#else
    (void)vsync;
#endif

    for (int p = 0; p < IVP_GPU_PROGRAM_COUNT; p++) {
        dev->programs[p] = link_program(program_sources[p][0], program_sources[p][1]);
        if (!dev->programs[p]) {
            ivp_gpu_device_destroy(dev);
            return IVP_ERR_RENDER_SHADER;
        }
    }

    glGenBuffers(1, &dev->ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, dev->ubo);
    glBufferData(GL_UNIFORM_BUFFER, UNIFORM_BUFFER_SIZE, NULL, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, dev->ubo);

    /* One vertex array; draws set the attribute layout of the pipeline */
    glGenVertexArrays(1, &dev->vao);
    glBindVertexArray(dev->vao);
    glDisable(GL_CULL_FACE);

    *out = dev;
    return IVP_OK;
}

void ivp_gpu_device_destroy(ivp_gpu_device_t *dev)
{
    if (!dev) return;
    for (int p = 0; p < IVP_GPU_PROGRAM_COUNT; p++)
        if (dev->programs[p]) glDeleteProgram(dev->programs[p]);
    if (dev->ubo) glDeleteBuffers(1, &dev->ubo);
    if (dev->vao) glDeleteVertexArrays(1, &dev->vao);
    SDL_GL_DestroyContext(dev->context);
    free(dev);
}

void ivp_gpu_wait_idle(ivp_gpu_device_t *dev)
{
    (void)dev;   /* GL keeps resources alive while frames use them */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Frame
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef __EMSCRIPTEN__
/* Return to the browser until its next animation frame (needs Asyncify): the
 * canvas is presented when control goes back to the event loop, and frames
 * follow the display like vsync.  Every pass of the example's loop must come
 * here, drawn or not, or the loop never gives the browser a turn and the tab
 * hangs.  `shown` counts the frame in Module.ivpFrames, for
 * tools/web/browser_smoke.py. */
EM_ASYNC_JS(void, ivp_gles_wait_animation_frame, (int shown), {
    if (shown) Module['ivpFrames'] = (Module['ivpFrames'] || 0) + 1;
    await new Promise(function(resolve) { requestAnimationFrame(resolve); });
});
#endif

ivp_gpu_frame_status_t ivp_gpu_frame_begin(ivp_gpu_device_t *dev)
{
    int pw, ph;
    SDL_GetWindowSizeInPixels(dev->window, &pw, &ph);
    if (pw <= 0 || ph <= 0) {
#ifdef __EMSCRIPTEN__
        /* a canvas of no height (a page shorter than its header) */
        ivp_gles_wait_animation_frame(0);
#endif
        return IVP_GPU_FRAME_SKIP;
    }
    dev->frame_w = pw;
    dev->frame_h = ph;
    dev->bound = NULL;
    return IVP_GPU_FRAME_READY;
}

void ivp_gpu_frame_end(ivp_gpu_device_t *dev)
{
#ifdef __EMSCRIPTEN__
    (void)dev;
    ivp_gles_wait_animation_frame(1);
#else
    SDL_GL_SwapWindow(dev->window);
#endif
}

void ivp_gpu_frame_cancel(ivp_gpu_device_t *dev)
{
    (void)dev;
#ifdef __EMSCRIPTEN__
    ivp_gles_wait_animation_frame(0);
#endif
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Pipelines
 * ═══════════════════════════════════════════════════════════════════════════ */

static int copy_attribs(ivp_gpu_attrib_desc_t dst[MAX_ATTRIBS],
                        const ivp_gpu_attrib_desc_t *src, int count)
{
    int n = 0;
    for (int i = 0; src && i < count && n < MAX_ATTRIBS; i++)
        if (src[i].location >= 0 && src[i].location < MAX_ATTRIBS) dst[n++] = src[i];
    return n;
}

ivp_gpu_pipeline_t *ivp_gpu_pipeline_create(ivp_gpu_device_t *dev,
                                            const ivp_gpu_pipeline_desc_t *desc)
{
    ivp_gpu_pipeline_t *p = (ivp_gpu_pipeline_t *)calloc(1, sizeof(*p));
    if (!p) return NULL;
    p->program      = dev->programs[desc->program];
    p->primitive    = desc->primitive == IVP_GPU_PRIMITIVE_LINES ? GL_LINES : GL_TRIANGLES;
    p->stride       = desc->stride;
    p->attrib_count = copy_attribs(p->attribs, desc->attribs, desc->attrib_count);
    p->inst_stride  = desc->inst_stride;
    p->inst_attrib_count = copy_attribs(p->inst_attribs, desc->inst_attribs,
                                        desc->inst_attrib_count);
    p->depth_test   = desc->depth_test;
    p->depth_write  = desc->depth_write;
    p->depth_always = desc->depth_always;
    p->blend        = desc->blend;
    p->depth_only   = desc->depth_only;
    return p;
}

void ivp_gpu_pipeline_destroy(ivp_gpu_device_t *dev, ivp_gpu_pipeline_t *pipe)
{
    (void)dev;   /* the programs belong to the device */
    free(pipe);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Buffers and meshes
 * ═══════════════════════════════════════════════════════════════════════════ */

ivp_gpu_buffer_t *ivp_gpu_buffer_create(ivp_gpu_device_t *dev, size_t size)
{
    (void)dev;
    (void)size;   /* allocated by each upload */
    ivp_gpu_buffer_t *b = (ivp_gpu_buffer_t *)calloc(1, sizeof(*b));
    if (!b) return NULL;
    glGenBuffers(1, &b->vbo);
    return b;
}

void ivp_gpu_buffer_destroy(ivp_gpu_device_t *dev, ivp_gpu_buffer_t *buf)
{
    (void)dev;
    if (!buf) return;
    glDeleteBuffers(1, &buf->vbo);
    free(buf);
}

void ivp_gpu_buffer_upload(ivp_gpu_device_t *dev, ivp_gpu_buffer_t *buf,
                           const void *data, size_t size)
{
    (void)dev;
    /* new storage each frame: the GPU may still read last frame's */
    glBindBuffer(GL_ARRAY_BUFFER, buf->vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)size, data, GL_STREAM_DRAW);
}

ivp_gpu_mesh_t *ivp_gpu_mesh_create(ivp_gpu_device_t *dev,
                                    const void *vtx_data, int vtx_count,
                                    int vtx_stride,
                                    const Uint16 *idx_data, int idx_count)
{
    (void)dev;
    ivp_gpu_mesh_t *m = (ivp_gpu_mesh_t *)calloc(1, sizeof(*m));
    if (!m) return NULL;
    m->index_count = idx_count;

    glGenBuffers(1, &m->vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m->vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)vtx_count * vtx_stride, vtx_data,
                 GL_STATIC_DRAW);

    glGenBuffers(1, &m->ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)idx_count * (GLsizeiptr)sizeof(Uint16),
                 idx_data, GL_STATIC_DRAW);
    return m;
}

void ivp_gpu_mesh_destroy(ivp_gpu_device_t *dev, ivp_gpu_mesh_t *mesh)
{
    (void)dev;
    if (!mesh) return;
    glDeleteBuffers(1, &mesh->vbo);
    glDeleteBuffers(1, &mesh->ibo);
    free(mesh);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Shadow map
 * ═══════════════════════════════════════════════════════════════════════════ */

ivp_gpu_shadow_map_t *ivp_gpu_shadow_map_create(ivp_gpu_device_t *dev, int size)
{
    (void)dev;
    ivp_gpu_shadow_map_t *map = (ivp_gpu_shadow_map_t *)calloc(1, sizeof(*map));
    if (!map) return NULL;
    map->size = size;

    /* Compared when sampled: LINEAR filtering gives 2x2 PCF */
    glGenTextures(1, &map->tex);
    glBindTexture(GL_TEXTURE_2D, map->tex);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_DEPTH_COMPONENT24, size, size);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenFramebuffers(1, &map->fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, map->fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                           map->tex, 0);
    const GLenum none = GL_NONE;
    glDrawBuffers(1, &none);
    glReadBuffer(GL_NONE);
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        fprintf(stderr, "shadow map framebuffer incomplete (0x%x)\n", (unsigned)status);
        ivp_gpu_shadow_map_destroy(dev, map);
        return NULL;
    }
    return map;
}

void ivp_gpu_shadow_map_destroy(ivp_gpu_device_t *dev, ivp_gpu_shadow_map_t *map)
{
    (void)dev;
    if (!map) return;
    glDeleteFramebuffers(1, &map->fbo);
    glDeleteTextures(1, &map->tex);
    free(map);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Passes and draws
 * ═══════════════════════════════════════════════════════════════════════════ */

ivp_gpu_pass_t *ivp_gpu_shadow_pass_begin(ivp_gpu_device_t *dev,
                                          ivp_gpu_shadow_map_t *map)
{
    glBindFramebuffer(GL_FRAMEBUFFER, map->fbo);
    glViewport(0, 0, map->size, map->size);
    glDepthMask(GL_TRUE);
    glClearDepthf(1.0f);
    glClear(GL_DEPTH_BUFFER_BIT);
    dev->pass.dev = dev;
    return &dev->pass;
}

ivp_gpu_pass_t *ivp_gpu_main_pass_begin(ivp_gpu_device_t *dev,
                                        const float clear[4])
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, dev->frame_w, dev->frame_h);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);
    glClearColor(clear[0], clear[1], clear[2], clear[3]);
    glClearDepthf(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    dev->pass.dev = dev;
    return &dev->pass;
}

void ivp_gpu_pass_end(ivp_gpu_pass_t *pass)
{
    (void)pass;
}

void ivp_gpu_bind_pipeline(ivp_gpu_pass_t *pass, ivp_gpu_pipeline_t *pipe)
{
    pass->dev->bound = pipe;
    glUseProgram(pipe->program);

    if (pipe->depth_test || pipe->depth_always) {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(pipe->depth_always ? GL_ALWAYS : GL_LESS);
    } else {
        glDisable(GL_DEPTH_TEST);
    }
    glDepthMask(pipe->depth_write ? GL_TRUE : GL_FALSE);

    if (pipe->blend) {
        glEnable(GL_BLEND);
        glBlendEquation(GL_FUNC_ADD);
        glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA,
                            GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    } else {
        glDisable(GL_BLEND);
    }

    if (pipe->depth_only) {
        /* SDL GPU's depth bias: constant 2.0, slope 2.5 */
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(2.5f, 2.0f);
    } else {
        glDisable(GL_POLYGON_OFFSET_FILL);
    }
}

void ivp_gpu_push_uniforms(ivp_gpu_pass_t *pass, const void *data, size_t size)
{
    glBindBuffer(GL_UNIFORM_BUFFER, pass->dev->ubo);
    glBufferSubData(GL_UNIFORM_BUFFER, 0,
                    (GLsizeiptr)(size < UNIFORM_BUFFER_SIZE ? size : UNIFORM_BUFFER_SIZE),
                    data);
}

void ivp_gpu_bind_shadow_map(ivp_gpu_pass_t *pass, ivp_gpu_shadow_map_t *map)
{
    (void)pass;
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, map->tex);
}

/* Point the pipeline's attributes at `vbo`; returns the locations set. */
static unsigned set_attribs(const ivp_gpu_attrib_desc_t *attribs, int count,
                            int stride, GLuint vbo, GLuint divisor)
{
    unsigned mask = 0;
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    for (int i = 0; i < count; i++) {
        GLuint loc = (GLuint)attribs[i].location;
        glEnableVertexAttribArray(loc);
        glVertexAttribPointer(loc, attribs[i].components, GL_FLOAT, GL_FALSE, stride,
                              (const void *)(uintptr_t)attribs[i].offset);
        glVertexAttribDivisor(loc, divisor);
        mask |= 1u << loc;
    }
    return mask;
}

static void disable_unused_attribs(ivp_gpu_device_t *dev, unsigned used)
{
    unsigned stale = dev->enabled_attribs & ~used;
    for (GLuint loc = 0; loc < MAX_ATTRIBS; loc++)
        if (stale & (1u << loc)) glDisableVertexAttribArray(loc);
    dev->enabled_attribs = used;
}

void ivp_gpu_draw(ivp_gpu_pass_t *pass, const ivp_gpu_buffer_t *buf,
                  int vertex_count)
{
    const ivp_gpu_pipeline_t *p = pass->dev->bound;
    unsigned used = set_attribs(p->attribs, p->attrib_count, p->stride, buf->vbo, 0);
    disable_unused_attribs(pass->dev, used);
    glDrawArrays(p->primitive, 0, vertex_count);
}

void ivp_gpu_draw_mesh_instanced(ivp_gpu_pass_t *pass, const ivp_gpu_mesh_t *mesh,
                                 const ivp_gpu_buffer_t *instances,
                                 int instance_count)
{
    const ivp_gpu_pipeline_t *p = pass->dev->bound;
    unsigned used = set_attribs(p->attribs, p->attrib_count, p->stride, mesh->vbo, 0);
    used |= set_attribs(p->inst_attribs, p->inst_attrib_count, p->inst_stride,
                        instances->vbo, 1);
    disable_unused_attribs(pass->dev, used);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->ibo);
    glDrawElementsInstanced(GL_TRIANGLES, mesh->index_count, GL_UNSIGNED_SHORT,
                            NULL, instance_count);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Screenshot
 * ═══════════════════════════════════════════════════════════════════════════ */

bool ivp_gpu_capture_begin(ivp_gpu_device_t *dev)
{
    (void)dev;
    fprintf(stderr, "screenshots are not supported by the OpenGL ES backend\n");
    return false;
}

unsigned char *ivp_gpu_capture_take(ivp_gpu_device_t *dev, int *width, int *height)
{
    (void)dev;
    *width = 0;
    *height = 0;
    return NULL;
}
