/* ivp_render_stream.c -- Vertex streams and instance buffers
 *
 * Both stage their data in CPU memory during the frame and upload it to a
 * backend buffer once, before the passes draw it.
 */

#include "ivp_render_gpu.h"
#include <ivp/base/ivp_allocator.h>

#include <stdio.h>
#include <string.h>

/* ── Vertex stream ─────────────────────────────────────────────────────── */

ivp_result_t ivp_gpu_stream_create(ivp_gpu_stream_t *out, ivp_gpu_device_t *dev,
                                   const ivp_gpu_pipeline_desc_t *desc,
                                   int capacity)
{
    memset(out, 0, sizeof(*out));

    out->pipeline = ivp_gpu_pipeline_create(dev, desc);
    if (!out->pipeline) return IVP_ERR_RENDER_SHADER;

    out->capacity = capacity;
    out->stride   = desc->stride;
    size_t size = (size_t)capacity * (size_t)desc->stride;

    out->buf = ivp_gpu_buffer_create(dev, size);
    if (!out->buf) goto fail;
    out->vertices = ivp_alloc(ivp_default_allocator(), size);
    if (!out->vertices) goto fail;
    return IVP_OK;

fail:
    ivp_gpu_stream_destroy(out, dev);
    return IVP_ERR_OUT_OF_MEMORY;
}

void ivp_gpu_stream_destroy(ivp_gpu_stream_t *s, ivp_gpu_device_t *dev)
{
    if (!s) return;
    if (s->vertices) {
        ivp_free(ivp_default_allocator(), s->vertices,
                 (size_t)s->capacity * (size_t)s->stride);
        s->vertices = NULL;
    }
    if (dev) {
        ivp_gpu_buffer_destroy(dev, s->buf);
        ivp_gpu_pipeline_destroy(dev, s->pipeline);
    }
    s->buf = NULL;
    s->pipeline = NULL;
}

void *ivp_gpu_stream_push(ivp_gpu_stream_t *s, int count)
{
    if (s->vertex_count + count > s->capacity) return NULL;
    char *ptr = (char *)s->vertices + (size_t)s->vertex_count * (size_t)s->stride;
    s->vertex_count += count;
    return ptr;
}

void ivp_gpu_stream_upload(ivp_gpu_stream_t *s, ivp_gpu_device_t *dev)
{
    if (s->vertex_count <= 0) return;
    ivp_gpu_buffer_upload(dev, s->buf, s->vertices,
                          (size_t)s->vertex_count * (size_t)s->stride);
}

void ivp_gpu_stream_draw(ivp_gpu_stream_t *s, ivp_gpu_pass_t *pass,
                         const void *uniform_data, size_t uniform_size)
{
    if (s->vertex_count <= 0) return;
    ivp_gpu_bind_pipeline(pass, s->pipeline);
    ivp_gpu_push_uniforms(pass, uniform_data, uniform_size);
    ivp_gpu_draw(pass, s->buf, s->vertex_count);
}

/* ── Instance buffer ───────────────────────────────────────────────────── */

ivp_result_t ivp_gpu_instance_buf_create(ivp_gpu_instance_buf_t *out,
                                         ivp_gpu_device_t *dev,
                                         int capacity, int stride)
{
    memset(out, 0, sizeof(*out));
    out->capacity = capacity;
    out->stride   = stride;
    size_t size = (size_t)capacity * (size_t)stride;

    out->buf = ivp_gpu_buffer_create(dev, size);
    if (!out->buf) goto fail;
    out->data = ivp_alloc(ivp_default_allocator(), size);
    if (!out->data) goto fail;
    return IVP_OK;

fail:
    ivp_gpu_instance_buf_destroy(out, dev);
    return IVP_ERR_OUT_OF_MEMORY;
}

void ivp_gpu_instance_buf_destroy(ivp_gpu_instance_buf_t *b, ivp_gpu_device_t *dev)
{
    if (!b) return;
    if (b->data) {
        ivp_free(ivp_default_allocator(), b->data,
                 (size_t)b->capacity * (size_t)b->stride);
        b->data = NULL;
    }
    if (dev) ivp_gpu_buffer_destroy(dev, b->buf);
    b->buf = NULL;
}

void *ivp_gpu_instance_buf_push(ivp_gpu_instance_buf_t *b, int count)
{
    if (b->count + count > b->capacity) return NULL;
    char *ptr = (char *)b->data + (size_t)b->count * (size_t)b->stride;
    b->count += count;
    return ptr;
}

void ivp_gpu_instance_buf_upload(ivp_gpu_instance_buf_t *b, ivp_gpu_device_t *dev)
{
    if (b->count <= 0) return;
    ivp_gpu_buffer_upload(dev, b->buf, b->data,
                          (size_t)b->count * (size_t)b->stride);
}

void ivp_gpu_instance_buf_draw(const ivp_gpu_instance_buf_t *b,
                               const ivp_gpu_mesh_t *mesh, ivp_gpu_pass_t *pass)
{
    if (!mesh || b->count <= 0) return;
    ivp_gpu_draw_mesh_instanced(pass, mesh, b->buf, b->count);
}
