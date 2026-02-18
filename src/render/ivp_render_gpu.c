/* ivp_render_gpu.c -- Shader compilation, VAO/VBO management */

#include "ivp_render_gpu.h"
#include <stdio.h>

/* ── Shader source strings ─────────────────────────────────────────────── */

const char *ivp_render_vs_3d_src =
    "#version 330 core\n"
    "layout(location=0) in vec3 aPos;\n"
    "layout(location=1) in vec3 aCol;\n"
    "uniform mat4 uMVP;\n"
    "out vec3 vCol;\n"
    "void main(){\n"
    "  gl_Position = uMVP * vec4(aPos, 1.0);\n"
    "  vCol = aCol;\n"
    "}\n";

const char *ivp_render_fs_3d_src =
    "#version 330 core\n"
    "in vec3 vCol;\n"
    "out vec4 FragColor;\n"
    "void main(){\n"
    "  FragColor = vec4(vCol, 1.0);\n"
    "}\n";

const char *ivp_render_vs_2d_src =
    "#version 330 core\n"
    "layout(location=0) in vec2 aPos;\n"
    "layout(location=1) in vec3 aCol;\n"
    "uniform mat4 uProj;\n"
    "out vec3 vCol;\n"
    "void main(){\n"
    "  gl_Position = uProj * vec4(aPos, 0.0, 1.0);\n"
    "  vCol = aCol;\n"
    "}\n";

const char *ivp_render_fs_2d_src =
    "#version 330 core\n"
    "in vec3 vCol;\n"
    "out vec4 FragColor;\n"
    "void main(){\n"
    "  FragColor = vec4(vCol, 1.0);\n"
    "}\n";

/* ── Shader compile/link helpers ───────────────────────────────────────── */

static GLuint compile_single_shader(GLenum type, const char *src, ivp_result_t *err)
{
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    GLint ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, 512, NULL, log);
        fprintf(stderr, "Shader compile error: %s\n", log);
        glDeleteShader(s);
        *err = IVP_ERR_RENDER_SHADER;
        return 0;
    }
    return s;
}

ivp_result_t ivp_render_shader_compile(ivp_render_shader_t *out,
                                       const char *vs_src,
                                       const char *fs_src,
                                       const char *uniform_name)
{
    ivp_result_t err = IVP_OK;
    GLuint vs = compile_single_shader(GL_VERTEX_SHADER, vs_src, &err);
    if (err != IVP_OK) return err;

    GLuint fs = compile_single_shader(GL_FRAGMENT_SHADER, fs_src, &err);
    if (err != IVP_OK) { glDeleteShader(vs); return err; }

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);

    GLint ok;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    glDeleteShader(vs);
    glDeleteShader(fs);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(prog, 512, NULL, log);
        fprintf(stderr, "Shader link error: %s\n", log);
        glDeleteProgram(prog);
        return IVP_ERR_RENDER_SHADER;
    }

    out->program = prog;
    out->u_mvp   = glGetUniformLocation(prog, uniform_name);
    return IVP_OK;
}

void ivp_render_shader_destroy(ivp_render_shader_t *s)
{
    if (s->program) {
        glDeleteProgram(s->program);
        s->program = 0;
    }
}

/* ── Vertex buffer management ──────────────────────────────────────────── */

ivp_result_t ivp_render_vbuf_create(ivp_render_vbuf_t *out,
                                    int capacity,
                                    int stride,
                                    const ivp_render_attrib_desc_t *attribs,
                                    int attrib_count)
{
    GLuint vao, vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)capacity * (GLsizeiptr)stride,
                 NULL, GL_DYNAMIC_DRAW);

    for (int i = 0; i < attrib_count; i++) {
        glVertexAttribPointer(
            attribs[i].index,
            attribs[i].size,
            GL_FLOAT,
            GL_FALSE,
            stride,
            (void *)(size_t)attribs[i].offset);
        glEnableVertexAttribArray(attribs[i].index);
    }
    glBindVertexArray(0);

    out->vao      = vao;
    out->vbo      = vbo;
    out->capacity = capacity;
    out->stride   = stride;
    return IVP_OK;
}

void ivp_render_vbuf_destroy(ivp_render_vbuf_t *buf)
{
    if (buf->vbo) { glDeleteBuffers(1, &buf->vbo); buf->vbo = 0; }
    if (buf->vao) { glDeleteVertexArrays(1, &buf->vao); buf->vao = 0; }
}

void ivp_render_vbuf_draw_lines(const ivp_render_vbuf_t *buf,
                                const ivp_render_shader_t *shader,
                                const float *matrix,
                                const void *data,
                                int vertex_count)
{
    if (vertex_count <= 0) return;
    glBindVertexArray(buf->vao);
    glBindBuffer(GL_ARRAY_BUFFER, buf->vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    (GLsizeiptr)vertex_count * (GLsizeiptr)buf->stride,
                    data);
    glUseProgram(shader->program);
    glUniformMatrix4fv(shader->u_mvp, 1, GL_FALSE, matrix);
    glDrawArrays(GL_LINES, 0, vertex_count);
    glBindVertexArray(0);
}
