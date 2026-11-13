#version 300 es
/* 3D lines: world-space position and colour. */
layout(std140) uniform U { mat4 mvp; };
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aCol;
out vec3 vCol;
void main() {
    gl_Position = mvp * vec4(aPos, 1.0);
    vCol = aCol;
}
