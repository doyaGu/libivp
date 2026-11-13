#version 300 es
/* 2D HUD lines in window pixels. */
layout(std140) uniform U { mat4 proj; };
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec3 aCol;
out vec3 vCol;
void main() {
    gl_Position = proj * vec4(aPos, 0.0, 1.0);
    vCol = aCol;
}
