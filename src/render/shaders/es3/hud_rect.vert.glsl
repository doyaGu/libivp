#version 300 es
/* Filled 2D rectangles (triangle list) with alpha: HUD panels and text. */
layout(std140) uniform U { mat4 proj; };
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec4 aCol;
out vec4 vCol;
void main() {
    gl_Position = proj * vec4(aPos, 0.0, 1.0);
    vCol = aCol;
}
