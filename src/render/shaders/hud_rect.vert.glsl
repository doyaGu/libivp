#version 450

layout(set=1, binding=0) uniform Uniforms { mat4 proj; } u;
layout(location=0) in vec2 aPos;
layout(location=1) in vec4 aCol;
layout(location=0) out vec4 vCol;

void main() {
    gl_Position = u.proj * vec4(aPos, 0.0, 1.0);
    vCol = aCol;
}
