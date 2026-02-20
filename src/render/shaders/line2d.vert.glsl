#version 450
layout(push_constant) uniform PushConstants { mat4 proj; } pc;
layout(location=0) in vec2 aPos;
layout(location=1) in vec3 aCol;
layout(location=0) out vec3 vCol;
void main() {
    gl_Position = pc.proj * vec4(aPos, 0.0, 1.0);
    vCol = aCol;
}
