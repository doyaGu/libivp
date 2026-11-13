#version 450
layout(set=1, binding=0) uniform Uniforms { mat4 mvp; } u;
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aCol;
layout(location=0) out vec3 vCol;
void main() {
    gl_Position = u.mvp * vec4(aPos, 1.0);
    vCol = aCol;
}
