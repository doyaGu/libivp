#version 450

layout(set=1, binding=0) uniform Uniforms {
    mat4 light_vp;
} u;

layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec4 iRow0;
layout(location=3) in vec4 iRow1;
layout(location=4) in vec4 iRow2;
layout(location=5) in vec4 iRow3;
layout(location=6) in vec4 iCol;

void main() {
    mat4 model = mat4(iRow0, iRow1, iRow2, iRow3);
    gl_Position = u.light_vp * (model * vec4(aPos, 1.0));
}
