#version 450

layout(push_constant) uniform PushConstants {
    mat4 mvp;
    mat4 normal_mat;
    vec4 light_dir;   /* xyz = direction, w = ambient */
} pc;

layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec3 aCol;

layout(location=0) out vec3 vCol;

void main() {
    gl_Position = pc.mvp * vec4(aPos, 1.0);

    vec3 nrm_ws = normalize(mat3(pc.normal_mat) * aNrm);
    float ndotl = max(dot(nrm_ws, pc.light_dir.xyz), 0.0);
    float ambient = pc.light_dir.w;
    vCol = aCol * (ndotl + ambient);
}
