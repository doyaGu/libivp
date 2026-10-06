#version 450

layout(push_constant) uniform PushConstants {
    mat4 vp;
    vec4 light_dir;   /* xyz = direction, w = ambient */
} pc;

/* Per-vertex (slot 0) */
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;

/* Per-instance (slot 1) */
layout(location=2) in vec4 iRow0;
layout(location=3) in vec4 iRow1;
layout(location=4) in vec4 iRow2;
layout(location=5) in vec4 iRow3;
layout(location=6) in vec3 iCol;

layout(location=0) out vec3 vCol;

void main() {
    mat4 model = mat4(iRow0, iRow1, iRow2, iRow3);
    vec4 world_pos = model * vec4(aPos, 1.0);
    gl_Position = pc.vp * world_pos;

    /* cofactor matrix = inverse transpose up to a positive scale, so normals
     * stay correct under non-uniform scaling (cylinders, cones, boxes) */
    vec3 c0 = model[0].xyz, c1 = model[1].xyz, c2 = model[2].xyz;
    mat3 nm = mat3(cross(c1, c2), cross(c2, c0), cross(c0, c1));
    vec3 nrm_ws = normalize(nm * aNrm);
    float ndotl = max(dot(nrm_ws, pc.light_dir.xyz), 0.0);
    float ambient = pc.light_dir.w * (0.75 - 0.25 * nrm_ws.y);  /* sky (-Y) / ground */
    vCol = iCol * (0.7 * ndotl + ambient);
}
