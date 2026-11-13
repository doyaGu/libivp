#version 300 es
/* Instanced lit solids (port of tri3d_inst.vert.msl). */
layout(std140) uniform U {
    mat4 vp;
    vec4 light_dir;   /* xyz = direction towards the light, w = ambient */
    mat4 light_vp;    /* world -> light clip space, depth in [0,1] */
    vec4 shadow;      /* x = 1 when the shadow map is valid */
};
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNrm;
layout(location = 2) in vec4 iRow0;
layout(location = 3) in vec4 iRow1;
layout(location = 4) in vec4 iRow2;
layout(location = 5) in vec4 iRow3;
layout(location = 6) in vec4 iCol;
out vec3 vAmb;     /* ambient part of the colour */
out vec3 vDif;     /* diffuse part, attenuated by the shadow */
out vec4 vSc;      /* xy = shadow map uv, z = light space depth, w = shadows on */
out float vAlpha;
void main() {
    mat4 model = mat4(iRow0, iRow1, iRow2, iRow3);
    vec4 world_pos = model * vec4(aPos, 1.0);
    gl_Position = vp * world_pos;

    /* cofactor matrix = inverse transpose up to a positive scale, so normals
     * stay correct under non-uniform scaling (cylinders, cones, boxes) */
    vec3 c0 = model[0].xyz, c1 = model[1].xyz, c2 = model[2].xyz;
    mat3 nm = mat3(cross(c1, c2), cross(c2, c0), cross(c0, c1));
    vec3 nrm_ws = normalize(nm * aNrm);
    float ndotl = max(dot(nrm_ws, light_dir.xyz), 0.0);
    float ambient = light_dir.w * (0.75 - 0.25 * nrm_ws.y);  /* sky (-Y) / ground */
    vAmb = iCol.rgb * ambient;
    vDif = iCol.rgb * (0.7 * ndotl);
    vAlpha = iCol.a;

    /* shadow lookup position, pushed along the normal against acne; GL
     * textures start at the bottom row, so v is not flipped as in Metal */
    vec4 lp = light_vp * vec4(world_pos.xyz + nrm_ws * 0.04, 1.0);
    vSc = vec4(lp.x * 0.5 + 0.5, lp.y * 0.5 + 0.5, lp.z, shadow.x);
}
