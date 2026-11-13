#version 300 es
/* Instanced lit solids with a 3x3 PCF shadow (port of tri3d_inst.frag.msl). */
precision highp float;
precision highp sampler2DShadow;
uniform sampler2DShadow shadow_map;
in vec3 vAmb;
in vec3 vDif;
in vec4 vSc;
in float vAlpha;
out vec4 fragColor;

const float SHADOW_TEXEL = 1.0 / 2048.0;
const float SHADOW_BIAS  = 0.0008;

void main() {
    float vis = 1.0;
    if (vSc.w > 0.5 && vSc.z <= 1.0 &&
        vSc.x >= 0.0 && vSc.x <= 1.0 && vSc.y >= 0.0 && vSc.y <= 1.0) {
        /* light_vp gives depth in [0,1]; GL maps clip depth [-1,1] to the
         * depth buffer, so the map holds 0.5 * z + 0.5 */
        float ref = 0.5 * (vSc.z - SHADOW_BIAS) + 0.5;
        float sum = 0.0;
        for (int y = -1; y <= 1; y++)
            for (int x = -1; x <= 1; x++)
                sum += texture(shadow_map,
                               vec3(vSc.xy + vec2(float(x), float(y)) * SHADOW_TEXEL, ref));
        vis = sum / 9.0;
    }
    fragColor = vec4(vAmb + vDif * vis, vAlpha);
}
