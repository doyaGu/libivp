#version 450

layout(set=2, binding=0) uniform sampler2DShadow uShadow;

layout(location=0) in vec3 vAmb;
layout(location=1) in vec3 vDif;
layout(location=2) in vec4 vSc;
layout(location=3) in float vAlpha;
layout(location=0) out vec4 FragColor;

const float SHADOW_TEXEL = 1.0 / 2048.0;
const float SHADOW_BIAS  = 0.0008;

void main() {
    float vis = 1.0;
    if (vSc.w > 0.5 && vSc.z <= 1.0 &&
        vSc.x >= 0.0 && vSc.x <= 1.0 && vSc.y >= 0.0 && vSc.y <= 1.0) {
        float ref = vSc.z - SHADOW_BIAS;
        float sum = 0.0;
        for (int y = -1; y <= 1; y++)
            for (int x = -1; x <= 1; x++)
                sum += texture(uShadow, vec3(vSc.xy + vec2(x, y) * SHADOW_TEXEL, ref));
        vis = sum / 9.0;
    }
    FragColor = vec4(vAmb + vDif * vis, vAlpha);
}
