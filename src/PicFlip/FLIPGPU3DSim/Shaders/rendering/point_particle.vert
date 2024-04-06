#version 450
#extension GL_EXT_debug_printf : enable
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"
#include "../simulation/common.glsl"

layout(location = 0) in vec3 inPosition;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

layout(std430, binding = 1) readonly buffer SdfSSBO {
    float sdf[ ];
};

layout(location = 0) out vec4 fragColor;

void main() {
    vec4 posWorld = vec4(inPosition, 1.0);
    vec4 viewPos = ubo.view * posWorld;

    gl_PointSize = ubo.screenHeight*ubo.radius/(viewPos.z*ubo.tanHalfFov);
    gl_Position = ubo.proj * viewPos;


    IdxWeight iw = particleIdxWeight(inPosition, vec3(0.0), 5);
    float w[8] = float[](
        (1.0 - iw.weight.x)*(1.0 - iw.weight.y)*(1.0 - iw.weight.z),
        (iw.weight.x)*(1.0 - iw.weight.y)*(1.0 - iw.weight.z),
        (1.0 - iw.weight.x)*(iw.weight.y)*(1.0 - iw.weight.z),
        (iw.weight.x)*(iw.weight.y)*(1.0 - iw.weight.z),
        (1.0 - iw.weight.x)*(1.0 - iw.weight.y)*(iw.weight.z),
        (iw.weight.x)*(1.0 - iw.weight.y)*(iw.weight.z),
        (1.0 - iw.weight.x)*(iw.weight.y)*(iw.weight.z),
        (iw.weight.x)*(iw.weight.y)*(iw.weight.z)
    );
    float pSdf =w[0]*sdf[gPosToIdx(iw.gIdx, ivec3(35, 35, 35))] +
                w[1]*sdf[gPosToIdx(iw.gIdx + ivec3(1, 0, 0), ivec3(35, 35, 35))] +
                w[2]*sdf[gPosToIdx(iw.gIdx + ivec3(0, 1, 0), ivec3(35, 35, 35))] +
                w[3]*sdf[gPosToIdx(iw.gIdx + ivec3(1, 1, 0), ivec3(35, 35, 35))] +
                w[4]*sdf[gPosToIdx(iw.gIdx + ivec3(0, 0, 1), ivec3(35, 35, 35))] +
                w[5]*sdf[gPosToIdx(iw.gIdx + ivec3(1, 0, 1), ivec3(35, 35, 35))] +
                w[6]*sdf[gPosToIdx(iw.gIdx + ivec3(0, 1, 1), ivec3(35, 35, 35))] +
                w[7]*sdf[gPosToIdx(iw.gIdx + ivec3(1, 1, 1), ivec3(35, 35, 35))];
    if (pSdf > 0) {
        fragColor = vec4(0.2f, 0.6f, 1.0f, 1);
    }
    else {
        fragColor = vec4(0.0f, 1.0f, 0.0f, 1.0f);
    }
//    fragColor = vec4(0.2f, 0.6f, 1.0f, 1);
}