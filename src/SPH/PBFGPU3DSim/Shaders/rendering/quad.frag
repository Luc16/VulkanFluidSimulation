#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_debug_printf : enable


#include "common.glsl"

layout (location = 0) in vec2 inUV;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

layout (binding = 1) uniform sampler2D samplerDepth;
layout (binding = 2) uniform sampler2D samplerThick;


layout (location = 0) out vec4 outFragColor;

float LinearizeDepth(float depth)
{
    float n = ubo.zNear;
    float f = ubo.zFar;
    float z = depth;
    return (2.0 * n) / (f + n - z * (f - n));
}

void main()
{
    if (ubo.renderType == 1) {
        float depth = texture(samplerDepth, inUV).r;
        if (depth == 1) {
            discard;
        }
        outFragColor = vec4(vec3(LinearizeDepth(depth)), 1.0);
    } else if (ubo.renderType == 2) {
        vec4 thickColor = vec4(texture(samplerThick, inUV).r);
        if (thickColor.r == 1) {
            discard;
        }
        outFragColor = vec4(vec3(thickColor), 1.0 - thickColor.r);
    }
}