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
layout (binding = 3) uniform sampler2D samplerNormals;
layout (binding = 4) uniform sampler2D samplerSmoothedDepth;


layout (location = 0) out vec4 outFragColor;

float linearizeDepth(float depth)
{
    float n = ubo.zNear;
    float f = ubo.zFar;
    float z = depth;
    return (2.0 * n) / (f + n - z * (f - n));
//    return  n + z * (f - n);
}

void main()
{
    float depth = texture(samplerDepth, inUV).r;
    if (depth == 1) {
        discard;
    }
    gl_FragDepth = -depth;
    switch(ubo.renderType) {
        case 1:
            outFragColor = vec4(vec3(linearizeDepth(depth)), 1.0);
        break;
        case 2:
            vec4 thickColor = vec4(texture(samplerThick, inUV).r);
            outFragColor = vec4(vec3(thickColor), 1.0 - thickColor.r);
        break;
        case 3:
            vec4 normal = texture(samplerNormals, inUV);
            outFragColor = vec4(0.5*normal.rgba + vec4(0.5));
        break;
        case 4:
            float smoothDepth = texture(samplerSmoothedDepth, inUV).r;
            outFragColor = vec4(vec3(linearizeDepth(smoothDepth)), 1.0);
        break;
    }
}