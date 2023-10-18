#version 450
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

layout (location = 0) out vec4 outColor;

void main()
{
    vec3 normal;
    normal.xy = gl_PointCoord*2.0 - vec2(1.0);
    float mag = dot(normal.xy, normal.xy);
    if (mag > 1.0) discard;

    float z = sqrt(1 - mag);
    float thickness = (ubo.renderType == 2) ? z/50 : 2*ubo.radius*z;

//    outColor = vec4(2*ubo.radius*z);
    outColor = vec4(thickness);
}