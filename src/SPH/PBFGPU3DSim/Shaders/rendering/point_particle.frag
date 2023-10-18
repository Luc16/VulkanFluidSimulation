#version 450
#extension GL_EXT_debug_printf : enable
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"

layout(binding = 1) uniform sampler2D depthSampler;

layout(location = 0) in vec3 worldPos;
layout(location = 1) in vec4 fragColor;

layout(location = 0) out vec4 outColor;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

const float AMBIENT = 0.05;

const uint RENDER_DEPTH = 0x1;

float LinearizeDepth(float depth)
{
    float n = ubo.zNear;
    float f = ubo.zFar;
    float z = depth;
    return (2.0 * n) / (f + n - z * (f - n));
}

void main() {
    vec3 normal;
    normal.xy = gl_PointCoord*2.0 - vec2(1.0);
    float mag = dot(normal.xy, normal.xy);
    if (mag > 1.0) discard;   // kill pixels outside circle

    normal.z = -sqrt(1.0-mag);

    // calculate lighting
    float diffuse = AMBIENT + max(0.0, dot(normal, ubo.lightDir));

    outColor = fragColor*diffuse;


}