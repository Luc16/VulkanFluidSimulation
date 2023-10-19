#version 450
#extension GL_EXT_debug_printf : enable
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"

layout(binding = 1) uniform sampler2D depthSampler;

layout(location = 0) in vec4 fragColor;

layout(location = 0) out vec4 outColor;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

const float AMBIENT = 0.05;

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