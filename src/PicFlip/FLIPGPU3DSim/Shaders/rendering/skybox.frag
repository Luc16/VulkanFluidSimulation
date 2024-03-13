#version 450
#extension GL_EXT_debug_printf : enable
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"

layout(binding = 1) uniform samplerCube samplerCubeMap;

layout(location = 0) in vec3 fragUVW;

layout(location = 0) out vec4 outColor;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

void main() {
    outColor = texture(samplerCubeMap, fragUVW);


}