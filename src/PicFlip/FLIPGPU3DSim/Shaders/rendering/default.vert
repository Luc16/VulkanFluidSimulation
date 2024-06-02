#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_debug_printf : enable

#include "common.glsl"

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;
layout(location = 3) in vec2 inTexCoord;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

layout(push_constant) uniform Push {
    mat4 model;
} push;


layout(location = 0) out vec2 fragTexCoord;
layout(location = 1) out vec3 fragPosWorld;
layout(location = 2) out vec3 fragNormalWorld;

void main() {
    vec4 posWorld = push.model * vec4(inPosition, 1.0);
    gl_Position = ubo.proj * ubo.view * posWorld;
    fragPosWorld = posWorld.xyz;
    fragNormalWorld = normalize(mat3(push.model) * inNormal);
    fragTexCoord = inTexCoord;
}