#version 450

// Per vertex data
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;
layout(location = 3) in vec2 inTexCoord;

// Per instance data
layout(location = 4) in vec3 instancePos;
layout(location = 5) in vec3 instanceCol;
layout(location = 6) in float instanceScale;
layout(location = 7) in float instanceApha;

layout(binding = 0) uniform UniformBufferObject {
    mat4 view;
    mat4 proj;
} ubo;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec3 fragPosWorld;
layout(location = 2) out float fragAlpha;

void main() {
    vec4 posWorld = vec4((inPosition.xyz*instanceScale) + instancePos, 1.0);
    gl_Position = ubo.proj * ubo.view * posWorld;
    fragPosWorld = posWorld.xyz;
    fragAlpha = instanceApha;
    fragColor = instanceCol;
}