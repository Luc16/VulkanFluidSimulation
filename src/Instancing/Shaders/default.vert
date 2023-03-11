#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;
layout(location = 3) in vec2 inTexCoord;

layout(binding = 0) uniform UniformBufferObject {
    mat4 view;
    mat4 proj;
    vec3 lightDirection;
} computeUniformBuffer;

layout(push_constant) uniform Push {
    mat4 model;
} push;


layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec3 fragPosWorld;
layout(location = 2) out vec3 fragNormalWorld;

void main() {
    vec4 posWorld = push.model * vec4(inPosition, 1.0);
    gl_Position = ubo.proj * ubo.view * posWorld;
    fragPosWorld = posWorld.xyz;
    fragNormalWorld = normalize(mat3(push.model) * inNormal);
    fragColor = inColor;
}