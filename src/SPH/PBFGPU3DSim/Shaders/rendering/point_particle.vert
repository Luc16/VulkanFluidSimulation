#version 450

layout(location = 0) in vec3 inPosition;

layout(binding = 0) uniform UniformBufferObject {
    mat4 view;
    mat4 proj;
    vec3 cameraPos;
    vec3 lightDir;
    float radius;
    uint renderType;
} ubo;

layout(location = 0) out vec3 worldPos;
layout(location = 1) out vec4 fragColor;

void main() {

    vec4 posWorld = vec4(inPosition, 1.0);
    gl_Position = ubo.proj * ubo.view * posWorld;

    float cameraDist = distance(inPosition, ubo.cameraPos);

    gl_PointSize = 1000*ubo.radius/cameraDist;

    worldPos = inPosition;
    fragColor = vec4(0.2f, 0.6f, 1.0f, 1.0f);
}