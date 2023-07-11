#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(binding = 0) uniform UniformBufferObject {
    mat4 viewProj;
    vec3 cameraPos;
    vec3 lightDirection;
} ubo;

layout(location = 0) out vec4 fragColor;

void main() {

    vec4 posWorld = vec4(inPosition, 1.0);
    gl_Position = ubo.viewProj * posWorld;

    float cameraDist = distance(inPosition.xyz, ubo.cameraPos);

    gl_PointSize = 1200/cameraDist;

    fragColor = inColor;
}