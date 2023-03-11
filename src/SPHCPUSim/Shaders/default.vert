#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(binding = 0) uniform UniformBufferObject {
    mat4 view;
    mat4 proj;
} ubo;

layout(location = 0) out vec3 fragColor;

void main() {

    gl_PointSize = 14.0;
    vec4 posWorld = vec4(inPosition, 1.0);
    gl_Position = ubo.proj * ubo.view * posWorld;
    fragColor = inColor.rgb;
}