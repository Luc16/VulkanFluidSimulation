#version 450

layout(location = 0) in vec2 inPosition;

layout(binding = 0) uniform UniformBufferObject {
    mat4 view;
    mat4 proj;
    float radius;
} ubo;

void main() {

    gl_PointSize = ubo.radius;
    vec4 posWorld = vec4(inPosition, 0.0, 1.0);
    gl_Position = ubo.proj * ubo.view * posWorld;
}