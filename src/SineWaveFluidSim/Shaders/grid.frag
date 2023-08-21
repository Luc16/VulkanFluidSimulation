#version 450

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec3 fragColor;

layout(binding = 0) uniform UniformBufferObject {
    mat4 viewProj;
    vec3 lightDirection;
    float time;
    float omega;
    float phi;
    float amp;
    uint numSines;
} ubo;

layout(location = 0) out vec4 outColor;

const float AMBIENT_LIGHT = 0.02f;

void main() {
    float diffuse = max(0, dot(fragNormal, ubo.lightDirection));

    float light = diffuse + AMBIENT_LIGHT;
    outColor = vec4(fragColor*light, 1.0);
}