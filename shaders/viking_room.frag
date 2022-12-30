#version 450

layout(binding = 1) uniform sampler2D texSampler;

layout(location = 0) in vec2 fragTexCoord;
layout(location = 1) in vec3 fragPosWorld;
layout(location = 2) in vec3 fragNormalWorld;

layout(binding = 0) uniform UniformBufferObject {
    mat4 view;
    mat4 proj;
    vec3 lightDirection;
} ubo;

layout(location = 0) out vec4 outColor;

const float AMBIENT = 0.05;

void main() {
    float lightIntensity = AMBIENT + max(dot(normalize(fragNormalWorld), normalize(ubo.lightDirection)), 0);

    outColor = lightIntensity*texture(texSampler, fragTexCoord);
}