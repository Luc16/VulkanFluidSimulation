#version 450
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"

layout(binding = 1) uniform sampler2D texSampler;

layout(location = 0) in vec2 fragTexCoord;
layout(location = 1) in vec3 fragPosWorld;
layout(location = 2) in vec3 fragNormalWorld;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

layout(location = 0) out vec4 outColor;

const float AMBIENT = 0.05;

void main() {
    float lightIntensity = AMBIENT + max(dot(normalize(fragNormalWorld), normalize(-ubo.lightDir)), 0);

    outColor = vec4(vec3(texture(texSampler, fragTexCoord))*lightIntensity, 1.0);
}