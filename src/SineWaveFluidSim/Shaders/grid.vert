#version 450

layout(location = 0) in vec3 inPosition;

layout(binding = 0) uniform UniformBufferObject {
    mat4 viewProj;
    vec3 lightDirection;
    float time;
    float omega;
    float phi;
    float amp;
    uint numSines;
} ubo;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragColor;

void main() {
    float omega = ubo.omega;
    float phi = ubo.phi;
    float amp = ubo.amp;
    float ampSum = 0;

    float height = 0;
    vec2 normal = vec2(0, 0);
    float seed = 0;
    for (uint i = 0; i < ubo.numSines; i++) {
        vec2 direction = vec2(cos(seed), sin(seed));

        float arg = omega * dot(direction, inPosition.xz) + phi*ubo.time;
        height += amp * sin(arg);

        //calculating the normal
        normal -= direction * amp * omega * cos(arg);

        omega *= 1.15;
        ampSum += amp;
        amp *= 0.88;
        seed += 1253.2131;
    }

    vec4 posWorld = vec4(inPosition.x, height, inPosition.z, 1.0);
    gl_Position = ubo.viewProj * posWorld;

    fragNormal = normalize(vec3(normal.x, 1, normal.y));

    fragColor = vec3(0.2f, 0.3f, 0.8f);
}