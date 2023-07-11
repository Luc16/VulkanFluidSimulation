#version 450

layout(location = 0) in vec4 fragColor;

layout(location = 0) out vec4 outColor;

layout(binding = 0) uniform UniformBufferObject {
    mat4 viewProj;
    vec3 cameraPos;
    vec3 lightDirection;
} ubo;

void main() {
    vec3 normal;
    normal.xy = gl_PointCoord*2.0 - vec2(1.0);
    float mag = dot(normal.xy, normal.xy);
    if (mag > 1.0) discard;

    normal.z = sqrt(1.0-mag);

    outColor = fragColor;
}