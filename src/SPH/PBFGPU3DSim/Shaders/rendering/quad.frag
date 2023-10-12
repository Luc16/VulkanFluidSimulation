#version 450

layout (binding = 1) uniform sampler2D samplerColor;

layout (location = 0) in vec2 inUV;

layout (location = 0) out vec4 outFragColor;

layout(binding = 0) uniform UniformBufferObject {
    mat4 view;
    mat4 proj;
    vec3 cameraPos;
    vec3 lightDir;
    float radius;
    uint renderType;
    float zNear;
    float zFar;
} ubo;

float LinearizeDepth(float depth)
{
    float n = ubo.zNear;
    float f = ubo.zFar;
    float z = depth;
    return (2.0 * n) / (f + n - z * (f - n));
}

void main()
{
    float depth = texture(samplerColor, inUV).r;
    outFragColor = vec4(vec3(1.0-LinearizeDepth(depth)), 1.0);
}