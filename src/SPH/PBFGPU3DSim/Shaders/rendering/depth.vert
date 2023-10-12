#version 450

layout (location = 0) in vec3 inPos;

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

out gl_PerVertex
{
    vec4 gl_Position;
    float gl_PointSize;
};


void main()
{
    float cameraDist = distance(inPos, ubo.cameraPos);
    gl_PointSize = 1000*ubo.radius/cameraDist;

    vec4 posWorld = vec4(inPos, 1.0);
    gl_Position = ubo.proj * ubo.view * posWorld;
}