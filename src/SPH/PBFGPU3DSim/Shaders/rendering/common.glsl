#ifndef COMMON_SHADER_H
#define COMMON_SHADER_H

struct UniformBufferObject {
    mat4 view;
    mat4 proj;
    vec3 lightDir;
    float radius;
    uint screenHeight;
    float tanHalfFov;
    uint renderType;
    float zNear;
    float zFar;
};

#endif //COMMON_SHADER_H