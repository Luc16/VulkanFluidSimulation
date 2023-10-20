#ifndef COMMON_SHADER_H
#define COMMON_SHADER_H

struct UniformBufferObject {
    mat4 view;
    mat4 proj;
    vec3 lightDir;
    float radius;
    float screenHeight;
    float screenWidth;
    float tanHalfFov;
    uint renderType;
    float zNear;
    float zFar;
    uint blurMode;
    int filterRadius;
    float blurScale;
    float blurDepthFalloff;
};

#endif //COMMON_SHADER_H