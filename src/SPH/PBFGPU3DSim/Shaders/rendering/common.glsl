#ifndef COMMON_SHADER_H
#define COMMON_SHADER_H

struct UniformBufferObject {
    mat4 view;
    mat4 proj;
    mat4 inverseView;
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
    vec3 planeSize;
    float restDens;
    float transparency;
};

#endif //COMMON_SHADER_H