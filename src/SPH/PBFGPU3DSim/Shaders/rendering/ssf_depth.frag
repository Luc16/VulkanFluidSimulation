#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_debug_printf : enable

#include "common.glsl"

layout(location = 0) in vec4 viewPos;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

void main()
{
    vec3 normal;
    normal.xy = gl_PointCoord*2.0 - vec2(1.0);
    float mag = dot(normal.xy, normal.xy);
    if (mag > 1.0) discard;

    normal.z = -sqrt(1 - mag);

    vec4 nviewPos = vec4(viewPos.xyz + normal*ubo.radius, 1.0);
    vec4 nclipPos = ubo.proj*nviewPos;
    float depth = nclipPos.z / nclipPos.w;
//    float q = ubo.zFar/(ubo.zFar - ubo.zNear);
//    debugPrintfEXT("depth = (%f/%f) = %f -> view: %f = converted %f ", nclipPos.z, nclipPos.w, depth, nviewPos.z, -ubo.zNear*q/(depth - q));

    gl_FragDepth = depth;

    // convert from [-1, 1] range to [0, 1]
//    gl_FragDepth = 0.5*(depth + 1);
}