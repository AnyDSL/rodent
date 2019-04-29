#ifndef OPTIX_PATH_TRACER_H
#define OPTIX_PATH_TRACER_H

struct Material {
    float3 kd;
    float3 ks;
    float3 ke;
    float3 tf;
    int map_kd;
    int map_ks;
    float ns;
    float ni;
    uint illum;
};

struct Light {
    float3 v0;
    float3 v1;
    float3 v2;
    float3 normal;
    float inv_area;
    float3 intensity;
};

#endif // OPTIX_PATH_TRACER_H
