#ifndef EMBREE_PATH_TRACER_H
#define EMBREE_PATH_TRACER_H

#if defined(ISPC) && !defined(ISPC_STD_C99_DATATYPES)
#define ISPC_STD_C99_DATATYPES
typedef unsigned int32 uint32_t;
typedef unsigned int64 uint64_t;
typedef int32 int32_t;
typedef int64 int64_t;
#endif

struct rgb_s {
    float r, g, b;
};

struct float3_s {
    float x, y, z;
};

struct float2_s {
    float x, y;
};

struct image_s {
    uint32_t* pixels;
    uint32_t width;
    uint32_t height;
};

struct camera_s {
    struct float3_s eye;
    struct float3_s dir;
    struct float3_s right;
    struct float3_s up;
    float w, h;
};

struct light_s {
    struct float3_s v0;
    struct float3_s v1;
    struct float3_s v2;
    struct float3_s n;
    float inv_area;
    struct rgb_s color;
};

struct material_s {
    struct rgb_s ke;
    struct rgb_s kd;
    struct rgb_s ks;
    struct rgb_s tf;
    float ns;
    float ni;
    int32_t map_kd;
    int32_t map_ks;
    int32_t light;
    uint32_t illum;
};

struct counters_s {
    int64_t total_rays;
    int64_t primary;
    int64_t shadow;
    int64_t shade;
    int64_t bounces;
    int64_t total;
};

struct scene_s {
    RTCScene scene;

    float* film;
    uint32_t film_width;
    uint32_t film_height;

    uint32_t*          indices;
    struct float3_s*   vertices;
    struct float3_s*   normals;
    struct float2_s*   texcoords;
    struct camera_s*   camera;
    struct image_s*    images;
    struct material_s* materials;
    struct light_s*    lights;

    float pdf_lightpick;
    uint32_t num_lights;
};

#endif // EMBREE_PATH_TRACER_H
