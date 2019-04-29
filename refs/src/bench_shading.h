#ifndef BENCH_SHADING_H
#define BENCH_SHADING_H

#if defined(ISPC) && !defined(ISPC_STD_C99_DATATYPES)
#define ISPC_STD_C99_DATATYPES
typedef unsigned int32 uint32_t;
typedef unsigned int64 uint64_t;
typedef int32 int32_t;
typedef int64 int64_t;
#endif

struct stream_s {
    uint32_t* rnd;
    int32_t* depth;
    int32_t* geom_id;
    int32_t* prim_id;
    float* mis;
    float* contrib_r;
    float* contrib_g;
    float* contrib_b;
    float* org_x;
    float* org_y;
    float* org_z;
    float* dir_x;
    float* dir_y;
    float* dir_z;
    float* tmin;
    float* tmax;
    float* t;
    float* u;
    float* v;
};

#endif // BENCH_SHADING
