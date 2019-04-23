#include <iostream>
#include <chrono>
#include <cstdint>
#include <vector>
#include <random>
#include <limits>
#include <memory>
#include <algorithm>

#if defined(__x86_64__) || defined(__amd64__) || defined(_M_X64)
#include <x86intrin.h>
#endif

#include <anydsl_runtime.hpp>

#include "float2.h"
#include "float3.h"
#include "interface.h"

#define BENCH_CUDA

#ifdef BENCH_CUDA
// Some external functions will refer to those symbols when compiled on the GPU
extern "C" float __nv_fminf(float a, float b) { return fminf(a, b); }
extern "C" float __nv_fmaxf(float a, float b) { return fmaxf(a, b); }
extern "C" float __nv_sqrtf(float x) { return sqrtf(x); }
extern "C" float __nv_floorf(float x) { return floorf(x); }
#endif

template <typename T>
void fill(anydsl::Array<T>& array, T val) {
    anydsl::Array<T> copy(array.size());
    std::fill(copy.begin(), copy.end(), val);
    anydsl::copy(copy, array);
}

template <typename T>
void set(anydsl::Array<T>& array, const std::vector<T>& vals) {
    anydsl::Array<T> host_array(vals.size());
    std::copy(vals.begin(), vals.end(), host_array.begin());
    copy(host_array, array);
}

int main(int argc, char** argv) {
#if defined(__x86_64__) || defined(__amd64__) || defined(_M_X64)
    _mm_setcsr(_mm_getcsr() | (_MM_FLUSH_ZERO_ON | _MM_DENORMALS_ZERO_ON));
#endif

#ifdef BENCH_CUDA
    auto plt = anydsl::Platform::Cuda;
    auto dev = anydsl::Device(0);
#else
    auto plt = anydsl::Platform::Host;
    auto dev = anydsl::Device(0);
#endif

    auto num_vertices = 4;
    auto num_triangles = 2;

    anydsl::Array<float3>  vertices (plt, dev, num_vertices);
    anydsl::Array<float3>  normals  (plt, dev, num_vertices);
    anydsl::Array<float2>  texcoords(plt, dev, num_vertices);
    anydsl::Array<int32_t> indices  (plt, dev, num_triangles * 4);

    // Create a quad
    set(vertices, {
        float3(-1.0f,  1.0f, 0.0f),
        float3(-1.0f, -1.0f, 0.0f),
        float3( 1.0f, -1.0f, 0.0f),
        float3( 1.0f,  1.0f, 0.0f)
    });
    set(normals, {
        float3(0.0f, 0.0f, 1.0f),
        float3(0.0f, 0.0f, 1.0f),
        float3(0.0f, 0.0f, 1.0f),
        float3(0.0f, 0.0f, 1.0f)
    });
    set(texcoords, {
        float2(-1.0f,  1.0f),
        float2(-1.0f, -1.0f),
        float2( 1.0f, -1.0f),
        float2( 1.0f,  1.0f)
    });
    set(indices, {
        0,
        1,
        2,
        -1,

        2,
        3,
        0,
        -1
    });

    int width  = 1024;
    int height = 1024;
    anydsl::Array<Color> pixels_kd(plt, dev, width * height);
    anydsl::Array<Color> pixels_ks(plt, dev, width * height);
    anydsl::Array<Color> pixels_ns(plt, dev, width * height);

    fill(pixels_kd, Color { 0.1f, 0.2f, 0.3f });
    fill(pixels_ks, Color { 1.0f, 0.5f, 0.1f });
    fill(pixels_ns, Color { 0.1f, 0.5f, 1.0f });

    Tex tex_kd {
        pixels_kd.data(),
        Color { 0.0f, 0.0f, 0.0f },
        0,
        1,
        width,
        height
    };

    Tex tex_ks {
        pixels_ks.data(),
        Color { 0.5f, 1.0f, 0.2f },
        2,
        0,
        width,
        height
    };

    Tex tex_ns {
        pixels_ns.data(),
        Color { 0.0f, 0.0f, 0.0f },
        1,
        1,
        width,
        height
    };

    ShadedMesh mesh {
        reinterpret_cast<Vec3*>(vertices.data()),
        reinterpret_cast<unsigned int*>(indices.data()),
        reinterpret_cast<Vec3*>(normals.data()),
        reinterpret_cast<Vec2*>(texcoords.data()),
        tex_kd,
        tex_ks,
        tex_ns
    };

    size_t N = 1024*1024;
    anydsl::Array<Vec3> host_in_dirs(N);
    anydsl::Array<Vec3> host_out_dirs(N);
    anydsl::Array<TriHit> host_tri_hits(N);

    uint32_t seed = 42;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<float> rnd(0.0f, 1.0f);
    for (size_t i = 0; i < N; ++i) {
        host_tri_hits[i].id = i % 2;
        host_tri_hits[i].uv.x = rnd(gen);
        host_tri_hits[i].uv.y = rnd(gen);

        auto in  = normalize(float3(rnd(gen), rnd(gen), rnd(gen)));
        auto out = normalize(float3(rnd(gen), rnd(gen), rnd(gen)));

        host_in_dirs[i].x = in.x;
        host_in_dirs[i].y = in.y;
        host_in_dirs[i].z = in.z;

        host_out_dirs[i].x = out.x;
        host_out_dirs[i].y = out.y;
        host_out_dirs[i].z = out.z;
    }
    anydsl::Array<Color>  colors  (plt, dev, N);
    anydsl::Array<Vec3>   in_dirs (plt, dev, N);
    anydsl::Array<Vec3>   out_dirs(plt, dev, N);
    anydsl::Array<TriHit> tri_hits(plt, dev, N);
    anydsl::copy(host_in_dirs,  in_dirs);
    anydsl::copy(host_out_dirs, out_dirs);
    anydsl::copy(host_tri_hits, tri_hits);

#ifdef BENCH_CUDA
    size_t iters = 1000;
#else
    size_t iters = 100;
#endif
    std::vector<double> times;
    for (size_t i = 0; i < iters; ++i) {
        auto t0 = std::chrono::high_resolution_clock::now();
        bench_interface(&mesh, tri_hits.data(), in_dirs.data(), out_dirs.data(), colors.data(), N);
        auto t1 = std::chrono::high_resolution_clock::now();
        times.push_back(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
    }
    std::sort(times.begin(), times.end());
    std::cout << N / times[iters/2] << " Mrays/s" << std::endl;

    return 0;
}
