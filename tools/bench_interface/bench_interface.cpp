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

int main(int argc, char** argv) {
#if defined(__x86_64__) || defined(__amd64__) || defined(_M_X64)
    _mm_setcsr(_mm_getcsr() | (_MM_FLUSH_ZERO_ON | _MM_DENORMALS_ZERO_ON));
#endif

    std::vector<float3> vertices;
    std::vector<float3> normals;
    std::vector<float2> texcoords;
    std::vector<int32_t> indices;

    // Create a quad
    vertices.emplace_back(-1.0f,  1.0f, 0.0f);
    vertices.emplace_back(-1.0f, -1.0f, 0.0f);
    vertices.emplace_back( 1.0f, -1.0f, 0.0f);
    vertices.emplace_back( 1.0f,  1.0f, 0.0f);

    normals.emplace_back(0.0f, 0.0f, 1.0f);
    normals.emplace_back(0.0f, 0.0f, 1.0f);
    normals.emplace_back(0.0f, 0.0f, 1.0f);
    normals.emplace_back(0.0f, 0.0f, 1.0f);

    texcoords.emplace_back(-1.0f,  1.0f);
    texcoords.emplace_back(-1.0f, -1.0f);
    texcoords.emplace_back( 1.0f, -1.0f);
    texcoords.emplace_back( 1.0f,  1.0f);

    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(2);
    indices.push_back(-1);

    indices.push_back(2);
    indices.push_back(3);
    indices.push_back(0);
    indices.push_back(-1);

    int width  = 1024;
    int height = 1024;
    std::unique_ptr<Color[]> pixels_kd(new Color[width * height]);
    std::unique_ptr<Color[]> pixels_ks(new Color[width * height]);
    std::unique_ptr<Color[]> pixels_ns(new Color[width * height]);

    std::fill(pixels_kd.get(), pixels_kd.get() + width * height, Color { 0.1f, 0.2f, 0.3f });
    std::fill(pixels_ks.get(), pixels_ks.get() + width * height, Color { 1.0f, 0.5f, 0.1f });
    std::fill(pixels_ns.get(), pixels_ns.get() + width * height, Color { 0.1f, 0.5f, 1.0f });

    Tex tex_kd {
        pixels_kd.get(),
        Color { 0.0f, 0.0f, 0.0f },
        0,
        1,
        width,
        height
    };

    Tex tex_ks {
        pixels_ks.get(),
        Color { 0.5f, 1.0f, 0.2f },
        2,
        0,
        width,
        height
    };

    Tex tex_ns {
        pixels_ns.get(),
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

    size_t N = 10000;
    std::unique_ptr<Vec3[]> in_dirs(new Vec3[N]);
    std::unique_ptr<Vec3[]> out_dirs(new Vec3[N]);
    std::unique_ptr<Color[]> colors(new Color[N]);
    std::unique_ptr<TriHit[]> tri_hits(new TriHit[N]);

    uint32_t seed = 42;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<float> rnd(0.0f, 1.0f);
    for (size_t i = 0; i < N; ++i) {
        tri_hits[i].id = i % 2;
        tri_hits[i].uv.x = rnd(gen);
        tri_hits[i].uv.y = rnd(gen);

        auto in  = normalize(float3(rnd(gen), rnd(gen), rnd(gen)));
        auto out = normalize(float3(rnd(gen), rnd(gen), rnd(gen)));

        in_dirs[i].x = in.x;
        in_dirs[i].y = in.y;
        in_dirs[i].z = in.z;

        out_dirs[i].x = out.x;
        out_dirs[i].y = out.y;
        out_dirs[i].z = out.z;
    }

    size_t iters = 1000;
    std::vector<double> times;
    for (size_t i = 0; i < iters; ++i) {
        auto t0 = std::chrono::high_resolution_clock::now();
        cpu_bench_interface(&mesh, tri_hits.get(), in_dirs.get(), out_dirs.get(), colors.get(), N);
        auto t1 = std::chrono::high_resolution_clock::now();
        times.push_back(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
    }
    std::sort(times.begin(), times.end());
    std::cout << N / times[iters/2] << " Mrays/s" << std::endl;

    return 0;
}
