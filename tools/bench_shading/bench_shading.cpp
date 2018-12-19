#include <iostream>
#include <cstdint>
#include <vector>
#include <random>
#include <limits>
#include <chrono>
#include <algorithm>

#include <anydsl_runtime.hpp>

#include "float2.h"
#include "float3.h"
#include "shading.h"

inline void get_ray_stream(RayStream& rays, float* ptr, size_t capacity) {
    rays.id = (int*)ptr + 0 * capacity;
    rays.org_x = ptr + 1 * capacity;
    rays.org_y = ptr + 2 * capacity;
    rays.org_z = ptr + 3 * capacity;
    rays.dir_x = ptr + 4 * capacity;
    rays.dir_y = ptr + 5 * capacity;
    rays.dir_z = ptr + 6 * capacity;
    rays.tmin  = ptr + 7 * capacity;
    rays.tmax  = ptr + 8 * capacity;
}

inline void get_primary_stream(PrimaryStream& primary, float* ptr, size_t capacity) {
    get_ray_stream(primary.rays, ptr, capacity);
    primary.geom_id   = (int*)ptr + 9 * capacity;
    primary.prim_id   = (int*)ptr + 10 * capacity;
    primary.t         = ptr + 11 * capacity;
    primary.u         = ptr + 12 * capacity;
    primary.v         = ptr + 13 * capacity;
    primary.rnd       = (unsigned int*)ptr + 14 * capacity;
    primary.mis       = ptr + 15 * capacity;
    primary.contrib_r = ptr + 16 * capacity;
    primary.contrib_g = ptr + 17 * capacity;
    primary.contrib_b = ptr + 18 * capacity;
    primary.depth     = (int*)ptr + 19 * capacity;
    primary.size = 0;
}

int main(int argc, char** argv) {
    std::vector<float3> vertices;
    std::vector<float3> normals;
    std::vector<float3> face_normals;
    std::vector<float2> texcoords;
    std::vector<int32_t> indices;
    std::vector<uint32_t> pixels;

    std::vector<int32_t> begins;
    std::vector<int32_t> ends;

    // Create a quad
    vertices.emplace_back(-1.0f,  1.0f, 0.0f);
    vertices.emplace_back(-1.0f, -1.0f, 0.0f);
    vertices.emplace_back( 1.0f, -1.0f, 0.0f);
    vertices.emplace_back( 1.0f,  1.0f, 0.0f);

    normals.emplace_back(0.0f, 0.0f, 1.0f);
    normals.emplace_back(0.0f, 0.0f, 1.0f);
    normals.emplace_back(0.0f, 0.0f, 1.0f);

    face_normals.emplace_back(0.0f, 0.0f, 1.0f);
    face_normals.emplace_back(0.0f, 0.0f, 1.0f);

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

    // Create a checkerboard picture
    size_t width  = 1024;
    size_t height = 1024;
    pixels.resize(width * height);
    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            pixels[y * width + x] = (x + y) % 2 != 0 ? uint32_t(-1) : 0;
        }
    }

    // Generate streams
    PrimaryStream primary;
    size_t num_rays = 1024 * 4;
    anydsl::Array<float> primary_data(20 * num_rays);
    anydsl::Array<float> tmp_data(20 * num_rays);
    get_primary_stream(primary, primary_data.data(), num_rays);

    // Generate rays
    float3 org(0.0f, 0.0f, -1.0f);
    uint32_t seed = 42;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<float> rnd(0.0f, 1.0f);
    size_t num_geometries = 4;
    size_t rays_per_geom = num_rays / num_geometries;
    begins.resize(num_geometries);
    ends.resize(num_geometries);
    for (size_t geom = 0, cur = 0; geom < num_geometries; ++geom, cur += rays_per_geom) {
        begins[geom] = cur;
        ends[geom] = cur + rays_per_geom;
        for (size_t i = 0; i < rays_per_geom; ++i) {
            // Generate a point on one triangle
            int32_t prim_id = rnd(gen) < 0.5f ? 0 : 1;
            float u = rnd(gen);
            float v = rnd(gen);
            if (u + v > 1.0f) {
                u = 1.0f - u;
                v = 1.0f - v;
            }
            int32_t i0 = indices[prim_id * 4 + 0];
            int32_t i1 = indices[prim_id * 4 + 1];
            int32_t i2 = indices[prim_id * 4 + 2];
            float3 p = (1.0f - u - v) * vertices[i0] + u * vertices[i1] + v * vertices[i2];
            float3 dir = p - org;

            // Create ray
            primary.rays.id[cur + i] = cur + i;
            primary.rays.org_x[cur + i] = org.x;
            primary.rays.org_y[cur + i] = org.y;
            primary.rays.org_z[cur + i] = org.z;
            primary.rays.dir_x[cur + i] = dir.x;
            primary.rays.dir_y[cur + i] = dir.y;
            primary.rays.dir_z[cur + i] = dir.z;
            primary.rays.tmin[cur + i] = 0.0f;
            primary.rays.tmax[cur + i] = std::numeric_limits<float>::max();

            primary.geom_id[cur + i] = geom;
            primary.prim_id[cur + i] = prim_id;
            primary.t[cur + i] = 1.0f;
            primary.u[cur + i] = u;
            primary.v[cur + i] = v;
            primary.rnd[cur + i] = 33 * geom + i;
            primary.mis[cur + i] = 0.5f;
            primary.contrib_r[cur + i] = 1.0f;
            primary.contrib_g[cur + i] = 1.0f;
            primary.contrib_b[cur + i] = 1.0f;
            primary.depth[cur + i] = 0;
        }
    }
    primary.size = ends[num_geometries - 1];
    anydsl::copy(primary_data, tmp_data);

    size_t num_iters = 1000;
    std::vector<uint64_t> us;
    for (size_t iter = 0; iter < num_iters; ++iter) {
        using namespace std::chrono;
        auto start = high_resolution_clock::now();
        cpu_bench_shading(&primary,
            (Vec3*)vertices.data(),
            (Vec3*)normals.data(),
            (Vec3*)face_normals.data(),
            (Vec2*)texcoords.data(),
            indices.data(),
            pixels.data(),
            width,
            height,
            begins.data(),
            ends.data(),
            2
        );
        auto end = high_resolution_clock::now();
        us.push_back(duration_cast<microseconds>(end - start).count());
        anydsl::copy(tmp_data, primary_data);
    }
    std::sort(us.begin(), us.end());
    std::cout << double(num_rays) / double(us[us.size() / 2]) << " Mrays/s" << std::endl;
    return 0;
}
