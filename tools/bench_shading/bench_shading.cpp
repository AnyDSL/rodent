#include <iostream>
#include <cstdint>
#include <vector>
#include <random>
#include <limits>
#include <algorithm>

#include <x86intrin.h>

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
    _mm_setcsr(_mm_getcsr() | (_MM_FLUSH_ZERO_ON | _MM_DENORMALS_ZERO_ON));

    std::vector<float3> vertices;
    std::vector<float3> normals;
    std::vector<float3> face_normals;
    std::vector<float2> texcoords;
    std::vector<int32_t> indices;
    std::vector<uint32_t> pixels;

    std::vector<int32_t> begins;
    std::vector<int32_t> ends;

    const bool sorted = true;

    // Create a quad
    vertices.emplace_back(-1.0f,  1.0f, 0.0f);
    vertices.emplace_back(-1.0f, -1.0f, 0.0f);
    vertices.emplace_back( 1.0f, -1.0f, 0.0f);
    vertices.emplace_back( 1.0f,  1.0f, 0.0f);

    normals.emplace_back(0.0f, 0.0f, 1.0f);
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
    PrimaryStream primary_in, primary_out;
    size_t num_rays = 4096;
    anydsl::Array<float> primary_in_data (20 * num_rays);
    anydsl::Array<float> primary_out_data(20 * num_rays);
    get_primary_stream(primary_in,  primary_in_data.data(), num_rays);
    get_primary_stream(primary_out, primary_out_data.data(), num_rays);

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
            primary_in.rays.id[cur + i] = cur + i;
            primary_in.rays.org_x[cur + i] = org.x;
            primary_in.rays.org_y[cur + i] = org.y;
            primary_in.rays.org_z[cur + i] = org.z;
            primary_in.rays.dir_x[cur + i] = dir.x;
            primary_in.rays.dir_y[cur + i] = dir.y;
            primary_in.rays.dir_z[cur + i] = dir.z;
            primary_in.rays.tmin[cur + i] = 0.0f;
            primary_in.rays.tmax[cur + i] = std::numeric_limits<float>::max();

            primary_in.geom_id[cur + i] = geom;
            primary_in.prim_id[cur + i] = prim_id;
            primary_in.t[cur + i] = 1.0f;
            primary_in.u[cur + i] = u;
            primary_in.v[cur + i] = v;
            primary_in.rnd[cur + i] = 33 * geom + i;
            primary_in.mis[cur + i] = 0.5f;
            primary_in.contrib_r[cur + i] = 1.0f;
            primary_in.contrib_g[cur + i] = 1.0f;
            primary_in.contrib_b[cur + i] = 1.0f;
            primary_in.depth[cur + i] = 0;
        }
    }
    primary_in.size = num_rays;
    if (!sorted) {
        std::vector<size_t> ids(num_rays);
        std::iota(ids.begin(), ids.end(), 0);
        std::shuffle(ids.begin(), ids.end(), gen);
        for (size_t i = 0; i < num_rays; ++i) {
            primary_out.rays.org_x[i] = primary_in.rays.org_x[ids[i]];
            primary_out.rays.org_y[i] = primary_in.rays.org_y[ids[i]];
            primary_out.rays.org_z[i] = primary_in.rays.org_z[ids[i]];
            primary_out.rays.dir_x[i] = primary_in.rays.dir_x[ids[i]];
            primary_out.rays.dir_y[i] = primary_in.rays.dir_y[ids[i]];
            primary_out.rays.dir_z[i] = primary_in.rays.dir_z[ids[i]];
            primary_out.rays.tmin[i]  = primary_in.rays.tmin[ids[i]];
            primary_out.rays.tmax[i]  = primary_in.rays.tmax[ids[i]];

            primary_out.geom_id[i]    = primary_in.geom_id[ids[i]];
            primary_out.prim_id[i]    = primary_in.prim_id[ids[i]];
            primary_out.t[i]          = primary_in.t[ids[i]];
            primary_out.u[i]          = primary_in.u[ids[i]];
            primary_out.v[i]          = primary_in.v[ids[i]];
            primary_out.rnd[i]        = primary_in.rnd[ids[i]];
            primary_out.mis[i]        = primary_in.mis[ids[i]];
            primary_out.contrib_r[i]  = primary_in.contrib_r[ids[i]];
            primary_out.contrib_g[i]  = primary_in.contrib_g[ids[i]];
            primary_out.contrib_b[i]  = primary_in.contrib_b[ids[i]];
            primary_out.depth[i]      = primary_in.depth[ids[i]];
        }
        anydsl::copy(primary_out_data, primary_in_data);
    }

    size_t num_iters = 100;
    size_t num_bench = 1000;
    int64_t cpu_mhz = 4000;
    std::vector<uint64_t> us;
    for (size_t i = 0; i < num_bench; ++i) {
        auto start = __rdtsc();
        cpu_bench_shading(
            &primary_in,
            &primary_out,
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
            2,
            num_iters
        );
        auto end = __rdtsc();
        us.push_back((end - start) / cpu_mhz);
    }
    std::sort(us.begin(), us.end());
    std::cout << double(num_rays * num_iters) / double(us[us.size()/2]) << " Mrays/s" << std::endl;
    return 0;
}
