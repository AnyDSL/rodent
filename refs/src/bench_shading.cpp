#include <iostream>
#include <cstdint>
#include <vector>
#include <random>
#include <limits>
#include <memory>
#include <algorithm>
#include <cassert>

#include <x86intrin.h>

#include <embree3/rtcore.h>

#include "float2.h"
#include "float3.h"
#include "bench_shading.h"
#include "embree_path_tracer.h"

extern "C" void shade_varying(stream_s* stream_in, stream_s* stream_out, float3_s* normals, float3_s* face_normals, float2_s* texcoords, int32_t* indices, image_s* image, int32_t num_rays, int32_t num_iters);
extern "C" void shade_uniform(stream_s* stream_in, stream_s* stream_out, float3_s* normals, float3_s* face_normals, float2_s* texcoords, int32_t* indices, image_s* image, int32_t* begins, int32_t* ends, int32_t num_iters);

inline void get_stream(stream_s& stream, float* ptr, size_t capacity) {
    stream.org_x     = ptr + 0 * capacity;
    stream.org_y     = ptr + 1 * capacity;
    stream.org_z     = ptr + 2 * capacity;
    stream.dir_x     = ptr + 3 * capacity;
    stream.dir_y     = ptr + 4 * capacity;
    stream.dir_z     = ptr + 5 * capacity;
    stream.tmin      = ptr + 6 * capacity;
    stream.tmax      = ptr + 7 * capacity;
    stream.geom_id   = (int32_t*)ptr + 8 * capacity;
    stream.prim_id   = (int32_t*)ptr + 9 * capacity;
    stream.t         = ptr + 10 * capacity;
    stream.u         = ptr + 11 * capacity;
    stream.v         = ptr + 12 * capacity;
    stream.rnd       = (uint32_t*)ptr + 13 * capacity;
    stream.mis       = ptr + 14 * capacity;
    stream.contrib_r = ptr + 15 * capacity;
    stream.contrib_g = ptr + 16 * capacity;
    stream.contrib_b = ptr + 17 * capacity;
    stream.depth     = (int32_t*)ptr + 18 * capacity;
}

int main(int argc, char** argv) {
    _mm_setcsr(_mm_getcsr() | (_MM_FLUSH_ZERO_ON | _MM_DENORMALS_ZERO_ON));

    const bool specialized = false;
    const bool sorted = false;

    // Create a quad
    std::vector<float3> vertices;
    std::vector<float3> normals;
    std::vector<float3> face_normals;
    std::vector<float2> texcoords;
    std::vector<int32_t> indices;

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
    std::vector<uint32_t> pixels;
    size_t width  = 1024;
    size_t height = 1024;
    pixels.resize(width * height);
    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            pixels[y * width + x] = (x + y) % 2 != 0 ? uint32_t(-1) : 0;
        }
    }

    image_s image = {
        pixels.data(),
        uint32_t(width),
        uint32_t(height)
    };

    // Generate streams
    stream_s stream_in, stream_out;
    size_t num_rays = 4096;
    size_t total_mem = 19 * num_rays;
    std::unique_ptr<float[]> stream_in_data(new float[total_mem]);
    std::unique_ptr<float[]> stream_out_data(new float[total_mem]);
    get_stream(stream_in,  stream_in_data.get(), num_rays);
    get_stream(stream_out, stream_out_data.get(), num_rays);

    // Generate rays
    float3 org(0.0f, 0.0f, -1.0f);
    uint32_t seed = 42;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<float> rnd(0.0f, 1.0f);
    size_t num_geometries = 4;
    size_t rays_per_geom = num_rays / num_geometries;
    assert(num_rays % num_geometries == 0);
    std::vector<int32_t> begins(num_geometries);
    std::vector<int32_t> ends(num_geometries);
    for (size_t geom = 0, cur = 0; geom < num_geometries; ++geom, cur += rays_per_geom) {
        begins[geom] = cur;
        ends[geom]   = cur + rays_per_geom;
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
            stream_in.org_x[cur + i] = org.x;
            stream_in.org_y[cur + i] = org.y;
            stream_in.org_z[cur + i] = org.z;
            stream_in.dir_x[cur + i] = dir.x;
            stream_in.dir_y[cur + i] = dir.y;
            stream_in.dir_z[cur + i] = dir.z;
            stream_in.tmin[cur + i] = 0.0f;
            stream_in.tmax[cur + i] = std::numeric_limits<float>::max();
            stream_in.geom_id[cur + i] = geom;
            stream_in.prim_id[cur + i] = prim_id;
            stream_in.t[cur + i] = 1.0f;
            stream_in.u[cur + i] = u;
            stream_in.v[cur + i] = v;
            stream_in.rnd[cur + i] = 33 * geom + i;
            stream_in.mis[cur + i] = 0.5f;
            stream_in.contrib_r[cur + i] = 1.0f;
            stream_in.contrib_g[cur + i] = 1.0f;
            stream_in.contrib_b[cur + i] = 1.0f;
            stream_in.depth[cur + i] = 0;
        }
    }
    if (!sorted) {
        std::vector<size_t> ids(num_rays);
        std::iota(ids.begin(), ids.end(), 0);
        std::shuffle(ids.begin(), ids.end(), gen);
        for (size_t i = 0; i < num_rays; ++i) {
            stream_out.org_x[i]     = stream_in.org_x[ids[i]];
            stream_out.org_y[i]     = stream_in.org_y[ids[i]];
            stream_out.org_z[i]     = stream_in.org_z[ids[i]];
            stream_out.dir_x[i]     = stream_in.dir_x[ids[i]];
            stream_out.dir_y[i]     = stream_in.dir_y[ids[i]];
            stream_out.dir_z[i]     = stream_in.dir_z[ids[i]];
            stream_out.tmin[i]      = stream_in.tmin[ids[i]];
            stream_out.tmax[i]      = stream_in.tmax[ids[i]];
            stream_out.geom_id[i]   = stream_in.geom_id[ids[i]];
            stream_out.prim_id[i]   = stream_in.prim_id[ids[i]];
            stream_out.t[i]         = stream_in.t[ids[i]];
            stream_out.u[i]         = stream_in.u[ids[i]];
            stream_out.v[i]         = stream_in.v[ids[i]];
            stream_out.rnd[i]       = stream_in.rnd[ids[i]];
            stream_out.mis[i]       = stream_in.mis[ids[i]];
            stream_out.contrib_r[i] = stream_in.contrib_r[ids[i]];
            stream_out.contrib_g[i] = stream_in.contrib_g[ids[i]];
            stream_out.contrib_b[i] = stream_in.contrib_b[ids[i]];
            stream_out.depth[i]     = stream_in.depth[ids[i]];
        }
        std::copy(stream_out_data.get(), stream_out_data.get() + total_mem, stream_in_data.get());
    }

    size_t num_iters = 1000;
    size_t num_bench = 100;
    int64_t cpu_mhz = 4000;
    std::vector<uint64_t> us;
    for (size_t i = 0; i < num_bench; ++i) {
        auto start = __rdtsc();
        if (specialized && sorted) {
            shade_uniform(&stream_in,
                &stream_out,
                (float3_s*)normals.data(),
                (float3_s*)face_normals.data(),
                (float2_s*)texcoords.data(),
                indices.data(),
                &image,
                begins.data(), ends.data(),
                num_iters
            );
        } else {
            shade_varying(&stream_in,
                &stream_out,
                (float3_s*)normals.data(),
                (float3_s*)face_normals.data(),
                (float2_s*)texcoords.data(),
                indices.data(),
                &image,
                num_rays,
                num_iters
            );
        }
        auto end = __rdtsc();
        us.push_back((end - start) / cpu_mhz);
    }
    std::sort(us.begin(), us.end());
    std::cout << double(num_rays * num_iters) / double(us[us.size() / 2]) << " Mrays/s" << std::endl;
    return 0;
}
