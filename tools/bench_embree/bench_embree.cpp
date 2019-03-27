#include <iostream>
#include <fstream>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <string>
#include <cstring>
#include <functional>

#if EMBREE_VERSION == 3
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>
#else
#include <embree2/rtcore.h>
#include <embree2/rtcore_ray.h>
#endif
#include <common/math/vec3.h>
#include <common/math/vec3fa.h>

#include <anydsl_runtime.hpp>

#include "traversal.h"
#include "driver/obj.h"
#include "driver/tri.h"
#include "load_rays.h"

#if EMBREE_VERSION == 3
template <>
struct RayTraits<RTCRay> {
    enum { RayPerPacket = 1 };
    static void write_ray(const float* org_dir, float tmin, float tmax, int j, RTCRay& ray) {
        ray.org_x = org_dir[0];
        ray.org_y = org_dir[1];
        ray.org_z = org_dir[2];
        ray.tnear = tmin;
        ray.dir_x = org_dir[3];
        ray.dir_y = org_dir[4];
        ray.dir_z = org_dir[5];
        ray.tfar = tmax;
        ray.mask = 0xFFFFFFFF;
        ray.time = 0.0f;
        ray.flags = 0;
    }
};

template <>
struct RayTraits<RTCRay4> {
    enum { RayPerPacket = 4 };
    static void write_ray(const float* org_dir, float tmin, float tmax, int j, RTCRay4& ray) {
        ray.org_x[j] = org_dir[0];
        ray.org_y[j] = org_dir[1];
        ray.org_z[j] = org_dir[2];
        ray.tnear[j] = tmin;
        ray.dir_x[j] = org_dir[3];
        ray.dir_y[j] = org_dir[4];
        ray.dir_z[j] = org_dir[5];
        ray.tfar[j] = tmax;
        ray.mask[j] = 0xFFFFFFFF;
        ray.time[j] = 0.0f;
        ray.flags[j] = 0;
    }
};

template <>
struct RayTraits<RTCRay8> {
    enum { RayPerPacket = 8 };
    static void write_ray(const float* org_dir, float tmin, float tmax, int j, RTCRay8& ray) {
        ray.org_x[j] = org_dir[0];
        ray.org_y[j] = org_dir[1];
        ray.org_z[j] = org_dir[2];
        ray.tnear[j] = tmin;
        ray.dir_x[j] = org_dir[3];
        ray.dir_y[j] = org_dir[4];
        ray.dir_z[j] = org_dir[5];
        ray.tfar[j] = tmax;
        ray.mask[j] = 0xFFFFFFFF;
        ray.time[j] = 0.0f;
        ray.flags[j] = 0;
    }
};

template <>
struct RayTraits<RTCRayHit> {
    enum { RayPerPacket = 1 };
    static void write_ray(const float* org_dir, float tmin, float tmax, int j, RTCRayHit& ray) {
        ray.ray.org_x = org_dir[0];
        ray.ray.org_y = org_dir[1];
        ray.ray.org_z = org_dir[2];
        ray.ray.tnear = tmin;
        ray.ray.dir_x = org_dir[3];
        ray.ray.dir_y = org_dir[4];
        ray.ray.dir_z = org_dir[5];
        ray.ray.tfar = tmax;
        ray.ray.mask = 0xFFFFFFFF;
        ray.ray.time = 0.0f;
        ray.ray.flags = 0;
        ray.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        ray.hit.primID = RTC_INVALID_GEOMETRY_ID;
        for (int i = 0; i < RTC_MAX_INSTANCE_LEVEL_COUNT; ++i)
            ray.hit.instID[i] = RTC_INVALID_GEOMETRY_ID;
    }
};

template <>
struct RayTraits<RTCRayHit4> {
    enum { RayPerPacket = 4 };
    static void write_ray(const float* org_dir, float tmin, float tmax, int j, RTCRayHit4& ray) {
        ray.ray.org_x[j] = org_dir[0];
        ray.ray.org_y[j] = org_dir[1];
        ray.ray.org_z[j] = org_dir[2];
        ray.ray.tnear[j] = tmin;
        ray.ray.dir_x[j] = org_dir[3];
        ray.ray.dir_y[j] = org_dir[4];
        ray.ray.dir_z[j] = org_dir[5];
        ray.ray.tfar[j] = tmax;
        ray.ray.mask[j] = 0xFFFFFFFF;
        ray.ray.time[j] = 0.0f;
        ray.ray.flags[j] = 0;
        ray.hit.geomID[j] = RTC_INVALID_GEOMETRY_ID;
        ray.hit.primID[j] = RTC_INVALID_GEOMETRY_ID;
        for (int i = 0; i < RTC_MAX_INSTANCE_LEVEL_COUNT; ++i)
            ray.hit.instID[i][j] = RTC_INVALID_GEOMETRY_ID;
    }
};

template <>
struct RayTraits<RTCRayHit8> {
    enum { RayPerPacket = 8 };
    static void write_ray(const float* org_dir, float tmin, float tmax, int j, RTCRayHit8& ray) {
        ray.ray.org_x[j] = org_dir[0];
        ray.ray.org_y[j] = org_dir[1];
        ray.ray.org_z[j] = org_dir[2];
        ray.ray.tnear[j] = tmin;
        ray.ray.dir_x[j] = org_dir[3];
        ray.ray.dir_y[j] = org_dir[4];
        ray.ray.dir_z[j] = org_dir[5];
        ray.ray.tfar[j] = tmax;
        ray.ray.mask[j] = 0xFFFFFFFF;
        ray.ray.time[j] = 0.0f;
        ray.ray.flags[j] = 0;
        ray.hit.geomID[j] = RTC_INVALID_GEOMETRY_ID;
        ray.hit.primID[j] = RTC_INVALID_GEOMETRY_ID;
        for (int i = 0; i < RTC_MAX_INSTANCE_LEVEL_COUNT; ++i)
            ray.hit.instID[i][j] = RTC_INVALID_GEOMETRY_ID;
    }
};
#else
template <>
struct RayTraits<RTCRay> {
    enum { RayPerPacket = 1 };
    static void write_ray(const float* org_dir, float tmin, float tmax, int j, RTCRay& ray) {
        ray.org[0] = org_dir[0];
        ray.org[1] = org_dir[1];
        ray.org[2] = org_dir[2];
        ray.tnear = tmin;
        ray.dir[0] = org_dir[3];
        ray.dir[1] = org_dir[4];
        ray.dir[2] = org_dir[5];
        ray.tfar = tmax;
        ray.mask = 0xFFFFFFFF;
        ray.time = 0.0f;
        ray.geomID = RTC_INVALID_GEOMETRY_ID;
        ray.primID = RTC_INVALID_GEOMETRY_ID;
        ray.instID = RTC_INVALID_GEOMETRY_ID;
    }
};

template <>
struct RayTraits<RTCRay4> {
    enum { RayPerPacket = 4 };
    static void write_ray(const float* org_dir, float tmin, float tmax, int j, RTCRay4& ray) {
        ray.orgx[j] = org_dir[0];
        ray.orgy[j] = org_dir[1];
        ray.orgz[j] = org_dir[2];
        ray.tnear[j] = tmin;
        ray.dirx[j] = org_dir[3];
        ray.diry[j] = org_dir[4];
        ray.dirz[j] = org_dir[5];
        ray.tfar[j] = tmax;
        ray.mask[j] = 0xFFFFFFFF;
        ray.time[j] = 0.0f;
        ray.geomID[j] = RTC_INVALID_GEOMETRY_ID;
        ray.primID[j] = RTC_INVALID_GEOMETRY_ID;
        ray.instID[j] = RTC_INVALID_GEOMETRY_ID;
    }
};

template <>
struct RayTraits<RTCRay8> {
    enum { RayPerPacket = 8 };
    static void write_ray(const float* org_dir, float tmin, float tmax, int j, RTCRay8& ray) {
        ray.orgx[j] = org_dir[0];
        ray.orgy[j] = org_dir[1];
        ray.orgz[j] = org_dir[2];
        ray.tnear[j] = tmin;
        ray.dirx[j] = org_dir[3];
        ray.diry[j] = org_dir[4];
        ray.dirz[j] = org_dir[5];
        ray.tfar[j] = tmax;
        ray.mask[j] = 0xFFFFFFFF;
        ray.time[j] = 0.0f;
        ray.geomID[j] = RTC_INVALID_GEOMETRY_ID;
        ray.primID[j] = RTC_INVALID_GEOMETRY_ID;
        ray.instID[j] = RTC_INVALID_GEOMETRY_ID;
    }
};
#endif

inline void check_argument(int i, int argc, char** argv) {
    if (i + 1 >= argc) {
        std::cerr << "Missing argument for " << argv[i] << std::endl;
        exit(1);
    }
}

inline void usage() {
    std::cout << "Usage: bench_embree [options]\n"
                 "Available options:\n"
                 "  -obj     --obj-file        Sets the OBJ file to use\n"
                 "  -ray     --ray-file        Sets the ray file to use\n"
                 "           --tmin            Sets the minimum distance along the rays (default: 0)\n"
                 "           --tmax            Sets the maximum distance along the rays (default: 1e9)\n"
                 "           --bench           Sets the number of benchmark iterations (default: 1)\n"
                 "           --warmup          Sets the number of warmup iterations (default: 0)\n"
                 "  -any                       Exit at the first intersection (disabled by default)\n"
                 "  -s       --single          Use single rays (disabled by default)\n"
                 "           --bvh-width       Sets the BVH width (4 or 8, default: 4)\n"
                 "           --ray-width       Sets the ray width (4 or 8, default: 8)\n"
                 "  -o       --output          Sets the output file name (no file is generated by default)\n";
}

static void create_triangles(const obj::File& obj_file, std::vector<Tri>& tris) {
    for (auto& object : obj_file.objects) {
        for (auto& group : object.groups) {
            for (auto& face : group.faces) {
                auto v0 = obj_file.vertices[face.indices[0].v];
                for (int i = 0; i < face.indices.size() - 2; i++) {
                    auto v1 = obj_file.vertices[face.indices[i + 1].v];
                    auto v2 = obj_file.vertices[face.indices[i + 2].v];
                    tris.emplace_back(float3(v0.x, v0.y, v0.z),
                                      float3(v1.x, v1.y, v1.z),
                                      float3(v2.x, v2.y, v2.z));
                }
            }
        }
    }
}

template <typename RayType, typename F, typename... Args>
double intersect_scene(const anydsl::Array<RayType>& rays, anydsl::Array<RayType>& hits, F f, Args... args) {
    using namespace std::chrono;

    // Restore tnear, tfar and other fields that have been modified
    anydsl::copy(rays, hits);

    auto t0 = high_resolution_clock::now();
#if EMBREE_VERSION == 3
    for (int i = 0; i < rays.size(); i++)
        f(args..., &hits[i]);
#else
    for (int i = 0; i < rays.size(); i++)
        f(args..., hits[i]);
#endif
    auto t1 = high_resolution_clock::now();
    return duration_cast<microseconds>(t1 - t0).count() * 1.0e-3;
}

static void error_handler(void*, const RTCError code, const char* str) {
#if EMBREE_VERSION == 3
    if (code == RTC_ERROR_NONE)
        return;

    std::cerr << "Embree error: ";
    switch (code) {
        case RTC_ERROR_UNKNOWN:           std::cerr << "RTC_ERROR_UNKNOWN";           break;
        case RTC_ERROR_INVALID_ARGUMENT:  std::cerr << "RTC_ERROR_INVALID_ARGUMENT";  break;
        case RTC_ERROR_INVALID_OPERATION: std::cerr << "RTC_ERROR_INVALID_OPERATION"; break;
        case RTC_ERROR_OUT_OF_MEMORY:     std::cerr << "RTC_ERROR_OUT_OF_MEMORY";     break;
        case RTC_ERROR_UNSUPPORTED_CPU:   std::cerr << "RTC_ERROR_UNSUPPORTED_CPU";   break;
        case RTC_ERROR_CANCELLED:         std::cerr << "RTC_ERROR_CANCELLED";         break;
        default:                          std::cerr << "invalid error code";          break;
    }
#else
    if (code == RTC_NO_ERROR)
        return;

    std::cerr << "Embree error: ";
    switch (code) {
        case RTC_UNKNOWN_ERROR:     std::cerr << "RTC_UNKNOWN_ERROR";       break;
        case RTC_INVALID_ARGUMENT:  std::cerr << "RTC_INVALID_ARGUMENT";    break;
        case RTC_INVALID_OPERATION: std::cerr << "RTC_INVALID_OPERATION";   break;
        case RTC_OUT_OF_MEMORY:     std::cerr << "RTC_OUT_OF_MEMORY";       break;
        case RTC_UNSUPPORTED_CPU:   std::cerr << "RTC_UNSUPPORTED_CPU";     break;
        case RTC_CANCELLED:         std::cerr << "RTC_CANCELLED";           break;
        default:                    std::cerr << "invalid error code";      break;
    }
#endif

    if (str) std::cerr << " (" << str << ")";
    std::cerr << std::endl;

    exit(1);
}

void bench(int N, const std::vector<Tri>& tris,
#if EMBREE_VERSION == 3
           std::function<double(RTCScene, RTCIntersectContext*)> iter_fn,
#else
           std::function<double(RTCScene)> iter_fn,
#endif
           std::function<size_t()> count_hits,
           int iters, int warmup,
           size_t num_rays) {
    using namespace embree;

    auto device = rtcNewDevice(N == 4 ? "tri_accel=bvh4.triangle4" : "tri_accel=bvh8.triangle4");

#if EMBREE_VERSION == 3
    error_handler(nullptr, rtcGetDeviceError(device), "");
    rtcSetDeviceErrorFunction(device, error_handler, nullptr);

    auto scene = rtcNewScene(device);
    auto mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    auto vertices = (float*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(float) * 4, tris.size() * 3);
    for (int i = 0; i < tris.size(); i++) {
        vertices[12 * i +  0] = tris[i].v0.x;
        vertices[12 * i +  1] = tris[i].v0.y;
        vertices[12 * i +  2] = tris[i].v0.z;
        vertices[12 * i +  3] = 1.0f;

        vertices[12 * i +  4] = tris[i].v1.x;
        vertices[12 * i +  5] = tris[i].v1.y;
        vertices[12 * i +  6] = tris[i].v1.z;
        vertices[12 * i +  7] = 1.0f;

        vertices[12 * i +  8] = tris[i].v2.x;
        vertices[12 * i +  9] = tris[i].v2.y;
        vertices[12 * i + 10] = tris[i].v2.z;
        vertices[12 * i + 11] = 1.0f;
    }

    auto indices = (int*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(int) * 3, tris.size());
    for (int i = 0; i < tris.size(); i++) {
        indices[i * 3 + 0] = i * 3 + 0;
        indices[i * 3 + 1] = i * 3 + 1;
        indices[i * 3 + 2] = i * 3 + 2;
    }

    rtcCommitGeometry(mesh);
    rtcAttachGeometry(scene, mesh);
    rtcReleaseGeometry(mesh);
    rtcCommitScene(scene);

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    std::vector<double> timings(iters);
    for (int i = 0; i < warmup; i++) iter_fn(scene, &context);
    for (int i = 0; i < iters ; i++) timings[i] = iter_fn(scene, &context);
#else
    error_handler(nullptr, rtcDeviceGetError(device), "");
    rtcDeviceSetErrorFunction2(device, error_handler, nullptr);

    auto scene = rtcDeviceNewScene(device, RTC_SCENE_STATIC, RTC_INTERSECT8 | RTC_INTERSECT4 | RTC_INTERSECT1);
    auto mesh = rtcNewTriangleMesh(scene, RTC_GEOMETRY_STATIC, tris.size(), tris.size() * 3);
    auto vertices = (float*)rtcMapBuffer(scene, mesh, RTC_VERTEX_BUFFER);
    for (int i = 0; i < tris.size(); i++) {
        vertices[12 * i + 0] = tris[i].v0.x;
        vertices[12 * i + 1] = tris[i].v0.y;
        vertices[12 * i + 2] = tris[i].v0.z;
        vertices[12 * i + 3] = 1.0f;

        vertices[12 * i + 4] = tris[i].v1.x;
        vertices[12 * i + 5] = tris[i].v1.y;
        vertices[12 * i + 6] = tris[i].v1.z;
        vertices[12 * i + 7] = 1.0f;

        vertices[12 * i +  8] = tris[i].v2.x;
        vertices[12 * i +  9] = tris[i].v2.y;
        vertices[12 * i + 10] = tris[i].v2.z;
        vertices[12 * i + 11] = 1.0f;
    }
    rtcUnmapBuffer(scene, mesh, RTC_VERTEX_BUFFER);

    auto triangles = (int*)rtcMapBuffer(scene, mesh, RTC_INDEX_BUFFER);
    for (int i = 0; i < tris.size(); i++) {
        triangles[i * 3 + 0] = i * 3 + 0;
        triangles[i * 3 + 1] = i * 3 + 1;
        triangles[i * 3 + 2] = i * 3 + 2;
    }
    rtcUnmapBuffer(scene, mesh, RTC_INDEX_BUFFER);
    rtcCommit(scene);

    std::vector<double> timings(iters);
    for (int i = 0; i < warmup; i++) iter_fn(scene);
    for (int i = 0; i < iters ; i++) timings[i] = iter_fn(scene);
#endif

    size_t intr = count_hits();

    std::sort(timings.begin(), timings.end());
    auto sum = std::accumulate(timings.begin(), timings.end(), 0.0);
    auto avg = sum / timings.size();
    auto med = timings[timings.size() / 2];
    auto min = *std::min_element(timings.begin(), timings.end());
    std::cout << sum << "ms for " << iters << " iteration(s)" << std::endl;
    std::cout << num_rays * iters / (1000.0 * sum) << " Mrays/sec" << std::endl;
    std::cout << "# Average: " << avg << " ms" << std::endl;
    std::cout << "# Median: " << med  << " ms" << std::endl;
    std::cout << "# Min: " << min << " ms" << std::endl;
    std::cout << intr << " intersection(s)" << std::endl;

#if EMBREE_VERSION == 3
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);
#else
    rtcDeleteScene(scene);
    rtcDeleteDevice(device);
#endif
}

int main(int argc, char** argv) {
    std::string ray_file;
    std::string obj_file;
    std::string out_file;
    float tmin = 0.0f, tmax = 1e9f;
    int iters = 1;
    int warmup = 0;
    int ray_width = 8;
    int bvh_width = 4;
    bool any_hit = false;
    bool single_ray = false;

    for (int i = 1; i < argc; i++) {
        auto arg = argv[i];
        if (arg[0] == '-') {
            if (!strcmp(arg, "-h") || !strcmp(arg, "--help")) {
                usage();
                return 0;
            } else if (!strcmp(arg, "-obj") || !strcmp(arg, "--obj-file")) {
                check_argument(i, argc, argv);
                obj_file = argv[++i];
            } else if (!strcmp(arg, "-ray") || !strcmp(arg, "--ray-file")) {
                check_argument(i, argc, argv);
                ray_file = argv[++i];
            } else if (!strcmp(arg, "--tmin")) {
                check_argument(i, argc, argv);
                tmin = strtof(argv[++i], nullptr);
            } else if (!strcmp(arg, "--tmax")) {
                check_argument(i, argc, argv);
                tmax = strtof(argv[++i], nullptr);
            } else if (!strcmp(arg, "--bench")) {
                check_argument(i, argc, argv);
                iters = strtol(argv[++i], nullptr, 10);
            } else if (!strcmp(arg, "--warmup")) {
                check_argument(i, argc, argv);
                warmup = strtol(argv[++i], nullptr, 10);
            } else if (!strcmp(arg, "-any")) {
                any_hit = true;
            } else if (!strcmp(arg, "-s") || !strcmp(arg, "--single")) {
                single_ray = true;
            } else if (!strcmp(arg, "--bvh-width")) {
                check_argument(i, argc, argv);
                bvh_width = strtol(argv[++i], nullptr, 10);
            } else if (!strcmp(arg, "--ray-width")) {
                check_argument(i, argc, argv);
                ray_width = strtol(argv[++i], nullptr, 10);
            } else if (!strcmp(arg, "-o") || !strcmp(arg, "--output")) {
                check_argument(i, argc, argv);
                out_file = argv[++i];
            } else {
                std::cerr << "Unknown option '" << arg << "'" << std::endl;
                return 1;
            }
        } else {
            std::cerr << "Invalid argument '" << arg << "'" << std::endl;
            return 1;
        }
    }

    if (obj_file == "") {
        std::cerr << "No OBJ file specified" << std::endl;
        return 1;
    }
    if (ray_file == "") {
        std::cerr << "No ray file specified" << std::endl;
        return 1;
    }
    if (bvh_width != 4 && bvh_width != 8) {
        std::cerr << "Invalid BVH width" << std::endl;
        return 1;
    }
    if (ray_width != 4 && ray_width != 8) {
        std::cerr << "Invalid ray width" << std::endl;
        return 1;
    }

    obj::File obj;
    if (!load_obj(obj_file, obj)) {
        std::cerr << "Cannot load OBJ file" << std::endl;
        return 1;
    }

    std::vector<Tri> tris;
    create_triangles(obj, tris);

#if EMBREE_VERSION == 3
    anydsl::Array<RTCRayHit>  rayhits1, rayhits1_res;
    anydsl::Array<RTCRayHit4> rayhits4, rayhits4_res;
    anydsl::Array<RTCRayHit8> rayhits8, rayhits8_res;
#endif
    anydsl::Array<RTCRay>  rays1, rays1_res;
    anydsl::Array<RTCRay4> rays4, rays4_res;
    anydsl::Array<RTCRay8> rays8, rays8_res;

    const int valid[8] alignas(32) = { -1, -1, -1, -1, -1, -1, -1, -1 };
#if EMBREE_VERSION == 3
    std::function<double(RTCScene, RTCIntersectContext*)> iter_fn;
#else
    std::function<double(RTCScene)> iter_fn;
#endif
    std::function<size_t()> count_hits;

    size_t ray_count = 0;
    if (single_ray) {
#if EMBREE_VERSION == 3
        bool rays_loaded = any_hit
            ? load_rays(ray_file, rays1, tmin, tmax, anydsl::Platform::Host, anydsl::Device(0))
            : load_rays(ray_file, rayhits1, tmin, tmax, anydsl::Platform::Host, anydsl::Device(0));
#else
        bool rays_loaded = load_rays(ray_file, rays1, tmin, tmax, anydsl::Platform::Host, anydsl::Device(0));
#endif
        if (!rays_loaded) {
            std::cerr << "Cannot load rays" << std::endl;
            return 1;
        }
        rays1_res = std::move(anydsl::Array<RTCRay>(rays1.size()));
#if EMBREE_VERSION == 3
        rayhits1_res = std::move(anydsl::Array<RTCRayHit>(rayhits1.size()));
        if (any_hit) iter_fn = [&] (RTCScene scene, RTCIntersectContext* context) { return intersect_scene(rays1, rays1_res, rtcOccluded1, scene, context); };
        else         iter_fn = [&] (RTCScene scene, RTCIntersectContext* context) { return intersect_scene(rayhits1, rayhits1_res, rtcIntersect1, scene, context); };
        ray_count = any_hit ? rays1.size() : rayhits1.size();
#else
        if (any_hit) iter_fn = [&] (RTCScene scene) { return intersect_scene(rays1, rays1_res, rtcOccluded,  scene); };
        else         iter_fn = [&] (RTCScene scene) { return intersect_scene(rays1, rays1_res, rtcIntersect, scene); };
        ray_count = rays1.size();
#endif
        count_hits = [&] {
            size_t intr = 0;
            if (any_hit) {
                for (auto hit : rays1_res) intr += hit.tfar < 0.0f;
            } else {
#if EMBREE_VERSION == 3
                for (auto hit : rayhits1_res) intr += hit.hit.geomID != RTC_INVALID_GEOMETRY_ID;
#else
                for (auto hit : rays1_res) intr += hit.geomID != RTC_INVALID_GEOMETRY_ID;
#endif
            }
            return intr; 
        };
    } else if (ray_width == 4) {
#if EMBREE_VERSION == 3
        bool rays_loaded = any_hit
            ? load_rays(ray_file, rays4, tmin, tmax, anydsl::Platform::Host, anydsl::Device(0))
            : load_rays(ray_file, rayhits4, tmin, tmax, anydsl::Platform::Host, anydsl::Device(0));
#else
        bool rays_loaded = load_rays(ray_file, rays4, tmin, tmax, anydsl::Platform::Host, anydsl::Device(0));
#endif
        if (!rays_loaded) {
            std::cerr << "Cannot load rays" << std::endl;
            return 1;
        }
        rays4_res = std::move(anydsl::Array<RTCRay4>(rays4.size()));
#if EMBREE_VERSION == 3
        rayhits4_res = std::move(anydsl::Array<RTCRayHit4>(rayhits4.size()));
        if (any_hit) iter_fn = [&] (RTCScene scene, RTCIntersectContext* context) { return intersect_scene(rays4, rays4_res, rtcOccluded4, valid, scene, context); };
        else         iter_fn = [&] (RTCScene scene, RTCIntersectContext* context) { return intersect_scene(rayhits4, rayhits4_res, rtcIntersect4, valid, scene, context); };
        ray_count = (any_hit ? rays4.size() : rayhits4.size()) * 4;
#else
        if (any_hit) iter_fn = [&] (RTCScene scene) { return intersect_scene(rays4, rays4_res, rtcOccluded4,  valid, scene); };
        else         iter_fn = [&] (RTCScene scene) { return intersect_scene(rays4, rays4_res, rtcIntersect4, valid, scene); };
        ray_count = rays4.size() * 4;
#endif
        count_hits = [&] {
            size_t intr = 0;
            if (any_hit) {
                for (auto hit : rays4_res) {
                    for (int i = 0; i < 4; i++) intr += hit.tfar[i] < 0.0f;
                }
            } else {
#if EMBREE_VERSION == 3
                for (auto hit : rayhits4_res) {
                    for (int i = 0; i < 4; i++) intr += hit.hit.geomID[i] != RTC_INVALID_GEOMETRY_ID;
                }
#else
                for (auto hit : rays4_res) {
                    for (int i = 0; i < 4; i++) intr += hit.geomID[i] != RTC_INVALID_GEOMETRY_ID;
                }
#endif
            }
            return intr; 
        };
    } else {
#if EMBREE_VERSION == 3
        bool rays_loaded = any_hit
            ? load_rays(ray_file, rays8, tmin, tmax, anydsl::Platform::Host, anydsl::Device(0))
            : load_rays(ray_file, rayhits8, tmin, tmax, anydsl::Platform::Host, anydsl::Device(0));
#else
        bool rays_loaded = load_rays(ray_file, rays8, tmin, tmax, anydsl::Platform::Host, anydsl::Device(0));
#endif
        if (!rays_loaded) {
            std::cerr << "Cannot load rays" << std::endl;
            return 1;
        }
        rays8_res = std::move(anydsl::Array<RTCRay8>(rays8.size()));
#if EMBREE_VERSION == 3
        rayhits8_res = std::move(anydsl::Array<RTCRayHit8>(rayhits8.size()));
        if (any_hit) iter_fn = [&] (RTCScene scene, RTCIntersectContext* context) { return intersect_scene(rays8, rays8_res, rtcOccluded8, valid, scene, context); };
        else         iter_fn = [&] (RTCScene scene, RTCIntersectContext* context) { return intersect_scene(rayhits8, rayhits8_res, rtcIntersect8, valid, scene, context); };
        ray_count = (any_hit ? rays8.size() : rayhits8.size()) * 8;
#else
        if (any_hit) iter_fn = [&] (RTCScene scene) { return intersect_scene(rays8, rays8_res, rtcOccluded8,  valid, scene); };
        else         iter_fn = [&] (RTCScene scene) { return intersect_scene(rays8, rays8_res, rtcIntersect8, valid, scene); };
        ray_count = rays8.size() * 8;
#endif
        count_hits = [&] {
            size_t intr = 0;
            if (any_hit) {
                for (auto hit : rays8_res) {
                    for (int i = 0; i < 8; i++) intr += hit.tfar[i] < 0.0f;
                }
            } else {
#if EMBREE_VERSION == 3
                for (auto hit : rayhits8_res) {
                    for (int i = 0; i < 8; i++) intr += hit.hit.geomID[i] != RTC_INVALID_GEOMETRY_ID;
                }
#else
                for (auto hit : rays8_res) {
                    for (int i = 0; i < 8; i++) intr += hit.geomID[i] != RTC_INVALID_GEOMETRY_ID;
                }
#endif
            }
            return intr; 
        };
    }

    std::cout << ray_count << " ray(s) in the distribution file." << std::endl;
    bench(bvh_width, tris, iter_fn, count_hits, iters, warmup, ray_count);

    if (out_file != "") {
        if (any_hit) {
            std::cerr << "Cannot create output file when the '-any' option is enabled" << std::endl;
            return 1;
        }
        std::ofstream of(out_file, std::ofstream::binary);
#if EMBREE_VERSION == 3
        if (single_ray) {
            for (auto& hit : rayhits1_res) of.write((char*)&hit.ray.tfar, sizeof(float));
        } else if (ray_width == 4) {
            for (auto& hit : rayhits4_res) {
                for (int i = 0; i < 4; i++)
                    of.write((char*)&hit.ray.tfar[i], sizeof(float));
            }
        } else {
            for (auto& hit : rayhits8_res) {
                for (int i = 0; i < 8; i++)
                    of.write((char*)&hit.ray.tfar[i], sizeof(float));
            }
        }
#else
        if (single_ray) {
            for (auto& hit : rays1_res) of.write((char*)&hit.tfar, sizeof(float));
        } else if (ray_width == 4) {
            for (auto& hit : rays4_res) {
                for (int i = 0; i < 4; i++)
                    of.write((char*)&hit.tfar[i], sizeof(float));
            }
        } else {
            for (auto& hit : rays8_res) {
                for (int i = 0; i < 8; i++)
                    of.write((char*)&hit.tfar[i], sizeof(float));
            }
        }
#endif
    }

    return 0;
}
