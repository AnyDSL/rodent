#include <unordered_map>
#include <memory>
#include <cstring>
#include <fstream>

#include <anydsl_runtime.hpp>

#include "interface.h"
#include "bvh.h"
#include "obj.h"
#include "image.h"
#include "buffer.h"

template <typename Node, typename Tri>
struct Bvh {
    anydsl::Array<Node> nodes;
    anydsl::Array<Tri>  tris;
};

using Bvh2Tri1 = Bvh<Node2, Tri1>;
using Bvh4Tri4 = Bvh<Node4, Tri4>;
using Bvh8Tri4 = Bvh<Node8, Tri4>;

#ifdef ENABLE_EMBREE_DEVICE
#include <embree3/rtcore.h>

template <int N>
struct EmbreeIntersect {};

template <>
struct EmbreeIntersect<8> {
    static void intersect(const int* valid, RTCScene scene, RTCRayHitNt<8>& ray_hit) {
        RTCIntersectContext context;
        context.flags = RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT;
        context.filter = NULL;
        context.instID[0] = RTC_INVALID_GEOMETRY_ID;
        rtcIntersect8(valid, scene, &context, reinterpret_cast<RTCRayHit8*>(&ray_hit));
    }

    static void occluded(const int* valid, RTCScene scene, RTCRayNt<8>& ray) {
        RTCIntersectContext context;
        context.flags = RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT;
        context.filter = NULL;
        context.instID[0] = RTC_INVALID_GEOMETRY_ID;
        rtcOccluded8(valid, scene, &context, reinterpret_cast<RTCRay8*>(&ray));
    }
};

template <>
struct EmbreeIntersect<4> {
    static void intersect(const int* valid, RTCScene scene, RTCRayHitNt<4>& ray_hit) {
        RTCIntersectContext context;
        context.flags = RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT;
        context.filter = NULL;
        context.instID[0] = RTC_INVALID_GEOMETRY_ID;
        rtcIntersect4(valid, scene, &context, reinterpret_cast<RTCRayHit4*>(&ray_hit));
    }

    static void occluded(const int* valid, RTCScene scene, RTCRayNt<4>& ray) {
        RTCIntersectContext context;
        context.flags = RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT;
        context.filter = NULL;
        context.instID[0] = RTC_INVALID_GEOMETRY_ID;
        rtcOccluded4(valid, scene, &context, reinterpret_cast<RTCRay4*>(&ray));
    }
};

struct EmbreeDevice {
    std::vector<uint32_t> indices;
    std::vector<float3>   vertices;
    RTCDevice device;
    RTCScene scene;

    static void error_handler(void*, const RTCError code, const char* str) {
        if (code == RTC_ERROR_NONE)
            return;
        std::cerr << "Embree error ";
        switch (code) {
            case RTC_ERROR_UNKNOWN:             std::cerr << "RTC_ERROR_UNKNOWN"; break;
            case RTC_ERROR_INVALID_ARGUMENT:    std::cerr << "RTC_ERROR_INVALID_ARGUMENT"; break;
            case RTC_ERROR_INVALID_OPERATION:   std::cerr << "RTC_ERROR_INVALID_OPERATION"; break;
            case RTC_ERROR_OUT_OF_MEMORY:       std::cerr << "RTC_ERROR_OUT_OF_MEMORY"; break;
            case RTC_ERROR_UNSUPPORTED_CPU:     std::cerr << "RTC_ERROR_UNSUPPORTED_CPU"; break;
            case RTC_ERROR_CANCELLED:           std::cerr << "RTC_ERROR_CANCELLED"; break;
            default: break;
        }
        if (str) std::cerr << ": " << str;
        std::cerr << std::endl;
        abort();
    }

    EmbreeDevice() {
        read_buffer("data/vertices.bin", vertices);
        read_buffer("data/indices.bin", indices);

        if (vertices.empty() || indices.empty())
            error("Cannot build scene due to missing data files");

        device = rtcNewDevice(nullptr);
        if (!device)
            error("Cannot initialize Embree");
        rtcSetDeviceErrorFunction(device, error_handler, nullptr);
        auto mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
        scene = rtcNewScene(device);

        auto vertex_ptr = (float4*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(float) * 4, vertices.size());
        for (auto& v : vertices)
            *(vertex_ptr++) = float4(v, 1.0f);
        auto index_ptr = (uint32_t*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(uint32_t) * 4, indices.size() / 4);
        for (auto& i : indices)
            *(index_ptr++) = i;

        rtcCommitGeometry(mesh);
        rtcAttachGeometry(scene, mesh);
        rtcCommitScene(scene);

        info("Embree device initialized successfully");
    }

    ~EmbreeDevice() {
        rtcReleaseScene(scene);
        rtcReleaseDevice(device);
    }

    template <int N>
    void load_ray(RTCRayHitNt<N>& ray_hit, const RayStream& rays, size_t i, size_t size) {
        size_t n = std::min(size, i + N);
        for (size_t j = i, k = 0; j < n; ++j, ++k) {
            ray_hit.ray.org_x[k] = rays.org_x[j];
            ray_hit.ray.org_y[k] = rays.org_y[j];
            ray_hit.ray.org_z[k] = rays.org_z[j];

            ray_hit.ray.dir_x[k] = rays.dir_x[j];
            ray_hit.ray.dir_y[k] = rays.dir_y[j];
            ray_hit.ray.dir_z[k] = rays.dir_z[j];

            ray_hit.ray.tnear[k] = rays.tmin[j];
            ray_hit.ray.tfar[k]  = rays.tmax[j];
            ray_hit.ray.mask[k]  = 0xFFFFFFFF;
            ray_hit.ray.id[k]    = i;
            ray_hit.ray.flags[k] = 0;

            ray_hit.hit.primID[k] = -1;
            ray_hit.hit.geomID[k] = -1;
        }
    }

    template <int N>
    void load_ray(RTCRayNt<N>& ray, const RayStream& rays, size_t i, size_t size) {
        size_t n = std::min(size, i + N);
        for (size_t j = i, k = 0; j < n; ++j, ++k) {
            ray.org_x[k] = rays.org_x[j];
            ray.org_y[k] = rays.org_y[j];
            ray.org_z[k] = rays.org_z[j];

            ray.dir_x[k] = rays.dir_x[j];
            ray.dir_y[k] = rays.dir_y[j];
            ray.dir_z[k] = rays.dir_z[j];

            ray.tnear[k] = rays.tmin[j];
            ray.tfar[k]  = rays.tmax[j];
            ray.mask[k]  = 0xFFFFFFFF;
            ray.id[k]    = i;
            ray.flags[k] = 0;
        }
    }

    template <int N>
    void store_hit(const RTCRayHitNt<N>& ray_hit, PrimaryStream& primary, size_t i, int32_t invalid_id) {
        size_t n = std::min(primary.size, int32_t(i + N));
        for (size_t j = i, k = 0; j < n; ++j, ++k) {
            auto prim_id = ray_hit.hit.primID[k];
            primary.geom_id[j] = prim_id == RTC_INVALID_GEOMETRY_ID ? invalid_id : indices[prim_id * 4 + 3];
            primary.prim_id[j] = prim_id;
            primary.t[j]       = ray_hit.ray.tfar[k];
            primary.u[j]       = ray_hit.hit.u[k];
            primary.v[j]       = ray_hit.hit.v[k];
        }
    }

    template <int N>
    void store_hit(const RTCRayNt<N>& ray, SecondaryStream& secondary, size_t i) {
        size_t n = std::min(secondary.size, int32_t(i + N));
        for (size_t j = i, k = 0; j < n; ++j, ++k) {
            secondary.prim_id[j] = ray.tfar[k] < 0 ? 0 : -1;
        }
    }

    template <int N>
    void intersect(PrimaryStream& primary, int32_t invalid_id) {
        alignas(32) int valid[N];
        alignas(32) RTCRayHitNt<N> ray_hit;
        memset(valid, 0xFF, sizeof(int) * N);
        for (size_t i = 0; i < primary.size; i += N) {
            load_ray(ray_hit, primary.rays, i, primary.size);
            EmbreeIntersect<N>::intersect(valid, scene, ray_hit);
            store_hit(ray_hit, primary, i, invalid_id);
        }
    }

    template <int N>
    void intersect(SecondaryStream& secondary) {
        alignas(32) int valid[N];
        alignas(32) RTCRayNt<N> ray;
        memset(valid, 0xFF, sizeof(int) * N);
        for (size_t i = 0; i < secondary.size; i += N) {
            load_ray(ray, secondary.rays, i, secondary.size);
            EmbreeIntersect<N>::occluded(valid, scene, ray);
            store_hit(ray, secondary, i);
        }
    }
};
#endif

struct Interface {
    using DeviceImage = std::tuple<anydsl::Array<uint8_t>, int32_t, int32_t>;

    struct DeviceData {
        std::unordered_map<std::string, Bvh2Tri1> bvh2_tri1;
        std::unordered_map<std::string, Bvh4Tri4> bvh4_tri4;
        std::unordered_map<std::string, Bvh8Tri4> bvh8_tri4;
        std::unordered_map<std::string, anydsl::Array<uint8_t>> buffers;
        std::unordered_map<std::string, DeviceImage> images;
        anydsl::Array<int32_t> tmp_buffer;
        anydsl::Array<float> first_primary;
        anydsl::Array<float> second_primary;
        anydsl::Array<float> secondary;
        anydsl::Array<float> film_pixels;
    };
    std::unordered_map<int32_t, DeviceData> devices;

    static thread_local anydsl::Array<float> cpu_primary;
    static thread_local anydsl::Array<float> cpu_secondary;

#ifdef ENABLE_EMBREE_DEVICE
    EmbreeDevice embree_device;
#endif

    anydsl::Array<float> host_pixels;
    size_t film_width;
    size_t film_height;

    Interface(size_t width, size_t height)
        : film_width(width)
        , film_height(height)
        , host_pixels(width * height * 3)
    {}

    template <typename T>
    anydsl::Array<T>& resize_array(int32_t dev, anydsl::Array<T>& array, size_t size, size_t multiplier) {
        auto capacity = (size & ~((1 << 5) - 1)) + 32; // round to 32
        if (array.size() < capacity) {
            auto n = capacity * multiplier;
            array = std::move(anydsl::Array<T>(dev, reinterpret_cast<T*>(anydsl_alloc(dev, sizeof(T) * n)), n));
        }
        return array;
    }

    anydsl::Array<float>& cpu_primary_stream(size_t size) {
        return resize_array(0, cpu_primary, size, 20);
    }

    anydsl::Array<float>& cpu_secondary_stream(size_t size) {
        return resize_array(0, cpu_secondary, size, 13);
    }

    anydsl::Array<float>& gpu_first_primary_stream(int32_t dev, size_t size) {
        return resize_array(dev, devices[dev].first_primary, size, 20);
    }

    anydsl::Array<float>& gpu_second_primary_stream(int32_t dev, size_t size) {
        return resize_array(dev, devices[dev].second_primary, size, 20);
    }

    anydsl::Array<float>& gpu_secondary_stream(int32_t dev, size_t size) {
        return resize_array(dev, devices[dev].secondary, size, 13);
    }

    anydsl::Array<int32_t>& gpu_tmp_buffer(int32_t dev, size_t size) {
        return resize_array(dev, devices[dev].tmp_buffer, size, 1);
    }

    const Bvh2Tri1& load_bvh2_tri1(int32_t dev, const std::string& filename) {
        auto& bvh2_tri1 = devices[dev].bvh2_tri1;
        auto it = bvh2_tri1.find(filename);
        if (it != bvh2_tri1.end())
            return it->second;
        return bvh2_tri1[filename] = std::move(load_bvh<Node2, Tri1>(dev, filename));
    }

    const Bvh4Tri4& load_bvh4_tri4(int32_t dev, const std::string& filename) {
        auto& bvh4_tri4 = devices[dev].bvh4_tri4;
        auto it = bvh4_tri4.find(filename);
        if (it != bvh4_tri4.end())
            return it->second;
        return bvh4_tri4[filename] = std::move(load_bvh<Node4, Tri4>(dev, filename));
    }

    const Bvh8Tri4& load_bvh8_tri4(int32_t dev, const std::string& filename) {
        auto& bvh8_tri4 = devices[dev].bvh8_tri4;
        auto it = bvh8_tri4.find(filename);
        if (it != bvh8_tri4.end())
            return it->second;
        return bvh8_tri4[filename] = std::move(load_bvh<Node8, Tri4>(dev, filename));
    }

    template <typename T>
    anydsl::Array<T> copy_to_device(int32_t dev, const T* data, size_t n) {
        anydsl::Array<T> array(dev, reinterpret_cast<T*>(anydsl_alloc(dev, n * sizeof(T))), n);
        anydsl_copy(0, data, 0, dev, array.data(), 0, sizeof(T) * n);
        return array;
    }

    template <typename T>
    anydsl::Array<T> copy_to_device(int32_t dev, const std::vector<T>& host) {
        return copy_to_device(dev, host.data(), host.size());
    }

    DeviceImage copy_to_device(int32_t dev, const ImageRgba32& img) {
        return DeviceImage(copy_to_device(dev, img.pixels.get(), img.width * img.height * 4), img.width, img.height);
    }

    template <typename Node, typename Tri>
    Bvh<Node, Tri> load_bvh(int32_t dev, const std::string& filename) {
        std::ifstream is(filename, std::ios::binary);
        if (!is)
            error("Cannot open BVH '", filename, "'");
        do {
            size_t node_size = 0, tri_size = 0;
            is.read((char*)&node_size, sizeof(uint32_t));
            is.read((char*)&tri_size,  sizeof(uint32_t));
            if (node_size == sizeof(Node) &&
                tri_size  == sizeof(Tri)) {
                info("Loaded BVH file '", filename, "'");
                std::vector<Node> nodes;
                std::vector<Tri>  tris;
                read_buffer(is, nodes);
                read_buffer(is, tris);
                return Bvh<Node, Tri> { std::move(copy_to_device(dev, nodes)), std::move(copy_to_device(dev, tris)) };
            }
            skip_buffer(is);
            skip_buffer(is);
        } while (!is.eof() && is);
        error("Invalid BVH file");
    }

    const anydsl::Array<uint8_t>& load_buffer(int32_t dev, const std::string& filename) {
        auto& buffers = devices[dev].buffers;
        auto it = buffers.find(filename);
        if (it != buffers.end())
            return it->second;
        std::ifstream is(filename, std::ios::binary);
        if (!is)
            error("Cannot open buffer '", filename, "'");
        std::vector<uint8_t> vector;
        read_buffer(is, vector);
        info("Loaded buffer '", filename, "'");
        return buffers[filename] = std::move(copy_to_device(dev, vector));
    }

    const DeviceImage& load_png(int32_t dev, const std::string& filename) {
        auto& images = devices[dev].images;
        auto it = images.find(filename);
        if (it != images.end())
            return it->second;
        ImageRgba32 img;
        if (!::load_png(filename, img))
            error("Cannot load PNG file '", filename, "'");
        info("Loaded PNG file '", filename, "'");
        return images[filename] = std::move(copy_to_device(dev, img));
    }

    const DeviceImage& load_jpg(int32_t dev, const std::string& filename) {
        auto& images = devices[dev].images;
        auto it = images.find(filename);
        if (it != images.end())
            return it->second;
        ImageRgba32 img;
        if (!::load_jpg(filename, img))
            error("Cannot load JPG file '", filename, "'");
        info("Loaded JPG file '", filename, "'");
        return images[filename] = std::move(copy_to_device(dev, img));
    }

    void present(int32_t dev) {
        anydsl::copy(devices[dev].film_pixels, host_pixels);
    }
    void clear() {
        std::fill(host_pixels.begin(), host_pixels.end(), 0.0f);
        for (auto& pair : devices) {
            auto& device_pixels = devices[pair.first].film_pixels;
            if (device_pixels.size())
                anydsl::copy(host_pixels, device_pixels);
        }
    }
};

thread_local anydsl::Array<float> Interface::cpu_primary;
thread_local anydsl::Array<float> Interface::cpu_secondary;

static std::unique_ptr<Interface> interface;

void setup_interface(size_t width, size_t height) {
    interface.reset(new Interface(width, height));
}

void cleanup_interface() {
    interface.reset();
}

float* get_pixels() {
    return interface->host_pixels.data();
}

void clear_pixels() {
    return interface->clear();
}

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

inline void get_secondary_stream(SecondaryStream& secondary, float* ptr, size_t capacity) {
    get_ray_stream(secondary.rays, ptr, capacity);
    secondary.prim_id = (int*)ptr + 9 * capacity;
    secondary.color_r = ptr + 10 * capacity;
    secondary.color_g = ptr + 11 * capacity;
    secondary.color_b = ptr + 12 * capacity;
    secondary.size = 0;
}

extern "C" {

void rodent_get_film_data(int32_t dev, float** pixels, int32_t* width, int32_t* height) {
    if (dev != 0) {
        auto& device = interface->devices[dev];
        if (!device.film_pixels.size()) {
            auto film_size = interface->film_width * interface->film_height * 3;
            auto film_data = reinterpret_cast<float*>(anydsl_alloc(dev, sizeof(float) * film_size));
            device.film_pixels = std::move(anydsl::Array<float>(dev, film_data, film_size));
            anydsl::copy(interface->host_pixels, device.film_pixels);
        }
        *pixels = device.film_pixels.data();
    } else {
        *pixels = interface->host_pixels.data();
    }
    *width  = interface->film_width;
    *height = interface->film_height;
}

void rodent_load_png(int32_t dev, const char* file, uint8_t** pixels, int32_t* width, int32_t* height) {
    auto& img = interface->load_png(dev, file);
    *pixels = const_cast<uint8_t*>(std::get<0>(img).data());
    *width  = std::get<1>(img);
    *height = std::get<2>(img);
}

void rodent_load_jpg(int32_t dev, const char* file, uint8_t** pixels, int32_t* width, int32_t* height) {
    auto& img = interface->load_jpg(dev, file);
    *pixels = const_cast<uint8_t*>(std::get<0>(img).data());
    *width  = std::get<1>(img);
    *height = std::get<2>(img);
}

uint8_t* rodent_load_buffer(int32_t dev, const char* file) {
    auto& array = interface->load_buffer(dev, file);
    return const_cast<uint8_t*>(array.data());
}

void rodent_load_bvh2_tri1(int32_t dev, const char* file, Node2** nodes, Tri1** tris) {
    auto& bvh = interface->load_bvh2_tri1(dev, file);
    *nodes = const_cast<Node2*>(bvh.nodes.data());
    *tris  = const_cast<Tri1*>(bvh.tris.data());
}

void rodent_load_bvh4_tri4(int32_t dev, const char* file, Node4** nodes, Tri4** tris) {
    auto& bvh = interface->load_bvh4_tri4(dev, file);
    *nodes = const_cast<Node4*>(bvh.nodes.data());
    *tris  = const_cast<Tri4*>(bvh.tris.data());
}

void rodent_load_bvh8_tri4(int32_t dev, const char* file, Node8** nodes, Tri4** tris) {
    auto& bvh = interface->load_bvh8_tri4(dev, file);
    *nodes = const_cast<Node8*>(bvh.nodes.data());
    *tris  = const_cast<Tri4*>(bvh.tris.data());
}

void rodent_cpu_get_primary_stream(PrimaryStream* primary, int32_t size) {
    auto& array = interface->cpu_primary_stream(size);
    get_primary_stream(*primary, array.data(), array.size() / 20);
}

void rodent_cpu_get_secondary_stream(SecondaryStream* secondary, int32_t size) {
    auto& array = interface->cpu_secondary_stream(size);
    get_secondary_stream(*secondary, array.data(), array.size() / 13);
}

void rodent_gpu_get_tmp_buffer(int32_t dev, int32_t** buf, int32_t size) {
    *buf = interface->gpu_tmp_buffer(dev, size).data();
}

void rodent_gpu_get_first_primary_stream(int32_t dev, PrimaryStream* primary, int32_t size) {
    auto& array = interface->gpu_first_primary_stream(dev, size);
    get_primary_stream(*primary, array.data(), array.size() / 20);
}

void rodent_gpu_get_second_primary_stream(int32_t dev, PrimaryStream* primary, int32_t size) {
    auto& array = interface->gpu_second_primary_stream(dev, size);
    get_primary_stream(*primary, array.data(), array.size() / 20);
}

void rodent_gpu_get_secondary_stream(int32_t dev, SecondaryStream* secondary, int32_t size) {
    auto& array = interface->gpu_secondary_stream(dev, size);
    get_secondary_stream(*secondary, array.data(), array.size() / 13);
}

#ifdef ENABLE_EMBREE_DEVICE
void rodent_cpu_intersect_primary_embree(PrimaryStream* primary, int32_t invalid_id) {
    interface->embree_device.intersect<8>(*primary, invalid_id);
}

void rodent_cpu_intersect_secondary_embree(SecondaryStream* secondary) {
    interface->embree_device.intersect<8>(*secondary);
}
#endif

void rodent_present(int32_t dev) {
    if (dev != 0)
        interface->present(dev);
}

} // extern "C"

