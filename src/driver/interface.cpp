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

struct Interface {
    using DeviceImage = std::tuple<anydsl::Array<uint8_t>, int32_t, int32_t>;

    struct DeviceData {
        std::unordered_map<std::string, Bvh2Tri1> bvh2_tri1;
        std::unordered_map<std::string, Bvh4Tri4> bvh4_tri4;
        std::unordered_map<std::string, Bvh8Tri4> bvh8_tri4;
        std::unordered_map<std::string, anydsl::Array<uint8_t>> buffers;
        std::unordered_map<std::string, DeviceImage> images;
        anydsl::Array<float> film_pixels;
    };
    std::unordered_map<int32_t, DeviceData> devices;

    static thread_local anydsl::Array<float> primary;
    static thread_local anydsl::Array<float> secondary;

    anydsl::Array<float> host_pixels;
    size_t film_width;
    size_t film_height;

    Interface(size_t width, size_t height)
        : film_width(width)
        , film_height(height)
        , host_pixels(width * height * 3)
    {}

    anydsl::Array<float>& primary_stream(size_t size) {
        size_t capacity = (size & ~((1 << 5) - 1)) + 32; // round to 32
        if (primary.size() < capacity) {
            primary = std::move(anydsl::Array<float>(capacity * 21));
        }
        return primary;
    }

    anydsl::Array<float>& secondary_stream(size_t size) {
        size_t capacity = (size & ~((1 << 5) - 1)) + 32; // round to 32
        if (secondary.size() < capacity) {
            secondary = std::move(anydsl::Array<float>(capacity * 13));
        }
        return secondary;
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
        } while (!is.eof());
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

thread_local anydsl::Array<float> Interface::primary;
thread_local anydsl::Array<float> Interface::secondary;

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

extern "C" {

void rodent_get_film_data(int32_t dev, float** pixels, int* width, int* height) {
    if (dev != 0) {
        auto& device = interface->devices[dev];
        if (!device.film_pixels.size()) {
            auto film_size = interface->film_width * interface->film_height * 3;
            auto film_data = reinterpret_cast<float*>(anydsl_alloc(dev, sizeof(float) * film_size));
            device.film_pixels = std::move(anydsl::Array<float>(dev, film_data, film_size));
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

void rodent_get_primary_stream(PrimaryStream* primary, int size) {
    auto& array = interface->primary_stream(size);
    auto capacity = array.size() / 21;
    primary->size = 0;
    get_ray_stream(primary->rays, array.data(), capacity);
    primary->shader_id = (int*)array.data() +  9 * capacity;
    primary->geom_id   = (int*)array.data() + 10 * capacity;
    primary->prim_id   = (int*)array.data() + 11 * capacity;
    primary->t         = array.data() + 12 * capacity;
    primary->u         = array.data() + 13 * capacity;
    primary->v         = array.data() + 14 * capacity;
    primary->rnd       = (unsigned int*)array.data() + 15 * capacity;
    primary->mis       = array.data() + 16 * capacity;
    primary->contrib_r = array.data() + 17 * capacity;
    primary->contrib_g = array.data() + 18 * capacity;
    primary->contrib_b = array.data() + 19 * capacity;
    primary->depth     = (int*)array.data() + 20 * capacity;
}

void rodent_get_secondary_stream(SecondaryStream* secondary, int size) {
    auto& array = interface->secondary_stream(size);
    auto capacity = array.size() / 13;
    secondary->size = 0;
    get_ray_stream(secondary->rays, array.data(), capacity);
    secondary->prim_id = (int*)array.data() + 9 * capacity;
    secondary->color_r = array.data() + 10 * capacity;
    secondary->color_g = array.data() + 11 * capacity;
    secondary->color_b = array.data() + 12 * capacity;
}

void rodent_present(int32_t dev) {
    if (dev != 0)
        interface->present(dev);
}

} // extern "C"

