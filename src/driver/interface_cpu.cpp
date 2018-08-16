#include <unordered_map>
#include <memory>
#include <cstring>
#include <fstream>

#include <anydsl_runtime.hpp>
#include <lz4.h>

#include "interface.h"
#include "bvh.h"
#include "obj.h"
#include "image.h"

static void skip_buffer(std::istream& is) {
    size_t in_size = 0, out_size = 0;
    is.read((char*)&in_size,  sizeof(uint32_t));
    is.read((char*)&out_size, sizeof(uint32_t));
    is.seekg(out_size, std::ios::cur);
}

template <typename T>
static void read_buffer(std::istream& is, anydsl::Array<T>& array) {
    size_t in_size = 0, out_size = 0;
    is.read((char*)&in_size,  sizeof(uint32_t));
    is.read((char*)&out_size, sizeof(uint32_t));
    std::vector<char> input(out_size);
    array = std::move(anydsl::Array<T>(in_size));
    is.read(input.data(), input.size());
    LZ4_decompress_safe(input.data(), (char*)array.data(), input.size(), array.size());
}

template <typename Node, typename Tri>
struct Bvh {
    anydsl::Array<Node> nodes;
    anydsl::Array<Tri>  tris;
};

using Bvh4Tri4 = Bvh<Node4, Tri4>;
using Bvh8Tri4 = Bvh<Node8, Tri4>;

struct CpuInterface {
    std::unordered_map<std::string, Bvh4Tri4> bvh4_tri4;
    std::unordered_map<std::string, Bvh8Tri4> bvh8_tri4;
    std::unordered_map<std::string, anydsl::Array<uint8_t>> buffers;
    std::unordered_map<std::string, ImageRgba32> images;
    std::unique_ptr<float[]> film_pixels;
    size_t film_width;
    size_t film_height;

    static thread_local anydsl::Array<float> primary;
    static thread_local anydsl::Array<float> secondary;

    CpuInterface(size_t width, size_t height)
        : film_pixels(new float[3 * width * height])
        , film_width(width)
        , film_height(height)
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

    const Bvh4Tri4& load_bvh4_tri4(const std::string& filename) {
        auto it = bvh4_tri4.find(filename);
        if (it != bvh4_tri4.end())
            return it->second;
        return bvh4_tri4[filename] = std::move(load_bvh<Node4, Tri4>(filename));
    }

    const Bvh8Tri4& load_bvh8_tri4(const std::string& filename) {
        auto it = bvh8_tri4.find(filename);
        if (it != bvh8_tri4.end())
            return it->second;
        return bvh8_tri4[filename] = std::move(load_bvh<Node8, Tri4>(filename));
    }

    template <typename Node, typename Tri>
    Bvh<Node, Tri> load_bvh(const std::string& filename) {
        std::ifstream is(filename, std::ios::binary);
        if (!is)
            error("Cannot open BVH '", filename, "'");
        do {
            size_t node_size = 0, tri_size = 0;
            is.read((char*)&node_size, sizeof(uint32_t));
            is.read((char*)&tri_size, sizeof(uint32_t));
            if (node_size == sizeof(Node) &&
                tri_size  == sizeof(Tri)) {
                info("Loaded BVH file '", filename, "'");
                Bvh<Node, Tri> bvh;
                read_buffer(is, bvh.nodes);
                read_buffer(is, bvh.tris);
                return bvh;
            }
            skip_buffer(is);
            skip_buffer(is);
        } while (!is.eof());
        error("Invalid BVH file");
    }

    const anydsl::Array<uint8_t>& load_buffer(const std::string& filename) {
        auto it = buffers.find(filename);
        if (it != buffers.end())
            return it->second;
        std::ifstream is(filename, std::ios::binary);
        if (!is)
            error("Cannot open buffer '", filename, "'");
        anydsl::Array<uint8_t> array;
        read_buffer(is, array);
        info("Loaded buffer '", filename, "'");
        return buffers[filename] = std::move(array);
    }

    const ImageRgba32& load_png(const std::string& filename) {
        auto it = images.find(filename);
        if (it != images.end())
            return it->second;
        ImageRgba32 img;
        if (!::load_png(filename, img))
            error("Cannot load PNG file '", filename, "'");
        info("Loaded PNG file '", filename, "'");
        return images[filename] = std::move(img);
    }

    const ImageRgba32& load_jpg(const std::string& filename) {
        auto it = images.find(filename);
        if (it != images.end())
            return it->second;
        ImageRgba32 img;
        if (!::load_jpg(filename, img))
            error("Cannot load JPG file '", filename, "'");
        info("Loaded JPG file '", filename, "'");
        return images[filename] = std::move(img);
    }
};

thread_local anydsl::Array<float> CpuInterface::primary;
thread_local anydsl::Array<float> CpuInterface::secondary;

static std::unique_ptr<CpuInterface> cpu;

void setup_cpu_interface(size_t width, size_t height) {
    cpu.reset(new CpuInterface(width, height));
}

void cleanup_cpu_interface() {
    cpu->primary   = std::move(anydsl::Array<float>());
    cpu->secondary = std::move(anydsl::Array<float>());
    cpu.reset();
}

float* get_cpu_pixels() {
    return cpu->film_pixels.get();
}

extern "C" {

void rodent_cpu_get_film_data(float** pixels, int* width, int* height) {
    *pixels = cpu->film_pixels.get();
    *width  = cpu->film_width;
    *height = cpu->film_height;
}

void rodent_cpu_load_png(const char* file, uint8_t** pixels, int32_t* width, int32_t* height) {
    auto& img = cpu->load_png(file);
    *pixels = img.pixels.get();
    *width  = img.width;
    *height = img.height;
}

void rodent_cpu_load_jpg(const char* file, uint8_t** pixels, int32_t* width, int32_t* height) {
    auto& img = cpu->load_jpg(file);
    *pixels = img.pixels.get();
    *width  = img.width;
    *height = img.height;
}

uint8_t* rodent_cpu_load_buffer(const char* file) {
    auto& array = cpu->load_buffer(file);
    return const_cast<uint8_t*>(array.data());
}

void rodent_cpu_load_bvh4_tri4(const char* file, Node4** nodes, Tri4** tris) {
    auto& bvh = cpu->load_bvh4_tri4(file);
    *nodes = const_cast<Node4*>(bvh.nodes.data());
    *tris  = const_cast<Tri4*>(bvh.tris.data());
}

void rodent_cpu_load_bvh8_tri4(const char* file, Node8** nodes, Tri4** tris) {
    auto& bvh = cpu->load_bvh8_tri4(file);
    *nodes = const_cast<Node8*>(bvh.nodes.data());
    *tris  = const_cast<Tri4*>(bvh.tris.data());
}

static void cpu_get_ray_stream(RayStream& rays, float* ptr, size_t capacity) {
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

void rodent_cpu_get_primary_stream(PrimaryStream* primary, int size) {
    auto& array = cpu->primary_stream(size);
    auto capacity = array.size() / 21;
    primary->size = 0;
    cpu_get_ray_stream(primary->rays, array.data(), capacity);
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

void rodent_cpu_get_secondary_stream(SecondaryStream* secondary, int size) {
    auto& array = cpu->secondary_stream(size);
    auto capacity = array.size() / 13;
    secondary->size = 0;
    cpu_get_ray_stream(secondary->rays, array.data(), capacity);
    secondary->prim_id = (int*)array.data() + 9 * capacity;
    secondary->color_r = array.data() + 10 * capacity;
    secondary->color_g = array.data() + 11 * capacity;
    secondary->color_b = array.data() + 12 * capacity;
}

} // extern "C"

