#include <unordered_map>
#include <memory>
#include <string>

#include <anydsl_runtime.hpp>

#include "interface.h"

class Interface {
public:
    Interface(int32_t dev, size_t width, size_t height)
        : dev_(dev), width_(width), height_(height)
    {}

    ~Interface() {
        if (film_data_)
            anydsl_release(dev_, film_data_->pixels);
    }

    PixelData image(const char* file) {
        auto it = images_.find(file);
        if (it != images_.end())
            return it->second;
        return images_[file] = load_image(dev_, file);
    }

    TriMesh tri_mesh(const char* file) {
        auto it = tri_meshes_.find(file);
        if (it != tri_meshes_.end())
            return it->second;
        return tri_meshes_[file] = load_tri_mesh(dev_, file);
    }

    PixelData film_data() {
        if (film_data_)
            return *film_data_;
        auto pixels = anydsl_alloc(dev_, sizeof(Color) * width_ * height_);
        film_data_.reset(new PixelData {
            reinterpret_cast<Color*>(pixels),
            static_cast<int>(width_),
            static_cast<int>(height_)
        });
    }

    Bvh8Tri4 bvh() {
        if (bvh_)
            return *bvh_;
        bvh_ = build_bvh(dev_);
    }

private:
    std::unordered_map<std::string, PixelData> images_;
    std::unordered_map<std::string, TriMesh>   tri_meshes_;
    std::unique_ptr<PixelData> film_data_;
    std::unique_ptr<Bvh8Tri4>  bvh_;
    size_t width_, height_;
    int32_t dev_;
};

// CPU Interface -------------------------------------------------------------------

static std::unique_ptr<Interface> cpu_interface;

void setup_cpu_interface(size_t width, size_t height) {
    cpu_interface.reset(new Interface(0, width, height));
}

extern "C" Bvh8Tri4 rodent_cpu_get_bvh8_tri4() {
    return cpu_interface->bvh();
}

extern "C" PixelData rodent_cpu_get_film_data() {
    return cpu_interface->film_data();
}

extern "C" TriMesh rodent_cpu_load_tri_mesh(const char* file) {
    return cpu_interface->tri_mesh(file);
}

extern "C" PixelData rodent_cpu_load_pixel_data(const char* file) {
    return cpu_interface->image(file);
}

// GPU Interface -------------------------------------------------------------------

// TODO
