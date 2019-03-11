#include <iostream>
#include <fstream>
#include <memory>
#include <cmath>
#include <random>
#include <cstring>

#include "traversal.h"
#include "load_bvh.h"
#include "load_rays.h"
#include "driver/float3.h"
#include "driver/bbox.h"

class RayGen {
public:
    virtual ~RayGen() {}
    virtual void generate_rays(std::ofstream&) = 0;
};

class PrimaryRayGen : public RayGen {
public:
    PrimaryRayGen(const float3& eye,
                  const float3& dir,
                  const float3& up,
                  float fov,
                  int width,
                  int height)
        : eye_(eye)
        , dir_(normalize(dir))
        , width_(width)
        , height_(height)
    {
        right_ = normalize(cross(dir, up));
        up_ = normalize(cross(right_, dir));

        auto scale = std::tan(fov * (M_PI / 360.0f));
        right_ *= scale;
        up_    *= (float(height) / float(width)) * scale;
    }

    void generate_rays(std::ofstream& os) override {
        auto sx = 2.0f / width_;
        auto sy = 2.0f / height_;
        for (int i = height_ - 1; i >= 0; i--) {
            for (int j = 0; j < width_; j++) {
                auto kx = sx * (j + 0.5f) - 1.0f;
                auto ky = sy * (i + 0.5f) - 1.0f ;
                auto dir = dir_ + kx * right_ + ky * up_;
                os.write((char*)&eye_, sizeof(float3));
                os.write((char*)&dir, sizeof(float3));
            }
        }
    }

private:
    float3 eye_, dir_, right_, up_;
    int width_, height_;
};

class ShadowRayGen : public RayGen {
public:
    ShadowRayGen(const float3& light,
                 const anydsl::Array<Ray1>& rays,
                 const std::vector<float>& float_buffer)
        : light_(light)
        , rays_(rays)
        , float_buffer_(float_buffer)
    {}

    void generate_rays(std::ofstream& os) override {
        for (int i = 0; i < rays_.size(); i++) {
            auto org = float3(rays_[i].org[0], rays_[i].org[1], rays_[i].org[2]);
            auto dir = float3(rays_[i].dir[0], rays_[i].dir[1], rays_[i].dir[2]);
            auto hit = org + float_buffer_[i] * dir;
            auto new_dir = hit - light_;
            os.write((char*)&light_, sizeof(float3));
            os.write((char*)&new_dir, sizeof(float3));
        }
    }

private:
    float3 light_;
    const anydsl::Array<Ray1>& rays_;
    const std::vector<float>& float_buffer_;
};

class RandomRayGen : public RayGen {
public:
    RandomRayGen(const BBox& bounds, size_t count, int seed)
        : bounds_(bounds)
        , count_(count)
        , gen_(seed)
    {}

    void generate_rays(std::ofstream& os) override {
        std::uniform_real_distribution<float> dis(0.0f, 1.0f);
        auto extents = bounds_.max - bounds_.min;
        for (int i = 0; i < count_; i++) {
            auto rnd1 = bounds_.min + extents * float3(dis(gen_), dis(gen_), dis(gen_));
            auto rnd2 = bounds_.min + extents * float3(dis(gen_), dis(gen_), dis(gen_));
            auto dir = rnd2 - rnd1;
            os.write((char*)&rnd1, sizeof(float3));
            os.write((char*)&dir, sizeof(float3));
        }
    }

private:
    BBox bounds_;
    size_t count_;
    std::mt19937_64 gen_;
};

inline void usage() {
    std::cout << "Usage: ray_gen mode arguments output\n"
                 "Available modes:\n"
                 "  primary                      Generates primary rays from a pinhole camera\n"
                 "    eye-x  eye-y eye-z         Camera position\n"
                 "    dir-x  dir-y dir-z         Camera direction\n"
                 "    up-x   up-y  up-z          Up vector\n"
                 "    fov                        Field of view in degrees\n"
                 "    width height               Viewport dimensions\n"
                 "\n"
                 "  shadow                     Generates shadow rays for a point light\n"
                 "    light-x light-y light-z    Light position\n"
                 "    ray-file                   Primary ray file\n"
                 "    fbuf-file                  Result of traversal for the ray file\n"
                 "\n"
                 "  random                     Generates random rays within a scene\n"
                 "    bvh-file                   BVH file from which the scene bounds will be extracted\n"
                 "    ray-count                  Number of rays to generate\n"
                 "    seed                       Random generator seed\n";
}

static bool extract_bounds(const std::string& bvh_file, BBox& bounds) {
    anydsl::Array<Node4> nodes;
    anydsl::Array<Tri4>  tris;
    if (!load_bvh(bvh_file, nodes, tris, BvhType::BVH4_TRI4, anydsl::Platform::Host, anydsl::Device(0))) return false;
    bounds = BBox::empty();
    for (int i = 0; i < 4; i++) {
        bounds.min = min(bounds.min, float3(nodes[0].bounds[0][i], nodes[0].bounds[2][i], nodes[0].bounds[4][i]));
        bounds.max = max(bounds.max, float3(nodes[0].bounds[1][i], nodes[0].bounds[3][i], nodes[0].bounds[5][i]));
    }
    return true;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Not enough arguments" << std::endl;
        return 1;
    }

    std::unique_ptr<RayGen> ray_gen;
    std::string output;
    if (!strcmp(argv[1], "primary")) {
        if (argc != 15) {
            std::cerr << "Incorrect number of arguments in primary mode" << std::endl;
            return 1;
        }

        auto eye = float3(strtof(argv[2], nullptr), strtof(argv[3], nullptr), strtof(argv[ 4], nullptr));
        auto dir = float3(strtof(argv[5], nullptr), strtof(argv[6], nullptr), strtof(argv[ 7], nullptr));
        auto up  = float3(strtof(argv[8], nullptr), strtof(argv[9], nullptr), strtof(argv[10], nullptr));
        auto fov = strtof(argv[11], nullptr);
        auto width  = strtol(argv[12], nullptr, 10);
        auto height = strtol(argv[13], nullptr, 10);
        output = argv[14];

        ray_gen.reset(new PrimaryRayGen(eye, dir, up, fov, width, height));
    } else if (!strcmp(argv[1], "shadow")) {
        if (argc != 10) {
            std::cerr << "Incorrect number of arguments in shadow mode" << std::endl;
            return 1;
        }

        auto light = float3(strtof(argv[2], nullptr), strtof(argv[3], nullptr), strtof(argv[4], nullptr));
        std::string ray_file  = argv[5];
        std::string fbuf_file = argv[6];
        auto width  = strtol(argv[7], nullptr, 10);
        auto height = strtol(argv[8], nullptr, 10);
        output = argv[9];

        anydsl::Array<Ray1> rays;
        if (!load_rays(ray_file, rays, 0.0f, 1.0f, anydsl::Platform::Host, anydsl::Device(0))) {
            std::cerr << "Cannot load rays" << std::endl;
            return 1;
        }
        
        std::ifstream fbuf(fbuf_file, std::ifstream::binary);
        std::vector<float> float_buffer(rays.size());
        if (!fbuf.read((char*)float_buffer.data(), rays.size() * sizeof(float))) {
            std::cerr << "Cannot load result of traversal" << std::endl;
            return 1;
        }

        ray_gen.reset(new ShadowRayGen(light, rays, float_buffer));
    } else if (!strcmp(argv[1], "random")) {
        if (argc != 6) {
            std::cerr << "Incorrect number of arguments in random mode" << std::endl;
            return 1;
        }

        std::string bvh_file  = argv[2];
        auto ray_count  = strtol(argv[3], nullptr, 10);
        auto seed = strtol(argv[4], nullptr, 10);
        output = argv[5];

        BBox bounds;
        if (!extract_bounds(bvh_file, bounds)) {
            std::cerr << "Cannot extract scene bounds" << std::endl;
            return 1;
        }

        ray_gen.reset(new RandomRayGen(bounds, ray_count, seed));
    } else if (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help")) {
        usage();
        return 0;
    } else {
        std::cerr << "Unknown mode" << std::endl;
        return 1;
    }

    std::ofstream os(output, std::ofstream::binary);
    ray_gen->generate_rays(os);

    return 0;
}
