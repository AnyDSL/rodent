#ifndef LOAD_RAYS_H
#define LOAD_RAYS_H

#include <fstream>
#include <anydsl_runtime.hpp>

inline bool load_rays(const std::string& filename,
                      anydsl::Array<RayAoS>& rays,
                      float tmin, float tmax,
                      bool use_gpu) {
    std::ifstream in(filename, std::ifstream::binary);
    if (!in) return false;

    in.seekg(0, std::ios_base::end);
    auto size = in.tellg();
    in.seekg(0, std::ios_base::beg);

    if (size % (sizeof(float) * 6) != 0) return false;

    auto host_rays = std::move(anydsl::Array<RayAoS>(size / (sizeof(float) * 6)));
    for (size_t i = 0; i < host_rays.size(); i++) {
        float ray[6];
        in.read((char*)ray, sizeof(float) * 6);
        host_rays[i].org[0] = ray[0];
        host_rays[i].org[1] = ray[1];
        host_rays[i].org[2] = ray[2];
        host_rays[i].dir[0] = ray[3];
        host_rays[i].dir[1] = ray[4];
        host_rays[i].dir[2] = ray[5];
        host_rays[i].tmin = tmin;
        host_rays[i].tmax = tmax;
    }

    if (use_gpu) {
        rays = std::move(anydsl::Array<RayAoS>(anydsl::Platform::Cuda, anydsl::Device(0), host_rays.size()));
        anydsl::copy(host_rays, rays);
    } else {
        rays = std::move(host_rays);
    }
    return true;
}

#endif // LOAD_RAYS_H
