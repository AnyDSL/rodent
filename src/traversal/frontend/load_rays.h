#ifndef LOAD_RAYS_H
#define LOAD_RAYS_H

#include <fstream>
#include <anydsl_runtime.hpp>

inline bool load_rays(const std::string& filename,
                      anydsl::Array<RayAoS>& rays,
                      float tmin, float tmax) {
    std::ifstream in(filename, std::ifstream::binary);
    if (!in) return false;

    in.seekg(0, std::ios_base::end);
    auto size = in.tellg();
    in.seekg(0, std::ios_base::beg);

    if (size % (sizeof(float) * 6) != 0) return false;

    rays = std::move(anydsl::Array<RayAoS>(size / (sizeof(float) * 6)));
    for (size_t i = 0; i < rays.size(); i++) {
        float ray[6];
        in.read((char*)ray, sizeof(float) * 6);
        rays[i].org[0] = ray[0];
        rays[i].org[1] = ray[1];
        rays[i].org[2] = ray[2];
        rays[i].dir[0] = ray[3];
        rays[i].dir[1] = ray[4];
        rays[i].dir[2] = ray[5];
        rays[i].tmin = tmin;
        rays[i].tmax = tmax;
    }
    return true;
}

#endif // LOAD_RAYS_H
