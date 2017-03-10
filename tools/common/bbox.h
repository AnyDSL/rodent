#ifndef BBOX_H
#define BBOX_H

#include <cfloat>

struct BBox {
    float3 min, max;
    BBox() {}
    BBox(const float3& f) : min(f), max(f) {}
    BBox(const float3& min, const float3& max) : min(min), max(max) {}

    BBox& extend(const float3& f) {
        min = ::min(min, f);
        max = ::max(max, f);
        return *this;
    }

    BBox& extend(const BBox& bb) {
        min = ::min(min, bb.min);
        max = ::max(max, bb.max);
        return *this;
    }

    BBox& overlap(const BBox& bb) {
        min = ::max(min, bb.min);
        max = ::min(max, bb.max);
        return *this;
    }

    float half_area() const {
        const float3 len = max - min;
        const float kx = std::max(len.x, 0.0f);
        const float ky = std::max(len.y, 0.0f);
        const float kz = std::max(len.z, 0.0f);
        return kx * (ky + kz) + ky * kz;
    }

    bool is_empty() const {
        return min.x > max.x || min.y > max.y || min.z > max.z;
    }

    bool is_inside(const float3& f) const {
        return f.x >= min.x && f.y >= min.y && f.z >= min.z &&
               f.x <= max.x && f.y <= max.y && f.z <= max.z;
    }

    bool is_overlapping(const BBox& bb) const {
        return min.x <= bb.max.x && max.x >= bb.min.x &&
               min.y <= bb.max.y && max.y >= bb.min.y &&
               min.z <= bb.max.z && max.z >= bb.min.z;
    }

    bool is_included(const BBox& bb) const {
        return min.x >= bb.min.x && max.x <= bb.max.x &&
               min.y >= bb.min.y && max.y <= bb.max.y &&
               min.z >= bb.min.z && max.z <= bb.max.z;
    }

    bool is_strictly_included(const BBox& bb) const {
        return is_included(bb) &&
               (min.x > bb.min.x || max.x < bb.max.x ||
                min.y > bb.min.y || max.y < bb.max.y ||
                min.z > bb.min.z || max.z < bb.max.z);
    }

    static BBox empty() { return BBox(float3(FLT_MAX), float3(-FLT_MAX)); }
    static BBox full() { return BBox(float3(-FLT_MAX), float3(FLT_MAX)); }
};

#endif // BBOX_H
