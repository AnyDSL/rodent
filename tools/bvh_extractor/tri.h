#ifndef TRI_H
#define TRI_H

#include "float3.h"
#include "bbox.h"

struct Tri {
    float3 v0, v1, v2;

    Tri() {}
    Tri(const float3& v0, const float3& v1, const float3& v2)
        : v0(v0), v1(v1), v2(v2)
    {}

    float3& operator[] (int i) { return i == 0 ? v0 : (i == 1 ? v1 : v2); }
    const float3& operator[] (int i) const { return i == 0 ? v0 : (i == 1 ? v1 : v2); }

    float area() const { return length(cross(v1 - v0, v2 - v0)) / 2; }

    /// Computes the triangle bounding box.
    void compute_bbox(BBox& bb) const {
        bb.min = min(v0, min(v1, v2));
        bb.max = max(v0, max(v1, v2));
    }

    /// Splits the triangle along one axis and returns the resulting two bounding boxes.
    void compute_split(BBox& left_bb, BBox& right_bb, int axis, float split) const {
        left_bb = BBox::empty();
        right_bb = BBox::empty();

        const float3& e0 = v1 - v0;
        const float3& e1 = v2 - v1;
        const float3& e2 = v0 - v2;

        const bool left0 = v0[axis] <= split;
        const bool left1 = v1[axis] <= split;
        const bool left2 = v2[axis] <= split;

        if (left0) left_bb.extend(v0);
        if (left1) left_bb.extend(v1);
        if (left2) left_bb.extend(v2);

        if (!left0) right_bb.extend(v0);
        if (!left1) right_bb.extend(v1);
        if (!left2) right_bb.extend(v2);

        if (left0 ^ left1) {
            const float3& p = clip_edge(axis, split, v0, e0);
            left_bb.extend(p);
            right_bb.extend(p);
        }
        if (left1 ^ left2) {
            const float3& p = clip_edge(axis, split, v1, e1);
            left_bb.extend(p);
            right_bb.extend(p);
        }
        if (left2 ^ left0) {
            const float3& p = clip_edge(axis, split, v2, e2);
            left_bb.extend(p);
            right_bb.extend(p);
        }
    }

private:
    static float3 clip_edge(int axis, float plane, const float3& p, const float3& edge) {
        const float t = (plane - p[axis]) / (edge[axis]);
        return p + t * edge;
    }
};

#endif // TRI_H
