#ifndef COLOR_H
#define COLOR_H

#include "float3.h"
#include "float4.h"

struct rgba;

struct rgb : public float3 {
    rgb() {}
    rgb(const float3& rgb) : float3(rgb) {}
    rgb(float r, float g, float b) : float3(r, g, b) {}
    explicit rgb(float x) : float3(x) {}
    explicit rgb(const rgba& rgba);

    rgb& operator += (const rgb& p) {
        *this = *this + p;
        return *this;
    }
};

struct rgba : public float4 {
    rgba() {}
    rgba(const float4& rgba) : float4(rgba) {}
    rgba(float r, float g, float b, float a) : float4(r, g, b, a) {}
    explicit rgba(float x) : float4(x) {}
    explicit rgba(const rgb& rgb, float a) : float4(rgb, a) {}

    rgba& operator += (const rgba& p) {
        *this = *this + p;
        return *this;
    }
};

inline rgb::rgb(const rgba& rgba) : float3(rgba) {}

inline rgb gamma(const rgb& c, float g = 0.5f) {
    return rgb(std::pow(c.x, g), std::pow(c.y, g), std::pow(c.z, g));
}

inline rgba gamma(const rgba& c, float g = 0.5f) {
    return rgba(std::pow(c.x, g), std::pow(c.y, g), std::pow(c.z, g), c.w);
}

inline rgb clamp(const rgb& val, const rgb& min, const rgb& max) {
    return rgb(clamp(val.x, min.x, max.x),
               clamp(val.y, min.y, max.y),
               clamp(val.z, min.z, max.z));
}

inline rgba clamp(const rgba& val, const rgba& min, const rgba& max) {
    return rgba(clamp(val.x, min.x, max.x),
                clamp(val.y, min.y, max.y),
                clamp(val.z, min.z, max.z),
                clamp(val.w, min.w, max.w));
}

#endif // COLOR_H
