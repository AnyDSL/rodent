#ifndef FLOAT4_H
#define FLOAT4_H

#include <cmath>
#include "common.h"
#include "float2.h"
#include "float3.h"

struct float4 {
    union {
        struct { float x, y, z, w; };
        float values[4];
    };

    float4() {}
    explicit float4(float x) : x(x), y(x), z(x), w(x) {}
    float4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    float4(const float3& xyz, float w) : x(xyz.x), y(xyz.y), z(xyz.z), w(w) {}
    float4(float x, const float3& yzw) : x(x), y(yzw.x), z(yzw.y), w(yzw.z) {}
    float4(const float2& xy, float z, float w) : x(xy.x), y(xy.y), z(z), w(w) {}
    float4(float x, const float2& yz, float w) : x(x), y(yz.x), z(yz.y), w(w) {}
    float4(float x, float y, const float2& zw) : x(x), y(y), z(zw.x), w(zw.y) {}
    float4(const float2& xy, const float2& zw) : x(xy.x), y(xy.y), z(zw.x), w(zw.y) {}

    bool operator == (const float4& other) const {
        return x == other.x && y == other.y && z == other.z && w != other.w;
    }

    bool operator != (const float4& other) const {
        return x != other.x || y != other.y || z != other.z || w != other.w;
    }

    float operator [] (size_t i) const { return values[i]; }
    float& operator [] (size_t i) { return values[i]; }

    float4& operator += (const float4& a) {
        x += a.x; y += a.y; z += a.z; w += a.w;
        return *this;
    }

    float4& operator -= (const float4& a) {
        x -= a.x; y -= a.y; z -= a.z; w -= a.w;
        return *this;
    }

    float4& operator *= (float a) {
        x *= a; y *= a; z *= a; w *= a;
        return *this;
    }

    float4& operator *= (const float4& a) {
        x *= a.x; y *= a.y; z *= a.z; w *= a.w;
        return *this;
    }
};

inline float2::float2(const float4& xy)
    : x(xy.x), y(xy.y)
{}

inline float3::float3(const float4& xyz)
    : x(xyz.x), y(xyz.y), z(xyz.z)
{}

inline float4 operator * (float a, const float4& b) {
    return float4(a * b.x, a * b.y, a * b.z, a * b.w);
}

inline float4 operator * (const float4& a, float b) {
    return float4(a.x * b, a.y * b, a.z * b, a.w * b);
}

inline float4 operator / (const float4& a, float b) {
    return a * (1.0f / b);
}

inline float4 operator - (const float4& a, const float4& b) {
    return float4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

inline float4 operator - (const float4& a) {
    return float4(-a.x, -a.y, -a.z, -a.w);
}

inline float4 operator + (const float4& a, const float4& b) {
    return float4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

inline float4 operator * (const float4& a, const float4& b) {
    return float4(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
}

inline float4 abs(const float4& a) {
    return float4(fabsf(a.x), fabsf(a.y), fabsf(a.z), fabsf(a.w));
}

inline float4 min(const float4& a, const float4& b) {
    return float4(a.x < b.x ? a.x : b.x,
                  a.y < b.y ? a.y : b.y,
                  a.z < b.z ? a.z : b.z,
                  a.w < b.w ? a.w : b.w);
}

inline float4 max(const float4& a, const float4& b) {
    return float4(a.x > b.x ? a.x : b.x,
                  a.y > b.y ? a.y : b.y,
                  a.z > b.z ? a.z : b.z,
                  a.w > b.w ? a.w : b.w);
}

inline float dot(const float4& a, const float4& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline float lensqr(const float4& a) {
    return dot(a, a);
}

inline float length(const float4& a) {
    return std::sqrt(dot(a, a));
}

inline float4 normalize(const float4& a) {
    return a * (1.0f / length(a));
}

inline float4 clamp(const float4& val, const float4& min, const float4& max) {
    return float4(clamp(val.x, min.x, max.x),
                  clamp(val.y, min.y, max.y),
                  clamp(val.z, min.z, max.z),
                  clamp(val.w, min.w, max.w));
}

#endif // FLOAT4_H
