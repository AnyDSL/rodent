#ifndef FLOAT2_H
#define FLOAT2_H

#include <cmath>
#include "common.h"

struct float3;
struct float4;

struct float2 {
    union {
        struct { float x, y; };
        float values[2];
    };

    float2() {}
    explicit float2(float x) : x(x), y(x) {}
    explicit float2(const float3& xy);
    explicit float2(const float4& xy);
    float2(float x, float y) : x(x), y(y) {}

    bool operator == (const float2& other) const {
        return x == other.x && y == other.y;
    }

    bool operator != (const float2& other) const {
        return x != other.x || y != other.y;
    }

    float operator [] (size_t i) const { return values[i]; }
    float& operator [] (size_t i) { return values[i]; }

    float2& operator += (const float2& a) {
        x += a.x; y += a.y;
        return *this;
    }

    float2& operator -= (const float2& a) {
        x -= a.x; y -= a.y;
        return *this;
    }

    float2& operator *= (float a) {
        x *= a; y *= a;
        return *this;
    }

    float2& operator *= (const float2& a) {
        x *= a.x; y *= a.y;
        return *this;
    }
};

inline float2 operator * (float a, const float2& b) {
    return float2(a * b.x, a * b.y);
}

inline float2 operator * (const float2& a, float b) {
    return float2(a.x * b, a.y * b);
}

inline float2 operator / (const float2& a, float b) {
    return a * (1.0f / b);
}

inline float2 operator - (const float2& a, const float2& b) {
    return float2(a.x - b.x, a.y - b.y);
}

inline float2 operator + (const float2& a, const float2& b) {
    return float2(a.x + b.x, a.y + b.y);
}

inline float2 operator * (const float2& a, const float2& b) {
    return float2(a.x * b.x, a.y * b.y);
}

inline float2 min(const float2& a, const float2& b) {
    return float2(a.x < b.x ? a.x : b.x,
                  a.y < b.y ? a.y : b.y);
}

inline float2 max(const float2& a, const float2& b) {
    return float2(a.x > b.x ? a.x : b.x,
                  a.y > b.y ? a.y : b.y);
}

inline float dot(const float2& a, const float2& b) {
    return a.x * b.x + a.y * b.y;
}

inline float lensqr(const float2& a) {
    return dot(a, a);
}

inline float length(const float2& a) {
    return std::sqrt(dot(a, a));
}

inline float2 normalize(const float2& a) {
    return a * (1.0f / length(a));
}

#endif // FLOAT2_H
