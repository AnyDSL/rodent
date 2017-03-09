#ifndef FLOAT3_H
#define FLOAT3_H

#include <cmath>
#include "float2.h"

struct float4;

struct float3 {
    float x, y, z;
    float3() {}
    explicit float3(float x) : x(x), y(x), z(x) {}
    explicit float3(const float4& f);
    float3(float x, float y, float z) : x(x), y(y), z(z) {}
    float3(const float2& f, float z) : x(f.x), y(f.y), z(z) {}
    float3(float x, const float2& f) : x(x), y(f.x), z(f.y) {}

    float operator [] (int axis) const { return *(&x + axis); }
    float& operator [] (int axis) { return *(&x + axis); }

    float3& operator += (const float3& a) {
        x += a.x; y += a.y; z += a.z;
        return *this;
    }

    float3& operator *= (float a) {
        x *= a; y *= a; z *= a;
        return *this;
    }

    float3& operator *= (const float3& a) {
        x *= a.x; y *= a.y; z *= a.z;
        return *this;
    }
};

inline float2::float2(const float3& f) : x(f.x), y(f.y) {}

inline float3 operator * (float a, const float3& b) {
    return float3(a * b.x, a * b.y, a * b.z);
}

inline float3 operator * (const float3& a, float b) {
    return float3(a.x * b, a.y * b, a.z * b);
}

inline float3 operator / (const float3& a, float b) {
    return float3(a.x / b, a.y / b, a.z / b);
}

inline float3 operator - (const float3& a, const float3& b) {
    return float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline float3 operator - (const float3& a) {
    return float3(-a.x, -a.y, -a.z);
}

inline float3 operator + (const float3& a, const float3& b) {
    return float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline float3 operator * (const float3& a, const float3& b) {
    return float3(a.x * b.x, a.y * b.y, a.z * b.z);
}

inline float3 cross(const float3& a, const float3& b) {
    return float3(a.y * b.z - a.z * b.y,
                  a.z * b.x - a.x * b.z,
                  a.x * b.y - a.y * b.x);
}

inline float3 rotate(const float3& v, const float3& axis, float angle) {
    float q[4];
    q[0] = axis.x * sinf(angle / 2);
    q[1] = axis.y * sinf(angle / 2);
    q[2] = axis.z * sinf(angle / 2);
    q[3] = cosf(angle / 2);

    float p[4];
    p[0] = q[3] * v.x + q[1] * v.z - q[2] * v.y;
    p[1] = q[3] * v.y - q[0] * v.z + q[2] * v.x;
    p[2] = q[3] * v.z + q[0] * v.y - q[1] * v.x;
    p[3] = -(q[0] * v.x + q[1] * v.y + q[2] * v.z);

    return float3(p[3] * -q[0] + p[0] * q[3] + p[1] * -q[2] - p[2] * -q[1],
                  p[3] * -q[1] - p[0] * -q[2] + p[1] * q[3] + p[2] * -q[0],
                  p[3] * -q[2] + p[0] * -q[1] - p[1] * -q[0] + p[2] * q[3]);
}

inline float3 abs(const float3& a) {
    return float3(fabsf(a.x), fabsf(a.y), fabsf(a.z));
}

inline float3 min(const float3& a, const float3& b) {
    return float3(a.x < b.x ? a.x : b.x,
                  a.y < b.y ? a.y : b.y,
                  a.z < b.z ? a.z : b.z);
}

inline float3 max(const float3& a, const float3& b) {
    return float3(a.x > b.x ? a.x : b.x,
                  a.y > b.y ? a.y : b.y,
                  a.z > b.z ? a.z : b.z);
}

inline float dot(const float3& a, const float3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float lensqr(const float3& a) {
    return dot(a, a);
}

inline float length(const float3& a) {
    return sqrtf(dot(a, a));
}

inline float3 normalize(const float3& a) {
    return a * (1.0f / length(a));
}

#endif // FLOAT3_H
