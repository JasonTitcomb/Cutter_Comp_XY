#ifndef VEC2_H
#define VEC2_H

#include <cmath>

struct Vec3 {
    float x = 0, y = 0;

    Vec3() = default;
    Vec3(float X, float Y) : x(X), y(Y) {}
};

static inline Vec3 v2(float x, float y) { return {x, y}; }
static inline Vec3 v2_add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y}; }
static inline Vec3 v2_sub(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y}; }
static inline Vec3 v2_mul(const Vec3& a, float s) { return {a.x * s, a.y * s}; }
static inline float v2_dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y; }
static inline float v2_cross(const Vec3& a, const Vec3& b) { return a.x * b.y - a.y * b.x; }
static inline float v2_len(const Vec3& a) { return std::sqrt(v2_dot(a, a)); }
static inline Vec3 v2_norm(const Vec3& a) {
    float l = v2_len(a);
    if (l < 1e-6f) return {0, 0};
    return {a.x / l, a.y / l};
}
static inline Vec3 v2_leftN(const Vec3& a) { return {-a.y, a.x}; }

#endif // VEC2_H
