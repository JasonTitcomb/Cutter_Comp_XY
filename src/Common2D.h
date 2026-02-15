#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef C2D_EPS
#define C2D_EPS 1e-6f
#endif
#define TOL 0.0001f

static inline float c2d_sqr(float x) { return x * x; }
static inline float c2d_clamp(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi
                                                                                             : x; }

struct Vec2
{
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float X, float Y) : x(X), y(Y) {}

    Vec2 operator+(const Vec2 &o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2 &o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
};

static inline Vec2 v2(float x, float y) { return {x, y}; }

static inline float dot(const Vec2 &a, const Vec2 &b) { return a.x * b.x + a.y * b.y; }
static inline float cross(const Vec2 &a, const Vec2 &b) { return a.x * b.y - a.y * b.x; }
static inline float len(const Vec2 &v) { return sqrtf(dot(v, v)); }

static inline Vec2 normalize(const Vec2 &v)
{
    float l = len(v);
    if (l < C2D_EPS)
        return {0, 0};
    return {v.x / l, v.y / l};
}

// Left normal (rotate +90)
static inline Vec2 leftNormal(const Vec2 &v) { return {-v.y, v.x}; }
// Right normal (rotate -90)
static inline Vec2 rightNormal(const Vec2 &v) { return {v.y, -v.x}; }

enum CompMode : uint8_t
{
    CM_STEADY = 0,
    CM_IN = 1, // VB CC_IN
    CM_OUT = 2 // VB CC_OUT
};

enum MotionType : uint8_t
{
    MOT_EMPTY = 0,
    MOT_LINE = 1,
    MOT_ARC = 2
};
enum ArcDir : uint8_t
{
    ARC_CW = 0,
    ARC_CCW = 1
};
enum CompSide : int8_t
{
    COMP_OFF = 0,
    COMP_LEFT = +1,
    COMP_RIGHT = -1
};

struct Move2D
{
    MotionType type = MOT_EMPTY;

    Vec2 p0{0, 0}; // start
    Vec2 p1{0, 0}; // end
    float feed = 0.0f;
    bool rapid = false;
    CompMode compMode = CM_STEADY;

    // Arc only:
    ArcDir arcDir = ARC_CW;
    Vec2 center{0, 0};
    float radius = 0.0f;
    // Track original end before any trimming/extension (VB: InitialEndPt)
    Vec2 initialEndPt{0, 0};

    // Tangent directions (VB: StartDirection/EndDirection)
    Vec2 startDir{0, 0};
    Vec2 endDir{0, 0};
};

inline void update_dirs(Move2D &m)
{
    if (m.type == MOT_LINE)
    {
        Vec2 d = m.p1 - m.p0;
        Vec2 u = normalize(d);
        m.startDir = u;
        m.endDir = u;
        return;
    }
    if (m.type == MOT_ARC)
    {
        // Tangent is +/- 90° from radius vector
        Vec2 rs = normalize(m.p0 - m.center);
        Vec2 re = normalize(m.p1 - m.center);

        // For CCW, tangent = leftNormal(radius); for CW, tangent = rightNormal(radius)
        if (m.arcDir == ARC_CCW)
        {
            m.startDir = leftNormal(rs);
            m.endDir = leftNormal(re);
        }
        else
        {
            m.startDir = rightNormal(rs);
            m.endDir = rightNormal(re);
        }
        return;
    }
    m.startDir = {0, 0};
    m.endDir = {0, 0};
}

static inline int WindingDirection(Vec2 a, Vec2 b)
{
    // VB: el1.EndDirection.WindingDirection(el2.StartDirection)
    // Implemented as sign of cross of normalized vectors.
    a = normalize(a);
    b = normalize(b);
    float z = cross(a, b);
    if (z >  C2D_EPS) return +1;
    if (z < -C2D_EPS) return -1;
    return 0;
}

