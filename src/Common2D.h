#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

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
  if (l < TOL)
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
  CM_IN = 1,
  CM_OUT = 2
};

enum MotionType : uint8_t
{
  MOT_EMPTY = 0,
  MOT_RAPID = 1, // G0
  MOT_LINE = 2,
  MOT_ARC = 3
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
  // copy of codeline for testing only
  char gcode_line[160] = {0};
  MotionType type = MOT_EMPTY;
  uint32_t seqNum = 0; // for debugging
  Vec2 p0{0, 0};       // start
  Vec2 p1{0, 0};       // end
  float feed = 0.0f;
  bool rapid = false;
  bool valid = true; // for output moves, indicates if move is valid (e.g. not a tiny line we want to skip)
  CompMode compMode = CM_STEADY;

  // Arc only:
  ArcDir arcDir = ARC_CW;
  Vec2 center{0, 0};
  float radius = 0.0f;
  // Track original end before any trimming/extension
  Vec2 initialStartPt{0, 0};
  Vec2 initialEndPt{0, 0};

  // Tangent directions
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
  // Implemented as sign of cross of normalized vectors.
  a = normalize(a);
  b = normalize(b);
  float z = cross(a, b);
  if (z > TOL)
    return +1;
  if (z < -TOL)
    return -1;
  return 0;
}

struct AABB2
{
  float minx, miny, maxx, maxy;
};

static inline AABB2 aabb_of(const Move2D &m)
{
  AABB2 b;
  b.minx = fminf(m.p0.x, m.p1.x);
  b.maxx = fmaxf(m.p0.x, m.p1.x);
  b.miny = fminf(m.p0.y, m.p1.y);
  b.maxy = fmaxf(m.p0.y, m.p1.y);

  if (m.type == MOT_ARC)
  {
    // conservative arc bounds: include full circle bounds (safe, a bit loose)
    b.minx = fminf(b.minx, m.center.x - m.radius);
    b.maxx = fmaxf(b.maxx, m.center.x + m.radius);
    b.miny = fminf(b.miny, m.center.y - m.radius);
    b.maxy = fmaxf(b.maxy, m.center.y + m.radius);
  }
  return b;
}

static inline bool aabb_intersects(const AABB2 &a, const AABB2 &b)
{
  return !(a.maxx < b.minx || a.minx > b.maxx || a.maxy < b.miny || a.miny > b.maxy);
}

static inline float angleNorm(float a)
{
  while (a < 0)
    a += 2.0f * (float)M_PI;
  while (a >= 2.0f * (float)M_PI)
    a -= 2.0f * (float)M_PI;
  return a;
}

static inline float sweepCCW(float a0, float a1)
{
  a0 = angleNorm(a0);
  a1 = angleNorm(a1);
  float d = a1 - a0;
  if (d < 0)
    d += 2.0f * (float)M_PI;
  return d;
}

static inline float sweepCW(float a0, float a1)
{
  // CW from a0 to a1 is CCW from a1 to a0
  return sweepCCW(a1, a0);
}

// param along LINE (0..1 if on segment)
static inline float line_t(const Move2D &m, Vec2 p)
{
  Vec2 d = m.p1 - m.p0;
  float L2 = dot(d, d);
  if (L2 < 1e-12f)
    return 0.0f;
  return dot(p - m.p0, d) / L2;
}

// distance-along-source used for "nearest crossing" selection
static inline float distFromStart_along(const Move2D &m, Vec2 p)
{
  if (m.type == MOT_LINE)
  {
    float t = line_t(m, p);
    t = c2d_clamp(t, 0.0f, 1.0f);
    return len(m.p1 - m.p0) * t;
  }
  if (m.type == MOT_ARC)
  {
    float a0 = atan2f(m.p0.y - m.center.y, m.p0.x - m.center.x);
    float ap = atan2f(p.y - m.center.y, p.x - m.center.x);
    float sw = (m.arcDir == ARC_CCW) ? sweepCCW(a0, ap) : sweepCW(a0, ap);
    return fabsf(m.radius) * sw;
  }
  return 0.0f;
}

static inline void invalidateRange(Move2D *moves, int i, int j)
{
  for (int k = i + 1; k < j; ++k)
  {
    moves[k].valid = false;
  }
}
// ----- helpers -----
static inline bool nearPt2(const Vec2 &a, const Vec2 &b)
{
  return len(a - b) <= TOL;
}

static inline bool isMotionValid(const Move2D &m)
{
  return m.valid && m.type != MOT_EMPTY;
}

// Check if two elements are colinear.
// For lines, checks if directions are parallel. For arcs, checks if centers/radii match.
static inline bool isColinearWith(const Move2D &a, const Move2D &b)
{
  // Must be same element type
  if (a.type != b.type)
    return false;

  // -------- LINE vs LINE --------
  if (a.type == MOT_LINE)
  {
    Vec2 da = a.startDir;
    Vec2 db = b.startDir;

    if (len(da) < TOL || len(db) < TOL)
      return false;

    da = normalize(da);
    db = normalize(db);

    // Cross(StartDirection, other.StartDirection).Length < TOL
    float cr = fabsf(cross(da, db));
    return cr < TOL;
  }

  // -------- ARC vs ARC --------
  if (a.type == MOT_ARC)
  {
    if (!nearPt2(a.center, b.center) || fabsf(a.radius - b.radius) > TOL)
      return false;

    if (fabsf(a.radius - b.radius) > TOL)
      return false;

    return true;
  }

  return false;
}

static inline float rad2deg(float r) { return r * (180.0f / (float)M_PI); }

static int next_valid_index(const Move2D *moves, int count, int i)
{
  for (int k = i + 1; k < count; ++k)
  {
    if (isMotionValid(moves[k]))
      return k;
  }
  return -1;
}

static int first_valid_index(const Move2D *moves, int count)
{
  for (int i = 0; i < count; ++i)
  {
    if (isMotionValid(moves[i]))
      return i;
  }
  return -1;
}

// colinear test: lines parallel + b.p0 lies on a's infinite line
static inline bool lines_colinear(const Move2D &a, const Move2D &b,
                                  float angleTolDeg, float distTol)
{
  if (a.type != MOT_LINE || b.type != MOT_LINE)
    return false;

  Vec2 da = a.p1 - a.p0;
  Vec2 db = b.p1 - b.p0;

  float la = len(da);
  float lb = len(db);
  if (la < TOL || lb < TOL)
    return false;

  Vec2 ua = da * (1.0f / la);
  Vec2 ub = db * (1.0f / lb);

  // parallel (ignore direction sign)
  float c = c2d_clamp(dot(ua, ub), -1.0f, 1.0f);
  float ang = rad2deg(acosf(fabsf(c)));
  if (ang > angleTolDeg)
    return false;

  // point-to-line distance: |(p - a0) x ua|
  float d = fabsf(cross(b.p0 - a.p0, ua));
  return d <= distTol;
}

// Find the first move after a CM_IN move
static int first_cutting_move(const Move2D* moves, int count)
{
    for (int i = 0; i < count; ++i) {
        if (isMotionValid(moves[i]) && moves[i].compMode == CM_IN) {
            // Found a CM_IN move, now find the next valid move
            return next_valid_index(moves, count, i);
        }
    }
    return -1; // No CM_IN found
}

// Find the last move before a CM_OUT move
static int last_cutting_move(const Move2D* moves, int count)
{
    for (int i = 0; i < count; ++i) {
        if (isMotionValid(moves[i]) && moves[i].compMode == CM_OUT) {
            // Found a CM_OUT move, now find the last valid move before it
            for (int k = i - 1; k >= 0; --k) {
                if (isMotionValid(moves[k])) {
                    return k;
                }
            }
            return -1; // No valid move before CM_OUT
        }
    }
    return -1; // No CM_OUT found
}