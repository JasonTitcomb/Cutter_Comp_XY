#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define TOL 0.00005f
#define INPUT_ARC_TOL 0.001f
#define EPS 1e-7f
#define PARALLEL_TOL 1e-3f
#define PI 3.14159265358979323846f
#define TWO_PI 6.2831853071795864769f
#define MAX_SWEEP_DEG 359.9f
#define MIN_ARC_LEN 0.001f

static inline float c2d_clamp(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi
                                                                                             : x; }
struct Vec3
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  Vec3() = default;
  Vec3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}
};

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
static inline Vec3 v3(float x, float y, float z) { return {x, y, z}; }

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
  CM_NONE = 0, // not in comp block
  CM_IN = 1,
  CM_STEADY = 2,
  CM_OUT = 3
};

enum CornerType : uint8_t
{
  CORNER_ROLL = 0,
  CORNER_CHAMFER= 1,
};

enum MachineType : uint8_t
{
  MAC_MILL = 0,
  MAC_LATHE_DIA = 1,
  MAC_LATHE_RAD = 2
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
enum IntersectType : uint8_t
{
  IT_NONE = 0,
  IT_TANGENT = 1,
  IT_INTERSECT = 2
};
enum CompSide : int8_t
{
  COMP_OFF = 0,
  COMP_LEFT = +1,
  COMP_RIGHT = -1
};

struct AABB2
{
  float minx, miny, maxx, maxy;
};

struct Move2D
{
#ifndef NDEBUG
  char gcode_line[160] = {0};
#endif

  Vec2 p_0{0, 0}; // working start
  Vec2 p_1{0, 0}; // working end
  Vec2 center{0, 0};

  Vec2 startDir{0, 0};
  Vec2 endDir{0, 0};
  float radius = 0.0f;
  float feed = 0.0f;

  MotionType type = MOT_EMPTY;
  ArcDir arcDir = ARC_CW;
  CompMode compMode = CM_NONE;
  bool valid = true; // for output moves, indicates if move is valid

  AABB2 bounds;        // precomputed bounding box for this move
  uint32_t seqNum = 0; // for debugging

  // Legacy/original move snapshot fields (kept for compatibility/debugging).
  Vec2 src_0{0, 0}; // original start (currently not used by core logic; write-only)
  Vec2 src_1{0, 0}; // original end (used by roll-arc center logic)
  Vec2 src_c{0, 0}; // original arc center (currently not used by core logic; write-only)

  // Track original endpoints before any trimming/extension.
  Vec2 o_0{0, 0};
  Vec2 o_1{0, 0};

  // Initial tangent directions.
  Vec2 initialStartDir{0, 0};
  Vec2 initialEndDir{0, 0};

};
static inline bool machine_is_lathe(MachineType mt)
{
  return (mt == MAC_LATHE_DIA || mt == MAC_LATHE_RAD);
}

static inline float machine_x_to_internal_y(float x, MachineType mt)
{
  if (mt == MAC_LATHE_DIA)
    return 0.5f * x;
  return x;
}

static inline float internal_y_to_machine_x(float y, MachineType mt)
{
  if (mt == MAC_LATHE_DIA)
    return 2.0f * y;
  return y;
}

// VB-equivalent axis permutations for turning (XZ) <-> milling (XY) thinking:
// Turning -> Milling: (X,Y,Z) => (Z,X,Y)
// Milling -> Turning: (X,Y,Z) => (Y,Z,X)
static inline Vec3 turning_to_milling_xyz(const Vec3 &coord)
{
  return v3(coord.z, coord.x, coord.y);
}

static inline Vec3 milling_to_turning_xyz(const Vec3 &coord)
{
  return v3(coord.y, coord.z, coord.x);
}

// Convert machine-space absolute point to internal XY space used by compensation.
// Mill: X/Y -> X/Y
// Lathe: X/Z -> Y/X (internal X is machine Z, internal Y is machine X[or X/2 in DIA mode])
static inline Vec2 machine_to_internal_xy(const Vec3 &p, MachineType mt)
{
  if (!machine_is_lathe(mt))
    return v2(p.x, p.y);

  Vec3 mill = turning_to_milling_xyz(p); // mill.x=Z, mill.y=X
  if (mt == MAC_LATHE_DIA)
    mill.y = machine_x_to_internal_y(mill.y, mt); // DIA endpoint X -> radius

  return v2(mill.x, mill.y);
}

// Convert internal XY absolute point back to machine-space coordinates.
static inline Vec3 internal_xy_to_machine(const Vec2 &p, MachineType mt)
{
  if (!machine_is_lathe(mt))
    return v3(p.x, p.y, 0.0f);

  Vec3 mill = v3(p.x, p.y, 0.0f);
  if (mt == MAC_LATHE_DIA)
    mill.y = internal_y_to_machine_x(mill.y, mt); // radius -> DIA endpoint X

  return milling_to_turning_xyz(mill);
}

// Same mapping for center offset vectors (I/J or I/K style offsets).
// NOTE: In lathe DIA mode, X endpoints are diameter values, but I center offsets
// are typically provided in radius units. So we do NOT apply DIA 2x/0.5x scaling
// to center offsets.
static inline Vec2 machine_delta_to_internal_xy(const Vec3 &d, MachineType mt)
{
  if (!machine_is_lathe(mt))
    return v2(d.x, d.y);
  return v2(d.z, d.x);
}

static inline Vec3 internal_delta_xy_to_machine(const Vec2 &d, MachineType mt)
{
  if (!machine_is_lathe(mt))
    return v3(d.x, d.y, 0.0f);
  return v3(d.y, 0.0f, d.x);
}


// Plot-space mapping (SVG still draws in XY):
// - Mill: plot X/Y
// - Lathe: plot Z/radius (internal X/internal Y)
//   This avoids DIA-mode visual stretching caused by plotting machine X-diameter.
static inline Vec2 internal_xy_to_plot_xy(const Vec2 &p, MachineType mt)
{
  (void)mt;
  return v2(p.x, p.y);
}

static inline void update_vectors(Move2D &m)
{
  if (m.type == MOT_LINE)
  {
    Vec2 d = m.p_1 - m.p_0;
    Vec2 u = normalize(d);
    m.startDir = u;
    m.endDir = u;
    return;
  }
  if (m.type == MOT_ARC)
  {
    Vec2 rs = normalize(m.p_0 - m.center);
    Vec2 re = normalize(m.p_1 - m.center);
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

static inline bool is_radius_consistent(const Move2D &m)
{
  float r0 = len(m.p_0 - m.center);
  float r1 = len(m.p_1 - m.center);
  bool isValid = fabsf(r0 - r1) <= INPUT_ARC_TOL;
  if (!isValid)
  {
    DBG_PRINT("Arc radius inconsistency detected! SeqNum: ");
    DBG_PRINTLN(m.seqNum);
    DBG_PRINT("r0: ");
    DBG_PRINT("%.6f", r0);
    DBG_PRINT(", r1: ");
    DBG_PRINT("%.6f", r1);
    // You can set a breakpoint on the line below to catch radius inconsistencies during debugging.
    // This can help identify issues with arc moves that may cause problems for compensation logic.
    // For example, if you see this triggered, check if the arc endpoints are very close together or if the center is far from both endpoints.
    // You may want to log the move details here for further analysis.
  }
  return isValid;
}

static inline int get_winding_dir(Vec2 a, Vec2 b)
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

static inline float angleNorm(float a)
{
  while (a < 0)
    a += TWO_PI;
  while (a >= TWO_PI)
    a -= TWO_PI;
  return a;
}

static inline float wrap2pi(float a)
{
  a = fmodf(a, TWO_PI);
  if (a < 0)
    a += TWO_PI;
  return a;
}

static inline float arcSweepDeg(const Move2D &m)
{
  float a0 = wrap2pi(atan2f(m.p_0.y - m.center.y, m.p_0.x - m.center.x));
  float a1 = wrap2pi(atan2f(m.p_1.y - m.center.y, m.p_1.x - m.center.x));

  if (m.arcDir == ARC_CCW)
  {
    float sw = a1 - a0;
    if (sw < 0)
      sw += TWO_PI;
    return sw * (180.0f / PI); // [0, 360)
  }
  else
  { // ARC_CW
    float sw = a0 - a1;
    if (sw < 0)
      sw += TWO_PI;
    return sw * (180.0f / PI); // [0, 360)
  }
}

static inline float sweepCCW(float a0, float a1)
{
  a0 = angleNorm(a0);
  a1 = angleNorm(a1);
  float d = a1 - a0;
  if (d < 0)
    d += TWO_PI;
  return d;
}

static inline float sweepCW(float a0, float a1)
{
  // CW from a0 to a1 is CCW from a1 to a0
  return sweepCCW(a1, a0);
}

static inline bool angle_on_arc_norm(float a0n, float a1n, float apn, ArcDir dir)
{
  if (dir == ARC_CCW)
  {
    return (apn >= a0n - EPS && apn <= a1n + EPS) ||
           (a0n > a1n && (apn >= a0n - EPS || apn <= a1n + EPS));
  }

  return (apn >= a1n - EPS && apn <= a0n + EPS) ||
         (a1n > a0n && (apn >= a1n - EPS || apn <= a0n + EPS));
}

static inline AABB2 aabb_of(const Move2D &m)
{
  AABB2 b;
  b.minx = fminf(m.p_0.x, m.p_1.x);
  b.maxx = fmaxf(m.p_0.x, m.p_1.x);
  b.miny = fminf(m.p_0.y, m.p_1.y);
  b.maxy = fmaxf(m.p_0.y, m.p_1.y);

  if (m.type == MOT_ARC && fabsf(m.radius) > TOL)
  {
    // Check silhouette points to get tighter bounds.
    float a0n = angleNorm(atan2f(m.p_0.y - m.center.y, m.p_0.x - m.center.x));
    float a1n = angleNorm(atan2f(m.p_1.y - m.center.y, m.p_1.x - m.center.x));

    Vec2 leftPoint = v2(m.center.x - m.radius, m.center.y);
    float leftAngle = angleNorm(atan2f(leftPoint.y - m.center.y, leftPoint.x - m.center.x));
    if (angle_on_arc_norm(a0n, a1n, leftAngle, m.arcDir))
      b.minx = leftPoint.x;

    Vec2 rightPoint = v2(m.center.x + m.radius, m.center.y);
    float rightAngle = angleNorm(atan2f(rightPoint.y - m.center.y, rightPoint.x - m.center.x));
    if (angle_on_arc_norm(a0n, a1n, rightAngle, m.arcDir))
      b.maxx = rightPoint.x;

    Vec2 topPoint = v2(m.center.x, m.center.y + m.radius);
    float topAngle = angleNorm(atan2f(topPoint.y - m.center.y, topPoint.x - m.center.x));
    if (angle_on_arc_norm(a0n, a1n, topAngle, m.arcDir))
      b.maxy = topPoint.y;

    Vec2 bottomPoint = v2(m.center.x, m.center.y - m.radius);
    float bottomAngle = angleNorm(atan2f(bottomPoint.y - m.center.y, bottomPoint.x - m.center.x));
    if (angle_on_arc_norm(a0n, a1n, bottomAngle, m.arcDir))
      b.miny = bottomPoint.y;
  }
  return b;
}

static void init_all_aabb(Move2D *moves, int count)
{
  for (int i = 0; i < count; ++i)
  {
    if (moves[i].type != MOT_EMPTY && moves[i].valid) // Only compute bounds for valid moves
      moves[i].bounds = aabb_of(moves[i]);
  }
}

static inline bool aabb_intersects(const AABB2 &a, const AABB2 &b)
{
  return !(a.maxx < b.minx || a.minx > b.maxx || a.maxy < b.miny || a.miny > b.maxy);
}

// param along LINE (0..1 if on segment)
static inline float line_t(const Move2D &m, Vec2 p)
{
  Vec2 d = m.p_1 - m.p_0;
  float L2 = dot(d, d);
  if (L2 < 1e-12f)
    return 0.0f;
  return dot(p - m.p_0, d) / L2;
}

// distance-along-source used for "nearest crossing" selection
static inline float distFromStart_along(const Move2D &m, Vec2 p)
{
  if (m.type == MOT_LINE)
  {
    float t = line_t(m, p);
    t = c2d_clamp(t, 0.0f, 1.0f);
    return len(m.p_1 - m.p_0) * t;
  }
  if (m.type == MOT_ARC)
  {
    float a0 = atan2f(m.p_0.y - m.center.y, m.p_0.x - m.center.x);
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
static inline bool is_near(const Vec2 &a, const Vec2 &b)
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
    if (!is_near(a.center, b.center) || fabsf(a.radius - b.radius) > TOL)
      return false;

    if (fabsf(a.radius - b.radius) > TOL)
      return false;

    return true;
  }

  return false;
}

// Currently unused helper chain with includedAngle*.
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


// Find the first move after a CM_IN move
static int first_cutting_move(const Move2D *moves, int count)
{
  for (int i = 0; i < count; ++i)
  {
    if (isMotionValid(moves[i]) && moves[i].compMode == CM_IN)
    {
      // Found a CM_IN move, now find the next valid move
      return next_valid_index(moves, count, i);
    }
  }
  return -1; // No CM_IN found
}

// Find the last move before a CM_OUT move
static int last_cutting_move(const Move2D *moves, int count,int startAt)
{
  for (int i = startAt; i < count; ++i)
  {
    if (moves[i].compMode == CM_OUT)
    {
      // Found a CM_OUT move, now find the last valid move before it
      for (int k = i - 1; k >= 0; --k)
      {
        if (isMotionValid(moves[k]))
        {
          return k;
        }
      }
      return -1; // No valid move before CM_OUT
    }
  }
  return -1; // No CM_OUT found
}

static inline bool angleOnSweepCCW(float a0, float a1, float ap)
{
  a0 = angleNorm(a0);
  a1 = angleNorm(a1);
  ap = angleNorm(ap);
  if (a0 <= a1)
    return (ap + EPS >= a0) && (ap <= a1 + EPS);
  // wrap
  return (ap >= a0 - EPS) || (ap <= a1 + EPS);
}

static inline bool angleOnSweepCW(float a0, float a1, float ap)
{
  // CW sweep from a0 down to a1 is CCW from a1 to a0
  return angleOnSweepCCW(a1, a0, ap);
}

static inline bool pointOnSegment(Vec2 a, Vec2 b, Vec2 p)
{
  Vec2 ab = b - a;
  float lab2 = dot(ab, ab);
  if (lab2 < TOL)
    return (len(p - a) < TOL);

  float t = dot(p - a, ab) / lab2;
  if (t < -TOL || t > 1.0f + TOL)
    return false;
  float d = fabsf(cross(p - a, ab)) / sqrtf(lab2);
  return d < TOL;
}

static inline bool pointOnArc(const Move2D &a, Vec2 p)
{
  float rp = len(p - a.center);
  if (fabsf(rp - a.radius) > TOL)
    return false;

  float a0 = atan2f(a.p_0.y - a.center.y, a.p_0.x - a.center.x);
  float a1 = atan2f(a.p_1.y - a.center.y, a.p_1.x - a.center.x);
  float ap = atan2f(p.y - a.center.y, p.x - a.center.x);

  if (a.arcDir == ARC_CCW)
    return angleOnSweepCCW(a0, a1, ap);
  else
    return angleOnSweepCW(a0, a1, ap);
}

static inline IntersectType intersectLineLine(const Move2D &ln1, const Move2D &ln2, Vec2 &ip, bool &tip)
{
  Vec2 p = ln1.o_0;
  Vec2 r = ln1.o_1 - ln1.o_0;
  Vec2 q = ln2.o_0;
  Vec2 s = ln2.o_1 - ln2.o_0;

  float lr = len(r);
  float ls = len(s);
  if (lr < TOL || ls < TOL)
  {
    tip = false;
    return IT_NONE;
  }

  float den = cross(r, s);
  float denTol = PARALLEL_TOL * lr * ls;
  if (fabsf(den) <= denTol)
  {
    tip = false;// too parallel to reliably intersect
    return IT_NONE;
  }

  float t = cross(q - p, s) / den;
  float u = cross(q - p, r) / den;
  ip = p + r * t;

  tip = (t >= -TOL && t <= 1.0f + TOL && u >= -TOL && u <= 1.0f + TOL);
  return IT_INTERSECT;
}

static inline IntersectType intersectCircleCircle(const Move2D &a1, const Move2D &a2, Vec2 &p1, Vec2 &p2, int &count)
{
  Vec2 c0 = a1.center, c1 = a2.center;
  float r0 = a1.radius, r1 = a2.radius;
  Vec2 d = c1 - c0;
  float distc = len(d);
  count = 0;

  if (distc < TOL)
    return IT_NONE;
  if (distc > r0 + r1 + TOL)
    return IT_NONE;
  if (distc < fabsf(r0 - r1) - TOL)
    return IT_NONE;

  float a = (r0 * r0 - r1 * r1 + distc * distc) / (2.0f * distc);
  float h2 = r0 * r0 - a * a;
  Vec2 u = d * (1.0f / distc);
  Vec2 mid = c0 + u * a;

  if (fabsf(h2) < TOL)
  {
    p1 = mid;
    count = 1;
    return IT_TANGENT;
  }

  float h = sqrtf(fmaxf(0.0f, h2));
  Vec2 perp = leftNormal(u);
  p1 = mid + perp * h;
  p2 = mid - perp * h;
  count = 2;
  return IT_INTERSECT;
}

static inline IntersectType intersectLineCircle(Vec2 l1, Vec2 a1, Vec2 ctr, float r, Vec2 &p1, Vec2 &p2, int &count)
{
  Vec2 d = a1 - l1;
  float dd = dot(d, d);
  count = 0;
  if (dd < 1e-20f)
    return IT_NONE;

  Vec2 f = l1 - ctr;
  float t0 = -dot(f, d) / dd;
  Vec2 q = l1 + d * t0;

  Vec2 qc = q - ctr;
  float dist2 = dot(qc, qc);

  float r2 = r * r;
  const float eps = 1e-5f;
  const float eps2 = eps * eps;

  float h2 = r2 - dist2;

  if (h2 < -eps2)
    return IT_NONE;

  if (fabsf(h2) <= eps2)
  {
    p1 = q;
    count = 1;
    return IT_TANGENT;
  }

  float h = sqrtf(h2);
  float invLen = 1.0f / sqrtf(dd);
  Vec2 u = d * invLen;

  p1 = q - u * h;
  p2 = q + u * h;
  count = 2;
  return IT_INTERSECT;
}

static inline Vec2 pickClosest(Vec2 ref, Vec2 a, Vec2 b)
{
  return (len(a - ref) <= len(b - ref)) ? a : b;
}

static inline bool isNearDir(Vec2 a, Vec2 b)
{
  a = normalize(a);
  b = normalize(b);
  return dot(a, b) > 0.9995f;
}