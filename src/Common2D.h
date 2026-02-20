#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define TOL 0.00005f
#define EPS 1e-7f

static inline float c2d_sqr(float x) { return x * x; }
static inline float c2d_clamp(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }

struct Vec3
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  Vec3() = default;
  Vec3(float X, float Y) : x(X), y(Y), z(0.0f) {}
  Vec3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}

  Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
};

static inline Vec3 v3(float x, float y, float z) { return {x, y, z}; }

static inline float dot(const Vec3 &a, const Vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float cross(const Vec3 &a, const Vec3 &b) { return a.x * b.y - a.y * b.x; }
static inline float len(const Vec3 &v) { return sqrtf(dot(v, v)); }

static inline Vec3 normalize(const Vec3 &v)
{
  float l = len(v);
  if (l < TOL)
    return {0, 0, 0};
  return {v.x / l, v.y / l, v.z / l};
}

// Left normal (rotate +90)
static inline Vec3 leftNormal(const Vec3 &v) { return {-v.y, v.x}; }
// Right normal (rotate -90)
static inline Vec3 rightNormal(const Vec3 &v) { return {v.y, -v.x}; }

enum CompMode : uint8_t
{
  CM_NONE = 0, // not in comp block
  CM_IN = 1,
  CM_STEADY = 2,
  CM_OUT = 3
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
  // copy of codeline for testing only
  char gcode_line[160] = {0};
  MotionType type = MOT_EMPTY;
  uint32_t seqNum = 0; // for debugging
  Vec3 p0{0, 0};       // start
  Vec3 p1{0, 0};       // end
  AABB2 bounds;        // precomputed bounding box for this move
  float feed = 0.0f;
  bool valid = true; // for output moves, indicates if move is valid
  CompMode compMode = CM_NONE;

  // Arc only:
  ArcDir arcDir = ARC_CW;
  Vec3 center{0, 0};
  float radius = 0.0f;
  // Track original end before any trimming/extension
  Vec3 initialStartPt{0, 0};
  Vec3 initialEndPt{0, 0};

  // Tangent directions
  Vec3 startDir{0, 0};
  Vec3 endDir{0, 0};
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
static inline Vec3 machine_to_internal_xy(const Vec3 &p, MachineType mt)
{
  if (!machine_is_lathe(mt))
    return v3(p.x, p.y, 0.0f);

  Vec3 mill = turning_to_milling_xyz(p); // mill.x=Z, mill.y=X
  if (mt == MAC_LATHE_DIA)
    mill.y = machine_x_to_internal_y(mill.y, mt); // DIA endpoint X -> radius

  return v3(mill.x, mill.y, 0.0f);
}

// Convert internal XY absolute point back to machine-space coordinates.
static inline Vec3 internal_xy_to_machine(const Vec3 &p, MachineType mt)
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
static inline Vec3 machine_delta_to_internal_xy(const Vec3 &d, MachineType mt)
{
  if (!machine_is_lathe(mt))
    return v3(d.x, d.y, 0.0f);
  return v3(d.z, d.x, 0.0f);
}

static inline Vec3 internal_delta_xy_to_machine(const Vec3 &d, MachineType mt)
{
  if (!machine_is_lathe(mt))
    return v3(d.x, d.y, 0.0f);
  return v3(d.y, 0.0f, d.x);
}

// Convenience aliases requested for pre/post mapping around compensation.
static inline Vec3 from_xz_to_xy(const Vec3 &machinePoint, MachineType mt)
{
  return machine_to_internal_xy(machinePoint, mt);
}

static inline Vec3 from_xy_to_xz(const Vec3 &internalPoint, MachineType mt)
{
  return internal_xy_to_machine(internalPoint, mt);
}

// Plot-space mapping (SVG still draws in XY):
// - Mill: plot X/Y
// - Lathe: plot Z/radius (internal X/internal Y)
//   This avoids DIA-mode visual stretching caused by plotting machine X-diameter.
static inline Vec3 internal_xy_to_plot_xy(const Vec3 &p, MachineType mt)
{
  (void)mt;
  return v3(p.x, p.y, 0.0f);
}


inline void update_dirs(Move2D &m)
{
  if (m.type == MOT_LINE)
  {
    Vec3 d = m.p1 - m.p0;
    Vec3 u = normalize(d);
    m.startDir = u;
    m.endDir = u;
    return;
  }
  if (m.type == MOT_ARC)
  {
    // Tangent is +/- 90° from radius vector
    Vec3 rs = normalize(m.p0 - m.center);
    Vec3 re = normalize(m.p1 - m.center);

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


static inline int WindingDirection(Vec3 a, Vec3 b)
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

static inline AABB2 aabb_of(const Move2D &m)
{
  AABB2 b;
  b.minx = fminf(m.p0.x, m.p1.x);
  b.maxx = fmaxf(m.p0.x, m.p1.x);
  b.miny = fminf(m.p0.y, m.p1.y);
  b.maxy = fmaxf(m.p0.y, m.p1.y);

  if (m.type == MOT_ARC)
  {
    // Check silhouette points to get tighter bounds
    // Left point (180 degrees, -X direction)
    Vec3 silhouettePoint = v3(m.center.x - m.radius, m.center.y, 0.0f);
    if (len(silhouettePoint - m.center) > TOL) // ensure valid point
    {
      float a0 = atan2f(m.p0.y - m.center.y, m.p0.x - m.center.x);
      float a1 = atan2f(m.p1.y - m.center.y, m.p1.x - m.center.x);
      float ap = atan2f(silhouettePoint.y - m.center.y, silhouettePoint.x - m.center.x);

      bool onArc = (m.arcDir == ARC_CCW)
                       ? (angleNorm(ap) >= angleNorm(a0) - EPS && angleNorm(ap) <= angleNorm(a1) + EPS) ||
                             (angleNorm(a0) > angleNorm(a1) && (angleNorm(ap) >= angleNorm(a0) - EPS || angleNorm(ap) <= angleNorm(a1) + EPS))
                       : (angleNorm(ap) >= angleNorm(a1) - EPS && angleNorm(ap) <= angleNorm(a0) + EPS) ||
                             (angleNorm(a1) > angleNorm(a0) && (angleNorm(ap) >= angleNorm(a1) - EPS || angleNorm(ap) <= angleNorm(a0) + EPS));

      if (onArc)
        b.minx = silhouettePoint.x;
    }

    // Right point (0 degrees, +X direction)
    silhouettePoint = v3(m.center.x + m.radius, m.center.y, 0.0f);
    if (len(silhouettePoint - m.center) > TOL)
    {
      float a0 = atan2f(m.p0.y - m.center.y, m.p0.x - m.center.x);
      float a1 = atan2f(m.p1.y - m.center.y, m.p1.x - m.center.x);
      float ap = atan2f(silhouettePoint.y - m.center.y, silhouettePoint.x - m.center.x);

      bool onArc = (m.arcDir == ARC_CCW)
                       ? (angleNorm(ap) >= angleNorm(a0) - EPS && angleNorm(ap) <= angleNorm(a1) + EPS) ||
                             (angleNorm(a0) > angleNorm(a1) && (angleNorm(ap) >= angleNorm(a0) - EPS || angleNorm(ap) <= angleNorm(a1) + EPS))
                       : (angleNorm(ap) >= angleNorm(a1) - EPS && angleNorm(ap) <= angleNorm(a0) + EPS) ||
                             (angleNorm(a1) > angleNorm(a0) && (angleNorm(ap) >= angleNorm(a1) - EPS || angleNorm(ap) <= angleNorm(a0) + EPS));

      if (onArc)
        b.maxx = silhouettePoint.x;
    }

    // Top point (90 degrees, +Y direction)
    silhouettePoint = v3(m.center.x, m.center.y + m.radius, 0.0f);
    if (len(silhouettePoint - m.center) > TOL)
    {
      float a0 = atan2f(m.p0.y - m.center.y, m.p0.x - m.center.x);
      float a1 = atan2f(m.p1.y - m.center.y, m.p1.x - m.center.x);
      float ap = atan2f(silhouettePoint.y - m.center.y, silhouettePoint.x - m.center.x);

      bool onArc = (m.arcDir == ARC_CCW)
                       ? (angleNorm(ap) >= angleNorm(a0) - EPS && angleNorm(ap) <= angleNorm(a1) + EPS) ||
                             (angleNorm(a0) > angleNorm(a1) && (angleNorm(ap) >= angleNorm(a0) - EPS || angleNorm(ap) <= angleNorm(a1) + EPS))
                       : (angleNorm(ap) >= angleNorm(a1) - EPS && angleNorm(ap) <= angleNorm(a0) + EPS) ||
                             (angleNorm(a1) > angleNorm(a0) && (angleNorm(ap) >= angleNorm(a1) - EPS || angleNorm(ap) <= angleNorm(a0) + EPS));

      if (onArc)
        b.maxy = silhouettePoint.y;
    }

    // Bottom point (270 degrees, -Y direction)
    silhouettePoint = v3(m.center.x, m.center.y - m.radius, 0.0f);
    if (len(silhouettePoint - m.center) > TOL)
    {
      float a0 = atan2f(m.p0.y - m.center.y, m.p0.x - m.center.x);
      float a1 = atan2f(m.p1.y - m.center.y, m.p1.x - m.center.x);
      float ap = atan2f(silhouettePoint.y - m.center.y, silhouettePoint.x - m.center.x);

      bool onArc = (m.arcDir == ARC_CCW)
                       ? (angleNorm(ap) >= angleNorm(a0) - EPS && angleNorm(ap) <= angleNorm(a1) + EPS) ||
                             (angleNorm(a0) > angleNorm(a1) && (angleNorm(ap) >= angleNorm(a0) - EPS || angleNorm(ap) <= angleNorm(a1) + EPS))
                       : (angleNorm(ap) >= angleNorm(a1) - EPS && angleNorm(ap) <= angleNorm(a0) + EPS) ||
                             (angleNorm(a1) > angleNorm(a0) && (angleNorm(ap) >= angleNorm(a1) - EPS || angleNorm(ap) <= angleNorm(a0) + EPS));

      if (onArc)
        b.miny = silhouettePoint.y;
    }
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
static inline float line_t(const Move2D &m, Vec3 p)
{
  Vec3 d = m.p1 - m.p0;
  float L2 = dot(d, d);
  if (L2 < 1e-12f)
    return 0.0f;
  return dot(p - m.p0, d) / L2;
}

// distance-along-source used for "nearest crossing" selection
static inline float distFromStart_along(const Move2D &m, Vec3 p)
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
static inline bool is_near(const Vec3 &a, const Vec3 &b)
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
    Vec3 da = a.startDir;
    Vec3 db = b.startDir;

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

  Vec3 da = a.p1 - a.p0;
  Vec3 db = b.p1 - b.p0;

  float la = len(da);
  float lb = len(db);
  if (la < TOL || lb < TOL)
    return false;

  Vec3 ua = da * (1.0f / la);
  Vec3 ub = db * (1.0f / lb);

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
static int last_cutting_move(const Move2D *moves, int count)
{
  for (int i = 0; i < count; ++i)
  {
    if (isMotionValid(moves[i]) && moves[i].compMode == CM_OUT)
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

static inline float includedAngleDeg(Vec3 v1, Vec3 v2)
{
  // VB does: v2 = -v2
  v1 = normalize(v1);
  v2 = normalize(v2) * -1.0f;

  float c = dot(v1, v2);
  c = c2d_clamp(c, -1.0f, 1.0f);
  return rad2deg(acosf(c));
}

static inline bool isNearDir(Vec3 a, Vec3 b)
{
  a = normalize(a);
  b = normalize(b);
  return dot(a, b) > 0.9995f;
}