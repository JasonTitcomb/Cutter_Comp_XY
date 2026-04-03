
/*
 * cc_math.h
 * Jason Titcomb 2026
 * MIT License – see LICENSE file in repository root
 */

#pragma once
#include <math.h>

#define TOL 0.0001f
#define ARC_TOL_IN 0.0005f    // tolerance for arc fitting and intersection calculations; also used as the minimum gap size for corner treatment
#define GAP_TOL_IN 0.001f     // if the gap between two moves is smaller than this, we will just make a bevel instead of trying to roll (generally helps with small gaps that can cause issues for the roll logic, but setting this too high can cause visible facets in compensation results)
#define EPS 1e-7f             // general small value for float comparisons
#define PARALLEL_TOL 1e-3f    // tolerance for considering two lines as parallel
#define BEVEL_VEC_TOL 1.0e-1f // if the turn is very slight (cosine of angle is close to 1) then just do a bevel instead of a roll, to avoid creating very large roll arcs that are visually indistinguishable from a bevel but more likely to cause issues for downstream processing and for CNC execution.
#define PI 3.14159265358979323846f
#define TWO_PI 6.2831853071795864769f
#define MAX_SWEEP_DEG 359.9f
#define MIN_ARC_LEN 0.001f
#define MIN(a, b) ((a) < (b) ? (a) : (b))


float arcTol = ARC_TOL_IN;
float gapTol = GAP_TOL_IN;
static float c2d_clamp(float x, float lo, float hi) { return (x < lo) ? lo : (x > hi) ? hi
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

static Vec2 v2(float x, float y) { return {x, y}; }
static float dot(const Vec2 &a, const Vec2 &b) { return a.x * b.x + a.y * b.y; }
static float cross(const Vec2 &a, const Vec2 &b) { return a.x * b.y - a.y * b.x; }
static float len(const Vec2 &v) { return sqrtf(dot(v, v)); }
static float dist(const Vec2 &a, const Vec2 &b) { return len(a - b); }
static bool is_near(const Vec2 &a, const Vec2 &b){ Vec2 d = a - b;  return dot(d, d) <= TOL * TOL;}
static bool is_equal(const Vec2 &a, const Vec2 &b){ return dist(a, b) <= EPS; }
static bool is_equal(const float a, const float b){ return fabsf(a - b) <= EPS; }

static Vec2 normalize(const Vec2 &v)
{
  float l = len(v);
  if (l < TOL)
    return {0, 0};
  return {v.x / l, v.y / l};
}

// Left normal (rotate +90)
static Vec2 leftNormal(const Vec2 &v) { return {-v.y, v.x}; }
// Right normal (rotate -90)
static Vec2 rightNormal(const Vec2 &v) { return {v.y, -v.x}; }

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
  CORNER_CHAMFER = 1,
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
  COMP_LEFT,
  COMP_RIGHT
};

enum CompError : uint8_t
{
  CE_ERROR = 0,
  CE_ARC_RADIUS_MISMATCH,
  CE_INVALID_MOVE,
  CE_COMP_MOVE_TOO_SHORT,
  CE_ARC_LT_TOOL_RAD,
  CE_FLIPPED_ARC,
  CE_COMP_IN_CROSSING,
  CE_COMP_OUT_CROSSING,
  CE_UNRESOLVED_GAP,
  CE_OUTPUT_BUFFER_OVERFLOW
};

enum Units : uint8_t
{
  UNITS_MM = 0,
  UNITS_INCH = 1
};


struct Move2D
{
  // char gcode_line[160] = {0};
  Vec2 p_0{0, 0}; // working start
  Vec2 p_1{0, 0}; // working end
  Vec2 center{0, 0};
  Vec2 rollCtr{0, 0};// center of roll arc for corner treatment.
  Vec2 startDir{0, 0};
  Vec2 endDir{0, 0};
  float radius = 0.0f;
  float feed = 0.0f;
  float z_0 = 0.0f;
  float z_1 = 0.0f;
  uint32_t lnNum = 0; // for debugging

  MotionType type = MOT_EMPTY;
  ArcDir arcDir = ARC_CW;
  CompMode compMode = CM_NONE;
  bool hasXY = false;
  bool hasZ = false;
  bool valid = true;
  bool suppressOutput = false;
};

static void update_vectors(Move2D &m)
{
  if (m.type == MOT_LINE)//TODO: do i need to update line vectors?
  {
    Vec2 d = m.p_1 - m.p_0;
    if (dot(d, d) < TOL * TOL)
      return;

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

// Recover the original endpoint of an offset line segment
static Vec2 roll_center(const Vec2 &p_offset, const Vec2 &dir, bool useLeft, float toolR)
{
  Vec2 normal = useLeft ? leftNormal(dir) : rightNormal(dir);
  return p_offset - normal * toolR;
}

static bool is_radius_consistent(const Move2D &m)
{
  float r0 = len(m.p_0 - m.center);
  float r1 = len(m.p_1 - m.center);
  return fabsf(r0 - r1) <= arcTol;
}

static int get_winding_dir(Vec2 a, Vec2 b)
{
  // a and b are already unit vectors (startDir/endDir from update_vectors)
  float z = cross(a, b);
  if (z > TOL)
    return +1;
  if (z < -TOL)
    return -1;
  return 0;
}

static float angleNorm(float a)
{
  a = fmodf(a, TWO_PI);
  if (a < 0.0f)
    a += TWO_PI;
  return a;
}


static float arcSweepDeg(const Move2D &m)
{
  Vec2 r0 = (m.arcDir == ARC_CCW) ? rightNormal(m.startDir) : leftNormal(m.startDir);
  Vec2 r1 = (m.arcDir == ARC_CCW) ? rightNormal(m.endDir)   : leftNormal(m.endDir);

  float sw = 0.0f;
  if (m.arcDir == ARC_CCW) {
    sw = atan2f(cross(r0, r1), dot(r0, r1));
  } else {
    sw = atan2f(cross(r1, r0), dot(r1, r0));
  }

  if (sw < 0.0f) sw += TWO_PI;
  return sw * (180.0f / PI);
}

// static float arcSweepDeg_(Move2D &m)
// {
//   float a0 = wrap2pi(atan2f(m.p_0.y - m.center.y, m.p_0.x - m.center.x));
//   float a1 = wrap2pi(atan2f(m.p_1.y - m.center.y, m.p_1.x - m.center.x));

//   if (m.arcDir == ARC_CCW)
//   {
//     float sw = a1 - a0;
//     if (sw < 0)
//       sw += TWO_PI;

//     // calculate length from sweep and radius.
//     // m.length = fabsf(m.radius) * sw;
//     return sw * (180.0f / PI); // [0, 360)
//   }
//   else
//   { // ARC_CW
//     float sw = a0 - a1;
//     if (sw < 0)
//       sw += TWO_PI;
//     // m.length = fabsf(m.radius) * sw;
//     return sw * (180.0f / PI); // [0, 360)
//   }
// }

static float sweepCCW(float a0, float a1)
{
  a0 = angleNorm(a0);
  a1 = angleNorm(a1);
  float d = a1 - a0;
  if (d < 0)
    d += TWO_PI;
  return d;
}

static float sweepCW(float a0, float a1)
{
  // CW from a0 to a1 is CCW from a1 to a0
  return sweepCCW(a1, a0);
}

static bool angle_on_arc_norm(float a0n, float a1n, float apn, ArcDir dir)
{
  if (dir == ARC_CCW)
  {
    return (apn >= a0n - EPS && apn <= a1n + EPS) ||
           (a0n > a1n && (apn >= a0n - EPS || apn <= a1n + EPS));
  }

  return (apn >= a1n - EPS && apn <= a0n + EPS) ||
         (a1n > a0n && (apn >= a1n - EPS || apn <= a0n + EPS));
}


// param along LINE (0..1 if on segment)
static float line_t(const Move2D &m, Vec2 p)
{
  Vec2 d = m.p_1 - m.p_0;
  float L2 = dot(d, d);
  if (L2 < 1e-12f)
    return 0.0f;
  return dot(p - m.p_0, d) / L2;
}

// distance-along-source used for "nearest crossing" selection
static float distFromStart_along(const Move2D &m, Vec2 p)
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


static bool isMotionValid(const Move2D &m)
{
  return m.valid && m.type != MOT_EMPTY;
}

// Check if two elements are colinear.
// For lines, checks if directions are parallel. For arcs, checks if centers/radii match.
static bool isColinearWith(const Move2D &a, const Move2D &b)
{
  // Must be same element type
  if (a.type != b.type)
    return false;

  // -------- LINE vs LINE --------
  if (a.type == MOT_LINE)
  {
    // startDir is already a unit vector from update_vectors
    Vec2 da = a.startDir;
    Vec2 db = b.startDir;

    if (len(da) < TOL || len(db) < TOL)
      return false;

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

static bool angleOnSweepCCW(float a0, float a1, float ap)
{
  a0 = angleNorm(a0);
  a1 = angleNorm(a1);
  ap = angleNorm(ap);
  if (a0 <= a1)
    return (ap + EPS >= a0) && (ap <= a1 + EPS);
  // wrap
  return (ap >= a0 - EPS) || (ap <= a1 + EPS);
}

static bool angleOnSweepCW(float a0, float a1, float ap)
{
  // CW sweep from a0 down to a1 is CCW from a1 to a0
  return angleOnSweepCCW(a1, a0, ap);
}

static bool pointOnSegment(Vec2 a, Vec2 b, Vec2 p)
{
  Vec2 ab = b - a;
  float lab2 = dot(ab, ab);
  if (lab2 < TOL)
  {
    Vec2 pa = p - a;
    return dot(pa, pa) < TOL * TOL;
  }

  float t = dot(p - a, ab) / lab2;
  if (t < -TOL || t > 1.0f + TOL)
    return false;

  float c = cross(p - a, ab);
  return (c * c) < (TOL * TOL * lab2);
}


/* ── Pre-computed arc sweep angles ───────────────────────────
   When testing multiple candidate points against the same arc,
   the arc's own start/end angles (a0, a1) are constant.
   Pre-computing them once avoids redundant atan2f calls:
     original pointOnArc  = 3× atan2f per call  (a0, a1, ap)
     pointOnArcCached     = 1× atan2f per call   (ap only)

   Typical savings in the crossing / TIP hot-path:
     LINE-ARC  (2 candidates): 6 → 4  atan2f  (saves 2)
     ARC-ARC   (2 candidates): 12 → 8 atan2f  (saves 4)
*/
struct ArcAngles
{
  float a0;   /* angle of arc start point  (p_0 relative to center) */
  float a1;   /* angle of arc end point    (p_1 relative to center) */
  ArcDir dir; /* CW or CCW sweep direction                         */
};

/* Compute start/end angles for an arc move (2× atan2f).
   Call once per arc, then pass to pointOnArcCached(). */
static ArcAngles precomputeArcAngles(const Move2D &m)
{
  ArcAngles aa;
  aa.a0 = atan2f(m.p_0.y - m.center.y, m.p_0.x - m.center.x);
  aa.a1 = atan2f(m.p_1.y - m.center.y, m.p_1.x - m.center.x);
  aa.dir = m.arcDir;
  return aa;
}

/* Test whether point p lies on the arc, reusing pre-computed sweep angles.
   Only 1× atan2f per call (for the test point) instead of 3×. */
static bool pointOnArcCached(const Move2D &a, Vec2 p, const ArcAngles &aa)
{
  /* Radius check — cheapest rejection test, no trig needed */
  float rp = len(p - a.center);
  if (fabsf(rp - a.radius) > TOL)
    return false;

  /* Angle of the test point relative to arc center (the only atan2f) */
  float ap = atan2f(p.y - a.center.y, p.x - a.center.x);

  if (aa.dir == ARC_CCW)
    return angleOnSweepCCW(aa.a0, aa.a1, ap);
  else
    return angleOnSweepCW(aa.a0, aa.a1, ap);
}

static IntersectType intersectLineLine(const Move2D &ln1, const Move2D &ln2, Vec2 &ip, bool &tip)
{
  Vec2 p = ln1.p_0;
  Vec2 q = ln2.p_0;

  float lr = dist(ln1.p_0, ln1.p_1);
  float ls = dist(ln2.p_0, ln2.p_1);
  Vec2 r = ln1.startDir;
  Vec2 s = ln2.startDir;

  if (lr < TOL || ls < TOL || len(r) < TOL || len(s) < TOL)
  {
    tip = false;
    //return IT_NONE;
  }

  float den = cross(r, s);
  float denTol = PARALLEL_TOL * len(r) * len(s);
  if (fabsf(den) <= denTol)
  {
    tip = false; // too parallel to reliably intersect
    return IT_NONE;
  }

  float t = cross(q - p, s) / den;
  float u = cross(q - p, r) / den;
  ip = p + r * t;

  tip = (t >= -TOL && t <= lr + TOL && u >= -TOL && u <= ls + TOL);
  return IT_INTERSECT;
}

static IntersectType intersectCircleCircle(const Move2D &a1, const Move2D &a2, Vec2 &p1, Vec2 &p2, int &count)
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

static IntersectType intersectLineCircle(Vec2 l1, Vec2 a1, Vec2 ctr, float r, Vec2 &p1, Vec2 &p2, int &count)
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

