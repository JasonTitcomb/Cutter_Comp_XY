#pragma once
#include <stdint.h>
#include <math.h>

// ----- basic types -----
struct Vec2 {
  float x=0, y=0;
};

static inline Vec2 operator+(Vec2 a, Vec2 b){ return {a.x+b.x, a.y+b.y}; }
static inline Vec2 operator-(Vec2 a, Vec2 b){ return {a.x-b.x, a.y-b.y}; }
static inline Vec2 operator*(Vec2 a, float s){ return {a.x*s, a.y*s}; }

static inline float dot(Vec2 a, Vec2 b){ return a.x*b.x + a.y*b.y; }
static inline float crossZ(Vec2 a, Vec2 b){ return a.x*b.y - a.y*b.x; }
static inline float len(Vec2 a){ return sqrtf(dot(a,a)); }
static inline Vec2 norm(Vec2 v){ float L=len(v); return (L>1e-9f)? Vec2{v.x/L,v.y/L} : Vec2{0,0}; }
static inline float dist(Vec2 a, Vec2 b){ return len(a-b); }

enum ElementType : uint8_t { EL_LINE=0, EL_ARC_CW=1, EL_ARC_CCW=2, EL_RAPID=3 };
enum Side : uint8_t { SIDE_LEFT=0, SIDE_RIGHT=1 };
enum CutterComp : uint8_t { CC_OFF=0, CC_IN=1, CC_OUT=2, CC_LEFT=3, CC_RIGHT=4 };

// This is your "segment/move" object (adapt to your existing Move2D)
struct Move2D {
  ElementType type = EL_LINE;
  Vec2 p1{}, p2{};     // start/end
  Vec2 ctr{};          // for arcs
  float r = 0;         // for arcs
  bool  valid = true;
  bool  trimBoundary = false;
  CutterComp comp = CC_OFF;

  // Derived directions (like VB's StartDirection / EndDirection)
  Vec2 startDir{}; // tangent direction at start (unit)
  Vec2 endDir{};   // tangent direction at end (unit)

  // --- these should be implemented by your geometry layer ---
  bool wallOffset(float uniform, float dx, float dy, Side side); // compute offset geometry
  void setToOffsetPoints();                                      // apply stored offset pts into p1/p2/ctr/r/dirs

  // Intersection results (like VB's CalcIntersect populating TIP/FIP state)
  // You can implement these however you want; WallOffset2D only needs:
  enum IntersectType : uint8_t { IT_NONE=0, IT_TANGENT=1, IT_INTERSECT=2 };
  IntersectType calcIntersect(const Move2D& other); // should cache potential intersection points somewhere

  // TIP/FIP candidates from last calcIntersect()
  bool hasTIP1=false, hasTIP2=false;  Vec2 TIP1{}, TIP2{};
  bool hasFIP1=false, hasFIP2=false;  Vec2 FIP1{}, FIP2{};

  // Trimming/extending (like VB TrimOffEnd/TrimOffStart/ExtendEnd/ExtendStart)
  void trimOffEnd(Vec2 pt);
  void trimOffStart(Vec2 pt);
  void extendEnd(Vec2 pt);
  void extendStart(Vec2 pt);

  // For VB's DistFromStart(pt) used in “nearest crossing”
  float distFromStart(Vec2 pt) const { return dist(p1, pt); }

  // For VB's "InitialEndPt" (used as roll-arc center)
  Vec2 initialEndPt{}; // stash original end before trimming if you want identical behavior
};

class WallOffset2D {
public:
  float cornerAngleToleranceDeg = 30.0f; // VB CornerAngleTolerance
  Side  offsetSide = SIDE_RIGHT;
  float offsetDist = 0;   // uniform (VB mOffsetDist)
  float offsetX = 0, offsetY = 0; // anisotropic (VB mOffsetDistX/Y)
  bool  performTrim = true;
  bool  useCutterComp = false;

  // Fixed output buffer (embedded friendly)
  static constexpr int MAX_OUT = 512;
  Move2D out[MAX_OUT];
  int outCount = 0;

  void begin(float uniformOffset, Side side){
    offsetDist = uniformOffset;
    offsetX = offsetY = 0;
    offsetSide = side;
    outCount = 0;
    prevValid = false;
  }

  // Feed already-parsed moves one by one (lines/arcs only for now)
  void addMove(const Move2D& inMove, bool forceRoll){
    Move2D cur = inMove;

    // --- 1) offset the element like VB: WallOffset() then SetToOffsetPoints() ---
    if(cur.trimBoundary){
      (void)cur.wallOffset(offsetDist, 0, 0, offsetSide);
    } else {
      (void)cur.wallOffset(offsetDist, offsetX, offsetY, offsetSide);
    }
    cur.setToOffsetPoints();

    if(!prevValid){
      push(cur);
      prev = cur;
      prevValid = true;
      return;
    }

    // --- 2) acute / comping flags like VB ---
    bool acute = false;
    float incAngDeg = includedAngleDeg(prev.endDir, cur.startDir);
    if(!forceRoll && incAngDeg < cornerAngleToleranceDeg) acute = true;

    bool comping = (prev.comp > CC_RIGHT) || (cur.comp > CC_RIGHT);

    // --- 3) dispatch by combo like VB ---
    Combo combo = getCombo(prev, cur);
    switch(combo){
      case Combo::LINE_LINE:   handleLineLine(prev, cur, acute, forceRoll, comping); break;
      case Combo::ARC_ARC:     handleArcArc(prev, cur, acute, forceRoll); break;
      case Combo::ARC_LINE:
      case Combo::LINE_ARC:    handleArcLine(prev, cur, acute, forceRoll, comping); break;
      default: break;
    }

    if(cur.valid) push(cur);
    prev = cur;
  }

private:
  Move2D prev;
  bool prevValid=false;

  enum class Combo : uint8_t { NONE=0, LINE_LINE, ARC_ARC, ARC_LINE, LINE_ARC };

  static float includedAngleDeg(Vec2 a, Vec2 b){
    a = norm(a); b = norm(b);
    float c = dot(a,b);
    if(c>1) c=1; if(c<-1) c=-1;
    return acosf(c) * (180.0f / 3.1415926535f);
  }

  static int windingDirection(Vec2 v1, Vec2 v2){
    float z = crossZ(v1, v2);
    if(z > 0) return +1;
    if(z < 0) return -1;
    return 0;
  }

  bool convex(const Move2D& p, const Move2D& c) const {
    int cw = windingDirection(p.endDir, c.startDir);
    if(cw == 0) return false;
    if(offsetSide == SIDE_LEFT) return !(cw > 0);
    return (cw > 0);
  }

  static Combo getCombo(const Move2D& a, const Move2D& b){
    bool aLine = (a.type == EL_LINE || a.type == EL_RAPID);
    bool bLine = (b.type == EL_LINE || b.type == EL_RAPID);
    bool aArc  = (a.type == EL_ARC_CW || a.type == EL_ARC_CCW);
    bool bArc  = (b.type == EL_ARC_CW || b.type == EL_ARC_CCW);
    if(aLine && bLine) return Combo::LINE_LINE;
    if(aArc && bArc)   return Combo::ARC_ARC;
    if(aArc && bLine)  return Combo::ARC_LINE;
    if(aLine && bArc)  return Combo::LINE_ARC;
    return Combo::NONE;
  }

  void push(const Move2D& m){
    if(outCount < MAX_OUT) out[outCount++] = m;
  }

  // --- Helpers that match VB names/behavior ---
  void trimToCommonTIP(Move2D& a, Move2D& b, Vec2 tip){
    if(!performTrim) return;
    a.trimOffEnd(tip);
    b.trimOffStart(tip);
  }

  bool extendToCommonFIP(Move2D& a, Move2D& b, Vec2 fip){
    if(!performTrim) return false;
    // VB checks direction constraints before extending :contentReference[oaicite:4]{index=4}
    float fipDir1 = dot(fip - a.p2, a.endDir);
    float fipDir2 = dot(fip - b.p1, b.startDir);
    if(fipDir1 > 0 && fipDir2 < 0){
      a.extendEnd(fip);
      b.extendStart(fip);
      return true;
    }
    return false;
  }

  // VB: InsertArcBetweenElements() uses center = el1.InitialEndPt and arc dir based on side :contentReference[oaicite:5]{index=5}
  void insertArcBetweenElements(const Move2D& a, const Move2D& b){
    Move2D roll;
    roll.type = (offsetSide == SIDE_LEFT) ? EL_ARC_CW : EL_ARC_CCW;
    roll.p1 = a.p2;
    roll.p2 = b.p1;
    roll.ctr = a.initialEndPt;  // match VB behavior exactly
    roll.r = dist(roll.ctr, roll.p1);
    roll.valid = true;
    push(roll);
  }

  // VB: InsertArcExtension() inserts a LINE “extension” and trims one side to the intersection :contentReference[oaicite:6]{index=6}
  void insertArcExtension(Move2D& e1, Move2D& e2){
    // This requires your line/arc intersection + trimExtend logic.
    // Implementation detail depends on your geometry layer.
    // But the *intent* is: create a line that forces e1/e2 to meet at a usable point (no true intersection).
  }

  // --- Combo handlers (mirror VB decision tree) ---
  void handleLineLine(Move2D& a, Move2D& b, bool acute, bool forceRoll, bool comping){
    if(isParallel(a,b)) return;

    (void)a.calcIntersect(b);

    // TIP logic
    bool haveTIP = (a.hasTIP1 || a.hasTIP2) && (b.hasTIP1 || b.hasTIP2);
    Vec2 tip = commonTIP(a,b);
    bool haveFIP = commonFIP(a,b, /*out*/tip);

    if(acute || forceRoll){
      if(convex(a,b)){
        insertArcBetweenElements(a,b);
      } else if(haveTIP){
        trimToCommonTIP(a,b, tip);
      } else if(comping && haveFIP){
        extendToCommonFIP(a,b, tip);
      }
    } else {
      if(haveTIP) trimToCommonTIP(a,b, tip);
      else if(haveFIP) extendToCommonFIP(a,b, tip);
    }
  }

  void handleArcArc(Move2D& a, Move2D& b, bool acute, bool forceRoll){
    if(dist(a.p2, b.p1) < 1e-6f || dist(a.ctr, b.ctr) < 1e-6f) return;

    auto it = a.calcIntersect(b);
    if(it == Move2D::IT_NONE){
      // VB: MUST roll if arcs can’t intersect :contentReference[oaicite:7]{index=7}
      insertArcBetweenElements(a,b);
      return;
    }
    Vec2 tip{};
    bool haveTIP = commonTIP2(a,b, tip);

    if(it == Move2D::IT_TANGENT){
      if(haveTIP) trimToCommonTIP(a,b, tip);
      return;
    }

    if(haveTIP){
      trimToCommonTIP(a,b, tip);
    } else {
      if(acute || forceRoll){
        insertArcBetweenElements(a,b);
      } else {
        Vec2 fip{};
        if(commonFIP2(a,b,fip)) extendToCommonFIP(a,b,fip);
      }
    }
  }

  void handleArcLine(Move2D& a, Move2D& b, bool acute, bool forceRoll, bool comping){
    if(isChained(a,b)) return;

    auto it = a.calcIntersect(b);

    if(it == Move2D::IT_NONE){
      if(convex(a,b)) insertArcBetweenElements(a,b);
      return;
    }

    if(it == Move2D::IT_TANGENT){
      // VB: if directions match, extend one side to meet; else roll :contentReference[oaicite:8]{index=8}
      if(isNearDir(a.endDir, b.startDir)){
        if(isArc(a)) b.extendStart(a.p2);
        else         a.extendEnd(b.p1);
      } else {
        insertArcBetweenElements(a,b);
      }
      return;
    }

    Vec2 tip{};
    if(commonTIP2(a,b, tip)){
      trimToCommonTIP(a,b, tip);
      return;
    }

    // No true intersection:
    if(comping){
      insertArcExtension(a,b);
      return;
    }

    Vec2 fip{};
    if(acute || forceRoll){
      if(convex(a,b)) insertArcBetweenElements(a,b);
    } else {
      if(commonFIP2(a,b,fip)) extendToCommonFIP(a,b,fip);
    }
  }

  // --- small geometric helpers you likely already have ---
  static bool isArc(const Move2D& m){ return m.type==EL_ARC_CW || m.type==EL_ARC_CCW; }

  static bool isNearDir(Vec2 a, Vec2 b){
    a=norm(a); b=norm(b);
    return dot(a,b) > 0.999f;
  }

  static bool isChained(const Move2D& a, const Move2D& b){
    return dist(a.p2, b.p1) < 1e-6f || dist(a.p1, b.p2) < 1e-6f;
  }

  static bool isParallel(const Move2D& a, const Move2D& b){
    Vec2 da = norm(a.endDir);
    Vec2 db = norm(b.startDir);
    return fabsf(crossZ(da,db)) < 1e-6f;
  }

  // These “common TIP/FIP” helpers are direct translations of your VB TryGetCommonTIP/FIP logic :contentReference[oaicite:9]{index=9}
  static Vec2 commonTIP(const Move2D& a, const Move2D& b){
    // simplest: return a TIP that matches any of b’s TIPs
    if(a.hasTIP1){
      if(b.hasTIP1 && dist(a.TIP1,b.TIP1)<1e-6f) return a.TIP1;
      if(b.hasTIP2 && dist(a.TIP1,b.TIP2)<1e-6f) return a.TIP1;
    }
    if(a.hasTIP2){
      if(b.hasTIP1 && dist(a.TIP2,b.TIP1)<1e-6f) return a.TIP2;
      if(b.hasTIP2 && dist(a.TIP2,b.TIP2)<1e-6f) return a.TIP2;
    }
    return a.TIP1;
  }

  static bool commonTIP2(const Move2D& a, const Move2D& b, Vec2& outTip){
    if(a.hasTIP1){
      if(b.hasTIP1 && dist(a.TIP1,b.TIP1)<1e-6f){ outTip=a.TIP1; return true; }
      if(b.hasTIP2 && dist(a.TIP1,b.TIP2)<1e-6f){ outTip=a.TIP1; return true; }
    }
    if(a.hasTIP2){
      if(b.hasTIP1 && dist(a.TIP2,b.TIP1)<1e-6f){ outTip=a.TIP2; return true; }
      if(b.hasTIP2 && dist(a.TIP2,b.TIP2)<1e-6f){ outTip=a.TIP2; return true; }
    }
    return false;
  }

  static bool commonFIP(const Move2D& a, const Move2D& b, Vec2& outFip){
    // mirror of VB TryGetCommonFIP: compare non-TIP intersection points
    if(a.hasFIP1){
      if(b.hasFIP1 && dist(a.FIP1,b.FIP1)<1e-6f){ outFip=a.FIP1; return true; }
      if(b.hasFIP2 && dist(a.FIP1,b.FIP2)<1e-6f){ outFip=a.FIP1; return true; }
    }
    if(a.hasFIP2){
      if(b.hasFIP1 && dist(a.FIP2,b.FIP1)<1e-6f){ outFip=a.FIP2; return true; }
      if(b.hasFIP2 && dist(a.FIP2,b.FIP2)<1e-6f){ outFip=a.FIP2; return true; }
    }
    return false;
  }

  static bool commonFIP2(const Move2D& a, const Move2D& b, Vec2& outFip){
    return commonFIP(a,b,outFip);
  }
};
