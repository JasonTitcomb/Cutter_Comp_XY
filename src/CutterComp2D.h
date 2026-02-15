// CutterComp2D.h  (VB.NET WallOffset.vb logic port)
// - No malloc, fixed buffers
// - 1-segment lookbehind (delayed output) so we can trim/extend previous segment
// - Corner handling matches your VB decision tree:
//   LINE-LINE, ARC-ARC, ARC-LINE/LINE-ARC with TIP/FIP, acute/forceRoll, convex, comping,
//   and InsertArcBetweenElements / InsertArcExtension behavior.
// Sources of behavior:
// - Offset decision tree (acute/convex/TIP/FIP/comping): :contentReference[oaicite:0]{index=0}
// - ExtendToCommonFIP direction tests: :contentReference[oaicite:1]{index=1}
// - InsertArcBetweenElements (center=InitialEndPt, CW/CCW based on side): :contentReference[oaicite:2]{index=2}
// - InsertArcExtension (bridge line insertion): :contentReference[oaicite:3]{index=3}

#pragma once
#include "Common2D.h"

class CutterComp2D
{
public:
    // Tune these to taste
    float cornerAngleToleranceDeg = 30.0f; // VB CornerAngleTolerance :contentReference[oaicite:4]{index=4}
    bool cornerRolling = true;             // VB rollAround flag
    bool performTrim = true;               // VB mPerformTrim

    // Buffers
    static constexpr int IN_CAP = 16;
    static constexpr int OUT_CAP = 32;

    void setToolRadius(float r) { toolR = (r < 0) ? -r : r; }
    void setComp(CompSide s)
    {
        comp = s;
        resetState();
    }
    void setCornerRolling(bool en) { cornerRolling = en; }
    void setCornerAngleTolerance(float deg) { cornerAngleToleranceDeg = deg; }
    void setPerformTrim(bool en) { performTrim = en; }

    bool pushIn(const Move2D &m)
    {
        if (inCount >= IN_CAP)
            return false;
        in[(inHead + inCount) % IN_CAP] = m;
        inCount++;
        return true;
    }

    // Main pump
    void process(bool forceRoll = false)
    {
        // If comp is OFF, just pass through immediately (no delay needed)
        if (comp == COMP_OFF || toolR < C2D_EPS)
        {
            while (inCount > 0)
            {
                if (!outHasSpace(1))
                    return;
                Move2D m = popIn();
                pushOut(m);
            }
            return;
        }

        // Comp ON: offset + VB-style pair logic with 1-element delay
        while (inCount > 0)
        {
            // Need output space for worst case: prev + (extension line OR roll arc) (2 moves)
            if (!outHasSpace(3))
                return;

            Move2D raw = popIn();
            if (raw.type == MOT_EMPTY)
                continue;

            Move2D curOff;
            if (!offsetMove(raw, curOff))
            {
                // fallback
                if (!havePrev)
                {
                    prevOff = raw;
                    havePrev = true;
                }
                else
                {
                    pushOut(prevOff);
                    prevOff = raw;
                }
                continue;
            }

            // Stash "InitialEndPt" equivalent: end BEFORE trimming/extension
            curOff.initialEndPt = raw.p1;

            update_dirs(curOff);

            if (!havePrev)
            {
                prevOff = curOff;
                havePrev = true;
                continue;
            }

            // Apply VB decision tree between prevOff and curOff
            Move2D inserts[2];
            int insertCount = 0;

            applyVBLogic(prevOff, curOff, forceRoll, inserts, insertCount);

            // Emit previous + inserts; hold curOff as new prev
            pushOut(prevOff);
            for (int i = 0; i < insertCount; ++i)
                pushOut(inserts[i]);

            prevOff = curOff;
        }
    }

    // Flush delayed last element
    void flush()
    {
        process(false);
        if (comp != COMP_OFF && toolR >= C2D_EPS)
        {
            if (havePrev && outHasSpace(1))
            {
                pushOut(prevOff);
                havePrev = false;
            }
        }
    }

    bool popOut(Move2D &m)
    {
        if (outCount == 0)
            return false;
        m = out[outHead];
        outHead = (outHead + 1) % OUT_CAP;
        outCount--;
        return true;
    }

private:
    // ---- Common2D.h additions required ----
    // Add these fields to Move2D in Common2D.h:
    //   Vec2 initialEndPt;
    //   Vec2 startDir;
    //   Vec2 endDir;
    //
    // And add update_dirs(Move2D&) helper (see below in this header, provided).
    //
    // If you already added them per our earlier message, you're set.

    // Ring buffers
    Move2D in[IN_CAP];
    int inHead = 0, inCount = 0;

    Move2D out[OUT_CAP];
    int outHead = 0, outCount = 0;

    // Settings
    float toolR = 0.0f;
    CompSide comp = COMP_OFF;

    // Delayed output state
    bool havePrev = false;
    Move2D prevOff;

private:
    // ---------- small helpers ----------
    bool outHasSpace(int n) const { return (outCount + n) <= OUT_CAP; }

    Move2D popIn()
    {
        Move2D m = in[inHead];
        inHead = (inHead + 1) % IN_CAP;
        inCount--;
        return m;
    }

    void pushOut(const Move2D &m)
    {
        out[(outHead + outCount) % OUT_CAP] = m;
        outCount++;
    }

    void resetState() { havePrev = false; }

    static inline float rad2deg(float r) { return r * (180.0f / (float)M_PI); }

    static inline float angleNorm(float a)
    {
        while (a < 0)
            a += 2.0f * (float)M_PI;
        while (a >= 2.0f * (float)M_PI)
            a -= 2.0f * (float)M_PI;
        return a;
    }

    static inline bool angleOnSweepCCW(float a0, float a1, float ap)
    {
        a0 = angleNorm(a0);
        a1 = angleNorm(a1);
        ap = angleNorm(ap);
        if (a0 <= a1)
            return (ap + 1e-7f >= a0) && (ap <= a1 + 1e-7f);
        // wrap
        return (ap >= a0 - 1e-7f) || (ap <= a1 + 1e-7f);
    }

    static inline bool angleOnSweepCW(float a0, float a1, float ap)
    {
        // CW sweep from a0 down to a1 is CCW from a1 to a0
        return angleOnSweepCCW(a1, a0, ap);
    }

    static inline float includedAngleDeg(Vec2 a, Vec2 b)
    {
        a = normalize(a);
        b = normalize(b);
        float c = dot(a, b);
        c = c2d_clamp(c, -1.0f, 1.0f);
        return rad2deg(acosf(c));
    }

    static inline bool isNearDir(Vec2 a, Vec2 b)
    {
        a = normalize(a);
        b = normalize(b);
        return dot(a, b) > 0.9995f;
    }

    bool convex(const Move2D &a, const Move2D &b) const
    {
        int cw = WindingDirection(a.endDir, b.startDir);
        if (cw == 0)
            return false;

        // VB:
        // If mOffsetSide = OffsetSide.LEFT Then Return Not(cw > 0) Else Return cw > 0
        // Map: COMP_LEFT == OffsetSide.LEFT
        if (comp == COMP_LEFT)
        {
            return !(cw > 0);
        }
        else
        {
            return (cw > 0);
        }
    }

    // ---------- offset primitives ----------
    bool offsetMove(const Move2D &src, Move2D &dst)
    {
        if (src.type == MOT_LINE)
            return offsetLine(src, dst);
        if (src.type == MOT_ARC)
            return offsetArc(src, dst);
        return false;
    }

    bool offsetLine(const Move2D &src, Move2D &dst)
    {
        Vec2 v = src.p1 - src.p0;
        float l = len(v);
        if (l < C2D_EPS)
            return false;
        Vec2 u = v * (1.0f / l);

        Vec2 n = (comp == COMP_LEFT) ? leftNormal(u) : rightNormal(u);
        Vec2 off = n * toolR;

        dst = src;
        dst.type = MOT_LINE;
        dst.p0 = src.p0 + off;
        dst.p1 = src.p1 + off;
        return true;
    }

    // Concentric arc offset like before (good enough for your VB logic)
    bool offsetArc(const Move2D &src, Move2D &dst)
    {
        float r0 = src.radius;
        if (r0 < C2D_EPS)
            r0 = len(src.p0 - src.center);
        if (r0 < C2D_EPS)
            return false;

        float dr = toolR;
        bool ccw = (src.arcDir == ARC_CCW);
        bool left = (comp == COMP_LEFT);

        float r1;
        if (ccw)
            r1 = r0 + (left ? -dr : +dr);
        else
            r1 = r0 + (left ? +dr : -dr);

        if (r1 < C2D_EPS)
            return false;

        Vec2 v0 = src.p0 - src.center;
        Vec2 v1 = src.p1 - src.center;
        float lv0 = len(v0), lv1 = len(v1);
        if (lv0 < C2D_EPS || lv1 < C2D_EPS)
            return false;

        dst = src;
        dst.type = MOT_ARC;
        dst.center = src.center;
        dst.radius = r1;
        dst.p0 = src.center + v0 * (r1 / lv0);
        dst.p1 = src.center + v1 * (r1 / lv1);
        return true;
    }

    // ---------- direction/tangent calculation (like VB StartDirection / EndDirection) ----------
    // This function expects you added startDir/endDir fields in Move2D.
    static inline void update_dirs(Move2D &m)
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
            Vec2 rs = normalize(m.p0 - m.center);
            Vec2 re = normalize(m.p1 - m.center);
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

    // ---------- intersection tests producing TIP/FIP ----------
    enum IntersectType : uint8_t
    {
        IT_NONE = 0,
        IT_TANGENT = 1,
        IT_INTERSECT = 2
    };

    static inline bool pointOnSegment(Vec2 a, Vec2 b, Vec2 p)
    {
        Vec2 ab = b - a;
        float lab2 = dot(ab, ab);
        if (lab2 < 1e-12f)
            return (len(p - a) < 1e-4f);
        float t = dot(p - a, ab) / lab2;
        if (t < -1e-6f || t > 1.0f + 1e-6f)
            return false;
        // distance to line
        float d = fabsf(cross(p - a, ab)) / sqrtf(lab2);
        return d < 1e-4f;
    }

    static inline bool pointOnArc(const Move2D &a, Vec2 p)
    {
        // radius match
        float rp = len(p - a.center);
        if (fabsf(rp - a.radius) > 5e-4f)
            return false;

        float a0 = atan2f(a.p0.y - a.center.y, a.p0.x - a.center.x);
        float a1 = atan2f(a.p1.y - a.center.y, a.p1.x - a.center.x);
        float ap = atan2f(p.y - a.center.y, p.x - a.center.x);

        if (a.arcDir == ARC_CCW)
            return angleOnSweepCCW(a0, a1, ap);
        else
            return angleOnSweepCW(a0, a1, ap);
    }

    static IntersectType intersectLineLine(const Move2D &A, const Move2D &B, Vec2 &ip, bool &tip)
    {
        Vec2 p = A.p0;
        Vec2 r = A.p1 - A.p0;
        Vec2 q = B.p0;
        Vec2 s = B.p1 - B.p0;

        float den = cross(r, s);
        if (fabsf(den) < 1e-9f)
        {
            tip = false;
            return IT_NONE;
        }

        float t = cross(q - p, s) / den;
        float u = cross(q - p, r) / den;
        ip = p + r * t;

        tip = (t >= -1e-6f && t <= 1.0f + 1e-6f && u >= -1e-6f && u <= 1.0f + 1e-6f);
        return IT_INTERSECT;
    }

    static IntersectType intersectCircleCircle(const Move2D &A, const Move2D &B, Vec2 &p1, Vec2 &p2, int &count)
    {
        // circles defined by center/radius
        Vec2 c0 = A.center, c1 = B.center;
        float r0 = A.radius, r1 = B.radius;
        Vec2 d = c1 - c0;
        float distc = len(d);
        count = 0;

        if (distc < 1e-9f)
            return IT_NONE;
        if (distc > r0 + r1 + 1e-6f)
            return IT_NONE;
        if (distc < fabsf(r0 - r1) - 1e-6f)
            return IT_NONE;

        float a = (r0 * r0 - r1 * r1 + distc * distc) / (2.0f * distc);
        float h2 = r0 * r0 - a * a;
        Vec2 u = d * (1.0f / distc);
        Vec2 mid = c0 + u * a;

        if (fabsf(h2) < 1e-8f)
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

    static IntersectType intersectLineCircle(Vec2 a, Vec2 b, Vec2 c, float r, Vec2 &p1, Vec2 &p2, int &count)
    {
        // Infinite line through a->b
        Vec2 d = b - a;
        float L2 = dot(d, d);
        count = 0;
        if (L2 < 1e-12f)
            return IT_NONE;

        Vec2 f = a - c;

        float A = dot(d, d);
        float B = 2.0f * dot(f, d);
        float C = dot(f, f) - r * r;

        float disc = B * B - 4 * A * C;
        if (disc < -1e-7f)
            return IT_NONE;
        if (fabsf(disc) < 1e-7f)
        {
            float t = -B / (2 * A);
            p1 = a + d * t;
            count = 1;
            return IT_TANGENT;
        }

        float sdisc = sqrtf(fmaxf(0.0f, disc));
        float t1 = (-B + sdisc) / (2 * A);
        float t2 = (-B - sdisc) / (2 * A);
        p1 = a + d * t1;
        p2 = a + d * t2;
        count = 2;
        return IT_INTERSECT;
    }

    // Choose point closer to prev end (matches VB using mTip1 first in practice)
    static Vec2 pickClosest(Vec2 ref, Vec2 a, Vec2 b)
    {
        return (len(a - ref) <= len(b - ref)) ? a : b;
    }

    // ---------- VB operations ----------
    // TrimToCommonTIP :contentReference[oaicite:7]{index=7}
    void trimToTIP(Move2D &a, Move2D &b, Vec2 tip)
    {
        if (!performTrim)
            return;
        a.p1 = tip;
        b.p0 = tip;
        update_dirs(a);
        update_dirs(b);
    }

    // ExtendToCommonFIP with dot tests :contentReference[oaicite:8]{index=8}
    bool extendToFIP(Move2D &a, Move2D &b, Vec2 fip)
    {
        if (!performTrim)
            return false;
        float fipDir1 = dot(fip - a.p1, a.endDir);
        float fipDir2 = dot(fip - b.p0, b.startDir);
        if (fipDir1 > 0 && fipDir2 < 0)
        {
            a.p1 = fip;
            b.p0 = fip;
            update_dirs(a);
            update_dirs(b);
            return true;
        }
        return false;
    }

    // InsertArcBetweenElements: center = el1.InitialEndPt, CW/CCW based on side :contentReference[oaicite:9]{index=9}
    Move2D makeRollArc(const Move2D &a, const Move2D &b) const
    {
        Move2D roll;
        roll.type = MOT_ARC;
        roll.rapid = false;
        roll.feed = (a.feed > 0) ? a.feed : b.feed;
        roll.p0 = a.p1;
        roll.p1 = b.p0;
        roll.center = a.initialEndPt;
        roll.radius = len(roll.p0 - roll.center);
        // VB: if offsetSide LEFT => CW else CCW :contentReference[oaicite:10]{index=10}
        roll.arcDir = (comp == COMP_LEFT) ? ARC_CW : ARC_CCW;
        update_dirs(roll);
        return roll;
    }

    // InsertArcExtension :contentReference[oaicite:11]{index=11}
    // Creates a line segment bridging arc<->line when comping and no TIP.
    // We mimic the VB approach: build a "probe" line along arc tangent, intersect with the other line,
    // then trim/extend endpoints and insert the bridging line.
    bool makeArcExtension(Move2D &a, Move2D &b, Move2D &extOut)
    {
        // Case: a is LINE, b is ARC
        if (a.type == MOT_LINE && b.type == MOT_ARC)
        {

            // VB: sp = e2.P1 + Negate(e2.StartDirection)
            Vec2 sp = b.p0 - b.startDir; // 1 unit back along arc start tangent

            // Probe line from sp -> b.p0 intersects line a
            Vec2 ip;
            bool tip = false;
            Move2D probe;
            probe.type = MOT_LINE;
            probe.p0 = sp;
            probe.p1 = b.p0;

            if (intersectLineLine(probe, a, ip, tip) == IT_NONE)
                return false;

            // Direction checks (VB-style idea):
            // - ip must be forward of a's end direction (extend end)
            // - ip must be "behind" b's start direction (extend/trim start toward it)
            float da = dot(ip - a.p1, a.endDir);
            float db = dot(ip - b.p0, b.startDir);

            if (!(da > 0 && db < 0))
                return false;

            // Extend/trim:
            a.p1 = ip;
            update_dirs(a);

            // Insert extension line from intersection to arc start
            extOut.type = MOT_LINE;
            extOut.rapid = false;
            extOut.feed = (a.feed > 0) ? a.feed : b.feed;
            extOut.p0 = ip;
            extOut.p1 = b.p0;
            update_dirs(extOut);

            return true;
        }

        // Case: a is ARC, b is LINE
        if (a.type == MOT_ARC && b.type == MOT_LINE)
        {

            // VB: exLine from e1.P2 to e1.P2 + e1.EndDirection
            Vec2 sp = a.p1;
            Vec2 ep = a.p1 + a.endDir; // 1 unit forward along arc end tangent

            Move2D probe;
            probe.type = MOT_LINE;
            probe.p0 = sp;
            probe.p1 = ep;

            Vec2 ip;
            bool tip = false;
            if (intersectLineLine(probe, b, ip, tip) == IT_NONE)
                return false;

            // Direction checks:
            // - ip must be forward of a's end direction (extend end toward ip)
            // - ip must be behind b's start direction (move b start back to ip)
            float da = dot(ip - a.p1, a.endDir);
            float db = dot(ip - b.p0, b.startDir);

            if (!(da > 0 && db < 0))
                return false;

            // Trim/extend:
            b.p0 = ip;
            update_dirs(b);

            // Insert extension line from arc end to intersection
            extOut.type = MOT_LINE;
            extOut.rapid = false;
            extOut.feed = (a.feed > 0) ? a.feed : b.feed;
            extOut.p0 = a.p1;
            extOut.p1 = ip;
            update_dirs(extOut);

            return true;
        }

        return false;
    }

    // ---------- main VB decision tree ----------
    void applyVBLogic(Move2D &a, Move2D &b, bool forceRoll, Move2D inserts[2], int &insertCount)
    {
        insertCount = 0;
        update_dirs(a);
        update_dirs(b);
        // "accute" (spelled that way in VB) :contentReference[oaiFFcite:16]{index=16}
        bool acute = false;
        if (!forceRoll && includedAngleDeg(a.endDir, b.startDir) < cornerAngleToleranceDeg)
            acute = true;

        // "comping" in VB is CC_IN/CC_OUT; we approximate as "comp is active".
        // That’s the branch gate used for InsertArcExtension. :contentReference[oaicite:17]{index=17}
        bool comping =
            (a.compMode == CM_IN || a.compMode == CM_OUT ||
             b.compMode == CM_IN || b.compMode == CM_OUT);

        // Dispatch combo :contentReference[oaicite:18]{index=18}
        if (a.type == MOT_LINE && b.type == MOT_LINE)
        {
            handleLineLine(a, b, acute, forceRoll, comping, inserts, insertCount);
        }
        else if (a.type == MOT_ARC && b.type == MOT_ARC)
        {
            handleArcArc(a, b, acute, forceRoll, inserts, insertCount);
        }
        else if ((a.type == MOT_ARC && b.type == MOT_LINE) || (a.type == MOT_LINE && b.type == MOT_ARC))
        {
            handleArcLine(a, b, acute, forceRoll, comping, inserts, insertCount);
        }
    }

    void handleLineLine(Move2D &a, Move2D &b, bool acute, bool forceRoll, bool comping,
                        Move2D inserts[2], int &insertCount)
    {
        Vec2 ip;
        bool tip = false;
        IntersectType it = intersectLineLine(a, b, ip, tip);
        if (it == IT_NONE)
            return;

        if (acute || forceRoll)
        {
            if (cornerRolling && convex(a, b))
            {
                inserts[insertCount++] = makeRollArc(a, b);
            }
            else
            {
                if (tip)
                {
                    trimToTIP(a, b, ip);
                }
                else if (comping)
                {
                    (void)extendToFIP(a, b, ip);
                }
            }
        }
        else
        {
            if (tip)
                trimToTIP(a, b, ip);
            else
                (void)extendToFIP(a, b, ip);
        }
    }

    void handleArcArc(Move2D &a, Move2D &b, bool acute, bool forceRoll,
                      Move2D inserts[2], int &insertCount)
    {

        if (len(a.p1 - b.p0) < TOL || len(a.center - b.center) < TOL)
        {
            return;
        }

        // CalcIntersect type usage mirrors VB :contentReference[oaicite:19]{index=19}
        Vec2 p1{}, p2{};
        int count = 0;
        IntersectType it = intersectCircleCircle(a, b, p1, p2, count);
        if (it == IT_NONE)
        {
            // MUST roll to close gap :contentReference[oaicite:20]{index=20}
            if (cornerRolling)
                inserts[insertCount++] = makeRollArc(a, b);
            return;
        }

        // Determine TIP candidates
        bool tip1 = (count >= 1) && pointOnArc(a, p1) && pointOnArc(b, p1);
        bool tip2 = (count == 2) && pointOnArc(a, p2) && pointOnArc(b, p2);

        if (it == IT_TANGENT)
        {
            if (tip1)
                trimToTIP(a, b, p1);
            return;
        }

        if (tip1 || tip2)
        {
            Vec2 tip = tip1 ? p1 : p2;
            if (tip1 && tip2)
                tip = pickClosest(a.p1, p1, p2);
            trimToTIP(a, b, tip);
            return;
        }

        // No true intersection
        if (acute || forceRoll)
        {
            if (cornerRolling)
                inserts[insertCount++] = makeRollArc(a, b);
        }
        else
        {
            // FIP: choose point closest to prev end, then extend test
            Vec2 fip = (count == 2) ? pickClosest(a.p1, p1, p2) : p1;
            (void)extendToFIP(a, b, fip);
        }
    }

    void handleArcLine(Move2D &a, Move2D &b, bool acute, bool forceRoll, bool comping,
                       Move2D inserts[2], int &insertCount)
    {
        // Determine which is arc/line
        Move2D *arc = nullptr;
        Move2D *lin = nullptr;
        bool arcFirst = (a.type == MOT_ARC);

        if (arcFirst)
        {
            arc = &a;
            lin = &b;
        }
        else
        {
            arc = &b;
            lin = &a;
        }

        // Intersect infinite line with circle
        Vec2 p1{}, p2{};
        int count = 0;
        IntersectType it = intersectLineCircle(lin->p0, lin->p1, arc->center, arc->radius, p1, p2, count);

        if (it == IT_NONE)
        {
            if (cornerRolling && convex(a, b))
            {
                inserts[insertCount++] = makeRollArc(a, b);
            }
            return;
        }

        // Evaluate TIP: point must lie on finite line segment and on arc sweep
        bool tip1 = false, tip2 = false;
        if (count >= 1)
            tip1 = pointOnSegment(lin->p0, lin->p1, p1) && pointOnArc(*arc, p1);
        if (count == 2)
            tip2 = pointOnSegment(lin->p0, lin->p1, p2) && pointOnArc(*arc, p2);

        if (it == IT_TANGENT)
        {
            // VB tangent logic: if directions match -> extend one side else roll :contentReference[oaicite:21]{index=21}
            if (isNearDir(a.endDir, b.startDir))
            {
                if (arcFirst)
                {
                    // a is arc: extend start of b to a end
                    b.p0 = a.p1;
                    update_dirs(b);
                }
                else
                {
                    // a is line: extend end of a to b start
                    a.p1 = b.p0;
                    update_dirs(a);
                }
            }
            else
            {
                if (cornerRolling)
                    inserts[insertCount++] = makeRollArc(a, b);
            }
            return;
        }

        // If any TIP exists: trim
        if (tip1 || tip2)
        {
            Vec2 tip = tip1 ? p1 : p2;
            if (tip1 && tip2)
                tip = pickClosest(a.p1, p1, p2);
            trimToTIP(a, b, tip);
            return;
        }

        // No TIP: either InsertArcExtension (comping) or extend/roll
        if (comping)
        {
            Move2D ext;
            if (makeArcExtension(a, b, ext))
            {
                inserts[insertCount++] = ext;
            }
            return;
        }

        if (acute || forceRoll)
        {
            if (cornerRolling && convex(a, b))
            {
                inserts[insertCount++] = makeRollArc(a, b);
            }
        }
        else
        {
            // FIP exists (line-circle intersection points). Pick closest and attempt extend
            Vec2 fip = (count == 2) ? pickClosest(a.p1, p1, p2) : p1;
            (void)extendToFIP(a, b, fip);
        }
    }
};
