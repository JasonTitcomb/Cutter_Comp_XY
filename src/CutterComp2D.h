// CutterComp2D.h - 2D cutter compensation with logic

#pragma once
#include "Common2D.h"

class CutterComp2D
{
public:
    // Tune these to taste
    float cornerAngleToleranceDeg = 30.0f; // CornerAngleTolerance
    bool cornerRolling = true;             // rollAround flag
    bool performTrim = true;               // performTrim flag
    MachineType machineType = MAC_MILL;    // machine type
    // Buffers
    static constexpr int IN_CAP = 16;
    static constexpr int OUT_CAP = 32;

    void setToolRadius(float r)
    {
        toolR = (r < 0) ? -r : r;
        toolSign = (r < 0) ? -1 : 1;
    }
    void setComp(CompSide s)
    {
        comp_state = s;
        resetState();
    }
    void setMachineType(MachineType mt) { machineType = mt; }
    void setCornerRolling(bool en) { cornerRolling = en; }
    void setCornerAngleTolerance(float deg) { cornerAngleToleranceDeg = deg; }
    void setPerformTrim(bool en) { performTrim = en; }

    bool pushIn(const Move2D &m)
    {
        if (inCount >= IN_CAP)
            return false;
        input_buffer[(inHead + inCount) % IN_CAP] = m;
        inCount++;
        return true;
    }

    // Main pump
    void process(bool forceRoll = false)
    {
        // DBG_PRINTLN(inCount);
        //  If comp is OFF, just pass through immediately (no delay needed)
        if (comp_state == COMP_OFF || toolR < TOL)
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

        while (inCount > 0)
        {
            // Need output space for worst case: prev + (extension line OR roll arc) (2 moves)
            if (!outHasSpace(3))
                return;

            Move2D raw = popIn();
            if (raw.type == MOT_EMPTY)
                continue;

            Move2D curOff;
            // if in or out then no offsetting, just pass through with comp mode set for downstream logic and rolling decisions.
            offsetMove(raw, curOff);
            // REMOVED: pushOut(prevOff);  // DON'T PUSH HERE!

            // Stash "InitialEndPt"
            curOff.initialEndPt = raw.p1;
            curOff.initialStartPt = raw.p0;

            update_vectors(curOff);

            if (!havePrev)
            {
                prevOff = curOff;
                havePrev = true;
                continue;
            }

            Move2D inserts[2];
            int insertCount = 0;

            bool canRoll = forceRoll; // no rolling when compong.
            if (prevOff.compMode == CM_IN)
            {
                // modify the previous move so that the end is the start of the current move,
                prevOff.p1 = curOff.p0;
                canRoll = false;
            }

            if (curOff.compMode == CM_OUT)
            {
                // modify the G40 start is the end of the previous move,
                curOff.p0 = prevOff.p1;
                canRoll = false;
            }

            // Apply decision tree between prevOff and curOff
            applyLogic(prevOff, curOff, canRoll, inserts, insertCount);

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
        process(cornerRolling);
        if (havePrev && outHasSpace(1))
        {
            pushOut(prevOff);
            havePrev = false;
        }
    }

    bool popOut(Move2D &m)
    {
        if (outCount == 0)
            return false;
        m = output_buffer[outHead];
        outHead = (outHead + 1) % OUT_CAP;
        outCount--;
        return true;
    }

private:
    // Ring buffers
    Move2D input_buffer[IN_CAP];
    int inHead = 0, inCount = 0;

    Move2D output_buffer[OUT_CAP];
    int outHead = 0, outCount = 0;

    // Settings
    float toolR = 0.0f;
    int8_t toolSign = 0;
    CompSide comp_state = COMP_OFF;

    // Delayed output state
    bool havePrev = false;
    Move2D prevOff;

private:
    // ---------- small helpers ----------
    bool outHasSpace(int n) const { return (outCount + n) <= OUT_CAP; }

    Move2D popIn()
    {
        Move2D m = input_buffer[inHead];
        inHead = (inHead + 1) % IN_CAP;
        inCount--;
        return m;
    }

    void pushOut(const Move2D &m)
    {
        output_buffer[(outHead + outCount) % OUT_CAP] = m;
        outCount++;
    }

    void resetState() { havePrev = false; }

    Move2D makeBevel(const Move2D &a, const Move2D &b) const
    {
        // DBG_PRINTLN("Bevel needed");
        Move2D m;
        m.type = MOT_LINE;
        m.feed = (a.feed > 0) ? a.feed : b.feed;
        m.p0 = a.p1;
        m.p1 = b.p0;
        m.initialEndPt = m.p1;
        m.initialStartPt = m.p0;
        update_vectors(m);
        return m;
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

    static inline float includedAngleDeg(Vec2 v1, Vec2 v2)
    {
        // VB does: v2 = -v2
        v1 = normalize(v1);
        v2 = normalize(v2) * -1.0f;

        float c = dot(v1, v2);
        c = c2d_clamp(c, -1.0f, 1.0f);
        return rad2deg(acosf(c));
    }

    static inline bool isNearDir(Vec2 a, Vec2 b)
    {
        a = normalize(a);
        b = normalize(b);
        return dot(a, b) > 0.9995f;
    }

    bool isTinyLine(const Move2D &m) const
    {
        if (m.type != MOT_LINE)
            return false;
        float L = len(m.p1 - m.p0);
        return L < (0.50f * toolR); // start with 0.5R threshold
    }

    bool convex(const Move2D &a, const Move2D &b) const
    {
        int cw = WindingDirection(a.endDir, b.startDir);
        if (cw == 0)
            return false;

        // Apply toolSign flip
        bool isLeft = (comp_state == COMP_LEFT);
        if (toolSign < 0)
            isLeft = !isLeft;

        if (isLeft)
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
        if (src.type == MOT_LINE || src.type == MOT_RAPID)
            return offsetLine(src, dst);
        if (src.type == MOT_ARC)
            return offsetArc(src, dst);

        return false;
    }

    bool offsetLine(const Move2D &src, Move2D &dst)
    {
        Vec2 v = src.p1 - src.p0;
        float l = len(v);
        if (l < TOL)
            return false;
        Vec2 u = v * (1.0f / l);

        // if comping in or out then offset should be zero.
        if (src.compMode == CM_IN || src.compMode == CM_OUT)
        {
            dst = src;
            dst.type = MOT_LINE;
            dst.initialStartPt = dst.p0;
            dst.initialEndPt = dst.p1;
            return false;
        }

        // Apply toolSign to flip the offset direction if negative tool radius
        bool useLeft = (comp_state == COMP_LEFT);
        if (toolSign < 0)
            useLeft = !useLeft;

        Vec2 n = useLeft ? leftNormal(u) : rightNormal(u);
        Vec2 off = n * toolR;

        dst = src;
        dst.type = src.type; // keep rapid vs feed
        dst.p0 = src.p0 + off;
        dst.p1 = src.p1 + off;
        dst.initialStartPt = dst.p0;
        dst.initialEndPt = dst.p1;
        return true;
    }

    // Concentric arc offset like before (good enough for your VB logic)
    bool offsetArc(const Move2D &src, Move2D &dst)
    {
        float r0 = src.radius;
        if (r0 < TOL)
            r0 = len(src.p0 - src.center);
        if (r0 < TOL)
            return false;

        float dr = toolR;
        bool ccw = (src.arcDir == ARC_CCW);
        // Apply toolSign to flip the offset side if negative tool radius
        bool left = (comp_state == COMP_LEFT);
        if (toolSign < 0)
            left = !left;

        float r1;
        if (ccw)
            r1 = r0 + (left ? -dr : +dr);
        else
            r1 = r0 + (left ? +dr : -dr);

        // Keep the rad regardless. If it's negative, the offset will flip to the other side of the center,
        // which is a valid geometry (though maybe not what you want for a real cutter comp).
        // The logic later should be able to handle it as long as we keep the direction semantics consistent.
        // if (r1 < TOL)
        //     return false; NO, allow negative radius for now and let logic handle it.

        Vec2 v0 = src.p0 - src.center;
        Vec2 v1 = src.p1 - src.center;
        float lv0 = len(v0), lv1 = len(v1);
        if (lv0 < TOL || lv1 < TOL)
            return false;

        dst = src;
        dst.type = MOT_ARC;
        dst.center = src.center;
        dst.radius = r1;
        dst.p0 = src.center + v0 * (r1 / lv0);
        dst.p1 = src.center + v1 * (r1 / lv1);
        dst.initialStartPt = dst.p0;
        dst.initialEndPt = dst.p1;
        return true;
    }

    // This function expects you added startDir/endDir fields in Move2D.
    static inline void update_vectors(Move2D &m)
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
        if (lab2 < TOL)
            return (len(p - a) < TOL);

        float t = dot(p - a, ab) / lab2;
        if (t < -TOL || t > 1.0f + TOL)
            return false;
        // distance to line
        float d = fabsf(cross(p - a, ab)) / sqrtf(lab2);
        return d < TOL;
    }

    static inline bool pointOnArc(const Move2D &a, Vec2 p)
    {
        // radius match
        float rp = len(p - a.center);
        if (fabsf(rp - a.radius) > TOL)
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
        if (fabsf(den) < TOL)
        {
            tip = false;
            return IT_NONE;
        }

        float t = cross(q - p, s) / den;
        float u = cross(q - p, r) / den;
        ip = p + r * t;

        tip = (t >= -TOL && t <= 1.0f + TOL && u >= -TOL && u <= 1.0f + TOL);
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

    static IntersectType intersectLineCircle(Vec2 a, Vec2 b, Vec2 c, float r,
                                             Vec2 &p1, Vec2 &p2, int &count)
    {
        Vec2 d = b - a;
        float dd = dot(d, d);
        count = 0;
        if (dd < 1e-20f)
            return IT_NONE; // degenerate

        // Closest point from center to the infinite line
        Vec2 f = a - c;
        float t0 = -dot(f, d) / dd;
        Vec2 q = a + d * t0;

        // distance^2 from center to line
        Vec2 qc = q - c;
        float dist2 = dot(qc, qc);

        float r2 = r * r;

        // tolerance in inches:
        const float eps = 1e-5f; // 0.00001"
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

    // Choose point closer to prev end (matches VB using mTip1 first in practice)
    static Vec2 pickClosest(Vec2 ref, Vec2 a, Vec2 b)
    {
        return (len(a - ref) <= len(b - ref)) ? a : b;
    }

    void trimToTIP(Move2D &a, Move2D &b, Vec2 tip)
    {
        if (!performTrim)
            return;
        a.p1 = tip;
        b.p0 = tip;
        update_vectors(a);
        update_vectors(b);
    }

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
            update_vectors(a);
            update_vectors(b);
            return true;
        }
        return false;
    }

    Move2D makeRollArc(const Move2D &a, const Move2D &b) const
    {
        Move2D roll;
        roll.type = MOT_ARC;
        roll.compMode = CM_STEADY;
        roll.feed = (a.feed > 0) ? a.feed : b.feed;
        roll.p0 = a.p1;
        roll.p1 = b.p0;
        roll.center = a.initialEndPt;
        roll.radius = len(roll.p0 - roll.center);

        // Apply toolSign flip to arc direction
        bool useLeft = (comp_state == COMP_LEFT);
        if (toolSign < 0)
            useLeft = !useLeft;

        roll.arcDir = useLeft ? ARC_CW : ARC_CCW;
        roll.initialStartPt = roll.p0; // Add this
        roll.initialEndPt = roll.p1;   // Add this
        roll.valid = true;
        update_vectors(roll);
        return roll;
    }

    // Creates a line segment bridging arc<->line when comping and no TIP.
    bool makeArcExtension(Move2D &a, Move2D &b, Move2D &extOut)
    {
        // Case: a is LINE, b is ARC
        if (a.type == MOT_LINE && b.type == MOT_ARC)
        {

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

            // - ip must be forward of a's end direction (extend end)
            // - ip must be "behind" b's start direction (extend/trim start toward it)
            float da = dot(ip - a.p1, a.endDir);
            float db = dot(ip - b.p0, b.startDir);

            if (!(da > 0 && db < 0))
                return false;

            // Extend/trim:
            a.p1 = ip;
            update_vectors(a);

            // Insert extension line from intersection to arc start
            extOut.type = MOT_LINE;
             extOut.feed = (a.feed > 0) ? a.feed : b.feed;
            extOut.p0 = ip;
            extOut.p1 = b.p0;
            update_vectors(extOut);

            return true;
        }

        // Case: a is ARC, b is LINE
        if (a.type == MOT_ARC && b.type == MOT_LINE)
        {

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
            update_vectors(b);

            // Insert extension line from arc end to intersection
            extOut.type = MOT_LINE;
            extOut.feed = (a.feed > 0) ? a.feed : b.feed;
            extOut.p0 = a.p1;
            extOut.p1 = ip;
            update_vectors(extOut);

            return true;
        }

        return false;
    }

    void applyLogic(Move2D &a, Move2D &b, bool forceRoll, Move2D inserts[2], int &insertCount)
    {
        insertCount = 0;
        update_vectors(a);
        update_vectors(b);

        // Determine if corner is acute (used in multiple branches, and forces roll if true)
        bool acute = false;
        if (!forceRoll && includedAngleDeg(a.endDir, b.startDir) < cornerAngleToleranceDeg)
            acute = true;

        bool comping = (a.compMode == CM_IN || a.compMode == CM_OUT || b.compMode == CM_IN || b.compMode == CM_OUT);

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

    void handleLineLine(Move2D &a, Move2D &b,
                        bool acute, bool forceRoll, bool comping,
                        Move2D inserts[2], int &insertCount)
    {
        // Keep directions up to date
        update_vectors(a);
        update_vectors(b);

        Vec2 ip;
        bool tip = false;
        IntersectType it = intersectLineLine(a, b, ip, tip);
        if (it == IT_NONE)
            return;

        // 1) TIP: true intersection within both finite segments -> trim
        if (tip)
        {
            trimToTIP(a, b, ip);
            return;
        }

        // Compute "far FIP" metrics (used in multiple branches)
        // float la = len(a.p1 - a.p0);
        // float lb = len(b.p1 - b.p0);

        // if (la < TOL || lb < TOL)
        // {
        //     inserts[insertCount++] = makeBevel(a, b);
        //     return;
        // }

        // float da = len(ip - a.p1);
        // float db = len(ip - b.p0);

        // Reject far intersections (prevents diagonal spikes across sawteeth)
        // const float FAR = 1.35f; // tune 1.1..2.0
        // bool far = false;//(da > FAR * la) || (db > FAR * lb);

        // direction gate (same condition used inside ExtendToCommonFIP)
        float fipDir1 = dot(ip - a.p1, a.endDir);
        float fipDir2 = dot(ip - b.p0, b.startDir);
        bool dirOK = (fipDir1 > 0 && fipDir2 < 0);

        // 2) Roll path: (acute OR forceRoll)
        if (acute || forceRoll)
        {
            // If Convex then InsertArcBetweenElements
            if (convex(a, b))
            {
                inserts[insertCount++] = makeRollArc(a, b);
                return;
            }

            // With transitions comping=true only for CM_IN/CM_OUT. Otherwise do NOT extend.
            if (comping && dirOK)
            {
                if (extendToFIP(a, b, ip))
                {
                    return;
                }
            }

            // If we get here: concave or no-good FIP -> bevel locally (prevents diagonals)
            inserts[insertCount++] = makeBevel(a, b);
            return;
        }

        // 3) Non-roll path:
        if (dirOK)
        {
            if (extendToFIP(a, b, ip))
            {
                return;
            }
        }

        // Fallback: bevel
        inserts[insertCount++] = makeBevel(a, b);
    }

    void handleArcArc(Move2D &a, Move2D &b, bool acute, bool forceRoll, Move2D inserts[2], int &insertCount)
    {
        if (nearPt2(a.p1, b.p0) || nearPt2(a.center, b.center))
        {
            return; // connected and concentric arcs are already tangent. No need to roll or trim.
        }

        Vec2 p1{}, p2{};
        int count = 0;
        IntersectType it = intersectCircleCircle(a, b, p1, p2, count);
        if (it == IT_NONE)
        {
            // MUST roll to close gap
            inserts[insertCount++] = makeRollArc(a, b);
            return;
        }

        // Determine TIP(true intersection point) candidates
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
            // if (cornerRolling)
            inserts[insertCount++] = makeRollArc(a, b);
        }
        else
        {
            // FIP(false intersection point): choose point closest to prev end, then extend test
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
            inserts[insertCount++] = makeRollArc(a, b);
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
            // tangent logic: if directions match -> extend one side else roll
            if (isNearDir(a.endDir, b.startDir))
            {
                if (arcFirst)
                {
                    // a is arc: extend start of b to a end
                    b.p0 = a.p1;
                    update_vectors(b);
                }
                else
                {
                    // a is line: extend end of a to b start
                    a.p1 = b.p0;
                    update_vectors(a);
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

    // Returns 0..2 TIPs that lie on BOTH finite elements
    static int commonTIP_any(const Move2D &A, const Move2D &B, Vec2 &tip1, Vec2 &tip2)
    {
        tip1 = {0, 0};
        tip2 = {0, 0};

        // LINE-LINE
        if (A.type == MOT_LINE && B.type == MOT_LINE)
        {
            Vec2 ip;
            bool tip = false;
            IntersectType it = intersectLineLine(A, B, ip, tip);
            if (it != IT_NONE && tip)
            {
                tip1 = ip;
                return 1;
            }
            return 0;
        }

        // LINE-ARC or ARC-LINE
        if ((A.type == MOT_LINE && B.type == MOT_ARC) || (A.type == MOT_ARC && B.type == MOT_LINE))
        {
            const Move2D &L = (A.type == MOT_LINE) ? A : B;
            const Move2D &C = (A.type == MOT_ARC) ? A : B;

            Vec2 p1{}, p2{};
            int count = 0;
            IntersectType it = intersectLineCircle(L.p0, L.p1, C.center, C.radius, p1, p2, count);
            if (it == IT_NONE)
                return 0;

            int n = 0;
            if (count >= 1 && pointOnSegment(L.p0, L.p1, p1) && pointOnArc(C, p1))
                tip1 = p1, n++;
            if (count == 2 && pointOnSegment(L.p0, L.p1, p2) && pointOnArc(C, p2))
            {
                if (n == 0)
                    tip1 = p2;
                else
                    tip2 = p2;
                n++;
            }
            return n;
        }

        // ARC-ARC
        if (A.type == MOT_ARC && B.type == MOT_ARC)
        {
            // early-out for chained or concentric arcs (important)
            if (nearPt2(A.p1, B.p0) || nearPt2(A.center, B.center))
                return 0;

            Vec2 p1{}, p2{};
            int count = 0;
            IntersectType it = intersectCircleCircle(A, B, p1, p2, count);
            if (it == IT_NONE)
                return 0;

            int n = 0;
            if (count >= 1 && pointOnArc(A, p1) && pointOnArc(B, p1))
                tip1 = p1, n++;
            if (count == 2 && pointOnArc(A, p2) && pointOnArc(B, p2))
            {
                if (n == 0)
                    tip1 = p2;
                else
                    tip2 = p2;
                n++;
            }
            return n;
        }

        return 0;
    }

    struct CrossingHit
    {
        bool hit = false;
        int j = -1;
        Vec2 tip{0, 0};
        float dist = 0;
    };

    static CrossingHit lookAheadForCrossing(Move2D *moves, int numMoves, int srcIdx, int startTargetIdx, int maxLookahead)
    {
        CrossingHit best;
        best.hit = false;
        best.dist = 1e30f;

        if (!moves[srcIdx].valid)
            return best;

        Move2D &src = moves[srcIdx];

        int j = startTargetIdx + 1; // start looking from the element after the immediate neighbor
        for (int r = 0; r <= maxLookahead && j < numMoves; ++r, ++j)
        {
            Move2D &target = moves[j];
            if (!target.valid)
                continue;

            // Skip immediate neighbor to avoid trimming the normal shared endpoint
            if (j < srcIdx + 2)
                continue;

            if (!aabb_intersects(src.bounds, target.bounds))
            {
                continue;
            }

            Vec2 t1, t2;
            int n = commonTIP_any(src, target, t1, t2);
            if (n <= 0)
                continue;

            // picks the nearest crossing along src from its start.
            Vec2 pick = t1;
            float d = distFromStart_along(src, t1);
            if (n == 2)
            {
                float d2 = distFromStart_along(src, t2);
                if (d2 < d)
                {
                    d = d2;
                    pick = t2;
                }
            }

            if (d <= best.dist)
            {
                best.hit = true;
                best.j = j;
                best.tip = pick;
                best.dist = d;
            }
        }

        return best;
    }

public:
    bool trimCrossingElements(Move2D *moves, int numMoves, int maxLookahead)
    {
        //calculate AABBs for all elements once upfront to speed up intersection testing in the lookahead loop.
        init_all_aabb(moves, numMoves);

        int src = 1; // skip the first element since it has no previous neighbor to cross with

        int firstIdx = first_cutting_move(moves, numMoves);
        if (firstIdx < 0)
            return false;

        int lastIdx = last_cutting_move(moves, numMoves);
        if (lastIdx < 0 || lastIdx <= firstIdx)
            return false;

        // if the first cutting move and the last cutting move cross
        // we have a bowtie shape that cannot be resolved by trimming,
        Vec2 tip1, tip2;
        // test for crossing between first and last cutting moves, but only if they are not adjacent (to avoid trivial shared endpoint case)
        if (lastIdx > firstIdx + 1)
        {
            if (commonTIP_any(moves[firstIdx], moves[lastIdx], tip1, tip2) > 0)
            {
                // found a crossing between first and last cutting moves.
                // This is a known edge case (bowtie shape) that can occur in complex toolpaths.

                if (lastIdx - firstIdx > 2)
                {
                    // If there are more elements beyond the crossing, we can skip trimming the first crossing and start processing from the element after the first cutting move.
                    // This allows us to still trim any other crossings that may exist in the middle of the path, while avoiding the unresolvable bowtie crossing at the end.
                    // Note: this means we will leave the bowtie crossing untrimmed, but at least we can clean up any other crossings in the middle of the path.
                    src = firstIdx + 1; // start src from the element after the first cutting move to continue processing any other crossings that may exist in the middle of the path.
                    numMoves--;         // effectively exclude the last element from processing since we know it crosses with the first and we are skipping trimming in this case.
                }
                else
                {
                    return false; // no other crossings to trim and we can just return.
                }
            }
        }

        bool trimmedAny = false;

        while (src < numMoves)
        {

            // do not trim comp in elements since they are already "offset" and thus less likely to have true crossings that need trimming.
            while (src < numMoves && (!moves[src].valid || moves[src].compMode == CM_IN))
                src++;

            if (src >= numMoves)
                break;

            int target = src + 1;
            while (target < numMoves && !moves[target].valid)
                target++;
            if (target >= numMoves)
                break;

            CrossingHit crossing = lookAheadForCrossing(moves, numMoves, src, target, maxLookahead);
            if (!crossing.hit)
            {
                src++;
                continue;
            }

            int j = crossing.j;

            // Re-calc is not needed here because our geometry isn't mutating intersection cache like VB;
            // but if you later add caching, this is where you'd "recalc".

            trimToTIP(moves[src], moves[j], crossing.tip);
            invalidateRange(moves, src, j);

            trimmedAny = true;

            // trimmedTo becomes new srcElement
            src = j;
        }

        return trimmedAny;
    }

    // Merges adjacent colinear LINE segments in-place by extending the first and invalidating the second.
    // Returns number of merges performed.
    int merge_all_colinear(Move2D *moves, int count)
    {
        if (count < 2)
            return 0;

        int merges = 0;

        int ia = first_valid_index(moves, count);
        if (ia < 0)
            return 0;

        while (true)
        {
            int ib = next_valid_index(moves, count, ia);
            if (ib < 0)
                break;

            Move2D &a = moves[ia];
            Move2D &b = moves[ib];

            // we know they are connected because all invalid motions are skipped,
            // so just check colinearity and merge if so.
            if (isColinearWith(a, b))
            {
                // Extend a to b end and invalidate b
                a.p1 = b.p1;
                a.initialEndPt = a.p1;
                update_vectors(a);

                b.valid = false;
                merges++;

                // Stay at same 'a' and try to merge with the next valid again
                continue;
            }
            // Advance
            ia = ib;
        }

        return merges;
    }
};
