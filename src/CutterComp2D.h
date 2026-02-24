// CutterComp2D.h - 2D cutter compensation with logic

#pragma once
#include "Common2D.h"

class CutterComp2D
{
public:
    // Tune these to taste
    // float acuteCornerAngleThresholdDeg = 30.0f; // AcuteCornerAngleThreshold
    CornerType cornerTreatment = CORNER_ROLL; // cornerTreatment flag
    bool performTrim = true;                  // performTrim flag
    // bool tryArcExtension = true;                // tryArcExtension flag
    // bool enableChamferTransitions = true;       // enableChamferTransitions flag
    MachineType machineType = MAC_MILL; // machine type
    // Buffers
    static constexpr int IN_CAP = 16;
    static constexpr int OUT_CAP = 32;

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

    void setMachineType(MachineType mt) { machineType = mt; }
    void setCornerTreatment(CornerType ct) { cornerTreatment = ct; }
    void setPerformTrim(bool en) { performTrim = en; }

    struct CrossingHit
    {
        bool hit = false;
        int j = -1;
        Vec2 tip{0, 0};
        float dist = 0;
    };

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
    bool pushIn(const Move2D &m)
    {
        if (inCount >= IN_CAP)
            return false;
        input_buffer[(inHead + inCount) % IN_CAP] = m;
        inCount++;
        return true;
    }

    // Main pump
    void process(void)
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
            // Need output space for worst case: prev + up to 3 inserted moves
            if (!outHasSpace(4))
                return;

            Move2D raw = popIn();
            if (raw.type == MOT_EMPTY)
                continue;

            update_vectors(raw); // before offsetting.

            Move2D curOff;
            // capture the original raw vectors before any comp modifications
            bool offsetOk = offsetMove(raw, curOff);
            curOff.initialStartDir = raw.startDir;
            curOff.initialEndDir = raw.endDir;

            if (!offsetOk)
            {
                // offset failed (e.g. arc radius too small after offset), pass through unmodified (but with updated vectors for downstream logic consistency).
                // continue;
            }

            update_vectors(curOff); // after offsetting.

            if (!havePrev)
            {
                prevOff = curOff;
                havePrev = true;
                continue;
            }

            Move2D inserts[3];
            int insertCount = 0;

            bool canRoll = true; // no rolling when compong.
            if (prevOff.compMode == CM_IN)
            {
                // modify the previous move so that the end is the start of the current move,
                prevOff.p_1 = curOff.p_0;
                canRoll = false;
            }

            if (curOff.compMode == CM_OUT)
            {
                // modify the G40 start is the end of the previous move,
                curOff.p_0 = prevOff.p_1;
                canRoll = false;
            }

            // Apply decision tree between prevOff and curOff
            applyLogic(prevOff, curOff, inserts, insertCount);

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
        process();
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
        m.p_0 = a.p_1;
        m.p_1 = b.p_0;
        m.o_1 = m.p_1;
        m.o_0 = m.p_0;
        update_vectors(m);
        return m;
    }

    bool convex_from_winding(int cw) const
    {
        if (cw == 0)
            return false;

        bool isLeft = (comp_state == COMP_LEFT);
        if (toolSign < 0)
            isLeft = !isLeft;

        if (isLeft)
            return !(cw > 0);
        return (cw > 0);
    }

    bool is_convex(const Move2D &a, const Move2D &b) const
    {
        int cw = get_winding_dir(a.endDir, b.startDir);
        return convex_from_winding(cw);
    }

    bool was_convex(const Move2D &a, const Move2D &b) const
    {
        int cw = get_winding_dir(a.initialEndDir, b.initialStartDir);
        return convex_from_winding(cw);
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
        // complete copy for non-comp moves or if tool radius is zero (also captures original vectors)
        dst = src;
        // capture the original move's start/end points before any comp modifications
        dst.src_0 = src.p_0;
        dst.src_1 = src.p_1;
        dst.src_c = src.center;
        dst.initialStartDir = src.startDir;
        dst.initialEndDir = src.endDir;

        Vec2 v = src.p_1 - src.p_0;
        float l = len(v);
        if (l < TOL)
        {
            dst.valid = false;
            return false;
        }
        Vec2 u = v * (1.0f / l);

        // if comping in or out then offset should be zero.
        if (src.compMode == CM_IN || src.compMode == CM_OUT)
        {
            dst.type = MOT_LINE;
            dst.o_0 = dst.p_0;
            dst.o_1 = dst.p_1;
            return false;
        }

        // Apply toolSign to flip the offset direction if negative tool radius
        bool useLeft = (comp_state == COMP_LEFT);
        if (toolSign < 0)
            useLeft = !useLeft;

        Vec2 n = useLeft ? leftNormal(u) : rightNormal(u);
        Vec2 off = n * toolR;

        dst.type = src.type; // keep rapid vs feed
        dst.p_0 = src.p_0 + off;
        dst.p_1 = src.p_1 + off;
        dst.o_0 = dst.p_0;
        dst.o_1 = dst.p_1;

        return true;
    }

    // Concentric arc offset like before (good enough for your VB logic)
    bool offsetArc(const Move2D &src, Move2D &dst)
    {
        float r0 = src.radius;
        if (r0 < TOL)
            r0 = len(src.p_0 - src.center);
        if (r0 < TOL)
            return false;

        // complete copy.
        dst = src;
        // backups of original geometry.
        dst.src_0 = src.p_0;
        dst.src_1 = src.p_1;
        dst.src_c = src.center;
        dst.initialStartDir = src.startDir;
        dst.initialEndDir = src.endDir;

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

        Vec2 v0 = src.p_0 - src.center;
        Vec2 v1 = src.p_1 - src.center;
        float lv0 = len(v0), lv1 = len(v1);
        if (lv0 < TOL || lv1 < TOL)
        {
            dst.valid = false;
            return false;
        }

        dst.type = MOT_ARC;
        dst.center = src.center;
        dst.radius = r1;
        dst.p_0 = src.center + v0 * (r1 / lv0);
        dst.p_1 = src.center + v1 * (r1 / lv1);
        dst.o_0 = dst.p_0;
        dst.o_1 = dst.p_1;
        return true;
    }

    static inline bool check_validity(Move2D &m)
    {
        if (m.type == MOT_LINE)
        {
            m.valid = (len(m.p_1 - m.p_0) >= TOL);
        }
        if (m.type == MOT_ARC)
        {
            float d = distFromStart_along(m, m.p_1);
            bool radius_ok = is_radius_consistent(m);
            m.valid = d >= TOL && radius_ok;
        }
        // add a debugger break if m.valid is false
        if (!m.valid)
        {
            DBG_PRINTLN("Invalid move detected!");
            // You can set a breakpoint on the line below to catch invalid moves during debugging.
            // This can help identify issues with the offset logic or edge cases.
            // For example, if you see this triggered, check if the move is a very short line or a degenerate arc.
            // You may want to log the move details here for further analysis.
        }
        return m.valid;
    }

    bool trimToTIP(Move2D &a, Move2D &b, Vec2 tip)
    {
        if (!performTrim)
            return false;
        a.p_1 = tip;
        b.p_0 = tip;

        update_vectors(a);
        update_vectors(b);

        check_validity(a);
        check_validity(b);
        return a.valid && b.valid;
    }

    bool extendToFIP(Move2D &a, Move2D &b, Vec2 fip)
    {
        if (!performTrim)
            return false;
        float fipDir1 = dot(fip - a.p_1, a.endDir);
        float fipDir2 = dot(fip - b.p_0, b.startDir);
        if (fipDir1 > 0 && fipDir2 < 0)
        {
            a.p_1 = fip;
            b.p_0 = fip;

            update_vectors(a);
            update_vectors(b);

            check_validity(a);
            check_validity(b);
        }
        return a.valid && b.valid;
    }

    Move2D makeRollArc(const Move2D &a, const Move2D &b) const
    {
        Move2D roll;
        roll.seqNum = (a.seqNum * 10) + 5; // for debugging
        roll.type = MOT_ARC;
        roll.compMode = CM_STEADY;
        roll.feed = (a.feed > 0) ? a.feed : b.feed;
        roll.p_0 = a.p_1;
        roll.p_1 = b.p_0;
        roll.center = a.src_1; // original non-offset end point to compute the center correctly

        // Find the best arc center for the roll.
        // We want to preserve the original radius as much as possible to avoid weird geometry changes
        // that could cause logic issues later.
        Vec2 v0 = roll.p_0 - roll.center;
        Vec2 v1 = roll.p_1 - roll.center;
        float r0 = len(v0);
        float r1 = len(v1);

        float r = 0.0f;
        if (r0 >= TOL && r1 >= TOL)
            r = 0.5f * (r0 + r1);
        else if (r0 >= TOL)
            r = r0;
        else if (r1 >= TOL)
            r = r1;
        else
            r = toolR;

        if (r0 >= TOL)
            roll.p_0 = roll.center + v0 * (r / r0);
        if (r1 >= TOL)
            roll.p_1 = roll.center + v1 * (r / r1);

        roll.radius = r;

        // Apply toolSign flip to arc direction
        bool useLeft = (comp_state == COMP_LEFT);
        if (toolSign < 0)
            useLeft = !useLeft;

        roll.arcDir = useLeft ? ARC_CW : ARC_CCW;
        roll.o_0 = roll.p_0;
        roll.o_1 = roll.p_1;
        roll.valid = true;
        update_vectors(roll);
        return roll;
    }

 
    uint16_t makeCornerTreatment(Move2D &a, Move2D &b, Move2D out[3])
    {
        uint16_t outCountLocal = 0;
        Move2D l1, l2;
        Move2D extA, extB;
        bool haveExtA = false;
        bool haveExtB = false;

        if (a.type == MOT_LINE)
        {
            l1 = a;
        }
        else
        {
            extA = makeArcExtensionLineOnly(a, true);
            if (!extA.valid)
                return 0;
            l1 = extA;
            haveExtA = true;
        }

        if (b.type == MOT_LINE)
        {
            l2 = b;
        }
        else
        {
            extB = makeArcExtensionLineOnly(b, false);
            if (!extB.valid)
                return 0;
            l2 = extB;
            haveExtB = true;
        }

        Vec2 partCorner = a.src_1;
        // create a bisector

        Vec2 vIn = normalize(l1.endDir * -1.0f);
        Vec2 vOut = normalize(l2.startDir);
        Vec2 bisector = normalize(vIn + vOut);
        if (len(bisector) < TOL)
            return 0;

        Vec2 chamferDir = normalize(leftNormal(bisector));
        if (len(chamferDir) < TOL)
            return 0;

        Vec2 offsetCap = partCorner + bisector * (-toolR);
        Move2D cap;
        cap.type = MOT_LINE;
        cap.compMode = CM_STEADY;
        cap.feed = (a.feed > 0) ? a.feed : b.feed;
        const float halfLen = 0.5f * (toolR + 2.0f);
        cap.p_0 = offsetCap - chamferDir * halfLen;
        cap.p_1 = offsetCap + chamferDir * halfLen;
        cap.o_0 = cap.p_0;
        cap.o_1 = cap.p_1;
        update_vectors(cap);

        // now create the intersections and trim the lines to the cap
        Vec2 ipForL1{0, 0};
        Vec2 ipForL2{0, 0};
        bool tip = false;
        IntersectType it = intersectLineLine(l1, cap, ipForL1, tip);
        if (it == IT_NONE)
            return 0;

        it = intersectLineLine(l2, cap, ipForL2, tip);
        if (it == IT_NONE)
            return 0;

        // define chamfer end points
        cap.p_0 = ipForL1;
        cap.p_1 = ipForL2;
        cap.o_0 = cap.p_0;
        cap.o_1 = cap.p_1;
        check_validity(cap);

        // if the cap length is smaller than the tolerance, we can't reliably intersect and trim to it, so just skip the chamfer and let it be a sharp corner.
        float caplen = len(cap.p_1 - cap.p_0);
        if (caplen < TOL)
            return 0;

        if (!cap.valid)
            return 0;

        if (a.type == MOT_LINE)
        {
            a.p_1 = ipForL1;
            a.o_1 = a.p_1;
            update_vectors(a);
            if (!check_validity(a))
                return 0;
        }

        if (b.type == MOT_LINE)
        {
            b.p_0 = ipForL2;
            b.o_0 = b.p_0;
            update_vectors(b);
            if (!check_validity(b))
                return 0;
        }

        if (haveExtA)
        {
            extA.p_1 = ipForL1;
            extA.o_1 = extA.p_1;
            update_vectors(extA);
            if (!check_validity(extA))
                return 0;
            out[outCountLocal++] = extA;
        }

        out[outCountLocal++] = cap;

        if (haveExtB)
        {
            extB.p_0 = ipForL2;
            extB.p_1 = b.p_0;
            extB.o_0 = extB.p_0;
            extB.o_1 = extB.p_1;
            update_vectors(extB);
            if (!check_validity(extB))
                return 0;
            out[outCountLocal++] = extB;
        }

        return outCountLocal;
    }

    // Creates only the extension line segment for arc<->line/arc without modifying inputs.
    Move2D makeArcExtensionLineOnly(const Move2D &arc, bool fromEnd) const
    {
        Move2D extLnOut;

        if (arc.type != MOT_ARC)
            return extLnOut;

        const float extent = toolR * 2.0f; // small extra length to ensure intersection with guide line
        Vec2 anchor = fromEnd ? arc.p_1 : arc.p_0;
        Vec2 dir = fromEnd ? arc.endDir : arc.startDir;

        if (len(dir) < TOL)
            return extLnOut;

        // dir = normalize(dir);

        extLnOut.type = MOT_LINE;
        extLnOut.compMode = arc.compMode;
        extLnOut.feed = arc.feed;
        extLnOut.p_0 = anchor;
        extLnOut.p_1 = anchor + dir * extent;
        extLnOut.o_0 = extLnOut.p_0;
        extLnOut.o_1 = extLnOut.p_1;
        extLnOut.src_0 = extLnOut.p_0;
        extLnOut.src_1 = extLnOut.p_1;
        extLnOut.startDir = dir;
        extLnOut.endDir = dir;
        extLnOut.initialStartDir = dir;
        extLnOut.initialEndDir = dir;
        update_vectors(extLnOut);
        check_validity(extLnOut);

        return extLnOut;
    }

 
    bool insertRollOrCorner(Move2D &a, Move2D &b, Move2D inserts[3], int &insertCount)
    {
        int startCount = insertCount;

        if (cornerTreatment == CORNER_ROLL)
        {
            Move2D roll = makeRollArc(a, b);
            float sw = arcSweep(roll);
            if (sw > PI * 1.5)
            {
                return false;
            }
            if (insertCount >= 3)
                return false;

            inserts[insertCount++] = roll;
            return true;
        }

        Move2D cornerSegs[3];
        int cornerCount = makeCornerTreatment(a, b, cornerSegs);

        for (int i = 0; i < cornerCount && insertCount < 3; ++i)
            inserts[insertCount++] = cornerSegs[i];

        return insertCount > startCount;
    }

    void applyLogic(Move2D &a, Move2D &b, Move2D inserts[3], int &insertCount)
    {
        insertCount = 0;
        update_vectors(a); // TODO: consider if we can avoid some of these updates by being smarter about when we modify the moves (e.g. only update after deciding on roll vs bevel)
        update_vectors(b);

        // Determine if corner is acute (used in multiple branches, and forces roll if true)
        // bool acute = false;
        // acute = includedAngleDeg(a.endDir, b.startDir) < acuteCornerAngleThresholdDeg;

        bool comping = (a.compMode == CM_IN || a.compMode == CM_OUT || b.compMode == CM_IN || b.compMode == CM_OUT);

        if (a.type == MOT_LINE && b.type == MOT_LINE)
        {
            handleLineLine(a, b, comping, inserts, insertCount);
        }
        else if (a.type == MOT_ARC && b.type == MOT_ARC)
        {
            handleArcArc(a, b, inserts, insertCount);
        }
        else if ((a.type == MOT_ARC && b.type == MOT_LINE) || (a.type == MOT_LINE && b.type == MOT_ARC))
        {
            handleArcLine(a, b, inserts, insertCount);
        }
    }

    void handleLineLine(Move2D &a, Move2D &b, bool comping, Move2D inserts[3], int &insertCount)
    {
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

        // direction gate (same condition used inside ExtendToCommonFIP)
        float fipDir1 = dot(ip - a.p_1, a.endDir);
        float fipDir2 = dot(ip - b.p_0, b.startDir);
        bool dirOK = (fipDir1 > 0 && fipDir2 < 0);

        // 2) Roll path: (acute OR forceRoll)
        // With transitions comping=true only for CM_IN/CM_OUT. Otherwise do NOT extend.
        if (comping && dirOK)
        {
            if (extendToFIP(a, b, ip))
            {
                return;
            }
        }

        if (a.compMode == CM_IN)
        {
            a.p_1 = b.p_0;
            return;
        }

        if (b.compMode == CM_OUT)
        {
            b.p_0 = a.p_1;
            return;
        }

        if (is_convex(a, b)) // TODO: do i need a convex test? line to line non-convex would cross and should be handled above.
        {
           insertRollOrCorner(a, b, inserts, insertCount);
            return;
        }

        // If we get here: concave or no-good FIP -> bevel locally (prevents diagonals)
        inserts[insertCount++] = makeBevel(a, b);
    }

    void handleArcArc(Move2D &a, Move2D &b, Move2D inserts[3], int &insertCount)
    {
        if (is_near(a.p_1, b.p_0) || is_near(a.center, b.center))
        {
            return; // connected and concentric arcs are already tangent. No need to roll or trim.
        }

        Vec2 p1{}, p2{};
        int tipCt = 0;
        IntersectType it = intersectCircleCircle(a, b, p1, p2, tipCt);
        if (it == IT_NONE) // no intersection so close the gap with a chamfer or roll.
        {
              if (!insertRollOrCorner(a, b, inserts, insertCount))
                    DBG_PRINTLN("handleArcArc unresolved gap (IT_NONE)");
           return;
        }

        // Determine TIP(true intersection point) candidates
        bool tip1 = (tipCt >= 1) && pointOnArc(a, p1) && pointOnArc(b, p1);
        bool tip2 = (tipCt == 2) && pointOnArc(a, p2) && pointOnArc(b, p2);

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
                tip = pickClosest(a.p_1, p1, p2);
            trimToTIP(a, b, tip);
            return;
        }

        if (!insertRollOrCorner(a, b, inserts, insertCount))
            DBG_PRINTLN("handleArcArc unresolved gap (no TIP)");

    }

    void handleArcLine(Move2D &a, Move2D &b, Move2D inserts[3], int &insertCount)
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

        // trivial check for chained elements
        if (is_near(a.p_1, b.p_0))
        {
            return; // already connected, no need to roll or trim.
        }

        // Intersect infinite line with circle
        Vec2 p1{}, p2{};
        int count = 0;
        IntersectType it = intersectLineCircle(lin->p_0, lin->p_1, arc->center, arc->radius, p1, p2, count);

        if (it == IT_NONE)
        {
            insertRollOrCorner(a, b, inserts, insertCount);

            return;
        }

        // Evaluate TIP: point must lie on finite line segment and on arc sweep
        bool tip1 = false, tip2 = false;
        if (count >= 1)
            tip1 = pointOnSegment(lin->p_0, lin->p_1, p1) && pointOnArc(*arc, p1);
        if (count == 2)
            tip2 = pointOnSegment(lin->p_0, lin->p_1, p2) && pointOnArc(*arc, p2);

        if (it == IT_TANGENT)
        {
            // tangent logic: if directions match -> extend one side else roll
            if (isNearDir(a.endDir, b.startDir))
            {
                if (arcFirst)
                {
                    // a is arc: extend start of b to a end
                    b.p_0 = a.p_1;
                    update_vectors(b);
                }
                else
                {
                    // a is line: extend end of a to b start
                    a.p_1 = b.p_0;
                    update_vectors(a);
                }
            }
            else
            {
                // if (cornerRolling)
                inserts[insertCount++] = makeRollArc(a, b);
            }
            return;
        }

        // If any TIP exists: trim
        if (tip1 || tip2)
        {
            Vec2 tip = tip1 ? p1 : p2;
            if (tip1 && tip2)
                tip = pickClosest(a.p_1, p1, p2);
            trimToTIP(a, b, tip);
            return;
        }

        insertRollOrCorner(a, b, inserts, insertCount);
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
            IntersectType it = intersectLineCircle(L.p_0, L.p_1, C.center, C.radius, p1, p2, count);
            if (it == IT_NONE)
                return 0;

            int n = 0;
            if (count >= 1 && pointOnSegment(L.p_0, L.p_1, p1) && pointOnArc(C, p1))
                tip1 = p1, n++;
            if (count == 2 && pointOnSegment(L.p_0, L.p_1, p2) && pointOnArc(C, p2))
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
            if (is_near(A.p_1, B.p_0) || is_near(A.center, B.center))
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
    // Fast check-only pass: detects whether profile has any crossing (no trimming),
    // and reports whether all valid arc moves are radius-consistent.
    bool is_validate_profile(Move2D *moves, int numMoves, int maxLookahead, bool &allRadiusConsistent)
    {
        allRadiusConsistent = true;

        for (int i = 0; i < numMoves; ++i)
        {
            if (!moves[i].valid)
                continue;
            if (moves[i].type == MOT_ARC && !is_radius_consistent(moves[i]))
                allRadiusConsistent = false;
        }

        // Build AABBs once for broad-phase crossing checks.
        init_all_aabb(moves, numMoves);

        int firstIdx = first_cutting_move(moves, numMoves);
        if (firstIdx < 0)
            return allRadiusConsistent;

        int lastIdx = last_cutting_move(moves, numMoves);
        if (lastIdx < 0 || lastIdx <= firstIdx)
            return allRadiusConsistent;

        // Early bowtie check used by trimCrossingElements.
        Vec2 tip1, tip2;
        if (lastIdx > firstIdx + 1)
        {
            if (commonTIP_any(moves[firstIdx], moves[lastIdx], tip1, tip2) > 0)
                return false;
        }

        int src = 1;
        while (src < numMoves)
        {
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
            if (crossing.hit)
                return false;

            src++;
        }

        return allRadiusConsistent;
    }

    bool trimCrossingElements(Move2D *moves, int numMoves, int maxLookahead)
    {
        // calculate AABBs for all elements once upfront to speed up intersection testing in the lookahead loop.
        init_all_aabb(moves, numMoves);

        int src = 1; // skip the first element since it has no previous neighbor to cross with

        // find the first and last cutting moves to check for the bowtie edge case that cannot be resolved by trimming.
        int firstIdx = first_cutting_move(moves, numMoves);
        if (firstIdx < 0)
            return false;

        int lastIdx = last_cutting_move(moves, numMoves);
        if (lastIdx < 0 || lastIdx <= firstIdx)
            return false;

        // if the first cutting move and the last cutting move cross
        // we have a bowtie shape that cannot be resolved by trimming,
        Vec2 tip1, tip2;
        // test for crossing between first and last cutting moves
        // but only if they are not adjacent (to avoid trivial shared endpoint case)
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
                    numMoves = lastIdx; // end-exclusive scan bound is now the last cutting move, so we won't look for crossings beyond it (including the known bowtie crossing at the end).
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

            (void)trimToTIP(moves[src], moves[j], crossing.tip);
            // test the above now to see if either one is invalid after trimming,
            // and if so invalidate the other as well since the crossing is resolved and we don't want to leave any tiny slivers that could cause more crossings or other issues downstream.

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
                a.p_1 = b.p_1;
                a.o_1 = a.p_1;
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
