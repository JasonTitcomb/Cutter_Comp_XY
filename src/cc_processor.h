/*
 * cc_processor.h - 2D cutter compensation with logic
 * Jason Titcomb 2026
 * MIT License – see LICENSE file in repository root
 */

#pragma once
#include "cc_math.h"
typedef void (*CcOutputCB)(const char *text, size_t len);
typedef void (*CcErrorCB)(const char *message, CompError err, uint32_t seqNum);
typedef void (*CcStartCompCB)(int toolRegister, int diaRegister);

struct CrossingHit
{
    bool hit = false;
    int j = -1;
    Vec2 tip{0, 0};
    float dist = 0;
};

struct AABB2
{
    float minx, miny, maxx, maxy;
};

struct CcMainOptions
{
    struct CcMainCallbacks
    {
        CcOutputCB output = nullptr;
        CcErrorCB error = nullptr;
        CcStartCompCB startComp = nullptr;
    } callbacks;

    float toolRadius = 0.0f;
    CornerType cornerTreatment = CORNER_ROLL;
    bool globalTrimCrossing = true;
    bool emitStatusComments = true;
};

class CutterComp2D
{
private:
    CcOutputCB outputCB_ = nullptr;
    CcErrorCB errorCB_ = nullptr;
    Units units = UNITS_MM;
    CcMainOptions options;
    bool hasCompError = false;
    uint32_t lastSeqNum = 0;

    void reportCompError(CompError err)
    {
        hasCompError = true;
        if (errorCB_)
        {
            switch (err)
            {
            case CE_ARC_RADIUS_MISMATCH:
                errorCB_("Arc radius inconsistency", err, lastSeqNum);
                break;
            case CE_INVALID_MOVE:
                errorCB_("Invalid move", err, lastSeqNum);
                break;
            case CE_COMP_MOVE_TOO_SHORT:
                errorCB_("Comp move too short", err, lastSeqNum);
                break;
            case CE_ARC_LT_TOOL_RAD:
                errorCB_("Arc smaller than tool radius", err, lastSeqNum);
                break;
            case CE_FLIPPED_ARC:
                errorCB_("Flipped arc", err, lastSeqNum);
                break;
            case CE_COMP_IN_CROSSING:
                errorCB_("Comp-in crossing", err, lastSeqNum);
                break;
            case CE_COMP_OUT_CROSSING:
                errorCB_("Comp-out crossing", err, lastSeqNum);
                break;
            case CE_UNRESOLVED_GAP:
                errorCB_("Unresolved gap", err, lastSeqNum);
                break;
            default:
                errorCB_("Unknown comp error", err, lastSeqNum);
            }
        }
    }

    // --- Moved from cc_math.h ---
    bool validate(Move2D &m)
    {
        float d = 0;
        bool radius_ok = true;
        float sw = 0;
        bool sweepOk = true;

        if (m.type == MOT_LINE)
        {
            m.valid = len(m.p_1 - m.p_0) >= TOL;
        }
        if (m.type == MOT_ARC)
        {
            // if we are not going to do global trim then we should test arc validity here, because we won't have another chance to validate before output.
            // if comp left and arc is CCW, then the arc must be  > tool rad.
            if (!options.globalTrimCrossing && comp_state == COMP_LEFT && m.arcDir == ARC_CCW)
            {
                if (len(m.p_1 - m.p_0) <= toolR)
                {
                    reportCompError(CE_ARC_LT_TOOL_RAD);
                    return false;
                }
            }
            if (!options.globalTrimCrossing && comp_state == COMP_RIGHT && m.arcDir == ARC_CW)
            {
                if (len(m.p_1 - m.p_0) <= toolR)
                {
                    reportCompError(CE_ARC_LT_TOOL_RAD);
                    return false;
                }
            }

            d = distFromStart_along(m, m.p_1);
            radius_ok = is_radius_consistent(m);
            sw = arcSweepDeg(m);
            sweepOk = (sw > MAX_SWEEP_DEG || sw < MIN_ARC_LEN) ? false : true;
            m.valid = d >= TOL && radius_ok && sweepOk;
            if (!radius_ok)
                reportCompError(CE_ARC_RADIUS_MISMATCH);
            if (!sweepOk)
                reportCompError(CE_INVALID_MOVE);
        }
        return m.valid;
    }

    void invalidateRange(Move2D *moves, int i, int j)
    {
        for (int k = i + 1; k < j; ++k)
        {
            moves[k].valid = false;
        }
    }

    CornerType cornerTreatment = CORNER_ROLL; // cornerTreatment flag
    // Buffers
    static constexpr int IN_CAP = 2;
    static constexpr int OUT_CAP = 4;

    // Ring buffers
    Move2D input_buffer[IN_CAP];
    int inHead = 0, inCount = 0;

    Move2D output_buffer[OUT_CAP];
    int outHead = 0, outCount = 0;

    // Settings
    float toolR = 0.0f;
    int8_t toolSign = 0;
    // Delayed output state
    bool havePrevMove2D = false;
    Move2D prevOff;

public:
    CompSide comp_state = COMP_OFF;
    void setOptions(const CcMainOptions &opts)
    {
        options = opts;
        setToolRadius(opts.toolRadius);
        cornerTreatment = opts.cornerTreatment;
        outputCB_ = opts.callbacks.output;
        errorCB_ = opts.callbacks.error;
    }

    void setUnits(Units u)
    {
        units = u;
        if (units == UNITS_INCH)
        {
            arcTol = ARC_TOL_IN;
            gapTol = GAP_TOL_IN;
        }
        else
        {
            arcTol = ARC_TOL_IN * 25.4f;
            gapTol = GAP_TOL_IN * 25.4f;
        }
    }

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
    bool process(void)
    {
        if (hasCompError)
            return false;

        while (inCount > 0)
        {
            // Need output space for worst case: prev + up to 3 inserted moves
            if (!outHasSpace(4))
                return false;

            Move2D raw = popIn();
            if (raw.seqNum != 0)
                lastSeqNum = raw.seqNum;

            if (raw.type == MOT_EMPTY)
                continue;

            update_vectors(raw); // before offsetting.

            Move2D curOff;
            offsetMove(raw, curOff);
            validate(curOff);

            // Z-only move: no XY displacement, nothing to offset
            if (!raw.hasXY && raw.hasZ)
            {
                if (havePrevMove2D)
                {
                    curOff.p_0 = prevOff.p_0;
                    curOff.p_1 = prevOff.p_0;
                }
                if (!outHasSpace(1))
                    return false;
                pushOut(curOff);
                continue;
            }

            // ── check comp-in / comp-out move length vs toolR ──
            if (raw.compMode == CM_IN || raw.compMode == CM_OUT)
            {
                float moveLen = len(raw.p_1 - raw.p_0);
                if (moveLen <= toolR)
                {
                    reportCompError(CE_COMP_MOVE_TOO_SHORT);
                    return false;
                }
            }

            if (!havePrevMove2D)
            {
                prevOff = curOff;
                havePrevMove2D = true;
                continue;
            }

            Move2D inserts[3]; // allow up to 3 inserts for corner treatment.
            int insertCount = 0;

            if (prevOff.compMode == CM_IN)
            {
                // modify the previous move so that the end is the start of the current move,
                prevOff.p_1 = curOff.p_0;
            }

            if (curOff.compMode == CM_OUT)
            {
                // modify the G40 start is the end of the previous move,
                curOff.p_0 = prevOff.p_1;
            }

            // Apply decision tree between prevOff and curOff
            applyLogic(prevOff, curOff, inserts, insertCount);

            // Emit previous + inserts; hold curOff as new prev
            if (prevOff.valid)
            {
                pushOut(prevOff);
                for (int i = 0; i < insertCount; ++i)
                    pushOut(inserts[i]);
            }
            prevOff = curOff;
        }
        return true;
    }

    // Flush delayed last element
    void flush()
    {
        process();
        if (havePrevMove2D && outHasSpace(1))
        {
            pushOut(prevOff);
            havePrevMove2D = false;
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
    // ---------- small helpers ----------
    bool outHasSpace(int n)
    {
        bool ok = (outCount + n) <= OUT_CAP;
        if (!ok)
            reportCompError(CE_OUTPUT_BUFFER_OVERFLOW);

        return ok;
    }

    Move2D popIn()
    {
        Move2D m = input_buffer[inHead];
        inHead = (inHead + 1) % IN_CAP;
        inCount--;
        return m;
    }

    void pushOut(const Move2D &m)
    {
        if (outCount >= OUT_CAP)
        {
            reportCompError(CE_OUTPUT_BUFFER_OVERFLOW);
            return;
        }
        output_buffer[(outHead + outCount) % OUT_CAP] = m;
        outCount++;
    }

    void resetState() { havePrevMove2D = false; }

    Move2D makeBevel(const Move2D &a, const Move2D &b)
    {
        Move2D m;
        m.hasXY = true;
        m.type = MOT_LINE;
        m.feed = (a.feed > 0) ? a.feed : b.feed;
        m.p_0 = a.p_1;
        m.p_1 = b.p_0;
        update_vectors(m);
        return m;
    }

    bool convex_from_winding(int cw)
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

    bool is_convex(const Move2D &a, const Move2D &b)
    {
        int cw = get_winding_dir(a.endDir, b.startDir);
        return convex_from_winding(cw);
    }

    int next_valid_index(const Move2D *moves, int count, int i)
    {
        for (int k = i + 1; k < count; ++k)
        {
            if (isMotionValid(moves[k]))
                return k;
        }
        return -1;
    }

    int prev_valid_index(const Move2D *moves, int i)
    {
        for (int k = i - 1; k >= 0; --k)
        {
            if (isMotionValid(moves[k]))
                return k;
        }
        return -1;
    }

    int first_valid_index(const Move2D *moves, int count)
    {
        for (int i = 0; i < count; ++i)
        {
            if (isMotionValid(moves[i]))
                return i;
        }
        return -1;
    }

    // Find the first move after a CM_IN move
    int first_comp_move(const Move2D *moves, int count)
    {
        for (int i = 0; i < count; ++i)
        {
            if (moves[i].compMode == CM_IN)
            {
                // Found a CM_IN move, now find the next valid move
                return next_valid_index(moves, count, i);
            }
        }
        return -1; // No CM_IN found
    }

    // Find the last move before a CM_OUT move
    int last_comp_move(const Move2D *moves, int count, int startAt)
    {
        for (int i = startAt; i < count; ++i)
        {
            if (moves[i].compMode == CM_OUT)
            {
                // Found a CM_OUT move, now find the last valid move before it
                return prev_valid_index(moves, i);
            }
        }
        return -1; // No CM_OUT found
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

        Vec2 v = src.p_1 - src.p_0;
        float l = len(v);
        if (l < TOL)
        {
            dst.valid = false;
            return false;
        }
        Vec2 u = v * (1.0f / l);

        // Apply toolSign to flip the offset direction if negative tool radius
        bool useLeft = (comp_state == COMP_LEFT);
        if (toolSign < 0)
            useLeft = !useLeft;

        Vec2 n = useLeft ? leftNormal(u) : rightNormal(u);
        Vec2 off = n * toolR;

        // if comping in or out then offset should be zero.
        if (src.compMode == CM_IN || src.compMode == CM_OUT)
        {
            off.x = 0.0f;
            off.y = 0.0f;
        }

        dst.type = src.type; // keep rapid vs feed
        dst.p_0 = src.p_0 + off;
        dst.p_1 = src.p_1 + off;
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
        validate(dst); // validate before offsetting

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

        dst.type = MOT_ARC;
        dst.center = src.center;
        dst.radius = r1;
        dst.p_0 = src.center + v0 * (r1 / lv0);
        dst.p_1 = src.center + v1 * (r1 / lv1);
        return true;
    }

    // Trim elements to a known intersection point.
    bool trimToTIP(Move2D &a, Move2D &b, Vec2 tip)
    {
        a.p_1 = tip;
        b.p_0 = tip;

        update_vectors(a);
        update_vectors(b);

        validate(a);
        validate(b);

        return a.valid && b.valid;
    }

    // Extend the moves so that they meet at the FIP.
    bool extendToFIP(Move2D &a, Move2D &b, Vec2 fip)
    {
        float fipDir1 = dot(fip - a.p_1, a.endDir);
        float fipDir2 = dot(fip - b.p_0, b.startDir);
        if (fipDir1 > 0 && fipDir2 < 0)
        {
            a.p_1 = fip;
            b.p_0 = fip;

            update_vectors(a);
            update_vectors(b);

            validate(a);
            validate(b);
        }
        return a.valid && b.valid;
    }

    Move2D makeRollArc(const Move2D &a, const Move2D &b)
    {
        Move2D roll;
        // roll.seqNum = (a.seqNum * 10) + 5; // for debugging
        roll.type = MOT_ARC;
        roll.compMode = CM_STEADY;
        roll.feed = (a.feed > 0) ? a.feed : b.feed;
        roll.p_0 = a.p_1;
        roll.p_1 = b.p_0;

        // Calculate the roll arc center using the angle bisector method
        roll.center = original_endpoint(a.p_1, a.endDir, (comp_state == COMP_LEFT) ^ (toolSign < 0), toolR);

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

        ArcDir preferredDir = useLeft ? ARC_CW : ARC_CCW;

        roll.arcDir = preferredDir;

        // Fast path: choose minor-arc direction from normalized turn sign.
        // Keep preferred direction when near-ambiguous (0 or 180 deg).
        const float turnEps = 1.0e-4f;
        if (r0 >= TOL && r1 >= TOL)
        {
            float turnSign = cross(v0, v1) / (r0 * r1);
            if (turnSign > turnEps)
            {
                if (preferredDir != ARC_CCW)
                    roll.arcDir = ARC_CCW;
            }
            else if (turnSign < -turnEps)
            {
                if (preferredDir != ARC_CW)
                    roll.arcDir = ARC_CW;
            }
        }
        roll.valid = true;
        update_vectors(roll);
        return roll;
    }

    int makeCornerTreatment(Move2D &a, Move2D &b, Move2D out[3])
    {
        int outCountLocal = 0;
        Move2D l1, l2;
        Move2D extA, extB;
        bool haveExtA = false;
        bool haveExtB = false;

        // nearly parallel
        float turnSign0 = cross(a.endDir * -1.0f, b.startDir);
        if (fabsf(turnSign0) <= BEVEL_VEC_TOL)
        {
            // we already know there is a gap > TOL because otherwise we wouldn't need corner treatment,
            // so just do a simple bevel.
            Move2D bevel = makeBevel(a, b);
            // float l = len(bevel.p_1 - bevel.p_0);
            out[outCountLocal++] = bevel;
            return outCountLocal;
        }

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

        Vec2 partCorner = original_endpoint(a.p_1, a.endDir, (comp_state == COMP_LEFT) ^ (toolSign < 0), toolR);
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
        // if the cap length is smaller than the tolerance.
        if (!validate(cap))
            return 0;

        if (a.type == MOT_LINE)
        {
            a.p_1 = ipForL1;
            // a.o_1 = a.p_1;
            update_vectors(a);
            if (!validate(a))
                return 0;
        }

        if (b.type == MOT_LINE)
        {
            b.p_0 = ipForL2;
            // b.o_0 = b.p_0;
            update_vectors(b);
            if (!validate(b))
                return 0;
        }

        if (haveExtA)
        {
            extA.p_1 = ipForL1;
            update_vectors(extA);
            if (!validate(extA))
                return 0;
            out[outCountLocal++] = extA;
        }

        out[outCountLocal++] = cap;

        if (haveExtB)
        {
            extB.p_0 = ipForL2;
            extB.p_1 = b.p_0;
            update_vectors(extB);
            if (!validate(extB))
                return 0;
            out[outCountLocal++] = extB;
        }

        return outCountLocal;
    }

    // Creates only the extension line segment for arc<->line/arc without modifying inputs.
    Move2D makeArcExtensionLineOnly(const Move2D &arc, bool fromEnd)
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
        extLnOut.startDir = dir;
        extLnOut.endDir = dir;
        update_vectors(extLnOut);
        validate(extLnOut);

        return extLnOut;
    }

    bool insertRollOrCorner(Move2D &a, Move2D &b, Move2D inserts[3], int &insertCount)
    {
        int startCount = insertCount;

        // with a very small offset we don't want to roll or bevel,
        // just connect them directly to avoid creating tiny segments that could cause issues later.
        float gap = len(b.p_0 - a.p_1);
        bool nearlyConnected = gap < gapTol;
        if (nearlyConnected)
        {
            // make a bevel to close the tiny gap
            Move2D bevel = makeBevel(a, b);
            if (!validate(bevel))
                return false;
            inserts[insertCount++] = bevel;
            return true;
        }

        if (cornerTreatment == CORNER_ROLL)
        {
            Move2D roll = makeRollArc(a, b);
            if (insertCount >= 3)
                return false;
            if (!validate(roll))
                return false;
            roll.hasXY = true;
            inserts[insertCount++] = roll;
            return true;
        }

        // else cornerTreatment == CORNER_CHAMFER
        Move2D cornerSegs[3];
        int cornerCount = makeCornerTreatment(a, b, cornerSegs);

        for (int i = 0; i < cornerCount && insertCount < 3; ++i)
        {
            cornerSegs[i].hasXY = true;
            inserts[insertCount++] = cornerSegs[i];
        }

        return insertCount > startCount;
    }

    void applyLogic(Move2D &a, Move2D &b, Move2D inserts[3], int &insertCount)
    {
        insertCount = 0;

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
            // when trimming we can get small segments that are invalid after trimming,
            if (!a.valid)
            {
                b.p_0 = a.p_1;
                return;
            }
            if (!b.valid)
            {
                a.p_1 = b.p_0;
                return;
            }
            return;
        }

        float gap = dist(b.p_0, a.p_1);
        bool nearlyConnected = gap < gapTol;
        if ((nearlyConnected || comping))
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

        if (is_convex(a, b))
        {
            if (!insertRollOrCorner(a, b, inserts, insertCount))
                reportCompError(CE_UNRESOLVED_GAP);
            return;
        }

        // If we get here: concave or no-good FIP -> just bevel.
        // It's better to have a small bevel than an unresolved gap or weird logic issues later.
        inserts[insertCount++] = makeBevel(a, b);
    }

    void handleArcArc(Move2D &a, Move2D &b, Move2D inserts[3], int &insertCount)
    {
        if (is_near(a.p_1, b.p_0) || is_near(a.center, b.center))
        {
            return; // connected arc or concentric
        }

        Vec2 ip1{}, ip2{};
        int tipCt = 0;
        IntersectType it = intersectCircleCircle(a, b, ip1, ip2, tipCt);

        if (it == IT_NONE || it == IT_TANGENT) // no intersection so close the gap with a chamfer or roll.
        {
            if (!insertRollOrCorner(a, b, inserts, insertCount))
                reportCompError(CE_UNRESOLVED_GAP);
            return;
        }

        /* Pre-compute arc angles once per arc (2× atan2f each) so that
           the per-candidate checks only need 1× atan2f for the test point. */
        ArcAngles aa = precomputeArcAngles(a);
        ArcAngles ba = precomputeArcAngles(b);

        // Determine TIP(true intersection point) candidates
        bool tip1 = (tipCt >= 1) && pointOnArcCached(a, ip1, aa) && pointOnArcCached(b, ip1, ba);
        bool tip2 = (tipCt == 2) && pointOnArcCached(a, ip2, aa) && pointOnArcCached(b, ip2, ba);

        if (tip1 || tip2)
        {
            Vec2 tip = tip1 ? ip1 : ip2;
            if (tip1 && tip2)
                tip = pickClosest(a.p_1, ip1, ip2);

            if (trimToTIP(a, b, tip))
                return;
        }

        // no tip but small gap: try extending to FIP (false intersection point)
        float gap = len(b.p_0 - a.p_1);
        bool nearlyConnected = gap < gapTol;
        if (nearlyConnected)
        {
            Vec2 tip = pickClosest(a.p_1, ip1, ip2);
            if (extendToFIP(a, b, tip))
            {
                return;
            }
        }

        if (!insertRollOrCorner(a, b, inserts, insertCount))
            reportCompError(CE_UNRESOLVED_GAP);
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
        // chained arc/line implies tangent if created from offsetting tangent adjacent elements.
        if (is_near(a.p_1, b.p_0))
        {
            b.p_0 = a.p_1; // snap together to avoid numerical issues later.
            return;        // already connected, no need to roll or trim.
        }

        // Intersect infinite line with circle
        Vec2 ip1{}, ip2{};
        int count = 0;
        IntersectType it = intersectLineCircle(lin->p_0, lin->p_1, arc->center, arc->radius, ip1, ip2, count);

        if (it == IT_NONE)
        {
            if (!insertRollOrCorner(a, b, inserts, insertCount))
                reportCompError(CE_UNRESOLVED_GAP);
            return;
        }

        // Evaluate TIP: point must lie on finite line segment and on arc sweep
        /* Pre-compute arc angles once
        (avoids redundant atan2f when testing multiple line-circle intersection candidates). */
        ArcAngles arca = precomputeArcAngles(*arc);

        bool tip1 = false, tip2 = false;
        if (count >= 1)
            tip1 = pointOnSegment(lin->p_0, lin->p_1, ip1) && pointOnArcCached(*arc, ip1, arca);
        if (count == 2)
            tip2 = pointOnSegment(lin->p_0, lin->p_1, ip2) && pointOnArcCached(*arc, ip2, arca);

        // If any TIP exists: trim
        if (tip1 || tip2)
        {
            Vec2 tip = tip1 ? ip1 : ip2;
            if (tip1 && tip2)
                tip = pickClosest(a.p_1, ip1, ip2);
            if (trimToTIP(a, b, tip))
                return;
        }

        // no tip but small gap: try extending to FIP (false intersection point)
        float gap = len(b.p_0 - a.p_1);
        bool nearlyConnected = gap < gapTol;
        if (nearlyConnected)
        {
            Vec2 tip = pickClosest(a.p_1, ip1, ip2);
            if (extendToFIP(a, b, tip))
            {
                return;
            }
        }

        // fallback to roll or chamfer if no intersection or extension possible.
        if (!insertRollOrCorner(a, b, inserts, insertCount))
            reportCompError(CE_UNRESOLVED_GAP);
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

            Vec2 ip1{}, ip2{};
            int count = 0;
            IntersectType it = intersectLineCircle(L.p_0, L.p_1, C.center, C.radius, ip1, ip2, count);
            if (it == IT_NONE)
                return 0;

            /* Pre-compute arc angles once (avoids redundant atan2f
               when testing multiple candidate points on the same arc). */
            ArcAngles ca = precomputeArcAngles(C);

            int n = 0;
            if (count >= 1 && pointOnSegment(L.p_0, L.p_1, ip1) && pointOnArcCached(C, ip1, ca))
                tip1 = ip1, n++;
            if (count == 2 && pointOnSegment(L.p_0, L.p_1, ip2) && pointOnArcCached(C, ip2, ca))
            {
                if (n == 0)
                    tip1 = ip2;
                else
                    tip2 = ip2;
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

            Vec2 ip1{}, ip2{};
            int count = 0;
            IntersectType it = intersectCircleCircle(A, B, ip1, ip2, count);
            if (it == IT_NONE)
                return 0;

            /* Pre-compute arc angles once per arc — avoids up to 8 redundant
               atan2f calls when testing two candidates against two arcs. */
            ArcAngles aa = precomputeArcAngles(A);
            ArcAngles ba = precomputeArcAngles(B);

            int n = 0;
            if (count >= 1 && pointOnArcCached(A, ip1, aa) && pointOnArcCached(B, ip1, ba))
                tip1 = ip1, n++;
            if (count == 2 && pointOnArcCached(A, ip2, aa) && pointOnArcCached(B, ip2, ba))
            {
                if (n == 0)
                    tip1 = ip2;
                else
                    tip2 = ip2;
                n++;
            }
            return n;
        }

        return 0;
    }

    static AABB2 aabb_of(const Move2D &m)
    {
        AABB2 b;
        b.minx = fminf(m.p_0.x, m.p_1.x);
        b.maxx = fmaxf(m.p_0.x, m.p_1.x);
        b.miny = fminf(m.p_0.y, m.p_1.y);
        b.maxy = fmaxf(m.p_0.y, m.p_1.y);

        if (m.type == MOT_ARC && fabsf(m.radius) > TOL)
        {
            // Cardinal angles are known constants — no atan2f needed.
            float a0n = angleNorm(atan2f(m.p_0.y - m.center.y, m.p_0.x - m.center.x));
            float a1n = angleNorm(atan2f(m.p_1.y - m.center.y, m.p_1.x - m.center.x));

            // right (+x) = 0, top (+y) = PI/2, left (-x) = PI, bottom (-y) = 3*PI/2
            if (angle_on_arc_norm(a0n, a1n, PI, m.arcDir))
                b.minx = m.center.x - m.radius;

            if (angle_on_arc_norm(a0n, a1n, 0.0f, m.arcDir))
                b.maxx = m.center.x + m.radius;

            if (angle_on_arc_norm(a0n, a1n, PI * 0.5f, m.arcDir))
                b.maxy = m.center.y + m.radius;

            if (angle_on_arc_norm(a0n, a1n, PI * 1.5f, m.arcDir))
                b.miny = m.center.y - m.radius;
        }
        return b;
    }

    // Compute bounding boxes for an array of moves, storing results in a parallel AABB2 array.
    static void init_all_aabb(const Move2D *moves, AABB2 *bounds, int start, int count)
    {
        for (int i = start; i < count; ++i)
        {
            if (moves[i].type != MOT_EMPTY && moves[i].valid)
                bounds[i] = aabb_of(moves[i]);
        }
    }

    static bool aabb_intersects(const AABB2 &a, const AABB2 &b)
    {
        return !(a.maxx < b.minx || a.minx > b.maxx || a.maxy < b.miny || a.miny > b.maxy);
    }

    static CrossingHit lookAheadForCrossing(Move2D *moves, AABB2 *bounds, int numMoves,
                                            int srcIdx,
                                            int startTargetIdx,
                                            int maxIdx,
                                            int maxLookahead,
                                            int firstCutIdx,
                                            int lastCutIdx)
    {
        CrossingHit best;
        best.hit = false;
        best.dist = 1e30f; // start with big distance so that any real crossing will be closer.

        if (!moves[srcIdx].valid)
            return best;

        Move2D &src = moves[srcIdx];

        int j = startTargetIdx + 1; // start looking from the element after the immediate neighbor
        for (int r = 0; r <= maxLookahead && j < maxIdx && j < numMoves; ++r, ++j)
        {
            Move2D &target = moves[j];
            if (!target.valid)
                continue;

            // Closed-loop seam case: do not trim when comparing first cutting move vs last cutting move.
            if (srcIdx == firstCutIdx && j == lastCutIdx)
                break;

            // Skip immediate neighbor to avoid trimming the normal shared endpoint
            if (j < srcIdx + 2)
                continue;

            if (!aabb_intersects(bounds[srcIdx], bounds[j]))
            {
                continue;
            }

            // compare the Z values within a tolerance
            // special case for helix moves.
            if (fabsf(src.z_0 - target.z_0) > TOL || fabsf(src.z_0 - target.z_1) > TOL)
                continue;

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
    // return false if failed to trim (which can only happen if a comp in move is crossing,
    bool trimCrossingElements(Move2D *moves, AABB2 *bounds, int &srcIdx, int maxIdx, int lookahead, int &hitTargetIdx)
    {
        int compInIdx = -1;
        int compOutIdx = -1;
        // calculate AABBs for all elements once upfront to speed up intersection testing in the lookahead loop.
        init_all_aabb(moves, bounds, srcIdx, maxIdx);

        // find the moves adjacent to the comp in and comp out.
        // we want to skip these in the crossing logic since they are allowed to "cross" in the sense that they share geometry but should not be trimmed since they are intentionally connected that way as part of the comp.
        int firstCutIdx = first_comp_move(moves, maxIdx);
        int lastCutIdx = -1;
        if (firstCutIdx >= 0)
            lastCutIdx = last_comp_move(moves, maxIdx, firstCutIdx);

        if (firstCutIdx >= 0)
        {
            compInIdx = firstCutIdx - 1; // comp in is immediately before first cut move
        }

        if (lastCutIdx >= 0)
        {
            compOutIdx = lastCutIdx + 1; // comp out is immediately after last cut move
        }

        if (moves[srcIdx].compMode == CM_IN)
        {
            // if comp in move is adjacent to flipped arc.
            if (firstCutIdx > -1)
            {
                if (moves[firstCutIdx].type == MOT_ARC && moves[firstCutIdx].radius <= 0)
                {
                    reportCompError(CE_FLIPPED_ARC);
                    return true;
                }
            }
        }

        if (moves[srcIdx].compMode == CM_OUT)
        {
            // if comp out move is adjacent to flipped arc.
            if (lastCutIdx > -1)
            {
                if (moves[lastCutIdx].type == MOT_ARC && moves[lastCutIdx].radius <= 0)
                {
                    reportCompError(CE_FLIPPED_ARC);
                    return true;
                }
            }
        }

        while (srcIdx < maxIdx)
        {
            while (srcIdx < maxIdx && (!moves[srcIdx].valid)) //|| moves[srcIdx].compMode == CM_IN
                srcIdx++;

            if (srcIdx >= maxIdx)
                break;

            int targetIdx = srcIdx + 1;
            while (targetIdx < maxIdx && !moves[targetIdx].valid)
                targetIdx++; // skip invalid targets
            if (targetIdx >= maxIdx)
                break; // if we have no valid targets ahead, we are done.

            CrossingHit crossing = lookAheadForCrossing(moves, bounds, maxIdx, srcIdx, targetIdx, maxIdx, lookahead, firstCutIdx, lastCutIdx);
            if (!crossing.hit)
            {
                srcIdx++;
                continue;
            }
            hitTargetIdx = crossing.j;

            // if we are here we have a crossing.
            if (moves[srcIdx].compMode == CM_IN)
            {
                if (hitTargetIdx < lastCutIdx)
                {
                    // comp in should never cross.
                    reportCompError(CE_COMP_IN_CROSSING);
                    return false;
                }
                srcIdx++;
                continue; // skip trimming for comp in move.
            }

            if (moves[hitTargetIdx].compMode == CM_OUT)
            {
                // comp out should never cross.
                reportCompError(CE_COMP_OUT_CROSSING);
                return false;
            }

            // only run this if we have a non-lead-in-out crossing and it is not a head-bites-tail.
            bool shouldTrim = compInIdx != -1 && compOutIdx != -1 && srcIdx == compInIdx && hitTargetIdx == compOutIdx;
            if (!shouldTrim)
            {
                (void)trimToTIP(moves[srcIdx], moves[hitTargetIdx], crossing.tip);
                CutterComp2D::invalidateRange(moves, srcIdx, hitTargetIdx);
            }
            // trimmedTo becomes new srcElement
            srcIdx = hitTargetIdx;
        }

        return true;
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
                // update_vectors(a);

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
