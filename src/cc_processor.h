/*
 * cc_processor.h - 2D cutter compensation with logic
 * Jason Titcomb 2026
 * MIT License – see LICENSE file in repository root
 */

#pragma once
#include "cc_math.h"
typedef void (*CcOutputCB)(const char *text, size_t len);
typedef void (*CcErrorCB)(const char *message, CompError err, uint32_t lineNum);

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
    } callbacks;

    float toolRadius = 0.0f;
    CornerType cornerTreatment = CORNER_ROLL;
    bool globalTrimCrossing = true;
    bool globalMerge = true;
    bool emitStatusComments = true;
};

enum JunctionType : uint8_t
{
    JT_NONE = 0,
    JT_TRIM_TO_INTERSECTION,
    JT_TRIM_ONE_SIDED,
    JT_EXTEND_TO_INTERSECTION,
    JT_ROLL_AROUND,
};

struct Junction
{
    JunctionType type = JT_NONE;
    Vec2 p{0, 0};
};

class CutterComp2D
{
private:
    CcOutputCB outputCB_ = nullptr;
    CcErrorCB errorCB_ = nullptr;
    Units units = UNITS_MM;
    CcMainOptions options;
    bool hasCompError = false;
    uint32_t lastLineNum = 0;

    void reportCompError(CompError err)
    {
        hasCompError = true;
        if (errorCB_)
        {
            switch (err)
            {
            case CE_ARC_RADIUS_MISMATCH:
                errorCB_("Arc radius inconsistency", err, lastLineNum);
                break;
            case CE_INVALID_MOVE:
                errorCB_("Invalid move", err, lastLineNum);
                break;
            case CE_COMP_MOVE_TOO_SHORT:
                errorCB_("Comp move too short", err, lastLineNum);
                break;
            case CE_ARC_LT_TOOL_RAD:
                errorCB_("Arc smaller than tool radius", err, lastLineNum);
                break;
            case CE_COMP_IN_CROSSING:
                errorCB_("Comp-in crossing", err, lastLineNum);
                break;
            case CE_COMP_OUT_CROSSING:
                errorCB_("Comp-out crossing", err, lastLineNum);
                break;
            case CE_UNRESOLVED_GAP:
                errorCB_("Unresolved gap", err, lastLineNum);
                break;
            default:
                errorCB_("Unknown comp error", err, lastLineNum);
            }
        }
    }

    CompSide effectiveCompSide() const
    {
        if (toolSign >= 0)
            return compSide;

        if (compSide == COMP_LEFT)
            return COMP_RIGHT;

        if (compSide == COMP_RIGHT)
            return COMP_LEFT;

        return compSide;
    }

    bool compUsesLeft() const
    {
        return effectiveCompSide() == COMP_LEFT;
    }

    static bool isLineLike(const Move2D &m)
    {
        return m.type == MOT_LINE || m.type == MOT_RAPID;
    }

    static bool hasRapidMove(const Move2D &a, const Move2D &b)
    {
        return a.type == MOT_RAPID || b.type == MOT_RAPID;
    }

    static int zMoveDirection(float z0, float z1)
    {
        if (is_equal(z0, z1))
            return 0;
        return (z1 > z0) ? 1 : -1;
    }

    static bool shouldReplacePendingZTarget(const Move2D &pending, const Move2D &candidate)
    {
        const int pendingDir = zMoveDirection(pending.z_0, pending.z_1);
        const int candidateDir = zMoveDirection(candidate.z_0, candidate.z_1);

        if (pendingDir != 0 && pendingDir == candidateDir)
            return fabsf(candidate.z_1) > fabsf(pending.z_1);

        return true;
    }

    void validateLineInversion(Move2D &m)
    {
        if (!m.valid || !isLineLike(m))
            return;

        const float lineLen = len(m.p_1 - m.p_0);
        if (lineLen < TOL)
            return;

        Vec2 u = (m.p_1 - m.p_0) * (1.0f / lineLen);
        if (dot(u, m.startDir) < -0.999f)
        {
            m.valid = true;
            if (!options.globalTrimCrossing)
                reportCompError(CE_INVALID_MOVE);
        }
    }

    bool validate(Move2D &m)
    {
        if (isLineLike(m))
        {
            const float lineLen = len(m.p_1 - m.p_0);
            if (lineLen < TOL)
            {
                m.hasXY = false;
                m.valid = false;
                return false;
            }

            m.valid = true;
            return true;
        }

        if (m.type != MOT_ARC)
            return m.valid;

        // const bool hasArcDirs = len(m.startDir) >= TOL || len(m.endDir) >= TOL;
        bool degenerate = fabsf(m.radius) < TOL;
        float sw = arcSweepDeg(m);
        bool keepTinyArc = (degenerate || sw < MIN_ARC_LEN);
        bool sweepOk = sw <= MAX_SWEEP_DEG && (sw >= MIN_ARC_LEN || keepTinyArc);

        if ((degenerate || !sweepOk) && !keepTinyArc)
        {
            m.valid = false;
            if (options.globalTrimCrossing)
                return m.valid;

            reportCompError(CE_ARC_LT_TOOL_RAD);
            return false;
        }

        m.valid = true;

        bool consistent = is_radius_consistent(m);

        if (!consistent)
            reportCompError(CE_ARC_RADIUS_MISMATCH);
        if (!sweepOk)
            reportCompError(CE_INVALID_MOVE);

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
    bool havePendingZMove = false;
    Move2D pendingZMove;

public:
    CompSide compSide = COMP_OFF;
    CompMode compMode = CM_NONE;
    void setOptions(const CcMainOptions &opts)
    {
        options = opts;
        setToolRadius(opts.toolRadius);
        resetState();
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
        CompSide prevSide = compSide;
        compSide = s;

        if (prevSide == COMP_OFF && s != COMP_OFF)
        {
            compMode = CM_IN;
            return;
        }

        if (prevSide != COMP_OFF && s == COMP_OFF)
        {
            compMode = CM_OUT;
            return;
        }

        if (s == COMP_OFF)
        {
            compMode = CM_NONE;
            return;
        }

        if (prevSide != s)
        {
            compMode = CM_IN;
            return;
        }

        // Same side and still in comp: keep CM_IN until the entry pair is consumed,
        // then remain in CM_STEADY for subsequent moves.
        if (compMode == CM_NONE || compMode == CM_OUT)
        {
            compMode = CM_IN;
            return;
        }

        if (compMode != CM_IN)
            compMode = CM_STEADY;
    }

    bool pushIn(const Move2D &m)
    {
        if (inCount >= IN_CAP)
            return false;
        input_buffer[(inHead + inCount) % IN_CAP] = m;
        inCount++;
        return true;
    }

    bool emitPendingZMoveAt(const Move2D &anchor)
    {
        if (!havePendingZMove)
            return true;

        Move2D zMove = pendingZMove;
        zMove.p_0 = anchor.p_1;
        zMove.p_1 = anchor.p_1;
        zMove.z_0 = anchor.z_1;
        zMove.hasXY = false;
        zMove.hasZ = !is_equal(zMove.z_1, zMove.z_0);

        if (zMove.hasZ)
            pushOut(zMove);

        havePendingZMove = false;
        return !hasCompError;
    }

    // Main pump
    bool process(void)
    {
        // if (hasCompError)
        //     return false;

        while (inCount > 0)
        {
            // Need output space for worst case: prev + up to 3 inserted moves
            if (!outHasSpace(4))
                return false;

            Move2D curOff = popIn();
            // Capture mode per move before offsetting so CM_IN/CM_OUT only affect
            // the intended entry/exit element.
            curOff.compMode = compMode;
            lastLineNum = curOff.lineNum;

            if (curOff.type == MOT_EMPTY)
                continue;

            update_vectors(curOff);

            curOff.hasZ = !is_equal(curOff.z_1, curOff.z_0);
            // Z-only move: no XY displacement, nothing to offset
            if (curOff.hasZ && !curOff.hasXY)
            {
                if (havePrevMove2D)
                {
                    if (havePendingZMove)
                    {
                        // Edge case: if we get multiple Z-only moves in a row, only keep the longest one in the same direction
                        // Check if the new Z move should replace the pending one
                        // (e.g. if it's a longer move in the same direction)
                        if (shouldReplacePendingZTarget(pendingZMove, curOff))
                        {
                            pendingZMove.z_1 = curOff.z_1;
                            pendingZMove.lineNum = curOff.lineNum;
                            pendingZMove.feed = curOff.feed;
                            pendingZMove.type = curOff.type;
                            pendingZMove.hasZ = !is_equal(pendingZMove.z_1, pendingZMove.z_0);
                        }
                    }
                    else
                    {
                        pendingZMove = curOff;
                        pendingZMove.p_0 = prevOff.p_1;
                        pendingZMove.p_1 = prevOff.p_1;
                        pendingZMove.z_0 = prevOff.z_1;
                        pendingZMove.hasXY = false;
                        pendingZMove.hasZ = !is_equal(pendingZMove.z_1, pendingZMove.z_0);
                        havePendingZMove = pendingZMove.hasZ;
                    }
                }
                else
                {
                    if (!outHasSpace(1))
                        return false;
                    pushOut(curOff);
                }
                continue;
            }

            if (!validate(curOff)) // validate before offsetting so that we don't waste time processing invalid moves, and also so that the comp mode is captured correctly for error reporting.
                return false;

            if (!offsetMove(curOff))
                return false;

            if (!havePrevMove2D)
            {
                prevOff = curOff;
                havePrevMove2D = true;
                if (curOff.compMode == CM_IN)
                    compMode = CM_STEADY;
                else if (curOff.compMode == CM_OUT)
                    compMode = CM_NONE;
                continue;
            }

            Move2D inserts[3]; // allow up to 3 inserts for corner treatment.
            int insertCount = 0;

            if (prevOff.compMode == CM_IN)
            {
                const float originalLen = len(prevOff.p_1 - prevOff.p_0);
                if (originalLen <= toolR + TOL)
                {
                    reportCompError(CE_COMP_MOVE_TOO_SHORT);
                    return false;
                }

                // Modify the previous move so that the end is the start of the current move.
                prevOff.p_1 = curOff.p_0;

                const float finalLen = len(prevOff.p_1 - prevOff.p_0);
                if (finalLen <= TOL)
                {
                    reportCompError(CE_COMP_MOVE_TOO_SHORT);
                    return false;
                }

                update_vectors(prevOff);
            }

            if (curOff.compMode == CM_OUT)
            {
                const float originalLen = len(curOff.p_1 - curOff.p_0);
                if (originalLen <= toolR + TOL)
                {
                    reportCompError(CE_COMP_MOVE_TOO_SHORT);
                    return false;
                }

                // Modify the G40 move so that the start is the end of the previous move.
                curOff.p_0 = prevOff.p_1;

                const float finalLen = len(curOff.p_1 - curOff.p_0);
                if (finalLen <= TOL)
                {
                    reportCompError(CE_COMP_MOVE_TOO_SHORT);
                    return false;
                }

                update_vectors(curOff);
            }

            // Apply decision tree between prevOff and curOff
            if (curOff.compMode == CM_STEADY)
                applyLogic(prevOff, curOff, inserts, insertCount);

            validateLineInversion(prevOff);    
            
            // Emit previous + inserts; hold curOff as new prev
            if (prevOff.valid)
            {
                pushOut(prevOff);
                if (!emitPendingZMoveAt(prevOff))
                    return false;
                for (int i = 0; i < insertCount; ++i)
                    pushOut(inserts[i]);
            }

            if (curOff.compMode == CM_IN)
                compMode = CM_STEADY;
            else if (curOff.compMode == CM_OUT)
                compMode = CM_NONE;

            prevOff = curOff;
        }
        return true;
    }

    // Flush delayed last element
    void flush()
    {
        process();
        if (havePrevMove2D && outHasSpace(havePendingZMove ? 2 : 1))
        {
            if (prevOff.valid && prevOff.type != MOT_EMPTY)
            {
                pushOut(prevOff);
                (void)emitPendingZMoveAt(prevOff);
            }
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

    void resetState()
    {
        havePrevMove2D = false;
        havePendingZMove = false;
    }

    Move2D makeBevel(const Move2D &a, const Move2D &b)
    {
        Move2D m;
        m.lineNum = a.lineNum;
        m.hasXY = true;
        m.type = MOT_LINE;
        m.feed = (a.feed > 0) ? a.feed : b.feed;
        m.p_0 = a.p_1;
        m.p_1 = b.p_0;
        m.z_0 = b.z_0;
        m.z_1 = b.z_0;
        m.hasZ = false;
        update_vectors(m);
        return m;
    }

    bool convex_from_winding(int cw)
    {
        if (cw == 0)
            return false;

        bool isLeft = compUsesLeft();

        if (isLeft)
            return !(cw > 0);
        return (cw > 0);
    }

    bool is_convex(const Move2D &a, const Move2D &b)
    {
        int cw = get_winding_dir(a.endDir, b.startDir);
        return convex_from_winding(cw);
    }

    bool roll_fits_line_line(Vec2 p0, Vec2 extensionPoint, Vec2 p1) const
    {
        float lenA = dist(p0, extensionPoint);
        float lenB = dist(extensionPoint, p1);
        if (lenA <= TOL || lenB <= TOL || toolR <= TOL)
            return false;

        Vec2 vin = normalize(extensionPoint - p0);
        Vec2 vout = normalize(p1 - extensionPoint);
        if (len(vin) <= TOL || len(vout) <= TOL)
            return false;

        float cosAlpha = c2d_clamp(dot(vin, vout), -1.0f, 1.0f);
        float alpha = std::acos(cosAlpha);

        if (alpha <= TOL || std::fabs(PI - alpha) <= TOL)
            return false;

        float need = toolR * std::tan(0.5f * alpha);
        return lenA >= need && lenB >= need;
    }

    static bool pointOnFiniteElem(const Move2D &m, Vec2 p)
    {
        if (isLineLike(m))
            return pointOnSegment(m.p_0, m.p_1, p);

        if (m.type == MOT_ARC)
        {
            ArcAngles aa = precomputeArcAngles(m);
            return pointOnArcCached(m, p, aa);
        }

        return false;
    }

    static int intersectCarrier(const Move2D &a, const Move2D &b, Vec2 pts[2])
    {
        if (isLineLike(a) && isLineLike(b))
        {
            bool tip = false;
            IntersectType it = intersectLineLine(a, b, pts[0], tip);
            return (it == IT_NONE) ? 0 : 1;
        }

        if (a.type == MOT_ARC && b.type == MOT_ARC)
        {
            if (is_near(a.center, b.center))
                return 0;

            int count = 0;
            IntersectType it = intersectCircleCircle(a, b, pts[0], pts[1], count);
            if (it == IT_NONE)
                return 0;
            return count;
        }

        const Move2D &line = isLineLike(a) ? a : b;
        const Move2D &arc = (a.type == MOT_ARC) ? a : b;
        int count = 0;
        IntersectType it = intersectLineCircle(line.p_0, line.p_1, arc.center, arc.radius, pts[0], pts[1], count);
        if (it == IT_NONE)
            return 0;
        return count;
    }

    static int finiteIntersectionPoints(const Move2D &a, const Move2D &b, Vec2 pts[2])
    {
        Vec2 carrierPts[2]{};
        int carrierCount = intersectCarrier(a, b, carrierPts);
        int finiteCount = 0;

        for (int i = 0; i < carrierCount; ++i)
        {
            Vec2 p = carrierPts[i];
            if (!pointOnFiniteElem(a, p) || !pointOnFiniteElem(b, p))
                continue;

            if (finiteCount > 0 && is_near(pts[0], p))
                continue;

            pts[finiteCount++] = p;
        }

        return finiteCount;
    }

    bool isForwardExtensionPoint(const Move2D &a, const Move2D &b, Vec2 p)
    {
        float fipDir1 = dot(p - a.p_1, a.endDir);
        float fipDir2 = dot(p - b.p_0, b.startDir);
        return fipDir1 > 0 && fipDir2 < 0;
    }

    bool solveJunction(const Move2D &a, const Move2D &b, Junction &outjunc)
    {
        Vec2 carrierPts[2]{};
        Vec2 trimPts[2]{};
        Vec2 bestTrimPoint{};
        Vec2 bestExtendPoint{};

        float bestTrimScore = 0.0f;
        float bestExtendScore = 0.0f;
        bool foundTrim = false;
        bool foundExtend = false;


        int carrierCount = intersectCarrier(a, b, carrierPts);
        int trimCount = finiteIntersectionPoints(a, b, trimPts);

        for (int i = 0; i < trimCount; ++i)
        {
            Vec2 p = trimPts[i];
            float score = distFromStart_along(a, p) + distFromStart_along(b, p);
            if (!foundTrim || score < bestTrimScore)
            {
                bestTrimPoint = p;
                bestTrimScore = score;
                foundTrim = true;
            }
        }

        for (int i = 0; i < carrierCount; ++i)
        {
            Vec2 p = carrierPts[i];
            if (isForwardExtensionPoint(a, b, p))
            {
                float score = dist(a.p_1, p) + dist(b.p_0, p);
                if (!foundExtend || score < bestExtendScore)
                {
                    bestExtendPoint = p;
                    bestExtendScore = score;
                    foundExtend = true;
                }
            }
        }

        if (foundTrim)
        {
            outjunc.type = JT_TRIM_TO_INTERSECTION;
            outjunc.p = bestTrimPoint;
            return true;
        }

        if (foundExtend)
        {
            outjunc.type = JT_EXTEND_TO_INTERSECTION;
            outjunc.p = bestExtendPoint;
            return true;
        }

        outjunc.type = JT_NONE;
        outjunc.p = Vec2{};
        return false;
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
    int first_steady_move(const Move2D *moves, int count)
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
    int last_steady_move(const Move2D *moves, int count, int startAt)
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
    bool offsetMove(Move2D &m)
    {
        if (m.type == MOT_LINE || m.type == MOT_RAPID)
            return offsetLine(m);
        if (m.type == MOT_ARC)
            return offsetArc(m);

        return false;
    }

    bool offsetLine(Move2D &m)
    {
        Vec2 v = m.p_1 - m.p_0;
        float l = len(v);
        if (l < TOL)
        {
            m.valid = false;
            return false;
        }
        Vec2 u = v * (1.0f / l);

        // Apply toolSign to flip the offset direction if negative tool radius
        bool useLeft = compUsesLeft();

        Vec2 n = useLeft ? leftNormal(u) : rightNormal(u);
        Vec2 off = n * toolR;

        // if comping in or out then offset should be zero.
        if (m.compMode == CM_IN || m.compMode == CM_OUT)
        {
            off.x = 0.0f;
            off.y = 0.0f;
        }

        m.p_0 = m.p_0 + off;
        m.p_1 = m.p_1 + off;
        return true;
    }

    // Concentric arc offset like before (good enough for your VB logic)
    bool offsetArc(Move2D &m)
    {
        float r0 = m.radius;
        if (fabsf(r0) < TOL)
            r0 = len(m.p_0 - m.center);
        if (fabsf(r0) < TOL)
            return false;

        float dr = toolR;
        bool ccw = (m.arcDir == ARC_CCW);
        bool left = compUsesLeft();

        float r1;
        if (ccw)
            r1 = r0 + (left ? -dr : +dr);
        else
            r1 = r0 + (left ? +dr : -dr);

        // Keep the rad regardless. If it's negative, the offset will flip to the other side of the center,
        // which is a valid geometry (though maybe not what you want for a real cutter comp).
        // The logic later should be able to handle it as long as we keep the direction semantics consistent.

        Vec2 v0 = m.p_0 - m.center;
        Vec2 v1 = m.p_1 - m.center;
        float lv0 = len(v0), lv1 = len(v1);

        if (lv0 < TOL || lv1 < TOL)
        {
            m.valid = false;
            return false;
        }
        m.radius = r1;

        if (!options.globalTrimCrossing)
        {
            if (m.radius < TOL)
            {
                //m.valid = false;
                //If i invalidate this arc there is a contition where I can't create a following roll arc.
                reportCompError(CE_ARC_LT_TOOL_RAD);
            }
        }

        m.p_0 = m.center + v0 * (r1 / lv0);
        m.p_1 = m.center + v1 * (r1 / lv1);
        return true;
    }

    // Trim elements to a known intersection point.
    bool trimTo(Move2D &a, Move2D &b, Vec2 tip)
    {
        a.p_1 = tip;
        b.p_0 = tip;

        // update_vectors(a); Deliberately defer updating the vectors until we process the next move, 
        // update_vectors(b);

        validate(a);
        validate(b);

        return a.valid && b.valid;
    }

    // Extend the moves so that they meet at the FIP.
    bool extendTo(Move2D &a, Move2D &b, Vec2 fip)
    {
        float fipDir1 = dot(fip - a.p_1, a.endDir);
        float fipDir2 = dot(fip - b.p_0, b.startDir);
        if (fipDir1 > 0 && fipDir2 < 0)
        {
            a.p_1 = fip;
            b.p_0 = fip;
            // defer updating the vectors until we process the next move.
            // update_vectors(a);
            // update_vectors(b);

            validate(a);
            validate(b);
        }
        return a.valid && b.valid;
    }

    Move2D makeRollArc(const Move2D &a, const Move2D &b)
    {
        Move2D roll;
        roll.lineNum = a.lineNum;
        roll.type = MOT_ARC;
        roll.compMode = CM_STEADY;
        roll.feed = (a.feed > 0) ? a.feed : b.feed;
        roll.p_0 = a.p_1;
        roll.p_1 = b.p_0;
        roll.z_0 = b.z_0;
        roll.z_1 = b.z_0;
        roll.hasZ = false;

        // Calculate the roll arc center using the angle bisector method
        roll.center = roll_center(a.p_1, a.endDir, compUsesLeft(), toolR);

        Vec2 v0 = roll.p_0 - roll.center;
        Vec2 v1 = roll.p_1 - roll.center;
        float r0 = len(v0);
        float r1 = len(v1);

        float r = toolR;
        if (r0 >= TOL && r1 >= TOL)
            r = 0.5f * (r0 + r1);
        else if (r0 >= TOL)
            r = r0;
        else if (r1 >= TOL)
            r = r1;

        if (r0 >= TOL)
            roll.p_0 = roll.center + v0 * (r / r0);
        if (r1 >= TOL)
            roll.p_1 = roll.center + v1 * (r / r1);

        roll.radius = r;

        // Apply toolSign flip to arc direction
        bool useLeft = compUsesLeft();

        ArcDir preferredDir = useLeft ? ARC_CW : ARC_CCW;

        roll.arcDir = preferredDir;

        roll.valid = true;
        update_vectors(roll);
        return roll;
    }

    int makeCornerTreatment(Move2D &a, Move2D &b, Move2D outmove[3])
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
            outmove[outCountLocal++] = bevel;
            return outCountLocal;
        }

        if (isLineLike(a))
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

        if (isLineLike(b))
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

        Vec2 partCorner = roll_center(a.p_1, a.endDir, compUsesLeft(), toolR);
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
        cap.lineNum = a.lineNum;
        cap.type = MOT_LINE;
        cap.compMode = CM_STEADY;
        cap.feed = (a.feed > 0) ? a.feed : b.feed;
        cap.z_0 = b.z_0;
        cap.z_1 = b.z_0;
        cap.hasZ = false;
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

        if (isLineLike(a))
        {
            a.p_1 = ipForL1;
            // a.o_1 = a.p_1;
            update_vectors(a);
            if (!validate(a))
                return 0;
        }

        if (isLineLike(b))
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
            extA.z_0 = b.z_0;
            extA.z_1 = b.z_0;
            extA.hasZ = false;
            update_vectors(extA);
            if (!validate(extA))
                return 0;
            outmove[outCountLocal++] = extA;
        }

        outmove[outCountLocal++] = cap;

        if (haveExtB)
        {
            extB.p_0 = ipForL2;
            extB.p_1 = b.p_0;
            extB.z_0 = b.z_0;
            extB.z_1 = b.z_0;
            extB.hasZ = false;
            update_vectors(extB);
            if (!validate(extB))
                return 0;
            outmove[outCountLocal++] = extB;
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
        extLnOut.lineNum = arc.lineNum;
        extLnOut.compMode = arc.compMode;
        extLnOut.feed = arc.feed;
        extLnOut.z_0 = fromEnd ? arc.z_1 : arc.z_0;
        extLnOut.z_1 = extLnOut.z_0;
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
        // just skip it
        float gap = len(b.p_0 - a.p_1);
        bool nearlyConnected = gap < gapTol;
        if (nearlyConnected)
        {
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
        if (!a.hasXY || !b.hasXY)
            return;

        if (isLineLike(a) && isLineLike(b))
        {
            handleLineLine(a, b, inserts, insertCount);
        }
        else if (a.type == MOT_ARC && b.type == MOT_ARC)
        {
            handleArcArc(a, b, inserts, insertCount);
        }
        else if ((a.type == MOT_ARC && isLineLike(b)) || (isLineLike(a) && b.type == MOT_ARC))
        {
            handleArcLine(a, b, inserts, insertCount);
        }
    }

    void handleLineLine(Move2D &a, Move2D &b, Move2D inserts[3], int &insertCount)
    {
        Junction junction;

        // trivial check for chained elements
        // chained line implies colinear if created from offsetting
        if (is_near(a.p_1, b.p_0))
        {
            return; // already connected, no need to roll or trim.
        }

        float gap = dist(b.p_0, a.p_1);
        bool anyRapid = hasRapidMove(a, b);
        bool avoidRoll = anyRapid || (gap < gapTol); // rapid corners should resolve by trim/extend instead of inserting a roll.
        bool convex = is_convex(a, b);

        solveJunction(a, b, junction);

        if (junction.type == JT_TRIM_TO_INTERSECTION)
        {
            trimTo(a, b, junction.p);
            return;
        }

        // Convex near-gap/rapid: prefer extending to FIP over rolling
        if (convex && avoidRoll && junction.type == JT_EXTEND_TO_INTERSECTION)
        {
            if (extendTo(a, b, junction.p))
                return;
        }

        if (a.compMode == CM_IN)
        {
            a.p_1 = b.p_0;
            update_vectors(a);
            validate(a);
            return;
        }

        if (b.compMode == CM_OUT)
        {
            b.p_0 = a.p_1;
            update_vectors(b);
            validate(b);
            return;
        }

        // Convex: try roll, with line-line fit check; fall back to extend if no fit
        if (!anyRapid && convex)
        {
            if (junction.type == JT_EXTEND_TO_INTERSECTION)
            {
                if (roll_fits_line_line(a.p_0, junction.p, b.p_1))
                {
                    if (!insertRollOrCorner(a, b, inserts, insertCount))
                        reportCompError(CE_UNRESOLVED_GAP);
                    return;
                }
                if (extendTo(a, b, junction.p))
                    return;
            }
        }

        // Concave: extend to FIP if available
        if (!convex && junction.type == JT_EXTEND_TO_INTERSECTION)
        {
            if (extendTo(a, b, junction.p))
                return;
        }


        // Line-line specific: one-sided trim fallback
        if (!options.globalTrimCrossing){
            Vec2 carrierPts[2]{};
            int cnt = intersectCarrier(a, b, carrierPts);
            for (int i = 0; i < cnt; ++i)
            {
                float ta = line_t(a, carrierPts[i]);
                float tb = line_t(b, carrierPts[i]);
                bool onA = (ta >= -TOL && ta <= 1.0f + TOL);
                bool onB = (tb >= -TOL && tb <= 1.0f + TOL);
                if (onA != onB)
                {
                    trimTo(a, b, carrierPts[i]);
                    return;
                }
            }
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

        Junction junction;
        float gap = len(b.p_0 - a.p_1);
        bool convex = is_convex(a, b);
        bool allowExtend = gap < gapTol;
        solveJunction(a, b, junction);

        if (junction.type == JT_TRIM_TO_INTERSECTION)
        {
            if (trimTo(a, b, junction.p))
            {
                update_vectors(a);
                update_vectors(b);
                return;
            }
        }

        // Extend: convex near-gap or concave
        if (junction.type == JT_EXTEND_TO_INTERSECTION && (!convex || allowExtend))
        {
            if (extendTo(a, b, junction.p))
            {
                update_vectors(a);
                update_vectors(b);
                return;
            }
        }

        if (insertRollOrCorner(a, b, inserts, insertCount))
        {
            return;
        }
        else
        {
            // if we are going to do a global trim then we can ignore
            if (!options.globalTrimCrossing)
                reportCompError(CE_UNRESOLVED_GAP);
        }
    }

    void handleArcLine(Move2D &a, Move2D &b, Move2D inserts[3], int &insertCount)
    {
        // trivial check for chained elements
        // chained arc/line implies tangent if created from offsetting tangent adjacent elements.
        if (is_near(a.p_1, b.p_0))
        {
            b.p_0 = a.p_1; // snap together to avoid numerical issues later.
            return;        // already connected, no need to roll or trim.
        }

        float gap = len(b.p_0 - a.p_1);
        Junction junction;
        bool anyRapid = hasRapidMove(a, b);
        bool convex = is_convex(a, b);
        bool allowExtend = anyRapid || (gap < gapTol);
        solveJunction(a, b, junction);

        if (junction.type == JT_TRIM_TO_INTERSECTION)
        {
            if (trimTo(a, b, junction.p))
            {
                update_vectors(a);
                update_vectors(b);
                return;
            }
        }

        // Extend: convex near-gap/rapid or concave
        if (junction.type == JT_EXTEND_TO_INTERSECTION && (!convex || allowExtend))
        {
            if (extendTo(a, b, junction.p))
            {
                update_vectors(a);
                update_vectors(b);
                return;
            }
        }

        if (!anyRapid && convex)
        {
            if (!insertRollOrCorner(a, b, inserts, insertCount))
                reportCompError(CE_UNRESOLVED_GAP);
            return;
        }

        // if we are going to do a global trim then we can ignore
        if (!options.globalTrimCrossing)
            reportCompError(CE_UNRESOLVED_GAP);

        inserts[insertCount++] = makeBevel(a, b);
    }

    // Returns 0..2 TIPs that lie on BOTH finite elements
    static int commonTIP_any(const Move2D &A, const Move2D &B, Vec2 &tip1, Vec2 &tip2)
    {
        tip1 = {0, 0};
        tip2 = {0, 0};

        // ARC-ARC
        if (A.type == MOT_ARC && B.type == MOT_ARC)
        {
            // early-out for chained or concentric arcs (important)
            if (is_near(A.p_1, B.p_0) || is_near(A.center, B.center))
                return 0;
        }

        Vec2 pts[2]{};
        int count = finiteIntersectionPoints(A, B, pts);
        if (count >= 1)
            tip1 = pts[0];
        if (count >= 2)
            tip2 = pts[1];
        return count;
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

    static void refresh_aabb(const Move2D *moves, AABB2 *bounds, int idx)
    {
        if (!bounds || idx < 0)
            return;

        if (isMotionValid(moves[idx]))
        {
            bounds[idx] = aabb_of(moves[idx]);
            return;
        }

        bounds[idx].minx = 0.0f;
        bounds[idx].miny = 0.0f;
        bounds[idx].maxx = 0.0f;
        bounds[idx].maxy = 0.0f;
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
        Move2D &src = moves[srcIdx];

        if (!isMotionValid(src))
            return best;

        refresh_aabb(moves, bounds, srcIdx);

        int j = startTargetIdx + 1; // start looking from the element after the immediate neighbor
        for (int r = 0; r <= maxLookahead && j < maxIdx && j < numMoves; ++r, ++j)
        {
            Move2D &target = moves[j];
            if (!isMotionValid(target))
                continue;

            refresh_aabb(moves, bounds, j);

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
            float length = distFromStart_along(src, t1);
            if (n == 2)
            {
                float d2 = distFromStart_along(src, t2);
                if (d2 < length)
                {
                    length = d2;
                    pick = t2;
                }
            }

            if (length <= best.dist)
            {
                best.hit = true;
                best.j = j;
                best.tip = pick;
                best.dist = length;
            }
        }
        return best;
    }

public:
    // return false if failed to trim (which can only happen if a comp in move is crossing,
    bool trimCrossingElements(Move2D *moves, AABB2 *bounds, int &srcIdx, int maxIdx, int lookahead)
    {
        int firstSteadyIdx = first_steady_move(moves, maxIdx);
        int lastSteadyIdx = last_steady_move(moves, maxIdx, firstSteadyIdx);
        int hitTargetIdx = -1;
        // if comp in move is adjacent to flipped arc.
        if (firstSteadyIdx > -1)
        {
            if (moves[firstSteadyIdx].type == MOT_ARC && moves[firstSteadyIdx].radius <= 0)
            {
                reportCompError(CE_ARC_LT_TOOL_RAD);
                return false;
            }
        }

        // if comp out move is adjacent to flipped arc.
        if (lastSteadyIdx > -1)
        {
            if (moves[lastSteadyIdx].type == MOT_ARC && moves[lastSteadyIdx].radius <= 0)
            {
                reportCompError(CE_ARC_LT_TOOL_RAD);
                return false;
            }
        }

        while (srcIdx < maxIdx)
        {
            while (srcIdx < maxIdx && !isMotionValid(moves[srcIdx])) //|| moves[srcIdx].compMode == CM_IN
                srcIdx++;

            if (srcIdx >= maxIdx)
                break;

            int targetIdx = srcIdx + 1;
            while (targetIdx < maxIdx && !isMotionValid(moves[targetIdx]))
                targetIdx++; // skip invalid targets
            if (targetIdx >= maxIdx)
                break; // if we have no valid targets ahead, we are done.

            CrossingHit crossing = lookAheadForCrossing(moves, bounds, maxIdx, srcIdx, targetIdx, maxIdx, lookahead, firstSteadyIdx, lastSteadyIdx);
            if (!crossing.hit)
            {
                srcIdx++;
                continue;
            }
            hitTargetIdx = crossing.j;

            // if we are here we have a crossing.
            if (moves[srcIdx].compMode == CM_IN)
            {
                if (hitTargetIdx - srcIdx < 3)
                {
                    // comp in should not cross within the first 3 moves.
                    reportCompError(CE_COMP_IN_CROSSING);
                    return false;
                }
                srcIdx++;
                continue; // skip trimming for comp in move.
            }

            if (moves[hitTargetIdx].compMode == CM_OUT)
            {
                // comp out should never cross.
                if(hitTargetIdx - srcIdx < 3){
                    reportCompError(CE_COMP_OUT_CROSSING);
                    return false;
                }
                srcIdx++;
                continue; // skip trimming for comp in move.
            }

            // only run this if we have a non-lead-in-out crossing and it is not a head-bites-tail.
            if(srcIdx == firstSteadyIdx && hitTargetIdx == lastSteadyIdx)
                continue; // skip trimming for this special case to avoid breaking the closed loop seam.
            
            trimTo(moves[srcIdx], moves[hitTargetIdx], crossing.tip);
            CutterComp2D::invalidateRange(moves, srcIdx, hitTargetIdx);
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
