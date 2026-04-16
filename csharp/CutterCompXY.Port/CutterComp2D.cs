using System;
namespace CutterCompXY.Port;

// Native callback delegates matching C++ signatures
public delegate void CcOutputCB(string text, int len);
public delegate void CcErrorCB(string message, int err, uint seqNum);
public delegate void CcStartCompCB(int toolRegister, int diaRegister);

public struct CcMainOptions
{
    public struct CcMainCallbacks
    {
        public CcOutputCB output;
        public CcErrorCB error;
        public CcStartCompCB startComp;
    }

    public CcMainOptions()
    {
        toolRadius = 0.0f;
        cornerTreatment = CornerType.CORNER_ROLL;
        globalTrimCrossing = true;
        globalMerge = true;
        emitStatusComments = true;
        callbacks = new CcMainCallbacks();
    }

    public float toolRadius;
    public CornerType cornerTreatment;
    public bool globalTrimCrossing;
    public bool globalMerge;
    public bool emitStatusComments;
    public CcMainCallbacks callbacks;
}
public sealed class CutterComp2D
{
    public CornerType cornerTreatment = CornerType.CORNER_ROLL;
    public bool performTrim = true;
    public const int IN_CAP = 2;
    public const int OUT_CAP = 4;
    public readonly Move2D[] input_buffer = new Move2D[IN_CAP];
    public int inHead = 0, inCount = 0;
    public readonly Move2D[] output_buffer = new Move2D[OUT_CAP];
    public int outHead = 0, outCount = 0;
    public float toolR = 0.0f;
    public sbyte toolSign = 0;
    public CompSide comp_state = CompSide.COMP_OFF;
    public CompMode compMode = CompMode.CM_NONE;
    public bool havePrevMove2D = false;
    public Move2D prevOff = new Move2D();
    public bool havePendingZMove = false;
    public Move2D pendingZMove = new Move2D();
    private Units units = Units.UNITS_MM;
    public float gapTol = CcConst.GAP_TOL_IN;
    public bool hasCompError = false;
    public uint lastSeqNum = 0;
    private CcMainOptions options = new CcMainOptions();

    public struct CrossingHit
    {
        public bool hit;
        public int j;
        public Vec2 tip;
        public float dist;
    }

    public void SetCornerTreatment(CornerType ct) => cornerTreatment = ct;
    public void SetPerformTrim(bool en) => performTrim = en;
    public void SetErrorCallback(CompErrorCB cb) => CcMath.ErrorCallback = cb;

    public void SetOptions(CcMainOptions opts)
    {
        options = opts;
        SetToolRadius(opts.toolRadius);
        ResetState();
        cornerTreatment = opts.cornerTreatment;
        performTrim = opts.globalTrimCrossing;
        SetErrorCallback(null);
    }

    public void SetUnits(Units u)
    {
        units = u;
        if (units == Units.UNITS_INCH)
        {
            CcMath.arcTol = CcConst.ARC_TOL_IN;
            gapTol = CcConst.GAP_TOL_IN;
        }
        else
        {
            CcMath.arcTol = CcConst.ARC_TOL_IN * CcConst.IN_TO_MM;
            gapTol = CcConst.GAP_TOL_IN * CcConst.IN_TO_MM;
        }
    }

    private static int ZMoveDirection(float z0, float z1)
    {
        if (MathF.Abs(z1 - z0) <= CcConst.EPS)
            return 0;
        return z1 > z0 ? 1 : -1;
    }

    private static bool ShouldReplacePendingZTarget(in Move2D pending, in Move2D candidate)
    {
        int pendingDir = ZMoveDirection(pending.z_0, pending.z_1);
        int candidateDir = ZMoveDirection(candidate.z_0, candidate.z_1);

        if (pendingDir != 0 && pendingDir == candidateDir)
            return MathF.Abs(candidate.z_1) > MathF.Abs(pending.z_1);

        return true;
    }

    public void SetToolRadius(float r)
    {
        toolR = r < 0 ? -r : r;
        toolSign = (sbyte)(r < 0 ? -1 : 1);
    }

    private CompSide EffectiveCompSide()
    {
        if (toolSign >= 0)
            return comp_state;

        if (comp_state == CompSide.COMP_LEFT)
            return CompSide.COMP_RIGHT;

        if (comp_state == CompSide.COMP_RIGHT)
            return CompSide.COMP_LEFT;

        return comp_state;
    }

    private bool CompUsesLeft() => EffectiveCompSide() == CompSide.COMP_LEFT;

    private static bool IsLineLike(in Move2D m) => CcMath.IsLineLike(m);

    private static bool HasRapidMove(in Move2D a, in Move2D b) =>
        a.type == MotionType.MOT_RAPID || b.type == MotionType.MOT_RAPID;

    private bool Validate(ref Move2D m)
    {
        if (IsLineLike(m))
        {
            float lineLen = CcMath.Len(m.p_1 - m.p_0);
            if (lineLen < CcConst.TOL)
            {
                m.hasXY = false;
                m.valid = false;
                return false;
            }

            m.valid = true;
            return true;
        }

        if (m.type != MotionType.MOT_ARC)
            return m.valid;

        bool degenerate = MathF.Abs(m.radius) < CcConst.TOL;
        float sweep = CcMath.ArcSweepDeg(m);
        bool sweepOk = sweep <= CcConst.MAX_SWEEP_DEG && sweep >= CcConst.MIN_ARC_LEN;

        if (degenerate || !sweepOk)
        {
            m.valid = false;
            if (options.globalTrimCrossing)
                return m.valid;

            ReportCompError(CompError.CE_ARC_LT_TOOL_RAD);
            return false;
        }

        if (!options.globalTrimCrossing)
        {
            CompSide side = EffectiveCompSide();
            bool innerArc =
                (side == CompSide.COMP_LEFT && m.arcDir == ArcDir.ARC_CCW) ||
                (side == CompSide.COMP_RIGHT && m.arcDir == ArcDir.ARC_CW);

            if (innerArc && MathF.Abs(m.radius) < toolR)
            {
                m.valid = false;
                ReportCompError(CompError.CE_ARC_LT_TOOL_RAD);
                return false;
            }
        }

        bool consistent = CcMath.IsRadiusConsistent(m);
        if (!consistent)
            ReportCompError(CompError.CE_ARC_RADIUS_MISMATCH);
        if (!sweepOk)
            ReportCompError(CompError.CE_INVALID_MOVE);

        return m.valid;
    }
    public void SetComp(CompSide s)
    {
        CompSide prevSide = comp_state;
        comp_state = s;

        if (prevSide == CompSide.COMP_OFF && s != CompSide.COMP_OFF)
        {
            compMode = CompMode.CM_IN;
            return;
        }

        if (prevSide != CompSide.COMP_OFF && s == CompSide.COMP_OFF)
        {
            compMode = CompMode.CM_OUT;
            return;
        }

        if (s == CompSide.COMP_OFF)
        {
            compMode = CompMode.CM_NONE;
            return;
        }

        if (prevSide != s)
        {
            compMode = CompMode.CM_IN;
            return;
        }

        if (compMode == CompMode.CM_NONE || compMode == CompMode.CM_OUT)
        {
            compMode = CompMode.CM_IN;
            return;
        }

        if (compMode != CompMode.CM_IN)
            compMode = CompMode.CM_STEADY;
    }

    public void ReportCompError(CompError err)
    {
        hasCompError = true;
        CcMath.ReportCompError(err, lastSeqNum);
    }

    public bool PushIn(in Move2D m)
    {
        if (inCount >= IN_CAP)
            return false;
        input_buffer[(inHead + inCount) % IN_CAP] = m;
        inCount++;
        return true;
    }

    private bool EmitPendingZMoveAt(in Move2D anchor)
    {
        if (!havePendingZMove)
            return true;

        Move2D zMove = pendingZMove;
        zMove.p_0 = anchor.p_1;
        zMove.p_1 = anchor.p_1;
        zMove.z_0 = anchor.z_1;
        zMove.hasXY = false;
        zMove.hasZ = MathF.Abs(zMove.z_1 - zMove.z_0) > CcConst.EPS;

        if (zMove.hasZ)
            PushOut(zMove);

        havePendingZMove = false;
        return !hasCompError;
    }

    public bool Process()
    {
        while (inCount > 0)
        {
            if (!OutHasSpace(4))
                return false;

            Move2D curOff = PopIn();
            curOff.compMode = compMode;
            if (curOff.seqNum != 0)
                lastSeqNum = curOff.seqNum;
            if (curOff.type == MotionType.MOT_EMPTY)
                continue;

            CcMath.UpdateVectors(ref curOff);

            curOff.hasZ = MathF.Abs(curOff.z_1 - curOff.z_0) > CcConst.EPS;

            if (curOff.hasZ && !curOff.hasXY)
            {
                if (havePrevMove2D)
                {
                    if (havePendingZMove)
                    {
                        if (ShouldReplacePendingZTarget(pendingZMove, curOff))
                        {
                            pendingZMove.z_1 = curOff.z_1;
                            pendingZMove.seqNum = curOff.seqNum;
                            pendingZMove.feed = curOff.feed;
                            pendingZMove.type = curOff.type;
                            pendingZMove.hasZ = MathF.Abs(pendingZMove.z_1 - pendingZMove.z_0) > CcConst.EPS;
                        }
                    }
                    else
                    {
                        pendingZMove = curOff;
                        pendingZMove.p_0 = prevOff.p_1;
                        pendingZMove.p_1 = prevOff.p_1;
                        pendingZMove.z_0 = prevOff.z_1;
                        pendingZMove.hasXY = false;
                        pendingZMove.hasZ = MathF.Abs(pendingZMove.z_1 - pendingZMove.z_0) > CcConst.EPS;
                        havePendingZMove = pendingZMove.hasZ;
                    }
                }
                else
                {
                    if (!OutHasSpace(1))
                        return false;
                    PushOut(curOff);
                }
                continue;
            }

            if (!Validate(ref curOff))
            {
                if (!options.globalTrimCrossing)
                    return false;

                if (havePrevMove2D && prevOff.valid)
                {
                    PushOut(prevOff);
                    if (!EmitPendingZMoveAt(prevOff))
                        return false;
                }

                prevOff = curOff;
                havePrevMove2D = true;

                if (curOff.compMode == CompMode.CM_IN)
                    compMode = CompMode.CM_STEADY;
                else if (curOff.compMode == CompMode.CM_OUT)
                    compMode = CompMode.CM_NONE;
                continue;
            }
            if (!OffsetMove(curOff, out curOff))
                return false;

            if (!havePrevMove2D)
            {
                prevOff = curOff;
                havePrevMove2D = true;
                if (curOff.compMode == CompMode.CM_IN)
                    compMode = CompMode.CM_STEADY;
                else if (curOff.compMode == CompMode.CM_OUT)
                    compMode = CompMode.CM_NONE;
                continue;
            }

            Move2D[] inserts = new Move2D[3];
            int insertCount = 0;

             if (prevOff.compMode == CompMode.CM_IN)
            {
                float originalLen = CcMath.Len(prevOff.p_1 - prevOff.p_0);
                if (originalLen <= toolR + CcConst.TOL)
                {
                    ReportCompError(CompError.CE_COMP_MOVE_TOO_SHORT);
                    return false;
                }

                // Modify the previous move so that the end is the start of the current move.
                prevOff.p_1 = curOff.p_0;

                float finalLen = CcMath.Len(prevOff.p_1 - prevOff.p_0);
                if (finalLen <= CcConst.TOL)
                {
                    ReportCompError(CompError.CE_COMP_MOVE_TOO_SHORT);
                    return false;
                }

                CcMath.UpdateVectors(ref prevOff);
            }

            if (curOff.compMode == CompMode.CM_OUT)
            {
                float originalLen = CcMath.Len(curOff.p_1 - curOff.p_0);
                if (originalLen <= toolR + CcConst.TOL)
                {
                    ReportCompError(CompError.CE_COMP_MOVE_TOO_SHORT);
                    return false;
                }

                // Modify the G40 move so that the start is the end of the previous move.
                curOff.p_0 = prevOff.p_1;

                float finalLen = CcMath.Len(curOff.p_1 - curOff.p_0);
                if (finalLen <= CcConst.TOL)
                {
                    ReportCompError(CompError.CE_COMP_MOVE_TOO_SHORT);
                    return false;
                }

                CcMath.UpdateVectors(ref curOff);
            }

            if (curOff.compMode == CompMode.CM_STEADY)
                ApplyLogic(ref prevOff, ref curOff, inserts, ref insertCount);

            Validate(ref curOff);

            if (prevOff.valid)
            {
                PushOut(prevOff);
                if (!EmitPendingZMoveAt(prevOff))
                    return false;
                for (int i = 0; i < insertCount; i++)
                    PushOut(inserts[i]);
            }

            if (curOff.compMode == CompMode.CM_IN)
                compMode = CompMode.CM_STEADY;
            else if (curOff.compMode == CompMode.CM_OUT)
                compMode = CompMode.CM_NONE;

            prevOff = curOff;
        }
        return true;
    }

    public void Flush()
    {
        Process();
        if (havePrevMove2D && OutHasSpace(havePendingZMove ? 2 : 1))
        {
            if (prevOff.valid && prevOff.type != MotionType.MOT_EMPTY)
            {
                PushOut(prevOff);
                _ = EmitPendingZMoveAt(prevOff);
            }
            havePrevMove2D = false;
        }
    }

    public bool PopOut(out Move2D m)
    {
        if (outCount == 0)
        {
            m = new Move2D();
            return false;
        }

        m = output_buffer[outHead];
        outHead = (outHead + 1) % OUT_CAP;
        outCount--;
        return true;
    }

    private bool OutHasSpace(int n)
    {
        bool ok = (outCount + n) <= OUT_CAP;
        if (!ok)
            ReportCompError(CompError.CE_OUTPUT_BUFFER_OVERFLOW);

        return ok;
    }

    private Move2D PopIn()
    {
        Move2D m = input_buffer[inHead];
        inHead = (inHead + 1) % IN_CAP;
        inCount--;
        return m;
    }

    private void PushOut(in Move2D m)
    {
        if (outCount >= OUT_CAP)
        {
            ReportCompError(CompError.CE_OUTPUT_BUFFER_OVERFLOW);
            return;
        }

        output_buffer[(outHead + outCount) % OUT_CAP] = m;
        outCount++;
    }

    private void ResetState()
    {
        havePrevMove2D = false;
        havePendingZMove = false;
    }

    private Move2D MakeBevel(in Move2D a, in Move2D b)
    {
        Move2D m = new Move2D();
        m.seqNum = a.seqNum;
        m.hasXY = true;
        m.type = MotionType.MOT_LINE;
        m.feed = a.feed > 0 ? a.feed : b.feed;
        m.p_0 = a.p_1;
        m.p_1 = b.p_0;
        m.z_0 = b.z_0;
        m.z_1 = b.z_0;
        m.hasZ = false;
        CcMath.UpdateVectors(ref m);
        return m;
    }

    private bool ConvexFromWinding(int cw)
    {
        if (cw == 0)
            return false;

        bool isLeft = CompUsesLeft();

        if (isLeft)
            return !(cw > 0);
        return cw > 0;
    }

    private bool IsConvex(in Move2D a, in Move2D b)
    {
        int cw = CcMath.GetWindingDir(a.endDir, b.startDir);
        return ConvexFromWinding(cw);
    }

    private static bool PointOnFiniteElem(in Move2D m, in Vec2 p)
    {
        if (IsLineLike(m))
            return CcMath.PointOnSegment(m.p_0, m.p_1, p);

        if (m.type == MotionType.MOT_ARC)
        {
            ArcAngles aa = CcMath.PrecomputeArcAngles(m);
            return CcMath.PointOnArcCached(m, p, aa);
        }

        return false;
    }

    private static int IntersectCarrier(in Move2D a, in Move2D b, out Vec2 p1, out Vec2 p2)
    {
        p1 = new Vec2(0, 0);
        p2 = new Vec2(0, 0);

        if (IsLineLike(a) && IsLineLike(b))
        {
            IntersectType it = CcMath.IntersectLineLine(a, b, out p1, out _);
            return (it == IntersectType.IT_NONE) ? 0 : 1;
        }

        if (a.type == MotionType.MOT_ARC && b.type == MotionType.MOT_ARC)
        {
            if (CcMath.IsNear(a.center, b.center))
                return 0;

            IntersectType it = CcMath.IntersectCircleCircle(a, b, out p1, out p2, out int count);
            if (it == IntersectType.IT_NONE)
                return 0;
            return count;
        }

        Move2D line = IsLineLike(a) ? a : b;
        Move2D arc = (a.type == MotionType.MOT_ARC) ? a : b;
        IntersectType it2 = CcMath.IntersectLineCircle(line.p_0, line.p_1, arc.center, arc.radius, out p1, out p2, out int count2);
        if (it2 == IntersectType.IT_NONE)
            return 0;
        return count2;
    }

    private static int FiniteIntersectionPoints(in Move2D a, in Move2D b, out Vec2 p1, out Vec2 p2)
    {
        p1 = new Vec2(0, 0);
        p2 = new Vec2(0, 0);

        int carrierCount = IntersectCarrier(a, b, out Vec2 c0, out Vec2 c1);
        Vec2[] carrierPts = { c0, c1 };
        int finiteCount = 0;

        for (int i = 0; i < carrierCount; ++i)
        {
            Vec2 p = carrierPts[i];
            if (!PointOnFiniteElem(a, p) || !PointOnFiniteElem(b, p))
                continue;

            if (finiteCount > 0 && CcMath.IsNear(p1, p))
                continue;

            if (finiteCount == 0)
                p1 = p;
            else
                p2 = p;

            finiteCount++;
        }

        return finiteCount;
    }

    private bool IsForwardExtensionPoint(in Move2D a, in Move2D b, in Vec2 p)
    {
        float fipDir1 = CcMath.Dot(p - a.p_1, a.endDir);
        float fipDir2 = CcMath.Dot(p - b.p_0, b.startDir);
        return fipDir1 > 0 && fipDir2 < 0;
    }

    private static bool IsUsableDegenerateArc(in Move2D m)
    {
        return m.type == MotionType.MOT_ARC && MathF.Abs(m.radius) < CcConst.TOL && m.hasXY;
    }

    private bool TryGetBestTrimPoint(in Move2D a, in Move2D b, out Vec2 bestPoint)
    {
        bestPoint = new Vec2(0, 0);
        int count = FiniteIntersectionPoints(a, b, out Vec2 p1, out Vec2 p2);
        if (count <= 0)
            return false;

        bestPoint = p1;
        float bestScore = CcMath.DistFromStartAlong(a, p1) + CcMath.DistFromStartAlong(b, p1);

        if (count > 1)
        {
            float score = CcMath.DistFromStartAlong(a, p2) + CcMath.DistFromStartAlong(b, p2);
            if (score < bestScore)
                bestPoint = p2;
        }

        return true;
    }

    private bool TryGetBestExtendPoint(in Move2D a, in Move2D b, out Vec2 bestPoint)
    {
        bestPoint = new Vec2(0, 0);
        int count = IntersectCarrier(a, b, out Vec2 p1, out Vec2 p2);
        Vec2[] carrierPts = { p1, p2 };
        bool found = false;
        float bestScore = 0.0f;

        for (int i = 0; i < count; i++)
        {
            Vec2 p = carrierPts[i];
            if (!IsForwardExtensionPoint(a, b, p))
                continue;

            float score = CcMath.Len(a.p_1 - p) + CcMath.Len(b.p_0 - p);
            if (!found || score < bestScore)
            {
                bestScore = score;
                bestPoint = p;
                found = true;
            }
        }

        return found;
    }

    private bool OffsetMove(in Move2D src, out Move2D dst)
    {
        if (src.type == MotionType.MOT_LINE || src.type == MotionType.MOT_RAPID)
            return OffsetLine(src, out dst);
        if (src.type == MotionType.MOT_ARC)
            return OffsetArc(src, out dst);

        dst = new Move2D();
        return false;
    }

    private bool OffsetLine(in Move2D src, out Move2D dst)
    {
        dst = src;
        dst.src_1 = src.p_1;

        Vec2 v = src.p_1 - src.p_0;
        float l = CcMath.Len(v);
        if (l < CcConst.TOL)
        {
            dst.valid = false;
            return false;
        }

        Vec2 u = v * (1.0f / l);

        bool useLeft = comp_state == CompSide.COMP_LEFT;
        if (toolSign < 0)
            useLeft = !useLeft;

        Vec2 n = useLeft ? CcMath.LeftNormal(u) : CcMath.RightNormal(u);
        Vec2 off = n * toolR;

        if (src.compMode == CompMode.CM_IN || src.compMode == CompMode.CM_OUT)
        {
            off.x = 0.0f;
            off.y = 0.0f;
        }

        dst.type = src.type;
        dst.p_0 = src.p_0 + off;
        dst.p_1 = src.p_1 + off;
        return true;
    }

    private bool OffsetArc(in Move2D src, out Move2D dst)
    {
        float r0 = src.radius;
        if (MathF.Abs(r0) < CcConst.TOL)
            r0 = CcMath.Len(src.p_0 - src.center);
        if (r0 < CcConst.TOL)
        {
            dst = new Move2D();
            return false;
        }

        dst = src;
        Validate(ref dst);
        dst.src_1 = src.p_1;

        float dr = toolR;
        bool ccw = src.arcDir == ArcDir.ARC_CCW;
        bool left = comp_state == CompSide.COMP_LEFT;
        if (toolSign < 0)
            left = !left;

        float r1;
        if (ccw)
            r1 = r0 + (left ? -dr : +dr);
        else
            r1 = r0 + (left ? +dr : -dr);

        Vec2 v0 = src.p_0 - src.center;
        Vec2 v1 = src.p_1 - src.center;
        float lv0 = CcMath.Len(v0);
        float lv1 = CcMath.Len(v1);

        dst.type = MotionType.MOT_ARC;
        dst.center = src.center;
        dst.radius = r1;
        dst.p_0 = src.center + v0 * (r1 / lv0);
        dst.p_1 = src.center + v1 * (r1 / lv1);
        return true;
    }

    private bool TrimToTIP(ref Move2D a, ref Move2D b, in Vec2 tip)
    {
        a.p_1 = tip;
        b.p_0 = tip;
        CcMath.UpdateVectors(ref a);
        CcMath.UpdateVectors(ref b);
        Validate(ref a);
        Validate(ref b);
        return a.valid && b.valid;
    }

    private bool ExtendToFIP(ref Move2D a, ref Move2D b, in Vec2 fip)
    {
        float fipDir1 = CcMath.Dot(fip - a.p_1, a.endDir);
        float fipDir2 = CcMath.Dot(fip - b.p_0, b.startDir);
        if (fipDir1 > 0 && fipDir2 < 0)
        {
            a.p_1 = fip;
            b.p_0 = fip;

            CcMath.UpdateVectors(ref a);
            CcMath.UpdateVectors(ref b);

            Validate(ref a);
            Validate(ref b);
        }

        return a.valid && b.valid;
    }

    private Move2D MakeRollArc(in Move2D a, in Move2D b)
    {
        Move2D roll = new Move2D();
        roll.seqNum = (a.seqNum * 10) + 5;
        roll.type = MotionType.MOT_ARC;
        roll.compMode = CompMode.CM_STEADY;
        roll.feed = a.feed > 0 ? a.feed : b.feed;
        roll.p_0 = a.p_1;
        roll.p_1 = b.p_0;
        roll.z_0 = b.z_0;
        roll.z_1 = b.z_0;
        roll.hasZ = false;

        bool useLeft = comp_state == CompSide.COMP_LEFT;
        if (toolSign < 0)
            useLeft = !useLeft;

        roll.center = CcMath.RollCenter(a.p_1, a.endDir, useLeft, toolR);

        Vec2 v0 = roll.p_0 - roll.center;
        Vec2 v1 = roll.p_1 - roll.center;
        float r0 = CcMath.Len(v0);
        float r1 = CcMath.Len(v1);

        float r;
        if (r0 >= CcConst.TOL && r1 >= CcConst.TOL)
            r = 0.5f * (r0 + r1);
        else if (r0 >= CcConst.TOL)
            r = r0;
        else if (r1 >= CcConst.TOL)
            r = r1;
        else
            r = toolR;

        if (r0 >= CcConst.TOL)
            roll.p_0 = roll.center + v0 * (r / r0);
        if (r1 >= CcConst.TOL)
            roll.p_1 = roll.center + v1 * (r / r1);

        roll.radius = r;

        ArcDir preferredDir = useLeft ? ArcDir.ARC_CW : ArcDir.ARC_CCW;
        roll.arcDir = preferredDir;

        roll.valid = true;
        CcMath.UpdateVectors(ref roll);
        return roll;
    }

    private int MakeCornerTreatment(ref Move2D a, ref Move2D b, Move2D[] output)
    {
        int outCountLocal = 0;
        Move2D l1;
        Move2D l2;
        Move2D extA = new Move2D();
        Move2D extB = new Move2D();
        bool haveExtA = false;
        bool haveExtB = false;

        float turnSign0 = CcMath.Cross(a.endDir * -1.0f, b.startDir);
        if (MathF.Abs(turnSign0) <= CcConst.BEVEL_VEC_TOL)
        {
            output[outCountLocal++] = MakeBevel(a, b);
            return outCountLocal;
        }

        if (IsLineLike(a))
            l1 = a;
        else
        {
            extA = MakeArcExtensionLineOnly(a, true);
            if (!extA.valid)
                return 0;
            l1 = extA;
            haveExtA = true;
        }

        if (IsLineLike(b))
            l2 = b;
        else
        {
            extB = MakeArcExtensionLineOnly(b, false);
            if (!extB.valid)
                return 0;
            l2 = extB;
            haveExtB = true;
        }

        Vec2 partCorner = a.src_1;
        Vec2 vIn = CcMath.Normalize(l1.endDir * -1.0f);
        Vec2 vOut = CcMath.Normalize(l2.startDir);
        Vec2 bisector = CcMath.Normalize(vIn + vOut);
        if (CcMath.Len(bisector) < CcConst.TOL)
            return 0;

        Vec2 chamferDir = CcMath.Normalize(CcMath.LeftNormal(bisector));
        if (CcMath.Len(chamferDir) < CcConst.TOL)
            return 0;

        Vec2 offsetCap = partCorner + bisector * (-toolR);
        Move2D cap = new Move2D();
        cap.seqNum = a.seqNum;
        cap.type = MotionType.MOT_LINE;
        cap.compMode = CompMode.CM_STEADY;
        cap.feed = a.feed > 0 ? a.feed : b.feed;
        cap.z_0 = b.z_0;
        cap.z_1 = b.z_0;
        cap.hasZ = false;
        float halfLen = 0.5f * (toolR + 2.0f);
        cap.p_0 = offsetCap - chamferDir * halfLen;
        cap.p_1 = offsetCap + chamferDir * halfLen;
        CcMath.UpdateVectors(ref cap);

        Vec2 ipForL1 = new Vec2(0, 0);
        Vec2 ipForL2 = new Vec2(0, 0);

        IntersectType it = CcMath.IntersectLineLine(l1, cap, out ipForL1, out _);
        if (it == IntersectType.IT_NONE)
            return 0;

        it = CcMath.IntersectLineLine(l2, cap, out ipForL2, out _);
        if (it == IntersectType.IT_NONE)
            return 0;

        cap.p_0 = ipForL1;
        cap.p_1 = ipForL2;
        if (!Validate(ref cap))
            return 0;

        if (IsLineLike(a))
        {
            a.p_1 = ipForL1;
            CcMath.UpdateVectors(ref a);
            if (!Validate(ref a))
                return 0;
        }

        if (IsLineLike(b))
        {
            b.p_0 = ipForL2;
            CcMath.UpdateVectors(ref b);
            if (!Validate(ref b))
                return 0;
        }

        if (haveExtA)
        {
            extA.p_1 = ipForL1;
            extA.z_0 = b.z_0;
            extA.z_1 = b.z_0;
            extA.hasZ = false;
            CcMath.UpdateVectors(ref extA);
            if (!Validate(ref extA))
                return 0;
            output[outCountLocal++] = extA;
        }

        output[outCountLocal++] = cap;

        if (haveExtB)
        {
            extB.p_0 = ipForL2;
            extB.p_1 = b.p_0;
            extB.z_0 = b.z_0;
            extB.z_1 = b.z_0;
            extB.hasZ = false;
            CcMath.UpdateVectors(ref extB);
            if (!Validate(ref extB))
                return 0;
            output[outCountLocal++] = extB;
        }

        return outCountLocal;
    }

    private Move2D MakeArcExtensionLineOnly(in Move2D arc, bool fromEnd)
    {
        Move2D extLnOut = new Move2D();
        if (arc.type != MotionType.MOT_ARC)
            return extLnOut;

        float extent = toolR * 2.0f;
        Vec2 anchor = fromEnd ? arc.p_1 : arc.p_0;
        Vec2 dir = fromEnd ? arc.endDir : arc.startDir;

        if (CcMath.Len(dir) < CcConst.TOL)
            return extLnOut;

        extLnOut.type = MotionType.MOT_LINE;
        extLnOut.seqNum = arc.seqNum;
        extLnOut.compMode = arc.compMode;
        extLnOut.feed = arc.feed;
        extLnOut.z_0 = fromEnd ? arc.z_1 : arc.z_0;
        extLnOut.z_1 = extLnOut.z_0;
        extLnOut.p_0 = anchor;
        extLnOut.p_1 = anchor + dir * extent;
        extLnOut.startDir = dir;
        extLnOut.endDir = dir;

        CcMath.UpdateVectors(ref extLnOut);
        Validate(ref extLnOut);
        return extLnOut;
    }

    private bool InsertRollOrCorner(ref Move2D a, ref Move2D b, Move2D[] inserts, ref int insertCount)
    {
        int startCount = insertCount;

        float gap = CcMath.Len(b.p_0 - a.p_1);
        bool nearlyConnected = gap < gapTol;
        if (nearlyConnected)
            return true;

        if (cornerTreatment == CornerType.CORNER_ROLL)
        {
            Move2D roll = MakeRollArc(a, b);
            if (insertCount >= 3)
                return false;
            if (!Validate(ref roll))
                return false;

            roll.hasXY = true;
            inserts[insertCount++] = roll;
            return true;
        }

        Move2D[] cornerSegs = new Move2D[3];
        int cornerCount = MakeCornerTreatment(ref a, ref b, cornerSegs);

        for (int i = 0; i < cornerCount && insertCount < 3; i++)
        {
            cornerSegs[i].hasXY = true;
            inserts[insertCount++] = cornerSegs[i];
        }

        return insertCount > startCount;
    }

    private void ApplyLogic(ref Move2D a, ref Move2D b, Move2D[] inserts, ref int insertCount)
    {
        insertCount = 0;

        if (!a.hasXY || !b.hasXY)
            return;

        if (IsLineLike(a) && IsLineLike(b))
            HandleLineLine(ref a, ref b, inserts, ref insertCount);
        else if (a.type == MotionType.MOT_ARC && b.type == MotionType.MOT_ARC)
            HandleArcArc(ref a, ref b, inserts, ref insertCount);
        else if ((a.type == MotionType.MOT_ARC && IsLineLike(b)) || (IsLineLike(a) && b.type == MotionType.MOT_ARC))
            HandleArcLine(ref a, ref b, inserts, ref insertCount);
    }

    private void HandleLineLine(ref Move2D a, ref Move2D b, Move2D[] inserts, ref int insertCount)
    {
        bool comping =
            a.compMode == CompMode.CM_IN || a.compMode == CompMode.CM_OUT ||
            b.compMode == CompMode.CM_IN || b.compMode == CompMode.CM_OUT;
        float gap = CcMath.Len(b.p_0 - a.p_1);
        bool anyRapid = HasRapidMove(a, b);
        bool allowExtend = anyRapid || (gap < gapTol) || comping;

        if (a.compMode == CompMode.CM_IN)
        {
            a.p_1 = b.p_0;
            return;
        }

        if (gap <= gapTol)
        {
            b.p_0 = a.p_1;
            CcMath.UpdateVectors(ref b);
            Validate(ref b);
            return;
        }

        IntersectType it = CcMath.IntersectLineLine(a, b, out Vec2 ip, out _);
        if (it != IntersectType.IT_NONE)
        {
            float projA = CcMath.Dot(ip - a.p_1, a.endDir);
            float projB = CcMath.Dot(ip - b.p_0, b.startDir);

            if (projA <= gapTol && projB >= -gapTol)
            {
                TrimToTIP(ref a, ref b, ip);
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

            if (allowExtend && projA > 0.0f && projB < 0.0f)
            {
                if (ExtendToFIP(ref a, ref b, ip))
                    return;
            }
        }

        if (!anyRapid && IsConvex(a, b))
        {
            if (!InsertRollOrCorner(ref a, ref b, inserts, ref insertCount))
                CcMath.ReportCompError(CompError.CE_UNRESOLVED_GAP, a.seqNum);
            return;
        }

        inserts[insertCount++] = MakeBevel(a, b);
    }

    private void HandleArcArc(ref Move2D a, ref Move2D b, Move2D[] inserts, ref int insertCount)
    {
        if (CcMath.IsNear(a.p_1, b.p_0) || CcMath.IsNear(a.center, b.center))
            return;

        float gap = CcMath.Len(b.p_0 - a.p_1);

        if (IsUsableDegenerateArc(a) || IsUsableDegenerateArc(b))
            return;

        if (TryGetBestTrimPoint(a, b, out Vec2 junctionPoint))
        {
            if (TrimToTIP(ref a, ref b, junctionPoint))
                return;
        }

        if (gap < gapTol && TryGetBestExtendPoint(a, b, out junctionPoint))
        {
            if (ExtendToFIP(ref a, ref b, junctionPoint))
                return;
        }

        if (InsertRollOrCorner(ref a, ref b, inserts, ref insertCount))
        {
            return;
        }
        else
        {
            if (!performTrim)
                CcMath.ReportCompError(CompError.CE_UNRESOLVED_GAP, a.seqNum);
        }
    }

    private void HandleArcLine(ref Move2D a, ref Move2D b, Move2D[] inserts, ref int insertCount)
    {
        if (CcMath.IsNear(a.p_1, b.p_0))
        {
            b.p_0 = a.p_1;
            return;
        }

        float gap = CcMath.Len(b.p_0 - a.p_1);
        bool anyRapid = HasRapidMove(a, b);

        if (IsUsableDegenerateArc(a) || IsUsableDegenerateArc(b))
            return;

        if (TryGetBestTrimPoint(a, b, out Vec2 junctionPoint))
        {
            if (TrimToTIP(ref a, ref b, junctionPoint))
                return;
        }

        if ((anyRapid || (gap < gapTol)) && TryGetBestExtendPoint(a, b, out junctionPoint))
        {
            if (ExtendToFIP(ref a, ref b, junctionPoint))
                return;
        }

        if (!anyRapid && IsConvex(a, b))
        {
            if (!InsertRollOrCorner(ref a, ref b, inserts, ref insertCount))
                CcMath.ReportCompError(CompError.CE_UNRESOLVED_GAP, a.seqNum);
            return;
        }

        if (!performTrim)
            CcMath.ReportCompError(CompError.CE_UNRESOLVED_GAP, a.seqNum);

        inserts[insertCount++] = MakeBevel(a, b);
    }

    private static int CommonTIPAny(in Move2D A, in Move2D B, out Vec2 tip1, out Vec2 tip2)
    {
        tip1 = new Vec2(0, 0);
        tip2 = new Vec2(0, 0);

        if (A.type == MotionType.MOT_ARC && B.type == MotionType.MOT_ARC)
        {
            if (CcMath.IsNear(A.p_1, B.p_0) || CcMath.IsNear(A.center, B.center))
                return 0;
        }

        return FiniteIntersectionPoints(A, B, out tip1, out tip2);
    }

    private static CrossingHit LookAheadForCrossing(Move2D[] moves, int numMoves, int srcIdx, int startTargetIdx, int maxIdx, int maxLookahead, int firstCutIdx, int lastCutIdx)
    {
        CrossingHit best = new CrossingHit
        {
            hit = false,
            j = -1,
            tip = new Vec2(0, 0),
            dist = 1e30f
        };

        if (!CcMath.IsMotionValid(moves[srcIdx]))
            return best;

        CcMath.RefreshAabb(moves, srcIdx);
        Move2D src = moves[srcIdx];

        int j = startTargetIdx + 1;
        for (int r = 0; r <= maxLookahead && j < maxIdx && j < numMoves; r++, j++)
        {
            Move2D target = moves[j];
            if (!CcMath.IsMotionValid(target))
                continue;

            CcMath.RefreshAabb(moves, j);
            target = moves[j];

            if (srcIdx == firstCutIdx && j == lastCutIdx)
                break;

            if (j < srcIdx + 2)
                continue;

            if (!CcMath.AabbIntersects(src.bounds, target.bounds))
                continue;

            if (MathF.Abs(src.z_0 - target.z_0) > CcConst.TOL || MathF.Abs(src.z_0 - target.z_1) > CcConst.TOL)
                continue;

            int n = CommonTIPAny(src, target, out Vec2 t1, out Vec2 t2);
            if (n <= 0)
                continue;

            Vec2 pick = t1;
            float d = CcMath.DistFromStartAlong(src, t1);
            if (n == 2)
            {
                float d2 = CcMath.DistFromStartAlong(src, t2);
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

    public bool TestForCrossingElements(Move2D[] moves, ref int srcIdx, int maxIdx, int lookahead)
    {
        CrossingHit crossing = LookAheadForCrossing(moves, maxIdx, srcIdx, srcIdx + 1, maxIdx, lookahead, -1, -1);
        return crossing.hit;
    }

    public bool TrimCrossingElements(Move2D[] moves, ref int srcIdx, int maxIdx, int lookahead, out int hitTargetIdx)
    {
        hitTargetIdx = -1;

        int compInIdx = -1;
        int compOutIdx = -1;

        int firstCutIdx = CcMath.FirstCompMove(moves, maxIdx);
        int lastCutIdx = -1;
        if (firstCutIdx >= 0)
            lastCutIdx = CcMath.LastCompMove(moves, maxIdx, firstCutIdx);

        if (firstCutIdx >= 0)
            compInIdx = firstCutIdx - 1;

        if (lastCutIdx >= 0)
            compOutIdx = lastCutIdx + 1;

        if (moves[srcIdx].compMode == CompMode.CM_IN)
        {
            if (firstCutIdx > -1)
            {
                if (moves[firstCutIdx].type == MotionType.MOT_ARC && moves[firstCutIdx].radius <= 0)
                {
                    CcMath.ReportCompError(CompError.CE_ARC_LT_TOOL_RAD, moves[firstCutIdx].seqNum);
                    return false;
                }
            }
        }

        if (moves[srcIdx].compMode == CompMode.CM_OUT)
        {
            if (lastCutIdx > -1)
            {
                if (moves[lastCutIdx].type == MotionType.MOT_ARC && moves[lastCutIdx].radius <= 0)
                {
                    CcMath.ReportCompError(CompError.CE_ARC_LT_TOOL_RAD, moves[lastCutIdx].seqNum);
                    return false;
                }
            }
        }

        while (srcIdx < maxIdx)
        {
            while (srcIdx < maxIdx && !CcMath.IsMotionValid(moves[srcIdx]))
                srcIdx++;

            if (srcIdx >= maxIdx)
                break;

            int targetIdx = srcIdx + 1;
            while (targetIdx < maxIdx && !CcMath.IsMotionValid(moves[targetIdx]))
                targetIdx++;
            if (targetIdx >= maxIdx)
                break;

            CrossingHit crossing = LookAheadForCrossing(moves, maxIdx, srcIdx, targetIdx, maxIdx, lookahead, firstCutIdx, lastCutIdx);
            if (!crossing.hit)
            {
                srcIdx++;
                continue;
            }

            hitTargetIdx = crossing.j;

            if (moves[srcIdx].compMode == CompMode.CM_IN)
            {
                if (hitTargetIdx < lastCutIdx)
                {
                    CcMath.ReportCompError(CompError.CE_COMP_IN_CROSSING, moves[srcIdx].seqNum);
                    return false;
                }
                srcIdx++;
                continue;
            }

            if (moves[hitTargetIdx].compMode == CompMode.CM_OUT)
            {
                CcMath.ReportCompError(CompError.CE_COMP_OUT_CROSSING, moves[hitTargetIdx].seqNum);
                return false;
            }

            bool shouldTrim = compInIdx != -1 && compOutIdx != -1 && srcIdx == compInIdx && hitTargetIdx == compOutIdx;
            if (!shouldTrim)
            {
                Move2D a = moves[srcIdx];
                Move2D b = moves[hitTargetIdx];
                TrimToTIP(ref a, ref b, crossing.tip);
                moves[srcIdx] = a;
                moves[hitTargetIdx] = b;
                CcMath.InvalidateRange(moves, srcIdx, hitTargetIdx);
            }

            srcIdx = hitTargetIdx;
        }

        return true;
    }

    public int MergeAllColinear(Move2D[] moves, int count)
    {
        if (count < 2)
            return 0;

        int merges = 0;
        int ia = CcMath.FirstValidIndex(moves, count);
        if (ia < 0)
            return 0;

        while (true)
        {
            int ib = CcMath.NextValidIndex(moves, count, ia);
            if (ib < 0)
                break;

            Move2D a = moves[ia];
            Move2D b = moves[ib];

            if (CcMath.IsColinearWith(a, b))
            {
                a.p_1 = b.p_1;
                b.valid = false;
                moves[ia] = a;
                moves[ib] = b;
                merges++;
                continue;
            }

            ia = ib;
        }

        return merges;
    }
}
