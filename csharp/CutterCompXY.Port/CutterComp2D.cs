using System;
using System.Runtime.InteropServices;
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
    public float toolRadius;
    public CornerType cornerTreatment;
    public bool globalTrimCrossing;
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
    public bool havePrevMove2D = false;
    public Move2D prevOff = new Move2D();
    private Units units = Units.UNITS_MM;
    public float gapTol = CcConst.GAP_TOL_IN;
    public bool hasCompError = false;
    public uint lastSeqNum = 0;

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

    public void SetUnits(Units u)
    {
        units = u;
        if (units == Units.UNITS_INCH)
        {
            gapTol = CcConst.GAP_TOL_IN;
        }
        else
        {
            gapTol = CcConst.GAP_TOL_IN * CcConst.IN_TO_MM;
        }
    }

    public void SetToolRadius(float r)
    {
        toolR = r < 0 ? -r : r;
        toolSign = (sbyte)(r < 0 ? -1 : 1);
    }

    public void SetComp(CompSide s)
    {
        comp_state = s;
        ResetState();
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

    public bool Process()
    {
        if (hasCompError)
            return false;
        if (comp_state == CompSide.COMP_OFF || toolR < CcConst.TOL)
        {
            while (inCount > 0)
            {
                if (!OutHasSpace(1))
                    return false;
                Move2D m = PopIn();
                PushOut(m);
            }
            return true;
        }

        while (inCount > 0)
        {
            if (!OutHasSpace(4))
                return false;

            Move2D raw = PopIn();
            if (raw.seqNum != 0)
                lastSeqNum = raw.seqNum;
            if (raw.type == MotionType.MOT_EMPTY)
                continue;

            CcMath.UpdateVectors(ref raw);

            Move2D curOff;
            OffsetMove(ref raw, out curOff);
            CcMath.Validate(ref curOff);

            if (!raw.hasXY && raw.hasZ)
            {
                if (havePrevMove2D)
                {
                    curOff.p_0 = prevOff.p_0;
                    curOff.p_1 = prevOff.p_0;
                }
                if (!OutHasSpace(1))
                    return false;
                PushOut(curOff);
                continue;
            }

            if (raw.compMode == CompMode.CM_IN || raw.compMode == CompMode.CM_OUT)
            {
                float moveLen = CcMath.Len(raw.p_1 - raw.p_0);
                if (moveLen <= toolR)
                {
                    ReportCompError(CompError.CE_COMP_MOVE_TOO_SHORT);
                    return false;
                }
            }

            if (!havePrevMove2D)
            {
                prevOff = curOff;
                havePrevMove2D = true;
                continue;
            }

            Move2D[] inserts = new Move2D[3];
            int insertCount = 0;

            if (prevOff.compMode == CompMode.CM_IN)
                prevOff.p_1 = curOff.p_0;

            if (curOff.compMode == CompMode.CM_OUT)
                curOff.p_0 = prevOff.p_1;

            ApplyLogic(ref prevOff, ref curOff, inserts, ref insertCount);

            if (prevOff.valid)
            {
                PushOut(prevOff);
                for (int i = 0; i < insertCount; i++)
                    PushOut(inserts[i]);
            }

            prevOff = curOff;
        }
        return true;
    }
    // ...existing code...

    public void Flush()
    {
        Process();
        if (havePrevMove2D && OutHasSpace(1))
        {
            PushOut(prevOff);
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

    private bool OutHasSpace(int n) => (outCount + n) <= OUT_CAP;

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
            return;

        output_buffer[(outHead + outCount) % OUT_CAP] = m;
        outCount++;
    }

    private void ResetState() => havePrevMove2D = false;

    private Move2D MakeBevel(in Move2D a, in Move2D b)
    {
        Move2D m = new Move2D();
        m.hasXY = true;
        m.type = MotionType.MOT_LINE;
        m.feed = a.feed > 0 ? a.feed : b.feed;
        m.p_0 = a.p_1;
        m.p_1 = b.p_0;
        CcMath.UpdateVectors(ref m);
        return m;
    }

    private bool ConvexFromWinding(int cw)
    {
        if (cw == 0)
            return false;

        bool isLeft = comp_state == CompSide.COMP_LEFT;
        if (toolSign < 0)
            isLeft = !isLeft;

        if (isLeft)
            return !(cw > 0);
        return cw > 0;
    }

    private bool IsConvex(in Move2D a, in Move2D b)
    {
        int cw = CcMath.GetWindingDir(a.endDir, b.startDir);
        return ConvexFromWinding(cw);
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
        if (r0 < CcConst.TOL)
            r0 = CcMath.Len(src.p_0 - src.center);
        if (r0 < CcConst.TOL)
        {
            dst = new Move2D();
            return false;
        }

        dst = src;
        CcMath.Validate(ref dst);
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
        CcMath.Validate(ref a);
        CcMath.Validate(ref b);
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

            CcMath.Validate(ref a);
            CcMath.Validate(ref b);
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
        roll.center = a.src_1;

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

        bool useLeft = comp_state == CompSide.COMP_LEFT;
        if (toolSign < 0)
            useLeft = !useLeft;

        ArcDir preferredDir = useLeft ? ArcDir.ARC_CW : ArcDir.ARC_CCW;
        roll.arcDir = preferredDir;

        const float turnEps = 1.0e-4f;
        if (r0 >= CcConst.TOL && r1 >= CcConst.TOL)
        {
            float turnSign = CcMath.Cross(v0, v1) / (r0 * r1);
            if (turnSign > turnEps && preferredDir != ArcDir.ARC_CCW)
                roll.arcDir = ArcDir.ARC_CCW;
            else if (turnSign < -turnEps && preferredDir != ArcDir.ARC_CW)
                roll.arcDir = ArcDir.ARC_CW;
        }

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

        if (a.type == MotionType.MOT_LINE)
            l1 = a;
        else
        {
            extA = MakeArcExtensionLineOnly(a, true);
            if (!extA.valid)
                return 0;
            l1 = extA;
            haveExtA = true;
        }

        if (b.type == MotionType.MOT_LINE)
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
        cap.type = MotionType.MOT_LINE;
        cap.compMode = CompMode.CM_STEADY;
        cap.feed = a.feed > 0 ? a.feed : b.feed;
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
        if (!CcMath.Validate(ref cap))
            return 0;

        if (a.type == MotionType.MOT_LINE)
        {
            a.p_1 = ipForL1;
            CcMath.UpdateVectors(ref a);
            if (!CcMath.Validate(ref a))
                return 0;
        }

        if (b.type == MotionType.MOT_LINE)
        {
            b.p_0 = ipForL2;
            CcMath.UpdateVectors(ref b);
            if (!CcMath.Validate(ref b))
                return 0;
        }

        if (haveExtA)
        {
            extA.p_1 = ipForL1;
            CcMath.UpdateVectors(ref extA);
            if (!CcMath.Validate(ref extA))
                return 0;
            output[outCountLocal++] = extA;
        }

        output[outCountLocal++] = cap;

        if (haveExtB)
        {
            extB.p_0 = ipForL2;
            extB.p_1 = b.p_0;
            CcMath.UpdateVectors(ref extB);
            if (!CcMath.Validate(ref extB))
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
        extLnOut.compMode = arc.compMode;
        extLnOut.feed = arc.feed;
        extLnOut.p_0 = anchor;
        extLnOut.p_1 = anchor + dir * extent;
        extLnOut.startDir = dir;
        extLnOut.endDir = dir;

        CcMath.UpdateVectors(ref extLnOut);
        CcMath.Validate(ref extLnOut);
        return extLnOut;
    }

    private bool InsertRollOrCorner(ref Move2D a, ref Move2D b, Move2D[] inserts, ref int insertCount)
    {
        int startCount = insertCount;

        float gap = CcMath.Len(b.p_0 - a.p_1);
        bool nearlyConnected = gap < gapTol;
        if (nearlyConnected)
        {
            Move2D bevel = MakeBevel(a, b);
            if (!CcMath.Validate(ref bevel))
                return false;
            inserts[insertCount++] = bevel;
            return true;
        }

        if (cornerTreatment == CornerType.CORNER_ROLL)
        {
            Move2D roll = MakeRollArc(a, b);
            if (insertCount >= 3)
                return false;
            if (!CcMath.Validate(ref roll))
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

        bool comping =
            a.compMode == CompMode.CM_IN || a.compMode == CompMode.CM_OUT ||
            b.compMode == CompMode.CM_IN || b.compMode == CompMode.CM_OUT;

        if (a.type == MotionType.MOT_LINE && b.type == MotionType.MOT_LINE)
            HandleLineLine(ref a, ref b, comping, inserts, ref insertCount);
        else if (a.type == MotionType.MOT_ARC && b.type == MotionType.MOT_ARC)
            HandleArcArc(ref a, ref b, inserts, ref insertCount);
        else if ((a.type == MotionType.MOT_ARC && b.type == MotionType.MOT_LINE) || (a.type == MotionType.MOT_LINE && b.type == MotionType.MOT_ARC))
            HandleArcLine(ref a, ref b, inserts, ref insertCount);
    }

    private void HandleLineLine(ref Move2D a, ref Move2D b, bool comping, Move2D[] inserts, ref int insertCount)
    {
        IntersectType it = CcMath.IntersectLineLine(a, b, out Vec2 ip, out bool tip);
        if (it == IntersectType.IT_NONE)
            return;

        if (tip)
        {
            TrimToTIP(ref a, ref b, ip);
            if (!a.valid)
                b.p_0 = a.p_1;
            if (!b.valid)
                a.p_1 = b.p_0;
            return;
        }

        float gap = CcMath.Len(b.p_0 - a.p_1);
        bool nearlyConnected = gap < gapTol;

        if (nearlyConnected || comping)
        {
            if (ExtendToFIP(ref a, ref b, ip))
                return;
        }

        if (a.compMode == CompMode.CM_IN)
        {
            a.p_1 = b.p_0;
            return;
        }

        if (b.compMode == CompMode.CM_OUT)
        {
            b.p_0 = a.p_1;
            return;
        }

        if (IsConvex(a, b))
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

        IntersectType it = CcMath.IntersectCircleCircle(a, b, out Vec2 p1, out Vec2 p2, out int tipCt);
        if (it == IntersectType.IT_NONE || it == IntersectType.IT_TANGENT)
        {
            if (!InsertRollOrCorner(ref a, ref b, inserts, ref insertCount))
                CcMath.ReportCompError(CompError.CE_UNRESOLVED_GAP, a.seqNum);
            return;
        }

        ArcAngles aa = CcMath.PrecomputeArcAngles(a);
        ArcAngles ba = CcMath.PrecomputeArcAngles(b);

        bool tip1 = (tipCt >= 1) && CcMath.PointOnArcCached(a, p1, aa) && CcMath.PointOnArcCached(b, p1, ba);
        bool tip2 = (tipCt == 2) && CcMath.PointOnArcCached(a, p2, aa) && CcMath.PointOnArcCached(b, p2, ba);

        if (tip1 || tip2)
        {
            Vec2 tip = tip1 ? p1 : p2;
            if (tip1 && tip2)
                tip = CcMath.PickClosest(a.p_1, p1, p2);
            if (TrimToTIP(ref a, ref b, tip))
                return;
        }

       // no tip but small gap: try extending to FIP (false intersection point)
        float gap = CcMath.Len(b.p_0 - a.p_1);
        bool nearlyConnected = gap < gapTol;
        if (nearlyConnected)
        {
            Vec2 tip = CcMath.PickClosest(a.p_1, ip1, ip2);
            if (CcMath.ExtendToFIP(ref a, ref b, tip))
            {
                return;
            }
        }

        if (!InsertRollOrCorner(ref a, ref b, inserts, ref insertCount))
            CcMath.ReportCompError(CompError.CE_UNRESOLVED_GAP, a.seqNum);
    }

    private void HandleArcLine(ref Move2D a, ref Move2D b, Move2D[] inserts, ref int insertCount)
    {
        bool arcFirst = a.type == MotionType.MOT_ARC;

        Move2D arc = arcFirst ? a : b;
        Move2D lin = arcFirst ? b : a;

        if (CcMath.IsNear(a.p_1, b.p_0))
        {
            b.p_0 = a.p_1;
            return;
        }

        IntersectType it = CcMath.IntersectLineCircle(lin.p_0, lin.p_1, arc.center, arc.radius, out Vec2 p1, out Vec2 p2, out int count);
        if (it == IntersectType.IT_NONE)
        {
            if (!InsertRollOrCorner(ref a, ref b, inserts, ref insertCount))
                CcMath.ReportCompError(CompError.CE_UNRESOLVED_GAP, a.seqNum);
            return;
        }

        ArcAngles arca = CcMath.PrecomputeArcAngles(arc);
        bool tip1 = count >= 1 && CcMath.PointOnSegment(lin.p_0, lin.p_1, p1) && CcMath.PointOnArcCached(arc, p1, arca);
        bool tip2 = count == 2 && CcMath.PointOnSegment(lin.p_0, lin.p_1, p2) && CcMath.PointOnArcCached(arc, p2, arca);

        if (tip1 || tip2)
        {
            Vec2 tip = tip1 ? p1 : p2;
            if (tip1 && tip2)
                tip = CcMath.PickClosest(a.p_1, p1, p2);
            if (TrimToTIP(ref a, ref b, tip))
                return;
        }

        // no tip but small gap: try extending to FIP (false intersection point)
        float gap = len(b.p_0 - a.p_1);
        bool nearlyConnected = gap < gapTol;
        if (nearlyConnected)
        {
            Vec2 tip = CcMath.PickClosest(a.p_1, ip1, ip2);
            if (CcMath.ExtendToFIP(ref a, ref b, tip))
            {
                return;
            }
        }

        if (!InsertRollOrCorner(ref a, ref b, inserts, ref insertCount))
            CcMath.ReportCompError(CompError.CE_UNRESOLVED_GAP, a.seqNum);
    }

    private static int CommonTIPAny(in Move2D A, in Move2D B, out Vec2 tip1, out Vec2 tip2)
    {
        tip1 = new Vec2(0, 0);
        tip2 = new Vec2(0, 0);

        if (A.type == MotionType.MOT_LINE && B.type == MotionType.MOT_LINE)
        {
            IntersectType it = CcMath.IntersectLineLine(A, B, out Vec2 ip, out bool tip);
            if (it != IntersectType.IT_NONE && tip)
            {
                tip1 = ip;
                return 1;
            }
            return 0;
        }

        if ((A.type == MotionType.MOT_LINE && B.type == MotionType.MOT_ARC) || (A.type == MotionType.MOT_ARC && B.type == MotionType.MOT_LINE))
        {
            Move2D L = A.type == MotionType.MOT_LINE ? A : B;
            Move2D C = A.type == MotionType.MOT_ARC ? A : B;

            IntersectType it = CcMath.IntersectLineCircle(L.p_0, L.p_1, C.center, C.radius, out Vec2 p1, out Vec2 p2, out int count);
            if (it == IntersectType.IT_NONE)
                return 0;

            ArcAngles ca = CcMath.PrecomputeArcAngles(C);

            int n = 0;
            if (count >= 1 && CcMath.PointOnSegment(L.p_0, L.p_1, p1) && CcMath.PointOnArcCached(C, p1, ca))
            {
                tip1 = p1;
                n++;
            }

            if (count == 2 && CcMath.PointOnSegment(L.p_0, L.p_1, p2) && CcMath.PointOnArcCached(C, p2, ca))
            {
                if (n == 0)
                    tip1 = p2;
                else
                    tip2 = p2;
                n++;
            }
            return n;
        }

        if (A.type == MotionType.MOT_ARC && B.type == MotionType.MOT_ARC)
        {
            if (CcMath.IsNear(A.p_1, B.p_0) || CcMath.IsNear(A.center, B.center))
                return 0;

            IntersectType it = CcMath.IntersectCircleCircle(A, B, out Vec2 p1, out Vec2 p2, out int count);
            if (it == IntersectType.IT_NONE)
                return 0;

            ArcAngles aa = CcMath.PrecomputeArcAngles(A);
            ArcAngles ba = CcMath.PrecomputeArcAngles(B);

            int n = 0;
            if (count >= 1 && CcMath.PointOnArcCached(A, p1, aa) && CcMath.PointOnArcCached(B, p1, ba))
            {
                tip1 = p1;
                n++;
            }

            if (count == 2 && CcMath.PointOnArcCached(A, p2, aa) && CcMath.PointOnArcCached(B, p2, ba))
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

    private static CrossingHit LookAheadForCrossing(Move2D[] moves, int numMoves, int srcIdx, int startTargetIdx, int maxIdx, int maxLookahead, int firstCutIdx, int lastCutIdx)
    {
        CrossingHit best = new CrossingHit
        {
            hit = false,
            j = -1,
            tip = new Vec2(0, 0),
            dist = 1e30f
        };

        if (!moves[srcIdx].valid)
            return best;

        Move2D src = moves[srcIdx];

        int j = startTargetIdx + 1;
        for (int r = 0; r <= maxLookahead && j < maxIdx && j < numMoves; r++, j++)
        {
            Move2D target = moves[j];
            if (!target.valid)
                continue;

            if (srcIdx == firstCutIdx && j == lastCutIdx)
                break;

            if (j < srcIdx + 2)
                continue;

            if (!CcMath.AabbIntersects(src.bounds, target.bounds))
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
        CcMath.InitAllAabb(moves, srcIdx, maxIdx);

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
                    CcMath.ReportCompError(CompError.CE_FLIPPED_ARC, moves[firstCutIdx].seqNum);
                    return true;
                }
            }
        }

        if (moves[srcIdx].compMode == CompMode.CM_OUT)
        {
            if (lastCutIdx > -1)
            {
                if (moves[lastCutIdx].type == MotionType.MOT_ARC && moves[lastCutIdx].radius <= 0)
                {
                    CcMath.ReportCompError(CompError.CE_FLIPPED_ARC, moves[lastCutIdx].seqNum);
                    return true;
                }
            }
        }

        while (srcIdx < maxIdx)
        {
            while (srcIdx < maxIdx && (!moves[srcIdx].valid))
                srcIdx++;

            if (srcIdx >= maxIdx)
                break;

            int targetIdx = srcIdx + 1;
            while (targetIdx < maxIdx && !moves[targetIdx].valid)
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
