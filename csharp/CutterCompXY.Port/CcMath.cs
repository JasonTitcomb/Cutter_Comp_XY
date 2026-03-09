namespace CutterCompXY.Port;

public static class CcConst
{
    public const float TOL = 0.0001f;
    public const float ARC_TOL_IN = 0.0005f;
    public const float GAP_TOL_IN = 0.001f;
    public const float INPUT_ARC_TOL = 0.001f;
    public const float EPS = 1e-7f;
    public const float PARALLEL_TOL = 1e-3f;
    public const float BEVEL_VEC_TOL = 1.0e-1f;
    public const float PI = 3.14159265358979323846f;
    public const float TWO_PI = 6.2831853071795864769f;
    public const float MAX_SWEEP_DEG = 359.9f;
    public const float MIN_ARC_LEN = 0.001f;
    public const float IN_TO_MM = 25.4f;
}

public struct Vec2
{
    public float x;
    public float y;

    public Vec2(float xIn, float yIn)
    {
        x = xIn;
        y = yIn;
    }

    public static Vec2 operator +(Vec2 a, Vec2 b) => new Vec2(a.x + b.x, a.y + b.y);
    public static Vec2 operator -(Vec2 a, Vec2 b) => new Vec2(a.x - b.x, a.y - b.y);
    public static Vec2 operator *(Vec2 a, float s) => new Vec2(a.x * s, a.y * s);
}

public enum CompMode : byte
{
    CM_NONE = 0,
    CM_IN = 1,
    CM_STEADY = 2,
    CM_OUT = 3
}

public enum CornerType : byte
{
    CORNER_ROLL = 0,
    CORNER_CHAMFER = 1
}

public enum MotionType : byte
{
    MOT_EMPTY = 0,
    MOT_RAPID = 1,
    MOT_LINE = 2,
    MOT_ARC = 3
}

public enum ArcDir : byte
{
    ARC_CW = 0,
    ARC_CCW = 1
}

public enum IntersectType : byte
{
    IT_NONE = 0,
    IT_TANGENT = 1,
    IT_INTERSECT = 2
}

public enum CompSide : sbyte
{
    COMP_OFF = 0,
    COMP_LEFT = 1,
    COMP_RIGHT = -1
}

public enum CompError : byte
{
    CE_NONE = 0,
    CE_ARC_RADIUS_MISMATCH,
    CE_INVALID_MOVE,
    CE_COMP_MOVE_TOO_SHORT,
    CE_FLIPPED_ARC,
    CE_COMP_IN_CROSSING,
    CE_COMP_OUT_CROSSING,
    CE_UNRESOLVED_GAP
}

public enum Units : byte
{
    UNITS_MM = 0,
    UNITS_INCH = 1
}

public delegate void CompErrorCB(CompError err, uint seqNum);

public struct AABB2
{
    public float minx;
    public float miny;
    public float maxx;
    public float maxy;
}

public struct Move2D
{
    public Vec2 p_0;
    public Vec2 p_1;
    public Vec2 src_1;
    public Vec2 center;
    public Vec2 startDir;
    public Vec2 endDir;
    public AABB2 bounds;
    public float radius;
    public float feed;
    public float z_0;
    public float z_1;
    public uint seqNum;
    public MotionType type;
    public ArcDir arcDir;
    public CompMode compMode;
    public bool hasXY;
    public bool hasZ;
    public bool valid;

    public Move2D()
    {
        p_0 = new Vec2(0, 0);
        p_1 = new Vec2(0, 0);
        src_1 = new Vec2(0, 0);
        center = new Vec2(0, 0);
        startDir = new Vec2(0, 0);
        endDir = new Vec2(0, 0);
        bounds = new AABB2();
        radius = 0.0f;
        feed = 0.0f;
        z_0 = 0.0f;
        z_1 = 0.0f;
        seqNum = 0;
        type = MotionType.MOT_EMPTY;
        arcDir = ArcDir.ARC_CW;
        compMode = CompMode.CM_NONE;
        hasXY = false;
        hasZ = false;
        valid = true;
    }
}

public struct ArcAngles
{
    public float a0;
    public float a1;
    public ArcDir dir;
}

public static class CcMath
{
    public static CompErrorCB ErrorCallback;

    public static void ReportCompError(CompError err, uint seqNum)
    {
        ErrorCallback?.Invoke(err, seqNum);
    }

    public static float Clamp(float x, float lo, float hi)
    {
        if (x < lo)
            return lo;
        if (x > hi)
            return hi;
        return x;
    }

    public static Vec2 V2(float x, float y) => new Vec2(x, y);

    public static float Dot(in Vec2 a, in Vec2 b) => a.x * b.x + a.y * b.y;
    public static float Cross(in Vec2 a, in Vec2 b) => a.x * b.y - a.y * b.x;
    public static float Len(in Vec2 v) => MathF.Sqrt(Dot(v, v));

    public static Vec2 Normalize(in Vec2 v)
    {
        float l = Len(v);
        if (l < CcConst.TOL)
            return new Vec2(0, 0);
        return new Vec2(v.x / l, v.y / l);
    }

    public static Vec2 LeftNormal(in Vec2 v) => new Vec2(-v.y, v.x);
    public static Vec2 RightNormal(in Vec2 v) => new Vec2(v.y, -v.x);

    public static void UpdateVectors(ref Move2D m)
    {
        if (m.type == MotionType.MOT_LINE)
        {
            Vec2 d = m.p_1 - m.p_0;
            Vec2 u = Normalize(d);
            m.startDir = u;
            m.endDir = u;
            return;
        }

        if (m.type == MotionType.MOT_ARC)
        {
            Vec2 rs = Normalize(m.p_0 - m.center);
            Vec2 re = Normalize(m.p_1 - m.center);
            if (m.arcDir == ArcDir.ARC_CCW)
            {
                m.startDir = LeftNormal(rs);
                m.endDir = LeftNormal(re);
            }
            else
            {
                m.startDir = RightNormal(rs);
                m.endDir = RightNormal(re);
            }
            return;
        }

        m.startDir = new Vec2(0, 0);
        m.endDir = new Vec2(0, 0);
    }

    public static bool IsRadiusConsistent(in Move2D m)
    {
        float r0 = Len(m.p_0 - m.center);
        float r1 = Len(m.p_1 - m.center);
        bool ok = MathF.Abs(r0 - r1) <= CcConst.INPUT_ARC_TOL;
        if (!ok)
            ReportCompError(CompError.CE_ARC_RADIUS_MISMATCH, m.seqNum);
        return ok;
    }

    public static int GetWindingDir(in Vec2 a, in Vec2 b)
    {
        float z = Cross(a, b);
        if (z > CcConst.TOL)
            return 1;
        if (z < -CcConst.TOL)
            return -1;
        return 0;
    }

    public static float AngleNorm(float a)
    {
        while (a < 0)
            a += CcConst.TWO_PI;
        while (a >= CcConst.TWO_PI)
            a -= CcConst.TWO_PI;
        return a;
    }

    public static float Wrap2Pi(float a)
    {
        a %= CcConst.TWO_PI;
        if (a < 0)
            a += CcConst.TWO_PI;
        return a;
    }

    public static float ArcSweepDeg(in Move2D m)
    {
        float a0 = Wrap2Pi(MathF.Atan2(m.p_0.y - m.center.y, m.p_0.x - m.center.x));
        float a1 = Wrap2Pi(MathF.Atan2(m.p_1.y - m.center.y, m.p_1.x - m.center.x));

        if (m.arcDir == ArcDir.ARC_CCW)
        {
            float sw = a1 - a0;
            if (sw < 0)
                sw += CcConst.TWO_PI;
            return sw * (180.0f / CcConst.PI);
        }

        float cw = a0 - a1;
        if (cw < 0)
            cw += CcConst.TWO_PI;
        return cw * (180.0f / CcConst.PI);
    }

    public static float SweepCCW(float a0, float a1)
    {
        a0 = AngleNorm(a0);
        a1 = AngleNorm(a1);
        float d = a1 - a0;
        if (d < 0)
            d += CcConst.TWO_PI;
        return d;
    }

    public static float SweepCW(float a0, float a1) => SweepCCW(a1, a0);

    public static bool AngleOnArcNorm(float a0n, float a1n, float apn, ArcDir dir)
    {
        if (dir == ArcDir.ARC_CCW)
        {
            return (apn >= a0n - CcConst.EPS && apn <= a1n + CcConst.EPS) ||
                   (a0n > a1n && (apn >= a0n - CcConst.EPS || apn <= a1n + CcConst.EPS));
        }

        return (apn >= a1n - CcConst.EPS && apn <= a0n + CcConst.EPS) ||
               (a1n > a0n && (apn >= a1n - CcConst.EPS || apn <= a0n + CcConst.EPS));
    }

    public static AABB2 AabbOf(in Move2D m)
    {
        AABB2 b = new AABB2();
        b.minx = MathF.Min(m.p_0.x, m.p_1.x);
        b.maxx = MathF.Max(m.p_0.x, m.p_1.x);
        b.miny = MathF.Min(m.p_0.y, m.p_1.y);
        b.maxy = MathF.Max(m.p_0.y, m.p_1.y);

        if (m.type == MotionType.MOT_ARC && MathF.Abs(m.radius) > CcConst.TOL)
        {
            float a0n = AngleNorm(MathF.Atan2(m.p_0.y - m.center.y, m.p_0.x - m.center.x));
            float a1n = AngleNorm(MathF.Atan2(m.p_1.y - m.center.y, m.p_1.x - m.center.x));

            if (AngleOnArcNorm(a0n, a1n, CcConst.PI, m.arcDir))
                b.minx = m.center.x - m.radius;
            if (AngleOnArcNorm(a0n, a1n, 0.0f, m.arcDir))
                b.maxx = m.center.x + m.radius;
            if (AngleOnArcNorm(a0n, a1n, CcConst.PI * 0.5f, m.arcDir))
                b.maxy = m.center.y + m.radius;
            if (AngleOnArcNorm(a0n, a1n, CcConst.PI * 1.5f, m.arcDir))
                b.miny = m.center.y - m.radius;
        }

        return b;
    }

    public static void InitAllAabb(Move2D[] moves, int start, int count)
    {
        for (int i = start; i < count; ++i)
        {
            if (moves[i].type != MotionType.MOT_EMPTY && moves[i].valid)
            {
                moves[i].bounds = AabbOf(moves[i]);
            }
        }
    }

    public static bool AabbIntersects(in AABB2 a, in AABB2 b)
    {
        return !(a.maxx < b.minx || a.minx > b.maxx || a.maxy < b.miny || a.miny > b.maxy);
    }

    public static float LineT(in Move2D m, in Vec2 p)
    {
        Vec2 d = m.p_1 - m.p_0;
        float l2 = Dot(d, d);
        if (l2 < 1e-12f)
            return 0.0f;
        return Dot(p - m.p_0, d) / l2;
    }

    public static float DistFromStartAlong(in Move2D m, in Vec2 p)
    {
        if (m.type == MotionType.MOT_LINE)
        {
            float t = LineT(m, p);
            t = Clamp(t, 0.0f, 1.0f);
            return Len(m.p_1 - m.p_0) * t;
        }

        if (m.type == MotionType.MOT_ARC)
        {
            float a0 = MathF.Atan2(m.p_0.y - m.center.y, m.p_0.x - m.center.x);
            float ap = MathF.Atan2(p.y - m.center.y, p.x - m.center.x);
            float sw = (m.arcDir == ArcDir.ARC_CCW) ? SweepCCW(a0, ap) : SweepCW(a0, ap);
            return MathF.Abs(m.radius) * sw;
        }

        return 0.0f;
    }

    public static bool Validate(ref Move2D m)
    {
        if (m.type == MotionType.MOT_LINE)
        {
            m.valid = Len(m.p_1 - m.p_0) >= CcConst.TOL;
        }
        else if (m.type == MotionType.MOT_ARC)
        {
            float d = DistFromStartAlong(m, m.p_1);
            bool radiusOk = IsRadiusConsistent(m);
            float sw = ArcSweepDeg(m);
            bool sweepOk = !(sw > CcConst.MAX_SWEEP_DEG || sw < CcConst.MIN_ARC_LEN);
            m.valid = d >= CcConst.TOL && radiusOk && sweepOk;
        }

        if (!m.valid)
            ReportCompError(CompError.CE_INVALID_MOVE, m.seqNum);
        return m.valid;
    }

    public static void InvalidateRange(Move2D[] moves, int i, int j)
    {
        for (int k = i + 1; k < j; ++k)
            moves[k].valid = false;
    }

    public static bool IsNear(in Vec2 a, in Vec2 b)
    {
        Vec2 d = a - b;
        return Dot(d, d) <= CcConst.TOL * CcConst.TOL;
    }

    public static bool IsMotionValid(in Move2D m) => m.valid && m.type != MotionType.MOT_EMPTY;

    public static bool IsColinearWith(in Move2D a, in Move2D b)
    {
        if (a.type != b.type)
            return false;

        if (a.type == MotionType.MOT_LINE)
        {
            Vec2 da = a.startDir;
            Vec2 db = b.startDir;
            if (Len(da) < CcConst.TOL || Len(db) < CcConst.TOL)
                return false;
            float cr = MathF.Abs(Cross(da, db));
            return cr < CcConst.TOL;
        }

        if (a.type == MotionType.MOT_ARC)
        {
            if (!IsNear(a.center, b.center) || MathF.Abs(a.radius - b.radius) > CcConst.TOL)
                return false;
            return true;
        }

        return false;
    }

    public static int NextValidIndex(Move2D[] moves, int count, int i)
    {
        for (int k = i + 1; k < count; ++k)
            if (IsMotionValid(moves[k]))
                return k;
        return -1;
    }

    public static int PrevValidIndex(Move2D[] moves, int i)
    {
        for (int k = i - 1; k >= 0; --k)
            if (IsMotionValid(moves[k]))
                return k;
        return -1;
    }

    public static int FirstValidIndex(Move2D[] moves, int count)
    {
        for (int i = 0; i < count; ++i)
            if (IsMotionValid(moves[i]))
                return i;
        return -1;
    }

    public static int FirstCompMove(Move2D[] moves, int count)
    {
        for (int i = 0; i < count; ++i)
        {
            if (moves[i].compMode == CompMode.CM_IN)
                return NextValidIndex(moves, count, i);
        }
        return -1;
    }

    public static int LastCompMove(Move2D[] moves, int count, int startAt)
    {
        for (int i = startAt; i < count; ++i)
        {
            if (moves[i].compMode == CompMode.CM_OUT)
                return PrevValidIndex(moves, i);
        }
        return -1;
    }

    public static bool AngleOnSweepCCW(float a0, float a1, float ap)
    {
        a0 = AngleNorm(a0);
        a1 = AngleNorm(a1);
        ap = AngleNorm(ap);
        if (a0 <= a1)
            return (ap + CcConst.EPS >= a0) && (ap <= a1 + CcConst.EPS);
        return (ap >= a0 - CcConst.EPS) || (ap <= a1 + CcConst.EPS);
    }

    public static bool AngleOnSweepCW(float a0, float a1, float ap) => AngleOnSweepCCW(a1, a0, ap);

    public static bool PointOnSegment(in Vec2 a, in Vec2 b, in Vec2 p)
    {
        Vec2 ab = b - a;
        float lab2 = Dot(ab, ab);
        if (lab2 < CcConst.TOL)
            return Len(p - a) < CcConst.TOL;

        float t = Dot(p - a, ab) / lab2;
        if (t < -CcConst.TOL || t > 1.0f + CcConst.TOL)
            return false;

        float d = MathF.Abs(Cross(p - a, ab)) / MathF.Sqrt(lab2);
        return d < CcConst.TOL;
    }

    public static bool PointOnArc(in Move2D a, in Vec2 p)
    {
        float rp = Len(p - a.center);
        if (MathF.Abs(rp - a.radius) > CcConst.TOL)
            return false;

        float a0 = MathF.Atan2(a.p_0.y - a.center.y, a.p_0.x - a.center.x);
        float a1 = MathF.Atan2(a.p_1.y - a.center.y, a.p_1.x - a.center.x);
        float ap = MathF.Atan2(p.y - a.center.y, p.x - a.center.x);
        return a.arcDir == ArcDir.ARC_CCW ? AngleOnSweepCCW(a0, a1, ap) : AngleOnSweepCW(a0, a1, ap);
    }

    public static ArcAngles PrecomputeArcAngles(in Move2D m)
    {
        ArcAngles aa;
        aa.a0 = MathF.Atan2(m.p_0.y - m.center.y, m.p_0.x - m.center.x);
        aa.a1 = MathF.Atan2(m.p_1.y - m.center.y, m.p_1.x - m.center.x);
        aa.dir = m.arcDir;
        return aa;
    }

    public static bool PointOnArcCached(in Move2D a, in Vec2 p, in ArcAngles aa)
    {
        float rp = Len(p - a.center);
        if (MathF.Abs(rp - a.radius) > CcConst.TOL)
            return false;

        float ap = MathF.Atan2(p.y - a.center.y, p.x - a.center.x);
        return aa.dir == ArcDir.ARC_CCW ? AngleOnSweepCCW(aa.a0, aa.a1, ap) : AngleOnSweepCW(aa.a0, aa.a1, ap);
    }

    public static IntersectType IntersectLineLine(in Move2D ln1, in Move2D ln2, out Vec2 ip, out bool tip)
    {
        Vec2 p = ln1.p_0;
        Vec2 r = ln1.p_1 - ln1.p_0;
        Vec2 q = ln2.p_0;
        Vec2 s = ln2.p_1 - ln2.p_0;

        float lr = Len(r);
        float ls = Len(s);
        if (lr < CcConst.TOL || ls < CcConst.TOL)
        {
            ip = new Vec2(0, 0);
            tip = false;
            return IntersectType.IT_NONE;
        }

        float den = Cross(r, s);
        float denTol = CcConst.PARALLEL_TOL * lr * ls;
        if (MathF.Abs(den) <= denTol)
        {
            ip = new Vec2(0, 0);
            tip = false;
            return IntersectType.IT_NONE;
        }

        float t = Cross(q - p, s) / den;
        float u = Cross(q - p, r) / den;
        ip = p + r * t;

        tip = (t >= -CcConst.TOL && t <= 1.0f + CcConst.TOL && u >= -CcConst.TOL && u <= 1.0f + CcConst.TOL);
        return IntersectType.IT_INTERSECT;
    }

    public static IntersectType IntersectCircleCircle(in Move2D a1, in Move2D a2, out Vec2 p1, out Vec2 p2, out int count)
    {
        Vec2 c0 = a1.center;
        Vec2 c1 = a2.center;
        float r0 = a1.radius;
        float r1 = a2.radius;
        Vec2 d = c1 - c0;
        float distc = Len(d);
        count = 0;
        p1 = new Vec2(0, 0);
        p2 = new Vec2(0, 0);

        if (distc < CcConst.TOL)
            return IntersectType.IT_NONE;
        if (distc > r0 + r1 + CcConst.TOL)
            return IntersectType.IT_NONE;
        if (distc < MathF.Abs(r0 - r1) - CcConst.TOL)
            return IntersectType.IT_NONE;

        float a = (r0 * r0 - r1 * r1 + distc * distc) / (2.0f * distc);
        float h2 = r0 * r0 - a * a;
        Vec2 u = d * (1.0f / distc);
        Vec2 mid = c0 + u * a;

        if (MathF.Abs(h2) < CcConst.TOL)
        {
            p1 = mid;
            count = 1;
            return IntersectType.IT_TANGENT;
        }

        float h = MathF.Sqrt(MathF.Max(0.0f, h2));
        Vec2 perp = LeftNormal(u);
        p1 = mid + perp * h;
        p2 = mid - perp * h;
        count = 2;
        return IntersectType.IT_INTERSECT;
    }

    public static IntersectType IntersectLineCircle(in Vec2 l1, in Vec2 a1, in Vec2 ctr, float r, out Vec2 p1, out Vec2 p2, out int count)
    {
        Vec2 d = a1 - l1;
        float dd = Dot(d, d);
        count = 0;
        p1 = new Vec2(0, 0);
        p2 = new Vec2(0, 0);
        if (dd < 1e-20f)
            return IntersectType.IT_NONE;

        Vec2 f = l1 - ctr;
        float t0 = -Dot(f, d) / dd;
        Vec2 q = l1 + d * t0;
        Vec2 qc = q - ctr;
        float dist2 = Dot(qc, qc);

        float r2 = r * r;
        const float eps = 1e-5f;
        const float eps2 = eps * eps;

        float h2 = r2 - dist2;
        if (h2 < -eps2)
            return IntersectType.IT_NONE;

        if (MathF.Abs(h2) <= eps2)
        {
            p1 = q;
            count = 1;
            return IntersectType.IT_TANGENT;
        }

        float h = MathF.Sqrt(h2);
        float invLen = 1.0f / MathF.Sqrt(dd);
        Vec2 u = d * invLen;

        p1 = q - u * h;
        p2 = q + u * h;
        count = 2;
        return IntersectType.IT_INTERSECT;
    }

    public static Vec2 PickClosest(in Vec2 @ref, in Vec2 a, in Vec2 b)
    {
        Vec2 da = a - @ref;
        Vec2 db = b - @ref;
        return Dot(da, da) <= Dot(db, db) ? a : b;
    }

    public static bool IsNearDir(Vec2 a, Vec2 b)
    {
        a = Normalize(a);
        b = Normalize(b);
        return Dot(a, b) > 0.9995f;
    }
}
