using System.Globalization;

namespace CutterCompXY.Port;

public struct ScanLine
{
    public bool isMove;
    public bool hasN;
    public int N;
    public bool hasG;
    public int G;
    public bool hasX;
    public float X;
    public bool hasY;
    public float Y;
    public bool hasZ;
    public float Z;
    public bool hasI;
    public float I;
    public bool hasJ;
    public float J;
    public bool hasR;
    public float R;
    public bool hasF;
    public float F;
    public bool hasD;
    public int D;
    public bool hasT;
    public int T;
    public bool hasS;
    public float S;

    // Special modal toggles
    public bool sawG17;
    public bool sawG90;
    public bool sawG41;
    public bool sawG42;
    public bool sawG40;
    public bool sawG20;
    public bool sawG21;

    // Motion mode tracking
    public bool sawG0;
    public bool sawG1;
    public bool sawG2;
    public bool sawG3;
    public bool sawG91;
}

public struct ModalState
{
    public bool planeXY;
    public bool absXYZ;
    public bool absoluteMode;
    public bool inchMode;
    public int motionG;
    public CompSide comp;
    public CompMode compMode;
    public float feed;
    public int D_Register;
    public int T_Register;
    public int N_number;
    public float speed;
    public Vec2 pos;
    public float z;

    public ModalState()
    {
        planeXY = true;
        absXYZ = true;
        absoluteMode = true;
        inchMode = true;
        motionG = 0;
        comp = CompSide.COMP_OFF;
        compMode = CompMode.CM_NONE;
        feed = 0.0f;
        D_Register = 0;
        T_Register = 0;
        N_number = 0;
        speed = 0.0f;
        pos = new Vec2(0, 0);
        z = 0.0f;
    }
}

public static class SimpleScan
{
    private static bool IsSpace(char c) => c == ' ' || c == '\t' || c == '\r' || c == '\n';
    public static char Up(char c) => char.ToUpperInvariant(c);

    private static int SkipWs(string s, int idx)
    {
        while (idx < s.Length && IsSpace(s[idx]))
            idx++;
        return idx;
    }

    private static int ParseFloat(string s, int idx, out float value)
    {
        idx = SkipWs(s, idx);
        int start = idx;

        if (idx < s.Length && (s[idx] == '+' || s[idx] == '-'))
            idx++;

        bool anyDigits = false;
        while (idx < s.Length && char.IsDigit(s[idx]))
        {
            anyDigits = true;
            idx++;
        }

        if (idx < s.Length && s[idx] == '.')
        {
            idx++;
            while (idx < s.Length && char.IsDigit(s[idx]))
            {
                anyDigits = true;
                idx++;
            }
        }

        if (!anyDigits)
        {
            value = 0;
            return start;
        }

        string token = s.Substring(start, idx - start);
        if (!float.TryParse(token, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
            value = 0;

        return idx;
    }

    private static int ParseInt(string s, int idx, out int value)
    {
        idx = ParseFloat(s, idx, out float f);
        value = (int)f;
        return idx;
    }

    public static string StripComments(string src)
    {
        Span<char> dst = stackalloc char[Math.Min(src.Length, 4096)];
        int w = 0;
        bool inParen = false;
        bool inSemi = false;
        bool lastWasSpace = true; // start true to skip leading whitespace

        for (int i = 0; i < src.Length && w < dst.Length; i++)
        {
            char c = src[i];

            if (!inParen && !inSemi && c == '(')
            {
                inParen = true;
                continue;
            }

            if (inParen)
            {
                if (c == ')')
                    inParen = false;
                continue;
            }

            if (c == ';')
            {
                inSemi = !inSemi;
                continue;
            }

            if (inSemi)
                continue;

            // Normalize whitespace: compress multiple spaces, skip leading
            if (IsSpace(c))
            {
                if (!lastWasSpace)
                {
                    dst[w++] = ' ';
                    lastWasSpace = true;
                }
                continue;
            }

            dst[w++] = c;
            lastWasSpace = false;
        }

        // Trim trailing space
        if (w > 0 && dst[w - 1] == ' ')
            w--;

        return new string(dst.Slice(0, w));
    }

    public static void ScanLineText(string line, ref ScanLine s)
    {
        s = default;
        int i = 0;

        while (i < line.Length)
        {
            i = SkipWs(line, i);
            if (i >= line.Length)
                break;

            char c = Up(line[i]);
            if (c < 'A' || c > 'Z')
            {
                i++;
                continue;
            }

            i++;
            if (c == 'N')
            {
                s.hasN = true;
                i = ParseInt(line, i, out s.N);
            }
            else if (c == 'G')
            {
                s.hasG = true;
                i = ParseInt(line, i, out s.G);
                if (s.G == 0)
                    s.sawG0 = true;
                if (s.G == 1)
                    s.sawG1 = true;
                if (s.G == 2)
                    s.sawG2 = true;
                if (s.G == 3)
                    s.sawG3 = true;
                if (s.G == 17)
                    s.sawG17 = true;
                if (s.G == 20)
                    s.sawG20 = true;
                if (s.G == 21)
                    s.sawG21 = true;
                if (s.G == 90)
                    s.sawG90 = true;
                if (s.G == 91)
                    s.sawG91 = true;
                if (s.G == 41)
                    s.sawG41 = true;
                if (s.G == 42)
                    s.sawG42 = true;
                if (s.G == 40)
                    s.sawG40 = true;
            }
            else if (c == 'X')
            {
                s.hasX = true;
                i = ParseFloat(line, i, out s.X);
                s.isMove = true;
            }
            else if (c == 'Y')
            {
                s.hasY = true;
                i = ParseFloat(line, i, out s.Y);
                s.isMove = true;
            }
            else if (c == 'Z')
            {
                s.hasZ = true;
                i = ParseFloat(line, i, out s.Z);
            }
            else if (c == 'I')
            {
                s.hasI = true;
                i = ParseFloat(line, i, out s.I);
            }
            else if (c == 'J')
            {
                s.hasJ = true;
                i = ParseFloat(line, i, out s.J);
            }
            else if (c == 'R')
            {
                s.hasR = true;
                i = ParseFloat(line, i, out s.R);
            }
            else if (c == 'F')
            {
                s.hasF = true;
                i = ParseFloat(line, i, out s.F);
            }
            else if (c == 'D')
            {
                s.hasD = true;
                i = ParseInt(line, i, out s.D);
            }
            else if (c == 'T')
            {
                s.hasT = true;
                i = ParseInt(line, i, out s.T);
            }
            else if (c == 'S')
            {
                s.hasS = true;
                i = ParseFloat(line, i, out s.S);
            }
            else
            {
                i = ParseFloat(line, i, out _);
            }
        }
    }

    public static bool ArcCenterFromR(Vec2 p0, Vec2 p1, float rIn, ArcDir dir, out Vec2 outC)
    {
        float r = MathF.Abs(rIn);
        bool useShortArc = rIn >= 0.0f;
        Vec2 chord = p1 - p0;
        float d = CcMath.Len(chord);

        if (d < CcConst.TOL || d > 2.0f * r + 1e-5f)
        {
            outC = new Vec2(0, 0);
            return false;
        }

        Vec2 m = (p0 + p1) * 0.5f;
        float half = 0.5f * d;
        float h = MathF.Sqrt(MathF.Max(0.0f, r * r - half * half));

        Vec2 u = chord * (1.0f / d);
        Vec2 perp = CcMath.LeftNormal(u);

        Vec2 c1 = m + perp * h;
        Vec2 c2 = m - perp * h;

        bool Ok(in Vec2 c)
        {
            Vec2 a = p0 - c;
            Vec2 b = p1 - c;
            float z = CcMath.Cross(a, b);
            if (dir == ArcDir.ARC_CCW)
                return useShortArc ? z > 0.0f : z < 0.0f;
            return useShortArc ? z < 0.0f : z > 0.0f;
        }

        bool ok1 = Ok(c1);
        bool ok2 = Ok(c2);

        if (ok1 && !ok2)
            outC = c1;
        else if (ok2 && !ok1)
            outC = c2;
        else
            outC = c1;

        return true;
    }

    public static Move2D InterpretMove(in ScanLine s, ref ModalState modeState)
    {
        if (s.sawG17)
            modeState.planeXY = true;
        if (s.sawG20)
            modeState.inchMode = true;
        if (s.sawG21)
            modeState.inchMode = false;
        if (s.sawG90)
            modeState.absoluteMode = true;
        if (s.sawG91)
            modeState.absoluteMode = false;
        modeState.absXYZ = modeState.absoluteMode;
        if (s.sawG40)
            modeState.comp = CompSide.COMP_OFF;
        if (s.sawG41)
            modeState.comp = CompSide.COMP_LEFT;
        if (s.sawG42)
            modeState.comp = CompSide.COMP_RIGHT;
        if (s.hasF)
            modeState.feed = s.F;
        if (s.hasD)
            modeState.D_Register = s.D;
        if (s.hasT)
            modeState.T_Register = s.T;
        if (s.hasN)
            modeState.N_number = s.N;
        if (s.hasS)
            modeState.speed = s.S;

        if (s.sawG0)
            modeState.motionG = 0;
        else if (s.sawG1)
            modeState.motionG = 1;
        else if (s.sawG2)
            modeState.motionG = 2;
        else if (s.sawG3)
            modeState.motionG = 3;

        Vec2 p0 = modeState.pos;
        Vec2 p1 = p0;
        float z0 = modeState.z;
        float z1 = z0;

        bool anyXYZ = false;
        if (s.hasX)
        {
            p1.x = modeState.absoluteMode ? s.X : p0.x + s.X;
            anyXYZ = true;
        }
        if (s.hasY)
        {
            p1.y = modeState.absoluteMode ? s.Y : p0.y + s.Y;
            anyXYZ = true;
        }
        if (s.hasZ)
        {
            z1 = modeState.absoluteMode ? s.Z : z0 + s.Z;
            anyXYZ = true;
        }

        Move2D output = new Move2D();

        if (s.hasN)
            output.seqNum = (uint)s.N;

        output.p_0 = p0;
        output.p_1 = p1;
        output.z_0 = z0;
        output.z_1 = z1;
        output.hasXY = s.hasX || s.hasY;
        output.hasZ = s.hasZ;
        output.feed = modeState.feed;

        if (modeState.compMode == CompMode.CM_IN || modeState.compMode == CompMode.CM_OUT)
            output.compMode = CompMode.CM_STEADY;
        else
            output.compMode = CompMode.CM_NONE;

        if (s.sawG41 || s.sawG42)
        {
            output.compMode = CompMode.CM_IN;
            modeState.compMode = CompMode.CM_IN;
        }
        else if (s.sawG40)
        {
            output.compMode = CompMode.CM_OUT;
            modeState.compMode = CompMode.CM_OUT;
        }

        if (modeState.motionG == 0 && anyXYZ)
        {
            output.type = MotionType.MOT_RAPID;
            modeState.pos = p1;
            modeState.z = z1;
            return output;
        }

        if (modeState.motionG == 1 && anyXYZ)
        {
            output.type = MotionType.MOT_LINE;
            modeState.pos = p1;
            modeState.z = z1;
            return output;
        }

        if ((modeState.motionG == 2 || modeState.motionG == 3) && anyXYZ)
        {
            output.type = MotionType.MOT_ARC;
            output.arcDir = modeState.motionG == 2 ? ArcDir.ARC_CW : ArcDir.ARC_CCW;
            output.hasXY = true;

            if (s.hasI || s.hasJ)
            {
                Vec2 ij = CcMath.V2(s.hasI ? s.I : 0.0f, s.hasJ ? s.J : 0.0f);
                output.center = p0 + ij;
                output.radius = CcMath.Len(p0 - output.center);
            }
            else if (s.hasR)
            {
                if (!ArcCenterFromR(p0, p1, s.R, output.arcDir, out Vec2 c))
                {
                    output.type = MotionType.MOT_LINE;
                }
                else
                {
                    output.center = c;
                    output.radius = CcMath.Len(p0 - c);
                }
            }
            else
            {
                output.type = MotionType.MOT_LINE;
            }

            modeState.pos = p1;
            modeState.z = z1;
            return output;
        }

        modeState.pos = p1;
        modeState.z = z1;
        return output;
    }
}
