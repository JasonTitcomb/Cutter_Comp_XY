using System.Globalization;

namespace CutterCompXY.Port;

internal static class Program
{
    private const float TOOL_RADIUS = 0.0651f;
    private const CornerType CORNER_TREATMENT = CornerType.CORNER_ROLL;
    private const bool PERFORM_TRIM = true;
    private const int MAX_LOOKAHEAD = 10;
    private const int TRIM_OVERLAP_MOVES = MAX_LOOKAHEAD + 2;
    private const int EMIT_HOLDBACK = TRIM_OVERLAP_MOVES;
    private const int TARGET_BATCH_EMIT_MOVES = 20;
    private const int MIN_PENDING_BEFORE_BATCH = EMIT_HOLDBACK + TARGET_BATCH_EMIT_MOVES;

    private static ModalState modalState = new ModalState();
    private static CutterComp2D cc = new CutterComp2D();
    private static readonly List<Move2D> profile = new List<Move2D>();

    private static bool hasStopError = false;

    private static void CompErrorHandler(CompError err, uint seqNum)
    {
        string msg = err switch
        {
            CompError.CE_ARC_RADIUS_MISMATCH => "Arc radius inconsistency",
            CompError.CE_INVALID_MOVE => "Invalid move",
            CompError.CE_COMP_MOVE_TOO_SHORT => "Comp move too short",
            CompError.CE_ARC_LT_TOOL_RAD => "Arc smaller than tool radius",
            CompError.CE_FLIPPED_ARC => "Flipped arc",
            CompError.CE_COMP_IN_CROSSING => "Comp-in crossing",
            CompError.CE_COMP_OUT_CROSSING => "Comp-out crossing",
            CompError.CE_UNRESOLVED_GAP => "Unresolved gap",
            _ => "Unknown comp error"
        };

        Console.WriteLine($"CompError: {msg} (N{seqNum})");
        hasStopError = true;
    }

    private static void ProfileReset() => profile.Clear();

    private static void ProfilePush(in Move2D m)
    {
        Move2D t = m;
        t.valid = true;
        profile.Add(t);
    }

    private static void UpdateUnitsModeFromLine(string line, ref bool inchUnits)
    {
        int i = 0;
        while (i < line.Length)
        {
            if (char.ToUpperInvariant(line[i]) != 'G')
            {
                i++;
                continue;
            }

            i++;
            while (i < line.Length && (line[i] == ' ' || line[i] == '\t'))
                i++;

            int start = i;
            while (i < line.Length && char.IsDigit(line[i]))
                i++;

            if (i == start)
                continue;

            if (!int.TryParse(line[start..i], out int code))
                continue;

            if (code == 20)
                inchUnits = true;
            else if (code == 21)
                inchUnits = false;
        }
    }

    private static void EmitMoveAsGcode(StreamWriter sw, in Move2D m, bool inchUnits)
    {
        int digits = inchUnits ? 4 : 3;

        if (m.type == MotionType.MOT_LINE || m.type == MotionType.MOT_RAPID)
        {
            if (m.seqNum != 0)
                sw.Write($"N{m.seqNum} ");
            sw.Write($"{(m.type == MotionType.MOT_RAPID ? "G0" : "G1")}");
            if (m.hasXY)
            {
                sw.Write($" X{m.p_1.x.ToString($"F{digits}", CultureInfo.InvariantCulture)}");
                sw.Write($" Y{m.p_1.y.ToString($"F{digits}", CultureInfo.InvariantCulture)}");
            }
            if (m.hasZ)
                sw.Write($" Z{m.z_1.ToString($"F{digits}", CultureInfo.InvariantCulture)}");
            sw.WriteLine();
            return;
        }

        if (m.type == MotionType.MOT_ARC)
        {
            Vec2 dCenter = m.center - m.p_0;
            if (m.seqNum != 0)
                sw.Write($"N{m.seqNum} ");
            sw.Write($"{(m.arcDir == ArcDir.ARC_CW ? "G2" : "G3")}");
            sw.Write($" X{m.p_1.x.ToString($"F{digits}", CultureInfo.InvariantCulture)}");
            sw.Write($" Y{m.p_1.y.ToString($"F{digits}", CultureInfo.InvariantCulture)}");
            sw.Write($" I{dCenter.x.ToString($"F{digits}", CultureInfo.InvariantCulture)}");
            sw.Write($" J{dCenter.y.ToString($"F{digits}", CultureInfo.InvariantCulture)}");
            if (m.hasZ)
                sw.Write($" Z{m.z_1.ToString($"F{digits}", CultureInfo.InvariantCulture)}");
            sw.WriteLine();
        }
    }

    private static bool EmitCompProfile(StreamWriter sw, int nextEmitIndex, int holdBackCount, bool flushAll, bool inchUnits, out int newNextEmitIndex)
    {
        int profileSize = profile.Count;
        if (nextEmitIndex < 0)
            nextEmitIndex = 0;

        int emitLimit = profileSize;
        if (!flushAll)
        {
            emitLimit = profileSize - holdBackCount;
            if (emitLimit < 0)
                emitLimit = 0;
        }

        if (nextEmitIndex >= emitLimit)
        {
            newNextEmitIndex = nextEmitIndex;
            return true;
        }

        for (int i = nextEmitIndex; i < emitLimit; i++)
        {
            Move2D m = profile[i];
            if (!m.valid || m.type == MotionType.MOT_EMPTY)
                continue;
            EmitMoveAsGcode(sw, m, inchUnits);
        }

        newNextEmitIndex = emitLimit;
        return true;
    }

    private static bool ProcessOneGcodeLine(ScanLine s)
    {
         Move2D mv = SimpleScan.InterpretMove(s, ref modalState);

        if (s.sawG41 || s.sawG42)
            cc.SetComp(modalState.comp);

        if (!cc.PushIn(mv))
        {
            Console.WriteLine("(comp input buffer full)");
            return false;
        }

        if (!cc.Process())
        {
            Console.WriteLine("(comp processing failed)");
            return false;
        }

        while (cc.PopOut(out Move2D outMove))
            ProfilePush(outMove);

        if (s.sawG40)
        {
            cc.Flush();
            cc.SetComp(CompSide.COMP_OFF);
            while (cc.PopOut(out Move2D outMove))
                ProfilePush(outMove);
        }

        return true;
    }

    private static void FlushPipeline()
    {
        cc.Flush();
        while (cc.PopOut(out Move2D outMove))
            ProfilePush(outMove);
    }

    private static bool TrimAndMergePendingProfile(int emittedProfileCount, int trimResumeIndex, out int newTrimResumeIndex)
    {
        int profileSize = profile.Count;
        int trimStart = trimResumeIndex;
        if (trimStart < emittedProfileCount)
            trimStart = emittedProfileCount;

        if (trimStart >= profileSize)
        {
            newTrimResumeIndex = trimResumeIndex;
            return true;
        }

        Move2D[] moves = profile.ToArray();

        if (PERFORM_TRIM)
        {
            int srcIdx = trimStart;
            if (!cc.TrimCrossingElements(moves, ref srcIdx, profileSize, MAX_LOOKAHEAD, out _))
            {
                newTrimResumeIndex = trimResumeIndex;
                return false;
            }

            int mergeStart = trimStart;
            if (mergeStart > emittedProfileCount)
                mergeStart -= 1;

            Move2D[] mergeSlice = new Move2D[profileSize - mergeStart];
            Array.Copy(moves, mergeStart, mergeSlice, 0, mergeSlice.Length);
            cc.MergeAllColinear(mergeSlice, mergeSlice.Length);
            Array.Copy(mergeSlice, 0, moves, mergeStart, mergeSlice.Length);
        }

        profile.Clear();
        profile.AddRange(moves);

        int nextTrimStart = profileSize - TRIM_OVERLAP_MOVES;
        if (nextTrimStart < emittedProfileCount)
            nextTrimStart = emittedProfileCount;
        if (nextTrimStart < 0)
            nextTrimStart = 0;

        newTrimResumeIndex = nextTrimStart;
        return true;
    }

    private static bool RunProfileStreaming(string inputPath, string emitGcodePath, float toolRadius, CornerType cornerTreatment)
    {
        modalState = new ModalState
        {
            planeXY = true,
            absXYZ = true,
            motionG = 0,
            comp = CompSide.COMP_OFF,
            feed = 0,
            pos = new Vec2(0, 0),
            z = 0.0f
        };

        cc = new CutterComp2D();
        float activeToolRadius = toolRadius;
        cc.SetToolRadius(activeToolRadius);
        cc.SetCornerTreatment(cornerTreatment);
        cc.SetPerformTrim(PERFORM_TRIM);
        cc.SetErrorCallback(CompErrorHandler);
        cc.SetComp(CompSide.COMP_OFF);

        ProfileReset();

        if (!File.Exists(inputPath))
        {
            Console.WriteLine($"Failed to open input file: {inputPath}");
            return false;
        }

        bool sawCompStart = false;
        bool sawG40 = false;
        bool compClosed = false;
        int emittedProfileCount = 0;
        bool inchUnits = true;
        int trimResumeIndex = 0;

        using StreamWriter outFile = new StreamWriter(emitGcodePath);
        foreach (string rawLine in File.ReadLines(inputPath))
        {
            if (hasStopError)
            {
                Console.WriteLine("(stopped due to previous errors)");
                break;
            }

            string line = rawLine.TrimEnd('\r');
            string clean = SimpleScan.StripComments(line);
            UpdateUnitsModeFromLine(clean, ref inchUnits);

            ScanLine s = default;
            SimpleScan.ScanLineText(clean, ref s);

            bool compIsOff = cc.comp_state == CompSide.COMP_OFF;
            bool entersComp = compIsOff && (s.sawG41 || s.sawG42) && !compClosed;

            if (entersComp)
            {
                sawCompStart = true;
                outFile.WriteLine($"(comp start: {(s.sawG41 ? "G41" : "G42")})");
            }

            if (compIsOff && !entersComp)
            {
                _ = SimpleScan.InterpretMove(s, ref modalState);
                if (!s.sawG40)
                    outFile.WriteLine(line);
                continue;
            }

            bool zOnlyMove = s.hasZ && !s.hasX && !s.hasY;
            if (zOnlyMove)
            {
                _ = SimpleScan.InterpretMove(s, ref modalState);
                if (!s.sawG40)
                    outFile.WriteLine(line);
                continue;
            }

            if (!ProcessOneGcodeLine(s))
                return false;

            if (cc.comp_state != CompSide.COMP_OFF)
            {
                int pendingProfileWindow = profile.Count - emittedProfileCount;
                if (pendingProfileWindow >= MIN_PENDING_BEFORE_BATCH)
                {
                    if (!TrimAndMergePendingProfile(emittedProfileCount, trimResumeIndex, out trimResumeIndex))
                        return false;

                    if (!EmitCompProfile(outFile, emittedProfileCount, EMIT_HOLDBACK, false, inchUnits, out emittedProfileCount))
                        return false;

                    outFile.WriteLine("(comp batch emit)");
                }
            }

            if (cc.comp_state == CompSide.COMP_OFF && sawCompStart)
            {
                sawG40 = true;
                compClosed = true;

                if (!TrimAndMergePendingProfile(emittedProfileCount, trimResumeIndex, out trimResumeIndex))
                    return false;

                if (!EmitCompProfile(outFile, emittedProfileCount, 0, true, inchUnits, out emittedProfileCount))
                    return false;

                outFile.WriteLine("(comp stop: G40)");
            }
        }

        if (sawCompStart && !compClosed)
        {
            FlushPipeline();

            if (!TrimAndMergePendingProfile(emittedProfileCount, trimResumeIndex, out trimResumeIndex))
                return false;

            if (!EmitCompProfile(outFile, emittedProfileCount, 0, true, inchUnits, out emittedProfileCount))
                return false;
        }

        if (!sawG40)
            Console.WriteLine("(warning: reached EOF before G40)");

        return !sawCompStart || profile.Count > 0;
    }

    private static int Main(string[] args)
    {
        string repoRoot = FindRepoRoot();
        string defaultInput = Path.Combine(repoRoot, "data", "TortureTestG90.nc");
        string inputPath = args.Length > 0 ? args[0] : defaultInput;

        string outputDir = Path.Combine(repoRoot, "output");
        Directory.CreateDirectory(outputDir);

        string inputBaseName = Path.GetFileNameWithoutExtension(inputPath);
        string ngcPath = Path.Combine(outputDir, inputBaseName + ".cs.ngc");

        bool ok = RunProfileStreaming(inputPath, ngcPath, TOOL_RADIUS, CORNER_TREATMENT);
        if (!ok)
        {
            Console.WriteLine("(warning: profile validation failed)");
            return 1;
        }

        Console.WriteLine($"Wrote: {ngcPath}");
        return 0;
    }

    private static string FindRepoRoot()
    {
        DirectoryInfo dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir != null)
        {
            string candidate = Path.Combine(dir.FullName, "data");
            if (Directory.Exists(candidate))
                return dir.FullName;
            dir = dir.Parent;
        }

        return Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", ".."));
    }
}
