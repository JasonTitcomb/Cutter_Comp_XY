using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace CutterCompXY.Port;

public sealed class CcMainRunner
{
    public const int MAX_LOOKAHEAD = 20;
    private const int TARGET_BATCH_EMIT_MOVES = 40;
    private const int PROFILE_BURST_MARGIN = 2;
    private const int TRIM_OVERLAP_MOVES = MAX_LOOKAHEAD + 2;
    private const int EMIT_HOLDBACK = TRIM_OVERLAP_MOVES;
    private const int MIN_PENDING_BEFORE_BATCH = EMIT_HOLDBACK + TARGET_BATCH_EMIT_MOVES;
    private const int MAX_PROFILE_MOVES = MIN_PENDING_BEFORE_BATCH + PROFILE_BURST_MARGIN;

    public CcMainOptions options;
    public CcOutputCB outputCB;
    public CcErrorCB errorCB;
    public CcStartCompCB startCompCB;
    public ModalState modalState;
    public CutterComp2D cc = new CutterComp2D();
    public readonly List<Move2D> profile = new List<Move2D>(MAX_PROFILE_MOVES);
    public bool sawCompStart;
    public bool sawG40;
    public bool compClosed;
    public bool runActive;
    public bool globalTrim;
    public bool emitComments;
    public bool inchMode;

    private bool hasLastFeed;
    private float lastFeed;
    private int emittedProfileCount;
    private int trimResumeIndex;

    public bool Begin(CcMainOptions opts)
    {
        options = opts;
        runActive = true;

        modalState = new ModalState();
        modalState.planeXY = true;
        modalState.absXYZ = true;
        modalState.absoluteMode = true;
        modalState.motionG = 0;
        modalState.comp = CompSide.COMP_OFF;
        modalState.compMode = CompMode.CM_NONE;
        modalState.feed = 0.0f;
        modalState.speed = 0.0f;
        modalState.pos = new Vec2(0, 0);
        modalState.z = 0.0f;
        modalState.N_number = 0;
        modalState.T_Register = 0;
        modalState.D_Register = 0;
        modalState.inchMode = true;
        inchMode = true;

        cc = new CutterComp2D();
        cc.SetOptions(options);
        cc.SetErrorCallback(OnCompError);
        SyncUnitsFromModal();

        outputCB = options.callbacks.output;
        errorCB = options.callbacks.error;
        startCompCB = options.callbacks.startComp;
        globalTrim = options.globalTrimCrossing;
        emittedProfileCount = 0;
        trimResumeIndex = 0;
        sawCompStart = false;
        sawG40 = false;
        compClosed = false;
        emitComments = options.emitStatusComments;
        hasLastFeed = false;
        lastFeed = 0.0f;
        ProfileReset();
        return true;
    }

    public void SetToolRadius(float r)
    {
        cc.SetToolRadius(r);
    }

    public bool ProcessLine(string line)
    {
        if (!runActive)
            return false;
        if (string.IsNullOrEmpty(line))
            return true;

        string peekClean = SimpleScan.StripComments(line);
        if (string.IsNullOrEmpty(peekClean))
            return true;

        ScanLine scanLn = new ScanLine();
        SimpleScan.ScanLineText(peekClean, ref scanLn);

        bool compIsOff = cc.comp_state == CompSide.COMP_OFF;
        bool emitNonComp = compIsOff && !scanLn.sawG41 && !scanLn.sawG42 && (!sawCompStart || compClosed);
        bool entersComp = compIsOff && (scanLn.sawG41 || scanLn.sawG42) && !compClosed;
        if (entersComp)
        {
            if (!scanLn.isMove)
            {
                ReportError("(move expected on G41/G42 line)", CompError.CE_ERROR);
                return false;
            }

            sawCompStart = true;
            EmitStatus("(COMP ON)\n");
            startCompCB?.Invoke(modalState.T_Register, modalState.D_Register);
        }

        if (emitNonComp)
        {
            if (!ProcessRawGcodeLine(line, scanLn))
            {
                runActive = false;
                return false;
            }

            return true;
        }

        if (!ProcessOneGcodeLine(scanLn))
        {
            runActive = false;
            return false;
        }

        if (cc.comp_state != CompSide.COMP_OFF)
        {
            int pendingProfileWindow = profile.Count - emittedProfileCount;
            if (pendingProfileWindow >= MIN_PENDING_BEFORE_BATCH)
            {
                if (!TrimAndMergePendingProfile())
                {
                    ReportError("(trim failed)", CompError.CE_ERROR);
                    runActive = false;
                    return false;
                }

                if (!EmitCompProfile(EMIT_HOLDBACK, false))
                {
                    ReportError("(emit failed)", CompError.CE_ERROR);
                    runActive = false;
                    return false;
                }

                ProfileCompact();
                EmitStatus("(BATCH)\n");
            }
        }

        if (cc.comp_state == CompSide.COMP_OFF && sawCompStart && !compClosed)
        {
            sawG40 = true;
            compClosed = true;
            if (!TrimAndMergePendingProfile())
            {
                ReportError("(trim failed)", CompError.CE_ERROR);
                runActive = false;
                return false;
            }

            if (!EmitCompProfile(0, true))
            {
                ReportError("(emit failed)", CompError.CE_ERROR);
                runActive = false;
                return false;
            }

            ProfileCompact();
            EmitStatus("(COMP OFF)\n");
        }

        return true;
    }

    public bool Finish()
    {
        if (!runActive)
            return false;

        if (sawCompStart && !compClosed)
        {
            if (!FlushPipeline())
            {
                runActive = false;
                return false;
            }

            if (!TrimAndMergePendingProfile())
            {
                ReportError("(final trim failed)", CompError.CE_ERROR);
                runActive = false;
                return false;
            }

            if (!EmitCompProfile(0, true))
            {
                ReportError("(final emit failed)", CompError.CE_ERROR);
                runActive = false;
                return false;
            }

            ProfileCompact();
        }

        if (!sawG40)
            ReportError("(warning: reached EOF before G40)", CompError.CE_ERROR);

        runActive = false;
        return true;
    }

    public bool RunProgram(string[] program, int lineCount, CcMainOptions opts)
    {
        if (program == null || lineCount <= 0)
            return true;

        if (!Begin(opts))
            return false;

        for (int i = 0; i < lineCount; ++i)
        {
            if (!ProcessLine(program[i]))
                return false;
        }

        return Finish();
    }

    private void EmitStatus(string text)
    {
        if (emitComments && outputCB != null && !string.IsNullOrEmpty(text))
            outputCB(text, text.Length);
    }

    private void EmitRawLine(string text)
    {
        if (outputCB == null || string.IsNullOrEmpty(text))
            return;

        outputCB(text, text.Length);
        if (text[text.Length - 1] != '\n')
            outputCB("\n", 1);
    }

    private void ReportError(string message, CompError err)
    {
        errorCB?.Invoke(message, (int)err, (uint)modalState.N_number);
    }

    private void OnCompError(CompError err, uint seqNum)
    {
        ReportError($"CompError {(uint)err} N{seqNum}", err);
    }

    private void ProfileReset()
    {
        profile.Clear();
    }

    private bool ProfilePush(Move2D m)
    {
        if (profile.Count >= MAX_PROFILE_MOVES)
            return false;

        profile.Add(m);
        return true;
    }

    private void ProfileCompact()
    {
        if (emittedProfileCount <= 0)
            return;

        int dropCount = emittedProfileCount;
        int keepCount = profile.Count - dropCount;
        if (keepCount > 0)
            profile.RemoveRange(0, dropCount);
        else
            profile.Clear();

        emittedProfileCount = 0;
        trimResumeIndex -= dropCount;
        if (trimResumeIndex < 0)
            trimResumeIndex = 0;
    }

    private static string TrimTrailingZeros(string value)
    {
        int dot = value.IndexOf('.');
        if (dot < 0)
            return value;

        int end = value.Length - 1;
        while (end > dot && value[end] == '0')
            end--;
        if (end == dot)
            end--;
        return value.Substring(0, end + 1);
    }

    private static void AppendCoord(StringBuilder sb, string axis, float value, int digits)
    {
        string num = TrimTrailingZeros(value.ToString("F" + digits, CultureInfo.InvariantCulture));
        sb.Append(' ');
        sb.Append(axis);
        sb.Append(num);
    }

    private void SyncUnitsFromModal()
    {
        inchMode = modalState.inchMode;
        cc.SetUnits(inchMode ? Units.UNITS_INCH : Units.UNITS_MM);
    }

    private void EmitMoveAsGcode(Move2D m)
    {
        if (outputCB == null)
            return;

        int posDigits = inchMode ? 4 : 3;
        StringBuilder sb = new StringBuilder(160);

        if (m.seqNum != 0)
            sb.Append("N").Append(m.seqNum).Append(' ');

        if (m.type == MotionType.MOT_LINE || m.type == MotionType.MOT_RAPID)
        {
            sb.Append(m.type == MotionType.MOT_RAPID ? "G0" : "G1");
            float dx = m.p_1.x - m.p_0.x;
            float dy = m.p_1.y - m.p_0.y;
            float dz = m.z_1 - m.z_0;
            bool isAbs = modalState.absoluteMode;

            if (m.hasXY)
            {
                if (isAbs)
                {
                    AppendCoord(sb, "X", m.p_1.x, posDigits);
                    AppendCoord(sb, "Y", m.p_1.y, posDigits);
                }
                else
                {
                    AppendCoord(sb, "X", dx, posDigits);
                    AppendCoord(sb, "Y", dy, posDigits);
                }
            }

            if (m.feed > 0.0f && (!hasLastFeed || m.feed != lastFeed))
            {
                AppendCoord(sb, "F", m.feed, posDigits);
                lastFeed = m.feed;
                hasLastFeed = true;
            }

            if (m.hasZ)
                AppendCoord(sb, "Z", isAbs ? m.z_1 : dz, posDigits);
        }
        else if (m.type == MotionType.MOT_ARC)
        {
            Vec2 dCenter = m.center - m.p_0;
            float dx = m.p_1.x - m.p_0.x;
            float dy = m.p_1.y - m.p_0.y;
            float dz = m.z_1 - m.z_0;
            bool isAbs = modalState.absoluteMode;

            sb.Append(m.arcDir == ArcDir.ARC_CW ? "G2" : "G3");
            AppendCoord(sb, "X", isAbs ? m.p_1.x : dx, posDigits);
            AppendCoord(sb, "Y", isAbs ? m.p_1.y : dy, posDigits);
            AppendCoord(sb, "I", dCenter.x, posDigits);
            AppendCoord(sb, "J", dCenter.y, posDigits);

            if (m.feed > 0.0f && (!hasLastFeed || m.feed != lastFeed))
            {
                AppendCoord(sb, "F", m.feed, posDigits);
                lastFeed = m.feed;
                hasLastFeed = true;
            }

            if (m.hasZ)
                AppendCoord(sb, "Z", isAbs ? m.z_1 : dz, posDigits);
        }

        if (sb.Length > 0)
        {
            sb.Append('\n');
            string line = sb.ToString();
            outputCB(line, line.Length);
        }
    }

    private bool EmitCompProfile(int holdBackCount, bool flushAll)
    {
        if (emittedProfileCount < 0)
            emittedProfileCount = 0;

        int emitLimit = profile.Count;
        if (!flushAll)
        {
            emitLimit = profile.Count - holdBackCount;
            if (emitLimit < 0)
                emitLimit = 0;
        }

        if (emittedProfileCount >= emitLimit)
            return true;

        for (int i = emittedProfileCount; i < emitLimit; ++i)
        {
            Move2D m = profile[i];
            if (!m.valid || m.type == MotionType.MOT_EMPTY)
                continue;
            if (m.type == MotionType.MOT_LINE && m.hasXY && !m.hasZ && CcMath.Len(m.p_1 - m.p_0) < CcConst.TOL)
                continue;
            EmitMoveAsGcode(m);
        }

        emittedProfileCount = emitLimit;
        return true;
    }

    private bool ProcessRawGcodeLine(string rawLine, ScanLine s)
    {
        _ = SimpleScan.InterpretMove(s, ref modalState);
        SyncUnitsFromModal();
        EmitRawLine(rawLine);
        return true;
    }

    private bool ProcessOneGcodeLine(ScanLine s)
    {
        Move2D mv = SimpleScan.InterpretMove(s, ref modalState);
        SyncUnitsFromModal();
        cc.SetComp(modalState.comp);

        if (mv.type == MotionType.MOT_EMPTY)
        {
            if (s.sawG40)
            {
                cc.Flush();
                while (cc.PopOut(out Move2D outputMove))
                {
                    if (!ProfilePush(outputMove))
                    {
                        ReportError("(profile buffer full)", CompError.CE_ERROR);
                        return false;
                    }
                }
            }

            return true;
        }

        if (!cc.PushIn(mv))
        {
            ReportError("(comp input buffer full)", CompError.CE_ERROR);
            return false;
        }

        if (!cc.Process())
        {
            ReportError("(comp processing failed!)", CompError.CE_ERROR);
            return false;
        }

        while (cc.PopOut(out Move2D outputMove))
        {
            if (!ProfilePush(outputMove))
            {
                ReportError("(profile buffer full)", CompError.CE_ERROR);
                return false;
            }
        }

        if (s.sawG40)
        {
            cc.Flush();
            while (cc.PopOut(out Move2D outputMove))
            {
                if (!ProfilePush(outputMove))
                {
                    ReportError("(profile buffer full)", CompError.CE_ERROR);
                    return false;
                }
            }
        }

        return true;
    }

    private bool FlushPipeline()
    {
        cc.Flush();
        while (cc.PopOut(out Move2D outputMove))
        {
            if (!ProfilePush(outputMove))
            {
                ReportError("(profile buffer full)", CompError.CE_ERROR);
                return false;
            }
        }

        return true;
    }

    private bool TrimAndMergePendingProfile()
    {
        if (!globalTrim)
            return true;

        int currentProfileCount = profile.Count;
        int trimStart = trimResumeIndex;
        if (trimStart < emittedProfileCount)
            trimStart = emittedProfileCount;

        if (trimStart >= currentProfileCount)
            return true;

        Move2D[] moves = profile.ToArray();

        if (options.globalTrimCrossing)
        {
            int srcIdx = trimStart;
            if (!cc.TrimCrossingElements(moves, ref srcIdx, currentProfileCount, MAX_LOOKAHEAD, out _))
                return false;
        }

        if (options.globalMerge)
        {
            int mergeStart = trimStart;
            if (mergeStart > emittedProfileCount)
                mergeStart -= 1;

            int mergeCount = currentProfileCount - mergeStart;
            Move2D[] mergeSlice = new Move2D[mergeCount];
            Array.Copy(moves, mergeStart, mergeSlice, 0, mergeCount);
            cc.MergeAllColinear(mergeSlice, mergeCount);
            Array.Copy(mergeSlice, 0, moves, mergeStart, mergeCount);
        }

        profile.Clear();
        profile.AddRange(moves);

        int nextTrimStart = currentProfileCount - TRIM_OVERLAP_MOVES;
        if (nextTrimStart < emittedProfileCount)
            nextTrimStart = emittedProfileCount;
        if (nextTrimStart < 0)
            nextTrimStart = 0;
        trimResumeIndex = nextTrimStart;
        return true;
    }
}
