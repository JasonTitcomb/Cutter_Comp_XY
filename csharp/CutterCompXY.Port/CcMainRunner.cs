using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace CutterCompXY.Port;

public sealed class CcMainRunner
{
    // Keep lookahead for trim solver window.
    public const int MAX_LOOKAHEAD = 20;

    public CcMainOptions options;
    public CcOutputCB outputCB;
    public CcErrorCB errorCB;
    public CcStartCompCB startCompCB;

    public ModalState modalState;
    public CutterComp2D cc = new CutterComp2D();

    public readonly List<Move2D> profile = new List<Move2D>();
    public bool sawCompStart = false;
    public bool sawG40 = false;
    public bool compClosed = false;
    public bool runActive = false;
    public bool globalTrim = false;
    public bool emitComments = true;
    public bool inchMode = true;
    private bool hasLastFeed = false;
    private float lastFeed = 0.0f;

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
        modalState.feed = 0;
        modalState.speed = 0;
        modalState.pos = new Vec2(0, 0);
        modalState.z = 0;
        modalState.N_number = 0;
        modalState.T_Register = 0;
        modalState.D_Register = 0;
        modalState.inchMode = true;
        inchMode = true;

        cc = new CutterComp2D();
        cc.SetToolRadius(options.toolRadius);
        cc.SetCornerTreatment(options.cornerTreatment);
        cc.SetPerformTrim(options.globalTrimCrossing);
        cc.SetErrorCallback(OnCompError);
        SyncUnitsFromModal();

        outputCB = options.callbacks.output;
        errorCB = options.callbacks.error;
        startCompCB = options.callbacks.startComp;
        globalTrim = options.globalTrimCrossing;
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
        if (!runActive || string.IsNullOrEmpty(line))
            return false;

        string peekClean = SimpleScan.StripComments(line);
        if (string.IsNullOrEmpty(peekClean))
            return true;

        ScanLine scanLn = new ScanLine();
        SimpleScan.ScanLineText(peekClean, ref scanLn);

        bool compIsOff = (cc.comp_state == CompSide.COMP_OFF);
        bool canEmitRaw = compIsOff && !scanLn.sawG41 && !scanLn.sawG42 && (!sawCompStart || compClosed);
        bool entersComp = compIsOff && (scanLn.sawG41 || scanLn.sawG42) && !compClosed;
        if (entersComp)
        {
            sawCompStart = true;
            EmitStatus("(COMP ON)\n");
            startCompCB?.Invoke(modalState.T_Register, modalState.D_Register);
        }

        if (canEmitRaw)
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
            if (!EmitCompProfile())
            {
                ReportError("(emit failed)", CompError.CE_ERROR);
                runActive = false;
                return false;
            }
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
            if (!EmitCompProfile())
            {
                ReportError("(final emit failed)", CompError.CE_ERROR);
                runActive = false;
                return false;
            }
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

    private void ReportError(string message, CompError err)
    {
        errorCB?.Invoke(message, (int)err, (uint)modalState.N_number);
    }

    private void OnCompError(CompError err, uint seqNum)
    {
        ReportError($"CompError {(uint)err} N{seqNum}", err);
    }

    private void SyncUnitsFromModal()
    {
        inchMode = modalState.inchMode;
        cc.SetUnits(inchMode ? Units.UNITS_INCH : Units.UNITS_MM);
    }

    private void EmitRawLine(string text)
    {
        if (outputCB == null || string.IsNullOrEmpty(text))
            return;

        outputCB(text, text.Length);
        if (text[text.Length - 1] != '\n')
            outputCB("\n", 1);
    }

    private void ProfileReset()
    {
        profile.Clear();
    }

    private bool ProfilePush(Move2D m)
    {
        m.valid = true;
        profile.Add(m);
        return true;
    }

    private bool TrimAndMergePendingProfile()
    {
        if (!globalTrim || profile.Count == 0)
            return true;

        Move2D[] moves = profile.ToArray();
        int srcIdx = 0;
        if (!cc.TrimCrossingElements(moves, ref srcIdx, moves.Length, MAX_LOOKAHEAD, out _))
            return false;

        cc.MergeAllColinear(moves, moves.Length);
        profile.Clear();
        profile.AddRange(moves);
        return true;
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

            if (m.hasXY)
            {
                if (modalState.absoluteMode)
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
                AppendCoord(sb, "Z", modalState.absoluteMode ? m.z_1 : dz, posDigits);
        }
        else if (m.type == MotionType.MOT_ARC)
        {
            Vec2 dCenter = m.center - m.p_0;
            float dx = m.p_1.x - m.p_0.x;
            float dy = m.p_1.y - m.p_0.y;
            float dz = m.z_1 - m.z_0;

            sb.Append(m.arcDir == ArcDir.ARC_CW ? "G2" : "G3");
            AppendCoord(sb, "X", modalState.absoluteMode ? m.p_1.x : dx, posDigits);
            AppendCoord(sb, "Y", modalState.absoluteMode ? m.p_1.y : dy, posDigits);
            AppendCoord(sb, "I", dCenter.x, posDigits);
            AppendCoord(sb, "J", dCenter.y, posDigits);

            if (m.feed > 0.0f && (!hasLastFeed || m.feed != lastFeed))
            {
                AppendCoord(sb, "F", m.feed, posDigits);
                lastFeed = m.feed;
                hasLastFeed = true;
            }

            if (m.hasZ)
                AppendCoord(sb, "Z", modalState.absoluteMode ? m.z_1 : dz, posDigits);
        }

        if (sb.Length > 0)
        {
            sb.Append('\n');
            string line = sb.ToString();
            outputCB(line, line.Length);
        }
    }

    private bool EmitCompProfile()
    {
        for (int i = 0; i < profile.Count; ++i)
        {
            Move2D m = profile[i];
            if (!m.valid || m.type == MotionType.MOT_EMPTY)
                continue;
            EmitMoveAsGcode(m);
        }
        profile.Clear();
        return true;
    }

    private bool ProcessRawGcodeLine(string rawLine, ScanLine s)
    {
        _ = SimpleScan.InterpretMove(s, ref modalState);
        SyncUnitsFromModal();
        EmitRawLine(rawLine);
        return true;
    }

    private bool FlushPipeline()
    {
        cc.Flush();
        Move2D outputMove;
        while (cc.PopOut(out outputMove))
        {
            if (!ProfilePush(outputMove))
            {
                ReportError("(profile buffer full)", CompError.CE_ERROR);
                return false;
            }
        }
        return true;
    }

    private bool ProcessOneGcodeLine(ScanLine s)
    {
        Move2D mv = SimpleScan.InterpretMove(s, ref modalState);
        Move2D outputMove;
        SyncUnitsFromModal();
        if (s.sawG41 || s.sawG42)
            cc.SetComp(modalState.comp);
        if (mv.type == MotionType.MOT_EMPTY)
        {
            if (s.sawG40)
            {
                cc.SetComp(CompSide.COMP_OFF);
                cc.Flush();
                while (cc.PopOut(out outputMove))
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
        while (cc.PopOut(out outputMove))
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
            cc.SetComp(CompSide.COMP_OFF);
            while (cc.PopOut(out outputMove))
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
}
