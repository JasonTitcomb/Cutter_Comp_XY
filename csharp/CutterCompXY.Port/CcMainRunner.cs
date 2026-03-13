using System;
using System.Globalization;

namespace CutterCompXY.Port;

public sealed class CcMainRunner
{
    // Constants for batching and lookahead
    public const int MAX_LOOKAHEAD = 10;
    public const int TARGET_BATCH_EMIT_MOVES = 20;
    public const int PROFILE_BURST_MARGIN = 2;
    public const int TRIM_OVERLAP_MOVES = MAX_LOOKAHEAD + 2;
    public const int EMIT_HOLDBACK = TRIM_OVERLAP_MOVES;
    public const int MIN_PENDING_BEFORE_BATCH = EMIT_HOLDBACK + TARGET_BATCH_EMIT_MOVES;
    public const int MAX_PROFILE_MOVES = MIN_PENDING_BEFORE_BATCH + PROFILE_BURST_MARGIN;

    public CcMainOptions options;
    public CcOutputCB outputCB;
    public CcErrorCB errorCB;
    public CcStartCompCB startCompCB;

    public ModalState modalState;
    public CutterComp2D cc = new CutterComp2D();

    public Move2D[] profile = new Move2D[MAX_PROFILE_MOVES];
    public int profileCount = 0;
    public int emittedProfileCount = 0;
    public int trimResumeIndex = 0;
    public bool sawCompStart = false;
    public bool sawG40 = false;
    public bool compClosed = false;
    public bool runActive = false;
    public bool globalTrim = false;
    public bool emitComments = true;

    public bool Begin(CcMainOptions opts)
    {
        options = opts;
        runActive = true;
        modalState = new ModalState();
        modalState.planeXY = true;
        modalState.absoluteMode = true;
        modalState.motionG = 0;
        modalState.comp = CompSide.COMP_OFF;
        modalState.feed = 0;
        modalState.speed = 0;
        modalState.pos = new Vec2(0, 0);
        modalState.N_number = 0;

        cc.SetOptions(options);
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
            // TODO: implement process_raw_gcode_line if needed
            return true;
        }

        if (!ProcessOneGcodeLine(scanLn))
        {
            runActive = false;
            return false;
        }

        if (cc.comp_state != CompSide.COMP_OFF)
        {
            int pendingProfileWindow = profileCount - emittedProfileCount;
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
            // TODO: implement flush_pipeline if needed
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

    private void ReportError(string message, CompError err)
    {
        errorCB?.Invoke(message, err, modalState.N_number);
    }

    private void ProfileReset()
    {
        profileCount = 0;
    }

    private bool ProfilePush(Move2D m)
    {
        if (profileCount >= MAX_PROFILE_MOVES)
            return false;
        m.valid = true;
        profile[profileCount++] = m;
        return true;
    }

    private void ProfileCompact()
    {
        if (emittedProfileCount <= 0)
            return;
        int dropCount = emittedProfileCount;
        int keepCount = profileCount - dropCount;
        if (keepCount > 0)
        {
            Array.Copy(profile, dropCount, profile, 0, keepCount);
        }
        profileCount = keepCount;
        emittedProfileCount = 0;
        trimResumeIndex -= dropCount;
        if (trimResumeIndex < 0)
            trimResumeIndex = 0;
    }

    private bool TrimAndMergePendingProfile()
    {
        // TODO: implement trimming and merging logic if needed
        return true;
    }

    private bool EmitCompProfile(int holdBackCount, bool flushAll)
    {
        if (emittedProfileCount < 0)
            emittedProfileCount = 0;
        int emitLimit = profileCount;
        if (!flushAll)
        {
            emitLimit = profileCount - holdBackCount;
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
            // TODO: implement emit_move_as_gcode if needed
        }
        emittedProfileCount = emitLimit;
        return true;
    }

    private bool ProcessOneGcodeLine(ScanLine s)
    {
        Move2D mv = SimpleScan.InterpretMove(s, ref modalState);
        if (s.sawG41 || s.sawG42)
            cc.SetComp(modalState.comp);
        if (mv.type == MotionType.MOT_EMPTY)
        {
            if (s.sawG40)
            {
                cc.SetComp(CompSide.COMP_OFF);
                cc.Flush();
                Move2D outputMove;
                while (cc.PopOut(outputMove))
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
        Move2D outputMove;
        while (cc.PopOut(outputMove))
        {
            if (!ProfilePush(outputMove))
            {
                ReportError("(profile buffer full)", CompError.CE_ERROR);
                return false;
            }
        }
        return true;
    }
}
