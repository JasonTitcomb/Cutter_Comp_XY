

/*
 * cc_main.h
 * Jason Titcomb 2026
 * MIT License – see LICENSE file in repository root
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "cc_simple_scan.h"
#include "cc_processor.h"

class CcMainRunner
{
private:
    /// These values can be set to 1 to disable lookahead
    static constexpr int MAX_LOOKAHEAD = 20;
    static constexpr int TARGET_BATCH_EMIT_MOVES = 40;
    static constexpr int PROFILE_BURST_MARGIN = 2;

    static constexpr int TRIM_OVERLAP_MOVES = MAX_LOOKAHEAD + 2;
    static constexpr int EMIT_HOLDBACK = TRIM_OVERLAP_MOVES;
    static constexpr int MIN_PENDING_BEFORE_BATCH = EMIT_HOLDBACK + TARGET_BATCH_EMIT_MOVES;
    static constexpr int MAX_PROFILE_MOVES = MIN_PENDING_BEFORE_BATCH + PROFILE_BURST_MARGIN;

    static_assert(TARGET_BATCH_EMIT_MOVES > 0, "TARGET_BATCH_EMIT_MOVES must be positive");
    static_assert(EMIT_HOLDBACK >= TRIM_OVERLAP_MOVES, "EMIT_HOLDBACK must preserve trim overlap across batches");
    static_assert(MAX_PROFILE_MOVES > MIN_PENDING_BEFORE_BATCH, "MAX_PROFILE_MOVES must exceed batch threshold");
  
    CcMainOptions options_{};
    CcOutputCB outputCB_ = nullptr;
    CcErrorCB errorCB_ = nullptr;
    CcStartCompCB startCompCB_ = nullptr;

    ModalState modalState_{};
    CutterComp2D cc_{};
    AABB2 aabbs[MAX_PROFILE_MOVES]{};
    Move2D profile_[MAX_PROFILE_MOVES]{};
    int profileCount_ = 0;

    int emittedProfileCount_ = 0;
    int trimResumeIndex_ = 0;
    bool sawCompStart_ = false;
    bool sawG40_ = false;
    bool compClosed_ = false;
    bool runActive_ = false;
    bool globalTrim_ = false;
    bool emitComments_ = true;
    bool inchMode_ = true;

public:
    bool begin(const CcMainOptions &options)
    {
        options_ = options;
        runActive_ = true;

        modalState_ = ModalState{};
        modalState_.planeXY = true;
        modalState_.absoluteMode = true;
        modalState_.motionG = 0;
        modalState_.comp = COMP_OFF;
        modalState_.feed = 0;
        modalState_.speed = 0;
        modalState_.pos = v2(0, 0);
        modalState_.N_number = 0;
        modalState_.T_Register = 0;
        modalState_.D_Register = 0;
        modalState_.inchMode = true;
        inchMode_ = true;

        cc_.setOptions(options_);

        outputCB_ = options_.callbacks.output;
        errorCB_ = options_.callbacks.error;
        startCompCB_ = options_.callbacks.startComp;

        globalTrim_ = options_.globalTrimCrossing;
        emittedProfileCount_ = 0;
        trimResumeIndex_ = 0;
        sawCompStart_ = false;
        sawG40_ = false;
        compClosed_ = false;
        emitComments_ = options_.emitStatusComments;
        profile_reset();
        return true;
    }

    void setToolRadius(float r)
    {
        cc_.setToolRadius(r);
    }

    bool processLine(const char *line)
    {
        if (!runActive_ || !line)
            return false;

        char peekClean[160];
        strip_comments(line, peekClean, sizeof(peekClean));
        if (peekClean[0] == 0)
            return true;

        ScanLine scanLn;
        scan_line(peekClean, scanLn);

        const bool compIsOff = (cc_.comp_state == COMP_OFF);
        const bool canEmitRaw = compIsOff && !scanLn.sawG41 && !scanLn.sawG42 && (!sawCompStart_ || compClosed_);
        const bool entersComp = compIsOff && (scanLn.sawG41 || scanLn.sawG42) && !compClosed_;
        if (entersComp)
        {
            sawCompStart_ = true;
            emit_status("(COMP ON)\n");
            if (startCompCB_)
            {
                startCompCB_(modalState_.T_Register, modalState_.D_Register);
            }
        }

        if (canEmitRaw)
        {
            if (!process_raw_gcode_line(line, scanLn))
            {
                runActive_ = false;
                return false;
            }

            return true;
        }

        if (!process_one_gcode_line(scanLn))
        {
            runActive_ = false;
            return false;
        }

        if (cc_.comp_state != COMP_OFF)
        {
            const int pendingProfileWindow = profileCount_ - emittedProfileCount_;
            if (pendingProfileWindow >= MIN_PENDING_BEFORE_BATCH)
            {
                if (!trim_and_merge_pending_profile())
                {
                    report_error("(trim failed)", CE_ERROR);
                    runActive_ = false;
                    return false;
                }

                if (!emit_comp_profile(EMIT_HOLDBACK, false))
                {
                    report_error("(emit failed)", CE_ERROR);
                    runActive_ = false;
                    return false;
                }

                profile_compact();
                emit_status("(BATCH)\n");
            }
        }

        if (cc_.comp_state == COMP_OFF && sawCompStart_ && !compClosed_)
        {
            sawG40_ = true;
            compClosed_ = true;

            if (!trim_and_merge_pending_profile())
            {
                report_error("(trim failed)", CE_ERROR);
                runActive_ = false;
                return false;
            }

            if (!emit_comp_profile(0, true))
            {
                report_error("(emit failed)", CE_ERROR);
                runActive_ = false;
                return false;
            }

            profile_compact();
            emit_status("(COMP OFF)\n");
        }

        return true;
    }

    bool finish()
    {
        if (!runActive_)
            return false;

        if (sawCompStart_ && !compClosed_)
        {
            if (!flush_pipeline())
            {
                runActive_ = false;
                return false;
            }

            if (!trim_and_merge_pending_profile())
            {
                report_error("(final trim failed)", CE_ERROR);
                runActive_ = false;
                return false;
            }

            if (!emit_comp_profile(0, true))
            {
                report_error("(final emit failed)", CE_ERROR);
                runActive_ = false;
                return false;
            }

            profile_compact();
        }

        if (!sawG40_)
            report_error("(warning: reached EOF before G40)", CE_ERROR);

        runActive_ = false;
        return true;
    }

    bool runProgram(const char *const *program, int lineCount, const CcMainOptions &options)
    {
        if (!program || lineCount <= 0)
            return true;

        if (!begin(options))
            return false;

        for (int i = 0; i < lineCount; ++i)
        {
            if (!processLine(program[i]))
                return false;
        }

        return finish();
    }

private:
    void onCompError(CompError err)
    {
        char msg[64];
        snprintf(msg, sizeof(msg), "CompError %u N%u", (unsigned)err, (unsigned)modalState_.N_number);
        report_error(msg, err);
    }

    void emit_status(const char *text)
    {
        if (emitComments_ && outputCB_ && text)
            outputCB_(text, strlen(text));
    }

    void emit_raw_line(const char *text)
    {
        if (!outputCB_ || !text || text[0] == 0)
            return;

        const size_t len = strlen(text);
        outputCB_(text, len);

        if (text[len - 1] != '\n')
            outputCB_("\n", 1);
    }

    void report_error(const char *message, CompError err)
    {
        if (errorCB_)
            errorCB_(message, err, modalState_.N_number);
    }

    void profile_reset()
    {
        profileCount_ = 0;
    }

    bool profile_push(const Move2D &m)
    {
        if (profileCount_ >= MAX_PROFILE_MOVES)
            return false;
        Move2D t = m;
        t.valid = true;
        profile_[profileCount_++] = t;
        return true;
    }

    void profile_compact()
    {
        if (emittedProfileCount_ <= 0)
            return;

        const int dropCount = emittedProfileCount_;
        const int keepCount = profileCount_ - dropCount;

        if (keepCount > 0)
            memmove(profile_, profile_ + dropCount, (size_t)keepCount * sizeof(Move2D));

        profileCount_ = keepCount;
        emittedProfileCount_ = 0;

        trimResumeIndex_ -= dropCount;
        if (trimResumeIndex_ < 0)
            trimResumeIndex_ = 0;
    }

    static void trim_trailing_zeros(char *s)
    {
        char *dot = strchr(s, '.');
        if (!dot)
            return;

        char *end = s + strlen(s) - 1;
        while (end > dot && *end == '0')
        {
            *end = '\0';
            --end;
        }

        if (end == dot)
            *end = '\0';
    }

    static int append_text(char *line, size_t cap, int n, const char *txt)
    {
        if (n < 0 || n >= (int)cap)
            return n;
        int wrote = snprintf(line + n, cap - (size_t)n, "%s", txt);
        if (wrote < 0)
            return n;
        return n + wrote;
    }

    static int append_coord(char *line, size_t cap, int n, const char *axis, float value, int digits)
    {
        char num[32];
        int wrote = snprintf(num, sizeof(num), "%.*f", digits, value);
        if (wrote < 0)
            return n;
        trim_trailing_zeros(num);

        n = append_text(line, cap, n, " ");
        n = append_text(line, cap, n, axis);
        n = append_text(line, cap, n, num);
        return n;
    }

    void emit_move_as_gcode(const Move2D &m)
    {
        char line[160];
        int n = 0;
        const int posDigits = inchMode_ ? 4 : 3;
        static bool hasLastFeed = false;
        static float lastFeed = 0.0f;

        // Emit sequence number if present
        if (m.seqNum != 0)
        {
            n = snprintf(line, sizeof(line), "N%u ", (unsigned)m.seqNum);
        }

        if (m.type == MOT_LINE || m.type == MOT_RAPID)
        {
            n += snprintf(line + n, sizeof(line) - (size_t)n, "%s", (m.type == MOT_RAPID) ? "G0" : "G1");

            float dx = m.p_1.x - m.p_0.x;
            float dy = m.p_1.y - m.p_0.y;
            float dz = m.z_1 - m.z_0;
            bool isAbs = modalState_.absoluteMode;

            if (m.hasXY)
            {
                if (isAbs)
                {
                    n = append_coord(line, sizeof(line), n, "X", m.p_1.x, posDigits);
                    n = append_coord(line, sizeof(line), n, "Y", m.p_1.y, posDigits);
                }
                else
                {
                    n = append_coord(line, sizeof(line), n, "X", dx, posDigits);
                    n = append_coord(line, sizeof(line), n, "Y", dy, posDigits);
                }
            }

            if (m.feed > 0.0f && (!hasLastFeed || m.feed != lastFeed))
            {
                n = append_coord(line, sizeof(line), n, "F", m.feed, posDigits);
                lastFeed = m.feed;
                hasLastFeed = true;
            }

            if (m.hasZ)
                n = append_coord(line, sizeof(line), n, "Z", isAbs ? m.z_1 : dz, posDigits);

            n = append_text(line, sizeof(line), n, "\n");
        }
        else if (m.type == MOT_ARC)
        {
            Vec2 dCenter = m.center - m.p_0;
            n += snprintf(line + n, sizeof(line) - (size_t)n, "%s", (m.arcDir == ARC_CW) ? "G2" : "G3");

            float dx = m.p_1.x - m.p_0.x;
            float dy = m.p_1.y - m.p_0.y;
            float dz = m.z_1 - m.z_0;
            bool isAbs = modalState_.absoluteMode;

            n = append_coord(line, sizeof(line), n, "X", isAbs ? m.p_1.x : dx, posDigits);
            n = append_coord(line, sizeof(line), n, "Y", isAbs ? m.p_1.y : dy, posDigits);
            n = append_coord(line, sizeof(line), n, "I", dCenter.x, posDigits);
            n = append_coord(line, sizeof(line), n, "J", dCenter.y, posDigits);

            if (m.feed > 0.0f && (!hasLastFeed || m.feed != lastFeed))
            {
                n = append_coord(line, sizeof(line), n, "F", m.feed, posDigits);
                lastFeed = m.feed;
                hasLastFeed = true;
            }

            if (m.hasZ)
                n = append_coord(line, sizeof(line), n, "Z", isAbs ? m.z_1 : dz, posDigits);

            n = append_text(line, sizeof(line), n, "\n");
        }

        if (outputCB_ && n > 0)
            outputCB_(line, n);
    }

    bool emit_comp_profile(int holdBackCount, bool flushAll)
    {
        if (emittedProfileCount_ < 0)
            emittedProfileCount_ = 0;

        int emitLimit = profileCount_;
        if (!flushAll)
        {
            emitLimit = profileCount_ - holdBackCount;
            if (emitLimit < 0)
                emitLimit = 0;
        }

        if (emittedProfileCount_ >= emitLimit)
            return true;

        for (int i = emittedProfileCount_; i < emitLimit; ++i)
        {
            const Move2D &m = profile_[i];
            if (!m.valid || m.type == MOT_EMPTY)
                continue;
            emit_move_as_gcode(m);
        }

        emittedProfileCount_ = emitLimit;
        return true;
    }

    bool process_one_gcode_line(ScanLine s)
    {
        Move2D mv = interpret_move(s, modalState_);

        if (s.sawG41 || s.sawG42)
            cc_.setComp(modalState_.comp);

        if (mv.type == MOT_EMPTY)
        {
            if (s.sawG40)
            {
                cc_.setComp(COMP_OFF);
                cc_.flush();
                Move2D out;
                while (cc_.popOut(out))
                {
                    if (!profile_push(out))
                    {
                        report_error("(profile buffer full)", CE_ERROR);
                        return false;
                    }
                }
            }
            return true;
        }

        if (!cc_.pushIn(mv))
        {
            report_error("(comp input buffer full)", CE_ERROR);
            return false;
        }

        if (!cc_.process())
        {
            report_error("(comp processing failed!)", CE_ERROR);
            return false;
        }

        Move2D out;
        while (cc_.popOut(out))
        {
            if (!profile_push(out))
            {
                report_error("(profile buffer full)", CE_ERROR);
                return false;
            }
        }

        if (s.sawG40)
        {
            cc_.flush();
            cc_.setComp(COMP_OFF);

            while (cc_.popOut(out))
            {
                if (!profile_push(out))
                {
                    report_error("(profile buffer full)", CE_ERROR);
                    return false;
                }
            }
        }

        return true;
    }

    bool process_raw_gcode_line(const char *rawLine, ScanLine s)
    {
        (void)interpret_move(s, modalState_);
        emit_raw_line(rawLine);
        return true;
    }

    bool flush_pipeline()
    {
        cc_.flush();
        Move2D out;
        while (cc_.popOut(out))
        {
            if (!profile_push(out))
            {
                report_error("(profile buffer full)", CE_ERROR);
                return false;
            }
        }
        return true;
    }

    bool trim_and_merge_pending_profile()
    {
        if (!globalTrim_)
            return true;

        const int currentProfileCount = profileCount_;
        int trimStart = trimResumeIndex_;
        if (trimStart < emittedProfileCount_)
            trimStart = emittedProfileCount_;

        if (trimStart >= currentProfileCount)
            return true;

        if (options_.globalTrimCrossing)
        {
            int srcIdx = trimStart;
            int retTargetIdx = -1;
            if (!cc_.trimCrossingElements(profile_, aabbs, srcIdx, currentProfileCount, MAX_LOOKAHEAD, retTargetIdx))
                return false;

            int mergeStart = trimStart;
            if (mergeStart > emittedProfileCount_)
                mergeStart -= 1;

            cc_.merge_all_colinear(profile_ + mergeStart, currentProfileCount - mergeStart);
        }

        int nextTrimStart = currentProfileCount - TRIM_OVERLAP_MOVES;
        if (nextTrimStart < emittedProfileCount_)
            nextTrimStart = emittedProfileCount_;
        if (nextTrimStart < 0)
            nextTrimStart = 0;
        trimResumeIndex_ = nextTrimStart;
        return true;
    }
};
