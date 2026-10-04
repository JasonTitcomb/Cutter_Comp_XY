#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>

#include "../src/cc_main.h"
#include "../host/compare.h"

static std::string emitted_gcode;
static bool saw_error;

static void capture_gcode(const char *text, size_t length)
{
    emitted_gcode.append(text, length);
}

static void capture_error(const char *message, CompError error, uint32_t lineNumber)
{
    std::fprintf(stderr, "N%u: %s (%u)\n", (unsigned)lineNumber,
                 message ? message : "comp error", (unsigned)error);
    saw_error = true;
}

static bool check_cancel(CcMainRunner &runner, const char *cancelLine,
                         const char *nextLine, int expectedMotion, bool restore,
                         bool comments, bool incremental, bool trim)
{
    CcMainOptions options;
    options.toolRadius = 0.1f;
    options.globalTrimCrossing = trim;
    options.globalMerge = false;
    options.emitStatusComments = comments;
    options.callbacks.output = capture_gcode;
    options.callbacks.error = capture_error;
    emitted_gcode.clear();
    saw_error = false;
    if (!runner.begin(options))
        return false;

    const char *program[] = {
        incremental ? "G20 G91 G0 X-2 Y0" : "G20 G90 G0 X-2 Y0",
        incremental ? "G1 G41 X2 Y0 F10" : "G1 G41 X0 Y0 F10",
        "G3 X0 Y2 I0 J1",
        cancelLine
    };
    for (const char *line : program)
    {
        runner.incrementLineNumber();
        if (!runner.processLine(line))
            return false;
    }

    const std::string beforeNext = emitted_gcode;
    runner.incrementLineNumber();
    if (!runner.processLine(nextLine) || !runner.finish() || saw_error)
        return false;

    if (emitted_gcode != beforeNext + nextLine + "\n")
    {
        puts("post-comp raw line was changed");
        return false;
    }
    const std::string restoredWord = "G" + std::to_string(expectedMotion) + "\n";
    const std::string expectedTail = (comments ? "(COMP OFF)\n" : "") +
                                     (restore ? restoredWord : "");
    if (beforeNext.size() < expectedTail.size() ||
        beforeNext.compare(beforeNext.size() - expectedTail.size(),
                           expectedTail.size(), expectedTail) != 0)
    {
        std::fprintf(stderr, "missing or unexpected motion restore for %s:\n%s",
                     cancelLine, beforeNext.c_str());
        return false;
    }

    ModalState outputState;
    int standaloneMotionCount = 0;
    size_t start = 0;
    while (start < beforeNext.size())
    {
        const size_t end = beforeNext.find('\n', start);
        const std::string line = beforeNext.substr(start, end - start);
        if (line == "G0" || line == "G1" || line == "G2" || line == "G3")
            standaloneMotionCount++;
        char clean[160];
        strip_comments(line.c_str(), clean, sizeof(clean));
        ScanLine scan;
        scan_line(clean, scan);
        (void)interpret_move(scan, outputState);
        start = end == std::string::npos ? beforeNext.size() : end + 1;
    }
    if (outputState.motionG != expectedMotion ||
        standaloneMotionCount != (restore ? 1 : 0))
    {
        puts("output modal motion does not match input at comp cancellation");
        return false;
    }
    return true;
}

static bool run_arc_turns_program(const char *arcLine, bool merge, bool trim, std::vector<float> &arcEndZ)
{
    CcMainRunner runner;
    CcMainOptions options;
    options.toolRadius = 0.125f;
    options.globalTrimCrossing = trim;
    options.globalMerge = merge;
    options.emitStatusComments = false;
    options.callbacks.output = capture_gcode;
    options.callbacks.error = capture_error;
    emitted_gcode.clear();
    saw_error = false;
    arcEndZ.clear();
    if (!runner.begin(options))
        return false;

    const char *program[] = {
        "G20 G90 G0 X0 Y0",
        "G1 Z-0.1 F10",
        "G1 G41 X1 Y0 F20",
        arcLine,
        "G1 G40 X0 Y0"};
    for (const char *line : program)
    {
        runner.incrementLineNumber();
        if (!runner.processLine(line))
            return false;
    }
    if (!runner.finish() || saw_error)
        return false;

    ModalState outputState;
    size_t start = 0;
    while (start < emitted_gcode.size())
    {
        const size_t end = emitted_gcode.find('\n', start);
        const std::string line = emitted_gcode.substr(start, end - start);
        char clean[160];
        strip_comments(line.c_str(), clean, sizeof(clean));
        ScanLine scan;
        scan_line(clean, scan);
        Move2D mv = interpret_move(scan, outputState);
        if (mv.type == MOT_ARC)
        {
            if (fabsf(mv.radius - 0.875f) > 0.0002f)
                return false;
            arcEndZ.push_back(mv.z_1);
        }
        start = end == std::string::npos ? emitted_gcode.size() : end + 1;
    }
    return true;
}

static bool check_arc_turns(bool merge, bool trim)
{
    std::vector<float> arcEndZ;
    const float expectedZ[] = {-0.23333f, -0.36667f, -0.4f};

    if (!run_arc_turns_program("G3 X0 Y1 Z-0.4 I-1 J0 P3", merge, trim, arcEndZ) || arcEndZ.size() != 3)
    {
        puts("G3 P3 did not produce two full turns and the final arc");
        return false;
    }
    for (size_t i = 0; i < arcEndZ.size(); ++i)
    {
        if (fabsf(arcEndZ[i] - expectedZ[i]) > 0.0002f)
        {
            puts("G3 P3 Z was not spread over the total angular travel");
            return false;
        }
    }

    if (!run_arc_turns_program("G3 X1 Y0 Z-2.1 I-1 J0 P40", merge, trim, arcEndZ) || arcEndZ.size() != 40 ||
        fabsf(arcEndZ.back() + 2.1f) > 0.0002f)
    {
        puts("G3 P40 lost turns or overflowed the profile buffer");
        return false;
    }

    if (run_arc_turns_program("G3 X0 Y1 I-1 J0 P2.5", merge, trim, arcEndZ) || !saw_error)
    {
        puts("non-integer arc P was not rejected");
        return false;
    }
    return true;
}

static bool check_physical_tolerances()
{
    CutterComp2D processor;
    for (int inch = 0; inch < 2; ++inch)
    {
        processor.setUnits(inch ? UNITS_INCH : UNITS_MM);
        const float scale = inch ? 1.0f / 25.4f : 1.0f;
        Move2D move;
        move.type = MOT_LINE;
        move.p_1 = v2(0.00006f * scale, 0.00006f * scale);
        move.z_1 = 0.00009f * scale;
        update_vectors(move);
        if (move.hasXY || move.hasZ || !is_near(v2(0, 0), move.p_1))
            return false;
        move.p_1 = v2(0.00008f * scale, 0.00008f * scale);
        move.z_1 = 0.00011f * scale;
        update_vectors(move);
        if (!move.hasXY || !move.hasZ || is_near(v2(0, 0), move.p_1) ||
            is_near(v2(0, 0), v2(0, 0.0014f * scale)))
            return false;
        if (!angleOnSweepCCW(0.0f, 1.0f, 1.0f + 0.5f * ANGLE_EPS) ||
            angleOnSweepCCW(0.0f, 1.0f, 1.0f + 2.0f * ANGLE_EPS))
            return false;
    }
    processor.setUnits(UNITS_MM);
    return true;
}

static bool check_compare_alignment()
{
    std::vector<Move2D> full(3);
    for (size_t index = 0; index < full.size(); ++index)
    {
        full[index].type = MOT_LINE;
        full[index].p_0 = v2((float)index, 0);
        full[index].p_1 = v2((float)index + 1, 0);
    }
    std::vector<Move2D> simple = full;
    Move2D connector;
    connector.type = MOT_LINE;
    connector.p_0 = full[1].p_0;
    connector.p_1 = connector.p_0 + v2(0, 0.0014f);
    simple.insert(simple.begin() + 1, connector);
    std::vector<ComparePair> pairs = align_compare_moves(full, simple, 0.005f);
    if (pairs.size() != 4 || pairs[1].fullIndex != full.size() || pairs[1].simpleIndex != 1 ||
        pairs[2].fullIndex != 1 || pairs[2].simpleIndex != 2 ||
        pairs[3].fullIndex != 2 || pairs[3].simpleIndex != 3)
        return false;
    pairs = align_compare_moves(simple, full, 0.005f);
    if (pairs.size() != 4 || pairs[1].fullIndex != 1 || pairs[1].simpleIndex != full.size())
        return false;
    simple = full;
    simple[1].p_1.y = 0.1f;
    pairs = align_compare_moves(full, simple, 0.005f);
    size_t differences = 0;
    for (const ComparePair &pair : pairs)
    {
        if (pair.fullIndex == full.size() || pair.simpleIndex == simple.size() ||
            !compare_moves_match(full[pair.fullIndex], simple[pair.simpleIndex], 0.005f))
            ++differences;
    }
    if (differences != 1 || pairs.back().fullIndex != 2 || pairs.back().simpleIndex != 2)
        return false;
    simple = full;
    simple.push_back(connector);
    pairs = align_compare_moves(full, simple, 0.005f);
    if (pairs.size() != 4 || pairs.back().fullIndex != full.size() || pairs.back().simpleIndex != 3)
        return false;
    Move2D clockwise;
    clockwise.type = MOT_ARC;
    Move2D counterclockwise = clockwise;
    counterclockwise.arcDir = ARC_CCW;
    return !compare_moves_match(clockwise, counterclockwise, 0.005f);
}

static bool check_small_fillet_program()
{
    const std::string sourcePath = __FILE__;
    const size_t separator = sourcePath.find_last_of("/\\");
    const std::string directory = sourcePath.substr(0, separator + 1);
#ifdef _WIN32
    const std::string inputPath = directory + "..\\data\\TortureTestSmallFilletsG91.nc";
#else
    const std::string inputPath = directory + "../data/TortureTestSmallFilletsG91.nc";
#endif
    std::ifstream input(inputPath);
    if (!input)
    {
        std::fprintf(stderr, "Failed to open regression input: %s\n", inputPath.c_str());
        return false;
    }
    CcMainRunner runner;
    CcMainOptions options;
    options.toolRadius = 0.03f;
    options.globalMerge = false;
    options.callbacks.output = capture_gcode;
    options.callbacks.error = capture_error;
    emitted_gcode.clear();
    saw_error = false;
    if (!runner.begin(options))
        return false;
    std::string line;
    while (std::getline(input, line))
    {
        runner.incrementLineNumber();
        if (!runner.processLine(line.c_str()))
            return false;
    }
    if (input.bad() || !runner.finish() || saw_error ||
        emitted_gcode.find("N56 G1 X0 Y0.0001\nN57 G1") == std::string::npos)
    {
        puts("small-fillet program lost the arc-to-line connector");
        return false;
    }
    return true;
}

int main(void)
{
    if (!check_physical_tolerances() || !check_compare_alignment() || !check_small_fillet_program())
    {
        puts("physical tolerance or comparison alignment checks failed");
        return 1;
    }
    for (int merge = 0; merge < 2; ++merge)
    {
        for (int trim = 0; trim < 2; ++trim)
        {
            if (!check_arc_turns(merge != 0, trim != 0))
                return 1;
        }
    }
    puts("arc P turn checks passed");

    CcMainRunner runner;
    for (int trim = 0; trim < 2; ++trim)
    {
        for (int comments = 0; comments < 2; ++comments)
        {
            for (int incremental = 0; incremental < 2; ++incremental)
            {
                if (!check_cancel(runner, "G1 G40", "N73 Z0.84", 1, true,
                                  comments != 0, incremental != 0, trim != 0) ||
                    !check_cancel(runner, "G0 G40", "N73 Z0.84", 0, true,
                                  comments != 0, incremental != 0, trim != 0) ||
                    !check_cancel(runner, "G2 G40",
                                  incremental ? "N73 X0 Y-2 I0 J-1" : "N73 X0 Y0 I0 J-1", 2, true,
                                  comments != 0, incremental != 0, trim != 0) ||
                    !check_cancel(runner, "G40",
                                  incremental ? "N73 X0 Y-2 I0 J-1" : "N73 X0 Y0 I0 J-1", 3, false,
                                  comments != 0, incremental != 0, trim != 0) ||
                    !check_cancel(runner, "G1 G40 X-2 Y2", "N73 Z0.84", 1, false,
                                  comments != 0, incremental != 0, trim != 0))
                    return 1;
            }
        }
    }
    puts("runner modal cancellation checks passed");
    return 0;
}
