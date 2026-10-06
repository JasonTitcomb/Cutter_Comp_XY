#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <cmath>
#include <string>

#include "../src/cc_main.h"

static int radius_error_count;
static uint32_t radius_error_line;
static std::string emitted_gcode;
static int inversion_error_count;
static uint32_t inversion_error_line;
static int gap_error_count;
static uint32_t gap_error_line;

static void capture_gap_error(const char *message, CompError error, uint32_t lineNumber)
{
    (void)message;
    if (error == CE_UNRESOLVED_GAP)
    {
        gap_error_count++;
        gap_error_line = lineNumber;
    }
}

static void capture_inversion_error(const char *message, CompError error, uint32_t lineNumber)
{
    (void)message;
    if (error == CE_INVALID_MOVE)
    {
        inversion_error_count++;
        inversion_error_line = lineNumber;
    }
}

static void capture_error(const char *message, CompError error, uint32_t lineNumber)
{
    (void)message;
    if (error == CE_ARC_RADIUS_MISMATCH)
    {
        radius_error_count++;
        radius_error_line = lineNumber;
    }
}

static void capture_gcode(const char *text, size_t length)
{
    emitted_gcode.append(text, length);
}

static Move2D roll_arc(float centerY, float radius, bool first)
{
    Move2D move;
    move.type = MOT_ARC;
    move.compMode = CM_STEADY;
    move.arcDir = ARC_CW;
    move.center = v2(0.0f, centerY);
    move.radius = radius;
    move.p_0 = first ? v2(-radius, centerY) : v2(0.0f, centerY - radius);
    move.p_1 = first ? v2(0.0f, centerY + radius) : v2(-radius, centerY);
    update_vectors(move);
    return move;
}

static Move2D line_move(Vec2 start, Vec2 end)
{
    Move2D move;
    move.type = MOT_LINE;
    move.compMode = CM_STEADY;
    move.p_0 = start;
    move.p_1 = end;
    update_vectors(move);
    return move;
}

static bool check_shared_lead_endpoint(float scale)
{
    set_physical_units(scale == 1.0f);
    const float toolRadius = 0.0010225861129f * 0.5f * scale;
    const Vec2 entry = v2(0.25f * scale - toolRadius, 0.55f * scale);
    const Vec2 exit = v2(0.25f * scale + 0.21693046f * toolRadius,
                         0.55f * scale - 0.97618706f * toolRadius);
    const Vec2 origin = v2(0.0f, 0.0f);
    for (int reverseFirst = 0; reverseFirst < 2; ++reverseFirst)
    {
        for (int reverseSecond = 0; reverseSecond < 2; ++reverseSecond)
        {
            Move2D first = line_move(reverseFirst ? entry : origin,
                                     reverseFirst ? origin : entry);
            Move2D second = line_move(reverseSecond ? origin : exit,
                                      reverseSecond ? exit : origin);
            first.type = MOT_RAPID;
            second.type = MOT_RAPID;
            Vec2 tip;
            if (intersectLineLine(first, second, tip) != IT_INTERSECT ||
                !same_point(tip, origin) ||
                intersectLineLine(second, first, tip) != IT_INTERSECT ||
                !same_point(tip, origin))
                return false;
        }
    }

    Move2D first = line_move(origin, entry);
    Move2D crossing = line_move(v2(0.0f, 0.275f * scale),
                                v2(0.5f * scale, 0.275f * scale));
    Vec2 tip;
    if (intersectLineLine(first, crossing, tip) != IT_INTERSECT ||
        !is_near(tip, entry * 0.5f))
        return false;

    Move2D parallel = line_move(entry, entry * 2.0f);
    if (intersectLineLine(first, parallel, tip) != IT_NONE)
        return false;

    Move2D collapsed = line_move(entry, entry);
    if (intersectLineLine(first, collapsed, tip) != IT_NONE)
        return false;

    collapsed.junctionOnly = true;
    collapsed.startDir = v2(1.0f, 0.0f);
    collapsed.endDir = collapsed.startDir;
    return intersectLineLine(first, collapsed, tip) == IT_INTERSECT &&
           same_point(tip, entry);
}

static bool check_early_inversion(float scale, bool reversePrevious, bool trimCrossings)
{
    CutterComp2D processor;
    CcMainOptions options;
    options.toolRadius = scale;
    options.globalTrimCrossing = trimCrossings;
    options.callbacks.error = capture_inversion_error;
    processor.setOptions(options);
    processor.setUnits(scale == 1.0f ? UNITS_INCH : UNITS_MM);
    processor.compSide = COMP_LEFT;
    processor.compMode = CM_STEADY;
    inversion_error_count = 0;
    inversion_error_line = 0;
    Move2D previous = reversePrevious ? line_move(v2(0.0f, 0.0f), v2(0.5f * scale, 0.0f))
                                      : line_move(v2(-3.0f * scale, 0.0f), v2(0.0f, 0.0f));
    Move2D current = reversePrevious ? line_move(previous.p_1, v2(0.5f * scale, 3.0f * scale))
                                     : line_move(previous.p_1, v2(0.0f, 0.5f * scale));
    previous.lineNum = 10;
    current.lineNum = 20;
    if (!processor.pushIn(previous) || !processor.process() || !processor.pushIn(current))
        return false;

    const bool processed = processor.process();
    if (trimCrossings)
        return processed && inversion_error_count == 0;

    Move2D output;
    if (processed || inversion_error_count != 1 ||
        inversion_error_line != (reversePrevious ? 10u : 20u) ||
        processor.popOut(output))
        return false;

    processor.flush();
    return !processor.process() && !processor.popOut(output) && inversion_error_count == 1;
}

static bool check_crossing_arcs(float scale)
{
    const float radius = 0.0051155f * scale;
    Move2D first = roll_arc(0.085999f * scale, radius, true);
    Move2D last = roll_arc(0.093601f * scale, radius, false);
    Vec2 tip1, tip2;
    int count = 0;
    if (intersectCircleCircle(first, last, tip1, tip2, count) != IT_INTERSECT || count != 2)
    {
        puts("crossing circles were classified as tangent");
        return false;
    }

    const ArcAngles firstAngles = precomputeArcAngles(first);
    const ArcAngles lastAngles = precomputeArcAngles(last);
    const bool onFirstTip = pointOnArcCached(first, tip1, firstAngles) &&
                            pointOnArcCached(last, tip1, lastAngles);
    const bool onSecondTip = pointOnArcCached(first, tip2, firstAngles) &&
                             pointOnArcCached(last, tip2, lastAngles);
    if (onFirstTip == onSecondTip)
    {
        puts("expected exactly one crossing on both finite roll arcs");
        return false;
    }
    const Vec2 expectedTip = onFirstTip ? tip1 : tip2;

    Move2D moves[5];
    moves[0] = first;
    moves[1] = line_move(first.p_1, v2(0.007627f * scale, first.p_1.y));
    moves[2] = line_move(moves[1].p_1, v2(0.0025115f * scale, last.p_0.y));
    moves[3] = line_move(moves[2].p_1, last.p_0);
    moves[4] = last;
    AABB2 bounds[5];
    CutterComp2D processor;
    CcMainOptions options;
    options.toolRadius = radius;
    processor.setOptions(options);
    processor.setUnits(scale == 1.0f ? UNITS_INCH : UNITS_MM);
    int sourceIndex = 0;
    if (!processor.trimCrossingElements(moves, bounds, sourceIndex, 5, 8) ||
        !is_near(moves[0].p_1, expectedTip) ||
        !is_near(moves[4].p_0, expectedTip) ||
        moves[1].valid || moves[2].valid || moves[3].valid)
    {
        puts("arc crossing was not trimmed or intervening moves survived");
        return false;
    }
    return true;
}

static bool check_circle_cases(void)
{
    Move2D first = roll_arc(0.0f, 1.0f, true);
    Move2D second = roll_arc(2.0f, 1.0f, false);
    Vec2 tip1, tip2;
    int count;
    if (intersectCircleCircle(first, second, tip1, tip2, count) != IT_TANGENT ||
        count != 1 || !is_near(tip1, v2(0.0f, 1.0f)))
        return false;
    second.center.y = 3.0f;
    if (intersectCircleCircle(first, second, tip1, tip2, count) != IT_NONE || count != 0)
        return false;
    second.center.y = 0.25f;
    second.radius = 0.5f;
    if (intersectCircleCircle(first, second, tip1, tip2, count) != IT_NONE || count != 0)
        return false;
    second.center.y = 0.5f;
    if (intersectCircleCircle(first, second, tip1, tip2, count) != IT_TANGENT || count != 1)
        return false;
    second.center = first.center;
    if (intersectCircleCircle(first, second, tip1, tip2, count) != IT_NONE || count != 0)
        return false;
    second.center.y = 1.0f;
    second.radius = -1.0f;
    return intersectCircleCircle(first, second, tip1, tip2, count) == IT_INTERSECT && count == 2;
}

static bool check_arc_radius_validation(float scale, bool trimCrossings)
{
    CutterComp2D processor;
    CcMainOptions options;
    options.globalTrimCrossing = trimCrossings;
    options.callbacks.error = capture_error;
    processor.setOptions(options);
    processor.setUnits(scale == 1.0f ? UNITS_INCH : UNITS_MM);
    radius_error_count = 0;
    radius_error_line = 0;

    Move2D arc;
    arc.type = MOT_ARC;
    arc.arcDir = ARC_CCW;
    arc.lineNum = 15;
    arc.p_0 = v2(0.0f, 5.55408f * scale);
    arc.center = v2(0.0f, (5.55408f + 0.51156f) * scale);
    arc.radius = 0.51156f * scale;
    arc.p_1 = v2(0.0f, (5.55408f + 2.0f * 0.51156f + 0.00025f) * scale);
    if (!processor.pushIn(arc) || !processor.process() || radius_error_count != 0)
    {
        puts("within-tolerance radius mismatch was rejected");
        return false;
    }
    processor.flush();
    Move2D output;
    if (!processor.popOut(output) || !output.valid)
        return false;

    arc.p_1 = v2(0.0f, 6.57804f * scale);
    if (!processor.pushIn(arc) || processor.process() ||
        radius_error_count != 1 || radius_error_line != 15)
    {
        puts("N15 radius mismatch did not stop the host");
        return false;
    }
    processor.flush();
    if (processor.process() || processor.popOut(output) || radius_error_count != 1)
    {
        puts("failed host processor resumed or emitted the rejected arc");
        return false;
    }
    processor.setOptions(options);
    processor.setUnits(scale == 1.0f ? UNITS_INCH : UNITS_MM);
    arc.p_1 = v2(0.0f, (5.55408f + 2.0f * 0.51156f) * scale);
    if (!processor.pushIn(arc) || !processor.process())
    {
        puts("reset processor could not restart after radius failure");
        return false;
    }
    return true;
}

static bool check_runner_radius_rejection(bool inches, bool trimCrossings)
{
    CcMainRunner runner;
    CcMainOptions options;
    options.toolRadius = inches ? 0.1f : 2.54f;
    options.globalTrimCrossing = trimCrossings;
    options.callbacks.error = capture_error;
    options.callbacks.output = capture_gcode;
    radius_error_count = 0;
    radius_error_line = 0;
    emitted_gcode.clear();
    if (!runner.begin(options))
        return false;
    const char *program[] = {
        inches ? "G90 G20 G0 X-2 Y0" : "G90 G21 G0 X-50.8 Y0",
        "G1 G41 X0 Y0",
        inches ? "G3 X0 Y2.00084 I0 J1" : "G3 X0 Y50.821336 I0 J25.4"
    };
    for (int lineIndex = 0; lineIndex < 3; ++lineIndex)
    {
        runner.incrementLineNumber();
        const bool accepted = runner.processLine(program[lineIndex]);
        if (accepted != (lineIndex < 2))
        {
            puts("runner did not reject the inconsistent input arc");
            return false;
        }
    }
    const std::string before = emitted_gcode;
    if (runner.processLine("G1 X1 Y1") || runner.finish() ||
        emitted_gcode != before || radius_error_count != 1 || radius_error_line != 3)
    {
        puts("runner continued after radius validation failure");
        return false;
    }
    return true;
}

static bool check_runner_unresolved_gap(bool trimCrossings)
{
    CcMainRunner runner;
    CcMainOptions options;
    options.toolRadius = 0.065f;
    options.globalTrimCrossing = trimCrossings;
    options.callbacks.error = capture_gap_error;
    options.callbacks.output = capture_gcode;
    gap_error_count = 0;
    gap_error_line = 0;
    emitted_gcode.clear();
    if (!runner.begin(options))
        return false;

    const char *program[] = {
        "G17 G0 G90 Y0.5 X1.5",
        "G41 G1 X1.4 Y0.1",
        "X0.75",
        "X0.1",
        "Y0.2",
        "X0.3 Y0.25",
        "X0.1 Y0.3"
    };
    for (int lineIndex = 0; lineIndex < 7; ++lineIndex)
    {
        runner.incrementLineNumber();
        const bool accepted = runner.processLine(program[lineIndex]);
        if (accepted != (trimCrossings || lineIndex < 6))
            return false;
    }
    if (trimCrossings)
        return gap_error_count == 0;

    const std::string before = emitted_gcode;
    return gap_error_count == 1 && gap_error_line == 7 &&
           !runner.processLine("X0.3 Y0.35") && !runner.finish() &&
           emitted_gcode == before && emitted_gcode.find("X0.284") == std::string::npos;
}

int main(void)
{
    if (!check_shared_lead_endpoint(1.0f) || !check_shared_lead_endpoint(25.4f))
    {
        puts("shared lead endpoint drifted or crossing/parallel behavior changed");
        return 1;
    }
    set_physical_units(false);

    if (!pointOnSegment(v2(0.0f, 0.0f), v2(0.007627f, 0.0f), v2(0.003f, 0.0f)) ||
        pointOnSegment(v2(0.0f, 0.0f), v2(0.007627f, 0.0f), v2(0.003f, 0.001f)))
    {
        puts("short-segment membership check failed");
        return 1;
    }
    if (!check_crossing_arcs(1.0f) || !check_crossing_arcs(25.4f))
        return 1;
    if (!check_circle_cases())
    {
        puts("circle tangency, separation, containment or signed-radius check failed");
        return 1;
    }
    for (int trim = 0; trim < 2; ++trim)
    {
        if (!check_runner_unresolved_gap(trim != 0))
        {
            puts("runner emitted an unresolved zigzag bevel or changed crossing-trim behavior");
            return 1;
        }
        if (!check_early_inversion(1.0f, false, trim != 0) ||
            !check_early_inversion(1.0f, true, trim != 0) ||
            !check_early_inversion(25.4f, false, trim != 0) ||
            !check_early_inversion(25.4f, true, trim != 0))
        {
            puts("early inversion rejection, source line or crossing-trim behavior failed");
            return 1;
        }
        if (!check_arc_radius_validation(1.0f, trim != 0) ||
            !check_arc_radius_validation(25.4f, trim != 0) ||
            !check_runner_radius_rejection(true, trim != 0) ||
            !check_runner_radius_rejection(false, trim != 0))
            return 1;
    }
    puts("host crossing checks passed");
    return 0;
}
