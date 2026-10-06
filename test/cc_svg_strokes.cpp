#include <stdint.h>
#include <stddef.h>
#include <cstdio>
#include <iterator>
#include "../host/writer.h"

static bool check_svg_strokes(float scale, bool emphasize, bool sweep, Units units)
{
    std::vector<Move2D> moves(3);
    moves[0].valid = true;
    moves[0].type = MOT_LINE;
    moves[0].p_0 = v2(0.0f, 0.0f);
    moves[0].p_1 = v2(scale, scale);
    moves[1] = moves[0];
    moves[1].type = MOT_RAPID;
    moves[1].p_0 = moves[0].p_1;
    moves[1].p_1 = v2(0.0f, scale);
    moves[2].valid = true;
    moves[2].type = MOT_EMPTY;
    moves[2].p_0 = moves[1].p_1;
    moves[2].pauseKind = PAUSE_M0;

    const char *path = "cc_svg_strokes.svg";
    const float diameter = 0.25f * scale;
    write_svg(path, moves, &moves, false, false, diameter, !sweep, sweep,
              false, "SVG test", 0.5f * diameter, false, nullptr, emphasize, units);
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        std::fprintf(stderr, "Could not read SVG test output\n");
        return false;
    }
    const std::string svg((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    input.close();
    if (std::remove(path) != 0)
    {
        std::fprintf(stderr, "Could not remove SVG test output\n");
        return false;
    }

    std::ostringstream expectedTitle;
    expectedTitle << "tool dia.=" << diameter
                  << (units == UNITS_INCH ? " in" : " mm") << "</text>";
    if (svg.find(expectedTitle.str()) == std::string::npos ||
        svg.find("tool radius=") != std::string::npos)
        return false;

    size_t position = 0;
    int pathCount = 0;
    int sweepCount = 0;
    int outlineCount = 0;
    while ((position = svg.find('<', position)) != std::string::npos)
    {
        const size_t end = svg.find('>', position);
        if (end == std::string::npos)
            return false;
        const std::string element = svg.substr(position, end - position + 1);
        const bool polyline = element.find("<polyline ") == 0;
        const bool toolSweep = polyline && element.find("stroke-linecap=\"round\"") != std::string::npos;
        const bool hasStroke = element.find(" stroke=") != std::string::npos;
        const bool group = element.find("<g ") == 0;
        const bool legendSweep = element.find("stroke-opacity=\"0.6\"") != std::string::npos;
        const bool originLine = element.find("<line ") == 0 && !hasStroke;
        if (toolSweep)
        {
            std::ostringstream expectedWidth;
            expectedWidth << "stroke-width=\"" << diameter << "\"";
            if (element.find(expectedWidth.str()) == std::string::npos ||
                element.find("non-scaling-stroke") != std::string::npos)
                return false;
            sweepCount++;
        }
        else if (!group && !legendSweep && (hasStroke || originLine))
        {
            if (element.find("vector-effect=\"non-scaling-stroke\"") == std::string::npos)
                return false;
            const char *width = polyline && emphasize ? "stroke-width=\"2\"" : "stroke-width=\"1\"";
            if (!originLine && element.find(width) == std::string::npos)
                return false;
            if (polyline)
            {
                pathCount++;
                if (element.find("stroke=\"#888888\"") != std::string::npos &&
                    element.find(emphasize ? "stroke-dasharray=\"8,8\"" : "stroke-dasharray=\"4,4\"") == std::string::npos)
                    return false;
            }
            else
                outlineCount++;
        }
        position = end + 1;
    }
    return pathCount == 4 && sweepCount == (sweep ? 2 : 0) && outlineCount >= 8;
}

int main()
{
    const float scales[] = {0.1f, 1000.0f};
    for (float scale : scales)
    {
        for (int emphasize = 0; emphasize < 2; ++emphasize)
        {
            for (int sweep = 0; sweep < 2; ++sweep)
            {
                for (int inches = 0; inches < 2; ++inches)
                {
                    if (!check_svg_strokes(scale, emphasize != 0, sweep != 0,
                                           inches ? UNITS_INCH : UNITS_MM))
                    {
                        std::fprintf(stderr, "SVG stroke check failed: scale=%g emphasize=%d sweep=%d inches=%d\n",
                                     scale, emphasize, sweep, inches);
                        return 1;
                    }
                }
            }
        }
    }
    return 0;
}
