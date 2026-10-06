#pragma once
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <algorithm>
#include <cmath>
#include "cc_processor.h"

struct Bounds
{
    float minx = +1e30f, miny = +1e30f, maxx = -1e30f, maxy = -1e30f;
    void add(Vec2 p)
    {
        minx = std::min(minx, p.x);
        miny = std::min(miny, p.y);
        maxx = std::max(maxx, p.x);
        maxy = std::max(maxy, p.y);
    }
};

static inline FILE *open_file_write_binary(const char *path)
{
#ifdef _MSC_VER
    FILE *f = nullptr;
    if (fopen_s(&f, path, "wb") != 0)
        return nullptr;
    return f;
#else
    return std::fopen(path, "wb");
#endif
}


static void svg_polyline(std::ostringstream &ss, const std::vector<Vec2> &pts, const char *stroke, float stroke_width = 1.0f)
{
    if (pts.size() < 2)
        return;
    ss << "<polyline fill=\"none\" vector-effect=\"non-scaling-stroke\" stroke=\"" << stroke << "\" stroke-width=\"" << stroke_width << "\" points=\"";
    for (auto &p : pts)
        ss << p.x << "," << p.y << " ";
    ss << "\" />\n";
}

static std::vector<Vec2> approx_move_points(const Move2D &m, int arcSegments = 24)
{
    std::vector<Vec2> pts;
    if (m.type == MOT_LINE || m.type == MOT_RAPID)
    {
        pts.push_back(m.p_0);
        pts.push_back(m.p_1);
        return pts;
    }
    if (m.type == MOT_ARC)
    {
        float a0 = std::atan2(m.p_0.y - m.center.y, m.p_0.x - m.center.x);
        float a1 = std::atan2(m.p_1.y - m.center.y, m.p_1.x - m.center.x);
        auto norm = [](float a)
        {
            while (a < 0)
                a += 2.0f * (float)M_PI;
            while (a >= 2.0f * (float)M_PI)
                a -= 2.0f * (float)M_PI;
            return a;
        };
        a0 = norm(a0);
        a1 = norm(a1);
        float sweep;
        if (m.arcDir == ARC_CCW)
        {
            sweep = a1 - a0;
            if (sweep < 0)
                sweep += 2.0f * (float)M_PI;
        }
        else
        {
            sweep = a0 - a1;
            if (sweep < 0)
                sweep += 2.0f * (float)M_PI;
            sweep = -sweep;
        }
        int n = std::max(6, arcSegments);
        pts.reserve(n + 1);
        for (int i = 0; i <= n; ++i)
        {
            float t = (float)i / (float)n;
            float a = a0 + sweep * t;
            Vec2 p = v2(m.center.x + m.radius * std::cos(a),
                        m.center.y + m.radius * std::sin(a));
            pts.push_back(p);
        }
        return pts;
    }
    return pts;
}

static Vec2 move_label_pos(const Move2D &m)
{
    if (m.type == MOT_LINE || m.type == MOT_RAPID)
    {
        return v2(0.5f * (m.p_0.x + m.p_1.x), 0.5f * (m.p_0.y + m.p_1.y));
    }
    if (m.type == MOT_ARC)
    {
        float a0 = std::atan2(m.p_0.y - m.center.y, m.p_0.x - m.center.x);
        float a1 = std::atan2(m.p_1.y - m.center.y, m.p_1.x - m.center.x);
        auto norm = [](float a)
        {
            while (a < 0)
                a += 2.0f * (float)M_PI;
            while (a >= 2.0f * (float)M_PI)
                a -= 2.0f * (float)M_PI;
            return a;
        };
        a0 = norm(a0);
        a1 = norm(a1);

        float sweep;
        if (m.arcDir == ARC_CCW)
        {
            sweep = a1 - a0;
            if (sweep < 0)
                sweep += 2.0f * (float)M_PI;
        }
        else
        {
            sweep = a0 - a1;
            if (sweep < 0)
                sweep += 2.0f * (float)M_PI;
            sweep = -sweep;
        }

        float amid = a0 + 0.5f * sweep;
        return v2(m.center.x + m.radius * std::cos(amid),
                  m.center.y + m.radius * std::sin(amid));
    }
    return m.p_0;
}

static void svg_polyline_dashed(std::ostringstream &ss, const std::vector<Vec2> &pts, const char *stroke, float stroke_width = 1.0f, float dash_length = 4.0f)
{
    if (pts.size() < 2)
        return;
    ss << "<polyline fill=\"none\" vector-effect=\"non-scaling-stroke\" stroke=\"" << stroke << "\" stroke-width=\"" << stroke_width
       << "\" stroke-dasharray=\"" << dash_length << "," << dash_length << "\" points=\"";
    for (auto &p : pts)
        ss << p.x << "," << p.y << " ";
    ss << "\" />\n";
}

static const char *pause_marker_label(PauseKind kind)
{
    if (kind == PAUSE_M0)
        return "M0";
    if (kind == PAUSE_M1)
        return "M1";
    if (kind == PAUSE_DWELL)
        return "G4";
    return "Pause";
}

static bool is_svg_pause_marker(const Move2D &m)
{
    return m.pauseKind != PAUSE_NONE || m.pause_after != 0.0f;
}

static void svg_pause_marker(std::ostringstream &ss, const Vec2 &p, float size, PauseKind kind)
{
    const float halfSize = 0.5f * size;
    ss << "<rect x=\"" << (p.x - halfSize) << "\" y=\"" << (p.y - halfSize)
       << "\" width=\"" << size << "\" height=\"" << size
       << "\" fill=\"#ffd166\" stroke=\"#111111\" vector-effect=\"non-scaling-stroke\" stroke-width=\"1"
       << "\"><title>Pause";
    if (kind != PAUSE_NONE)
        ss << " " << pause_marker_label(kind);
    ss << "</title></rect>\n";
}

enum class SvgLegendSymbol
{
    COMPENSATED,
    ORIGINAL,
    RAPID,
    TOOL_SWEEP,
    TOOL_CIRCLES,
    PAUSE,
    ORIGIN
};

static void svg_legend_entry(std::ostringstream &ss, float x, float y, float fontSize,
                             SvgLegendSymbol symbol, const char *label)
{
    const float x0 = x + 0.2f * fontSize;
    const float x1 = x + 2.0f * fontSize;
    const float centerX = 0.5f * (x0 + x1);
    const float halfMark = 0.35f * fontSize;
    const float strokeWidth = 1.0f;

    switch (symbol)
    {
    case SvgLegendSymbol::COMPENSATED:
    case SvgLegendSymbol::ORIGINAL:
    case SvgLegendSymbol::RAPID:
    case SvgLegendSymbol::TOOL_SWEEP:
    {
        const char *color = symbol == SvgLegendSymbol::COMPENSATED ? "#d62728" :
                            symbol == SvgLegendSymbol::ORIGINAL ? "#1f77b4" :
                            symbol == SvgLegendSymbol::TOOL_SWEEP ? "#bbbbbb" : "#888888";
        ss << "<line x1=\"" << x0 << "\" y1=\"" << y << "\" x2=\"" << x1 << "\" y2=\"" << y
           << "\" stroke=\"" << color << "\" stroke-width=\""
           << (symbol == SvgLegendSymbol::TOOL_SWEEP ? 0.5f * fontSize : strokeWidth) << "\"";
        if (symbol == SvgLegendSymbol::RAPID)
            ss << " stroke-dasharray=\"4,4\"";
        if (symbol == SvgLegendSymbol::TOOL_SWEEP)
            ss << " stroke-opacity=\"0.6\" stroke-linecap=\"round\"";
        else
            ss << " vector-effect=\"non-scaling-stroke\"";
        ss << " />\n";
        break;
    }
    case SvgLegendSymbol::TOOL_CIRCLES:
        ss << "<circle cx=\"" << centerX << "\" cy=\"" << y << "\" r=\"" << halfMark
           << "\" fill=\"none\" stroke=\"#888888\" stroke-width=\"" << strokeWidth
           << "\" vector-effect=\"non-scaling-stroke\" stroke-dasharray=\"4,4\" />\n";
        break;
    case SvgLegendSymbol::PAUSE:
        ss << "<rect x=\"" << (centerX - halfMark) << "\" y=\"" << (y - halfMark)
           << "\" width=\"" << (2.0f * halfMark) << "\" height=\"" << (2.0f * halfMark)
           << "\" fill=\"#ffd166\" stroke=\"#111111\" vector-effect=\"non-scaling-stroke\" stroke-width=\"" << strokeWidth << "\" />\n";
        break;
    case SvgLegendSymbol::ORIGIN:
        ss << "<g stroke=\"#008000\" stroke-width=\"" << strokeWidth << "\">"
           << "<line vector-effect=\"non-scaling-stroke\" x1=\"" << (centerX - halfMark) << "\" y1=\"" << y << "\" x2=\"" << (centerX + halfMark)
           << "\" y2=\"" << y << "\" /><line vector-effect=\"non-scaling-stroke\" x1=\"" << centerX << "\" y1=\"" << (y - halfMark)
           << "\" x2=\"" << centerX << "\" y2=\"" << (y + halfMark) << "\" /></g>\n";
        break;
    }

    ss << "<text x=\"" << (x + 2.6f * fontSize) << "\" y=\"" << y
       << "\" fill=\"#222222\" font-size=\"" << fontSize
       << "\" dominant-baseline=\"middle\">" << label << "</text>\n";
}

static void write_svg(const char *path,
                      const std::vector<Move2D> &moves,
                      const std::vector<Move2D> *original = nullptr,
                      bool mirror_x = false,
                      bool mirror_y = false,
                      float tool_diameter = 0.0f,
                      bool show_tool_circles = true,
                      bool show_tool_sweep = false,
                      bool show_seq_numbers = true,
                      const char *input_base_name = nullptr,
                      float tool_radius = 0.0f,
                      bool plot_invalid_elements = false,
                      const char *source_label = nullptr,
                      bool emphasize_paths = false,
                      Units tool_units = UNITS_MM)
{
    Bounds b;
    auto accumulate_bounds = [&](const std::vector<Move2D> &mv, bool onlyValid)
    {
        for (auto &m : mv)
        {
            if (onlyValid && !m.valid)
                continue;
            if (is_svg_pause_marker(m))
            {
                b.add(m.p_0);
                continue;
            }
            if (onlyValid && m.type == MOT_EMPTY)
                continue;
            auto pts = approx_move_points(m);
            for (auto &p : pts)
            {
                b.add(p);
            }
        }
    };
    if (original)
        accumulate_bounds(*original, false);
    accumulate_bounds(moves, !plot_invalid_elements);
    float pad = 0.05f * std::max((b.maxx - b.minx), (b.maxy - b.miny));
    float minx = b.minx - pad, miny = b.miny - pad;
    float w = (b.maxx - b.minx) + 2 * pad;
    float h = (b.maxy - b.miny) + 2 * pad;
     const float textSize = 0.015f * std::max(w, h);
     const float originX = mirror_x ? b.maxx + b.minx : 0.0f;
     const float originY = mirror_y ? b.maxy + b.miny : 0.0f;
    const float originMarkSize = 0.01f * std::max(w, h);
    const float originExtent = 2.0f * originMarkSize;
    const float scale = std::max(w, h);
    const float legendFontSize = 0.009f * scale;
    const float legendPadding = 0.6f * legendFontSize;
    const float legendGap = 0.035f * scale;
    bool hasRapid = false;
    bool hasPause = false;
    if (original)
    {
        for (const Move2D &m : *original)
        {
            hasRapid = hasRapid || (m.valid && m.type == MOT_RAPID);
            hasPause = hasPause || (m.valid && is_svg_pause_marker(m));
        }
    }
    for (const Move2D &m : moves)
    {
        hasRapid = hasRapid || (m.valid && m.type == MOT_RAPID);
        hasPause = hasPause || (m.valid && is_svg_pause_marker(m));
    }
    const bool showSweepLegend = show_tool_sweep && tool_diameter > 0.0f;
    const bool showCirclesLegend = show_tool_circles && !show_tool_sweep && tool_diameter > 0.0f;
    const int legendCount = 1 + (original ? 1 : 0) + (hasRapid ? 1 : 0) +
                            (showSweepLegend ? 1 : 0) + (showCirclesLegend ? 1 : 0) +
                            (hasPause ? 1 : 0) + 1;
    const float legendY = miny + h + legendGap;
    const float legendHeight = 2.0f * legendPadding + 1.5f * legendFontSize;
     const float viewMinX = std::min(minx, originX - originExtent);
     const float viewMinY = std::min(miny, originY - originExtent);
    const float baseViewMaxX = std::max(minx + w, originX + originExtent);
    const float requiredLegendWidth = legendCount * 12.5f * legendFontSize + 2.0f * legendGap;
    const float viewMaxX = std::max(baseViewMaxX, viewMinX + requiredLegendWidth);
    const float viewMaxY = std::max(std::max(miny + h, originY + originExtent), legendY + legendHeight + legendPadding);
    std::ostringstream ss;
    ss << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\""
         << viewMinX << " " << viewMinY << " " << (viewMaxX - viewMinX) << " "
         << (viewMaxY - viewMinY) << "\">\n";
     ss << "<rect x=\"" << viewMinX << "\" y=\"" << viewMinY << "\" width=\"" << (viewMaxX - viewMinX)
         << "\" height=\"" << (viewMaxY - viewMinY) << "\" fill=\"white\" />\n";
    const float textNudge = 0.015f * std::max(w, h);
    const float pathWidth = emphasize_paths ? 2.0f : 1.0f;
    const float dashLength = 4.0f * pathWidth;
     if (input_base_name && input_base_name[0] != '\0')
     {
          const float headerX = minx + 0.5f * w;
          const float headerY = miny + 1.5f * textSize;
          ss << "<text x=\"" << headerX << "\" y=\"" << headerY
              << "\" fill=\"#111111\" font-size=\"" << textSize
              << "\" text-anchor=\"middle\" dominant-baseline=\"middle\">"
              << input_base_name;
          if (source_label)
              ss << "  |  " << source_label;
          ss << "  |  tool dia.=" << fabsf(tool_radius * 2.0f)
             << (tool_units == UNITS_INCH ? " in" : " mm") << "</text>\n";
     }
    const float legendEntryY = legendY + legendPadding + 0.5f * legendFontSize;
    const float legendEntryWidth = (viewMaxX - viewMinX - 2.0f * legendGap) / legendCount;
    float legendEntryX = viewMinX + legendGap;
    svg_legend_entry(ss, legendEntryX, legendEntryY, legendFontSize, SvgLegendSymbol::COMPENSATED, "Compensated");
    legendEntryX += legendEntryWidth;
    if (original)
    {
        svg_legend_entry(ss, legendEntryX, legendEntryY, legendFontSize, SvgLegendSymbol::ORIGINAL, "Original");
        legendEntryX += legendEntryWidth;
    }
    if (hasRapid)
    {
        svg_legend_entry(ss, legendEntryX, legendEntryY, legendFontSize, SvgLegendSymbol::RAPID, "Rapid");
        legendEntryX += legendEntryWidth;
    }
    if (showSweepLegend)
    {
        svg_legend_entry(ss, legendEntryX, legendEntryY, legendFontSize, SvgLegendSymbol::TOOL_SWEEP, "Tool sweep");
        legendEntryX += legendEntryWidth;
    }
    if (showCirclesLegend)
    {
        svg_legend_entry(ss, legendEntryX, legendEntryY, legendFontSize, SvgLegendSymbol::TOOL_CIRCLES, "Tool dia.");
        legendEntryX += legendEntryWidth;
    }
    if (hasPause)
    {
        svg_legend_entry(ss, legendEntryX, legendEntryY, legendFontSize, SvgLegendSymbol::PAUSE, "Pause");
        legendEntryX += legendEntryWidth;
    }
    svg_legend_entry(ss, legendEntryX, legendEntryY, legendFontSize, SvgLegendSymbol::ORIGIN, "Origin");
    if (original)
    {
        for (auto &m : *original)
        {
            if (!m.valid)
                continue;
            if (m.type == MOT_EMPTY)
                continue;
            auto pts = approx_move_points(m);
            for (auto &p : pts)
            {
                if (mirror_x)
                    p.x = b.maxx + b.minx - p.x;
                if (mirror_y)
                    p.y = b.maxy + b.miny - p.y;
            }
            // Use dashed style for rapid moves, solid for others
            if (m.type == MOT_RAPID)
            {
                svg_polyline_dashed(ss, pts, "#888888", pathWidth, dashLength); // Gray color for rapid moves
            }
            else
            {
                svg_polyline(ss, pts, "#1f77b4", pathWidth);
            }
            if (!pts.empty())
            {
                float r = 0.001f * std::max(w, h);
                ss << "<circle cx=\"" << pts.front().x << "\" cy=\"" << pts.front().y << "\" r=\"" << r << "\" fill=\"#1f77b4\" />\n";
                ss << "<circle cx=\"" << pts.back().x << "\" cy=\"" << pts.back().y << "\" r=\"" << r << "\" fill=\"#1f77b4\" />\n";
            }
            if (show_seq_numbers)
            {
                Vec2 tp = move_label_pos(m);
                if (mirror_x)
                    tp.x = b.maxx + b.minx - tp.x;
                if (mirror_y)
                    tp.y = b.maxy + b.miny - tp.y;
                ss << "<text x=\"" << (tp.x + textNudge) << "\" y=\"" << (tp.y - textNudge)
                   << "\" fill=\"#1fb482\" font-size=\"" << textSize
                   << "\" text-anchor=\"middle\" dominant-baseline=\"middle\">"
                   << m.lineNum << "</text>\n";
            }
        }
    }
    // Draw smooth tool sweep as a continuous swath along each move.
    if (show_tool_sweep && tool_diameter > 0.0f)
    {
        // Two passes (feed then rapid) each in a group with shared opacity
        // so overlapping segments don't darken.
        struct SweepPass { const char *color; float opacity; bool rapid; };
        SweepPass passes[] = {
            {"#bbbbbb", 0.50f, false},
            {"#999999", 0.35f, true}
        };
        for (auto &pass : passes)
        {
            ss << "<g opacity=\"" << pass.opacity << "\">\n";
            for (auto &m : moves)
            {
                if (!m.valid || m.type == MOT_EMPTY)
                    continue;
                if ((m.type == MOT_RAPID) != pass.rapid)
                    continue;

                auto pts = approx_move_points(m, 24);
                if (pts.size() < 2)
                    continue;

                for (auto &p : pts)
                {
                    if (mirror_x)
                        p.x = b.maxx + b.minx - p.x;
                    if (mirror_y)
                        p.y = b.maxy + b.miny - p.y;
                }

                ss << "<polyline fill=\"none\" stroke=\"" << pass.color << "\" stroke-width=\""
                   << tool_diameter
                   << "\" stroke-linecap=\"round\" stroke-linejoin=\"round\" points=\"";
                for (auto &p : pts)
                    ss << p.x << "," << p.y << " ";
                ss << "\" />\n";
            }
            ss << "</g>\n";
        }
    }

    // Draw tool diameter circles along the offset profile (moves)
    if (show_tool_circles && !show_tool_sweep && tool_diameter > 0.0f)
    {
        float step = 0.5f * tool_diameter; // step size for lerping, 50% of tool diameter
        for (auto &m : moves)
        {
            if (!m.valid || m.type == MOT_EMPTY)
                continue;
            auto pts = approx_move_points(m, 10); // finer for smoother lerp
            for (size_t i = 0; i + 1 < pts.size(); ++i)
            {
                Vec2 p0 = pts[i], p1 = pts[i + 1];
                if (mirror_x)
                {
                    p0.x = b.maxx + b.minx - p0.x;
                    p1.x = b.maxx + b.minx - p1.x;
                }
                if (mirror_y)
                {
                    p0.y = b.maxy + b.miny - p0.y;
                    p1.y = b.maxy + b.miny - p1.y;
                }
                float seg_len = std::sqrt((p1.x - p0.x) * (p1.x - p0.x) + (p1.y - p0.y) * (p1.y - p0.y));
                int nsteps = std::max(1, (int)(seg_len / step));
                for (int s = 0; s <= nsteps; ++s)
                {
                    float t = (float)s / (float)nsteps;
                    float x = p0.x + t * (p1.x - p0.x);
                    float y = p0.y + t * (p1.y - p0.y);
                    ss << "<circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"" << (0.5f * tool_diameter) << "\" stroke=\"#888\" vector-effect=\"non-scaling-stroke\" stroke-width=\"1\" fill=\"none\" stroke-dasharray=\"4,4\" />\n";
                }
            }
        }
    }
    for (auto &m : moves)
    {
        if (!plot_invalid_elements && !m.valid)
            continue;
        if (is_svg_pause_marker(m))
        {
            Vec2 p = m.p_0;
            if (mirror_x)
                p.x = b.maxx + b.minx - p.x;
            if (mirror_y)
                p.y = b.maxy + b.miny - p.y;
            svg_pause_marker(ss, p, 0.02f * std::max(w, h), m.pauseKind);
            continue;
        }
        if (m.type == MOT_EMPTY)
            continue;
        auto pts = approx_move_points(m);
        for (auto &p : pts)
        {
            if (mirror_x)
                p.x = b.maxx + b.minx - p.x;
            if (mirror_y)
                p.y = b.maxy + b.miny - p.y;
        }
        const char *moveColor = m.valid ? "#d62728" : "#000000";
        // Use dashed style for rapid moves, solid for others
        if (m.type == MOT_RAPID && m.valid)
        {
            svg_polyline_dashed(ss, pts, "#888888", pathWidth, dashLength); // Gray color for rapid moves
        }
        else
        {
            svg_polyline(ss, pts, moveColor, pathWidth);
        }
        if (!pts.empty())
        {
            float r = 0.001f * std::max(w, h);
            ss << "<circle cx=\"" << pts.front().x << "\" cy=\"" << pts.front().y << "\" r=\"" << r << "\" fill=\"" << moveColor << "\" />\n";
            ss << "<circle cx=\"" << pts.back().x << "\" cy=\"" << pts.back().y << "\" r=\"" << r << "\" fill=\"" << moveColor << "\" />\n";
        }
        if (show_seq_numbers)
        {
            Vec2 tp = move_label_pos(m);
            if (mirror_x)
                tp.x = b.maxx + b.minx - tp.x;
            if (mirror_y)
                tp.y = b.maxy + b.miny - tp.y;
                ss << "<text x=\"" << (tp.x + textNudge) << "\" y=\"" << (tp.y - textNudge)
                    << "\" fill=\"" << moveColor << "\" font-size=\"" << textSize
               << "\" text-anchor=\"middle\" dominant-baseline=\"middle\">"
               << m.lineNum << "</text>\n";
        }
    }
     const float originStrokeWidth = 1.0f;
     ss << "<g stroke=\"#008000\" stroke-width=\"" << originStrokeWidth << "\">\n"
         << "<line vector-effect=\"non-scaling-stroke\" x1=\"" << (originX - originMarkSize) << "\" y1=\"" << originY
         << "\" x2=\"" << (originX + originMarkSize) << "\" y2=\"" << originY << "\" />\n"
         << "<line vector-effect=\"non-scaling-stroke\" x1=\"" << originX << "\" y1=\"" << (originY - originMarkSize)
         << "\" x2=\"" << originX << "\" y2=\"" << (originY + originMarkSize) << "\" />\n"
         << "</g>\n";
     ss << "</svg>\n";
    std::ofstream out(path, std::ios::binary);
    out << ss.str();
}
