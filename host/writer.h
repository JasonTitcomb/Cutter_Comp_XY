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

// -------------------- Emit helpers (host files) --------------------
static void emit_move_as_gcode(FILE *f, const Move2D &m, bool inchUnits = true)
{
    const int posDigits = inchUnits ? 4 : 3;
    const int centerDigits = inchUnits ? 4 : 3;
    const int zDigits = inchUnits ? 4 : 3;

    if (m.type == MOT_LINE || m.type == MOT_RAPID)
    {
        if (m.seqNum != 0)
            std::fprintf(f, "N%d ", (int)m.seqNum);
        std::fprintf(f, "%s", m.type == MOT_RAPID ? "G0" : "G1");
        if (m.hasXY)
            std::fprintf(f, " X%.*f Y%.*f", posDigits, m.p_1.x, posDigits, m.p_1.y);
        if (m.hasZ)
            std::fprintf(f, " Z%.*f", zDigits, m.z_1);
        std::fprintf(f, "\n");
        return;
    }

    if (m.type == MOT_ARC)
    {
        Vec2 dCenter = m.center - m.p_0;
        if (m.seqNum != 0)
            std::fprintf(f, "N%d ", (int)m.seqNum);
        std::fprintf(f, "%s X%.*f Y%.*f I%.*f J%.*f",
                     (m.arcDir == ARC_CW) ? "G2" : "G3",
                     posDigits, m.p_1.x, posDigits, m.p_1.y,
                     centerDigits, dCenter.x, centerDigits, dCenter.y);
        if (m.hasZ)
            std::fprintf(f, " Z%.*f", zDigits, m.z_1);
        std::fprintf(f, "\n");
        return;
    }
}

void write_gcode(const char *path,
                 const std::vector<Move2D> &moves,
                 float toolRadius = 0.0f,
                 bool inchUnits = true)
{
    // G-code file
    if (FILE *f = open_file_write_binary(path))
    {
        std::fprintf(f, "(tool_radius=%.6f)\n", toolRadius);
        for (auto &m : moves)
        {
            if (!m.valid || m.type == MOT_EMPTY)
                continue;
            emit_move_as_gcode(f, m, inchUnits);
        }
        std::fclose(f);
    }
}

static void svg_polyline(std::ostringstream &ss, const std::vector<Vec2> &pts, const char *stroke, float stroke_width = 0.002f)
{
    if (pts.size() < 2)
        return;
    ss << "<polyline fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"" << stroke_width << "\" points=\"";
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

static void svg_polyline_dashed(std::ostringstream &ss, const std::vector<Vec2> &pts, const char *stroke, float stroke_width = 0.002f)
{
    if (pts.size() < 2)
        return;
    ss << "<polyline fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"" << stroke_width << "\" stroke-dasharray=\"0.01,0.01\" points=\"";
    for (auto &p : pts)
        ss << p.x << "," << p.y << " ";
    ss << "\" />\n";
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
                      bool plot_invalid_elements = false)
{
    Bounds b;
    auto accumulate_bounds = [&](const std::vector<Move2D> &mv, bool onlyValid)
    {
        for (auto &m : mv)
        {
            if (onlyValid && (!m.valid || m.type == MOT_EMPTY))
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
    std::ostringstream ss;
    ss << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\""
       << minx << " " << miny << " " << w << " " << h << "\">\n";
    ss << "<rect x=\"" << minx << "\" y=\"" << miny << "\" width=\"" << w
       << "\" height=\"" << h << "\" fill=\"white\" />\n";
    const float textSize = 0.012f * std::max(w, h);
    const float textNudge = 0.006f * std::max(w, h);
     if (input_base_name && input_base_name[0] != '\0')
     {
          const float headerX = minx + 0.5f * w;
          const float headerY = miny + 1.5f * textSize;
          ss << "<text x=\"" << headerX << "\" y=\"" << headerY
              << "\" fill=\"#111111\" font-size=\"" << textSize
              << "\" text-anchor=\"middle\" dominant-baseline=\"middle\">"
              << input_base_name << "  |  tool radius=" << tool_radius << "</text>\n";
     }
    if (original)
    {
        for (auto &m : *original)
        {
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
                svg_polyline_dashed(ss, pts, "#888888", 0.003f); // Gray color for rapid moves
            }
            else
            {
                svg_polyline(ss, pts, "#1f77b4", 0.005f);
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
                   << "\" fill=\"#1f77b4\" font-size=\"" << textSize
                   << "\" text-anchor=\"middle\" dominant-baseline=\"middle\">"
                   << m.seqNum << "</text>\n";
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
                    ss << "<circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"" << (0.5f * tool_diameter) << "\" stroke=\"#888\" stroke-width=\"0.001\" fill=\"none\" stroke-dasharray=\"0.01,0.01\" />\n";
                }
            }
        }
    }
    for (auto &m : moves)
    {
        if (m.type == MOT_EMPTY)
            continue;
        if (!plot_invalid_elements && !m.valid)
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
            svg_polyline_dashed(ss, pts, "#888888"); // Gray color for rapid moves
        }
        else
        {
            svg_polyline(ss, pts, moveColor);
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
               << m.seqNum << "</text>\n";
        }
    }
    ss << "</svg>\n";
    std::ofstream out(path, std::ios::binary);
    out << ss.str();
}
