#pragma once
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <algorithm>
#include <cmath>
#include "CutterComp2D.h"

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
static void emit_move_as_gcode(FILE *f, const Move2D &m, MachineType machineType)
{
    const bool latheMode = machine_is_lathe(machineType);
    // Reverse of parser mapping: internal milling-like XY -> machine turning axes.
    // (VB parity with ConvertToTurning)
    Vec3 p1m = internal_xy_to_machine(m.p_1, machineType);

    if (m.type == MOT_LINE || m.type == MOT_RAPID)
    {
        if (latheMode)
        {
            std::fprintf(f, "N%d %s X%.4f Z%.4f\n",
                         (int)m.seqNum, m.type == MOT_RAPID ? "G0" : "G1", p1m.x, p1m.z);
        }
        else
        {
            std::fprintf(f, "N%d %s X%.4f Y%.4f\n",
                         (int)m.seqNum, m.type == MOT_RAPID ? "G0" : "G1", p1m.x, p1m.y);
        }
        return;
    }

    if (m.type == MOT_ARC)
    {
        Vec2 dInternal = m.center - m.p_0;
        // Internal center deltas converted back to machine deltas:
        // mill => I/J, lathe => I/K.
        Vec3 dMachine = internal_delta_xy_to_machine(dInternal, machineType);

        if (latheMode)
        {
            std::fprintf(f, "N%d %s X%.4f Z%.4f I%.4f K%.4f\n",
                         (int)m.seqNum, (m.arcDir == ARC_CW) ? "G2" : "G3",
                         p1m.x, p1m.z, dMachine.x, dMachine.z);
        }
        else
        {
            std::fprintf(f, "N%d %s X%.4f Y%.4f I%.4f J%.4f\n",
                         (int)m.seqNum, (m.arcDir == ARC_CW) ? "G2" : "G3",
                         p1m.x, p1m.y, dMachine.x, dMachine.y);
        }
        return;
    }
}

void write_gcode(const char *path, const std::vector<Move2D> &moves, MachineType machineType = MAC_MILL)
{
    // G-code file
    if (FILE *f = open_file_write_binary(path))
    {
        for (auto &m : moves)
        {
            if (!m.valid || m.type == MOT_EMPTY)
                continue;
            emit_move_as_gcode(f, m, machineType);
        }
        std::fclose(f);
    }
}

static void svg_polyline(std::ostringstream &ss, const std::vector<Vec2> &pts, const char *stroke)
{
    if (pts.size() < 2)
        return;
    ss << "<polyline fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"0.002\" points=\"";
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

static void svg_polyline_dashed(std::ostringstream &ss, const std::vector<Vec2> &pts, const char *stroke)
{
    if (pts.size() < 2)
        return;
    ss << "<polyline fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"0.002\" stroke-dasharray=\"0.01,0.01\" points=\"";
    for (auto &p : pts)
        ss << p.x << "," << p.y << " ";
    ss << "\" />\n";
}

static void write_svg(const char *path,
                      const std::vector<Move2D> &moves,
                      const std::vector<Move2D> *original = nullptr,
                      MachineType machineType = MAC_MILL,
                      bool mirror_x = false,
                      bool mirror_y = false,
                      float tool_diameter = 0.0f,
                      bool show_tool_circles = true,
                      bool show_tool_sweep = false,
                      bool show_seq_numbers = true)
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
                p = internal_xy_to_plot_xy(p, machineType);
                b.add(p);
            }
        }
    };
    if (original)
        accumulate_bounds(*original, false);
    accumulate_bounds(moves, true);
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
    if (original)
    {
        for (auto &m : *original)
        {
            if (m.type == MOT_EMPTY)
                continue;
            auto pts = approx_move_points(m);
            for (auto &p : pts)
            {
                p = internal_xy_to_plot_xy(p, machineType);
                if (mirror_x)
                    p.x = b.maxx + b.minx - p.x;
                if (mirror_y)
                    p.y = b.maxy + b.miny - p.y;
            }
            // Use dashed style for rapid moves, solid for others
            if (m.type == MOT_RAPID)
            {
                svg_polyline_dashed(ss, pts, "#888888"); // Gray color for rapid moves
            }
            else
            {
                svg_polyline(ss, pts, "#1f77b4");
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
                tp = internal_xy_to_plot_xy(tp, machineType);
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
        for (auto &m : moves)
        {
            if (!m.valid || m.type == MOT_EMPTY || m.type == MOT_RAPID)
                continue;

            auto pts = approx_move_points(m, 24);
            if (pts.size() < 2)
                continue;

            for (auto &p : pts)
            {
                p = internal_xy_to_plot_xy(p, machineType);
                if (mirror_x)
                    p.x = b.maxx + b.minx - p.x;
                if (mirror_y)
                    p.y = b.maxy + b.miny - p.y;
            }

            ss << "<polyline fill=\"none\" stroke=\"#bbbbbb\" stroke-width=\""
               << tool_diameter
               << "\" stroke-linecap=\"round\" stroke-linejoin=\"round\" opacity=\"0.50\" points=\"";
            for (auto &p : pts)
                ss << p.x << "," << p.y << " ";
            ss << "\" />\n";
        }
    }

    // Draw tool diameter circles along the offset profile (moves)
    if (show_tool_circles && !show_tool_sweep && tool_diameter > 0.0f)
    {
        float step = 0.50f * tool_diameter; // step size for lerping, 50% of tool diameter
        for (auto &m : moves)
        {
            if (!m.valid || m.type == MOT_EMPTY)
                continue;
            auto pts = approx_move_points(m, 10); // finer for smoother lerp
            for (size_t i = 0; i + 1 < pts.size(); ++i)
            {
                Vec2 p0 = pts[i], p1 = pts[i + 1];
                p0 = internal_xy_to_plot_xy(p0, machineType);
                p1 = internal_xy_to_plot_xy(p1, machineType);
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
    // Then modify the final loop in write_svg (around line 169) to:
    for (auto &m : moves)
    {
        if (!m.valid || m.type == MOT_EMPTY)
            continue; // Remove m.rapid check
        auto pts = approx_move_points(m);
        for (auto &p : pts)
        {
            p = internal_xy_to_plot_xy(p, machineType);
            if (mirror_x)
                p.x = b.maxx + b.minx - p.x;
            if (mirror_y)
                p.y = b.maxy + b.miny - p.y;
        }
        // Use dashed style for rapid moves, solid for others
        if (m.type == MOT_RAPID)
        {
            svg_polyline_dashed(ss, pts, "#888888"); // Gray color for rapid moves
        }
        else
        {
            svg_polyline(ss, pts, "#d62728");
        }
        if (!pts.empty())
        {
            float r = 0.001f * std::max(w, h);
            ss << "<circle cx=\"" << pts.front().x << "\" cy=\"" << pts.front().y << "\" r=\"" << r << "\" fill=\"#d62728\" />\n";
            ss << "<circle cx=\"" << pts.back().x << "\" cy=\"" << pts.back().y << "\" r=\"" << r << "\" fill=\"#d62728\" />\n";
        }
        if (show_seq_numbers)
        {
            Vec2 tp = move_label_pos(m);
            tp = internal_xy_to_plot_xy(tp, machineType);
            if (mirror_x)
                tp.x = b.maxx + b.minx - tp.x;
            if (mirror_y)
                tp.y = b.maxy + b.miny - tp.y;
            ss << "<text x=\"" << (tp.x + textNudge) << "\" y=\"" << (tp.y - textNudge)
               << "\" fill=\"#d62728\" font-size=\"" << textSize
               << "\" text-anchor=\"middle\" dominant-baseline=\"middle\">"
               << m.seqNum << "</text>\n";
        }
    }
    ss << "</svg>\n";
    std::ofstream out(path, std::ios::binary);
    out << ss.str();
}
