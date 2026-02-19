#pragma once
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
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

// -------------------- Emit helpers (host files) --------------------
static void emit_move_as_gcode(FILE *f, const Move2D &m)
{
    if (m.type == MOT_LINE || m.type == MOT_RAPID)
    {
        std::fprintf(f, "N%d %s X%.4f Y%.4f\n",
                     (int)m.seqNum, m.type == MOT_RAPID ? "G0" : "G1", m.p1.x, m.p1.y);
        return;
    }

    if (m.type == MOT_ARC)
    {
        float I = m.center.x - m.p0.x;
        float J = m.center.y - m.p0.y;
        std::fprintf(f, "N%d %s X%.4f Y%.4f I%.4f J%.4f\n",
                     (int)m.seqNum, (m.arcDir == ARC_CW) ? "G2" : "G3",
                     m.p1.x, m.p1.y, I, J);
        return;
    }
}

void write_gcode(const char *path, const std::vector<Move2D> &moves)
{
    // G-code file
    if (FILE *f = std::fopen(path, "wb"))
    {
        for (auto &m : moves)
        {
            if (!m.valid || m.type == MOT_EMPTY)
                continue;
            emit_move_as_gcode(f, m);
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
        pts.push_back(m.p0);
        pts.push_back(m.p1);
        return pts;
    }
    if (m.type == MOT_ARC)
    {
        float a0 = std::atan2(m.p0.y - m.center.y, m.p0.x - m.center.x);
        float a1 = std::atan2(m.p1.y - m.center.y, m.p1.x - m.center.x);
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

// Add this helper function after svg_polyline:
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
                      bool mirror_x = false,
                      bool mirror_y = false,
                      float tool_diameter = 0.0f)
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
        }
    }
    // Draw tool diameter circles along the offset profile (moves)
    if (tool_diameter > 0.0f)
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
    }
    ss << "</svg>\n";
    std::ofstream out(path, std::ios::binary);
    out << ss.str();
}
