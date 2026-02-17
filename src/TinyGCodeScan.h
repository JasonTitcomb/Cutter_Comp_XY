#pragma once
#include <stddef.h> // size_t
#include "Common2D.h"

// Token scan result for one line
struct ScanLine
{
    //copy of codeline for testing only
    //char raw[160] = {0};
    bool hasN = false;
    int32_t N = 0;

    bool hasG = false;
    int32_t G = 0; // last G word wins (fine for our subset)
    bool hasX = false;
    float X = 0;
    bool hasY = false;
    float Y = 0;
    bool hasI = false;
    float I = 0;
    bool hasJ = false;
    float J = 0;
    bool hasR = false;
    float R = 0;
    bool hasF = false;
    float F = 0;

    // Special modal toggles
    bool sawG17 = false;
    bool sawG90 = false;
    bool sawG41 = false;
    bool sawG42 = false;
    bool sawG40 = false;
};

// Minimal modal state
struct ModalState
{
    bool planeXY = true; // G17
    bool absXY = true;   // G90 (only mode supported here)
    int motionG = 0;     // 0/1/2/3 modal
    CompSide comp = COMP_OFF;
    float feed = 0.0f;

    Vec2 pos{0, 0}; // current XY
};

// -------------------------
// Tiny ASCII helpers
// -------------------------
static inline bool is_space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
static inline char up(char c) { return (c >= 'a' && c <= 'z') ? char(c - ('a' - 'A')) : c; }

static inline const char *skip_ws(const char *p)
{
    while (*p && is_space(*p))
        ++p;
    return p;
}

// Very small float parser: [-]ddd[.ddd]
// No exponent; good for typical G-code.
static inline const char *parse_float(const char *p, float &out)
{
    p = skip_ws(p);
    bool neg = false;
    if (*p == '+' || *p == '-')
    {
        neg = (*p == '-');
        ++p;
    }

    uint32_t ip = 0;
    bool any = false;
    while (*p >= '0' && *p <= '9')
    {
        any = true;
        ip = ip * 10u + uint32_t(*p - '0');
        ++p;
    }

    float val = float(ip);

    if (*p == '.')
    {
        ++p;
        float place = 0.1f;
        while (*p >= '0' && *p <= '9')
        {
            any = true;
            val += float(*p - '0') * place;
            place *= 0.1f;
            ++p;
        }
    }

    if (!any)
    {
        out = 0;
        return p;
    }
    out = neg ? -val : val;
    return p;
}

static inline const char *parse_int(const char *p, int32_t &out)
{
    float f = 0;
    p = parse_float(p, f);
    out = (int32_t)f;
    return p;
}

// Remove comments: anything after ';' and any '(...)' blocks.
// Writes into dst with maxLen, always null-terminated.
static inline char *strip_comments(const char *src, char *dst, size_t maxLen)
{
    size_t w = 0;
    bool inParen = false;
    for (size_t i = 0; src[i] && w + 1 < maxLen; ++i)
    {
        char c = src[i];
        if (!inParen && c == ';')
            break;
        if (!inParen && c == '(')
        {
            inParen = true;
            continue;
        }
        if (inParen)
        {
            if (c == ')')
                inParen = false;
            continue;
        }
        dst[w++] = c;
    }
    dst[w] = 0;
    return dst;
}

// Scan one line into ScanLine.
// Captures letters followed by number: G.. X.. Y.. I.. J.. R.. F.. N..
static inline void scan_line(const char *line, ScanLine &s)
{
    s = ScanLine{}; // reset
    const char *p = line;
    
    // Copy raw line for testing only
    //strncpy(s.raw, line, sizeof(s.raw) - 1);
    //s.raw[sizeof(s.raw) - 1] = '\0';

    while (*p)
    {
        p = skip_ws(p);
        char c = up(*p);
        if (!c)
            break;

        if ((c >= 'A' && c <= 'Z'))
        {
            ++p;
            if (c == 'N')
            {
                s.hasN = true;
                p = parse_int(p, s.N);
            }
            else if (c == 'G')
            {
                s.hasG = true;
                p = parse_int(p, s.G);
                if (s.G == 17)
                    s.sawG17 = true;
                if (s.G == 90)
                    s.sawG90 = true;
                if (s.G == 41)
                    s.sawG41 = true;
                if (s.G == 42)
                    s.sawG42 = true;
                if (s.G == 40)
                    s.sawG40 = true;
            }
            else if (c == 'X')
            {
                s.hasX = true;
                p = parse_float(p, s.X);
            }
            else if (c == 'Y')
            {
                s.hasY = true;
                p = parse_float(p, s.Y);
            }
            else if (c == 'I')
            {
                s.hasI = true;
                p = parse_float(p, s.I);
            }
            else if (c == 'J')
            {
                s.hasJ = true;
                p = parse_float(p, s.J);
            }
            else if (c == 'R')
            {
                s.hasR = true;
                p = parse_float(p, s.R);
            }
            else if (c == 'F')
            {
                s.hasF = true;
                p = parse_float(p, s.F);
            }
            else
            {
                // Unknown word: skip a number if present
                float dummy = 0;
                p = parse_float(p, dummy);
            }
        }
        else
        {
            ++p;
        }
    }
}

// Compute center from R for G2/G3 arc in XY.
// Chooses the center matching CW/CCW; assumes "shorter" arc when ambiguous.
static inline bool arc_center_from_R(const Vec2 &p0, const Vec2 &p1, float R, ArcDir dir, Vec2 &outC)
{
    float r = fabsf(R);
    Vec2 chord = p1 - p0;
    float d = len(chord);
    if (d < TOL)
        return false;
    if (d > 2.0f * r + 1e-5f)
        return false;

    Vec2 M = (p0 + p1) * 0.5f;
    float half = 0.5f * d;
    float h = sqrtf(fmaxf(0.0f, r * r - half * half));

    Vec2 u = chord * (1.0f / d);
    Vec2 perp = leftNormal(u);

    Vec2 C1 = M + perp * h;
    Vec2 C2 = M - perp * h;

    auto ok = [&](const Vec2 &C)
    {
        Vec2 a = p0 - C, b = p1 - C;
        float z = cross(a, b);
        return (dir == ARC_CCW) ? (z > 0) : (z < 0);
    };

    bool ok1 = ok(C1), ok2 = ok(C2);
    if (ok1 && !ok2)
        outC = C1;
    else if (ok2 && !ok1)
        outC = C2;
    else
        outC = C1; // ambiguous: pick one
    return true;
}

// Turn a scanned line into a Move2D (or MOT_EMPTY if no XY motion).
// Updates modal state (pos, motion mode, comp, feed).
static inline Move2D interpret_to_move(const ScanLine &s, ModalState &m)
{
    // Update modal toggles first
    if (s.sawG17)
        m.planeXY = true;
    if (s.sawG90)
        m.absXY = true;
    if (s.sawG40)
        m.comp = COMP_OFF;
    if (s.sawG41)
        m.comp = COMP_LEFT;
    if (s.sawG42)
        m.comp = COMP_RIGHT;
    if (s.hasF)
        m.feed = s.F;

    // Update motion mode if explicitly provided
    if (s.hasG)
    {
        if (s.G == 0 || s.G == 1 || s.G == 2 || s.G == 3)
            m.motionG = s.G;
    }

    // Absolute XY only (tiny subset)
    Vec2 p0 = m.pos;
    Vec2 p1 = p0;

    bool anyXY = false;
    if (s.hasX)
    {
        p1.x = s.X;
        anyXY = true;
    }
    if (s.hasY)
    {
        p1.y = s.Y;
        anyXY = true;
    }

    if (!anyXY)
    {
        return Move2D{}; // MOT_EMPTY
    }

    Move2D out;
    if (s.hasN)
        out.seqNum = s.N;

    out.p0 = p0;
    out.p1 = p1;
    out.feed = m.feed;
    out.rapid = (m.motionG == 0);

    if (s.sawG41 || s.sawG42)
        out.compMode = COMP_MODE_IN;
    else if (s.sawG40)
        out.compMode = COMP_MODE_OUT;

    if (m.motionG == 1)
    {
        out.type = MOT_LINE;
        m.pos = p1;
        return out;
    }

    if (m.motionG == 0)
    {
        out.type = MOT_RAPID;
        m.pos = p1;
        return out;
    }

    if (m.motionG == 2 || m.motionG == 3)
    {
        out.type = MOT_ARC;
        out.arcDir = (m.motionG == 2) ? ARC_CW : ARC_CCW;

        // Resolve center either from I/J or from R
        if (s.hasI || s.hasJ)
        {
            Vec2 ij = v2(s.hasI ? s.I : 0.0f, s.hasJ ? s.J : 0.0f);
            out.center = p0 + ij; // I/J incremental
            out.radius = len(p0 - out.center);
        }
        else if (s.hasR)
        {
            Vec2 C;
            if (!arc_center_from_R(p0, p1, s.R, out.arcDir, C))
            {
                out.type = MOT_LINE; // fallback
            }
            else
            {
                out.center = C;
                out.radius = len(p0 - C);
            }
        }
        else
        {
            out.type = MOT_LINE; // fallback
        }

        m.pos = p1;
        return out;
    }

    // Unknown motion: treat as line
    out.type = MOT_LINE;
    m.pos = p1;

    return out;
}
