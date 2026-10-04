
/*
 * cc_simple_scan.h
 * Jason Titcomb 2026
 * MIT License – see LICENSE file in repository root
 */

#pragma once
// #include <stddef.h> // size_t
#include "cc_math.h"

// Token scan result for one line
struct ScanLine
{
    bool isMove = false;
    bool hasG = false;
    int32_t G = 0;
    bool hasX = false;
    float X = 0;
    bool hasY = false;
    float Y = 0;
    bool hasZ = false;
    float Z = 0;
    bool hasI = false;
    float I = 0;
    bool hasJ = false;
    float J = 0;
    bool hasR = false;
    float R = 0;
    bool hasF = false;
    float F = 0;
    bool hasD = false;
    float D = 0;
    bool hasT = false;
    int32_t T = 0;
    bool hasS = false;
    float S = 0;
    bool hasM = false;
    int32_t M = 0;
    bool hasP = false;
    float P = 0;

    // Special modal toggles
    bool sawG17 = false;
    bool sawG90 = false;
    bool sawG41 = false;
    bool sawG42 = false;
    bool sawG40 = false;
    bool sawG20 = false; // G20: inch
    bool sawG21 = false; // G21: mm

    // Motion mode tracking
    bool sawG0 = false;
    bool sawG1 = false;
    bool sawG2 = false;
    bool sawG3 = false;
    bool sawG4 = false;
    bool sawG91 = false;
};

// Minimal modal state
struct ModalState
{
    bool planeXY = true;      // G17
    bool absoluteMode = true; // G90/G91
    int motionG = 0;          // 0/1/2/3 modal
    CompSide comp = COMP_OFF;
    CompMode compMode = CM_NONE;
    float feed = 0.0f;
    float D_Register = 0;
    int32_t T_Register = 0;
    float speed = 0.0f;
    Vec2 pos{0, 0}; // current internal XY position; updated by interpret_to_move
    float z = 0.0f;
    // Units: true = inch, false = mm (default)
    bool inchMode = false;
    uint32_t lineNumber = 0; // input file line number
};

// -------------------------
// Tiny ASCII helpers
// -------------------------
static bool is_space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
static char up(char c) { return (c >= 'a' && c <= 'z') ? char(c - ('a' - 'A')) : c; }

static const char *skip_ws(const char *p)
{
    while (*p && is_space(*p))
        ++p;
    return p;
}

// Very small float parser: [-]ddd[.ddd]
// No exponent; good for typical G-code.
static const char *parse_float(const char *p, float &out)
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

static const char *parse_int(const char *p, int32_t &out)
{
    float f = 0;
    p = parse_float(p, f);
    out = (int32_t)f;
    return p;
}

// Remove comments and normalize whitespace.
// Supported comments:
// - '(...)' block comments
// - ';...;' comment spans (if closing ';' exists)
// - ';...' to end-of-line when no closing ';' exists
// Whitespace handling:
// - Leading/trailing whitespace removed
// - Multiple consecutive spaces compressed to single space
// Writes into dst with maxLen, always null-terminated.
static char *strip_comments(const char *src, char *dst, uint32_t maxLen)
{
    uint32_t w = 0;
    bool inParen = false;
    bool inSemi = false;
    bool lastWasSpace = true; // start true to skip leading whitespace
    for (uint32_t i = 0; src[i] && w + 1 < maxLen; ++i)
    {
        char c = src[i];

        if (!inParen && !inSemi && c == '(')
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

        if (c == ';')
        {
            inSemi = !inSemi;
            continue;
        }

        if (inSemi)
            continue;

        // Normalize whitespace: compress multiple spaces, skip leading
        if (is_space(c))
        {
            if (!lastWasSpace)
            {
                dst[w++] = ' ';
                lastWasSpace = true;
            }
            continue;
        }

        dst[w++] = c;
        lastWasSpace = false;
    }
    // Trim trailing space
    if (w > 0 && dst[w - 1] == ' ')
        w--;
    dst[w] = 0;
    return dst;
}

// Scan one line into ScanLine.
// Captures letters followed by number: G.. X.. Y.. I.. J.. R.. F.. N..
static void scan_line(const char *line, ScanLine &s)
{
    s = ScanLine{}; // reset
    const char *p = line;

    while (*p)
    {
        p = skip_ws(p);
        char c = up(*p);
        if (!c)
            break;

        if ((c >= 'A' && c <= 'Z'))
        {
            ++p;
            if (c == 'G')
            {
                s.hasG = true;
                p = parse_int(p, s.G);
                if (s.G == 0)
                    s.sawG0 = true;
                if (s.G == 1)
                    s.sawG1 = true;
                if (s.G == 2)
                    s.sawG2 = true;
                if (s.G == 3)
                    s.sawG3 = true;
                if (s.G == 4)
                    s.sawG4 = true;
                if (s.G == 17)
                    s.sawG17 = true;
                if (s.G == 20)
                    s.sawG20 = true;
                if (s.G == 21)
                    s.sawG21 = true;
                if (s.G == 90)
                    s.sawG90 = true;
                if (s.G == 91)
                    s.sawG91 = true;
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
                s.isMove = true;    
            }
            else if (c == 'Y')
            {
                s.hasY = true;
                p = parse_float(p, s.Y);
                s.isMove = true;
            }
            else if (c == 'Z')
            {
                s.hasZ = true;
                p = parse_float(p, s.Z);
                
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
            else if (c == 'D')
            {
                s.hasD = true;
                p = parse_float(p, s.D);
            }
            else if (c == 'T')
            {
                s.hasT = true;
                p = parse_int(p, s.T);
            }
            else if (c == 'S')
            {
                s.hasS = true;
                p = parse_float(p, s.S);
            }
            else if (c == 'M')
            {
                s.hasM = true;
                p = parse_int(p, s.M);
            }
            else if (c == 'P')
            {
                s.hasP = true;
                p = parse_float(p, s.P);
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
static bool arc_center_from_R(const Vec2 &p0, const Vec2 &p1, float R, ArcDir dir, Vec2 &outC)
{
    float r = fabsf(R);
    bool useShortArc = (R >= 0.0f);
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
        if (dir == ARC_CCW)
            return useShortArc ? (z > 0.0f) : (z < 0.0f);
        return useShortArc ? (z < 0.0f) : (z > 0.0f);
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
static Move2D interpret_move(const ScanLine &s, ModalState &modeState)
{
    // Update modal toggles first
    if (s.sawG17)
        modeState.planeXY = true;
    if (s.sawG20)
        modeState.inchMode = true;
    if (s.sawG21)
        modeState.inchMode = false;
    if (s.sawG90)
        modeState.absoluteMode = true;
    if (s.sawG91)
        modeState.absoluteMode = false;
    if (s.sawG40)
        modeState.comp = COMP_OFF;
    if (s.sawG41)
        modeState.comp = COMP_LEFT;
    if (s.sawG42)
        modeState.comp = COMP_RIGHT;
    if (s.hasF)
        modeState.feed = s.F;
    if (s.hasD)
        modeState.D_Register = s.D;
    if (s.hasS)
        modeState.speed = s.S;
    if (s.hasT)
        modeState.T_Register = s.T;

    // Update motion mode if explicitly provided
    if (s.sawG0)
        modeState.motionG = 0;
    else if (s.sawG1)
        modeState.motionG = 1;
    else if (s.sawG2)
        modeState.motionG = 2;
    else if (s.sawG3)
        modeState.motionG = 3;

    // Coordinates are parsed in machine-space, then mapped to internal XY space.
    Vec2 p0 = modeState.pos;
    Vec2 p1 = p0;
    float z0 = modeState.z;
    float z1 = z0;

    bool anyXYZ = false;
    if (s.hasX)
    {
        if (modeState.absoluteMode)
        {
            p1.x = s.X;
        }
        else
        {
            p1.x = p0.x + s.X;
        }
        anyXYZ = true;
    }
    if (s.hasY)
    {
        if (modeState.absoluteMode)
        {
            p1.y = s.Y;
        }
        else
        {
            p1.y = p0.y + s.Y;
        }
        anyXYZ = true;
    }
    if (s.hasZ)
    {
        if (modeState.absoluteMode)
        {
            z1 = s.Z;
        }
        else
        {
            z1 = z0 + s.Z;
        }
        anyXYZ = true;
    }

    Move2D out;
    out.lineNum = modeState.lineNumber;
    out.p_0 = p0;
    out.p_1 = p1;
    out.z_0 = z0;
    out.z_1 = z1;
    out.hasXY = (s.hasX || s.hasY);
    out.hasZ = s.hasZ;
    out.feed = modeState.feed;

    if (s.hasM && (s.M == 0 || s.M == 1))
    {
        out.type = MOT_EMPTY;
        out.pause_after = -1.0f;
        out.pauseKind = (s.M == 0) ? PAUSE_M0 : PAUSE_M1;
        return out;
    }

    if (s.sawG4 && s.hasP && s.P >= 0.0f)
    {
        out.type = MOT_EMPTY;
        out.pause_after = s.P;
        out.pauseKind = PAUSE_DWELL;
        return out;
    }

    if (modeState.compMode == CM_IN || modeState.compMode == CM_OUT)
    {
        out.compMode = CM_STEADY; // in a comp block but no change from previous move, so steady comp mode.
    }
    else
    {
        out.compMode = CM_NONE;
    }

    if (s.sawG41 || s.sawG42)
    {
        out.compMode = CM_IN;
        modeState.compMode = CM_IN;
    }
    else if (s.sawG40)
    {
        out.compMode = CM_OUT;
        modeState.compMode = CM_OUT;
    }

    if (modeState.motionG == 0 && anyXYZ)
    {
        out.type = MOT_RAPID;
        modeState.pos = p1;
        modeState.z = z1;
        return out;
    }

    if (modeState.motionG == 1 && anyXYZ)
    {
        out.type = MOT_LINE;
        modeState.pos = p1;
        modeState.z = z1;
        return out;
    }

    if ((modeState.motionG == 2 || modeState.motionG == 3) && anyXYZ)
    {
        out.type = MOT_ARC;
        out.arcDir = (modeState.motionG == 2) ? ARC_CW : ARC_CCW;
        if (s.hasP)
            out.turns = (s.P >= 1.0f && s.P == floorf(s.P)) ? (int32_t)s.P : 0;
        // out.hasXY = true; // arcs must have XY for center calculations;

        // Resolve center either from I/J or from R
        if (s.hasI || s.hasJ)
        {
            Vec2 ij = v2(s.hasI ? s.I : 0.0f, s.hasJ ? s.J : 0.0f); // I/J are already in internal XY space
            out.center = p0 + ij;                                   // I/J incremental
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

        modeState.pos = p1;
        modeState.z = z1;
        return out;
    }

    modeState.pos = p1;
    modeState.z = z1;

    return out;
}