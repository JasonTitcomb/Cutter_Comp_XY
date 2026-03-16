#pragma once

#include "cc_math.h"
#include "cc_xy.h"

static inline CcXyVec2 ccxy_from_vec2(const Vec2 &v)
{
    return ccxy_v2(v.x, v.y);
}

static inline Vec2 ccxy_to_vec2(const CcXyVec2 &v)
{
    return v2(v.x, v.y);
}

static inline CcXyCompSide ccxy_from_comp_side(CompSide side)
{
    if (side == COMP_LEFT)
        return CCXY_COMP_LEFT;
    if (side == COMP_RIGHT)
        return CCXY_COMP_RIGHT;
    return CCXY_COMP_OFF;
}

static inline CompSide ccxy_to_comp_side(CcXyCompSide side)
{
    if (side == CCXY_COMP_LEFT)
        return COMP_LEFT;
    if (side == CCXY_COMP_RIGHT)
        return COMP_RIGHT;
    return COMP_OFF;
}

static inline CcXyUnits ccxy_from_units(Units units)
{
    return (units == UNITS_INCH) ? CCXY_UNITS_INCH : CCXY_UNITS_MM;
}

static inline Units ccxy_to_units(CcXyUnits units)
{
    return (units == CCXY_UNITS_INCH) ? UNITS_INCH : UNITS_MM;
}

static inline CcXyMove2D ccxy_from_move2d(const Move2D &src)
{
    CcXyMove2D dst{};
    dst.p_0 = ccxy_from_vec2(src.p_0);
    dst.p_1 = ccxy_from_vec2(src.p_1);
    dst.center = ccxy_from_vec2(src.center);
    dst.startDir = ccxy_from_vec2(src.startDir);
    dst.endDir = ccxy_from_vec2(src.endDir);
    dst.radius = src.radius;
    dst.feed = src.feed;
    dst.z_0 = src.z_0;
    dst.z_1 = src.z_1;
    dst.seqNum = src.seqNum;
    dst.type = (uint8_t)src.type;
    dst.arcDir = (uint8_t)src.arcDir;
    dst.compMode = (uint8_t)src.compMode;
    dst.hasXY = src.hasXY;
    dst.hasZ = src.hasZ;
    dst.valid = src.valid;
    return dst;
}

static inline Move2D ccxy_to_move2d(const CcXyMove2D &src)
{
    Move2D dst{};
    dst.p_0 = ccxy_to_vec2(src.p_0);
    dst.p_1 = ccxy_to_vec2(src.p_1);
    dst.center = ccxy_to_vec2(src.center);
    dst.startDir = ccxy_to_vec2(src.startDir);
    dst.endDir = ccxy_to_vec2(src.endDir);
    dst.radius = src.radius;
    dst.feed = src.feed;
    dst.z_0 = src.z_0;
    dst.z_1 = src.z_1;
    dst.seqNum = src.seqNum;
    dst.type = (MotionType)src.type;
    dst.arcDir = (ArcDir)src.arcDir;
    dst.compMode = (CompMode)src.compMode;
    dst.hasXY = src.hasXY;
    dst.hasZ = src.hasZ;
    dst.valid = src.valid;
    return dst;
}