#pragma once

#include "cc_math.h"
#include "../mcu/cutter_comp.h"

static inline vec2 cc_from_vec2(const Vec2 &v)
{
    return cc_v2(v.x, v.y);
}

static inline Vec2 cc_to_vec2(const vec2 &v)
{
    return v2(v.x, v.y);
}

static inline comp_side cc_from_comp_side(CompSide side)
{
    if (side == COMP_LEFT)
        return CC_COMP_LEFT;
    if (side == COMP_RIGHT)
        return CC_COMP_RIGHT;
    return CC_COMP_OFF;
}

static inline CompSide cc_to_comp_side(comp_side side)
{
    if (side == CC_COMP_LEFT)
        return COMP_LEFT;
    if (side == CC_COMP_RIGHT)
        return COMP_RIGHT;
    return COMP_OFF;
}

static inline move2d cc_from_move2d(const Move2D &src)
{
    move2d dst{};
    dst.p_0 = cc_from_vec2(src.p_0);
    dst.p_1 = cc_from_vec2(src.p_1);
    dst.center = cc_from_vec2(src.center);
    dst.startDir = cc_from_vec2(src.startDir);
    dst.endDir = cc_from_vec2(src.endDir);
    dst.radius = src.radius;
    dst.feed = src.feed;
    dst.z_0 = src.z_0;
    dst.z_1 = src.z_1;
    dst.seqNum = src.seqNum;
    dst.type = (uint8_t)src.type;
    dst.arcDir = (uint8_t)src.arcDir;
    dst.compMode = (uint8_t)src.compMode;
    dst.valid = src.valid;
    return dst;
}

static inline Move2D cc_to_move2d(const move2d &src)
{
    Move2D dst{};
    dst.p_0 = cc_to_vec2(src.p_0);
    dst.p_1 = cc_to_vec2(src.p_1);
    dst.center = cc_to_vec2(src.center);
    dst.startDir = cc_to_vec2(src.startDir);
    dst.endDir = cc_to_vec2(src.endDir);
    dst.radius = src.radius;
    dst.feed = src.feed;
    dst.z_0 = src.z_0;
    dst.z_1 = src.z_1;
    dst.seqNum = src.seqNum;
    dst.type = (MotionType)src.type;
    dst.arcDir = (ArcDir)src.arcDir;
    dst.compMode = (CompMode)src.compMode;
    dst.valid = src.valid;
    return dst;
}