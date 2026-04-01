#pragma once

#ifndef __cplusplus
#error "cc_xy_host_bridge.h requires C++"
#endif

#include "cc_math.h"
#include "../mcu/cutter_comp_grblhal.h"

static inline vec2 cc_from_vec2(const Vec2 &v)
{
    return cc_v2(v.x, v.y);
}

static inline Vec2 cc_to_vec2(const vec2 &v)
{
    return v2(v.x, v.y);
}

static inline move2d cc_from_move2d(const Move2D &src)
{
    move2d dst = {};
    dst.p_0 = cc_from_vec2(src.p_0);
    dst.p_1 = cc_from_vec2(src.p_1);
    dst.center = cc_from_vec2(src.center);
    dst.startDir = cc_from_vec2(src.startDir);
    dst.endDir = cc_from_vec2(src.endDir);
    dst.radius = src.radius;
    dst.feed = src.feed;
    dst.z_0 = src.z_0;
    dst.z_1 = src.z_1;
    dst.lineNum = src.lnNum;
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
    dst.lnNum = src.lineNum;
    dst.type = (MotionType)src.type;
    dst.arcDir = (ArcDir)src.arcDir;
    dst.compMode = (CompMode)src.compMode;
    dst.valid = src.valid;
    return dst;
}