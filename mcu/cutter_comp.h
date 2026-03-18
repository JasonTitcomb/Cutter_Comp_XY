/*
 * cc_xy_core.h
 * Jason Titcomb 2026
 * MIT License - see LICENSE file in repository root
 *
 * Thin grblHAL-oriented runner for the standalone cc_xy engine.
 * Uses fixed-size callbacks and move structs only.
 */

#ifndef CUTTER_COMP_H
#define CUTTER_COMP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC_IN_CAP 2
#define CC_INSERT_CAP 1

#ifndef CC_ENABLE_LOOKAHEAD
#define CC_ENABLE_LOOKAHEAD 1
#endif

#ifndef CC_LOOKAHEAD_CAP
#define CC_LOOKAHEAD_CAP 8
#endif

#ifndef CC_LOOKAHEAD_STEPS
#define CC_LOOKAHEAD_STEPS 4
#endif

#ifndef CC_LA_TARGET_BATCH_EMIT
#define CC_LA_TARGET_BATCH_EMIT 3
#endif

#ifndef CC_LA_TRIM_OVERLAP
#define CC_LA_TRIM_OVERLAP (CC_LOOKAHEAD_STEPS + 2)
#endif

#ifndef CC_LA_EMIT_HOLDBACK
#define CC_LA_EMIT_HOLDBACK CC_LA_TRIM_OVERLAP
#endif

#ifndef CC_LA_MIN_PENDING
#define CC_LA_MIN_PENDING (CC_LA_EMIT_HOLDBACK + CC_LA_TARGET_BATCH_EMIT)
#endif

#ifndef CC_OUT_CAP
#if CC_ENABLE_LOOKAHEAD
#define CC_OUT_CAP (CC_LOOKAHEAD_CAP + 1)
#else
#define CC_OUT_CAP 2
#endif
#endif


typedef struct
{
    float x;
    float y;
} vec2;

typedef enum
{
    CC_CM_NONE = 0,
    CC_CM_IN = 1,
    CC_CM_STEADY = 2,
    CC_CM_OUT = 3
} comp_state;

typedef enum
{
    CC_MOT_EMPTY = 0,
    CC_MOT_RAPID = 1,
    CC_MOT_LINE = 2,
    CC_MOT_ARC = 3
} motion_type;

typedef enum
{
    CC_ARC_CW = 0,
    CC_ARC_CCW = 1
} arc_dir;

typedef enum
{
    CC_IT_NONE = 0,
    CC_IT_TANGENT = 1,
    CC_IT_INTERSECT = 2
} intersect_type;

typedef enum
{
    CC_COMP_OFF = 0,
    CC_COMP_LEFT = 1,
    CC_COMP_RIGHT = -1
} comp_side;

typedef enum
{
    CC_OK = 0,
    CC_ERROR,
    CC_ARC_RADIUS_MISMATCH,
    CC_INVALID_MOVE,
    CC_COMP_MOVE_TOO_SHORT,
    CC_ARC_LT_TOOL_RAD,
    CC_FLIPPED_ARC,
    CC_COMP_IN_CROSSING,
    CC_COMP_OUT_CROSSING,
    CC_UNRESOLVED_GAP,
    CC_OUTPUT_BUFFER_OVERFLOW
} cc_comp_status;


typedef struct
{
    vec2 p_0;
    vec2 p_1;
    vec2 center;
    vec2 startDir;
    vec2 endDir;
    float radius;
    float feed;
    float z_0;
    float z_1;
    uint32_t seqNum;
    uint8_t type;
    uint8_t arcDir;
    uint8_t compMode;
    bool valid;
} move2d;

typedef void (*cc_err_cb)(const char *message, cc_comp_status err, uint32_t seqNum);
typedef void (*emit_move_cb)(const move2d *move);

typedef enum
{
    CC_JT_NONE = 0,
    CC_JT_TRIM_TO_INTERSECTION,
    CC_JT_EXTEND_TO_INTERSECTION,
    CC_JT_ROLL_AROUND,
 } junction_type;

typedef struct
{
    junction_type jtype;
    vec2 p;
} junction;

typedef struct
{
    float a0;
    float a1;
    uint8_t dir;
} arc_angles;

typedef struct
{
    cc_comp_status status;

    float toolR;
    int8_t toolSign;
    comp_side compState;
    uint32_t lastSeqNum;
    
    int inHead;
    int inCount;
    int outHead;
    int outCount;
    bool hasCompError;
    bool havePrevMove;
    
    move2d prevOff;
    move2d input_buffer[CC_IN_CAP];
    move2d output_buffer[CC_OUT_CAP];
#if CC_ENABLE_LOOKAHEAD
    move2d lookahead_buffer[CC_LOOKAHEAD_CAP];
    int lookahead_count;
#endif
} cc_context;

vec2 cc_v2(float x, float y);

void cc_api_init(float radius,emit_move_cb emitCb, cc_err_cb errCb);
cc_comp_status cc_api_process_move(const move2d *move);
void cc_api_set_comp(comp_side side);

#ifdef __cplusplus
}
#endif

#endif // CUTTER_COMP_H