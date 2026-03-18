#pragma once

#include "../mcu/cutter_comp.h"
#include "../mcu/grbl_data_portable.h"

/*
 * mc_line / mc_arc are the gateway to the motion controller.
 *
 * In grblHAL: these are the real planner entry points (declared in motion_control.h).
 * In the host test harness: implementations in main_host.cpp capture moves for SVG/comparison.
 *
 * cc_emit_via_mc() is registered as the cc_api emit callback; it converts a
 * move2d into grbl-style arguments and dispatches through mc_line / mc_arc so
 * the same code path can be dropped into grblHAL unchanged.
 *
 * Call convention mirrors grblHAL:
 *   mc_arc(..., turns): positive = CCW, negative = CW
 *   ijk[]: offsets from current position to arc center
 * 
 * In grblhal, do this when in cutter comp mode:
 * cc_mc_set_plan_data(&plan_data);
 * ok = cc_api_process_move(&xy_move);
 * cc_mc_set_plan_data(0);
 * 
 */

#ifdef __cplusplus
extern "C" {
#endif

bool mc_line(float *xyz, plan_line_data_t *pl_data);
void mc_arc(float *xyz, plan_line_data_t *pl_data, float *position, float *ijk, float radius, plane_t plane, int32_t turns);

#ifdef __cplusplus
}
#endif
static plan_line_data_t *cc_mc_active_plan_data = 0;


cc_comp_status cc_mc_line_in(gc_ccomp_t cc,float *xyz, plan_line_data_t *pl_data){
    cc_mc_active_plan_data = pl_data;
    if(cc.side == CComp_Left || cc.side == CComp_Right){
            return cc_api_process_move(nullptr);
    }
    mc_line(xyz, pl_data);
    return CC_OK;
}

cc_comp_status cc_mc_arc_in(gc_ccomp_t cc,float *xyz, plan_line_data_t *pl_data, float *position, float *ijk, float radius, plane_t plane, int32_t turns){
    cc_mc_active_plan_data = pl_data;
    if(cc.side == CComp_Left || cc.side == CComp_Right){
        return cc_api_process_move(nullptr);
    }
    mc_arc(xyz, pl_data, position, ijk, radius, plane, turns);
    return CC_OK;
}


static inline void cc_emit_via_mc(const move2d *mv)
{
    plan_line_data_t local_pl_data = {0};
    plan_line_data_t *pl_data = &local_pl_data;

    if (!mv)
        return;

    if (cc_mc_active_plan_data)
        local_pl_data = *cc_mc_active_plan_data;

    local_pl_data.feed_rate = mv->feed;
    local_pl_data.condition.rapid_motion = (mv->type == CC_MOT_RAPID) ? 1 : 0;

    if (mv->type == CC_MOT_LINE || mv->type == CC_MOT_RAPID)
    {
        float xyz[N_AXIS] = {0};
        xyz[0] = mv->p_1.x;
        xyz[1] = mv->p_1.y;
        xyz[2] = mv->z_1;
        mc_line(xyz, pl_data);
    }
    else if (mv->type == CC_MOT_ARC)
    {
        float xyz[N_AXIS] = {0};
        xyz[0] = mv->p_1.x;
        xyz[1] = mv->p_1.y;
        xyz[2] = mv->z_1;

        float position[N_AXIS] = {0};
        position[0] = mv->p_0.x;
        position[1] = mv->p_0.y;
        position[2] = mv->z_0;

        float ijk[3] = {0};
        ijk[0] = mv->center.x - mv->p_0.x;
        ijk[1] = mv->center.y - mv->p_0.y;
        ijk[2] = 0.0f;

        plane_t plane = {0};
        plane.axis_0 = 0;
        plane.axis_1 = 1;
        plane.axis_linear = 2;

        int32_t turns = (mv->arcDir == CC_ARC_CCW) ? 1 : -1;
        mc_arc(xyz, pl_data, position, ijk, mv->radius, plane, turns);
    }
}

// typedef enum {TODO: make sure this aligns
//     CComp_Off = 0, //!< 0 - G40 - Default, must be zero
//     CComp_Left,    //!< 1 - G41, G41.1
//     CComp_Right    //!< 2 - G42, G42.1
// } ccomp_mode_t;
static inline comp_side cc_from_comp_side(int16_t side)
{
    if (side == CComp_Left)
        return CC_COMP_LEFT;
    if (side == CComp_Right)
        return CC_COMP_RIGHT;
    return CC_COMP_OFF;
}

static inline int16_t cc_to_comp_side(comp_side side)
{
    if (side == CC_COMP_LEFT)
        return CComp_Left;
    if (side == CC_COMP_RIGHT)
        return CComp_Right;
    return CComp_Off;
}
