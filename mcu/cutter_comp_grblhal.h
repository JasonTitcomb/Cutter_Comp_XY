/*
 * SHIM BETWEEN GRBLHAL AND THE CUTTER COMPENSATION CORE
 * Overall, this file serves as a bridge between grblHAL and the cutter compensation core, enabling them to work together seamlessly while keeping their internal implementations decoupled.
 * MIT License - see LICENSE file in repository root
 */
#ifndef CUTTER_COMP_GRBLHAL_H
#define CUTTER_COMP_GRBLHAL_H
#include "config.h"

#ifdef __cplusplus
extern "C"
{
#endif

#if CUTTER_COMP_ENABLE

#include "../mcu/cutter_comp.h"
#include "../mcu/grbl_data_portable.h"

    // forward declarations
    // these are implemented in grblhal's motion_control.c, but declared here so they can be called from the cc_emit_via_mc callback.
    bool mc_line(float *xyz, plan_line_data_t *pl_data);
    void mc_arc(float *xyz, plan_line_data_t *pl_data, float *position, float *ijk, float radius, plane_t plane, int32_t turns);
    void report_message(const char *msg, message_type_t type);
    static plan_line_data_t *cc_mc_active_plan_data = 0;
    static float cc_mc_input_pos[N_AXIS] = {0};

    static void cc_report_error(status_code_t err, uint32_t seqNum)
    {
        if (err != Status_OK)
        {
            char msg[40];

            strcpy(msg, "CC_ERROR:");
            strcat(msg, uitoa(err));
            strcat(msg, ", N");
            strcat(msg, uitoa(seqNum));
            report_message(msg, Message_Error);
        }
    }

    static inline uint8_t cc_mc_comp_mode_from_input(gc_ccomp_t cc)
    {
        if (cc.side == CComp_Left || cc.side == CComp_Right)
            return (uint8_t)(cc.first_move ? CC_CM_IN : CC_CM_STEADY);

        if (cc_api_get_comp() != CC_COMP_OFF)
            return (uint8_t)CC_CM_OUT;

        return (uint8_t)CC_CM_NONE;
    }

    static inline move2d cc_mc_to_move2d(gc_ccomp_t cc,
                                         float *xyz,
                                         plan_line_data_t *pl_data,
                                         float *position,
                                         float *ijk,
                                         float radius,
                                         int32_t turns,
                                         bool is_arc)
    {
        move2d mv = {0};
        mv.p_0 = cc_v2(cc_mc_input_pos[0], cc_mc_input_pos[1]);
        mv.p_1 = cc_v2(xyz[0], xyz[1]);
        mv.z_0 = cc_mc_input_pos[2];
        mv.z_1 = xyz[2];
        mv.feed = pl_data ? pl_data->feed_rate : 0.0f;
        mv.compMode = cc_mc_comp_mode_from_input(cc);
        mv.valid = true;

        if (is_arc)
        {
            mv.type = CC_MOT_ARC;
            if (position)
            {
                mv.p_0 = cc_v2(position[0], position[1]);
                mv.z_0 = position[2];
            }
            if (ijk)
            {
                mv.center = cc_v2(mv.p_0.x + ijk[0], mv.p_0.y + ijk[1]);
            }
            mv.radius = radius;
            mv.arcDir = (uint8_t)((turns >= 0) ? CC_ARC_CCW : CC_ARC_CW);
        }
        else
        {
            mv.type = (uint8_t)((pl_data && pl_data->condition.rapid_motion) ? CC_MOT_RAPID : CC_MOT_LINE);
        }

        cc_mc_input_pos[0] = xyz[0];
        cc_mc_input_pos[1] = xyz[1];
        cc_mc_input_pos[2] = xyz[2];
        return mv;
    }

    // replaces mc_line when cutter compensation is active. If compensation is not active, passes through to mc_line.
    status_code_t cc_mc_line_in(gc_ccomp_t cc, float *xyz, plan_line_data_t *pl_data)
    {
        cc_mc_active_plan_data = pl_data;

        if (cc.side == CComp_Off && cc_api_get_comp() == CC_COMP_OFF)
        {
            cc_mc_input_pos[0] = xyz[0];
            cc_mc_input_pos[1] = xyz[1];
            cc_mc_input_pos[2] = xyz[2];
            mc_line(xyz, pl_data);
            return Status_OK;
        }

        if (cc_api_get_comp() == CC_COMP_OFF && (cc.side == CComp_Left || cc.side == CComp_Right))
        {
            // convert from grbl-style comp mode to cc style and set in API
            comp_side side = CC_COMP_OFF;
            if (cc.side == CComp_Left)
                side = CC_COMP_LEFT;
            else if (cc.side == CComp_Right)
                side = CC_COMP_RIGHT;

            cc_api_set_comp(side);
        }

        move2d mv = cc_mc_to_move2d(cc, xyz, pl_data, 0, 0, 0.0f, 0, false);
        status_code_t st = cc_api_process_move(&mv);
        if (st != Status_OK)
            return st;

        if (cc.side == CComp_Off)
        {
            st = cc_api_process_move(0);
            if (st != Status_OK)
                return st;
            cc_api_set_comp(CC_COMP_OFF);
            report_message("CC_Off", Message_Info);
        }

        return Status_OK;
    }

    // replaces mc_arc when cutter compensation is active. If compensation is not active, passes through to mc_arc.
    status_code_t cc_mc_arc_in(gc_ccomp_t cc, float *xyz, plan_line_data_t *pl_data, float *position, float *ijk, float radius, plane_t plane, int32_t turns)
    {
        cc_mc_active_plan_data = pl_data;

        if (cc.side == CComp_Off && cc_api_get_comp() == CC_COMP_OFF)
        {
            cc_mc_input_pos[0] = xyz[0];
            cc_mc_input_pos[1] = xyz[1];
            cc_mc_input_pos[2] = xyz[2];
            mc_arc(xyz, pl_data, position, ijk, radius, plane, turns);
            return Status_OK;
        }

        if ((cc.side == CComp_Left || cc.side == CComp_Right))
        {
            comp_side side = CC_COMP_OFF;
            if (cc.side == CComp_Left)
                side = CC_COMP_LEFT;
            else if (cc.side == CComp_Right)
                side = CC_COMP_RIGHT;
            cc_api_set_comp(side);
            cc.first_move = false;
        }

        move2d mv = cc_mc_to_move2d(cc, xyz, pl_data, position, ijk, radius, turns, true);
        status_code_t st = cc_api_process_move(&mv);
        if (st != Status_OK)
            return st;

        if (cc.side == CComp_Off) // this should never happen for arcs, but just in case
        {
            st = cc_api_process_move(0);
            if (st != Status_OK)
                return st;
            cc_api_set_comp(CC_COMP_OFF);
        }

        return Status_OK;
    }

    static void cc_error_cb(const char *message, status_code_t err, uint32_t seqNum)
    {
        // what should i do here?
        // I don't have a real "host" to report errors to, and the API requires me to provide an error callback.
        // For now, I'll just ignore it, but in a real grblHAL environment, this could be hooked up to grbl's error reporting system.
        (void)(message);
        (void)(err);
        (void)(seqNum);
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
#endif
#ifdef __cplusplus
}
#endif
#endif