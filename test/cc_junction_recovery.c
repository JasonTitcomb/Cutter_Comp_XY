#include <stdio.h>

#include "../mcu/cutter_comp.c"

static move2d line_move(float x0, float y0, float x1, float y1)
{
    move2d move = {0};
    move.type = CC_MOT_LINE;
    move.compMode = CC_CM_STEADY;
    move.valid = true;
    move.p_0 = cc_v2(x0, y0);
    move.p_1 = cc_v2(x1, y1);
    cc_update_vectors(&move);
    return move;
}

static int emitted_count;
static int inversion_error_count;
static uint32_t inversion_error_line;

static void capture_inversion_error(cc_status_code_t status, msg_type_t severity, uint32_t lineNumber)
{
    if (status == cc_status_InvalidMove && severity == CC_MSG_ERROR)
    {
        inversion_error_count++;
        inversion_error_line = lineNumber;
    }
}

static bool check_early_inversion(bool reversePrevious, bool lookahead)
{
    cc_context ctx;
    move2d previous = reversePrevious ? line_move(0.0f, 0.0f, 0.5f, 0.0f)
                                      : line_move(-3.0f, 0.0f, 0.0f, 0.0f);
    move2d current = reversePrevious ? line_move(0.5f, 0.0f, 0.5f, 3.0f)
                                     : line_move(0.0f, 0.0f, 0.0f, 0.5f);
    move2d output;
    previous.lineNum = 10;
    current.lineNum = 20;
    cc_api_init(1.0f, CC_UNITS_MM, NULL, capture_inversion_error);
    cc_init_internal(&ctx, 1.0f);
    ctx.compSide = CC_COMP_LEFT;
    ctx.compMode = CC_CM_STEADY;
    ctx.lookaheadEnabled = lookahead;
    inversion_error_count = 0;
    inversion_error_line = 0;
    if (!cc_push_in(&ctx, &previous) || !cc_process(&ctx) ||
        !cc_push_in(&ctx, &current))
        return false;

    bool processed = cc_process(&ctx);
    if (lookahead)
        return processed && !ctx.stopErr && inversion_error_count == 0;

    if (processed || !ctx.stopErr || ctx.status != cc_status_InvalidMove ||
        inversion_error_count != 1 ||
        inversion_error_line != (reversePrevious ? 10u : 20u) ||
        cc_pop_out(&ctx, &output))
        return false;

    cc_flush(&ctx);
    return !cc_process(&ctx) && !cc_pop_out(&ctx, &output) &&
           inversion_error_count == 1;
}

static bool count_emitted_move(const move2d *move)
{
    (void)move;
    emitted_count++;
    return true;
}

static bool check_drain_filter(void)
{
    move2d move = line_move(0.0f, 0.0f, 0.0f, 0.0f);
    cc_api_init(0.0f, CC_UNITS_MM, count_emitted_move, NULL);
    emitted_count = 0;
    cc_push_out(&g_core_ctx, &move);
    move.type = CC_MOT_RAPID;
    cc_push_out(&g_core_ctx, &move);
    move.type = CC_MOT_ARC;
    move.center = cc_v2(-1.0f, 0.0f);
    move.radius = 1.0f;
    move.startDir = cc_v2(0.0f, 1.0f);
    move.endDir = move.startDir;
    cc_push_out(&g_core_ctx, &move);
    if (!cc_core_drain() || emitted_count != 0)
        return false;

    move = line_move(0.0f, 0.0f, 0.0f, 0.0f);
    move.z_1 = -1.0f;
    cc_update_vectors(&move);
    cc_push_out(&g_core_ctx, &move);
    move.type = CC_MOT_RAPID;
    move.z_0 = -1.0f;
    move.z_1 = 0.0f;
    cc_push_out(&g_core_ctx, &move);
    move = line_move(0.0f, 0.0f, 1.0f, 0.0f);
    cc_push_out(&g_core_ctx, &move);
    if (!cc_core_drain() || emitted_count != 3)
        return false;

    move = line_move(1.0f, 0.0f, 0.0f, 1.0f);
    move.type = CC_MOT_ARC;
    move.radius = 1.0f;
    move.arcDir = CC_ARC_CCW;
    cc_update_vectors(&move);
    cc_push_out(&g_core_ctx, &move);
    move.type = CC_MOT_EMPTY;
    move.p_0 = move.p_1;
    move.pause_after = -1.0f;
    cc_push_out(&g_core_ctx, &move);
    move.pause_after = 0.5f;
    cc_push_out(&g_core_ctx, &move);
    return cc_core_drain() && emitted_count == 6 && !g_core_ctx.stopErr;
}

static bool check_geometry_tolerances(void)
{
    move2d move = {0};
    float start = 0.5f;
    float end = 1.0f;

    move.type = CC_MOT_LINE;
    move.p_0 = cc_v2(0.0f, 0.0f);
    move.p_1 = cc_v2(CC_TOL * 0.5f, CC_TOL * 0.5f);
    move.z_1 = CC_TOL * 0.5f;
    cc_update_vectors(&move);
    if (move.hasXY || move.hasZ)
        return false;

    move.p_1 = cc_v2(CC_TOL * 0.8f, CC_TOL * 0.8f);
    move.z_1 = CC_TOL * 1.5f;
    cc_update_vectors(&move);
    if (!move.hasXY || !move.hasZ)
        return false;

    if (!cc_angle_on_sweep_ccw(
            start,
            end,
            start - 0.5f * CC_ANGLE_EPS) ||
        cc_angle_on_sweep_ccw(
            start,
            end,
            start - 2.0f * CC_ANGLE_EPS) ||
        !cc_angle_on_sweep_ccw(
            start,
            end,
            end + 0.5f * CC_ANGLE_EPS) ||
        cc_angle_on_sweep_ccw(
            start,
            end,
            end + 2.0f * CC_ANGLE_EPS))
        return false;

    return true;
}

static float last_emitted_z;
static int overflow_error_count;

static bool record_emitted_move(const move2d *move)
{
    emitted_count++;
    last_emitted_z = move->z_1;
    return true;
}

static void capture_overflow_error(cc_status_code_t status, msg_type_t severity, uint32_t lineNumber)
{
    (void)lineNumber;
    if (status == cc_status_OutputBufferOverflow && severity == CC_MSG_ERROR)
        overflow_error_count++;
}

static move2d z_move(float x, float y, float z0, float z1)
{
    move2d move = line_move(x, y, x, y);
    move.z_0 = z0;
    move.z_1 = z1;
    return move;
}

static bool check_flush_with_trailing_z_moves(bool lookahead, bool retractOnCompOff)
{
    move2d move;
    int i;

    cc_api_init(1.0f, CC_UNITS_MM, record_emitted_move, capture_overflow_error);
    cc_api_set_lookahead_enabled(lookahead);
    emitted_count = 0;
    overflow_error_count = 0;
    last_emitted_z = 0.0f;

    cc_api_set_comp(CC_COMP_LEFT);
    move = line_move(0.0f, -5.0f, 0.0f, 0.0f);
    if (cc_api_process_move(&move) != cc_status_OK)
        return false;

    cc_api_set_comp(CC_COMP_LEFT);
    for (i = 0; i < CC_LOOKAHEAD_CAP; ++i)
    {
        float x0 = (float)(i * 10);
        move = line_move(x0, (i & 1) ? 1.0f : 0.0f, x0 + 10.0f, (i & 1) ? 0.0f : 1.0f);
        if (cc_api_process_move(&move) != cc_status_OK)
            return false;
    }

    move = z_move(80.0f, 0.0f, 0.0f, 1.0f);
    if (cc_api_process_move(&move) != cc_status_OK)
        return false;

    if (retractOnCompOff)
        cc_api_set_comp(CC_COMP_OFF);
    move = z_move(80.0f, 0.0f, 1.0f, 25.0f);
    if (cc_api_process_move(&move) != cc_status_OK)
        return false;

    return cc_api_process_move(NULL) == cc_status_OK &&
           overflow_error_count == 0 && emitted_count > CC_LOOKAHEAD_CAP &&
           fabsf(last_emitted_z - 25.0f) <= CC_TOL;
}

#define MAX_RECORDED_ARCS 8
static move2d recorded_arcs[MAX_RECORDED_ARCS];
static int recorded_arc_count;
static int invalid_move_error_count;

static bool record_emitted_arc(const move2d *move)
{
    emitted_count++;
    if (move->type == CC_MOT_ARC && recorded_arc_count < MAX_RECORDED_ARCS)
        recorded_arcs[recorded_arc_count++] = *move;
    return true;
}

static void capture_invalid_move_error(cc_status_code_t status, msg_type_t severity, uint32_t lineNumber)
{
    (void)lineNumber;
    if (status == cc_status_InvalidMove && severity == CC_MSG_ERROR)
        invalid_move_error_count++;
}

static move2d arc_move(vec2 p0, vec2 p1, vec2 center, arc_dir dir, float z0, float z1)
{
    move2d move = {0};
    move.type = CC_MOT_ARC;
    move.compMode = CC_CM_STEADY;
    move.valid = true;
    move.arcDir = (uint8_t)dir;
    move.p_0 = p0;
    move.p_1 = p1;
    move.center = center;
    move.radius = cc_dist(p0, center);
    move.z_0 = z0;
    move.z_1 = z1;
    return move;
}

static bool check_full_circle(bool lookahead, arc_dir dir)
{
    const float circleRadius = 10.0f;
    const float toolRadius = 1.0f;
    float expectedRadius = (dir == CC_ARC_CCW) ? circleRadius - toolRadius : circleRadius + toolRadius;
    move2d move;
    int i;

    cc_api_init(toolRadius, CC_UNITS_MM, record_emitted_arc, capture_invalid_move_error);
    cc_api_set_lookahead_enabled(lookahead);
    emitted_count = 0;
    recorded_arc_count = 0;
    invalid_move_error_count = 0;

    cc_api_set_comp(CC_COMP_LEFT);
    move = line_move(0.0f, 0.0f, circleRadius, 0.0f);
    if (cc_api_process_move(&move) != cc_status_OK)
        return false;

    cc_api_set_comp(CC_COMP_LEFT);
    move = arc_move(cc_v2(circleRadius, 0.0f), cc_v2(circleRadius, 0.0f), cc_v2(0.0f, 0.0f), dir, 0.0f, 0.0f);
    if (cc_api_process_move(&move) != cc_status_OK)
        return false;

    move = arc_move(cc_v2(circleRadius, 0.0f), cc_v2(circleRadius, 0.0f), cc_v2(0.0f, 0.0f), dir, 0.0f, -2.0f);
    if (cc_api_process_move(&move) != cc_status_OK)
        return false;

    cc_api_set_comp(CC_COMP_OFF);
    move = line_move(circleRadius, 0.0f, 0.0f, 0.0f);
    move.z_0 = -2.0f;
    move.z_1 = -2.0f;
    if (cc_api_process_move(&move) != cc_status_OK || cc_api_process_move(NULL) != cc_status_OK)
        return false;

    if (recorded_arc_count != 4 || invalid_move_error_count != 0)
        return false;

    for (i = 0; i < recorded_arc_count; ++i)
    {
        const move2d *arc = &recorded_arcs[i];
        if (fabsf(fabsf(arc->radius) - expectedRadius) > CC_TOL ||
            fabsf(cc_dist(arc->p_0, arc->center) - expectedRadius) > CC_TOL ||
            fabsf(cc_dist(arc->p_1, arc->center) - expectedRadius) > CC_TOL ||
            cc_is_near(arc->p_0, arc->p_1, CC_TOL))
            return false;
    }

    return fabsf(recorded_arcs[2].z_1 + 1.0f) <= CC_TOL &&
           fabsf(recorded_arcs[3].z_0 + 1.0f) <= CC_TOL &&
           fabsf(recorded_arcs[3].z_1 + 2.0f) <= CC_TOL;
}

static bool check_full_circle_entry_rejected(void)
{
    move2d move = arc_move(cc_v2(10.0f, 0.0f), cc_v2(10.0f, 0.0f), cc_v2(0.0f, 0.0f), CC_ARC_CCW, 0.0f, 0.0f);

    cc_api_init(1.0f, CC_UNITS_MM, record_emitted_arc, capture_invalid_move_error);
    emitted_count = 0;
    recorded_arc_count = 0;
    invalid_move_error_count = 0;

    cc_api_set_comp(CC_COMP_LEFT);
    return cc_api_process_move(&move) == cc_status_InvalidMove &&
           invalid_move_error_count == 1 && emitted_count == 0;
}

static bool check_same_circle_continuation(void)
{
    cc_context ctx;
    move2d first;
    move2d second;
    move2d inserts[CC_INSERT_CAP];
    int insertCount = 0;
    float centerShift = 0.0003f * 25.4f;

    cc_init_internal(&ctx, 1.0f);
    ctx.compSide = CC_COMP_LEFT;
    ctx.arcTol = CC_ARC_TOL_MM;
    ctx.gapTol = CC_GAP_TOL_MM;

    first = arc_move(cc_v2(1.0f, 0.0f), cc_v2(-1.0f, 0.0f), cc_v2(0.0f, 0.0f), CC_ARC_CCW, 0.0f, 0.0f);
    second = arc_move(cc_v2(-1.0f - centerShift, 0.0f), cc_v2(1.0f - centerShift, 0.0f),
                      cc_v2(-centerShift, 0.0f), CC_ARC_CCW, 0.0f, 0.0f);
    cc_update_vectors(&first);
    cc_update_vectors(&second);

    cc_handle_arc_arc(&ctx, &first, &second, inserts, &insertCount);
    return insertCount == 0 && cc_is_near(second.p_0, first.p_1, CC_TOL);
}

static bool check_tiny_roll_is_line(void)
{
    cc_context ctx;
    move2d first = line_move(0.0f, 0.0f, 10.0f, 0.0f);
    move2d second;
    move2d inserts[CC_INSERT_CAP];
    int insertCount = 0;

    cc_init_internal(&ctx, 1.0f);
    ctx.compSide = CC_COMP_LEFT;
    ctx.cornerTreatmentMode = CC_CTM_ROLL;
    ctx.arcTol = CC_ARC_TOL_MM;
    ctx.gapTol = CC_GAP_TOL_MM;

    second = line_move(10.0f, 0.005f, 20.0f, 0.005f);
    return cc_insert_roll_or_corner(&ctx, &first, &second, inserts, &insertCount) &&
           insertCount == 1 && inserts[0].type == CC_MOT_LINE;
}

static bool check_shared_lead_endpoint(void)
{
    float toolRadius = 0.0010225861129f * 25.4f * 0.5f;
    move2d leadIn = line_move(0.0f, 0.0f, 6.35f - toolRadius, 13.97f);
    move2d leadOut = line_move(6.35f + 0.21693046f * toolRadius,
                               13.97f - 0.97618706f * toolRadius, 0.0f, 0.0f);
    cc_context ctx;
    vec2 tip1;
    vec2 tip2;
    int count;

    leadIn.type = CC_MOT_RAPID;
    leadIn.compMode = CC_CM_IN;
    leadOut.type = CC_MOT_RAPID;
    leadOut.compMode = CC_CM_OUT;
    leadOut.lineNum = 13;
    count = cc_common_tip_any(&leadIn, &leadOut, &tip1, &tip2);
    if (count != 1 || !cc_is_near(tip1, leadIn.p_0, 0.0f) ||
        !cc_is_expected_lead_endpoint_touch(&leadIn, &leadOut, count, tip1, tip2))
        return false;

    for (int reverseFirst = 0; reverseFirst < 2; ++reverseFirst)
    {
        for (int reverseSecond = 0; reverseSecond < 2; ++reverseSecond)
        {
            move2d first = reverseFirst ? line_move(leadIn.p_1.x, leadIn.p_1.y, 0.0f, 0.0f) : leadIn;
            move2d second = reverseSecond ? line_move(0.0f, 0.0f, leadOut.p_0.x, leadOut.p_0.y) : leadOut;
            vec2 tip;
            if (cc_intersect_line_line(&first, &second, &tip) != CC_IT_INTERSECT ||
                !cc_same_point(tip, cc_v2(0.0f, 0.0f)) ||
                cc_intersect_line_line(&second, &first, &tip) != CC_IT_INTERSECT ||
                !cc_same_point(tip, cc_v2(0.0f, 0.0f)))
                return false;
        }
    }

    cc_init_internal(&ctx, toolRadius);
    if (!cc_check_staged_lead_in(&ctx, &leadIn))
        return false;
    ctx.lookaheadLeadInSkipNext = false;
    if (!cc_check_staged_lead_in(&ctx, &leadOut) || ctx.stopErr ||
        ctx.lookaheadLeadInValid)
        return false;

    leadOut = line_move(0.0f, 7.0f, 10.0f, 7.0f);
    leadOut.compMode = CC_CM_OUT;
    leadOut.lineNum = 13;
    cc_init_internal(&ctx, toolRadius);
    if (!cc_check_staged_lead_in(&ctx, &leadIn))
        return false;
    ctx.lookaheadLeadInSkipNext = false;
    return !cc_check_staged_lead_in(&ctx, &leadOut) && ctx.stopErr &&
           ctx.status == cc_status_CompInCrossing && ctx.lastLineNum == 13;
}

int main(void)
{
    cc_context ctx;
    move2d previous = line_move(-1.0f, 0.0f, 0.0f, 0.0f);
    move2d collapsed = line_move(0.0f, 0.0f, 1.0f, 0.0f);
    move2d next = line_move(2.0f, 1.0f, 2.0f, 2.0f);
    move2d inserts[CC_INSERT_CAP];
    int insertCount;

    if (!check_shared_lead_endpoint())
    {
        puts("shared lead endpoint drifted or genuine lead crossing was accepted");
        return 1;
    }

    if (!check_geometry_tolerances())
    {
        puts("coordinate or angular tolerance boundary failed");
        return 1;
    }

    for (int lookahead = 0; lookahead < 2; ++lookahead)
    {
        if (!check_full_circle(lookahead != 0, CC_ARC_CCW) ||
            !check_full_circle(lookahead != 0, CC_ARC_CW))
        {
            puts("full circle was dropped, cut at the wrong radius, or lost its Z");
            return 1;
        }
    }

    if (!check_full_circle_entry_rejected())
    {
        puts("full circle as a compensation entry move was not rejected");
        return 1;
    }

    if (!check_same_circle_continuation())
    {
        puts("same-circle arc continuation inserted a corner");
        return 1;
    }

    if (!check_tiny_roll_is_line())
    {
        puts("near-zero roll was not replaced by a line");
        return 1;
    }

    for (int lookahead = 0; lookahead < 2; ++lookahead)
    {
        if (!check_flush_with_trailing_z_moves(lookahead != 0, false) ||
            !check_flush_with_trailing_z_moves(lookahead != 0, true))
        {
            puts("flush with trailing Z moves overflowed or lost the retract");
            return 1;
        }
    }

    if (!check_drain_filter())
    {
        puts("drain filter lost motion/events or emitted zero-length motion");
        return 1;
    }
    puts("drain filter checks passed");

    for (int lookahead = 0; lookahead < 2; ++lookahead)
    {
        if (!check_early_inversion(false, lookahead != 0) ||
            !check_early_inversion(true, lookahead != 0))
        {
            puts("early inversion rejection, source line or lookahead behavior failed");
            return 1;
        }
    }

    cc_init_internal(&ctx, 1.0f);
    ctx.compSide = CC_COMP_LEFT;
    ctx.gapTol = CC_GAP_TOL_MM;

    if (!cc_trim_to(&ctx, &previous, &collapsed, collapsed.p_1) ||
        collapsed.valid || !collapsed.junctionOnly || collapsed.hasXY)
    {
        puts("collapsed line was not kept for junction recovery");
        return 1;
    }

    cc_apply_logic(&ctx, &collapsed, &next, inserts, &insertCount);
    if (!collapsed.valid || collapsed.junctionOnly || !collapsed.hasXY ||
        !cc_is_near(collapsed.p_1, cc_v2(2.0f, 0.0f), CC_TOL) ||
        !cc_is_near(next.p_0, collapsed.p_1, CC_TOL) || insertCount != 0)
    {
        puts("collapsed line was not extended to the next junction");
        return 1;
    }

    cc_init_internal(&ctx, 0.0f);
    ctx.compSide = CC_COMP_LEFT;
    ctx.compMode = CC_CM_STEADY;
    ctx.gapTol = CC_GAP_TOL_MM;
    ctx.prevOff = line_move(1.0f, 0.0f, 1.0f, 0.0f);
    ctx.prevOff.startDir = cc_v2(1.0f, 0.0f);
    ctx.prevOff.endDir = ctx.prevOff.startDir;
    ctx.prevOff.valid = false;
    ctx.prevOff.junctionOnly = true;
    ctx.havePrevMove = true;
    next = line_move(2.0f, 1.0f, 2.0f, 2.0f);
    if (!cc_push_in(&ctx, &next) || !cc_process(&ctx) ||
        !cc_pop_out(&ctx, &collapsed) || !collapsed.valid || collapsed.junctionOnly ||
        !cc_is_near(collapsed.p_1, cc_v2(2.0f, 0.0f), CC_TOL))
    {
        puts("process did not emit the recovered line");
        return 1;
    }

    cc_init_internal(&ctx, 1.0f);
    ctx.prevOff = line_move(1.0f, 0.0f, 1.0f, 0.0f);
    ctx.prevOff.valid = false;
    ctx.prevOff.junctionOnly = true;
    ctx.havePrevMove = true;
    cc_flush(&ctx);
    if (ctx.outCount != 0)
    {
        puts("unrecovered collapsed line was emitted");
        return 1;
    }

    cc_init_internal(&ctx, 1.5875f);
    ctx.compSide = CC_COMP_LEFT;
    ctx.gapTol = CC_GAP_TOL_MM;
    move2d arc = {0};
    arc.type = CC_MOT_ARC;
    arc.arcDir = CC_ARC_CCW;
    arc.valid = true;
    arc.center = cc_v2(92.3696f, 53.5990f);
    arc.p_0 = cc_v2(87.579f, 52.140f);
    arc.p_1 = cc_v2(92.3696f, 48.5938568f);
    arc.radius = 5.00514f;
    cc_update_vectors(&arc);
    move2d first = line_move(92.3696f, 48.5953f, 100.4291f, 48.5953f);
    move2d second = line_move(98.9405f, 47.5594f, 104.8942f, 63.6274f);
    vec2 intersections[2];
    cc_handle_arc_line(&ctx, &arc, &first, inserts, &insertCount);
    if (fabsf(first.p_0.y - 48.5953f) > CC_TOL ||
        cc_finite_intersection_points(&first, &second, intersections) != 1 ||
        fabsf(intersections[0].x - 99.3243f) > 0.001f ||
        fabsf(intersections[0].y - 48.5953f) > CC_TOL)
    {
        puts("arc-line gap changed the following line intersection");
        return 1;
    }

    return 0;
}