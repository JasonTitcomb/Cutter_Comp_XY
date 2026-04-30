#ifndef PLAN_LINE_DATA_PORTABLE_H
#define PLAN_LINE_DATA_PORTABLE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Minimal, self-contained copy of grblHAL planner line data dependencies.
 * FOR DEVELOPMENT ONLY. This is not intended to be a complete or long-term solution for hosting the cutter compensation engine.
 * 
 * Source of truth in grblHAL_Due:
 * - src/grbl/planner.h
 * - src/grbl/gcode.h
 * - src/grbl/coolant_control.h
 * - src/grbl/spindle_control.h
 */
#define EXEC_FEED_HOLD (1 << 0) 
#define EXEC_STOP (1 << 1)
void protocol_buffer_synchronize(void){};
void system_set_exec_state_flag(uint8_t flag){(void)flag;};
void protocol_execute_realtime(void){};


#ifndef ENABLE_JERK_ACCELERATION
#define ENABLE_JERK_ACCELERATION 0
#endif

#ifndef KINEMATICS_API
#define KINEMATICS_API 0
#endif

#ifndef ENABLE_ACCELERATION_PROFILES
#define ENABLE_ACCELERATION_PROFILES 0
#endif

#ifndef ENABLE_PATH_BLENDING
#define ENABLE_PATH_BLENDING 0
#endif

#ifndef PLANNER_ADD_MOTION_MODE
#define PLANNER_ADD_MOTION_MODE 0
#endif

#ifndef N_AXIS
#define N_AXIS 3
#endif

#ifndef MAX_OFFSET_ENTRIES
#define MAX_OFFSET_ENTRIES 4
#endif

#ifndef NGC_PARAMETERS_ENABLE
#define NGC_PARAMETERS_ENABLE 0
#endif

/* gcode.h */
typedef int8_t offset_id_t;
typedef int32_t tool_id_t;
typedef int16_t pocket_id_t;

typedef uint8_t override_t;

typedef float coord_data_t[N_AXIS];
typedef float coord_system_data_t[N_AXIS];

typedef struct {
    float x;
    float y;
} point_2d_t;

typedef int16_t non_modal_t;
typedef int16_t tool_action_t;
typedef int16_t override_mode_t;
typedef int16_t user_mcode_t;
typedef int16_t modal_state_action_t;
typedef int16_t macro_call_t;

/* coolant_control.h */
typedef union {
    uint8_t bits;
    uint8_t mask;
    uint8_t value;
    struct {
        uint8_t flood          :1;
        uint8_t mist           :1;
        uint8_t shower         :1;
        uint8_t trough_spindle :1;
        uint8_t unused         :4;
    };
} coolant_state_t;

/* planner.h */
typedef union {
    uint32_t value;
    struct {
        uint16_t rapid_motion         :1;
        uint16_t system_motion        :1;
        uint16_t jog_motion           :1;
        uint16_t backlash_motion      :1;
        uint16_t no_feed_override     :1;
        uint16_t inverse_time         :1;
        uint16_t units_per_rev        :1;
        uint16_t is_rpm_rate_adjusted :1;
        uint16_t is_laser_ppi_mode    :1;
        uint16_t target_valid         :1;
        uint16_t target_validated     :1;
        uint16_t probing_toolsetter   :1;
#if ENABLE_JERK_ACCELERATION
        uint16_t jerk                 :1;
        uint16_t unassigned           :3;
#else
        uint16_t unassigned           :4;
#endif
        coolant_state_t coolant;
    };
} planner_cond_t;

/* gcode.h */
typedef struct output_command {
    bool is_digital;
    uint8_t port;
    int32_t value;
    struct output_command *next;
} output_command_t;

/* gcode.h */
typedef union {
    uint16_t value;
    struct {
        uint16_t spindle_rpm_disable  :8;
        uint16_t feed_rates_disable   :1;
        uint16_t feed_hold_disable    :1;
        uint16_t parking_disable      :1;
        uint16_t spindle_wait_disable :1;
        uint16_t reserved             :2;
        uint16_t sync                 :1;
    };
    struct {
        uint16_t spindle_rpm          :8;
        uint16_t feed_rates           :1;
        uint16_t feed_hold            :1;
        uint16_t parking              :1;
        uint16_t spindle              :1;
        uint16_t reserved1            :2;
        uint16_t reserved2            :1;
    };
} gc_override_flags_t;

/* gcode.h */
typedef enum {
    MotionMode_Seek = 0,
    MotionMode_Linear = 1,
    MotionMode_CwArc = 2,
    MotionMode_CcwArc = 3,
    MotionMode_CubicSpline = 5,
    MotionMode_QuadraticSpline = 51,
    MotionMode_SpindleSynchronized = 33,
    MotionMode_RigidTapping = 331,
    MotionMode_DrillChipBreak = 73,
    MotionMode_Threading = 76,
    MotionMode_CannedCycle81 = 81,
    MotionMode_CannedCycle82 = 82,
    MotionMode_CannedCycle83 = 83,
    MotionMode_CannedCycle84 = 84,
    MotionMode_CannedCycle85 = 85,
    MotionMode_CannedCycle86 = 86,
    MotionMode_CannedCycle89 = 89,
    MotionMode_ProbeToward = 140,
    MotionMode_ProbeTowardNoError = 141,
    MotionMode_ProbeAway = 142,
    MotionMode_ProbeAwayNoError = 143,
    MotionMode_None = 80
} motion_mode_t;

/* spindle_control.h */
typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t on               :1;
        uint8_t ccw              :1;
        uint8_t pwm              :1;
        uint8_t reserved         :1;
        uint8_t override_disable :1;
        uint8_t encoder_error    :1;
        uint8_t at_speed         :1;
        uint8_t synchronized     :1;
    };
} spindle_state_t;

/* spindle_control.h */
typedef enum {
    SpindleSpeedMode_RPM = 0,
    SpindleSpeedMode_CSS = 1
} spindle_rpm_mode_t;

/* spindle_control.h */
typedef struct {
    float surface_speed;
    float target_rpm;
    float delta_rpm;
    float max_rpm;
    float tool_offset;
    uint_fast8_t axis;
} spindle_css_data_t;

typedef struct spindle_ptrs spindle_ptrs_t;

/* gcode.h */
typedef union {
    uint8_t value;
    struct {
        uint8_t is_rpm_rate_adjusted :1;
        uint8_t is_laser_ppi_mode    :1;
        uint8_t unassigned           :6;
    };
} spindle_cond_t;

/* gcode.h */
typedef struct {
    spindle_state_t state;
    spindle_rpm_mode_t rpm_mode;
    spindle_cond_t condition;
    spindle_css_data_t *css;
    float rpm;
    spindle_ptrs_t *hal;
} spindle_t;

typedef struct {
    spindle_state_t state;
    spindle_rpm_mode_t rpm_mode;
} spindle_modal_t;

typedef union {
    uint32_t mask;
    uint32_t value;
} parameter_words_t;

typedef struct {
    float f;
    float p;
    float q;
    float r;
    float s;
    float xyz[N_AXIS];
    float ijk[3];
} gc_values_t;

typedef struct {
    coord_data_t offset;
    float radius;
    tool_id_t tool_id;
} tool_data_t;

typedef struct {
    motion_mode_t motion;
    coolant_state_t coolant;
    spindle_t spindle;
    gc_override_flags_t override_ctrl;
    bool canned_cycle_active;
    float tool_length_offset[N_AXIS];
} gc_modal_t;

typedef enum {
    CComp_Off = 0, //!< 0 - G40 - Default, must be zero
    CComp_Left,    //!< 1 - G41, G41.1
    CComp_Right    //!< 2 - G42, G42.1
} ccomp_mode_t;

typedef struct {
    ccomp_mode_t side;
    bool first_move;
    float radius;
} gc_ccomp_t;

typedef struct {
    float xyz[3];
    float delta;
    float dwell;
    float retract_position;
    bool rapid_retract;
    bool spindle_off;
    bool change;
} gc_canned_t;

typedef struct {
    gc_modal_t modal;
    gc_canned_t canned;
    spindle_t *spindle;
    float feed_rate;
    float distance_per_rev;
    float position[N_AXIS];
#if ENABLE_PATH_BLENDING
    float path_tolerance;
    float cam_tolerance;
#endif
    uint32_t line_number;
    tool_id_t tool_pending;
    tool_id_t g43_pending;
    bool file_run;
    bool file_stream;
    bool is_laser_ppi_mode;
    bool is_rpm_rate_adjusted;
    bool tool_change;
    bool skip_blocks;
    //status_code_t last_error;
    offset_id_t offset_id;
    coord_data_t offset_queue[MAX_OFFSET_ENTRIES];
    bool g92_offset_applied;
    coord_system_data_t g92_offset;
    tool_data_t *tool;
} parser_state_t;

typedef struct {
    non_modal_t non_modal_command;
    tool_action_t tool_action;
    override_mode_t override_command;
    user_mcode_t user_mcode;
    bool user_mcode_sync;
    gc_modal_t modal;
    spindle_modal_t spindle_modal;
    gc_values_t values;
    parameter_words_t words;
    output_command_t output_command;
    uint32_t arc_turns;
    parameter_words_t g65_words;
#if NGC_PARAMETERS_ENABLE
    macro_call_t macro_call;
    modal_state_action_t state_action;
#endif
} parser_block_t;

typedef parser_block_t gc_block_t;
typedef parser_state_t gc_state_t;

extern gc_state_t gc_state;

/* planner.h */
typedef struct {
    float feed_rate;
#if KINEMATICS_API
    float rate_multiplier;
#endif
#if ENABLE_ACCELERATION_PROFILES
    float acceleration_factor;
#endif
#if ENABLE_PATH_BLENDING
    float path_tolerance;
    float cam_tolerance;
#endif
#if PLANNER_ADD_MOTION_MODE
    motion_mode_t motion_mode;
#endif
    spindle_t spindle;
    planner_cond_t condition;
    gc_override_flags_t overrides;
    offset_id_t offset_id;
    uint32_t line_number;
    char *message;
    output_command_t *output_commands;
} plan_line_data_t;

//! Axis index to plane assignment.
typedef union {
    uint8_t axis[3];
    struct {
        uint8_t axis_0;
        uint8_t axis_1;
        uint8_t axis_linear;
    };
} plane_t;

typedef enum {
    Message_Plain = 0,
    Message_Info,
    Message_Warning,
    Message_Error,
    Message_Debug
} message_type_t;


typedef struct {
    bool single_block;
} sys_flags_t;

typedef struct {
    sys_flags_t flags;
} system_t;

static system_t sys = {0};

// Converts an uint32 variable to string.
char buf[40];
char *uitoa (uint32_t n)
{   

    char *bptr = buf + sizeof(buf);

    *--bptr = '\0';

    if (n == 0)
        *--bptr = '0';
    else while (n) {
        *--bptr = '0' + (n % 10);
        n /= 10;
    }

    return bptr;
}

void mc_dwell(float seconds){(void)seconds;};


#endif /* PLAN_LINE_DATA_PORTABLE_H */
