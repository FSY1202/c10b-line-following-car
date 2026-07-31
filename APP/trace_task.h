#ifndef __TRACE_TASK_H__
#define __TRACE_TASK_H__

#include <stdint.h>
#include "trace_config.h"

#define LINE_SENSOR_COUNT  (8u)

typedef enum {
    LINE_STATE_LOST = 0,
    LINE_STATE_TRACKING,
    LINE_STATE_CURVE_RIGHT,
    LINE_STATE_RECOVERY,
    LINE_STATE_ALL_BLACK,
    LINE_STATE_FINISHED
} line_state_t;
#include "grayscale_sensor.h"  // 获取GRAYSCALE_SENSOR_CHANNELS定义

typedef struct {
    // PID参数
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数

    float last_error;       // 上次偏差
    float integral;         // 积分累加

    // 控制参数
    int16_t base_speed;     // 基础速度(mm/s)
    int16_t max_speed;      // 最大速度(mm/s)

    // 8个传感器的位置权重
    float sensor_weights[8];

    // 3-sample majority filter and debug values for Keil Watch
    uint8_t sensor_history[LINE_SENSOR_COUNT];
    uint8_t filtered_mask;
    uint8_t active_sensor_count;
    uint8_t valid_tracking_streak;
    line_state_t state;

    // Clockwise 750 mm radius curve control and Keil Watch values
    float curve_kp;
    float curve_ki;
    float curve_kd;
    float curve_last_error;
    float curve_integral;
    float curve_pid_output;
    int16_t curve_outer_speed;
    int16_t curve_inner_speed;
    uint8_t curve_entry_count;
    uint8_t curve_exit_count;

    // Last valid line information and all-white recovery debug values
    float last_valid_error;
    uint8_t last_valid_mask;
    int8_t recovery_direction;       // -1=left, 0=unknown, 1=right
    uint8_t has_valid_line;
    uint8_t recovery_from_curve;

    // 10 ms outer-loop scheduling and finish-line timing
    uint32_t software_time_ms;
    uint32_t last_control_ms;
    uint32_t run_start_ms;
    uint32_t run_elapsed_ms;
    uint32_t finish_candidate_start_ms;
    uint8_t run_timer_started;
    uint8_t finish_candidate_active;
    uint8_t finish_latched;

} line_following_t;

extern line_following_t g_line_controller;

void line_following_init(line_following_t* controller);
float calculate_error(line_following_t* controller, uint8_t filtered_mask);
float pid_control(line_following_t* controller, float error);
void differential_speed_control(line_following_t* controller, float pid_output,
                                int16_t* left_speed, int16_t* right_speed);
void follow_line(line_following_t* controller, uint16_t* sensor_values,
                 uint16_t line_raw_value);

#endif
