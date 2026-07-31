#include "trace_task.h"
#include "app_motor.h"
#include <math.h>
#include <string.h>

// 电机控制函数适配层
#define Contrl_Speed  Motion_Set_Speed  // 速度闭环控制(左轮, 右轮)

extern uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];

static uint8_t sensor_history_has_majority(uint8_t history) {
    uint8_t active_samples = 0;

    for (uint8_t i = 0; i < TRACE_FILTER_SAMPLE_COUNT; i++) {
        active_samples += (history >> i) & 0x01u;
    }

    return (active_samples >= TRACE_FILTER_MAJORITY_COUNT) ? 1u : 0u;
}

static uint8_t update_sensor_filter(line_following_t* controller,
                                    uint16_t* sensor_values,
                                    uint16_t line_raw_value) {
    uint8_t filtered_mask = 0;
    uint8_t active_sensor_count = 0;
    uint8_t history_mask =
        (uint8_t)((1u << TRACE_FILTER_SAMPLE_COUNT) - 1u);

    for (uint8_t i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        uint8_t is_line = (sensor_values[i] == line_raw_value) ? 1u : 0u;
        uint8_t history = (uint8_t)(((controller->sensor_history[i] << 1) |
                                     is_line) & history_mask);

        controller->sensor_history[i] = history;

        if (sensor_history_has_majority(history)) {
            filtered_mask |= (uint8_t)(1u << i);
            active_sensor_count++;
        }
    }

    controller->filtered_mask = filtered_mask;
    controller->active_sensor_count = active_sensor_count;

    return filtered_mask;
}

static void reset_line_pid(line_following_t* controller) {
    controller->last_error = 0.0f;
    controller->integral = 0.0f;
}

static void reset_curve_control(line_following_t* controller) {
    controller->curve_last_error = 0.0f;
    controller->curve_integral = 0.0f;
    controller->curve_pid_output = 0.0f;
    controller->curve_entry_count = 0u;
    controller->curve_exit_count = 0u;
}

static inline float clamp_float(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

static float apply_minimum_correction(float output, float minimum) {
    if ((output > 0.0f) && (output < minimum)) return minimum;
    if ((output < 0.0f) && (output > -minimum)) return -minimum;
    return output;
}

static inline int16_t clamp_int16(int16_t value, int16_t min_val, int16_t max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// 8路加权平均计算偏差值
float calculate_error(line_following_t* controller, uint8_t filtered_mask) {
    float weighted_sum = 0.0f;
    int active_sensors = 0;

    for (int i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if ((filtered_mask & (uint8_t)(1u << i)) != 0u) {
            weighted_sum += controller->sensor_weights[i];
            active_sensors++;
        }
    }

    // 没有检测到线(丢线): 沿用上一次偏差, 保持转向直到重新压线
    if (active_sensors == 0) return 0.0f;

    return weighted_sum / active_sensors;
}

// PID控制计算(带死区/过零清积分/动态积分限幅)
float pid_control(line_following_t* controller, float error) {

    // 死区: 忽略微小偏差
    if (fabsf(error) < 0.6f) {
        error = 0.0f;
    }

    // 偏差过零时清空积分, 避免残留积分导致震荡
    if ((controller->last_error > 0 && error < 0) ||
        (controller->last_error < 0 && error > 0)) {
        controller->integral = 0.0f;
    }

    // 按偏差大小动态调整积分限幅
    float integral_limit;
    if (fabsf(error) > 3.0f) {
        integral_limit = 80.0f;
    } else if (fabsf(error) > 1.5f) {
        integral_limit = 50.0f;
    } else {
        integral_limit = 20.0f;
    }

    controller->integral += error;
    controller->integral = clamp_float(controller->integral, -integral_limit, integral_limit);

    float derivative = error - controller->last_error;

    float output = (controller->kp * error +
                   controller->ki * controller->integral +
                   controller->kd * derivative);

    // 非零修正必须大于编码器测速分辨率，否则轮速内环难以及时执行。
    output = apply_minimum_correction(
        output, TRACE_STRAIGHT_MIN_CORRECTION_MM_S);
    output = clamp_float(
        output, -TRACE_STRAIGHT_CORRECTION_LIMIT_MM_S,
        TRACE_STRAIGHT_CORRECTION_LIMIT_MM_S);

    controller->last_error = error;

    return output;
}

static float curve_pid_control(line_following_t* controller, float error) {
    float derivative = error - controller->curve_last_error;

    controller->curve_integral += error;
    controller->curve_integral = clamp_float(controller->curve_integral,
                                             -20.0f, 20.0f);

    controller->curve_pid_output =
        controller->curve_kp * error +
        controller->curve_ki * controller->curve_integral +
        controller->curve_kd * derivative;

    controller->curve_pid_output =
        apply_minimum_correction(controller->curve_pid_output,
                                 TRACE_CURVE_MIN_CORRECTION_MM_S);

    controller->curve_pid_output =
        clamp_float(controller->curve_pid_output,
                    -TRACE_CURVE_CORRECTION_LIMIT_MM_S,
                    TRACE_CURVE_CORRECTION_LIMIT_MM_S);

    controller->curve_last_error = error;

    return controller->curve_pid_output;
}

static void curve_right_speed_control(line_following_t* controller,
                                      float correction,
                                      int16_t* left_speed,
                                      int16_t* right_speed) {
    int16_t left = controller->curve_outer_speed + (int16_t)correction;
    int16_t right = controller->curve_inner_speed - (int16_t)correction;

    *left_speed = clamp_int16(left, 0, controller->max_speed);
    *right_speed = clamp_int16(right, 0, controller->max_speed);
}

static void line_recovery_speed_control(line_following_t* controller) {
    int16_t outer_speed =
        clamp_int16(TRACE_RECOVERY_OUTER_SPEED_MM_S, 0,
                    controller->max_speed);
    int16_t inner_speed =
        clamp_int16(TRACE_RECOVERY_INNER_SPEED_MM_S, 0,
                    controller->max_speed);

    if (controller->recovery_direction > 0) {
        Contrl_Speed(outer_speed, inner_speed);
    } else if (controller->recovery_direction < 0) {
        Contrl_Speed(inner_speed, outer_speed);
    } else {
        Contrl_Speed(0, 0);
    }
}

// 差速控制: 由PID输出量计算左右轮目标速度
void differential_speed_control(line_following_t* controller, float pid_output,
                                int16_t* left_speed, int16_t* right_speed) {
    int16_t left = controller->base_speed + (int16_t)pid_output;
    int16_t right = controller->base_speed - (int16_t)pid_output;

    // 低速调试阶段禁止倒转，任一车轮速度均限制在0~max_speed
    *left_speed = clamp_int16(left, 0, controller->max_speed);
    *right_speed = clamp_int16(right, 0, controller->max_speed);
}

// 巡线主函数
void follow_line(line_following_t* controller, uint16_t* sensor_values,
                 uint16_t line_raw_value) {
    uint8_t filtered_mask = update_sensor_filter(controller, sensor_values,
                                                 line_raw_value);
    uint32_t now_ms = g_system_tick_ms;

    // 主循环约1ms调用一次：灰度每次滤波，速度目标按10ms固定周期更新。
    controller->software_time_ms++;

    if (controller->finish_latched != 0u) {
        controller->state = LINE_STATE_FINISHED;
        Contrl_Speed(0, 0);
        return;
    }

    if ((controller->software_time_ms - controller->last_control_ms) <
        TRACE_LINE_CONTROL_PERIOD_MS) {
        return;
    }
    controller->last_control_ms = controller->software_time_ms;

    if (controller->run_timer_started != 0u) {
        controller->run_elapsed_ms = now_ms - controller->run_start_ms;
    }

    if ((controller->run_timer_started != 0u) &&
        (controller->run_elapsed_ms >= TRACE_FINISH_ENABLE_TIME_MS) &&
        (controller->active_sensor_count >=
         TRACE_FINISH_MIN_ACTIVE_SENSORS)) {
        if (controller->finish_candidate_active == 0u) {
            controller->finish_candidate_active = 1u;
            controller->finish_candidate_start_ms = now_ms;
        }

        controller->state = LINE_STATE_ALL_BLACK;
        if ((now_ms - controller->finish_candidate_start_ms) >=
            TRACE_FINISH_CONFIRM_TIME_MS) {
            controller->finish_latched = 1u;
            controller->state = LINE_STATE_FINISHED;
            reset_line_pid(controller);
            reset_curve_control(controller);
            Contrl_Speed(0, 0);
        }
        return;
    }

    controller->finish_candidate_active = 0u;
    controller->finish_candidate_start_ms = 0u;

    if (controller->active_sensor_count == 0u) {
        controller->valid_tracking_streak = 0u;
        reset_line_pid(controller);
        reset_curve_control(controller);

        if (controller->has_valid_line == 0u) {
            controller->state = LINE_STATE_LOST;
            controller->recovery_from_curve = 0u;
            Contrl_Speed(0, 0);
            return;
        }

        if (controller->state == LINE_STATE_CURVE_RIGHT) {
            controller->recovery_from_curve = 1u;
            controller->recovery_direction = 1;
        }

        controller->state = LINE_STATE_RECOVERY;
        line_recovery_speed_control(controller);
        return;
    }

    if (controller->active_sensor_count == GRAYSCALE_SENSOR_CHANNELS) {
        controller->valid_tracking_streak = 0u;

        // 30秒前的全黑不作为终点。若启动时就在黑区，先直行驶离黑区；
        // 运行中遇到全黑则保持上一次左右轮目标不变。
        if ((controller->run_timer_started == 0u) &&
            (controller->has_valid_line == 0u)) {
            Contrl_Speed(controller->base_speed, controller->base_speed);
        }
        return;
    }

    float error = calculate_error(controller, filtered_mask);
    uint8_t was_recovering =
        (controller->state == LINE_STATE_RECOVERY) ? 1u : 0u;

    if ((controller->state == LINE_STATE_CURVE_RIGHT) ||
        (controller->recovery_from_curve != 0u)) {
        controller->recovery_direction = 1;
    } else if (error > TRACE_RECOVERY_DIRECTION_DEADBAND) {
        controller->recovery_direction = 1;
    } else if (error < -TRACE_RECOVERY_DIRECTION_DEADBAND) {
        controller->recovery_direction = -1;
    }

    controller->last_valid_error = error;
    controller->last_valid_mask = filtered_mask;
    controller->has_valid_line = 1u;

    if (controller->run_timer_started == 0u) {
        controller->run_timer_started = 1u;
        controller->run_start_ms = now_ms;
        controller->run_elapsed_ms = 0u;
    }

    if (controller->valid_tracking_streak < TRACE_TRACKING_REACQUIRE_COUNT) {
        controller->valid_tracking_streak++;
    }

    if (controller->valid_tracking_streak < TRACE_TRACKING_REACQUIRE_COUNT) {
        reset_line_pid(controller);

        if (was_recovering != 0u) {
            line_recovery_speed_control(controller);
        } else {
            Contrl_Speed(0, 0);
        }
        return;
    }

    if (was_recovering != 0u) {
        if (controller->recovery_from_curve != 0u) {
            controller->state = LINE_STATE_CURVE_RIGHT;
            controller->curve_integral = 0.0f;
            controller->curve_last_error = error;
            controller->curve_pid_output = 0.0f;
            controller->curve_entry_count = 0u;
            controller->curve_exit_count = 0u;
        } else {
            controller->state = LINE_STATE_TRACKING;
            controller->integral = 0.0f;
            controller->last_error = error;
        }

        controller->recovery_from_curve = 0u;
    }

    if (controller->state == LINE_STATE_CURVE_RIGHT) {
        controller->curve_entry_count = 0u;

        if (((filtered_mask & TRACE_CURVE_LEFT_SENSOR_MASK) != 0u) &&
            (error <= TRACE_CURVE_EXIT_ERROR)) {
            if (controller->curve_exit_count <
                TRACE_CURVE_EXIT_CONFIRM_COUNT) {
                controller->curve_exit_count++;
            }
        } else {
            controller->curve_exit_count = 0u;
        }

        if (controller->curve_exit_count <
            TRACE_CURVE_EXIT_CONFIRM_COUNT) {
            float curve_correction = curve_pid_control(controller, error);
            int16_t curve_left_speed;
            int16_t curve_right_speed;

            curve_right_speed_control(controller, curve_correction,
                                      &curve_left_speed, &curve_right_speed);
            Contrl_Speed(curve_left_speed, curve_right_speed);
            return;
        }

        controller->state = LINE_STATE_TRACKING;
        controller->curve_exit_count = 0u;
        controller->curve_pid_output = 0.0f;
        controller->curve_integral = 0.0f;
        controller->integral = 0.0f;
        controller->last_error = error;

        if (error < -TRACE_RECOVERY_DIRECTION_DEADBAND) {
            controller->recovery_direction = -1;
        }
    }

    if (controller->state != LINE_STATE_TRACKING) {
        controller->integral = 0.0f;
        controller->last_error = error;
        controller->curve_entry_count = 0u;
        controller->curve_exit_count = 0u;
        controller->curve_pid_output = 0.0f;
    }

    controller->state = LINE_STATE_TRACKING;

    if (((filtered_mask & TRACE_CURVE_RIGHT_SENSOR_MASK) != 0u) &&
        (error >= TRACE_CURVE_ENTRY_ERROR)) {
        if (controller->curve_entry_count <
            TRACE_CURVE_ENTRY_CONFIRM_COUNT) {
            controller->curve_entry_count++;
        }
    } else {
        controller->curve_entry_count = 0u;
    }

    if (controller->curve_entry_count >=
        TRACE_CURVE_ENTRY_CONFIRM_COUNT) {
        float curve_correction;
        int16_t curve_left_speed;
        int16_t curve_right_speed;

        controller->state = LINE_STATE_CURVE_RIGHT;
        controller->curve_entry_count = 0u;
        controller->curve_exit_count = 0u;
        controller->curve_integral = 0.0f;
        controller->curve_last_error = error;
        controller->recovery_direction = 1;

        curve_correction = curve_pid_control(controller, error);
        curve_right_speed_control(controller, curve_correction,
                                  &curve_left_speed, &curve_right_speed);
        Contrl_Speed(curve_left_speed, curve_right_speed);
        return;
    }

    float pid_output = pid_control(controller, error);

    int16_t left_speed, right_speed;
    differential_speed_control(controller, pid_output, &left_speed, &right_speed);

    Contrl_Speed(left_speed, right_speed);
}
