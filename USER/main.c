#include "AllHeader.h"

// 所有需要手动调节的循迹参数统一放在 APP/trace_config.h。
// 修改后请重新 Rebuild，并烧录 OBJ/C10B_Trace.hex。

//#######################################################
// 重要配置参数
//#######################################################
// 已实测确认: 探头压在黑线上时指示灯亮, 即OUT=1对应"检测到黑线"。
//#######################################################

uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
line_following_t g_line_controller;

// 巡线参数初始化(仅第一阶段用, 匀速循迹调试; 后续加变速策略/圈数状态机时再扩展)
void line_following_init(line_following_t* controller) {
    controller->kp = TRACE_STRAIGHT_KP;
    controller->ki = TRACE_STRAIGHT_KI;
    controller->kd = TRACE_STRAIGHT_KD;

    controller->last_error = 0.0f;
    controller->integral = 0.0f;

    controller->base_speed = TRACE_STRAIGHT_BASE_SPEED_MM_S;
    controller->max_speed = TRACE_MAX_WHEEL_SPEED_MM_S;

    controller->curve_kp = TRACE_CURVE_KP;
    controller->curve_ki = TRACE_CURVE_KI;
    controller->curve_kd = TRACE_CURVE_KD;
    controller->curve_last_error = 0.0f;
    controller->curve_integral = 0.0f;
    controller->curve_pid_output = 0.0f;
    controller->curve_outer_speed = TRACE_CURVE_OUTER_SPEED_MM_S;
    controller->curve_inner_speed = TRACE_CURVE_INNER_SPEED_MM_S;
    controller->curve_entry_count = 0u;
    controller->curve_exit_count = 0u;
    controller->last_valid_error = 0.0f;
    controller->last_valid_mask = 0u;
    controller->recovery_direction = 0;
    controller->has_valid_line = 0u;
    controller->recovery_from_curve = 0u;
    controller->software_time_ms = 0u;
    controller->last_control_ms = 0u;
    controller->run_start_ms = 0u;
    controller->run_elapsed_ms = 0u;
    controller->finish_candidate_start_ms = 0u;
    controller->run_timer_started = 0u;
    controller->finish_candidate_active = 0u;
    controller->finish_latched = 0u;

    controller->sensor_weights[0] = TRACE_SENSOR_WEIGHT_0;
    controller->sensor_weights[1] = TRACE_SENSOR_WEIGHT_1;
    controller->sensor_weights[2] = TRACE_SENSOR_WEIGHT_2;
    controller->sensor_weights[3] = TRACE_SENSOR_WEIGHT_3;
    controller->sensor_weights[4] = TRACE_SENSOR_WEIGHT_4;
    controller->sensor_weights[5] = TRACE_SENSOR_WEIGHT_5;
    controller->sensor_weights[6] = TRACE_SENSOR_WEIGHT_6;
    controller->sensor_weights[7] = TRACE_SENSOR_WEIGHT_7;

    for (uint8_t i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        controller->sensor_history[i] = 0u;
    }

    controller->filtered_mask = 0u;
    controller->active_sensor_count = 0u;
    controller->valid_tracking_streak = 0u;
    controller->state = LINE_STATE_LOST;

}

int main(void)
{
    bsp_init();

    Grayscale_Sensor_Init();
    line_following_init(&g_line_controller);

    TIM6_Init();  // 10ms定时器(驱动编码器读取+电机PID速度闭环)

    Odometry_Init();
    RDK_Link_Init();
    GCS_Cmd_Init();  // 测速工程：上电自动启动并平滑升至配置目标速度

    while (1)
    {
        GCS_Cmd_Poll();
        GCS_Cmd_Update();

        Grayscale_Sensor_Read_All(g_sensor_data);

        if (GCS_Cmd_Is_Started())
        {
            follow_line(&g_line_controller, g_sensor_data, TRACE_LINE_RAW_VALUE);
        }
        else
        {
            Motion_Set_Speed(0, 0);
        }

        Odometry_Update();
        RDK_Link_Report_Tick();

        delay_ms(1);
    }
}
