#ifndef __TRACE_CONFIG_H__
#define __TRACE_CONFIG_H__

//==============================================================================
// 8路循迹参数集中配置
// 修改参数后必须重新 Rebuild 并烧录新的 HEX。
// g_sensor_data[0] 对应最左侧探头，[7] 对应最右侧探头。
//==============================================================================

//------------------------------------------------------------------------------
// 1. 黑白电平
//------------------------------------------------------------------------------
// 当前模块实测：白色=0，黑线=1。
#define TRACE_LINE_RAW_VALUE                 (1u)

//------------------------------------------------------------------------------
// 2. 直道参数
//------------------------------------------------------------------------------
// 直道 PID：缓慢摆动时减小 KP 或增大 KD；高频抖动时减小 KD。
#define TRACE_STRAIGHT_KP                    (8.0f)
#define TRACE_STRAIGHT_KI                    (0.0f)
#define TRACE_STRAIGHT_KD                    (2.0f)

// 直道基础速度和任意单轮允许的最高目标速度，单位 mm/s。
#define TRACE_STRAIGHT_BASE_SPEED_MM_S       (170)
#define TRACE_MAX_WHEEL_SPEED_MM_S           (170)

// 编码器10ms测速每个脉冲约16.9 mm/s，小于该值的差速难以稳定执行。
// 因此非零纠偏至少给20 mm/s，同时限制最大纠偏避免突然甩动。
#define TRACE_STRAIGHT_MIN_CORRECTION_MM_S    (22.0f)
#define TRACE_STRAIGHT_CORRECTION_LIMIT_MM_S  (60.0f)

//------------------------------------------------------------------------------
// 3. 8路探头位置权重
//------------------------------------------------------------------------------
// 数值从左到右递增。绝对值越大，最外侧探头触发时转向修正越强。
#define TRACE_SENSOR_WEIGHT_0                (-5.0f)  // 最左
#define TRACE_SENSOR_WEIGHT_1                (-4.0f)
#define TRACE_SENSOR_WEIGHT_2                (-2.0f)
#define TRACE_SENSOR_WEIGHT_3                (-1.0f)
#define TRACE_SENSOR_WEIGHT_4                ( 1.0f)
#define TRACE_SENSOR_WEIGHT_5                ( 2.0f)
#define TRACE_SENSOR_WEIGHT_6                ( 4.0f)
#define TRACE_SENSOR_WEIGHT_7                ( 5.0f)  // 最右

//------------------------------------------------------------------------------
// 4. 灰度滤波与恢复行驶确认
//------------------------------------------------------------------------------
// 每路保存最近3次采样，至少2次为黑线才判定该路有效。
// 采样次数范围为1~8，多数阈值不能大于采样次数。
#define TRACE_FILTER_SAMPLE_COUNT            (3u)
#define TRACE_FILTER_MAJORITY_COUNT          (2u)

// 循迹外环每10ms更新一次，与编码器速度内环周期一致；灰度仍保持约1ms采样。
#define TRACE_LINE_CONTROL_PERIOD_MS         (10u)

// 从全白/全黑停车状态恢复后，连续检测到有效黑线的确认次数。
#define TRACE_TRACKING_REACQUIRE_COUNT       (2u)

//------------------------------------------------------------------------------
// 5. 顺时针右侧圆弧识别
//------------------------------------------------------------------------------
// bit4~bit7为右侧区域；bit0~bit2为左侧区域。
// 右侧区域持续满足入弯误差后进入右弯；左侧区域持续满足出弯误差后退出。
#define TRACE_CURVE_RIGHT_SENSOR_MASK        (0xF0u)
#define TRACE_CURVE_LEFT_SENSOR_MASK         (0x07u)
#define TRACE_CURVE_ENTRY_ERROR              (1.5f)
#define TRACE_CURVE_EXIT_ERROR               (-1.5f)

// 连续确认次数按10ms外环周期计数；2次约20ms，3次约30ms。
#define TRACE_CURVE_ENTRY_CONFIRM_COUNT      (2u)
#define TRACE_CURVE_EXIT_CONFIRM_COUNT       (3u)

//------------------------------------------------------------------------------
// 6. 半径750 mm圆弧速度与弯道PID
//------------------------------------------------------------------------------
// 理论内轮约124 mm/s；当前设为120，并由弯道PD继续修正实际偏差。
#define TRACE_CURVE_OUTER_SPEED_MM_S         (170)
#define TRACE_CURVE_INNER_SPEED_MM_S         (120)

// 弯道 PID 只在圆弧前馈速度上做修正，不影响直道 PID。
#define TRACE_CURVE_KP                       (8.0f)
#define TRACE_CURVE_KI                       (0.0f)
#define TRACE_CURVE_KD                       (2.5f)

// 弯道 PID 最大修正量，单位 mm/s。过弯不足可适当增大，摆动明显则减小。
#define TRACE_CURVE_MIN_CORRECTION_MM_S       (22.0f)
#define TRACE_CURVE_CORRECTION_LIMIT_MM_S    (45.0f)

//------------------------------------------------------------------------------
// 7. 全白丢线恢复
//------------------------------------------------------------------------------
// 丢线前黑线在右侧时：左轮为外轮、右轮为内轮；黑线在左侧时反向使用。
// 恢复速度越悬殊，找线转弯越急。重新检测到有效黑线后自动退出恢复状态。
#define TRACE_RECOVERY_OUTER_SPEED_MM_S      (140)
#define TRACE_RECOVERY_INNER_SPEED_MM_S      (50)

// 只有最后有效误差超过该值时才更新恢复方向，避免中央附近噪声反复改变方向。
#define TRACE_RECOVERY_DIRECTION_DEADBAND    (0.6f)

//------------------------------------------------------------------------------
// 8. 终点识别
//------------------------------------------------------------------------------
// 从首次识别到正常黑线开始计时；达到该时间后，8路全黑即锁存终点并停车。
// 10秒以前遇到全黑不会判终点，避免把起点黑色区域误判为终点。
#define TRACE_FINISH_ENABLE_TIME_MS           (10000u)

#if (TRACE_FILTER_SAMPLE_COUNT < 1u) || (TRACE_FILTER_SAMPLE_COUNT > 8u)
#error "TRACE_FILTER_SAMPLE_COUNT must be in the range 1..8"
#endif

#if (TRACE_FILTER_MAJORITY_COUNT < 1u) || \
    (TRACE_FILTER_MAJORITY_COUNT > TRACE_FILTER_SAMPLE_COUNT)
#error "TRACE_FILTER_MAJORITY_COUNT must be in the range 1..TRACE_FILTER_SAMPLE_COUNT"
#endif

#if (TRACE_LINE_CONTROL_PERIOD_MS < 1u)
#error "TRACE_LINE_CONTROL_PERIOD_MS must be at least 1"
#endif

#if (TRACE_TRACKING_REACQUIRE_COUNT < 1u) || \
    (TRACE_TRACKING_REACQUIRE_COUNT > 255u) || \
    (TRACE_CURVE_ENTRY_CONFIRM_COUNT < 1u) || \
    (TRACE_CURVE_ENTRY_CONFIRM_COUNT > 255u) || \
    (TRACE_CURVE_EXIT_CONFIRM_COUNT < 1u) || \
    (TRACE_CURVE_EXIT_CONFIRM_COUNT > 255u)
#error "Trace confirmation counts must be in the range 1..255"
#endif

#endif
