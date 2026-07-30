#ifndef  __PID_MOTOR_H
#define  __PID_MOTOR_H

#include "AllHeader.h"

#define PI      (3.1415926f)

// 编码器速度内环使用纯I控制：
// PWM(k) = PWM(k-1) + KI * (目标速度 - 实际速度)
// KI越大，克服静摩擦和加速越快；若速度上下波动明显，应适当减小。
#define PID_MOTOR_KP  (0.0f)   // 速度环不使用P
#define PID_MOTOR_KI  (0.80f)  // 提高纯I响应，缩短轮速差建立时间
#define PID_MOTOR_KD  (0.0f)   // 速度环不使用D

typedef struct _pid
{
    float target_val;      // 目标值
    float output_val;      // 输出值
    float pwm_output;      // PWM输出值

    float Kp, Ki, Kd;      // 比例、积分、微分系数
    float err;              // 当前偏差值
    float err_last;         // 上一次偏差值(增量式)
    float err_next;         // 上上次偏差值(增量式)
    float integral;         // 积分累计值(位置式)
} pid_t;

// 注意: 这里用字面量2而不是Motor_MAX, 因为bsp_motor.h通过AllHeader.h形成循环包含,
// 本文件被解析到时Motor_MAX这个枚举值还未定义。如果以后改成2个以上电机, 这里要同步改。
#define MOTOR_DATA_COUNT  (2)

typedef struct _motor_data_t
{
    float speed_mm_s[MOTOR_DATA_COUNT];   // 输入值: 编码器计算出的实际速度(mm/s)
    float speed_pwm[MOTOR_DATA_COUNT];    // 输出值: PID计算出的PWM值
    int16_t speed_set[MOTOR_DATA_COUNT];  // 速度设定值(mm/s)
} motor_data_t;

// 供odometry.c/rdk_link.c等只读消费。放在这里(而不是app_motor.h)是因为
// motor_data_t类型就在本文件定义, 同文件内顺序引用不会有循环include顺序问题。
extern motor_data_t motor_data;

void PID_Param_Init(void);
void PID_Calc_Motor(motor_data_t* motor);
void PID_Set_Motor_Target(uint8_t motor_id, float target);
float PID_Incre_Calc(pid_t *pid, float actual_val);

#endif
