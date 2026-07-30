#include "PID_Motor.h"

pid_t pid_motor[Motor_MAX];

// 初始化PID参数
void PID_Param_Init(void)
{
    for (int i = 0; i < Motor_MAX; i++)
    {
        pid_motor[i].target_val = 0.0f;
        pid_motor[i].pwm_output = 0.0f;
        pid_motor[i].err = 0.0f;
        pid_motor[i].err_last = 0.0f;
        pid_motor[i].err_next = 0.0f;
        pid_motor[i].integral = 0.0f;

        pid_motor[i].Kp = PID_MOTOR_KP;
        pid_motor[i].Ki = PID_MOTOR_KI;
        pid_motor[i].Kd = PID_MOTOR_KD;
    }
}

// 编码器速度内环：纯I控制
float PID_Incre_Calc(pid_t *pid, float actual_val)
{
    float output_limit = (float)(MOTOR_MAX_PULSE - MOTOR_IGNORE_PULSE);

    pid->err = pid->target_val - actual_val;

    // 目标为0时立即清空积分，确保丢线/全黑停车不会保留旧PWM。
    if (pid->target_val == 0.0f)
    {
        pid->pwm_output = 0.0f;
        pid->integral = 0.0f;
        pid->err = 0.0f;
        pid->err_last = 0.0f;
        pid->err_next = 0.0f;
        return 0.0f;
    }

    // 纯积分：积分误差直接换算为PWM。
    pid->integral += pid->err;
    pid->pwm_output = pid->Ki * pid->integral;

    // 正目标只允许正PWM，负目标只允许负PWM，避免低速时来回反转。
    if (pid->target_val > 0.0f)
    {
        if (pid->pwm_output > output_limit)
            pid->pwm_output = output_limit;
        if (pid->pwm_output < 0.0f)
            pid->pwm_output = 0.0f;
    }
    else
    {
        if (pid->pwm_output < -output_limit)
            pid->pwm_output = -output_limit;
        if (pid->pwm_output > 0.0f)
            pid->pwm_output = 0.0f;
    }

    // 用限幅后的输出反算积分，防止积分继续饱和。
    if (pid->Ki > 0.0f)
        pid->integral = pid->pwm_output / pid->Ki;
    else
        pid->integral = 0.0f;

    pid->err_last = pid->err_next;
    pid->err_next = pid->err;

    return pid->pwm_output;
}

// 计算左右两轮的PID输出值
void PID_Calc_Motor(motor_data_t* motor)
{
    for (int i = 0; i < Motor_MAX; i++)
    {
        motor->speed_pwm[i] = PID_Incre_Calc(&pid_motor[i], motor->speed_mm_s[i]);
    }
}

// 设置PID目标速度, 单位为mm/s; motor_id传Motor_MAX时对左右轮设为同一目标值
void PID_Set_Motor_Target(uint8_t motor_id, float target)
{
    if (motor_id > Motor_MAX) return;

    if (motor_id == Motor_MAX)
    {
        for (int i = 0; i < Motor_MAX; i++)
        {
            pid_motor[i].target_val = target;
        }
    }
    else
    {
        pid_motor[motor_id].target_val = target;
    }
}
