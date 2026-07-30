#include "app_motor.h"

// 控制周期(10ms)前后编码器计数快照
static int g_Encoder_All_Now[Motor_MAX] = {0};
static int g_Encoder_All_Last[Motor_MAX] = {0};
static int g_Encoder_All_Offset[Motor_MAX] = {0};

uint8_t g_start_ctrl = 0;
uint8_t g_motor_reverse_fault[Motor_MAX] = {0};

car_data_t car_data;
motor_data_t motor_data;

float Motion_Get_APB(void)
{
    return STM32Car_APB;
}

float Motion_Get_Circle_MM(void)
{
    return WHEEL_CIRCLE_MM;
}

// 读取编码器原始计数, 计算与上次的差值
void Motion_Get_Encoder(void)
{
    Encoder_Get_ALL(g_Encoder_All_Now);

    for (uint8_t i = 0; i < Motor_MAX; i++)
    {
        g_Encoder_All_Offset[i] = g_Encoder_All_Now[i] - g_Encoder_All_Last[i];
        g_Encoder_All_Last[i] = g_Encoder_All_Now[i];
    }
}

// 直接按PWM值控制左右轮(不经过速度闭环), Motor_x=[-MOTOR_MAX_PULSE, MOTOR_MAX_PULSE]
void Motion_Set_Pwm(int16_t Motor_L_Pwm, int16_t Motor_R_Pwm)
{
    if (Motor_L_Pwm >= -MOTOR_MAX_PULSE && Motor_L_Pwm <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(Motor_L, Motor_L_Pwm);
    }
    if (Motor_R_Pwm >= -MOTOR_MAX_PULSE && Motor_R_Pwm <= MOTOR_MAX_PULSE)
    {
        Motor_Set_Pwm(Motor_R, Motor_R_Pwm);
    }
}

// 设置左右轮目标速度, 单位mm/s, 交给10ms定时器里的速度闭环处理
void Motion_Set_Speed(int16_t speed_left, int16_t speed_right)
{
    g_start_ctrl = 1;
    motor_data.speed_set[Motor_L] = speed_left;
    motor_data.speed_set[Motor_R] = speed_right;

    PID_Set_Motor_Target(Motor_L, motor_data.speed_set[Motor_L] * 1.0f);
    PID_Set_Motor_Target(Motor_R, motor_data.speed_set[Motor_R] * 1.0f);
}

// 从编码器计算左右轮实际速度(mm/s), 并驱动PID计算出新的PWM值
void Motion_Get_Speed(car_data_t* car)
{
    float speed_mm[Motor_MAX] = {0};
    float circle_mm = Motion_Get_Circle_MM();
    float robot_apb = Motion_Get_APB();

    Motion_Get_Encoder();

    // 10ms周期内的脉冲数 -> mm/s: (脉冲数/每圈脉冲数) * 每圈毫米数 * (1000ms/10ms)
    for (uint8_t i = 0; i < Motor_MAX; i++)
    {
        speed_mm[i] = (g_Encoder_All_Offset[i]) * 100.0f * circle_mm / ENCODER_PULSE_PER_REV;
    }

    car->Vx = (speed_mm[Motor_L] + speed_mm[Motor_R]) / 2.0f;
    car->Vz = (speed_mm[Motor_R] - speed_mm[Motor_L]) / (2.0f * robot_apb) * 1000.0f;

    if (g_start_ctrl)
    {
        motor_data.speed_mm_s[Motor_L] = speed_mm[Motor_L];
        motor_data.speed_mm_s[Motor_R] = speed_mm[Motor_R];

        PID_Calc_Motor(&motor_data);
    }
}

// 运动控制句柄, 每10ms调用一次(由bsp_timer.c的TIM6中断驱动)
void Motion_Handle(void)
{
    Motion_Get_Speed(&car_data);

    if (g_start_ctrl)
    {
        for (uint8_t i = 0; i < Motor_MAX; i++)
        {
            if (motor_data.speed_set[i] > 0 &&
                motor_data.speed_mm_s[i] < -MOTOR_REVERSE_FAULT_SPEED_MM_S)
            {
                g_motor_reverse_fault[i] = 1;
            }

            if (g_motor_reverse_fault[i])
            {
                motor_data.speed_pwm[i] = 0.0f;
            }
        }

        Motion_Set_Pwm((int16_t)motor_data.speed_pwm[Motor_L], (int16_t)motor_data.speed_pwm[Motor_R]);
    }
}
