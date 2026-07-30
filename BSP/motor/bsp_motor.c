#include "bsp_motor.h"

static int16_t Motor_Ignore_Dead_Zone(int16_t pulse)
{
    if (pulse > 0) return pulse + MOTOR_IGNORE_PULSE;
    if (pulse < 0) return pulse - MOTOR_IGNORE_PULSE;
    return 0;
}

// 初始化2路电机的4个PWM引脚(TIM3默认映射, 无需重映射)
void motor_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    gpio_t motor_pwm[] =
    {
        {MotorR_IN1_Port, MotorR_IN1_Pin, MotorR_IN1_Clk},
        {MotorR_IN2_Port, MotorR_IN2_Pin, MotorR_IN2_Clk},
        {MotorL_IN1_Port, MotorL_IN1_Pin, MotorL_IN1_Clk},
        {MotorL_IN2_Port, MotorL_IN2_Pin, MotorL_IN2_Clk},
    };

    for (u8 i = 0; i < Motor_MAX * 2; i++)
    {
        RCC_APB2PeriphClockCmd(motor_pwm[i].clock, ENABLE);
        GPIO_InitStructure.GPIO_Pin = motor_pwm[i].pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出(TIM3 PWM)
        GPIO_Init(motor_pwm[i].port, &GPIO_InitStructure);
    }
}

// TIM3 4通道PWM初始化, arr/psc决定PWM频率(默认20kHz: 72MHz/3600)
void motor_pwm_init(uint16_t arr, uint16_t psc)
{
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_Period = arr - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // PA6  Motor_R IN1
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); // PA7  Motor_R IN2
    TIM_OC3Init(TIM3, &TIM_OCInitStructure); // PB0  Motor_L IN1
    TIM_OC4Init(TIM3, &TIM_OCInitStructure); // PB1  Motor_L IN2

    TIM_Cmd(TIM3, ENABLE);
}

// 设置指定电机速度, speed:[-MOTOR_MAX_PULSE, MOTOR_MAX_PULSE], 正负代表转向
void Motor_Set_Pwm(uint8_t id, int16_t speed)
{
    int16_t pulse = Motor_Ignore_Dead_Zone(speed);

    if (pulse >= MOTOR_MAX_PULSE)
        pulse = MOTOR_MAX_PULSE;
    if (pulse <= -MOTOR_MAX_PULSE)
        pulse = -MOTOR_MAX_PULSE;

    switch (id)
    {
    case Motor_L:
    {
        if (pulse >= 0)
        {
            PWM_L_IN1 = 0;
            PWM_L_IN2 = pulse;
        }
        else
        {
            PWM_L_IN1 = -pulse;
            PWM_L_IN2 = 0;
        }
        break;
    }
    case Motor_R:
    {
        if (pulse >= 0)
        {
            PWM_R_IN1 = pulse;
            PWM_R_IN2 = 0;
        }
        else
        {
            PWM_R_IN1 = 0;
            PWM_R_IN2 = -pulse;
        }
        break;
    }
    default:
        break;
    }
}

// 停止所有电机, brake!=0时刹车(两个IN同时拉高), 否则自由滑行(两个IN同时置0)
void Motor_Stop(uint8_t brake)
{
    int16_t hold = (brake != 0) ? MOTOR_MAX_PULSE : 0;
    PWM_R_IN1 = hold;
    PWM_R_IN2 = hold;
    PWM_L_IN1 = hold;
    PWM_L_IN2 = hold;
}
