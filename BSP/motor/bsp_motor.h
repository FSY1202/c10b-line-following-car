#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "AllHeader.h"

#define MOTOR_IGNORE_PULSE  (350)   // 提高Ki后降低死区补偿，兼顾起步与低速平顺性
#define MOTOR_MAX_PULSE     (3600)  // PWM周期计数值(ARR), 对应TIM3 20kHz
#define MOTOR_FREQ_DIVIDE   (0)     // TIM3预分频(PSC)

// C10B主板双电机接口(AT8236 x2), 均为TIM3默认映射引脚, 无需AFIO重映射
// Motor_R: TIM3 CH1/CH2 = PA6/PA7 -> AT8236(U6) -> AO1/AO2 -> CN3(含编码器PB6/PB7)
#define MotorR_IN1_Port GPIOA
#define MotorR_IN1_Pin  GPIO_Pin_6
#define MotorR_IN1_Clk  RCC_APB2Periph_GPIOA
#define MotorR_IN2_Port GPIOA
#define MotorR_IN2_Pin  GPIO_Pin_7
#define MotorR_IN2_Clk  RCC_APB2Periph_GPIOA

// Motor_L: TIM3 CH3/CH4 = PB0/PB1 -> AT8236(U5) -> BO1/BO2 -> CN2(含编码器PC6/PC7)
#define MotorL_IN1_Port GPIOB
#define MotorL_IN1_Pin  GPIO_Pin_0
#define MotorL_IN1_Clk  RCC_APB2Periph_GPIOB
#define MotorL_IN2_Port GPIOB
#define MotorL_IN2_Pin  GPIO_Pin_1
#define MotorL_IN2_Clk  RCC_APB2Periph_GPIOB

#define PWM_R_IN1  TIM3->CCR1   // PA6
#define PWM_R_IN2  TIM3->CCR2   // PA7
#define PWM_L_IN1  TIM3->CCR3   // PB0
#define PWM_L_IN2  TIM3->CCR4   // PB1

// MOTOR: L=左轮, R=右轮 (两轮差速底盘, 仅2路电机, 无M3/M4)
typedef enum
{
    Motor_L = 0,
    Motor_R,
    Motor_MAX
} Motor_ID;

void motor_gpio_init(void);
void motor_pwm_init(uint16_t arr, uint16_t psc);
void Motor_Set_Pwm(uint8_t id, int16_t speed);
void Motor_Stop(uint8_t brake);

#endif
