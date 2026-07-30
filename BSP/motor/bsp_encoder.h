#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "AllHeader.h"

// C10B主板双编码器接口, 均为TIM默认映射引脚, 无需AFIO重映射
// 编码器L(左轮, 随Motor_L/CN2一起走线): TIM8 CH1/CH2 = PC6/PC7
#define ENC_L_A_CLK  RCC_APB2Periph_GPIOC
#define ENC_L_A_PORT GPIOC
#define ENC_L_A_PIN  GPIO_Pin_6
#define ENC_L_B_CLK  RCC_APB2Periph_GPIOC
#define ENC_L_B_PORT GPIOC
#define ENC_L_B_PIN  GPIO_Pin_7

// 编码器R(右轮, 随Motor_R/CN3一起走线): TIM4 CH1/CH2 = PB6/PB7
#define ENC_R_A_CLK  RCC_APB2Periph_GPIOB
#define ENC_R_A_PORT GPIOB
#define ENC_R_A_PIN  GPIO_Pin_6
#define ENC_R_B_CLK  RCC_APB2Periph_GPIOB
#define ENC_R_B_PORT GPIOB
#define ENC_R_B_PIN  GPIO_Pin_7

#define ENCODER_TIM_PERIOD  (uint16_t)(65535)

void motor_encode_init(void);
void Encoder_Update_Count(void);
void Encoder_Get_ALL(int* Encoder_all);
int Encoder_Get_Count_Now(uint8_t Motor_id);

#endif
