#ifndef __GRAYSCALE_SENSOR_H
#define __GRAYSCALE_SENSOR_H

#include <stdint.h>
#include "AllHeader.h"
#include "delay.h"

//=====================================================================================
//  引脚配置接口 (Pin Configuration) —— C10B主板(STM32F103RCT6)闲置IO，见H3/H5排针
//=====================================================================================
// --- 通道选择引脚定义 (AD0, AD1, AD2) ---
#define SENSOR_AD0_PORT         GPIOA
#define SENSOR_AD0_PIN          GPIO_Pin_2

#define SENSOR_AD1_PORT         GPIOA
#define SENSOR_AD1_PIN          GPIO_Pin_3

#define SENSOR_AD2_PORT         GPIOA
#define SENSOR_AD2_PIN          GPIO_Pin_8

#define GrayS_OUT_PORT          GPIOA
#define GrayS_OUT_PIN           GPIO_Pin_11

//=====================================================================================
//  GPIO操作抽象接口 (GPIO Operation Macros)
//=====================================================================================
#define SENSOR_AD0_WRITE(state)  GPIO_WriteBit(SENSOR_AD0_PORT, SENSOR_AD0_PIN, (state) ? Bit_SET : Bit_RESET)
#define SENSOR_AD1_WRITE(state)  GPIO_WriteBit(SENSOR_AD1_PORT, SENSOR_AD1_PIN, (state) ? Bit_SET : Bit_RESET)
#define SENSOR_AD2_WRITE(state)  GPIO_WriteBit(SENSOR_AD2_PORT, SENSOR_AD2_PIN, (state) ? Bit_SET : Bit_RESET)

#define SENSOR_OUT_READ()        GPIO_ReadInputDataBit(GrayS_OUT_PORT, GrayS_OUT_PIN)

//=====================================================================================
//  驱动函数接口 (Driver API)
//=====================================================================================

#define GRAYSCALE_SENSOR_CHANNELS   8   // 传感器通道总数

void Grayscale_Sensor_Init(void);
void Grayscale_Sensor_Read_All(uint16_t* sensor_values);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);

#endif // __GRAYSCALE_SENSOR_H
