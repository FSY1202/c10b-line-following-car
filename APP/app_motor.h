#ifndef  __APP_MOTOR_H
#define  __APP_MOTOR_H

#include "AllHeader.h"

// GM37-520编码电机: 11PPR AB双相霍尔编码器, TIM按TI12模式4倍频计数, 减速比i30(输出轴320rpm)
// 每圈脉冲数 = 11(线数) * 4(4倍频) * 30(减速比) = 1320
// !!!假设编码器装在电机高速轴(减速箱前), 这是这类GM37系列的常见装法。
// !!!上电后务必用手把轮子转整整一圈, 核对Encoder_Get_Count_Now()读数是否约为1320,
// !!!如果读数约为44(=11*4), 说明编码器装在输出轴(减速箱后), 需要把*30去掉重新定义。
#define ENCODER_PULSE_PER_REV   (1320.0f)

// 车轮外圈周长(mm), 来自实测"22cm多一点", 建议后续用卷尺/走已知距离核实修正
#define WHEEL_CIRCLE_MM         (223.0f)

// 左右驱动轮轮距的一半(mm), 来自实测轮距23-24cm, 取中间值235mm/2, 待底盘定稿后精确测量
#define STM32Car_APB            (117.5f)

// 低速调试保护: 目标向前但实测反向超过该速度时锁存故障并停止对应车轮
#define MOTOR_REVERSE_FAULT_SPEED_MM_S  (50.0f)

typedef struct _car_data
{
    float Vx;   // 车体线速度 mm/s (左右轮平均值)
    float Vz;   // 车体角速度 mrad/s (供后续里程计/位置反算使用, 当前循迹逻辑暂不消费)
} car_data_t;

// [0]=左轮, [1]=右轮; 置1后保持到主板复位, 可在Keil Watch中观察
extern uint8_t g_motor_reverse_fault[2];

// 供odometry.c/rdk_link.c等只读消费, 不在这些模块里重新计算, 避免逻辑分叉
extern car_data_t car_data;
// motor_data的extern声明放在PID_Motor.h里(它的类型motor_data_t在那里定义),
// 不要挪到这里 —— 会重现Motor_MAX那次踩过的循环include顺序问题。

void Motion_Set_Pwm(int16_t Motor_L_Pwm, int16_t Motor_R_Pwm);
void Motion_Get_Encoder(void);
void Motion_Set_Speed(int16_t speed_left, int16_t speed_right);
void Motion_Handle(void);
void Motion_Get_Speed(car_data_t* car);
float Motion_Get_Circle_MM(void);
float Motion_Get_APB(void);

#endif
