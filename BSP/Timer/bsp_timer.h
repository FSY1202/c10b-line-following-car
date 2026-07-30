#ifndef __BSP_TIMER_H__
#define __BSP_TIMER_H__

#include "AllHeader.h"


void TIM6_Init(void);
void TIM7_Init(void);

void my_delay_10ms(u16 time);

// 自由累加的真实毫秒计数(由TIM6硬件中断每1ms精确自增一次, 不会像"数主循环
// 次数"那样受循环体实际耗时影响), 供odometry.c/rdk_link.c等需要准确计时的
// 模块使用, 只增不清零。
extern volatile uint32_t g_system_tick_ms;

#endif
