#ifndef __ODOMETRY_H
#define __ODOMETRY_H

// 里程计: 累计小车沿赛道行驶的弧长(mm), 只做加减法, 不涉及电机/循迹逻辑。
// 由main.c的while(1)循环每次调用Odometry_Update(), 内部用bsp_timer.c提供的
// 真实毫秒计数g_system_tick_ms(而不是数循环次数)计算实际经过的时间, 每次
// 间隔至少5ms才真正累加一次距离。

void Odometry_Init(void);
void Odometry_Update(void);
float Odometry_Get_Distance_mm(void);
void Odometry_Reset(void);

#endif
