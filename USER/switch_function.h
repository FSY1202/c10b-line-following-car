#ifndef __SWITCH_FUNCTION_H
#define __SWITCH_FUNCTION_H

// 0:关闭  1:开启
// 当前只做循迹这一部分, 一键启动/急停/圈数状态机/位置上报等留到下一阶段再加
#define ENABLE_MOTOR   1   // 电机+编码器+速度闭环

#define DEBUG_USARTx USART1 // 板载CH9102F/Type-C串口，与RDK链路共用USART1

#endif
