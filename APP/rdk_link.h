#ifndef __RDK_LINK_H
#define __RDK_LINK_H

//==============================================================================
// STM32 <-> RDK X5(地瓜派) 通信
//------------------------------------------------------------------------------
// 硬件: C10B板载CH9102F USB转串口 = USART1, 波特率115200, 8N1
//       PA9  = TX(单片机经CH9102F/Type-C发给上位机或RDK)
//       PA10 = RX(单片机经CH9102F/Type-C接收上位机或RDK指令)
//       电脑端直接连接主板Type-C并选择CH9102对应的COM口, 无需再接CN6。
//
// 上报报文(STM32 -> RDK X5), 约10Hz发送一行:
//   CAR,dist=<累计里程mm整数>,state=<小车状态枚举值>,vl=<左轮mm/s整数>,vr=<右轮mm/s整数>\r\n
//   例: CAR,dist=1234,state=1,vl=170,vr=170\r\n
//   state取值对应trace_task.h里的line_state_t:
//     0=LOST(丢线停车) 1=TRACKING(直道循迹) 2=CURVE_RIGHT(弯道)
//     3=RECOVERY(丢线恢复中) 4=ALL_BLACK(压在全黑区域) 5=FINISHED(已到终点停车)
//
// 下行指令(RDK X5 -> STM32): 以'\n'结尾的一行文本, 具体指令字符串后续再定义
//   (比如无人机抛投完成后的提速触发)。当前版本只负责收行、暴露给上层读取,
//   还没有接到循迹/变速逻辑里。
//==============================================================================

#include <stdint.h>

void RDK_Link_Init(void);
void RDK_Link_Report_Tick(void);

// 有新指令返回1并拷贝到out_buf(以'\0'结尾), 没有新指令返回0。
// max_len包含结尾的'\0', 单行最长63字节, 超过部分会被丢弃。
uint8_t RDK_Link_Read_Command(char* out_buf, uint8_t max_len);

#endif
