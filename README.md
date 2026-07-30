# C10B_Trace

STM32F103RCT6（轮趣科技C10B主板）循迹小车固件，Keil MDK + 标准外设库(SPL)。

## 目录结构

- `APP/` — 循迹算法(`trace_task`)、电机控制(`app_motor`/`PID_Motor`)、里程计(`odometry`)、与上位机通信(`rdk_link`)
- `BSP/` — 板级驱动：电机(`motor`)、灰度传感器(`Grayscale_Sensor`)、串口(`USART`)、定时器(`Timer`)
- `CMSIS/` `FWLib/` — STM32F10x标准外设库
- `USER/` — 工程入口(`main.c`)、Keil工程文件(`C10B_Trace.uvprojx`)

## 硬件

- 主控：STM32F103RCT6
- 循迹：8路灰度传感器模块（CD4051多路选通）
- 电机：GM37-520编码电机 x2，AT8236驱动
- 与上位机(RDK X5)通信：UART4，PC10=TX / PC11=RX，115200-8N1，协议见`APP/rdk_link.h`
