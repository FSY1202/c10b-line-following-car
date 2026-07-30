#include "AllHeader.h"
#include "odometry.h"
#include "rdk_link.h"

#define RDK_REPORT_PERIOD_MS   (100u)   // 上报频率约10Hz
#define RDK_RX_BUF_LEN         (64u)

static uint32_t g_last_report_ms = 0;

// 接收: 中断里按字节收集到g_rx_buf, 遇到'\n'就拷贝成一整行到g_rx_line并置位
static char g_rx_buf[RDK_RX_BUF_LEN];
static uint8_t g_rx_index = 0;
static char g_rx_line[RDK_RX_BUF_LEN];
static volatile uint8_t g_rx_line_ready = 0;

static long round_to_long(float value)
{
    return (value >= 0.0f) ? (long)(value + 0.5f) : (long)(value - 0.5f);
}

static void RDK_UART4_SendByte(uint8_t ch)
{
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
    USART_SendData(UART4, ch);
}

static void RDK_UART4_SendString(const char* s)
{
    while (*s != '\0')
    {
        RDK_UART4_SendByte((uint8_t)*s);
        s++;
    }
}

void RDK_Link_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    // TX: PC10 (UART4引脚固定, 不支持重映射)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // RX: PC11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART4, ENABLE);

    // 抢占优先级特意设为3(本工程NVIC_PriorityGroup_2下的最低抢占优先级),
    // 严格低于TIM6(抢占优先级2, 负责电机测速/PID闭环)。这样即便地瓜派那头
    // 字节到达得很密集, 也只会是UART4中断被TIM6短暂延后, 不会反过来干扰
    // 电机控制这个更关键的实时任务。
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    g_last_report_ms = g_system_tick_ms;
    g_rx_index = 0;
    g_rx_line_ready = 0;
}

// UART4接收中断: 按行收集地瓜派发来的指令, 遇到'\n'认为一行结束(忽略'\r')
void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = (uint8_t)USART_ReceiveData(UART4);

        if (ch == '\n')
        {
            if (g_rx_index > 0)
            {
                uint8_t i;
                // g_rx_index此时最大为RDK_RX_BUF_LEN-1(63), 终止符就写在这个
                // 位置, 两个数组都声明为RDK_RX_BUF_LEN大小, 所以这里是最后一个
                // 合法下标, 没有越界。
                g_rx_buf[g_rx_index] = '\0';

                for (i = 0; i <= g_rx_index && i < RDK_RX_BUF_LEN; i++)
                {
                    g_rx_line[i] = g_rx_buf[i];
                }
                g_rx_line_ready = 1;
            }
            g_rx_index = 0;
        }
        else if (ch != '\r')
        {
            if (g_rx_index < (RDK_RX_BUF_LEN - 1))
            {
                g_rx_buf[g_rx_index++] = (char)ch;
            }
            else
            {
                // 单行超长(超过63字节), 丢弃重新开始, 防止缓冲区溢出
                g_rx_index = 0;
            }
        }
    }
}

// 主循环每次调用都行, 内部用真实毫秒计数g_system_tick_ms按100ms周期(约10Hz)
// 实际发送, 不用"数主循环次数"的方式(那样在主循环单次耗时不稳定时会导致
// 实际发送间隔跟预期的100ms偏差较大)。
void RDK_Link_Report_Tick(void)
{
    uint32_t now = g_system_tick_ms;

    if ((now - g_last_report_ms) < RDK_REPORT_PERIOD_MS)
    {
        return;
    }
    g_last_report_ms = now;

    char line[64];
    long dist_mm = round_to_long(Odometry_Get_Distance_mm());
    long vl = round_to_long(motor_data.speed_mm_s[Motor_L]);
    long vr = round_to_long(motor_data.speed_mm_s[Motor_R]);

    sprintf(line, "CAR,dist=%ld,state=%d,vl=%ld,vr=%ld\r\n",
            dist_mm, (int)g_line_controller.state, vl, vr);

    RDK_UART4_SendString(line);
}

uint8_t RDK_Link_Read_Command(char* out_buf, uint8_t max_len)
{
    uint8_t i;

    if (!g_rx_line_ready)
    {
        return 0;
    }

    if (max_len == 0u)
    {
        // 调用方传了0长度缓冲区, 没地方放结尾符, 直接当作没有可用指令处理。
        // 注意不清g_rx_line_ready, 等调用方传对了长度还能再读一次。
        return 0;
    }

    for (i = 0; g_rx_line[i] != '\0' && i < (max_len - 1); i++)
    {
        out_buf[i] = g_rx_line[i];
    }
    out_buf[i] = '\0';

    g_rx_line_ready = 0;
    return 1;
}
