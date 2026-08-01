#include "AllHeader.h"
#include "odometry.h"
#include "rdk_link.h"

#define RDK_REPORT_PERIOD_MS   (100u)   // 上报频率约10Hz
#define RDK_RX_BUF_LEN         (64u)
#define RDK_RX_QUEUE_DEPTH     (8u)

static uint32_t g_last_report_ms = 0;

// 接收: 中断按字节组帧，完整行进入环形队列，避免连续指令互相覆盖。
static char g_rx_buf[RDK_RX_BUF_LEN];
static uint8_t g_rx_index = 0;
static char g_rx_queue[RDK_RX_QUEUE_DEPTH][RDK_RX_BUF_LEN];
static volatile uint8_t g_rx_queue_head = 0;
static volatile uint8_t g_rx_queue_tail = 0;

static long round_to_long(float value)
{
    return (value >= 0.0f) ? (long)(value + 0.5f) : (long)(value - 0.5f);
}

static void RDK_USART1_SendByte(uint8_t ch)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, ch);
}

static void RDK_USART1_SendString(const char* s)
{
    while (*s != '\0')
    {
        RDK_USART1_SendByte((uint8_t)*s);
        s++;
    }
}

void RDK_Link_Init(void)
{
    // USART1/PA9/PA10已由bsp_init()按115200、8N1初始化，并连接板载
    // CH9102F。这里接管USART1接收中断，使RDK链路可直接通过Type-C通信。
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    g_last_report_ms = g_system_tick_ms;
    g_rx_index = 0;
    g_rx_queue_head = 0;
    g_rx_queue_tail = 0;
}

// USART1接收中断: 按行收集地瓜派发来的指令, 遇到'\n'认为一行结束(忽略'\r')
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = (uint8_t)USART_ReceiveData(USART1);

        if (ch == '\n')
        {
            if (g_rx_index > 0)
            {
                uint8_t i;
                uint8_t next_head =
                    (uint8_t)((g_rx_queue_head + 1u) % RDK_RX_QUEUE_DEPTH);

                // 容量为7条，足以容纳3个题号帧和3个启动帧。
                if (next_head != g_rx_queue_tail)
                {
                    for (i = 0; i < g_rx_index; i++)
                    {
                        g_rx_queue[g_rx_queue_head][i] = g_rx_buf[i];
                    }
                    g_rx_queue[g_rx_queue_head][g_rx_index] = '\0';
                    g_rx_queue_head = next_head;
                }
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

    RDK_USART1_SendString(line);
}

uint8_t RDK_Link_Read_Command(char* out_buf, uint8_t max_len)
{
    uint8_t i;
    uint8_t tail;

    if (g_rx_queue_tail == g_rx_queue_head)
    {
        return 0;
    }

    if (max_len == 0u)
    {
        // 调用方传了0长度缓冲区, 没地方放结尾符, 直接当作没有可用指令处理。
        // 不移动队列尾指针，调用方传入有效缓冲区后仍可再次读取。
        return 0;
    }

    tail = g_rx_queue_tail;
    for (i = 0; g_rx_queue[tail][i] != '\0' && i < (max_len - 1); i++)
    {
        out_buf[i] = g_rx_queue[tail][i];
    }
    out_buf[i] = '\0';

    g_rx_queue_tail =
        (uint8_t)((g_rx_queue_tail + 1u) % RDK_RX_QUEUE_DEPTH);
    return 1;
}
