#include "AllHeader.h"
#include "odometry.h"

// 至少间隔这么久才累加一次, 避免主循环空转时做无意义的浮点运算
#define ODOMETRY_MIN_UPDATE_MS   (5u)

static float g_distance_mm = 0.0f;
static uint32_t g_last_update_ms = 0;

void Odometry_Init(void)
{
    g_distance_mm = 0.0f;
    g_last_update_ms = g_system_tick_ms;
}

// 主循环每次调用都行, 内部用TIM6硬件中断驱动的真实毫秒计数g_system_tick_ms
// 算出本次实际经过的时间, 而不是假设固定10ms —— 避免主循环单次耗时不稳定
// (比如读8路灰度传感器有多次delay_us)导致"以为过了10ms、实际过了更久"
// 从而把里程算少。
void Odometry_Update(void)
{
    uint32_t now = g_system_tick_ms;
    uint32_t elapsed_ms = now - g_last_update_ms;

    if (elapsed_ms < ODOMETRY_MIN_UPDATE_MS)
    {
        return;
    }
    g_last_update_ms = now;

    // 必须用"当前速度 x 实际经过的时间"逐步累加, 不能用平均速度整体估算,
    // 否则后续加了变速(抛投期间减速/抛投完成后提速)之后累计弧长会算错。
    // 详见项目记忆 car_rdk_report_plan.md。
    g_distance_mm += car_data.Vx * ((float)elapsed_ms / 1000.0f);
}

float Odometry_Get_Distance_mm(void)
{
    return g_distance_mm;
}

void Odometry_Reset(void)
{
    g_distance_mm = 0.0f;
    g_last_update_ms = g_system_tick_ms;
}
