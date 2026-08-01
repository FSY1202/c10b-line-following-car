#include "AllHeader.h"
#include "rdk_link.h"
#include "gcs_cmd.h"

#define GCS_CMD_BUF_LEN   (32u)
#define GCS_SPEED_RAMP_PERIOD_MS   (10u)
#define GCS_SPEED_RAMP_STEP_MM_S   (1)

#define GCS_TASK1_STABLE_START_SPEED_MM_S   (60)
#define GCS_TASK1_INITIAL_RAMP_END_MS        (600u)
#define GCS_TASK1_HOLD_END_MS                (4600u)
#define GCS_TASK1_PROFILE_END_MS             (5000u)
#define GCS_TASK2_INITIAL_FAST_TIME_MS        (5000u)

// 第一题需要更长的伴飞窗口，第二题保持已经验证过的慢速。
#define GCS_SPEED_TASK1_SLOW_MM_S   (100)
#define GCS_SPEED_TASK2_SLOW_MM_S   (130)
#define GCS_SPEED_FAST_MM_S         (200)

static uint8_t g_car_started = 0u;
static uint8_t g_task_number = 0u;
static int16_t g_current_speed_mm_s = 0;
static int16_t g_target_speed_mm_s = GCS_SPEED_TASK2_SLOW_MM_S;
static uint32_t g_last_ramp_ms = 0u;
static uint32_t g_task1_profile_start_ms = 0u;
static uint32_t g_task2_initial_fast_start_ms = 0u;
static uint8_t g_task2_initial_fast_active = 0u;

// 计算payload_start(含)到star(不含)之间所有字符的逐字节XOR
static uint8_t compute_checksum(const char* payload_start, const char* star)
{
    uint8_t ck = 0;
    const char* p;

    for (p = payload_start; p < star; p++)
    {
        ck ^= (uint8_t)(*p);
    }
    return ck;
}

// 解析形如 $CAR,V,<code>*<CK> 的一行(不含\r\n, RDK_Link已经剥掉了)。
// 成功返回1并把code写进*out_code; 帧头/校验/格式任何一处不对都返回0。
static uint8_t parse_car_speed_cmd(const char* line, int* out_code)
{
    const char* star;
    uint8_t ck_calc;
    char* endptr;
    long ck_recv;

    if (line[0] != '$')
    {
        return 0;
    }

    star = strchr(line, '*');
    if (star == NULL)
    {
        return 0;
    }

    ck_calc = compute_checksum(line + 1, star);
    ck_recv = strtol(star + 1, &endptr, 16);
    if (endptr == (star + 1))
    {
        return 0;  // *后面没有合法的十六进制校验位
    }
    if ((uint8_t)ck_recv != ck_calc)
    {
        return 0;  // 校验错, 丢弃(跟地面站参考实现一致)
    }

    if (strncmp(line + 1, "CAR,V,", 6) != 0)
    {
        return 0;  // 不是本车认识的指令类型
    }

    *out_code = atoi(line + 7);
    return 1;
}

// 启动帧固定为$CAR,S,1*32，RDK_Link已剥掉行尾的CR/LF。
static uint8_t parse_car_start_cmd(const char* line)
{
    if ((line[0] != '$') ||
        (line[1] != 'C') || (line[2] != 'A') || (line[3] != 'R') ||
        (line[4] != ',') || (line[5] != 'S') || (line[6] != ',') ||
        (line[7] != '1') || (line[8] != '*') ||
        (line[9] != '3') || (line[10] != '2') || (line[11] != '\0'))
    {
        return 0;
    }

    return (compute_checksum(line + 1, line + 8) == 0x32u) ? 1u : 0u;
}

// 题号帧固定为$CAR,T,1*35或$CAR,T,2*36，重复接收只覆盖同一锁存值。
static uint8_t parse_car_task_cmd(const char* line, uint8_t* out_task)
{
    uint8_t expected_checksum;

    if ((line[0] != '$') ||
        (line[1] != 'C') || (line[2] != 'A') || (line[3] != 'R') ||
        (line[4] != ',') || (line[5] != 'T') || (line[6] != ',') ||
        ((line[7] != '1') && (line[7] != '2')) || (line[8] != '*') ||
        (line[9] != '3') || (line[11] != '\0'))
    {
        return 0;
    }

    expected_checksum = (line[7] == '1') ? 0x35u : 0x36u;
    if ((compute_checksum(line + 1, line + 8) != expected_checksum) ||
        (line[10] != ((line[7] == '1') ? '5' : '6')))
    {
        return 0;
    }

    *out_task = (uint8_t)(line[7] - '0');
    return 1;
}

// 慢速档: 直道和弯道速度一起降, base_speed/max_speed都下调
// 确保PID修正也不会把单轮瞬时速度顶到超过慢速上限。
static void apply_slow_speed(void)
{
    if (g_task_number == 1u)
    {
        g_target_speed_mm_s = GCS_SPEED_TASK1_SLOW_MM_S;
    }
    else if (g_task2_initial_fast_active != 0u)
    {
        g_target_speed_mm_s = GCS_SPEED_FAST_MM_S;
    }
    else
    {
        g_target_speed_mm_s = GCS_SPEED_TASK2_SLOW_MM_S;
    }
}

// 快速档: 直道和弯道都恢复到trace_config.h里原本调好的正常速度
static void apply_fast_speed(void)
{
    g_target_speed_mm_s = GCS_SPEED_FAST_MM_S;
}

// 平滑公共前进速度，循迹PD产生的左右轮差速仍实时生效。
static void apply_profile_speed(int16_t speed_mm_s)
{
    g_line_controller.base_speed = speed_mm_s;
    g_line_controller.max_speed = speed_mm_s;
    g_line_controller.curve_outer_speed = speed_mm_s;
    g_line_controller.curve_inner_speed =
        (int16_t)(((int32_t)speed_mm_s * TRACE_CURVE_INNER_SPEED_MM_S +
                   TRACE_CURVE_OUTER_SPEED_MM_S / 2) /
                  TRACE_CURVE_OUTER_SPEED_MM_S);
}

// 上电默认档位: 慢速。必须在main.c里line_following_init()之后调用,
// 否则会被line_following_init()里设的快速初值覆盖回去。
// 没有修改trace_task.c/trace_config.h本身的初始化逻辑或调好的正常速度
// 常数, 只是在它们跑完之后, 从这里再改一次g_line_controller的速度字段。
void GCS_Cmd_Init(void)
{
    g_car_started = 0u;
    g_task_number = 0u;
    g_current_speed_mm_s = 0;
    g_target_speed_mm_s = GCS_SPEED_TASK2_SLOW_MM_S;
    g_last_ramp_ms = g_system_tick_ms;
    g_task1_profile_start_ms = g_system_tick_ms;
    g_task2_initial_fast_start_ms = g_system_tick_ms;
    g_task2_initial_fast_active = 0u;
    apply_profile_speed(0);
}

uint8_t GCS_Cmd_Is_Started(void)
{
    return g_car_started;
}

// 使用TIM6的真实毫秒时基，每10ms将公共速度改变1mm/s。
void GCS_Cmd_Update(void)
{
    uint32_t now;
    uint32_t elapsed_ms;
    uint32_t steps;
    int32_t next_speed;

    if (g_car_started == 0u)
    {
        return;
    }

    now = g_system_tick_ms;

    // Task 2 runs at 200 mm/s for the first five seconds, then returns to
    // the normal 130 mm/s landing speed using the existing smooth ramp.
    if ((g_task2_initial_fast_active != 0u) &&
        ((now - g_task2_initial_fast_start_ms) >=
         GCS_TASK2_INITIAL_FAST_TIME_MS))
    {
        g_task2_initial_fast_active = 0u;
        g_target_speed_mm_s = GCS_SPEED_TASK2_SLOW_MM_S;
        g_last_ramp_ms = now;
    }

    // Keep task 1 out of the unstable ultra-low-speed steering range while
    // preserving a five-second A-B catch-up window for the aircraft.
    if ((g_task_number == 1u) &&
        (g_target_speed_mm_s == GCS_SPEED_TASK1_SLOW_MM_S) &&
        ((now - g_task1_profile_start_ms) <= GCS_TASK1_PROFILE_END_MS))
    {
        elapsed_ms = now - g_task1_profile_start_ms;

        if (elapsed_ms < GCS_TASK1_INITIAL_RAMP_END_MS)
        {
            next_speed = (int32_t)(elapsed_ms / GCS_SPEED_RAMP_PERIOD_MS);
        }
        else if (elapsed_ms < GCS_TASK1_HOLD_END_MS)
        {
            next_speed = GCS_TASK1_STABLE_START_SPEED_MM_S;
        }
        else
        {
            next_speed = GCS_TASK1_STABLE_START_SPEED_MM_S +
                (int32_t)((elapsed_ms - GCS_TASK1_HOLD_END_MS) /
                          GCS_SPEED_RAMP_PERIOD_MS);
            if (next_speed > GCS_SPEED_TASK1_SLOW_MM_S)
            {
                next_speed = GCS_SPEED_TASK1_SLOW_MM_S;
            }
        }

        g_current_speed_mm_s = (int16_t)next_speed;
        g_last_ramp_ms = now;
        apply_profile_speed(g_current_speed_mm_s);
        return;
    }

    elapsed_ms = now - g_last_ramp_ms;
    if (elapsed_ms < GCS_SPEED_RAMP_PERIOD_MS)
    {
        return;
    }

    steps = elapsed_ms / GCS_SPEED_RAMP_PERIOD_MS;
    g_last_ramp_ms += steps * GCS_SPEED_RAMP_PERIOD_MS;
    next_speed = g_current_speed_mm_s;

    if (g_current_speed_mm_s < g_target_speed_mm_s)
    {
        next_speed += (int32_t)steps * GCS_SPEED_RAMP_STEP_MM_S;
        if (next_speed > g_target_speed_mm_s)
        {
            next_speed = g_target_speed_mm_s;
        }
    }
    else if (g_current_speed_mm_s > g_target_speed_mm_s)
    {
        next_speed -= (int32_t)steps * GCS_SPEED_RAMP_STEP_MM_S;
        if (next_speed < g_target_speed_mm_s)
        {
            next_speed = g_target_speed_mm_s;
        }
    }

    g_current_speed_mm_s = (int16_t)next_speed;
    apply_profile_speed(g_current_speed_mm_s);
}

// 主循环每次调用都行: 启动帧每次上电只生效一次，重复帧不会重置小车。
void GCS_Cmd_Poll(void)
{
    char line[GCS_CMD_BUF_LEN];
    uint8_t task_number;
    int code;

    if (!RDK_Link_Read_Command(line, sizeof(line)))
    {
        return;
    }

    if (parse_car_task_cmd(line, &task_number))
    {
        if (g_car_started == 0u)
        {
            g_task_number = task_number;
        }
        return;
    }

    if (parse_car_start_cmd(line))
    {
        if ((g_car_started == 0u) && (g_task_number != 0u))
        {
            uint32_t start_ms = g_system_tick_ms;

            g_current_speed_mm_s = 0;
            g_task2_initial_fast_active =
                (g_task_number == 2u) ? 1u : 0u;
            g_task2_initial_fast_start_ms = start_ms;
            apply_slow_speed();
            g_last_ramp_ms = start_ms;
            g_task1_profile_start_ms = start_ms;
            apply_profile_speed(0);
            Odometry_Reset();
            g_car_started = 1u;
        }
        return;
    }

    if (!parse_car_speed_cmd(line, &code))
    {
        return;
    }

    if (g_car_started == 0u)
    {
        return;
    }

    if (code == 1)
    {
        apply_slow_speed();
    }
    else if (code == 2)
    {
        g_task2_initial_fast_active = 0u;
        apply_fast_speed();
    }
    // 其他code值: 不认识, 忽略不处理
}
