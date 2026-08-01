#include <AllHeader.h>
#include <gcs_cmd.h>
#include <speed_test_config.h>

static uint8_t g_car_started = 0u;
static int16_t g_current_speed_mm_s = 0;
static uint32_t g_last_ramp_ms = 0u;

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

void GCS_Cmd_Init(void)
{
    g_current_speed_mm_s = 0;
    g_last_ramp_ms = g_system_tick_ms;
    apply_profile_speed(0);
    Odometry_Reset();
    g_car_started = 1u;
}

uint8_t GCS_Cmd_Is_Started(void)
{
    return g_car_started;
}

void GCS_Cmd_Update(void)
{
    uint32_t now;
    uint32_t elapsed_ms;
    uint32_t steps;
    int32_t next_speed;

    now = g_system_tick_ms;
    elapsed_ms = now - g_last_ramp_ms;
    if (elapsed_ms < SPEED_TEST_RAMP_PERIOD_MS)
    {
        return;
    }

    steps = elapsed_ms / SPEED_TEST_RAMP_PERIOD_MS;
    g_last_ramp_ms += steps * SPEED_TEST_RAMP_PERIOD_MS;
    next_speed = g_current_speed_mm_s +
        (int32_t)steps * SPEED_TEST_RAMP_STEP_MM_S;

    if (next_speed > SPEED_TEST_TARGET_MM_S)
    {
        next_speed = SPEED_TEST_TARGET_MM_S;
    }

    g_current_speed_mm_s = (int16_t)next_speed;
    apply_profile_speed(g_current_speed_mm_s);
}

void GCS_Cmd_Poll(void)
{
    // Standalone speed test: all RDK control frames are intentionally ignored.
}
