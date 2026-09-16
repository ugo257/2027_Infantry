#include "yaw_identification_test.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

volatile uint8_t g_yaw_test_double_up = 0u;

static void update(float angle_deg, float rate_rad_s, uint8_t ready)
{
    YawTest_Update(angle_deg, rate_rad_s, 1u, 0.01f, ready);
}

static void advance_until(uint8_t expected_state, uint32_t max_steps)
{
    for (uint32_t i = 0u; i < max_steps &&
                         g_yaw_test_state != expected_state; ++i) {
        update(10.0f, 0.0f, 1u);
    }
    assert(g_yaw_test_state == expected_state);
}

static void assert_planned_reference(void)
{
    assert(isfinite(YawTest_GetTarget()));
    assert(isfinite(YawTest_GetReferenceRate()));
    assert(isfinite(YawTest_GetReferenceAcceleration()));
    assert(fabsf(YawTest_GetCurrentInjection()) < 0.001f);
    assert(YawTest_GetTarget() >= -0.001f);
    assert(YawTest_GetTarget() <= 20.001f);
    assert(fabsf(YawTest_GetReferenceAcceleration()) <=
           g_yaw_test_wave_accel_limit_debug + 0.01f);
}

int main(void)
{
    uint8_t saw_positive_rate = 0u;
    uint8_t saw_negative_rate = 0u;
    float minimum_target = 1.0e6f;
    float maximum_target = -1.0e6f;

    YawTest_Init();
    g_yaw_test_wave_frequency_hz = 3.0f;
    g_yaw_test_wave_peak_to_peak_deg = 20.0f;
    g_yaw_test_max_accel_rad_s2 = 50.0f;
    g_yaw_test_accel_ff_enable = 1u;

    g_yaw_test_double_up = 0u;
    update(10.0f, 0.0f, 1u);
    assert(g_yaw_test_state == YAW_TEST_IDLE);
    assert(YawTest_IsActive() == 0u);

    g_yaw_test_double_up = 1u;
    update(10.0f, 0.0f, 1u);
    assert(g_yaw_test_state == YAW_TEST_ARM);
    assert(YawTest_IsActive() != 0u);
    assert(fabsf(YawTest_GetTarget() - 10.0f) < 0.01f);

    advance_until(YAW_TEST_QP_TRACKING, 100u);
    assert(YawTest_GetAccelerationFeedforwardEnable() != 0u);
    for (uint32_t i = 0u; i < 100u; ++i) {
        update(10.0f, 0.0f, 1u);
        assert(g_yaw_test_sample_valid != 0u);
        assert_planned_reference();
        if (YawTest_GetTarget() < minimum_target) {
            minimum_target = YawTest_GetTarget();
        }
        if (YawTest_GetTarget() > maximum_target) {
            maximum_target = YawTest_GetTarget();
        }
        if (YawTest_GetReferenceRate() > 0.10f) saw_positive_rate = 1u;
        if (YawTest_GetReferenceRate() < -0.10f) saw_negative_rate = 1u;
    }
    assert(g_yaw_test_wave_frequency_debug >= 2.99f);
    assert(g_yaw_test_wave_frequency_debug <= 3.01f);
    assert(g_yaw_test_wave_accel_limit_debug <= 50.01f);
    assert(g_yaw_test_planned_peak_to_peak_debug > 12.0f);
    assert(g_yaw_test_planned_peak_to_peak_debug < 14.0f);
    assert(g_yaw_test_planner_transition_debug > 0.05f);
    assert(minimum_target < 5.0f);
    assert(maximum_target > 15.0f);
    assert(saw_positive_rate != 0u);
    assert(saw_negative_rate != 0u);

    advance_until(YAW_TEST_DONE, 1200u);
    assert(g_yaw_test_finished != 0u);
    assert(fabsf(YawTest_GetTarget() - 10.0f) < 0.001f);

    g_yaw_test_double_up = 0u;
    update(10.0f, 0.0f, 1u);
    assert(g_yaw_test_state == YAW_TEST_IDLE);
    assert(YawTest_IsActive() == 0u);

    g_yaw_test_wave_frequency_hz = 5.0f;
    g_yaw_test_double_up = 1u;
    update(10.0f, 0.0f, 1u);
    advance_until(YAW_TEST_QP_TRACKING, 100u);
    update(10.0f, 0.0f, 1u);
    assert_planned_reference();
    assert(g_yaw_test_wave_frequency_debug >= 4.99f);
    assert(g_yaw_test_wave_frequency_debug <= 5.01f);
    assert(g_yaw_test_planned_peak_to_peak_debug > 6.0f);
    assert(g_yaw_test_planned_peak_to_peak_debug < 8.0f);

    g_yaw_test_double_up = 0u;
    update(10.0f, 0.0f, 1u);
    assert(g_yaw_test_state == YAW_TEST_IDLE);

    puts("yaw_identification_test_test: PASS");
    return 0;
}
