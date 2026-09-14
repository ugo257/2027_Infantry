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

static void update_status(float angle_deg,
                          float rate_rad_s,
                          uint8_t feedback_ok,
                          uint8_t ready)
{
    YawTest_Update(angle_deg, rate_rad_s, feedback_ok, 0.01f, ready);
}

static void advance_until(uint8_t expected_state, uint32_t max_steps)
{
    for (uint32_t i = 0u; i < max_steps &&
                         g_yaw_test_state != expected_state; ++i) {
        update(10.0f, 0.0f, 1u);
    }
    assert(g_yaw_test_state == expected_state);
}

int main(void)
{
    YawTest_Init();
    g_yaw_test_double_up = 1u;
    update(10.0f, 0.0f, 0u);
    assert(g_yaw_test_state == YAW_TEST_IDLE);
    assert(YawTest_IsActive() == 0u);

    update(10.0f, 0.0f, 1u);
    assert(g_yaw_test_state == YAW_TEST_ARM);
    assert(YawTest_IsActive() != 0u);
    assert(fabsf(YawTest_GetTarget() - 10.0f) < 0.001f);
    assert(YawTest_GetCurrentInjection() == 0.0f);

    advance_until(YAW_TEST_PRBS_LOW, 250u);
    update(10.0f, 0.0f, 1u);
    assert(g_yaw_test_sample_valid != 0u);
    assert(fabsf(YawTest_GetCurrentInjection()) == 1800.0f);

    advance_until(YAW_TEST_PRBS_HIGH, 1100u);
    update(10.0f, 0.0f, 1u);
    assert(g_yaw_test_sample_valid != 0u);
    assert(fabsf(YawTest_GetCurrentInjection()) == 3000.0f);

    advance_until(YAW_TEST_CURRENT_CHIRP, 1600u);
    for (uint32_t i = 0u; i < 25u; ++i) {
        update(10.0f, 0.0f, 1u);
    }
    assert(g_yaw_test_sample_valid != 0u);
    assert(fabsf(YawTest_GetCurrentInjection()) <= 2400.0f);
    assert(g_yaw_test_chirp_frequency_debug >= 0.30f);

    for (uint32_t i = 0u;
         i < 1600u && g_yaw_test_state != YAW_TEST_RECOVER;
         ++i) {
        update(10.0f, 0.0f, 1u);
        assert(isfinite(YawTest_GetCurrentInjection()));
        assert(fabsf(YawTest_GetCurrentInjection()) <= 2400.0f);
    }
    assert(g_yaw_test_state == YAW_TEST_RECOVER);
    assert(YawTest_GetCurrentInjection() == 0.0f);
    assert(g_yaw_test_sample_valid == 0u);
    advance_until(YAW_TEST_DONE, 250u);
    assert(g_yaw_test_finished != 0u);

    g_yaw_test_double_up = 0u;
    update(10.0f, 0.0f, 1u);
    assert(g_yaw_test_state == YAW_TEST_IDLE);
    assert(YawTest_IsActive() == 0u);

    g_yaw_test_double_up = 1u;
    update(10.0f, 0.0f, 1u);
    advance_until(YAW_TEST_PRBS_LOW, 250u);
    update_status(31.0f, 0.0f, 0u, 0u);
    assert(g_yaw_test_state == YAW_TEST_PRBS_LOW);
    assert(fabsf(YawTest_GetCurrentInjection()) == 1800.0f);

    puts("yaw_identification_test_test: PASS");
    return 0;
}
