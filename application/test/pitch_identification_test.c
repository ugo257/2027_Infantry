#include "pitch_identification_test.h"

#include <math.h>

#include "robot_types.h"
#include "robot_cmd.h"

#define PITCH_TEST_MIN_DT_S          (0.0001f)
#define PITCH_TEST_MAX_DT_S          (0.01f)
#define PITCH_TEST_DEFAULT_DT_S      (0.001f)
#define PITCH_TEST_ARM_TIME_S        (2.0f)
#define PITCH_TEST_HOLD_TIME_S       (4.0f)
#define PITCH_TEST_TARGET_SPEED_RAD_S (0.05f)
#define PITCH_TEST_TARGET_LIMIT_RAD  (0.25f)
#define PITCH_TEST_TARGET_DEADBAND_RAD (0.002f)

volatile uint8_t g_pitch_test_enable = 0u;
volatile uint8_t g_pitch_test_abort = 0u;
volatile uint8_t g_pitch_test_state = PITCH_TEST_IDLE;
volatile uint8_t g_pitch_test_target_index = 0u;
volatile uint8_t g_pitch_test_finished = 0u;
volatile float g_pitch_test_target_debug = 0.0f;
volatile float g_pitch_test_target_velocity_debug = 0.0f;
volatile float g_pitch_test_target_acceleration_debug = 0.0f;
volatile float g_pitch_test_elapsed_debug = 0.0f;
volatile uint8_t g_pitch_test_abort_reason = 0u;

static const float pitch_test_targets[] = {
    0.0f,
    0.125f,
    0.250f,
    0.0f,
    -0.125f,
    -0.250f,
};

static float pitch_test_target = 0.0f;
static float pitch_test_hold_elapsed = 0.0f;
static float pitch_test_arm_elapsed = 0.0f;
static float pitch_test_start_angle = 0.0f;
static uint8_t pitch_test_active = 0u;

static float PitchTestClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float PitchTestAbs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float PitchTestDt(float dt_s)
{
    if (!isfinite(dt_s) || dt_s < PITCH_TEST_MIN_DT_S ||
        dt_s > PITCH_TEST_MAX_DT_S) {
        return PITCH_TEST_DEFAULT_DT_S;
    }
    return dt_s;
}

static void PitchTestPublish(void)
{
    g_pitch_test_target_debug = pitch_test_target;
    g_pitch_test_elapsed_debug =
        (g_pitch_test_state == PITCH_TEST_ARM) ? pitch_test_arm_elapsed :
        pitch_test_hold_elapsed;
}

void PitchTest_Init(void)
{
    g_pitch_test_state = PITCH_TEST_IDLE;
    g_pitch_test_target_index = 0u;
    g_pitch_test_finished = 0u;
    g_pitch_test_abort_reason = 0u;
    pitch_test_target = 0.0f;
    pitch_test_hold_elapsed = 0.0f;
    pitch_test_arm_elapsed = 0.0f;
    pitch_test_active = 0u;
    PitchTestPublish();
}

void PitchTest_Update(float theta_meas_rad, float dt_s, uint8_t gimbal_mode)
{
    const float dt = PitchTestDt(dt_s);
    const uint8_t run_request =
        (g_pitch_test_enable != 0u || g_pitch_test_double_up != 0u) ? 1u : 0u;

    g_pitch_test_target_velocity_debug = 0.0f;
    g_pitch_test_target_acceleration_debug = 0.0f;

    if (g_pitch_test_abort != 0u) {
        pitch_test_active = 0u;
        g_pitch_test_state = PITCH_TEST_ABORT;
        g_pitch_test_abort_reason = 1u;
        PitchTestPublish();
        return;
    }

    if (pitch_test_active == 0u) {
        if (run_request == 0u || gimbal_mode != GIMBAL_GYRO_MODE) {
            g_pitch_test_state = PITCH_TEST_IDLE;
            g_pitch_test_finished = 0u;
            PitchTestPublish();
            return;
        }
        pitch_test_active = 1u;
        g_pitch_test_state = PITCH_TEST_ARM;
        g_pitch_test_target_index = 0u;
        g_pitch_test_finished = 0u;
        g_pitch_test_abort_reason = 0u;
        pitch_test_start_angle = theta_meas_rad;
        pitch_test_target = theta_meas_rad;
        pitch_test_arm_elapsed = 0.0f;
        pitch_test_hold_elapsed = 0.0f;
    }

    if (run_request == 0u || gimbal_mode != GIMBAL_GYRO_MODE) {
        pitch_test_active = 0u;
        g_pitch_test_state = PITCH_TEST_ABORT;
        g_pitch_test_abort_reason = 2u;
        PitchTestPublish();
        return;
    }

    switch ((PitchTestState_e)g_pitch_test_state) {
    case PITCH_TEST_ARM:
        pitch_test_target = pitch_test_start_angle;
        pitch_test_arm_elapsed += dt;
        if (pitch_test_arm_elapsed >= PITCH_TEST_ARM_TIME_S) {
            g_pitch_test_state = PITCH_TEST_MOVE;
            pitch_test_target = 0.0f;
        }
        break;

    case PITCH_TEST_MOVE: {
        const float desired = PitchTestClamp(
            pitch_test_targets[g_pitch_test_target_index],
            -PITCH_TEST_TARGET_LIMIT_RAD,
            PITCH_TEST_TARGET_LIMIT_RAD);
        const float delta = desired - pitch_test_target;
        const float max_delta = PITCH_TEST_TARGET_SPEED_RAD_S * dt;

        pitch_test_target += PitchTestClamp(delta, -max_delta, max_delta);
        g_pitch_test_target_velocity_debug =
            PitchTestClamp(delta / dt,
                           -PITCH_TEST_TARGET_SPEED_RAD_S,
                           PITCH_TEST_TARGET_SPEED_RAD_S);
        if (PitchTestAbs(desired - pitch_test_target) <=
            PITCH_TEST_TARGET_DEADBAND_RAD) {
            pitch_test_target = desired;
            pitch_test_hold_elapsed = 0.0f;
            g_pitch_test_state = PITCH_TEST_HOLD;
        }
        break;
    }

    case PITCH_TEST_HOLD:
        pitch_test_target = PitchTestClamp(
            pitch_test_targets[g_pitch_test_target_index],
            -PITCH_TEST_TARGET_LIMIT_RAD,
            PITCH_TEST_TARGET_LIMIT_RAD);
        pitch_test_hold_elapsed += dt;
        if (pitch_test_hold_elapsed >= PITCH_TEST_HOLD_TIME_S) {
            if (g_pitch_test_target_index + 1u <
                (uint8_t)(sizeof(pitch_test_targets) / sizeof(pitch_test_targets[0]))) {
                g_pitch_test_target_index++;
                g_pitch_test_state = PITCH_TEST_MOVE;
                pitch_test_hold_elapsed = 0.0f;
            } else {
                g_pitch_test_state = PITCH_TEST_DONE;
                g_pitch_test_finished = 1u;
            }
        }
        break;

    case PITCH_TEST_DONE:
        /* Hold the final point. There is intentionally no automatic return. */
        pitch_test_target = PitchTestClamp(
            pitch_test_targets[g_pitch_test_target_index],
            -PITCH_TEST_TARGET_LIMIT_RAD,
            PITCH_TEST_TARGET_LIMIT_RAD);
        break;

    case PITCH_TEST_ABORT:
    case PITCH_TEST_IDLE:
    default:
        pitch_test_active = 0u;
        break;
    }

    PitchTestPublish();
}

uint8_t PitchTest_IsActive(void)
{
    return pitch_test_active;
}

float PitchTest_GetTarget(void)
{
    return pitch_test_target;
}
