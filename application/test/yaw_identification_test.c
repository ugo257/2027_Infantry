#include "yaw_identification_test.h"

#include <math.h>

#include "robot_cmd.h"
#include "robot_types.h"

#define YAW_TEST_MIN_DT_S                 (0.0001f)
#define YAW_TEST_MAX_DT_S                 (0.02f)
#define YAW_TEST_DEFAULT_DT_S             (0.001f)
#define YAW_TEST_ARM_TIME_S               (0.50f)
#define YAW_TEST_CENTER_SPEED_DEG_S       (240.0f)
#define YAW_TEST_CENTER_TOL_DEG           (2.0f)
#define YAW_TEST_CENTER_RATE_TOL_DEG_S    (25.0f)
#define YAW_TEST_CENTER_TIMEOUT_S          (2.0f)
#define YAW_TEST_STEP_DWELL_S             (0.55f)
#define YAW_TEST_CHIRP_DURATION_S         (14.0f)
#define YAW_TEST_CHIRP_AMPLITUDE_DEG      (60.0f)
#define YAW_TEST_CHIRP_FREQUENCY_START_HZ (0.25f)
#define YAW_TEST_CHIRP_FREQUENCY_END_HZ   (1.50f)
#define YAW_TEST_CHIRP_RAMP_S             (0.50f)
#define YAW_TEST_PI                       (3.14159265358979323846f)

static const float yaw_test_step_targets_deg[] = {
    -110.0f, 110.0f, -70.0f, 70.0f, -30.0f, 30.0f, 0.0f,
};

#define YAW_TEST_STEP_COUNT \
    ((uint8_t)(sizeof(yaw_test_step_targets_deg) / \
               sizeof(yaw_test_step_targets_deg[0])))

volatile uint8_t g_yaw_test_state = YAW_TEST_IDLE;
volatile uint8_t g_yaw_test_finished = 0u;
volatile uint8_t g_yaw_test_sample_valid = 0u;
volatile uint8_t g_yaw_test_step_index = 0u;
volatile uint32_t g_yaw_test_sample_count = 0u;
volatile float g_yaw_test_target_debug = 0.0f;
volatile float g_yaw_test_target_offset_debug = 0.0f;
volatile float g_yaw_test_target_velocity_debug = 0.0f;
volatile float g_yaw_test_target_acceleration_debug = 0.0f;
volatile float g_yaw_test_elapsed_debug = 0.0f;
volatile float g_yaw_test_chirp_frequency_debug = 0.0f;

static float yaw_test_target = 0.0f;
static float yaw_test_state_elapsed = 0.0f;
static float yaw_test_chirp_elapsed = 0.0f;
static float yaw_test_chirp_phase = 0.0f;
static uint8_t yaw_test_active = 0u;

static float YawTestAbs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float YawTestClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float YawTestDt(float dt_s)
{
    if (!isfinite(dt_s) || dt_s < YAW_TEST_MIN_DT_S ||
        dt_s > YAW_TEST_MAX_DT_S) {
        return YAW_TEST_DEFAULT_DT_S;
    }
    return dt_s;
}

static void YawTestSetState(YawTestState_e state)
{
    g_yaw_test_state = (uint8_t)state;
    yaw_test_state_elapsed = 0.0f;
    g_yaw_test_elapsed_debug = 0.0f;
}

static void YawTestStop(float yaw_measure_deg)
{
    yaw_test_active = 0u;
    g_yaw_test_state = YAW_TEST_IDLE;
    g_yaw_test_finished = 0u;
    g_yaw_test_sample_valid = 0u;
    g_yaw_test_step_index = 0u;
    g_yaw_test_target_debug = yaw_measure_deg;
    g_yaw_test_target_offset_debug = 0.0f;
    g_yaw_test_target_velocity_debug = 0.0f;
    g_yaw_test_target_acceleration_debug = 0.0f;
    g_yaw_test_elapsed_debug = 0.0f;
    g_yaw_test_chirp_frequency_debug = 0.0f;
    yaw_test_target = yaw_measure_deg;
    yaw_test_state_elapsed = 0.0f;
    yaw_test_chirp_elapsed = 0.0f;
    yaw_test_chirp_phase = 0.0f;
}

static void YawTestStart(float yaw_measure_deg)
{
    yaw_test_active = 1u;
    g_yaw_test_finished = 0u;
    g_yaw_test_sample_valid = 0u;
    g_yaw_test_step_index = 0u;
    g_yaw_test_sample_count = 0u;
    yaw_test_target = yaw_measure_deg;
    yaw_test_state_elapsed = 0.0f;
    yaw_test_chirp_elapsed = 0.0f;
    yaw_test_chirp_phase = 0.0f;
    g_yaw_test_target_debug = yaw_measure_deg;
    g_yaw_test_target_offset_debug = 0.0f;
    g_yaw_test_chirp_frequency_debug = 0.0f;
    YawTestSetState(YAW_TEST_ARM);
}

void YawTest_Init(void)
{
    g_yaw_test_state = YAW_TEST_IDLE;
    g_yaw_test_finished = 0u;
    g_yaw_test_sample_valid = 0u;
    g_yaw_test_step_index = 0u;
    g_yaw_test_sample_count = 0u;
    g_yaw_test_target_debug = 0.0f;
    g_yaw_test_target_offset_debug = 0.0f;
    g_yaw_test_target_velocity_debug = 0.0f;
    g_yaw_test_target_acceleration_debug = 0.0f;
    g_yaw_test_elapsed_debug = 0.0f;
    g_yaw_test_chirp_frequency_debug = 0.0f;
    yaw_test_target = 0.0f;
    yaw_test_state_elapsed = 0.0f;
    yaw_test_chirp_elapsed = 0.0f;
    yaw_test_chirp_phase = 0.0f;
    yaw_test_active = 0u;
}

void YawTest_Update(float yaw_measure_deg,
                    float yaw_rate_deg_s,
                    uint8_t feedback_ok,
                    float dt_s,
                    uint8_t gimbal_mode)
{
    const float dt = YawTestDt(dt_s);

    g_yaw_test_sample_valid = 0u;
    g_yaw_test_target_velocity_debug = 0.0f;
    g_yaw_test_target_acceleration_debug = 0.0f;
    yaw_test_state_elapsed += dt;
    g_yaw_test_elapsed_debug = yaw_test_state_elapsed;

    if (g_yaw_test_double_up == 0u) {
        YawTestStop(yaw_measure_deg);
        return;
    }
    if (gimbal_mode != GIMBAL_GYRO_MODE || feedback_ok == 0u ||
        !isfinite(yaw_measure_deg) || !isfinite(yaw_rate_deg_s)) {
        YawTestStop(yaw_measure_deg);
        return;
    }
    if (yaw_test_active == 0u) {
        YawTestStart(yaw_measure_deg);
    }

    switch ((YawTestState_e)g_yaw_test_state) {
    case YAW_TEST_ARM:
        /* Hold the measured starting point before issuing a target motion. */
        yaw_test_target = yaw_measure_deg;
        g_yaw_test_target_debug = yaw_test_target;
        if (yaw_test_state_elapsed >= YAW_TEST_ARM_TIME_S) {
            YawTestSetState(YAW_TEST_CENTER);
        }
        break;

    case YAW_TEST_CENTER: {
        const float delta = -yaw_test_target;
        const float step = YawTestClamp(delta,
                                        -YAW_TEST_CENTER_SPEED_DEG_S * dt,
                                        YAW_TEST_CENTER_SPEED_DEG_S * dt);
        yaw_test_target += step;
        g_yaw_test_target_velocity_debug = step / dt;
        g_yaw_test_target_debug = yaw_test_target;
        g_yaw_test_target_offset_debug = yaw_test_target;
        if ((YawTestAbs(yaw_test_target) <= YAW_TEST_CENTER_TOL_DEG &&
             YawTestAbs(yaw_rate_deg_s) <= YAW_TEST_CENTER_RATE_TOL_DEG_S) ||
            yaw_test_state_elapsed >= YAW_TEST_CENTER_TIMEOUT_S) {
            yaw_test_target = 0.0f;
            g_yaw_test_target_debug = 0.0f;
            g_yaw_test_target_offset_debug = 0.0f;
            YawTestSetState(YAW_TEST_STEP);
        }
        break;
    }

    case YAW_TEST_STEP: {
        const float previous_target = yaw_test_target;
        yaw_test_target = yaw_test_step_targets_deg[g_yaw_test_step_index];
        g_yaw_test_target_debug = yaw_test_target;
        g_yaw_test_target_offset_debug = yaw_test_target;
        g_yaw_test_target_velocity_debug =
            (yaw_test_target - previous_target) / dt;
        g_yaw_test_sample_valid = 1u;
        g_yaw_test_sample_count++;
        if (yaw_test_state_elapsed >= YAW_TEST_STEP_DWELL_S) {
            if (g_yaw_test_step_index + 1u < YAW_TEST_STEP_COUNT) {
                g_yaw_test_step_index++;
                YawTestSetState(YAW_TEST_STEP);
            } else {
                yaw_test_chirp_elapsed = 0.0f;
                yaw_test_chirp_phase = 0.0f;
                YawTestSetState(YAW_TEST_CHIRP);
            }
        }
        break;
    }

    case YAW_TEST_CHIRP: {
        const float duration = YAW_TEST_CHIRP_DURATION_S;
        const float frequency_slope =
            (YAW_TEST_CHIRP_FREQUENCY_END_HZ -
             YAW_TEST_CHIRP_FREQUENCY_START_HZ) / duration;
        const float frequency = YAW_TEST_CHIRP_FREQUENCY_START_HZ +
                                frequency_slope * yaw_test_chirp_elapsed;
        const float angular_frequency = 2.0f * YAW_TEST_PI * frequency;
        const float angular_frequency_dot = 2.0f * YAW_TEST_PI * frequency_slope;
        float amplitude_scale = 1.0f;
        float amplitude_scale_dot = 0.0f;
        float amplitude_scale_ddot = 0.0f;

        if (yaw_test_chirp_elapsed < YAW_TEST_CHIRP_RAMP_S) {
            const float r = yaw_test_chirp_elapsed / YAW_TEST_CHIRP_RAMP_S;
            amplitude_scale = 3.0f * r * r - 2.0f * r * r * r;
            amplitude_scale_dot = (6.0f * r - 6.0f * r * r) /
                                  YAW_TEST_CHIRP_RAMP_S;
            amplitude_scale_ddot = (6.0f - 12.0f * r) /
                                   (YAW_TEST_CHIRP_RAMP_S *
                                    YAW_TEST_CHIRP_RAMP_S);
        }
        yaw_test_target = YAW_TEST_CHIRP_AMPLITUDE_DEG * amplitude_scale *
                          sinf(yaw_test_chirp_phase);
        g_yaw_test_target_velocity_debug =
            YAW_TEST_CHIRP_AMPLITUDE_DEG *
            (amplitude_scale_dot * sinf(yaw_test_chirp_phase) +
             amplitude_scale * angular_frequency *
             cosf(yaw_test_chirp_phase));
        g_yaw_test_target_acceleration_debug =
            YAW_TEST_CHIRP_AMPLITUDE_DEG *
            (amplitude_scale_ddot * sinf(yaw_test_chirp_phase) +
             2.0f * amplitude_scale_dot * angular_frequency *
             cosf(yaw_test_chirp_phase) +
             amplitude_scale *
             (angular_frequency_dot * cosf(yaw_test_chirp_phase) -
              angular_frequency * angular_frequency *
              sinf(yaw_test_chirp_phase)));
        g_yaw_test_target_debug = yaw_test_target;
        g_yaw_test_target_offset_debug = yaw_test_target;
        g_yaw_test_chirp_frequency_debug = frequency;
        g_yaw_test_sample_valid = 1u;
        g_yaw_test_sample_count++;

        yaw_test_chirp_phase += angular_frequency * dt;
        yaw_test_chirp_elapsed += dt;
        g_yaw_test_elapsed_debug = yaw_test_chirp_elapsed;
        if (yaw_test_chirp_elapsed >= duration) {
            yaw_test_target = 0.0f;
            g_yaw_test_target_debug = 0.0f;
            g_yaw_test_target_offset_debug = 0.0f;
            g_yaw_test_target_velocity_debug = 0.0f;
            g_yaw_test_target_acceleration_debug = 0.0f;
            g_yaw_test_finished = 1u;
            YawTestSetState(YAW_TEST_DONE);
        }
        break;
    }

    case YAW_TEST_DONE:
        yaw_test_target = 0.0f;
        g_yaw_test_target_debug = 0.0f;
        g_yaw_test_target_offset_debug = 0.0f;
        g_yaw_test_finished = 1u;
        break;

    case YAW_TEST_IDLE:
    default:
        YawTestStop(yaw_measure_deg);
        break;
    }
}

uint8_t YawTest_IsActive(void)
{
    return yaw_test_active;
}

float YawTest_GetTarget(void)
{
    return yaw_test_target;
}
